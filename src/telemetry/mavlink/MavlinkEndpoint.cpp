/**
 * MavlinkEndpoint.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/MavlinkEndpoint.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string_view>

#include "src/state/GroundSafety.h"
#include "src/utils/Logging.h"

namespace arduflite::mavlink {

namespace {

/// Inbound bytes handled per service() pass, so a flood cannot starve output.
constexpr std::size_t kMaxReadPerPass = 512;

/// STATUSTEXT carries 50 characters per message.
constexpr std::size_t kTextChunk = 50;

struct StreamMessage { Stream stream; std::uint32_t msgId; };

constexpr StreamMessage kStreamMessages[] = {
    { Stream::Heartbeat, MAVLINK_MSG_ID_HEARTBEAT },
    { Stream::SysStatus, MAVLINK_MSG_ID_SYS_STATUS },
    { Stream::Attitude,  MAVLINK_MSG_ID_ATTITUDE },
    { Stream::VfrHud,    MAVLINK_MSG_ID_VFR_HUD },
    { Stream::ScaledImu, MAVLINK_MSG_ID_SCALED_IMU },
    { Stream::Values,    MAVLINK_MSG_ID_NAMED_VALUE_FLOAT },
};

std::optional<Stream> streamFor(float msgIdParam)
{
    if (!std::isfinite(msgIdParam) || msgIdParam < 0.0f) { return std::nullopt; }
    const auto msgId = static_cast<std::uint32_t>(msgIdParam);
    for (const StreamMessage& entry : kStreamMessages)
    {
        if (entry.msgId == msgId) { return entry.stream; }
    }
    return std::nullopt;
}

bool requestsAutopilotVersion(float msgIdParam)
{
    return msgIdParam == static_cast<float>(MAVLINK_MSG_ID_AUTOPILOT_VERSION);
}

/// A parameter name as received: 16 bytes, NUL-terminated only when shorter.
std::string_view paramId(const char (&id)[kParamIdLength])
{
    return { id, static_cast<std::size_t>(std::find(id, id + kParamIdLength, '\0') - id) };
}

} // namespace

MavlinkEndpoint::MavlinkEndpoint(hal::ByteStream& stream, StatusTextQueue* statusText) noexcept
    : _stream(stream), _statusText(statusText)
{
}

void MavlinkEndpoint::configure(const EndpointConfig& config, RebootHook reboot) noexcept
{
    _config = config;
    _reboot = reboot;
    _scheduler.configure(config.intervals, config.budgetBitsPerSecond);
}

void MavlinkEndpoint::service(std::uint32_t nowMs, const TelemetryData& telemetry)
{
    receive(nowMs, telemetry);
    sendStreams(nowMs, telemetry);
    sendStatusText(nowMs);
    sendParamList(nowMs);
}

// ── Inbound ─────────────────────────────────────────────────────────────────

void MavlinkEndpoint::receive(std::uint32_t nowMs, const TelemetryData& telemetry)
{
    std::uint8_t buffer[64];
    std::size_t  budget = kMaxReadPerPass;

    while (budget > 0 && _stream.available() > 0)
    {
        const std::size_t n = _stream.read(buffer, (budget < sizeof(buffer)) ? budget : sizeof(buffer));
        if (n == 0) { break; }
        budget -= n;

        for (std::size_t i = 0; i < n; ++i)
        {
            if (const mavlink_message_t* msg = _parser.feed(buffer[i]))
            {
                ++_counters.framesIn;
                handle(*msg, nowMs, telemetry);
            }
        }
    }
}

void MavlinkEndpoint::handle(const mavlink_message_t& msg, std::uint32_t nowMs,
                             const TelemetryData& telemetry)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            mavlink_param_request_list_t request;
            mavlink_msg_param_request_list_decode(&msg, &request);
            if (isForUs(request.target_system)) { _paramCursor = 0; }
            break;
        }

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            handleParamRead(msg, nowMs);
            break;

        case MAVLINK_MSG_ID_PARAM_SET:
            handleParamSet(msg, nowMs, telemetry);
            break;

        case MAVLINK_MSG_ID_COMMAND_LONG:
            handleCommand(msg, nowMs, telemetry);
            break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        {
            // No mission storage: answer with an empty list rather than let the
            // ground station time out.
            mavlink_mission_request_list_t request;
            mavlink_msg_mission_request_list_decode(&msg, &request);
            if (!isForUs(request.target_system)) { break; }

            mavlink_message_t reply;
            mavlink_msg_mission_count_pack_status(_config.systemId, MAV_COMP_ID_AUTOPILOT1, &_txStatus,
                                                  &reply, msg.sysid, msg.compid, 0,
                                                  request.mission_type, 0);
            (void)send(reply, nowMs);
            break;
        }

        default:
            break;
    }
}

void MavlinkEndpoint::handleParamRead(const mavlink_message_t& msg, std::uint32_t nowMs)
{
    mavlink_param_request_read_t request;
    mavlink_msg_param_request_read_decode(&msg, &request);
    if (!isForUs(request.target_system)) { return; }

    const std::optional<ParamValue> value =
        (request.param_index >= 0) ? readParam(static_cast<std::uint16_t>(request.param_index))
                                   : readParam(paramId(request.param_id));
    if (value) { (void)sendParamValue(*value, nowMs); }
}

void MavlinkEndpoint::handleParamSet(const mavlink_message_t& msg, std::uint32_t nowMs,
                                     const TelemetryData& telemetry)
{
    mavlink_param_set_t request;
    mavlink_msg_param_set_decode(&msg, &request);
    if (!isForUs(request.target_system)) { return; }

    const std::string_view id = paramId(request.param_id);
    const int idLength = static_cast<int>(id.size());

    if (const char* refusal = writeRefusal(telemetry))
    {
        LOG_WARN("MAVLink: %.*s not changed: %s", idLength, id.data(), refusal);
    }
    else
    {
        switch (writeParam(id, request.param_value))
        {
            case ParamWrite::Unknown:
                return;
            case ParamWrite::Rejected:
                LOG_WARN("MAVLink: %.*s rejected %g (wrong type or out of range)",
                         idLength, id.data(), static_cast<double>(request.param_value));
                break;
            case ParamWrite::Applied:
                break;
        }
    }

    // The ground station confirms a write by the value that comes back, so it
    // gets the current value whether or not the write took effect.
    if (const std::optional<ParamValue> value = readParam(id)) { (void)sendParamValue(*value, nowMs); }
}

void MavlinkEndpoint::handleCommand(const mavlink_message_t& msg, std::uint32_t nowMs,
                                    const TelemetryData& telemetry)
{
    mavlink_command_long_t command;
    mavlink_msg_command_long_decode(&msg, &command);
    if (!isForUs(command.target_system)) { return; }

    const std::uint8_t result = runCommand(command, nowMs, telemetry);

    mavlink_message_t ack;
    mavlink_msg_command_ack_pack_status(_config.systemId, MAV_COMP_ID_AUTOPILOT1, &_txStatus, &ack,
                                        command.command, result, 0, 0, msg.sysid, msg.compid);
    (void)send(ack, nowMs);

    if (command.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && result == MAV_RESULT_ACCEPTED)
    {
        _reboot();
    }
}

std::uint8_t MavlinkEndpoint::runCommand(const mavlink_command_long_t& command, std::uint32_t nowMs,
                                         const TelemetryData& telemetry)
{
    switch (command.command)
    {
        case MAV_CMD_REQUEST_MESSAGE:
            if (requestsAutopilotVersion(command.param1))
            {
                return sendAutopilotVersion(nowMs) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;
            }
            if (const std::optional<Stream> stream = streamFor(command.param1))
            {
                _scheduler.requestNow(*stream);
                return MAV_RESULT_ACCEPTED;
            }
            return MAV_RESULT_UNSUPPORTED;

        case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
            return sendAutopilotVersion(nowMs) ? MAV_RESULT_ACCEPTED : MAV_RESULT_TEMPORARILY_REJECTED;

        case MAV_CMD_SET_MESSAGE_INTERVAL:
        {
            const std::optional<Stream> stream = streamFor(command.param1);
            if (!stream || !std::isfinite(command.param2)) { return MAV_RESULT_UNSUPPORTED; }

            // param2: interval in microseconds; -1 stops the stream, 0 restores its default.
            const float intervalUs = command.param2;
            std::uint32_t intervalMs = 0;
            if (intervalUs == 0.0f)     { intervalMs = _scheduler.defaultInterval(*stream); }
            else if (intervalUs > 0.0f) { intervalMs = std::max<std::uint32_t>(1, static_cast<std::uint32_t>(intervalUs / 1000.0f)); }
            _scheduler.setInterval(*stream, intervalMs);
            return MAV_RESULT_ACCEPTED;
        }

        case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            if (command.param1 != 1.0f || _reboot == nullptr) { return MAV_RESULT_UNSUPPORTED; }
            if (const char* refusal = writeRefusal(telemetry))
            {
                LOG_WARN("MAVLink: reboot refused: %s", refusal);
                return MAV_RESULT_DENIED;
            }
            return MAV_RESULT_ACCEPTED;

        default:
            return MAV_RESULT_UNSUPPORTED;
    }
}

// ── Outbound ────────────────────────────────────────────────────────────────

void MavlinkEndpoint::sendStreams(std::uint32_t nowMs, const TelemetryData& telemetry)
{
    // A stream that did not fit is still marked sent: it waits its interval
    // rather than holding back the lower-priority streams behind it.
    while (const std::optional<Stream> due = _scheduler.nextDue(nowMs))
    {
        (void)sendStream(*due, nowMs, telemetry);
        _scheduler.markSent(*due, nowMs);
    }
}

bool MavlinkEndpoint::sendStream(Stream stream, std::uint32_t nowMs, const TelemetryData& telemetry)
{
    const Origin from = origin();
    mavlink_message_t msg;

    switch (stream)
    {
        case Stream::Heartbeat:
            packHeartbeat(from, telemetry, msg);
            return send(msg, nowMs);

        case Stream::SysStatus:
        {
            const auto errors = static_cast<std::uint16_t>(std::min<std::uint32_t>(_parser.badFrames(), UINT16_MAX));
            packSysStatus(from, telemetry, errors, msg);
            return send(msg, nowMs);
        }

        case Stream::Attitude:
            packAttitude(from, telemetry, nowMs, msg);
            return send(msg, nowMs);

        case Stream::VfrHud:
            packVfrHud(from, telemetry, msg);
            return send(msg, nowMs);

        case Stream::ScaledImu:
            packScaledImu(from, telemetry, nowMs, msg);
            return send(msg, nowMs);

        case Stream::Values:
        {
            bool all = true;
            for (std::size_t i = 0; i < kNamedValueCount; ++i)
            {
                packValue(from, telemetry, i, nowMs, msg);
                all = send(msg, nowMs) && all;
            }
            return all;
        }

        case Stream::Count:
            break;
    }
    return false;
}

void MavlinkEndpoint::sendStatusText(std::uint32_t nowMs)
{
    while (_statusText != nullptr)
    {
        if (!_text)
        {
            StatusTextQueue::Entry entry;
            if (!_statusText->pop(entry)) { return; }
            _text      = entry;
            _textChunk = 0;

            // A line longer than one message is sent as chunks sharing an id;
            // a single-message line has id 0.
            _textId = 0;
            if (std::strlen(entry.text) > kTextChunk)
            {
                _lastTextId = static_cast<std::uint16_t>(_lastTextId + 1);
                if (_lastTextId == 0) { _lastTextId = 1; }
                _textId = _lastTextId;
            }
        }

        const std::size_t length = std::strlen(_text->text);
        const std::size_t offset = static_cast<std::size_t>(_textChunk) * kTextChunk;

        char chunk[kTextChunk + 1]{};
        if (offset < length)
        {
            std::memcpy(chunk, _text->text + offset, std::min(kTextChunk, length - offset));
        }

        mavlink_message_t msg;
        mavlink_msg_statustext_pack_status(_config.systemId, MAV_COMP_ID_AUTOPILOT1, &_txStatus, &msg,
                                           _text->severity, chunk, _textId, _textChunk);
        if (!send(msg, nowMs)) { return; }

        // A chunked line ends with a chunk shorter than 50 characters, which
        // is an empty one when the length is an exact multiple.
        const std::size_t chunks = (_textId == 0) ? 1 : length / kTextChunk + 1;
        ++_textChunk;
        if (_textChunk >= chunks) { _text.reset(); }
    }
}

void MavlinkEndpoint::sendParamList(std::uint32_t nowMs)
{
    while (_paramCursor)
    {
        const std::uint16_t index = *_paramCursor;
        if (index >= paramCount())
        {
            _paramCursor.reset();
            return;
        }

        if (const std::optional<ParamValue> value = readParam(index))
        {
            if (!sendParamValue(*value, nowMs)) { return; }
        }
        _paramCursor = static_cast<std::uint16_t>(index + 1);
    }
}

bool MavlinkEndpoint::sendParamValue(const ParamValue& value, std::uint32_t nowMs)
{
    mavlink_message_t msg;
    mavlink_msg_param_value_pack_status(_config.systemId, MAV_COMP_ID_AUTOPILOT1, &_txStatus, &msg,
                                        value.id, value.value, value.type, value.count, value.index);
    return send(msg, nowMs);
}

bool MavlinkEndpoint::sendAutopilotVersion(std::uint32_t nowMs)
{
    mavlink_message_t msg;
    packAutopilotVersion(origin(), msg);
    return send(msg, nowMs);
}

bool MavlinkEndpoint::send(const mavlink_message_t& msg, std::uint32_t nowMs)
{
    std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const std::uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);

    if (_stream.writable() < length || !_scheduler.trySpend(length, nowMs))
    {
        ++_counters.framesSkipped;
        return false;
    }
    (void)_stream.write(buffer, length);
    return true;
}

// ── Policy ──────────────────────────────────────────────────────────────────

bool MavlinkEndpoint::isForUs(std::uint8_t targetSystem) const noexcept
{
    return targetSystem == 0 || targetSystem == _config.systemId;
}

const char* MavlinkEndpoint::writeRefusal(const TelemetryData& telemetry) const noexcept
{
    if (!_config.acceptWrites) { return "writes are disabled on this port"; }

    switch (groundCommandBlock(telemetry.armed, static_cast<FlightState>(telemetry.flight_state)))
    {
        case GroundBlock::Armed:    return "armed";
        case GroundBlock::InFlight: return "in flight";
        case GroundBlock::None:     break;
    }
    return nullptr;
}

} // namespace arduflite::mavlink
