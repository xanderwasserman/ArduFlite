/**
 * MavlinkEndpoint.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_ENDPOINT_H
#define ARDUFLITE_TELEMETRY_MAVLINK_ENDPOINT_H

#include <cstddef>
#include <cstdint>
#include <optional>

#include "src/hal/platform/ByteStream.h"
#include "src/telemetry/TelemetryData.h"
#include "src/telemetry/mavlink/FrameParser.h"
#include "src/telemetry/mavlink/MavlinkParams.h"
#include "src/telemetry/mavlink/MessageScheduler.h"
#include "src/telemetry/mavlink/StatusText.h"
#include "src/telemetry/mavlink/TelemetryMessages.h"

namespace arduflite::mavlink {

struct EndpointConfig
{
    std::uint8_t    systemId = 1;
    StreamIntervals intervals{};

    /// 0: limited only by the stream's own write space.
    std::uint32_t   budgetBitsPerSecond = 0;

    /// Parameter writes and reboot. Telemetry rate requests are always honoured.
    bool            acceptWrites = true;
};

/**
 * @brief MAVLink 2 on one byte stream: the protocol, without the task.
 *
 * service() is called periodically with the latest telemetry. It reads and
 * answers whatever has arrived, then sends what is due: periodic streams, queued
 * status text, and the rest of a parameter list in progress. It never blocks —
 * a message that does not fit the stream's write space or the byte budget is
 * skipped and counted.
 *
 * Sends MAVLink 2 only and accepts MAVLink 1 inbound (D7). State-changing
 * requests are refused unless the port accepts writes and the aircraft is on
 * the ground (groundCommandBlock(), ADR-068).
 */
class MavlinkEndpoint
{
public:
    using RebootHook = void (*)();

    struct Counters
    {
        std::uint32_t framesIn      = 0;
        std::uint32_t framesBad     = 0;   ///< CRC or signature failures
        std::uint32_t framesSkipped = 0;   ///< outbound, no room or no budget
    };

    MavlinkEndpoint(hal::ByteStream& stream, StatusTextQueue* statusText) noexcept;

    /// Call before the first service().
    void configure(const EndpointConfig& config, RebootHook reboot) noexcept;

    void service(std::uint32_t nowMs, const TelemetryData& telemetry);

    [[nodiscard]] Counters counters() const noexcept
    {
        Counters counters = _counters;
        counters.framesBad = _parser.badFrames();
        return counters;
    }

private:
    void receive(std::uint32_t nowMs, const TelemetryData& telemetry);
    void handle(const mavlink_message_t& msg, std::uint32_t nowMs, const TelemetryData& telemetry);
    void handleParamRead(const mavlink_message_t& msg, std::uint32_t nowMs);
    void handleParamSet(const mavlink_message_t& msg, std::uint32_t nowMs, const TelemetryData& telemetry);
    void handleCommand(const mavlink_message_t& msg, std::uint32_t nowMs, const TelemetryData& telemetry);

    /// MAV_RESULT for a COMMAND_LONG.
    std::uint8_t runCommand(const mavlink_command_long_t& cmd, std::uint32_t nowMs,
                            const TelemetryData& telemetry);

    void sendStreams(std::uint32_t nowMs, const TelemetryData& telemetry);
    bool sendStream(Stream stream, std::uint32_t nowMs, const TelemetryData& telemetry);
    void sendStatusText(std::uint32_t nowMs);
    void sendParamList(std::uint32_t nowMs);
    bool sendParamValue(const ParamValue& value, std::uint32_t nowMs);
    bool sendAutopilotVersion(std::uint32_t nowMs);

    /// Write @p msg if the stream has room and the budget allows it.
    bool send(const mavlink_message_t& msg, std::uint32_t nowMs);

    [[nodiscard]] bool isForUs(std::uint8_t targetSystem) const noexcept;

    /// Why a state-changing request must be refused, or nullptr if it may run.
    [[nodiscard]] const char* writeRefusal(const TelemetryData& telemetry) const noexcept;

    [[nodiscard]] Origin origin() noexcept { return { _config.systemId, MAV_COMP_ID_AUTOPILOT1, &_txStatus }; }

    hal::ByteStream& _stream;
    StatusTextQueue* _statusText;
    EndpointConfig   _config{};
    RebootHook       _reboot = nullptr;
    MessageScheduler _scheduler;
    Counters         _counters{};

    mavlink_status_t _txStatus{};
    FrameParser      _parser;

    std::optional<std::uint16_t> _paramCursor;   ///< next index of a list in progress

    std::optional<StatusTextQueue::Entry> _text;   ///< line being sent, chunk by chunk
    std::uint8_t  _textChunk  = 0;
    std::uint16_t _textId     = 0;                 ///< 0 for a single-chunk line
    std::uint16_t _lastTextId = 0;
};

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_ENDPOINT_H
