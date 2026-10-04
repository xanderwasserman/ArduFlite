/**
 * test_mavlink_endpoint.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * The endpoint against a simulated ground station on an in-memory byte stream.
 * Frames are produced and checked with the vendored library itself.
 */
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <deque>
#include <set>
#include <string>
#include <vector>

#include "hal_host/HostPlatform.h"
#include "include/ConfigKeys.h"
#include "src/telemetry/mavlink/MavlinkEndpoint.h"
#include "src/utils/ConfigRegistry.h"

using namespace arduflite::mavlink;

namespace {

constexpr std::uint8_t kGcsSystem    = 255;
constexpr std::uint8_t kGcsComponent = MAV_COMP_ID_MISSIONPLANNER;

/// An in-memory port. `room` bytes may be written per service pass.
class FakeStream final : public arduflite::hal::ByteStream
{
public:
    std::deque<std::uint8_t>  toEndpoint;
    std::vector<std::uint8_t> fromEndpoint;
    std::size_t               room = 1 << 20;

    std::size_t available() override { return toEndpoint.size(); }

    std::size_t read(std::uint8_t* dst, std::size_t maxLen) override
    {
        std::size_t n = 0;
        while (n < maxLen && !toEndpoint.empty())
        {
            dst[n++] = toEndpoint.front();
            toEndpoint.pop_front();
        }
        return n;
    }

    std::size_t writable() override { return room - _writtenThisPass; }

    std::size_t write(const std::uint8_t* src, std::size_t len) override
    {
        EXPECT_LE(len, writable()) << "the endpoint wrote more than writable() allowed";
        fromEndpoint.insert(fromEndpoint.end(), src, src + len);
        _writtenThisPass += len;
        return len;
    }

    void newPass() { _writtenThisPass = 0; }

private:
    std::size_t _writtenThisPass = 0;
};

class MavlinkEndpointTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { ConfigRegistry::instance().init(); }

    void SetUp() override
    {
        queue.setMutex(mutex);
        endpoint.configure(config, &MavlinkEndpointTest::onReboot);
        rebootRequested = false;
        telemetry.flight_state = PREFLIGHT;
    }

    /// Pack a message as the ground station, and queue it for the endpoint.
    template <typename Pack>
    void fromGroundStation(Pack pack, bool mavlink1 = false)
    {
        if (mavlink1) { gcsStatus.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1; }
        mavlink_message_t msg;
        pack(gcsStatus, msg);
        std::uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const std::uint16_t length = mavlink_msg_to_send_buffer(buffer, &msg);
        stream.toEndpoint.insert(stream.toEndpoint.end(), buffer, buffer + length);
        gcsStatus.flags &= static_cast<std::uint8_t>(~MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }

    void service(int passes = 1, std::uint32_t stepMs = 10)
    {
        for (int i = 0; i < passes; ++i)
        {
            stream.newPass();
            endpoint.service(nowMs, telemetry);
            nowMs += stepMs;
        }
    }

    /// Every complete frame the endpoint has written so far.
    std::vector<mavlink_message_t> sent()
    {
        std::vector<mavlink_message_t> frames;
        mavlink_message_t buffer{}, msg{};
        mavlink_status_t  status{}, report{};
        for (const std::uint8_t byte : stream.fromEndpoint)
        {
            if (mavlink_frame_char_buffer(&buffer, &status, byte, &msg, &report) == MAVLINK_FRAMING_OK)
            {
                frames.push_back(msg);
            }
        }
        return frames;
    }

    std::vector<mavlink_message_t> sent(std::uint32_t msgId)
    {
        std::vector<mavlink_message_t> matching;
        for (const mavlink_message_t& msg : sent())
        {
            if (msg.msgid == msgId) { matching.push_back(msg); }
        }
        return matching;
    }

    void paramSet(const char* id, float value)
    {
        fromGroundStation([&](mavlink_status_t& s, mavlink_message_t& m) {
            mavlink_msg_param_set_pack_status(kGcsSystem, kGcsComponent, &s, &m, 1, 1, id, value,
                                              MAV_PARAM_TYPE_REAL32);
        });
    }

    void rebootCommand()
    {
        fromGroundStation([](mavlink_status_t& s, mavlink_message_t& m) {
            mavlink_msg_command_long_pack_status(kGcsSystem, kGcsComponent, &s, &m, 1, 1,
                                                 MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0,
                                                 1, 0, 0, 0, 0, 0, 0);
        });
    }

    static void onReboot() { rebootRequested = true; }
    static inline bool rebootRequested = false;

    EndpointConfig config{ 1, StreamIntervals{ 1000, 1000, 40, 100, 40, 200 }, 0, true };

    arduflite::hal::host::HostMutex mutex;
    StatusTextQueue                 queue;
    FakeStream                      stream;
    MavlinkEndpoint                 endpoint{ stream, &queue };
    mavlink_status_t                gcsStatus{};
    TelemetryData                   telemetry{};
    std::uint32_t                   nowMs = 1000;
};

} // namespace

TEST_F(MavlinkEndpointTest, HeartbeatAndAttitudeCarryModeArmingAndRadians)
{
    telemetry.armed            = true;
    telemetry.flight_mode      = ATTITUDE_MODE;
    telemetry.orientation.roll = 30.0f;
    telemetry.orientation.yaw  = -90.0f;
    service();

    const auto heartbeats = sent(MAVLINK_MSG_ID_HEARTBEAT);
    ASSERT_EQ(heartbeats.size(), 1u);
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&heartbeats[0], &heartbeat);
    EXPECT_EQ(heartbeat.type, MAV_TYPE_FIXED_WING);
    EXPECT_EQ(heartbeat.custom_mode, static_cast<std::uint32_t>(ATTITUDE_MODE));
    EXPECT_TRUE(heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
    EXPECT_EQ(heartbeats[0].magic, MAVLINK_STX) << "always MAVLink 2";

    const auto attitudes = sent(MAVLINK_MSG_ID_ATTITUDE);
    ASSERT_EQ(attitudes.size(), 1u);
    mavlink_attitude_t attitude;
    mavlink_msg_attitude_decode(&attitudes[0], &attitude);
    EXPECT_NEAR(attitude.roll, 0.5236f, 1e-4f);
    EXPECT_NEAR(attitude.yaw, -1.5708f, 1e-4f);
}

TEST_F(MavlinkEndpointTest, ParameterListDeliversEveryParameterOnce)
{
    fromGroundStation([](mavlink_status_t& s, mavlink_message_t& m) {
        mavlink_msg_param_request_list_pack_status(kGcsSystem, kGcsComponent, &s, &m, 1, 1);
    });
    service(20);

    std::set<std::uint16_t> indices;
    for (const mavlink_message_t& msg : sent(MAVLINK_MSG_ID_PARAM_VALUE))
    {
        mavlink_param_value_t value;
        mavlink_msg_param_value_decode(&msg, &value);
        EXPECT_EQ(value.param_count, paramCount());
        EXPECT_TRUE(indices.insert(value.param_index).second) << "index " << value.param_index << " twice";
    }
    EXPECT_EQ(indices.size(), paramCount());
}

TEST_F(MavlinkEndpointTest, ParameterWritesNeedTheGroundAndAWritablePort)
{
    const float before = readParam("RATE_RLL_KP")->value;

    telemetry.armed = true;
    paramSet("RATE_RLL_KP", 0.3f);
    service();
    EXPECT_FLOAT_EQ(readParam("RATE_RLL_KP")->value, before) << "refused while armed";

    mavlink_param_value_t echoed;
    mavlink_msg_param_value_decode(&sent(MAVLINK_MSG_ID_PARAM_VALUE).back(), &echoed);
    EXPECT_FLOAT_EQ(echoed.param_value, before) << "a refusal is answered with the current value";

    telemetry.armed = false;
    paramSet("RATE_RLL_KP", 0.3f);
    service();
    EXPECT_FLOAT_EQ(readParam("RATE_RLL_KP")->value, 0.3f);

    EndpointConfig readOnly = config;
    readOnly.acceptWrites = false;
    endpoint.configure(readOnly, &MavlinkEndpointTest::onReboot);
    paramSet("RATE_RLL_KP", 0.4f);
    service();
    EXPECT_FLOAT_EQ(readParam("RATE_RLL_KP")->value, 0.3f) << "refused on a read-only port";

    ConfigRegistry::instance().reset(CONFIG_KEY_RATE_ROLL_KP);
}

TEST_F(MavlinkEndpointTest, RebootIsAcceptedOnlyOnTheGround)
{
    telemetry.armed = true;
    rebootCommand();
    service();
    EXPECT_FALSE(rebootRequested);

    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&sent(MAVLINK_MSG_ID_COMMAND_ACK).back(), &ack);
    EXPECT_EQ(ack.result, MAV_RESULT_DENIED);

    telemetry.armed = false;
    rebootCommand();
    service();
    EXPECT_TRUE(rebootRequested);
    mavlink_msg_command_ack_decode(&sent(MAVLINK_MSG_ID_COMMAND_ACK).back(), &ack);
    EXPECT_EQ(ack.result, MAV_RESULT_ACCEPTED);
}

TEST_F(MavlinkEndpointTest, AcceptsMavlink1InboundAndAnswersInMavlink2)
{
    fromGroundStation([](mavlink_status_t& s, mavlink_message_t& m) {
        mavlink_msg_param_request_read_pack_status(kGcsSystem, kGcsComponent, &s, &m, 1, 1, "", 0);
    }, /*mavlink1=*/true);
    service();

    const auto values = sent(MAVLINK_MSG_ID_PARAM_VALUE);
    ASSERT_EQ(values.size(), 1u);
    EXPECT_EQ(values[0].magic, MAVLINK_STX);
}

TEST_F(MavlinkEndpointTest, NeverWritesAPartialFrameIntoAFullStream)
{
    stream.room = 40;   // room for a heartbeat, not for everything due
    service(50);

    std::size_t framedBytes = 0;
    for (const mavlink_message_t& msg : sent()) { framedBytes += msg.len + MAVLINK_NUM_NON_PAYLOAD_BYTES; }
    EXPECT_EQ(stream.fromEndpoint.size(), framedBytes) << "every byte written belongs to a complete frame";
    EXPECT_GT(endpoint.counters().framesSkipped, 0u);
}

TEST_F(MavlinkEndpointTest, StaysWithinTheByteBudget)
{
    EndpointConfig radio = config;
    radio.budgetBitsPerSecond = 4800;   // 600 bytes per second
    endpoint.configure(radio, &MavlinkEndpointTest::onReboot);

    service(1000);   // ten seconds of the USB stream set, far more than fits

    const std::size_t burst = 280;   // the bucket's capacity at this rate
    EXPECT_LE(stream.fromEndpoint.size(), 600u * 10 + burst);
    EXPECT_GT(stream.fromEndpoint.size(), 600u * 9) << "the budget should be used, not just respected";
}

TEST_F(MavlinkEndpointTest, LongLogLinesArriveAsChunksOfOneMessage)
{
    const std::string line(120, 'x');
    queue.push(MAV_SEVERITY_WARNING, line.c_str());
    service();

    std::string received;
    std::set<std::uint16_t> ids;
    std::uint8_t expectedChunk = 0;
    for (const mavlink_message_t& msg : sent(MAVLINK_MSG_ID_STATUSTEXT))
    {
        mavlink_statustext_t text;
        mavlink_msg_statustext_decode(&msg, &text);
        EXPECT_EQ(text.chunk_seq, expectedChunk++);
        ids.insert(text.id);
        received.append(text.text, strnlen(text.text, sizeof(text.text)));
    }
    EXPECT_EQ(expectedChunk, 3);
    ASSERT_EQ(ids.size(), 1u);
    EXPECT_NE(*ids.begin(), 0) << "a chunked line needs a non-zero id";
    EXPECT_EQ(received, line);
}

TEST(FrameParser, FindsAGroundStationAmongTypedTextAndLineNoise)
{
    mavlink_status_t  gcs{};
    mavlink_message_t heartbeat;
    mavlink_msg_heartbeat_pack_status(kGcsSystem, kGcsComponent, &gcs, &heartbeat, MAV_TYPE_GCS,
                                      MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    std::uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const std::uint16_t length = mavlink_msg_to_send_buffer(frame, &heartbeat);

    std::vector<std::uint8_t> bytes;
    for (const char c : std::string("config set rate.roll.kp 0.1\r\n")) { bytes.push_back(static_cast<std::uint8_t>(c)); }
    bytes.insert(bytes.end(), frame, frame + length);
    bytes.back() ^= 0xFF;                                    // a corrupted copy
    bytes.insert(bytes.end(), frame, frame + length);        // then a good one

    FrameParser parser;
    int frames = 0;
    for (const std::uint8_t byte : bytes)
    {
        if (const mavlink_message_t* msg = parser.feed(byte))
        {
            ++frames;
            EXPECT_EQ(msg->msgid, static_cast<std::uint32_t>(MAVLINK_MSG_ID_HEARTBEAT));
        }
    }
    EXPECT_EQ(frames, 1);
    EXPECT_EQ(parser.badFrames(), 1u);
}

