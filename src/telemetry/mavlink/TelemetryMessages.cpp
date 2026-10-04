/**
 * TelemetryMessages.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/TelemetryMessages.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace arduflite::mavlink {

namespace {

constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;

/// SCALED_IMU magnetometer unit: 1 milligauss = 0.1 microtesla.
constexpr float kMilligaussPerMicrotesla = 10.0f;

/// SYS_STATUS battery fields that mean "not measured".
constexpr std::uint16_t kNoVoltage   = UINT16_MAX;
constexpr std::int16_t  kNoCurrent   = -1;
constexpr std::int8_t   kNoRemaining = -1;

std::int16_t saturate16(float value)
{
    const float clamped = std::clamp(value, -32768.0f, 32767.0f);
    return static_cast<std::int16_t>(std::lround(clamped));
}

std::uint8_t baseMode(const TelemetryData& t)
{
    unsigned mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    if (t.flight_mode != MANUAL_MODE) { mode |= MAV_MODE_FLAG_STABILIZE_ENABLED; }
    if (t.armed)                      { mode |= MAV_MODE_FLAG_SAFETY_ARMED; }
    return static_cast<std::uint8_t>(mode);
}

std::uint8_t systemState(const TelemetryData& t)
{
    if (t.in_failsafe) { return MAV_STATE_CRITICAL; }
    return t.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;
}

} // namespace

void packHeartbeat(const Origin& origin, const TelemetryData& t, mavlink_message_t& out)
{
    mavlink_msg_heartbeat_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                      MAV_TYPE_FIXED_WING, MAV_AUTOPILOT_GENERIC, baseMode(t),
                                      static_cast<std::uint32_t>(t.flight_mode), systemState(t));
}

void packSysStatus(const Origin& origin, const TelemetryData& t, std::uint16_t commErrors,
                   mavlink_message_t& out)
{
    std::uint32_t present = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL
                          | MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
    std::uint32_t healthy = 0;

    if (t.imu_healthy)                         { healthy |= MAV_SYS_STATUS_SENSOR_3D_GYRO
                                                          | MAV_SYS_STATUS_SENSOR_3D_ACCEL; }
    if (!t.in_failsafe && t.link_quality > 0)  { healthy |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER; }
    if (t.mag_valid)
    {
        present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
        healthy |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    }

    const std::uint16_t voltage   = t.battery_valid
        ? static_cast<std::uint16_t>(std::lround(t.battery_voltage * 1000.0f)) : kNoVoltage;
    const std::int16_t  current   = t.battery_valid
        ? saturate16(t.battery_current * 100.0f) : kNoCurrent;
    const std::int8_t   remaining = t.battery_valid
        ? static_cast<std::int8_t>(std::min<std::uint8_t>(t.battery_remaining, 100)) : kNoRemaining;

    mavlink_msg_sys_status_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                       present, present, healthy, 0, voltage, current, remaining,
                                       0, commErrors, 0, 0, 0, 0, 0, 0, 0);
}

void packAttitude(const Origin& origin, const TelemetryData& t, std::uint32_t timeMs,
                  mavlink_message_t& out)
{
    mavlink_msg_attitude_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                     timeMs,
                                     t.orientation.roll  * kDegToRad,
                                     t.orientation.pitch * kDegToRad,
                                     t.orientation.yaw   * kDegToRad,
                                     t.gyro.x * kDegToRad,
                                     t.gyro.y * kDegToRad,
                                     t.gyro.z * kDegToRad);
}

void packVfrHud(const Origin& origin, const TelemetryData& t, mavlink_message_t& out)
{
    const float heading     = (t.orientation.yaw < 0.0f) ? t.orientation.yaw + 360.0f : t.orientation.yaw;
    const long  headingDeg  = std::lround(heading) % 360;
    const long  throttlePct = std::lround(std::clamp(t.throttle, 0.0f, 1.0f) * 100.0f);

    mavlink_msg_vfr_hud_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                    0.0f, 0.0f, static_cast<std::int16_t>(headingDeg),
                                    static_cast<std::uint16_t>(throttlePct), t.altitude, t.climb_rate);
}

void packScaledImu(const Origin& origin, const TelemetryData& t, std::uint32_t timeMs,
                   mavlink_message_t& out)
{
    constexpr float kMilli = 1000.0f;
    mavlink_msg_scaled_imu_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                       timeMs,
                                       saturate16(t.accel.x * kMilli),
                                       saturate16(t.accel.y * kMilli),
                                       saturate16(t.accel.z * kMilli),
                                       saturate16(t.gyro.x * kDegToRad * kMilli),
                                       saturate16(t.gyro.y * kDegToRad * kMilli),
                                       saturate16(t.gyro.z * kDegToRad * kMilli),
                                       saturate16(t.mag.x * kMilligaussPerMicrotesla),
                                       saturate16(t.mag.y * kMilligaussPerMicrotesla),
                                       saturate16(t.mag.z * kMilligaussPerMicrotesla),
                                       0);
}

void packValue(const Origin& origin, const TelemetryData& t, std::size_t index, std::uint32_t timeMs,
               mavlink_message_t& out)
{
    struct Named { const char* name; float value; };
    const Named values[kNamedValueCount] = {
        { "ATT_SP_R",  t.attitudeSetpoint.roll  },
        { "ATT_SP_P",  t.attitudeSetpoint.pitch },
        { "ATT_SP_Y",  t.attitudeSetpoint.yaw   },
        { "RATE_SP_R", t.rateSetpoint.roll      },
        { "RATE_SP_P", t.rateSetpoint.pitch     },
        { "RATE_SP_Y", t.rateSetpoint.yaw       },
        { "MAG_HDG",   t.mag_heading            },
        { "MAG_UT",    t.mag_field              },
        { "RC_LQ",     static_cast<float>(t.link_quality) },
    };

    const Named& value = values[index];
    mavlink_msg_named_value_float_pack_status(origin.systemId, origin.componentId, origin.status,
                                              &out, timeMs, value.name, value.value);
}

void packAutopilotVersion(const Origin& origin, mavlink_message_t& out)
{
    constexpr std::uint64_t kCapabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2
                                          | MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT
                                          | MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST;
    constexpr std::uint8_t kNoCustomVersion[8]{};
    constexpr std::uint8_t kNoUid2[18]{};

    mavlink_msg_autopilot_version_pack_status(origin.systemId, origin.componentId, origin.status, &out,
                                              kCapabilities, 0, 0, 0, 0,
                                              kNoCustomVersion, kNoCustomVersion, kNoCustomVersion,
                                              0, 0, 0, kNoUid2);
}

} // namespace arduflite::mavlink
