/**
 * TelemetryMessages.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief TelemetryData as MAVLink messages: the unit conversions live here.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_MESSAGES_H
#define ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_MESSAGES_H

#include <cstddef>
#include <cstdint>

#include "src/telemetry/TelemetryData.h"
#include "src/telemetry/mavlink/Mavlink.h"

namespace arduflite::mavlink {

/// Who a message is from, and the sequence state of the link it goes out on.
struct Origin
{
    std::uint8_t      systemId    = 1;
    std::uint8_t      componentId = MAV_COMP_ID_AUTOPILOT1;
    mavlink_status_t* status      = nullptr;
};

/// Number of NAMED_VALUE_FLOAT values packValue() can produce.
inline constexpr std::size_t kNamedValueCount = 9;

/// custom_mode carries ArduFliteMode; the armed flag and system state come
/// from the snapshot.
void packHeartbeat(const Origin& origin, const TelemetryData& t, mavlink_message_t& out);

/// @param commErrors frames this link has rejected, reported as errors_comm.
void packSysStatus(const Origin& origin, const TelemetryData& t, std::uint16_t commErrors,
                   mavlink_message_t& out);

void packAttitude(const Origin& origin, const TelemetryData& t, std::uint32_t timeMs,
                  mavlink_message_t& out);

void packVfrHud(const Origin& origin, const TelemetryData& t, mavlink_message_t& out);

void packScaledImu(const Origin& origin, const TelemetryData& t, std::uint32_t timeMs,
                   mavlink_message_t& out);

/// One of the named values: setpoints, magnetometer heading and field, and
/// link quality. @p index is below kNamedValueCount.
void packValue(const Origin& origin, const TelemetryData& t, std::size_t index, std::uint32_t timeMs,
               mavlink_message_t& out);

/// Advertises MAVLink 2, float parameters and the C-cast parameter encoding.
void packAutopilotVersion(const Origin& origin, mavlink_message_t& out);

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_MESSAGES_H
