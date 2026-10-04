/**
 * MavlinkTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_H
#define ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_H

#include "src/hal/platform/ByteStream.h"
#include "src/telemetry/PeriodicTelemetryBackend.h"
#include "src/telemetry/mavlink/MavlinkEndpoint.h"

namespace arduflite::mavlink {

/**
 * @brief A MavlinkEndpoint running in its own telemetry task.
 *
 * One instance per port: the USB console after `mavlink on`, and the telemetry
 * UART when the board has one and it is enabled. configure() before begin().
 */
class MavlinkTelemetry final : public PeriodicTelemetryBackend
{
public:
    MavlinkTelemetry(const char* taskName, float loopHz, hal::ByteStream& stream,
                     StatusTextQueue* statusText) noexcept;

    void configure(const EndpointConfig& config, MavlinkEndpoint::RebootHook reboot) noexcept
    {
        _endpoint.configure(config, reboot);
    }

protected:
    void runLoop() override;

private:
    /// Room for a frame on the stack at each level of the receive path.
    static constexpr std::uint32_t kStackBytes = 6144;

    MavlinkEndpoint _endpoint;
};

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_TELEMETRY_H
