/**
 * MavlinkTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/MavlinkTelemetry.h"

#include <chrono>

#include "src/hal/board/Board.h"

namespace arduflite::mavlink {

MavlinkTelemetry::MavlinkTelemetry(const char* taskName, float loopHz, hal::ByteStream& stream,
                                   StatusTextQueue* statusText) noexcept
    : PeriodicTelemetryBackend(taskName, loopHz, kStackBytes),
      _endpoint(stream, statusText)
{
}

void MavlinkTelemetry::runLoop()
{
    auto& board = board::Board::instance();
    auto& scheduler = board.scheduler();
    const auto& clock = board.clock();
    hal::WatchdogGuard watchdog(board.watchdog());

    const std::chrono::milliseconds period{ static_cast<std::int64_t>(intervalMs()) };
    TelemetryData telemetry{};

    while (shouldRun())
    {
        watchdog.feed();

        (void)snapshot(telemetry);
        const auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock.now().time_since_epoch()).count();
        _endpoint.service(static_cast<std::uint32_t>(nowMs), telemetry);

        scheduler.sleepFor(period);
    }
}

} // namespace arduflite::mavlink
