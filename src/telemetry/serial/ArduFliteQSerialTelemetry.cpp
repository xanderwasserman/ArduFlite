/**
 * ArduFliteQSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/core/ConsoleWriter.h"
#include "src/hal/board/Board.h"
#include "ArduFliteQSerialTelemetry.h"
#include "src/utils/Logging.h"

ArduFliteQSerialTelemetry::ArduFliteQSerialTelemetry(float frequencyHz)
    : PeriodicTelemetryBackend("SerialTelTask", frequencyHz)
{
}

void ArduFliteQSerialTelemetry::runLoop()
{
    // This task streams quaternion attitude data to serial for real-time
    // attitude visualisation (the "Q" prefix denotes quaternion-only output).
    // It is intentional that only the quaternion is logged — other backends
    // (FlashTelemetry) capture the full TelemetryData struct.
    // sleepUntil, not sleepFor: a fixed cadence measured from the start of each
    // iteration, so work inside the loop does not accumulate into drift.
    std::uint64_t lastWake = 0;
    const auto period =
        std::chrono::milliseconds{ static_cast<std::int64_t>(intervalMs()) };

    auto& board     = arduflite::board::Board::instance();
    auto& scheduler = board.scheduler();

    // Constructed once — it holds only a reference to the console.
    arduflite::ConsoleWriter out(board.console());

    TelemetryData localCopy{};

    while (shouldRun())
    {
        // SKIP the row on a lock timeout rather than repeating the last one.
        // The output is a CSV row for a plotter, so a duplicate is a
        // fabricated sample — worse than a gap, which the timestamps show.
        // The other backends reuse instead; snapshot() reports staleness so
        // each can choose.
        if (snapshot(localCopy))
        {
            // The quaternion as w,x,y,z CSV, consumed by visualisation tools.
            // Written straight to the console, NOT through the logger: a log
            // line landing mid-row would corrupt what the plotter parses, and
            // `log off` would silently stop the stream (ADR-044).
            out.printf("%f,%f,%f,%f\n",
                       localCopy.quat.w, localCopy.quat.x,
                       localCopy.quat.y, localCopy.quat.z);
        }

        scheduler.sleepUntil(lastWake, period);
    }
}
