/**
 * ArduFliteDebugSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/core/ConsoleWriter.h"
#include "src/hal/board/Board.h"
#include "ArduFliteDebugSerialTelemetry.h"

ArduFliteDebugSerialTelemetry::ArduFliteDebugSerialTelemetry(float frequencyHz)
    : PeriodicTelemetryBackend("DebugTelemetryTask", frequencyHz)
{
}

void ArduFliteDebugSerialTelemetry::runLoop()
{
    // sleepUntil, not sleepFor: a fixed cadence measured from the start of each
    // iteration, so the work inside the loop does not accumulate into drift.
    std::uint64_t lastWake = 0;
    const auto period =
        std::chrono::milliseconds{ static_cast<std::int64_t>(intervalMs()) };
    TelemetryData localCopy{};

    auto& board     = arduflite::board::Board::instance();
    auto& scheduler = board.scheduler();

    // Data channel, not the diagnostic logger (ADR-044). Constructed once —
    // it holds only a reference.
    arduflite::ConsoleWriter out(board.console());

    while (shouldRun())
    {
        // A failed snapshot leaves localCopy alone, so the display holds the
        // previous frame rather than printing zeros.
        (void)snapshot(localCopy);

        // Clear screen and move cursor to home position
        out.printf("\033[2J\033[H");

        // Flight state & mode
        const char* stateStr = localCopy.flight_state == 0 ? "UNKNOWN" :
                               localCopy.flight_state == 1 ? "PREFLIGHT" :
                               localCopy.flight_state == 2 ? "INFLIGHT" : "LANDED";
        const char* modeStr  = localCopy.flight_mode == 0 ? "ATTITUDE" :
                               localCopy.flight_mode == 1 ? "RATE" :
                               localCopy.flight_mode == 2 ? "MANUAL" : "UNKNOWN";
        out.printf("Flight State: %s | Mode: %s | Armed: %s\n", stateStr, modeStr, localCopy.armed ? "YES" : "NO");

        // Altitude & climb rate
        out.printf("Altitude: %.2f m | Climb Rate: %.2f m/s\n", localCopy.altitude, localCopy.climb_rate);

        // Raw sensor data
        out.printf("Accel: %.3f, %.3f, %.3f\n", localCopy.accel.x, localCopy.accel.y, localCopy.accel.z);
        out.printf("Gyro: %.3f, %.3f, %.3f\n", localCopy.gyro.x, localCopy.gyro.y, localCopy.gyro.z);
        out.printf("Quat: %.4f, %.4f, %.4f, %.4f\n", localCopy.quat.w, localCopy.quat.x, localCopy.quat.y, localCopy.quat.z);

        // Orientation (current)
        out.printf("Orientation (P/R/Y): %.2f, %.2f, %.2f\n", localCopy.orientation.pitch, localCopy.orientation.roll, localCopy.orientation.yaw);

        out.printf("IMU Snapshot: retries=%lu | max=%lu | limit hits=%lu\n",
              (unsigned long)localCopy.imu_snapshot_retries,
              (unsigned long)localCopy.imu_snapshot_max_retries,
              (unsigned long)localCopy.imu_snapshot_retry_limit_hits);

        // Setpoints (target)
        out.printf("Attitude Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeSetpoint.roll, localCopy.attitudeSetpoint.pitch, localCopy.attitudeSetpoint.yaw);
        out.printf("Rate Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateSetpoint.roll, localCopy.rateSetpoint.pitch, localCopy.rateSetpoint.yaw);

        // Command outputs
        out.printf("Attitude Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeCmd.roll, localCopy.attitudeCmd.pitch, localCopy.attitudeCmd.yaw);
        out.printf("Rate Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateCmd.roll, localCopy.rateCmd.pitch, localCopy.rateCmd.yaw);

        out.printf("\nPress any key to stop...\n\n");

        scheduler.sleepUntil(lastWake, period);
    }
}
