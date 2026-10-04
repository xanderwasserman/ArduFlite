/**
 * CLICommandsTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include <chrono>

#include "src/hal/board/Board.h"
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/utils/Logging.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * @brief Stream live IMU telemetry to the serial console.
 *
 * Continuously prints sensor data until any key is pressed.
 * Uses screen clearing for a live dashboard view.
 */
void cmdStream(const String &args)
{
    arduflite::estimation::InertialSubsystem* imu = getCliIMU();
    if (!imu)
    {
        LOG_ERR("IMU not set!");
        return;
    }

    // Parse optional frequency argument (default 1 Hz)
    float freqHz = 1.0f;
    String freqArg = args;
    freqArg.trim();
    if (!freqArg.isEmpty())
    {
        float parsed = 0.0f;
        if (!parseFloatStrict(freqArg, parsed))
        {
            LOG_ERR("stream: frequency must be a number from 0.1 to 100 Hz");
            return;
        }
        if (parsed > 0.0f && parsed <= 100.0f)
        {
            freqHz = parsed;
        }
        else
        {
            LOG_ERR("stream: frequency must be > 0 and <= 100 Hz");
            return;
        }
    }
    const unsigned long intervalMs = static_cast<unsigned long>(1000.0f / freqHz);

    LOG("Streaming telemetry at %.1f Hz. Press any key to stop...\n", freqHz);

    auto&       board     = arduflite::board::Board::instance();
    auto&       scheduler = board.scheduler();
    const auto& clock     = board.clock();

    scheduler.sleepFor(std::chrono::milliseconds{ 500 });  // let the line be read

    // Drain any pending input
    { auto& c = arduflite::board::Board::instance().console();
      while (c.available()) { (void)c.readByte(); } }

    for (;;)
    {
        const auto startTime = clock.now();

        const arduflite::estimation::ImuState snap = imu->state();
        const auto snapshotHealth = imu->snapshotHealth();

        // Clear screen and move cursor to home position
        LOG_N("\033[2J\033[H");

        const char* stateStr = getFlightState() == UNKNOWN_STATE ? "UNKNOWN" :
                               getFlightState() == PREFLIGHT     ? "PREFLIGHT" :
                               getFlightState() == INFLIGHT      ? "INFLIGHT" : "LANDED";
        LOG_N("Flight State: %s\n", stateStr);

        LOG_N("Altitude: %.2f m | Climb Rate: %.2f m/s\n", snap.altitude_m, snap.climbRate_mps);
        LOG_N("Accel: %.3f, %.3f, %.3f g\n", snap.accel_g.x, snap.accel_g.y, snap.accel_g.z);
        LOG_N("Gyro: %.3f, %.3f, %.3f deg/s\n", snap.gyro_dps.x, snap.gyro_dps.y, snap.gyro_dps.z);
        LOG_N("Quat: %.4f, %.4f, %.4f, %.4f\n", snap.orientation_quat.w, snap.orientation_quat.x, snap.orientation_quat.y, snap.orientation_quat.z);
        LOG_N("Orientation (P/R/Y): %.2f, %.2f, %.2f deg\n",
              snap.euler_deg.pitch, snap.euler_deg.roll, snap.euler_deg.yaw);
        LOG_N("IMU Snapshot: retries=%lu | max=%lu | limit hits=%lu\n",
              (unsigned long)snapshotHealth.totalRetries,
              (unsigned long)snapshotHealth.maxRetries,
              (unsigned long)snapshotHealth.retryLimitHits);

        LOG_N("\nPress any key to stop...\n\n");

        if (arduflite::board::Board::instance().console().available())
        {
            { auto& c = arduflite::board::Board::instance().console();
      while (c.available()) { (void)c.readByte(); } }  // Drain buffer
            LOG_N("\033[2J\033[H");  // Clear screen
            LOG("Streaming stopped.");
            return;
        }

        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            clock.now() - startTime);
        const std::chrono::milliseconds interval{ static_cast<std::int64_t>(intervalMs) };
        if (elapsed < interval)
        {
            scheduler.sleepFor(interval - elapsed);
        }
    }
}
