/**
 * CLICommandsTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
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
    ArduFliteIMU* imu = getCliIMU();
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
    vTaskDelay(pdMS_TO_TICKS(500)); // Brief pause before clearing screen

    // Drain any pending input
    while (Serial.available()) Serial.read();

    for (;;)
    {
        unsigned long startMs = millis();

        ImuSnapshot snap = imu->getSnapshot();
        ImuSnapshotHealth snapshotHealth = imu->getSnapshotHealth();

        // Clear screen and move cursor to home position
        LOG_N("\033[2J\033[H");

        const char* stateStr = snap.flightState == UNKNOWN_STATE ? "UNKNOWN" :
                               snap.flightState == PREFLIGHT     ? "PREFLIGHT" :
                               snap.flightState == INFLIGHT      ? "INFLIGHT" : "LANDED";
        LOG_N("Flight State: %s\n", stateStr);

        LOG_N("Altitude: %.2f m | Climb Rate: %.2f m/s\n", snap.altitude, snap.climbRate);
        LOG_N("Accel: %.3f, %.3f, %.3f g\n", snap.accel.x, snap.accel.y, snap.accel.z);
        LOG_N("Gyro: %.3f, %.3f, %.3f deg/s\n", snap.gyro.x, snap.gyro.y, snap.gyro.z);
        LOG_N("Quat: %.4f, %.4f, %.4f, %.4f\n", snap.quat.w, snap.quat.x, snap.quat.y, snap.quat.z);
        LOG_N("Orientation (P/R/Y): %.2f, %.2f, %.2f deg\n",
              snap.orientation.pitch, snap.orientation.roll, snap.orientation.yaw);
        LOG_N("IMU Snapshot: retries=%lu | max=%lu | limit hits=%lu\n",
              (unsigned long)snapshotHealth.totalReadRetries,
              (unsigned long)snapshotHealth.maxReadRetries,
              (unsigned long)snapshotHealth.retryLimitHits);

        LOG_N("\nPress any key to stop...\n\n");

        if (Serial.available())
        {
            while (Serial.available()) Serial.read();  // Drain buffer
            LOG_N("\033[2J\033[H");  // Clear screen
            LOG("Streaming stopped.");
            return;
        }

        unsigned long elapsed = millis() - startMs;
        if (elapsed < intervalMs)
        {
            vTaskDelay(pdMS_TO_TICKS(intervalMs - elapsed));
        }
    }
}
