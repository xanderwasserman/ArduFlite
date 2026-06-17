/**
 * ArduFliteDebugSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteDebugSerialTelemetry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

ArduFliteDebugSerialTelemetry::ArduFliteDebugSerialTelemetry(float frequencyHz)
{
    // Clamp to a sane range before the reciprocal — frequencyHz==0 would produce
    // +Inf, making (int)+Inf undefined behaviour in the task delay calculation.
    frequencyHz = constrain(frequencyHz, 0.1f, 200.0f);
    _intervalMs = 1000.0f / frequencyHz;
}

ArduFliteDebugSerialTelemetry::~ArduFliteDebugSerialTelemetry()
{
    // Delete the background task first — on single-core ESP32-C3 vTaskDelete()
    // removes the task from the scheduler immediately so it cannot access
    // _pendingData or _mutex after they are destroyed below.
    if (_taskHandle)
    {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }
    if (_mutex)
    {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

void ArduFliteDebugSerialTelemetry::begin()
{
    // Idempotency guard — a second begin() call would leak the existing mutex
    // and spawn a second task that races with the first on the same _pendingData.
    if (_mutex)
    {
        LOG_WARN("DebugSerialTelemetry::begin() called more than once — ignoring");
        return;
    }

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex)
    {
        LOG_ERR("DebugSerialTelemetry: failed to create mutex — telemetry disabled");
        return;
    }

    // Store handle so destructor can stop the task cleanly.
    if (xTaskCreate(telemetryTask, "DebugTelemetryTask", 4096, this, 1, &_taskHandle) != pdPASS)
    {
        LOG_ERR("DebugSerialTelemetry: failed to create task");
        // Release the mutex we just created so the object stays in a clean state.
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

void ArduFliteDebugSerialTelemetry::publish(const TelemetryData& telemData)
{
    if (!_mutex) return;

    SemaphoreLock lock(_mutex);
    if (!lock.acquired()) return;
    _pendingData = telemData;
}

void ArduFliteDebugSerialTelemetry::telemetryTask(void* pvParameters)
{
    ArduFliteDebugSerialTelemetry* self = static_cast<ArduFliteDebugSerialTelemetry*>(pvParameters);

    // Use vTaskDelayUntil to prevent cumulative timing drift (unlike vTaskDelay,
    // which counts from the end of each iteration rather than the start).
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(self->_intervalMs);
    TelemetryData localCopy{};

    for (;;)
    {
        // Copy local data under mutex — if lock cannot be acquired keep previous
        // localCopy (stale-but-safe) rather than printing garbage zeros.
        {
            SemaphoreLock lock(self->_mutex);
            if (lock.acquired())
                localCopy = self->_pendingData;
        }

        // Clear screen and move cursor to home position
        LOG_N("\033[2J\033[H");

        // Flight state & mode
        const char* stateStr = localCopy.flight_state == 0 ? "UNKNOWN" :
                               localCopy.flight_state == 1 ? "PREFLIGHT" :
                               localCopy.flight_state == 2 ? "INFLIGHT" : "LANDED";
        const char* modeStr  = localCopy.flight_mode == 0 ? "ATTITUDE" :
                               localCopy.flight_mode == 1 ? "RATE" :
                               localCopy.flight_mode == 2 ? "MANUAL" : "UNKNOWN";
        LOG_N("Flight State: %s | Mode: %s | Armed: %s\n", stateStr, modeStr, localCopy.armed ? "YES" : "NO");

        // Altitude & climb rate
        LOG_N("Altitude: %.2f m | Climb Rate: %.2f m/s\n", localCopy.altitude, localCopy.climb_rate);

        // Raw sensor data
        LOG_N("Accel: %.3f, %.3f, %.3f\n", localCopy.accel.x, localCopy.accel.y, localCopy.accel.z);
        LOG_N("Gyro: %.3f, %.3f, %.3f\n", localCopy.gyro.x, localCopy.gyro.y, localCopy.gyro.z);
        LOG_N("Quat: %.4f, %.4f, %.4f, %.4f\n", localCopy.quat.w, localCopy.quat.x, localCopy.quat.y, localCopy.quat.z);

        // Orientation (current)
        LOG_N("Orientation (P/R/Y): %.2f, %.2f, %.2f\n", localCopy.orientation.pitch, localCopy.orientation.roll, localCopy.orientation.yaw);

        LOG_N("IMU Snapshot: retries=%lu | max=%lu | limit hits=%lu\n",
              (unsigned long)localCopy.imu_snapshot_retries,
              (unsigned long)localCopy.imu_snapshot_max_retries,
              (unsigned long)localCopy.imu_snapshot_retry_limit_hits);

        // Setpoints (target)
        LOG_N("Attitude Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeSetpoint.roll, localCopy.attitudeSetpoint.pitch, localCopy.attitudeSetpoint.yaw);
        LOG_N("Rate Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateSetpoint.roll, localCopy.rateSetpoint.pitch, localCopy.rateSetpoint.yaw);

        // Command outputs
        LOG_N("Attitude Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeCmd.roll, localCopy.attitudeCmd.pitch, localCopy.attitudeCmd.yaw);
        LOG_N("Rate Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateCmd.roll, localCopy.rateCmd.pitch, localCopy.rateCmd.yaw);

        LOG_N("\nPress any key to stop...\n\n");

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
