/**
 * ArduFliteQSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 25 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteQSerialTelemetry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

ArduFliteQSerialTelemetry::ArduFliteQSerialTelemetry(float frequencyHz)
{
    // Clamp to a sane range before the reciprocal — frequencyHz==0 would produce
    // +Inf, making (int)+Inf undefined behaviour in the task delay calculation.
    frequencyHz = constrain(frequencyHz, 0.1f, 200.0f);
    _intervalMs = 1000.0f / frequencyHz;
}

ArduFliteQSerialTelemetry::~ArduFliteQSerialTelemetry()
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

void ArduFliteQSerialTelemetry::begin()
{
    // Idempotency guard — a second begin() call would leak the existing mutex
    // and spawn a second task that races with the first on the same _pendingData.
    if (_mutex)
    {
        LOG_WARN("QSerialTelemetry::begin() called more than once — ignoring");
        return;
    }

    _mutex = xSemaphoreCreateMutex();
    if (!_mutex)
    {
        LOG_ERR("QSerialTelemetry: failed to create mutex — telemetry disabled");
        return;
    }

    // Store handle so destructor can stop the task cleanly.
    if (xTaskCreate(telemetryTask, "SerialTelTask", 4096, this, 1, &_taskHandle) != pdPASS)
    {
        LOG_ERR("QSerialTelemetry: failed to create task");
        // Release the mutex we just created so the object stays in a clean state.
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

void ArduFliteQSerialTelemetry::publish(const TelemetryData& telemData)
{
    if (!_mutex) return;

    // Skip this sample if the mutex cannot be acquired — consistent with Flash backend.
    {
        SemaphoreLock lock(_mutex);
        if (!lock.acquired()) return;
        _pendingData = telemData;
    }
}

void ArduFliteQSerialTelemetry::telemetryTask(void* pvParameters)
{
    // This task streams quaternion attitude data to serial for real-time
    // attitude visualisation (the "Q" prefix denotes quaternion-only output).
    // It is intentional that only the quaternion is logged — other backends
    // (FlashTelemetry) capture the full TelemetryData struct.
    ArduFliteQSerialTelemetry* self = static_cast<ArduFliteQSerialTelemetry*>(pvParameters);

    // Use vTaskDelayUntil to prevent cumulative timing drift (unlike vTaskDelay,
    // which counts from the end of each iteration rather than the start).
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(self->_intervalMs);

    for (;;)
    {
        // Copy latest quaternion data under the mutex.
        // If the lock cannot be acquired, keep localCopy from the previous
        // iteration (stale-but-safe) rather than logging garbage zeros.
        TelemetryData localCopy;
        bool gotFreshData = false;

        {
            SemaphoreLock lock(self->_mutex);
            if (lock.acquired())
            {
                localCopy    = self->_pendingData;
                gotFreshData = true;
            }
        }

        if (gotFreshData)
        {
            // Print the quaternion — w,x,y,z CSV format consumed by visualisation tools.
            LOG("%f,%f,%f,%f",
                localCopy.quat.w, localCopy.quat.x,
                localCopy.quat.y, localCopy.quat.z);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
