/**
 * ArduFliteQSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteQSerialTelemetry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

ArduFliteQSerialTelemetry::ArduFliteQSerialTelemetry(float frequencyHz)
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteQSerialTelemetry::begin() 
{
    // Create a mutex to protect pendingData
    telemetryMutex = xSemaphoreCreateMutex();

    // Create a FreeRTOS task to print periodically
    xTaskCreate(
        telemetryTask,
        "SerialTelemetryTask",
        4096,      // stack size
        this,      // parameter
        1,         // priority
        NULL
    );
}

void ArduFliteQSerialTelemetry::publish(const TelemetryData& telemData, const ConfigData& configData)
{
    // Store new data so the task can print it
    {
        SemaphoreLock lock(telemetryMutex);
        pendingData = telemData; 
    }
}

void ArduFliteQSerialTelemetry::telemetryTask(void* pvParameters) 
{
    ArduFliteQSerialTelemetry* self = static_cast<ArduFliteQSerialTelemetry*>(pvParameters);

    for (;;) 
    {
        unsigned long startMs = millis();

        // Copy local data under mutex
        TelemetryData localCopy;

        {
            SemaphoreLock lock(self->telemetryMutex);
            localCopy = self->pendingData;
        }

        // Print the quaternion
        LOG("%f,%f,%f,%f", localCopy.quat.w, localCopy.quat.x, localCopy.quat.y, localCopy.quat.z);

        // Delay for the remainder of the interval
        unsigned long elapsed = millis() - startMs;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
