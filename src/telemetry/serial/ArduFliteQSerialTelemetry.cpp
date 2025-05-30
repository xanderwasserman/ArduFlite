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

ArduFliteQSerialTelemetry::ArduFliteQSerialTelemetry(float frequencyHz)
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteQSerialTelemetry::begin() {
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

void ArduFliteQSerialTelemetry::publish(const TelemetryData& data) {
    // Store new data so the task can print it
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pendingData = data;
        xSemaphoreGive(telemetryMutex);
    }
}

void ArduFliteQSerialTelemetry::telemetryTask(void* pvParameters) {
    ArduFliteQSerialTelemetry* self = static_cast<ArduFliteQSerialTelemetry*>(pvParameters);

    for (;;) {
        unsigned long startMs = millis();

        // Copy local data under mutex
        TelemetryData localCopy;
        if (xSemaphoreTake(self->telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localCopy = self->pendingData;
            xSemaphoreGive(self->telemetryMutex);
        }

        // Print the quaternion
        LOG_C("%f,%f,%f,%f", localCopy.quat.w, localCopy.quat.x, localCopy.quat.y, localCopy.quat.z);

        // Delay for the remainder of the interval
        unsigned long elapsed = millis() - startMs;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
