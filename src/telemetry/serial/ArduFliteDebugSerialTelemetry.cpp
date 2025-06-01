/**
 * ArduFliteDebugSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteDebugSerialTelemetry.h"
#include "src/utils/Logging.h"

ArduFliteDebugSerialTelemetry::ArduFliteDebugSerialTelemetry(float frequencyHz)
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteDebugSerialTelemetry::begin() {
    // Create a mutex to protect pendingData
    telemetryMutex = xSemaphoreCreateMutex();

    // Create a FreeRTOS task to print periodically
    xTaskCreate(
        telemetryTask,
        "DebugTelemetryTask",
        4096,      // stack size
        this,      // parameter
        1,         // priority
        NULL
    );
}

void ArduFliteDebugSerialTelemetry::publish(const TelemetryData& telemData, const ConfigData& configData) 
{
    // Store new data so the task can print it
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) 
    {
        pendingData = telemData;
        xSemaphoreGive(telemetryMutex);
    }
}

void ArduFliteDebugSerialTelemetry::telemetryTask(void* pvParameters) {
    ArduFliteDebugSerialTelemetry* self = static_cast<ArduFliteDebugSerialTelemetry*>(pvParameters);

    for (;;) {
        unsigned long startMs = millis();

        // Copy local data under mutex
        TelemetryData localCopy;
        if (xSemaphoreTake(self->telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localCopy = self->pendingData;
            xSemaphoreGive(self->telemetryMutex);
        }

        LOG_N("\033[2J\033[H");
        LOG_N("In-flight: %s\n", localCopy.flight_state==0?"UNKNOW":localCopy.flight_state==1?"PREFLIGHT":localCopy.flight_state==2?"INFLIGHT":"LANDED");
        LOG_N("Altitude: %f\n", localCopy.altitude);
        LOG_N("Accel: %f, %f, %f\n", localCopy.accel.x, localCopy.accel.y, localCopy.accel.z);
        LOG_N("Gyro: %f, %f, %f\n", localCopy.gyro.x, localCopy.gyro.y, localCopy.gyro.z);
        LOG_N("Q: %f, %f, %f, %f\n", localCopy.quat.w, localCopy.quat.x, localCopy.quat.y, localCopy.quat.z);
        LOG_N("Orientation: %f, %f, %f\n", localCopy.orientation.pitch, localCopy.orientation.roll, localCopy.orientation.yaw);
        LOG_N("Attitude Cmd: %f, %f, %f\n", localCopy.attitudeCmd.roll, localCopy.attitudeCmd.pitch, localCopy.attitudeCmd.yaw);
        LOG_N("Rate Cmd: %f, %f, %f\n", localCopy.rateCmd.roll, localCopy.rateCmd.pitch, localCopy.rateCmd.yaw);

        // Delay for the remainder of the interval
        unsigned long elapsed = millis() - startMs;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
