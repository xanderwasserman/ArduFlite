/**
 * ArduFliteDebugSerialTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteDebugSerialTelemetry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

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

void ArduFliteDebugSerialTelemetry::publish(const TelemetryData& telemData) 
{
    // Store new data so the task can print it
    {
        SemaphoreLock lock(telemetryMutex);
        pendingData = telemData; 
    }
}

void ArduFliteDebugSerialTelemetry::telemetryTask(void* pvParameters) 
{
    ArduFliteDebugSerialTelemetry* self = static_cast<ArduFliteDebugSerialTelemetry*>(pvParameters);

    for (;;) 
    {
        unsigned long startMs = millis();

        // Copy local data under mutex
        TelemetryData localCopy;
        
        {
            SemaphoreLock lock(self->telemetryMutex);
            localCopy = self->pendingData; 
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

        // Setpoints (target)
        LOG_N("Attitude Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeSetpoint.roll, localCopy.attitudeSetpoint.pitch, localCopy.attitudeSetpoint.yaw);
        LOG_N("Rate Setpoint (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateSetpoint.roll, localCopy.rateSetpoint.pitch, localCopy.rateSetpoint.yaw);

        // Command outputs
        LOG_N("Attitude Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.attitudeCmd.roll, localCopy.attitudeCmd.pitch, localCopy.attitudeCmd.yaw);
        LOG_N("Rate Cmd (R/P/Y): %.2f, %.2f, %.2f\n", localCopy.rateCmd.roll, localCopy.rateCmd.pitch, localCopy.rateCmd.yaw);

        // Delay for the remainder of the interval
        unsigned long elapsed = millis() - startMs;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
