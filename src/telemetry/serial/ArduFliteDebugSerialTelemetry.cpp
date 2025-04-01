// ArduFliteDebugSerialTelemetry.cpp
#include "ArduFliteDebugSerialTelemetry.h"

ArduFliteDebugSerialTelemetry::ArduFliteDebugSerialTelemetry(float frequencyHz)
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteDebugSerialTelemetry::begin() {
    // Create a mutex to protect pendingData
    telemetryMutex = xSemaphoreCreateMutex();

    // Create a FreeRTOS task to print periodically
    xTaskCreatePinnedToCore(
        telemetryTask,
        "DebugTelemetryTask",
        4096,      // stack size
        this,      // parameter
        1,         // priority
        NULL,
        1          // run on core 1 (ESP32)
    );
}

void ArduFliteDebugSerialTelemetry::publish(const TelemetryData& data) {
    // Store new data so the task can print it
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        pendingData = data;
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

        Serial.printf("Accel: %f, %f, %f\n", localCopy.accelX, localCopy.accelY, localCopy.accelZ);
        Serial.print("Gyro: %f, %f, %f\n", localCopy.gyroX, localCopy.gyroY, localCopy.gyroZ);
        Serial.print("Q: %f, %f, %f, %f\n", localCopy.qw, localCopy.qx, localCopy.qy, localCopy.qz);
        Serial.print("Orientation: %f, %f, %f\n", localCopy.pitch, localCopy.roll, localCopy.yaw);
        Serial.print("Cmd: %f, %f, %f\n", localCopy.pitchCmd, localCopy.rollCmd, localCopy.yawCmd);

        // Delay for the remainder of the interval
        unsigned long elapsed = millis() - startMs;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
