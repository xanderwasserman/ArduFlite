#include "ArduFliteMqttTelemetry.h"
#include "TelemetryData.h"

SemaphoreHandle_t telemetryMutex;
TelemetryData telemetryData;

ArduFliteMqttTelemetry::ArduFliteMqttTelemetry(PubSubClient& client, float frequencyHz)
    : mqttClient(client) {
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteMqttTelemetry::begin() {
    telemetryMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(
        telemetryTask, "TelemetryTask", 4096, this, 1, NULL, 1
    );
}

void ArduFliteMqttTelemetry::telemetryTask(void* pvParameters) {
    auto* self = static_cast<ArduFliteMqttTelemetry*>(pvParameters);
    TelemetryData localCopy;

    while (true) {
      unsigned long lastTick = millis();

      if (xSemaphoreTake(telemetryMutex, portMAX_DELAY)) {
          localCopy = telemetryData;  // copy data under mutex protection
          xSemaphoreGive(telemetryMutex);
      }

      self->publish(localCopy);

      unsigned long elapsed = millis() - lastTick;
      uint32_t delayMs = (uint32_t)max(1.0f, self->intervalMs - elapsed);
      vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void ArduFliteMqttTelemetry::publish(const TelemetryData& data) {
    mqttClient.publish("arduflite/accel/x", String(data.accelX, 3).c_str());
    mqttClient.publish("arduflite/accel/y", String(data.accelY, 3).c_str());
    mqttClient.publish("arduflite/accel/z", String(data.accelZ, 3).c_str());

    mqttClient.publish("arduflite/gyro/x", String(data.gyroX, 3).c_str());
    mqttClient.publish("arduflite/gyro/y", String(data.gyroY, 3).c_str());
    mqttClient.publish("arduflite/gyro/z", String(data.gyroZ, 3).c_str());

    mqttClient.publish("arduflite/quaternion/w", String(data.qw, 4).c_str());
    mqttClient.publish("arduflite/quaternion/x", String(data.qx, 4).c_str());
    mqttClient.publish("arduflite/quaternion/y", String(data.qy, 4).c_str());
    mqttClient.publish("arduflite/quaternion/z", String(data.qz, 4).c_str());

    mqttClient.publish("arduflite/orientation/pitch", String(data.pitch, 2).c_str());
    mqttClient.publish("arduflite/orientation/roll", String(data.roll, 2).c_str());
    mqttClient.publish("arduflite/orientation/yaw", String(data.yaw, 2).c_str());

    mqttClient.publish("arduflite/commands/rollCmd", String(data.rollCmd, 3).c_str());
    mqttClient.publish("arduflite/commands/pitchCmd", String(data.pitchCmd, 3).c_str());
    mqttClient.publish("arduflite/commands/yawCmd", String(data.yawCmd, 3).c_str());
}
