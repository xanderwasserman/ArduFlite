// ArduFliteMqttTelemetry.cpp

#include "ArduFliteMqttTelemetry.h"
#include <Arduino.h>

ArduFliteMqttTelemetry::ArduFliteMqttTelemetry(float frequencyHz)
  : custom_mqtt_server("server", "MQTT Server", mqttServer.c_str(), 40)
  , custom_mqtt_port("port", "MQTT Port", String(mqttPort).c_str(), 6)
  , custom_mqtt_user("user", "MQTT Username", mqttUser.c_str(), 32)
  , custom_mqtt_pass("pass", "MQTT Password", mqttPass.c_str(), 32)
  , mqttClient(wifiClient)  // also call the PubSubClient ctor
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteMqttTelemetry::begin() {
     // If the task is already running, do nothing
     if (taskHandle) {
        return;
    }

    // Create the FreeRTOS task that will handle WiFi + MQTT
    BaseType_t result = xTaskCreatePinnedToCore(
        telemetryTask,
        "MqttTelemetryTask",
        8192,           // stack size
        this,           // task parameter
        1,              // priority
        &taskHandle,    // store the handle
        1               // run on core 1 (ESP32)
    );

    if (result != pdPASS) {
        Serial.println("Failed to create MqttTelemetryTask!");
    }
}

void ArduFliteMqttTelemetry::publish(const TelemetryData& data) {
    // Store new data to a local buffer (pendingData),
    // so the telemetryTask can publish it in the background
    if (telemetryMutex && xSemaphoreTake(telemetryMutex, 10) == pdTRUE) {
        pendingData = data; 
        xSemaphoreGive(telemetryMutex);
    }
}

void ArduFliteMqttTelemetry::connectToMqtt() {
    if (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT... ");
        if (mqttUser.length() > 0) {
            if (mqttClient.connect("ArduFlite", mqttUser.c_str(), mqttPass.c_str())) {
                Serial.println("connected with auth");
            } else {
                Serial.print("failed, rc=");
                Serial.println(mqttClient.state());
            }
        } else {
            // No user/pass
            if (mqttClient.connect("ArduFlite")) {
                Serial.println("connected");
            } else {
                Serial.print("failed, rc=");
                Serial.println(mqttClient.state());
            }
        }
    }
}

void ArduFliteMqttTelemetry::telemetryTask(void* pvParameters) {
    auto* self = static_cast<ArduFliteMqttTelemetry*>(pvParameters);

    // ----------------------
    // 1) WiFiManager config
    // ----------------------
    // Add the member parameters to wifiManager
    self->wifiManager.addParameter(&self->custom_mqtt_server);
    self->wifiManager.addParameter(&self->custom_mqtt_port);
    self->wifiManager.addParameter(&self->custom_mqtt_user);
    self->wifiManager.addParameter(&self->custom_mqtt_pass);

    // Optionally set timeouts:
    // self->wifiManager.setConfigPortalTimeout(30); // e.g. 30s

    // Attempt to connect or open the config portal
    if (!self->wifiManager.autoConnect("ArduFliteAP")) {
        Serial.println("WiFi connection failed. Telemetry task will exit.");
        // If we want the task to keep trying, we could do a loop. Otherwise, stop.
        vTaskDelete(nullptr); // kill this task
        return;
    }

    // If we get here, WiFi is connected. Grab user-provided MQTT credentials
    self->mqttServer = self->custom_mqtt_server.getValue();
    self->mqttPort   = atoi(self->custom_mqtt_port.getValue());
    self->mqttUser   = self->custom_mqtt_user.getValue();
    self->mqttPass   = self->custom_mqtt_pass.getValue();

    // Set up the MQTT client
    self->mqttClient.setServer(self->mqttServer.c_str(), self->mqttPort);

    // Create a mutex for data
    self->telemetryMutex = xSemaphoreCreateMutex();

    Serial.println("Telemetry WiFi + MQTT setup complete. Entering publish loop...");

    // ----------------------
    // 2) Main publish loop
    // ----------------------
    while(true) {
        unsigned long startMillis = millis();

        // Make sure MQTT is connected
        self->connectToMqtt();

        TelemetryData localCopy;
        // Copy data under mutex
        if (xSemaphoreTake(self->telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            localCopy = self->pendingData;
            xSemaphoreGive(self->telemetryMutex);
        }

        // Publish it
        if (self->mqttClient.connected()) {
            // You could optionally check return codes if needed
            self->mqttClient.publish("arduflite/accel/x", String(localCopy.accelX, 3).c_str());
            self->mqttClient.publish("arduflite/accel/y", String(localCopy.accelY, 3).c_str());
            self->mqttClient.publish("arduflite/accel/z", String(localCopy.accelZ, 3).c_str());

            self->mqttClient.publish("arduflite/gyro/x", String(localCopy.gyroX, 3).c_str());
            self->mqttClient.publish("arduflite/gyro/y", String(localCopy.gyroY, 3).c_str());
            self->mqttClient.publish("arduflite/gyro/z", String(localCopy.gyroZ, 3).c_str());

            self->mqttClient.publish("arduflite/quaternion/w", String(localCopy.qw, 4).c_str());
            self->mqttClient.publish("arduflite/quaternion/x", String(localCopy.qx, 4).c_str());
            self->mqttClient.publish("arduflite/quaternion/y", String(localCopy.qy, 4).c_str());
            self->mqttClient.publish("arduflite/quaternion/z", String(localCopy.qz, 4).c_str());

            self->mqttClient.publish("arduflite/orientation/pitch", String(localCopy.pitch, 2).c_str());
            self->mqttClient.publish("arduflite/orientation/roll", String(localCopy.roll, 2).c_str());
            self->mqttClient.publish("arduflite/orientation/yaw", String(localCopy.yaw, 2).c_str());

            self->mqttClient.publish("arduflite/commands/rollCmd", String(localCopy.rollCmd, 3).c_str());
            self->mqttClient.publish("arduflite/commands/pitchCmd", String(localCopy.pitchCmd, 3).c_str());
            self->mqttClient.publish("arduflite/commands/yawCmd", String(localCopy.yawCmd, 3).c_str());
        }

        // Let PubSubClient handle incoming messages (if you care about subscriptions)
        self->mqttClient.loop();

        // Delay enough to match frequency
        unsigned long elapsed = millis() - startMillis;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void ArduFliteMqttTelemetry::reset()
{
    Serial.println("ArduFliteMqttTelemetry::reset() called. Stopping task & clearing Wi-Fi credentials...");

    // Disconnect MQTT
    mqttClient.disconnect();

    // Clear Wi-Fi credentials so that the next autoConnect() shows the portal
    wifiManager.resetSettings();

    // Stop the current telemetry task if running
    if (taskHandle) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    // Start again
    begin();
}