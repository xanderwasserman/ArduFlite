// ArduFliteMqttTelemetry.cpp

#include "ArduFliteMqttTelemetry.h"
#include <Arduino.h>

ArduFliteMqttTelemetry::ArduFliteMqttTelemetry(float frequencyHz) : mqttClient(wifiClient) 
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;
}

void ArduFliteMqttTelemetry::begin() {
    // Only do this once if the task isn't already running.
    // If you expect multiple calls to begin(), guard or stop any existing task.
    if (taskHandle) {
        // Task is already running. Optionally return or stop the task first.
        return;
    }

    // 1) Setup Wi-Fi manager parameters
    WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqttServer.c_str(), 40);
    WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", String(mqttPort).c_str(), 6);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT Username", mqttUser.c_str(), 32);
    WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqttPass.c_str(), 32);

    WiFiManager wifiManager;
    wifiManager.addParameter(&custom_mqtt_server);
    wifiManager.addParameter(&custom_mqtt_port);
    wifiManager.addParameter(&custom_mqtt_user);
    wifiManager.addParameter(&custom_mqtt_pass);

    // Optional: don't re-use old credentials
    // wifiManager.resetSettings();

    // 2) Connect or start AP
    if (!wifiManager.autoConnect("ArduFliteAP")) {
        Serial.println("WiFi connection failed. Restarting...");
        ESP.restart();
    }

    // 3) Capture user-provided MQTT credentials
    mqttServer = custom_mqtt_server.getValue();
    mqttPort   = atoi(custom_mqtt_port.getValue());
    mqttUser   = custom_mqtt_user.getValue();
    mqttPass   = custom_mqtt_pass.getValue();

    // Configure MQTT server
    mqttClient.setServer(mqttServer.c_str(), mqttPort);

    // 4) Create a mutex to protect telemetry data
    telemetryMutex = xSemaphoreCreateMutex();

    // 5) Create a FreeRTOS task that repeatedly publishes data
    BaseType_t result = xTaskCreatePinnedToCore(
        telemetryTask,
        "TelemetryTask",
        4096,        // stack size
        this,        // parameter
        1,           // priority
        &taskHandle, // <--- store the handle
        1            // run on core 1
    );

    if (result != pdPASS) {
        Serial.println("Failed to create TelemetryTask!");
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

void ArduFliteMqttTelemetry::telemetryTask(void* pvParameters) {
    ArduFliteMqttTelemetry* self = static_cast<ArduFliteMqttTelemetry*>(pvParameters);

    while(true) {
        unsigned long startMillis = millis();

        // Make sure MQTT is connected
        self->connectToMqtt();

        // Copy data under mutex
        TelemetryData localCopy;
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

void ArduFliteMqttTelemetry::connectToMqtt() {
    // Reconnect logic if needed
    if (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT... ");
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

void ArduFliteMqttTelemetry::reset() {
    Serial.println("ArduFliteMqttTelemetry::reset() called, stopping task and re-entering WiFiManager...");

    mqttClient.disconnect();
    wifiManager.resetSettings();

    // 1) Stop the current task if it’s running
    if (taskHandle) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    begin();
}