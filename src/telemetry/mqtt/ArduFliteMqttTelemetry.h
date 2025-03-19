// ArduFliteMqttTelemetry.h

#pragma once

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"

class ArduFliteMqttTelemetry : public ArduFliteTelemetry {
public:
    ArduFliteMqttTelemetry(float frequencyHz = 10.0f);
    virtual ~ArduFliteMqttTelemetry() {}

    void begin() override;
    void publish(const TelemetryData& data) override;
    void reset() override;

private:
    static void telemetryTask(void* pvParameters);
    void connectToMqtt();

    // For Wi-Fi & MQTT
    WiFiManager wifiManager;
    WiFiClient wifiClient;
    PubSubClient mqttClient;
    float intervalMs;
    String mqttServer   = "192.168.0.10";
    int    mqttPort     = 1883;
    String mqttUser     = "";
    String mqttPass     = "";

    // Data sync
    SemaphoreHandle_t telemetryMutex    = nullptr;
    TelemetryData     pendingData;

    // We store the task handle so we can stop it on reset
    TaskHandle_t      taskHandle        = nullptr;
};
