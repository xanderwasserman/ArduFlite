#pragma once

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
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
    // This single task will do WiFi setup + telemetry loop
    static void telemetryTask(void* pvParameters);

    // Called within the telemetry task to attempt an MQTT connection
    void connectToMqtt();

    // Non-volatile preferences "namespace"
    static constexpr const char* PREF_NAMESPACE = "mqtt";

    String mqttServer   = "192.168.0.10";
    int    mqttPort     = 1883;
    String mqttUser     = "";
    String mqttPass     = "";

    WiFiManagerParameter custom_mqtt_server;
    WiFiManagerParameter custom_mqtt_port;
    WiFiManagerParameter custom_mqtt_user;
    WiFiManagerParameter custom_mqtt_pass;

    // For Wi-Fi & MQTT
    WiFiClient   wifiClient;
    PubSubClient mqttClient;
    float        intervalMs;

    // Data sync
    SemaphoreHandle_t telemetryMutex = nullptr;
    TelemetryData     pendingData;

    // FreeRTOS Task handle so we can kill/restart it on reset()
    TaskHandle_t      taskHandle     = nullptr;

    // Use Preferences to load/save custom MQTT settings
    void loadPreferences();
    void savePreferences();
};
