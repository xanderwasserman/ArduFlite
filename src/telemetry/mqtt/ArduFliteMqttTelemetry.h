/**
 * ArduFliteMqttTelemetry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_MQTT_TELEMETRY_H
#define ARDUFLITE_MQTT_TELEMETRY_H

#include "include/PinConfiguration.h"
#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/TelemetryData.h"
#include "src/utils/CommandSystem.h"
#include "src/controller/ArduFliteController.h"
#include "src/telemetry/ConfigData.h"

#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

class ArduFliteMqttTelemetry : public ArduFliteTelemetry {
public:
    ArduFliteMqttTelemetry(float frequencyHz);
    virtual ~ArduFliteMqttTelemetry() {}

    void begin() override;
    void publish(const TelemetryData& telemData, const ConfigData& configData)  override;
    void reset() override;

private:
    static ArduFliteMqttTelemetry* instance;
    // This single task will do WiFi setup + telemetry loop
    static void telemetryTask(void* pvParameters);

    // Called within the telemetry task to attempt an MQTT connection
    void connectToMqtt();

    // Use Preferences to load/save custom MQTT settings
    void loadPreferences();
    void savePreferences();
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
    void pushSystemCommand(SystemCommand cmd);
    void handleAttitudeControl(const JsonDocument& doc);
    void handlePidConfig(ControlLoopType loop, const JsonDocument& doc);
    void handleRateAlpha(float alpha);
    void publishPidConfig(const char* topic, const PIDConfig& pc);

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
    SemaphoreHandle_t   telemetryMutex = nullptr;
    TelemetryData       pendingData;
    ConfigData          pendingConfigData;

    // FreeRTOS Task handle so we can kill/restart it on reset()
    TaskHandle_t      taskHandle     = nullptr;
};

#endif //ARDUFLITE_MQTT_TELEMETRY_H