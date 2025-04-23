/**
 * ArduFliteMqttTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "ArduFliteMqttTelemetry.h"
#include <Arduino.h>

ArduFliteMqttTelemetry* ArduFliteMqttTelemetry::instance = nullptr;

ArduFliteMqttTelemetry::ArduFliteMqttTelemetry(float frequencyHz, CommandSystem* cmdSys)
  : custom_mqtt_server("server", "MQTT Server", mqttServer.c_str(), 40)
  , custom_mqtt_port("port", "MQTT Port", String(mqttPort).c_str(), 6)
  , custom_mqtt_user("user", "MQTT Username", mqttUser.c_str(), 32)
  , custom_mqtt_pass("pass", "MQTT Password", mqttPass.c_str(), 32)
  , mqttClient(wifiClient)
  , _cmdSys(cmdSys)
{
    intervalMs = (1.0f / frequencyHz) * 1000.0f;

    // 1) Load previously saved MQTT settings from Preferences
    loadPreferences();

    // 2) Update WiFiManagerParameter defaults
    // (The above constructor lines set initial defaults,
    //  but if loadPreferences() changed them, let's re-sync.)
    custom_mqtt_server.setValue(mqttServer.c_str(), 40);
    custom_mqtt_port.setValue(String(mqttPort).c_str(), 6);
    custom_mqtt_user.setValue(mqttUser.c_str(), 32);
    custom_mqtt_pass.setValue(mqttPass.c_str(), 32);

    instance = this;
}

void ArduFliteMqttTelemetry::begin() 
{
     // If the task is already running, do nothing
     if (taskHandle) 
     {
        return;
    }

    // Set the MQTT callback for incoming messages.
    mqttClient.setCallback(mqttCallback);

    // Create the FreeRTOS task that will handle WiFi + MQTT
    BaseType_t result = xTaskCreate(
        telemetryTask,
        "MqttTelemetryTask",
        8192,           // stack size
        this,           // task parameter
        1,              // priority
        &taskHandle    // store the handle
    );

    if (result != pdPASS) 
    {
        Serial.println("Failed to create MqttTelemetryTask!");
    }
}

void ArduFliteMqttTelemetry::publish(const TelemetryData& data) 
{
    // Store new data to a local buffer (pendingData),
    // so the telemetryTask can publish it in the background
    if (telemetryMutex && xSemaphoreTake(telemetryMutex, 10) == pdTRUE) 
    {
        pendingData = data; 
        xSemaphoreGive(telemetryMutex);
    }
}

void ArduFliteMqttTelemetry::connectToMqtt() 
{
    if (!mqttClient.connected()) 
    {
        Serial.printf("Connecting to MQTT with credentials:\nBroker: %s\nPort: %d\nUsername: %s\nPassword: %s\n", mqttServer.c_str(), mqttPort, mqttUser.c_str(), mqttPass.c_str());
        if (mqttUser.length() > 0) 
        {
            if (mqttClient.connect("ArduFlite", mqttUser.c_str(), mqttPass.c_str())) 
            {
                Serial.println("connected with auth");
            } 
            else 
            {
                Serial.print("failed, rc=");
                Serial.println(mqttClient.state());
            }
        } 
        else 
        {
            // No user/pass
            if (mqttClient.connect("ArduFlite")) 
            {
                Serial.println("connected");
            } 
            else 
            {
                Serial.print("failed, rc=");
                Serial.println(mqttClient.state());
            }
        }

        // Subscribe to command topics after connection.
        mqttClient.subscribe("arduflite/command/reset");
        mqttClient.subscribe("arduflite/command/calibrate");
        mqttClient.subscribe("arduflite/command/mode");

    }
}

void ArduFliteMqttTelemetry::telemetryTask(void* pvParameters) 
{
    auto* self = static_cast<ArduFliteMqttTelemetry*>(pvParameters);

    // ----------------------
    // 1) WiFiManager config
    // ----------------------
    // Reload saved preferences so we have the latest MQTT config
    self->loadPreferences();
    // Update our custom parameters with the (potentially new) values
    self->custom_mqtt_server.setValue(self->mqttServer.c_str(), 40);
    self->custom_mqtt_port.setValue(String(self->mqttPort).c_str(), 6);
    self->custom_mqtt_user.setValue(self->mqttUser.c_str(), 32);
    self->custom_mqtt_pass.setValue(self->mqttPass.c_str(), 32);

    WiFiManager localManager;

    // Add the member parameters to wifiManager
    localManager.addParameter(&self->custom_mqtt_server);
    localManager.addParameter(&self->custom_mqtt_port);
    localManager.addParameter(&self->custom_mqtt_user);
    localManager.addParameter(&self->custom_mqtt_pass);

    // Optionally set timeouts:
    // self->wifiManager.setConfigPortalTimeout(30); // e.g. 30s

    // Attempt to connect or open the config portal
    if (!localManager.autoConnect("ArduFliteAP")) 
    {
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

    // Save them so they persist across reboots
    self->savePreferences();

    // Set up the MQTT client
    self->mqttClient.setServer(self->mqttServer.c_str(), self->mqttPort);

    // Create a mutex for data
    self->telemetryMutex = xSemaphoreCreateMutex();

    Serial.println("Telemetry WiFi + MQTT setup complete. Entering publish loop...");

    // ----------------------
    // 2) Main publish loop
    // ----------------------
    while(true) 
    {
        unsigned long startMillis = millis();

        // Make sure MQTT is connected
        self->connectToMqtt();

        TelemetryData localCopy;
        // Copy data under mutex
        if (xSemaphoreTake(self->telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) 
        {
            localCopy = self->pendingData;
            xSemaphoreGive(self->telemetryMutex);
        }

        // Publish it
        if (self->mqttClient.connected()) 
        {
            // You could optionally check return codes if needed
            self->mqttClient.publish("arduflite/imu/flight/state", String(localCopy.flight_state, 3).c_str());
            self->mqttClient.publish("arduflite/imu/flight/mode", String(localCopy.flight_mode, 3).c_str());
            self->mqttClient.publish("arduflite/imu/barometer/altitude", String(localCopy.altitude, 3).c_str());

            self->mqttClient.publish("arduflite/imu/accel/x", String(localCopy.accel.x, 3).c_str());
            self->mqttClient.publish("arduflite/imu/accel/y", String(localCopy.accel.y, 3).c_str());
            self->mqttClient.publish("arduflite/imu/accel/z", String(localCopy.accel.z, 3).c_str());

            self->mqttClient.publish("arduflite/imu/gyro/x", String(localCopy.gyro.x, 3).c_str());
            self->mqttClient.publish("arduflite/imu/gyro/y", String(localCopy.gyro.y, 3).c_str());
            self->mqttClient.publish("arduflite/imu/gyro/z", String(localCopy.gyro.z, 3).c_str());

            self->mqttClient.publish("arduflite/imu/quaternion/w", String(localCopy.quat.w, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/x", String(localCopy.quat.x, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/y", String(localCopy.quat.y, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/z", String(localCopy.quat.z, 4).c_str());

            self->mqttClient.publish("arduflite/imu/orientation/pitch", String(localCopy.orientation.pitch, 2).c_str());
            self->mqttClient.publish("arduflite/imu/orientation/roll", String(localCopy.orientation.roll, 2).c_str());
            self->mqttClient.publish("arduflite/imu/orientation/yaw", String(localCopy.orientation.yaw, 2).c_str());

            self->mqttClient.publish("arduflite/controller-setpoint/rate/roll", String(localCopy.rateSetpoint.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/rate/pitch", String(localCopy.rateSetpoint.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/rate/yaw", String(localCopy.rateSetpoint.yaw, 3).c_str());

            self->mqttClient.publish("arduflite/controller-setpoint/attitude/roll", String(localCopy.attitudeSetpoint.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/attitude/pitch", String(localCopy.attitudeSetpoint.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/attitude/yaw", String(localCopy.attitudeSetpoint.yaw, 3).c_str());
            
            self->mqttClient.publish("arduflite/controller/rate/roll", String(localCopy.rateCmd.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller/rate/pitch", String(localCopy.rateCmd.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller/rate/yaw", String(localCopy.rateCmd.yaw, 3).c_str());

            self->mqttClient.publish("arduflite/controller/attitude/roll", String(localCopy.attitudeCmd.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller/attitude/pitch", String(localCopy.attitudeCmd.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller/attitude/yaw", String(localCopy.attitudeCmd.yaw, 3).c_str());
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
    WiFiManager wifiManager;
    wifiManager.resetSettings(); 

    // Stop the current telemetry task if running
    if (taskHandle) 
    {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }

    // Start again
    begin();
}

// ------------------------
// Load & Save to Preferences
// ------------------------
void ArduFliteMqttTelemetry::loadPreferences()
{
    Preferences prefs;
    if (!prefs.begin(PREF_NAMESPACE, true))// read-only mode
    { 
        Serial.println("[MQTT] Preferences not found, using defaults");
        return;
    }

    mqttServer = prefs.getString("server", mqttServer);
    mqttPort   = prefs.getShort("port",   mqttPort);
    mqttUser   = prefs.getString("user",  mqttUser);
    mqttPass   = prefs.getString("pass",  mqttPass);

    Serial.printf("[MQTT] Loaded from NVS:\nServer=%s Port=%d User=%s Pass=%s\n",
        mqttServer.c_str(), mqttPort, mqttUser.c_str(), mqttPass.c_str());

    prefs.end();
}

void ArduFliteMqttTelemetry::savePreferences()
{
    Preferences prefs;
    if (!prefs.begin(PREF_NAMESPACE, false)) // read/write
    { 
        Serial.println("[MQTT] Failed to open prefs for writing");
        return;
    }

    prefs.putString("server", mqttServer);
    prefs.putShort("port",   mqttPort);
    prefs.putString("user",  mqttUser);
    prefs.putString("pass",  mqttPass);
    prefs.end();

    Serial.println("[MQTT] Saved new config to NVS");
}

// Static MQTT callback for subscribed messages.
void ArduFliteMqttTelemetry::mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    if (!instance) return;

    String message;
    for (unsigned int i = 0; i < length; i++) 
    {
        message += (char)payload[i];
    }
    message.trim();

    // Convert the topic to a String and to lower-case for case-insensitive comparison.
    String topicStr(topic);
    topicStr.toLowerCase();
    
    Serial.printf("Received on topic: %s: %s\n",topicStr, message);

    if (topicStr =="arduflite/command/reset")
    {
        // For reset, check if payload equals "1".
        if (message.equals("1")) 
        {
            instance->pushSystemCommand(CMD_RESET);
        }
    }
    else if (topicStr == "arduflite/command/calibrate")
    {
        // For calibrate, check if payload equals "1".
        if (message.equals("1")) 
        {
            Serial.println("Calibrate command received.");
            instance->pushSystemCommand(CMD_CALIBRATE);
        }
    }
    else if (topicStr == "arduflite/command/mode")
    {
        // For mode command, convert payload to integer and switch.
        int modeValue = message.toInt();
        switch (modeValue) 
        {
            case 1:
                Serial.println("Mode command received: Setting mode to ASSIST_MODE.");
                instance->pushSystemCommand(CMD_MODE_ASSIST);
                break;
            case 2:
                Serial.println("Mode command received: Setting mode to STABILIZED_MODE.");
                instance->pushSystemCommand(CMD_MODE_STABILIZED);
                break;
            default:
                Serial.println("Unknown mode command payload.");
                break;
        }
    }
}

void ArduFliteMqttTelemetry::pushSystemCommand(SystemCommandType type) 
{
    if (!_cmdSys) return;
    SystemCommand cmd{ type };
    _cmdSys->pushCommand(cmd);
}