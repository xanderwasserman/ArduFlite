/**
 * ArduFliteMqttTelemetry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/utils/Logging.h"

#include <Arduino.h>
#include <ArduinoJson.h>

ArduFliteMqttTelemetry* ArduFliteMqttTelemetry::instance = nullptr;

ArduFliteMqttTelemetry::ArduFliteMqttTelemetry(float frequencyHz)
  : custom_mqtt_server("server", "MQTT Server", mqttServer.c_str(), 40)
  , custom_mqtt_port("port", "MQTT Port", String(mqttPort).c_str(), 6)
  , custom_mqtt_user("user", "MQTT Username", mqttUser.c_str(), 32)
  , custom_mqtt_pass("pass", "MQTT Password", mqttPass.c_str(), 32)
  , mqttClient(wifiClient)
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
        LOG_ERR("Failed to create MqttTelemetryTask!");
    }
}

void ArduFliteMqttTelemetry::publish(const TelemetryData& telemData, const ConfigData& configData) 
{
    // Store new data to a local buffer (pendingData),
    // so the telemetryTask can publish it in the background
    if (telemetryMutex && xSemaphoreTake(telemetryMutex, 10) == pdTRUE) 
    {
        pendingData         = telemData; 
        pendingConfigData   = configData; 
        xSemaphoreGive(telemetryMutex);
    }
}

void ArduFliteMqttTelemetry::connectToMqtt() 
{
    if (!mqttClient.connected()) 
    {
        LOG_INF("Connecting to MQTT with credentials:\nBroker: %s\nPort: %d\nUsername: %s\nPassword: %s", mqttServer.c_str(), mqttPort, mqttUser.c_str(), mqttPass.c_str());
        if (mqttUser.length() > 0) 
        {
            if (mqttClient.connect("ArduFlite", mqttUser.c_str(), mqttPass.c_str())) 
            {
                LOG_INF("connected with auth");
            } 
            else 
            {
                LOG_ERR("auth failed, rc=%d", mqttClient.state());
            }
        } 
        else 
        {
            // No user/pass
            if (mqttClient.connect("ArduFlite")) 
            {
                LOG_INF("connected");
            } 
            else 
            {
                LOG_ERR("failed, rc=%d", mqttClient.state());
            }
        }

        // Subscribe to command topics after connection.
        mqttClient.subscribe("arduflite/command/reset");
        mqttClient.subscribe("arduflite/command/calibrate");
        mqttClient.subscribe("arduflite/command/mode");
        mqttClient.subscribe("arduflite/controller-setpoint/attitude/set");
        mqttClient.subscribe("arduflite/config/attitude/roll/pid/set");
        mqttClient.subscribe("arduflite/config/attitude/pitch/pid/set");
        mqttClient.subscribe("arduflite/config/attitude/yaw/pid/set");
        mqttClient.subscribe("arduflite/config/rate/roll/pid/set");
        mqttClient.subscribe("arduflite/config/rate/pitch/pid/set");
        mqttClient.subscribe("arduflite/config/rate/yaw/pid/set");
        mqttClient.subscribe("arduflite/config/rate/alpha/set");
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

// #if BOARD_TYPE == BOARD_TYPE_WEMOS
//     WiFi.setTxPower(WIFI_POWER_8_5dBm);
// #endif

    // Attempt to connect or open the config portal
    if (!localManager.autoConnect("ArduFliteAP")) 
    {
        LOG_ERR("WiFi connection failed. Telemetry task will exit.");
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

    LOG_INF("Telemetry WiFi + MQTT setup complete. Entering publish loop...");

    // ----------------------
    // 2) Main publish loop
    // ----------------------
    while(true) 
    {
        unsigned long startMillis = millis();

        // Let PubSubClient handle incoming messages (if you care about subscriptions)
        self->mqttClient.loop();

        // Make sure MQTT is connected
        self->connectToMqtt();

        TelemetryData   localTelemCopy;
        ConfigData      localConfigCopy;
        // Copy data under mutex
        if (xSemaphoreTake(self->telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE) 
        {
            localTelemCopy = self->pendingData;
            localConfigCopy = self->pendingConfigData;
            xSemaphoreGive(self->telemetryMutex);
        }

        // Publish it
        if (self->mqttClient.connected()) 
        {
            // You could optionally check return codes if needed
            self->mqttClient.publish("arduflite/imu/flight/state", String(localTelemCopy.flight_state, 3).c_str());
            self->mqttClient.publish("arduflite/imu/flight/mode", String(localTelemCopy.flight_mode, 3).c_str());
            self->mqttClient.publish("arduflite/imu/barometer/altitude", String(localTelemCopy.altitude, 3).c_str());

            self->mqttClient.publish("arduflite/imu/accel/x", String(localTelemCopy.accel.x, 3).c_str());
            self->mqttClient.publish("arduflite/imu/accel/y", String(localTelemCopy.accel.y, 3).c_str());
            self->mqttClient.publish("arduflite/imu/accel/z", String(localTelemCopy.accel.z, 3).c_str());

            self->mqttClient.publish("arduflite/imu/gyro/x", String(localTelemCopy.gyro.x, 3).c_str());
            self->mqttClient.publish("arduflite/imu/gyro/y", String(localTelemCopy.gyro.y, 3).c_str());
            self->mqttClient.publish("arduflite/imu/gyro/z", String(localTelemCopy.gyro.z, 3).c_str());

            self->mqttClient.publish("arduflite/imu/quaternion/w", String(localTelemCopy.quat.w, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/x", String(localTelemCopy.quat.x, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/y", String(localTelemCopy.quat.y, 4).c_str());
            self->mqttClient.publish("arduflite/imu/quaternion/z", String(localTelemCopy.quat.z, 4).c_str());

            self->mqttClient.publish("arduflite/imu/orientation/pitch", String(localTelemCopy.orientation.pitch, 2).c_str());
            self->mqttClient.publish("arduflite/imu/orientation/roll", String(localTelemCopy.orientation.roll, 2).c_str());
            self->mqttClient.publish("arduflite/imu/orientation/yaw", String(localTelemCopy.orientation.yaw, 2).c_str());

            self->mqttClient.publish("arduflite/controller-setpoint/rate/roll", String(localTelemCopy.rateSetpoint.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/rate/pitch", String(localTelemCopy.rateSetpoint.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/rate/yaw", String(localTelemCopy.rateSetpoint.yaw, 3).c_str());

            self->mqttClient.publish("arduflite/controller-setpoint/attitude/roll", String(localTelemCopy.attitudeSetpoint.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/attitude/pitch", String(localTelemCopy.attitudeSetpoint.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller-setpoint/attitude/yaw", String(localTelemCopy.attitudeSetpoint.yaw, 3).c_str());
            
            self->mqttClient.publish("arduflite/controller/rate/roll", String(localTelemCopy.rateCmd.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller/rate/pitch", String(localTelemCopy.rateCmd.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller/rate/yaw", String(localTelemCopy.rateCmd.yaw, 3).c_str());

            self->mqttClient.publish("arduflite/controller/attitude/roll", String(localTelemCopy.attitudeCmd.roll, 3).c_str());
            self->mqttClient.publish("arduflite/controller/attitude/pitch", String(localTelemCopy.attitudeCmd.pitch, 3).c_str());
            self->mqttClient.publish("arduflite/controller/attitude/yaw", String(localTelemCopy.attitudeCmd.yaw, 3).c_str());

            // 1) Attitude → Roll
            self->publishPidConfig("arduflite/config/attitude/roll/pid", localConfigCopy.attitude_pidRoll);

            // 2) Attitude → Pitch
            self->publishPidConfig("arduflite/config/attitude/pitch/pid", localConfigCopy.attitude_pidPitch);

            // 3) Attitude → Yaw
            self->publishPidConfig("arduflite/config/attitude/yaw/pid", localConfigCopy.attitude_pidYaw);

            // 4) Rate → Roll
            self->publishPidConfig("arduflite/config/rate/roll/pid", localConfigCopy.rate_pidRoll);

            // 5) Rate → Pitch
            self->publishPidConfig("arduflite/config/rate/pitch/pid", localConfigCopy.rate_pidPitch);

            // 6) Rate → Yaw
            self->publishPidConfig("arduflite/config/rate/yaw/pid", localConfigCopy.rate_pidYaw);

            // 7) Finally, rate-filter alpha can still be separate, e.g.:
            {
                char bufA[64];
                int mA = snprintf(bufA, sizeof(bufA), "{\"alpha\":%.3f}",
                                localConfigCopy.filter_alpha);
                if (mA > 0 && mA < (int)sizeof(bufA)) {
                    self->mqttClient.publish("arduflite/config/rate/alpha", bufA);
                } else {
                    LOG_ERR("publishRateAlpha: snprintf error or overflow (%d)", mA);
                }
            }
        }

        // Delay enough to match frequency
        unsigned long elapsed = millis() - startMillis;
        int delayMs = (int)max(1.0f, self->intervalMs - elapsed);
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

void ArduFliteMqttTelemetry::reset()
{
    LOG_INF("ArduFliteMqttTelemetry::reset() called. Stopping task & clearing Wi-Fi credentials...");

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
        LOG_WARN("[MQTT] Preferences not found, using defaults");
        return;
    }

    mqttServer = prefs.getString("server", mqttServer);
    mqttPort   = prefs.getShort("port",   mqttPort);
    mqttUser   = prefs.getString("user",  mqttUser);
    mqttPass   = prefs.getString("pass",  mqttPass);

    LOG_INF("[MQTT] Loaded from NVS:\nServer=%s Port=%d User=%s Pass=%s",
        mqttServer.c_str(), mqttPort, mqttUser.c_str(), mqttPass.c_str());

    prefs.end();
}

void ArduFliteMqttTelemetry::savePreferences()
{
    Preferences prefs;
    if (!prefs.begin(PREF_NAMESPACE, false)) // read/write
    { 
        LOG_ERR("[MQTT] Failed to open prefs for writing");
        return;
    }

    prefs.putString("server", mqttServer);
    prefs.putShort("port",   mqttPort);
    prefs.putString("user",  mqttUser);
    prefs.putString("pass",  mqttPass);
    prefs.end();

    LOG_INF("[MQTT] Saved new config to NVS");
}

// Static MQTT callback for subscribed messages.
void ArduFliteMqttTelemetry::mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    if (!instance) return;

    // 1) Convert topic to lowercase String for comparison
    String topicStr(topic);
    topicStr.toLowerCase();

    // 2) Pull payload into a String (for simple "1"/"2" commands)
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    message.trim();

    LOG_DBG("Received on topic: %s: %s\n", topicStr.c_str(), message.c_str());

    //
    // --- HANDLE SIMPLE COMMANDS (non‐JSON) ---
    //
    if (topicStr == "arduflite/command/reset") 
    {
        // For reset, check if payload equals "1"
        if (message.equals("1")) {
            SystemCommand cmd;
            cmd.type = CMD_RESET;
            instance->pushSystemCommand(cmd);
        }
        return;  // done
    }
    else if (topicStr == "arduflite/command/calibrate") 
    {
        // For calibrate, check if payload equals "1"
        if (message.equals("1")) {
            SystemCommand cmd;
            cmd.type = CMD_CALIBRATE;
            instance->pushSystemCommand(cmd);
        }
        return;
    }
    else if (topicStr == "arduflite/command/mode") 
    {
        // Mode command: payload is an integer (1 or 2)
        int modeValue = message.toInt();
        SystemCommand cmd;
        cmd.type = CMD_SET_MODE;

        switch (modeValue) 
        {
            case 1:
                LOG_INF("Mode command received: Setting mode to ATTITUDE_MODE.");
                cmd.mode = ATTITUDE_MODE;
                instance->pushSystemCommand(cmd);
                break;
            case 2:
                LOG_INF("Mode command received: Setting mode to RATE_MODE.");
                cmd.mode = RATE_MODE;
                instance->pushSystemCommand(cmd);
                break;
            default:
                LOG_WARN("Unknown mode command payload: %s", message.c_str());
                break;
        }
        return;
    }

    //
    // --- ALL REMAINING TOPICS EXPECT A JSON PAYLOAD ---
    //
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) 
    {
        LOG_ERR("JSON parse error on %s: %s", topicStr.c_str(), err.c_str());
        return;
    }

    //
    // --- ATTITUDE CONTROL (EulerAngles JSON) ---
    //
    if (topicStr == "arduflite/controller-setpoint/attitude/set") 
    {
        LOG_DBG("Attitude control JSON received: %s", message.c_str());
        instance->handleAttitudeControl(doc);
        return;
    }

    //
    // --- PID CONFIG UPDATES (each takes a full PIDConfig JSON) ---
    //
    // Note: Your JSON must include keys "kp", "ki", "kd", "outLimit", "maxIntegral", "derivativeAlpha".
    //
    if (topicStr == "arduflite/config/attitude/roll/pid/set") 
    {
        LOG_INF("Config JSON for ATTITUDE_ROLL_LOOP: %s", message.c_str());

        instance->handlePidConfig(ATTITUDE_ROLL_LOOP, doc);
        return;
    }
    else if (topicStr == "arduflite/config/attitude/pitch/pid/set") 
    {
        LOG_INF("Config JSON for ATTITUDE_PITCH_LOOP: %s", message.c_str());

        instance->handlePidConfig(ATTITUDE_PITCH_LOOP, doc);
        return;
    }
    else if (topicStr == "arduflite/config/attitude/yaw/pid/set") 
    {
        LOG_INF("Config JSON for ATTITUDE_YAW_LOOP: %s", message.c_str());

        instance->handlePidConfig(ATTITUDE_YAW_LOOP, doc);
        return;
    }
    else if (topicStr == "arduflite/config/rate/roll/pid/set") 
    {
        LOG_INF("Config JSON for RATE_ROLL_LOOP: %s", message.c_str());

        instance->handlePidConfig(RATE_ROLL_LOOP, doc);
        return;
    }
    else if (topicStr == "arduflite/config/rate/pitch/pid/set") 
    {
        LOG_INF("Config JSON for RATE_PITCH_LOOP: %s", message.c_str());

        instance->handlePidConfig(RATE_PITCH_LOOP, doc);
        return;
    }
    else if (topicStr == "arduflite/config/rate/yaw/pid/set") 
    {
        LOG_INF("Config JSON for RATE_YAW_LOOP: %s", message.c_str());

        instance->handlePidConfig(RATE_YAW_LOOP, doc);
        return;
    }

    //
    // --- RATE FILTER ALPHA (single‐float JSON key: "alpha") ---
    //
    if (topicStr == "arduflite/config/rate/alpha/set") 
    {
        LOG_INF("Config received for rate controller alpha: %s", message.c_str());
        float a = doc["alpha"] | 0.0f;
        instance->handleRateAlpha(a);
        return;
    }
}

void ArduFliteMqttTelemetry::pushSystemCommand(SystemCommand cmd) 
{
    CommandSystem::instance().pushCommand(cmd);
}

void ArduFliteMqttTelemetry::handleAttitudeControl(const JsonDocument& doc) 
{
    EulerAngles ea 
    {
        doc["roll"]  | 0.0f,
        doc["pitch"] | 0.0f,
        doc["yaw"]   | 0.0f
    };
    
    SystemCommand cmd;
    cmd.type            = CMD_SET_CONFIG_ATTITUDE;
    cmd.attitudeConfig  = ea;

    pushSystemCommand(cmd);
}

void ArduFliteMqttTelemetry::handlePidConfig(ControlLoopType loop, const JsonDocument& doc) 
{
    // Build a PIDConfig from JSON. Keys must be:
    //   "kp", "ki", "kd", "outLimit", "maxIntegral", "derivativeAlpha"
    PIDConfig pc;
    pc.kp              = doc["kp"]             | 0.0f;
    pc.ki              = doc["ki"]             | 0.0f;
    pc.kd              = doc["kd"]             | 0.0f;
    pc.outLimit        = doc["outLimit"]       | 0.0f;
    pc.maxIntegral     = doc["maxIntegral"]    | 0.0f;
    pc.derivativeAlpha = doc["derivativeAlpha"]| 0.0f;

    SystemCommand cmd;
    cmd.type      = CMD_SET_CONFIG_PID;
    cmd.pidLoop   = loop;
    cmd.pidConfig = pc;

    pushSystemCommand(cmd);
}


void ArduFliteMqttTelemetry::handleRateAlpha(float alpha) 
{
    SystemCommand cmd;
    cmd.type    = CMD_SET_CONFIG_RATE_ALPHA;
    cmd.value   = alpha;

    pushSystemCommand(cmd);
}

/**
 * @brief Format a PIDConfig as JSON and publish it.
 *
 * JSON schema: 
 *   { 
 *     "kp":            <float>,
 *     "ki":            <float>,
 *     "kd":            <float>,
 *     "outLimit":      <float>,
 *     "maxIntegral":   <float>,
 *     "derivativeAlpha":<float>
 *   }
 *
 * We use snprintf() to avoid pulling in a full JSON library.
 */
void ArduFliteMqttTelemetry::publishPidConfig(const char* topic, const PIDConfig& pc) 
{
    // Buffer must be large enough for six floats plus keys/punctuation.
    // 128 bytes is usually safe for this schema.
    char buf[128];

    // Build JSON string:
    // {"kp":%.3f,"ki":%.3f,"kd":%.3f,"outLimit":%.3f,"maxIntegral":%.3f,"derivativeAlpha":%.3f}
    int len = snprintf(
        buf, sizeof(buf),
        "{\"kp\":%.3f,\"ki\":%.3f,\"kd\":%.3f"
        ",\"outLimit\":%.3f,\"maxIntegral\":%.3f"
        ",\"derivativeAlpha\":%.3f}",
        pc.kp,
        pc.ki,
        pc.kd,
        pc.outLimit,
        pc.maxIntegral,
        pc.derivativeAlpha
    );

    // snprintf returns the number of bytes (not including the '\0').
    // If it returns <0 or >= sizeof(buf), the output was truncated.
    if (len > 0 && len < (int)sizeof(buf)) {
        this->mqttClient.publish(topic, buf);
    } else {
        LOG_ERR("publishPidConfig: snprintf error or buffer overflow (%d)", len);
    }
}