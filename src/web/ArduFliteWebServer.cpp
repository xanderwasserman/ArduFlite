/**
 * ArduFliteWebServer.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "include/WebConfiguration.h"

#if ENABLE_WEB_SERVER

#include "src/web/ArduFliteWebServer.h"
#include "src/web/WebUI.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"
#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "include/ConfigKeys.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

ArduFliteWebServer& ArduFliteWebServer::instance()
{
    static ArduFliteWebServer _instance;
    return _instance;
}

ArduFliteWebServer::ArduFliteWebServer()
{
}

bool ArduFliteWebServer::begin(ArduFliteController* controller,
                                ArduFliteIMU* imu,
                                ArduFliteFlashTelemetry* flashTelemetry)
{
    if (_running)
    {
        LOG_WARN("WebServer already running");
        return true;
    }

    _controller = controller;
    _imu = imu;
    _flashTelemetry = flashTelemetry;

    // Create server on heap
    _server = new WebServer(HTTP_PORT);
    if (!_server)
    {
        LOG_ERR("Failed to allocate WebServer");
        return false;
    }

    setupRoutes();

    // Start server task
    BaseType_t res = xTaskCreate(
        serverTask,
        "Web",
        TASK_STACK_SIZE,
        this,
        TASK_PRIORITY,
        &_taskHandle
    );

    if (res != pdPASS)
    {
        LOG_ERR("Failed to create WebServer task");
        delete _server;
        _server = nullptr;
        return false;
    }

    _running = true;
    LOG_INF("WebServer started on port %d", HTTP_PORT);
    return true;
}

void ArduFliteWebServer::stop()
{
    if (!_running) return;

    LOG_INF("Stopping WebServer");
    _running = false;

    if (_taskHandle)
    {
        vTaskDelete(_taskHandle);
        _taskHandle = nullptr;
    }

    if (_server)
    {
        _server->stop();
        delete _server;
        _server = nullptr;
    }
}

void ArduFliteWebServer::serverTask(void* pv)
{
    auto* self = static_cast<ArduFliteWebServer*>(pv);
    self->run();
}

void ArduFliteWebServer::run()
{
    LOG_INF("WebServer task started");
    _server->begin();

    while (_running)
    {
        _server->handleClient();
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms polling for responsive connections
    }

    vTaskDelete(nullptr);
}

void ArduFliteWebServer::setupRoutes()
{
    // Web UI (root and static assets)
    _server->on("/", HTTP_GET, std::bind(&ArduFliteWebServer::handleWebUI, this));
    _server->on("/styles.css", HTTP_GET, std::bind(&ArduFliteWebServer::handleCSS, this));
    _server->on("/app.js", HTTP_GET, std::bind(&ArduFliteWebServer::handleJS, this));

    // Config API
    _server->on("/api/config", HTTP_GET, std::bind(&ArduFliteWebServer::handleConfigList, this));
    _server->on("/api/config/export", HTTP_GET, std::bind(&ArduFliteWebServer::handleConfigExport, this));
    _server->on("/api/config/import", HTTP_POST, std::bind(&ArduFliteWebServer::handleConfigImport, this));
    _server->on("/api/config/reset", HTTP_POST, std::bind(&ArduFliteWebServer::handleConfigReset, this));
    _server->on("/api/config/reboot", HTTP_POST, std::bind(&ArduFliteWebServer::handleConfigReboot, this));

    // System API
    _server->on("/api/system/status", HTTP_GET, std::bind(&ArduFliteWebServer::handleSystemStatus, this));
    _server->on("/api/telemetry", HTTP_GET, std::bind(&ArduFliteWebServer::handleTelemetry, this));
    _server->on("/api/system/calibrate", HTTP_POST, std::bind(&ArduFliteWebServer::handleCalibrate, this));

    // Flash logs API
    _server->on("/api/flash", HTTP_GET, std::bind(&ArduFliteWebServer::handleFlashList, this));

    // Wildcard routes for parameterized paths
    _server->onNotFound(std::bind(&ArduFliteWebServer::handleNotFound, this));
}

// ═══════════════════════════════════════════════════════════════════════════
// Web UI Handlers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleWebUI()
{
    // Content is gzip-compressed in PROGMEM
    _server->sendHeader("Content-Encoding", "gzip");
    _server->setContentLength(WEB_UI_HTML_GZ_LEN);
    _server->send(200, "text/html", "");
    _server->sendContent_P(reinterpret_cast<const char*>(WEB_UI_HTML_GZ), WEB_UI_HTML_GZ_LEN);
}

void ArduFliteWebServer::handleCSS()
{
    _server->sendHeader("Cache-Control", "max-age=86400");
    _server->sendHeader("Content-Encoding", "gzip");
    _server->setContentLength(WEB_UI_CSS_GZ_LEN);
    _server->send(200, "text/css", "");
    _server->sendContent_P(reinterpret_cast<const char*>(WEB_UI_CSS_GZ), WEB_UI_CSS_GZ_LEN);
}

void ArduFliteWebServer::handleJS()
{
    _server->sendHeader("Cache-Control", "max-age=86400");
    _server->sendHeader("Content-Encoding", "gzip");
    _server->setContentLength(WEB_UI_JS_GZ_LEN);
    _server->send(200, "application/javascript", "");
    _server->sendContent_P(reinterpret_cast<const char*>(WEB_UI_JS_GZ), WEB_UI_JS_GZ_LEN);
}

// ═══════════════════════════════════════════════════════════════════════════
// Config API Handlers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleConfigList()
{
    // Optional pattern filter: /api/config?pattern=rate.*
    String pattern = _server->hasArg("pattern") ? _server->arg("pattern") : "*";

    auto& reg = ConfigRegistry::instance();
    auto params = reg.getAllParams();

    // Collect and sort keys
    std::vector<std::string> keys;
    for (const auto& kv : params)
    {
        keys.push_back(kv.first);
    }
    std::sort(keys.begin(), keys.end());

    // Use chunked transfer encoding for large responses to avoid memory issues
    _server->setContentLength(CONTENT_LENGTH_UNKNOWN);
    _server->sendHeader("Access-Control-Allow-Origin", "*");
    _server->send(200, "application/json", "");
    
    // Stream JSON array directly to avoid large String allocation
    _server->sendContent("[");
    bool first = true;

    for (const auto& key : keys)
    {
        const auto& p = params[key];
        
        // Simple pattern matching
        bool match = (pattern == "*");
        if (!match && pattern.endsWith("*"))
        {
            String prefix = pattern.substring(0, pattern.length() - 1);
            match = String(key.c_str()).startsWith(prefix);
        }
        if (!match)
        {
            match = (String(key.c_str()) == pattern);
        }

        if (match)
        {
            // Build single item JSON (small, fits in memory)
            JsonDocument doc;
            JsonObject obj = doc.to<JsonObject>();
            obj["key"] = p.key;
            obj["desc"] = p.description;
            obj["type"] = (int)p.type;
            obj["reboot"] = p.requiresReboot;
            obj["dirty"] = p.dirty;

            switch (p.type)
            {
                case ConfigType::FLOAT:
                    obj["value"] = p.currentVal.f;
                    obj["default"] = p.defaultVal.f;
                    obj["min"] = p.minVal.f;
                    obj["max"] = p.maxVal.f;
                    break;
                case ConfigType::INT32:
                    obj["value"] = p.currentVal.i;
                    obj["default"] = p.defaultVal.i;
                    obj["min"] = p.minVal.i;
                    obj["max"] = p.maxVal.i;
                    break;
                case ConfigType::UINT8:
                    obj["value"] = p.currentVal.u8;
                    obj["default"] = p.defaultVal.u8;
                    obj["min"] = p.minVal.u8;
                    obj["max"] = p.maxVal.u8;
                    break;
                case ConfigType::BOOL:
                    obj["value"] = p.currentVal.b;
                    obj["default"] = p.defaultVal.b;
                    break;
                case ConfigType::STRING:
                    obj["value"] = p.currentVal.s;
                    obj["default"] = p.defaultVal.s;
                    break;
            }

            // Stream with comma separator
            String item;
            serializeJson(doc, item);
            if (!first) _server->sendContent(",");
            _server->sendContent(item);
            first = false;
        }
    }

    _server->sendContent("]");
    _server->sendContent("");  // End chunked transfer
}

void ArduFliteWebServer::handleConfigGet()
{
    // Extract key from URI: /api/config/rate.roll.kp
    String uri = _server->uri();
    if (!uri.startsWith("/api/config/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String key = uri.substring(12);  // Skip "/api/config/"
    if (key.isEmpty())
    {
        sendError(400, "Key required");
        return;
    }

    auto& reg = ConfigRegistry::instance();
    auto optParam = reg.getParam(key.c_str());

    if (!optParam)
    {
        sendError(404, "Key not found");
        return;
    }

    const auto& p = *optParam;
    JsonDocument doc;
    doc["key"] = p.key;
    doc["desc"] = p.description;
    doc["type"] = (int)p.type;
    doc["reboot"] = p.requiresReboot;
    doc["dirty"] = p.dirty;

    switch (p.type)
    {
        case ConfigType::FLOAT:
            doc["value"] = p.currentVal.f;
            doc["default"] = p.defaultVal.f;
            doc["min"] = p.minVal.f;
            doc["max"] = p.maxVal.f;
            break;
        case ConfigType::INT32:
            doc["value"] = p.currentVal.i;
            doc["default"] = p.defaultVal.i;
            doc["min"] = p.minVal.i;
            doc["max"] = p.maxVal.i;
            break;
        case ConfigType::UINT8:
            doc["value"] = p.currentVal.u8;
            doc["default"] = p.defaultVal.u8;
            doc["min"] = p.minVal.u8;
            doc["max"] = p.maxVal.u8;
            break;
        case ConfigType::BOOL:
            doc["value"] = p.currentVal.b;
            doc["default"] = p.defaultVal.b;
            break;
        case ConfigType::STRING:
            doc["value"] = p.currentVal.s;
            doc["default"] = p.defaultVal.s;
            break;
    }

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleConfigSet()
{
    // Extract key from URI: PUT /api/config/rate.roll.kp
    String uri = _server->uri();
    if (!uri.startsWith("/api/config/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String key = uri.substring(12);
    if (key.isEmpty())
    {
        sendError(400, "Key required");
        return;
    }

    // Parse JSON body
    if (!_server->hasArg("plain"))
    {
        sendError(400, "Body required");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, _server->arg("plain"));
    if (err)
    {
        sendError(400, "Invalid JSON");
        return;
    }

    if (doc["value"].isNull())
    {
        sendError(400, "Value required");
        return;
    }

    auto& reg = ConfigRegistry::instance();
    auto optParam = reg.getParam(key.c_str());

    if (!optParam)
    {
        sendError(404, "Key not found");
        return;
    }

    const auto& p = *optParam;
    bool success = false;

    switch (p.type)
    {
        case ConfigType::FLOAT:
            success = reg.set<float>(key.c_str(), doc["value"].as<float>());
            break;
        case ConfigType::INT32:
            success = reg.set<int32_t>(key.c_str(), doc["value"].as<int32_t>());
            break;
        case ConfigType::UINT8:
            success = reg.set<uint8_t>(key.c_str(), doc["value"].as<uint8_t>());
            break;
        case ConfigType::BOOL:
            success = reg.set<bool>(key.c_str(), doc["value"].as<bool>());
            break;
        case ConfigType::STRING:
            success = reg.set<String>(key.c_str(), doc["value"].as<String>());
            break;
    }

    if (success)
    {
        // Auto-save dirty params
        ConfigPersistence::saveIfDirty();
        sendJson(200, "{\"ok\":true}");
    }
    else
    {
        sendError(400, "Validation failed");
    }
}

void ArduFliteWebServer::handleConfigReset()
{
    LOG_INF("Web: Resetting all config to defaults");
    ConfigRegistry::instance().resetAll();
    ConfigPersistence::saveIfDirty();
    sendJson(200, "{\"ok\":true}");
}

void ArduFliteWebServer::handleConfigExport()
{
    String json = ConfigPersistence::exportJson();
    sendJson(200, json);
}

void ArduFliteWebServer::handleConfigImport()
{
    if (!_server->hasArg("plain"))
    {
        sendError(400, "Body required");
        return;
    }

    String json = _server->arg("plain");
    size_t imported = ConfigPersistence::importJson(json);
    
    JsonDocument doc;
    doc["ok"] = true;
    doc["imported"] = imported;

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleConfigReboot()
{
    LOG_INF("Web: Reboot requested");
    sendJson(200, "{\"ok\":true,\"message\":\"Rebooting...\"}");
    
    // Delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
}

// ═══════════════════════════════════════════════════════════════════════════
// System API Handlers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleSystemStatus()
{
    JsonDocument doc;

    // Basic system info
    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["min_heap"] = ESP.getMinFreeHeap();
    doc["chip_model"] = ESP.getChipModel();
    doc["sdk_version"] = ESP.getSdkVersion();

    // Controller state (if available)
    if (_controller)
    {
        doc["armed"] = _controller->isArmed();
        doc["mode"] = (int)_controller->getMode();
        doc["throttle_cut"] = _controller->isThrottleCut();
    }

    // IMU state (if available)
    if (_imu)
    {
        doc["imu_healthy"] = _imu->isHealthy();
        doc["flight_state"] = static_cast<int>(_imu->getFlightState());
    }

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleSystemLogs()
{
    // Placeholder for log streaming - would require Server-Sent Events
    // For now, return recent log buffer if available
    sendJson(501, "{\"error\":\"Log streaming not implemented\"}");
}

void ArduFliteWebServer::handleTelemetry()
{
    JsonDocument doc;

    // IMU orientation and state
    if (_imu)
    {
        auto euler = _imu->getOrientation();
        doc["roll"] = euler.roll;
        doc["pitch"] = euler.pitch;
        doc["yaw"] = euler.yaw;

        auto gyro = _imu->getGyro();
        doc["roll_rate"] = gyro.x;
        doc["pitch_rate"] = gyro.y;
        doc["yaw_rate"] = gyro.z;

        doc["altitude"] = _imu->getAltitude();
        doc["climb_rate"] = _imu->getClimbRate();
        doc["imu_healthy"] = _imu->isHealthy();
        doc["flight_state"] = static_cast<int>(_imu->getFlightState());
    }

    // Controller state
    if (_controller)
    {
        doc["armed"] = _controller->isArmed();
        doc["mode"] = (int)_controller->getMode();
        doc["throttle_cut"] = _controller->isThrottleCut();
    }

    doc["uptime_ms"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleCalibrate()
{
    LOG_INF("Web: IMU calibration requested");

    // Push calibration command through CommandSystem (thread-safe)
    SystemCommand cmd;
    cmd.type = CMD_CALIBRATE;
    CommandSystem::instance().pushCommand(cmd);

    sendJson(200, "{\"ok\":true,\"message\":\"Calibration started\"}");
}

// ═══════════════════════════════════════════════════════════════════════════
// Flash Logs API Handlers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleFlashList()
{
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    // List files in LittleFS
    if (!LittleFS.begin(false))
    {
        sendError(500, "Filesystem not mounted");
        return;
    }

    File root = LittleFS.open("/");
    if (!root || !root.isDirectory())
    {
        sendError(500, "Cannot open root directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (!file.isDirectory() && String(file.name()).startsWith("log_"))
        {
            JsonObject obj = arr.add<JsonObject>();
            obj["name"] = String(file.name());
            obj["size"] = file.size();
        }
        file = root.openNextFile();
    }

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleFlashGet()
{
    // Extract filename from URI: /api/flash/log_001.csv
    String uri = _server->uri();
    if (!uri.startsWith("/api/flash/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String filename = "/" + uri.substring(11);  // Add leading /

    if (!LittleFS.exists(filename))
    {
        sendError(404, "File not found");
        return;
    }

    File file = LittleFS.open(filename, "r");
    if (!file)
    {
        sendError(500, "Cannot open file");
        return;
    }

    _server->setContentLength(file.size());
    _server->sendHeader("Content-Disposition", "attachment; filename=\"" + String(file.name()) + "\"");
    _server->send(200, "text/csv", "");

    // Stream file in chunks
    uint8_t buf[512];
    while (file.available())
    {
        size_t len = file.read(buf, sizeof(buf));
        _server->client().write(buf, len);
    }

    file.close();
}

void ArduFliteWebServer::handleFlashDelete()
{
    // Extract filename from URI: DELETE /api/flash/log_001.csv
    String uri = _server->uri();
    if (!uri.startsWith("/api/flash/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String filename = "/" + uri.substring(11);

    if (!LittleFS.exists(filename))
    {
        sendError(404, "File not found");
        return;
    }

    if (LittleFS.remove(filename))
    {
        sendJson(200, "{\"ok\":true}");
    }
    else
    {
        sendError(500, "Delete failed");
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Not Found Handler (routes parameterized paths)
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleNotFound()
{
    String uri = _server->uri();
    HTTPMethod method = _server->method();

    // Route /api/config/:key
    if (uri.startsWith("/api/config/") && uri.length() > 12)
    {
        if (method == HTTP_GET)
        {
            handleConfigGet();
            return;
        }
        else if (method == HTTP_PUT || method == HTTP_POST)
        {
            handleConfigSet();
            return;
        }
    }

    // Route /api/flash/:filename
    if (uri.startsWith("/api/flash/") && uri.length() > 11)
    {
        if (method == HTTP_GET)
        {
            handleFlashGet();
            return;
        }
        else if (method == HTTP_DELETE)
        {
            handleFlashDelete();
            return;
        }
    }

    sendError(404, "Not found");
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::sendJson(int code, const String& json)
{
    _server->sendHeader("Access-Control-Allow-Origin", "*");
    _server->send(code, "application/json", json);
}

void ArduFliteWebServer::sendError(int code, const char* message)
{
    String json = "{\"error\":\"" + String(message) + "\"}";
    sendJson(code, json);
}

#endif // ENABLE_WEB_SERVER
