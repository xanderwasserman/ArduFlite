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
#include "src/web/WiFiManager.h"
#include "src/web/WebUI.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"
#include "src/controller/ArduFliteController.h"
#include "src/core/FlightTypes.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/state/StateManagement.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "include/ConfigKeys.h"

#include <ArduinoJson.h>
#include "src/hal/board/Board.h"
#include <esp_system.h>

namespace
{
const char* WEB_HEADER_KEYS[] = { "X-ArduFlite-Token" };
constexpr size_t WEB_HEADER_KEY_COUNT = sizeof(WEB_HEADER_KEYS) / sizeof(WEB_HEADER_KEYS[0]);

bool isLogFilename(const String& name)
{
    if (name.length() != 11) return false;
    if (!name.startsWith("log_") || !name.endsWith(".csv")) return false;

    for (int i = 4; i <= 6; ++i)
    {
        if (name[i] < '0' || name[i] > '9') return false;
    }

    return true;
}
}

ArduFliteWebServer& ArduFliteWebServer::instance()
{
    static ArduFliteWebServer _instance;
    return _instance;
}

ArduFliteWebServer::ArduFliteWebServer()
{
}

bool ArduFliteWebServer::begin(ArduFliteController* controller,
                                arduflite::estimation::InertialSubsystem* imu,
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
    auto& system = arduflite::board::Board::instance().system();
    snprintf(_csrfToken, sizeof(_csrfToken), "%08lX%08lX",
             static_cast<unsigned long>(system.randomWord()),
             static_cast<unsigned long>(system.randomWord()));

    // Create server on heap
    _server = new WebServer(HTTP_PORT);
    if (!_server)
    {
        LOG_ERR("Failed to allocate WebServer");
        return false;
    }

    _server->collectHeaders(WEB_HEADER_KEYS, WEB_HEADER_KEY_COUNT);

    setupRoutes();

    // Start server task
    arduflite::hal::TaskConfig taskConfig;
    taskConfig.name       = "Web";
    taskConfig.stackBytes = TASK_STACK_SIZE;
    taskConfig.priority   = arduflite::hal::Priority::Web;

    auto task = arduflite::board::Board::instance().scheduler().spawn(
        taskConfig, &serverTask, this);
    if (!task)
    {
        LOG_ERR("Failed to create WebServer task");
        delete _server;
        _server = nullptr;
        return false;
    }
    _taskHandle = task.value();

    _running = true;
    LOG_INF("WebServer started on port %d", HTTP_PORT);
    return true;
}

void ArduFliteWebServer::stop()
{
    if (!_running) return;

    LOG_INF("Stopping WebServer");
    _running = false;

    // Cooperative: _running is already false above, and the loop checks it.
    if (_taskHandle != nullptr)
    {
        _taskHandle->requestStop();
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

    auto& scheduler = arduflite::board::Board::instance().scheduler();

    while (_running)
    {
        WiFiManager::instance().processDns();
        _server->handleClient();
        scheduler.sleepFor(std::chrono::milliseconds{ 5 });
    }

    // Just return: the scheduler's trampoline ends the task (ADR-058).
}

void ArduFliteWebServer::setupRoutes()
{
    // Web UI (root and static assets)
    _server->on("/", HTTP_GET, std::bind(&ArduFliteWebServer::handleWebUI, this));
    _server->on("/index.html", HTTP_GET, std::bind(&ArduFliteWebServer::handleWebUI, this));

    // Captive portal probes used by phones/laptops when joining an AP with no internet.
    _server->on("/generate_204", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/gen_204", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/hotspot-detect.html", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/library/test/success.html", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/success.txt", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/connecttest.txt", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));
    _server->on("/ncsi.txt", HTTP_ANY, std::bind(&ArduFliteWebServer::handleCaptivePortalProbe, this));

    // Config API
    _server->on("/api/session", HTTP_GET, std::bind(&ArduFliteWebServer::handleSession, this));
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
    _server->sendHeader("Cache-Control", "no-store");
    _server->sendHeader("Content-Encoding", "gzip");
    _server->send_P(200, PSTR("text/html"), reinterpret_cast<const char*>(WEB_UI_HTML_GZ), WEB_UI_HTML_GZ_LEN);
}

void ArduFliteWebServer::handleCaptivePortalProbe()
{
    String target = "http://" + WiFiManager::instance().getIP().toString() + "/";
    _server->sendHeader("Location", target, true);
    _server->sendHeader("Cache-Control", "no-store");
    _server->send(302, "text/plain", "Redirecting to ArduFlite");
}

void ArduFliteWebServer::handleSession()
{
    JsonDocument doc;
    doc["token"] = _csrfToken;

    String response;
    serializeJson(doc, response);

    _server->sendHeader("Cache-Control", "no-store");
    sendJson(200, response);
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
            // Redact sensitive credential keys — never expose them over the REST API.
            if (std::string(p.key) == CONFIG_KEY_WEB_AP_PASS) continue;

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

    // Redact sensitive credential keys — return 403 rather than expose the value.
    if (key == CONFIG_KEY_WEB_AP_PASS)
    {
        sendError(403, "Forbidden: sensitive key");
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
    if (!isMutationAuthorized()) return;

    // Extract key from URI: PUT /api/config/rate.roll.kp
    String uri = _server->uri();
    if (!uri.startsWith("/api/config/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    // Block configuration changes while armed — modifying PID gains in flight is unsafe.
    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before changing configuration");
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

    // Reject oversized payloads to prevent heap exhaustion on ESP32-C3.
    if (_server->arg("plain").length() > 512)
    {
        sendError(413, "Payload too large");
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
            success = reg.set<std::string>(key.c_str(), std::string(doc["value"].as<const char*>() ? doc["value"].as<const char*>() : ""));
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
    if (!isMutationAuthorized()) return;

    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before resetting configuration");
        return;
    }
    LOG_INF("Web: Resetting all config to defaults");
    ConfigRegistry::instance().resetAll();
    ConfigPersistence::saveIfDirty();
    sendJson(200, "{\"ok\":true}");
}

void ArduFliteWebServer::handleConfigExport()
{
    // Export config, then strip sensitive credential keys before sending.
    String json = ConfigPersistence::exportJson();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err || !doc["params"].is<JsonObject>())
    {
        sendError(500, "Config export failed");
        return;
    }

    doc["params"].as<JsonObject>().remove(CONFIG_KEY_WEB_AP_PASS);

    String filtered;
    serializeJson(doc, filtered);
    sendJson(200, filtered);
}

void ArduFliteWebServer::handleConfigImport()
{
    if (!isMutationAuthorized()) return;

    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before importing configuration");
        return;
    }

    if (!_server->hasArg("plain"))
    {
        sendError(400, "Body required");
        return;
    }

    String json = _server->arg("plain");

    // Reject oversized payloads to prevent heap exhaustion on ESP32-C3.
    if (json.length() > 16384)
    {
        sendError(413, "Payload too large");
        return;
    }

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
    if (!isMutationAuthorized()) return;

    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before rebooting");
        return;
    }
    LOG_INF("Web: Reboot requested");
    sendJson(200, "{\"ok\":true,\"message\":\"Rebooting...\"}");

    // Delay to allow response to be sent
    auto& board = arduflite::board::Board::instance();
    board.scheduler().sleepFor(std::chrono::milliseconds{ 500 });
    board.system().reboot();
}

// ═══════════════════════════════════════════════════════════════════════════
// System API Handlers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::handleSystemStatus()
{
    JsonDocument doc;

    // Basic system info
    auto& board  = arduflite::board::Board::instance();
    auto& system = board.system();
    doc["uptime_ms"]   = static_cast<std::uint32_t>(
        board.clock().now().time_since_epoch().count() / 1000);
    doc["free_heap"]   = system.freeHeapBytes();
    doc["min_heap"]    = system.minFreeHeapBytes();
    doc["chip_model"]  = system.platformName();
    doc["sdk_version"] = system.sdkVersion();

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
        const auto snapshotHealth = _imu->snapshotHealth();
        doc["imu_healthy"] = _imu->healthy();
        doc["flight_state"] = static_cast<int>(getFlightState());
        doc["imu_snapshot_retries"] = snapshotHealth.totalRetries;
        doc["imu_snapshot_max_retries"] = snapshotHealth.maxRetries;
        doc["imu_snapshot_retry_limit_hits"] = snapshotHealth.retryLimitHits;
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
        // One lock-free read for the whole frame.
        const arduflite::estimation::ImuState snapshot = _imu->state();
        const auto snapshotHealth = _imu->snapshotHealth();

        auto euler = snapshot.euler_deg;
        doc["roll"] = euler.roll;
        doc["pitch"] = euler.pitch;
        doc["yaw"] = euler.yaw;

        auto gyro = snapshot.gyro_dps;
        doc["roll_rate"] = gyro.x;
        doc["pitch_rate"] = gyro.y;
        doc["yaw_rate"] = gyro.z;

        doc["altitude"] = snapshot.altitude_m;
        doc["climb_rate"] = snapshot.climbRate_mps;
        doc["imu_healthy"] = _imu->healthy();
        doc["flight_state"] = static_cast<int>(getFlightState());
        doc["imu_snapshot_retries"] = snapshotHealth.totalRetries;
        doc["imu_snapshot_max_retries"] = snapshotHealth.maxRetries;
        doc["imu_snapshot_retry_limit_hits"] = snapshotHealth.retryLimitHits;
    }

    // Controller state
    if (_controller)
    {
        doc["armed"] = _controller->isArmed();
        doc["mode"] = (int)_controller->getMode();
        doc["throttle_cut"] = _controller->isThrottleCut();
    }

    auto& statusBoard = arduflite::board::Board::instance();
    doc["uptime_ms"] = static_cast<std::uint32_t>(
        statusBoard.clock().now().time_since_epoch().count() / 1000);
    doc["free_heap"] = statusBoard.system().freeHeapBytes();

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleCalibrate()
{
    if (!isMutationAuthorized()) return;

    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before calibrating");
        return;
    }
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
    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before accessing flash logs");
        return;
    }
    if (_flashTelemetry && _flashTelemetry->isLogging())
    {
        sendError(423, "Locked: stop logging before accessing flash logs");
        return;
    }

    auto& store = arduflite::board::Board::instance().logs();
    if (store.begin() != arduflite::Status::Ok)
    {
        sendError(500, "Filesystem not mounted");
        return;
    }

    JsonDocument doc;
    JsonArray arr = doc["files"].to<JsonArray>();

    // Enumerated through the store rather than by walking the directory: the
    // store only knows about log sessions, so a stray file cannot appear in the
    // UI's log list at all.
    constexpr size_t kMaxListed = 64;
    uint16_t indices[kMaxListed];
    const size_t count = store.listSessions(indices, kMaxListed);

    for (size_t i = 0; i < count; ++i)
    {
        char name[32];
        snprintf(name, sizeof(name), "log_%03u.csv", (unsigned)indices[i]);

        uint32_t bytes = 0;
        (void)store.sessionSize(indices[i], bytes);

        JsonObject obj = arr.add<JsonObject>();
        obj["name"] = name;
        obj["size"] = bytes;
    }

    // Usage, so the UI can show a space indicator.
    uint32_t used = 0, total = 0;
    (void)store.usage(used, total);
    doc["used"]  = used;
    doc["total"] = total;

    String response;
    serializeJson(doc, response);
    sendJson(200, response);
}

void ArduFliteWebServer::handleFlashGet()
{
    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before downloading flash logs");
        return;
    }
    if (_flashTelemetry && _flashTelemetry->isLogging())
    {
        sendError(423, "Locked: stop logging before downloading flash logs");
        return;
    }

    // Extract filename from URI: /api/flash/log_001.csv
    String uri = _server->uri();
    if (!uri.startsWith("/api/flash/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String name = uri.substring(11);  // bare filename, no leading /

    // Guard against path traversal: reject any name containing '/' or "..".
    if (name.isEmpty() || name.indexOf('/') >= 0 || name.indexOf("..") >= 0 || !isLogFilename(name))
    {
        sendError(400, "Invalid filename");
        return;
    }

    // The filename was validated above; recover the index it names.
    unsigned index = 0;
    if (sscanf(name.c_str(), "log_%03u.csv", &index) != 1 || index > 999u)
    {
        sendError(400, "Invalid filename");
        return;
    }

    auto& store = arduflite::board::Board::instance().logs();
    uint32_t size = 0;
    if (store.sessionSize((uint16_t)index, size) != arduflite::Status::Ok)
    {
        sendError(404, "File not found");
        return;
    }

    _server->setContentLength(size);
    _server->sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
    _server->send(200, "text/csv", "");

    // Stream in chunks. The offset walk is why device::LogStore::readSession
    // takes one — a flight log is far larger than any buffer here.
    uint8_t buf[512];
    size_t  offset = 0;
    while (_server->client().connected())
    {
        size_t length = 0;
        if (store.readSession((uint16_t)index, buf, sizeof(buf), offset, length)
                != arduflite::Status::Ok)
        {
            break;
        }
        if (length == 0) { break; }   // end of data

        _server->client().write(buf, length);
        offset += length;
        arduflite::board::Board::instance().scheduler().sleepFor(
            std::chrono::milliseconds{ 1 });
    }
}

void ArduFliteWebServer::handleFlashDelete()
{
    if (!isMutationAuthorized()) return;

    if (_controller && _controller->isArmed())
    {
        sendError(423, "Locked: disarm before deleting flash logs");
        return;
    }
    if (_flashTelemetry && _flashTelemetry->isLogging())
    {
        sendError(423, "Locked: stop logging before deleting flash logs");
        return;
    }

    // Extract filename from URI: DELETE /api/flash/log_001.csv
    String uri = _server->uri();
    if (!uri.startsWith("/api/flash/"))
    {
        sendError(400, "Invalid path");
        return;
    }

    String name = uri.substring(11);  // bare filename, no leading /

    // Guard against path traversal: reject any name containing '/' or "..".
    if (name.isEmpty() || name.indexOf('/') >= 0 || name.indexOf("..") >= 0 || !isLogFilename(name))
    {
        sendError(400, "Invalid filename");
        return;
    }

    String filename = "/" + name;

    unsigned index = 0;
    if (sscanf(name.c_str(), "log_%03u.csv", &index) != 1 || index > 999u)
    {
        sendError(400, "Invalid filename");
        return;
    }
    auto& store = arduflite::board::Board::instance().logs();

    if (store.removeSession((uint16_t)index) == arduflite::Status::Ok)
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

    if (uri.startsWith("/api/"))
    {
        sendError(404, "Not found");
        return;
    }

    if (method == HTTP_GET || method == HTTP_HEAD)
    {
        handleCaptivePortalProbe();
        return;
    }

    sendError(404, "Not found");
}

// ═══════════════════════════════════════════════════════════════════════════
// Helpers
// ═══════════════════════════════════════════════════════════════════════════

void ArduFliteWebServer::sendJson(int code, const String& json)
{
    // No CORS wildcard — the embedded web UI is same-origin (served from this same server)
    // and does not need cross-origin headers. Wildcard CORS would allow any page loaded
    // on a device connected to the AP to call mutating endpoints cross-origin.
    _server->send(code, "application/json", json);
}

void ArduFliteWebServer::sendError(int code, const char* message)
{
    // Use snprintf with a fixed buffer to avoid Arduino String heap allocations.
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"error\":\"%s\"}", message);
    sendJson(code, buf);
}

bool ArduFliteWebServer::isMutationAuthorized()
{
    String token = _server->header("X-ArduFlite-Token");
    if (token.length() == 0 || token != _csrfToken)
    {
        LOG_WARN("Web: rejected mutating request without a valid session token");
        sendError(403, "Forbidden");
        return false;
    }

    return true;
}

#endif // ENABLE_WEB_SERVER
