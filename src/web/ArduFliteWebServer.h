/**
 * ArduFliteWebServer.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Web server for configuration and monitoring.
 *        Provides REST API and serves web UI for config management.
 */
#ifndef ARDUFLITE_WEB_SERVER_H
#define ARDUFLITE_WEB_SERVER_H

#include "include/WebConfiguration.h"

#if ENABLE_WEB_SERVER

#include <Arduino.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Forward declarations
class ArduFliteController;
class ArduFliteIMU;
class ArduFliteFlashTelemetry;

/**
 * @class ArduFliteWebServer
 * @brief Singleton web server providing REST API and web UI.
 *
 * Runs in a dedicated FreeRTOS task at low priority (1).
 * Provides endpoints for:
 * - Configuration CRUD via REST API
 * - System status and monitoring
 * - Flash log download
 * - Web-based configuration UI
 */
class ArduFliteWebServer {
public:
    /**
     * @brief Get singleton instance.
     */
    static ArduFliteWebServer& instance();

    /**
     * @brief Start the web server task.
     * @param controller Pointer to controller for status info
     * @param imu Pointer to IMU for status info
     * @param flashTelemetry Pointer to flash telemetry for log access
     * @return true if started successfully
     */
    bool begin(ArduFliteController* controller = nullptr,
               ArduFliteIMU* imu = nullptr,
               ArduFliteFlashTelemetry* flashTelemetry = nullptr);

    /**
     * @brief Stop the web server.
     */
    void stop();

    /**
     * @brief Check if server is running.
     */
    bool isRunning() const { return _running; }

private:
    ArduFliteWebServer();
    ArduFliteWebServer(const ArduFliteWebServer&) = delete;
    ArduFliteWebServer& operator=(const ArduFliteWebServer&) = delete;

    // Task management
    static void serverTask(void* pv);
    void run();

    // Route setup
    void setupRoutes();

    // API handlers
    void handleRoot();
    void handleNotFound();
    
    // Config API
    void handleConfigList();
    void handleConfigGet();
    void handleConfigSet();
    void handleConfigReset();
    void handleConfigExport();
    void handleConfigImport();
    void handleConfigReboot();

    // System API
    void handleSystemStatus();
    void handleSystemLogs();
    void handleTelemetry();
    void handleCalibrate();

    // Flash logs API
    void handleFlashList();
    void handleFlashGet();
    void handleFlashDelete();

    // Web UI (served from PROGMEM)
    void handleWebUI();
    void handleCSS();
    void handleJS();

    // Helpers
    void sendJson(int code, const String& json);
    void sendError(int code, const char* message);
    String getContentType(const String& filename);

    // State
    WebServer*              _server = nullptr;
    TaskHandle_t            _taskHandle = nullptr;
    bool                    _running = false;

    // Dependencies (optional, for status info)
    ArduFliteController*    _controller = nullptr;
    ArduFliteIMU*           _imu = nullptr;
    ArduFliteFlashTelemetry* _flashTelemetry = nullptr;

    static constexpr uint16_t HTTP_PORT = 80;
    static constexpr size_t TASK_STACK_SIZE = 12288;  // 12KB for JSON + HTTP handling
    static constexpr UBaseType_t TASK_PRIORITY = 1;
};

#endif // ENABLE_WEB_SERVER

#endif // ARDUFLITE_WEB_SERVER_H
