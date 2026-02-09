/**
 * WiFiManager.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief WiFi Access Point manager for configuration web server.
 *        Creates a hotspot for connecting from mobile/desktop browsers.
 */
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "include/WebConfiguration.h"

#if ENABLE_WEB_SERVER

#include <Arduino.h>
#include <WiFi.h>

/**
 * @class WiFiManager
 * @brief Singleton manager for WiFi Access Point mode.
 *
 * Starts a WiFi hotspot with configurable SSID/password from ConfigRegistry.
 * Clients connect directly to the ESP32 to access the web configuration UI.
 */
class WiFiManager {
public:
    /**
     * @brief Get singleton instance.
     */
    static WiFiManager& instance();

    /**
     * @brief Initialize and start the WiFi Access Point.
     * 
     * Reads configuration from ConfigRegistry:
     * - web.ap_ssid: Base SSID (chip ID suffix appended for uniqueness)
     * - web.ap_pass: Password (empty = open network)
     * 
     * @return true if AP started successfully
     */
    bool begin();

    /**
     * @brief Stop the WiFi Access Point.
     */
    void stop();

    /**
     * @brief Check if WiFi AP is active.
     */
    bool isActive() const { return _active; }

    /**
     * @brief Get the AP IP address.
     * @return IP address (typically 192.168.4.1)
     */
    IPAddress getIP() const;

    /**
     * @brief Get the full SSID (including chip ID suffix).
     */
    const String& getSSID() const { return _ssid; }

    /**
     * @brief Get number of connected clients.
     */
    uint8_t getClientCount() const;

private:
    WiFiManager() = default;
    WiFiManager(const WiFiManager&) = delete;
    WiFiManager& operator=(const WiFiManager&) = delete;

    bool    _active = false;
    String  _ssid;
    String  _password;
};

#endif // ENABLE_WEB_SERVER

#endif // WIFI_MANAGER_H
