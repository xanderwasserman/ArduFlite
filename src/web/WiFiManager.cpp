/**
 * WiFiManager.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "include/WebConfiguration.h"

#if ENABLE_WEB_SERVER

#include "src/web/WiFiManager.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"
#include "include/ConfigKeys.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>

WiFiManager& WiFiManager::instance()
{
    static WiFiManager _instance;
    return _instance;
}

bool WiFiManager::begin()
{
    if (_active)
    {
        LOG_WARN("WiFiManager already active");
        return true;
    }

    auto& reg = ConfigRegistry::instance();

    // Get base SSID from config, append chip ID for uniqueness
    String baseSsid = reg.get<String>(CONFIG_KEY_WEB_AP_SSID);
    if (baseSsid.isEmpty())
    {
        baseSsid = "ArduFlite";
    }

    // Get last 4 hex digits of chip ID for uniqueness
    uint32_t chipId = 0;
    for (int i = 0; i < 6; i++)
    {
        chipId += ((uint32_t)ESP.getEfuseMac() >> (i * 8)) & 0xFF;
    }
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%04X", (uint16_t)(chipId & 0xFFFF));
    _ssid = baseSsid + String(suffix);

    // Get password (empty = open network)
    _password = reg.get<String>(CONFIG_KEY_WEB_AP_PASS);

    LOG_INF("Starting WiFi AP: %s", _ssid.c_str());

    // Disconnect from any existing WiFi and set to AP mode
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);

    // Start AP (open or with password)
    bool success;
    if (_password.isEmpty())
    {
        success = WiFi.softAP(_ssid.c_str());
        LOG_INF("WiFi AP mode: Open (no password)");
    }
    else
    {
        success = WiFi.softAP(_ssid.c_str(), _password.c_str());
        LOG_INF("WiFi AP mode: WPA2 protected");
    }

    if (!success)
    {
        LOG_ERR("Failed to start WiFi AP!");
        return false;
    }

    // Set power save mode off for better responsiveness
    esp_wifi_set_ps(WIFI_PS_NONE);

    // Start mDNS responder for arduflite.local
    if (MDNS.begin("arduflite"))
    {
        MDNS.addService("http", "tcp", 80);
        LOG_INF("mDNS started: http://arduflite.local");
    }
    else
    {
        LOG_WARN("mDNS failed to start");
    }

    _active = true;
    LOG_INF("WiFi AP started. Connect to: %s", _ssid.c_str());
    LOG_INF("AP IP address: %s", WiFi.softAPIP().toString().c_str());

    return true;
}

void WiFiManager::stop()
{
    if (!_active) return;

    LOG_INF("Stopping WiFi AP");
    MDNS.end();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _active = false;
}

IPAddress WiFiManager::getIP() const
{
    return WiFi.softAPIP();
}

uint8_t WiFiManager::getClientCount() const
{
    return WiFi.softAPgetStationNum();
}

#endif // ENABLE_WEB_SERVER
