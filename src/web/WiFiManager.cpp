/**
 * WiFiManager.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include <chrono>
#include "src/hal/board/Board.h"
#include "include/WebConfiguration.h"

#if ENABLE_WEB_SERVER

#include "src/web/WiFiManager.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"
#include "include/ConfigKeys.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>

namespace
{
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);
const IPAddress AP_DHCP_START(192, 168, 4, 2);
}

WiFiManager& WiFiManager::instance()
{
    static WiFiManager _instance;
    return _instance;
}

namespace {
/// @return 0-15, or -1 for anything that is not a hex digit (including NUL).
int hexDigitValue(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    return -1;
}
} // namespace

bool WiFiManager::begin()
{
    if (_active)
    {
        LOG_WARN("WiFiManager already active");
        return true;
    }

    auto& reg = ConfigRegistry::instance();

    // Get base SSID from config, append chip ID for uniqueness
    String baseSsid = String(reg.get<std::string>(CONFIG_KEY_WEB_AP_SSID).c_str());
    if (baseSsid.isEmpty())
    {
        baseSsid = "ArduFlite";
    }

    // Get last 4 hex digits of chip ID for uniqueness.
    //
    // uniqueId() is the eFuse MAC formatted as 12 hex digits ("%012llX"), so
    // the bytes are parsed back out rather than summed as characters. That
    // reproduces the previous eFuse-MAC arithmetic EXACTLY, which
    // matters: this value ends up in the access point's SSID, and changing it
    // would silently rename the network every existing client has saved.
    const char* const uniqueId = arduflite::board::Board::instance().system().uniqueId();

    uint32_t chipId = 0;
    for (int i = 0; i < 6; i++)
    {
        const int high = hexDigitValue(uniqueId[i * 2]);
        const int low  = hexDigitValue(uniqueId[i * 2 + 1]);
        if (high < 0 || low < 0) { break; }   // short or malformed id
        chipId += static_cast<uint32_t>((high << 4) | low);
    }
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "-%04X", (uint16_t)(chipId & 0xFFFF));
    _ssid = baseSsid + String(suffix);

    // Get password. WPA2 requires 8+ characters; fall back for older/default configs.
    _password = String(reg.get<std::string>(CONFIG_KEY_WEB_AP_PASS).c_str());
    if (_password.length() < 8 || _password == "arduflite")
    {
        LOG_WARN("WiFi AP password is unset/default; using unique SSID as temporary password. Set web.ap_pass before field use.");
        _password = _ssid;
    }

    LOG_INF("Starting WiFi AP: %s", _ssid.c_str());

    // Disconnect from any existing WiFi and set to AP mode
    WiFi.disconnect(true);
    WiFi.mode(WIFI_AP);
    WiFi.softAPsetHostname("arduflite");

    if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET, AP_DHCP_START, AP_IP))
    {
        LOG_WARN("WiFi AP static config failed; continuing with core defaults");
    }

    bool success = WiFi.softAP(_ssid.c_str(), _password.c_str());
    LOG_INF("WiFi AP mode: WPA2 protected");

    if (!success)
    {
        LOG_ERR("Failed to start WiFi AP!");
        return false;
    }

    // Let the AP settle before touching power-save settings.
    arduflite::board::Board::instance().scheduler().sleepFor(
        std::chrono::milliseconds{ 100 });

    // Set power save mode off for better responsiveness
    // The last raw platform call outside src/hal, and it stays deliberately.
    //
    // This whole file is Arduino-WiFi: WiFi.softAP(), softAPConfig(),
    // DNSServer. None of those match the burn-down's pattern, so routing just
    // this one through a HAL would move the counter without moving the
    // coupling — the module would be exactly as portable as it is now. A WiFi
    // HAL is the real fix, and it is not worth inventing for a captive portal
    // that only exists on the ground (ADR-029: no interface without a second
    // implementation asking for it).
    esp_wifi_set_ps(WIFI_PS_NONE);

    _dnsServer.setTTL(60);
    _dnsActive = _dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    if (_dnsActive)
    {
        LOG_INF("Captive DNS started: all hostnames -> %s", WiFi.softAPIP().toString().c_str());
    }
    else
    {
        LOG_WARN("Captive DNS failed to start");
    }

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
    if (_dnsActive)
    {
        _dnsServer.stop();
        _dnsActive = false;
    }
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

void WiFiManager::processDns()
{
    if (_dnsActive)
    {
        _dnsServer.processNextRequest();
    }
}

#endif // ENABLE_WEB_SERVER
