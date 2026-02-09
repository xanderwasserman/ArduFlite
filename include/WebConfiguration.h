/**
 * WebConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 09 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Compile-time configuration for Web Server feature.
 *
 * The web server adds ~500KB to flash due to WiFi/TCP/HTTP libraries.
 * Disable it for minimal "flight-only" firmware.
 *
 * Build variants:
 *   ./build.sh lolin        # Full build with web server (default)
 *   ./build.sh lolin lite   # Minimal build without web server (~800KB)
 */
#ifndef WEB_CONFIGURATION_H
#define WEB_CONFIGURATION_H

// ═══════════════════════════════════════════════════════════════════════════
// ENABLE_WEB_SERVER - Compile-time feature flag
// ═══════════════════════════════════════════════════════════════════════════
//
// Set to 1 to include WiFi AP and WebServer (adds ~500KB flash)
// Set to 0 for minimal flight-only firmware
//
// This can be overridden via build flags:
//   -DENABLE_WEB_SERVER=0
//
#ifndef ENABLE_WEB_SERVER
#define ENABLE_WEB_SERVER 1
#endif

#endif // WEB_CONFIGURATION_H
