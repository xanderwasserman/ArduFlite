/**
 * ConfigPersistence.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief NVS-based persistence for configuration parameters.
 *        Handles loading, saving, schema versioning, and migrations.
 */
#ifndef CONFIG_PERSISTENCE_H
#define CONFIG_PERSISTENCE_H

#include <Arduino.h>
#include "src/hal/board/Board.h"
#include "src/hal/platform/Storage.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <functional>
#include <vector>
#include <atomic>

#include "src/hal/platform/Mutex.h"

// Current schema version - increment when making breaking changes.
//   v2: unit suffixes on every unit-bearing key (att.deadband -> att.deadband_rad,
//       servo.*.min_pulse -> servo.*.min_pulse_us, ...). Renamed keys hash to new
//       NVS entries, so v1 values are simply not found and defaults apply.
//       WIPE NVS AFTER FLASHING - the code defaults are the current flying values.
#define CONFIG_SCHEMA_VERSION 2

// NVS namespace for configuration storage
#define CONFIG_NVS_NAMESPACE "arduflite"

/**
 * @brief Migration function signature.
 *        Called when loading config from an older schema version.
 * @param fromVersion The version being migrated from
 * @param toVersion   The version being migrated to
 */
using ConfigMigrationFn = std::function<void(uint32_t fromVersion, uint32_t toVersion)>;

/**
 * @brief Handles persistent storage of configuration parameters using ESP32 NVS.
 *
 * Features:
 * - Load all registered params from NVS on boot
 * - Save individual params or all dirty params
 * - Schema versioning with migration support
 * - JSON export/import for backup and sharing
 */
class ConfigPersistence {
public:
    /**
     * @brief Initialize the persistence layer.
     *        Call once at startup before loading config.
     */
    static void begin();

    /**
     * @brief Load all configuration from NVS.
     *        Falls back to defaults for missing keys.
     *        Runs migrations if schema version mismatch.
     */
    static void load();

    /**
     * @brief Save a single parameter to NVS.
     * @param key Parameter key
     * @return true if saved successfully
     */
    static bool save(const char* key);

    /**
     * @brief Save all dirty parameters to NVS.
     * @return Number of parameters saved
     */
    static size_t saveIfDirty();

    /**
     * @brief Save all parameters to NVS (ignoring dirty flags).
     * @return Number of parameters saved
     */
    static size_t saveAll();

    /**
     * @brief Erase all stored configuration from NVS.
     *        Registry values are NOT reset - call ConfigRegistry::resetAll() separately.
     */
    static void eraseAll();

    /**
     * @brief Register a migration function for schema upgrades.
     * @param fromVersion Source schema version
     * @param toVersion   Target schema version
     * @param migration   Function to run during migration
     */
    static void registerMigration(uint32_t fromVersion, uint32_t toVersion, ConfigMigrationFn migration);

    /**
     * @brief Export all configuration to JSON string.
     * @return JSON string containing all params
     */
    static String exportJson();

    /**
     * @brief Import configuration from JSON string.
     * @param json JSON string to import
     * @return Number of parameters successfully imported
     */
    static size_t importJson(const String& json);

    /**
     * @brief Get the stored schema version.
     * @return Schema version from NVS, or 0 if not found
     */
    static uint32_t getStoredVersion();

private:
    /// Borrowed from Board; not owned. See ADR-027 — this class used to
    /// include <Preferences.h>, which is Arduino-ESP32 only.
    static arduflite::hal::KeyValueStore& store();
    static bool _initialized;
    static std::atomic<arduflite::hal::Mutex*> _mutex;  ///< Protects store access
    
    struct Migration {
        uint32_t fromVersion;
        uint32_t toVersion;
        ConfigMigrationFn fn;
    };
    static std::vector<Migration> _migrations;

    static void ensureMutex();
    static void runMigrations(uint32_t fromVersion, uint32_t toVersion);
    static String shortenKey(const char* key);  // NVS keys max 15 chars
};

#endif // CONFIG_PERSISTENCE_H
