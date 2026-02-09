/**
 * ConfigPersistence.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ConfigPersistence.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"  // For SemaphoreLock

#include <ArduinoJson.h>  // For JSON export/import

// Static member initialization
Preferences ConfigPersistence::_prefs;
bool ConfigPersistence::_initialized = false;
std::atomic<SemaphoreHandle_t> ConfigPersistence::_mutex{nullptr};
std::vector<ConfigPersistence::Migration> ConfigPersistence::_migrations;

// NVS key for schema version
static const char* const SCHEMA_VERSION_KEY = "_schema_ver";

// Maximum JSON document size for export/import (bound memory usage)
static constexpr size_t MAX_JSON_DOC_SIZE = 16384;

// Maximum params to export (prevent unbounded memory usage)
static constexpr size_t MAX_EXPORT_PARAMS = 200;

// ═══════════════════════════════════════════════════════════════════════════
// LOCK ORDERING (must follow to prevent deadlock):
//   1. ConfigRegistry._mutex (if needed)
//   2. ConfigPersistence._mutex (if needed)
//
// RULE: Always acquire Registry snapshot BEFORE acquiring Persistence lock.
//       Never call Registry methods while holding Persistence lock.
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Mutex Management
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::ensureMutex() {
    // Thread-safe mutex initialization using atomic compare-exchange
    SemaphoreHandle_t expected = nullptr;
    if (_mutex.load(std::memory_order_acquire) == nullptr) {
        SemaphoreHandle_t newMutex = xSemaphoreCreateMutex();
        if (newMutex == nullptr) {
            LOG_ERR("ConfigPersistence: Failed to create mutex - out of memory");
            return;  // Will fail on lock acquisition
        }
        // Atomically set _mutex if it's still nullptr
        if (!_mutex.compare_exchange_strong(expected, newMutex,
                                             std::memory_order_release,
                                             std::memory_order_relaxed)) {
            // Another thread already created a mutex, delete ours
            vSemaphoreDelete(newMutex);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::begin() {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;
    
    if (_initialized) return;
    
    if (!_prefs.begin(CONFIG_NVS_NAMESPACE, false)) {
        LOG_ERR("Failed to open NVS namespace: %s", CONFIG_NVS_NAMESPACE);
        return;
    }
    
    _initialized = true;
    LOG_INF("ConfigPersistence initialized");
}

// ═══════════════════════════════════════════════════════════════════════════
// Key Shortening (NVS keys limited to 15 chars)
// ═══════════════════════════════════════════════════════════════════════════

String ConfigPersistence::shortenKey(const char* key) {
    // NVS keys are limited to 15 characters.
    // Use a full 64-bit hash for long keys to minimize collisions.
    String shortKey;
    size_t len = strlen(key);
    
    if (len <= 15) {
        shortKey = key;
    } else {
        // Use full 64-bit FNV-1a hash for better distribution
        // This gives us 15 hex chars (60 bits) which is sufficient
        uint64_t hash = 14695981039346656037ULL;  // FNV offset basis
        for (size_t i = 0; i < len; i++) {
            hash ^= static_cast<uint64_t>(key[i]);
            hash *= 1099511628211ULL;  // FNV prime
        }
        // Use underscore prefix to indicate shortened key, then 14 hex chars
        char buf[16];
        snprintf(buf, sizeof(buf), "_%014llX", (unsigned long long)(hash & 0xFFFFFFFFFFFFFFULL));
        shortKey = buf;
    }
    
    return shortKey;
}

// ═══════════════════════════════════════════════════════════════════════════
// Load
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::load() {
    // Ensure initialized (outside lock to avoid recursion issues)
    if (!_initialized) {
        begin();
        if (!_initialized) return;
    }
    
    // Get params snapshot BEFORE acquiring our lock (honors lock ordering)
    auto params = ConfigRegistry::instance().getAllParams();
    
    // Structure to hold loaded values (key -> value pairs)
    struct LoadedParam {
        std::string key;
        ConfigValue value;
    };
    std::vector<LoadedParam> loadedValues;
    loadedValues.reserve(params.size());
    
    size_t defaults = 0;
    
    // Read all NVS values under Persistence lock
    {
        ensureMutex();
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return;

        // Check schema version
        uint32_t storedVersion = _prefs.getUInt(SCHEMA_VERSION_KEY, 0);
        if (storedVersion > 0 && storedVersion < CONFIG_SCHEMA_VERSION) {
            LOG_INF("Config schema migration: v%u -> v%u", storedVersion, CONFIG_SCHEMA_VERSION);
            runMigrations(storedVersion, CONFIG_SCHEMA_VERSION);
        }

        for (const auto& [key, param] : params) {
            String shortKey = shortenKey(key.c_str());
            
            // Check if key exists in NVS
            if (!_prefs.isKey(shortKey.c_str())) {
                // Not in NVS, keep default value
                defaults++;
                continue;
            }

            ConfigValue value;
            
            switch (param.type) {
                case ConfigType::FLOAT:
                    value.f = _prefs.getFloat(shortKey.c_str(), param.defaultVal.f);
                    break;
                case ConfigType::INT32:
                    value.i = _prefs.getInt(shortKey.c_str(), param.defaultVal.i);
                    break;
                case ConfigType::UINT8:
                    value.u8 = _prefs.getUChar(shortKey.c_str(), param.defaultVal.u8);
                    break;
                case ConfigType::BOOL:
                    value.b = _prefs.getBool(shortKey.c_str(), param.defaultVal.b);
                    break;
                case ConfigType::STRING:
                    {
                        String strVal = _prefs.getString(shortKey.c_str(), param.defaultVal.s);
                        strncpy(value.s, strVal.c_str(), CONFIG_STRING_MAX_LEN - 1);
                        value.s[CONFIG_STRING_MAX_LEN - 1] = '\0';
                    }
                    break;
            }

            loadedValues.push_back({key, value});
        }

        // Update schema version in NVS
        _prefs.putUInt(SCHEMA_VERSION_KEY, CONFIG_SCHEMA_VERSION);
    }
    // Lock released here
    
    // Now apply loaded values to Registry OUTSIDE the Persistence lock
    // (honors lock ordering: Registry first, Persistence second)
    for (const auto& lp : loadedValues) {
        ConfigRegistry::instance().setRaw(lp.key.c_str(), lp.value);
    }

    LOG_INF("Config loaded: %u from NVS, %u defaults", loadedValues.size(), defaults);
}

// ═══════════════════════════════════════════════════════════════════════════
// Save
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigPersistence::save(const char* key) {
    // Ensure initialized (outside lock to avoid recursion issues)
    if (!_initialized) {
        begin();
        if (!_initialized) return false;
    }
    
    // Get param snapshot BEFORE acquiring our lock (avoids lock ordering issues)
    auto optParam = ConfigRegistry::instance().getParam(key);
    if (!optParam) {
        LOG_WARN("Cannot save unknown config key: %s", key);
        return false;
    }
    const ConfigParam param = *optParam;  // Local copy

    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return false;
    
    String shortKey = shortenKey(key);
    size_t written = 0;

    switch (param.type) {
        case ConfigType::FLOAT:
            written = _prefs.putFloat(shortKey.c_str(), param.currentVal.f);
            break;
        case ConfigType::INT32:
            written = _prefs.putInt(shortKey.c_str(), param.currentVal.i);
            break;
        case ConfigType::UINT8:
            written = _prefs.putUChar(shortKey.c_str(), param.currentVal.u8);
            break;
        case ConfigType::BOOL:
            written = _prefs.putBool(shortKey.c_str(), param.currentVal.b);
            break;
        case ConfigType::STRING:
            written = _prefs.putString(shortKey.c_str(), param.currentVal.s);
            break;
    }

    // Only clear dirty flag if NVS write succeeded (written > 0)
    if (written > 0) {
        ConfigRegistry::instance().clearDirty(key);
        return true;
    } else {
        LOG_WARN("NVS write failed for key: %s", key);
        return false;
    }
}

size_t ConfigPersistence::saveIfDirty() {
    // Ensure initialized
    if (!_initialized) {
        begin();
        if (!_initialized) return 0;
    }

    // Check if anything is dirty (thread-safe in Registry)
    if (!ConfigRegistry::instance().hasDirty()) {
        return 0;
    }

    // Get dirty keys snapshot (thread-safe copies from Registry)
    std::vector<String> dirtyKeys = ConfigRegistry::instance().getDirtyKeys();
    size_t saved = 0;

    for (const String& key : dirtyKeys) {
        if (save(key.c_str())) {
            saved++;
        }
    }

    if (saved > 0) {
        LOG_INF("Config saved: %u params to NVS", saved);
    }

    return saved;
}

size_t ConfigPersistence::saveAll() {
    // Ensure initialized (outside lock to avoid recursion issues)
    if (!_initialized) {
        begin();
        if (!_initialized) return 0;
    }
    
    // Get params snapshot BEFORE acquiring our lock (avoids lock ordering issues)
    auto params = ConfigRegistry::instance().getAllParams();
    
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0;
    size_t saved = 0;

    for (const auto& [key, param] : params) {
        String shortKey = shortenKey(key.c_str());

        switch (param.type) {
            case ConfigType::FLOAT:
                _prefs.putFloat(shortKey.c_str(), param.currentVal.f);
                break;
            case ConfigType::INT32:
                _prefs.putInt(shortKey.c_str(), param.currentVal.i);
                break;
            case ConfigType::UINT8:
                _prefs.putUChar(shortKey.c_str(), param.currentVal.u8);
                break;
            case ConfigType::BOOL:
                _prefs.putBool(shortKey.c_str(), param.currentVal.b);
                break;
            case ConfigType::STRING:
                _prefs.putString(shortKey.c_str(), param.currentVal.s);
                break;
        }
        saved++;
    }

    ConfigRegistry::instance().clearAllDirty();
    _prefs.putUInt(SCHEMA_VERSION_KEY, CONFIG_SCHEMA_VERSION);

    LOG_INF("Config saved: all %u params to NVS", saved);
    return saved;
}

// ═══════════════════════════════════════════════════════════════════════════
// Erase
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::eraseAll() {
    // Ensure initialized (outside lock to avoid recursion issues)
    if (!_initialized) {
        begin();
        if (!_initialized) return;
    }
    
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    _prefs.clear();
    LOG_INF("Config erased from NVS");
}

// ═══════════════════════════════════════════════════════════════════════════
// Schema Version
// ═══════════════════════════════════════════════════════════════════════════

uint32_t ConfigPersistence::getStoredVersion() {
    // Ensure initialized (outside lock to avoid recursion issues)
    if (!_initialized) {
        begin();
        if (!_initialized) return 0;
    }
    
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0;
    
    return _prefs.getUInt(SCHEMA_VERSION_KEY, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// Migrations
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::registerMigration(uint32_t fromVersion, uint32_t toVersion, ConfigMigrationFn migration) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;
    
    _migrations.push_back({fromVersion, toVersion, migration});
}

void ConfigPersistence::runMigrations(uint32_t fromVersion, uint32_t toVersion) {
    for (uint32_t v = fromVersion; v < toVersion; v++) {
        for (const auto& m : _migrations) {
            if (m.fromVersion == v && m.toVersion == v + 1) {
                LOG_INF("Running migration v%u -> v%u", v, v + 1);
                m.fn(v, v + 1);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON Export
// ═══════════════════════════════════════════════════════════════════════════

String ConfigPersistence::exportJson() {
    // Get params snapshot BEFORE acquiring our lock (avoids lock ordering issues)
    auto allParams = ConfigRegistry::instance().getAllParams();
    
    // Note: We don't need the Persistence mutex for JSON export since we're
    // not accessing NVS - just serializing the in-memory snapshot.
    // This also avoids potential deadlocks with Registry.
    
    // ArduinoJson 7.x: Use JsonDocument with capacity control
    JsonDocument doc;

    doc["version"] = CONFIG_SCHEMA_VERSION;
    doc["exported"] = millis();  // TODO: Add RTC timestamp if available

    JsonObject params = doc["params"].to<JsonObject>();
    
    size_t paramCount = 0;
    for (const auto& [key, param] : allParams) {
        // Check if we're exceeding reasonable limits
        if (++paramCount > MAX_EXPORT_PARAMS) {
            LOG_WARN("JSON export truncated: param count exceeded %u", MAX_EXPORT_PARAMS);
            break;
        }
        
        switch (param.type) {
            case ConfigType::FLOAT:
                params[key] = param.currentVal.f;
                break;
            case ConfigType::INT32:
                params[key] = param.currentVal.i;
                break;
            case ConfigType::UINT8:
                params[key] = param.currentVal.u8;
                break;
            case ConfigType::BOOL:
                params[key] = param.currentVal.b;
                break;
            case ConfigType::STRING:
                params[key] = param.currentVal.s;
                break;
        }
    }

    String output;
    serializeJsonPretty(doc, output);
    return output;
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON Import
// ═══════════════════════════════════════════════════════════════════════════

size_t ConfigPersistence::importJson(const String& json) {
    // Size check before parsing
    if (json.length() > MAX_JSON_DOC_SIZE) {
        LOG_ERR("JSON too large for import: %u > %u bytes", json.length(), MAX_JSON_DOC_SIZE);
        return 0;
    }
    
    // Note: We don't need the Persistence mutex for JSON import since we're
    // not accessing NVS directly - ConfigRegistry::set() handles its own locking.
    // This avoids potential deadlocks with Registry.
    
    // ArduinoJson 7.x: Use JsonDocument (dynamically sized)
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        LOG_ERR("JSON parse error: %s", error.c_str());
        return 0;
    }

    // Optionally check version
    uint32_t jsonVersion = doc["version"] | 0;
    if (jsonVersion > CONFIG_SCHEMA_VERSION) {
        LOG_WARN("JSON version %u > current %u, some params may be ignored", 
                 jsonVersion, CONFIG_SCHEMA_VERSION);
    }

    JsonObject params = doc["params"];
    if (params.isNull()) {
        LOG_ERR("JSON missing 'params' object");
        return 0;
    }

    size_t imported = 0;
    size_t errors = 0;

    for (JsonPair kv : params) {
        const char* key = kv.key().c_str();
        JsonVariant value = kv.value();

        auto optParam = ConfigRegistry::instance().getParam(key);
        if (!optParam) {
            LOG_WARN("Import: unknown key '%s', skipping", key);
            errors++;
            continue;
        }
        const ConfigParam& param = *optParam;

        bool ok = false;
        switch (param.type) {
            case ConfigType::FLOAT:
                if (value.is<float>() || value.is<double>()) {
                    ok = ConfigRegistry::instance().set<float>(key, value.as<float>());
                }
                break;
            case ConfigType::INT32:
                if (value.is<int>()) {
                    ok = ConfigRegistry::instance().set<int32_t>(key, value.as<int32_t>());
                }
                break;
            case ConfigType::UINT8:
                if (value.is<int>()) {
                    // Check range before casting to prevent silent truncation
                    int v = value.as<int>();
                    if (v >= 0 && v <= 255) {
                        ok = ConfigRegistry::instance().set<uint8_t>(key, static_cast<uint8_t>(v));
                    } else {
                        LOG_WARN("Import: value %d out of uint8 range for '%s'", v, key);
                    }
                }
                break;
            case ConfigType::BOOL:
                if (value.is<bool>()) {
                    ok = ConfigRegistry::instance().set<bool>(key, value.as<bool>());
                }
                break;
            case ConfigType::STRING:
                if (value.is<const char*>()) {
                    ok = ConfigRegistry::instance().set<String>(key, String(value.as<const char*>()));
                }
                break;
        }

        if (ok) {
            imported++;
        } else {
            LOG_WARN("Import: failed to set '%s'", key);
            errors++;
        }
    }

    LOG_INF("Config import: %u succeeded, %u errors", imported, errors);
    return imported;
}
