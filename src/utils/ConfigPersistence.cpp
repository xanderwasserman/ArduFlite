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
#include <mutex>

#include "src/hal/board/Board.h"

#include <ArduinoJson.h>  // For JSON export/import

namespace {

/**
 * @brief Serialised size of a config value, by type.
 *
 * Strings are stored at their actual length including the terminator, not the
 * full CONFIG_STRING_MAX_LEN buffer — writing 64 bytes for an 8-character SSID
 * wastes NVS space and, more importantly, makes a short read indistinguishable
 * from a correct one.
 */
std::size_t serialisedSize(ConfigType type, const ConfigValue& value)
{
    switch (type)
    {
        case ConfigType::FLOAT:  return sizeof(value.f);
        case ConfigType::INT32:  return sizeof(value.i);
        case ConfigType::UINT8:  return sizeof(value.u8);
        case ConfigType::BOOL:   return sizeof(value.b);
        case ConfigType::STRING: return strnlen(value.s, CONFIG_STRING_MAX_LEN - 1) + 1;
    }
    return 0;
}

/// Raw bytes of a value, for KeyValueStore::write().
const void* valueBytes(ConfigType type, const ConfigValue& value)
{
    switch (type)
    {
        case ConfigType::FLOAT:  return &value.f;
        case ConfigType::INT32:  return &value.i;
        case ConfigType::UINT8:  return &value.u8;
        case ConfigType::BOOL:   return &value.b;
        case ConfigType::STRING: return value.s;
    }
    return nullptr;
}

/**
 * @brief Read one value, leaving `out` untouched if the key is absent.
 *
 * @return false when the key is missing OR the stored length does not match
 *         what the type expects. A length mismatch means the key was written
 *         by a build where that key had a different type; silently
 *         reinterpreting those bytes would produce a plausible wrong value
 *         rather than a visible failure.
 */
bool readValue(arduflite::hal::KeyValueStore& store, const char* key,
               ConfigType type, ConfigValue& out)
{
    std::size_t length = 0;

    if (type == ConfigType::STRING)
    {
        char buffer[CONFIG_STRING_MAX_LEN]{};
        if (store.read(key, buffer, sizeof(buffer), length) != arduflite::Status::Ok)
        {
            return false;
        }
        buffer[CONFIG_STRING_MAX_LEN - 1] = '\0';
        strncpy(out.s, buffer, CONFIG_STRING_MAX_LEN - 1);
        out.s[CONFIG_STRING_MAX_LEN - 1] = '\0';
        return true;
    }

    ConfigValue scratch;
    void* dst = nullptr;
    std::size_t expected = 0;
    switch (type)
    {
        case ConfigType::FLOAT: dst = &scratch.f;  expected = sizeof(scratch.f);  break;
        case ConfigType::INT32: dst = &scratch.i;  expected = sizeof(scratch.i);  break;
        case ConfigType::UINT8: dst = &scratch.u8; expected = sizeof(scratch.u8); break;
        case ConfigType::BOOL:  dst = &scratch.b;  expected = sizeof(scratch.b);  break;
        default: return false;
    }

    if (store.read(key, dst, expected, length) != arduflite::Status::Ok) { return false; }
    if (length != expected) { return false; }

    out = scratch;
    return true;
}

} // namespace

// Static member initialization
/// The configuration store, borrowed from Board. Not owned: Board holds it in
/// constinit storage that outlives everything here.
arduflite::hal::KeyValueStore& ConfigPersistence::store()
{
    return arduflite::board::Board::instance().configStore();
}
bool ConfigPersistence::_initialized = false;
std::atomic<arduflite::hal::Mutex*> ConfigPersistence::_mutex{nullptr};
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
    // Thread-safe lazy initialisation via atomic compare-exchange.
    //
    // The loser of the race cannot return its surplus mutex: allocMutex() hands
    // out entries from a fixed pool and nothing reclaims them (ADR-011). That
    // leaks one pool slot, once, and only if two tasks reach a persistence API
    // for the very first time simultaneously.
    if (_mutex.load(std::memory_order_acquire) != nullptr) { return; }

    auto allocated = arduflite::board::Board::instance().allocMutex();
    if (!allocated) {
        LOG_ERR("ConfigPersistence: no mutex available - persistence disabled");
        return;  // Will fail on lock acquisition
    }

    arduflite::hal::Mutex* expected = nullptr;
    (void)_mutex.compare_exchange_strong(expected, allocated.value(),
                                         std::memory_order_release,
                                         std::memory_order_relaxed);
}

// ═══════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::begin() {
    ensureMutex();
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return; }
    std::unique_lock lock(*mutex);
    
    if (_initialized) return;
    
    if (store().begin() != arduflite::Status::Ok) {
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
        auto* mutex = _mutex.load(std::memory_order_acquire);
        if (mutex == nullptr) { return; }
        std::unique_lock lock(*mutex);

        // Check schema version
        uint32_t storedVersion = 0;
        {
            std::size_t length = 0;
            if (store().read(SCHEMA_VERSION_KEY, &storedVersion, sizeof(storedVersion), length)
                    != arduflite::Status::Ok || length != sizeof(storedVersion))
            {
                storedVersion = 0;
            }
        }
        if (storedVersion > 0 && storedVersion < CONFIG_SCHEMA_VERSION) {
            LOG_INF("Config schema migration: v%u -> v%u", storedVersion, CONFIG_SCHEMA_VERSION);
            runMigrations(storedVersion, CONFIG_SCHEMA_VERSION);
        }

        for (const auto& [key, param] : params) {
            String shortKey = shortenKey(key.c_str());
            
            ConfigValue value = param.defaultVal;
            
            // Absent or unreadable keys keep the default that `value` already
            // holds. One mechanism for that decision, not two: no separate
            // isKey() probe, and no per-getter fallback argument.
            if (!readValue(store(), shortKey.c_str(), param.type, value)) {
                defaults++;
                continue;
            }

            loadedValues.push_back({key, value});
        }

        // Update schema version in NVS. If this fails the values are still
        // loaded, but the NEXT boot sees a stale version and re-runs migrations
        // that have already been applied — worth a loud line.
        const uint32_t schemaVersion = CONFIG_SCHEMA_VERSION;
        if (store().write(SCHEMA_VERSION_KEY, &schemaVersion, sizeof(schemaVersion))
                != arduflite::Status::Ok) {
            LOG_ERR("Config: schema version not written - migrations may re-run at next boot");
        }
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
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return false; }
    std::unique_lock lock(*mutex);
    
    String shortKey = shortenKey(key);

    const bool written =
        store().write(shortKey.c_str(),
                      valueBytes(param.type, param.currentVal),
                      serialisedSize(param.type, param.currentVal)) == arduflite::Status::Ok;

    // Only clear the dirty flag if the write actually succeeded.
    if (written) {
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
    std::vector<std::string> dirtyKeys = ConfigRegistry::instance().getDirtyKeys();
    size_t saved = 0;

    for (const std::string& key : dirtyKeys) {
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
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return 0; }
    std::unique_lock lock(*mutex);
    size_t saved = 0;

    for (const auto& [key, param] : params) {
        String shortKey = shortenKey(key.c_str());

        // Counted only on success. Counting unconditionally would report
        // "saved all N" even when every write failed — exactly the situation
        // where the operator most needs to know it did not work.
        if (store().write(shortKey.c_str(),
                          valueBytes(param.type, param.currentVal),
                          serialisedSize(param.type, param.currentVal)) == arduflite::Status::Ok)
        {
            saved++;
        }
    }

    ConfigRegistry::instance().clearAllDirty();

    const uint32_t schemaVersion = CONFIG_SCHEMA_VERSION;
    if (store().write(SCHEMA_VERSION_KEY, &schemaVersion, sizeof(schemaVersion))
            != arduflite::Status::Ok) {
        LOG_ERR("Config: schema version not written - migrations may re-run at next boot");
    }

    if (saved != params.size()) {
        LOG_ERR("Config save incomplete: %u of %u params written", saved, params.size());
    } else {
        LOG_INF("Config saved: all %u params", saved);
    }
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
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return; }
    std::unique_lock lock(*mutex);

    const arduflite::Status status = store().eraseAll();
    if (status != arduflite::Status::Ok) {
        // Never claim success here. Someone erasing config is usually trying to
        // recover from a bad state, and a false confirmation sends them looking
        // for the fault somewhere else entirely.
        LOG_ERR("Config erase FAILED (%s) - stored values are unchanged",
                arduflite::toString(status));
        return;
    }
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
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return 0; }
    std::unique_lock lock(*mutex);
    
    uint32_t version = 0;
    std::size_t length = 0;
    if (store().read(SCHEMA_VERSION_KEY, &version, sizeof(version), length) != arduflite::Status::Ok
        || length != sizeof(version))
    {
        return 0;
    }
    return version;
}

// ═══════════════════════════════════════════════════════════════════════════
// Migrations
// ═══════════════════════════════════════════════════════════════════════════

void ConfigPersistence::registerMigration(uint32_t fromVersion, uint32_t toVersion, ConfigMigrationFn migration) {
    ensureMutex();
    auto* mutex = _mutex.load(std::memory_order_acquire);
    if (mutex == nullptr) { return; }
    std::unique_lock lock(*mutex);
    
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
    doc["exported"] = static_cast<std::uint32_t>(
        arduflite::board::Board::instance().clock().now()
            .time_since_epoch().count() / 1000);  // TODO: RTC timestamp if available

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
                    ok = ConfigRegistry::instance().set<std::string>(key, std::string(value.as<const char*>()));
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
