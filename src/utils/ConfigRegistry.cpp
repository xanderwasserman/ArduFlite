/**
 * ConfigRegistry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ConfigRegistry.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <cstring>  // For strcmp, strncmp

// ═══════════════════════════════════════════════════════════════════════════
// LOCK ORDERING (must follow to prevent deadlock):
//   1. ConfigRegistry._mutex (this class)
//   2. ConfigPersistence._mutex (if persistence operations needed)
//
// RULE: Registry lock can be acquired first. Persistence must acquire
//       Registry snapshot BEFORE acquiring its own lock.
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Singleton Instance
// ═══════════════════════════════════════════════════════════════════════════

ConfigRegistry& ConfigRegistry::instance() {
    static ConfigRegistry inst;
    return inst;
}

// ═══════════════════════════════════════════════════════════════════════════
// Initialization
// ═══════════════════════════════════════════════════════════════════════════

void ConfigRegistry::init() {
    if (_initialized) return;
    
    // Create the mutex now that FreeRTOS is ready
    ensureMutex();
    
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;
    
    // Process any pending registrations from static initialization
    for (const auto& pending : _pendingRegistrations) {
        if (_params.find(pending.key) != _params.end()) {
            continue;  // Already registered
        }
        
        ConfigParam param;
        param.key           = pending.key;
        param.description   = pending.description;
        param.type          = pending.type;
        param.defaultVal    = pending.defaultVal;
        param.minVal        = pending.minVal;
        param.maxVal        = pending.maxVal;
        param.currentVal    = pending.defaultVal;
        param.dirty         = false;
        param.requiresReboot = pending.requiresReboot;
        
        _params[pending.key] = param;
    }
    
    _pendingRegistrations.clear();
    _pendingRegistrations.shrink_to_fit();  // Release memory
    _initialized = true;
    
    LOG_INF("ConfigRegistry initialized with %u params", _params.size());
}

// ═══════════════════════════════════════════════════════════════════════════
// Mutex Management
// ═══════════════════════════════════════════════════════════════════════════

void ConfigRegistry::ensureMutex() const {
    // Thread-safe mutex initialization using atomic compare-exchange
    SemaphoreHandle_t expected = nullptr;
    if (_mutex.load(std::memory_order_acquire) == nullptr) {
        SemaphoreHandle_t newMutex = xSemaphoreCreateMutex();
        if (newMutex == nullptr) {
            LOG_ERR("ConfigRegistry: Failed to create mutex - out of memory");
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
// Registration
// ═══════════════════════════════════════════════════════════════════════════

void ConfigRegistry::registerParam(
    const char* key,
    ConfigType  type,
    ConfigValue defaultVal,
    ConfigValue minVal,
    ConfigValue maxVal,
    const char* description,
    bool        requiresReboot
) {
    // If called before init() (during static initialization), queue it
    if (!_initialized) {
        _pendingRegistrations.push_back({
            key, type, defaultVal, minVal, maxVal, description, requiresReboot
        });
        return;
    }
    
    // Normal registration path (FreeRTOS is ready)
    registerParamInternal(key, type, defaultVal, minVal, maxVal, description, requiresReboot);
}

void ConfigRegistry::registerParamInternal(
    const char* key,
    ConfigType  type,
    ConfigValue defaultVal,
    ConfigValue minVal,
    ConfigValue maxVal,
    const char* description,
    bool        requiresReboot
) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    // Check for duplicate registration
    if (_params.find(key) != _params.end()) {
        LOG_WARN("Config key already registered: %s", key);
        return;
    }

    ConfigParam param;
    param.key           = key;
    param.description   = description;
    param.type          = type;
    param.defaultVal    = defaultVal;
    param.minVal        = minVal;
    param.maxVal        = maxVal;
    param.currentVal    = defaultVal;  // Start with default
    param.dirty         = false;
    param.requiresReboot = requiresReboot;

    _params[key] = param;
}

// ═══════════════════════════════════════════════════════════════════════════
// Get Template Specializations
// ═══════════════════════════════════════════════════════════════════════════

template<>
float ConfigRegistry::get<float>(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0.0f;

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return 0.0f;
    }
    if (it->second.type != ConfigType::FLOAT) {
        LOG_WARN("Config type mismatch for %s: expected FLOAT", key);
        return 0.0f;
    }
    return it->second.currentVal.f;
}

template<>
int32_t ConfigRegistry::get<int32_t>(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0;

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return 0;
    }
    if (it->second.type != ConfigType::INT32) {
        LOG_WARN("Config type mismatch for %s: expected INT32", key);
        return 0;
    }
    return it->second.currentVal.i;
}

template<>
uint8_t ConfigRegistry::get<uint8_t>(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0;

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return 0;
    }
    if (it->second.type != ConfigType::UINT8) {
        LOG_WARN("Config type mismatch for %s: expected UINT8", key);
        return 0;
    }
    return it->second.currentVal.u8;
}

template<>
bool ConfigRegistry::get<bool>(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return false;

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return false;
    }
    if (it->second.type != ConfigType::BOOL) {
        LOG_WARN("Config type mismatch for %s: expected BOOL", key);
        return false;
    }
    return it->second.currentVal.b;
}

template<>
String ConfigRegistry::get<String>(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return String();

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return String();
    }
    if (it->second.type != ConfigType::STRING) {
        LOG_WARN("Config type mismatch for %s: expected STRING", key);
        return String();
    }
    return String(it->second.currentVal.s);
}

// ═══════════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigRegistry::validate(const ConfigParam& param, const ConfigValue& value) const {
    switch (param.type) {
        case ConfigType::FLOAT:
            return (value.f >= param.minVal.f && value.f <= param.maxVal.f);
        case ConfigType::INT32:
            return (value.i >= param.minVal.i && value.i <= param.maxVal.i);
        case ConfigType::UINT8:
            return (value.u8 >= param.minVal.u8 && value.u8 <= param.maxVal.u8);
        case ConfigType::BOOL:
            return true;  // Booleans are always valid
        case ConfigType::STRING:
            return true;  // Strings not validated for range
        default:
            return false;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Set Template Specializations
// ═══════════════════════════════════════════════════════════════════════════

template<>
bool ConfigRegistry::set<float>(const char* key, float value) {
    ensureMutex();
    
    ConfigChange change;
    change.key = key;
    change.type = ConfigType::FLOAT;
    change.newValue.f = value;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            LOG_WARN("Config key not found: %s", key);
            return false;
        }
        if (it->second.type != ConfigType::FLOAT) {
            LOG_WARN("Config type mismatch for %s: expected FLOAT", key);
            return false;
        }

        ConfigValue newVal;
        newVal.f = value;

        if (!validate(it->second, newVal)) {
            LOG_WARN("Config validation failed for %s: %.3f not in [%.3f, %.3f]",
                     key, value, it->second.minVal.f, it->second.maxVal.f);
            return false;
        }

        change.oldValue = it->second.currentVal;
        it->second.currentVal.f = value;
        it->second.dirty = true;
    }

    // Notify observers outside lock
    notifyObservers(change);
    return true;
}

template<>
bool ConfigRegistry::set<int32_t>(const char* key, int32_t value) {
    ensureMutex();
    
    ConfigChange change;
    change.key = key;
    change.type = ConfigType::INT32;
    change.newValue.i = value;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            LOG_WARN("Config key not found: %s", key);
            return false;
        }
        if (it->second.type != ConfigType::INT32) {
            LOG_WARN("Config type mismatch for %s: expected INT32", key);
            return false;
        }

        ConfigValue newVal;
        newVal.i = value;

        if (!validate(it->second, newVal)) {
            LOG_WARN("Config validation failed for %s: %d not in [%d, %d]",
                     key, value, it->second.minVal.i, it->second.maxVal.i);
            return false;
        }

        change.oldValue = it->second.currentVal;
        it->second.currentVal.i = value;
        it->second.dirty = true;
    }

    notifyObservers(change);
    return true;
}

template<>
bool ConfigRegistry::set<uint8_t>(const char* key, uint8_t value) {
    ensureMutex();
    
    ConfigChange change;
    change.key = key;
    change.type = ConfigType::UINT8;
    change.newValue.u8 = value;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            LOG_WARN("Config key not found: %s", key);
            return false;
        }
        if (it->second.type != ConfigType::UINT8) {
            LOG_WARN("Config type mismatch for %s: expected UINT8", key);
            return false;
        }

        ConfigValue newVal;
        newVal.u8 = value;

        if (!validate(it->second, newVal)) {
            LOG_WARN("Config validation failed for %s: %u not in [%u, %u]",
                     key, value, it->second.minVal.u8, it->second.maxVal.u8);
            return false;
        }

        change.oldValue = it->second.currentVal;
        it->second.currentVal.u8 = value;
        it->second.dirty = true;
    }

    notifyObservers(change);
    return true;
}

template<>
bool ConfigRegistry::set<bool>(const char* key, bool value) {
    ensureMutex();
    
    ConfigChange change;
    change.key = key;
    change.type = ConfigType::BOOL;
    change.newValue.b = value;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            LOG_WARN("Config key not found: %s", key);
            return false;
        }
        if (it->second.type != ConfigType::BOOL) {
            LOG_WARN("Config type mismatch for %s: expected BOOL", key);
            return false;
        }

        change.oldValue = it->second.currentVal;
        it->second.currentVal.b = value;
        it->second.dirty = true;
    }

    notifyObservers(change);
    return true;
}

template<>
bool ConfigRegistry::set<String>(const char* key, String value) {
    ensureMutex();
    
    // Validate string length
    if (value.length() >= CONFIG_STRING_MAX_LEN) {
        LOG_WARN("Config string too long for %s: %u >= %u",
                 key, value.length(), CONFIG_STRING_MAX_LEN);
        return false;
    }
    
    ConfigChange change;
    change.key = key;
    change.type = ConfigType::STRING;
    strncpy(change.newValue.s, value.c_str(), CONFIG_STRING_MAX_LEN - 1);
    change.newValue.s[CONFIG_STRING_MAX_LEN - 1] = '\0';

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            LOG_WARN("Config key not found: %s", key);
            return false;
        }
        if (it->second.type != ConfigType::STRING) {
            LOG_WARN("Config type mismatch for %s: expected STRING", key);
            return false;
        }

        change.oldValue = it->second.currentVal;
        strncpy(it->second.currentVal.s, value.c_str(), CONFIG_STRING_MAX_LEN - 1);
        it->second.currentVal.s[CONFIG_STRING_MAX_LEN - 1] = '\0';
        it->second.dirty = true;
    }

    notifyObservers(change);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Raw Set (for persistence loading)
// ═══════════════════════════════════════════════════════════════════════════

void ConfigRegistry::setRaw(const char* key, ConfigValue value) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    auto it = _params.find(key);
    if (it != _params.end()) {
        it->second.currentVal = value;
        it->second.dirty = false;  // Loaded from storage, not dirty
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Requires Reboot
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigRegistry::requiresReboot(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return false;

    auto it = _params.find(key);
    return (it != _params.end() && it->second.requiresReboot);
}

// ═══════════════════════════════════════════════════════════════════════════
// Reset
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigRegistry::reset(const char* key) {
    ensureMutex();
    
    ConfigChange change;
    change.key = key;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return false;

        auto it = _params.find(key);
        if (it == _params.end()) {
            return false;
        }

        change.type = it->second.type;
        change.oldValue = it->second.currentVal;
        change.newValue = it->second.defaultVal;

        it->second.currentVal = it->second.defaultVal;
        it->second.dirty = true;
    }

    notifyObservers(change);
    return true;
}

void ConfigRegistry::resetAll() {
    ensureMutex();
    
    std::vector<ConfigChange> changes;

    {
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return;

        for (auto& [key, param] : _params) {
            ConfigChange change;
            change.key = param.key;
            change.type = param.type;
            change.oldValue = param.currentVal;
            change.newValue = param.defaultVal;

            param.currentVal = param.defaultVal;
            param.dirty = true;

            changes.push_back(change);
        }
    }

    // Notify observers for all changes
    for (const auto& change : changes) {
        notifyObservers(change);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Observer Management
// ═══════════════════════════════════════════════════════════════════════════

void ConfigRegistry::subscribe(const char* pattern, ConfigObserver callback) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    _observers.emplace_back(pattern, callback);
}

bool ConfigRegistry::matchPattern(const char* pattern, const char* key) const {
    // Exact match
    if (strcmp(pattern, key) == 0) return true;
    
    // Root wildcard matches everything
    if (strcmp(pattern, "*") == 0) return true;

    // Trailing wildcard pattern (e.g., "rate.roll.*")
    size_t patLen = strlen(pattern);
    if (patLen >= 2 && pattern[patLen - 1] == '*' && pattern[patLen - 2] == '.') {
        // Check prefix without ".*"
        size_t prefixLen = patLen - 2;
        return (strncmp(pattern, key, prefixLen) == 0 && 
                (key[prefixLen] == '.' || key[prefixLen] == '\0'));
    }

    // Pattern like "rate.*" (wildcard at end after dot)
    if (patLen >= 1 && pattern[patLen - 1] == '*') {
        size_t prefixLen = patLen - 1;
        return (strncmp(pattern, key, prefixLen) == 0);
    }

    return false;
}

void ConfigRegistry::notifyObservers(const ConfigChange& change) {
    // Copy observers under lock, then notify outside lock
    std::vector<ConfigObserver> matchingObservers;
    
    {
        ensureMutex();
        SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
        if (!lock.acquired()) return;

        for (const auto& [pattern, callback] : _observers) {
            if (matchPattern(pattern.c_str(), change.key.c_str())) {
                matchingObservers.push_back(callback);
            }
        }
    }

    // Notify outside lock to prevent deadlocks
    // Note: ESP32 has exceptions disabled by default, so callbacks must not throw
    for (const auto& callback : matchingObservers) {
        callback(change);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Dirty Tracking
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigRegistry::hasDirty() const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return false;

    for (const auto& [key, param] : _params) {
        if (param.dirty) return true;
    }
    return false;
}

std::vector<String> ConfigRegistry::getDirtyKeys() const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    
    std::vector<String> result;
    if (!lock.acquired()) return result;

    for (const auto& [key, param] : _params) {
        if (param.dirty) {
            result.push_back(key.c_str());  // Copy key as String
        }
    }
    return result;
}

void ConfigRegistry::clearDirty(const char* key) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    auto it = _params.find(key);
    if (it != _params.end()) {
        it->second.dirty = false;
    }
}

void ConfigRegistry::clearAllDirty() {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    for (auto& [key, param] : _params) {
        param.dirty = false;
    }
}

void ConfigRegistry::markDirty(const char* key) {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return;

    auto it = _params.find(key);
    if (it != _params.end()) {
        it->second.dirty = true;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Introspection
// ═══════════════════════════════════════════════════════════════════════════

std::optional<ConfigParam> ConfigRegistry::getParam(const char* key) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return std::nullopt;

    auto it = _params.find(key);
    if (it != _params.end()) {
        return it->second;  // Return copy
    }
    return std::nullopt;
}

std::unordered_map<std::string, ConfigParam> ConfigRegistry::getAllParams() const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return {};
    return _params;  // Return copy
}

size_t ConfigRegistry::count() const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    if (!lock.acquired()) return 0;
    return _params.size();
}

std::vector<ConfigParam> ConfigRegistry::list(const char* pattern) const {
    ensureMutex();
    SemaphoreLock lock(_mutex.load(), portMAX_DELAY);
    
    std::vector<ConfigParam> result;
    if (!lock.acquired()) return result;

    for (const auto& [key, param] : _params) {
        if (matchPattern(pattern, key.c_str())) {
            result.push_back(param);  // Copy
        }
    }
    return result;
}
