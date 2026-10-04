/**
 * ConfigRegistry.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ConfigRegistry.h"

#include <chrono>
#include <mutex>
#include "src/utils/Logging.h"
#include "include/ConfigKeys.h"

#include <cstring>  // For strcmp, strncmp

/// How long a caller waits for the registry before giving up. Plain integer
/// milliseconds, not TickType_t — nothing here needs the RTOS's time base.
static constexpr int CONFIG_LOCK_TIMEOUT_MS = 100;

namespace {

/**
 * @brief Scoped lock over the registry mutex, tolerant of it being absent.
 *
 * Presents the same acquired() surface the 27 call sites already use, so the
 * FreeRTOS-to-hal::Mutex migration did not have to touch any of them.
 *
 * A null mutex reports ACQUIRED, deliberately: that is the pre-Board::begin()
 * window, where the only caller is static-init registration on one thread.
 * Reporting failure there would make every early registration silently fail.
 */
class RegistryLock {
public:
    explicit RegistryLock(arduflite::hal::Mutex* mutex) {
        if (mutex == nullptr) { _acquired = true; return; }
        _lock = std::unique_lock<arduflite::hal::Mutex>(
            *mutex, std::chrono::milliseconds(CONFIG_LOCK_TIMEOUT_MS));
        _acquired = _lock.owns_lock();
    }

    [[nodiscard]] bool acquired() const { return _acquired; }

private:
    std::unique_lock<arduflite::hal::Mutex> _lock;
    bool _acquired = false;
};

} // namespace

// Maximum time (ms) to wait for the ConfigRegistry mutex.
// Bounded to prevent infinite blocking if a lower-priority task holds the
// lock (priority inversion). 100 ms is generous for an in-memory map op.

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

    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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

void ConfigRegistry::setMutex(arduflite::hal::Mutex* mutex) {
    _mutex.store(mutex, std::memory_order_release);
}

void ConfigRegistry::ensureMutex() const {
    // Nothing to do. The mutex is injected by setMutex() from Board's pool,
    // because params register at static-init time before the scheduler exists.
    // Before setMutex() the registry is single-threaded by construction, and
    // after it there is exactly one mutex — so there is no race to guard.
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
std::string ConfigRegistry::get<std::string>(const char* key) const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return std::string();

    auto it = _params.find(key);
    if (it == _params.end()) {
        LOG_WARN("Config key not found: %s", key);
        return std::string();
    }
    if (it->second.type != ConfigType::STRING) {
        LOG_WARN("Config type mismatch for %s: expected STRING", key);
        return std::string();
    }
    return std::string(it->second.currentVal.s);
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
bool ConfigRegistry::set<std::string>(const char* key, std::string value) {
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return;

    auto it = _params.find(key);
    if (it != _params.end()) {
        if (!validate(it->second, value)) {
            // Out-of-range NVS value: log and keep the current (default) value.
            LOG_WARN("Config: NVS value for '%s' failed range validation — keeping default.", key);
            it->second.dirty = false;
        } else {
            it->second.currentVal = value;
            it->second.dirty = false;  // Loaded from storage, not dirty
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Requires Reboot
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigRegistry::requiresReboot(const char* key) const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
        RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return false;

    for (const auto& [key, param] : _params) {
        if (param.dirty) return true;
    }
    return false;
}

std::vector<std::string> ConfigRegistry::getDirtyKeys() const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));

    std::vector<std::string> result;
    if (!lock.acquired()) return result;

    for (const auto& [key, param] : _params) {
        if (param.dirty) {
            result.push_back(key.c_str());  // Copy key as std::string
        }
    }
    return result;
}

void ConfigRegistry::clearDirty(const char* key) {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return;

    auto it = _params.find(key);
    if (it != _params.end()) {
        it->second.dirty = false;
    }
}

void ConfigRegistry::clearAllDirty() {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return;

    for (auto& [key, param] : _params) {
        param.dirty = false;
    }
}

void ConfigRegistry::markDirty(const char* key) {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
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
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return std::nullopt;

    auto it = _params.find(key);
    if (it != _params.end()) {
        return it->second;  // Return copy
    }
    return std::nullopt;
}

std::unordered_map<std::string, ConfigParam> ConfigRegistry::getAllParams() const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return {};
    return _params;  // Return copy
}

size_t ConfigRegistry::count() const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));
    if (!lock.acquired()) return 0;
    return _params.size();
}

std::vector<ConfigParam> ConfigRegistry::list(const char* pattern) const {
    ensureMutex();
    RegistryLock lock(_mutex.load(std::memory_order_acquire));

    std::vector<ConfigParam> result;
    if (!lock.acquired()) return result;

    for (const auto& [key, param] : _params) {
        if (matchPattern(pattern, key.c_str())) {
            result.push_back(param);  // Copy
        }
    }
    return result;
}
