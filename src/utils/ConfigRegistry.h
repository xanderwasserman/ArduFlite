/**
 * ConfigRegistry.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Central registry for all configuration parameters.
 *        Provides type-safe get/set with validation, observer callbacks,
 *        and dirty-tracking for persistence.
 */
#ifndef CONFIG_REGISTRY_H
#define CONFIG_REGISTRY_H

#include <Arduino.h>
#include <functional>
#include <vector>
#include <unordered_map>
#include <optional>
#include <string>
#include <atomic>

// ═══════════════════════════════════════════════════════════════════════════
// Configuration Value Types
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Supported configuration parameter types.
 */
enum class ConfigType : uint8_t {
    FLOAT = 0,
    INT32,
    UINT8,
    BOOL,
    STRING
};

/// Maximum string length for config values (including null terminator)
static constexpr size_t CONFIG_STRING_MAX_LEN = 32;

/**
 * @brief Union to hold any config value type.
 */
union ConfigValue {
    float    f;
    int32_t  i;
    uint8_t  u8;
    bool     b;
    char     s[CONFIG_STRING_MAX_LEN];  // Max string length for config values

    ConfigValue() : i(0) {}
    ConfigValue(float v)   : f(v) {}
    ConfigValue(int32_t v) : i(v) {}
    ConfigValue(uint8_t v) : u8(v) {}
    ConfigValue(bool v)    : b(v) {}
    
    // String constructor - copies up to CONFIG_STRING_MAX_LEN-1 chars
    ConfigValue(const char* str) {
        if (str) {
            strncpy(s, str, CONFIG_STRING_MAX_LEN - 1);
            s[CONFIG_STRING_MAX_LEN - 1] = '\0';
        } else {
            s[0] = '\0';
        }
    }
};

/**
 * @brief Describes a single configuration parameter.
 */
struct ConfigParam {
    const char* key;           ///< Unique key (e.g., "rate.roll.kp")
    const char* description;   ///< Human-readable description
    ConfigType  type;          ///< Data type
    ConfigValue defaultVal;    ///< Default value
    ConfigValue minVal;        ///< Minimum allowed value
    ConfigValue maxVal;        ///< Maximum allowed value
    ConfigValue currentVal;    ///< Current value
    bool        dirty;         ///< True if modified since last save
    bool        requiresReboot;///< True if change requires reboot
};

/**
 * @brief Change notification passed to observers.
 *        Key is stored as String to prevent dangling pointers after lock release.
 */
struct ConfigChange {
    String      key;   ///< Owned copy of key string
    ConfigType  type;
    ConfigValue oldValue;
    ConfigValue newValue;
};

/**
 * @brief Observer callback signature.
 */
using ConfigObserver = std::function<void(const ConfigChange&)>;

// ═══════════════════════════════════════════════════════════════════════════
// ConfigRegistry Singleton
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Central registry for all configuration parameters.
 *
 * Thread-safe singleton that manages registration, validation, persistence
 * tracking, and observer notifications for all config values.
 *
 * Usage:
 *   // Get value (returns default if not found)
 *   float kp = ConfigRegistry::get<float>(CONFIG_KEY_RATE_ROLL_KP);
 *
 *   // Set value (validates, marks dirty, notifies observers)
 *   bool ok = ConfigRegistry::set(CONFIG_KEY_RATE_ROLL_KP, 0.12f);
 *
 *   // Subscribe to changes
 *   ConfigRegistry::subscribe(CONFIG_KEY_RATE_ALL, myCallback);
 */
class ConfigRegistry {
public:
    /**
     * @brief Get singleton instance.
     */
    static ConfigRegistry& instance();

    /**
     * @brief Initialize the registry (call after FreeRTOS is ready).
     *        Processes any registrations that occurred before FreeRTOS started.
     *        Safe to call multiple times.
     */
    void init();

    /**
     * @brief Register a new configuration parameter.
     * @param key         Unique key string
     * @param type        Value type
     * @param defaultVal  Default value
     * @param minVal      Minimum value (for validation)
     * @param maxVal      Maximum value (for validation)
     * @param description Human-readable description
     * @param requiresReboot True if changing this requires a reboot
     */
    void registerParam(
        const char* key,
        ConfigType  type,
        ConfigValue defaultVal,
        ConfigValue minVal,
        ConfigValue maxVal,
        const char* description,
        bool        requiresReboot = false
    );

    /**
     * @brief Get a parameter value by key.
     * @tparam T Expected type (float, int32_t, uint8_t, bool)
     * @param key Parameter key
     * @return Current value, or default if not found
     */
    template<typename T>
    T get(const char* key) const;

    /**
     * @brief Set a parameter value.
     * @tparam T Value type
     * @param key   Parameter key
     * @param value New value
     * @return true if set succeeded, false if validation failed or key not found
     */
    template<typename T>
    bool set(const char* key, T value);

    /**
     * @brief Check if a parameter requires reboot after change.
     */
    bool requiresReboot(const char* key) const;

    /**
     * @brief Reset a parameter to its default value.
     * @return true if reset succeeded
     */
    bool reset(const char* key);

    /**
     * @brief Reset all parameters to defaults.
     */
    void resetAll();

    /**
     * @brief Subscribe to configuration changes.
     * @param pattern Key pattern (exact key or wildcard like "rate.roll.*")
     * @param callback Function to call on matching changes
     */
    void subscribe(const char* pattern, ConfigObserver callback);

    /**
     * @brief Check if any parameter is dirty (modified since last save).
     */
    bool hasDirty() const;

    /**
     * @brief Get list of dirty parameter keys (safe copies).
     * @return Vector of key strings (copies, not references)
     */
    std::vector<String> getDirtyKeys() const;

    /**
     * @brief Clear dirty flag for a parameter.
     */
    void clearDirty(const char* key);

    /**
     * @brief Clear dirty flags for all parameters.
     */
    void clearAllDirty();

    /**
     * @brief Mark a parameter as dirty.
     */
    void markDirty(const char* key);

    /**
     * @brief Get parameter descriptor (for introspection).
     * @return Copy of param, or std::nullopt if key not found
     * @note Returns a copy since the internal map may be modified concurrently.
     */
    std::optional<ConfigParam> getParam(const char* key) const;

    /**
     * @brief Get all registered parameters (thread-safe copy).
     * @return Copy of entire params map (safe to iterate outside lock)
     */
    std::unordered_map<std::string, ConfigParam> getAllParams() const;

    /**
     * @brief Get number of registered parameters.
     */
    size_t count() const;

    /**
     * @brief List all parameters matching a pattern.
     * @param pattern Key pattern (supports trailing wildcard "*")
     * @return Vector of copies of matching params (safe to use outside lock)
     */
    std::vector<ConfigParam> list(const char* pattern = "*") const;

    /**
     * @brief Set a parameter's current value directly (for persistence loading).
     *        Does NOT validate or notify - use only during load().
     */
    void setRaw(const char* key, ConfigValue value);

private:
    ConfigRegistry() = default;
    ConfigRegistry(const ConfigRegistry&) = delete;
    ConfigRegistry& operator=(const ConfigRegistry&) = delete;

    bool matchPattern(const char* pattern, const char* key) const;
    void notifyObservers(const ConfigChange& change);
    bool validate(const ConfigParam& param, const ConfigValue& value) const;
    
    /**
     * @brief Internal registration after init.
     */
    void registerParamInternal(
        const char* key,
        ConfigType  type,
        ConfigValue defaultVal,
        ConfigValue minVal,
        ConfigValue maxVal,
        const char* description,
        bool        requiresReboot
    );

    std::unordered_map<std::string, ConfigParam> _params;
    std::vector<std::pair<String, ConfigObserver>> _observers;
    mutable std::atomic<SemaphoreHandle_t> _mutex{nullptr};
    
    // Pending registration queue for params registered before FreeRTOS
    struct PendingRegistration {
        const char* key;
        ConfigType  type;
        ConfigValue defaultVal;
        ConfigValue minVal;
        ConfigValue maxVal;
        const char* description;
        bool        requiresReboot;
    };
    std::vector<PendingRegistration> _pendingRegistrations;
    bool _initialized = false;  // True after init() called

    void ensureMutex() const;
};

// ═══════════════════════════════════════════════════════════════════════════
// Template Specializations (declared here, defined in .cpp)
// ═══════════════════════════════════════════════════════════════════════════

template<> float    ConfigRegistry::get<float>(const char* key) const;
template<> int32_t  ConfigRegistry::get<int32_t>(const char* key) const;
template<> uint8_t  ConfigRegistry::get<uint8_t>(const char* key) const;
template<> bool     ConfigRegistry::get<bool>(const char* key) const;
template<> String   ConfigRegistry::get<String>(const char* key) const;

template<> bool ConfigRegistry::set<float>(const char* key, float value);
template<> bool ConfigRegistry::set<int32_t>(const char* key, int32_t value);
template<> bool ConfigRegistry::set<uint8_t>(const char* key, uint8_t value);
template<> bool ConfigRegistry::set<bool>(const char* key, bool value);
template<> bool ConfigRegistry::set<String>(const char* key, String value);

// ═══════════════════════════════════════════════════════════════════════════
// Convenience Macros for Static Registration
// ═══════════════════════════════════════════════════════════════════════════

/**
 * @brief Helper struct for static registration at startup.
 */
struct ConfigRegistrar {
    ConfigRegistrar(
        const char* key,
        ConfigType  type,
        ConfigValue defaultVal,
        ConfigValue minVal,
        ConfigValue maxVal,
        const char* description,
        bool        requiresReboot = false
    ) {
        ConfigRegistry::instance().registerParam(
            key, type, defaultVal, minVal, maxVal, description, requiresReboot
        );
    }
};

/**
 * @brief Helper macros for unique identifier generation.
 * __COUNTER__ needs indirection to expand before token pasting.
 */
#define CONFIG_CONCAT_IMPL(a, b) a##b
#define CONFIG_CONCAT(a, b) CONFIG_CONCAT_IMPL(a, b)
#define CONFIG_UNIQUE_NAME(prefix) CONFIG_CONCAT(prefix, __COUNTER__)

/**
 * @brief Register a float parameter.
 * @param key_macro    Key #define (e.g., CONFIG_KEY_RATE_ROLL_KP)
 * @param default_val  Default value
 * @param min_val      Minimum value
 * @param max_val      Maximum value
 * @param desc         Description string
 */
#define CONFIG_FLOAT(key_macro, default_val, min_val, max_val, desc) \
    static ConfigRegistrar CONFIG_UNIQUE_NAME(_cfg_)( \
        key_macro, ConfigType::FLOAT, \
        ConfigValue{(float)(default_val)}, \
        ConfigValue{(float)(min_val)}, \
        ConfigValue{(float)(max_val)}, \
        desc, false \
    )

/**
 * @brief Register an int32 parameter.
 */
#define CONFIG_INT(key_macro, default_val, min_val, max_val, desc) \
    static ConfigRegistrar CONFIG_UNIQUE_NAME(_cfg_)( \
        key_macro, ConfigType::INT32, \
        ConfigValue{(int32_t)(default_val)}, \
        ConfigValue{(int32_t)(min_val)}, \
        ConfigValue{(int32_t)(max_val)}, \
        desc, false \
    )

/**
 * @brief Register a uint8 parameter.
 */
#define CONFIG_UINT8(key_macro, default_val, min_val, max_val, desc) \
    static ConfigRegistrar CONFIG_UNIQUE_NAME(_cfg_)( \
        key_macro, ConfigType::UINT8, \
        ConfigValue{(uint8_t)(default_val)}, \
        ConfigValue{(uint8_t)(min_val)}, \
        ConfigValue{(uint8_t)(max_val)}, \
        desc, false \
    )

/**
 * @brief Register a boolean parameter.
 */
#define CONFIG_BOOL(key_macro, default_val, desc) \
    static ConfigRegistrar CONFIG_UNIQUE_NAME(_cfg_)( \
        key_macro, ConfigType::BOOL, \
        ConfigValue{(bool)(default_val)}, \
        ConfigValue{false}, \
        ConfigValue{true}, \
        desc, false \
    )

/**
 * @brief Register a string parameter.
 * @param key_macro    Key #define
 * @param default_val  Default string value (max CONFIG_STRING_MAX_LEN-1 chars)
 * @param desc         Description string
 */
#define CONFIG_STRING(key_macro, default_val, desc) \
    static ConfigRegistrar CONFIG_UNIQUE_NAME(_cfg_)( \
        key_macro, ConfigType::STRING, \
        ConfigValue{(const char*)(default_val)}, \
        ConfigValue{(const char*)""}, \
        ConfigValue{(const char*)""}, \
        desc, false \
    )

#endif // CONFIG_REGISTRY_H
