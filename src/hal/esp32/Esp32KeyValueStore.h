/**
 * Esp32KeyValueStore.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief KeyValueStore over ESP32 NVS, via the Arduino Preferences shim.
 *
 * Exists so ConfigPersistence stops including <Preferences.h> directly
 * (ADR-027). That header is Arduino-ESP32 specific and does not travel even to
 * another FreeRTOS target, which made the configuration system — one of the
 * least hardware-dependent parts of the firmware — one of the least portable.
 *
 * @note Byte-oriented: a caller that wants a float serialises it. A typed API
 *       would need fifteen methods differing only in the size they write, and
 *       every one reimplemented on any other platform.
 */
#ifndef ARDUFLITE_HAL_ESP32_KEY_VALUE_STORE_H
#define ARDUFLITE_HAL_ESP32_KEY_VALUE_STORE_H

#include "src/hal/platform/Storage.h"

namespace arduflite::hal::esp32 {

class Esp32KeyValueStore final : public KeyValueStore
{
public:
    /// @param namespaceName NVS namespace, max 15 characters.
    ///
    /// @note constexpr: this lives in BoardStorage, which is constinit. The
    ///       Preferences object is opened per operation rather than held, which
    ///       is what keeps this trivially constructible and safe to call from
    ///       any task.
    constexpr explicit Esp32KeyValueStore(const char* namespaceName) noexcept
        : _namespace(namespaceName) {}

    Status begin() override;
    Status read (const char* key, void* dst, std::size_t capacity, std::size_t& outLen) override;
    Status write(const char* key, const void* src, std::size_t len) override;
    Status erase(const char* key) override;
    Status eraseAll() override;
    Status commit() override;

private:
    const char* _namespace;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_KEY_VALUE_STORE_H
