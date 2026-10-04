/**
 * Esp32SettingsStore.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief SettingsStore over ESP32 NVS, with a CRC on every blob.
 *
 * Calibration data, keyed and CRC-protected. Two properties matter:
 *
 * 1. **Wear levelling.** NVS manages its own, rather than rewriting a whole
 *    flash page on every commit.
 *
 * 2. **A CRC**, which is the one that matters. A magic number alone detects
 *    "never written" but not "written and later corrupted", and a single
 *    flipped bit in a stored offset would be applied as a calibration —
 *    quietly biasing the aircraft's idea of level with nothing in the log to
 *    say so.
 */
#ifndef ARDUFLITE_HAL_ESP32_SETTINGS_STORE_H
#define ARDUFLITE_HAL_ESP32_SETTINGS_STORE_H

#include "src/hal/device/Peripherals.h"

namespace arduflite::hal::esp32 {

class Esp32SettingsStore final : public device::SettingsStore
{
public:
    /// @param namespaceName NVS namespace. Max 15 characters, an NVS limit.
    ///
    /// @note constexpr because this lives in BoardStorage, which is `constinit`.
    ///       Any dynamic initialisation here is a compile error — deliberately,
    ///       since it would run before main() in an unspecified order relative
    ///       to everything else. Preferences is opened per call rather than
    ///       held open, which keeps this trivially constructible.
    constexpr explicit Esp32SettingsStore(const char* namespaceName) noexcept
        : _namespace(namespaceName) {}

    Status load (const char* key, void* dst, std::size_t len) override;
    Status save (const char* key, const void* src, std::size_t len) override;
    Status erase(const char* key) override;

private:
    const char* _namespace;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_SETTINGS_STORE_H
