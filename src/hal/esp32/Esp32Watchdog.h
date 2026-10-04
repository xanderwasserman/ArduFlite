/**
 * Esp32Watchdog.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_ESP32_WATCHDOG_H
#define ARDUFLITE_HAL_ESP32_WATCHDOG_H

#include <cstdint>

#include "src/hal/platform/Watchdog.h"

namespace arduflite::hal::esp32 {

class Esp32Watchdog final : public Watchdog
{
public:
    /// Configure the task watchdog. Idempotent: reconfigures if already running,
    /// which the existing ArdufliteApp init already has to handle.
    Status begin(std::uint32_t timeout_ms, bool panicOnTimeout = true);

    Status registerCurrentTask() override;
    void   feed() noexcept override;
    Status unregisterCurrentTask() override;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_WATCHDOG_H
