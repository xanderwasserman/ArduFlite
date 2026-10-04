/**
 * Esp32Clock.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @note No Arduino headers here — they live in the .cpp. Nothing that includes
 *       this file should acquire a dependency on the Arduino core.
 */
#ifndef ARDUFLITE_HAL_ESP32_CLOCK_H
#define ARDUFLITE_HAL_ESP32_CLOCK_H

#include "src/hal/platform/Clock.h"

namespace arduflite::hal::esp32 {

/**
 * @brief Monotonic microsecond clock backed by esp_timer_get_time().
 *
 * esp_timer_get_time() is genuinely 64-bit, unlike Arduino's micros(), which is
 * a 32-bit truncation and wraps every ~71 minutes.
 */
class Esp32Clock final : public Clock
{
public:
    [[nodiscard]] time_point now() const noexcept override;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_CLOCK_H
