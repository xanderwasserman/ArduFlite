/**
 * Esp32Clock.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Clock.h"

#include <esp_timer.h>

namespace arduflite::hal::esp32 {

Clock::time_point Esp32Clock::now() const noexcept
{
    // esp_timer_get_time() returns int64_t microseconds since boot — no wrap.
    return time_point{ duration{ esp_timer_get_time() } };
}

} // namespace arduflite::hal::esp32
