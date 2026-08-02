/**
 * Clock.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Monotonic time source. std::chrono, not raw integers: it removes the
 *        32-bit micros() wrap and the "is this ms or us" ambiguity at zero cost.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_CLOCK_H
#define ARDUFLITE_HAL_PLATFORM_CLOCK_H

#include <chrono>

#include "src/hal/core/NonCopyable.h"

namespace arduflite::hal {

class Clock : private NonCopyable
{
public:
    using duration   = std::chrono::microseconds;   ///< 64-bit: no 71-minute wrap
    using rep        = duration::rep;
    using period     = duration::period;
    using time_point = std::chrono::time_point<Clock, duration>;

    static constexpr bool is_steady = true;

    virtual ~Clock() = default;

    [[nodiscard]] virtual time_point now() const noexcept = 0;
};

/// Seconds as a float — what the estimator and PID loops actually want.
[[nodiscard]] inline float toSeconds(Clock::duration d) noexcept
{
    return std::chrono::duration<float>(d).count();
}

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_CLOCK_H
