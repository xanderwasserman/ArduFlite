/**
 * Mutex.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Satisfies the standard Lockable / TimedLockable requirements, so
 *        std::lock_guard, std::unique_lock and std::scoped_lock work directly.
 *        There is no bespoke lock wrapper — SemaphoreLock is retired.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_MUTEX_H
#define ARDUFLITE_HAL_PLATFORM_MUTEX_H

#include <chrono>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"

namespace arduflite::hal {

class Mutex : private NonCopyable
{
public:
    virtual ~Mutex() = default;

    /// Unbounded wait. Init paths only — never a control loop.
    virtual void lock() = 0;

    [[nodiscard]] virtual bool try_lock() = 0;

    virtual void unlock() noexcept = 0;

    /// Bounded wait. Templated on duration, so it cannot be virtual; forwards.
    template <class Rep, class Period>
    [[nodiscard]] bool try_lock_for(std::chrono::duration<Rep, Period> d)
    {
        return try_lock_for_us(
            std::chrono::duration_cast<std::chrono::microseconds>(d).count());
    }

protected:
    [[nodiscard]] virtual bool try_lock_for_us(std::int64_t us) = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_MUTEX_H
