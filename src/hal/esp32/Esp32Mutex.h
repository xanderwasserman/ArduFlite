/**
 * Esp32Mutex.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_ESP32_MUTEX_H
#define ARDUFLITE_HAL_ESP32_MUTEX_H

#include "src/hal/core/Status.h"
#include "src/hal/platform/Mutex.h"

namespace arduflite::hal::esp32 {

/**
 * @brief FreeRTOS recursive-free mutex behind the standard Lockable interface.
 *
 * Because this satisfies Lockable, callers use std::unique_lock / std::lock_guard
 * / std::scoped_lock directly.
 */
class Esp32Mutex final : public Mutex
{
public:
    /// @note Trivially constructible ON PURPOSE. Creating a FreeRTOS semaphore in
    ///       a static constructor runs before the scheduler exists and before
    ///       main() — precisely the static-initialisation hazard `constinit` on
    ///       BoardStorage is there to forbid. The semaphore is created in begin().
    constexpr Esp32Mutex() noexcept = default;
    ~Esp32Mutex() override;

    /// Create the underlying semaphore. Idempotent.
    Status begin() noexcept;

    /// @return true once begin() has succeeded. Until then every lock attempt
    ///         fails closed rather than silently succeeding.
    [[nodiscard]] bool valid() const noexcept { return _handle != nullptr; }

    void lock() override;
    [[nodiscard]] bool try_lock() override;
    void unlock() noexcept override;

protected:
    [[nodiscard]] bool try_lock_for_us(std::int64_t us) override;

private:
    void* _handle = nullptr;   ///< SemaphoreHandle_t, type-erased to keep FreeRTOS out of this header
};

/**
 * @brief Recursive variant, for bus locks.
 *
 * RegisterDevice::busLock() must be recursive: a driver grouping several
 * transactions holds it while each transaction also locks internally. With a
 * plain mutex that is an immediate self-deadlock.
 */
class Esp32RecursiveMutex final : public Mutex
{
public:
    constexpr Esp32RecursiveMutex() noexcept = default;
    ~Esp32RecursiveMutex() override;

    Status begin() noexcept;

    [[nodiscard]] bool valid() const noexcept { return _handle != nullptr; }

    void lock() override;
    [[nodiscard]] bool try_lock() override;
    void unlock() noexcept override;

protected:
    [[nodiscard]] bool try_lock_for_us(std::int64_t us) override;

private:
    void* _handle = nullptr;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_MUTEX_H
