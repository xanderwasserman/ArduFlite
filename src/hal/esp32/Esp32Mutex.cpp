/**
 * Esp32Mutex.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Mutex.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace arduflite::hal::esp32 {

namespace {
inline SemaphoreHandle_t handleOf(void* h) { return static_cast<SemaphoreHandle_t>(h); }
} // namespace

Status Esp32Mutex::begin() noexcept
{
    if (_handle != nullptr) { return Status::Ok; }
    _handle = xSemaphoreCreateMutex();
    return (_handle != nullptr) ? Status::Ok : Status::NoSpace;
}

Esp32Mutex::~Esp32Mutex()
{
    if (_handle != nullptr)
    {
        vSemaphoreDelete(handleOf(_handle));
        _handle = nullptr;
    }
}

void Esp32Mutex::lock()
{
    if (_handle == nullptr) { return; }
    (void)xSemaphoreTake(handleOf(_handle), portMAX_DELAY);
}

bool Esp32Mutex::try_lock()
{
    if (_handle == nullptr) { return false; }
    return xSemaphoreTake(handleOf(_handle), 0) == pdTRUE;
}

void Esp32Mutex::unlock() noexcept
{
    if (_handle == nullptr) { return; }
    (void)xSemaphoreGive(handleOf(_handle));
}

bool Esp32Mutex::try_lock_for_us(std::int64_t us)
{
    if (_handle == nullptr) { return false; }
    if (us <= 0)            { return try_lock(); }

    // Round up to whole ticks: a sub-tick timeout must not silently become 0,
    // which would turn a bounded wait into a non-blocking poll.
    const std::int64_t usPerTick = 1000000 / configTICK_RATE_HZ;
    TickType_t ticks = static_cast<TickType_t>((us + usPerTick - 1) / usPerTick);
    if (ticks == 0) { ticks = 1; }

    return xSemaphoreTake(handleOf(_handle), ticks) == pdTRUE;
}

// ── Esp32RecursiveMutex ─────────────────────────────────────────────────────

Status Esp32RecursiveMutex::begin() noexcept
{
    if (_handle != nullptr) { return Status::Ok; }
    _handle = xSemaphoreCreateRecursiveMutex();
    return (_handle != nullptr) ? Status::Ok : Status::NoSpace;
}

Esp32RecursiveMutex::~Esp32RecursiveMutex()
{
    if (_handle != nullptr)
    {
        vSemaphoreDelete(handleOf(_handle));
        _handle = nullptr;
    }
}

void Esp32RecursiveMutex::lock()
{
    if (_handle == nullptr) { return; }
    (void)xSemaphoreTakeRecursive(handleOf(_handle), portMAX_DELAY);
}

bool Esp32RecursiveMutex::try_lock()
{
    if (_handle == nullptr) { return false; }
    return xSemaphoreTakeRecursive(handleOf(_handle), 0) == pdTRUE;
}

void Esp32RecursiveMutex::unlock() noexcept
{
    if (_handle == nullptr) { return; }
    (void)xSemaphoreGiveRecursive(handleOf(_handle));
}

bool Esp32RecursiveMutex::try_lock_for_us(std::int64_t us)
{
    if (_handle == nullptr) { return false; }
    if (us <= 0)            { return try_lock(); }

    const std::int64_t usPerTick = 1000000 / configTICK_RATE_HZ;
    TickType_t ticks = static_cast<TickType_t>((us + usPerTick - 1) / usPerTick);
    if (ticks == 0) { ticks = 1; }

    return xSemaphoreTakeRecursive(handleOf(_handle), ticks) == pdTRUE;
}

} // namespace arduflite::hal::esp32
