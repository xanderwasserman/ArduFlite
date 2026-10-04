/**
 * Esp32Scheduler.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_ESP32_SCHEDULER_H
#define ARDUFLITE_HAL_ESP32_SCHEDULER_H

#include <array>

#include "src/hal/platform/Scheduler.h"

namespace arduflite::hal::esp32 {

class Esp32Task final : public Task
{
public:
    constexpr Esp32Task() noexcept = default;

    void requestStop() override { _stopRequested = true; }
    [[nodiscard]] bool stopRequested() const override { return _stopRequested; }
    [[nodiscard]] bool isRunning() const override { return _running; }

    /// Set by spawn(), cleared by the trampoline once the body returns. Tracking
    /// the HANDLE instead reported "running" forever, because the handle is
    /// assigned once and never cleared.
    volatile bool     _running       = false;
    volatile bool     _stopRequested = false;

    void*             _handle        = nullptr;   ///< TaskHandle_t, type-erased
    void            (*_entry)(void*) = nullptr;   ///< user body, called by the trampoline
    void*             _arg           = nullptr;
};

/**
 * @brief FreeRTOS task creation with the priority ladder as a type.
 *
 * Task objects live in a fixed array — ADR-011 forbids heap after boot.
 */
class Esp32Scheduler final : public Scheduler
{
public:
    static constexpr std::uint8_t kMaxTasks = 12;

    Result<Task*> spawn(const TaskConfig& cfg, void (*entry)(void*), void* arg) override;

    void sleepFor(std::chrono::milliseconds d) override;
    void sleepUntil(std::uint64_t& lastWakeUs, std::chrono::milliseconds period) override;
    void yield() override;

private:
    std::array<Esp32Task, kMaxTasks> _tasks{};
    std::uint8_t                     _taskCount = 0;
};

} // namespace arduflite::hal::esp32

#endif // ARDUFLITE_HAL_ESP32_SCHEDULER_H
