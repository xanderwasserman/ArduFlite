/**
 * Scheduler.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Task creation and sleeping, with the priority ladder as a TYPE rather
 *        than magic integers scattered across eight xTaskCreate calls.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_SCHEDULER_H
#define ARDUFLITE_HAL_PLATFORM_SCHEDULER_H

#include <chrono>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Result.h"

namespace arduflite::hal {

/**
 * @brief The whole system's priority ladder, in one place.
 *
 * @note Duplicate values are intentional and correct — those tasks really do
 *       share a FreeRTOS priority. Do not "fix" this by spreading them out.
 */
enum class Priority : std::uint8_t
{
    Cli        = 0,
    Web        = 1,
    Telemetry  = 1,
    OuterLoop  = 2,
    RcLink     = 3,
    InnerLoop  = 3,
    Inertial   = 4,
};

struct TaskConfig
{
    const char*   name       = "task";
    std::uint32_t stackBytes = 4096;
    Priority      priority   = Priority::Telemetry;
    std::int8_t   core       = -1;      ///< -1 = no affinity
};

class Task : private NonCopyable
{
public:
    virtual ~Task() = default;
    virtual void requestStop() = 0;
    [[nodiscard]] virtual bool isRunning() const = 0;
};

class Scheduler : private NonCopyable
{
public:
    virtual ~Scheduler() = default;

    virtual Result<Task*> spawn(const TaskConfig& cfg, void (*entry)(void*), void* arg) = 0;

    virtual void sleepFor(std::chrono::milliseconds d) = 0;

    /// Fixed-cadence sleep (vTaskDelayUntil). `lastWake` is opaque scheduler state.
    virtual void sleepUntil(std::uint64_t& lastWakeUs, std::chrono::milliseconds period) = 0;

    virtual void yield() = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_SCHEDULER_H
