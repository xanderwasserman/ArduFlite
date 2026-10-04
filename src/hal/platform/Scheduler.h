/**
 * Scheduler.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Task creation and sleeping, with the priority ladder as a TYPE rather
 *        than magic integers scattered across multiple xTaskCreate calls.
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
 * @warning If you change a value here, you are changing flight behaviour. Do it
 *          deliberately, in its own change, with a bench measurement.
 *
 * @note Duplicate values are intentional and correct — those tasks really do
 *       share a FreeRTOS priority. Do not "fix" this by spreading them out.
 */
enum class Priority : std::uint8_t
{
    // 1 — background. Everything that must never preempt control.
    Cli        = 1,   ///< ArduFliteCLI
    Web        = 1,   ///< ArduFliteWebServer
    Config     = 1,   ///< ConfigTask
    Telemetry  = 1,   ///< flash, CRSF and both serial telemetry backends
    Mission    = 1,   ///< MissionPlanner
    Indicator  = 1,   ///< NeoPixelIndicator

    // 2 — outer control and RC input.
    OuterLoop  = 2,   ///< ArduFliteController, attitude loop
    RcLink     = 2,   ///< CrsfLink

    // 3-4 — inner control and sensing.
    InnerLoop  = 3,   ///< ArduFliteController, rate loop
    Inertial   = 4,   ///< InertialSubsystem
};

struct TaskConfig
{
    const char*   name       = "task";
    std::uint32_t stackBytes = 4096;
    Priority      priority   = Priority::Telemetry;
    std::int8_t   core       = -1;      ///< -1 = no affinity
};

/**
 * @brief A spawned task, and the only handle anyone outside the HAL gets.
 *
 * Stopping is **cooperative**: requestStop() asks, and the task body is what
 * decides when it is safe to leave. There is deliberately no preemptive kill.
 * FreeRTOS offers one — vTaskDelete() — and it is the wrong primitive here: it
 * can strike inside a critical section, with a mutex held or a half-written log
 * record, and it cannot be expressed on a host at all.
 *
 * The cost is that a task body which never checks stopRequested() will never
 * stop. That is the contract, not an oversight, and it is why the accessor
 * exists rather than the flag being private scheduler state.
 */
class Task : private NonCopyable
{
public:
    virtual ~Task() = default;

    /// Ask the task to leave its loop at the next safe point.
    virtual void requestStop() = 0;

    /**
     * @brief Has a stop been asked for? Called BY the task body, every loop.
     *
     * A body that does not poll this never stops, and requestStop() becomes a
     * silent no-op. That is the contract, not an oversight.
     */
    [[nodiscard]] virtual bool stopRequested() const = 0;

    /// False once the body has returned. Not merely "was spawned".
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
