/**
 * Esp32Scheduler.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Scheduler.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace arduflite::hal::esp32 {

namespace {

/**
 * @brief Wraps every task body so that returning from it is legal.
 *
 * A FreeRTOS task function that simply returns takes the whole system down —
 * the scheduler expects it to call vTaskDelete(NULL) instead. Routing every
 * body through here means a task can end by leaving its loop, which is what
 * makes cooperative stopping usable, and it is also where isRunning() becomes
 * truthful.
 */
void taskTrampoline(void* slotPointer)
{
    auto* slot = static_cast<Esp32Task*>(slotPointer);

    slot->_entry(slot->_arg);

    slot->_running = false;
    slot->_handle  = nullptr;
    vTaskDelete(nullptr);
}

} // namespace

Result<Task*> Esp32Scheduler::spawn(const TaskConfig& cfg, void (*entry)(void*), void* arg)
{
    if (entry == nullptr)          { return Status::InvalidArg; }
    if (_taskCount >= kMaxTasks)   { return Status::NoSpace; }

    Esp32Task& slot = _tasks[_taskCount];
    slot._entry = entry;
    slot._arg   = arg;

    // Set BEFORE the task can run: the body may finish before xTaskCreate even
    // returns, and clearing a flag that was never set would leave isRunning()
    // stuck true for the rest of the flight.
    slot._running = true;

    // FreeRTOS wants the stack depth in WORDS; TaskConfig carries BYTES, because
    // TaskConfig carries BYTES, which is what every call site writes.
    const std::uint32_t depthWords = cfg.stackBytes / sizeof(StackType_t);

    TaskHandle_t handle = nullptr;
    BaseType_t   ok;

    if (cfg.core < 0)
    {
        ok = xTaskCreate(&taskTrampoline, cfg.name, depthWords, &slot,
                         static_cast<UBaseType_t>(cfg.priority), &handle);
    }
    else
    {
        ok = xTaskCreatePinnedToCore(&taskTrampoline, cfg.name, depthWords, &slot,
                                     static_cast<UBaseType_t>(cfg.priority), &handle,
                                     static_cast<BaseType_t>(cfg.core));
    }

    if (ok != pdPASS)
    {
        slot._running = false;
        return Status::NoSpace;
    }

    slot._handle = handle;
    ++_taskCount;
    return static_cast<Task*>(&slot);
}

void Esp32Scheduler::sleepFor(std::chrono::milliseconds d)
{
    vTaskDelay(pdMS_TO_TICKS(d.count()));
}

void Esp32Scheduler::sleepUntil(std::uint64_t& lastWakeUs, std::chrono::milliseconds period)
{
    // lastWakeUs is opaque scheduler state; here it holds a TickType_t.
    TickType_t lastWake = static_cast<TickType_t>(lastWakeUs);
    if (lastWake == 0)
    {
        lastWake = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(period.count()));
    lastWakeUs = static_cast<std::uint64_t>(lastWake);
}

void Esp32Scheduler::yield()
{
    taskYIELD();
}

} // namespace arduflite::hal::esp32
