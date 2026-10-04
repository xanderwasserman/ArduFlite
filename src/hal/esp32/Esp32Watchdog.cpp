/**
 * Esp32Watchdog.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32Watchdog.h"

#include <esp_task_wdt.h>

namespace arduflite::hal::esp32 {

Status Esp32Watchdog::begin(std::uint32_t timeout_ms, bool panicOnTimeout)
{
    esp_task_wdt_config_t cfg{};
    cfg.timeout_ms     = timeout_ms;
    cfg.idle_core_mask = 0;              // do not watch idle tasks
    cfg.trigger_panic  = panicOnTimeout;

    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE)
    {
        // Already initialised — the Arduino core may have started it.
        err = esp_task_wdt_reconfigure(&cfg);
    }
    return (err == ESP_OK) ? Status::Ok : Status::IoError;
}

Status Esp32Watchdog::registerCurrentTask()
{
    const esp_err_t err = esp_task_wdt_add(nullptr);   // nullptr = current task
    // Already-registered is not a failure: WatchdogGuard may nest during init.
    if (err == ESP_OK || err == ESP_ERR_INVALID_ARG) { return Status::Ok; }
    return Status::IoError;
}

void Esp32Watchdog::feed() noexcept
{
    (void)esp_task_wdt_reset();
}

Status Esp32Watchdog::unregisterCurrentTask()
{
    const esp_err_t err = esp_task_wdt_delete(nullptr);
    if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) { return Status::Ok; }
    return Status::IoError;
}

} // namespace arduflite::hal::esp32
