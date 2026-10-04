/**
 * Esp32System.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/esp32/Esp32System.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_random.h>

#include <Arduino.h>
#include <esp_system.h>

#include <cstdio>

namespace arduflite::hal::esp32 {

ResetCause Esp32System::resetCause() const
{
    switch (esp_reset_reason())
    {
        case ESP_RST_POWERON:  return ResetCause::PowerOn;
        case ESP_RST_SW:       return ResetCause::Software;
        case ESP_RST_PANIC:    return ResetCause::Panic;
        case ESP_RST_TASK_WDT:
        case ESP_RST_INT_WDT:
        case ESP_RST_WDT:      return ResetCause::Watchdog;
        case ESP_RST_BROWNOUT: return ResetCause::Brownout;
        default:               return ResetCause::Unknown;
    }
}

std::uint32_t Esp32System::freeHeapBytes()    const { return ESP.getFreeHeap(); }
std::uint32_t Esp32System::minFreeHeapBytes() const { return ESP.getMinFreeHeap(); }

const char* Esp32System::uniqueId() const
{
    if (_id[0] == '\0')
    {
        const std::uint64_t mac = ESP.getEfuseMac();
        std::snprintf(_id, sizeof(_id), "%012llX", static_cast<unsigned long long>(mac));
    }
    return _id;
}

const char* Esp32System::platformName() const
{
    return ESP.getChipModel();
}

const char* Esp32System::sdkVersion() const
{
    return ESP.getSdkVersion();
}

std::uint32_t Esp32System::randomWord()
{
    // esp_random() draws from the hardware RNG. It is only truly random once
    // RF is running (WiFi or BT); before that the IDF documents it as
    // pseudo-random. That is acceptable for the one caller — the web server
    // generates its CSRF token in begin(), which runs after WiFi is up.
    return static_cast<std::uint32_t>(esp_random());
}

Status Esp32System::taskReport(char* buffer, std::size_t capacity) const
{
    if (buffer == nullptr || capacity == 0) { return Status::InvalidArg; }

    // vTaskList() takes no length and writes roughly 40-50 bytes per task, so
    // the ONLY defence against overrunning the caller's buffer is refusing when
    // the task count says it will not fit. 64 is deliberately above the
    // observed per-task width.
    constexpr std::size_t kBytesPerTask = 64;

    const std::size_t taskCount = uxTaskGetNumberOfTasks();
    if ((taskCount + 1) * kBytesPerTask > capacity)
    {
        buffer[0] = '\0';
        return Status::NoSpace;
    }

    vTaskList(buffer);
    buffer[capacity - 1] = '\0';
    return Status::Ok;
}

void Esp32System::reboot()
{
    ESP.restart();
    for (;;) { }   // ESP.restart() does not return; satisfies [[noreturn]]
}

} // namespace arduflite::hal::esp32
