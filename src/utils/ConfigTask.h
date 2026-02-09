/**
 * ConfigTask.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Lightweight FreeRTOS task for periodic config persistence
 *        and background JSON import processing.
 */
#ifndef CONFIG_TASK_H
#define CONFIG_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

namespace ConfigTaskConfig {
    constexpr uint32_t SAVE_CHECK_INTERVAL_MS = 5000;   ///< Check for dirty params every 5s
    constexpr size_t   TASK_STACK_SIZE        = 4096;   ///< Stack size (needs room for JSON)
    constexpr UBaseType_t TASK_PRIORITY       = 1;      ///< Lowest priority
    constexpr size_t   IMPORT_QUEUE_SIZE      = 2;      ///< Max pending imports
    constexpr size_t   MAX_IMPORT_SIZE        = 8192;   ///< Max JSON import size
}

/**
 * @brief Background task for configuration management.
 *
 * Responsibilities:
 * - Periodically save dirty parameters to NVS
 * - Process queued JSON import requests
 * - Runs at lowest priority to avoid impacting control loops
 */
class ConfigTask {
public:
    /**
     * @brief Start the config task.
     *        Safe to call multiple times (only starts once).
     */
    static void start();

    /**
     * @brief Stop the config task.
     */
    static void stop();

    /**
     * @brief Queue a JSON string for import (non-blocking).
     *        The import is processed in the background task.
     * @param json JSON string to import (will be copied)
     * @return true if queued successfully, false if queue full
     */
    static bool queueImport(const String& json);

    /**
     * @brief Check if the config task is running.
     */
    static bool isRunning();

private:
    static void taskLoop(void* pvParameters);

    static TaskHandle_t  _taskHandle;
    static QueueHandle_t _importQueue;
    static SemaphoreHandle_t _taskExitedSem;  ///< Signals task has exited
    static std::atomic<bool> _running;
};

#endif // CONFIG_TASK_H
