/**
 * ConfigTask.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ConfigTask.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/Logging.h"

// Static member initialization
TaskHandle_t  ConfigTask::_taskHandle  = nullptr;
QueueHandle_t ConfigTask::_importQueue = nullptr;
SemaphoreHandle_t ConfigTask::_taskExitedSem = nullptr;
std::atomic<bool> ConfigTask::_running{false};

// ═══════════════════════════════════════════════════════════════════════════
// Task Management
// ═══════════════════════════════════════════════════════════════════════════

void ConfigTask::start() {
    if (_running) return;

    // Create import queue
    _importQueue = xQueueCreate(
        ConfigTaskConfig::IMPORT_QUEUE_SIZE,
        sizeof(char*) // Queue holds pointers to heap-allocated strings
    );

    if (!_importQueue) {
        LOG_ERR("Failed to create config import queue");
        return;
    }

    // Create exit semaphore (binary semaphore, starts empty)
    _taskExitedSem = xSemaphoreCreateBinary();
    if (!_taskExitedSem) {
        LOG_ERR("Failed to create config task exit semaphore");
        vQueueDelete(_importQueue);
        _importQueue = nullptr;
        return;
    }

    // Set running BEFORE creating task to prevent immediate exit
    _running = true;

    // Create task
    BaseType_t result = xTaskCreate(
        taskLoop,
        "ConfigTask",
        ConfigTaskConfig::TASK_STACK_SIZE,
        nullptr,
        ConfigTaskConfig::TASK_PRIORITY,
        &_taskHandle
    );

    if (result != pdPASS) {
        LOG_ERR("Failed to create ConfigTask");
        _running = false;
        vSemaphoreDelete(_taskExitedSem);
        _taskExitedSem = nullptr;
        vQueueDelete(_importQueue);
        _importQueue = nullptr;
        return;
    }

    LOG_INF("ConfigTask started");
}

void ConfigTask::stop() {
    if (!_running) return;

    // Signal the task to stop
    _running = false;

    // Wait for task to signal exit via semaphore (with timeout)
    if (_taskExitedSem) {
        if (xSemaphoreTake(_taskExitedSem, pdMS_TO_TICKS(1000)) != pdTRUE) {
            LOG_WARN("ConfigTask did not exit gracefully within 1s");
        }
        vSemaphoreDelete(_taskExitedSem);
        _taskExitedSem = nullptr;
    }
    _taskHandle = nullptr;

    if (_importQueue) {
        // Clean up any pending imports
        char* pendingJson = nullptr;
        while (xQueueReceive(_importQueue, &pendingJson, 0) == pdTRUE) {
            if (pendingJson) {
                free(pendingJson);
            }
        }
        vQueueDelete(_importQueue);
        _importQueue = nullptr;
    }

    LOG_INF("ConfigTask stopped");
}

bool ConfigTask::isRunning() {
    return _running;
}

// ═══════════════════════════════════════════════════════════════════════════
// Import Queue
// ═══════════════════════════════════════════════════════════════════════════

bool ConfigTask::queueImport(const String& json) {
    if (!_running || !_importQueue) {
        return false;
    }

    if (json.length() > ConfigTaskConfig::MAX_IMPORT_SIZE) {
        LOG_ERR("JSON too large for import: %u > %u bytes",
                json.length(), ConfigTaskConfig::MAX_IMPORT_SIZE);
        return false;
    }

    // Allocate copy on heap (freed by task after processing)
    char* jsonCopy = (char*)malloc(json.length() + 1);
    if (!jsonCopy) {
        LOG_ERR("Failed to allocate memory for JSON import");
        return false;
    }
    strcpy(jsonCopy, json.c_str());

    // Queue the pointer (non-blocking)
    if (xQueueSend(_importQueue, &jsonCopy, 0) != pdTRUE) {
        free(jsonCopy);
        LOG_WARN("Config import queue full");
        return false;
    }

    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// Task Loop
// ═══════════════════════════════════════════════════════════════════════════

void ConfigTask::taskLoop(void* pvParameters) {
    (void)pvParameters;

    TickType_t lastSaveCheck = xTaskGetTickCount();
    const TickType_t saveInterval = pdMS_TO_TICKS(ConfigTaskConfig::SAVE_CHECK_INTERVAL_MS);
    const TickType_t queueTimeout = pdMS_TO_TICKS(100);

    while (_running) {
        // Check for pending imports (with short timeout - this IS the yield)
        char* jsonToImport = nullptr;
        if (_importQueue && xQueueReceive(_importQueue, &jsonToImport, queueTimeout) == pdTRUE) {
            if (jsonToImport) {
                LOG_INF("Processing queued config import...");
                
                // Import new config
                size_t imported = ConfigPersistence::importJson(String(jsonToImport));
                
                // Free the queued string
                free(jsonToImport);
                
                if (imported > 0) {
                    // Save imported config to NVS
                    ConfigPersistence::saveIfDirty();
                }
            }
        }

        // Periodic dirty-save check
        TickType_t now = xTaskGetTickCount();
        if ((now - lastSaveCheck) >= saveInterval) {
            lastSaveCheck = now;

            if (ConfigRegistry::instance().hasDirty()) {
                ConfigPersistence::saveIfDirty();
            }
        }
        // No additional vTaskDelay needed - xQueueReceive already provides the yield
    }
    
    // Signal that task is exiting, then delete ourselves
    if (_taskExitedSem) {
        xSemaphoreGive(_taskExitedSem);
    }
    vTaskDelete(nullptr);  // Delete current task
}
