/**
 * ConfigTask.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 06 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ConfigTask.h"

#include <chrono>

#include "src/hal/board/Board.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/Logging.h"

// Static member initialization
arduflite::hal::Task* ConfigTask::_task = nullptr;
QueueHandle_t ConfigTask::_importQueue = nullptr;
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

    // Set running BEFORE creating the task, or it exits on its first check.
    _running = true;

    arduflite::hal::TaskConfig config;
    config.name       = "ConfigTask";
    config.stackBytes = ConfigTaskConfig::TASK_STACK_SIZE;
    config.priority   = arduflite::hal::Priority::Config;

    auto task = arduflite::board::Board::instance().scheduler().spawn(
        config, &taskLoop, nullptr);
    if (!task) {
        LOG_ERR("Failed to create ConfigTask");
        _running = false;
        vQueueDelete(_importQueue);
        _importQueue = nullptr;
        return;
    }
    _task = task.value();

    LOG_INF("ConfigTask started");
}

void ConfigTask::stop() {
    if (!_running) return;

    // Signal the task to stop
    _running = false;

    // Join by polling isRunning(), which the scheduler clears once the body
    // returns. The task blocks up to 100 ms in xQueueReceive, so it can take
    // that long to notice _running went false; 1 s of headroom covers it.
    if (_task != nullptr) {
        auto& board = arduflite::board::Board::instance();
        const auto deadline = board.clock().now() + std::chrono::seconds{ 1 };

        while (_task->isRunning() && board.clock().now() < deadline) {
            board.scheduler().sleepFor(std::chrono::milliseconds{ 10 });
        }
        if (_task->isRunning()) {
            LOG_WARN("ConfigTask did not exit gracefully within 1s");
        }
        _task = nullptr;
    }

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

    const auto& clock = arduflite::board::Board::instance().clock();

    auto lastSaveCheck = clock.now();
    const auto saveInterval =
        std::chrono::milliseconds{ ConfigTaskConfig::SAVE_CHECK_INTERVAL_MS };
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
        const auto now = clock.now();
        if ((now - lastSaveCheck) >= saveInterval) {
            lastSaveCheck = now;

            if (ConfigRegistry::instance().hasDirty()) {
                ConfigPersistence::saveIfDirty();
            }
        }
        // No additional vTaskDelay needed - xQueueReceive already provides the yield
    }
    
    // Just return. The scheduler's trampoline is what ends the FreeRTOS task
    // and clears isRunning(), which is how stop() observes the exit (ADR-058).
}
