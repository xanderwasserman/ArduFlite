/**
 * ArduFlite.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 01 February 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#ifndef ARDUFLITE_H
#define ARDUFLITE_H

#include <Arduino.h>

// Hold time for the IMU calibration button
#define CALIB_HOLD_TIME         3000

// Default mutex timeout for control loops (5ms)
// This prevents indefinite blocking while still allowing for contention
#define MUTEX_TIMEOUT_MS        5

/**
 * @brief RAII wrapper for FreeRTOS mutex with configurable timeout.
 * 
 * Provides automatic mutex release on scope exit. Unlike portMAX_DELAY,
 * this uses a bounded timeout to prevent indefinite blocking in control loops.
 * 
 * Usage:
 *   {
 *       SemaphoreLock lock(myMutex);
 *       if (!lock.acquired()) {
 *           // Handle timeout - use last known values
 *           return;
 *       }
 *       // Safe to access protected data
 *   } // Automatically releases mutex
 * 
 * For indefinite wait (blocking operations), use portMAX_DELAY:
 *   SemaphoreLock lock(myMutex, portMAX_DELAY);
 */
struct SemaphoreLock 
{
    SemaphoreHandle_t h;
    bool _acquired;
    
    /**
     * @brief Construct and attempt to acquire mutex with timeout.
     * @param h_ Mutex handle
     * @param timeout Timeout in milliseconds, or portMAX_DELAY for indefinite wait.
     *                When portMAX_DELAY is passed, it's used directly as ticks.
     *                Otherwise the value is treated as milliseconds and converted.
     */
    SemaphoreLock(SemaphoreHandle_t h_, TickType_t timeout = MUTEX_TIMEOUT_MS)
        : h(h_), _acquired(false) 
    { 
        // Special handling: portMAX_DELAY is passed directly as ticks (infinite wait)
        // Other values are treated as milliseconds and converted to ticks
        TickType_t ticks = (timeout == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout);
        _acquired = (xSemaphoreTake(h, ticks) == pdTRUE);
    }
    
    ~SemaphoreLock() 
    { 
        if (_acquired) {
            xSemaphoreGive(h); 
        }
    }
    
    /**
     * @brief Check if the mutex was successfully acquired.
     * @return true if lock was acquired, false if timeout occurred
     */
    bool acquired() const { return _acquired; }
    
    // Prevent copying
    SemaphoreLock(const SemaphoreLock&) = delete;
    SemaphoreLock& operator=(const SemaphoreLock&) = delete;
};


void arduflite_init();
void arduflite_loop();

#endif // ARDUFLITE_H
