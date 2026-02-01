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
 */
struct SemaphoreLock 
{
    SemaphoreHandle_t h;
    bool _acquired;
    
    /**
     * @brief Construct and attempt to acquire mutex with timeout.
     * @param h_ Mutex handle
     * @param timeoutMs Maximum time to wait (default: MUTEX_TIMEOUT_MS)
     */
    SemaphoreLock(SemaphoreHandle_t h_, TickType_t timeoutMs = MUTEX_TIMEOUT_MS)
        : h(h_), _acquired(false) 
    { 
        _acquired = (xSemaphoreTake(h, pdMS_TO_TICKS(timeoutMs)) == pdTRUE);
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
