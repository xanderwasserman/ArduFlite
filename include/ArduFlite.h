/**
 * ArduFlite.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#ifndef ARDUFLITE_H
#define ARDUFLITE_H

#include <Arduino.h>

// Hold time for the IMU calibration button
#define CALIB_HOLD_TIME         3000

struct SemaphoreLock 
{
  SemaphoreHandle_t h;
  SemaphoreLock(SemaphoreHandle_t h_): h(h_) { xSemaphoreTake(h, portMAX_DELAY); }
  ~SemaphoreLock()            { xSemaphoreGive(h); }
};


void arduflite_init();
void arduflite_loop();

#endif // ARDUFLITE_H
