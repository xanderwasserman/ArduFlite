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

// Hold time for the IMU calibration button, in milliseconds.
#define CALIB_HOLD_TIME         3000


void arduflite_init();
void arduflite_loop();

#endif // ARDUFLITE_H
