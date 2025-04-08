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

#define PRINT_EVERY_N_UPDATES 50
#define LOOP_PERIOD_MILLIS    2  // 2 ms for 500 Hz

#define LEFT_AIL_PIN    17
#define RIGHT_AIL_PIN   16
#define PITCH_PIN       4
#define YAW_PIN         12

// Calibration Button definition
#define USER_BUTTON_PIN 27
#define CALIB_HOLD_TIME 3000

float getRollCmd();
float getPitchCmd();
float getYawCmd();

#endif // ARDUFLITE_H
