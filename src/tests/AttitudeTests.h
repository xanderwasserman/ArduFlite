/**
 * AttitudeTests.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ATTITUDE_TESTS_H
#define ATTITUDE_TESTS_H

#include "src/controller/ArduFliteController.h"

/**
* @brief Runs a time-sliced test sequence that wiggles the wings.
* Must be called repeatedly from the main loop (it uses millis() internally).
* @param arduflite Reference to the ArduFliteController.
* @param angle     Peak roll angle in degrees (default 20°).
* @param time      Duration of each half-step in seconds (default 1 s).
*/
void runAttitudeTest_wiggle(ArduFliteController &arduflite, float angle = 20.0f, float time = 1.0f);

#endif //ATTITUDE_TESTS_H