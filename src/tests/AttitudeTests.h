/**
 * AttitudeTests.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ATTITUDE_TESTS_H
#define ATTITUDE_TESTS_H

#include "src/controller/ArduFliteController.h"

/**
* @brief Runs a test sequence for the attitude controller
* That is meant to wiggle the wings.
* @param arduflite Reference to the ArduFliteController.
*/
void runAttitudeTest_wiggle(ArduFliteController &arduflite);

#endif //ATTITUDE_TESTS_H