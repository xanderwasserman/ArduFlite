/**
 * ControlLoopTests.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROL_LOOP_TESTS_H
#define CONTROL_LOOP_TESTS_H

#include "src/controller/ArduFliteController.h"

/**
 * @brief Regression test for outer/inner loop dt computation.
 *
 * Reads `getOuterLoopStats()` and `getInnerLoopStats()` from the running
 * controller and logs PASS/FAIL against expected periods (10 ms outer,
 * 2 ms inner). Intended to be called once, ~5 seconds after startup,
 * to ensure enough samples have accumulated.
 *
 * @param ctrl Reference to the running ArduFliteController.
 */
void runControlLoopTest_dtComputation(ArduFliteController &ctrl);

#endif // CONTROL_LOOP_TESTS_H
