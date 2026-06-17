/**
 * BaroTests.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef BARO_TESTS_H
#define BARO_TESTS_H

#include "src/orientation/ArduFliteIMU.h"

/**
 * @brief Field-safe regression test for the inline barometer + lock-free snapshot.
 *
 * Verifies, against the running IMU:
 *  - the boot seed produced no false climb-rate spike (|climbRate| ≈ 0 on the ground),
 *  - altitude and climb rate are finite (no NaN/Inf leaking from the baro read),
 *  - the versioned snapshot stays coherent under the live 500 Hz writer (unit-norm quat),
 *  - snapshot read health is clean (no retry-limit fallbacks).
 *
 * Read-only and actuator-free. Call once, ~5 s after boot (post filter warm-up and
 * baro seed) while the aircraft is STATIONARY on the ground.
 *
 * @param imu Reference to the running ArduFliteIMU.
 */
void runBaroTest_seedAndSnapshotHealth(ArduFliteIMU &imu);

#endif // BARO_TESTS_H
