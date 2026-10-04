/**
 * ImuState.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief What the estimation layer publishes to every other task.
 */
#ifndef ARDUFLITE_ESTIMATION_IMU_STATE_H
#define ARDUFLITE_ESTIMATION_IMU_STATE_H

#include <cstdint>

#include "src/hal/core/Vec3.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::estimation {

/// Debounced motion events derived from filtered sensor data.
struct MotionSignals
{
    bool launchDetected = false;   ///< sustained throw/launch acceleration
    bool stableDetected = false;   ///< sustained stillness (landed)
};

/// Euler angles in degrees, in the convention the existing telemetry expects.
struct EulerAnglesDeg
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
};

/// Orientation as a unit quaternion.
struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

/**
 * @brief Which sensor instance produced the current state.
 *
 * Published even though nothing can switch yet (ADR-026). A redundant system
 * whose logs cannot answer "did it switch, and when?" cannot be investigated
 * after an incident, and that is the first question anyone asks.
 */
struct SelectionState
{
    std::uint8_t  activeAccel = 0;
    std::uint8_t  activeGyro  = 0;
    std::uint8_t  activeBaro  = 0;
    std::uint8_t  activeMag   = 0;
    std::uint32_t switchCount = 0;   ///< cumulative since boot
    hal::Clock::time_point lastSwitch{};
};

/**
 * @brief One coherent snapshot of the aircraft's inertial state.
 *
 * @note Sensor data only. FlightState is deliberately NOT here — StateManagement
 *       owns it. Two owners for one value is how it drifts.
 *
 * @note Must stay trivially copyable: SeqLock publishes it with a raw copy
 *       between memory fences. The static_assert below is the guarantee.
 */
struct ImuState
{
    Vec3f accel_g{};      ///< body frame, calibrated, filtered
    Vec3f gyro_dps{};     ///< body frame, calibrated, filtered
    Vec3f mag_ut{};       ///< body frame; zero when no magnetometer is fitted

    /// @note Named for its type on purpose: a bare `orientation` is ambiguous
    ///       between Euler angles and a quaternion, and that ambiguity compiles.
    Quaternion     orientation_quat{};
    EulerAnglesDeg euler_deg{};

    float altitude_m    = 0.0f;   ///< above the calibrated ground reference
    float climbRate_mps = 0.0f;

    MotionSignals  motion{};
    SelectionState selection{};

    bool healthy = true;

    /// True when the tick that produced this snapshot fused a magnetometer
    /// reading. Worth publishing rather than inferring from a non-zero mag_ut:
    /// it is the difference between a heading and an integrated gyro drift, and
    /// on a board with no magnetometer the answer is permanently false.
    bool magnetometerFused = false;

    hal::Clock::time_point time{};
};

static_assert(std::is_trivially_copyable_v<ImuState>,
              "ImuState is published through a SeqLock, which copies it with "
              "plain loads and stores between fences. A non-trivial member "
              "would make that copy undefined behaviour under a concurrent "
              "write, not merely torn.");

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_IMU_STATE_H
