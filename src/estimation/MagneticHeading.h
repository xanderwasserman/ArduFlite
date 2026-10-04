/**
 * MagneticHeading.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Tilt-compensated magnetic heading, for instrumentation.
 *
 * Deliberately NOT part of the estimation tick. Nothing in the control path
 * consumes a heading (ADR-055), so this is computed on demand by whoever wants
 * to display or log one — at telemetry rate, not at 500 Hz. Six transcendental
 * calls per tick on a chip with no FPU (§00 2.9b) would be real cost for a
 * number nothing steers by.
 */
#ifndef ARDUFLITE_ESTIMATION_MAGNETIC_HEADING_H
#define ARDUFLITE_ESTIMATION_MAGNETIC_HEADING_H

#include <cmath>

#include "src/core/FlightTypes.h"
#include "src/hal/core/Units.h"
#include "src/hal/core/Vec3.h"

namespace arduflite::estimation {

/// Field magnitude in microtesla. The single most useful magnetometer
/// diagnostic: Earth's field is 25-65 uT and its MAGNITUDE does not depend on
/// which way the aircraft points. Magnitude that moves with attitude means a
/// hard-iron offset; magnitude that moves with throttle means the motor, which
/// no hard-iron calibration can fix.
[[nodiscard]] inline float magneticFieldStrength_ut(const Vec3f& mag_ut)
{
    return std::sqrt(mag_ut.x * mag_ut.x + mag_ut.y * mag_ut.y + mag_ut.z * mag_ut.z);
}

/**
 * @brief Magnetic heading in degrees, 0 = magnetic north, increasing clockwise.
 *
 * Tilt-compensated: the body-frame field is rotated back through roll and pitch
 * into the horizontal plane before the heading is taken. Skipping that gives a
 * heading correct only while level, swinging wildly in a banked turn — which
 * reads as a noisy magnetometer rather than as missing maths.
 *
 * @param mag_ut     body-frame field, microtesla (any scale works; only the
 *                   direction is used)
 * @param attitude   roll and pitch in degrees. Yaw is ignored — supplying it
 *                   would be circular.
 * @return heading in [0, 360). Returns 0 for a zero-length field.
 *
 * @note Directly comparable to the `yaw` column, which is also a true angle —
 *       but yaw is signed (-180..180) and referenced to wherever the filter
 *       initialised, while this is a compass heading in 0..360. Expect them to
 *       differ by a constant, plus gyro drift, plus magnetic declination.
 */
[[nodiscard]] inline float magneticHeading_deg(const Vec3f& mag_ut,
                                               const AttitudeDeg& attitude)
{
    if (magneticFieldStrength_ut(mag_ut) <= 0.0f) { return 0.0f; }

    const float roll_rad  = attitude.roll  * units::kDegToRad;
    const float pitch_rad = attitude.pitch * units::kDegToRad;

    const float sinRoll  = std::sin(roll_rad),  cosRoll  = std::cos(roll_rad);
    const float sinPitch = std::sin(pitch_rad), cosPitch = std::cos(pitch_rad);

    // Rotate the body-frame field into the level frame: Ry(pitch) * Rx(roll).
    const float levelX = mag_ut.x * cosPitch +
                         mag_ut.y * sinRoll * sinPitch +
                         mag_ut.z * cosRoll * sinPitch;
    const float levelY = mag_ut.y * cosRoll - mag_ut.z * sinRoll;

    // Negated Y: heading increases clockwise (north-east-down), while a rotation
    // to the right moves the field to the left in body axes.
    float heading_deg = std::atan2(-levelY, levelX) * units::kRadToDeg;
    if (heading_deg < 0.0f) { heading_deg += 360.0f; }
    return heading_deg;
}

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_MAGNETIC_HEADING_H
