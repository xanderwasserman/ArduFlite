/**
 * FlightTypes.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Vocabulary types shared across the flight layer.
 *
 * Deliberately not the estimation layer's types: ControlMixer produces a
 * setpoint from stick input without touching a sensor.
 *
 * @note There is no single "Euler angles" type. Attitude, angular rate and
 *       normalised command are three different quantities; one type spanning
 *       all three would let a deg/s value reach a -1..+1 consumer with no
 *       compiler complaint. `Vector3` is a plain triple for values with no
 *       single quantity attached.
 */
#ifndef ARDUFLITE_CORE_FLIGHT_TYPES_H
#define ARDUFLITE_CORE_FLIGHT_TYPES_H

#include "src/estimation/ImuState.h"
#include "src/hal/core/Vec3.h"

/**
 * @name Control-path quantities — one type per quantity (ADR-037)
 *
 * All three are roll/pitch/yaw triples with identical layout, kept as distinct
 * TYPES so the compiler refuses to interchange them. Assigning a rate setpoint
 * to an axis command otherwise compiles cleanly and puts a value in deg/s on
 * the mixer as a full-deflection demand.
 *
 * Not a units library — no operator overloading, no dimensional analysis. Three
 * names the compiler will not silently confuse.
 * @{
 */

/// Attitude: an angle, in degrees.
struct AttitudeDeg
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
};

/// Angular rate, in degrees per second.
struct AngularRateDps
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
};

/**
 * @brief Per-AXIS control demand, normalised to -1..+1. Dimensionless.
 *
 * The input to AirframeMixer::mix(), which turns it into per-SURFACE outputs.
 *
 * @note Distinct from `actuators::SurfaceCommands`, which holds what each
 *       servo does. The pipeline reads:
 *
 *           AxisCommand -> AirframeMixer::mix() -> SurfaceCommands
 */
struct AxisCommand
{
    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
};
/// @}

/**
 * @name Explicit conversions
 *
 * Deliberately free functions with names, not constructors or operators. A
 * conversion between these quantities is always a modelling decision, and it
 * should be visible at the call site that one is being made.
 * @{
 */

/// Clamp a raw stick triple, already in -1..+1, to a surface command.
[[nodiscard]] inline AxisCommand toAxisCommand(float roll, float pitch, float yaw)
{
    const auto clamp1 = [](float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); };
    return AxisCommand{ clamp1(roll), clamp1(pitch), clamp1(yaw) };
}

/**
 * @brief Scale a rate against per-axis maxima to a normalised command.
 *
 * @warning There is NO conversion from AngularRateDps to AxisCommand that
 *          does not need the maxima. That is the missing information the old
 *          code silently assumed away when it assigned one to the other.
 */
[[nodiscard]] inline AxisCommand rateToAxisCommand(const AngularRateDps& rate,
                                                         float maxRoll_dps,
                                                         float maxPitch_dps,
                                                         float maxYaw_dps)
{
    const auto ratio = [](float v, float max) { return max > 0.0f ? v / max : 0.0f; };
    return toAxisCommand(ratio(rate.roll,  maxRoll_dps),
                            ratio(rate.pitch, maxPitch_dps),
                            ratio(rate.yaw,   maxYaw_dps));
}
/// @}


/**
 * @brief What the aircraft believes it is doing.
 *
 * Owned by StateManagement, which is the only thing that changes it.
 * Deliberately absent from estimation::ImuState: the estimation layer has no
 * business knowing whether the aircraft thinks it is flying, and two owners for
 * one value is how it drifts.
 */
enum FlightState
{
    UNKNOWN_STATE = 0,
    PREFLIGHT     = 1,   ///< on the ground, before launch
    INFLIGHT      = 2,   ///< launched
    LANDED        = 3,   ///< come to rest after flight
};

/// Debounced motion events. Alias, so there is one definition.
using MotionSignals = arduflite::estimation::MotionSignals;

/// @name Boundary conversions
///
/// The estimation layer publishes Vec3f with unit-suffixed field names
/// (`accel_g`, `gyro_dps`); the control layer uses the quantity types above.
///
/// @note No Vec3f <-> Vector3 conversion: it would discard the unit that
///       Vec3f's field name carries. Read Vec3f directly for a magnitude.
/// @{
[[nodiscard]] inline AttitudeDeg toAttitudeDeg(const arduflite::estimation::EulerAnglesDeg& e)
{
    return AttitudeDeg{ e.roll, e.pitch, e.yaw };
}

[[nodiscard]] inline AngularRateDps toAngularRateDps(const arduflite::Vec3f& gyro_dps)
{
    return AngularRateDps{ gyro_dps.x, gyro_dps.y, gyro_dps.z };
}
/// @}

#endif // ARDUFLITE_CORE_FLIGHT_TYPES_H
