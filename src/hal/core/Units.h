/**
 * Units.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Unit constants and conversions.
 *
 * ADR-008 chose unit SUFFIXES over strong unit types: every field and parameter
 * carrying a physical quantity is named with its unit (gyro_dps, pressure_pa,
 * altitude_m). Time is the exception and uses std::chrono, because the standard
 * library already provides the type.
 */
#ifndef ARDUFLITE_HAL_CORE_UNITS_H
#define ARDUFLITE_HAL_CORE_UNITS_H

#include <cmath>

namespace arduflite::units {

inline constexpr float kGravity_mps2   = 9.80665f;
inline constexpr float kDegToRad       = 0.017453292519943295f;
inline constexpr float kRadToDeg       = 57.29577951308232f;
inline constexpr float kPaToHpa        = 0.01f;
inline constexpr float kSeaLevel_hpa   = 1013.25f;

constexpr float degToRad(float deg) noexcept { return deg * kDegToRad; }
constexpr float radToDeg(float rad) noexcept { return rad * kRadToDeg; }

/**
 * @brief Barometric altitude above a reference pressure.
 *
 * Single precision deliberately: the ESP32-C3 has no FPU (see
 * specs/hal/00-current-state.md 2.9b), so double is emulated at roughly twice
 * the cost of float.
 *
 * @return NAN if either pressure is non-positive — the caller has no reading
 *         yet, and 0 metres would be indistinguishable from being at the
 *         reference.
 *
 * @param pressure_hpa   Measured pressure, hPa.
 * @param reference_hpa  Ground reference pressure, hPa.
 * @return Altitude in metres above the reference.
 */
inline float altitudeFromPressure_m(float pressure_hpa, float reference_hpa) noexcept
{
    if (!(pressure_hpa > 0.0f) || !(reference_hpa > 0.0f)) { return NAN; }
    return 44330.0f * (1.0f - powf(pressure_hpa / reference_hpa, 0.1903f));
}

} // namespace arduflite::units

#endif // ARDUFLITE_HAL_CORE_UNITS_H
