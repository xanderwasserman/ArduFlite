/**
 * MotionDetector.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/estimation/MotionDetector.h"

namespace arduflite::estimation {

namespace {

constexpr float squared(float v) { return v * v; }

} // namespace

void MotionDetector::resetTimers(hal::Clock::time_point now)
{
    _motionStart = now;
    _stableStart = now;
    _seeded      = true;
    _signals     = {};
}

MotionSignals MotionDetector::update(const Vec3f& accel_g, const Vec3f& gyro_dps,
                                     hal::Clock::time_point now)
{
    // First call seeds the windows. Without this they start at the epoch, so the
    // very first tick sees an elapsed time of "since boot" and both conditions
    // fire immediately.
    if (!_seeded) { resetTimers(now); }

    const float accelMagSq = accel_g.x  * accel_g.x  + accel_g.y  * accel_g.y  + accel_g.z  * accel_g.z;
    const float gyroMagSq  = gyro_dps.x * gyro_dps.x + gyro_dps.y * gyro_dps.y + gyro_dps.z * gyro_dps.z;

    const float throwHiSq  = squared(1.0f + _config.accelThrowThreshold_g);
    const float throwLoSq  = squared(1.0f - _config.accelThrowThreshold_g);
    const float stableHiSq = squared(1.0f + _config.accelStableThreshold_g);
    const float stableLoSq = squared(1.0f - _config.accelStableThreshold_g);
    const float gyroThrowMinSq  = squared(_config.gyroThrowMin_dps);
    const float gyroThrowMaxSq  = squared(_config.gyroThrowMax_dps);
    const float gyroStableSq    = squared(_config.gyroStableThreshold_dps);

    // Launch: accel OR gyro may trigger, but gyro must stay below the maximum.
    // The OR supports soft hand-launches, where the accel deviation is small but
    // the pitch-up rotation clearly exceeds the minimum. The upper gyro bound
    // rejects tumbling, which is not a launch.
    const bool accelThrow = (accelMagSq > throwHiSq || accelMagSq < throwLoSq);
    if ((accelThrow || gyroMagSq > gyroThrowMinSq) && gyroMagSq < gyroThrowMaxSq)
    {
        _signals.launchDetected = (now - _motionStart) > _config.launchDebounce;
    }
    else
    {
        _motionStart            = now;   // restart the window
        _signals.launchDetected = false;
    }

    // Stability: sustained near-1 g with almost no rotation. The stable
    // threshold is deliberately looser than the throw one, to tolerate grass,
    // rough ground and slight inclines without blocking the LANDED transition.
    const bool accelStable = (accelMagSq >= stableLoSq && accelMagSq <= stableHiSq);
    if (accelStable && gyroMagSq < gyroStableSq)
    {
        _signals.stableDetected = (now - _stableStart) >= _config.stableDebounce;
    }
    else
    {
        _stableStart            = now;
        _signals.stableDetected = false;
    }

    return _signals;
}

} // namespace arduflite::estimation
