/**
 * MotionDetector.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Debounced launch and stability detection from filtered inertial data.
 *
 * Pure signal processing over accelerometer and gyroscope. Thresholds and
 * debounce times are unchanged — StateManagement drives the PREFLIGHT → INFLIGHT
 * → LANDED transitions off these two booleans, so a change here changes when the
 * aircraft believes it has been launched.
 */
#ifndef ARDUFLITE_ESTIMATION_MOTION_DETECTOR_H
#define ARDUFLITE_ESTIMATION_MOTION_DETECTOR_H

#include <chrono>

#include "src/estimation/ImuState.h"
#include "src/hal/core/Vec3.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::estimation {

class MotionDetector
{
public:
    /**
     * Tuning. Defaults are the values that have been flying.
     *
     * @note Comparisons are done on SQUARED magnitudes so the 500 Hz path needs
     *       no sqrtf. |a| deviating from 1 g by more than T is equivalent to
     *       a_magSq > (1+T)^2 or a_magSq < (1-T)^2.
     */
    struct Config
    {
        float accelThrowThreshold_g   = 0.10f;   ///< deviation from 1 g meaning "thrown"
        float accelStableThreshold_g  = 0.30f;   ///< deviation within which it is "still"
        float gyroThrowMin_dps        = 15.0f;   ///< rotation qualifying as a throw
        float gyroThrowMax_dps        = 150.0f;  ///< above this it is tumbling, not a launch
        float gyroStableThreshold_dps = 2.0f;

        std::chrono::milliseconds launchDebounce{ 50 };
        std::chrono::milliseconds stableDebounce{ 2000 };
    };

    MotionDetector() = default;
    explicit MotionDetector(const Config& config) : _config(config) {}

    /**
     * @brief One tick of detection.
     * @param accel_g  filtered accelerometer, body frame
     * @param gyro_dps filtered gyroscope, body frame
     * @param now      monotonic time; debounce windows are measured against it
     */
    MotionSignals update(const Vec3f& accel_g, const Vec3f& gyro_dps,
                         hal::Clock::time_point now);

    [[nodiscard]] const MotionSignals& signals() const { return _signals; }

    /**
     * @brief Restart both debounce windows at `now`.
     *
     * Called after anything that suspends sampling — calibration in particular.
     * Without it, the gap reads as a long period of satisfied conditions and
     * stableDetected latches true the instant sampling resumes.
     */
    void resetTimers(hal::Clock::time_point now);

private:
    Config        _config;
    MotionSignals _signals{};

    hal::Clock::time_point _motionStart{};
    hal::Clock::time_point _stableStart{};
    bool                   _seeded = false;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_MOTION_DETECTOR_H
