/**
 * CalibrationService.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Calibration as a state machine driven from inside the sampling task.
 *
 * Accumulation runs INSIDE the sampling task, so no other task has to be
 * paused or spin-waited on while a calibration is in progress.
 *
 * That protocol existed because calibration needed the I2C bus, and the bus is
 * owned by the IMU task — so calibration had to stop that task, take the bus,
 * and feed the watchdog itself for ten seconds while the control loops ran on
 * stale state. It was the only thing in the codebase that required a task to be
 * suspended, and it needed careful mutex choreography to avoid deadlocking
 * against the task it was suspending.
 *
 * Here, the task that owns the bus is the task that calibrates. request() sets
 * a flag; step 11 of each tick contributes ONE sample. A ten-second gyro
 * calibration is 5000 ticks of accumulation rather than a blocking loop, so:
 *
 *   - nothing is paused, so there is nothing to deadlock against;
 *   - the control loops keep receiving fresh attitude throughout;
 *   - the watchdog keeps being fed by the normal path.
 *
 * @note Sampling continues during calibration. This is not a behaviour change
 *       being smuggled in — the aircraft was always producing attitude during
 *       calibration, just from a frozen snapshot. Now it is live.
 */
#ifndef ARDUFLITE_ESTIMATION_CALIBRATION_SERVICE_H
#define ARDUFLITE_ESTIMATION_CALIBRATION_SERVICE_H

#include <atomic>
#include <chrono>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/core/Vec3.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::estimation {

/**
 * @brief Inertial zero offsets, in the SENSOR's own frame.
 *
 * @warning Sensor frame, NOT body frame. These are averaged from raw readings
 *          before any AxisTransform, and the sampling loop subtracts them
 *          before the transform too (§03, step 4). Applying them after the
 *          transform would subtract a sensor-frame correction from a
 *          possibly-negated axis, doubling the bias instead of removing it.
 */
struct InertialOffsets
{
    Vec3f accel_g{};
    Vec3f gyro_dps{};
};

class CalibrationService : private NonCopyable
{
public:
    enum class Kind : std::uint8_t
    {
        Inertial,     ///< accelerometer and gyroscope zero offsets, together
        Barometric,   ///< ground-level reference pressure
    };

    enum class State : std::uint8_t
    {
        Idle,
        Running,
        Complete,   ///< results readable; call acknowledge() to clear
        Failed,     ///< lastError() says why; call acknowledge() to clear
    };

    struct Config
    {
        /// Long enough to average out gyro noise.
        std::chrono::milliseconds duration{ 10000 };

        /// Below this the average is not trustworthy enough to store. A floor,
        /// not merely "more than zero": one reading taken during a bus fault
        /// would otherwise be stored as a calibration.
        std::uint32_t minimumSamples = 100;
    };

    CalibrationService() = default;
    explicit CalibrationService(const Config& config) : _config(config) {}

    /**
     * @brief Ask for a calibration run. Callable from any task.
     *
     * @return Status::Busy if a run is already active or awaiting acknowledge().
     */
    Status request(Kind kind);

    /// Abandon an active run and return to Idle. Any task.
    void cancel();

    /**
     * @brief Contribute one tick. Called ONLY from the sampling task (step 11).
     *
     * @param accel_g       SENSOR frame, before AxisTransform
     * @param gyro_dps      SENSOR frame, before AxisTransform
     * @param pressure_hpa  0 or negative when no barometer is fitted; such
     *                      samples are skipped rather than averaged as zero
     * @param now           monotonic time, for the duration window
     */
    void service(const Vec3f& accel_g, const Vec3f& gyro_dps, float pressure_hpa,
                 hal::Clock::time_point now);

    [[nodiscard]] State  state() const { return _state.load(std::memory_order_acquire); }
    [[nodiscard]] Kind   activeKind() const { return _kind; }
    [[nodiscard]] Status lastError() const { return _lastError; }

    /// 0..100. Meaningful while Running; 100 once Complete.
    [[nodiscard]] std::uint8_t progressPercent() const
    {
        return _progressPercent.load(std::memory_order_relaxed);
    }

    /// Valid only when state() == Complete and activeKind() == Inertial.
    [[nodiscard]] InertialOffsets inertialOffsets() const { return _offsets; }

    /// Valid only when state() == Complete and activeKind() == Barometric.
    [[nodiscard]] float referencePressure_hpa() const { return _referencePressure_hpa; }

    /// Consume a Complete/Failed result and return to Idle.
    void acknowledge();

private:
    void finish(State outcome, Status error);

    Config _config{};

    // Written by the sampling task, read by anyone.
    std::atomic<State>        _state{ State::Idle };
    std::atomic<std::uint8_t> _progressPercent{ 0 };

    /// Set by request() before the state goes Running, so the sampling task
    /// never observes Running with a stale kind.
    std::atomic<bool> _requested{ false };
    Kind              _requestedKind = Kind::Inertial;

    Kind   _kind      = Kind::Inertial;
    Status _lastError = Status::Ok;

    // Accumulators — touched only by the sampling task.
    hal::Clock::time_point _start{};
    std::uint32_t          _sampleCount = 0;
    Vec3f                  _accelSum{};
    Vec3f                  _gyroSum{};
    float                  _pressureSum_hpa = 0.0f;

    InertialOffsets _offsets{};
    float           _referencePressure_hpa = 0.0f;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_CALIBRATION_SERVICE_H
