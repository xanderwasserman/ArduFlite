/**
 * InertialSubsystem.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The sampling loop: sensors in, ImuState out.
 *
 * Absorbs most of what ArduFliteIMU did. The important structural difference is
 * that the per-tick work is a plain method, tick(), with no FreeRTOS in it at
 * all — the task is a thin wrapper that calls it. That is what lets the twelve
 * step ordering contract be tested on a host, and what makes L3 log replay
 * possible: replay is just tick() driven from recorded samples.
 */
#ifndef ARDUFLITE_ESTIMATION_INERTIAL_SUBSYSTEM_H
#define ARDUFLITE_ESTIMATION_INERTIAL_SUBSYSTEM_H

#include <atomic>
#include <span>

#include "src/estimation/AltitudeFilter.h"
#include "src/estimation/AttitudeEstimator.h"
#include "src/estimation/CalibrationService.h"
#include "src/estimation/ImuState.h"
#include "src/estimation/LowPassBank.h"
#include "src/estimation/MotionDetector.h"
#include "src/estimation/SensorSelector.h"
#include "src/hal/core/AxisTransform.h"
#include "src/hal/core/SeqLock.h"
#include "src/hal/device/Peripherals.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/Watchdog.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::estimation {

class InertialSubsystem : private NonCopyable
{
public:
    struct Dependencies
    {
        /// Everything needing sample() — one entry per PART, not per measurement.
        std::span<device::Sensor* const> devices;

        SensorSelector&    selector;
        AttitudeEstimator& estimator;
        const hal::Clock&  clock;

        hal::Scheduler&        scheduler;
        hal::Watchdog&         watchdog;
        device::SettingsStore& settings;   ///< calibration blobs
    };

    struct Config
    {
        std::uint16_t taskRate_hz = 500;

        float accelAlpha = 1.0f;
        float gyroAlpha  = 1.0f;
        float magAlpha   = 1.0f;
        float altiAlpha  = 1.0f;

        /**
         * @brief Fuse the magnetometer into attitude (nine-axis).
         *
         * Default OFF, and that is a decision rather than a placeholder. A
         * magnetometer being FITTED is a board fact the descriptor owns; a
         * magnetometer being TRUSTED in the fusion loop is a judgement about
         * calibration, and both must be true. With this false the part is still
         * probed, sampled, transformed, filtered and published — it simply does
         * not steer the estimate. See ADR-055.
         */
        bool fuseMagnetometer = false;

        /// How the IMU is mounted. Applied AFTER offsets — see tick().
        AxisMap axes{};

        /// Plausibility limits. Beyond these a reading is a fault, not a
        /// manoeuvre. Configurable because they are airframe-dependent — an
        /// aerobatic model legitimately sees rates a glider never will.
        float        maxAccel_g   = 16.0f;
        float        maxGyro_dps  = 2000.0f;
        std::uint8_t failThreshold = 10;

        /// Launch and stability detection (imu.launch_* keys).
        MotionDetector::Config motion{};
    };

    /**
     * Default-constructible so it can live at file scope alongside the
     * controller and CLI, which take a pointer to it during static
     * initialisation. Its dependencies come from Board and therefore do not
     * exist until Board::begin() has run — the same ordering problem Board
     * itself solves with constinit storage and a begin().
     *
     * Every method is safe to call before bind(); they report no sensors.
     */
    InertialSubsystem() = default;

    explicit InertialSubsystem(const Dependencies& deps) { bind(deps); }

    /// Supply the dependencies. Call once, after Board::begin().
    void bind(const Dependencies& deps);

    void configure(const Config& config);

    /**
     * @brief Load stored calibration and settle the fusion filter.
     *
     * @return false only if there is no usable accelerometer or gyroscope.
     *         A missing barometer is reported and tolerated.
     */
    bool begin();

    /**
     * @brief Spawn the sampling task.
     *
     * Goes through hal::Scheduler rather than xTaskCreate, which is what keeps
     * this layer free of the RTOS and therefore host-testable. tick() itself
     * never touches the scheduler.
     */
    Status startTask();

    /**
     * @brief One iteration of the twelve-step contract (§03 3.8).
     *
     * No FreeRTOS, no blocking, no logging. Everything it needs arrives through
     * Dependencies, so a host test can drive it with fakes and a virtual clock.
     *
     * @param dt_s seconds since the previous tick, already clamped by the caller
     */
    void tick(float dt_s);

    /// Lock-free read. THE public API for every other task.
    [[nodiscard]] ImuState state() const { return _state.read(); }

    /**
     * @brief Run a calibration to completion, blocking the CALLING task.
     *
     * The sampling task is NOT suspended — it keeps ticking, publishing attitude
     * and feeding the watchdog, and accumulates one sample per tick (ADR-034).
     * Only the caller waits, and it holds nothing while waiting.
     */
    bool calibrate(CalibrationService::Kind kind);

    /// Persisted offsets, CRC-protected. Returns false if none are stored.
    bool loadStoredOffsets(InertialOffsets& out);
    bool storeOffsets(const InertialOffsets& offsets);

    [[nodiscard]] CalibrationService& calibration() { return _calibration; }

    /**
     * @brief Publish new zero offsets, in SENSOR frame (ADR-033).
     *
     * Callable from any task. Staged and adopted at the top of the next tick
     * rather than written into the live set: the tick reads all six floats, and
     * a plain cross-task write lets it see three new components and three old
     * ones — a one-tick attitude glitch after every calibration. Staging keeps
     * the tick lock-free.
     */
    void setOffsets(const InertialOffsets& offsets);

    /// The offsets currently in force. May lag setOffsets() by one tick.
    [[nodiscard]] InertialOffsets offsets() const { return _offsets; }

    /// Staged the same way as setOffsets(), and for the same reason.
    void setReferencePressure_hpa(float hpa);

    /**
     * @brief Drop all filter history.
     *
     * Call after applying new offsets. Every filter here is recursive, so
     * without this the pre-calibration values keep influencing the output, and
     * the altitude derivative spans the discontinuity and invents a climb rate.
     */
    void resetFilters();

    [[nodiscard]] bool healthy() const { return _healthy; }

    /// Retry counters from the snapshot publisher, for the `stats` command.
    [[nodiscard]] SeqLock<ImuState>::Health snapshotHealth() const { return _state.health(); }

    /// Task body. Public only so the entry-point trampoline can reach it.
    void runTaskLoop();

private:
    static constexpr float kMinDt_s = 0.000001f;
    static constexpr float kMaxDt_s = 0.05f;

    /// Squared magnitude below which a magnetic reading is treated as absent.
    /// Earth's field is 25-65 uT everywhere, so 1 uT rejects only a dead part,
    /// never a real measurement. Squared to avoid a sqrt in the tick.
    static constexpr float kMinMagSquared_ut2 = 1.0f;

    /**
     * @brief Seconds of unbroken good readings before the magnetometer is fused.
     *
     * Dropping OUT is immediate; only engaging waits. A second, not something
     * reflex-fast, because:
     *
     *  - the part converts at 100 Hz and read() is cached, so a tenth of a
     *    second is only ~10 distinct conversions — enough to rule out one
     *    dropped burst, not an intermittent fault;
     *  - the field low-pass defaults to alpha 0.04, giving a ~50 ms time
     *    constant at 500 Hz. A second is 20 tau; a tenth is 2;
     *  - waiting costs a heading no control loop reads, while engaging early
     *    costs roll and pitch authority (ADR-054).
     */
    static constexpr float kMagEngageDelay_s = 1.0f;

    bool validate(const Vec3f& accel_g, const Vec3f& gyro_dps, bool burstOk);

    // Pointers rather than the reference-holding Dependencies struct, so this
    // object can be constructed before its dependencies exist.
    std::span<device::Sensor* const> _devices{};
    SensorSelector*        _selector  = nullptr;
    AttitudeEstimator*     _estimator = nullptr;
    const hal::Clock*      _clock     = nullptr;
    hal::Scheduler*        _scheduler = nullptr;
    hal::Watchdog*         _watchdog  = nullptr;
    device::SettingsStore* _settings  = nullptr;

    Config _config{};

    AxisTransform _transform{};

    LowPassVec3    _accelFilter;
    LowPassVec3    _gyroFilter;
    LowPassVec3    _magFilter;
    AltitudeFilter _altitude;

    MotionDetector     _motion;
    CalibrationService _calibration;

    InertialOffsets   _offsets{};
    InertialOffsets   _pendingOffsets{};
    std::atomic<bool> _offsetsPending{ false };

    float             _pendingReferencePressure_hpa = 0.0f;
    std::atomic<bool> _referencePressurePending{ false };

    SeqLock<ImuState> _state{};

    hal::Task*    _task = nullptr;
    std::uint16_t _baroTickCounter = 0;

    /// Consecutive ticks with a valid magnetic reading, saturating at
    /// _magEngageTicks. Asymmetric on purpose — see tick().
    std::uint16_t _magValidStreak = 0;
    std::uint16_t _magEngageTicks = 1;
    std::uint8_t  _consecutiveFailures = 0;
    bool          _healthy = true;

    /// Guards the tick's sample() pass so a part is not sampled faster than it
    /// can convert. Index matches Dependencies::devices.
    static constexpr std::size_t kMaxDevices = 8;
    std::uint16_t _sampleDivider[kMaxDevices]{};
    std::uint16_t _sampleCounter[kMaxDevices]{};
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_INERTIAL_SUBSYSTEM_H
