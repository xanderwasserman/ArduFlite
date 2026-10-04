/**
 * InertialSubsystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/estimation/InertialSubsystem.h"

#include <cmath>

namespace arduflite::estimation {

namespace {

bool finite(const Vec3f& v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

bool withinMagnitude(const Vec3f& v, float limit)
{
    return std::fabs(v.x) <= limit && std::fabs(v.y) <= limit && std::fabs(v.z) <= limit;
}

} // namespace

void InertialSubsystem::bind(const Dependencies& deps)
{
    _devices   = deps.devices;
    _selector  = &deps.selector;
    _estimator = &deps.estimator;
    _clock     = &deps.clock;
    _scheduler = &deps.scheduler;
    _watchdog  = &deps.watchdog;
    _settings  = &deps.settings;
}

void InertialSubsystem::configure(const Config& config)
{
    _config    = config;
    _transform = AxisTransform{ config.axes };

    _accelFilter.setAlpha(config.accelAlpha);
    _gyroFilter.setAlpha(config.gyroAlpha);
    _magFilter.setAlpha(config.magAlpha);

    // Expressed as a duration, not a tick count, so it does not silently change
    // meaning when the task rate does.
    _magEngageTicks =
        static_cast<std::uint16_t>(static_cast<float>(config.taskRate_hz) * kMagEngageDelay_s);
    if (_magEngageTicks == 0) { _magEngageTicks = 1; }
    _altitude.setAlpha(config.altiAlpha);

    // Rate-aware sampling. A part that converts at 15 Hz gains nothing from
    // being read at 500 Hz — it returns the same conversion repeatedly while
    // costing a bus transaction every tick. The divider is derived from the
    // part's own declared rate, so there is no second constant to keep in step.
    for (std::size_t i = 0; i < _devices.size() && i < kMaxDevices; ++i)
    {
        const auto* device = _devices[i];
        const std::uint16_t rate = (device != nullptr) ? device->nativeRate_hz() : 0;

        _sampleDivider[i] = (rate == 0 || rate >= config.taskRate_hz)
                                ? 1
                                : static_cast<std::uint16_t>(config.taskRate_hz / rate);
        _sampleCounter[i] = 0;
    }
}

void InertialSubsystem::resetFilters()
{
    _accelFilter.reset();
    _gyroFilter.reset();
    _magFilter.reset();
    _altitude.reset();
    _baroTickCounter     = 0;
    _magValidStreak      = 0;
    _consecutiveFailures = 0;
    _healthy             = true;
}

bool InertialSubsystem::validate(const Vec3f& accel_g, const Vec3f& gyro_dps, bool burstOk)
{
    // A failed burst leaves the previous sample in place. Those values are
    // finite and in range, so nothing below would catch them — staleness has to
    // be judged from the read result, not from the numbers.
    const bool valid = burstOk &&
                       finite(accel_g) && finite(gyro_dps) &&
                       withinMagnitude(accel_g, _config.maxAccel_g) &&
                       withinMagnitude(gyro_dps, _config.maxGyro_dps);

    if (valid)
    {
        _consecutiveFailures = 0;
        _healthy             = true;
    }
    else if (_consecutiveFailures < _config.failThreshold)
    {
        // Saturating rather than wrapping: at 500 Hz a uint8_t counter would
        // wrap every half second of sustained failure and keep re-crossing the
        // threshold instead of latching.
        ++_consecutiveFailures;
        if (_consecutiveFailures >= _config.failThreshold) { _healthy = false; }
    }

    return valid;
}

namespace {

/// NVS key for the calibration blob.
constexpr const char* kCalibrationKey = "imu_calib";

/// The sampling task's entry point. Everything RTOS-shaped lives here; tick()
/// stays a plain function so the host tests can drive it.
struct TaskEntry
{
    static void run(void* argument)
    {
        auto* self = static_cast<InertialSubsystem*>(argument);
        self->runTaskLoop();
    }
};

} // namespace

bool InertialSubsystem::begin()
{
    InertialOffsets stored{};
    if (loadStoredOffsets(stored))
    {
        setOffsets(stored);
    }

    // Settle the fusion filter before anything reads an attitude from it. A
    // fixed dt rather than a measured one: this is a warm-up, and making it
    // depend on how fast the loop happens to run makes it non-reproducible.
    const float dt = 1.0f / static_cast<float>(_config.taskRate_hz);
    for (int i = 0; i < 2000; ++i) { tick(dt); }

    // The warm-up is not flight data. Reset so the first real tick does not
    // differentiate the altitude across the seeding loop, nor read the loop as
    // two thousand ticks of sustained stillness and report a landing.
    resetFilters();
    return true;
}

Status InertialSubsystem::startTask()
{
    if (_scheduler == nullptr) { return Status::NotPresent; }
    if (_task != nullptr) { return Status::Ok; }

    hal::TaskConfig config;
    config.name       = "IMU Task";
    config.stackBytes = 4096;
    config.priority   = hal::Priority::Inertial;

    Result<hal::Task*> task = (*_scheduler).spawn(config, &TaskEntry::run, this);
    if (!task) { return task.status(); }

    _task = task.value();
    return Status::Ok;
}

void InertialSubsystem::runTaskLoop()
{
    // Registration must happen from inside the task, which is why it is here
    // and not in startTask(). A failure is not fatal: losing watchdog coverage
    // is bad, but refusing to sample at all is worse.
    (void)(*_watchdog).registerCurrentTask();

    std::uint64_t lastWake = 0;
    auto lastTime = (*_clock).now();

    const auto period = std::chrono::milliseconds{ 1000 / _config.taskRate_hz };

    for (;;)
    {
        (*_watchdog).feed();

        const auto now = (*_clock).now();
        float dt_s = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;

        // Clamp: a first iteration, or one delayed by a long calibration, would
        // otherwise integrate a huge step into the attitude.
        if (dt_s < kMinDt_s) { dt_s = kMinDt_s; }
        if (dt_s > kMaxDt_s) { dt_s = kMaxDt_s; }

        tick(dt_s);

        (*_scheduler).sleepUntil(lastWake, period);
    }
}

bool InertialSubsystem::calibrate(CalibrationService::Kind kind)
{
    if (_scheduler == nullptr) { return false; }
    if (_calibration.request(kind) != Status::Ok) { return false; }

    // If the task is not running yet — begin() calls this for the barometric
    // reference — drive the machine here instead of waiting on a task that does
    // not exist.
    const bool driveLocally = (_task == nullptr);
    const float dt = 1.0f / static_cast<float>(_config.taskRate_hz);

    for (;;)
    {
        const CalibrationService::State state = _calibration.state();
        if (state == CalibrationService::State::Complete) { return true; }
        if (state == CalibrationService::State::Failed)   { return false; }

        if (driveLocally) { tick(dt); }
        (*_scheduler).sleepFor(std::chrono::milliseconds{ driveLocally ? 2 : 50 });
    }
}

bool InertialSubsystem::loadStoredOffsets(InertialOffsets& out)
{
    if (_settings == nullptr) { return false; }

    const Status status = (*_settings).load(kCalibrationKey, &out, sizeof(out));
    if (status == Status::Ok) { return true; }

    // A CRC failure is worth distinguishing from "never written": it means a
    // blob that WAS written correctly has since decayed, and applying it would
    // silently bias the aircraft's idea of level.
    return false;
}

bool InertialSubsystem::storeOffsets(const InertialOffsets& offsets)
{
    if (_settings == nullptr) { return false; }
    return (*_settings).save(kCalibrationKey, &offsets, sizeof(offsets)) == Status::Ok;
}

void InertialSubsystem::setOffsets(const InertialOffsets& offsets)
{
    _pendingOffsets = offsets;
    _offsetsPending.store(true, std::memory_order_release);
}

void InertialSubsystem::setReferencePressure_hpa(float hpa)
{
    _pendingReferencePressure_hpa = hpa;
    _referencePressurePending.store(true, std::memory_order_release);
}

void InertialSubsystem::tick(float dt_s)
{
    // Callable before bind(): the CLI and telemetry hold a pointer to this from
    // static-init time, and a boot that fails before Board::begin() must not
    // fault here.
    if (_selector == nullptr || _estimator == nullptr || _clock == nullptr) { return; }

    // ── 0. adopt anything staged by another task ───────────────────────────
    // Done here, once, so the rest of the tick sees a single coherent set of
    // offsets rather than a mixture written under it mid-computation.
    if (_offsetsPending.load(std::memory_order_acquire))
    {
        _offsets = _pendingOffsets;
        _offsetsPending.store(false, std::memory_order_relaxed);
    }
    if (_referencePressurePending.load(std::memory_order_acquire))
    {
        _altitude.setReferencePressure_hpa(_pendingReferencePressure_hpa);
        _referencePressurePending.store(false, std::memory_order_relaxed);
    }

    // ── 1. sample, rate-aware. The ONLY bus access in the system ────────────
    // Every device is sampled, not just the selected one: reading all instances
    // every tick is what leaves crossfading and median voting open as future
    // selection policies without touching this loop again.
    for (std::size_t i = 0; i < _devices.size() && i < kMaxDevices; ++i)
    {
        device::Sensor* device = _devices[i];
        if (device == nullptr) { continue; }

        if (++_sampleCounter[i] >= _sampleDivider[i])
        {
            _sampleCounter[i] = 0;
            (void)device->sample();   // health is read below; a failure is not fatal here
        }
    }

    // ── 2. choose instances ────────────────────────────────────────────────
    const auto now = (*_clock).now();
    (*_selector).evaluate(now);

    // ── 3. read — cached, no bus ───────────────────────────────────────────
    Vec3f rawAccel_g{};
    Vec3f rawGyro_dps{};
    bool  burstOk = false;

    device::Accelerometer* accelerometer = (*_selector).primaryAccel();
    device::Gyroscope*     gyroscope     = (*_selector).primaryGyro();

    if (accelerometer != nullptr && gyroscope != nullptr)
    {
        device::AccelSample accelSample{};
        device::GyroSample  gyroSample{};
        burstOk = accelerometer->read(accelSample) == Status::Ok &&
                  gyroscope->read(gyroSample) == Status::Ok;
        rawAccel_g  = accelSample.accel_g;
        rawGyro_dps = gyroSample.rate_dps;
    }

    // The magnetometer is optional and read the same way — from cache, no bus.
    // A board without one leaves magValid false for its whole life, and every
    // magnetometer-dependent branch below collapses to the six-axis path.
    Vec3f rawMag_ut{};
    bool  magValid = false;

    device::Magnetometer* magnetometer = (*_selector).primaryMag();
    if (magnetometer != nullptr && magnetometer->health() == device::SensorHealth::Ok)
    {
        device::MagSample magSample{};
        if (magnetometer->read(magSample) == Status::Ok)
        {
            rawMag_ut = magSample.field_ut;

            // An exactly-zero field is not a reading. It is what a part that has
            // been reset but not configured returns, and it is also what the
            // filter would divide by when it normalises the vector to a
            // direction. Rejecting it here keeps that decision out of the
            // fusion code, where it would depend on which filter is installed.
            const float magnitudeSquared = rawMag_ut.x * rawMag_ut.x +
                                           rawMag_ut.y * rawMag_ut.y +
                                           rawMag_ut.z * rawMag_ut.z;
            magValid = magnitudeSquared > kMinMagSquared_ut2;
        }
    }

    // ── 4. subtract calibration offsets — SENSOR frame, before the transform ─
    // Order matters and is the opposite of what §03 originally specified. The
    // offsets were averaged from untransformed readings, so they live in the
    // sensor's frame; subtracting them after a transform that negates axes would
    // apply the correction with the wrong sign on those axes, doubling the bias
    // instead of removing it (ADR-033).
    const Vec3f correctedAccel_g{ rawAccel_g.x - _offsets.accel_g.x,
                                  rawAccel_g.y - _offsets.accel_g.y,
                                  rawAccel_g.z - _offsets.accel_g.z };
    const Vec3f correctedGyro_dps{ rawGyro_dps.x - _offsets.gyro_dps.x,
                                   rawGyro_dps.y - _offsets.gyro_dps.y,
                                   rawGyro_dps.z - _offsets.gyro_dps.z };

    // ── 5. axis transform, into body frame ─────────────────────────────────
    // applyAngularRate for the gyroscope, NOT applyMeasurement: angular rate is
    // a pseudovector, so a mirrored mount needs the determinant applied as well.
    const Vec3f bodyAccel_g  = _transform.applyMeasurement(correctedAccel_g);
    const Vec3f bodyGyro_dps = _transform.applyAngularRate(correctedGyro_dps);

    // applyMeasurement, not applyAngularRate: the magnetic field is a true
    // vector. It shares the mount and therefore the transform — a magnetometer
    // left in sensor frame while the accelerometer is rotated into body frame
    // gives a heading that is wrong by exactly the mounting rotation, which
    // looks like a compass that merely needs offsetting.
    const Vec3f bodyMag_ut = _transform.applyMeasurement(rawMag_ut);

    // ── 6. low-pass bank ───────────────────────────────────────────────────
    const Vec3f filteredAccel_g  = _accelFilter.update(bodyAccel_g);
    const Vec3f filteredGyro_dps = _gyroFilter.update(bodyGyro_dps);

    // Only advanced on ticks with a real reading. Feeding the filter zeros on a
    // failed read would drag the field towards the origin and, worse, towards
    // the magnitude threshold above — turning one bad read into a run of them.
    const Vec3f filteredMag_ut = magValid ? _magFilter.update(bodyMag_ut)
                                          : _magFilter.value();

    // A second to engage, one tick to drop, and the asymmetry is the point.
    // Losing the heading is safe — the filter just stops correcting yaw.
    // Regaining it is expensive: Madgwick normalises the combined
    // accelerometer-plus-magnetometer gradient as ONE vector, so a large
    // magnetic residual takes correction authority away from the accelerometer
    // while the heading slews in. Roll and pitch are what the loops fly on.
    //
    // The filter keeps advancing during the wait, so this engages against a
    // settled field rather than a cold one.
    if (magValid)
    {
        if (_magValidStreak < _magEngageTicks) { ++_magValidStreak; }
    }
    else
    {
        _magValidStreak = 0;
    }
    const bool fuseMag = _config.fuseMagnetometer &&
                         magValid && _magValidStreak >= _magEngageTicks;

    // ── 7. validate → health ───────────────────────────────────────────────
    validate(filteredAccel_g, filteredGyro_dps, burstOk);

    // ── 8. barometer, on decimation ticks only ─────────────────────────────
    float pressure_hpa = 0.0f;
    device::Barometer* barometer = (*_selector).primaryBaro();

    // Decimation derived from the barometer ITSELF, not from a separate
    // configured constant. Those were two independent numbers and they
    // disagreed: the part was sampled every taskRate/nativeRate ticks (33 at
    // 500 Hz for a 15 Hz BMP280) while being read every baroDecimation ticks
    // (10). Two of every three reads returned the same conversion, and the
    // climb-rate derivative divided by the READ interval rather than the
    // CONVERSION interval — overstating vertical speed by the ratio between
    // them.
    const std::uint16_t baroRate = (barometer != nullptr) ? barometer->nativeRate_hz() : 0;
    const std::uint16_t baroDivider =
        (baroRate == 0 || baroRate >= _config.taskRate_hz)
            ? 1
            : static_cast<std::uint16_t>(_config.taskRate_hz / baroRate);

    if (barometer != nullptr && ++_baroTickCounter >= baroDivider)
    {
        _baroTickCounter = 0;

        device::BaroSample baroSample{};
        if (barometer->read(baroSample) == Status::Ok)
        {
            pressure_hpa = baroSample.pressure_pa / 100.0f;

            // The interval between CONVERSIONS, which is what the derivative
            // is actually over.
            const float baroDt_s =
                static_cast<float>(baroDivider) / static_cast<float>(_config.taskRate_hz);
            _altitude.update(pressure_hpa, baroDt_s);
        }
    }

    // ── 9. fusion ──────────────────────────────────────────────────────────
    // Decided per tick, not once at begin(): a magnetometer that fails in
    // flight degrades to six-axis on the next tick rather than feeding the
    // filter a frozen heading.
    //
    // The part converts at 100 Hz inside a 500 Hz tick, so the same reading is
    // fused several times. Harmless — the filter uses the field as a direction,
    // and a repeated direction applies a consistent correction.
    if (fuseMag)
    {
        (*_estimator).updateWithMagnetometer(filteredGyro_dps, filteredAccel_g,
                                             filteredMag_ut, dt_s);
    }
    else
    {
        (*_estimator).update(filteredGyro_dps, filteredAccel_g, dt_s);
    }

    // ── 10. motion signals ─────────────────────────────────────────────────
    const MotionSignals motion = _motion.update(filteredAccel_g, filteredGyro_dps, now);

    // ── 11. calibration, if any is pending ─────────────────────────────────
    // Raw, sensor-frame values, matching the frame the offsets are stored in.
    _calibration.service(rawAccel_g, rawGyro_dps, pressure_hpa, now);

    // ── 12. publish — one seqlock write, last ──────────────────────────────
    ImuState published{};
    published.accel_g       = filteredAccel_g;
    published.gyro_dps      = filteredGyro_dps;
    published.mag_ut        = filteredMag_ut;
    published.orientation_quat = (*_estimator).orientation();
    published.euler_deg     = (*_estimator).euler_deg();
    published.altitude_m    = _altitude.altitude_m();
    published.climbRate_mps = _altitude.climbRate_mps();
    published.motion        = motion;
    published.selection     = (*_selector).state();
    published.healthy       = _healthy;
    published.magnetometerFused = fuseMag;
    published.time          = now;

    _state.publish(published);
}

} // namespace arduflite::estimation
