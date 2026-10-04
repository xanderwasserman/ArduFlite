/**
 * test_inertial_subsystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the twelve-step tick contract.
 *
 * tick() has no FreeRTOS in it, which is the whole point: the ordering of the
 * steps is a correctness property that cannot be checked by reading the code —
 * every plausible order compiles and runs — but can be checked by feeding it
 * fakes and asserting on what comes out.
 *
 * The ordering that matters most here is offsets-before-transform (ADR-033).
 */
#include <gtest/gtest.h>

#include "hal_host/HostPlatform.h"
#include "src/estimation/InertialSubsystem.h"

using namespace arduflite;
using namespace arduflite::estimation;
using arduflite::hal::host::VirtualClock;

namespace {

/// One fake part providing accelerometer and gyroscope, as the MPU-6500 does.
class FakeImu final : public device::Sensor,
                      public device::Accelerometer,
                      public device::Gyroscope
{
public:
    Status probe() override { return Status::Ok; }
    Status begin() override { return Status::Ok; }
    Status sample() override { ++sampleCount; return sampleResult; }

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return rate_hz; }
    [[nodiscard]] const char* name() const override { return "FakeIMU"; }
    [[nodiscard]] device::SensorHealth health() const override { return healthValue; }

    Status read(device::AccelSample& out) const override { out.accel_g = accel_g; return readResult; }
    Status setRange_g(std::uint8_t) override { return Status::Ok; }
    [[nodiscard]] std::uint8_t range_g() const override { return 4; }

    Status read(device::GyroSample& out) const override { out.rate_dps = gyro_dps; return readResult; }
    Status setRange_dps(std::uint16_t) override { return Status::Ok; }
    [[nodiscard]] std::uint16_t range_dps() const override { return 500; }

    Vec3f accel_g{ 0.0f, 0.0f, 1.0f };
    Vec3f gyro_dps{};
    int   sampleCount = 0;
    std::uint16_t rate_hz = 500;
    Status sampleResult = Status::Ok;
    Status readResult   = Status::Ok;
    device::SensorHealth healthValue = device::SensorHealth::Ok;
};

class FakeBaro final : public device::Sensor, public device::Barometer
{
public:
    Status probe() override { return Status::Ok; }
    Status begin() override { return Status::Ok; }
    Status sample() override { ++sampleCount; return Status::Ok; }

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return rate_hz; }
    [[nodiscard]] const char* name() const override { return "FakeBaro"; }
    [[nodiscard]] device::SensorHealth health() const override { return device::SensorHealth::Ok; }

    Status read(device::BaroSample& out) const override
    {
        ++readCount;
        out.pressure_pa = pressure_pa;
        return Status::Ok;
    }


    float pressure_pa = 101325.0f;
    int   sampleCount = 0;
    mutable int readCount = 0;
    std::uint16_t rate_hz = 25;
};

class FakeMag final : public device::Sensor, public device::Magnetometer
{
public:
    Status probe() override { return Status::Ok; }
    Status begin() override { return Status::Ok; }
    Status sample() override { ++sampleCount; return Status::Ok; }

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return rate_hz; }
    [[nodiscard]] const char* name() const override { return "FakeMag"; }
    [[nodiscard]] device::SensorHealth health() const override { return healthValue; }

    Status read(device::MagSample& out) const override
    {
        ++readCount;
        out.field_ut = field_ut;
        return readResult;
    }

    Vec3f field_ut{ 20.0f, 0.0f, 40.0f };   ///< a plausible Earth field
    int   sampleCount = 0;
    mutable int readCount = 0;
    std::uint16_t rate_hz = 100;
    Status readResult = Status::Ok;
    device::SensorHealth healthValue = device::SensorHealth::Ok;
};

/// Records what the tick handed the estimator.
class RecordingEstimator final : public AttitudeEstimator
{
public:
    void begin(float) override {}
    void update(const Vec3f& gyro, const Vec3f& accel, float dt) override
    {
        lastGyro = gyro; lastAccel = accel; lastDt = dt; ++updateCount;
    }
    void updateWithMagnetometer(const Vec3f& gyro, const Vec3f& accel,
                                const Vec3f& mag, float dt) override
    {
        lastGyro = gyro; lastAccel = accel; lastMag = mag; lastDt = dt;
        ++magUpdateCount;
    }
    [[nodiscard]] Quaternion orientation() const override { return _q; }
    [[nodiscard]] EulerAnglesDeg euler_deg() const override { return _e; }
    void setOrientation(const Quaternion& q) override { _q = q; }

    Vec3f lastAccel{}, lastGyro{}, lastMag{};
    float lastDt = 0.0f;
    int   updateCount    = 0;   ///< six-axis calls
    int   magUpdateCount = 0;   ///< nine-axis calls

    Quaternion     _q{};
    EulerAnglesDeg _e{ 1.0f, 2.0f, 3.0f };
};

class SubsystemTest : public ::testing::Test
{
protected:
    void build(const InertialSubsystem::Config& config) { build(config, false); }

    /// Same board plus a magnetometer, which is the SEN0697 arrangement.
    void buildWithMagnetometer(const InertialSubsystem::Config& config)
    {
        build(config, true);
    }

    void build(const InertialSubsystem::Config& config, bool withMagnetometer)
    {
        devices = { &imu, &baro };
        accels  = { &imu };
        gyros   = { &imu };
        baros   = { &baro };
        mags.clear();

        if (withMagnetometer)
        {
            devices.push_back(&mag);
            mags.push_back(&mag);
        }

        selector  = std::make_unique<FirstHealthySelector>(accels, gyros, baros, mags);
        subsystem = std::make_unique<InertialSubsystem>(
            InertialSubsystem::Dependencies{ devices, *selector, estimator, clock,
                                             scheduler, watchdog, settings });
        subsystem->configure(config);
    }

    static InertialSubsystem::Config defaultConfig()
    {
        InertialSubsystem::Config config;
        config.taskRate_hz    = 500;
        config.accelAlpha     = 1.0f;   // unfiltered, so assertions are exact
        config.gyroAlpha      = 1.0f;
        config.altiAlpha      = 1.0f;
        config.magAlpha       = 1.0f;
        // Opt in: the shipping default is OFF (ADR-055). These tests exercise
        // the fusion path itself; the default is pinned separately below.
        config.fuseMagnetometer = true;
        return config;
    }

    /// Tick past the engage delay so nine-axis fusion is live, then zero the
    /// counters so a test asserts on what happened AFTER it engaged.
    /// One second at 500 Hz.
    static constexpr int kEngageTicks = 500;

    void settleMagnetometer()
    {
        for (int i = 0; i < kEngageTicks; ++i) { subsystem->tick(0.002f); }
        estimator.updateCount    = 0;
        estimator.magUpdateCount = 0;
    }

    FakeImu  imu;
    FakeBaro baro;
    FakeMag  mag;
    RecordingEstimator estimator;
    VirtualClock       clock;
    arduflite::hal::host::RecordingScheduler scheduler;
    arduflite::hal::host::NullWatchdog       watchdog;
    arduflite::hal::host::MemorySettingsStore settings;

    // Vectors rather than arrays: the magnetometer is present in some fixtures
    // and absent in others, which is the distinction under test.
    std::vector<device::Sensor*>        devices{};
    std::vector<device::Accelerometer*> accels{};
    std::vector<device::Gyroscope*>     gyros{};
    std::vector<device::Barometer*>     baros{};
    std::vector<device::Magnetometer*>  mags{};

    std::unique_ptr<FirstHealthySelector> selector;
    std::unique_ptr<InertialSubsystem>    subsystem;
};

// ── Optional magnetometer ───────────────────────────────────────────────────
//
// The decision "fuse a heading reference or don't" is made per tick, from what
// the board actually has and what it just returned. Every board shipped so far
// has no magnetometer, so the six-axis path is the one that must stay correct;
// the SEN0697 module is the first to exercise the other branch.

/// The default for every existing board. A magnetometer-shaped hole must not
/// turn into a zero-vector heading reference.
TEST_F(SubsystemTest, WithoutAMagnetometerFusionStaysSixAxis)
{
    build(defaultConfig());
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.updateCount, 1);
    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_FALSE(subsystem->state().magnetometerFused);
}

TEST_F(SubsystemTest, WithoutAMagnetometerThePublishedFieldStaysZero)
{
    build(defaultConfig());
    subsystem->tick(0.002f);

    const ImuState state = subsystem->state();
    EXPECT_FLOAT_EQ(state.mag_ut.x, 0.0f);
    EXPECT_FLOAT_EQ(state.mag_ut.y, 0.0f);
    EXPECT_FLOAT_EQ(state.mag_ut.z, 0.0f);
}

TEST_F(SubsystemTest, AMagnetometerOnTheBoardSwitchesFusionToNineAxis)
{
    buildWithMagnetometer(defaultConfig());
    settleMagnetometer();
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 1);
    EXPECT_EQ(estimator.updateCount, 0);
    EXPECT_TRUE(subsystem->state().magnetometerFused);

    EXPECT_FLOAT_EQ(estimator.lastMag.x, 20.0f);
    EXPECT_FLOAT_EQ(estimator.lastMag.z, 40.0f);
}

/// The field is a true vector sharing the IMU's mount, so it takes the same
/// transform as the accelerometer. Leaving it in sensor frame yields a heading
/// wrong by exactly the mounting rotation — which reads as a compass needing
/// calibration rather than as a bug.
TEST_F(SubsystemTest, TheMagneticFieldIsRotatedIntoBodyFrame)
{
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::PlusY, SignedAxis::MinusX, SignedAxis::PlusZ };
    buildWithMagnetometer(config);

    mag.field_ut = Vec3f{ 30.0f, 10.0f, 40.0f };
    settleMagnetometer();
    subsystem->tick(0.002f);

    EXPECT_FLOAT_EQ(estimator.lastMag.x,  10.0f);
    EXPECT_FLOAT_EQ(estimator.lastMag.y, -30.0f);
    EXPECT_FLOAT_EQ(estimator.lastMag.z,  40.0f);
}

/// applyMeasurement, not applyAngularRate: a mirrored mount must NOT flip the
/// field's sign the way it flips a gyro's. Using the pseudovector path would
/// invert the heading on exactly the boards that need it most.
TEST_F(SubsystemTest, AMirroredMountDoesNotNegateTheField)
{
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::MinusX, SignedAxis::PlusY, SignedAxis::PlusZ };
    buildWithMagnetometer(config);

    mag.field_ut = Vec3f{ 0.0f, 25.0f, 0.0f };
    settleMagnetometer();
    subsystem->tick(0.002f);

    EXPECT_FLOAT_EQ(estimator.lastMag.y, 25.0f)
        << "the determinant belongs to angular rate, not to a field measurement";
}

/// Decided per tick, not once at begin(). A magnetometer that dies in flight
/// must stop contributing on the next tick rather than pin the heading to
/// wherever it was pointing when it failed.
TEST_F(SubsystemTest, AMagnetometerThatGoesUnhealthyDegradesToSixAxis)
{
    buildWithMagnetometer(defaultConfig());
    settleMagnetometer();
    subsystem->tick(0.002f);
    ASSERT_EQ(estimator.magUpdateCount, 1);

    mag.healthValue = device::SensorHealth::Failed;
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 1) << "no further nine-axis updates";
    EXPECT_EQ(estimator.updateCount, 1) << "dropped on the very next tick";
    EXPECT_FALSE(subsystem->state().magnetometerFused);
}

TEST_F(SubsystemTest, AFailedMagnetometerReadDegradesToSixAxis)
{
    buildWithMagnetometer(defaultConfig());

    mag.readResult = Status::IoError;
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_EQ(estimator.updateCount, 1);
}

/// An exactly-zero field is what an unconfigured part returns, and it is also
/// what a filter divides by when it normalises the vector to a direction.
TEST_F(SubsystemTest, AZeroFieldIsTreatedAsNoReadingAtAll)
{
    buildWithMagnetometer(defaultConfig());

    mag.field_ut = Vec3f{ 0.0f, 0.0f, 0.0f };
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_EQ(estimator.updateCount, 1);
    EXPECT_FALSE(subsystem->state().magnetometerFused);
}

/// Feeding the low-pass zeros on a bad read would walk the field towards the
/// origin and eventually under the rejection threshold, turning one dropped
/// read into a permanent loss of heading.
TEST_F(SubsystemTest, ABadReadDoesNotDragTheFilterTowardsZero)
{
    auto config = defaultConfig();
    config.magAlpha = 0.5f;          // enough smoothing for a zero to show
    buildWithMagnetometer(config);

    mag.field_ut = Vec3f{ 40.0f, 0.0f, 0.0f };
    for (int i = 0; i < 20; ++i) { subsystem->tick(0.002f); }
    const float settled = subsystem->state().mag_ut.x;
    ASSERT_GT(settled, 30.0f);

    mag.readResult = Status::IoError;
    for (int i = 0; i < 20; ++i) { subsystem->tick(0.002f); }

    EXPECT_FLOAT_EQ(subsystem->state().mag_ut.x, settled)
        << "the last good field is held, not decayed";
}

/// Sampled on its own rate like every other part: 100 Hz inside a 500 Hz tick
/// is one burst every five ticks.
TEST_F(SubsystemTest, TheMagnetometerIsSampledAtItsNativeRate)
{
    buildWithMagnetometer(defaultConfig());

    for (int i = 0; i < 50; ++i) { subsystem->tick(0.002f); }

    EXPECT_EQ(mag.sampleCount, 10);
    EXPECT_EQ(mag.readCount, 50) << "read is cached and happens every tick";
}

/// The published field is what was fused, in body frame — not the raw sensor
/// reading. Publishing the raw one would make the log disagree with the filter.
TEST_F(SubsystemTest, ThePublishedFieldMatchesWhatWasFused)
{
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::PlusZ, SignedAxis::PlusY, SignedAxis::MinusX };
    buildWithMagnetometer(config);

    mag.field_ut = Vec3f{ 12.0f, 34.0f, 56.0f };
    settleMagnetometer();
    subsystem->tick(0.002f);

    const ImuState state = subsystem->state();
    EXPECT_FLOAT_EQ(state.mag_ut.x, estimator.lastMag.x);
    EXPECT_FLOAT_EQ(state.mag_ut.y, estimator.lastMag.y);
    EXPECT_FLOAT_EQ(state.mag_ut.z, estimator.lastMag.z);
}

/// The shipping default. A magnetometer being fitted is a board fact; trusting
/// it in the fusion loop is a separate judgement, and today the answer is no.
/// Every other test in this file opts in explicitly.
TEST_F(SubsystemTest, MagnetometerFusionIsOffUnlessExplicitlyEnabled)
{
    auto config = defaultConfig();
    config.fuseMagnetometer = false;   // i.e. InertialSubsystem::Config's default
    buildWithMagnetometer(config);

    for (int i = 0; i < kEngageTicks * 2; ++i) { subsystem->tick(0.002f); }

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_FALSE(subsystem->state().magnetometerFused);
}

/// ...but the part is still sampled, transformed, filtered and published. That
/// is the whole point: the magnetometer is instrumentation before it is a
/// control input, and it cannot be evaluated on an airframe without data.
TEST_F(SubsystemTest, ADisabledMagnetometerIsStillSampledAndPublished)
{
    auto config = defaultConfig();
    config.fuseMagnetometer = false;
    config.axes = AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };
    buildWithMagnetometer(config);

    mag.field_ut = Vec3f{ 20.0f, 30.0f, 40.0f };
    for (int i = 0; i < 10; ++i) { subsystem->tick(0.002f); }

    EXPECT_GT(mag.sampleCount, 0) << "the bus burst still happens";

    const ImuState state = subsystem->state();
    EXPECT_FLOAT_EQ(state.mag_ut.x,  20.0f);
    EXPECT_FLOAT_EQ(state.mag_ut.y, -30.0f) << "still transformed into body frame";
    EXPECT_FLOAT_EQ(state.mag_ut.z,  40.0f);
}

// ── Engage delay ────────────────────────────────────────────────────────────
//
// Asymmetric by design: dropping the magnetometer is immediate, picking it back
// up waits. Madgwick normalises the combined accelerometer+magnetometer
// gradient as one vector, so a large magnetic residual steals correction
// authority from the accelerometer while it slews the heading in. Roll and
// pitch are what the control loops fly on; yaw is not in the control path at
// all today. A magnetometer flickering at tick rate would therefore repeatedly
// weaken the correction that matters, for the benefit of one that nothing uses.

TEST_F(SubsystemTest, NineAxisFusionDoesNotEngageOnTheFirstGoodTick)
{
    buildWithMagnetometer(defaultConfig());
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_EQ(estimator.updateCount, 1);
    EXPECT_FALSE(subsystem->state().magnetometerFused);
}

TEST_F(SubsystemTest, NineAxisFusionEngagesAfterOneSecondOfGoodReadings)
{
    buildWithMagnetometer(defaultConfig());

    for (int i = 0; i < kEngageTicks - 1; ++i) { subsystem->tick(0.002f); }
    EXPECT_EQ(estimator.magUpdateCount, 0) << "still counting";

    subsystem->tick(0.002f);
    EXPECT_EQ(estimator.magUpdateCount, 1) << "engaged on the 500th good tick";
    EXPECT_TRUE(subsystem->state().magnetometerFused);
}

/// The delay is a duration, not a tick count. A board ticking at 250 Hz must
/// wait 250 ticks for the same one second, not 500.
TEST_F(SubsystemTest, TheEngageDelayScalesWithTheTaskRate)
{
    auto config = defaultConfig();
    config.taskRate_hz = 250;
    buildWithMagnetometer(config);

    for (int i = 0; i < 249; ++i) { subsystem->tick(0.004f); }
    EXPECT_EQ(estimator.magUpdateCount, 0);

    subsystem->tick(0.004f);
    EXPECT_EQ(estimator.magUpdateCount, 1);
}

/// One bad tick costs the full delay again. That is the safe direction: the
/// cost of waiting is a heading nothing steers by, and the cost of chattering
/// is roll and pitch, which everything steers by.
TEST_F(SubsystemTest, ASingleBadReadingRestartsTheEngageDelay)
{
    buildWithMagnetometer(defaultConfig());
    settleMagnetometer();

    mag.readResult = Status::IoError;
    subsystem->tick(0.002f);
    mag.readResult = Status::Ok;

    for (int i = 0; i < kEngageTicks - 1; ++i) { subsystem->tick(0.002f); }
    EXPECT_EQ(estimator.magUpdateCount, 0) << "the streak restarted from zero";

    subsystem->tick(0.002f);
    EXPECT_EQ(estimator.magUpdateCount, 1);
}

/// A magnetometer alternating good/bad every tick must never fuse — this is the
/// case the delay exists for, and the one a naive per-tick decision gets wrong.
TEST_F(SubsystemTest, AFlickeringMagnetometerNeverEngages)
{
    buildWithMagnetometer(defaultConfig());

    for (int i = 0; i < 4000; ++i)   // eight times the engage delay
    {
        mag.readResult = (i % 2 == 0) ? Status::Ok : Status::IoError;
        subsystem->tick(0.002f);
    }

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_EQ(estimator.updateCount, 4000) << "every tick took the six-axis path";
}

/// The streak saturates rather than running up to 65535 and wrapping — a wrap
/// would drop fusion for a tenth of a second every couple of minutes.
TEST_F(SubsystemTest, TheEngageCounterSaturatesRatherThanWrapping)
{
    buildWithMagnetometer(defaultConfig());

    for (int i = 0; i < 100000; ++i) { subsystem->tick(0.002f); }

    EXPECT_TRUE(subsystem->state().magnetometerFused)
        << "fusion dropped out, so the streak counter wrapped";
}

/// resetFilters() is called after calibration. The field held in the low-pass
/// is discarded there, so the delay must restart too — engaging against a
/// freshly-reset filter is exactly the cold-field case the wait avoids.
TEST_F(SubsystemTest, ResetFiltersAlsoRestartsTheEngageDelay)
{
    buildWithMagnetometer(defaultConfig());
    settleMagnetometer();

    subsystem->resetFilters();
    subsystem->tick(0.002f);

    EXPECT_EQ(estimator.magUpdateCount, 0);
    EXPECT_EQ(estimator.updateCount, 1);
}

// ── The ordering that matters ───────────────────────────────────────────────

/**
 * ADR-033. Offsets are stored in SENSOR frame and must be subtracted BEFORE the
 * axis transform. With a Y-negating mount and a +0.2 g Y offset:
 *
 *   correct:   T(raw - offset) = -(0.5 - 0.2) = -0.3
 *   inverted:  T(raw) - offset =  -0.5 - 0.2  = -0.7
 *
 * The wrong order does not merely fail to remove the bias, it adds it again.
 */
TEST_F(SubsystemTest, OffsetsAreSubtractedBeforeTheAxisTransform)
{
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };
    build(config);

    InertialOffsets offsets;
    offsets.accel_g = Vec3f{ 0.0f, 0.2f, 0.0f };
    subsystem->setOffsets(offsets);

    imu.accel_g = Vec3f{ 0.0f, 0.5f, 0.0f };
    subsystem->tick(0.002f);

    EXPECT_NEAR(estimator.lastAccel.y, -0.3f, 1e-5f)
        << "transform-then-subtract would give -0.7 and double the bias";
}

TEST_F(SubsystemTest, GyroUsesAngularRateTransformNotMeasurement)
{
    // A mirrored map (determinant -1) distinguishes the two: a pseudovector
    // picks up the determinant, a measurement does not.
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };
    ASSERT_TRUE(config.axes.isMirrored());
    build(config);

    imu.accel_g  = Vec3f{ 1.0f, 1.0f, 1.0f };
    imu.gyro_dps = Vec3f{ 1.0f, 1.0f, 1.0f };
    subsystem->tick(0.002f);

    // Measurement: only Y flips.
    EXPECT_NEAR(estimator.lastAccel.x,  1.0f, 1e-5f);
    EXPECT_NEAR(estimator.lastAccel.y, -1.0f, 1e-5f);

    // Angular rate: the whole thing is additionally multiplied by det = -1.
    EXPECT_NEAR(estimator.lastGyro.x, -1.0f, 1e-5f);
    EXPECT_NEAR(estimator.lastGyro.y,  1.0f, 1e-5f);
    EXPECT_NEAR(estimator.lastGyro.z, -1.0f, 1e-5f);
}

// ── Rate-aware sampling ─────────────────────────────────────────────────────

TEST_F(SubsystemTest, SlowSensorsAreSampledLessOften)
{
    build(defaultConfig());   // 500 Hz task, 25 Hz baro -> divider 20

    for (int i = 0; i < 100; ++i) { subsystem->tick(0.002f); }

    EXPECT_EQ(imu.sampleCount, 100) << "the IMU runs at the task rate";
    EXPECT_EQ(baro.sampleCount, 5)
        << "reading a 25 Hz part 100 times would burn bus time for repeat conversions";
}

/**
 * The barometer is READ at the rate it CONVERTS, not at some separately
 * configured rate.
 *
 * These were two independent numbers and they disagreed: a 25 Hz baro at 500 Hz
 * is sampled every 20 ticks, but the old `baroDecimation` read it every 10 — so
 * every other read returned the identical cached conversion, and the climb-rate
 * derivative divided by the READ interval instead of the CONVERSION interval,
 * overstating vertical speed by the ratio between them.
 */
TEST_F(SubsystemTest, BarometerIsReadAtItsConversionRateNotFaster)
{
    baro.rate_hz = 25;              // 500 / 25 = every 20 ticks
    build(defaultConfig());

    for (int i = 0; i < 100; ++i) { subsystem->tick(0.002f); }

    EXPECT_EQ(baro.sampleCount, 5) << "sampled at its own rate";
    EXPECT_EQ(baro.readCount, 5)
        << "and read exactly as often - a read without a new conversion feeds "
           "the altitude filter a duplicate and mis-scales the derivative";
}

TEST_F(SubsystemTest, SensorFasterThanTheTaskIsSampledEveryTick)
{
    imu.rate_hz = 1000;
    build(defaultConfig());

    for (int i = 0; i < 10; ++i) { subsystem->tick(0.002f); }
    EXPECT_EQ(imu.sampleCount, 10);
}

// ── Publication ─────────────────────────────────────────────────────────────

TEST_F(SubsystemTest, PublishesAfterEveryTick)
{
    build(defaultConfig());
    clock.advanceMs(1234);

    imu.accel_g = Vec3f{ 0.1f, 0.2f, 0.9f };
    subsystem->tick(0.002f);

    const ImuState state = subsystem->state();
    EXPECT_NEAR(state.accel_g.x, 0.1f, 1e-5f);
    EXPECT_NEAR(state.euler_deg.roll, 1.0f, 1e-5f);
    EXPECT_NE(state.time.time_since_epoch().count(), 0);
    EXPECT_TRUE(state.healthy);
}

TEST_F(SubsystemTest, SelectionStateReachesTheSnapshot)
{
    build(defaultConfig());
    subsystem->tick(0.002f);

    // Always index 0 today; published so a flash log can answer the question at
    // all once redundancy exists.
    EXPECT_EQ(subsystem->state().selection.activeAccel, 0);
    EXPECT_EQ(subsystem->state().selection.switchCount, 0u);
}

// ── Health ──────────────────────────────────────────────────────────────────

TEST_F(SubsystemTest, SustainedReadFailuresMarkItUnhealthy)
{
    build(defaultConfig());
    subsystem->tick(0.002f);
    ASSERT_TRUE(subsystem->healthy());

    imu.readResult = Status::IoError;
    for (int i = 0; i < 20; ++i) { subsystem->tick(0.002f); }

    EXPECT_FALSE(subsystem->healthy())
        << "stale data is finite and in range, so only the read result reveals it";
    EXPECT_FALSE(subsystem->state().healthy);
}

TEST_F(SubsystemTest, ASingleGlitchDoesNotLatchUnhealthy)
{
    build(defaultConfig());
    subsystem->tick(0.002f);

    imu.readResult = Status::IoError;
    subsystem->tick(0.002f);
    imu.readResult = Status::Ok;
    subsystem->tick(0.002f);

    EXPECT_TRUE(subsystem->healthy()) << "one NAK must not ground the aircraft";
}

TEST_F(SubsystemTest, ImplausibleReadingsCountAsFailures)
{
    build(defaultConfig());
    imu.accel_g = Vec3f{ 500.0f, 0.0f, 0.0f };   // far beyond any real manoeuvre

    for (int i = 0; i < 20; ++i) { subsystem->tick(0.002f); }
    EXPECT_FALSE(subsystem->healthy());
}

// ── Calibration integration ─────────────────────────────────────────────────

TEST_F(SubsystemTest, CalibrationIsFedRawSensorFrameValues)
{
    auto config = defaultConfig();
    config.axes = AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };
    build(config);

    ASSERT_EQ(subsystem->calibration().request(CalibrationService::Kind::Inertial), Status::Ok);

    imu.accel_g  = Vec3f{ 0.0f, 0.3f, 1.0f };
    imu.gyro_dps = Vec3f{ 0.0f, 0.5f, 0.0f };

    for (int i = 0; i < 6000; ++i)
    {
        clock.advanceMs(2);
        subsystem->tick(0.002f);
    }

    ASSERT_EQ(subsystem->calibration().state(), CalibrationService::State::Complete);
    const auto offsets = subsystem->calibration().inertialOffsets();

    // Sensor-frame, so the Y values are the raw readings — NOT negated by the
    // mount. Feeding post-transform values here would store offsets that then
    // get subtracted in the wrong frame.
    EXPECT_NEAR(offsets.accel_g.y,  0.3f, 1e-3f);
    EXPECT_NEAR(offsets.gyro_dps.y, 0.5f, 1e-3f);
}

// ── Degraded hardware ───────────────────────────────────────────────────────

TEST_F(SubsystemTest, RunsWithNoSensorsFittedAtAll)
{
    std::array<device::Sensor*, 0>        noDevices{};
    std::array<device::Accelerometer*, 0> noAccels{};
    std::array<device::Gyroscope*, 0>     noGyros{};
    std::array<device::Barometer*, 0>     noBaros{};

    FirstHealthySelector emptySelector{ noAccels, noGyros, noBaros };
    InertialSubsystem    bare{ InertialSubsystem::Dependencies{ noDevices, emptySelector,
                                                               estimator, clock, scheduler,
                                                               watchdog, settings } };
    bare.configure(defaultConfig());

    // The spare bench board has no IMU. This must not fault.
    for (int i = 0; i < 10; ++i) { bare.tick(0.002f); }

    EXPECT_FALSE(bare.healthy()) << "no sensor is not healthy, but it is not a crash either";
}

TEST_F(SubsystemTest, ResetFiltersClearsHistoryAndHealth)
{
    build(defaultConfig());
    imu.readResult = Status::IoError;
    for (int i = 0; i < 20; ++i) { subsystem->tick(0.002f); }
    ASSERT_FALSE(subsystem->healthy());

    subsystem->resetFilters();
    EXPECT_TRUE(subsystem->healthy());
}

} // namespace
