/**
 * test_calibration_service.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for estimation::CalibrationService.
 *
 * Calibration accumulates INSIDE the sampling task, as a state machine driven
 * one tick at a time. That is what makes the failure paths — no samples, a dead
 * barometer, a request arriving mid-run — reachable from a host test at all,
 * rather than only on hardware by hand.
 */
#include <gtest/gtest.h>

#include "src/estimation/CalibrationService.h"

using namespace arduflite;
using arduflite::estimation::CalibrationService;

namespace {

using Clock = arduflite::hal::Clock;
using Kind  = CalibrationService::Kind;
using State = CalibrationService::State;

Clock::time_point at(std::int64_t ms)
{
    return Clock::time_point{ std::chrono::microseconds{ ms * 1000 } };
}

constexpr Vec3f kLevelAccel{ 0.0f, 0.0f, 1.0f };   ///< level, Z up
constexpr Vec3f kZeroGyro{ 0.0f, 0.0f, 0.0f };

/// Drive `count` ticks spread evenly across `spanMs`.
void run(CalibrationService& service, int count, std::int64_t spanMs,
         const Vec3f& accel = kLevelAccel, const Vec3f& gyro = kZeroGyro,
         float pressure = 1013.25f)
{
    for (int i = 0; i < count; ++i)
    {
        service.service(accel, gyro, pressure, at((spanMs * i) / count));
    }
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

TEST(CalibrationService, StartsIdleAndIgnoresTicks)
{
    CalibrationService service;
    EXPECT_EQ(service.state(), State::Idle);

    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(0));
    EXPECT_EQ(service.state(), State::Idle) << "must not accumulate unrequested";
}

TEST(CalibrationService, RequestBeginsOnTheNextTickNotImmediately)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    // The requesting task is not the sampling task; nothing has ticked yet.
    EXPECT_EQ(service.state(), State::Idle);

    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(0));
    EXPECT_EQ(service.state(), State::Running);
}

TEST(CalibrationService, SecondRequestWhileRunningIsRejected)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(0));
    ASSERT_EQ(service.state(), State::Running);

    EXPECT_EQ(service.request(Kind::Barometric), Status::Busy);
}

TEST(CalibrationService, CompletedResultMustBeAcknowledgedBeforeReRunning)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);
    run(service, 500, 9999);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10000));
    ASSERT_EQ(service.state(), State::Complete);

    EXPECT_EQ(service.request(Kind::Inertial), Status::Busy)
        << "overwriting an unread result would discard offsets the caller is about to store";

    service.acknowledge();
    EXPECT_EQ(service.state(), State::Idle);
    EXPECT_EQ(service.request(Kind::Inertial), Status::Ok);
}

TEST(CalibrationService, CancelAbandonsARunningCalibration)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);
    run(service, 100, 5000);
    ASSERT_EQ(service.state(), State::Running);

    service.cancel();
    EXPECT_EQ(service.state(), State::Idle);

    // Ticks after a cancel must not resurrect the run.
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(11000));
    EXPECT_EQ(service.state(), State::Idle);
}

// ── Inertial results ────────────────────────────────────────────────────────

TEST(CalibrationService, GyroBiasIsTheAverageReading)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    const Vec3f bias{ 0.5f, -1.25f, 2.0f };
    run(service, 500, 9999, kLevelAccel, bias);
    service.service(kLevelAccel, bias, 1013.25f, at(10000));

    ASSERT_EQ(service.state(), State::Complete);
    const auto offsets = service.inertialOffsets();
    EXPECT_NEAR(offsets.gyro_dps.x,  0.5f,  1e-4f);
    EXPECT_NEAR(offsets.gyro_dps.y, -1.25f, 1e-4f);
    EXPECT_NEAR(offsets.gyro_dps.z,  2.0f,  1e-4f);
}

/**
 * The single most consequential line in this class: Z has gravity subtracted,
 * X and Y do not. Getting it wrong leaves a 1 g offset on Z, and the aircraft
 * believes it is in freefall while sitting on the bench.
 */
TEST(CalibrationService, AccelOffsetRemovesGravityFromZOnly)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    const Vec3f reading{ 0.02f, -0.03f, 1.04f };   // level, with small biases
    run(service, 500, 9999, reading);
    service.service(reading, kZeroGyro, 1013.25f, at(10000));

    ASSERT_EQ(service.state(), State::Complete);
    const auto offsets = service.inertialOffsets();
    EXPECT_NEAR(offsets.accel_g.x,  0.02f,  1e-4f);
    EXPECT_NEAR(offsets.accel_g.y, -0.03f,  1e-4f);
    EXPECT_NEAR(offsets.accel_g.z,  0.04f,  1e-4f) << "1.04 g measured minus 1 g of gravity";
}

TEST(CalibrationService, PerfectlyCalibratedSensorYieldsZeroOffsets)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);
    run(service, 500, 9999);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10000));

    const auto offsets = service.inertialOffsets();
    EXPECT_NEAR(offsets.accel_g.z, 0.0f, 1e-5f);
    EXPECT_NEAR(offsets.gyro_dps.x, 0.0f, 1e-5f);
}

// ── Barometric ──────────────────────────────────────────────────────────────

TEST(CalibrationService, ReferencePressureIsTheAverage)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Barometric), Status::Ok);
    run(service, 500, 9999, kLevelAccel, kZeroGyro, 990.0f);
    service.service(kLevelAccel, kZeroGyro, 990.0f, at(10000));

    ASSERT_EQ(service.state(), State::Complete);
    EXPECT_NEAR(service.referencePressure_hpa(), 990.0f, 1e-3f);
}

/// A dead barometer reports nothing usable. Averaging in zeros would drag the
/// ground reference down and offset every altitude for the whole flight.
TEST(CalibrationService, DeadBarometerFailsRatherThanAveragingZeros)
{
    CalibrationService service;

    // Establish a good reference first, so the assertion below is about the
    // failed run PRESERVING it rather than about the initial value.
    ASSERT_EQ(service.request(Kind::Barometric), Status::Ok);
    run(service, 500, 9999, kLevelAccel, kZeroGyro, 990.0f);
    service.service(kLevelAccel, kZeroGyro, 990.0f, at(10000));
    ASSERT_EQ(service.state(), State::Complete);
    ASSERT_NEAR(service.referencePressure_hpa(), 990.0f, 1e-3f);
    service.acknowledge();

    // Now the barometer dies and a second run is attempted.
    ASSERT_EQ(service.request(Kind::Barometric), Status::Ok);
    for (int i = 0; i < 500; ++i)
    {
        service.service(kLevelAccel, kZeroGyro, 0.0f, at(20000 + (9999 * i) / 500));
    }
    service.service(kLevelAccel, kZeroGyro, 0.0f, at(30001));

    EXPECT_EQ(service.state(), State::Failed);
    EXPECT_EQ(service.lastError(), Status::IoError);
    EXPECT_NEAR(service.referencePressure_hpa(), 990.0f, 1e-3f)
        << "a failed run must leave the last good reference intact - overwriting it "
           "with zero would offset every altitude for the rest of the flight";
}

// ── Sample sufficiency ──────────────────────────────────────────────────────

TEST(CalibrationService, TooFewSamplesFailsEvenIfSomeArrived)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    // Ten ticks across the whole window: enough to be non-zero, nowhere near
    // enough to average meaningfully — "more than zero" is not enough.
    run(service, 10, 9999);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10000));

    EXPECT_EQ(service.state(), State::Failed);
    EXPECT_EQ(service.lastError(), Status::IoError);
}

TEST(CalibrationService, FailedResultAlsoNeedsAcknowledging)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);
    run(service, 5, 9999);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10000));
    ASSERT_EQ(service.state(), State::Failed);

    EXPECT_EQ(service.request(Kind::Inertial), Status::Busy);
    service.acknowledge();
    EXPECT_EQ(service.request(Kind::Inertial), Status::Ok);
}

// ── Progress ────────────────────────────────────────────────────────────────

TEST(CalibrationService, ProgressTracksTimeNotSampleCount)
{
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(0));
    EXPECT_EQ(service.progressPercent(), 0);

    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(5000));
    EXPECT_EQ(service.progressPercent(), 50)
        << "a stalled sensor must still show the run advancing to its timeout";

    run(service, 500, 9999);
    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10000));
    EXPECT_EQ(service.progressPercent(), 100);
}

TEST(CalibrationService, RunIsBoundedByTimeNotByTickCount)
{
    // 5000 ticks at 500 Hz is exactly the ten-second window. The service must
    // finish on elapsed time, so a task running slow does not extend it.
    CalibrationService service;
    ASSERT_EQ(service.request(Kind::Inertial), Status::Ok);

    for (int i = 0; i < 200; ++i)
    {
        service.service(kLevelAccel, kZeroGyro, 1013.25f, at(i * 25));   // 5 s of ticks
    }
    EXPECT_EQ(service.state(), State::Running) << "half the window elapsed";

    service.service(kLevelAccel, kZeroGyro, 1013.25f, at(10001));
    EXPECT_EQ(service.state(), State::Complete);
}

} // namespace
