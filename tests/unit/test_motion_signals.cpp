/**
 * test_motion_signals.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for estimation::MotionDetector.
 *
 * These drive the real MotionDetector, not a harness re-implementing its
 * threshold and debounce arithmetic. A test that duplicates the logic it checks
 * keeps passing happily while the real code diverges.
 *
 * What these signals drive: StateManagement uses launchDetected to move
 * PREFLIGHT → INFLIGHT and stableDetected to move INFLIGHT → LANDED. A false
 * launch arms the aircraft on the bench; a missed one leaves it disarmed on the
 * throw.
 */
#include <gtest/gtest.h>

#include "src/estimation/MotionDetector.h"

using namespace arduflite;
using arduflite::estimation::MotionDetector;

namespace {

using Clock = arduflite::hal::Clock;

/// Time base for the tests. Deliberately not zero: a detector that forgot to
/// seed its debounce windows would measure "since the epoch" and fire at once.
constexpr Clock::time_point kStart{ std::chrono::microseconds{ 5'000'000 } };

Clock::time_point at(std::int64_t ms)
{
    return kStart + std::chrono::microseconds{ ms * 1000 };
}

/// Accelerometer reading with a given squared magnitude, put entirely on Z.
Vec3f accelWithMagnitude(float g) { return Vec3f{ 0.0f, 0.0f, g }; }
Vec3f gyroWithMagnitude(float dps) { return Vec3f{ 0.0f, 0.0f, dps }; }

constexpr Vec3f kAtRest{ 0.0f, 0.0f, 1.0f };
constexpr Vec3f kNoRotation{ 0.0f, 0.0f, 0.0f };

// ── Launch detection ────────────────────────────────────────────────────────

TEST(MotionDetector, FirstTickNeverFires)
{
    MotionDetector detector;

    // Conditions that would satisfy both branches, on the very first call.
    const auto signals = detector.update(kAtRest, kNoRotation, at(0));

    EXPECT_FALSE(signals.launchDetected);
    EXPECT_FALSE(signals.stableDetected)
        << "an unseeded debounce window measures from the epoch and fires instantly";
}

TEST(MotionDetector, GyroOnlyLaunchRequiresDebounce)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    // 20 dps: above the 15 dps minimum, below the 150 dps maximum. Accelerometer
    // stays at rest, so this is the soft hand-launch path.
    const Vec3f gyro = gyroWithMagnitude(20.0f);

    EXPECT_FALSE(detector.update(kAtRest, gyro, at(10)).launchDetected);
    EXPECT_FALSE(detector.update(kAtRest, gyro, at(50)).launchDetected)
        << "debounce is strictly greater-than, so exactly 50 ms is not enough";
    EXPECT_TRUE(detector.update(kAtRest, gyro, at(51)).launchDetected);
}

TEST(MotionDetector, SustainedMotionMustBeContinuous)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    const Vec3f gyro = gyroWithMagnitude(20.0f);

    detector.update(kAtRest, gyro, at(40));                  // 40 ms accumulated
    detector.update(kAtRest, kNoRotation, at(45));           // condition breaks
    EXPECT_FALSE(detector.update(kAtRest, gyro, at(60)).launchDetected)
        << "the window must restart on the break, not resume";
    EXPECT_TRUE(detector.update(kAtRest, gyro, at(100)).launchDetected);
}

TEST(MotionDetector, TumblingIsNotALaunch)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    // 200 dps exceeds the 150 dps ceiling.
    const Vec3f tumbling = gyroWithMagnitude(200.0f);

    detector.update(kAtRest, tumbling, at(100));
    EXPECT_FALSE(detector.update(kAtRest, tumbling, at(500)).launchDetected)
        << "above the ceiling is a tumble, not a throw - no debounce should rescue it";
}

TEST(MotionDetector, AccelThresholdIsStrictlyOutsideTheBand)
{
    MotionDetector detector;

    // 1.10 g is exactly the threshold; the comparison is strict, so it must NOT
    // count as a throw. 1.11 g must.
    detector.resetTimers(at(0));
    detector.update(accelWithMagnitude(1.10f), kNoRotation, at(10));
    EXPECT_FALSE(detector.update(accelWithMagnitude(1.10f), kNoRotation, at(200)).launchDetected);

    detector.resetTimers(at(0));
    detector.update(accelWithMagnitude(1.11f), kNoRotation, at(10));
    EXPECT_TRUE(detector.update(accelWithMagnitude(1.11f), kNoRotation, at(200)).launchDetected);
}

TEST(MotionDetector, FreefallTriggersLaunchToo)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    // A drop reads BELOW 1 g. The threshold is a deviation in either direction,
    // which is what makes a discus or drop launch detectable.
    const Vec3f falling = accelWithMagnitude(0.5f);

    detector.update(falling, kNoRotation, at(10));
    EXPECT_TRUE(detector.update(falling, kNoRotation, at(200)).launchDetected);
}

// ── Stability detection ─────────────────────────────────────────────────────

TEST(MotionDetector, StableRequiresTwoFullSeconds)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    EXPECT_FALSE(detector.update(kAtRest, kNoRotation, at(1999)).stableDetected);
    EXPECT_TRUE(detector.update(kAtRest, kNoRotation, at(2000)).stableDetected)
        << "stability uses >=, unlike the launch window's >";
}

TEST(MotionDetector, StableAccelBoundsAreInclusive)
{
    MotionDetector detector;

    // Exactly 1.30 g is the boundary and must still count as stable — the loose
    // band exists to tolerate rough ground, so excluding the edge would block
    // the LANDED transition on a slope.
    detector.resetTimers(at(0));
    EXPECT_TRUE(detector.update(accelWithMagnitude(1.30f), kNoRotation, at(2500)).stableDetected);

    detector.resetTimers(at(0));
    EXPECT_FALSE(detector.update(accelWithMagnitude(1.31f), kNoRotation, at(2500)).stableDetected);
}

TEST(MotionDetector, StableGyroBoundIsExclusive)
{
    MotionDetector detector;

    detector.resetTimers(at(0));
    EXPECT_FALSE(detector.update(kAtRest, gyroWithMagnitude(2.0f), at(2500)).stableDetected)
        << "exactly at the threshold is not below it";

    detector.resetTimers(at(0));
    EXPECT_TRUE(detector.update(kAtRest, gyroWithMagnitude(1.99f), at(2500)).stableDetected);
}

TEST(MotionDetector, ResetTimersClearsBothSignals)
{
    MotionDetector detector;
    detector.resetTimers(at(0));
    ASSERT_TRUE(detector.update(kAtRest, kNoRotation, at(3000)).stableDetected);

    // Calibration suspends sampling. Without a reset the gap reads as satisfied
    // conditions and stability latches the moment sampling resumes.
    detector.resetTimers(at(10'000));
    EXPECT_FALSE(detector.signals().stableDetected);
    EXPECT_FALSE(detector.update(kAtRest, kNoRotation, at(11'000)).stableDetected)
        << "the window must restart from the reset, not from before the gap";
}

TEST(MotionDetector, LaunchAndStabilityAreMutuallyExclusiveInPractice)
{
    MotionDetector detector;
    detector.resetTimers(at(0));

    // Being thrown: well outside the stable accel band.
    const auto signals = detector.update(accelWithMagnitude(2.0f), gyroWithMagnitude(20.0f), at(3000));

    EXPECT_TRUE(signals.launchDetected);
    EXPECT_FALSE(signals.stableDetected);
}

} // namespace
