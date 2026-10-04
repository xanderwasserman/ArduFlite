/**
 * test_madgwick_estimator.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Unit tests for the own Madgwick implementation (Phase 9).
 *
 * The log replay in test_log_replay.cpp proves the filter tracks one real
 * flight. It cannot prove much about inputs that flight never contained:
 * degenerate vectors, free fall, a stationary aircraft converging from a bad
 * initial guess, or the magnetometer path — the logs have no magnetometer
 * column because no board carried one.
 */
#include <gtest/gtest.h>

#include <cmath>

#include "src/estimation/MadgwickEstimator.h"

using namespace arduflite;
using namespace arduflite::estimation;

namespace {

constexpr Vec3f kStill{ 0.0f, 0.0f, 0.0f };
/// Level: the accelerometer reads +1 g on Z, matching the filter's identity
/// attitude. Sign convention is pinned by ConvergesToLevelFromATiltedStart.
constexpr Vec3f kLevel1g{ 0.0f, 0.0f, 1.0f };

/// Run the filter to steady state at 500 Hz.
void settle(MadgwickEstimator& filter, const Vec3f& accel_g, int steps = 20000)
{
    for (int i = 0; i < steps; ++i) { filter.update(kStill, accel_g, 0.002f); }
}

// ── Degenerate input ────────────────────────────────────────────────────────

/**
 * The reason this implementation guards the gradient explicitly.
 *
 * When the measured acceleration exactly matches the predicted gravity the
 * objective function is zero, so its gradient is zero, and the correction step
 * divides by that length. `1/sqrt(0)` is infinity and `0 * inf` is NaN, and a
 * NaN quaternion never recovers — so the guard is not an optimisation.
 *
 * Perfectly consistent input is rare from a real sensor and routine from a
 * synthetic one, which is what host_sim and every test here feed.
 */
TEST(MadgwickEstimator, PerfectlyConsistentInputDoesNotProduceNaN)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    // Identity attitude predicts exactly this vector, so the residual is zero.
    for (int i = 0; i < 100; ++i) { filter.update(kStill, kLevel1g, 0.002f); }

    const Quaternion q = filter.orientation();
    ASSERT_FALSE(std::isnan(q.w)) << "a NaN quaternion never recovers";
    ASSERT_FALSE(std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z));

    const EulerAnglesDeg euler = filter.euler_deg();
    EXPECT_FALSE(std::isnan(euler.roll) || std::isnan(euler.pitch) || std::isnan(euler.yaw));
    EXPECT_NEAR(euler.roll,  0.0f, 0.01f);
    EXPECT_NEAR(euler.pitch, 0.0f, 0.01f);
}

/// Free fall, or a dead accelerometer. Neither carries attitude information, so
/// the filter must coast on the gyro rather than divide by zero.
TEST(MadgwickEstimator, ZeroAccelerationCoastsOnTheGyroscope)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    for (int i = 0; i < 500; ++i)
    {
        filter.update(Vec3f{ 0.0f, 0.0f, 90.0f }, Vec3f{ 0.0f, 0.0f, 0.0f }, 0.002f);
    }

    const Quaternion q = filter.orientation();
    ASSERT_FALSE(std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z));

    // 90 deg/s for 1 s about Z is 90 degrees of yaw, uncorrected by anything.
    EXPECT_NEAR(filter.euler_deg().yaw, 90.0f, 1.0f);
}

TEST(MadgwickEstimator, SurvivesAZeroLengthOrientationBeingForcedOnIt)
{
    MadgwickEstimator filter;
    filter.setOrientation(Quaternion{ 0.0f, 0.0f, 0.0f, 0.0f });

    const Quaternion q = filter.orientation();
    EXPECT_FLOAT_EQ(q.w, 1.0f) << "falls back to identity rather than staying degenerate";
    EXPECT_FLOAT_EQ(q.x, 0.0f);
}

/// Every step afterwards assumes a unit quaternion, so a caller handing over
/// an unnormalised one must not be able to break the filter.
TEST(MadgwickEstimator, SetOrientationNormalises)
{
    MadgwickEstimator filter;
    filter.setOrientation(Quaternion{ 2.0f, 0.0f, 0.0f, 0.0f });

    const Quaternion q = filter.orientation();
    const float length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    EXPECT_NEAR(length, 1.0f, 1e-6f);
}

/// The other deliberate divergence: the reference cached Euler angles behind a
/// flag that setQuaternion() did not clear, so they could lag the quaternion.
TEST(MadgwickEstimator, EulerAnglesFollowSetOrientationImmediately)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    // 90 degrees of roll.
    const float halfAngle = 45.0f * 3.14159265f / 180.0f;
    filter.setOrientation(Quaternion{ std::cos(halfAngle), std::sin(halfAngle), 0.0f, 0.0f });

    EXPECT_NEAR(filter.euler_deg().roll, 90.0f, 0.01f)
        << "read back before any update() - must not be stale";
}

// ── Convergence ─────────────────────────────────────────────────────────────

/// The accelerometer correction must pull a wrong attitude back to level, and
/// must do so with the right sign. A sign error here would fly inverted.
TEST(MadgwickEstimator, ConvergesToLevelFromATiltedStart)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    const float halfAngle = 20.0f * 3.14159265f / 180.0f;   // 40 degrees of roll
    filter.setOrientation(Quaternion{ std::cos(halfAngle), std::sin(halfAngle), 0.0f, 0.0f });
    ASSERT_NEAR(filter.euler_deg().roll, 40.0f, 0.1f);

    settle(filter, kLevel1g);

    EXPECT_NEAR(filter.euler_deg().roll,  0.0f, 0.5f);
    EXPECT_NEAR(filter.euler_deg().pitch, 0.0f, 0.5f);
}

/**
 * The accelerometer-to-roll sign, derived rather than fitted.
 *
 * At equilibrium the objective function is zero, so its second row gives
 * `ay = 2(q0q1 + q2q3)`, and roll is `atan2(q0q1 + q2q3, ...)`. For small
 * angles that is `roll ~ ay`: **positive Y acceleration means positive roll**.
 *
 * Sign conventions are the single most expensive thing to get wrong here and
 * the cheapest to get wrong silently, so this is stated as a derivation and not
 * as a number that happened to make the test pass. The six-orientation bench
 * check is what confirms it against the physical airframe.
 */
TEST(MadgwickEstimator, ConvergesToAHeldRollAngle)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    const float roll_rad = 30.0f * 3.14159265f / 180.0f;
    settle(filter, Vec3f{ 0.0f, std::sin(roll_rad), std::cos(roll_rad) });

    EXPECT_NEAR(filter.euler_deg().roll, 30.0f, 0.5f);
    EXPECT_NEAR(filter.euler_deg().pitch, 0.0f, 0.5f);
}

/// The mirror of the above: the sign must actually follow the input, not be a
/// constant that happens to match one case.
TEST(MadgwickEstimator, RollFollowsTheSignOfLateralAcceleration)
{
    const float roll_rad = 30.0f * 3.14159265f / 180.0f;

    MadgwickEstimator filter;
    filter.begin(500.0f);
    settle(filter, Vec3f{ 0.0f, -std::sin(roll_rad), std::cos(roll_rad) });

    EXPECT_NEAR(filter.euler_deg().roll, -30.0f, 0.5f);
}

/// Same derivation on the first row: `ax = 2(q1q3 - q0q2)` at equilibrium and
/// `pitch = asin(-2(q1q3 - q0q2))`, so **positive X acceleration means NEGATIVE
/// pitch**. The opposite sign to roll, which is exactly why it is worth pinning.
TEST(MadgwickEstimator, ConvergesToAHeldPitchAngle)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    const float pitch_rad = 25.0f * 3.14159265f / 180.0f;
    settle(filter, Vec3f{ -std::sin(pitch_rad), 0.0f, std::cos(pitch_rad) });

    EXPECT_NEAR(filter.euler_deg().pitch, 25.0f, 0.5f);
    EXPECT_NEAR(filter.euler_deg().roll,  0.0f, 0.5f);
}

/// Gyro integration alone, with the accelerometer consistent so it contributes
/// no correction. Checks the sign and scale of the rate path.
TEST(MadgwickEstimator, IntegratesGyroRateWithTheCorrectSignAndScale)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    // 30 deg/s about Z for two seconds. Z is unconstrained by gravity, so the
    // accelerometer neither helps nor fights.
    for (int i = 0; i < 1000; ++i) { filter.update(Vec3f{ 0, 0, 30.0f }, kLevel1g, 0.002f); }

    EXPECT_NEAR(filter.euler_deg().yaw, 60.0f, 1.0f);
}

// ── Conventions that must not drift ─────────────────────────────────────────

/**
 * Yaw is a TRUE angle, signed, with identity attitude reporting zero.
 *
 * Two things depend on it. The CRSF attitude frame encodes attitude as int16
 * radians x 10000, which saturates at +-187.7 degrees — an unsigned 0..360 yaw
 * overflows it for most of a turn and the radio displays a heading sweeping the
 * wrong way. And MagneticHeading's compass heading is only comparable to yaw if
 * yaw is not offset from it.
 */
TEST(MadgwickEstimator, YawIsSignedAndZeroAtIdentity)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    EXPECT_NEAR(filter.euler_deg().yaw, 0.0f, 0.01f);
}

/// The whole signed range has to survive the CRSF int16 encoding.
TEST(MadgwickEstimator, YawStaysWithinTheCrsfEncodableRange)
{
    for (float yawDeg : { -179.0f, -90.0f, 0.0f, 90.0f, 179.0f })
    {
        MadgwickEstimator filter;
        const float half = yawDeg * 3.14159265f / 180.0f * 0.5f;
        filter.setOrientation(Quaternion{ std::cos(half), 0.0f, 0.0f, std::sin(half) });

        const float reported = filter.euler_deg().yaw;
        EXPECT_NEAR(reported, yawDeg, 0.01f);

        const float encoded = reported * 3.14159265f / 180.0f * 10000.0f;
        EXPECT_LT(std::fabs(encoded), 32767.0f)
            << "yaw " << yawDeg << " overflows the CRSF int16 attitude field";
    }
}

TEST(MadgwickEstimator, RollIsSignedAroundZero)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);
    settle(filter, Vec3f{ 0.0f, -0.30f, 0.95f });

    EXPECT_LT(filter.euler_deg().roll, 0.0f)
        << "roll must be signed, not wrapped into 0..360";
}

/// A quaternion one ulp off unit length can push asin's argument outside
/// [-1, 1] at exactly 90 degrees of pitch, and asin returns NaN for that.
TEST(MadgwickEstimator, VerticalPitchDoesNotProduceNaN)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    const float halfAngle = 45.0f * 3.14159265f / 180.0f;   // nose straight up
    filter.setOrientation(Quaternion{ std::cos(halfAngle), 0.0f, std::sin(halfAngle), 0.0f });

    const EulerAnglesDeg euler = filter.euler_deg();
    ASSERT_FALSE(std::isnan(euler.pitch));
    EXPECT_NEAR(euler.pitch, 90.0f, 0.01f);
}

// ── Beta ────────────────────────────────────────────────────────────────────

TEST(MadgwickEstimator, DefaultBetaMatchesTheImplementationItReplaces)
{
    MadgwickEstimator filter;
    EXPECT_FLOAT_EQ(filter.beta(), 0.1f)
        << "a board that switches implementations without touching config must "
           "behave the same";
}

/// Beta is the correction rate. A larger one must converge sooner — if it did
/// not, the gradient normalisation would be wrong and beta would not mean what
/// the tuning docs say it means.
TEST(MadgwickEstimator, LargerBetaConvergesFaster)
{
    const float halfAngle = 20.0f * 3.14159265f / 180.0f;
    const Quaternion tilted{ std::cos(halfAngle), std::sin(halfAngle), 0.0f, 0.0f };

    MadgwickEstimator slow, fast;
    slow.begin(500.0f); slow.setBeta(0.05f); slow.setOrientation(tilted);
    fast.begin(500.0f); fast.setBeta(0.40f); fast.setOrientation(tilted);

    for (int i = 0; i < 200; ++i)
    {
        slow.update(kStill, kLevel1g, 0.002f);
        fast.update(kStill, kLevel1g, 0.002f);
    }

    EXPECT_LT(std::fabs(fast.euler_deg().roll), std::fabs(slow.euler_deg().roll));
}

// ── Magnetometer path ───────────────────────────────────────────────────────
//
// Off by default (ADR-055) and absent from every flight log, so the replay
// cannot reach this code at all. These are the only tests it has.

TEST(MadgwickEstimator, NineAxisConvergesToAKnownHeading)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);
    filter.setBeta(0.3f);

    // Level, nose 90 degrees from magnetic north: the horizontal component of
    // the field lies along -Y in body axes.
    const Vec3f field{ 0.0f, -20.0f, 44.0f };

    for (int i = 0; i < 40000; ++i)
    {
        filter.updateWithMagnetometer(kStill, kLevel1g, field, 0.002f);
    }

    const EulerAnglesDeg euler = filter.euler_deg();
    EXPECT_NEAR(euler.roll,  0.0f, 1.0f) << "the field must not disturb roll";
    EXPECT_NEAR(euler.pitch, 0.0f, 1.0f) << "the field must not disturb pitch";

    EXPECT_NEAR(euler.yaw, 90.0f, 2.0f);
}

/// A zero field is not a reading. Falling back to six-axis is right; dividing
/// by its length is not.
TEST(MadgwickEstimator, NineAxisFallsBackToSixAxisForAZeroField)
{
    MadgwickEstimator nineAxis, sixAxis;
    nineAxis.begin(500.0f);
    sixAxis.begin(500.0f);

    for (int i = 0; i < 200; ++i)
    {
        nineAxis.updateWithMagnetometer(Vec3f{ 5.0f, 0, 0 }, kLevel1g,
                                        Vec3f{ 0.0f, 0.0f, 0.0f }, 0.002f);
        sixAxis.update(Vec3f{ 5.0f, 0, 0 }, kLevel1g, 0.002f);
    }

    const Quaternion a = nineAxis.orientation();
    const Quaternion b = sixAxis.orientation();
    ASSERT_FALSE(std::isnan(a.w));
    EXPECT_NEAR(a.w, b.w, 1e-6f);
    EXPECT_NEAR(a.x, b.x, 1e-6f);
}

TEST(MadgwickEstimator, NineAxisSurvivesAFieldParallelToGravity)
{
    MadgwickEstimator filter;
    filter.begin(500.0f);

    // Straight down: the horizontal reference collapses to zero length, so the
    // heading is undefined. It must degrade, not produce NaN.
    for (int i = 0; i < 500; ++i)
    {
        filter.updateWithMagnetometer(kStill, kLevel1g, Vec3f{ 0.0f, 0.0f, 50.0f }, 0.002f);
    }

    const Quaternion q = filter.orientation();
    EXPECT_FALSE(std::isnan(q.w) || std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z));
}

} // namespace
