/**
 * test_control_types.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Phase 6B — the conversions between control-path quantities.
 *
 * The compiler now enforces that a rate cannot be assigned to a surface
 * command. What it cannot check is whether the CONVERSIONS are right, and those
 * are where the remaining risk lives: a wrong scale here reaches the servos.
 */
#include <gtest/gtest.h>

#include <type_traits>

#include "src/core/FlightTypes.h"

namespace {

// ── The type separation itself ──────────────────────────────────────────────

/**
 * The property the whole phase exists for. Before it, `actuatorCmd =
 * localRateSetpoint` compiled and drove every surface hard over on IMU failure
 * in RATE_MODE (ADR-037).
 */
TEST(ControlTypes, QuantitiesAreNotInterchangeable)
{
    static_assert(!std::is_assignable_v<AxisCommand&, AngularRateDps>,
                  "a rate must not be assignable to a surface demand");
    static_assert(!std::is_assignable_v<AxisCommand&, AttitudeDeg>,
                  "an attitude must not be assignable to a surface demand");
    static_assert(!std::is_assignable_v<AngularRateDps&, AttitudeDeg>,
                  "degrees and deg/s are different quantities");
    static_assert(!std::is_convertible_v<AttitudeDeg, AngularRateDps>,
                  "no implicit conversion in either direction");
    SUCCEED();
}

TEST(ControlTypes, AllThreeDefaultToNeutral)
{
    // A default-constructed command must be centred, not indeterminate: the
    // IMU-failure path relies on `= {}` producing neutral surfaces.
    EXPECT_FLOAT_EQ(AxisCommand{}.roll, 0.0f);
    EXPECT_FLOAT_EQ(AxisCommand{}.pitch, 0.0f);
    EXPECT_FLOAT_EQ(AxisCommand{}.yaw, 0.0f);
    EXPECT_FLOAT_EQ(AngularRateDps{}.roll, 0.0f);
    EXPECT_FLOAT_EQ(AttitudeDeg{}.yaw, 0.0f);
}

// ── toAxisCommand: the clamp ────────────────────────────────────────────────

TEST(ControlTypes, AxisCommandClampsToUnitRange)
{
    const AxisCommand c = toAxisCommand(2.5f, -3.0f, 0.5f);
    EXPECT_FLOAT_EQ(c.roll,  1.0f);
    EXPECT_FLOAT_EQ(c.pitch, -1.0f);
    EXPECT_FLOAT_EQ(c.yaw,   0.5f);
}

TEST(ControlTypes, AxisCommandPassesValuesAlreadyInRange)
{
    const AxisCommand c = toAxisCommand(-1.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(c.roll, -1.0f);
    EXPECT_FLOAT_EQ(c.pitch, 0.0f);
    EXPECT_FLOAT_EQ(c.yaw,   1.0f);
}

// ── rateToAxisCommand ──────────────────────────────────────────────────────

/**
 * The maxima are REQUIRED arguments on purpose. Done implicitly by assignment,
 * this conversion has no scale, and a 200 deg/s rate lands on the mixer as a
 * full-deflection command. There is no rate-to-command conversion without
 * knowing the range, so the signature refuses to let one be written.
 */
TEST(ControlTypes, RateScalesAgainstItsMaxima)
{
    const AngularRateDps rate{ 100.0f, -50.0f, 25.0f };
    const AxisCommand c = rateToAxisCommand(rate, 200.0f, 200.0f, 100.0f);

    EXPECT_FLOAT_EQ(c.roll,   0.5f);
    EXPECT_FLOAT_EQ(c.pitch, -0.25f);
    EXPECT_FLOAT_EQ(c.yaw,    0.25f);
}

TEST(ControlTypes, RateBeyondItsMaximumSaturatesRatherThanOverdriving)
{
    // A 300 deg/s demand against a 200 deg/s limit is full deflection, not 1.5.
    const AngularRateDps rate{ 300.0f, -300.0f, 0.0f };
    const AxisCommand c = rateToAxisCommand(rate, 200.0f, 200.0f, 200.0f);

    EXPECT_FLOAT_EQ(c.roll,   1.0f);
    EXPECT_FLOAT_EQ(c.pitch, -1.0f);
}

/// A zero or unset maximum must not divide by zero and must not command anything.
TEST(ControlTypes, ZeroMaximumYieldsNeutralNotInfinity)
{
    const AngularRateDps rate{ 100.0f, 100.0f, 100.0f };
    const AxisCommand c = rateToAxisCommand(rate, 0.0f, 0.0f, 0.0f);

    EXPECT_FLOAT_EQ(c.roll,  0.0f);
    EXPECT_FLOAT_EQ(c.pitch, 0.0f);
    EXPECT_FLOAT_EQ(c.yaw,   0.0f);
}

/**
 * The ADR-037 scenario, expressed as arithmetic. A rate setpoint produced by
 * mixRate() at full stick, if it reached the mixer uninterpreted, is a command
 * of 200 — clamped to full deflection on every axis. Converted properly against
 * the same maxima it is 1.0, which is also full deflection but for the right
 * reason. The distinction that matters is that there is now no path which
 * SKIPS the conversion.
 */
TEST(ControlTypes, TheFailsafeScenarioNoLongerHasAnUnconvertedPath)
{
    const AngularRateDps fullStickRate{ 200.0f, 0.0f, 0.0f };

    // This is the only way to get from one to the other, and it needs the maxima.
    const AxisCommand converted = rateToAxisCommand(fullStickRate, 200.0f, 200.0f, 200.0f);
    EXPECT_FLOAT_EQ(converted.roll, 1.0f);

    // And a neutral command is what the demotion path installs instead.
    const AxisCommand neutral{};
    EXPECT_FLOAT_EQ(neutral.roll, 0.0f);
}

} // namespace
