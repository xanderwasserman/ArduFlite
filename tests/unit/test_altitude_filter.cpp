/**
 * test_altitude_filter.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for AltitudeFilter and LowPassVec3.
 *
 * The seeding behaviour is the reason these exist. Both filters have a "first
 * sample" case that looks like an optimisation and is actually a correctness
 * requirement: getting it wrong produces a fabricated climb rate at boot, which
 * StateManagement can read as a launch on the bench.
 */
#include <gtest/gtest.h>

#include <cmath>

#include "src/estimation/AltitudeFilter.h"
#include "src/estimation/LowPassBank.h"

using namespace arduflite;
using arduflite::estimation::AltitudeFilter;
using arduflite::estimation::LowPassVec3;

namespace {

// ── LowPassVec3 ─────────────────────────────────────────────────────────────

TEST(LowPassVec3, AdoptsTheFirstSampleOutright)
{
    LowPassVec3 filter;
    filter.setAlpha(0.01f);   // very heavy smoothing

    const Vec3f gravity{ 0.0f, 0.0f, 1.0f };
    const Vec3f out = filter.update(gravity);

    EXPECT_FLOAT_EQ(out.z, 1.0f)
        << "blending from zero would make gravity fade in over the first second";
}

TEST(LowPassVec3, ConvergesTowardTheInput)
{
    LowPassVec3 filter;
    filter.setAlpha(0.5f);

    filter.update(Vec3f{ 0.0f, 0.0f, 0.0f });
    EXPECT_FLOAT_EQ(filter.update(Vec3f{ 0.0f, 0.0f, 1.0f }).z, 0.5f);
    EXPECT_FLOAT_EQ(filter.update(Vec3f{ 0.0f, 0.0f, 1.0f }).z, 0.75f);
}

TEST(LowPassVec3, AlphaOfOneIsPassthrough)
{
    LowPassVec3 filter;
    filter.setAlpha(1.0f);

    filter.update(Vec3f{ 5.0f, 5.0f, 5.0f });
    const Vec3f out = filter.update(Vec3f{ -1.0f, 2.0f, 3.0f });

    EXPECT_FLOAT_EQ(out.x, -1.0f);
    EXPECT_FLOAT_EQ(out.y,  2.0f);
    EXPECT_FLOAT_EQ(out.z,  3.0f);
}

TEST(LowPassVec3, ResetMakesTheNextSampleSeedAgain)
{
    LowPassVec3 filter;
    filter.setAlpha(0.1f);
    filter.update(Vec3f{ 100.0f, 0.0f, 0.0f });
    ASSERT_TRUE(filter.seeded());

    filter.reset();
    EXPECT_FALSE(filter.seeded());
    EXPECT_FLOAT_EQ(filter.update(Vec3f{ 1.0f, 0.0f, 0.0f }).x, 1.0f)
        << "after a discontinuity the old value must not drag the new one";
}

// ── AltitudeFilter ──────────────────────────────────────────────────────────

TEST(AltitudeFilter, AtTheReferencePressureAltitudeIsZero)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);

    EXPECT_NEAR(filter.pressureToAltitude_m(1013.25f), 0.0f, 1e-3f);
}

TEST(AltitudeFilter, LowerPressureMeansHigherAltitude)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);

    const float higher = filter.pressureToAltitude_m(1000.0f);
    EXPECT_GT(higher, 0.0f);

    // ~8.3 m per hPa near sea level.
    EXPECT_NEAR(higher, 110.0f, 15.0f);
    EXPECT_LT(filter.pressureToAltitude_m(1020.0f), 0.0f) << "below the reference reads negative";
}

/// The bug this prevents: a first sample differentiated against zero yields a
/// climb rate of hundreds of m/s, which the flight state machine can read as a
/// launch while the aircraft sits on the bench.
TEST(AltitudeFilter, FirstSampleProducesNoClimbRate)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(0.2f);

    ASSERT_TRUE(filter.update(950.0f, 0.02f));   // ~550 m up

    EXPECT_NEAR(filter.altitude_m(), 550.0f, 30.0f);
    EXPECT_FLOAT_EQ(filter.climbRate_mps(), 0.0f)
        << "no predecessor exists to differentiate against";
}

TEST(AltitudeFilter, SteadyAltitudeGivesZeroClimbRate)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(1.0f);

    for (int i = 0; i < 10; ++i) { filter.update(1000.0f, 0.02f); }

    EXPECT_NEAR(filter.climbRate_mps(), 0.0f, 1e-3f);
}

TEST(AltitudeFilter, ClimbingProducesPositiveRate)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(1.0f);   // unfiltered, so the derivative is exact

    filter.update(1013.25f, 0.02f);              // seed at 0 m
    const float before = filter.altitude_m();
    filter.update(1012.25f, 0.02f);              // 1 hPa lower

    EXPECT_GT(filter.climbRate_mps(), 0.0f);
    EXPECT_NEAR(filter.climbRate_mps(),
                (filter.altitude_m() - before) / 0.02f, 1e-2f);
}

TEST(AltitudeFilter, RejectsNonPositivePressureWithoutPoisoningTheFilter)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(0.5f);

    ASSERT_TRUE(filter.update(1000.0f, 0.02f));
    const float good = filter.altitude_m();

    EXPECT_FALSE(filter.update(0.0f, 0.02f)) << "a dead barometer reads zero";
    EXPECT_FALSE(filter.update(-5.0f, 0.02f));

    EXPECT_FLOAT_EQ(filter.altitude_m(), good)
        << "one NaN would propagate through every subsequent EMA term forever";
    EXPECT_TRUE(std::isfinite(filter.climbRate_mps()));
}

TEST(AltitudeFilter, ResetSuppressesTheSpikeAcrossAGap)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(1.0f);

    filter.update(1013.25f, 0.02f);
    filter.update(1013.00f, 0.02f);
    ASSERT_NE(filter.climbRate_mps(), 0.0f);

    // Recalibration, or any pause in sampling.
    filter.reset();
    EXPECT_FLOAT_EQ(filter.climbRate_mps(), 0.0f);

    // The altitude on the far side of the gap is very different. Without the
    // reset this would differentiate across the gap and invent a huge climb.
    ASSERT_TRUE(filter.update(950.0f, 0.02f));
    EXPECT_FLOAT_EQ(filter.climbRate_mps(), 0.0f);
}

TEST(AltitudeFilter, ZeroDtDoesNotDivideByZero)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(1013.25f);
    filter.setAlpha(1.0f);

    filter.update(1013.25f, 0.02f);
    filter.update(1012.00f, 0.0f);

    EXPECT_TRUE(std::isfinite(filter.climbRate_mps()));
}

TEST(AltitudeFilter, UnsetReferencePressureYieldsNonFiniteRatherThanNonsense)
{
    AltitudeFilter filter;
    filter.setReferencePressure_hpa(0.0f);

    EXPECT_FALSE(std::isfinite(filter.pressureToAltitude_m(1000.0f)));
    EXPECT_FALSE(filter.update(1000.0f, 0.02f))
        << "better to report no altitude than one measured against a bogus ground";
}

} // namespace
