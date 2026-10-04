/**
 * test_airframe_mixer.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * These call the real AirframeMixer, not a copy of its formulas, so they fail
 * when production changes rather than drifting quietly out of step with it.
 */
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "src/actuators/AirframeMixer.h"

using arduflite::actuators::AirframeMixer;
using arduflite::actuators::SurfaceCommands;
using arduflite::actuators::WingDesign;
using arduflite::actuators::isImplemented;

namespace {
constexpr float kEps = 1e-6f;
}

// ── Conventional ────────────────────────────────────────────────────────────

TEST(AirframeMixer, ConventionalPassesAxesStraightThrough)
{
    const auto c = AirframeMixer::mix(WingDesign::Conventional, 0.5f, -0.25f, 0.75f);
    EXPECT_NEAR(c.elevator, -0.25f, kEps);
    EXPECT_NEAR(c.rudder,    0.75f, kEps);
    EXPECT_NEAR(c.aileronLeft, 0.5f, kEps);
}

TEST(AirframeMixer, ConventionalAileronsAreDifferential)
{
    // The right aileron is negated in the mix:
    //   desiredRight = neutral + (invert ? +raw : -raw)
    const auto c = AirframeMixer::mix(WingDesign::Conventional, 0.4f, 0.0f, 0.0f);
    EXPECT_NEAR(c.aileronLeft,   0.4f, kEps);
    EXPECT_NEAR(c.aileronRight, -0.4f, kEps);
    EXPECT_NEAR(c.aileronLeft + c.aileronRight, 0.0f, kEps) << "pure roll must not pitch";
}

// ── Delta wing ──────────────────────────────────────────────────────────────

TEST(AirframeMixer, DeltaElevonMixingHalvesTheSum)
{
    // The elevon halving means a simultaneous full-pitch and full-roll demand
    // stays inside one surface's travel, so the
    // normalised form carries the halving.
    const auto c = AirframeMixer::mix(WingDesign::DeltaWing, 0.5f, 0.5f, 0.0f);
    EXPECT_NEAR(c.aileronLeft,  0.0f, kEps);   // (pitch - roll) / 2
    EXPECT_NEAR(c.aileronRight, 0.5f, kEps);   // (pitch + roll) / 2
}

TEST(AirframeMixer, DeltaPurePitchMovesBothSurfacesTogether)
{
    const auto c = AirframeMixer::mix(WingDesign::DeltaWing, 0.0f, 0.8f, 0.0f);
    EXPECT_NEAR(c.aileronLeft,  0.4f, kEps);
    EXPECT_NEAR(c.aileronRight, 0.4f, kEps);
}

TEST(AirframeMixer, DeltaPureRollMovesThemOpposite)
{
    const auto c = AirframeMixer::mix(WingDesign::DeltaWing, 0.8f, 0.0f, 0.0f);
    EXPECT_NEAR(c.aileronLeft, -c.aileronRight, kEps);
}

TEST(AirframeMixer, DeltaSaturatesRatherThanExceedingRange)
{
    const auto c = AirframeMixer::mix(WingDesign::DeltaWing, -1.0f, 1.0f, 0.0f);
    EXPECT_LE(std::fabs(c.aileronLeft),  1.0f);
    EXPECT_LE(std::fabs(c.aileronRight), 1.0f);
    EXPECT_NEAR(c.aileronLeft, 1.0f, kEps);
}

// ── V-tail ──────────────────────────────────────────────────────────────────

TEST(AirframeMixer, VTailIsAnUnimplementedStub)
{
    // The obvious ruddervator mix ignores roll, giving a V-tail airframe no
    // roll authority. It was never flown or tested, so it is reserved rather
    // than ported. Callers must check isImplemented().
    EXPECT_FALSE(isImplemented(WingDesign::VTail));

    const auto c = AirframeMixer::mix(WingDesign::VTail, 0.5f, 0.6f, 0.4f);
    EXPECT_NEAR(c.aileronLeft,  0.0f, kEps);
    EXPECT_NEAR(c.aileronRight, 0.0f, kEps);
    EXPECT_NEAR(c.elevator,     0.0f, kEps);
    EXPECT_NEAR(c.rudder,       0.0f, kEps);
}

TEST(AirframeMixer, FlyableGeometriesAreMarkedImplemented)
{
    EXPECT_TRUE(isImplemented(WingDesign::Conventional));
    EXPECT_TRUE(isImplemented(WingDesign::DeltaWing));
}

// ── Guards, for every geometry ──────────────────────────────────────────────

class MixerGeometry : public ::testing::TestWithParam<WingDesign> {};

TEST_P(MixerGeometry, ClampsInputsToUnitRange)
{
    const auto over  = AirframeMixer::mix(GetParam(),  5.0f,  5.0f,  5.0f);
    const auto atMax = AirframeMixer::mix(GetParam(),  1.0f,  1.0f,  1.0f);
    EXPECT_NEAR(over.aileronLeft,  atMax.aileronLeft,  kEps);
    EXPECT_NEAR(over.aileronRight, atMax.aileronRight, kEps);
    EXPECT_NEAR(over.elevator,     atMax.elevator,     kEps);
    EXPECT_NEAR(over.rudder,       atMax.rudder,       kEps);
}

TEST_P(MixerGeometry, NonFiniteInputYieldsZeroNotPoison)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    for (const auto c : { AirframeMixer::mix(GetParam(), nan, 0.0f, 0.0f),
                          AirframeMixer::mix(GetParam(), 0.0f, inf, 0.0f),
                          AirframeMixer::mix(GetParam(), 0.0f, 0.0f, -inf) })
    {
        EXPECT_TRUE(std::isfinite(c.aileronLeft));
        EXPECT_TRUE(std::isfinite(c.aileronRight));
        EXPECT_TRUE(std::isfinite(c.elevator));
        EXPECT_TRUE(std::isfinite(c.rudder));
        EXPECT_FLOAT_EQ(c.aileronLeft, 0.0f);
    }
}

TEST_P(MixerGeometry, NeutralInputGivesNeutralOutput)
{
    const auto c = AirframeMixer::mix(GetParam(), 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(c.aileronLeft,  0.0f, kEps);
    EXPECT_NEAR(c.aileronRight, 0.0f, kEps);
    EXPECT_NEAR(c.elevator,     0.0f, kEps);
    EXPECT_NEAR(c.rudder,       0.0f, kEps);
}

INSTANTIATE_TEST_SUITE_P(AllGeometries, MixerGeometry,
                         ::testing::Values(WingDesign::Conventional,
                                           WingDesign::DeltaWing));

// ── The enum must keep matching the stored config encoding ─────────────────

TEST(AirframeMixer, WingDesignEncodingMatchesStoredConfig)
{
    // servo.wing_design is persisted as an int; changing these breaks every
    // saved configuration.
    EXPECT_EQ(static_cast<int>(WingDesign::Conventional), 0);
    EXPECT_EQ(static_cast<int>(WingDesign::DeltaWing),    1);
    EXPECT_EQ(static_cast<int>(WingDesign::VTail),        2);
}
