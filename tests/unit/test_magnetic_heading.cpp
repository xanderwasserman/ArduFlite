/**
 * test_magnetic_heading.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the tilt-compensated magnetic heading.
 *
 * The property worth testing is not "north reads zero" — that one is obvious
 * enough to get right by accident. It is that heading is INVARIANT under roll
 * and pitch. An uncompensated atan2(my, mx) passes every level test and then
 * swings tens of degrees in a banked turn, which reads as a noisy magnetometer
 * rather than as missing maths.
 */
#include <gtest/gtest.h>

#include "src/estimation/MagneticHeading.h"

using namespace arduflite;
using arduflite::estimation::magneticFieldStrength_ut;
using arduflite::estimation::magneticHeading_deg;

namespace {

/// Roughly mid-latitude northern hemisphere: 20 uT horizontal, 44 uT down.
constexpr float kHoriz_ut = 20.0f;
constexpr float kDown_ut  = 44.0f;

/// The body-frame field for an aircraft that is LEVEL on a given heading.
Vec3f levelFieldFor(float heading_deg)
{
    const float rad = heading_deg * 3.14159265358979f / 180.0f;
    return Vec3f{  kHoriz_ut * std::cos(rad),
                  -kHoriz_ut * std::sin(rad),
                   kDown_ut };
}

/// Rotate a LEVEL-frame field into body axes: Rx(-roll) then Ry(-pitch) — the
/// exact inverse of what the function under test undoes.
Vec3f tiltField(const Vec3f& level, float roll_deg, float pitch_deg)
{
    constexpr float kDegToRad = 3.14159265358979f / 180.0f;
    const float sr = std::sin(roll_deg * kDegToRad),  cr = std::cos(roll_deg * kDegToRad);
    const float sp = std::sin(pitch_deg * kDegToRad), cp = std::cos(pitch_deg * kDegToRad);

    // Undo Ry(pitch)
    const Vec3f afterPitch{  level.x * cp - level.z * sp,
                             level.y,
                             level.x * sp + level.z * cp };
    // Undo Rx(roll)
    return Vec3f{ afterPitch.x,
                  afterPitch.y * cr + afterPitch.z * sr,
                 -afterPitch.y * sr + afterPitch.z * cr };
}

AttitudeDeg attitude(float roll, float pitch)
{
    AttitudeDeg a{};
    a.roll  = roll;
    a.pitch = pitch;
    a.yaw   = 123.0f;   // must be ignored; using it would be circular
    return a;
}

// ── Cardinal points, level ──────────────────────────────────────────────────

TEST(MagneticHeading, ReadsZeroPointingNorth)
{
    EXPECT_NEAR(magneticHeading_deg(levelFieldFor(0.0f), attitude(0, 0)), 0.0f, 0.01f);
}

TEST(MagneticHeading, IncreasesClockwiseThroughTheCardinalPoints)
{
    EXPECT_NEAR(magneticHeading_deg(levelFieldFor(90.0f),  attitude(0, 0)),  90.0f, 0.01f);
    EXPECT_NEAR(magneticHeading_deg(levelFieldFor(180.0f), attitude(0, 0)), 180.0f, 0.01f);
    EXPECT_NEAR(magneticHeading_deg(levelFieldFor(270.0f), attitude(0, 0)), 270.0f, 0.01f);
}

TEST(MagneticHeading, WrapsIntoZeroToThreeSixty)
{
    for (float h = 0.0f; h < 360.0f; h += 15.0f)
    {
        const float result = magneticHeading_deg(levelFieldFor(h), attitude(0, 0));
        EXPECT_GE(result, 0.0f)   << "heading " << h;
        EXPECT_LT(result, 360.0f) << "heading " << h;
    }
}

// ── The property that matters ───────────────────────────────────────────────

/// Bank the aircraft without turning it. The heading must not move. This is the
/// test an uncompensated implementation fails.
TEST(MagneticHeading, IsInvariantUnderRoll)
{
    const Vec3f level = levelFieldFor(45.0f);

    for (float roll = -60.0f; roll <= 60.0f; roll += 15.0f)
    {
        const float result = magneticHeading_deg(tiltField(level, roll, 0.0f),
                                                 attitude(roll, 0.0f));
        EXPECT_NEAR(result, 45.0f, 0.05f) << "roll " << roll;
    }
}

TEST(MagneticHeading, IsInvariantUnderPitch)
{
    const Vec3f level = levelFieldFor(200.0f);

    for (float pitch = -45.0f; pitch <= 45.0f; pitch += 15.0f)
    {
        const float result = magneticHeading_deg(tiltField(level, 0.0f, pitch),
                                                 attitude(0.0f, pitch));
        EXPECT_NEAR(result, 200.0f, 0.05f) << "pitch " << pitch;
    }
}

TEST(MagneticHeading, IsInvariantUnderCombinedRollAndPitch)
{
    const Vec3f level = levelFieldFor(310.0f);
    const float result = magneticHeading_deg(tiltField(level, 35.0f, -20.0f),
                                             attitude(35.0f, -20.0f));
    EXPECT_NEAR(result, 310.0f, 0.05f);
}

/// Proves the tilt tests above are not vacuous: skipping compensation on the
/// same input is visibly wrong, so the tests would catch its removal.
TEST(MagneticHeading, UncompensatedHeadingWouldBeVisiblyWrongWhenBanked)
{
    const Vec3f banked = tiltField(levelFieldFor(45.0f), 45.0f, 0.0f);

    const float naive_deg =
        std::atan2(-banked.y, banked.x) * 180.0f / 3.14159265358979f;
    EXPECT_GT(std::fabs(naive_deg - 45.0f), 10.0f)
        << "if this is small, the tilt tests prove nothing";
}

// ── Degenerate input ────────────────────────────────────────────────────────

TEST(MagneticHeading, ReturnsZeroForAZeroLengthField)
{
    EXPECT_FLOAT_EQ(magneticHeading_deg(Vec3f{ 0.0f, 0.0f, 0.0f }, attitude(0, 0)), 0.0f);
}

TEST(MagneticHeading, IgnoresTheYawItIsGiven)
{
    AttitudeDeg a = attitude(10.0f, 5.0f);
    AttitudeDeg b = a;
    b.yaw = -97.0f;

    const Vec3f field = tiltField(levelFieldFor(77.0f), 10.0f, 5.0f);
    EXPECT_FLOAT_EQ(magneticHeading_deg(field, a), magneticHeading_deg(field, b));
}

// ── Field strength ──────────────────────────────────────────────────────────

/// Magnitude is orientation-independent, which is precisely what makes it the
/// useful diagnostic: anything that moves it is interference, not manoeuvring.
TEST(MagneticHeading, FieldStrengthDoesNotDependOnAttitude)
{
    const float expected = std::sqrt(kHoriz_ut * kHoriz_ut + kDown_ut * kDown_ut);

    for (float heading = 0.0f; heading < 360.0f; heading += 45.0f)
    {
        const Vec3f tilted = tiltField(levelFieldFor(heading), 30.0f, -15.0f);
        EXPECT_NEAR(magneticFieldStrength_ut(tilted), expected, 0.01f);
    }
}

} // namespace
