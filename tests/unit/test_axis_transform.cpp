/**
 * test_axis_transform.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * Tests for arduflite::AxisTransform.
 *
 * The load-bearing test is ShippingOrientationIsExact: it pins the transform the
 * aircraft is trimmed against. If it fails, attitude signs have changed and the
 * six-orientation bench check is the only way to find out how.
 */
#include <gtest/gtest.h>

#include "src/hal/core/AxisTransform.h"

using arduflite::AlignmentTrim;
using arduflite::AxisMap;
using arduflite::AxisTransform;
using arduflite::Rotation;
using arduflite::SignedAxis;
using arduflite::Vec3f;

namespace {

/// Reference transform, written out longhand as an oracle
/// at commit 9ca8484, for ORIENTATION_SENSOR_FLIPPED_YZ.
struct ShippingOrientation
{
    static Vec3f accel(Vec3f a) { return { a.x, -a.y, a.z }; }
    static Vec3f gyro (Vec3f g) { return { -g.x, g.y, -g.z }; }
};

constexpr AxisMap kFlightMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };

} // namespace

// ── The map the aircraft flies with ─────────────────────────────────────────

TEST(AxisTransform, ShippingOrientationIsExact)
{
    const AxisTransform t{ kFlightMap };

    // A deliberately asymmetric sample so a wrong sign cannot pass by accident.
    const Vec3f accelIn{ 0.123f, -0.456f, 0.987f };
    const Vec3f gyroIn { -12.5f,  33.25f, -7.125f };

    const Vec3f accelOut = t.applyMeasurement(accelIn);
    const Vec3f gyroOut  = t.applyAngularRate(gyroIn);

    const Vec3f accelExpected = ShippingOrientation::accel(accelIn);
    const Vec3f gyroExpected  = ShippingOrientation::gyro(gyroIn);

    // Exact equality: with zero trim the matrix holds only +/-1 and 0.
    EXPECT_EQ(accelOut, accelExpected) << "accel path diverges from flying behaviour";
    EXPECT_EQ(gyroOut,  gyroExpected)  << "gyro path diverges from flying behaviour";
}

TEST(AxisTransform, FlightMapIsMirrored)
{
    // det == -1 is the signal that the part's internal axis convention differs
    // from the datasheet. It is expected here, and worth surfacing.
    EXPECT_TRUE(kFlightMap.isValid());
    EXPECT_EQ(kFlightMap.determinant(), -1);
    EXPECT_TRUE(AxisTransform{ kFlightMap }.isMirrored());
}

TEST(AxisTransform, AngularRateCarriesTheDeterminant)
{
    const AxisTransform mirrored{ kFlightMap };
    const AxisTransform proper{ toAxisMap(Rotation::None) };

    const Vec3f v{ 1.0f, 2.0f, 3.0f };

    // For a mirrored map the two entry points must DIFFER by exactly a negation.
    EXPECT_EQ(mirrored.applyAngularRate(v), -mirrored.applyMeasurement(v));

    // For a proper rotation they must agree.
    EXPECT_EQ(proper.applyAngularRate(v), proper.applyMeasurement(v));
}

// ── Map algebra ─────────────────────────────────────────────────────────────

TEST(AxisMap, IdentityIsIdentity)
{
    const AxisTransform t{ AxisMap{} };
    const Vec3f v{ 1.5f, -2.5f, 3.5f };
    EXPECT_EQ(t.applyMeasurement(v), v);
    EXPECT_EQ(t.applyAngularRate(v), v);
}

TEST(AxisMap, DuplicateAxesAreRejected)
{
    EXPECT_FALSE((AxisMap{ SignedAxis::PlusX, SignedAxis::PlusX, SignedAxis::PlusZ }).isValid());
    EXPECT_FALSE((AxisMap{ SignedAxis::PlusZ, SignedAxis::MinusZ, SignedAxis::PlusY }).isValid());
    EXPECT_TRUE ((AxisMap{ SignedAxis::PlusZ, SignedAxis::MinusX, SignedAxis::PlusY }).isValid());
}

TEST(AxisMap, AxisIndexAndSign)
{
    EXPECT_EQ(arduflite::axisIndex(SignedAxis::PlusX),  0);
    EXPECT_EQ(arduflite::axisIndex(SignedAxis::MinusX), 0);
    EXPECT_EQ(arduflite::axisIndex(SignedAxis::PlusY),  1);
    EXPECT_EQ(arduflite::axisIndex(SignedAxis::MinusZ), 2);
    EXPECT_EQ(arduflite::axisSign(SignedAxis::PlusY),   1);
    EXPECT_EQ(arduflite::axisSign(SignedAxis::MinusY), -1);
}

// ── The 24 named rotations ──────────────────────────────────────────────────

TEST(Rotation, AllTwentyFourAreValidProperRotations)
{
    for (std::uint8_t i = 0; i < static_cast<std::uint8_t>(Rotation::Count); ++i)
    {
        const auto r   = static_cast<Rotation>(i);
        const auto map = toAxisMap(r);

        EXPECT_TRUE(map.isValid())        << "rotation " << int(i) << " is singular";
        EXPECT_EQ(map.determinant(), +1)  << "rotation " << int(i) << " is not a proper rotation";
    }
}

TEST(Rotation, AllTwentyFourAreDistinct)
{
    // 24 proper rotations means 24 distinct maps — a duplicate would mean the
    // compose() table is wrong.
    constexpr int kCount = static_cast<int>(Rotation::Count);
    ASSERT_EQ(kCount, 24);

    for (int i = 0; i < kCount; ++i)
    {
        for (int j = i + 1; j < kCount; ++j)
        {
            EXPECT_FALSE(toAxisMap(static_cast<Rotation>(i)) == toAxisMap(static_cast<Rotation>(j)))
                << "rotations " << i << " and " << j << " collide";
        }
    }
}

TEST(Rotation, KnownValues)
{
    EXPECT_TRUE(toAxisMap(Rotation::None) == (AxisMap{}));

    // Sensor yawed +90 about Z: its X axis points along body Y.
    EXPECT_TRUE(toAxisMap(Rotation::Yaw90)
                == (AxisMap{ SignedAxis::MinusY, SignedAxis::PlusX, SignedAxis::PlusZ }));

    EXPECT_TRUE(toAxisMap(Rotation::Yaw180)
                == (AxisMap{ SignedAxis::MinusX, SignedAxis::MinusY, SignedAxis::PlusZ }));

    EXPECT_TRUE(toAxisMap(Rotation::Roll180)
                == (AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::MinusZ }));
}

TEST(Rotation, YawIsCyclic)
{
    // Four 90-degree yaws must return to the identity.
    const AxisMap y90 = toAxisMap(Rotation::Yaw90);
    AxisMap acc{};
    for (int i = 0; i < 4; ++i) { acc = compose(y90, acc); }
    EXPECT_TRUE(acc == (AxisMap{})) << "Yaw90 applied four times is not the identity";
}

TEST(Rotation, GravityPointsTheRightWayUnderRoll180)
{
    // Board upside down: a sensor reading +1g on its Z should report -1g in body.
    const AxisTransform t{ toAxisMap(Rotation::Roll180) };
    const Vec3f sensorLevel{ 0.0f, 0.0f, 1.0f };
    EXPECT_EQ(t.applyMeasurement(sensorLevel), (Vec3f{ 0.0f, 0.0f, -1.0f }));
}

// ── Alignment trim ──────────────────────────────────────────────────────────

TEST(AlignmentTrim, ZeroTrimIsBitIdenticalToPlainMap)
{
    const AxisTransform plain{ kFlightMap };
    const AxisTransform trimmed{ kFlightMap, AlignmentTrim{} };

    const Vec3f v{ 0.317f, -1.914f, 2.718f };
    EXPECT_EQ(plain.applyMeasurement(v), trimmed.applyMeasurement(v));
    EXPECT_EQ(plain.applyAngularRate(v), trimmed.applyAngularRate(v));
}

TEST(AlignmentTrim, NinetyDegreeYawTrimMatchesTheNamedRotation)
{
    // A trim is a rotation like any other: yaw-trim of 90 degrees applied on top
    // of the identity map must equal Rotation::Yaw90 (within float tolerance).
    const AxisTransform trimmed{ AxisMap{}, AlignmentTrim{ 0.0f, 0.0f, 90.0f } };
    const AxisTransform named  { toAxisMap(Rotation::Yaw90) };

    const Vec3f v{ 1.0f, 2.0f, 3.0f };
    const Vec3f a = trimmed.applyMeasurement(v);
    const Vec3f b = named.applyMeasurement(v);

    EXPECT_NEAR(a.x, b.x, 1e-5f);
    EXPECT_NEAR(a.y, b.y, 1e-5f);
    EXPECT_NEAR(a.z, b.z, 1e-5f);
}

TEST(AlignmentTrim, SmallTrimPerturbsButDoesNotFlip)
{
    const AxisTransform t{ AxisMap{}, AlignmentTrim{ 2.0f, -1.0f, 0.5f } };
    const Vec3f level{ 0.0f, 0.0f, 1.0f };
    const Vec3f out = t.applyMeasurement(level);

    EXPECT_GT(out.z, 0.99f);            // still essentially upright
    EXPECT_LT(std::fabs(out.x), 0.05f); // small lean only
    EXPECT_LT(std::fabs(out.y), 0.05f);
    EXPECT_FALSE(t.isMirrored());
}
