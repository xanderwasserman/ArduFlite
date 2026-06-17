/**
 * test_quaternion.cpp — Unit tests for FliteQuaternion.
 *
 * Tests cover: identity, conjugate, multiply (Hamilton product),
 * normSq, normalize, inverse, and toAxisAngle.
 */
#include <gtest/gtest.h>
#include "FliteQuaternion.h"

#include <cmath>

static constexpr float kEps  = 1e-5f;
static constexpr float kLoose = 1e-4f;   // for multi-step float ops

// ── Helpers ───────────────────────────────────────────────────────────────────

static FliteQuaternion makeYaw90()
{
    // 90° rotation around Z-axis: w=cos(45°), z=sin(45°)
    float half = static_cast<float>(M_PI / 4.0);
    return FliteQuaternion(std::cos(half), 0.0f, 0.0f, std::sin(half));
}

// ── Identity ──────────────────────────────────────────────────────────────────

TEST(FliteQuaternion, Identity_NormSqIsOne)
{
    FliteQuaternion q;   // default = identity (1,0,0,0)
    EXPECT_NEAR(q.normSq(), 1.0f, kEps);
}

TEST(FliteQuaternion, Identity_Conjugate_IsIdentity)
{
    FliteQuaternion q;
    FliteQuaternion c = q.conjugate();
    EXPECT_NEAR(c.w, 1.0f, kEps);
    EXPECT_NEAR(c.x, 0.0f, kEps);
    EXPECT_NEAR(c.y, 0.0f, kEps);
    EXPECT_NEAR(c.z, 0.0f, kEps);
}

TEST(FliteQuaternion, Identity_Multiply_Identity_IsIdentity)
{
    FliteQuaternion i;
    FliteQuaternion r = i * i;
    EXPECT_NEAR(r.w, 1.0f, kEps);
    EXPECT_NEAR(r.x, 0.0f, kEps);
    EXPECT_NEAR(r.y, 0.0f, kEps);
    EXPECT_NEAR(r.z, 0.0f, kEps);
}

// ── Conjugate ─────────────────────────────────────────────────────────────────

TEST(FliteQuaternion, Conjugate_NegatesXYZ)
{
    FliteQuaternion q(0.7071f, 0.7071f, 0.0f, 0.0f);
    FliteQuaternion c = q.conjugate();
    EXPECT_NEAR(c.w,  0.7071f, kEps);
    EXPECT_NEAR(c.x, -0.7071f, kEps);
    EXPECT_NEAR(c.y,  0.0f,    kEps);
    EXPECT_NEAR(c.z,  0.0f,    kEps);
}

TEST(FliteQuaternion, Conjugate_DoubleConjugate_IsOriginal)
{
    FliteQuaternion q(0.5f, 0.5f, -0.5f, 0.5f);
    FliteQuaternion cc = q.conjugate().conjugate();
    EXPECT_NEAR(cc.w, q.w, kEps);
    EXPECT_NEAR(cc.x, q.x, kEps);
    EXPECT_NEAR(cc.y, q.y, kEps);
    EXPECT_NEAR(cc.z, q.z, kEps);
}

// ── normSq ────────────────────────────────────────────────────────────────────

TEST(FliteQuaternion, NormSq_UnitQuaternion)
{
    FliteQuaternion q = makeYaw90();
    EXPECT_NEAR(q.normSq(), 1.0f, kLoose);
}

TEST(FliteQuaternion, NormSq_ScaledQuaternion)
{
    FliteQuaternion q(2.0f, 0.0f, 0.0f, 0.0f);
    EXPECT_NEAR(q.normSq(), 4.0f, kEps);
}

// ── Multiply (Hamilton product) ───────────────────────────────────────────────

TEST(FliteQuaternion, Multiply_UnitTimesConjugate_NormIsOne)
{
    // q * conj(q) should equal identity for a unit quaternion
    FliteQuaternion q = makeYaw90();
    FliteQuaternion r = q * q.conjugate();
    EXPECT_NEAR(r.normSq(), 1.0f, kLoose);
    EXPECT_NEAR(r.w, 1.0f, kLoose);
    EXPECT_NEAR(r.x, 0.0f, kLoose);
    EXPECT_NEAR(r.y, 0.0f, kLoose);
    EXPECT_NEAR(r.z, 0.0f, kLoose);
}

TEST(FliteQuaternion, Multiply_NonCommutative)
{
    // Quaternion multiplication is generally not commutative: q*p ≠ p*q
    float half = static_cast<float>(M_PI / 4.0);
    FliteQuaternion qz(std::cos(half), 0.0f, 0.0f, std::sin(half)); // 90° around Z
    FliteQuaternion qx(std::cos(half), std::sin(half), 0.0f, 0.0f); // 90° around X

    FliteQuaternion qzqx = qz * qx;
    FliteQuaternion qxqz = qx * qz;

    // At least one component must differ
    bool differs = (std::fabs(qzqx.x - qxqz.x) > 1e-4f ||
                    std::fabs(qzqx.y - qxqz.y) > 1e-4f ||
                    std::fabs(qzqx.z - qxqz.z) > 1e-4f);
    EXPECT_TRUE(differs);
}

TEST(FliteQuaternion, Multiply_DoubleYaw90_IsYaw180)
{
    FliteQuaternion q = makeYaw90();
    FliteQuaternion q180 = q * q;    // two 90° yaw rotations = 180° yaw
    // 180° around Z: w=cos(90°)=0, z=sin(90°)=1
    EXPECT_NEAR(q180.w, 0.0f, kLoose);
    EXPECT_NEAR(q180.x, 0.0f, kLoose);
    EXPECT_NEAR(q180.y, 0.0f, kLoose);
    EXPECT_NEAR(std::fabs(q180.z), 1.0f, kLoose);
}

// ── normalize() ──────────────────────────────────────────────────────────────

TEST(FliteQuaternion, Normalize_ScaledQuaternion_BecomesUnit)
{
    FliteQuaternion q(3.0f, 0.0f, 0.0f, 0.0f);
    q.normalize();
    EXPECT_NEAR(q.normSq(), 1.0f, kEps);
    EXPECT_NEAR(q.w, 1.0f, kEps);
}

TEST(FliteQuaternion, Normalize_AlreadyUnit_Unchanged)
{
    FliteQuaternion q = makeYaw90();
    float wBefore = q.w, zBefore = q.z;
    q.normalize();
    EXPECT_NEAR(q.w, wBefore, kLoose);
    EXPECT_NEAR(q.z, zBefore, kLoose);
}

TEST(FliteQuaternion, Normalize_NearZero_Stable)
{
    // normalize() has a guard for near-zero norm: must not produce NaN.
    FliteQuaternion q(1e-8f, 0.0f, 0.0f, 0.0f);
    q.normalize();
    EXPECT_FALSE(std::isnan(q.w));
    EXPECT_FALSE(std::isnan(q.x));
}

// ── inverse() ────────────────────────────────────────────────────────────────

TEST(FliteQuaternion, Inverse_TimesOriginal_IsIdentity)
{
    FliteQuaternion q = makeYaw90();
    FliteQuaternion r = q.inverse() * q;
    EXPECT_NEAR(r.w, 1.0f, kLoose);
    EXPECT_NEAR(r.x, 0.0f, kLoose);
    EXPECT_NEAR(r.y, 0.0f, kLoose);
    EXPECT_NEAR(r.z, 0.0f, kLoose);
}

TEST(FliteQuaternion, Inverse_NearZeroNorm_ReturnsIdentity)
{
    // inverse() guards ns < 1e-9: must return identity, not NaN.
    FliteQuaternion q(0.0f, 0.0f, 0.0f, 0.0f);
    FliteQuaternion inv = q.inverse();
    EXPECT_NEAR(inv.w, 1.0f, kEps);
    EXPECT_NEAR(inv.x, 0.0f, kEps);
    EXPECT_NEAR(inv.y, 0.0f, kEps);
    EXPECT_NEAR(inv.z, 0.0f, kEps);
}

// ── toAxisAngle() ─────────────────────────────────────────────────────────────

TEST(FliteQuaternion, ToAxisAngle_Identity_AngleNearZero)
{
    FliteQuaternion q;   // identity
    float rx, ry, rz, angle;
    q.toAxisAngle(rx, ry, rz, angle);
    EXPECT_NEAR(angle, 0.0f, kLoose);
}

TEST(FliteQuaternion, ToAxisAngle_Yaw90_CorrectAngleAndAxis)
{
    FliteQuaternion q = makeYaw90();
    float rx, ry, rz, angle;
    q.toAxisAngle(rx, ry, rz, angle);

    EXPECT_NEAR(angle, static_cast<float>(M_PI / 2.0), kLoose);
    // Axis should be Z (0, 0, 1) for a yaw rotation
    EXPECT_NEAR(rx, 0.0f, kLoose);
    EXPECT_NEAR(ry, 0.0f, kLoose);
    EXPECT_NEAR(std::fabs(rz), 1.0f, kLoose);
}

// ── Euler-specific quaternion constructions ───────────────────────────────────

TEST(FliteQuaternion, Roll90_Components)
{
    // 90° rotation around X-axis: q = [cos(45°), sin(45°), 0, 0]
    const float half = static_cast<float>(M_PI / 4.0);
    FliteQuaternion q(std::cosf(half), std::sinf(half), 0.0f, 0.0f);
    EXPECT_NEAR(q.w, std::cosf(half), kLoose);
    EXPECT_NEAR(q.x, std::sinf(half), kLoose);
    EXPECT_NEAR(q.y, 0.0f,            kLoose);
    EXPECT_NEAR(q.z, 0.0f,            kLoose);
    EXPECT_NEAR(q.normSq(), 1.0f, kLoose);
}

TEST(FliteQuaternion, Pitch90_Components)
{
    // 90° rotation around Y-axis: q = [cos(45°), 0, sin(45°), 0]
    const float half = static_cast<float>(M_PI / 4.0);
    FliteQuaternion q(std::cosf(half), 0.0f, std::sinf(half), 0.0f);
    EXPECT_NEAR(q.w, std::cosf(half), kLoose);
    EXPECT_NEAR(q.x, 0.0f,            kLoose);
    EXPECT_NEAR(q.y, std::sinf(half), kLoose);
    EXPECT_NEAR(q.z, 0.0f,            kLoose);
    EXPECT_NEAR(q.normSq(), 1.0f, kLoose);
}

// ── Yaw extraction ────────────────────────────────────────────────────────────

TEST(FliteQuaternion, YawExtraction_Yaw90_ReturnsHalfPi)
{
    // For a pure yaw quaternion (rotation around Z), extractYaw formula:
    //   yaw = atan2f(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    FliteQuaternion q = makeYaw90();
    float yaw = std::atan2f(2.0f * (q.w * q.z + q.x * q.y),
                            1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    EXPECT_NEAR(yaw, static_cast<float>(M_PI / 2.0), kLoose);
}

TEST(FliteQuaternion, YawExtraction_Identity_ReturnsZero)
{
    FliteQuaternion q;  // identity
    float yaw = std::atan2f(2.0f * (q.w * q.z + q.x * q.y),
                            1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    EXPECT_NEAR(yaw, 0.0f, kLoose);
}

// ── wrapAngle formula ─────────────────────────────────────────────────────────
// Verified inline — mirrors the static wrapAngle() in ArduFliteAttitudeController.cpp.

static float wrapAngle(float angle)
{
    angle -= 2.0f * static_cast<float>(M_PI)
             * std::floorf((angle + static_cast<float>(M_PI))
                           / (2.0f * static_cast<float>(M_PI)));
    return angle;
}

TEST(FliteQuaternion, WrapAngle_LargePositive_WrapsToNegative)
{
    // 3π wraps to π − 2π = −π (edge), checked as absolute
    float wrapped = wrapAngle(3.0f * static_cast<float>(M_PI));
    EXPECT_NEAR(std::fabs(wrapped), static_cast<float>(M_PI), kLoose);
}

TEST(FliteQuaternion, WrapAngle_LargeNegative_WrapsToPositive)
{
    // −3π should wrap to π or −π (both represent the same rotation)
    float wrapped = wrapAngle(-3.0f * static_cast<float>(M_PI));
    EXPECT_NEAR(std::fabs(wrapped), static_cast<float>(M_PI), kLoose);
}

TEST(FliteQuaternion, WrapAngle_SmallAngle_Unchanged)
{
    // 0.5 rad is well within (−π, π) — must pass through unchanged.
    EXPECT_NEAR(wrapAngle(0.5f), 0.5f, kLoose);
}

// ── Multiplication identities ─────────────────────────────────────────────────

TEST(FliteQuaternion, LeftIdentity_IdentityTimesQ_IsQ)
{
    FliteQuaternion id;          // identity = (1, 0, 0, 0)
    FliteQuaternion q = makeYaw90();
    FliteQuaternion r = id * q;
    EXPECT_NEAR(r.w, q.w, kLoose);
    EXPECT_NEAR(r.x, q.x, kLoose);
    EXPECT_NEAR(r.y, q.y, kLoose);
    EXPECT_NEAR(r.z, q.z, kLoose);
}

TEST(FliteQuaternion, RightIdentity_QTimesIdentity_IsQ)
{
    FliteQuaternion id;
    FliteQuaternion q = makeYaw90();
    FliteQuaternion r = q * id;
    EXPECT_NEAR(r.w, q.w, kLoose);
    EXPECT_NEAR(r.x, q.x, kLoose);
    EXPECT_NEAR(r.y, q.y, kLoose);
    EXPECT_NEAR(r.z, q.z, kLoose);
}
