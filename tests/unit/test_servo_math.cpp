/**
 * test_servo_math.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * Unit tests for the servo mixing formulas from ServoManager.cpp.
 *
 * ServoManager depends on ESP32Servo.h and ConfigRegistry.h (hardware/NVS).
 * Rather than mocking those, the formulas are reproduced verbatim here so the
 * pure mixing math can be validated on the host.  Any change to the production
 * mixing logic must be mirrored in this file.
 *
 * Formulas under test:
 *   mapFloatToInt  — clamped linear map float→int
 *   CONVENTIONAL   — pitch/yaw to single surface; roll to opposing ailerons
 *   DELTA_WING     — elevon mixing: left=(pitch-roll), right=(pitch+roll)
 *   V_TAIL         — ruddervator mixing: left=(pitch+yaw), right=(pitch-yaw)
 *   NaN/Inf guard  — isfinite check on all three axes
 *   Input clamp    — commands constrained to [-1, 1] before mixing
 */
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

// ── Helpers matching ServoManager.cpp exactly ─────────────────────────────────

/**
 * @brief Linear map from float [inMin, inMax] to int [outMin, outMax].
 * Input is clamped before mapping (matches ServoManager::mapFloatToInt).
 */
static int mapFloatToInt(float val, float inMin, float inMax, int outMin, int outMax)
{
    if (val > inMax) val = inMax;
    if (val < inMin) val = inMin;
    return static_cast<int>(
        (val - inMin) * static_cast<float>(outMax - outMin) / (inMax - inMin)
        + static_cast<float>(outMin));
}

/** @brief Clamp input commands to [-1, 1] (matches ServoManager::writeCommands). */
static float clampCmd(float v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

/** @brief NaN/Inf guard — mirrors the isfinite check in ServoManager::writeCommands. */
static bool anyNonFinite(float r, float p, float y)
{
    return !std::isfinite(r) || !std::isfinite(p) || !std::isfinite(y);
}

// Default servo parameters (from ServoManager default constructor)
static constexpr int NEUTRAL = 90;
static constexpr int DEFLECT = 80;

// ── Result types for each wing geometry ──────────────────────────────────────

struct ConvResult  { int pitch, yaw, rollLeft, rollRight; };
struct ElevonResult { int left, right; };

/**
 * @brief CONVENTIONAL: each axis drives a dedicated surface.
 *   pitch → elevator   (inversion optional)
 *   yaw   → rudder     (inversion optional)
 *   roll  → dual ailerons, left opposite to right
 */
static ConvResult mixConventional(float rollCmd, float pitchCmd, float yawCmd,
                                   bool pitchInv = false, bool yawInv   = false,
                                   bool leftInv  = false, bool rightInv = false)
{
    rollCmd  = clampCmd(rollCmd);
    pitchCmd = clampCmd(pitchCmd);
    yawCmd   = clampCmd(yawCmd);

    int rawPitch = mapFloatToInt(pitchCmd, -1.0f, 1.0f, -DEFLECT, DEFLECT);
    if (pitchInv) rawPitch = -rawPitch;

    int rawYaw = mapFloatToInt(yawCmd, -1.0f, 1.0f, -DEFLECT, DEFLECT);
    if (yawInv) rawYaw = -rawYaw;

    int rawRoll   = mapFloatToInt(rollCmd, -1.0f, 1.0f, -DEFLECT, DEFLECT);
    int rollLeft  = NEUTRAL + (leftInv  ? -rawRoll :  rawRoll);
    int rollRight = NEUTRAL + (rightInv ?  rawRoll : -rawRoll);

    return { NEUTRAL + rawPitch, NEUTRAL + rawYaw, rollLeft, rollRight };
}

/**
 * @brief DELTA_WING elevon mixing.
 *   left  = mapFloatToInt(pitch - roll, -2, 2, -defl, defl)
 *   right = mapFloatToInt(pitch + roll, -2, 2, -defl, defl)
 */
static ElevonResult mixDelta(float rollCmd, float pitchCmd,
                              bool leftInv = false, bool rightInv = false)
{
    rollCmd  = clampCmd(rollCmd);
    pitchCmd = clampCmd(pitchCmd);

    int rawLeft  = mapFloatToInt(pitchCmd - rollCmd, -2.0f, 2.0f, -DEFLECT, DEFLECT);
    int rawRight = mapFloatToInt(pitchCmd + rollCmd, -2.0f, 2.0f, -DEFLECT, DEFLECT);

    if (leftInv)  rawLeft  = -rawLeft;
    if (rightInv) rawRight = -rawRight;

    return { NEUTRAL + rawLeft, NEUTRAL + rawRight };
}

/**
 * @brief V_TAIL ruddervator mixing.
 *   left  = mapFloatToInt(pitch + yaw, -2, 2, -defl, defl)
 *   right = mapFloatToInt(pitch - yaw, -2, 2, -defl, defl)
 */
static ElevonResult mixVTail(float pitchCmd, float yawCmd,
                              bool leftInv = false, bool rightInv = false)
{
    pitchCmd = clampCmd(pitchCmd);
    yawCmd   = clampCmd(yawCmd);

    int rawLeft  = mapFloatToInt(pitchCmd + yawCmd, -2.0f, 2.0f, -DEFLECT, DEFLECT);
    int rawRight = mapFloatToInt(pitchCmd - yawCmd, -2.0f, 2.0f, -DEFLECT, DEFLECT);

    if (leftInv)  rawLeft  = -rawLeft;
    if (rightInv) rawRight = -rawRight;

    return { NEUTRAL + rawLeft, NEUTRAL + rawRight };
}

// ── mapFloatToInt ─────────────────────────────────────────────────────────────

TEST(ServoMath, MapFloatToInt_FullPositive)
{
    EXPECT_EQ(mapFloatToInt(1.0f, -1.0f, 1.0f, -80, 80), 80);
}

TEST(ServoMath, MapFloatToInt_FullNegative)
{
    EXPECT_EQ(mapFloatToInt(-1.0f, -1.0f, 1.0f, -80, 80), -80);
}

TEST(ServoMath, MapFloatToInt_Center)
{
    EXPECT_EQ(mapFloatToInt(0.0f, -1.0f, 1.0f, -80, 80), 0);
}

TEST(ServoMath, MapFloatToInt_HalfScale)
{
    // (0.5 - (-1)) / 2 * 160 + (-80) = 1.5/2*160 - 80 = 120 - 80 = 40
    EXPECT_EQ(mapFloatToInt(0.5f, -1.0f, 1.0f, -80, 80), 40);
}

TEST(ServoMath, MapFloatToInt_ClampAboveMax)
{
    // val > inMax → clamped to inMax → output = outMax
    EXPECT_EQ(mapFloatToInt(2.0f, -1.0f, 1.0f, -80, 80), 80);
}

TEST(ServoMath, MapFloatToInt_ClampBelowMin)
{
    EXPECT_EQ(mapFloatToInt(-2.0f, -1.0f, 1.0f, -80, 80), -80);
}

// ── NaN / Inf guard ───────────────────────────────────────────────────────────

TEST(ServoMath, NanInfGuard_NanRoll_Detected)
{
    EXPECT_TRUE(anyNonFinite(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f));
}

TEST(ServoMath, NanInfGuard_InfPitch_Detected)
{
    EXPECT_TRUE(anyNonFinite(0.0f, std::numeric_limits<float>::infinity(), 0.0f));
}

TEST(ServoMath, NanInfGuard_NegInfYaw_Detected)
{
    EXPECT_TRUE(anyNonFinite(0.0f, 0.0f, -std::numeric_limits<float>::infinity()));
}

TEST(ServoMath, NanInfGuard_ValidInputs_Pass)
{
    EXPECT_FALSE(anyNonFinite(0.5f, -0.5f, 0.0f));
}

// ── Input clamping ────────────────────────────────────────────────────────────

TEST(ServoMath, InputClamp_OverRangeRoll_ClampedTo1)
{
    // A roll command of 2.0 should be treated identically to 1.0.
    auto r_clamped = mixConventional(2.0f, 0.0f, 0.0f);
    auto r_full    = mixConventional(1.0f, 0.0f, 0.0f);
    EXPECT_EQ(r_clamped.rollLeft,  r_full.rollLeft);
    EXPECT_EQ(r_clamped.rollRight, r_full.rollRight);
}

TEST(ServoMath, InputClamp_UnderRangePitch_ClampedToMinus1)
{
    auto r_clamped  = mixConventional(0.0f, -3.0f, 0.0f);
    auto r_full     = mixConventional(0.0f, -1.0f, 0.0f);
    EXPECT_EQ(r_clamped.pitch, r_full.pitch);
}

// ── CONVENTIONAL mixing ───────────────────────────────────────────────────────

TEST(ServoMath, Conventional_Neutral_AllServosAtNeutral)
{
    auto r = mixConventional(0.0f, 0.0f, 0.0f);
    EXPECT_EQ(r.pitch,     NEUTRAL);
    EXPECT_EQ(r.yaw,       NEUTRAL);
    EXPECT_EQ(r.rollLeft,  NEUTRAL);
    EXPECT_EQ(r.rollRight, NEUTRAL);
}

TEST(ServoMath, Conventional_FullPitchUp_ElevatorFullyDeflected)
{
    auto r = mixConventional(0.0f, 1.0f, 0.0f);
    EXPECT_EQ(r.pitch, NEUTRAL + DEFLECT);
    EXPECT_EQ(r.yaw,   NEUTRAL);
}

TEST(ServoMath, Conventional_FullYawRight_RudderFullyDeflected)
{
    auto r = mixConventional(0.0f, 0.0f, 1.0f);
    EXPECT_EQ(r.yaw, NEUTRAL + DEFLECT);
}

TEST(ServoMath, Conventional_RollRight_AileronsOpposed)
{
    // Roll right (+1): left aileron up, right aileron down.
    auto r = mixConventional(1.0f, 0.0f, 0.0f);
    EXPECT_EQ(r.rollLeft,  NEUTRAL + DEFLECT);
    EXPECT_EQ(r.rollRight, NEUTRAL - DEFLECT);
}

TEST(ServoMath, Conventional_RollLeft_AileronsOpposed)
{
    auto r = mixConventional(-1.0f, 0.0f, 0.0f);
    EXPECT_EQ(r.rollLeft,  NEUTRAL - DEFLECT);
    EXPECT_EQ(r.rollRight, NEUTRAL + DEFLECT);
}

TEST(ServoMath, Conventional_PitchInversion_FlipsElevator)
{
    // With inversion, full pitch-up becomes full pitch-down deflection.
    auto r_normal  = mixConventional(0.0f, 1.0f, 0.0f, false);
    auto r_inverted = mixConventional(0.0f, 1.0f, 0.0f, true);
    EXPECT_EQ(r_inverted.pitch, NEUTRAL - DEFLECT);
    // The two pitch outputs must be symmetric about neutral.
    EXPECT_EQ(r_normal.pitch + r_inverted.pitch, 2 * NEUTRAL);
}

// ── DELTA_WING mixing ─────────────────────────────────────────────────────────

TEST(ServoMath, Delta_Neutral_BothElevonsAtNeutral)
{
    auto r = mixDelta(0.0f, 0.0f);
    EXPECT_EQ(r.left,  NEUTRAL);
    EXPECT_EQ(r.right, NEUTRAL);
}

TEST(ServoMath, Delta_PurePitch_BothElevonsEqualAndAboveNeutral)
{
    // pitch=+1, roll=0: left=right=mapFTI(1, -2, 2, -80, 80)=(3/4)*160-80=40
    auto r = mixDelta(0.0f, 1.0f);
    EXPECT_EQ(r.left, r.right);
    EXPECT_GT(r.left, NEUTRAL);
}

TEST(ServoMath, Delta_PureRoll_ElevonsOppositeAndSymmetric)
{
    // roll=+1, pitch=0: left=mapFTI(-1, -2, 2, -80, 80)=-40
    //                   right=mapFTI(+1, -2, 2, -80, 80)=+40
    auto r = mixDelta(1.0f, 0.0f);
    EXPECT_LT(r.left,  NEUTRAL);
    EXPECT_GT(r.right, NEUTRAL);
    EXPECT_EQ(r.left + r.right, 2 * NEUTRAL);  // symmetric about neutral
}

TEST(ServoMath, Delta_FullPitchAndRoll_LeftAtNeutralRightFullyDeflected)
{
    // pitch=+1, roll=+1: pitch-roll=0 → left at neutral
    //                    pitch+roll=2 → right fully deflected
    auto r = mixDelta(1.0f, 1.0f);
    EXPECT_EQ(r.left,  NEUTRAL);
    EXPECT_EQ(r.right, NEUTRAL + DEFLECT);
}

TEST(ServoMath, Delta_LeftInversion_MirrorsLeftElevon)
{
    auto r_normal = mixDelta(0.0f, 1.0f, false, false);
    auto r_inv    = mixDelta(0.0f, 1.0f, true,  false);
    EXPECT_EQ(r_inv.left, 2 * NEUTRAL - r_normal.left);
    EXPECT_EQ(r_inv.right, r_normal.right);  // right side unchanged
}

// ── V_TAIL mixing ─────────────────────────────────────────────────────────────

TEST(ServoMath, VTail_Neutral_BothRuddervatorAtNeutral)
{
    auto r = mixVTail(0.0f, 0.0f);
    EXPECT_EQ(r.left,  NEUTRAL);
    EXPECT_EQ(r.right, NEUTRAL);
}

TEST(ServoMath, VTail_PurePitch_BothRuddervatorEqualAndAboveNeutral)
{
    auto r = mixVTail(1.0f, 0.0f);
    EXPECT_EQ(r.left, r.right);
    EXPECT_GT(r.left, NEUTRAL);
}

TEST(ServoMath, VTail_PureYaw_RuddervatorOppositeAndSymmetric)
{
    // yaw=+1, pitch=0: left=mapFTI(1,-2,2,-80,80)=+40, right=mapFTI(-1,...)=-40
    auto r = mixVTail(0.0f, 1.0f);
    EXPECT_GT(r.left,  NEUTRAL);
    EXPECT_LT(r.right, NEUTRAL);
    EXPECT_EQ(r.left + r.right, 2 * NEUTRAL);
}

TEST(ServoMath, VTail_FullPitchAndYaw_RightAtNeutralLeftFullyDeflected)
{
    // pitch=+1, yaw=+1: pitch+yaw=2 → left fully deflected
    //                   pitch-yaw=0 → right at neutral
    auto r = mixVTail(1.0f, 1.0f);
    EXPECT_EQ(r.right, NEUTRAL);
    EXPECT_EQ(r.left,  NEUTRAL + DEFLECT);
}

TEST(ServoMath, VTail_RightInversion_MirrorsRightRuddervator)
{
    auto r_normal = mixVTail(1.0f, 0.0f, false, false);
    auto r_inv    = mixVTail(1.0f, 0.0f, false, true);
    EXPECT_EQ(r_inv.right, 2 * NEUTRAL - r_normal.right);
    EXPECT_EQ(r_inv.left,  r_normal.left);  // left side unchanged
}
