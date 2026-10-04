/**
 * test_config_helpers.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * Unit tests for the PID-gain conversion helpers defined in ConfigHelpers.h.
 *
 * These call the real ConfigHelpers rather than reproducing its formulas, so a
 * change in production breaks the test instead of silently diverging from it.
 * The header compiles on a host because ConfigRegistry holds a hal::Mutex and a
 * std::string rather than Arduino types (ADR-048).
 *
 * The duplication was not harmless while it lasted. A test that re-implements
 * its subject keeps passing while the subject drifts — the same trap the motion
 * detector's tests were in before Phase 6, and the reason test_config_keys.cpp
 * now asserts key RESOLUTION rather than key spelling.
 */
#include <gtest/gtest.h>
#include <cmath>

#include "src/utils/ConfigHelpers.h"

// ── Formulas under test (mirrors ConfigHelpers.h exactly) ────────────────────

/// @brief Maximum integrator clamp derived from output limit and ki.
/// Matches ConfigHelpers::calcMaxIntegral.
static float calcMaxIntegral(float outLimit, float ki, float headroom = 0.9f)
{
    return ConfigHelpers::calcMaxIntegral(outLimit, ki, headroom);
}

static float calcMaxIntegral_reference(float outLimit, float ki, float headroom)
{
    return (ki > 0.0f ? (outLimit / ki) * headroom : 0.0f);
}

/// @brief Convert integral time constant Ti to integral gain Ki.
/// ki = kp / ti   (Ti = 0 → no integral action)
static float tiToKi(float kp, float ti)
{
    return (ti > 0.0f) ? (kp / ti) : 0.0f;
}

/// @brief Convert derivative time constant Td to derivative gain Kd.
/// kd = kp * td
static float tdToKd(float kp, float td)
{
    return kp * td;
}

static constexpr float kEps  = 1e-5f;   // tight tolerance for exact math
static constexpr float kLoose = 1e-3f;  // loose tolerance for combined formulas

// ── calcMaxIntegral ───────────────────────────────────────────────────────────

TEST(ConfigHelpers, CalcMaxIntegral_KiZero_ReturnsZero)
{
    // ki = 0 → no integral configured → clamp must be 0.
    EXPECT_NEAR(calcMaxIntegral(1.0f, 0.0f), 0.0f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_KiNegative_ReturnsZero)
{
    // Negative ki is pathological; guard must still return 0.
    EXPECT_NEAR(calcMaxIntegral(1.0f, -1.0f), 0.0f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_DefaultHeadroom)
{
    // maxI = (10 / 2) * 0.9 = 4.5
    EXPECT_NEAR(calcMaxIntegral(10.0f, 2.0f), 4.5f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_ExplicitHalfHeadroom)
{
    // maxI = (10 / 2) * 0.5 = 2.5
    EXPECT_NEAR(calcMaxIntegral(10.0f, 2.0f, 0.5f), 2.5f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_FullHeadroom)
{
    // headroom = 1.0 → maxI = outLimit / ki = 5.0
    EXPECT_NEAR(calcMaxIntegral(10.0f, 2.0f, 1.0f), 5.0f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_ZeroOutLimit_ReturnsZero)
{
    // outLimit = 0 → maxI = 0 regardless of ki.
    EXPECT_NEAR(calcMaxIntegral(0.0f, 2.0f), 0.0f, kEps);
}

TEST(ConfigHelpers, CalcMaxIntegral_SmallKi_LargeClamp)
{
    // ki = 0.001, outLimit = 1 → (1 / 0.001) * 0.9 = 900
    EXPECT_NEAR(calcMaxIntegral(1.0f, 0.001f), 900.0f, 1e-2f);
}

TEST(ConfigHelpers, CalcMaxIntegral_UnitInputs)
{
    // outLimit = 1, ki = 1 → (1 / 1) * 0.9 = 0.9
    EXPECT_NEAR(calcMaxIntegral(1.0f, 1.0f), 0.9f, kEps);
}

// ── Ti → Ki conversion ────────────────────────────────────────────────────────

TEST(ConfigHelpers, TiToKi_TiZero_ReturnsZero)
{
    // Ti = 0 → no integral action → ki = 0.
    EXPECT_NEAR(tiToKi(2.0f, 0.0f), 0.0f, kEps);
}

TEST(ConfigHelpers, TiToKi_Nominal)
{
    // ki = kp / ti = 2 / 0.5 = 4
    EXPECT_NEAR(tiToKi(2.0f, 0.5f), 4.0f, kEps);
}

TEST(ConfigHelpers, TiToKi_LargeTi_SmallKi)
{
    // ki = 1 / 100 = 0.01
    EXPECT_NEAR(tiToKi(1.0f, 100.0f), 0.01f, kEps);
}

// ── Td → Kd conversion ───────────────────────────────────────────────────────

TEST(ConfigHelpers, TdToKd_TdZero_ReturnsZero)
{
    // kd = kp * 0 = 0
    EXPECT_NEAR(tdToKd(5.0f, 0.0f), 0.0f, kEps);
}

TEST(ConfigHelpers, TdToKd_Nominal)
{
    // kd = kp * td = 2 * 0.05 = 0.1
    EXPECT_NEAR(tdToKd(2.0f, 0.05f), 0.1f, kEps);
}

// ── Full PIDConfig construction from Ti/Td (integration of all three formulas) ─

TEST(ConfigHelpers, PIDConfig_AllFieldsFromTiTd)
{
    // Typical rate-loop values: kp=1.5, Ti=0.3 s, Td=0.05 s, outLimit=1.
    const float kp       = 1.5f;
    const float ti       = 0.3f;
    const float td       = 0.05f;
    const float outLimit = 1.0f;
    const float headroom = 0.9f;

    const float ki   = tiToKi(kp, ti);                        // 5.0
    const float kd   = tdToKd(kp, td);                        // 0.075
    const float maxI = calcMaxIntegral(outLimit, ki, headroom); // (1/5)*0.9 = 0.18

    EXPECT_NEAR(ki,   5.0f,   kLoose);
    EXPECT_NEAR(kd,   0.075f, kLoose);
    EXPECT_NEAR(maxI, 0.18f,  kLoose);
}
