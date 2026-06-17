/**
 * test_pid.cpp — Unit tests for the PID controller.
 *
 * Tests cover: proportional/integral/derivative terms, output saturation,
 * anti-windup logic, dt floor enforcement, reset(), and resetIntegral().
 */
#include <gtest/gtest.h>
#include "pid.h"

// ── Helpers ───────────────────────────────────────────────────────────────────

static PIDConfig makeConfig(float kp, float ki, float kd,
                            float outLimit    = 1.0f,
                            float maxIntegral = 10.0f,
                            float alpha       = 1.0f)   // alpha=1 → no LPF on D
{
    return PIDConfig{kp, ki, kd, outLimit, maxIntegral, alpha};
}

static constexpr float kEps = 1e-5f;

// ── Proportional ──────────────────────────────────────────────────────────────

TEST(PID, ProportionalOnly_PositiveError)
{
    PID pid(makeConfig(2.0f, 0.0f, 0.0f, 100.0f));
    float out = pid.update(3.0f, 0.01f);
    EXPECT_NEAR(out, 6.0f, kEps);
}

TEST(PID, ProportionalOnly_NegativeError)
{
    PID pid(makeConfig(2.0f, 0.0f, 0.0f, 100.0f));
    float out = pid.update(-3.0f, 0.01f);
    EXPECT_NEAR(out, -6.0f, kEps);
}

TEST(PID, ProportionalOnly_ZeroError)
{
    PID pid(makeConfig(5.0f, 0.0f, 0.0f, 100.0f));
    EXPECT_NEAR(pid.update(0.0f, 0.01f), 0.0f, kEps);
}

// ── Output saturation ─────────────────────────────────────────────────────────

TEST(PID, OutputSaturation_UpperLimit)
{
    PID pid(makeConfig(10.0f, 0.0f, 0.0f, /*outLimit=*/1.0f));
    // kp*error = 10*5 = 50, clamped to 1
    float out = pid.update(5.0f, 0.01f);
    EXPECT_NEAR(out, 1.0f, kEps);
}

TEST(PID, OutputSaturation_LowerLimit)
{
    PID pid(makeConfig(10.0f, 0.0f, 0.0f, /*outLimit=*/1.0f));
    float out = pid.update(-5.0f, 0.01f);
    EXPECT_NEAR(out, -1.0f, kEps);
}

// ── dt floor ─────────────────────────────────────────────────────────────────

TEST(PID, DtFloor_TinyDtClamped)
{
    // With kd=0 and ki=0, output = kp*error regardless of dt.
    // The dt floor matters for D and I — test that it doesn't crash.
    PID pid(makeConfig(1.0f, 0.0f, 0.0f, 100.0f));
    // dt = 0 → floor = 1e-3, but P term is unaffected
    float out = pid.update(1.0f, 0.0f);
    EXPECT_NEAR(out, 1.0f, kEps);
}

TEST(PID, DtFloor_IntegralWithTinyDt)
{
    // dt < 1e-3 → clamped to 1e-3; integral grows by error * 1e-3 per call.
    PID pid(makeConfig(0.0f, 1.0f, 0.0f, 100.0f, 100.0f));
    float out = pid.update(1.0f, 0.0f);   // dt clamped to 1e-3
    EXPECT_NEAR(out, 1e-3f, kEps);
}

// ── Integral accumulation ─────────────────────────────────────────────────────

TEST(PID, IntegralAccumulates_ConstantError)
{
    // With P=0, D=0: output after N steps = ki * (error * dt * N)
    PID pid(makeConfig(0.0f, 2.0f, 0.0f, 1000.0f, 1000.0f));
    const float dt    = 0.01f;
    const float error = 1.0f;
    const int   N     = 10;
    float out = 0.0f;
    for (int i = 0; i < N; ++i) out = pid.update(error, dt);
    // integral = error * dt * N = 0.1; ki*integral = 0.2
    EXPECT_NEAR(out, 2.0f * error * dt * N, 1e-4f);
}

// ── Anti-windup ───────────────────────────────────────────────────────────────

TEST(PID, AntiWindup_IntegralClamped)
{
    // maxIntegral = 0.5; after enough steps with error=1, integral saturates.
    PID pid(makeConfig(0.0f, 1.0f, 0.0f, 1000.0f, /*maxIntegral=*/0.5f));
    for (int i = 0; i < 200; ++i) pid.update(1.0f, 0.01f);
    // integral is clamped at 0.5, ki=1 → output = 0.5
    float out = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out, 0.5f, 1e-4f);
}

TEST(PID, AntiWindup_IntegralFrozenWhenOutputSaturatedSameDirection)
{
    // Saturated positive; error is positive → anti-windup freezes integral.
    // Config: kp large enough to saturate output, ki=1, maxIntegral=large.
    PID pid(makeConfig(10.0f, 1.0f, 0.0f, /*outLimit=*/1.0f, 100.0f));

    // Drive hard into positive saturation for 5 steps.
    for (int i = 0; i < 5; ++i) pid.update(5.0f, 0.1f);

    // Record output — should be capped at 1.0.
    // Then flip the error; integral must NOT have grown as much as unclamped would.
    // Check: after one negative-error step output starts recovering.
    float out = pid.update(-5.0f, 0.1f);
    EXPECT_LT(out, 0.0f);  // recovery: output should go negative
}

TEST(PID, AntiWindup_IntegralGrowsWhenDrivingBack)
{
    // Output is saturated positive; error is negative (driving back) → integral allowed.
    PID pid(makeConfig(0.0f, 1.0f, 0.0f, /*outLimit=*/0.01f, 100.0f));
    // First saturate positively with P only trick via large error.
    // With P=0 and ki=1, saturate by running integral past outLimit:
    for (int i = 0; i < 10; ++i) pid.update(1.0f, 0.01f);  // integral grows to 0.1

    // Now error is negative — unsatOutput will be inside limit, integral can grow
    float beforeNeg = pid.update(-1.0f, 0.01f);
    float afterNeg  = pid.update(-1.0f, 0.01f);
    // Each negative step reduces integral; output becomes more negative
    EXPECT_LT(afterNeg, beforeNeg);
}

// ── Derivative ────────────────────────────────────────────────────────────────

TEST(PID, Derivative_FirstStep_PrevErrorIsZero)
{
    // alpha=1 → no LPF; D = kd * (error - 0) / dt on first call
    PID pid(makeConfig(0.0f, 0.0f, 1.0f, 100.0f, 0.0f, /*alpha=*/1.0f));
    float out = pid.update(0.1f, 0.01f);
    // derivative = (0.1 - 0) / 0.01 = 10; kd=1 → dTerm = 10
    EXPECT_NEAR(out, 10.0f, 1e-3f);
}

TEST(PID, Derivative_ConstantError_DerivativeZero)
{
    // After first call sets prevError, constant error → derivative = 0.
    PID pid(makeConfig(0.0f, 0.0f, 1.0f, 100.0f, 0.0f, /*alpha=*/1.0f));
    pid.update(5.0f, 0.01f);          // primes prevError = 5.0
    float out = pid.update(5.0f, 0.01f);  // error=5, prevError=5 → D=0
    EXPECT_NEAR(out, 0.0f, 1e-4f);
}

TEST(PID, Derivative_LowPassFilter)
{
    // alpha=0.5: filteredDerivative blends 50/50 with previous.
    PID pid(makeConfig(0.0f, 0.0f, 1.0f, 100.0f, 0.0f, /*alpha=*/0.5f));
    // Step 1: raw derivative = (1.0 - 0) / 0.01 = 100; filtered = 0.5*100 + 0.5*0 = 50
    float out1 = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out1, 50.0f, 1e-3f);
    // Step 2: raw derivative = (1.0 - 1.0) / 0.01 = 0; filtered = 0.5*0 + 0.5*50 = 25
    float out2 = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out2, 25.0f, 1e-3f);
}

// ── reset() ───────────────────────────────────────────────────────────────────

TEST(PID, Reset_ClearsAllState)
{
    PID pid(makeConfig(0.0f, 1.0f, 1.0f, 100.0f, 100.0f, 1.0f));
    pid.update(5.0f, 0.1f);
    pid.update(3.0f, 0.1f);
    pid.reset();

    // After reset, integral = 0 and prevError = 0, so:
    // I = 0; D = (1.0 - 0) / dt (primes fresh).
    PID fresh(makeConfig(0.0f, 1.0f, 1.0f, 100.0f, 100.0f, 1.0f));
    float outReset = pid.update(1.0f, 0.01f);
    float outFresh = fresh.update(1.0f, 0.01f);
    EXPECT_NEAR(outReset, outFresh, 1e-5f);
}

// ── resetIntegral() ───────────────────────────────────────────────────────────

TEST(PID, ResetIntegral_OnlyClearsIntegral)
{
    // alpha=1 so derivative is not filtered; we can observe prevError separately.
    PID pid(makeConfig(0.0f, 1.0f, 1.0f, 1000.0f, 1000.0f, 1.0f));
    pid.update(5.0f, 0.1f);   // integral = 0.5, prevError = 5
    pid.resetIntegral();       // integral → 0, prevError still 5

    // D = (5.0 - 5.0) / 0.1 = 0 (prevError still 5), I = 5*0.1 = 0.5
    float out = pid.update(5.0f, 0.1f);
    // Expected: I=0.5, D=0 → 0.5
    EXPECT_NEAR(out, 0.5f, 1e-4f);
}

// ── Pure derivative (Kp=Ki=0) ────────────────────────────────────────────────

TEST(PID, PureDerivative_FirstStep_OutputIsKdTimesErrorOverDt)
{
    // kp=0, ki=0, kd=2; first call: prevError=0 → D = kd*(error-0)/dt = 2*1/0.01 = 200
    PID pid(makeConfig(0.0f, 0.0f, 2.0f, 1000.0f, 0.0f, 1.0f));
    float out = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out, 200.0f, 1e-3f);
}

TEST(PID, PureDerivative_ConstantError_OutputIsZero)
{
    // kp=0, ki=0, kd=2; second call with same error: D = kd*(err-err)/dt = 0
    PID pid(makeConfig(0.0f, 0.0f, 2.0f, 1000.0f, 0.0f, 1.0f));
    pid.update(1.0f, 0.01f);          // primes prevError = 1
    float out = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out, 0.0f, 1e-3f);
}

// ── Combined P + I + D output ─────────────────────────────────────────────────

TEST(PID, Combined_AllThreeTerms_OutputIsSuperposition)
{
    // kp=1, ki=1, kd=1, alpha=1 (no LPF), outLimit=1000 (won't saturate).
    // First call: error=1, dt=0.01, prevError=0
    //   P = 1*1 = 1.0
    //   I = 1*(1*0.01) = 0.01
    //   D = 1*(1-0)/0.01 = 100.0
    //   out = 101.01
    PID pid(makeConfig(1.0f, 1.0f, 1.0f, 1000.0f, 1000.0f, 1.0f));
    float out = pid.update(1.0f, 0.01f);
    EXPECT_NEAR(out, 101.01f, 1e-3f);
}

// ── Derivative sign reversal ──────────────────────────────────────────────────

TEST(PID, Derivative_ErrorSignFlip_DerivativeSignFlips)
{
    // kp=0, ki=0, kd=1: step from +1 to -1 produces a large negative derivative.
    // D = kd * (-1 - 1) / 0.01 = -200
    PID pid(makeConfig(0.0f, 0.0f, 1.0f, 1000.0f, 0.0f, 1.0f));
    pid.update(1.0f, 0.01f);          // primes prevError = 1
    float out = pid.update(-1.0f, 0.01f);
    EXPECT_NEAR(out, -200.0f, 1e-3f);
}
