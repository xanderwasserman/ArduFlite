/**
 * test_dt.cpp — the control loops' dt clamp.
 *
 * Both loops clamp dt to [1 ms, 20 ms] before it reaches a PID. The floor stops
 * a division blowing up on a near-zero interval; the ceiling stops the first
 * tick after a stall being integrated as one enormous step.
 *
 * The arithmetic here MIRRORS the loops rather than calling them — the clamp is
 * three lines inside a task body and cannot be reached from a host. The mirror
 * is therefore tied to the real source by a contract test in
 * test_production_contracts.cpp, which fails if the constants drift apart.
 *
 * @note There is nothing here about counter wraparound. Timing comes from
 *       hal::Clock, whose 64-bit microsecond count does not wrap in any
 *       realistic runtime, so the loops never do modular subtraction.
 */
#include <gtest/gtest.h>
#include <cstdint>
#include <algorithm>

// ── Constants mirrored from ArduFliteController.cpp ──────────────────────────
//    Pinned to the real source by ControlLoopDtClampIsUnchanged in
//    test_production_contracts.cpp — change them here and that test fails.
static constexpr unsigned long OUTER_PERIOD_US = 10000UL;  // 100 Hz
static constexpr unsigned long INNER_PERIOD_US =  2000UL;  // 500 Hz
static constexpr unsigned long MIN_DT_US =  1000UL;        // 1 ms floor
static constexpr unsigned long MAX_DT_US = 20000UL;        // 20 ms ceiling

// Reproduces the exact dt computation from the outer/inner loop tasks.
static float computeDt(uint32_t currentMicros, uint32_t lastMicros)
{
    unsigned long dtMicro = (unsigned long)(currentMicros - lastMicros);
    if (dtMicro < MIN_DT_US) dtMicro = MIN_DT_US;
    if (dtMicro > MAX_DT_US) dtMicro = MAX_DT_US;
    return static_cast<float>(dtMicro) / 1e6f;
}





// ── Clamping tests ────────────────────────────────────────────────────────────

TEST(DtComputation, Clamp_DtBelowFloor_ClampsToFloor)
{
    // current - last = 100 us, below MIN_DT_US (1 ms) -> clamped
    float dt = computeDt(1000100UL, 1000000UL);
    EXPECT_NEAR(dt, static_cast<float>(MIN_DT_US) / 1e6f, 1e-9f);
}

TEST(DtComputation, Clamp_DtAboveCeiling_ClampsToCeiling)
{
    // current - last = 200 ms, above MAX_DT_US (20 ms) -> clamped
    float dt = computeDt(1200000UL, 1000000UL);
    EXPECT_NEAR(dt, static_cast<float>(MAX_DT_US) / 1e6f, 1e-9f);
}

TEST(DtComputation, Clamp_DtExactlyCeiling_Unclamped)
{
    float dt = computeDt(1020000UL, 1000000UL);
    EXPECT_NEAR(dt, static_cast<float>(MAX_DT_US) / 1e6f, 1e-9f);
}

TEST(DtComputation, Clamp_NominalOuterLoop_Unclamped)
{
    // Exact outer loop period — should pass through unchanged.
    float dt = computeDt(1010000UL, 1000000UL);
    EXPECT_NEAR(dt, static_cast<float>(OUTER_PERIOD_US) / 1e6f, 1e-9f);
}

TEST(DtComputation, Clamp_NominalInnerLoop_Unclamped)
{
    float dt = computeDt(1002000UL, 1000000UL);
    EXPECT_NEAR(dt, static_cast<float>(INNER_PERIOD_US) / 1e6f, 1e-9f);
}
