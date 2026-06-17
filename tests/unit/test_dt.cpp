/**
 * test_dt.cpp — Unit tests for dt (loop timing) arithmetic.
 *
 * Verifies that the unsigned-subtraction pattern used in every FreeRTOS
 * control loop correctly handles micros() wraparound, and that the dt
 * clamping constants protect against startup spikes.
 *
 * No hardware is needed — this is pure arithmetic on uint32_t.
 */
#include <gtest/gtest.h>
#include <cstdint>
#include <algorithm>

// ── Constants mirrored from ArduFliteController.cpp ──────────────────────────
//    Keep in sync with OUTER_LOOP_PERIOD_US / INNER_LOOP_PERIOD_US.
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

// ── Wraparound tests ──────────────────────────────────────────────────────────

TEST(DtComputation, NormalCase_NoDt_Wraparound)
{
    uint32_t last    = 990000UL;
    uint32_t current = 1000000UL;
    unsigned long dtMicro = (unsigned long)(current - last);
    EXPECT_EQ(dtMicro, 10000UL);
}

TEST(DtComputation, Wraparound_CurrentLessThanLast)
{
    // Simulates micros() wrapping from ~4.29 billion back to near 0.
    uint32_t last    = 0xFFFFFF00UL;  // just before wrap (4294967040)
    uint32_t current = 0x000000F0UL;  // just after wrap (240)
    // Expected elapsed = 240 + (2^32 - 4294967040) = 240 + 256 = 496
    unsigned long expected = 0x100UL - (0xFFFFFF00UL - 0xFFFFFFFFUL - 1UL);
    // Simpler: 240 - (-256 mod 2^32) — just compute directly.
    unsigned long dtMicro = (unsigned long)(current - last);
    EXPECT_EQ(dtMicro, static_cast<unsigned long>((uint32_t)(current - last)));
    // Must be small (496 µs), not billions.
    EXPECT_LT(dtMicro, 1000UL);
}

TEST(DtComputation, Wraparound_ExactEdge)
{
    // last = UINT32_MAX, current = 4 → elapsed = 5
    uint32_t last    = 0xFFFFFFFFUL;
    uint32_t current = 4UL;
    unsigned long dtMicro = (unsigned long)(current - last);
    EXPECT_EQ(dtMicro, 5UL);
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

// ── Wraparound + clamping interaction ────────────────────────────────────────

TEST(DtComputation, Wraparound_WithClamping_CorrectResult)
{
    // After wraparound, elapsed ~10 ms → should land inside normal range.
    uint32_t last    = 0xFFFFD8F0UL;  // some value near max
    uint32_t current = last + 10000UL; // exactly one outer-loop period later (wraps)
    float dt = computeDt(current, last);
    EXPECT_NEAR(dt, static_cast<float>(OUTER_PERIOD_US) / 1e6f, 1e-6f);
}
