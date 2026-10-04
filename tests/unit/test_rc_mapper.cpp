/**
 * test_rc_mapper.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * The critical tests here are the EQUIVALENCE ones. The pilot input path runs
 * raw -> microseconds -> normalised, and any drift in that round trip shifts a
 * stick centre or an endpoint — silently re-trimming the aircraft. These pin it
 * against the reference raw-domain formulas below, which are what the aircraft
 * is trimmed against.
 */
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "src/hal/drivers/rc/CrsfLink.h"
#include "src/input/RcMapper.h"

using arduflite::drivers::CrsfLink;
using arduflite::input::ChannelMap;
using arduflite::input::ChannelShape;
using arduflite::input::RcMapper;

namespace {

// ── Reference formulas, used only as an ORACLE ─────────────────────────────
// Not the production path: these are what it must agree with, expressed
// directly in the raw domain against the CRSF endpoints (172 / 992 / 1811).

constexpr float kRawMin    = 172.0f;
constexpr float kRawCentre = 992.0f;
constexpr float kRawMax    = 1811.0f;

float referenceDualThrow(std::uint16_t raw)
{
    const float r = std::clamp(static_cast<float>(raw), kRawMin, kRawMax);
    return (r < kRawCentre) ? (r - kRawCentre) / (kRawCentre - kRawMin)
                            : (r - kRawCentre) / (kRawMax - kRawCentre);
}
float referenceSingleThrow(std::uint16_t raw)
{
    const float r = std::clamp(static_cast<float>(raw), kRawMin, kRawMax);
    return (r - kRawMin) / (kRawMax - kRawMin);
}
float referenceBoolean(std::uint16_t raw) { return raw > kRawCentre ? 1.0f : 0.0f; }

float referenceTriState(std::uint16_t raw, float lo, float hi)
{
    const float n = referenceSingleThrow(raw);
    if (n < lo) { return -1.0f; }
    if (n > hi) { return  1.0f; }
    return 0.0f;
}

float viaNewPath(ChannelShape s, std::uint16_t raw, float lo = 0.33f, float hi = 0.66f)
{
    ChannelMap m{};
    m.shape = s; m.thrLow = lo; m.thrHigh = hi;
    return RcMapper::shape(m, CrsfLink::rawToMicroseconds(raw));
}

} // namespace

// ── The us conversion itself ────────────────────────────────────────────────

TEST(CrsfScaling, AnchorsMatchTheCrsfEndpoints)
{
    // 172 / 992 / 1811 are what a transmitter actually sends at -100% / centre
    // / +100%. Anchoring anywhere else costs stick travel at both ends.
    EXPECT_EQ(CrsfLink::rawToMicroseconds(CrsfLink::kRawMin),    1000);
    EXPECT_EQ(CrsfLink::rawToMicroseconds(CrsfLink::kRawCentre), 1500);
    EXPECT_EQ(CrsfLink::rawToMicroseconds(CrsfLink::kRawMax),    2000);
}

/// Full stick must reach full deflection. Anchoring on the raw 0..2047 field
/// instead of the endpoints leaves ~20 % of the microsecond window unused,
/// which reads as an under-responsive aircraft rather than as a scaling bug.
TEST(CrsfScaling, FullStickReachesTheFullMicrosecondWindow)
{
    EXPECT_EQ(CrsfLink::rawToMicroseconds(CrsfLink::kRawMax)
              - CrsfLink::rawToMicroseconds(CrsfLink::kRawMin), 1000);
}

/// A radio configured beyond 100 % travel stops at the endpoint rather than
/// driving the servo past it.
TEST(CrsfScaling, ClampsBeyondTheEndpoints)
{
    EXPECT_EQ(CrsfLink::rawToMicroseconds(0),    1000);
    EXPECT_EQ(CrsfLink::rawToMicroseconds(2047), 2000);
}

TEST(CrsfScaling, IsMonotonicAcrossTheWholeRange)
{
    std::uint16_t prev = 0;
    for (std::uint32_t raw = 0; raw <= 2047; ++raw)
    {
        const std::uint16_t us = CrsfLink::rawToMicroseconds(static_cast<std::uint16_t>(raw));
        EXPECT_GE(us, prev) << "raw " << raw;
        EXPECT_GE(us, 1000);
        EXPECT_LE(us, 2000);
        prev = us;
    }
}

TEST(CrsfScaling, ClampsOutOfRangeRawValues)
{
    EXPECT_EQ(CrsfLink::rawToMicroseconds(65535), 2000);
}

// ── Equivalence with the reference shaping ─────────────────────────────────

TEST(RcMapperEquivalence, DualThrowMatchesTheReferenceAcrossTheRange)
{
    // 1 us of quantisation over a 1000 us span is 0.002 in normalised terms;
    // allow a shade more for float rounding.
    constexpr float kTol = 0.003f;

    for (std::uint32_t raw = 0; raw <= 2047; raw += 7)
    {
        const auto r = static_cast<std::uint16_t>(raw);
        EXPECT_NEAR(viaNewPath(ChannelShape::DualThrow, r), referenceDualThrow(r), kTol)
            << "raw " << raw;
    }
}

TEST(RcMapperEquivalence, DualThrowEndpointsAndCentreAreExact)
{
    // These three are the ones a pilot would notice: sticks centred must give
    // exactly zero, and full stick must give exactly full deflection.
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::DualThrow, CrsfLink::kRawMin),    -1.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::DualThrow, CrsfLink::kRawCentre),  0.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::DualThrow, CrsfLink::kRawMax),     1.0f);
}

TEST(RcMapperEquivalence, SingleThrowMatchesTheReference)
{
    constexpr float kTol = 0.003f;
    for (std::uint32_t raw = 0; raw <= 2047; raw += 7)
    {
        const auto r = static_cast<std::uint16_t>(raw);
        EXPECT_NEAR(viaNewPath(ChannelShape::SingleThrow, r), referenceSingleThrow(r), kTol)
            << "raw " << raw;
    }
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::SingleThrow, 0),    0.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::SingleThrow, 2047), 1.0f);
}

TEST(RcMapperEquivalence, BooleanThresholdMatchesTheReference)
{
    // The threshold is raw > 1024, i.e. strictly above centre. An
    // off-by-one here means a switch that arms one position early.
    for (std::uint32_t raw = 0; raw <= 2047; raw += 3)
    {
        const auto r = static_cast<std::uint16_t>(raw);
        const float want = referenceBoolean(r);
        const float got  = viaNewPath(ChannelShape::Boolean, r);

        // Only the immediate neighbourhood of the threshold may disagree, and
        // only because 2048 raw steps map to 1001 microsecond steps.
        if (raw < 1020 || raw > 1028)
        {
            EXPECT_FLOAT_EQ(got, want) << "raw " << raw;
        }
    }
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::Boolean, 0),    0.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::Boolean, 2047), 1.0f);
}

TEST(RcMapperEquivalence, TriStateMatchesTheReferenceAwayFromThresholds)
{
    constexpr float lo = 0.33f, hi = 0.66f;
    for (std::uint32_t raw = 0; raw <= 2047; raw += 5)
    {
        const auto  r = static_cast<std::uint16_t>(raw);
        const float n = static_cast<float>(raw) / 2047.0f;
        // Skip a narrow band either side of each threshold, where quantisation
        // can legitimately land on the other side.
        if (std::fabs(n - lo) < 0.004f || std::fabs(n - hi) < 0.004f) { continue; }

        EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::TriState, r, lo, hi),
                        referenceTriState(r, lo, hi)) << "raw " << raw;
    }
}

TEST(RcMapperEquivalence, TriStateHitsAllThreePositions)
{
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::TriState, 100),  -1.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::TriState, 1024),  0.0f);
    EXPECT_FLOAT_EQ(viaNewPath(ChannelShape::TriState, 2000),  1.0f);
}

// ── Callback dispatch ───────────────────────────────────────────────────────

namespace {
int   g_calls = 0;
std::uint8_t g_lastCh = 0xFF;
float g_lastVal = 0.0f;
void  recordCb(std::uint8_t ch, float v) { ++g_calls; g_lastCh = ch; g_lastVal = v; }

arduflite::device::RcFrame frameWith(std::uint8_t ch, std::uint16_t us)
{
    arduflite::device::RcFrame f{};
    f.channelCount = 16;
    for (auto& c : f.channel_us) { c = 1500; }
    f.channel_us[ch] = us;
    return f;
}
} // namespace

class RcMapperCallbacks : public ::testing::Test
{
protected:
    void SetUp() override { g_calls = 0; g_lastCh = 0xFF; g_lastVal = 0.0f; }
};

TEST_F(RcMapperCallbacks, FirstFrameSeedsWithoutFiring)
{
    // A switch already ON at power-up must not look like a fresh toggle —
    // otherwise booting with the arm switch up would fire onArm immediately.
    RcMapper m;
    m.configure(4, { ChannelShape::Boolean, 0.33f, 0.66f, recordCb });

    m.apply(frameWith(4, 2000));
    EXPECT_EQ(g_calls, 0) << "first frame must seed, not fire";
}

TEST_F(RcMapperCallbacks, FiresOnChangeOnly)
{
    RcMapper m;
    m.configure(4, { ChannelShape::Boolean, 0.33f, 0.66f, recordCb });

    m.apply(frameWith(4, 1000));   // seed
    m.apply(frameWith(4, 2000));   // change -> fire
    EXPECT_EQ(g_calls, 1);
    EXPECT_EQ(g_lastCh, 4);
    EXPECT_FLOAT_EQ(g_lastVal, 1.0f);

    m.apply(frameWith(4, 2000));   // same -> no fire
    EXPECT_EQ(g_calls, 1);
}

TEST_F(RcMapperCallbacks, UnusedAndUnconfiguredChannelsNeverFire)
{
    RcMapper m;   // nothing configured
    m.apply(frameWith(7, 1000));
    m.apply(frameWith(7, 2000));
    EXPECT_EQ(g_calls, 0);
}
