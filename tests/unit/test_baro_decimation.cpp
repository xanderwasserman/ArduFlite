/**
 * test_baro_decimation.cpp - Host tests for ArduFliteIMU inline baro decimation.
 *
 * Mirrors the decimation / climb-rate / altitude-EMA arithmetic that
 * ArduFliteIMU::update() now performs inline (the separate Baro Task was removed).
 * The altitude EMA and climb-rate derivative run at the baro rate (every
 * BARO_DECIMATION_FACTOR IMU ticks), and the first sample after boot/recalibration
 * seeds the filter without producing a false climb-rate spike.
 *
 * Keep these constants in sync with ArduFliteIMU.h.
 */
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>

namespace {

constexpr int   IMU_UPDATE_INTERVAL_MS  = 2;
constexpr int   BARO_UPDATE_INTERVAL_MS = 20;
constexpr int   BARO_DECIMATION_FACTOR  = BARO_UPDATE_INTERVAL_MS / IMU_UPDATE_INTERVAL_MS;  // 10
constexpr float baroDt                  = BARO_UPDATE_INTERVAL_MS * 0.001f;                  // 0.020 s

/**
 * Reproduces the per-tick baro state machine inside update():
 *  - read raw altitude only every BARO_DECIMATION_FACTOR ticks (skip non-finite reads)
 *  - on the first valid sample, seed filteredAltitude and emit climbRate == 0
 *  - thereafter low-pass the altitude and differentiate it, both at the baro rate
 */
struct BaroHarness {
    uint32_t baroTickCounter      = 0;
    float    altitude             = 0.0f;   // raw baro, held between decimation ticks
    float    filteredAltitude     = 0.0f;
    float    lastFilteredAlt      = 0.0f;
    float    climbRate            = 0.0f;
    bool     baroFilterInitialized = false;
    float    altiAlpha            = 0.2f;

    // rawAltitude is only consumed on a decimation tick. Returns true on a baro tick.
    bool tick(float rawAltitude) {
        bool baroUpdated = false;
        if (++baroTickCounter >= BARO_DECIMATION_FACTOR) {
            baroTickCounter = 0;
            if (std::isfinite(rawAltitude)) {   // matches the NaN/Inf guard in update()
                altitude = rawAltitude;
                if (!baroFilterInitialized) {
                    filteredAltitude      = altitude;
                    lastFilteredAlt       = altitude;
                    climbRate             = 0.0f;
                    baroFilterInitialized = true;
                } else {
                    filteredAltitude = altiAlpha * altitude + (1.0f - altiAlpha) * filteredAltitude;
                    climbRate = (filteredAltitude - lastFilteredAlt) / baroDt;
                    lastFilteredAlt = filteredAltitude;
                }
                baroUpdated = true;
            }
        }
        return baroUpdated;
    }
};

} // namespace

/// The decimation factor must stay derived from the interval ratio (header static_asserts).
TEST(BaroDecimation, FactorMatchesDerivedConstant) {
    EXPECT_EQ(BARO_DECIMATION_FACTOR, BARO_UPDATE_INTERVAL_MS / IMU_UPDATE_INTERVAL_MS);
    EXPECT_GT(BARO_DECIMATION_FACTOR, 0);
    EXPECT_EQ(BARO_UPDATE_INTERVAL_MS % IMU_UPDATE_INTERVAL_MS, 0);
}

/// Baro is read exactly once every BARO_DECIMATION_FACTOR ticks (~50 Hz).
TEST(BaroDecimation, BaroUpdatesEveryNthTick) {
    BaroHarness h;
    int updates = 0;
    for (int i = 1; i <= 100; ++i) if (h.tick(123.0f)) ++updates;
    EXPECT_EQ(updates, 100 / BARO_DECIMATION_FACTOR);
}

/// climbRate is held constant on the non-baro ticks between decimation boundaries.
TEST(BaroDecimation, ClimbRateUnchangedBetweenBaroTicks) {
    BaroHarness h;
    for (int i = 1; i < BARO_DECIMATION_FACTOR; ++i) {
        const float before = h.climbRate;
        EXPECT_FALSE(h.tick(50.0f));
        EXPECT_FLOAT_EQ(h.climbRate, before);
    }
    EXPECT_TRUE(h.tick(50.0f));  // tick 10: baro update
}

/// The first valid sample seeds the filter (climbRate == 0, no false spike).
TEST(BaroDecimation, FirstSampleSeedsWithoutSpike) {
    BaroHarness h;
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(500.0f);
    EXPECT_TRUE(h.baroFilterInitialized);
    EXPECT_NEAR(h.filteredAltitude, 500.0f, 1e-3f);
    EXPECT_FLOAT_EQ(h.climbRate, 0.0f);  // seeded, not differentiated against 0
}

/// climbRate uses the fixed baro interval, not the 2 ms IMU tick dt.
TEST(BaroDecimation, ClimbRateUsesFixedBaroDt) {
    BaroHarness h;
    h.altiAlpha = 1.0f;  // EMA passes raw through, so filteredAltitude == altitude
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(0.0f);  // window 1: seed at 0
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(1.0f);  // window 2: +1 m
    EXPECT_NEAR(h.climbRate, 1.0f / baroDt, 1e-3f);                 // 1 m / 0.02 s = 50 m/s
}

/// The altitude EMA converges toward a steady reading over successive baro windows.
TEST(BaroDecimation, AltitudeEmaConvergesToReading) {
    BaroHarness h;
    h.altiAlpha = 0.2f;
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(0.0f);  // seed at 0
    for (int window = 0; window < 200; ++window) {
        for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(500.0f);
    }
    EXPECT_NEAR(h.filteredAltitude, 500.0f, 1e-2f);
}

/// A non-finite reading is ignored: altitude is held and no climb-rate update occurs.
TEST(BaroDecimation, NonFiniteReadIsRejected) {
    BaroHarness h;
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) h.tick(100.0f);  // seed to 100 m
    const float heldAlt   = h.altitude;
    const float heldClimb = h.climbRate;
    const float nan = std::nanf("");
    for (int i = 0; i < BARO_DECIMATION_FACTOR; ++i) {
        EXPECT_FALSE(h.tick(nan));  // NaN read never counts as a baro update
    }
    EXPECT_FLOAT_EQ(h.altitude, heldAlt);
    EXPECT_FLOAT_EQ(h.climbRate, heldClimb);
    EXPECT_TRUE(std::isfinite(h.filteredAltitude));
}

/// The decimation counter stays bounded — it can never approach uint32_t overflow.
TEST(BaroDecimation, TickCounterStaysBounded) {
    BaroHarness h;
    for (int i = 0; i < 10000; ++i) {
        h.tick(0.0f);
        EXPECT_LE(h.baroTickCounter, static_cast<uint32_t>(BARO_DECIMATION_FACTOR - 1));
    }
}
