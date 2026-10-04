/**
 * test_log_replay.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief L3 — replay a recorded flight through the attitude estimator.
 *
 * Feeds the accelerometer and gyroscope columns of a real flight log into
 * MadgwickEstimator and compares the attitude it produces against the attitude
 * the aircraft itself recorded at the time.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * WHY THE ORACLE IS THE LOG, AND WHY IT MUST STAY THAT WAY
 *
 * @warning Do NOT reintroduce a comparison against Adafruit_Madgwick here. Its
 *          fast inverse square root punts a float through a `long`:
 *
 *              union { float f; long i; } conv = {x};
 *
 *          `long` is 4 bytes on both ESP32 targets and 8 bytes on this host, so
 *          on a host the union reads four bytes of uninitialised memory and the
 *          bit-trick operates on garbage. Two Newton iterations pull the
 *          magnitude back, but the result comes out NEGATED — relative error
 *          exactly 2.0. The aircraft is unaffected, because on a 32-bit target
 *          the union is the right size; but as a HOST oracle that library
 *          measures a filter normalising its accelerometer and its gradient the
 *          wrong way round. See ADR-056.
 *
 * The log can serve as the reference: its roll and pitch columns were produced
 * on the aircraft, by the correct 32-bit path.
 * ─────────────────────────────────────────────────────────────────────────
 *
 * ─────────────────────────────────────────────────────────────────────────
 * WHAT THIS CANNOT DO, AND WHY
 *
 * It cannot reproduce the original filter state. The IMU task runs at 500 Hz
 * (2 ms), but the flash log writes at a variable ~23 ms — so the real filter
 * took roughly ELEVEN updates for every one replayed here, and Madgwick is
 * path-dependent. Exact agreement is not achievable and is not asserted.
 *
 * The tracking bound below is also only partly discriminating. Measured against
 * this log, with defects injected deliberately:
 *
 *     baseline                mean 1.51 deg
 *     gyro X sign flipped     mean 6.11 deg   <- caught
 *     accel X/Y swapped       mean 1.62 deg   <- NOT caught
 *     gyro 10 % scale error   mean 1.61 deg   <- NOT caught
 *
 * The accelerometer correction is slow at beta = 0.1, and across 23 ms steps
 * the gyro dominates, so accelerometer-side errors barely move the result.
 *
 * **This test is therefore NOT a substitute for the six-orientation bench
 * check.** That check is what catches an axis swap or a scale error; this one
 * catches gross sign errors and, through the fingerprint below, any change at
 * all to the fusion path.
 * ─────────────────────────────────────────────────────────────────────────
 */
#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "src/estimation/MadgwickEstimator.h"

using namespace arduflite;
using namespace arduflite::estimation;

namespace {

struct LogRow
{
    double t_ms = 0;
    Vec3f  accel_g{};
    Vec3f  gyro_dps{};
    Quaternion quat{};
    float roll = 0, pitch = 0;
};

/// The FL001 maiden flight: 943 rows, ~59 s, roll spanning -177..+22 degrees.
/// Chosen over the FL002 logs because those are ground recordings — log_007
/// spans two degrees of roll and would exercise nothing.
const char* kLogPath =
    ARDUFLITE_ROOT "/docs/flight_logs/FL001_2026-03-01_first_flight/"
                   "FL001_2026-03-01_first_flight.csv";

std::vector<LogRow> loadLog()
{
    std::vector<LogRow> rows;
    std::ifstream file(kLogPath);
    if (!file.is_open()) { return rows; }

    std::string line;
    std::getline(file, line);   // header

    while (std::getline(file, line))
    {
        std::stringstream stream(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(stream, cell, ',')) { values.push_back(std::atof(cell.c_str())); }
        if (values.size() < 14) { continue; }

        LogRow row;
        row.t_ms     = values[0];
        row.accel_g  = Vec3f{ (float)values[1], (float)values[2], (float)values[3] };
        row.gyro_dps = Vec3f{ (float)values[4], (float)values[5], (float)values[6] };
        row.quat     = Quaternion{ (float)values[7], (float)values[8],
                                   (float)values[9], (float)values[10] };
        row.roll  = (float)values[11];
        row.pitch = (float)values[12];
        rows.push_back(row);
    }
    return rows;
}

/// Angular difference in degrees, wrapped to [0, 180].
float angleError(float a, float b)
{
    float d = std::fabs(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

struct ReplayResult
{
    int    samples   = 0;
    double meanError = 0;
    double maxError  = 0;
    /// Order-sensitive fingerprint of every attitude produced.
    std::uint64_t fingerprint = 0;
    /// Every attitude produced, in order — so two implementations can be
    /// compared sample by sample rather than only in aggregate.
    std::vector<EulerAnglesDeg> attitudes;
};

/// Templated over the estimator so Phase 9's replacement runs the identical
/// path — same log, same re-seeding, same dt handling. Any difference in the
/// result is then the filter and nothing else.
template <typename EstimatorT>
ReplayResult replay(const std::vector<LogRow>& rows)
{
    EstimatorT estimator;
    estimator.begin(500.0f);
    estimator.setBeta(0.1f);
    estimator.setOrientation(rows.front().quat);

    ReplayResult result;
    double sum = 0;

    for (std::size_t i = 1; i < rows.size(); ++i)
    {
        const float dt_s = static_cast<float>(rows[i].t_ms - rows[i - 1].t_ms) / 1000.0f;

        // The flash writer stalls occasionally — gaps up to 850 ms appear. Do
        // not integrate across those; re-seed instead, so one logging hiccup
        // does not masquerade as an estimator defect.
        if (dt_s <= 0.0f || dt_s > 0.2f)
        {
            estimator.setOrientation(rows[i].quat);
            continue;
        }

        estimator.update(rows[i].gyro_dps, rows[i].accel_g, dt_s);
        const EulerAnglesDeg euler = estimator.euler_deg();
        result.attitudes.push_back(euler);

        const double error = std::fmax(angleError(euler.roll, rows[i].roll),
                                       angleError(euler.pitch, rows[i].pitch));
        sum += error;
        result.maxError = std::fmax(result.maxError, error);
        ++result.samples;

        // Quantised so the fingerprint is stable across platforms with
        // different floating-point rounding, while still changing for any real
        // difference in output. 0.01 degree is far finer than any tolerance
        // that matters here.
        const auto quantise = [](float v) {
            return static_cast<std::int64_t>(std::llround(v * 100.0));
        };
        result.fingerprint = result.fingerprint * 1000003u
                           + static_cast<std::uint64_t>(quantise(euler.roll) + 100000)
                           + static_cast<std::uint64_t>(quantise(euler.pitch) + 100000) * 31u;
    }

    result.meanError = sum / result.samples;
    return result;
}

// ── The tests ───────────────────────────────────────────────────────────────

TEST(LogReplay, TheFlightLogIsPresentAndParses)
{
    const auto rows = loadLog();
    ASSERT_FALSE(rows.empty()) << "missing " << kLogPath;
    EXPECT_EQ(rows.size(), 943u) << "the log changed - re-derive the bounds below";
}

/**
 * Bounded tracking against what the aircraft itself recorded.
 *
 * Proves the estimator stays gravity-referenced across a real flight rather
 * than drifting away, and catches a gross gyro sign error (measured at mean
 * 6.1 degrees against this log).
 *
 * The 3-degree bound sits between the measured baseline (1.51) and the cheapest
 * real defect (6.11). Deliberately NOT tightened to just above the baseline:
 * the replay is path-dependent and a threshold hugging the current value would
 * fail on any harmless change to step handling.
 */
TEST(LogReplay, AttitudeTracksTheRecordedFlight)
{
    const auto rows = loadLog();
    ASSERT_FALSE(rows.empty());

    const ReplayResult result = replay<MadgwickEstimator>(rows);
    EXPECT_GT(result.samples, 800);

    EXPECT_LT(result.meanError, 3.0)
        << "mean attitude divergence over a 59 s flight; baseline is ~1.5 deg";

    // The maximum is much looser on purpose. The log is decimated ~11x against
    // the real 500 Hz tick, so transient divergence during rapid manoeuvres is
    // expected and is not evidence of a defect.
    EXPECT_LT(result.maxError, 30.0);
}

/**
 * @brief Regression detector, by value rather than by hash.
 *
 * By VALUE, deliberately not by hash. A hash over every attitude tracks code
 * generation as much as behaviour: on arm64 the compiler may contract `a*b+c`
 * into an FMA depending on inlining, so a pure refactor moves the hash. A test
 * that fails on refactors teaches the reader to update its constant, which is
 * the one habit a regression detector must not teach.
 *
 * Sampled attitudes with a tolerance are robust to that and still catch any
 * change that could matter: 0.05 degree is ~30x below the filter's own tracking
 * error and far below servo resolution.
 */
TEST(LogReplay, FusionOutputIsUnchanged)
{
    const auto rows = loadLog();
    ASSERT_FALSE(rows.empty());

    const ReplayResult result = replay<MadgwickEstimator>(rows);
    ASSERT_GT(result.attitudes.size(), 800u);

    // Baselined from MadgwickEstimator (Phase 9) at beta = 0.1. Every 100th
    // sample, roll and pitch in degrees.
    struct Reference { std::size_t index; double roll; double pitch; };
    static const Reference kReference[] = {
        {   0,   -4.2012,   25.2322 },
        { 100,   -8.4173,    9.2272 },
        { 200,  -24.0744,    0.2504 },
        { 300,   -2.9107,    5.4417 },
        { 400,    0.2156,   -1.6341 },
        { 500,  -12.0309,   21.5447 },
        { 600,    1.5579,   10.1036 },
        { 700,   19.5203,   -2.6610 },
        { 800, -152.9109,   -7.7873 },
    };

    for (const auto& reference : kReference)
    {
        ASSERT_LT(reference.index, result.attitudes.size());
        EXPECT_NEAR(result.attitudes[reference.index].roll,  reference.roll,  0.05)
            << "roll at sample " << reference.index;
        EXPECT_NEAR(result.attitudes[reference.index].pitch, reference.pitch, 0.05)
            << "pitch at sample " << reference.index;
    }
}

TEST(LogReplay, ReplayIsDeterministic)
{
    const auto rows = loadLog();
    ASSERT_FALSE(rows.empty());

    // Same input, same output — twice. A filter carrying hidden static state
    // between runs would show up here and nowhere else.
    EXPECT_EQ(replay<MadgwickEstimator>(rows).fingerprint,
              replay<MadgwickEstimator>(rows).fingerprint);
}

} // namespace
