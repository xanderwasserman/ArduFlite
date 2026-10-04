/**
 * BaroTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/tests/BaroTests.h"
#include "src/utils/Logging.h"

#include <Arduino.h>
#include <math.h>

/// Stationary on the ground, the climb rate must sit near zero (no boot/seed spike).
static constexpr float CLIMB_RATE_GROUND_TOL = 0.5f;   // m/s
/// Number of back-to-back snapshot reads used to probe seqlock coherence.
static constexpr int   SNAPSHOT_PROBE_READS  = 2000;
/// A coherent orientation quaternion must stay close to unit norm.
static constexpr float QUAT_NORM_TOL         = 0.1f;

/**
 * @brief Verifies boot seeding, baro finiteness, and snapshot health on the running IMU.
 *
 * Logs the full state and a PASS / FAIL verdict via the Logger. Call once ~5 s after
 * start-up while STATIONARY on the ground (see header).
 *
 * @param imu Reference to the running arduflite::estimation::InertialSubsystem.
 */
void runBaroTest_seedAndSnapshotHealth(arduflite::estimation::InertialSubsystem &imu)
{
    LOG_INF("=== Baro seed + snapshot health test ===");

    // 1) Boot-seed postcondition: finite altitude, and no false climb-rate spike on the ground.
    const arduflite::estimation::ImuState state = imu.state();
    const float alt   = state.altitude_m;
    const float climb = state.climbRate_mps;
    const bool altOk   = !isnan(alt)   && !isinf(alt);
    const bool climbOk = !isnan(climb) && !isinf(climb) && fabsf(climb) <= CLIMB_RATE_GROUND_TOL;
    LOG_INF("Altitude=%.2f m  climbRate=%.3f m/s", alt, climb);

    // 2) Snapshot coherence under the live 500 Hz writer: the seqlock must never hand
    //    back a torn copy, so the orientation quaternion stays near unit norm.
    bool coherent = true;
    for (int i = 0; i < SNAPSHOT_PROBE_READS; ++i)
    {
        const arduflite::estimation::ImuState s = imu.state();
        const auto& q = s.orientation_quat;
        const float n = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
        if (isnan(n) || fabsf(n - 1.0f) > QUAT_NORM_TOL)
        {
            coherent = false;
            break;
        }
    }

    // 3) Snapshot read health: the stale-coherent fallback should never have been needed.
    const auto h = imu.snapshotHealth();
    LOG_INF("Snapshot health: totalRetries=%lu maxRetries=%lu retryLimitHits=%lu",
            (unsigned long)h.totalRetries,
            (unsigned long)h.maxRetries,
            (unsigned long)h.retryLimitHits);
    const bool healthOk = (h.retryLimitHits == 0);

    if (altOk && climbOk && coherent && healthOk)
    {
        LOG_INF("Baro test: PASS");
    }
    else
    {
        if (!altOk)
            LOG_ERR("Baro test: FAIL — altitude NaN/Inf");
        if (!climbOk)
            LOG_ERR("Baro test: FAIL — ground climbRate %.3f m/s exceeds +/-%.2f (seed spike?)",
                    climb, CLIMB_RATE_GROUND_TOL);
        if (!coherent)
            LOG_ERR("Baro test: FAIL — torn snapshot (orientation quat not unit-norm)");
        if (!healthOk)
            LOG_ERR("Baro test: FAIL — %lu snapshot retry-limit hits (writer starving readers)",
                    (unsigned long)h.retryLimitHits);
    }

    LOG_INF("========================================");
}
