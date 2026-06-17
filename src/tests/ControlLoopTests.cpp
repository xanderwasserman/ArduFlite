/**
 * ControlLoopTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/tests/ControlLoopTests.h"
#include "src/utils/Logging.h"

#include <Arduino.h>

/// Tolerance band: outer loop target 10 ms, inner loop target 2 ms.
static constexpr float OUTER_TARGET_MS   = 10.0f;
static constexpr float INNER_TARGET_MS   =  2.0f;
/// Allow ±25 % deviation from target before flagging as a failure.
static constexpr float DT_TOLERANCE      =  0.25f;

/**
 * @brief Regression test for outer/inner loop dt computation.
 *
 * Reads LoopStats from the running controller and checks that the rolling
 * average dt is within ±25 % of the expected period.  Logs the full stats
 * table and a PASS / FAIL verdict via the Logger.
 *
 * Call once ~5 s after start-up so enough samples have accumulated.
 *
 * @param ctrl Reference to the running ArduFliteController.
 */
void runControlLoopTest_dtComputation(ArduFliteController &ctrl)
{
    LoopStats outer = ctrl.getOuterLoopStats();
    LoopStats inner = ctrl.getInnerLoopStats();

    LOG_INF("=== ControlLoop dt regression test ===");
    LOG_INF("Outer loop: avg=%.2f ms  max=%.2f ms  overruns=%lu  samples=%lu",
            outer.avgDt, outer.maxDt, outer.overrunCount, outer.sampleCount);
    LOG_INF("Inner loop: avg=%.2f ms  max=%.2f ms  overruns=%lu  samples=%lu",
            inner.avgDt, inner.maxDt, inner.overrunCount, inner.sampleCount);

    bool outerOk = (outer.sampleCount > 0) &&
                   (outer.avgDt >= OUTER_TARGET_MS * (1.0f - DT_TOLERANCE)) &&
                   (outer.avgDt <= OUTER_TARGET_MS * (1.0f + DT_TOLERANCE));

    bool innerOk = (inner.sampleCount > 0) &&
                   (inner.avgDt >= INNER_TARGET_MS * (1.0f - DT_TOLERANCE)) &&
                   (inner.avgDt <= INNER_TARGET_MS * (1.0f + DT_TOLERANCE));

    if (outerOk && innerOk)
    {
        LOG_INF("ControlLoop dt test: PASS");
    }
    else
    {
        if (!outerOk)
            LOG_ERR("ControlLoop dt test: FAIL — outer avg %.2f ms outside [%.2f, %.2f] ms",
                    outer.avgDt,
                    OUTER_TARGET_MS * (1.0f - DT_TOLERANCE),
                    OUTER_TARGET_MS * (1.0f + DT_TOLERANCE));
        if (!innerOk)
            LOG_ERR("ControlLoop dt test: FAIL — inner avg %.2f ms outside [%.2f, %.2f] ms",
                    inner.avgDt,
                    INNER_TARGET_MS * (1.0f - DT_TOLERANCE),
                    INNER_TARGET_MS * (1.0f + DT_TOLERANCE));
    }

    LOG_INF("======================================");
}
