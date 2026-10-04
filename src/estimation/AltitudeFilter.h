/**
 * AltitudeFilter.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Barometric altitude, smoothed, plus the climb rate derived from it.
 *
 * Runs at the BAROMETER's rate, not the task rate. Filtering at 500 Hz a signal
 * that updates at ~15 Hz would make the EMA's real cutoff depend on how many
 * ticks happen to fall between conversions, so the alpha would no longer mean
 * what it says.
 */
#ifndef ARDUFLITE_ESTIMATION_ALTITUDE_FILTER_H
#define ARDUFLITE_ESTIMATION_ALTITUDE_FILTER_H

#include "src/hal/core/Vec3.h"

namespace arduflite::estimation {

class AltitudeFilter
{
public:
    void setAlpha(float alpha) { _alpha = alpha; }

    /// Ground-level reference, in hPa. Altitude is measured against this.
    void setReferencePressure_hpa(float hpa) { _referencePressure_hpa = hpa; }
    [[nodiscard]] float referencePressure_hpa() const { return _referencePressure_hpa; }

    /**
     * @brief Convert a pressure reading to altitude above the reference.
     *
     * Single-precision powf, not the double-precision barometric formula: the
     * ESP32-C3 is rv32imc with a soft-float ABI and has NO FPU, so both are
     * emulated but double costs several times more. This runs inside the
     * sampling task's critical section.
     *
     * @return metres, or a non-finite value if the pressure was not usable.
     */
    [[nodiscard]] float pressureToAltitude_m(float pressure_hpa) const;

    /**
     * @brief Feed one barometer sample.
     *
     * @param pressure_hpa station pressure; values <= 0 are rejected
     * @param dt_s         seconds since the previous accepted sample
     * @return true if the sample was accepted
     *
     * @note A single NaN would permanently poison the EMA — every subsequent
     *       term propagates it — and with it the climb rate and the published
     *       snapshot. Bad samples are therefore dropped and the last good
     *       altitude retained.
     */
    bool update(float pressure_hpa, float dt_s);

    [[nodiscard]] float altitude_m()    const { return _filtered_m; }
    [[nodiscard]] float climbRate_mps() const { return _climbRate_mps; }

    /**
     * @brief Forget the history so the next sample re-seeds.
     *
     * Called after anything that breaks continuity — recalibration, or a gap in
     * sampling. Without it the next sample is differentiated against a stale one
     * across the gap, producing a climb-rate spike out of nothing.
     */
    void reset();

    [[nodiscard]] bool seeded() const { return _seeded; }

private:
    float _referencePressure_hpa = 1013.25f;
    float _alpha                 = 1.0f;

    float _filtered_m     = 0.0f;
    float _lastFiltered_m = 0.0f;
    float _climbRate_mps  = 0.0f;
    bool  _seeded         = false;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_ALTITUDE_FILTER_H
