/**
 * AltitudeFilter.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/estimation/AltitudeFilter.h"

#include <cmath>

#include "src/hal/core/Units.h"

namespace arduflite::estimation {

float AltitudeFilter::pressureToAltitude_m(float pressure_hpa) const
{
    return units::altitudeFromPressure_m(pressure_hpa, _referencePressure_hpa);
}

void AltitudeFilter::reset()
{
    _seeded        = false;
    _climbRate_mps = 0.0f;
}

bool AltitudeFilter::update(float pressure_hpa, float dt_s)
{
    const float altitude_m = pressureToAltitude_m(pressure_hpa);
    if (!std::isfinite(altitude_m))
    {
        return false;
    }

    if (!_seeded)
    {
        // Seed WITHOUT computing a derivative. The first sample after boot or
        // after a reset has no predecessor to differentiate against; treating
        // the previous value (zero, or a pre-gap altitude) as one manufactures a
        // climb rate of tens of metres per second out of nothing, which the
        // flight state machine can read as a launch.
        _filtered_m     = altitude_m;
        _lastFiltered_m = altitude_m;
        _climbRate_mps  = 0.0f;
        _seeded         = true;
        return true;
    }

    _filtered_m = _alpha * altitude_m + (1.0f - _alpha) * _filtered_m;

    if (dt_s > 0.0f)
    {
        _climbRate_mps = (_filtered_m - _lastFiltered_m) / dt_s;
    }
    _lastFiltered_m = _filtered_m;
    return true;
}

} // namespace arduflite::estimation
