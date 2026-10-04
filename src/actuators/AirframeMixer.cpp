/**
 * AirframeMixer.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/actuators/AirframeMixer.h"

#include <cmath>

namespace arduflite::actuators {

namespace {

constexpr float clamp1(float v) noexcept
{
    if (v >  1.0f) { return  1.0f; }
    if (v < -1.0f) { return -1.0f; }
    return v;
}

} // namespace

SurfaceCommands AirframeMixer::mix(WingDesign design,
                                   float roll, float pitch, float yaw) noexcept
{
    // NaN/Inf guard. Holding the last position is the Actuator's job —
    // stage() ignores non-finite input — so the mixer only has to avoid
    // propagating the poison.
    if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    {
        return {};
    }

    roll  = clamp1(roll);
    pitch = clamp1(pitch);
    yaw   = clamp1(yaw);

    SurfaceCommands out{};

    switch (design)
    {
        case WingDesign::Conventional:
            out.elevator     = pitch;
            out.rudder       = yaw;
            out.aileronLeft  =  roll;
            // The differential lives in the MIX, not in the channel: the
            // right aileron is negated here, and per-channel invert flips it.
            out.aileronRight = -roll;
            break;

        case WingDesign::DeltaWing:
            // Elevon mixing. The halving keeps a simultaneous full-pitch and
            // full-roll demand inside one surface's travel.
            out.aileronLeft  = clamp1((pitch - roll) * 0.5f);
            out.aileronRight = clamp1((pitch + roll) * 0.5f);
            break;

        case WingDesign::VTail:
            // STUB — deliberately produces no deflection.
            //
            // The obvious ruddervator mix — left = (pitch + yaw)/2,
            // right = (pitch - yaw)/2 — ignores roll entirely, which leaves a
            // V-tail airframe with NO roll authority. Shipping that would be a
            // latent bug; inventing correct mixing would ship untested flight
            // code. Neither is acceptable, so the geometry
            // is reserved and left empty.
            //
            // isImplemented() exists so callers refuse this rather than flying an
            // aircraft whose surfaces never move. To implement it later you need
            // ruddervators for pitch+yaw AND separate ailerons for roll, which
            // also means more than four surfaces — see the board descriptor.
            break;
    }

    return out;
}

} // namespace arduflite::actuators
