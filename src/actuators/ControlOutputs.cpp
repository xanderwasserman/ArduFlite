/**
 * ControlOutputs.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/actuators/ControlOutputs.h"

namespace arduflite::actuators {

ControlOutputs ControlOutputs::resolve(device::ActuatorBank& bank)
{
    ControlOutputs o{};
    o.aileronLeft  = bank.byRole("aileron_left");
    o.aileronRight = bank.byRole("aileron_right");
    o.elevator     = bank.byRole("elevator");
    o.rudder       = bank.byRole("rudder");
    o.throttle     = bank.byRole("throttle");
    return o;
}

void ControlOutputs::stageSurfaces(const SurfaceCommands& c) const
{
    if (aileronLeft  != nullptr) { aileronLeft->stage(c.aileronLeft); }
    if (aileronRight != nullptr) { aileronRight->stage(c.aileronRight); }
    if (elevator     != nullptr) { elevator->stage(c.elevator); }
    if (rudder       != nullptr) { rudder->stage(c.rudder); }
}

void ControlOutputs::stageThrottle(float normalised) const
{
    if (throttle != nullptr) { throttle->stage(normalised); }
}

} // namespace arduflite::actuators
