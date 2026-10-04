/**
 * ControlOutputs.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Named actuator references, resolved by role once at composition time.
 *
 * The single path from normalised axis demands to hardware. Two things
 * it buys beyond readability:
 *
 *  - No magic indices. An index/role mismatch after a board-descriptor edit is a
 *    bug class that stops existing.
 *  - A Latching actuator (parachute, payload release) is simply NOT a member
 *    here, so the mixer STRUCTURALLY cannot reach it. That is a compile-time
 *    guarantee replacing a runtime ActuatorKind check somebody has to remember.
 */
#ifndef ARDUFLITE_ACTUATORS_CONTROLOUTPUTS_H
#define ARDUFLITE_ACTUATORS_CONTROLOUTPUTS_H

#include "src/actuators/AirframeMixer.h"
#include "src/hal/device/Actuator.h"

namespace arduflite::actuators {

struct ControlOutputs
{
    device::Actuator* aileronLeft  = nullptr;
    device::Actuator* aileronRight = nullptr;   ///< null when single-aileron
    device::Actuator* elevator     = nullptr;
    device::Actuator* rudder       = nullptr;
    device::Actuator* throttle     = nullptr;   ///< null on a glider

    /// Resolve every role from a bank. Missing optional roles stay null.
    static ControlOutputs resolve(device::ActuatorBank& bank);

    /// Surfaces the controller cannot fly without.
    [[nodiscard]] bool hasRequiredSurfaces() const
    {
        return aileronLeft != nullptr && elevator != nullptr;
    }

    /// Stage mixed commands. Does NOT commit — the caller owns that, because
    /// commit is a batched bus operation.
    void stageSurfaces(const SurfaceCommands& c) const;
    void stageThrottle(float normalised) const;
};

} // namespace arduflite::actuators

#endif // ARDUFLITE_ACTUATORS_CONTROLOUTPUTS_H
