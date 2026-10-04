/**
 * GroundSafety.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The one rule for commands that are only safe on the ground.
 *
 * Changing configuration, calibrating, rebooting and similar commands are
 * refused while the aircraft is armed or in flight, whichever interface they
 * arrive from: the CLI and MAVLink both ask here (ADR-068).
 */
#ifndef ARDUFLITE_STATE_GROUND_SAFETY_H
#define ARDUFLITE_STATE_GROUND_SAFETY_H

#include <cstdint>

#include "src/core/FlightTypes.h"

enum class GroundBlock : std::uint8_t
{
    None,       ///< safe: the command may run
    Armed,
    InFlight,
};

[[nodiscard]] constexpr GroundBlock groundCommandBlock(bool armed, FlightState state) noexcept
{
    if (armed)              { return GroundBlock::Armed; }
    if (state == INFLIGHT)  { return GroundBlock::InFlight; }
    return GroundBlock::None;
}

#endif // ARDUFLITE_STATE_GROUND_SAFETY_H
