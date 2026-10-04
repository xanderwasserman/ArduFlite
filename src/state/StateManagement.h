/**
 * StateManagement.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 13 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef STATE_MANAGEMENT_H
#define STATE_MANAGEMENT_H

#include "src/core/FlightTypes.h"

void handleModeState();
void handleFlightState();


/**
 * @name Flight state ownership
 *
 * StateManagement decides these transitions, so it holds the value. It used to
 * live in ArduFliteIMU purely because telemetry already had an IMU handle and
 * could read it there cheaply — which put the storage in one module and the
 * authority in another.
 *
 * Atomic because the transitions run in the state task while the controller,
 * CLI and telemetry read it from theirs.
 * @{
 */
[[nodiscard]] FlightState getFlightState();
void setFlightState(FlightState state);
/** @} */

#endif // STATE_MANAGEMENT_H