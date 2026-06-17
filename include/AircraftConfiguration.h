/**
 * AircraftConfiguration.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 24 May 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef AIRCRAFT_CONFIGURATION_H
#define AIRCRAFT_CONFIGURATION_H

// ─────────────────────────────────────────────────────────────────────────────
// Aircraft Propulsion Type
//
// Select the propulsion type that matches the airframe being flown.
// This controls whether throttle-cut state is used as a gate for flight state
// transitions and flash logging triggers.
//
//   AIRCRAFT_TYPE_POWERED  — Motor/ESC equipped. Throttle cut must be released
//                            before INFLIGHT transition and logging can start.
//                            Prevents false launches and wastes no flash on
//                            ground-check sessions.
//
//   AIRCRAFT_TYPE_GLIDER   — No motor/ESC. Throttle-cut gates are compiled out.
//                            Flash logging starts on throw detection (INFLIGHT)
//                            and stops on landing (LANDED).
// ─────────────────────────────────────────────────────────────────────────────
#define AIRCRAFT_TYPE_POWERED   0
#define AIRCRAFT_TYPE_GLIDER    1

#ifndef AIRCRAFT_TYPE
#define AIRCRAFT_TYPE           AIRCRAFT_TYPE_POWERED
#endif

#if AIRCRAFT_TYPE != AIRCRAFT_TYPE_POWERED && AIRCRAFT_TYPE != AIRCRAFT_TYPE_GLIDER
#  error "AircraftConfiguration.h: unknown AIRCRAFT_TYPE value — must be AIRCRAFT_TYPE_POWERED or AIRCRAFT_TYPE_GLIDER"
#endif

#endif // AIRCRAFT_CONFIGURATION_H
