/**
 * BoardSelect.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief THE ONE #if IN THE CODEBASE.
 *
 * Adding a board is:
 *   1. create boards/<name>.h with an McuProfile and a BoardDescriptor
 *   2. add one #elif here
 *   3. add the board to build.sh and the CI matrix
 * Then build, and fix whatever the static_asserts say.
 */
#ifndef ARDUFLITE_HAL_BOARD_BOARDSELECT_H
#define ARDUFLITE_HAL_BOARD_BOARDSELECT_H

#if   defined(ARDUFLITE_BOARD_LOLIN_C3_MINI)
#  include "src/hal/board/boards/lolin_c3_mini.h"
#elif defined(ARDUFLITE_BOARD_FIREBEETLE_ESP32E)
#  include "src/hal/board/boards/firebeetle_esp32e.h"
#else
#  error "No board selected. Define ARDUFLITE_BOARD_<NAME> — see specs/hal/04-board-descriptors.md"
#endif

#include "src/hal/board/BoardValidate.h"

// consteval, so these cannot silently degrade into runtime checks.
static_assert(arduflite::board::validate::allPinsValid(arduflite::board::kBoard),
              "Board descriptor: a pin is outside the MCU's GPIO range, or is reserved");
static_assert(arduflite::board::validate::allPinsUnique(arduflite::board::kBoard),
              "Board descriptor: the same GPIO is assigned to two functions");
static_assert(arduflite::board::validate::allOutputPinsCanOutput(arduflite::board::kBoard),
              "Board descriptor: an input-only GPIO is assigned to an output");
static_assert(arduflite::board::validate::outputCountWithinMcu(arduflite::board::kBoard),
              "Board descriptor: more PWM outputs than the MCU has channels");
static_assert(arduflite::board::validate::allRolesUnique(arduflite::board::kBoard),
              "Board descriptor: two outputs claim the same role - byRole() would be ambiguous");
static_assert(arduflite::board::validate::allSensorAxesValid(arduflite::board::kBoard),
              "Board descriptor: a sensor axis map is not a valid permutation");
static_assert(arduflite::board::validate::requiredPeripheralsPresent(arduflite::board::kBoard),
              "Board descriptor: a fitted part has no bus or pins assigned");
static_assert(arduflite::board::validate::isValid(arduflite::board::kBoard),
              "Board descriptor is invalid - see specs/hal/04-board-descriptors.md");

#endif // ARDUFLITE_HAL_BOARD_BOARDSELECT_H
