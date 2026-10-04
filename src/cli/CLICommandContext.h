/**
 * CLICommandContext.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CLI_COMMAND_CONTEXT_H
#define CLI_COMMAND_CONTEXT_H

#include <cstdint>

#include "src/controller/ArduFliteController.h"
#include "src/core/FlightTypes.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/state/StateManagement.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

void setCliController(ArduFliteController* controller);
void setCliIMU(arduflite::estimation::InertialSubsystem* imu);
void setFlashTelemetry(ArduFliteFlashTelemetry* telem);

ArduFliteController* getCliController();
arduflite::estimation::InertialSubsystem* getCliIMU();
ArduFliteFlashTelemetry* getCliFlashTelemetry();

bool rejectUnsafeGroundCommand(const char* action);

/// Hands the console to MAVLink. Run by the CLI task after it stops reading.
using ConsoleHandover = void (*)();

/// Fed each console byte; true when it completes a valid MAVLink frame, i.e. a
/// ground station is talking on this port.
using ConsoleMavlinkDetector = bool (*)(std::uint8_t byte);

void setConsoleHandover(ConsoleHandover handover, ConsoleMavlinkDetector detector);

bool consoleCarriesMavlink(std::uint8_t byte);

/// Ask the CLI to give up the console after the current command. False when
/// this build has no handover.
bool requestConsoleHandover();

/// The handover the CLI task must run before it exits, or nullptr.
ConsoleHandover pendingConsoleHandover();

#endif // CLI_COMMAND_CONTEXT_H
