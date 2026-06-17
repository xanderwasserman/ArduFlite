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

#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

void setCliController(ArduFliteController* controller);
void setCliIMU(ArduFliteIMU* imu);
void setFlashTelemetry(ArduFliteFlashTelemetry* telem);

ArduFliteController* getCliController();
ArduFliteIMU* getCliIMU();
ArduFliteFlashTelemetry* getCliFlashTelemetry();

bool rejectUnsafeGroundCommand(const char* action);

#endif // CLI_COMMAND_CONTEXT_H
