/**
 * CLICommandsConfig.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CLI_COMMANDS_CONFIG_H
#define CLI_COMMANDS_CONFIG_H

#include "src/cli/CLICommands.h" 
#include "src/controller/ArduFliteController.h" 
#include "src/orientation/ArduFliteIMU.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"

// Declare the command table and its size as extern.
extern CLICommand cliCommands[];
extern const size_t numCLICommands;

// Declare the command functions.
void cmdHelp(const String &args);
void cmdReset(const String &args);
void cmdStats(const String &args);
void cmdTasks(const String &args);
void cmdSetMode(const String &args);
void cmdCalibrateIMU(const String &args);

// Function to set the controller pointer (to be used by the CLI task)
void setCliController(ArduFliteController* controller);
void setCliIMU(ArduFliteIMU* imu);
void setFlashTelemetry(ArduFliteFlashTelemetry* telem);

#endif // CLI_COMMANDS_CONFIG_H
