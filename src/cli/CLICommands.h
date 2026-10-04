/**
 * CLICommands.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CLI_COMMANDS_H
#define CLI_COMMANDS_H

#include <Arduino.h>

// Definition of the function signature for CLI command functions.
// The argument is the remainder of the command line (i.e. all text after the command name).
typedef void (*CLICommandFunction)(const String &args);

/**
 * @brief Structure representing a CLI command.
 */
struct CLICommand {
    const char* command;         // The command keyword (e.g., "help")
    const char* description;     // A short description of the command.
    CLICommandFunction execute;  // The function to execute when this command is invoked.
};

extern const CLICommand cliCommands[];
extern const size_t numCLICommands;

void cmdHelp(const String &args);
void cmdReset(const String &args);
void cmdStats(const String &args);
void cmdTasks(const String &args);
void cmdSetMode(const String &args);
void cmdCalibrateIMU(const String &args);
void cmdFlash(const String &args);
void cmdConfig(const String &args);
void cmdStream(const String &args);
void cmdMavlink(const String &args);
void cmdTest(const String &args);

#endif // CLI_COMMANDS_H
