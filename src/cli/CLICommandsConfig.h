#ifndef CLI_COMMANDS_CONFIG_H
#define CLI_COMMANDS_CONFIG_H

#include "src/controller/CLICommands.h"  // Your CLICommands.h that defines CLICommand, etc.
#include "src/controller/ArduFliteController.h"  // If needed for the command functions

// Declare the command table and its size as extern.
extern CLICommand cliCommands[];
extern const size_t numCLICommands;

// Declare the command functions.
void cmdHelp(const String &args);
void cmdStats(const String &args);
void cmdTasks(const String &args);
void cmdSetMode(const String &args);

// Function to set the controller pointer (to be used by the CLI task)
void setCLIController(ArduFliteController* controller);

#endif // CLI_COMMANDS_CONFIG_H
