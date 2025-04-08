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

#endif // CLI_COMMANDS_H
