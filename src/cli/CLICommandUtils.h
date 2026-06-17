/**
 * CLICommandUtils.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CLI_COMMAND_UTILS_H
#define CLI_COMMAND_UTILS_H

#include <Arduino.h>
#include <stdint.h>

struct ParsedCommand
{
    String command;
    String remainder;
};

ParsedCommand parseCommandArgs(const String& args);
bool parseIntStrict(const String& input, int32_t& out);
bool parseUInt8Strict(const String& input, uint8_t& out);
bool parseFloatStrict(const String& input, float& out);
bool parseBoolStrict(String input, bool& out);

#endif // CLI_COMMAND_UTILS_H
