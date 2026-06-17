/**
 * CLICommandUtils.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommandUtils.h"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <math.h>

ParsedCommand parseCommandArgs(const String& args)
{
    String input = args;
    input.trim();

    const int spacePos = input.indexOf(' ');
    ParsedCommand parsed;
    parsed.command = (spacePos < 0) ? input : input.substring(0, spacePos);
    parsed.remainder = (spacePos < 0) ? "" : input.substring(spacePos + 1);
    parsed.command.trim();
    parsed.command.toLowerCase();
    parsed.remainder.trim();
    return parsed;
}

bool parseIntStrict(const String& input, int32_t& out)
{
    String trimmed = input;
    trimmed.trim();
    if (trimmed.length() == 0) return false;

    errno = 0;
    char* end = nullptr;
    long value = strtol(trimmed.c_str(), &end, 10);
    if (errno != 0 || end == trimmed.c_str() || *end != '\0') return false;
    if (value < INT32_MIN || value > INT32_MAX) return false;

    out = static_cast<int32_t>(value);
    return true;
}

bool parseUInt8Strict(const String& input, uint8_t& out)
{
    int32_t value = 0;
    if (!parseIntStrict(input, value) || value < 0 || value > UINT8_MAX) return false;

    out = static_cast<uint8_t>(value);
    return true;
}

bool parseFloatStrict(const String& input, float& out)
{
    String trimmed = input;
    trimmed.trim();
    if (trimmed.length() == 0) return false;

    errno = 0;
    char* end = nullptr;
    float value = strtof(trimmed.c_str(), &end);
    if (errno != 0 || end == trimmed.c_str() || *end != '\0' || !isfinite(value)) return false;

    out = value;
    return true;
}

bool parseBoolStrict(String input, bool& out)
{
    input.trim();
    input.toLowerCase();

    if (input == "true" || input == "1" || input == "yes" || input == "on")
    {
        out = true;
        return true;
    }
    if (input == "false" || input == "0" || input == "no" || input == "off")
    {
        out = false;
        return true;
    }

    return false;
}
