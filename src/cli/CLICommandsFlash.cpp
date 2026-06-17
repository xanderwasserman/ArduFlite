/**
 * CLICommandsFlash.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/utils/Logging.h"

static bool parseFlashIndex(const String& param, const char* commandName, int32_t& index)
{
    if (param.length() == 0)
    {
        LOG("Usage: flash %s <index>", commandName);
        return false;
    }

    if (!parseIntStrict(param, index) || index < 0 || index > 999)
    {
        LOG_ERR("flash %s: index must be an integer from 0 to 999", commandName);
        return false;
    }

    return true;
}

static void printFlashHelp()
{
    LOG("Unknown flash command. Available:");
    LOG("  flash start       → begin a new flight log");
    LOG("  flash stop        → end current flight log");
    LOG("  flash list        → list existing logs");
    LOG("  flash dump <idx>  → stream log #<idx> over serial");
    LOG("  flash delete <idx>→ remove log #<idx>");
    LOG("  flash reset       → erase entire LittleFS");
}

void cmdFlash(const String &args)
{
    ArduFliteFlashTelemetry* flashTelemetry = getCliFlashTelemetry();
    if (!flashTelemetry)
    {
        LOG("Flash telemetry not initialized!");
        return;
    }

    ParsedCommand parsed = parseCommandArgs(args);
    const String& cmd = parsed.command;
    const String& param = parsed.remainder;

    if (cmd == "start")
    {
        if (flashTelemetry->startLogging())
            LOG("Flash logging STARTED");
        else
            LOG("Flash logging FAILED to start — see log for details");
    }
    else if (cmd == "stop")
    {
        flashTelemetry->stopLogging();
        LOG("Flash logging STOPPED");
    }
    else if (cmd == "list")
    {
        LOG("Listing flash logs:");
        flashTelemetry->listLogs();
    }
    else if (cmd == "dump")
    {
        int32_t idx = 0;
        if (!parseFlashIndex(param, "dump", idx)) return;

        LOG("Dumping log %d:\n", idx);
        flashTelemetry->dumpLog(idx);
    }
    else if (cmd == "delete" || cmd == "del" || cmd == "rm")
    {
        int32_t idx = 0;
        if (!parseFlashIndex(param, "delete", idx)) return;
        if (rejectUnsafeGroundCommand("delete flash logs")) return;

        LOG("Deleting log %d: ", idx);
        flashTelemetry->deleteLog(idx);
    }
    else if (cmd == "reset")
    {
        if (rejectUnsafeGroundCommand("format flash logs")) return;

        LOG("Formatting LittleFS (erasing all logs)...");
        flashTelemetry->reset();
        LOG("Done.");
    }
    else
    {
        printFlashHelp();
    }
}
