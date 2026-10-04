/**
 * CLICommandsSystem.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/board/Board.h"
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

static unsigned long overrunPercentage(const LoopStats& stats)
{
    if (stats.sampleCount == 0) return 0UL;
    return static_cast<unsigned long>(
        (static_cast<uint64_t>(stats.overrunCount) * 100ULL) / stats.sampleCount);
}

void cmdReset(const String &args)
{
    if (rejectUnsafeGroundCommand("reset")) return;

    LOG("Resetting system...");
    arduflite::board::Board::instance().system().reboot();
}

void cmdStats(const String &args)
{
    ArduFliteController* controller = getCliController();
    if (!controller)
    {
        LOG_ERR("Controller not set!");
        return;
    }

    LoopStats outerStats = controller->getOuterLoopStats();
    LoopStats innerStats = controller->getInnerLoopStats();

    const unsigned long outerPct = overrunPercentage(outerStats);
    const unsigned long innerPct = overrunPercentage(innerStats);
    LOG("Outer Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu",
        outerStats.avgDt, outerStats.maxDt, outerStats.overrunCount, outerPct);
    LOG("Inner Loop: avg dt: %.2f ms, max dt: %.2f ms, overruns: %lu, percentage: %lu",
        innerStats.avgDt, innerStats.maxDt, innerStats.overrunCount, innerPct);
}

void cmdTasks(const String &args)
{
    static char taskListBuffer[3072];

    const arduflite::Status status =
        arduflite::board::Board::instance().system().taskReport(
            taskListBuffer, sizeof(taskListBuffer));

    if (status != arduflite::Status::Ok)
    {
        LOG_ERR("Task list unavailable (%s)", arduflite::toString(status));
        return;
    }
    LOG("Task List:");
    LOG("%s", taskListBuffer);
}

void cmdSetMode(const String &args)
{
    if (!getCliController())
    {
        LOG_ERR("Controller not set!");
        return;
    }

    ParsedCommand parsed = parseCommandArgs(args);
    if (!parsed.remainder.isEmpty())
    {
        LOG("Unknown mode. Use 'assist' or 'stabilized'.");
        return;
    }

    SystemCommand cmd = {};
    cmd.type = CMD_SET_MODE;

    if (parsed.command == "assist")
    {
        LOG_INF("Changing Flight Control mode to: ATTITUDE_MODE.");
        cmd.mode = ATTITUDE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    }
    else if (parsed.command == "stabilized")
    {
        LOG_INF("Changing Flight Control mode to: RATE_MODE.");
        cmd.mode = RATE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    }
    else
    {
        LOG("Unknown mode. Use 'assist' or 'stabilized'.");
    }
}

void cmdCalibrateIMU(const String &args)
{
    if (rejectUnsafeGroundCommand("calibrate")) return;

    ParsedCommand target = parseCommandArgs(args);
    if ((target.command.isEmpty() || target.command == "imu") && target.remainder.isEmpty())
    {
        // Route through CommandSystem so task pause/resume is centralized there.
        SystemCommand cmd = {};
        cmd.type = CMD_CALIBRATE;
        CommandSystem::instance().pushCommand(cmd);

        LOG("IMU calibration queued. Tasks will pause during calibration.");
    }
    else
    {
        LOG("Unknown calibration target. Use 'calibrate imu'.");
    }
}
