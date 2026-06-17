/**
 * CLICommandsTests.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommands.h"
#include "src/cli/CLICommandContext.h"
#include "src/cli/CLICommandUtils.h"
#include "src/tests/ControlLoopTests.h"
#include "src/tests/BaroTests.h"
#include "src/utils/Logging.h"

void cmdTest(const String &args)
{
    ParsedCommand parsed = parseCommandArgs(args);

    if ((parsed.command.isEmpty() || parsed.command == "help") && parsed.remainder.isEmpty())
    {
        LOG("Available integration tests:");
        LOG("  test loops              — verify outer (10 ms) and inner (2 ms) loop dt");
        LOG("  test baro               — verify boot seed (no climb spike) + snapshot health");
        return;
    }

    if (!parsed.remainder.isEmpty())
    {
        LOG_ERR("test: unexpected argument '%s'. Type 'test help'.", parsed.remainder.c_str());
        return;
    }

    ArduFliteIMU* imu = getCliIMU();
    if (imu && imu->getFlightState() == INFLIGHT)
    {
        LOG_ERR("test: disabled while INFLIGHT.");
        return;
    }

    if (parsed.command == "loops")
    {
        ArduFliteController* controller = getCliController();
        if (!controller)
        {
            LOG_ERR("test: controller not set");
            return;
        }
        runControlLoopTest_dtComputation(*controller);
        return;
    }

    if (parsed.command == "baro")
    {
        if (!imu)
        {
            LOG_ERR("test: IMU not set");
            return;
        }
        runBaroTest_seedAndSnapshotHealth(*imu);
        return;
    }

    LOG_ERR("test: unknown sub-command '%s'. Type 'test help'.", parsed.command.c_str());
}
