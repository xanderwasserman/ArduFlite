/**
 * CLICommandContext.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommandContext.h"
#include "src/utils/Logging.h"

namespace
{
ArduFliteController* cliController = nullptr;
ArduFliteIMU* cliIMU = nullptr;
ArduFliteFlashTelemetry* cliFlashTelemetry = nullptr;
}

void setCliController(ArduFliteController* controller)
{
    cliController = controller;
}

void setCliIMU(ArduFliteIMU* imu)
{
    cliIMU = imu;
}

void setFlashTelemetry(ArduFliteFlashTelemetry* telem)
{
    cliFlashTelemetry = telem;
}

ArduFliteController* getCliController()
{
    return cliController;
}

ArduFliteIMU* getCliIMU()
{
    return cliIMU;
}

ArduFliteFlashTelemetry* getCliFlashTelemetry()
{
    return cliFlashTelemetry;
}

bool rejectUnsafeGroundCommand(const char* action)
{
    if (cliController && cliController->isArmed())
    {
        LOG_ERR("Cannot %s while armed — disarm first!", action);
        return true;
    }

    if (cliIMU && cliIMU->getFlightState() == INFLIGHT)
    {
        LOG_ERR("Cannot %s while INFLIGHT.", action);
        return true;
    }

    return false;
}
