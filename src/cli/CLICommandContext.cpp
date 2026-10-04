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
arduflite::estimation::InertialSubsystem* cliIMU = nullptr;
ArduFliteFlashTelemetry* cliFlashTelemetry = nullptr;
}

void setCliController(ArduFliteController* controller)
{
    cliController = controller;
}

void setCliIMU(arduflite::estimation::InertialSubsystem* imu)
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

arduflite::estimation::InertialSubsystem* getCliIMU()
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

    // Deliberately NOT guarded on cliIMU being non-null. The flight state is
    // owned by StateManagement and is valid regardless of whether the CLI holds
    // an IMU handle; keeping the old `cliIMU &&` would let a dangerous command
    // through in flight on any path where that pointer was never set.
    if (getFlightState() == INFLIGHT)
    {
        LOG_ERR("Cannot %s while INFLIGHT.", action);
        return true;
    }

    return false;
}
