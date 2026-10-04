/**
 * CLICommandContext.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 14 June 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/cli/CLICommandContext.h"
#include "src/state/GroundSafety.h"
#include "src/utils/Logging.h"

namespace
{
ArduFliteController* cliController = nullptr;
arduflite::estimation::InertialSubsystem* cliIMU = nullptr;
ArduFliteFlashTelemetry* cliFlashTelemetry = nullptr;
ConsoleHandover consoleHandover = nullptr;
ConsoleMavlinkDetector mavlinkDetector = nullptr;
bool handoverRequested = false;   // set and read only by the CLI task
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
    const bool armed = (cliController != nullptr) && cliController->isArmed();

    switch (groundCommandBlock(armed, getFlightState()))
    {
        case GroundBlock::Armed:
            LOG_ERR("Cannot %s while armed — disarm first!", action);
            return true;
        case GroundBlock::InFlight:
            LOG_ERR("Cannot %s while INFLIGHT.", action);
            return true;
        case GroundBlock::None:
            break;
    }
    return false;
}

void setConsoleHandover(ConsoleHandover handover, ConsoleMavlinkDetector detector)
{
    consoleHandover = handover;
    mavlinkDetector = detector;
}

bool consoleCarriesMavlink(std::uint8_t byte)
{
    return mavlinkDetector != nullptr && mavlinkDetector(byte);
}

bool requestConsoleHandover()
{
    handoverRequested = (consoleHandover != nullptr);
    return handoverRequested;
}

ConsoleHandover pendingConsoleHandover()
{
    return handoverRequested ? consoleHandover : nullptr;
}
