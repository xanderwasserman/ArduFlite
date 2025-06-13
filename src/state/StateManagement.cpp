/**
 * StateManagement.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 13 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#include "src/state/StateManagement.h"
#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "include/PinConfiguration.h"
#include "src/utils/StatusLED.h"
#include "src/utils/Logging.h"

extern ArduFliteController      controller;
extern ArduFliteIMU             myIMU;
extern ArduFliteFlashTelemetry  flashTelemetry;

#if BOARD_TYPE == BOARD_TYPE_WEMOS
extern StatusLED            statusLED;
#endif

void handleModeState()
{
    // Retrieve the current flight state from the IMU.
    static ArduFliteMode lastMode = UNKNOWN_MODE;
    ArduFliteMode currentMode = controller.getMode();

    if (currentMode != lastMode) 
    {
        switch (currentMode) 
        {
            case ATTITUDE_MODE:
                #if BOARD_TYPE == BOARD_TYPE_WEMOS
                statusLED.setPattern(Patterns::Assist);
                #endif
                break;
            case RATE_MODE:
                #if BOARD_TYPE == BOARD_TYPE_WEMOS
                statusLED.setPattern(Patterns::Stabilized);
                #endif
                break;
            default:
                break;
        }
        lastMode = currentMode;
    }
}

void handleFlightState()
{
    // Retrieve the current flight state from the IMU.
    static FlightState lastState = UNKNOWN_STATE;
    FlightState currentState = myIMU.getFlightState();

    // Print transitions when they occur.
    if (currentState != lastState) 
    {
        switch (currentState) 
        {
            case PREFLIGHT:
                LOG_INF("Aircraft is in PREFLIGHT state.");;
                break;
            case INFLIGHT:
                LOG_INF("Aircraft is in FLIGHT state.");
                flashTelemetry.startLogging();
                
                break;
            case LANDED:
                LOG_INF("Aircraft has LANDED.");

                flashTelemetry.stopLogging();
                break;
            default:
                break;
        }
        lastState = currentState;
    }
}