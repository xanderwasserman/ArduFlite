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

/**
 * @brief Owns the FlightState machine, applying an arm guard on INFLIGHT transitions.
 *
 * Reads debounced motion signals from the IMU each loop tick and applies state
 * transitions. The IMU no longer owns FlightState; it only produces signals.
 *
 * Transitions:
 *   PREFLIGHT / LANDED → INFLIGHT  : launchDetected AND controller.isArmed()
 *   INFLIGHT           → LANDED    : stableDetected (stopLogging called here)
 *
 * startLogging() is called by CommandSystem on ARM so telemetry captures
 * the entire flight including any ground taxi / hand-launch wind-up.
 */
void handleFlightState()
{
    static FlightState currentState = PREFLIGHT;

    MotionSignals motion = myIMU.getMotionSignals();
    FlightState   newState = currentState;

    switch (currentState)
    {
        case PREFLIGHT:
        case LANDED:
            if (motion.launchDetected)
            {
                if (controller.isArmed())
                {
                    newState = INFLIGHT;
                }
                else
                {
                    LOG_WARN("Throw detected but aircraft is NOT armed — ignoring.");
                }
            }
            break;

        case INFLIGHT:
            if (motion.stableDetected)
            {
                newState = LANDED;
            }
            break;

        default:
            newState = PREFLIGHT;
            break;
    }

    if (newState != currentState)
    {
        currentState = newState;
        myIMU.setFlightState(currentState);

        switch (currentState)
        {
            case PREFLIGHT:
                LOG_INF("Aircraft is in PREFLIGHT state.");
                break;
            case INFLIGHT:
                LOG_INF("Aircraft is in FLIGHT state.");
                break;
            case LANDED:
                LOG_INF("Aircraft has LANDED.");
                flashTelemetry.stopLogging();
                break;
            default:
                break;
        }
    }
}