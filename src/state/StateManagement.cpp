/**
 * StateManagement.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 13 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#include "src/state/StateManagement.h"

#include <atomic>
#include "src/controller/ArduFliteController.h"
#include "src/core/FlightTypes.h"
#include "src/hal/board/Board.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "include/AircraftConfiguration.h"
#include "src/hal/device/Peripherals.h"
#include "src/utils/Colors.h"
#include "src/utils/Logging.h"

namespace {

/// The aircraft's belief about what it is doing. Written only by
/// updateFlightState() below; read from the controller, CLI and telemetry tasks.
std::atomic<int> g_flightState{ PREFLIGHT };

} // namespace

FlightState getFlightState()
{
    return static_cast<FlightState>(g_flightState.load(std::memory_order_acquire));
}

void setFlightState(FlightState state)
{
    g_flightState.store(static_cast<int>(state), std::memory_order_release);
}

extern ArduFliteController      controller;
extern arduflite::estimation::InertialSubsystem myIMU;
extern ArduFliteFlashTelemetry  flashTelemetry;



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
                if (auto* led = arduflite::board::Board::instance().indicator())
                {
                    led->setPattern(Patterns::Assist);
                }
                break;
            case RATE_MODE:
                if (auto* led = arduflite::board::Board::instance().indicator())
                {
                    led->setPattern(Patterns::Stabilized);
                }
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
 * transitions. The estimation layer only produces signals; it does not own
 * FlightState.
 *
 * Transitions:
 *   PREFLIGHT / LANDED → INFLIGHT  : launchDetected AND armed
 *                                     (powered only: throttle must NOT be cut)
 *   INFLIGHT           → LANDED    : stableDetected (stopLogging called here)
 *
 * Logging triggers (compile-time selected via AIRCRAFT_TYPE):
 *   AIRCRAFT_TYPE_POWERED — startLogging() called by CommandSystem on arm + throttle-cut release.
 *   AIRCRAFT_TYPE_GLIDER  — startLogging() called here on INFLIGHT transition;
 *                           stopLogging() called here on LANDED (both aircraft types).
 */
void handleFlightState()
{
    // SINGLE-TASK ONLY: static locals below are not thread-safe.
    // handleFlightState() must be called exclusively from arduflite_loop().
    static FlightState currentState    = PREFLIGHT;
    static bool        lastLaunchSeen  = false;   // tracks rising edge for one-shot warnings

    MotionSignals motion = myIMU.state().motion;
    FlightState   newState = currentState;

    switch (currentState)
    {
        case PREFLIGHT:
        case LANDED:
            if (motion.launchDetected)
            {
#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_POWERED
                if (controller.isArmed() && !controller.isThrottleCut())
                {
                    newState = INFLIGHT;
                }
                else if (!lastLaunchSeen)
                {
                    // Log only on the rising edge to avoid spamming at loop rate.
                    if (!controller.isArmed())
                    {
                        LOG_WARN("Throw detected but aircraft is NOT armed — ignoring.");
                    }
                    else
                    {
                        LOG_WARN("Throw detected but throttle cut is active — ignoring.");
                    }
                }
#else
                // Glider: no throttle-cut gate — arm is the only prerequisite.
                if (controller.isArmed())
                {
                    newState = INFLIGHT;
                }
                else if (!lastLaunchSeen)
                {
                    LOG_WARN("Throw detected but aircraft is NOT armed — ignoring.");
                }
#endif
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

    lastLaunchSeen = motion.launchDetected;

    if (newState != currentState)
    {
        currentState = newState;
        setFlightState(currentState);

        switch (currentState)
        {
            case PREFLIGHT:
                LOG_INF("Aircraft is in PREFLIGHT state.");
                break;
            case INFLIGHT:
                LOG_INF("Aircraft is in FLIGHT state.");
#if AIRCRAFT_TYPE == AIRCRAFT_TYPE_GLIDER
                // Glider: logging is gated on flight, not on arm+throttle-cut.
                // startLogging() here is symmetric with stopLogging() on LANDED below.
                if (!flashTelemetry.startLogging())
                {
                    LOG_ERR("Flight log FAILED to start on INFLIGHT — flight will NOT be recorded!");
                }
#endif
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