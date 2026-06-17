/**
 * ButtonCallbacks.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 13 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */

#include "src/utils/ButtonCallbacks.h"
#include "src/utils/CommandSystem.h"
#include "src/controller/ArduFliteController.h"
#include "src/utils/StatusLED.h"
#include "src/utils/Logging.h"

extern StatusLED                statusLED;
extern ArduFliteController      controller;
extern ArduFliteIMU             myIMU;

 // Callback for calibrate button.
void onCalibrateHold(void)
{
#if BOARD_TYPE == BOARD_TYPE_WEMOS
    statusLED.setPattern({0,0,255, 100,100});// fast blue blink
#endif
    LOG_INF("Calibrating IMU (via CommandSystem)...");
    // Route through CommandSystem so the handler also pauses CRSF telemetry,
    // matching the full pause/resume sequence used by the CLI calibrate command.
    // Direct calls to controller.pauseTasks() / myIMU.selfCalibrate() here would
    // bypass that step and violate the CommandSystem contract (AGENTS.md §1).
    SystemCommand cmd;
    cmd.type = CMD_CALIBRATE;
    CommandSystem::instance().pushCommand(cmd);
}

// Callback for telemetry reset button.
void onModeDoubleTap(void)
{
    LOG_INF("Toggling Controller Mode...");

    SystemCommand cmd;
    cmd.type = CMD_SET_MODE;

    if (controller.getMode() == ATTITUDE_MODE)
    {
        LOG_INF("Changing Flight Control mode to: RATE_MODE.");
        cmd.mode = RATE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    }
    else
    {
        LOG_INF("Changing Flight Control mode to: ATTITUDE_MODE.");
        cmd.mode = ATTITUDE_MODE;
        CommandSystem::instance().pushCommand(cmd);
    }
}

// Callback for triple-tap action.
void onResetTripleTap(void)
{
    // TODO: Implement triple-tap action (e.g., toggle debug mode, reset flash telemetry)
    LOG_INF("Triple-tap detected - no action configured");
}