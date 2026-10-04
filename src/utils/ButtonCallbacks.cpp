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
#include "src/hal/board/Board.h"
#include "src/hal/device/Peripherals.h"
#include "src/utils/Colors.h"
#include "src/utils/Logging.h"

extern ArduFliteController      controller;
extern arduflite::estimation::InertialSubsystem myIMU;

 // Callback for calibrate button.
void onCalibrateHold(void)
{
    if (auto* led = arduflite::board::Board::instance().indicator())
    {
        led->setPattern(Pattern{ Colors::Blue, 100, 100 });   // fast blue blink
    }
    LOG_INF("Calibrating IMU (via CommandSystem)...");
    // Queue it rather than calibrating here. This runs in a button callback;
    // calibration must happen on the command task, and going straight to the
    // controller or the estimator from here bypasses that (AGENTS.md §1).
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