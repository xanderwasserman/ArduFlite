/**
 * ArdufliteCRSFCallbacks.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/receiver/crsf/ArdufliteCRSFCallbacks.h"
#include "src/controller/ArduFliteController.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"

void onRoll(uint8_t ch, float v) {

}

void onPitch(uint8_t ch, float v) {

}
void onYaw(uint8_t ch, float v) {

}
void onModeSwitch(uint8_t ch, float v) {

    SystemCommand cmd;
    cmd.type = CMD_SET_MODE;
    bool newState  = (v > 0.5f);  // v is 0.0 or 1.0, but guard anyway

    if (!newState) { //Stabilized mode when receiving false
        LOG_INF("Changing Flight Control mode to: STABILIZED_MODE.");
        cmd.mode = STABILIZED_MODE;
        CommandSystem::instance().pushCommand(cmd);
    } else {
        LOG_INF("Changing Flight Control mode to: ASSIST_MODE.");
        cmd.mode = ASSIST_MODE;
        CommandSystem::instance().pushCommand(cmd);
    }
}