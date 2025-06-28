/**
 * ArdufliteCRSFCallbacks.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 07 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/receiver/crsf/ArdufliteCRSFCallbacks.h"
#include "src/utils/Logging.h"

namespace CRSFCallbacks
{

    // Called by the receiver when failsafe timeout fires
    void onFailsafe() 
    {
        // 1) Enable Throttle-Cut
        SystemCommand throttleCmd{};
        throttleCmd.type = CMD_SET_THROTTLE_CUT;
        throttleCmd.x_value = true;
        CommandSystem::instance().pushCommand(throttleCmd);

        // 2) Force ATTITUDE mode so our setpoints are interpreted as angles
        SystemCommand modeCmd{};
        modeCmd.type = CMD_SET_MODE;
        modeCmd.mode = ATTITUDE_MODE;
        CommandSystem::instance().pushCommand(modeCmd);

        // 3) Push a nominal attitude setpoint:  10° bank +  -5° pitch
        EulerAngles fsAttitude;
        fsAttitude.roll  = 10.0f;   // tilt right 10°
        fsAttitude.pitch = -5.0f;   // nose down 5°
        fsAttitude.yaw   = 0.0f;    // hold current heading
        SystemCommand attCmd{};
        attCmd.type           = CMD_SET_CONFIG_ATTITUDE;
        attCmd.attitudeConfig = fsAttitude;
        CommandSystem::instance().pushCommand(attCmd);
    }

    void onRoll(uint8_t ch, float v) 
    {
        LOG_DBG("onRoll: %f", v);
        ControlMixer::handleChannelInput(ch, v);
    }

    void onPitch(uint8_t ch, float v) 
    {
        LOG_DBG("onPitch: %f", v);
        ControlMixer::handleChannelInput(ch, v);
    }

    void onThrottle(uint8_t ch, float v) 
    {
        LOG_DBG("onThrottle: %f", v);

        SystemCommand cmd;
        cmd.type = CMD_RECEIVER_SETPOINT_THROTTLE;
        
        cmd.value = v;
        CommandSystem::instance().pushCommand(cmd);
    }

    void onYaw(uint8_t ch, float v) 
    {
        LOG_DBG("onYaw: %f", v);
        ControlMixer::handleChannelInput(ch, v);
    }

    void onModeSwitch(uint8_t ch, float v) 
    {
        LOG_DBG("onModeSwitch: %f", v);
        SystemCommand cmd{};
        cmd.type = CMD_SET_MODE;
        bool newState  = (v > 0.5f);  // v is 0.0 or 1.0, but guard anyway

        if (v == -1) // ATTITUDE_MODE
        { 
            LOG_INF("Changing Flight Control mode to: ATTITUDE_MODE.");
            cmd.mode = ATTITUDE_MODE;
            CommandSystem::instance().pushCommand(cmd);
        } 
        else if (v == 0) // RATE_MODE
        {
            LOG_INF("Changing Flight Control mode to: RATE_MODE.");
            cmd.mode = RATE_MODE;
            CommandSystem::instance().pushCommand(cmd);
        }
        else // MANUAL_MODE (v == 1)
        {
            LOG_INF("Changing Flight Control mode to: MANUAL_MODE.");
            cmd.mode = MANUAL_MODE;
            CommandSystem::instance().pushCommand(cmd);
        }
    }

    void onActivateMission(uint8_t ch, float v) 
    {
        SystemCommand cmd;
        cmd.type = CMD_SET_MISSION;
        bool newState  = (v > 0.5f);  // v is 0.0 or 1.0, but guard anyway

        LOG_INF("Changing Mission state to: %s.", newState?"START":"STOP");
        cmd.x_value = newState;
        CommandSystem::instance().pushCommand(cmd);
    }

    void onArm(uint8_t ch, float v) 
    {
        SystemCommand cmd;
        cmd.type = CMD_SET_ARM;
        bool newState  = (v > 0.5f);  // v is 0.0 or 1.0, but guard anyway

        LOG_INF("Controller ARMED: %s.", newState?"YES":"NO");
        cmd.x_value = newState;
        CommandSystem::instance().pushCommand(cmd);
    }

    void onThrottleCut(uint8_t ch, float v) 
    {
        SystemCommand cmd;
        cmd.type = CMD_SET_THROTTLE_CUT;
        bool newState  = (v > 0.5f);  // v is 0.0 or 1.0, but guard anyway

        LOG_INF("Controller THROTTLE CUT: %s.", newState?"YES":"NO");
        cmd.x_value = newState;
        CommandSystem::instance().pushCommand(cmd);
    }



} // namespace CRSFCallbacks