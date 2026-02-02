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
#include "include/ReceiverConfiguration.h"

namespace CRSFCallbacks
{
    // Store the mode before failsafe so we can restore it when link recovers
    static ArduFliteMode savedModeBeforeFailsafe = ATTITUDE_MODE;

    /**
     * @brief Called by the receiver when failsafe timeout fires.
     * 
     * Implements a throttle-cut spiral descent failsafe:
     * 1. Cuts throttle immediately (fire/injury prevention)
     * 2. Switches to ATTITUDE mode for predictable behavior
     * 3. Commands a gentle bank angle for controlled spiral descent
     * 
     * The spiral keeps the aircraft in a contained area while descending
     * at a controlled rate. This is safer than level flight which could
     * result in the aircraft flying away.
     */
    void onFailsafe() 
    {
        LOG_WARN("=== FAILSAFE ACTIVATED ===");
        LOG_WARN("Cutting throttle and entering controlled spiral descent.");
        LOG_WARN("Mode before failsafe: %d (will restore on link recovery)", savedModeBeforeFailsafe);

        // savedModeBeforeFailsafe is tracked by onModeSwitch() whenever the pilot
        // changes modes, so it already contains the correct pre-failsafe mode.

        // 1) Cut throttle immediately - this is the most critical safety action
        SystemCommand throttleCmd{};
        throttleCmd.type = CMD_SET_THROTTLE_CUT;
        throttleCmd.x_value = true;
        CommandSystem::instance().pushCommand(throttleCmd);

        // 2) Force ATTITUDE mode so our setpoints are interpreted as angles
        SystemCommand modeCmd{};
        modeCmd.type = CMD_SET_MODE;
        modeCmd.mode = ATTITUDE_MODE;
        CommandSystem::instance().pushCommand(modeCmd);

        // 3) Command a gentle spiral descent using configured failsafe angles
        // NOTE: yaw=0 holds heading in ATTITUDE_MODE. Combined with constant bank,
        // this creates a tightening spiral rather than coordinated turn. This is
        // intentional: the spiral descends quickly into a small area, making the
        // aircraft easier to locate after link loss.
        EulerAngles fsAttitude;
        fsAttitude.roll  = FailsafeConfig::FAILSAFE_BANK_DEG;   // Gentle bank for contained spiral
        fsAttitude.pitch = FailsafeConfig::FAILSAFE_PITCH_DEG;  // Slight nose-down for glide
        fsAttitude.yaw   = 0.0f;                                 // Hold heading (intentional spiral)
        
        SystemCommand attCmd{};
        attCmd.type           = CMD_SET_CONFIG_ATTITUDE;
        attCmd.attitudeConfig = fsAttitude;
        CommandSystem::instance().pushCommand(attCmd);

        LOG_WARN("Failsafe: Bank=%.1f° Pitch=%.1f° Throttle=CUT",
                 FailsafeConfig::FAILSAFE_BANK_DEG,
                 FailsafeConfig::FAILSAFE_PITCH_DEG);
    }

    /**
     * @brief Called by the receiver when RC link is restored after failsafe.
     * 
     * Restores the flight mode to the state before failsafe was triggered.
     * NOTE: Throttle cut remains ENABLED for safety - pilot must manually
     * re-enable throttle to confirm they are ready to resume control.
     */
    void onFailsafeExit() 
    {
        LOG_INF("=== FAILSAFE EXITED - RC LINK RESTORED ===");
        LOG_INF("Restoring flight mode. Throttle cut remains ON for safety.");

        // Restore the mode that was active before failsafe
        SystemCommand modeCmd{};
        modeCmd.type = CMD_SET_MODE;
        modeCmd.mode = savedModeBeforeFailsafe;
        CommandSystem::instance().pushCommand(modeCmd);

        // Zero out the failsafe attitude setpoints so pilot has clean control
        EulerAngles zeroAttitude;
        zeroAttitude.roll  = 0.0f;
        zeroAttitude.pitch = 0.0f;
        zeroAttitude.yaw   = 0.0f;
        
        SystemCommand attCmd{};
        attCmd.type           = CMD_SET_CONFIG_ATTITUDE;
        attCmd.attitudeConfig = zeroAttitude;
        CommandSystem::instance().pushCommand(attCmd);

        LOG_INF("Mode restored to %d. Pilot must disengage throttle cut to resume.",
                savedModeBeforeFailsafe);
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

        if (v == -1) // ATTITUDE_MODE
        { 
            LOG_INF("Changing Flight Control mode to: ATTITUDE_MODE.");
            cmd.mode = ATTITUDE_MODE;
            savedModeBeforeFailsafe = ATTITUDE_MODE;  // Track current mode for failsafe recovery
            CommandSystem::instance().pushCommand(cmd);
        } 
        else if (v == 0) // RATE_MODE
        {
            LOG_INF("Changing Flight Control mode to: RATE_MODE.");
            cmd.mode = RATE_MODE;
            savedModeBeforeFailsafe = RATE_MODE;  // Track current mode for failsafe recovery
            CommandSystem::instance().pushCommand(cmd);
        }
        else // MANUAL_MODE (v == 1)
        {
            LOG_INF("Changing Flight Control mode to: MANUAL_MODE.");
            cmd.mode = MANUAL_MODE;
            savedModeBeforeFailsafe = MANUAL_MODE;  // Track current mode for failsafe recovery
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