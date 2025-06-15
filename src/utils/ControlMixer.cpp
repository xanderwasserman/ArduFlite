/**
 * ControlMixer.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 11 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/utils/ControlMixer.h"
#include "src/utils/Logging.h"

// statics
EulerAngles          ControlMixer::s_raw{};
ArduFliteController* ControlMixer::s_ctrl = nullptr;

void ControlMixer::init(ArduFliteController& ctrl) 
{
    s_ctrl = &ctrl;
}

void ControlMixer::handleChannelInput(uint8_t ch, float v) 
{
    if (!s_ctrl) 
    {
        LOG_ERR("ControlMixer: not initialized!");
        return;
    }

    // ——— optional single-axis tuning ———
  #ifdef ENABLE_SINGLE_AXIS_TUNING
    if (ch != TUNED_AXIS_CH) return;
  #endif

    // update raw value
    switch (ch) 
    {
      case CH_ROLL:
        s_raw.roll  = v; 
        break;
      case CH_PITCH:
        s_raw.pitch = v; 
        break;
      case CH_YAW:
        s_raw.yaw   = v; 
        break;
      default: 
        return;
    }

    // grab mode
    auto mode = s_ctrl->getMode();
    EulerAngles sp{0.0f, 0.0f, 0.0f};

  #ifdef ENABLE_CONTROL_MAPPING
    if (mode == ATTITUDE_MODE) 
    {
        // — scale into absolute angles —
        sp.roll  = s_raw.roll  * MAX_ATT_ROLL;
        sp.pitch = s_raw.pitch * MAX_ATT_PITCH;
        sp.yaw   = s_raw.yaw   * MAX_ATT_YAW;

        #ifdef ENABLE_MIXING // SAFE-style mixing
          
          // roll ← yaw
          sp.roll  += MIX_ATT_ROLL_FROM_YAW * (s_raw.yaw  * MAX_ATT_ROLL);

          // pitch ← roll
          sp.pitch += MIX_ATT_PITCH_FROM_ROLL * ( fabsf(s_raw.roll) * MAX_ATT_PITCH );

          // yaw ← roll
          sp.yaw   += MIX_ATT_YAW_FROM_ROLL  *  (s_raw.roll   * MAX_ATT_YAW);
        #endif

        sp.roll  = constrain(sp.roll,  -MAX_ATT_ROLL,  MAX_ATT_ROLL);
        sp.pitch = constrain(sp.pitch, -MAX_ATT_PITCH, MAX_ATT_PITCH);
        sp.yaw   = constrain(sp.yaw,   -MAX_ATT_YAW,   MAX_ATT_YAW);

        // send one attitude-setpoint command
        SystemCommand cmd{};
        cmd.type            = CMD_SET_CONFIG_ATTITUDE;
        cmd.attitudeConfig  = sp;
        CommandSystem::instance().pushCommand(cmd);

    } 
    else if (mode == RATE_MODE) 
    {
        // — RATE_MODE (and others) —
        sp.roll  = s_raw.roll  * MAX_RATE_ROLL;
        sp.pitch = s_raw.pitch * MAX_RATE_PITCH;
        sp.yaw   = s_raw.yaw   * MAX_RATE_YAW;

        // send one attitude-setpoint command
        SystemCommand cmd{};
        cmd.type            = CMD_SET_CONFIG_ATTITUDE;
        cmd.attitudeConfig  = sp;
        CommandSystem::instance().pushCommand(cmd);
    }
    else if (mode == MANUAL_MODE) 
    {
      // — RATE_MODE (and others) —
      sp.roll  = s_raw.roll;
      sp.pitch = s_raw.pitch;
      sp.yaw   = s_raw.yaw ;

      // send one attitude-setpoint command
      SystemCommand cmd{};
      cmd.type            = CMD_SET_CONFIG_ATTITUDE;
      cmd.attitudeConfig  = sp;
      CommandSystem::instance().pushCommand(cmd);
    }
  #else
    // — no mapping: forward raw as rate setpoints —
    SystemCommand cmd{};
    cmd.type            = CMD_SET_CONFIG_ATTITUDE;
    cmd.attitudeConfig  = s_raw;
    CommandSystem::instance().pushCommand(cmd);
  #endif
}
