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
#include "src/controller/ArduFliteController.h"
#include "src/utils/CommandSystem.h"

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
    // update raw stick state
    switch (ch) 
    {
      case CH_ROLL:  s_raw.roll  = v;  break;
      case CH_PITCH: s_raw.pitch = v;  break;
      case CH_YAW:   s_raw.yaw   = v;  break;
      default: return;
    }
    // grab mode & mix
    EulerAngles sp = mix(s_raw, s_ctrl->getMode());
    sendSetpoint(sp);
}

EulerAngles ControlMixer::mixAttitude(const EulerAngles &raw) 
{
    EulerAngles sp;
    sp.roll  = raw.roll  * MAX_ATT_ROLL;
    sp.pitch = raw.pitch * MAX_ATT_PITCH;
    sp.yaw   = raw.yaw   * MAX_ATT_YAW;

  #ifdef ENABLE_MIXING
    // SAFE-style cross-mixing
    sp.roll  += MIX_ATT_ROLL_FROM_YAW   * (raw.yaw   * MAX_ATT_ROLL);
    sp.pitch += MIX_ATT_PITCH_FROM_ROLL * (fabsf(raw.roll) * MAX_ATT_PITCH);
    sp.yaw   += MIX_ATT_YAW_FROM_ROLL   * (raw.roll  * MAX_ATT_YAW);
  #endif

    sp.roll  = constrain(sp.roll,  -MAX_ATT_ROLL,  MAX_ATT_ROLL);
    sp.pitch = constrain(sp.pitch, -MAX_ATT_PITCH, MAX_ATT_PITCH);
    sp.yaw   = constrain(sp.yaw,   -MAX_ATT_YAW,   MAX_ATT_YAW);
    return sp;
}

EulerAngles ControlMixer::mixRate(const EulerAngles &raw) 
{
    return {
        raw.roll  * MAX_RATE_ROLL,
        raw.pitch * MAX_RATE_PITCH,
        raw.yaw   * MAX_RATE_YAW
    };
}

EulerAngles ControlMixer::mixManual(const EulerAngles &raw) 
{
    // raw is already in –1…+1, so we just pass it through
    return raw;
}

EulerAngles ControlMixer::mix(const EulerAngles &raw, ArduFliteMode mode) 
{
    switch (mode) 
    {
      case ATTITUDE_MODE: return mixAttitude(raw);
      case RATE_MODE:     return mixRate(raw);
      case MANUAL_MODE:   return mixManual(raw);
      default:            return {0,0,0};
    }
}

void ControlMixer::sendSetpoint(const EulerAngles &sp) 
{
    SystemCommand cmd{};
    cmd.type           = CMD_SET_CONFIG_ATTITUDE;
    cmd.attitudeConfig = sp;
    CommandSystem::instance().pushCommand(cmd);
}