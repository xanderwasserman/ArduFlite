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
#include "src/utils/ConfigRegistry.h"
#include "include/ConfigKeys.h"

EulerAngles          ControlMixer::s_raw{};
ArduFliteController* ControlMixer::s_ctrl = nullptr;
MixerConfig          ControlMixer::s_config{};

void ControlMixer::init(ArduFliteController& ctrl) 
{
    s_ctrl = &ctrl;
    reloadConfig();
}

void ControlMixer::reloadConfig()
{
    auto& reg = ConfigRegistry::instance();
    
    // Attitude limits
    s_config.maxAttRoll  = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_ROLL);
    s_config.maxAttPitch = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_PITCH);
    s_config.maxAttYaw   = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_YAW);
    
    // Rate limits
    s_config.maxRateRoll  = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_ROLL);
    s_config.maxRatePitch = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_PITCH);
    s_config.maxRateYaw   = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_YAW);
    
    // Mixing coefficients
    s_config.mixRollFromYaw   = reg.get<float>(CONFIG_KEY_MIX_ROLL_FROM_YAW);
    s_config.mixPitchFromRoll = reg.get<float>(CONFIG_KEY_MIX_PITCH_FROM_ROLL);
    s_config.mixYawFromRoll   = reg.get<float>(CONFIG_KEY_MIX_YAW_FROM_ROLL);
    
    LOG_DBG("ControlMixer: config reloaded (maxAttRoll=%.1f, maxRateRoll=%.1f)", 
            s_config.maxAttRoll, s_config.maxRateRoll);
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
    sp.roll  = raw.roll  * s_config.maxAttRoll;
    sp.pitch = raw.pitch * s_config.maxAttPitch;
    sp.yaw   = raw.yaw   * s_config.maxAttYaw;

  #ifdef ENABLE_MIXING
    // SAFE-style cross-mixing
    sp.roll  += s_config.mixRollFromYaw   * (raw.yaw   * s_config.maxAttRoll);
    sp.pitch += s_config.mixPitchFromRoll * (fabsf(raw.roll) * s_config.maxAttPitch);
    sp.yaw   += s_config.mixYawFromRoll   * (raw.roll  * s_config.maxAttYaw);
  #endif

    sp.roll  = constrain(sp.roll,  -s_config.maxAttRoll,  s_config.maxAttRoll);
    sp.pitch = constrain(sp.pitch, -s_config.maxAttPitch, s_config.maxAttPitch);
    sp.yaw   = constrain(sp.yaw,   -s_config.maxAttYaw,   s_config.maxAttYaw);
    return sp;
}

EulerAngles ControlMixer::mixRate(const EulerAngles &raw) 
{
    return {
        raw.roll  * s_config.maxRateRoll,
        raw.pitch * s_config.maxRatePitch,
        raw.yaw   * s_config.maxRateYaw
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
    cmd.type     = CMD_SET_SETPOINT;
    cmd.setpoint = sp;
    CommandSystem::instance().pushCommand(cmd);
}