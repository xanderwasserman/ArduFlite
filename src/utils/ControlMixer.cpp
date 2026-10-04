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

AxisCommand          ControlMixer::s_raw{};
ArduFliteController* ControlMixer::s_ctrl = nullptr;
MixerConfig          ControlMixer::s_config{};
arduflite::hal::Mutex* ControlMixer::s_configMutex = nullptr;

void ControlMixer::init(ArduFliteController& ctrl, arduflite::hal::Mutex* configMutex)
{
    // Create the mutex and load config BEFORE publishing s_ctrl. handleChannelInput()
    // bails while s_ctrl is null, so assigning it last guarantees no callback can mix
    // against a default-zero config or a not-yet-created mutex.
    s_configMutex = configMutex;
    if (!s_configMutex) {
        LOG_ERR("ControlMixer: no config mutex supplied - mixing disabled");
    }
    reloadConfig();
    s_ctrl = &ctrl;
}

void ControlMixer::reloadConfig()
{
    auto& reg = ConfigRegistry::instance();

    // Build new config outside the lock (ConfigRegistry has its own mutex).
    MixerConfig newCfg;
    newCfg.maxAttRoll_deg  = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_ROLL_DEG);
    newCfg.maxAttPitch_deg = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_PITCH_DEG);
    newCfg.maxAttYaw_deg   = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_YAW_DEG);
    newCfg.maxRateRoll_dps  = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_ROLL_DPS);
    newCfg.maxRatePitch_dps = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_PITCH_DPS);
    newCfg.maxRateYaw_dps   = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_YAW_DPS);
    newCfg.mixRollFromYaw   = reg.get<float>(CONFIG_KEY_MIX_ROLL_FROM_YAW);
    newCfg.mixPitchFromRoll = reg.get<float>(CONFIG_KEY_MIX_PITCH_FROM_ROLL);
    newCfg.mixYawFromRoll   = reg.get<float>(CONFIG_KEY_MIX_YAW_FROM_ROLL);

    // Publish atomically under the config mutex.
    if (s_configMutex)
    {
        std::unique_lock lock(*s_configMutex);
        if (!lock.owns_lock()) {
            LOG_WARN("ControlMixer: config reload skipped; mutex busy");
            return;
        }
        s_config = newCfg;
    }
    else
    {
        s_config = newCfg;  // Pre-FreeRTOS init path (no tasks yet)
    }
    LOG_DBG("ControlMixer: config reloaded (maxAttRoll_deg=%.1f, maxRateRoll_dps=%.1f)",
            newCfg.maxAttRoll_deg, newCfg.maxRateRoll_dps);
}

void ControlMixer::handleChannelInput(uint8_t ch, float v)
{
    // NOTE: s_raw is intentionally lock-free. This relies on handleChannelInput()
    // being called from a single task (the CRSF receiver task, via onRoll/onPitch/
    // onYaw). If a second producer (e.g. a web "virtual stick") is ever added, s_raw
    // must be guarded — it is read/written here without synchronisation.
    if (!s_ctrl)
    {
        LOG_ERR("ControlMixer: not initialized!");
        return;
    }
    // Reject non-finite stick values at the RC→control boundary: constrain() does not
    // catch NaN, so a NaN here would propagate into the setpoint and downstream PIDs.
    if (!isfinite(v))
    {
        LOG_WARN("ControlMixer: non-finite input on ch %u — ignored", ch);
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
    // Read the mode ONCE, then mix AND dispatch for that mode. Reading it here
    // to pick a scaling and again in CommandSystem to pick a setter would let a
    // mode change in between deliver a rate-scaled value to the attitude
    // setter, so the kind travels with the value instead.
    //
    // On a config-reload lock miss the mixer cannot produce a valid setpoint;
    // skip the frame rather than commanding a spurious centre. The next channel
    // update re-mixes.
    bool ok = true;
    switch (s_ctrl->getMode())
    {
        case ATTITUDE_MODE:
        {
            const AttitudeDeg sp = mixAttitude(s_raw, &ok);
            if (ok) { sendSetpoint(SystemCommand::SetpointKind::Attitude, sp.roll, sp.pitch, sp.yaw); }
            break;
        }
        case RATE_MODE:
        {
            const AngularRateDps sp = mixRate(s_raw, &ok);
            if (ok) { sendSetpoint(SystemCommand::SetpointKind::Rate, sp.roll, sp.pitch, sp.yaw); }
            break;
        }
        default:
        {
            const AxisCommand sp = mixManual(s_raw);
            sendSetpoint(SystemCommand::SetpointKind::Manual, sp.roll, sp.pitch, sp.yaw);
            break;
        }
    }
}

AttitudeDeg ControlMixer::mixAttitude(const AxisCommand &raw, bool* ok)
{
    MixerConfig cfg;
    if (s_configMutex) {
        std::unique_lock lock(*s_configMutex, std::try_to_lock);
        if (!lock.owns_lock()) { if (ok) *ok = false; return {0.0f, 0.0f, 0.0f}; }
        cfg = s_config;
    } else {
        cfg = s_config;
    }
    if (ok) *ok = true;

    AttitudeDeg sp;
    sp.roll  = raw.roll  * cfg.maxAttRoll_deg;
    sp.pitch = raw.pitch * cfg.maxAttPitch_deg;
    sp.yaw   = raw.yaw   * cfg.maxAttYaw_deg;

  #ifdef ENABLE_MIXING
    // SAFE-style cross-mixing
    sp.roll  += cfg.mixRollFromYaw   * (raw.yaw   * cfg.maxAttRoll_deg);
    sp.pitch += cfg.mixPitchFromRoll * (fabsf(raw.roll) * cfg.maxAttPitch_deg);
    sp.yaw   += cfg.mixYawFromRoll   * (raw.roll  * cfg.maxAttYaw_deg);
  #endif

    sp.roll  = constrain(sp.roll,  -cfg.maxAttRoll_deg,  cfg.maxAttRoll_deg);
    sp.pitch = constrain(sp.pitch, -cfg.maxAttPitch_deg, cfg.maxAttPitch_deg);
    sp.yaw   = constrain(sp.yaw,   -cfg.maxAttYaw_deg,   cfg.maxAttYaw_deg);
    return sp;
}

AngularRateDps ControlMixer::mixRate(const AxisCommand &raw, bool* ok)
{
    MixerConfig cfg;
    if (s_configMutex) {
        std::unique_lock lock(*s_configMutex, std::try_to_lock);
        if (!lock.owns_lock()) { if (ok) *ok = false; return {0.0f, 0.0f, 0.0f}; }
        cfg = s_config;
    } else {
        cfg = s_config;
    }
    if (ok) *ok = true;

    // Clamp to ±maxRate for parity with mixAttitude(): a stick value outside [-1,1]
    // (e.g. a future Raw/Custom channel) must not produce an out-of-range rate setpoint.
    return {
        constrain(raw.roll  * cfg.maxRateRoll_dps,  -cfg.maxRateRoll_dps,  cfg.maxRateRoll_dps),
        constrain(raw.pitch * cfg.maxRatePitch_dps, -cfg.maxRatePitch_dps, cfg.maxRatePitch_dps),
        constrain(raw.yaw   * cfg.maxRateYaw_dps,   -cfg.maxRateYaw_dps,   cfg.maxRateYaw_dps)
    };
}

AxisCommand ControlMixer::mixManual(const AxisCommand &raw)
{
    // raw is already in –1…+1, so we just pass it through
    return raw;
}

void ControlMixer::sendSetpoint(SystemCommand::SetpointKind kind,
                                float roll, float pitch, float yaw)
{
    SystemCommand cmd{};
    cmd.type          = CMD_SET_SETPOINT;
    cmd.setpointKind  = kind;
    cmd.setpointRoll  = roll;
    cmd.setpointPitch = pitch;
    cmd.setpointYaw   = yaw;
    CommandSystem::instance().pushCommand(cmd);
}
