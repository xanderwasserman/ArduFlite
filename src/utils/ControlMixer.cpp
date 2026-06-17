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
SemaphoreHandle_t    ControlMixer::s_configMutex = nullptr;

void ControlMixer::init(ArduFliteController& ctrl)
{
    // Create the mutex and load config BEFORE publishing s_ctrl. handleChannelInput()
    // bails while s_ctrl is null, so assigning it last guarantees no callback can mix
    // against a default-zero config or a not-yet-created mutex.
    if (!s_configMutex) {
        s_configMutex = xSemaphoreCreateMutex();
        if (!s_configMutex) {
            LOG_ERR("ControlMixer: failed to create config mutex");
        }
    }
    reloadConfig();
    s_ctrl = &ctrl;
}

void ControlMixer::reloadConfig()
{
    auto& reg = ConfigRegistry::instance();

    // Build new config outside the lock (ConfigRegistry has its own mutex).
    MixerConfig newCfg;
    newCfg.maxAttRoll  = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_ROLL);
    newCfg.maxAttPitch = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_PITCH);
    newCfg.maxAttYaw   = reg.get<float>(CONFIG_KEY_MIX_MAX_ATT_YAW);
    newCfg.maxRateRoll  = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_ROLL);
    newCfg.maxRatePitch = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_PITCH);
    newCfg.maxRateYaw   = reg.get<float>(CONFIG_KEY_MIX_MAX_RATE_YAW);
    newCfg.mixRollFromYaw   = reg.get<float>(CONFIG_KEY_MIX_ROLL_FROM_YAW);
    newCfg.mixPitchFromRoll = reg.get<float>(CONFIG_KEY_MIX_PITCH_FROM_ROLL);
    newCfg.mixYawFromRoll   = reg.get<float>(CONFIG_KEY_MIX_YAW_FROM_ROLL);

    // Publish atomically under the config mutex.
    if (s_configMutex)
    {
        SemaphoreLock lock(s_configMutex);
        if (!lock.acquired()) {
            LOG_WARN("ControlMixer: config reload skipped; mutex busy");
            return;
        }
        s_config = newCfg;
    }
    else
    {
        s_config = newCfg;  // Pre-FreeRTOS init path (no tasks yet)
    }
    LOG_DBG("ControlMixer: config reloaded (maxAttRoll=%.1f, maxRateRoll=%.1f)",
            newCfg.maxAttRoll, newCfg.maxRateRoll);
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
    // grab mode & mix
    bool ok = true;
    EulerAngles sp = mix(s_raw, s_ctrl->getMode(), &ok);
    // On a config-reload lock-miss the mixer can't produce a valid setpoint; skip this
    // frame rather than commanding a spurious zero/center. The next channel update re-mixes.
    if (ok)
    {
        sendSetpoint(sp);
    }
}

EulerAngles ControlMixer::mixAttitude(const EulerAngles &raw, bool* ok)
{
    MixerConfig cfg;
    if (s_configMutex) {
        SemaphoreLock lock(s_configMutex, 0);
        if (!lock.acquired()) { if (ok) *ok = false; return {0.0f, 0.0f, 0.0f}; }
        cfg = s_config;
    } else {
        cfg = s_config;
    }
    if (ok) *ok = true;

    EulerAngles sp;
    sp.roll  = raw.roll  * cfg.maxAttRoll;
    sp.pitch = raw.pitch * cfg.maxAttPitch;
    sp.yaw   = raw.yaw   * cfg.maxAttYaw;

  #ifdef ENABLE_MIXING
    // SAFE-style cross-mixing
    sp.roll  += cfg.mixRollFromYaw   * (raw.yaw   * cfg.maxAttRoll);
    sp.pitch += cfg.mixPitchFromRoll * (fabsf(raw.roll) * cfg.maxAttPitch);
    sp.yaw   += cfg.mixYawFromRoll   * (raw.roll  * cfg.maxAttYaw);
  #endif

    sp.roll  = constrain(sp.roll,  -cfg.maxAttRoll,  cfg.maxAttRoll);
    sp.pitch = constrain(sp.pitch, -cfg.maxAttPitch, cfg.maxAttPitch);
    sp.yaw   = constrain(sp.yaw,   -cfg.maxAttYaw,   cfg.maxAttYaw);
    return sp;
}

EulerAngles ControlMixer::mixRate(const EulerAngles &raw, bool* ok)
{
    MixerConfig cfg;
    if (s_configMutex) {
        SemaphoreLock lock(s_configMutex, 0);
        if (!lock.acquired()) { if (ok) *ok = false; return {0.0f, 0.0f, 0.0f}; }
        cfg = s_config;
    } else {
        cfg = s_config;
    }
    if (ok) *ok = true;

    // Clamp to ±maxRate for parity with mixAttitude(): a stick value outside [-1,1]
    // (e.g. a future Raw/Custom channel) must not produce an out-of-range rate setpoint.
    return {
        constrain(raw.roll  * cfg.maxRateRoll,  -cfg.maxRateRoll,  cfg.maxRateRoll),
        constrain(raw.pitch * cfg.maxRatePitch, -cfg.maxRatePitch, cfg.maxRatePitch),
        constrain(raw.yaw   * cfg.maxRateYaw,   -cfg.maxRateYaw,   cfg.maxRateYaw)
    };
}

EulerAngles ControlMixer::mixManual(const EulerAngles &raw)
{
    // raw is already in –1…+1, so we just pass it through
    return raw;
}

EulerAngles ControlMixer::mix(const EulerAngles &raw, ArduFliteMode mode, bool* ok)
{
    // Default to a valid result; only the config-reload lock-miss inside the
    // attitude/rate readers clears it. MANUAL and the safe-zero default always succeed.
    if (ok) *ok = true;
    switch (mode)
    {
      case ATTITUDE_MODE: return mixAttitude(raw, ok);
      case RATE_MODE:     return mixRate(raw, ok);
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
