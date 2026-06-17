/**
 * ControlMixer.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 11 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"
#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"

#include <Arduino.h>

// Channel indices (match your CRSF config)
static constexpr uint8_t CH_ROLL  = 0;
static constexpr uint8_t CH_PITCH = 1;
static constexpr uint8_t CH_YAW   = 3;

/**
 * @brief Cached mixer configuration values.
 *        Loaded from ConfigRegistry, updated via observer pattern.
 */
struct MixerConfig {
    // Attitude limits (degrees)
    float maxAttRoll;
    float maxAttPitch;
    float maxAttYaw;

    // Rate limits (deg/s)
    float maxRateRoll;
    float maxRatePitch;
    float maxRateYaw;

    // Mixing coefficients
    float mixRollFromYaw;
    float mixPitchFromRoll;
    float mixYawFromRoll;
};

/**
 * @class ControlMixer
 * @brief Centralizes mode-dependent scaling + mixing.
 *
 * 1. Call init() once in setup(),
 * 2. In each onRoll()/onPitch()/onYaw() callback invoke handleChannelInput().
 */
class ControlMixer {
public:
    /// Must be called once before any mixing.
    static void init(ArduFliteController& ctrl);

    /// Reload config values from ConfigRegistry (called by observer)
    static void reloadConfig();

    /// Called on each channel update.
    /// @note Single-producer only: must be called from one task (the CRSF receiver
    ///       task). s_raw is updated lock-free under that assumption.
    static void handleChannelInput(uint8_t ch, float v);

    /// mix raw RC [-1..1] into an attitude setpoint (degrees) with optional mixing.
    /// @param ok if non-null, set false when the config snapshot could not be taken
    ///           (reload in progress) and the result is not usable; true otherwise.
    static EulerAngles mixAttitude(const EulerAngles &raw, bool* ok = nullptr);

    /// mix raw RC [-1..1] into a rate setpoint (deg/s). @param ok see mixAttitude().
    static EulerAngles mixRate(const EulerAngles &raw, bool* ok = nullptr);

    /// direct passthrough, raw → servo commands
    static EulerAngles mixManual(const EulerAngles &raw);

    /// general dispatcher: chooses Attitude/Rate/Manual based on mode.
    /// @param ok see mixAttitude(); always true for Manual/default modes.
    static EulerAngles mix(const EulerAngles &raw, ArduFliteMode mode, bool* ok = nullptr);

    /// actually send that setpoint into the controller/command bus
    static void sendSetpoint(const EulerAngles &sp);

private:
    static EulerAngles            s_raw;    ///< latest raw sticks
    static ArduFliteController*   s_ctrl;   ///< your controller pointer
    static MixerConfig            s_config; ///< cached config values (protected by s_configMutex)
    static SemaphoreHandle_t      s_configMutex; ///< protects s_config from concurrent reads/writes
};


#endif // CONTROL_MIXER_H