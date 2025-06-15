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

#include "include/ControlMixerConfiguration.h"
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

    /// Called on each channel update
    static void handleChannelInput(uint8_t ch, float v);

    /// mix raw RC [-1..1] into an attitude setpoint (degrees) with optional mixing
    static EulerAngles mixAttitude(const EulerAngles &raw);

    /// mix raw RC [-1..1] into a rate setpoint (deg/s)
    static EulerAngles mixRate(const EulerAngles &raw);

    /// direct passthrough, raw → servo commands
    static EulerAngles mixManual(const EulerAngles &raw);

    /// general dispatcher: chooses Attitude/Rate/Manual based on mode
    static EulerAngles mix(const EulerAngles &raw, ArduFliteMode mode);

    /// actually send that setpoint into the controller/command bus
    static void sendSetpoint(const EulerAngles &sp);

private:
    static EulerAngles            s_raw;  ///< latest raw sticks
    static ArduFliteController*   s_ctrl; ///< your controller pointer
};


#endif // CONTROL_MIXER_H