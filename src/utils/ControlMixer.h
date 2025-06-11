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
static constexpr uint8_t CH_YAW   = 2;

/**
 * @class ControlMixer
 * @brief Centralizes mode-dependent scaling + mixing.
 *
 * 1. Call init() once in setup(),
 * 2. In each onRoll()/onPitch()/onYaw() callback invoke handleChannelInput().
 */
class ControlMixer {
public:
    /**
     * @brief Remember which controller to drive.
     * @param ctrl your global ArduFliteController instance
     */
    static void init(ArduFliteController& ctrl);

    /**
     * @brief Feed a new raw channel value.
     * @param ch channel index (CH_ROLL, CH_PITCH, CH_YAW)
     * @param v  normalized input (–1…+1)
     *
     * Updates an internal RawInputs, computes a full 3-axis setpoint
     * according to current mode, then pushes the right command(s).
     */
    static void handleChannelInput(uint8_t ch, float v);

private:
    static EulerAngles            s_raw;  ///< latest raw sticks
    static ArduFliteController*   s_ctrl; ///< your controller pointer
};


#endif // CONTROL_MIXER_H