/**
 * ControlMixer.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.1 | 09 June 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef CONTROL_MIXER_H
#define CONTROL_MIXER_H

#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"
#include "src/controller/ArduFliteController.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/orientation/EulerAngles.h"

#include <Arduino.h>

// Pick exactly one:
#if defined(ENABLE_SAFE_MIXING) && defined(ENABLE_SINGLE_AXIS_CONTROL)
#  error "Only one of ENABLE_SAFE_MIXING or ENABLE_SINGLE_AXIS_CONTROL may be defined"
#endif

/**
 * @class ControlMixer
 * @brief Singleton that applies mode-based scaling (+ optional mixing)
 *        and pushes SystemCommands for your flight-controller.
 */
class ControlMixer {
public:
    /** @return the global instance */
    static ControlMixer& instance() {
        static ControlMixer _inst;
        return _inst;
    }

    /** @brief Initialize with optional custom limits */
    void begin( float maxAttitudeDeg = 45.0f,
                float maxRateDegPerSec = 180.0f );

    /** @brief Update the current flight mode (call from your mode switch) */
    void setMode(ArduFliteMode mode);

    /** @brief Provide a new raw roll input (–1…+1) */
    void setRawRoll(float v);

    /** @brief Provide a new raw pitch input (–1…+1) */
    void setRawPitch(float v);

    /** @brief Provide a new raw yaw input (–1…+1) */
    void setRawYaw(float v);

#ifdef ENABLE_SAFE_MIXING
    /** @brief Adjust SAFE-mix coefficients (defaults = 0.5,0.5) */
    void setSafeMixCoeffs(float yawToRoll, float pitchToRoll);
#endif

#ifdef ENABLE_SINGLE_AXIS_CONTROL
    /** @brief Enable/disable one axis: 0=roll,1=pitch,2=yaw */
    void enableAxis(uint8_t axis, bool enable);
#endif

private:
    ControlMixer();
    ~ControlMixer() = default;
    ControlMixer(const ControlMixer&)            = delete;
    ControlMixer& operator=(const ControlMixer&) = delete;

    /** @brief Recompute mapped setpoints and push a command */
    void computeAndPush_();

    ArduFliteMode currentMode_;
    float raw_[3];             ///< –1…+1 for roll,pitch,yaw
    float maxAttDeg_, maxRateDeg_;

#ifdef ENABLE_SAFE_MIXING
    float mixYtoR_, mixPtoR_;
#endif

#ifdef ENABLE_SINGLE_AXIS_CONTROL
    bool axisEn_[3];
#endif
};


#endif // CONTROL_MIXER_H