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

#include <mutex>

#include "src/hal/platform/Mutex.h"

#include "src/utils/CommandSystem.h"
#include "src/utils/Logging.h"
#include "src/controller/ArduFliteController.h"
#include "src/core/FlightTypes.h"


// Channel indices (match your CRSF config)
static constexpr uint8_t CH_ROLL  = 0;
static constexpr uint8_t CH_PITCH = 1;
static constexpr uint8_t CH_YAW   = 3;

/**
 * @brief Runtime mixer limits, mirroring the mix.* config keys.
 *
 * Loaded from ConfigRegistry and refreshed through the observer pattern.
 *
 * @note The unit suffixes are load-bearing, not decoration. These are bare
 *       floats, so the identifier is the ONLY thing carrying the unit — and the
 *       attitude and rate limits differ by roughly a factor of ten while
 *       looking almost identical at a call site.
 */
struct MixerConfig {
    /// Attitude limits — the maximum commanded angle at full stick.
    float maxAttRoll_deg;
    float maxAttPitch_deg;
    float maxAttYaw_deg;

    /// Rate limits — the maximum commanded angular rate at full stick.
    float maxRateRoll_dps;
    float maxRatePitch_dps;
    float maxRateYaw_dps;

    /// Cross-axis mixing coefficients. Dimensionless ratios, so deliberately
    /// unsuffixed — a suffix here would imply a unit that does not exist.
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
    /// @param configMutex guards the cached mixer config. Until it is
    ///        supplied the mixer refuses to mix, rather than reading an
    ///        unguarded config from an RC callback.
    static void init(ArduFliteController& ctrl, arduflite::hal::Mutex* configMutex);

    /// Reload config values from ConfigRegistry (called by observer)
    static void reloadConfig();

    /// Called on each channel update.
    /// @note Single-producer only: must be called from one task (the CRSF receiver
    ///       task). s_raw is updated lock-free under that assumption.
    static void handleChannelInput(uint8_t ch, float v);

    /// mix raw RC [-1..1] into an attitude setpoint (degrees) with optional mixing.
    /// @param ok if non-null, set false when the config snapshot could not be taken
    ///           (reload in progress) and the result is not usable; true otherwise.
    static AttitudeDeg mixAttitude(const AxisCommand &raw, bool* ok = nullptr);

    /// mix raw RC [-1..1] into a rate setpoint (deg/s). @param ok see mixAttitude().
    static AngularRateDps mixRate(const AxisCommand &raw, bool* ok = nullptr);

    /// direct passthrough, raw → servo commands
    static AxisCommand mixManual(const AxisCommand &raw);

    /// general dispatcher: chooses Attitude/Rate/Manual based on mode.
    /// @param ok see mixAttitude(); always true for Manual/default modes.
    /// @deprecated Removed in Phase 6B — it had to return three different
    ///             quantities from one signature. onChannel() now mixes and
    ///             dispatches together, reading the mode exactly once.

    /// actually send that setpoint into the controller/command bus
    static void sendSetpoint(SystemCommand::SetpointKind kind, float roll, float pitch, float yaw);

private:
    static AxisCommand            s_raw;    ///< latest raw sticks, -1..+1
    static ArduFliteController*   s_ctrl;   ///< your controller pointer
    static MixerConfig            s_config; ///< cached config values (protected by s_configMutex)
    /// Protects s_config. Injected by init(), not created here — see the
    /// note in ArduFliteRateController's constructor.
    static arduflite::hal::Mutex* s_configMutex;
};


#endif // CONTROL_MIXER_H