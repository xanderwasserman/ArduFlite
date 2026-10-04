/**
 * PwmActuatorBank.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief PWM implementation of Actuator / ActuatorBank.
 *
 * Per-output calibration (endpoints, inversion, trim, travel limits, slew) lives
 * on PwmActuator because it must apply to EVERY writer — controller, manual
 * passthrough and failsafe alike. Airframe mixing does not live here.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_OUT_PWMACTUATORBANK_H
#define ARDUFLITE_HAL_DRIVERS_OUT_PWMACTUATORBANK_H

#include <array>
#include <atomic>
#include <span>

#include "src/hal/core/Result.h"
#include "src/hal/device/Actuator.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Io.h"

namespace arduflite::drivers {

/// Transport-specific tuning — deliberately NOT in ActuatorChannelConfig.
struct PwmChannelTuning
{
    std::uint16_t minPulse_us     = 1000;
    std::uint16_t neutralPulse_us = 1500;
    std::uint16_t maxPulse_us     = 2000;
    std::uint16_t frameRate_hz    = 50;
};

class PwmActuator final : public device::Actuator
{
public:
    Status bind(hal::PwmOut& out, const PwmChannelTuning& tuning,
                const device::ActuatorChannelConfig& cfg, const hal::Clock& clock);

    void stage(float normalised) override;

    [[nodiscard]] float                 lastCommand() const override { return _last; }
    [[nodiscard]] device::ActuatorState state()       const override { return _state; }
    [[nodiscard]] device::ActuatorKind  kind()        const override { return _cfg.kind; }
    [[nodiscard]] const char*           role()        const override { return _cfg.role; }

    /// Convert the staged command to a pulse and push it. Called by the bank.
    void flush();

    /// Apply this channel's FailsafeAction. Latching channels are never actuated.
    void applyFailsafe();

    [[nodiscard]] std::uint16_t toPulseUs(float normalised) const;

private:
    hal::PwmOut*                  _out   = nullptr;
    const hal::Clock*             _clock = nullptr;
    PwmChannelTuning              _tuning{};
    device::ActuatorChannelConfig _cfg{};

    float                  _staged = 0.0f;   ///< raw, pre-slew
    float                  _last   = 0.0f;   ///< post-slew, post-clamp
    hal::Clock::time_point _lastSlew{};
    bool                   _slewSeeded = false;
    device::ActuatorState  _state  = device::ActuatorState::Ok;
};

class PwmActuatorBank final : public device::ActuatorBank
{
public:
    static constexpr std::uint8_t kMaxChannels = 8;

    PwmActuatorBank(std::span<hal::PwmOut* const> pins,
                    std::span<const PwmChannelTuning> tuning,
                    const hal::Clock& clock);

    Status begin(std::span<const device::ActuatorChannelConfig> cfgs) override;

    [[nodiscard]] std::span<device::Actuator* const> actuators() override
    {
        return { _ptrs.data(), _count };
    }
    [[nodiscard]] device::Actuator* byRole(std::string_view role) override;

    device::CommitResult commit() override;
    Status               disable() override;

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return _frameHz; }
    [[nodiscard]] const char*   transport()     const override { return "PWM"; }

private:
    std::span<hal::PwmOut* const>     _pins;
    std::span<const PwmChannelTuning> _tuning;
    const hal::Clock&                 _clock;

    std::array<PwmActuator, kMaxChannels>       _channels{};
    std::array<device::Actuator*, kMaxChannels> _ptrs{};
    std::uint8_t                                _count   = 0;
    std::uint16_t                               _frameHz = 50;

    /// Latched by disable() from any task; checked by commit(), which becomes a
    /// no-op until re-armed. A disarm that has to wait for a lock is not a disarm.
    std::atomic<bool> _disabled{ false };
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_OUT_PWMACTUATORBANK_H
