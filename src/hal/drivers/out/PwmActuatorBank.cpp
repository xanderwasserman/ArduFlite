/**
 * PwmActuatorBank.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/out/PwmActuatorBank.h"

#include <cmath>

namespace arduflite::drivers {

using device::ActuatorKind;
using device::ActuatorState;
using device::FailsafeAction;
using device::OutputRange;

namespace {

constexpr float clampf(float v, float lo, float hi) noexcept
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

} // namespace

// ── PwmActuator ─────────────────────────────────────────────────────────────

Status PwmActuator::bind(hal::PwmOut& out, const PwmChannelTuning& tuning,
                         const device::ActuatorChannelConfig& cfg,
                         const hal::Clock& clock)
{
    if (tuning.minPulse_us >= tuning.maxPulse_us) { return Status::InvalidArg; }

    _out    = &out;
    _clock  = &clock;
    _tuning = tuning;
    _cfg    = cfg;

    ARDUFLITE_TRY(_out->attach(tuning.minPulse_us, tuning.maxPulse_us, tuning.frameRate_hz));

    // Start at the range's resting value so the first flush() does not jump.
    _staged     = (cfg.range == OutputRange::Unipolar) ? 0.0f : 0.0f;
    _last       = _staged;
    _slewSeeded = false;
    _state      = ActuatorState::Ok;
    return Status::Ok;
}

void PwmActuator::stage(float normalised)
{
    // NaN/Inf holds the last commanded value. Clamping would NOT catch it —
    // every comparison against NaN is false, so it passes straight through and
    // the subsequent cast to an integer pulse width is undefined behaviour.
    if (!std::isfinite(normalised)) { return; }

    // Latching outputs are never driven by the mixer. Reaching here means a
    // caller has one it should not; ignore rather than fire a parachute.
    if (_cfg.kind == ActuatorKind::Latching) { return; }

    _staged = normalised;
}

std::uint16_t PwmActuator::toPulseUs(float normalised) const
{
    const float lo = (_cfg.range == OutputRange::Unipolar) ? 0.0f : -1.0f;
    float v = clampf(normalised, lo, 1.0f);

    if (_cfg.invert) { v = -v; }
    v += _cfg.trim;
    v = clampf(v, _cfg.minOutput, _cfg.maxOutput);

    // Map to the pulse range. Bipolar pivots about neutral so an asymmetric
    // servo (neutral not centred between min and max) still centres correctly.
    float us;
    if (_cfg.range == OutputRange::Unipolar)
    {
        const float span = static_cast<float>(_tuning.maxPulse_us - _tuning.minPulse_us);
        us = static_cast<float>(_tuning.minPulse_us) + clampf(v, 0.0f, 1.0f) * span;
    }
    else
    {
        const float up   = static_cast<float>(_tuning.maxPulse_us - _tuning.neutralPulse_us);
        const float down = static_cast<float>(_tuning.neutralPulse_us - _tuning.minPulse_us);
        us = static_cast<float>(_tuning.neutralPulse_us) + v * ((v >= 0.0f) ? up : down);
    }

    return static_cast<std::uint16_t>(clampf(us,
                                             static_cast<float>(_tuning.minPulse_us),
                                             static_cast<float>(_tuning.maxPulse_us)) + 0.5f);
}

void PwmActuator::flush()
{
    if (_out == nullptr || _clock == nullptr) { return; }

    float target = _staged;

    // Binary outputs snap; slew limiting is meaningless for a retract.
    if (_cfg.kind == ActuatorKind::Binary)
    {
        target = (target >= 0.5f) ? _cfg.maxOutput : _cfg.minOutput;
    }
    else if (_cfg.maxSlew_perSec > 0.0f)
    {
        const auto now = _clock->now();
        if (!_slewSeeded)
        {
            _lastSlew   = now;
            _slewSeeded = true;
        }
        const float dt_s = hal::toSeconds(now - _lastSlew);
        _lastSlew = now;

        const float maxDelta = _cfg.maxSlew_perSec * dt_s;
        target = clampf(target, _last - maxDelta, _last + maxDelta);
    }

    const float before = target;
    target = clampf(target, _cfg.minOutput, _cfg.maxOutput);
    _state = (target != before) ? ActuatorState::Saturated : ActuatorState::Ok;

    _last = target;
    _out->writeMicroseconds(toPulseUs(target));
}

void PwmActuator::applyFailsafe()
{
    if (_out == nullptr) { return; }

    // A parachute or payload release must NOT be actuated by a disarm.
    if (_cfg.kind == ActuatorKind::Latching) { return; }

    switch (_cfg.onDisable)
    {
        case FailsafeAction::Hold:
            _out->writeMicroseconds(toPulseUs(_last));
            break;

        case FailsafeAction::Neutral:
            _staged = (_cfg.range == OutputRange::Unipolar) ? 0.0f : 0.0f;
            _last   = _staged;
            _out->writeMicroseconds(toPulseUs(_last));
            break;

        case FailsafeAction::Release:
            _out->idle();   // stop pulsing entirely
            break;
    }
}

// ── PwmActuatorBank ─────────────────────────────────────────────────────────

PwmActuatorBank::PwmActuatorBank(std::span<hal::PwmOut* const> pins,
                                 std::span<const PwmChannelTuning> tuning,
                                 const hal::Clock& clock)
    : _pins(pins), _tuning(tuning), _clock(clock)
{
}

Status PwmActuatorBank::begin(std::span<const device::ActuatorChannelConfig> cfgs)
{
    if (cfgs.size() > kMaxChannels)   { return Status::NoSpace; }
    if (cfgs.size() > _pins.size())   { return Status::InvalidArg; }
    if (cfgs.size() > _tuning.size()) { return Status::InvalidArg; }

    _count   = 0;
    _frameHz = 50;

    for (std::size_t i = 0; i < cfgs.size(); ++i)
    {
        if (_pins[i] == nullptr) { return Status::InvalidArg; }

        // NOT ARDUFLITE_TRY: one bad channel must not leave the others unbound
        // and silently un-driven. Report and keep going, like Board's sensor loop.
        const Status s = _channels[i].bind(*_pins[i], _tuning[i], cfgs[i], _clock);
        if (s != Status::Ok) { return s; }

        _ptrs[i] = &_channels[i];
        ++_count;

        // The bank commits at the slowest frame rate its channels want.
        if (_tuning[i].frameRate_hz > _frameHz) { _frameHz = _tuning[i].frameRate_hz; }
    }

    return Status::Ok;
}

device::Actuator* PwmActuatorBank::byRole(std::string_view role)
{
    for (std::uint8_t i = 0; i < _count; ++i)
    {
        if (role == std::string_view{ _channels[i].role() }) { return &_channels[i]; }
    }
    return nullptr;
}

device::CommitResult PwmActuatorBank::commit()
{
    device::CommitResult r{};

    // Once disable() has latched, commit() is a no-op until re-armed. This is
    // what stops a control loop from re-driving surfaces after a disarm.
    if (_disabled.load(std::memory_order_acquire))
    {
        r.status    = Status::NotInitialised;
        r.staleMask = (_count >= 32) ? 0xFFFFFFFFu
                                     : ((1u << _count) - 1u);
        return r;
    }

    for (std::uint8_t i = 0; i < _count; ++i)
    {
        _channels[i].flush();
        ++r.committedCount;
    }

    // PWM cannot fail: writing an LEDC duty register has no error path. A CAN
    // bank is where staleMask earns its keep.
    return r;
}

Status PwmActuatorBank::disable()
{
    _disabled.store(true, std::memory_order_release);

    for (std::uint8_t i = 0; i < _count; ++i)
    {
        _channels[i].applyFailsafe();
    }
    return Status::Ok;
}

} // namespace arduflite::drivers
