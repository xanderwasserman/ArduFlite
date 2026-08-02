/**
 * Actuator.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Two levels (ADR-025):
 *          Actuator     - ONE output. Flight code holds this, BY NAME.
 *          ActuatorBank - ONE TRANSPORT's outputs. commit() and disable() live
 *                         here because they are batched BUS operations.
 *
 * THREADING CONTRACT:
 *   stage(), commit(), begin() are single-writer — the control loop only.
 *   disable() is the ONE method callable from any task, and must remain so:
 *   a disarm that has to wait for a lock is not a disarm.
 */
#ifndef ARDUFLITE_HAL_DEVICE_ACTUATOR_H
#define ARDUFLITE_HAL_DEVICE_ACTUATOR_H

#include <cstdint>
#include <span>
#include <string_view>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::device {

enum class OutputRange : std::uint8_t { Bipolar, Unipolar };   ///< [-1,1] vs [0,1]

/**
 * @brief What KIND of thing this output drives. Orthogonal to the transport:
 *        a binary retract can hang off PWM or CAN.
 */
enum class ActuatorKind : std::uint8_t
{
    Proportional,  ///< Control surface, throttle — continuous, slew-limited
    Binary,        ///< Retract, relay, cowl flap — snaps to min/max, slew ignored
    Latching,      ///< Parachute, payload release — one-shot. disable() must NOT
                   ///< actuate it, and it is never handed to the mixer at all.
};

enum class FailsafeAction : std::uint8_t
{
    Hold,       ///< Freeze at the last commanded value
    Neutral,    ///< Drive to the configured neutral
    Release,    ///< Stop driving entirely (no pulse / no CAN command / torque off)
};

enum class ActuatorState : std::uint8_t
{
    Ok,
    Saturated,   ///< Command clipped by travel limits
    Stale,       ///< Last commit() did not reach the hardware
    Fault,       ///< Device reported a fault
    Offline,     ///< Device not responding (CAN node dropped, etc.)
};

/// Transport-NEUTRAL. No microseconds, no node IDs — those go in the driver ctor.
struct ActuatorChannelConfig
{
    const char*    role           = "";
    ActuatorKind   kind           = ActuatorKind::Proportional;
    OutputRange    range          = OutputRange::Bipolar;
    bool           invert         = false;
    float          trim           = 0.0f;
    float          minOutput      = -1.0f;
    float          maxOutput      =  1.0f;
    float          maxSlew_perSec = 4.0f;    ///< 0 = unlimited; ignored for Binary
    FailsafeAction onDisable      = FailsafeAction::Neutral;
};

/// Only meaningful where hasFeedback(): CANopen drives, serial servos, DShot ESCs.
struct ActuatorFeedback
{
    float                  position   = 0.0f;   ///< normalised, MEASURED
    float                  current_a  = 0.0f;
    float                  temp_c     = 0.0f;
    std::uint16_t          faultFlags = 0;
    hal::Clock::time_point time{};
};

/**
 * @brief ONE output.
 */
class Actuator : private NonCopyable
{
public:
    virtual ~Actuator() = default;

    /// Stage a command. Applies trim, travel limits, inversion and slew.
    /// NaN/Inf holds the previous value. Purely local — no bus access, cannot fail.
    virtual void stage(float normalised) = 0;

    [[nodiscard]] virtual float         lastCommand() const = 0;   ///< post-slew, post-clamp
    [[nodiscard]] virtual ActuatorState state()       const = 0;
    [[nodiscard]] virtual ActuatorKind  kind()        const = 0;
    [[nodiscard]] virtual const char*   role()        const = 0;

    [[nodiscard]] virtual bool hasFeedback() const { return false; }

    virtual Status readFeedback(ActuatorFeedback& out) const
    {
        (void)out;
        return Status::NotSupported;
    }
};

/**
 * @brief Result of commit(). staleMask names WHICH channels failed to update —
 *        "something failed" is not actionable in flight; "the elevator is stale" is.
 */
struct CommitResult
{
    Status        status         = Status::Ok;
    std::uint32_t staleMask      = 0;   ///< bit N = channel N did NOT update
    std::uint8_t  committedCount = 0;

    [[nodiscard]] constexpr bool allOk() const noexcept
    {
        return status == Status::Ok && staleMask == 0;
    }
};

/**
 * @brief ONE TRANSPORT's worth of outputs.
 */
class ActuatorBank : private NonCopyable
{
public:
    virtual ~ActuatorBank() = default;

    virtual Status begin(std::span<const ActuatorChannelConfig> cfgs) = 0;

    /// Flight code resolves these ONCE at composition time and then holds
    /// Actuator& — it does not index into the bank per tick.
    [[nodiscard]] virtual std::span<Actuator* const> actuators() = 0;
    [[nodiscard]] virtual Actuator* byRole(std::string_view role) = 0;

    /// Push staged commands to hardware, atomically WITHIN this bank.
    virtual CommitResult commit() = 0;

    /// Applies each channel's FailsafeAction. Safe from ANY task.
    /// Latching channels are never actuated by this.
    virtual Status disable() = 0;

    /// Symmetric with Sensor::nativeRate_hz(). PWM 50-333, DShot up to 32k,
    /// CANopen whatever the PDO budget allows.
    [[nodiscard]] virtual std::uint16_t nativeRate_hz() const = 0;

    [[nodiscard]] virtual const char* transport() const = 0;   ///< "PWM", "CANopen"
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_ACTUATOR_H
