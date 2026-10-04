/**
 * SensorSelector.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Chooses which sensor instance feeds the estimator.
 *
 * Redundancy is a board-descriptor change (ADR-019); *choosing between*
 * instances is flight-layer policy, and this is where it lives.
 *
 * Today every span holds one entry and FirstHealthySelector picks index 0. The
 * interface is pinned now, while it is cheap, so that adding real failover later
 * is a new implementation plus a branch in InertialSubsystem::tick() — no
 * interface change, nothing above the estimation layer, no descriptor change
 * (ADR-026).
 */
#ifndef ARDUFLITE_ESTIMATION_SENSOR_SELECTOR_H
#define ARDUFLITE_ESTIMATION_SENSOR_SELECTOR_H

#include <span>

#include "src/estimation/ImuState.h"
#include "src/hal/core/NonCopyable.h"
#include "src/hal/device/Sensor.h"

namespace arduflite::estimation {

enum class SelectionPolicy : std::uint8_t
{
    FirstHealthy,   ///< lowest index whose health() is Ok. Shipped.
    Blended,        ///< future: crossfade across a switch over N ticks
    Median,         ///< future: median-of-three, never switches
};

class SensorSelector : private NonCopyable
{
public:
    virtual ~SensorSelector() = default;

    /// Called once per tick, AFTER sample() and BEFORE read(). Sampling every
    /// instance every tick is what leaves crossfading and median voting open.
    virtual void evaluate(hal::Clock::time_point now) = 0;

    [[nodiscard]] virtual device::Accelerometer* primaryAccel() = 0;
    [[nodiscard]] virtual device::Gyroscope*     primaryGyro()  = 0;
    [[nodiscard]] virtual device::Barometer*     primaryBaro()  = 0;

    /// Null when the board carries no magnetometer, which is the normal case
    /// for the boards shipped today. Callers must branch on it rather than
    /// assume a heading reference exists (ADR-052).
    [[nodiscard]] virtual device::Magnetometer*  primaryMag()   = 0;

    [[nodiscard]] virtual SelectionState state() const = 0;

    /// True only on the tick a switch happened, so the estimator can re-seed or
    /// start a crossfade. This is the hook that makes the failover transient
    /// (review R14) solvable without redesigning anything.
    [[nodiscard]] virtual bool switchedThisTick() const = 0;
};

/**
 * @brief Lowest-indexed healthy instance wins.
 *
 * Deliberately the simplest policy that is still honest about switching: it
 * counts switches and timestamps them, so the flash log can answer "did it
 * switch, and when?" even though the answer is currently always "no".
 *
 * @note Holds no ownership. The spans must outlive it — they point into
 *       BoardStorage, which is a file-scope object, so this always holds.
 */
class FirstHealthySelector final : public SensorSelector
{
public:
    /// The magnetometer span defaults to empty: a board without one passes
    /// three arguments and gets six-axis fusion, with no further opt-out.
    FirstHealthySelector(std::span<device::Accelerometer* const> accelerometers,
                         std::span<device::Gyroscope* const>     gyroscopes,
                         std::span<device::Barometer* const>     barometers,
                         std::span<device::Magnetometer* const>  magnetometers = {})
        : _accelerometers(accelerometers)
        , _gyroscopes(gyroscopes)
        , _barometers(barometers)
        , _magnetometers(magnetometers) {}

    void evaluate(hal::Clock::time_point now) override;

    [[nodiscard]] device::Accelerometer* primaryAccel() override;
    [[nodiscard]] device::Gyroscope*     primaryGyro()  override;
    [[nodiscard]] device::Barometer*     primaryBaro()  override;
    [[nodiscard]] device::Magnetometer*  primaryMag()   override;

    [[nodiscard]] SelectionState state() const override { return _state; }
    [[nodiscard]] bool switchedThisTick() const override { return _switchedThisTick; }

private:
    std::span<device::Accelerometer* const> _accelerometers;
    std::span<device::Gyroscope* const>     _gyroscopes;
    std::span<device::Barometer* const>     _barometers;
    std::span<device::Magnetometer* const>  _magnetometers;

    SelectionState _state{};
    bool           _switchedThisTick = false;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_SENSOR_SELECTOR_H
