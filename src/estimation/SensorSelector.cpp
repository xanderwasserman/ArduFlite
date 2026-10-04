/**
 * SensorSelector.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/estimation/SensorSelector.h"

namespace arduflite::estimation {

namespace {

/**
 * @brief Index of the first instance reporting Ok, or the current one.
 *
 * Staying put when nothing is healthy is deliberate. Switching to an equally
 * broken instance would churn the selection, inflate switchCount and, once
 * crossfading exists, keep the estimator permanently mid-transition — all
 * without improving the data.
 */
template <typename T>
std::uint8_t firstHealthy(std::span<T* const> instances, std::uint8_t current)
{
    for (std::uint8_t i = 0; i < instances.size(); ++i)
    {
        if (instances[i] != nullptr &&
            instances[i]->health() == device::SensorHealth::Ok)
        {
            return i;
        }
    }
    return current;
}

} // namespace

void FirstHealthySelector::evaluate(hal::Clock::time_point now)
{
    const std::uint8_t accel = firstHealthy(_accelerometers, _state.activeAccel);
    const std::uint8_t gyro  = firstHealthy(_gyroscopes,     _state.activeGyro);
    const std::uint8_t baro  = firstHealthy(_barometers,     _state.activeBaro);
    const std::uint8_t mag   = firstHealthy(_magnetometers,  _state.activeMag);

    _switchedThisTick = (accel != _state.activeAccel) ||
                        (gyro  != _state.activeGyro)  ||
                        (baro  != _state.activeBaro)  ||
                        (mag   != _state.activeMag);

    if (_switchedThisTick)
    {
        // One increment per tick that changed anything, not one per sensor: the
        // count answers "how many transients did the estimator see?", and a tick
        // where the accelerometer and gyroscope switch together is one transient.
        ++_state.switchCount;
        _state.lastSwitch = now;
    }

    _state.activeAccel = accel;
    _state.activeGyro  = gyro;
    _state.activeBaro  = baro;
    _state.activeMag   = mag;
}

device::Accelerometer* FirstHealthySelector::primaryAccel()
{
    if (_state.activeAccel >= _accelerometers.size()) { return nullptr; }
    return _accelerometers[_state.activeAccel];
}

device::Gyroscope* FirstHealthySelector::primaryGyro()
{
    if (_state.activeGyro >= _gyroscopes.size()) { return nullptr; }
    return _gyroscopes[_state.activeGyro];
}

device::Barometer* FirstHealthySelector::primaryBaro()
{
    if (_state.activeBaro >= _barometers.size()) { return nullptr; }
    return _barometers[_state.activeBaro];
}

device::Magnetometer* FirstHealthySelector::primaryMag()
{
    // An empty span lands here on every tick of every board without a
    // magnetometer, so this is the ordinary path, not an error path.
    if (_state.activeMag >= _magnetometers.size()) { return nullptr; }
    return _magnetometers[_state.activeMag];
}

} // namespace arduflite::estimation
