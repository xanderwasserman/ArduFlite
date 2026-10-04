/**
 * SimSensors.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Synthetic sensors for the host_sim board.
 *
 * They implement the same device:: interfaces the real drivers do, so the
 * estimation layer cannot tell the difference — which is the whole point of
 * this exercise.
 *
 * The motion model is deliberately simple: a body rotating at a constant rate
 * about one axis, with gravity resolved into the body frame. Enough to drive
 * the estimator through a real trajectory and check it tracks; not a flight
 * dynamics model, and not pretending to be one.
 */
#ifndef ARDUFLITE_HOST_SIM_SENSORS_H
#define ARDUFLITE_HOST_SIM_SENSORS_H

#include <cmath>

#include "src/hal/device/Sensor.h"

namespace arduflite::sim {

/// Accelerometer + gyroscope on one simulated part, as an MPU-6500 would be.
class SimImu final : public device::Sensor,
                     public device::Accelerometer,
                     public device::Gyroscope
{
public:
    Status probe() override { return Status::Ok; }
    Status begin() override { return Status::Ok; }

    Status sample() override
    {
        ++sampleCount;

        // Advance the modelled attitude by the commanded rate.
        _roll_rad += rollRate_dps * (M_PI / 180.0) * dt_s;

        // Gravity resolved into the body frame for that roll angle.
        //
        // Sign matters and was wrong first time round. A world vector expressed
        // in a frame rotating by +phi about X transforms by R_x(-phi), giving
        // (0, +sin phi, cos phi) — NOT -sin. With -sin the accelerometer
        // described a roll in the opposite direction to the gyroscope, the two
        // fought each other through the fusion filter, and the estimate stalled
        // around 20 degrees while the "truth" ran to 60. A physically
        // impossible sensor pair produces a plausible-looking wrong answer.
        _accel = Vec3f{ 0.0f,
                        static_cast<float>(std::sin(_roll_rad)),
                        static_cast<float>(std::cos(_roll_rad)) };
        _gyro  = Vec3f{ rollRate_dps, 0.0f, 0.0f };
        return Status::Ok;
    }

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 500; }
    [[nodiscard]] const char* name() const override { return "SimIMU"; }
    [[nodiscard]] device::SensorHealth health() const override { return _health; }

    Status read(device::AccelSample& out) const override { out.accel_g = _accel; return Status::Ok; }
    Status setRange_g(std::uint8_t) override { return Status::Ok; }
    [[nodiscard]] std::uint8_t range_g() const override { return 4; }

    Status read(device::GyroSample& out) const override { out.rate_dps = _gyro; return Status::Ok; }
    Status setRange_dps(std::uint16_t) override { return Status::Ok; }
    [[nodiscard]] std::uint16_t range_dps() const override { return 500; }

    [[nodiscard]] double roll_deg() const { return _roll_rad * 180.0 / M_PI; }

    float rollRate_dps = 0.0f;
    float dt_s         = 0.002f;
    int   sampleCount  = 0;
    device::SensorHealth _health = device::SensorHealth::Ok;

private:
    double _roll_rad = 0.0;
    Vec3f  _accel{ 0.0f, 0.0f, 1.0f };
    Vec3f  _gyro{};
};

/// Barometer at a fixed altitude, with the pressure that implies.
class SimBarometer final : public device::Sensor, public device::Barometer
{
public:
    Status probe() override { return Status::Ok; }
    Status begin() override { return Status::Ok; }
    Status sample() override { ++sampleCount; return Status::Ok; }

    [[nodiscard]] std::uint16_t nativeRate_hz() const override { return 25; }
    [[nodiscard]] const char* name() const override { return "SimBaro"; }
    [[nodiscard]] device::SensorHealth health() const override { return device::SensorHealth::Ok; }

    Status read(device::BaroSample& out) const override
    {
        // Inverse of the barometric formula the altitude filter applies, so a
        // commanded altitude round-trips back through it.
        const double ratio = 1.0 - (altitude_m / 44330.0);
        out.pressure_pa = static_cast<float>(101325.0 * std::pow(ratio, 1.0 / 0.1903));
        return Status::Ok;
    }

    float altitude_m  = 0.0f;
    int   sampleCount = 0;
};

} // namespace arduflite::sim

#endif // ARDUFLITE_HOST_SIM_SENSORS_H
