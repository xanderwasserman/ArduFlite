/**
 * Sensor.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief One interface per MEASUREMENT, never per chip (ADR-019).
 *
 * A 6-DOF IMU is not a thing flight code knows about; it is a chip that happens
 * to implement Accelerometer AND Gyroscope. Discrete parts produce the same view
 * of the world, and redundant sensors are just more entries in a list.
 *
 * THREADING CONTRACT — read before using any of this:
 *
 *   sample() and read() may be called ONLY from the sampling task that owns the
 *   device. A driver's cache is unsynchronised state: sample() writes it, read()
 *   reads it, and there is no lock because a lock has no place in a 500 Hz path.
 *   Every other consumer reads the published SeqLock<ImuState>, never a driver.
 */
#ifndef ARDUFLITE_HAL_DEVICE_SENSOR_H
#define ARDUFLITE_HAL_DEVICE_SENSOR_H

#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/core/Vec3.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::device {

enum class SensorHealth : std::uint8_t { Unknown, Ok, Degraded, Failed, NotPresent };

constexpr const char* toString(SensorHealth h) noexcept
{
    switch (h)
    {
        case SensorHealth::Unknown:    return "Unknown";
        case SensorHealth::Ok:         return "Ok";
        case SensorHealth::Degraded:   return "Degraded";
        case SensorHealth::Failed:     return "Failed";
        case SensorHealth::NotPresent: return "NotPresent";
    }
    return "Unknown";
}

/**
 * @brief A physical chip on a bus. One per part, whatever it measures.
 */
class Sensor : private NonCopyable
{
public:
    virtual ~Sensor() = default;

    /// WHO_AM_I / ACK. Returns Status::NotPresent when the part is not fitted.
    virtual Status probe() = 0;
    virtual Status begin() = 0;

    /// Perform the bus transaction(s) refreshing EVERY reading this chip provides.
    /// The sampling task calls this once per device per tick.
    /// THIS IS THE ONLY METHOD THAT TOUCHES THE BUS.
    virtual Status sample() = 0;

    /// Drives decimation. Replaces BARO_DECIMATION_FACTOR and its mirrored
    /// constants: the sampling loop computes taskRate / nativeRate_hz().
    [[nodiscard]] virtual std::uint16_t nativeRate_hz() const = 0;

    [[nodiscard]] virtual const char*  name()   const = 0;   ///< "MPU-6500"
    [[nodiscard]] virtual SensorHealth health() const = 0;
};

// ── Measurement interfaces — independent, no common base ────────────────────
// read() returns the value cached by the last sample(). It never touches the bus.

struct AccelSample { Vec3f accel_g;     hal::Clock::time_point time{}; };
struct GyroSample  { Vec3f rate_dps;    hal::Clock::time_point time{}; };
struct MagSample   { Vec3f field_ut;    hal::Clock::time_point time{}; };
struct BaroSample  { float pressure_pa = 0.0f; float temp_c = 0.0f;
                     hal::Clock::time_point time{}; };
struct TempSample  { float temp_c = 0.0f; hal::Clock::time_point time{}; };

class Accelerometer : private NonCopyable
{
public:
    virtual ~Accelerometer() = default;
    virtual Status read(AccelSample& out) const = 0;
    virtual Status setRange_g(std::uint8_t g) = 0;
    [[nodiscard]] virtual std::uint8_t range_g() const = 0;
};

class Gyroscope : private NonCopyable
{
public:
    virtual ~Gyroscope() = default;
    virtual Status read(GyroSample& out) const = 0;
    virtual Status setRange_dps(std::uint16_t dps) = 0;
    [[nodiscard]] virtual std::uint16_t range_dps() const = 0;
};

class Magnetometer : private NonCopyable
{
public:
    virtual ~Magnetometer() = default;
    virtual Status read(MagSample& out) const = 0;
};

class Barometer : private NonCopyable
{
public:
    virtual ~Barometer() = default;
    virtual Status read(BaroSample& out) const = 0;
};

class Thermometer : private NonCopyable
{
public:
    virtual ~Thermometer() = default;
    virtual Status read(TempSample& out) const = 0;
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_SENSOR_H
