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

    /// Drives decimation. The rate is read from the part rather than
    /// constants: the sampling loop computes taskRate / nativeRate_hz().
    [[nodiscard]] virtual std::uint16_t nativeRate_hz() const = 0;

    [[nodiscard]] virtual const char*  name()   const = 0;   ///< "MPU-6500"
    [[nodiscard]] virtual SensorHealth health() const = 0;
};

// ── Measurement interfaces ──────────────────────────────────────────────────
// read() returns the value cached by the last sample(). It never touches the bus.

/**
 * @brief What every measurement interface has in common: whether to trust it.
 *
 * estimation::SensorSelector holds measurement interfaces and has to ask "is
 * this instance healthy?", so health has to live here rather than only on
 * Sensor — with `-fno-rtti` there is no cross-cast to reach the part.
 *
 * Health is genuinely per-measurement, not per-part. On an MPU-9250 the
 * magnetometer is a separate die behind an auxiliary bus and can fail while the
 * accelerometer and gyroscope keep working; a part-level answer would feed the
 * estimator a dead magnetometer.
 *
 * A part whose measurements share a fate implements this once — the single
 * override satisfies both this and Sensor::health().
 */
class Measurement
{
public:
    virtual ~Measurement() = default;
    [[nodiscard]] virtual SensorHealth health() const = 0;

    /**
     * @brief How often this measurement actually produces new data, in Hz.
     *
     * On the Measurement rather than only on Sensor for the same reason health
     * is: a consumer holding a Barometer* needs it, and cannot reach the part.
     *
     * Reading faster than this returns the SAME conversion again. That is not
     * merely wasteful — a consumer that differentiates the value (climb rate
     * from altitude) and assumes its own read interval will scale the result by
     * the ratio between the two rates.
     */
    [[nodiscard]] virtual std::uint16_t nativeRate_hz() const = 0;
};

struct AccelSample { Vec3f accel_g;     hal::Clock::time_point time{}; };
struct GyroSample  { Vec3f rate_dps;    hal::Clock::time_point time{}; };
struct MagSample   { Vec3f field_ut;    hal::Clock::time_point time{}; };
struct BaroSample  { float pressure_pa = 0.0f; float temp_c = 0.0f;
                     hal::Clock::time_point time{}; };
struct TempSample  { float temp_c = 0.0f; hal::Clock::time_point time{}; };

class Accelerometer : public Measurement, private NonCopyable
{
public:
    virtual ~Accelerometer() = default;
    virtual Status read(AccelSample& out) const = 0;
    virtual Status setRange_g(std::uint8_t g) = 0;
    [[nodiscard]] virtual std::uint8_t range_g() const = 0;
};

class Gyroscope : public Measurement, private NonCopyable
{
public:
    virtual ~Gyroscope() = default;
    virtual Status read(GyroSample& out) const = 0;
    virtual Status setRange_dps(std::uint16_t dps) = 0;
    [[nodiscard]] virtual std::uint16_t range_dps() const = 0;
};

class Magnetometer : public Measurement, private NonCopyable
{
public:
    virtual ~Magnetometer() = default;
    virtual Status read(MagSample& out) const = 0;
};

class Barometer : public Measurement, private NonCopyable
{
public:
    virtual ~Barometer() = default;
    virtual Status read(BaroSample& out) const = 0;
};

class Thermometer : public Measurement, private NonCopyable
{
public:
    virtual ~Thermometer() = default;
    virtual Status read(TempSample& out) const = 0;
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_SENSOR_H
