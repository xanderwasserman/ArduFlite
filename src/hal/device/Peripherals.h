/**
 * Peripherals.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief GNSS, airspeed, rangefinder, power, indicator, storage and console.
 */
#ifndef ARDUFLITE_HAL_DEVICE_PERIPHERALS_H
#define ARDUFLITE_HAL_DEVICE_PERIPHERALS_H

#include <cstddef>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Result.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Storage.h"

namespace arduflite::device {

// ── GNSS ────────────────────────────────────────────────────────────────────

struct GnssFix
{
    enum class Type : std::uint8_t { None, Dead, Fix2D, Fix3D, Dgps, Rtk };

    double                 latitude_deg  = 0.0;   ///< double: 1e-7 deg needs it
    double                 longitude_deg = 0.0;
    float                  altitudeMsl_m = 0.0f;
    float                  groundSpeed_mps = 0.0f;
    float                  courseOverGround_deg = 0.0f;
    float                  hdop = 0.0f;
    float                  vdop = 0.0f;
    std::uint8_t           satellites = 0;
    Type                   type = Type::None;
    hal::Clock::time_point time{};
};

/**
 * @brief A fix is an EVENT, not a continuously available quantity — which is the
 *        one justified deviation from the sample()/read() idiom. The driver still
 *        implements Sensor; sample() drains the UART and parses.
 */
class Gnss : private NonCopyable
{
public:
    virtual ~Gnss() = default;
    /// True if newer than the last call.
    [[nodiscard]] virtual bool readFix(GnssFix& out) const = 0;
};

// ── Air data, range, power ──────────────────────────────────────────────────

struct AirspeedSample { float differential_pa = 0.0f; float indicated_mps = 0.0f;
                        hal::Clock::time_point time{}; };
struct RangeSample    { float distance_m = 0.0f; std::uint8_t quality_pct = 0;
                        hal::Clock::time_point time{}; };
struct PowerSample    { float voltage_v = 0.0f; float current_a = 0.0f;
                        float consumed_mah = 0.0f; hal::Clock::time_point time{}; };

class Airspeed : private NonCopyable
{
public:
    virtual ~Airspeed() = default;
    virtual Status read(AirspeedSample& out) const = 0;
};

class RangeFinder : private NonCopyable
{
public:
    virtual ~RangeFinder() = default;
    virtual Status read(RangeSample& out) const = 0;
    [[nodiscard]] virtual float maxRange_m() const = 0;
};

class PowerMonitor : private NonCopyable
{
public:
    virtual ~PowerMonitor() = default;
    virtual Status read(PowerSample& out) const = 0;
};

// ── Indicator ───────────────────────────────────────────────────────────────

struct Rgb { std::uint8_t r = 0, g = 0, b = 0; };

struct BlinkPattern
{
    Rgb           colour{};
    std::uint16_t onMs    = 200;
    std::uint16_t offMs   = 200;
    std::uint8_t  repeats = 0;   ///< 0 = forever
};

class Indicator : private NonCopyable
{
public:
    virtual ~Indicator() = default;
    virtual Status begin() = 0;
    virtual void   setColour(Rgb c) = 0;
    virtual void   setPattern(const BlinkPattern& p) = 0;
    virtual void   off() = 0;
};

// ── Storage and console ─────────────────────────────────────────────────────

class LogStore : private NonCopyable
{
public:
    virtual ~LogStore() = default;

    virtual Status begin() = 0;
    virtual Result<std::uint16_t> startSession(const char* prefix) = 0;
    virtual Status appendLine(const char* line, std::size_t len) = 0;
    virtual Status endSession() = 0;
    [[nodiscard]] virtual bool isRecording() const = 0;

    virtual std::size_t listSessions(hal::FileInfo* out, std::size_t maxEntries) = 0;
    virtual Status readSession(std::uint16_t index, void* dst, std::size_t maxLen,
                               std::size_t& outLen) = 0;
    virtual Status removeSession(std::uint16_t index) = 0;
    virtual Status usage(std::uint32_t& used, std::uint32_t& total) = 0;
    virtual Status formatAll() = 0;
};

/// Calibration blobs. Replaces raw EEPROM, and adds a CRC.
class SettingsStore : private NonCopyable
{
public:
    virtual ~SettingsStore() = default;

    /// Returns Status::Corrupt on a CRC mismatch, NotPresent if never written.
    virtual Status load (const char* key, void* dst, std::size_t len) = 0;
    virtual Status save (const char* key, const void* src, std::size_t len) = 0;
    virtual Status erase(const char* key) = 0;
};

class Console : private NonCopyable
{
public:
    virtual ~Console() = default;

    virtual std::size_t write(const char* s, std::size_t len) = 0;
    /// Returns 0 if no complete line is available. Never blocks.
    [[nodiscard]] virtual std::size_t readLine(char* dst, std::size_t maxLen) = 0;
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_PERIPHERALS_H
