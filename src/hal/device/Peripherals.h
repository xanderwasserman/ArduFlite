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
#include "src/hal/platform/ByteStream.h"
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

/**
 * @brief Numbered append-only session storage. Flight logs.
 *
 * @note Sessions are identified by an INDEX chosen by the caller, not by a name
 *       or a prefix the store invents. The interface originally had
 *       `startSession(const char* prefix)` returning an allocated index, which
 *       put the allocation rule inside the store — and that rule (monotonic,
 *       then lowest-gap; see core/LogRotationPolicy.h) is pure logic worth
 *       testing without a filesystem. Keeping it out leaves the store dumb
 *       enough that an in-memory implementation is a faithful stand-in.
 *
 * Writes are append-only within a session, and exactly one session may be open
 * at a time.
 */
class LogStore : private NonCopyable
{
public:
    virtual ~LogStore() = default;

    /// Mount, formatting if the mount fails. Safe to call twice.
    virtual Status begin() = 0;

    /// @param out receives the indices in use, unordered.
    /// @return how many were written, capped at maxEntries.
    virtual std::size_t listSessions(std::uint16_t* out, std::size_t maxEntries) = 0;

    /// Create or truncate the session with this index and open it for writing.
    virtual Status openSession(std::uint16_t index) = 0;

    /// Append to the open session. Returns NotPresent if none is open.
    virtual Status append(const char* data, std::size_t len) = 0;

    /// Push buffered bytes to the medium. Called on a cadence, not per row.
    virtual Status flush() = 0;

    virtual Status closeSession() = 0;
    [[nodiscard]] virtual bool isOpen() const = 0;

    /**
     * @param offset byte offset to read from
     * @param outLen receives the byte count actually read; 0 at end of data
     *
     * @note The offset exists because a flight log is far too large to read
     *       whole — dumping one to the console streams it in buffer-sized
     *       chunks. An offset-less version compiled fine and made dumping
     *       impossible.
     */
    virtual Status readSession(std::uint16_t index, void* dst, std::size_t maxLen,
                               std::size_t offset, std::size_t& outLen) = 0;

    /// Byte length of a stored session. NotPresent if it does not exist.
    virtual Status sessionSize(std::uint16_t index, std::uint32_t& bytes) = 0;

    virtual Status removeSession(std::uint16_t index) = 0;

    /// Bytes used and total capacity. Both zero if unknown.
    virtual Status usage(std::uint32_t& used, std::uint32_t& total) = 0;

    virtual Status formatAll() = 0;
};

/// Calibration blobs, CRC-protected.
class SettingsStore : private NonCopyable
{
public:
    virtual ~SettingsStore() = default;

    /// Returns Status::Corrupt on a CRC mismatch, NotPresent if never written.
    virtual Status load (const char* key, void* dst, std::size_t len) = 0;
    virtual Status save (const char* key, const void* src, std::size_t len) = 0;
    virtual Status erase(const char* key) = 0;
};

/// The diagnostic console: log output and the CLI's input. A ByteStream, so
/// the same port can be handed to a binary protocol (`mavlink on`).
class Console : public hal::ByteStream
{
public:
    using hal::ByteStream::write;

    std::size_t write(const char* s, std::size_t len)
    {
        return write(reinterpret_cast<const std::uint8_t*>(s), len);
    }

    /// One byte, or -1 if none are ready. Never blocks.
    [[nodiscard]] int readByte()
    {
        std::uint8_t byte = 0;
        return (read(&byte, 1) == 1) ? byte : -1;
    }

    /**
     * @brief Push buffered output to the wire.
     *
     * On the C3 this port is USB CDC and writes are buffered, so a watchdog
     * reset or a panic can discard whatever has not drained — including the
     * lines that explain the reset. Logging flushes after every Error for
     * exactly that reason.
     */
    virtual void flushOutput() = 0;

    /**
     * @note Byte-level, not line-level. Line assembly carries echo and
     *       backspace policy with it, and that belongs to the consumer: the CLI
     *       echoes, the telemetry drain does not.
     */
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_PERIPHERALS_H
