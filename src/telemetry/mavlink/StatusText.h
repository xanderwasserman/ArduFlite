/**
 * StatusText.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Log lines delivered to ground stations as MAVLink STATUSTEXT (ADR-067).
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_STATUS_TEXT_H
#define ARDUFLITE_TELEMETRY_MAVLINK_STATUS_TEXT_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "src/hal/platform/Mutex.h"
#include "src/utils/Logging.h"

namespace arduflite::mavlink {

/**
 * @brief Lines waiting to be sent by one endpoint.
 *
 * Any task may push; the endpoint pops. Pushing never blocks: a line that
 * cannot be queued immediately — queue full, or the lock busy — is dropped and
 * counted, because the caller may be a flight-critical task that just logged.
 */
class StatusTextQueue
{
public:
    static constexpr std::size_t kCapacity = 8;
    static constexpr std::size_t kMaxText  = 150;   ///< three STATUSTEXT chunks

    struct Entry
    {
        std::uint8_t severity = 0;                   ///< MAV_SEVERITY
        char         text[kMaxText + 1]{};
    };

    void setMutex(hal::Mutex& mutex) noexcept { _mutex = &mutex; }

    void push(std::uint8_t severity, const char* text) noexcept;
    [[nodiscard]] bool pop(Entry& out) noexcept;

    [[nodiscard]] std::uint32_t dropped() const noexcept
    {
        return _dropped.load(std::memory_order_relaxed);
    }

private:
    hal::Mutex*                   _mutex = nullptr;
    std::array<Entry, kCapacity>  _entries{};
    std::size_t                   _head  = 0;
    std::size_t                   _count = 0;
    std::atomic<std::uint32_t>    _dropped{ 0 };
};

/**
 * @brief The logger's handler while MAVLink may be running.
 *
 * Passes every line to the console handler it wraps, and copies it to each
 * attached queue whose minimum level it meets. When the USB console is handed
 * to MAVLink, releaseConsole() stops the text output so it cannot corrupt the
 * binary stream; from then on logs reach the ground only as STATUSTEXT.
 */
class MavlinkLogRouter final : public LogHandler
{
public:
    enum class Port : std::uint8_t { Usb, Radio };

    explicit MavlinkLogRouter(LogHandler* console) noexcept : _console(console) {}

    void attach(Port port, StatusTextQueue& queue, LogLevel minimum) noexcept;
    void releaseConsole() noexcept { _console.store(nullptr, std::memory_order_release); }

    void log(LogLevel level, const char* fmt, va_list args) override;
    void log_nl(LogLevel level, const char* fmt, va_list args) override;

private:
    struct Sink
    {
        std::atomic<StatusTextQueue*> queue{ nullptr };
        std::atomic<LogLevel>         minimum{ LogLevel::Off };
    };

    void copyToQueues(LogLevel level, const char* fmt, va_list args) noexcept;

    std::atomic<LogHandler*> _console;
    std::array<Sink, 2>      _sinks{};
};

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_STATUS_TEXT_H
