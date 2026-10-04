/**
 * StatusText.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/StatusText.h"

#include <cstdio>
#include <cstring>
#include <mutex>

#include "src/telemetry/mavlink/Mavlink.h"

namespace arduflite::mavlink {

namespace {

/// Plain output (LOG) ranks as Info, so a Warn-and-above port does not carry
/// the CLI's replies.
LogLevel effectiveLevel(LogLevel level) noexcept
{
    return (level == LogLevel::Clear) ? LogLevel::Info : level;
}

std::uint8_t severityOf(LogLevel level) noexcept
{
    switch (effectiveLevel(level))
    {
        case LogLevel::Debug: return MAV_SEVERITY_DEBUG;
        case LogLevel::Warn:  return MAV_SEVERITY_WARNING;
        case LogLevel::Error: return MAV_SEVERITY_ERROR;
        default:              return MAV_SEVERITY_INFO;
    }
}

} // namespace

void StatusTextQueue::push(std::uint8_t severity, const char* text) noexcept
{
    if (_mutex == nullptr) { return; }

    std::unique_lock lock(*_mutex, std::try_to_lock);
    if (!lock.owns_lock() || _count == kCapacity)
    {
        _dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    Entry& entry = _entries[(_head + _count) % kCapacity];
    entry.severity = severity;
    std::strncpy(entry.text, text, kMaxText);
    entry.text[kMaxText] = '\0';
    ++_count;
}

bool StatusTextQueue::pop(Entry& out) noexcept
{
    if (_mutex == nullptr) { return false; }

    std::unique_lock lock(*_mutex, std::try_to_lock);
    if (!lock.owns_lock() || _count == 0) { return false; }

    out   = _entries[_head];
    _head = (_head + 1) % kCapacity;
    --_count;
    return true;
}

void MavlinkLogRouter::attach(Port port, StatusTextQueue& queue, LogLevel minimum) noexcept
{
    Sink& sink = _sinks[static_cast<std::size_t>(port)];
    sink.minimum.store(minimum, std::memory_order_relaxed);
    sink.queue.store(&queue, std::memory_order_release);
}

void MavlinkLogRouter::log(LogLevel level, const char* fmt, va_list args)
{
    if (LogHandler* console = _console.load(std::memory_order_acquire))
    {
        va_list copy;
        va_copy(copy, args);
        console->log(level, fmt, copy);
        va_end(copy);
    }
    copyToQueues(level, fmt, args);
}

void MavlinkLogRouter::log_nl(LogLevel level, const char* fmt, va_list args)
{
    if (LogHandler* console = _console.load(std::memory_order_acquire))
    {
        va_list copy;
        va_copy(copy, args);
        console->log_nl(level, fmt, copy);
        va_end(copy);
    }
    copyToQueues(level, fmt, args);
}

void MavlinkLogRouter::copyToQueues(LogLevel level, const char* fmt, va_list args) noexcept
{
    const LogLevel effective = effectiveLevel(level);

    char text[StatusTextQueue::kMaxText + 1];
    bool formatted = false;

    for (Sink& sink : _sinks)
    {
        StatusTextQueue* queue = sink.queue.load(std::memory_order_acquire);
        if (queue == nullptr || effective < sink.minimum.load(std::memory_order_relaxed))
        {
            continue;
        }

        if (!formatted)
        {
            va_list copy;
            va_copy(copy, args);
            const int written = std::vsnprintf(text, sizeof(text), fmt, copy);
            va_end(copy);
            if (written <= 0) { return; }

            std::size_t length = std::strlen(text);
            while (length > 0 && (text[length - 1] == '\n' || text[length - 1] == '\r'))
            {
                text[--length] = '\0';
            }
            if (length == 0) { return; }
            formatted = true;
        }
        queue->push(severityOf(level), text);
    }
}

} // namespace arduflite::mavlink
