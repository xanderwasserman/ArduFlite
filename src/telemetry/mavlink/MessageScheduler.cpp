/**
 * MessageScheduler.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/MessageScheduler.h"

#include <algorithm>

namespace arduflite::mavlink {

namespace {

/// A MAVLink 2 frame is at most 280 bytes (unsigned).
constexpr std::uint32_t kMaxFrameBytes = 280;

/// The bucket holds a quarter-second of budget, but never less than one frame.
constexpr std::uint32_t kBurstDivisor = 4;

/// Refill no more than one second at a time, so the arithmetic cannot overflow
/// after a long stall.
constexpr std::uint32_t kMaxRefillMs = 1000;

} // namespace

void MessageScheduler::configure(const StreamIntervals& intervals,
                                 std::uint32_t budgetBitsPerSecond) noexcept
{
    _defaults  = intervals;
    _intervals = intervals;

    _bytesPerSecond = budgetBitsPerSecond / 8;
    const std::uint32_t capacity = std::max(_bytesPerSecond / kBurstDivisor, kMaxFrameBytes);
    _capacityMilli = capacity * 1000;
    _tokensMilli   = _capacityMilli;
}

void MessageScheduler::setInterval(Stream stream, std::uint32_t intervalMs) noexcept
{
    _intervals[streamIndex(stream)] = intervalMs;
}

std::uint32_t MessageScheduler::defaultInterval(Stream stream) const noexcept
{
    return _defaults[streamIndex(stream)];
}

void MessageScheduler::requestNow(Stream stream) noexcept
{
    _requested[streamIndex(stream)] = true;
}

std::optional<Stream> MessageScheduler::nextDue(std::uint32_t nowMs) const noexcept
{
    for (std::size_t i = 0; i < kStreamCount; ++i)
    {
        if (_requested[i]) { return static_cast<Stream>(i); }

        const std::uint32_t intervalMs = _intervals[i];
        if (intervalMs == 0) { continue; }
        if (!_everSent[i] || nowMs - _lastSentMs[i] >= intervalMs)
        {
            return static_cast<Stream>(i);
        }
    }
    return std::nullopt;
}

void MessageScheduler::markSent(Stream stream, std::uint32_t nowMs) noexcept
{
    const std::size_t i = streamIndex(stream);
    _lastSentMs[i] = nowMs;
    _everSent[i]   = true;
    _requested[i]  = false;
}

bool MessageScheduler::trySpend(std::size_t bytes, std::uint32_t nowMs) noexcept
{
    if (_bytesPerSecond == 0) { return true; }

    refill(nowMs);
    const std::uint64_t cost = static_cast<std::uint64_t>(bytes) * 1000;
    if (cost > _tokensMilli) { return false; }

    _tokensMilli -= static_cast<std::uint32_t>(cost);
    return true;
}

void MessageScheduler::refill(std::uint32_t nowMs) noexcept
{
    const std::uint32_t elapsedMs = std::min(nowMs - _lastRefillMs, kMaxRefillMs);
    _lastRefillMs = nowMs;

    const std::uint64_t topped =
        static_cast<std::uint64_t>(_tokensMilli) + static_cast<std::uint64_t>(elapsedMs) * _bytesPerSecond;
    _tokensMilli = static_cast<std::uint32_t>(std::min<std::uint64_t>(topped, _capacityMilli));
}

} // namespace arduflite::mavlink
