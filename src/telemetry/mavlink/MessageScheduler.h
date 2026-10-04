/**
 * MessageScheduler.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_MESSAGE_SCHEDULER_H
#define ARDUFLITE_TELEMETRY_MAVLINK_MESSAGE_SCHEDULER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace arduflite::mavlink {

/// The periodic telemetry an endpoint sends, highest priority first.
enum class Stream : std::uint8_t
{
    Heartbeat,
    SysStatus,
    Attitude,
    VfrHud,
    ScaledImu,
    Values,      ///< NAMED_VALUE_FLOAT group: setpoints, magnetometer, link quality
    Count
};

inline constexpr std::size_t kStreamCount = static_cast<std::size_t>(Stream::Count);

[[nodiscard]] constexpr std::size_t streamIndex(Stream stream) noexcept
{
    return static_cast<std::size_t>(stream);
}

/// Milliseconds between sends of each stream, indexed by streamIndex(); 0 is off.
using StreamIntervals = std::array<std::uint32_t, kStreamCount>;

/**
 * @brief When each stream is due, and how many bytes a port may send.
 *
 * The byte budget is a token bucket. It refills at the configured rate and
 * holds at least one maximum-size frame, so every message can eventually be
 * sent however low the rate is set.
 */
class MessageScheduler
{
public:
    /// @param budgetBitsPerSecond 0 for no budget: the port's own write space
    ///                            is then the only limit.
    void configure(const StreamIntervals& intervals, std::uint32_t budgetBitsPerSecond) noexcept;

    void setInterval(Stream stream, std::uint32_t intervalMs) noexcept;
    [[nodiscard]] std::uint32_t defaultInterval(Stream stream) const noexcept;

    /// Send @p stream at the next opportunity, whatever its interval.
    void requestNow(Stream stream) noexcept;

    /// The highest-priority stream that is due, if any.
    [[nodiscard]] std::optional<Stream> nextDue(std::uint32_t nowMs) const noexcept;
    void markSent(Stream stream, std::uint32_t nowMs) noexcept;

    /// Take @p bytes from the budget if it holds that many.
    [[nodiscard]] bool trySpend(std::size_t bytes, std::uint32_t nowMs) noexcept;

private:
    void refill(std::uint32_t nowMs) noexcept;

    StreamIntervals                   _defaults{};
    StreamIntervals                   _intervals{};
    std::array<std::uint32_t, kStreamCount> _lastSentMs{};
    std::array<bool, kStreamCount>    _requested{};
    std::array<bool, kStreamCount>    _everSent{};

    std::uint32_t _bytesPerSecond = 0;   ///< 0: unlimited
    std::uint32_t _capacityMilli  = 0;   ///< bucket size, in thousandths of a byte
    std::uint32_t _tokensMilli    = 0;
    std::uint32_t _lastRefillMs   = 0;
};

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_MESSAGE_SCHEDULER_H
