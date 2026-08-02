/**
 * RcLink.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Pilot input, protocol-agnostic.
 *
 * Deliberately absent: ChannelConfig, ChannelType, ChannelCallback,
 * configureChannel(). A protocol decoder must not know that channel 5 is ARM —
 * that policy lives in input::RcMapper (ADR-006). Yielding microseconds is what
 * makes CRSF, PWM and SBUS genuinely interchangeable.
 */
#ifndef ARDUFLITE_HAL_DEVICE_RCLINK_H
#define ARDUFLITE_HAL_DEVICE_RCLINK_H

#include <chrono>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/platform/Clock.h"

namespace arduflite::device {

struct RcFrame
{
    static constexpr std::uint8_t kMaxChannels = 16;

    std::uint16_t          channel_us[kMaxChannels] = {};  ///< normalised 988..2012
    std::uint8_t           channelCount             = 0;
    hal::Clock::time_point time{};
};

struct RcLinkStats
{
    std::uint8_t linkQuality_pct = 0;
    std::int8_t  rssi_dbm        = 0;
    std::int8_t  snr_db          = 0;
    bool         valid           = false;   ///< false until the first stats frame
};

class RcLink : private NonCopyable
{
public:
    virtual ~RcLink() = default;

    virtual Status begin() = 0;

    /// True if a frame newer than the last call is available.
    [[nodiscard]] virtual bool readFrame(RcFrame& out) = 0;

    [[nodiscard]] virtual bool isFailsafe() const = 0;
    virtual void setFailsafeTimeout(std::chrono::milliseconds timeout) = 0;

    [[nodiscard]] virtual RcLinkStats stats() const = 0;
    [[nodiscard]] virtual const char* name() const = 0;   ///< "CRSF", "PWM"
};

} // namespace arduflite::device

#endif // ARDUFLITE_HAL_DEVICE_RCLINK_H
