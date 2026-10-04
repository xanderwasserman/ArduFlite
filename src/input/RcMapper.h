/**
 * RcMapper.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Channel -> role mapping and stick shaping.
 *
 * Channel-to-role mapping and stick shaping, kept OUT of the CRSF driver
 * with embedded callbacks, so the protocol decoder called CRSFCallbacks::onArm
 * directly. A decoder must not know that channel 5 is ARM (ADR-006).
 */
#ifndef ARDUFLITE_INPUT_RCMAPPER_H
#define ARDUFLITE_INPUT_RCMAPPER_H

#include <cstdint>

#include "src/hal/device/RcLink.h"

namespace arduflite::input {

enum class ChannelShape : std::uint8_t
{
    Unused,
    DualThrow,    ///< -1 .. +1, centred
    SingleThrow,  ///<  0 .. +1
    Boolean,      ///<  0 or 1
    TriState,     ///< -1, 0, +1
};

using ChannelCallback = void (*)(std::uint8_t channelIdx, float value);

struct ChannelMap
{
    ChannelShape    shape    = ChannelShape::Unused;
    float           thrLow   = 0.33f;   ///< TriState only
    float           thrHigh  = 0.66f;   ///< TriState only
    ChannelCallback callback = nullptr; ///< invoked on CHANGE only
};

/**
 * @brief Turns an RcFrame into role callbacks.
 *
 * Microsecond in, normalised out. The shaping matches the
 * ArdufliteCRSFReceiver::applyMapping() exactly — see the test suite, which
 * pins the endpoints and centre.
 */
class RcMapper
{
public:
    static constexpr std::uint8_t kChannels = device::RcFrame::kMaxChannels;

    void configure(std::uint8_t idx, const ChannelMap& map);

    /// Apply a frame, firing callbacks for channels whose value changed.
    void apply(const device::RcFrame& frame);

    /// Shape one microsecond value. Static and pure, so it is trivially testable.
    [[nodiscard]] static float shape(const ChannelMap& map, std::uint16_t us) noexcept;

    /// us -> [-1, +1], the inverse of CrsfLink::rawToMicroseconds().
    [[nodiscard]] static constexpr float toBipolar(std::uint16_t us) noexcept
    {
        const float v = (static_cast<float>(us) - 1500.0f) / 500.0f;
        return (v < -1.0f) ? -1.0f : ((v > 1.0f) ? 1.0f : v);
    }

    /// us -> [0, 1].
    [[nodiscard]] static constexpr float toUnipolar(std::uint16_t us) noexcept
    {
        const float v = (static_cast<float>(us) - 1000.0f) / 1000.0f;
        return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
    }

private:
    ChannelMap    _maps[kChannels]{};
    std::uint16_t _lastUs[kChannels]{};
    bool          _seeded = false;
};

} // namespace arduflite::input

#endif // ARDUFLITE_INPUT_RCMAPPER_H
