/**
 * RcMapper.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/input/RcMapper.h"

namespace arduflite::input {

void RcMapper::configure(std::uint8_t idx, const ChannelMap& map)
{
    if (idx < kChannels) { _maps[idx] = map; }
}

float RcMapper::shape(const ChannelMap& map, std::uint16_t us) noexcept
{
    switch (map.shape)
    {
        case ChannelShape::Unused:
            return 0.0f;

        case ChannelShape::DualThrow:
            return toBipolar(us);

        case ChannelShape::SingleThrow:
            return toUnipolar(us);

        case ChannelShape::Boolean:
            // Above centre, i.e. raw > 1024.
            return (us > 1500) ? 1.0f : 0.0f;

        case ChannelShape::TriState:
        {
            const float n = toUnipolar(us);
            if (n < map.thrLow)  { return -1.0f; }
            if (n > map.thrHigh) { return  1.0f; }
            return 0.0f;
        }
    }
    return 0.0f;
}

void RcMapper::apply(const device::RcFrame& frame)
{
    const std::uint8_t n = (frame.channelCount < kChannels) ? frame.channelCount : kChannels;

    for (std::uint8_t i = 0; i < n; ++i)
    {
        const std::uint16_t us = frame.channel_us[i];

        // Fire on CHANGE only. The first
        // frame after boot seeds without firing, so a switch already in the
        // "on" position at power-up does not look like a fresh toggle.
        const bool changed = !_seeded || (us != _lastUs[i]);
        _lastUs[i] = us;

        if (!changed) { continue; }
        if (_maps[i].shape == ChannelShape::Unused) { continue; }
        if (_maps[i].callback == nullptr)           { continue; }
        if (!_seeded)                               { continue; }

        _maps[i].callback(i, shape(_maps[i], us));
    }

    _seeded = true;
}

} // namespace arduflite::input
