/**
 * CrsfParser.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/hal/drivers/rc/CrsfParser.h"

namespace arduflite::drivers {

std::uint8_t CrsfParser::crc8(const std::uint8_t* data, std::uint8_t len) noexcept
{
    std::uint8_t crc = 0;
    while (len-- != 0)
    {
        crc ^= *data++;
        for (std::uint8_t i = 0; i < 8; ++i)
        {
            crc = (crc & 0x80) ? static_cast<std::uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<std::uint8_t>(crc << 1);
        }
    }
    return crc;
}

void CrsfParser::decodeChannels(const std::uint8_t* p) noexcept
{
    // 16 channels packed as 11 bits each, little-endian, spanning byte
    // boundaries.
    for (std::uint8_t i = 0; i < kChannels; ++i)
    {
        const std::uint32_t bit  = static_cast<std::uint32_t>(i) * 11u;
        const std::uint32_t byte = bit / 8u;
        const std::uint32_t off  = bit % 8u;

        std::uint32_t val = static_cast<std::uint32_t>(p[byte]) >> off;
        val |= static_cast<std::uint32_t>(p[byte + 1]) << (8u - off);
        if (off > 5u) { val |= static_cast<std::uint32_t>(p[byte + 2]) << (16u - off); }

        _channels[i] = static_cast<std::uint16_t>(val & 0x07FFu);
    }
}

CrsfParser::Event CrsfParser::feed(std::uint8_t b)
{
    if (_bufLen == 0)
    {
        // Resync: discard anything that is not a frame start.
        if (b != kDestFc) { return Event::None; }
        _buf[0] = b;
        _bufLen = 1;
        return Event::None;
    }

    _buf[_bufLen++] = b;

    if (_bufLen == 2)
    {
        _expectedLen = static_cast<std::size_t>(b) + 2u;
        // Bounds BOTH ways: too long overruns _buf, too short underflows the
        // crc8 length calculation below.
        if (_expectedLen > kMaxFrame) { _bufLen = 0; return Event::None; }
        if (_expectedLen < 4)         { _bufLen = 0; return Event::None; }
        return Event::None;
    }

    if (_bufLen < _expectedLen) { return Event::None; }

    const std::uint8_t crcReceived = _buf[_expectedLen - 1];
    const std::uint8_t crcComputed =
        crc8(_buf + 2, static_cast<std::uint8_t>(_expectedLen - 3));

    _bufLen = 0;

    if (crcComputed != crcReceived)
    {
        ++_crcErrors;
        return Event::CrcError;
    }

    ++_framesParsed;

    const std::uint8_t  type       = _buf[2];
    const std::size_t   payloadLen = _expectedLen - 4;
    const std::uint8_t* payload    = _buf + 3;

    if (type == kCrsfFrameRcChannelsPacked)
    {
        // 16 x 11 bits = 22 bytes. A short frame would read past the payload.
        if (payloadLen < 22) { return Event::OtherFrame; }
        decodeChannels(payload);
        return Event::RcChannels;
    }

    if (type == kCrsfFrameLinkStatistics)
    {
        if (payloadLen < 10) { return Event::OtherFrame; }
        _stats.uplinkRssi1         = payload[0];
        _stats.uplinkRssi2         = payload[1];
        _stats.uplinkLinkQuality   = payload[2];
        _stats.uplinkSnr           = static_cast<std::int8_t>(payload[3]);
        _stats.activeAntenna       = payload[4];
        _stats.rfMode              = payload[5];
        _stats.uplinkTxPower       = payload[6];
        _stats.downlinkRssi        = payload[7];
        _stats.downlinkLinkQuality = payload[8];
        _stats.downlinkSnr         = static_cast<std::int8_t>(payload[9]);
        return Event::LinkStatistics;
    }

    return Event::OtherFrame;
}

} // namespace arduflite::drivers
