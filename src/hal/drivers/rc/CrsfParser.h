/**
 * CrsfParser.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief CRSF frame parser — pure byte-in, state-out. No UART, no FreeRTOS.
 *
 * Keeping framing, CRC and 11-bit channel unpacking free of the UART is what
 * lets them be tested against captured bytes on a host.
 */
#ifndef ARDUFLITE_HAL_DRIVERS_RC_CRSFPARSER_H
#define ARDUFLITE_HAL_DRIVERS_RC_CRSFPARSER_H

#include <cstddef>
#include <cstdint>

#include "src/hal/protocol/CrsfProtocol.h"

namespace arduflite::drivers {

/**
 * @brief Byte-at-a-time CRSF framer.
 *
 * feed() returns what the byte completed, so the caller can react without the
 * parser knowing anything about tasks, timeouts or channel semantics.
 */
class CrsfParser
{
public:
    enum class Event : std::uint8_t
    {
        None,           ///< Frame still building, or byte discarded during resync
        RcChannels,     ///< channels() now holds 16 fresh 11-bit values
        LinkStatistics, ///< linkStats() now holds fresh metrics
        OtherFrame,     ///< Valid frame, type we do not decode
        CrcError,       ///< Complete frame, CRC mismatch — frame dropped
    };

    static constexpr std::uint8_t  kChannels  = 16;
    static constexpr std::uint8_t  kDestFc    = 0xC8;
    static constexpr std::size_t   kMaxFrame  = 64;

    Event feed(std::uint8_t b);

    [[nodiscard]] const std::uint16_t* channels() const noexcept { return _channels; }
    [[nodiscard]] const CrsfLinkStatistics& linkStats() const noexcept { return _stats; }

    /// Diagnostics — a rising crcErrors is the first sign of a marginal link.
    [[nodiscard]] std::uint32_t crcErrors()   const noexcept { return _crcErrors; }
    [[nodiscard]] std::uint32_t framesParsed() const noexcept { return _framesParsed; }

    void reset() noexcept { _bufLen = 0; _expectedLen = 0; }

    /// CRC8, polynomial 0xD5. Public so tests can build valid frames.
    [[nodiscard]] static std::uint8_t crc8(const std::uint8_t* data, std::uint8_t len) noexcept;

private:
    void decodeChannels(const std::uint8_t* p) noexcept;

    std::uint8_t  _buf[kMaxFrame]{};
    std::size_t   _bufLen      = 0;
    std::size_t   _expectedLen = 0;

    std::uint16_t      _channels[kChannels]{};
    CrsfLinkStatistics _stats{};

    std::uint32_t _crcErrors    = 0;
    std::uint32_t _framesParsed = 0;
};

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_DRIVERS_RC_CRSFPARSER_H
