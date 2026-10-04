/**
 * CrsfProtocol.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief CRSF wire format: frame type IDs and payload layouts. Nothing else.
 *
 * Separate from CrsfParser.h because CRSF has TWO implementations in this
 * codebase that must agree byte for byte — the receive-side parser in
 * drivers::CrsfParser, and the telemetry encoder in src/telemetry/crsf/ — and
 * only one of them is a driver. Before this split the telemetry adapter reached
 * into the driver header for the link-statistics struct, which the layering
 * check correctly flagged.
 *
 * @note This header contains NO hardware access: no bus, no pin, no register,
 *       no I/O of any kind. That is what makes it safe to include from any
 *       layer, and it is the condition tools/ci/check_layering.sh relies on.
 *       Keep it that way — if something here needs a bus, it belongs in a
 *       driver instead.
 */
#ifndef ARDUFLITE_HAL_PROTOCOL_CRSF_PROTOCOL_H
#define ARDUFLITE_HAL_PROTOCOL_CRSF_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace arduflite::drivers {

/// Inbound frame types this parser cares about.
inline constexpr std::uint8_t kCrsfFrameRcChannelsPacked = 0x16;
inline constexpr std::uint8_t kCrsfFrameLinkStatistics   = 0x14;

/// Link Statistics payload, 10 bytes. CRSF spec Rev07.
/// @note This is a WIRE FORMAT: the telemetry backend memcpy's it onto the UART.
///       Every member is one byte so no padding is possible, but the
///       static_assert below makes that a guarantee rather than an assumption —
///       these are serialised byte by byte, never memcpy'd as a struct.
struct CrsfLinkStatistics
{
    std::uint8_t uplinkRssi1        = 0;   ///< dBm + 64
    std::uint8_t uplinkRssi2        = 0;
    std::uint8_t uplinkLinkQuality  = 0;   ///< 0-255 %
    std::int8_t  uplinkSnr          = 0;
    std::uint8_t activeAntenna      = 0;
    std::uint8_t rfMode             = 0;
    std::uint8_t uplinkTxPower      = 0;
    std::uint8_t downlinkRssi       = 0;
    std::uint8_t downlinkLinkQuality = 0;
    std::int8_t  downlinkSnr        = 0;
};

static_assert(sizeof(CrsfLinkStatistics) == 10,
              "CRSF link-statistics frame is 10 bytes on the wire");

} // namespace arduflite::drivers

#endif // ARDUFLITE_HAL_PROTOCOL_CRSF_PROTOCOL_H
