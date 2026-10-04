/**
 * Crc32.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief CRC-32 (IEEE 802.3), computed bitwise.
 *
 * No lookup table: the 1 KiB a table costs buys speed this project never needs.
 * Every use is a settings blob written or read once at boot, tens of bytes long.
 */
#ifndef ARDUFLITE_HAL_CORE_CRC32_H
#define ARDUFLITE_HAL_CORE_CRC32_H

#include <cstddef>
#include <cstdint>

namespace arduflite {

/**
 * @brief CRC over a byte range.
 *
 * Takes std::uint8_t rather than void so it can be evaluated at compile time —
 * a cast from const void* is not permitted in a constant expression. The void
 * overload below exists for callers holding an opaque buffer.
 */
[[nodiscard]] constexpr std::uint32_t crc32(const std::uint8_t* bytes, std::size_t length) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFu;

    for (std::size_t i = 0; i < length; ++i)
    {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            // 0xEDB88320 is the reversed form of the standard polynomial, which
            // is what pairs with this LSB-first shift.
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
        }
    }

    return ~crc;
}

/// Convenience overload for opaque buffers. Not usable in a constant expression.
[[nodiscard]] inline std::uint32_t crc32(const void* data, std::size_t length) noexcept
{
    return crc32(static_cast<const std::uint8_t*>(data), length);
}

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_CRC32_H
