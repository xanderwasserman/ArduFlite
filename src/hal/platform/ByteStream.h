/**
 * ByteStream.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_HAL_PLATFORM_BYTESTREAM_H
#define ARDUFLITE_HAL_PLATFORM_BYTESTREAM_H

#include <cstddef>
#include <cstdint>

#include "src/hal/core/NonCopyable.h"

namespace arduflite::hal {

/**
 * @brief A bidirectional byte stream: a UART, or the USB console.
 *
 * Reads never block. write() may block when asked for more than writable()
 * reports, so a caller that must not stall — a telemetry task on a slow radio
 * link — checks writable() first and skips rather than waits (ADR-065).
 */
class ByteStream : private NonCopyable
{
public:
    virtual ~ByteStream() = default;

    /// Bytes received and ready to read.
    [[nodiscard]] virtual std::size_t available() = 0;

    /// Copy up to @p maxLen received bytes into @p dst. Returns the count.
    [[nodiscard]] virtual std::size_t read(std::uint8_t* dst, std::size_t maxLen) = 0;

    /// Bytes write() accepts right now without blocking.
    [[nodiscard]] virtual std::size_t writable() = 0;

    /// Queue @p len bytes for transmission. Returns the count queued.
    virtual std::size_t write(const std::uint8_t* src, std::size_t len) = 0;
};

} // namespace arduflite::hal

#endif // ARDUFLITE_HAL_PLATFORM_BYTESTREAM_H
