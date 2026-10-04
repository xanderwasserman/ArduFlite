/**
 * test_crc32.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Host tests for the CRC-32 used to guard stored calibration.
 *
 * The value a CRC adds over a bare magic number is detecting a blob
 * that WAS written correctly and later decayed. The magic number only ever
 * answered "has anything been written here?", so a single flipped bit in a
 * stored offset passed the check and was applied as a calibration — biasing the
 * aircraft's idea of level with nothing in the log to say so.
 */
#include <gtest/gtest.h>

#include <array>
#include <cstring>

#include "src/hal/core/Crc32.h"

namespace {

/// Pinned against the standard IEEE 802.3 check value, so an implementation
/// change cannot silently redefine "correct" and still agree with itself.
TEST(Crc32, MatchesTheStandardCheckVector)
{
    const char* input = "123456789";
    EXPECT_EQ(arduflite::crc32(input, 9), 0xCBF43926u)
        << "the published check value for CRC-32/ISO-HDLC";
}

TEST(Crc32, EmptyInputIsZero)
{
    EXPECT_EQ(arduflite::crc32("", 0), 0u);
}

TEST(Crc32, IsComputableAtCompileTime)
{
    // constexpr matters here: it means the CRC of a fixed schema can be a
    // compile-time constant rather than a startup cost.
    static constexpr std::uint8_t kData[] = { 'a', 'r', 'd', 'u', 'f', 'l', 'i', 't', 'e' };
    static constexpr std::uint32_t kCrc = arduflite::crc32(kData, sizeof(kData));
    static_assert(kCrc != 0, "constexpr evaluation failed");
    EXPECT_EQ(arduflite::crc32(kData, sizeof(kData)), kCrc);
}

/// The case the magic number could not catch.
TEST(Crc32, DetectsASingleFlippedBit)
{
    struct Offsets { float accelX, accelY, accelZ, gyroX, gyroY, gyroZ; };
    Offsets good{ 0.01f, -0.02f, 0.03f, 0.4f, -0.5f, 0.6f };

    const std::uint32_t reference = arduflite::crc32(&good, sizeof(good));

    // Flip one bit in the middle of the blob, as flash decay would.
    std::array<std::uint8_t, sizeof(Offsets)> bytes{};
    std::memcpy(bytes.data(), &good, sizeof(good));
    bytes[10] ^= 0x01;

    EXPECT_NE(arduflite::crc32(bytes.data(), bytes.size()), reference);
}

TEST(Crc32, DetectsTruncationAndReordering)
{
    const std::array<std::uint8_t, 4> original{ 1, 2, 3, 4 };
    const std::array<std::uint8_t, 4> swapped{ 2, 1, 3, 4 };

    const std::uint32_t reference = arduflite::crc32(original.data(), original.size());

    EXPECT_NE(arduflite::crc32(swapped.data(), swapped.size()), reference)
        << "order must matter, or a byte-swapped struct would validate";
    EXPECT_NE(arduflite::crc32(original.data(), original.size() - 1), reference)
        << "a short read must not validate against the full-length CRC";
}

TEST(Crc32, AllZeroBlobHasANonZeroChecksum)
{
    // Erased flash reads as zeros or as 0xFF. Neither must produce a checksum
    // that happens to match the zeros stored next to it.
    const std::array<std::uint8_t, 24> zeros{};
    std::array<std::uint8_t, 24> ones{};
    ones.fill(0xFF);

    EXPECT_NE(arduflite::crc32(zeros.data(), zeros.size()), 0u);
    EXPECT_NE(arduflite::crc32(ones.data(), ones.size()), 0u);
    EXPECT_NE(arduflite::crc32(zeros.data(), zeros.size()),
              arduflite::crc32(ones.data(), ones.size()));
}

} // namespace
