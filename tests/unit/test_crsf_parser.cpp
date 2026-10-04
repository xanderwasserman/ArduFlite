/**
 * test_crsf_parser.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * CRSF framing, CRC and 11-bit channel unpacking are free of the UART and of
 * FreeRTOS, which is what lets these drive the real parser byte by byte against
 * captured frames.
 */
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "src/hal/drivers/rc/CrsfParser.h"

using arduflite::drivers::CrsfParser;
using Event = CrsfParser::Event;

namespace {

/// Build a well-formed CRSF frame: [addr][len][type][payload...][crc]
std::vector<std::uint8_t> makeFrame(std::uint8_t type,
                                    const std::vector<std::uint8_t>& payload)
{
    std::vector<std::uint8_t> f;
    f.push_back(CrsfParser::kDestFc);
    f.push_back(static_cast<std::uint8_t>(payload.size() + 2));  // type + payload + crc
    f.push_back(type);
    f.insert(f.end(), payload.begin(), payload.end());

    std::vector<std::uint8_t> crcOver;
    crcOver.push_back(type);
    crcOver.insert(crcOver.end(), payload.begin(), payload.end());
    f.push_back(CrsfParser::crc8(crcOver.data(), static_cast<std::uint8_t>(crcOver.size())));
    return f;
}

/// Pack 16 x 11-bit channel values the way a transmitter does.
std::vector<std::uint8_t> packChannels(const std::uint16_t ch[16])
{
    std::vector<std::uint8_t> out(22, 0);
    for (int i = 0; i < 16; ++i)
    {
        const std::uint32_t bit  = static_cast<std::uint32_t>(i) * 11u;
        const std::uint32_t byte = bit / 8u;
        const std::uint32_t off  = bit % 8u;
        const std::uint32_t v    = ch[i] & 0x07FFu;

        out[byte]     |= static_cast<std::uint8_t>((v << off) & 0xFF);
        out[byte + 1] |= static_cast<std::uint8_t>((v >> (8 - off)) & 0xFF);
        if (off > 5) { out[byte + 2] |= static_cast<std::uint8_t>((v >> (16 - off)) & 0xFF); }
    }
    return out;
}

/// Feed a whole frame, return the last non-None event.
Event feedAll(CrsfParser& p, const std::vector<std::uint8_t>& bytes)
{
    Event last = Event::None;
    for (const auto b : bytes)
    {
        const Event e = p.feed(b);
        if (e != Event::None) { last = e; }
    }
    return last;
}

} // namespace

// ── Channel decoding ────────────────────────────────────────────────────────

TEST(CrsfParser, DecodesAllSixteenChannels)
{
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = static_cast<std::uint16_t>(172 + i * 100); }

    CrsfParser p;
    ASSERT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))), Event::RcChannels);

    for (int i = 0; i < 16; ++i)
    {
        EXPECT_EQ(p.channels()[i], ch[i]) << "channel " << i;
    }
}

TEST(CrsfParser, HandlesTheFullElevenBitRange)
{
    // 172 / 992 / 1811 are the CRSF endpoints and centre.
    std::uint16_t ch[16] = { 172, 992, 1811, 0, 2047, 1, 2046, 992,
                             172, 992, 1811, 0, 2047, 1, 2046, 992 };
    CrsfParser p;
    ASSERT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))), Event::RcChannels);
    for (int i = 0; i < 16; ++i) { EXPECT_EQ(p.channels()[i], ch[i]) << "channel " << i; }
}

// ── Framing ─────────────────────────────────────────────────────────────────

TEST(CrsfParser, IgnoresGarbageBeforeAFrame)
{
    std::uint16_t ch[16] = {};
    for (int i = 0; i < 16; ++i) { ch[i] = 992; }

    CrsfParser p;
    for (const std::uint8_t junk : { 0x00, 0xFF, 0x42, 0x13, 0x37 })
    {
        EXPECT_EQ(p.feed(junk), Event::None);
    }
    EXPECT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))), Event::RcChannels)
        << "must resync after garbage";
}

TEST(CrsfParser, RecoversAfterATruncatedFrame)
{
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = static_cast<std::uint16_t>(500 + i); }

    const auto good = makeFrame(0x16, packChannels(ch));

    CrsfParser p;
    // Half a frame, then a stream resync and a whole one.
    for (std::size_t i = 0; i < good.size() / 2; ++i) { p.feed(good[i]); }
    p.reset();
    EXPECT_EQ(feedAll(p, good), Event::RcChannels);
}

TEST(CrsfParser, SurvivesAFrameSplitAcrossReads)
{
    // A UART read boundary can land anywhere; the parser is byte-at-a-time so
    // this must be a non-event.
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = static_cast<std::uint16_t>(300 + i * 7); }
    const auto frame = makeFrame(0x16, packChannels(ch));

    for (std::size_t split = 1; split < frame.size(); ++split)
    {
        CrsfParser p;
        Event last = Event::None;
        for (std::size_t i = 0; i < frame.size(); ++i)
        {
            const Event e = p.feed(frame[i]);
            if (e != Event::None) { last = e; }
        }
        EXPECT_EQ(last, Event::RcChannels) << "split at " << split;
    }
}

TEST(CrsfParser, RejectsOverlongLengthWithoutOverrunning)
{
    CrsfParser p;
    p.feed(CrsfParser::kDestFc);
    p.feed(0xFF);                       // claims 255 + 2 bytes, over kMaxFrame
    EXPECT_EQ(p.framesParsed(), 0u);

    // Parser must have reset, so a following good frame still works.
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = 992; }
    EXPECT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))), Event::RcChannels);
}

TEST(CrsfParser, RejectsUndersizedLengthThatWouldUnderflowCrc)
{
    // len < 4 would make the crc8 length calculation (_expectedLen - 3) wrap.
    CrsfParser p;
    p.feed(CrsfParser::kDestFc);
    p.feed(0x01);
    EXPECT_EQ(p.framesParsed(), 0u);
    EXPECT_EQ(p.crcErrors(), 0u);
}

// ── CRC ─────────────────────────────────────────────────────────────────────

TEST(CrsfParser, RejectsACorruptedFrameAndCountsIt)
{
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = 992; }

    auto frame = makeFrame(0x16, packChannels(ch));
    frame[5] ^= 0xFF;                   // corrupt a payload byte

    CrsfParser p;
    EXPECT_EQ(feedAll(p, frame), Event::CrcError);
    EXPECT_EQ(p.crcErrors(), 1u);
    EXPECT_EQ(p.framesParsed(), 0u);
}

TEST(CrsfParser, CorruptFrameDoesNotClobberTheLastGoodChannels)
{
    std::uint16_t good[16];
    for (int i = 0; i < 16; ++i) { good[i] = static_cast<std::uint16_t>(1000 + i); }

    CrsfParser p;
    ASSERT_EQ(feedAll(p, makeFrame(0x16, packChannels(good))), Event::RcChannels);

    std::uint16_t bad[16];
    for (int i = 0; i < 16; ++i) { bad[i] = 1; }
    auto corrupt = makeFrame(0x16, packChannels(bad));
    corrupt[4] ^= 0x55;
    EXPECT_EQ(feedAll(p, corrupt), Event::CrcError);

    for (int i = 0; i < 16; ++i)
    {
        EXPECT_EQ(p.channels()[i], good[i]) << "a bad frame must not corrupt good data";
    }
}

// ── Link statistics ─────────────────────────────────────────────────────────

TEST(CrsfParser, DecodesLinkStatistics)
{
    const std::vector<std::uint8_t> payload{
        200,            // uplink RSSI 1
        190,            // uplink RSSI 2
        87,             // uplink link quality %
        static_cast<std::uint8_t>(static_cast<std::int8_t>(-9)),   // SNR
        1, 4, 3,        // antenna, rf mode, tx power
        180, 95,        // downlink RSSI, LQ
        static_cast<std::uint8_t>(static_cast<std::int8_t>(-12)),
    };

    CrsfParser p;
    ASSERT_EQ(feedAll(p, makeFrame(0x14, payload)), Event::LinkStatistics);

    const auto& s = p.linkStats();
    EXPECT_EQ(s.uplinkLinkQuality,   87);
    EXPECT_EQ(s.uplinkSnr,           -9);
    EXPECT_EQ(s.downlinkLinkQuality, 95);
    EXPECT_EQ(s.downlinkSnr,         -12);
    EXPECT_EQ(s.rfMode,              4);
}

TEST(CrsfParser, ShortLinkStatisticsFrameIsRejectedNotRead)
{
    // A truncated frame must not read past its payload.
    CrsfParser p;
    EXPECT_EQ(feedAll(p, makeFrame(0x14, { 1, 2, 3 })), Event::OtherFrame);
    EXPECT_EQ(p.linkStats().uplinkLinkQuality, 0) << "stats must be untouched";
}

TEST(CrsfParser, ShortRcFrameIsRejectedNotRead)
{
    CrsfParser p;
    EXPECT_EQ(feedAll(p, makeFrame(0x16, std::vector<std::uint8_t>(10, 0xAA))),
              Event::OtherFrame);
}

TEST(CrsfParser, UnknownFrameTypeIsAcceptedButIgnored)
{
    CrsfParser p;
    EXPECT_EQ(feedAll(p, makeFrame(0x28, { 1, 2, 3, 4 })), Event::OtherFrame);
    EXPECT_EQ(p.framesParsed(), 1u);
    EXPECT_EQ(p.crcErrors(), 0u);
}

TEST(CrsfParser, InterleavedRcAndLinkStatsBothDecode)
{
    std::uint16_t ch[16];
    for (int i = 0; i < 16; ++i) { ch[i] = static_cast<std::uint16_t>(700 + i); }
    const std::vector<std::uint8_t> stats{ 200, 190, 55, 0, 0, 2, 1, 180, 60, 0 };

    CrsfParser p;
    EXPECT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))),  Event::RcChannels);
    EXPECT_EQ(feedAll(p, makeFrame(0x14, stats)),             Event::LinkStatistics);
    EXPECT_EQ(feedAll(p, makeFrame(0x16, packChannels(ch))),  Event::RcChannels);

    EXPECT_EQ(p.framesParsed(), 3u);
    EXPECT_EQ(p.linkStats().uplinkLinkQuality, 55);
    EXPECT_EQ(p.channels()[0], ch[0]);
}
