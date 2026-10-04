/**
 * FrameParser.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/FrameParser.h"

namespace arduflite::mavlink {

const mavlink_message_t* FrameParser::feed(std::uint8_t byte) noexcept
{
    mavlink_status_t report;
    switch (mavlink_frame_char_buffer(&_buffer, &_status, byte, &_frame, &report))
    {
        case MAVLINK_FRAMING_OK:
            return &_frame;
        case MAVLINK_FRAMING_BAD_CRC:
        case MAVLINK_FRAMING_BAD_SIGNATURE:
            ++_badFrames;
            resynchronise(byte);
            return nullptr;
        default:
            return nullptr;
    }
}

/// After a bad frame the parser must be reset by hand, as mavlink_parse_char()
/// does for the channel-based API. A failing byte that is itself a start marker
/// begins the next frame.
void FrameParser::resynchronise(std::uint8_t byte) noexcept
{
    _status.msg_received = MAVLINK_FRAMING_INCOMPLETE;
    _status.parse_state  = MAVLINK_PARSE_STATE_IDLE;
    if (byte == MAVLINK_STX)
    {
        _status.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        _buffer.len = 0;
        mavlink_start_checksum(&_buffer);
    }
}

} // namespace arduflite::mavlink
