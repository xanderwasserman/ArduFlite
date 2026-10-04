/**
 * FrameParser.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_FRAME_PARSER_H
#define ARDUFLITE_TELEMETRY_MAVLINK_FRAME_PARSER_H

#include <cstdint>

#include "src/telemetry/mavlink/Mavlink.h"

namespace arduflite::mavlink {

/**
 * @brief MAVLink framing for one byte stream, MAVLink 1 or 2.
 *
 * Owns its parser state, so any number can run side by side without the
 * library's per-channel globals. Bytes that do not form a frame with a valid
 * checksum are skipped; a bad frame is counted and parsing resumes at the next
 * start marker.
 */
class FrameParser
{
public:
    /// The frame @p byte completed, or nullptr. Valid until the next feed().
    [[nodiscard]] const mavlink_message_t* feed(std::uint8_t byte) noexcept;

    [[nodiscard]] std::uint32_t badFrames() const noexcept { return _badFrames; }

private:
    void resynchronise(std::uint8_t byte) noexcept;

    mavlink_message_t _buffer{};
    mavlink_message_t _frame{};
    mavlink_status_t  _status{};
    std::uint32_t     _badFrames = 0;
};

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_FRAME_PARSER_H
