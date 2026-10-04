/**
 * Mavlink.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The only include of the vendored MAVLink C library (ADR-066).
 *
 * Configures the library before it is seen, then includes the common dialect.
 *
 * - MAVLINK_ALIGNED_FIELDS 0: fields are packed byte by byte. The default
 *   writes them through casts to wider pointer types at unaligned offsets.
 * - MAVLINK_COMM_NUM_BUFFERS 1: every endpoint owns its parser and sequence
 *   state (mavlink_status_t members) and uses the *_status / *_buffer API, so
 *   the library's per-channel globals are never used.
 *
 * The library's own warnings are suppressed here so that the -Wall -Wextra
 * checks report on ArduFlite code only.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_H
#define ARDUFLITE_TELEMETRY_MAVLINK_H

#define MAVLINK_ALIGNED_FIELDS   0
#define MAVLINK_COMM_NUM_BUFFERS 1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wfloat-conversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "src/third_party/mavlink/common/mavlink.h"
#pragma GCC diagnostic pop

#endif // ARDUFLITE_TELEMETRY_MAVLINK_H
