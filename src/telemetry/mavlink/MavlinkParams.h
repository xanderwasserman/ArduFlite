/**
 * MavlinkParams.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief ConfigRegistry as the MAVLink parameter protocol sees it (ADR-069).
 *
 * Every numeric configuration key appears under a MAVLink name of at most 16
 * characters, in a fixed order that is also its parameter index. The name keeps
 * the key's unit suffix (rate.roll.ti_s is RATE_RLL_TI_S). String keys are not
 * exposed: MAVLink parameters are numeric, and one of them is the WiFi password.
 *
 * Values use the C-cast encoding (MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST):
 * an integer parameter travels as the float of its value.
 */
#ifndef ARDUFLITE_TELEMETRY_MAVLINK_PARAMS_H
#define ARDUFLITE_TELEMETRY_MAVLINK_PARAMS_H

#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace arduflite::mavlink {

/// Longest parameter name the protocol carries.
inline constexpr std::size_t kParamIdLength = 16;

struct ParamName
{
    const char* key;   ///< ConfigRegistry key
    const char* id;    ///< MAVLink parameter name
};

struct ParamValue
{
    char          id[kParamIdLength + 1]{};
    float         value = 0.0f;
    std::uint8_t  type  = 0;   ///< MAV_PARAM_TYPE
    std::uint16_t index = 0;
    std::uint16_t count = 0;
};

enum class ParamWrite : std::uint8_t
{
    Applied,
    Unknown,    ///< no parameter has that name
    Rejected,   ///< wrong type or out of range; the old value stands
};

/// The full table, in parameter-index order.
[[nodiscard]] std::span<const ParamName> paramNames() noexcept;

[[nodiscard]] std::uint16_t paramCount() noexcept;

[[nodiscard]] std::optional<ParamValue> readParam(std::uint16_t index);

/// @p id as received: up to 16 characters, not necessarily NUL-terminated.
[[nodiscard]] std::optional<ParamValue> readParam(std::string_view id);

/// Validated by ConfigRegistry exactly as a CLI `config set` is. Callers check
/// ground safety first.
[[nodiscard]] ParamWrite writeParam(std::string_view id, float value);

} // namespace arduflite::mavlink

#endif // ARDUFLITE_TELEMETRY_MAVLINK_PARAMS_H
