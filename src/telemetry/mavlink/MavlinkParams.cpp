/**
 * MavlinkParams.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 04 October 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#include "src/telemetry/mavlink/MavlinkParams.h"

#include <array>
#include <cmath>
#include <cstring>

#include "include/ConfigKeys.h"
#include "src/telemetry/mavlink/Mavlink.h"
#include "src/utils/ConfigRegistry.h"

namespace arduflite::mavlink {

namespace {

constexpr std::array kParams = std::to_array<ParamName>({
    { CONFIG_KEY_RATE_ROLL_KP,             "RATE_RLL_KP" },
    { CONFIG_KEY_RATE_ROLL_TI_S,           "RATE_RLL_TI_S" },
    { CONFIG_KEY_RATE_ROLL_TD_S,           "RATE_RLL_TD_S" },
    { CONFIG_KEY_RATE_ROLL_OUTLIMIT,       "RATE_RLL_OLIM" },
    { CONFIG_KEY_RATE_ROLL_HEADROOM,       "RATE_RLL_HDRM" },
    { CONFIG_KEY_RATE_ROLL_ALPHA,          "RATE_RLL_ALPHA" },
    { CONFIG_KEY_RATE_PITCH_KP,            "RATE_PIT_KP" },
    { CONFIG_KEY_RATE_PITCH_TI_S,          "RATE_PIT_TI_S" },
    { CONFIG_KEY_RATE_PITCH_TD_S,          "RATE_PIT_TD_S" },
    { CONFIG_KEY_RATE_PITCH_OUTLIMIT,      "RATE_PIT_OLIM" },
    { CONFIG_KEY_RATE_PITCH_HEADROOM,      "RATE_PIT_HDRM" },
    { CONFIG_KEY_RATE_PITCH_ALPHA,         "RATE_PIT_ALPHA" },
    { CONFIG_KEY_RATE_YAW_KP,              "RATE_YAW_KP" },
    { CONFIG_KEY_RATE_YAW_TI_S,            "RATE_YAW_TI_S" },
    { CONFIG_KEY_RATE_YAW_TD_S,            "RATE_YAW_TD_S" },
    { CONFIG_KEY_RATE_YAW_OUTLIMIT,        "RATE_YAW_OLIM" },
    { CONFIG_KEY_RATE_YAW_HEADROOM,        "RATE_YAW_HDRM" },
    { CONFIG_KEY_RATE_YAW_ALPHA,           "RATE_YAW_ALPHA" },
    { CONFIG_KEY_RATE_OUT_LP_ALPHA,        "RATE_OUT_ALPHA" },
    { CONFIG_KEY_ATT_ROLL_KP,              "ATT_RLL_KP" },
    { CONFIG_KEY_ATT_ROLL_TI_S,            "ATT_RLL_TI_S" },
    { CONFIG_KEY_ATT_ROLL_TD_S,            "ATT_RLL_TD_S" },
    { CONFIG_KEY_ATT_ROLL_OUTLIMIT_DPS,    "ATT_RLL_OLIM_DPS" },
    { CONFIG_KEY_ATT_ROLL_HEADROOM,        "ATT_RLL_HDRM" },
    { CONFIG_KEY_ATT_ROLL_ALPHA,           "ATT_RLL_ALPHA" },
    { CONFIG_KEY_ATT_PITCH_KP,             "ATT_PIT_KP" },
    { CONFIG_KEY_ATT_PITCH_TI_S,           "ATT_PIT_TI_S" },
    { CONFIG_KEY_ATT_PITCH_TD_S,           "ATT_PIT_TD_S" },
    { CONFIG_KEY_ATT_PITCH_OUTLIMIT_DPS,   "ATT_PIT_OLIM_DPS" },
    { CONFIG_KEY_ATT_PITCH_HEADROOM,       "ATT_PIT_HDRM" },
    { CONFIG_KEY_ATT_PITCH_ALPHA,          "ATT_PIT_ALPHA" },
    { CONFIG_KEY_ATT_YAW_KP,               "ATT_YAW_KP" },
    { CONFIG_KEY_ATT_YAW_TI_S,             "ATT_YAW_TI_S" },
    { CONFIG_KEY_ATT_YAW_TD_S,             "ATT_YAW_TD_S" },
    { CONFIG_KEY_ATT_YAW_OUTLIMIT_DPS,     "ATT_YAW_OLIM_DPS" },
    { CONFIG_KEY_ATT_YAW_HEADROOM,         "ATT_YAW_HDRM" },
    { CONFIG_KEY_ATT_YAW_ALPHA,            "ATT_YAW_ALPHA" },
    { CONFIG_KEY_ATT_DEADBAND_RAD,         "ATT_DEADBAND_RAD" },
    { CONFIG_KEY_MIX_MAX_ATT_ROLL_DEG,     "MIX_ATT_RLL_DEG" },
    { CONFIG_KEY_MIX_MAX_ATT_PITCH_DEG,    "MIX_ATT_PIT_DEG" },
    { CONFIG_KEY_MIX_MAX_ATT_YAW_DEG,      "MIX_ATT_YAW_DEG" },
    { CONFIG_KEY_MIX_MAX_RATE_ROLL_DPS,    "MIX_RATE_RLL_DPS" },
    { CONFIG_KEY_MIX_MAX_RATE_PITCH_DPS,   "MIX_RATE_PIT_DPS" },
    { CONFIG_KEY_MIX_MAX_RATE_YAW_DPS,     "MIX_RATE_YAW_DPS" },
    { CONFIG_KEY_MIX_ROLL_FROM_YAW,        "MIX_RLL_FROM_YAW" },
    { CONFIG_KEY_MIX_PITCH_FROM_ROLL,      "MIX_PIT_FROM_RLL" },
    { CONFIG_KEY_MIX_YAW_FROM_ROLL,        "MIX_YAW_FROM_RLL" },
    { CONFIG_KEY_SERVO_WING_DESIGN,        "SRV_WING_DESIGN" },
    { CONFIG_KEY_SERVO_DUAL_AILERONS,      "SRV_DUAL_AIL" },
    { CONFIG_KEY_SERVO_MAX_SLEW_DPS,       "SRV_MAX_SLEW_DPS" },
    { CONFIG_KEY_SERVO_MAX_THR_SLEW_PER_S, "SRV_THSLEW_PER_S" },
    { CONFIG_KEY_SERVO_PITCH_MIN_US,       "SRV_PIT_MIN_US" },
    { CONFIG_KEY_SERVO_PITCH_MAX_US,       "SRV_PIT_MAX_US" },
    { CONFIG_KEY_SERVO_PITCH_NEUTRAL_DEG,  "SRV_PIT_NTR_DEG" },
    { CONFIG_KEY_SERVO_PITCH_DEFL_DEG,     "SRV_PIT_DFL_DEG" },
    { CONFIG_KEY_SERVO_PITCH_INV,          "SRV_PIT_INVERT" },
    { CONFIG_KEY_SERVO_YAW_MIN_US,         "SRV_YAW_MIN_US" },
    { CONFIG_KEY_SERVO_YAW_MAX_US,         "SRV_YAW_MAX_US" },
    { CONFIG_KEY_SERVO_YAW_NEUTRAL_DEG,    "SRV_YAW_NTR_DEG" },
    { CONFIG_KEY_SERVO_YAW_DEFL_DEG,       "SRV_YAW_DFL_DEG" },
    { CONFIG_KEY_SERVO_YAW_INV,            "SRV_YAW_INVERT" },
    { CONFIG_KEY_SERVO_LAIL_MIN_US,        "SRV_LAIL_MIN_US" },
    { CONFIG_KEY_SERVO_LAIL_MAX_US,        "SRV_LAIL_MAX_US" },
    { CONFIG_KEY_SERVO_LAIL_NEUTRAL_DEG,   "SRV_LAIL_NTR_DEG" },
    { CONFIG_KEY_SERVO_LAIL_DEFL_DEG,      "SRV_LAIL_DFL_DEG" },
    { CONFIG_KEY_SERVO_LAIL_INV,           "SRV_LAIL_INVERT" },
    { CONFIG_KEY_SERVO_RAIL_MIN_US,        "SRV_RAIL_MIN_US" },
    { CONFIG_KEY_SERVO_RAIL_MAX_US,        "SRV_RAIL_MAX_US" },
    { CONFIG_KEY_SERVO_RAIL_NEUTRAL_DEG,   "SRV_RAIL_NTR_DEG" },
    { CONFIG_KEY_SERVO_RAIL_DEFL_DEG,      "SRV_RAIL_DFL_DEG" },
    { CONFIG_KEY_SERVO_RAIL_INV,           "SRV_RAIL_INVERT" },
    { CONFIG_KEY_SERVO_THR_MIN_US,         "SRV_THR_MIN_US" },
    { CONFIG_KEY_SERVO_THR_MAX_US,         "SRV_THR_MAX_US" },
    { CONFIG_KEY_IMU_ACCEL_ALPHA,          "IMU_ACCEL_ALPHA" },
    { CONFIG_KEY_IMU_GYRO_ALPHA,           "IMU_GYRO_ALPHA" },
    { CONFIG_KEY_IMU_MAG_ALPHA,            "IMU_MAG_ALPHA" },
    { CONFIG_KEY_IMU_ALTI_ALPHA,           "IMU_ALTI_ALPHA" },
    { CONFIG_KEY_IMU_MADGWICK_BETA,        "IMU_MADG_BETA" },
    { CONFIG_KEY_IMU_FUSE_MAG,             "IMU_FUSE_MAG" },
    { CONFIG_KEY_IMU_MAX_ACCEL_G,          "IMU_MAX_ACCEL_G" },
    { CONFIG_KEY_IMU_MAX_GYRO_DPS,         "IMU_MAX_GYRO_DPS" },
    { CONFIG_KEY_IMU_FAIL_THRESHOLD,       "IMU_FAIL_THRESH" },
    { CONFIG_KEY_IMU_GYRO_BIAS_MAX_DPS,    "IMU_BIAS_MAX_DPS" },
    { CONFIG_KEY_IMU_EXPECTED_G,           "IMU_EXP_G" },
    { CONFIG_KEY_IMU_GRAVITY_TOL_G,        "IMU_GRAV_TOL_G" },
    { CONFIG_KEY_IMU_LAUNCH_ACCEL_G,       "IMU_LCH_ACCEL_G" },
    { CONFIG_KEY_IMU_LAUNCH_GYRO_DPS,      "IMU_LCH_GYRO_DPS" },
    { CONFIG_KEY_IMU_LAUNCH_MS,            "IMU_LCH_MS" },
    { CONFIG_KEY_FS_BANK_DEG,              "FS_BANK_DEG" },
    { CONFIG_KEY_FS_PITCH_DEG,             "FS_PIT_DEG" },
    { CONFIG_KEY_FS_THROTTLE,              "FS_THROTTLE" },
    { CONFIG_KEY_FS_MIN_LQ_ARM_PCT,        "FS_ARM_LQ_PCT" },
    { CONFIG_KEY_CRSF_TRI_LOW,             "CRSF_TRI_LOW" },
    { CONFIG_KEY_CRSF_TRI_HIGH,            "CRSF_TRI_HIGH" },
    { CONFIG_KEY_WEB_ENABLED,              "WEB_ENABLED" },
    { CONFIG_KEY_MAV_SYSID,                "MAV_SYSID" },
    { CONFIG_KEY_MAV_UART_ENABLED,         "MAV_UART_ENABLED" },
    { CONFIG_KEY_MAV_UART_BAUD,            "MAV_UART_BAUD" },
    { CONFIG_KEY_MAV_UART_MAX_BPS,         "MAV_UART_MAX_BPS" },
    { CONFIG_KEY_MAV_UART_WRITES,          "MAV_UART_WRITES" },
});

const ParamName* findName(std::string_view id)
{
    for (const ParamName& p : kParams)
    {
        if (id == p.id) { return &p; }
    }
    return nullptr;
}

std::optional<ParamValue> describe(std::uint16_t index)
{
    const ParamName& name = kParams[index];
    const std::optional<ConfigParam> param = ConfigRegistry::instance().getParam(name.key);
    if (!param) { return std::nullopt; }

    ParamValue out;
    std::strncpy(out.id, name.id, kParamIdLength);
    out.index = index;
    out.count = paramCount();

    switch (param->type)
    {
        case ConfigType::FLOAT:
            out.value = param->currentVal.f;
            out.type  = MAV_PARAM_TYPE_REAL32;
            break;
        case ConfigType::INT32:
            out.value = static_cast<float>(param->currentVal.i);
            out.type  = MAV_PARAM_TYPE_INT32;
            break;
        case ConfigType::UINT8:
            out.value = static_cast<float>(param->currentVal.u8);
            out.type  = MAV_PARAM_TYPE_UINT8;
            break;
        case ConfigType::BOOL:
            out.value = param->currentVal.b ? 1.0f : 0.0f;
            out.type  = MAV_PARAM_TYPE_UINT8;
            break;
        case ConfigType::STRING:
            return std::nullopt;
    }
    return out;
}

/// The int32 range as floats. 2^31 is not representable as an int32, so the
/// upper bound is the largest float below it.
constexpr float kInt32Min = -2147483648.0f;
constexpr float kInt32Max = 2147483520.0f;

bool isWhole(float value, float min, float max)
{
    return std::isfinite(value) && std::trunc(value) == value && value >= min && value <= max;
}

} // namespace

std::span<const ParamName> paramNames() noexcept { return kParams; }

std::uint16_t paramCount() noexcept { return static_cast<std::uint16_t>(kParams.size()); }

std::optional<ParamValue> readParam(std::uint16_t index)
{
    if (index >= kParams.size()) { return std::nullopt; }
    return describe(index);
}

std::optional<ParamValue> readParam(std::string_view id)
{
    const ParamName* name = findName(id);
    if (name == nullptr) { return std::nullopt; }
    return describe(static_cast<std::uint16_t>(name - kParams.data()));
}

ParamWrite writeParam(std::string_view id, float value)
{
    const ParamName* name = findName(id);
    if (name == nullptr) { return ParamWrite::Unknown; }

    ConfigRegistry& registry = ConfigRegistry::instance();
    const std::optional<ConfigParam> param = registry.getParam(name->key);
    if (!param) { return ParamWrite::Unknown; }

    bool applied = false;
    switch (param->type)
    {
        case ConfigType::FLOAT:
            applied = std::isfinite(value) && registry.set<float>(name->key, value);
            break;
        case ConfigType::INT32:
            applied = isWhole(value, kInt32Min, kInt32Max)
                   && registry.set<int32_t>(name->key, static_cast<int32_t>(value));
            break;
        case ConfigType::UINT8:
            applied = isWhole(value, 0.0f, 255.0f)
                   && registry.set<uint8_t>(name->key, static_cast<uint8_t>(value));
            break;
        case ConfigType::BOOL:
            applied = (value == 0.0f || value == 1.0f)
                   && registry.set<bool>(name->key, value == 1.0f);
            break;
        case ConfigType::STRING:
            return ParamWrite::Unknown;
    }
    return applied ? ParamWrite::Applied : ParamWrite::Rejected;
}

} // namespace arduflite::mavlink
