/**
 * lolin_c3_mini.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Lolin C3 Mini (ESP32-C3). The board the aircraft currently flies.
 */
#ifndef ARDUFLITE_HAL_BOARD_LOLIN_C3_MINI_H
#define ARDUFLITE_HAL_BOARD_LOLIN_C3_MINI_H

#include "src/hal/board/BoardDescriptor.h"

namespace arduflite::board {

/// @note reservedMask is a PLACEHOLDER — verify against the ESP32-C3 datasheet and
///       the specific module before relying on it. Too small makes validation
///       useless; too large makes it a false blocker.
inline constexpr McuProfile kEsp32C3{
    .name            = "ESP32-C3",
    .minGpio         = 0,
    .maxGpio         = 21,
    .inputOnlyMask   = 0,
    .reservedMask    = 0x000000000003F000ull,  // GPIO12-17: SPI flash
    .uartCount       = 2,
    .pwmChannelCount = 6,
};

inline constexpr BoardDescriptor kBoard{
    .name     = "Lolin C3 Mini",
    .maturity = BoardMaturity::Supported,
    .mcu      = kEsp32C3,

    .sensorBus   = { .sda = 3, .scl = 5, .clock_hz = 400000 },
    .rcUart      = { .port = 1, .rx = 6, .tx = 8, .baud = 420000, .invertRx = false },
    .consoleUart = { .port = 0, .rx = kNoPin, .tx = kNoPin, .baud = 115200, .invertRx = false },

    .sensors = { {
        // Mirrored map (det = -1) — the transform the prototype currently flies
        // with, ported verbatim from applyOrientation(). See specs/hal 00 2.3.
        SensorMount{ .part = SensorPart::Mpu6500, .bus = BusKind::I2c, .address = 0x68,
                     .axes = { SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ },
                     .label = "imu0" },
        SensorMount{ .part = SensorPart::Bmp280,  .bus = BusKind::I2c, .address = 0x76,
                     .axes = {}, .label = "baro0" },
    } },
    .sensorCount = 2,

    .rcLink = RcPart::Crsf,

    .actuatorBanks = { {
        ActuatorBankDesc{
            .transport = ActuatorTransport::Pwm,
            .busIndex  = 0,
            .outputs = { {
                ActuatorOutputDesc{ .role = "aileron_right", .pin = 1  },
                ActuatorOutputDesc{ .role = "aileron_left",  .pin = 2  },
                ActuatorOutputDesc{ .role = "elevator",      .pin = 0  },
                ActuatorOutputDesc{ .role = "rudder",        .pin = 4  },
                ActuatorOutputDesc{ .role = "throttle",      .pin = 10 },
            } },
            .outputCount = 5,
        },
    } },
    .bankCount = 1,

    .userButton = { .pin = 9, .mode = hal::PinMode::InputPullUp, .role = "user" },
    .statusLed  = { .pin = 7, .pixelCount = 1, .brightness = 50 },
};

} // namespace arduflite::board

#endif // ARDUFLITE_HAL_BOARD_LOLIN_C3_MINI_H
