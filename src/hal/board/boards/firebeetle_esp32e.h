/**
 * firebeetle_esp32e.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief DFRobot FireBeetle 2 ESP32-E.
 *
 * @warning UNTESTED. The CRSF pins are not known, so the RC link is
 *          RcPart::Unknown and the board is BoardMaturity::Untested — which
 *          relaxes the "fitted parts must be wired" assertion and makes the
 *          firmware warn loudly at boot. Put a meter on the board, fill in the
 *          pins, then promote it to Supported.
 */
#ifndef ARDUFLITE_HAL_BOARD_FIREBEETLE_ESP32E_H
#define ARDUFLITE_HAL_BOARD_FIREBEETLE_ESP32E_H

#include "src/hal/board/BoardDescriptor.h"

namespace arduflite::board {

inline constexpr McuProfile kEsp32{
    .name            = "ESP32",
    .minGpio         = 0,
    .maxGpio         = 39,
    // GPIO34-39 are input-only on the classic ESP32 — bits 34..39, which do NOT
    // fit in a uint32_t. This was wrong before the mask was widened to 64 bits.
    .inputOnlyMask   = 0x000000FC00000000ull,
    .reservedMask    = 0x0000000000000FC0ull,   // GPIO6-11: SPI flash
    .uartCount       = 3,
    .pwmChannelCount = 16,
};

inline constexpr BoardDescriptor kBoard{
    .name     = "DFRobot FireBeetle 2 ESP32-E",
    .maturity = BoardMaturity::Untested,
    .mcu      = kEsp32,

    .sensorBus   = { .sda = 21, .scl = 22, .clock_hz = 400000 },
    .rcUart      = { .port = 1, .rx = kNoPin, .tx = kNoPin, .baud = 420000, .invertRx = false },
    .consoleUart = { .port = 0, .rx = kNoPin, .tx = kNoPin, .baud = 115200, .invertRx = false },

    .sensors = { {
        SensorMount{ .part = SensorPart::Mpu6500, .bus = BusKind::I2c, .address = 0x68,
                     .axes = { SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ },
                     .label = "imu0" },
        SensorMount{ .part = SensorPart::Bmp280,  .bus = BusKind::I2c, .address = 0x76,
                     .axes = {}, .label = "baro0" },
    } },
    .sensorCount = 2,

    .rcLink = RcPart::Unknown,   // pins unknown — see the warning above

    .actuatorBanks = { {
        ActuatorBankDesc{
            .transport = ActuatorTransport::Pwm,
            .busIndex  = 0,
            .outputs = { {
                ActuatorOutputDesc{ .role = "aileron_right", .pin = 16 },
                ActuatorOutputDesc{ .role = "aileron_left",  .pin = 17 },
                ActuatorOutputDesc{ .role = "elevator",      .pin = 4  },
                ActuatorOutputDesc{ .role = "rudder",        .pin = 12 },
                // Unassigned until the board is metered. GPIO 9 is inside the
                // classic ESP32's SPI-flash range (GPIO 6-11) and cannot drive a
                // servo, so it is not a safe guess to carry here.
                ActuatorOutputDesc{ .role = "throttle",      .pin = kNoPin },
            } },
            .outputCount = 5,
        },
    } },
    .bankCount = 1,

    .userButton = { .pin = 27, .mode = hal::PinMode::InputPullUp, .role = "user" },
    .statusLed  = { .pin = kNoPin, .pixelCount = 0, .brightness = 0 },
};

} // namespace arduflite::board

#endif // ARDUFLITE_HAL_BOARD_FIREBEETLE_ESP32E_H
