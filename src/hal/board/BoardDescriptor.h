/**
 * BoardDescriptor.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Declarative board description. Replaces PinConfiguration.h's five
 *        #if BOARD_TYPE blocks, and makes pin defects compile errors.
 */
#ifndef ARDUFLITE_HAL_BOARD_BOARDDESCRIPTOR_H
#define ARDUFLITE_HAL_BOARD_BOARDDESCRIPTOR_H

#include <array>
#include <cstdint>

#include "src/hal/core/AxisTransform.h"
#include "src/hal/device/Actuator.h"
#include "src/hal/platform/Io.h"

namespace arduflite::board {

using Pin = std::int8_t;
inline constexpr Pin kNoPin = -1;

/// A port in progress must be able to compile before every pin is known.
/// Untested relaxes the "fitted parts must be wired" assertion and makes the
/// board warn loudly at boot. Nothing may be released as Untested.
enum class BoardMaturity : std::uint8_t { Supported, Untested };

struct McuProfile
{
    const char*   name            = "";
    Pin           minGpio         = 0;
    Pin           maxGpio         = 0;
    /// bit N: GPIO N cannot be an output. 64-bit because the classic ESP32 has
    /// GPIO up to 39 — a uint32_t silently cannot represent GPIO 32-39, and
    /// shifting one by >= 32 is undefined behaviour.
    std::uint64_t inputOnlyMask   = 0;
    std::uint64_t reservedMask    = 0;   ///< bit N: flash/PSRAM/strapping
    std::uint8_t  uartCount       = 0;
    std::uint8_t  pwmChannelCount = 0;
};

struct I2cBusDesc { Pin sda = kNoPin; Pin scl = kNoPin; std::uint32_t clock_hz = 400000; };
struct UartDesc   { std::uint8_t port = 0; Pin rx = kNoPin; Pin tx = kNoPin;
                    std::uint32_t baud = 115200; bool invertRx = false; };
struct GpioDesc   { Pin pin = kNoPin; hal::PinMode mode = hal::PinMode::Input;
                    const char* role = ""; };
struct LedDesc    { Pin pin = kNoPin; std::uint16_t pixelCount = 0;
                    std::uint8_t brightness = 50; };

// ── Fitted sensors — one entry per PHYSICAL CHIP ────────────────────────────

enum class SensorPart : std::uint8_t
{
    None, Mpu6500, Mpu9250, Bmp280, UbloxGnss, Ina226, Sim,
};

enum class BusKind : std::uint8_t { None, I2c, Spi, Uart };

/// `axes` is PER SENSOR INSTANCE, not per board: on custom hardware a discrete
/// gyro and a discrete accelerometer can be mounted at different angles.
struct SensorMount
{
    SensorPart   part    = SensorPart::None;
    BusKind      bus     = BusKind::None;
    std::uint8_t address = 0;      ///< I2C address, SPI CS index, or UART port
    AxisMap      axes{};           ///< ignored by baro / GNSS / power
    const char*  label   = "";     ///< "imu0" — appears in logs and the CLI
};

// ── Fitted actuators — grouped BY TRANSPORT, one group per ActuatorBank ─────

enum class ActuatorTransport : std::uint8_t { Pwm, CanOpen, DShot, Sim };

struct ActuatorOutputDesc
{
    const char*          role   = "";   ///< unique across the WHOLE board
    device::ActuatorKind kind   = device::ActuatorKind::Proportional;
    Pin                  pin    = kNoPin;   ///< PWM / DShot only
    std::uint8_t         nodeId = 0;        ///< CANopen only
};

struct ActuatorBankDesc
{
    static constexpr std::uint8_t kMaxPerBank = 8;

    ActuatorTransport transport = ActuatorTransport::Pwm;
    std::uint8_t      busIndex  = 0;
    std::array<ActuatorOutputDesc, kMaxPerBank> outputs{};
    std::uint8_t      outputCount = 0;
};

enum class RcPart : std::uint8_t { None, Crsf, Sim, Unknown };

// ── The board ───────────────────────────────────────────────────────────────

struct BoardDescriptor
{
    static constexpr std::uint8_t kMaxSensors = 8;
    static constexpr std::uint8_t kMaxBanks   = 3;

    const char*   name     = "";
    BoardMaturity maturity = BoardMaturity::Untested;
    McuProfile    mcu{};

    I2cBusDesc sensorBus{};
    UartDesc   rcUart{};
    UartDesc   consoleUart{};

    std::array<SensorMount, kMaxSensors> sensors{};
    std::uint8_t                         sensorCount = 0;

    RcPart rcLink = RcPart::None;

    std::array<ActuatorBankDesc, kMaxBanks> actuatorBanks{};
    std::uint8_t                            bankCount = 0;

    GpioDesc userButton{};
    LedDesc  statusLed{};
};

} // namespace arduflite::board

#endif // ARDUFLITE_HAL_BOARD_BOARDDESCRIPTOR_H
