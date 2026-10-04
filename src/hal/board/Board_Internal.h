/**
 * Board_Internal.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Concrete driver storage for Board.
 *
 * @warning INCLUDED ONLY BY Board.cpp. Including this anywhere else defeats the
 *          layering split and will be caught by tools/ci/check_layering.sh.
 *
 * Every member is a value, not a pointer: ADR-011 forbids heap allocation after
 * boot, and this whole struct is one file-scope object in Board.cpp.
 */
#ifndef ARDUFLITE_HAL_BOARD_BOARD_INTERNAL_H
#define ARDUFLITE_HAL_BOARD_BOARD_INTERNAL_H

#include <array>
#include <optional>

#include "src/hal/board/BoardSelect.h"
#include "src/hal/drivers/baro/Bmp280.h"
#include "src/hal/drivers/baro/Bmp581.h"
#include "src/hal/drivers/imu/Bmi323.h"
#include "src/hal/drivers/mag/Bmm350.h"
#include "src/hal/drivers/imu/Mpu6500.h"
#include "src/hal/esp32/Esp32Clock.h"
#include "src/hal/drivers/indicator/NeoPixelIndicator.h"
#include "src/hal/drivers/log/LittleFsLogStore.h"
#include "src/hal/esp32/Esp32Console.h"
#include "src/hal/esp32/Esp32I2cBus.h"
#include "src/hal/esp32/Esp32Io.h"
#include "src/hal/esp32/Esp32Mutex.h"
#include "src/hal/esp32/Esp32Scheduler.h"
#include "src/hal/esp32/Esp32KeyValueStore.h"
#include "src/hal/esp32/Esp32SettingsStore.h"
#include "src/hal/esp32/Esp32System.h"
#include "src/hal/esp32/Esp32Uart.h"
#include "src/hal/esp32/Esp32Watchdog.h"

namespace arduflite::board {

struct BoardStorage
{
    hal::esp32::Esp32Clock     clock{};
    hal::esp32::Esp32Scheduler scheduler{};
    hal::esp32::Esp32Watchdog  watchdog{};
    hal::esp32::Esp32System    system{};

    hal::esp32::Esp32I2cBus sensorBus{ kBoard.sensorBus.sda, kBoard.sensorBus.scl };

    /// Shared by the CRSF receiver (reads) and CRSF telemetry (writes).
    hal::esp32::Esp32Uart rcUart{ kBoard.rcUart.port, kBoard.rcUart.rx, kBoard.rcUart.tx };

    std::array<hal::esp32::Esp32PwmOut, BoardDescriptor::kMaxBanks *
                                        ActuatorBankDesc::kMaxPerBank> pwm{};
    std::uint8_t pwmCount = 0;

    hal::esp32::Esp32GpioPin userButton{};

    /// NVS namespace for calibration blobs. 15 characters is the NVS limit.
    hal::esp32::Esp32SettingsStore settings{ "arduflite" };

    hal::esp32::Esp32Console console{};

    /**
     * optional because LittleFsLogStore holds an Arduino File, whose
     * constructor is not constexpr — and BoardStorage is constinit. The guard
     * caught this immediately, which is the third time it has caught a
     * static-init hazard in this HAL (Esp32I2cBus, Esp32SettingsStore, this).
     * Emplaced by Board::begin().
     */
    std::optional<drivers::LittleFsLogStore> logs{};

    /// Status LED, if the descriptor declares one. Absent on boards with
    /// statusLed.pin == kNoPin — the FireBeetle has no pixel fitted.
    std::optional<drivers::NeoPixelIndicator> indicator{};

    /// Runtime configuration. Separate namespace from `settings` so a
    /// config factory-reset cannot wipe the calibration blob.
    hal::esp32::Esp32KeyValueStore config{ "aflite-cfg" };

    /**
     * Sensor drivers. optional because a driver needs a RegisterDevice, and
     * those are carved out of the bus at begin() — after this object is
     * constant-initialised. An empty optional is also how a genuinely absent
     * sensor is represented: the spare board has no IMU, and Board::imu()
     * returning nullptr is what lets the rest of the system degrade rather
     * than hang waiting for a part that is not fitted.
     */
    /// Cap on instances of any one sensor kind.
    static constexpr std::uint8_t kMaxPerKind = 4;

    /**
     * ARRAYS, not single optionals.
     *
     * They were singular until the Phase 8 extensibility proof added a second
     * MPU-6500 to the lolin descriptor. That change compiled cleanly and was
     * wrong: the second emplace() destroyed the first driver and constructed
     * the replacement in its place, so both span entries pointed at one object
     * and the sample() list held it twice — sampling the same chip twice per
     * tick while the redundant part was never read at all.
     *
     * Compiling is not the test. This is what the proof exists to catch.
     */
    std::array<std::optional<drivers::Mpu6500>, kMaxPerKind> imus{};
    std::array<std::optional<drivers::Bmi323>,  kMaxPerKind> bmiImus{};
    std::array<std::optional<drivers::Bmp280>,  kMaxPerKind> baros{};
    std::array<std::optional<drivers::Bmp581>,  kMaxPerKind> baros581{};
    std::array<std::optional<drivers::Bmm350>,  kMaxPerKind> magnetometers{};
    /// How many DRIVER slots are used. Distinct from the span counters below,
    /// which count interface pointers — one part can appear in several spans.
    std::uint8_t imuDriverCount    = 0;
    std::uint8_t bmiImuDriverCount = 0;
    std::uint8_t baroDriverCount    = 0;
    std::uint8_t baro581DriverCount = 0;
    std::uint8_t magDriverCount     = 0;

    /**
     * Interface views onto the drivers above, built once in beginSensors().
     *
     * Fixed capacity, no heap (ADR-011). A driver that failed to initialise is
     * simply never added, so the spans are exactly the working parts — callers
     * iterate without null checks.
     */

    std::array<device::Sensor*,        kMaxPerKind> sensorList{};
    std::array<device::Accelerometer*, kMaxPerKind> accelList{};
    std::array<device::Gyroscope*,     kMaxPerKind> gyroList{};
    std::array<device::Magnetometer*,  kMaxPerKind> magList{};
    std::array<device::Barometer*,     kMaxPerKind> baroList{};

    std::uint8_t sensorCount = 0;
    std::uint8_t accelCount  = 0;
    std::uint8_t gyroCount   = 0;
    std::uint8_t magCount    = 0;
    std::uint8_t baroCount   = 0;

    /// Pool for flight-layer classes. Sized from what actually asks:
    /// ArduFliteController takes 3, telemetry backends 2 each. 16 is generous.
    static constexpr std::uint8_t kMutexPoolSize = 16;
    std::array<hal::esp32::Esp32Mutex, kMutexPoolSize> mutexPool{};
    std::uint8_t                                       mutexesUsed = 0;
};

} // namespace arduflite::board

#endif // ARDUFLITE_HAL_BOARD_BOARD_INTERNAL_H
