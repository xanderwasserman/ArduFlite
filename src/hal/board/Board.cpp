/**
 * Board.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The only translation unit that names a concrete driver type.
 */
#include "src/hal/board/Board.h"

#include "src/hal/board/Board_Internal.h"
#include "src/utils/Logging.h"

namespace arduflite::board {

namespace {

/**
 * constinit guarantees constant initialisation — a compile error if this would
 * need dynamic init. That removes the static-initialisation-order hazard the
 * existing initFromConfig() pattern exists to work around: the storage is laid
 * out in .bss before any constructor anywhere runs.
 */
constinit BoardStorage g_storage{};

const char* toString(SensorPart p)
{
    switch (p)
    {
        case SensorPart::Mpu6500:   return "MPU-6500";
        case SensorPart::Bmi323:    return "BMI323";
        case SensorPart::Bmm350:    return "BMM350";
        case SensorPart::Bmp581:    return "BMP581";
        case SensorPart::Mpu9250:   return "MPU-9250";
        case SensorPart::Bmp280:    return "BMP280";
        case SensorPart::UbloxGnss: return "u-blox GNSS";
        case SensorPart::Ina226:    return "INA226";
        case SensorPart::Sim:       return "Sim";
        case SensorPart::None:      return "none";
    }
    return "?";
}

const char* toString(ActuatorTransport t)
{
    switch (t)
    {
        case ActuatorTransport::Pwm:     return "PWM";
        case ActuatorTransport::CanOpen: return "CANopen";
        case ActuatorTransport::DShot:   return "DShot";
        case ActuatorTransport::Sim:     return "Sim";
    }
    return "?";
}

/// Adds a driver to the sample() list. Called once per PART, not once per
/// measurement — sampling an MPU-6500 twice per tick would double bus traffic
/// and hand the estimator two different bursts as if they were one.
void registerSensor(BoardStorage& s, device::Sensor& sensor)
{
    if (s.sensorCount < BoardStorage::kMaxPerKind)
    {
        s.sensorList[s.sensorCount++] = &sensor;
    }
    else
    {
        LOG_ERR("[Board] sensor list full (%u) - raise kMaxPerKind",
                static_cast<unsigned>(BoardStorage::kMaxPerKind));
    }
}

} // namespace

Board::Board() noexcept = default;

Board& Board::instance() noexcept
{
    static Board board;
    return board;
}

BoardStorage& Board::storage() noexcept { return g_storage; }

Status Board::begin()
{
    if (_started) { return Status::Ok; }

    BoardStorage& s = storage();

    if (kBoard.maturity == BoardMaturity::Untested)
    {
        LOG_WARN("BOARD '%s' IS MARKED UNTESTED - pins are unverified. Do not fly.",
                 kBoard.name);
    }

    // ── Tier 0 ──────────────────────────────────────────────────────────────
    // A bus failing to come up IS fatal, unlike a missing sensor: nothing
    // downstream can degrade gracefully around a dead I2C controller.
    // The console comes up first: everything below logs, and a log line
    // emitted before the port is open is simply lost.
    // Deliberately NOT fatal. Today Esp32Console::begin() cannot fail, but if a
    // future console could, losing it must not ground the aircraft — an
    // aircraft that flies without a diagnostic port is fine; one that refuses
    // to boot because it has no console is not. There is also nowhere to report
    // this failure to, by definition.
    (void)s.console.begin(115200);

    ARDUFLITE_TRY(s.sensorBus.begin(kBoard.sensorBus.clock_hz));

    if (kBoard.rcLink != RcPart::None && kBoard.rcUart.rx != kNoPin)
    {
        ARDUFLITE_TRY(s.rcUart.begin(kBoard.rcUart.baud, kBoard.rcUart.invertRx));
    }

    // PWM outputs are bound here but deliberately NOT attached: Phase 3 owns the
    // actuator cutover, and attaching would start pulsing servos with no
    // ActuatorBank to command them. See specs/hal/06 Phase 2.
    s.pwmCount = 0;
    for (std::uint8_t b = 0; b < kBoard.bankCount; ++b)
    {
        const ActuatorBankDesc& bank = kBoard.actuatorBanks[b];
        if (bank.transport != ActuatorTransport::Pwm &&
            bank.transport != ActuatorTransport::DShot)
        {
            continue;
        }

        for (std::uint8_t i = 0; i < bank.outputCount; ++i)
        {
            const Pin pin = bank.outputs[i].pin;
            if (pin == kNoPin)                    { continue; }
            if (s.pwmCount >= s.pwm.size())       { break; }
            s.pwm[s.pwmCount].bind(pin);
            ++s.pwmCount;
        }
    }

    ARDUFLITE_TRY(beginSensors());

    if (kBoard.statusLed.pin != kNoPin && kBoard.statusLed.pixelCount > 0)
    {
        auto& led = s.indicator.emplace(static_cast<std::uint8_t>(kBoard.statusLed.pin),
                                        s.scheduler,
                                        kBoard.statusLed.pixelCount,
                                        kBoard.statusLed.brightness);
        if (led.begin() != Status::Ok)
        {
            LOG_ERR("[Board] status LED failed to start");
            s.indicator.reset();
        }
    }

    if (kBoard.userButton.pin != kNoPin)
    {
        s.userButton.bind(kBoard.userButton.pin);
        ARDUFLITE_TRY(s.userButton.setMode(kBoard.userButton.mode));
    }

    _started = true;
    return Status::Ok;
}

/**
 * @brief Bring up whatever the descriptor says is fitted.
 *
 * A missing or broken sensor is NOT fatal. It is logged loudly and left null;
 * the flight layer decides what it can still do. The spare bench board has no
 * IMU at all and must still boot to a usable CLI.
 */
Status Board::beginSensors()
{
    BoardStorage& s = storage();

    for (std::uint8_t i = 0; i < kBoard.sensorCount; ++i)
    {
        const SensorMount& mount = kBoard.sensors[i];
        if (mount.bus != BusKind::I2c) { continue; }

        Result<hal::RegisterDevice*> dev = s.sensorBus.openDevice(mount.address);
        if (!dev)
        {
            LOG_ERR("[Board] %s: no device handle at 0x%02X", mount.label, mount.address);
            continue;
        }

        switch (mount.part)
        {
            case SensorPart::Mpu6500:
            {
                if (s.imuDriverCount >= BoardStorage::kMaxPerKind)
                {
                    LOG_ERR("[Board] %s: too many IMUs (max %u)", mount.label,
                            (unsigned)BoardStorage::kMaxPerKind);
                    break;
                }
                drivers::Mpu6500& imu =
                    s.imus[s.imuDriverCount].emplace(*dev.value(), s.clock, s.scheduler);

                // probe() before begin(): writing configuration to whatever else
                // happens to sit at 0x68 is how you brick an unrelated device.
                const Status probed = imu.probe();
                if (probed != Status::Ok)
                {
                    // The actual ID matters here. The maintainer suspects a
                    // counterfeit part on the prototype airframe, and FastIMU
                    // discarded this byte — it compared against 0x70 and returned
                    // -1. Log it whether or not we go on to use the part.
                    LOG_ERR("[Board] %s: WHO_AM_I=0x%02X, expected 0x%02X - "
                            "unsupported or counterfeit part, IMU disabled",
                            mount.label, imu.whoAmI(), drivers::Mpu6500::kWhoAmIExpected);
                    s.imus[s.imuDriverCount].reset();
                    break;
                }

                if (const Status st = imu.begin(); st != Status::Ok)
                {
                    LOG_ERR("[Board] %s: configuration failed (%s), IMU disabled",
                            mount.label, toString(st));
                    s.imus[s.imuDriverCount].reset();
                    break;
                }
                ++s.imuDriverCount;
                registerSensor(s, imu);
                if (s.accelCount < BoardStorage::kMaxPerKind) { s.accelList[s.accelCount++] = &imu; }
                if (s.gyroCount  < BoardStorage::kMaxPerKind) { s.gyroList[s.gyroCount++]   = &imu; }

                LOG_INF("[Board] %s: %s ready (%u dps, %u g, %u Hz)",
                        mount.label, imu.name(), imu.range_dps(), imu.range_g(),
                        imu.nativeRate_hz());
                break;
            }

            case SensorPart::Bmi323:
            {
                if (s.bmiImuDriverCount >= BoardStorage::kMaxPerKind)
                {
                    LOG_ERR("[Board] %s: too many IMUs", mount.label);
                    break;
                }
                drivers::Bmi323& imu =
                    s.bmiImus[s.bmiImuDriverCount].emplace(*dev.value(), s.clock, s.scheduler);

                if (imu.probe() != Status::Ok)
                {
                    LOG_ERR("[Board] %s: CHIP_ID=0x%04X, expected 0x%04X - "
                            "unsupported part, IMU disabled",
                            mount.label, imu.chipId(), drivers::Bmi323::kChipId);
                    s.bmiImus[s.bmiImuDriverCount].reset();
                    break;
                }

                if (const Status st = imu.begin(); st != Status::Ok)
                {
                    LOG_ERR("[Board] %s: configuration failed (%s), IMU disabled",
                            mount.label, toString(st));
                    s.bmiImus[s.bmiImuDriverCount].reset();
                    break;
                }
                ++s.bmiImuDriverCount;

                registerSensor(s, imu);
                if (s.accelCount < BoardStorage::kMaxPerKind) { s.accelList[s.accelCount++] = &imu; }
                if (s.gyroCount  < BoardStorage::kMaxPerKind) { s.gyroList[s.gyroCount++]   = &imu; }

                LOG_INF("[Board] %s: %s ready (%u dps, %u g, %u Hz)",
                        mount.label, imu.name(), imu.range_dps(), imu.range_g(),
                        imu.nativeRate_hz());
                break;
            }

            case SensorPart::Bmp280:
            {
                if (s.baroDriverCount >= BoardStorage::kMaxPerKind)
                {
                    LOG_ERR("[Board] %s: too many barometers", mount.label);
                    break;
                }
                drivers::Bmp280& baro =
                    s.baros[s.baroDriverCount].emplace(*dev.value(), s.clock, s.scheduler);

                if (const Status probed = baro.probe(); probed != Status::Ok)
                {
                    LOG_ERR("[Board] %s: chip ID=0x%02X, expected 0x%02X - "
                            "barometer disabled (0x60 would be a BME280)",
                            mount.label, baro.chipId(), drivers::Bmp280::kIdExpected);
                    s.baros[s.baroDriverCount].reset();
                    break;
                }

                if (const Status st = baro.begin(); st != Status::Ok)
                {
                    LOG_ERR("[Board] %s: configuration failed (%s), barometer disabled",
                            mount.label, toString(st));
                    s.baros[s.baroDriverCount].reset();
                    break;
                }
                ++s.baroDriverCount;
                registerSensor(s, baro);
                if (s.baroCount < BoardStorage::kMaxPerKind) { s.baroList[s.baroCount++] = &baro; }

                LOG_INF("[Board] %s: %s ready", mount.label, baro.name());
                break;
            }

            case SensorPart::Bmp581:
            {
                if (s.baro581DriverCount >= BoardStorage::kMaxPerKind)
                {
                    LOG_ERR("[Board] %s: too many barometers", mount.label);
                    break;
                }
                drivers::Bmp581& baro =
                    s.baros581[s.baro581DriverCount].emplace(*dev.value(), s.clock, s.scheduler);

                if (baro.probe() != Status::Ok)
                {
                    LOG_ERR("[Board] %s: chip ID=0x%02X, expected 0x%02X - barometer disabled",
                            mount.label, baro.chipId(), drivers::Bmp581::kChipId);
                    s.baros581[s.baro581DriverCount].reset();
                    break;
                }

                if (const Status st = baro.begin(); st != Status::Ok)
                {
                    LOG_ERR("[Board] %s: configuration failed (%s), barometer disabled",
                            mount.label, toString(st));
                    s.baros581[s.baro581DriverCount].reset();
                    break;
                }
                ++s.baro581DriverCount;

                registerSensor(s, baro);
                if (s.baroCount < BoardStorage::kMaxPerKind) { s.baroList[s.baroCount++] = &baro; }

                LOG_INF("[Board] %s: %s ready", mount.label, baro.name());
                break;
            }

            case SensorPart::Bmm350:
            {
                if (s.magDriverCount >= BoardStorage::kMaxPerKind)
                {
                    LOG_ERR("[Board] %s: too many magnetometers", mount.label);
                    break;
                }
                drivers::Bmm350& magnetometer =
                    s.magnetometers[s.magDriverCount].emplace(*dev.value(), s.clock, s.scheduler);

                if (magnetometer.probe() != Status::Ok)
                {
                    LOG_ERR("[Board] %s: chip ID=0x%02X, expected 0x%02X - "
                            "magnetometer disabled",
                            mount.label, magnetometer.chipId(), drivers::Bmm350::kChipId);
                    s.magnetometers[s.magDriverCount].reset();
                    break;
                }

                // begin() reads 32 OTP words over I2C. It is the slowest sensor
                // bring-up on the board, and the only one that can fail for a
                // reason other than a missing part.
                if (const Status st = magnetometer.begin(); st != Status::Ok)
                {
                    LOG_ERR("[Board] %s: trimming read failed (%s), magnetometer disabled",
                            mount.label, toString(st));
                    s.magnetometers[s.magDriverCount].reset();
                    break;
                }
                ++s.magDriverCount;

                registerSensor(s, magnetometer);
                if (s.magCount < BoardStorage::kMaxPerKind)
                {
                    s.magList[s.magCount++] = &magnetometer;
                }

                LOG_INF("[Board] %s: %s ready (%u Hz)",
                        mount.label, magnetometer.name(), magnetometer.nativeRate_hz());
                break;
            }

            default:
                LOG_WARN("[Board] %s: %s declared but no driver is built in",
                         mount.label, toString(mount.part));
                break;
        }
    }

    // Deliberately Ok even with nothing fitted. Refusing to boot without an IMU
    // would make the spare board — and every bench session on it — useless.
    return Status::Ok;
}

// ── Accessors ───────────────────────────────────────────────────────────────

hal::Clock&     Board::clock()     noexcept { return storage().clock; }
hal::Scheduler& Board::scheduler() noexcept { return storage().scheduler; }
hal::Watchdog&  Board::watchdog()  noexcept { return storage().watchdog; }
hal::System&    Board::system()    noexcept { return storage().system; }
hal::I2cBus&    Board::sensorBus() noexcept { return storage().sensorBus; }
hal::Uart&      Board::rcUart()    noexcept { return storage().rcUart; }
hal::Uart*      Board::telemetryUart() noexcept
{
    return (kBoard.telemetryUart.port != kNoUart) ? &storage().telemetryUart : nullptr;
}
hal::GpioPin&   Board::userButton() noexcept { return storage().userButton; }
device::SettingsStore& Board::settings() noexcept { return storage().settings; }
device::Console&       Board::console()  noexcept { return storage().console; }
device::Indicator* Board::indicator() noexcept
{
    BoardStorage& s = storage();
    return s.indicator ? &*s.indicator : nullptr;
}

device::LogStore& Board::logs() noexcept
{
    BoardStorage& s = storage();
    // Emplaced here rather than in begin(): the store mounts lazily on its own
    // first begin(), and nothing should pay LittleFS mount cost at boot on a
    // build that never logs.
    if (!s.logs) { s.logs.emplace(); }
    return *s.logs;
}
hal::KeyValueStore& Board::configStore() noexcept { return storage().config; }

std::span<device::Sensor* const> Board::sensors() noexcept
{
    BoardStorage& s = storage();
    return { s.sensorList.data(), s.sensorCount };
}

std::span<device::Accelerometer* const> Board::accelerometers() noexcept
{
    BoardStorage& s = storage();
    return { s.accelList.data(), s.accelCount };
}

std::span<device::Gyroscope* const> Board::gyroscopes() noexcept
{
    BoardStorage& s = storage();
    return { s.gyroList.data(), s.gyroCount };
}

std::span<device::Magnetometer* const> Board::magnetometers() noexcept
{
    BoardStorage& s = storage();
    return { s.magList.data(), s.magCount };
}

std::span<device::Barometer* const> Board::barometers() noexcept
{
    BoardStorage& s = storage();
    // baroCount, not baroDriverCount: there are two barometer DRIVERS (BMP280
    // and BMP581) filling one shared list of interface pointers, so the driver
    // slot counters and the span counter are different numbers. Returning a
    // driver counter here made a BMP581-only board report no barometer at all,
    // and a BMP280 board report a leading null.
    return { s.baroList.data(), s.baroCount };
}

Result<hal::Mutex*> Board::allocMutex() noexcept
{
    BoardStorage& s = storage();
    if (s.mutexesUsed >= BoardStorage::kMutexPoolSize)
    {
        LOG_ERR("Board: mutex pool exhausted (%u) - raise kMutexPoolSize",
                static_cast<unsigned>(BoardStorage::kMutexPoolSize));
        return Status::NoSpace;
    }

    hal::esp32::Esp32Mutex& m = s.mutexPool[s.mutexesUsed];
    const Status st = m.begin();
    if (st != Status::Ok) { return st; }

    ++s.mutexesUsed;
    return static_cast<hal::Mutex*>(&m);
}

std::uint8_t Board::pwmOutputCount() const noexcept { return g_storage.pwmCount; }

hal::PwmOut* Board::pwmOutput(std::uint8_t index) noexcept
{
    BoardStorage& s = storage();
    if (index >= s.pwmCount) { return nullptr; }
    return &s.pwm[index];
}

// ── Boot inventory ──────────────────────────────────────────────────────────

void Board::logInventory() const
{
    LOG_INF("[Board] %s (%s)%s", kBoard.name, kBoard.mcu.name,
            kBoard.maturity == BoardMaturity::Untested ? "  *** UNTESTED ***" : "");

    for (std::uint8_t i = 0; i < kBoard.sensorCount; ++i)
    {
        const SensorMount& m = kBoard.sensors[i];
        if (m.bus == BusKind::I2c)
        {
            LOG_INF("[Board]   %-6s %-11s i2c@0x%02X  axes=%s,%s,%s%s",
                    m.label, toString(m.part), m.address,
                    toString(m.axes.x), toString(m.axes.y), toString(m.axes.z),
                    m.axes.isMirrored() ? " (mirrored)" : "");
        }
        else
        {
            LOG_INF("[Board]   %-6s %-11s", m.label, toString(m.part));
        }
    }

    for (std::uint8_t b = 0; b < kBoard.bankCount; ++b)
    {
        const ActuatorBankDesc& bank = kBoard.actuatorBanks[b];
        LOG_INF("[Board]   Outputs %s x%u", toString(bank.transport), bank.outputCount);
        for (std::uint8_t i = 0; i < bank.outputCount; ++i)
        {
            LOG_INF("[Board]     %-14s gpio%d", bank.outputs[i].role,
                    static_cast<int>(bank.outputs[i].pin));
        }
    }

    if (kBoard.telemetryUart.port != kNoUart)
    {
        LOG_INF("[Board]   Telemetry UART%u  rx gpio%d  tx gpio%d",
                static_cast<unsigned>(kBoard.telemetryUart.port),
                static_cast<int>(kBoard.telemetryUart.rx),
                static_cast<int>(kBoard.telemetryUart.tx));
    }

    LOG_INF("[Board]   Heap   %u free (%u min)",
            static_cast<unsigned>(g_storage.system.freeHeapBytes()),
            static_cast<unsigned>(g_storage.system.minFreeHeapBytes()));
    LOG_INF("[Board]   Reset  %s", hal::toString(g_storage.system.resetCause()));
}

} // namespace arduflite::board
