/**
 * Board.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The composition root — the ONE place that knows concrete driver types.
 *
 * This header deliberately names NO driver type. The concrete members live in
 * BoardStorage (Board_Internal.h), which only Board.cpp includes. Without that
 * split, every consumer of Board.h would transitively acquire every driver
 * header and the layering check in tools/ci/check_layering.sh would be theatre.
 * See specs/hal/08-review.md R3.
 *
 * @note Only arduflite_init() should touch this. Everything else receives
 *       interface references injected into its constructor. That is the
 *       difference from AP_HAL's global `hal` object: the singleton exists, but
 *       exactly one function reaches into it.
 */
#ifndef ARDUFLITE_HAL_BOARD_BOARD_H
#define ARDUFLITE_HAL_BOARD_BOARD_H

#include <span>

#include "src/hal/board/BoardSelect.h"
#include "src/hal/device/Peripherals.h"
#include "src/hal/device/Sensor.h"
#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Status.h"
#include "src/hal/platform/Buses.h"
#include "src/hal/platform/Clock.h"
#include "src/hal/core/Result.h"
#include "src/hal/platform/Io.h"
#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/Storage.h"
#include "src/hal/platform/Watchdog.h"

namespace arduflite::board {

struct BoardStorage;   // defined in Board_Internal.h, seen only by Board.cpp

class Board : private NonCopyable
{
public:
    static Board& instance() noexcept;

    /// Construct and start Tier 0. Idempotent.
    /// A failure here means a bus did not come up — that is fatal in a way a
    /// missing sensor is not.
    Status begin();

    [[nodiscard]] bool started() const noexcept { return _started; }

    // ── Tier 0 ──────────────────────────────────────────────────────────────
    [[nodiscard]] hal::Clock&     clock()     noexcept;
    [[nodiscard]] hal::Scheduler& scheduler() noexcept;
    [[nodiscard]] hal::Watchdog&  watchdog()  noexcept;
    [[nodiscard]] hal::System&    system()    noexcept;
    [[nodiscard]] hal::I2cBus&    sensorBus() noexcept;

    /// The RC UART. Shared: the CRSF receiver reads it and CRSF telemetry writes
    /// it, on the same port with separate pins. Board owning it makes that
    /// explicit rather than a comment in ArdufliteApp.cpp.
    [[nodiscard]] hal::Uart&      rcUart()    noexcept;

    /// The MAVLink radio UART, or nullptr when the board declares none. Not
    /// started by begin(): its baud is runtime configuration (mav.uart.baud).
    [[nodiscard]] hal::Uart*      telemetryUart() noexcept;

    /// PWM outputs in board-descriptor order. Phase 3 wires these to an
    /// ActuatorBank; nothing consumes them yet.
    [[nodiscard]] hal::PwmOut* pwmOutput(std::uint8_t index) noexcept;
    [[nodiscard]] std::uint8_t pwmOutputCount() const noexcept;

    [[nodiscard]] hal::GpioPin& userButton() noexcept;

    /// Calibration blobs, CRC-protected.
    [[nodiscard]] device::SettingsStore& settings() noexcept;

    /// The USB serial console: log output, and the CLI's input.
    [[nodiscard]] device::Console& console() noexcept;

    /// Flight-log storage. Mounted lazily by its first begin().
    [[nodiscard]] device::LogStore& logs() noexcept;

    /// Status LED, or nullptr when the board has none. Callers must check —
    /// board differences live in the descriptor, not in preprocessor blocks,
    /// were always true and so drove a pixel on a board that has no pixel.
    [[nodiscard]] device::Indicator* indicator() noexcept;

    /// Runtime configuration store, backing ConfigPersistence.
    [[nodiscard]] hal::KeyValueStore& configStore() noexcept;

    /**
     * @name Sensors
     *
     * Spans, not single pointers, because redundancy must not require an
     * interface change (ADR-019/ADR-026). Today every span holds 0 or 1 entry;
     * a board fitting two IMUs changes only its descriptor and Board.cpp.
     *
     * **An empty span is a supported state, not an error.** The spare bench
     * board has no IMU and the firmware must still boot to a usable CLI.
     *
     * Entries are measurement interfaces rather than concrete drivers, so
     * nothing upstream of the HAL names a chip.
     * @{
     */

    /// Everything that needs sample() called on it once per tick. A part
    /// providing several measurements (the MPU-6500 provides two) appears
    /// ONCE here and once in each measurement span.
    [[nodiscard]] std::span<device::Sensor* const>        sensors()        noexcept;

    [[nodiscard]] std::span<device::Accelerometer* const> accelerometers() noexcept;
    [[nodiscard]] std::span<device::Gyroscope* const>     gyroscopes()     noexcept;
    [[nodiscard]] std::span<device::Magnetometer* const>  magnetometers()  noexcept;
    [[nodiscard]] std::span<device::Barometer* const>     barometers()     noexcept;
    /** @} */



    /**
     * @brief Hand out a mutex from a fixed pool.
     *
     * Flight-layer classes cannot construct an Esp32Mutex — that would put a
     * platform type in flight code and the layering check would reject it. The
     * composition root owns them instead, in static storage (ADR-011: no heap
     * after boot).
     *
     * @return Status::NoSpace once the pool is exhausted, which is a
     *         compile-time-fixable bug rather than a runtime condition.
     */
    [[nodiscard]] Result<hal::Mutex*> allocMutex() noexcept;

    /// The compile-time description this board was built from.
    [[nodiscard]] const BoardDescriptor& descriptor() const noexcept { return kBoard; }

    /// Log the fitted-parts table. First thing in every flight log.
    void logInventory() const;

private:
    Board() noexcept;

    /// Probes and configures every sensor the descriptor declares.
    Status beginSensors();

    BoardStorage& storage() noexcept;

    bool _started = false;
};

} // namespace arduflite::board

#endif // ARDUFLITE_HAL_BOARD_BOARD_H
