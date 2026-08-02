# 04 — Board Descriptors and the Composition Root

**Status:** Draft · **Date:** 2026-08-02

Replaces `include/PinConfiguration.h`, `include/IMUConfiguration.h`, the board
`#if` blocks in `ArdufliteApp.cpp`, and the pin halves of
`include/ReceiverConfiguration.h` / `include/CSRFConfiguration.h`.

---

## 1. Problem being solved

Today, adding a board means editing five `#if BOARD_TYPE ==` blocks across
`PinConfiguration.h` plus application code, with no validation. Section 00 §2.4
lists five real defects that survive in the current Wemos table because nothing
checks it.

The fix: **one `constexpr` struct per board, validated by `static_assert`, and
exactly one `#if` in the codebase that picks which one is compiled.**

## 2. The descriptor schema

```cpp
// src/hal/board/BoardDescriptor.h
namespace arduflite::board {

using Pin = int8_t;
constexpr Pin kNoPin = -1;

// ── What the MCU can physically do ──────────────────────────────────────────
struct McuProfile {
    const char* name;
    Pin         minGpio;
    Pin         maxGpio;
    uint32_t    inputOnlyMask;      // bit N set → GPIO N cannot be an output
    uint32_t    reservedMask;       // bit N set → flash/PSRAM/strapping, do not use
    uint8_t     uartCount;
    uint8_t     pwmChannelCount;
};

// ── Peripherals ─────────────────────────────────────────────────────────────
struct I2cBusDesc  { Pin sda, scl; uint32_t clock_hz; };
struct UartDesc    { uint8_t port; Pin rx, tx; uint32_t baud; bool invertRx; };
struct GpioDesc    { Pin pin; hal::PinMode mode; const char* role; };
// ── Actuator outputs ────────────────────────────────────────────────────────
// Updated for ADR-024/025: outputs are grouped BY TRANSPORT (one group per
// ActuatorBank), and each output carries its role and kind. The previous flat
// `PwmOutDesc outputs[]` could not express a CAN throttle or a latching release.
enum class ActuatorTransport : uint8_t { Pwm, CanOpen, DShot, Sim };

struct ActuatorOutputDesc {
    const char*  role;        // "elevator" — must be unique across the WHOLE board
    ActuatorKind kind = ActuatorKind::Proportional;
    Pin          pin    = kNoPin;   // PWM/DShot only
    uint8_t      nodeId = 0;        // CANopen only
};

struct ActuatorBankDesc {
    ActuatorTransport transport;
    uint8_t           busIndex = 0;      // which CanBus, when transport == CanOpen
    static constexpr uint8_t kMaxPerBank = 8;
    ActuatorOutputDesc outputs[kMaxPerBank];
    uint8_t            outputCount;
};
struct LedDesc     { Pin pin; uint16_t pixelCount; uint8_t brightness; };

// ── Fitted parts ────────────────────────────────────────────────────────────
// One entry per PHYSICAL CHIP. A part that measures several quantities (an MPU-6500
// measures acceleration and angular rate) appears once and is bound to several
// device interfaces by Board.cpp — see 03-interfaces.md §3.1.
enum class SensorPart : uint8_t {
    None,
    Mpu6500, Mpu9250,          // Accelerometer + Gyroscope (+ Magnetometer on 9250)
    Bmp280,                    // Barometer
    UbloxGnss,                 // Gnss
    Ina226,                    // PowerMonitor
    Sim,                       // host/sim stand-in
};

enum class BusKind : uint8_t { I2c, Spi, Uart, None };
enum class RcPart  : uint8_t { None, Crsf, Sim, Unknown };

// Review finding R4: an in-progress board port must be able to compile with pins
// it does not know yet. `Untested` relaxes the "fitted parts must be wired"
// assertion and makes the board log a loud warning at boot. It is not a way to
// silence validation permanently — nothing may be released as Untested.
enum class BoardMaturity : uint8_t { Supported, Untested };

// `axes` is the default sensor→body alignment; ConfigRegistry can override it at
// runtime (see 03-interfaces.md §1.4). AxisMap rather than Rotation because the
// current aircraft needs a mirrored map, which no proper rotation can express.
// It is PER SENSOR INSTANCE, not per board: on custom hardware a discrete gyro and
// a discrete accelerometer can be mounted at different angles.
struct SensorMount {
    SensorPart part;
    BusKind    bus;
    uint8_t    address;        // I2C address, SPI CS index, or UART port
    AxisMap    axes;           // ignored by non-vector sensors (baro, GNSS, power)
    const char* label;         // "imu0", "imu1" — appears in logs and CLI
};

// ── The board ───────────────────────────────────────────────────────────────
struct BoardDescriptor {
    const char*       name;
    BoardMaturity     maturity;
    McuProfile        mcu;

    I2cBusDesc        sensorBus;
    UartDesc          rcUart;
    UartDesc          consoleUart;

    // A LIST, so redundant sensors are a data change rather than a code change.
    // Two MPU-6500s at 0x68 and 0x69 is just two entries.
    static constexpr uint8_t kMaxSensors = 8;
    SensorMount       sensors[kMaxSensors];
    uint8_t           sensorCount;

    RcPart            rcLink;

    // One entry per ActuatorBank. Board.cpp constructs one bank per entry and
    // wraps them in a CompositeActuatorBank when bankCount > 1 (§03 3.3).
    static constexpr uint8_t kMaxBanks = 3;
    ActuatorBankDesc  actuatorBanks[kMaxBanks];
    uint8_t           bankCount;

    GpioDesc          userButton;
    LedDesc           statusLed;
};

}   // namespace arduflite::board
```

Every pin the firmware will ever touch is now in one struct, enumerable at compile
time. That is what makes validation possible.

## 3. A board definition

```cpp
// src/hal/board/boards/lolin_c3_mini.h
#pragma once
#include "src/hal/board/BoardDescriptor.h"

namespace arduflite::board {

constexpr McuProfile kEsp32C3 {
    .name            = "ESP32-C3",
    .minGpio         = 0,
    .maxGpio         = 21,
    .inputOnlyMask   = 0,
    // PLACEHOLDER — verify against the ESP32-C3 datasheet and the specific module
    // before relying on it. Getting this wrong makes validation either useless
    // (mask too small) or a false blocker (mask too large).
    .reservedMask    = 0x0003F000,     // GPIO12–17: SPI flash on most C3 modules
    .uartCount       = 2,
    .pwmChannelCount = 6,
};

constexpr BoardDescriptor kBoard {
    .name     = "Lolin C3 Mini",
    .maturity = BoardMaturity::Supported,
    .mcu      = kEsp32C3,

    .sensorBus   = { .sda = 3, .scl = 5, .clock_hz = 400000 },
    .rcUart      = { .port = 1, .rx = 6, .tx = 8, .baud = 420000, .invertRx = false },
    .consoleUart = { .port = 0, .rx = kNoPin, .tx = kNoPin, .baud = 115200, .invertRx = false },

    .sensors = {
        // Mirrored map (det = -1) — the transform the prototype currently flies
        // with, ported verbatim from applyOrientation(). See 00 §2.3.
        { SensorPart::Mpu6500, BusKind::I2c, 0x68,
          { SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ }, "imu0" },
        { SensorPart::Bmp280,  BusKind::I2c, 0x76, {}, "baro0" },
    },
    .sensorCount = 2,

    .rcLink = RcPart::Crsf,

    .actuatorBanks = {{
        .transport = ActuatorTransport::Pwm,
        .outputs = {
            { .role = "aileron_right", .pin = 1  },
            { .role = "aileron_left",  .pin = 2  },
            { .role = "elevator",      .pin = 0  },
            { .role = "rudder",        .pin = 4  },
            { .role = "throttle",      .pin = 10 },
        },
        .outputCount = 5,
    }},
    .bankCount = 1,

    .userButton = { 9, hal::PinMode::InputPullUp, "user" },
    .statusLed  = { 7, 1, 50 },
};

}   // namespace arduflite::board
```

Note what is now visible in one screen: the mount axis map, the I2C addresses, the
output roles, the LED pin (currently hardcoded in `ArdufliteApp.cpp`). And note the
PWM-input pins are simply gone — this board uses CRSF, so the dead, conflicting
`PwmInputConfig` table has no place to live.

## 4. Compile-time validation

```cpp
// src/hal/board/BoardValidate.h
namespace arduflite::board::validate {

constexpr bool inRange(const McuProfile& m, Pin p)
    { return p == kNoPin || (p >= m.minGpio && p <= m.maxGpio); }

constexpr bool notReserved(const McuProfile& m, Pin p)
    { return p == kNoPin || ((m.reservedMask >> p) & 1u) == 0u; }

constexpr bool canOutput(const McuProfile& m, Pin p)
    { return p == kNoPin || ((m.inputOnlyMask >> p) & 1u) == 0u; }

// Collects every assigned pin into a fixed array, then checks pairwise uniqueness.
constexpr bool allPinsUnique(const BoardDescriptor& b);
constexpr bool allPinsValid (const BoardDescriptor& b);
constexpr bool allOutputPinsCanOutput(const BoardDescriptor& b);
constexpr bool outputCountWithinMcu   (const BoardDescriptor& b);   // PWM banks vs LEDC channels
constexpr bool allRolesUnique         (const BoardDescriptor& b);   // across ALL banks
// "Every fitted part has a bus and pins." Skipped for Untested boards (R4) —
// a port in progress must be able to compile before every pin is known.
constexpr bool requiredPeripheralsPresent(const BoardDescriptor& b);

constexpr bool isValid(const BoardDescriptor& b) {
    return allPinsValid(b) && allPinsUnique(b) && allOutputPinsCanOutput(b)
        && outputCountWithinMcu(b) && allRolesUnique(b)
        && requiredPeripheralsPresent(b);
}

}   // namespace arduflite::board::validate

// Instantiated once, in BoardSelect.h:
static_assert(arduflite::board::validate::allPinsValid(arduflite::board::kBoard),
              "Board descriptor: a pin is outside the MCU's GPIO range or is reserved");
static_assert(arduflite::board::validate::allPinsUnique(arduflite::board::kBoard),
              "Board descriptor: the same GPIO is assigned to two functions");
static_assert(arduflite::board::validate::allOutputPinsCanOutput(arduflite::board::kBoard),
              "Board descriptor: an input-only GPIO is assigned to an output");
static_assert(arduflite::board::validate::outputCountWithinMcu(arduflite::board::kBoard),
              "Board descriptor: more PWM outputs than the MCU has channels");
static_assert(arduflite::board::validate::allRolesUnique(arduflite::board::kBoard),
              "Board descriptor: two outputs claim the same role — byRole() would be ambiguous");
static_assert(arduflite::board::validate::requiredPeripheralsPresent(arduflite::board::kBoard),
              "Board descriptor: a fitted part has no bus or pin assigned");
```

Applied to today's `BOARD_TYPE_WEMOS` table, `allPinsValid` fails on GPIO 32 and
`allPinsUnique` fails three times. All five defects from §00 2.4 become build
errors with a message that names the rule.

## 5. Board selection — the only `#if`

```cpp
// src/hal/board/BoardSelect.h
#pragma once

#if   defined(ARDUFLITE_BOARD_LOLIN_C3_MINI)
#  include "src/hal/board/boards/lolin_c3_mini.h"
#elif defined(ARDUFLITE_BOARD_FIREBEETLE_ESP32E)
#  include "src/hal/board/boards/firebeetle_esp32e.h"
#elif defined(ARDUFLITE_BOARD_HOST_SIM)
#  include "src/hal/board/boards/host_sim.h"
#else
#  error "No board selected. Define ARDUFLITE_BOARD_<NAME> (see specs/hal/04-board-descriptors.md)"
#endif

#include "src/hal/board/BoardValidate.h"
// static_asserts from §4 live here
```

`build.sh` gains `-DARDUFLITE_BOARD_LOLIN_C3_MINI` alongside the existing flags. A CI
check greps for `BOARD_TYPE` and `#if.*BOARD` outside `board/` and fails the build
if any reappear.

## 6. Composition root

`Board::begin()` is the one place that knows concrete types. It is ~150 readable
lines of "construct, probe, report".

```cpp
// src/hal/board/Board.cpp   (sketch)
Status Board::begin()
{
    constexpr auto& d = kBoard;

    // ── Tier 0 ──────────────────────────────────────────────────────────────
    ARDUFLITE_TRY(_i2c.begin(d.sensorBus.clock_hz));
    ARDUFLITE_TRY(_rcUart.begin(d.rcUart.baud, d.rcUart.invertRx));
    ARDUFLITE_TRY(_console.begin(d.consoleUart.baud));
    ARDUFLITE_TRY(_nvs.begin());
    ARDUFLITE_TRY(_fs.begin());

    for (uint8_t b = 0; b < d.bankCount; ++b) {
        const auto& bank = d.actuatorBanks[b];
        for (uint8_t i = 0; i < bank.outputCount; ++i) {
            if (bank.outputs[i].pin != kNoPin) ARDUFLITE_TRY(_pwm[i].bind(bank.outputs[i].pin));
        }
    }

    // ── Tier 1: sensors ─────────────────────────────────────────────────────
    // One loop over the descriptor. Each chip is constructed once, then bound to
    // every measurement interface it provides (§03 3.1).
    for (uint8_t i = 0; i < d.sensorCount; ++i) {
        const SensorMount& m = d.sensors[i];
        device::Sensor* dev = nullptr;

        switch (m.part) {
        case SensorPart::Mpu6500: {
            auto handle = openBus(m);                       // I2C / SPI / UART
            if (!handle) { logSensorFailure(m, handle.status()); continue; }
            _mpu6500.emplace(*handle.value(), m.axes);
            dev = &*_mpu6500;
            _accelerometers.add(&*_mpu6500);                // provides 2 interfaces
            _gyroscopes.add(&*_mpu6500);
            break;
        }
        case SensorPart::Mpu9250: /* … + _magnetometers.add(...) */ break;
        case SensorPart::Bmp280: {
            auto handle = openBus(m);
            if (!handle) { logSensorFailure(m, handle.status()); continue; }
            _bmp280.emplace(*handle.value());
            dev = &*_bmp280;
            _barometers.add(&*_bmp280);
            break;
        }
        case SensorPart::UbloxGnss: /* … _gnss = &*_ublox; */ break;
        case SensorPart::Ina226:    /* … _powerMonitor = &*_ina226; */ break;
        case SensorPart::Sim:       /* … host/sim stand-ins */ break;
        case SensorPart::None:      continue;
        }

        if (dev == nullptr) continue;

        const Status s = dev->probe();
        if (s != Status::Ok) {
            // Degrade, do not hang. A missing baro costs altitude, not control;
            // a missing gyro is caught by the preflight check at arm time.
            LOG_ERR("Sensor %s (%s) probe failed: %s", m.label, dev->name(), toString(s));
            unbind(dev);          // remove from every list it was added to
            continue;
        }
        // NOT ARDUFLITE_TRY — that would abort the whole board on one bad sensor,
        // contradicting the independent-degradation promise below (review R8).
        // General hazard: ARDUFLITE_TRY inside a loop is almost always wrong.
        const Status b = dev->begin();
        if (b != Status::Ok) {
            LOG_ERR("Sensor %s begin failed: %s", m.label, toString(b));
            unbind(dev);
            continue;
        }
        _sensors.add(dev);  // the sampling task's iteration list
    }

    // ── Tier 1: RC link, outputs, LED … same shape ──────────────────────────

    _outputs.begin(outputConfigsFromRegistry(d), d.outputCount);

    logInventory();                     // prints the fitted-parts table at boot
    return Status::Ok;
}
```

Two properties worth calling out:

* **`emplace` into `std::optional` members**, not `new`. Storage is inside the
  `Board` object, sized at compile time. No heap after boot (§02 4.2).
* **Each sensor fails independently.** A dead barometer removes `Barometer` from
  the list and costs altitude; it does not stop the gyro from flying the aircraft.
  Today `ArdufliteApp.cpp:233` does `while (1);` when the IMU fails. With `Status`, the caller can distinguish
  `NotPresent` from `IoError` and choose: on a normal boot, refuse to arm; on a
  watchdog recovery, go straight to `MANUAL_MODE` (the existing behaviour, now
  expressible without a special case).

### Boot inventory

`logInventory()` prints one table at startup, which becomes the first thing in every
flight log:

```
[Board] Lolin C3 Mini (ESP32-C3)
[Board]   imu0    MPU-6500  i2c0@0x68  axes=+X,-Y,+Z (mirrored)  who_am_i=0x70  OK
[Board]           └─ provides: Accelerometer(4g), Gyroscope(500dps)
[Board]   baro0   BMP280    i2c0@0x76  25 Hz               OK
[Board]           └─ provides: Barometer
[Board]   RC      CRSF      uart1 rx=6 tx=8 @420000       OK
[Board]   Outputs PWM x5                                 OK
[Board]   Out[0]  aileron_right  gpio1
[Board]   Out[1]  aileron_left   gpio2
[Board]   Out[2]  elevator       gpio0
[Board]   Out[3]  rudder         gpio4
[Board]   Out[4]  throttle       gpio10
[Board]   LED     WS2812    gpio7 x1                      OK
[Board]   Store   LittleFS  1.9 MB (12% used)             OK
```

Post-crash triage currently requires knowing which build flags were used. This makes
the answer part of the artefact.

## 7. Runtime vs compile-time: the rule

The current split is convention-only (§00 2.5). The new rule is mechanical:

| Kind of thing | Where | Why |
|---|---|---|
| Pin numbers, bus assignments, fitted parts, I2C addresses, MCU limits | **Board descriptor** (compile-time) | Changing them means a different physical board. `static_assert`-able |
| Sensor axis map + alignment trim | **Board descriptor default, overridable by `ConfigRegistry`** | Usually fixed by the PCB — but a re-mount, a replacement module, or a clone with odd axis signs must be fixable without a rebuild |
| Servo endpoints, neutral, deflection, inversion, slew rate | **`ConfigRegistry`** (runtime, NVS) | Differs per airframe and per servo; tuned in the field. Unchanged from today |
| Airframe geometry (`WingDesign`), dual ailerons | **`ConfigRegistry`** | Same board flies different airframes. Unchanged from today |
| PID gains, mixer limits, failsafe attitudes | **`ConfigRegistry`** | Unchanged from today |
| Calibration offsets | **`SettingsStore`** (NVS blob + CRC) | Was raw EEPROM; now one persistence mechanism |

`ActuatorBank::begin()` therefore receives configs assembled from *both*: pins from
the descriptor, everything else from the registry. That assembly happens in
`Board.cpp` and nowhere else — the split stops leaking into `ServoManager`.

## 8. Adding a board — the checklist

1. Create `src/hal/board/boards/<name>.h` with an `McuProfile` and a
   `BoardDescriptor`.
2. Add one `#elif defined(ARDUFLITE_BOARD_<NAME>)` to `BoardSelect.h`.
3. Add the board to `build.sh`'s case statement and to the CI build matrix.
4. Build. Fix whatever the `static_assert`s tell you.

No other file changes. Compare with today: five `#if` blocks in
`PinConfiguration.h`, plus `ArdufliteApp.cpp`, plus no validation.

## 9. Adding a sensor — the checklist

1. Write `src/hal/drivers/<kind>/<Part>.{h,cpp}` implementing `device::Sensor`
   plus **one interface per quantity it measures** (`Accelerometer`, `Gyroscope`, …),
   against `hal::RegisterDevice`. `sample()` does the bus work; each `read()` returns
   cached data.
2. Write `tests/unit/test_<part>_driver.cpp` against `host::FakeRegisterDevice`.
3. Add one enumerator to `SensorPart` and one `case` to the loop in `Board.cpp`,
   registering it in each interface list it belongs to.
4. Add one line to the board descriptor's `sensors[]`.

No flight code changes, no new `#if`.

### Adding a *redundant* sensor

Steps 1–3 are already done. Add a second `sensors[]` entry with a different address
and label:

```cpp
{ SensorPart::Mpu6500, BusKind::I2c, 0x68, {...}, "imu0" },
{ SensorPart::Mpu6500, BusKind::I2c, 0x69, {...}, "imu1" },   // AD0 pulled high
```

`accelerometers()` and `gyroscopes()` now return two entries each.
`estimation::SensorSelector` picks the healthy one. No interface changes, no driver
changes, no flight-code changes.

### Adding a sensor on custom hardware with discrete parts

A board with a standalone accelerometer and a standalone gyroscope is two `sensors[]`
entries implementing one interface each, with **independent** `axes` maps. Nothing
above the board layer can tell it apart from a single 6-DOF part — which is the whole
reason the interfaces are split per measurement rather than per chip.

## 10. Adding a non-PWM actuator — the checklist

1. Implement `device::ActuatorBank` for the transport (e.g. `CanOpenActuatorBank`),
   taking its transport-specific config in the constructor — never in
   `ActuatorChannelConfig`.
2. If it needs a new bus, add the Tier 0 interface (`hal::CanBus` is already
   specified in §03 2.3) and its ESP32 implementation.
3. Construct it in `Board.cpp`; wrap it and the PWM bank in a
   `CompositeActuatorBank` if the aircraft uses both.
4. Add the descriptor group (e.g. `canOutputs[]`).

`AirframeMixer`, `ArduFliteController`, the failsafe path and telemetry are
untouched — they only ever saw `ActuatorBank`. See §03 3.3 for the honest scoping
of what a CANopen stack actually costs beyond the interface.
