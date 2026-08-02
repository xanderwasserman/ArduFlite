# 01 — Goals, Non-Goals and Design Principles

**Status:** Draft · **Date:** 2026-08-02

---

## 1. Goals

Ordered by priority. When two goals conflict, the higher one wins.

**G1 — The aircraft keeps flying.** Every step of this work ships a firmware that is
at least as safe as the one before it. No phase leaves the tree in a
"works once we finish the next bit" state.

**G2 — Swapping a sensor is a one-file change plus one board-descriptor line.**
Adding an ICM-42688 means writing `drivers/imu/Icm42688.cpp` and naming it in the
board descriptor. No flight code changes, no `#if` added anywhere.

**G3 — Porting to a different microcontroller is confined to one directory.**
`src/hal/platform/<target>/` implements ~12 small interfaces. Nothing above it
changes. This implies **no Arduino type may appear in any HAL or device header**
(`String`, `HardwareSerial`, `Print`, `File`, `Servo`, …).

**G4 — Every driver and every algorithm is testable on a laptop.** A host
implementation of the platform layer means the IMU driver, the CRSF parser, the
mixer, the estimator and the controller all run under GoogleTest with no hardware.
Tests exercise *production code*, never a copy of it.

**G5 — Readable by one person on a Sunday afternoon.** Interfaces are small and
role-named. A reader should be able to answer "where does gyro data come from and
what happens to it" by following four files, not by grepping for `#if`.

**G6 — Hardware mistakes fail at compile time.** Pin collisions, out-of-range GPIOs,
input-only pins used as outputs, and missing peripherals are `static_assert`
failures, not silent bugs found in a field.

**G7 — No regression in real-time behaviour.** The 500 Hz inner loop and 500 Hz IMU
task keep their timing. Loop-stat overrun counts before and after must be equal or
better, measured on hardware, not argued about.

## 2. Non-goals

Explicitly **out of scope** for this work. Naming them keeps the change reviewable.

* **Rewriting `ConfigRegistry` / `ConfigPersistence`.** They stay. The HAL is a
  consumer of config, not a replacement for it.
* **Changing control laws, PID structure, or tuning.** Byte-identical behaviour is
  the target for the cascade controller.
* **Changing telemetry formats or the CSV log schema.** Existing flight logs and the
  `tools/` analysis scripts must keep working.
* **Changing the web UI or REST API.**
* **Supporting a second airframe at runtime**, DMA/interrupt-driven sensor reads,
  or an EKF. The design must not *preclude* these; it does not deliver them.
* **A full dimensional-analysis unit library.** See ADR-008 — deferred, not rejected.

## 3. Design principles

**P1 — Dependencies are visible in signatures.** A class that needs a gyro takes
`device::Gyroscope&` in its constructor. There is no ambient singleton to reach
through. If a constructor takes eight references, that class does too much — the
signature is the design review.

**P2 — Interfaces are roles, not devices — one per *measurement*.** `Accelerometer`,
`Gyroscope`, `Barometer`, `RcLink`, `ActuatorBank` describe what the flight code
*needs*. They are not "the MPU-6500 API with the chip name removed", and they are
never a bundle of quantities that merely share a package: a 6-DOF IMU is a chip that
implements `Accelerometer` **and** `Gyroscope`, so discrete parts and redundant
sensors present the same view. Interface Segregation applies literally — an estimator
that only reads gyro takes `Gyroscope&`, nothing wider.

**P3 — Two tiers, one direction.** *Platform* (bus, pin, clock, task, storage)
knows nothing about flight. *Devices* (IMU, baro, RC link, outputs) are written
against the platform tier only. Flight code is written against the device tier only
and never sees a bus or a pin. Dependencies point downward, always.

**P4 — Policy lives above the driver.** A CRSF decoder produces channel
microseconds and link health. It does not know that channel 5 is ARM. A PWM output
bank produces pulses. It does not know what a V-tail is. Every mapping,
mixing and interpretation decision moves up into the flight layers.

**P5 — Composition at one place, once.** A single `Board` object constructs every
driver at boot, in static storage, and hands out references. No dynamic allocation
after `begin()` returns. No object constructs its own dependencies.

**P6 — Errors are values.** Every fallible operation returns `Status` or
`Result<T>`. Logging is what you do *with* an error, not instead of returning it.

**P7 — The board is data.** Pins, buses, addresses, sensor axis maps and fitted
parts are one `constexpr` descriptor per board, validated at compile time. Exactly
one `#if` in the codebase selects which descriptor is used.

**P8 — Concurrency is explicit and centrally declared.** Task priorities, stack
sizes and periods live in one table, typed. A driver cannot invent a priority.

**P9 — Preserve hard-won behaviour.** Where the current code encodes a lesson
learned in flight (single I2C owner, seqlock fallback, WDT fast-boot to manual),
the new structure must make that lesson *structural* rather than a comment.

---

## 4. Lessons from ArduPilot's `AP_HAL`

ArduPilot's HAL is the reference point the user named. It is a mature design that
has flown millions of hours; the criticisms below are about *fit for this project*,
not about whether it works.

### 4.1 What to take

| Idea | Why | Where it lands here |
|---|---|---|
| **`AP_HAL::Device`** — a bus-agnostic register-access handle, so one driver works over I2C or SPI | Removes the biggest source of driver duplication | `hal::RegisterDevice` (§03) |
| **`hwdef.dat`** — a declarative, per-board hardware description that generates the pin/peripheral header | The single strongest part of AP_HAL. Board porting becomes data entry | `board::BoardDescriptor` (§04) — as a `constexpr` C++ struct rather than a codegen step, because the scale here doesn't justify a generator |
| **`Rotation` naming** — the orthogonal sensor mountings as one named set | Gives the common cases readable names that match community advice | `arduflite::Rotation`, as a subset of the more general `AxisMap` — which the enum alone could not express (ADR-007) |
| **SITL** — implementing the HAL for the host so the whole vehicle runs natively | The single biggest testability win available | `hal::host` platform + log replay (§05) |
| **Backend probe/detect** — drivers report whether the chip actually answered | Distinguishes "not fitted" from "broken" | `Sensor::probe()` returning `Status` |
| **Explicit sensor instances** — support for multiple IMUs by index | Redundancy without redesign | Board exposes a `Span` per measurement interface, sized by the board descriptor (ADR-019) |

### 4.2 What to reject, and what we do instead

| AP_HAL trait | Problem | ArduFlite decision |
|---|---|---|
| **`extern const AP_HAL::HAL& hal;`** — one global god object holding ~18 subsystem pointers, included by nearly every file | Dependencies are invisible; nothing is unit-testable in isolation; any file can touch any peripheral | **No global HAL.** Constructor injection of narrow references (P1). The `Board` composition root is the only object that knows the whole set, and it is not globally reachable from drivers |
| **`AP_HAL::HAL` as a single struct** (uartA…uartI, spi, i2c, gpio, rcin, rcout, scheduler, util, flash, dsp, can…) | Interface Segregation violation; the type must change whenever any peripheral class is added | **Many small interfaces**, no aggregate type. The board exposes accessors, not a struct |
| **`CONFIG_HAL_BOARD` `#if` in ordinary source files** | The thing the HAL was supposed to eliminate leaks back in | **Exactly one `#if`** in the tree, in `board/BoardSelect.h` |
| **`UARTDriver : BetterStream : Print`** — an Arduino inheritance chain inside a "portable" HAL | Ties the portable layer to an Arduino-shaped API | `hal::Uart` is a byte-oriented interface with no `Print`, no `String`, no `Stream` |
| **Fuzzy HAL/driver boundary** — `AP_HAL::Flash` is in the HAL but `AP_InertialSensor_Invensense` is not, yet reaches through `hal.spi` | No clear rule for where a new thing goes | **Explicit two-tier rule** (P3), stated once and enforced by an include-direction CI check |
| **`AP_Param`** — index-based storage with 16-char names and hand-written migration functions | Stored layout is fragile across versions | Not adopted. `ConfigRegistry` already solves this with string keys and schema versioning |
| **`AP_HAL::panic()`** as an error path | Loses the reason; unrecoverable | `Status`/`Result<T>` (P6); panic only where continuing is genuinely unsafe |
| **Heavyweight `detect()` chains** with per-board `#if` and `HAL_INS_PROBE_LIST` macros | Unreadable, and the macro is per-board again | Fitted parts are named in the board descriptor; probing is a short loop over that list |

### 4.3 The one-sentence difference

> AP_HAL abstracts *the platform* behind a global object that everything reaches
> into. ArduFlite abstracts *the platform and the devices* behind small interfaces
> that are handed to the code that needs them.

That single change is what buys G4 (host testability) and G5 (readability), and it
is the reason a HAL of this size is worth building at all for a one-person project.
