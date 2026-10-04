# ArduFlite Hardware Abstraction Layer — Design Specification

**Status:** Draft, awaiting review · **Date:** 2026-08-02 · **Baseline commit:** `9ca8484`

This directory contains the design for replacing ArduFlite's current ad-hoc hardware
handling with a proper hardware abstraction layer.

---

## The goal in one paragraph

Make swapping a sensor a one-file change, make porting to a different
microcontroller a one-directory change, and make every driver and algorithm run
under test on a laptop — without changing what the aircraft does, and without a
single flight-day where the firmware is half-migrated.

## Documents

| # | Document | What it answers |
|---|---|---|
| 00 | [Current State](00-current-state.md) | What exists today, what's wrong with it, with file:line evidence |
| 01 | [Goals and Principles](01-goals-and-principles.md) | Requirements, non-goals, design principles, and what to take from / reject in ArduPilot's AP_HAL |
| 02 | [Architecture](02-architecture.md) | The tier model, directory layout, data flow, and the structural decisions |
| 03 | [Interfaces](03-interfaces.md) | The proposed headers, in code |
| 04 | [Board Descriptors](04-board-descriptors.md) | Declarative board definition, compile-time validation, composition root |
| 05 | [Testing Strategy](05-testing-strategy.md) | The host platform, what becomes testable, log replay, CI |
| 06 | [Migration Plan](06-migration-plan.md) | Ten phases, each shipping a flyable firmware |
| 07 | [Decisions](07-decisions.md) | 22 ADRs with alternatives considered — all settled |
| 08 | [Review](08-review.md) | Adversarial self-review: 15 findings, 9 fixed, 4 accepted, 2 deferred |
| 09 | [C++ Conventions](09-cpp-conventions.md) | The C++20 baseline, verified against the toolchain, and how the HAL uses it |
| 10 | [Diagrams](10-diagrams.md) | Ten Mermaid diagrams — tiers, class models, the 500 Hz tick, calibration, phases |

**Start with [10-diagrams.md](10-diagrams.md)** if you want the shape before the
prose. Then 00 → 01 → 02 for the argument, 03 → 04 → 09 for the design, and 06 before
writing any code. **Read 08 before trusting any of it** — it lists what a critical
pass found wrong, including four issues that would have caused real bugs.

## The design in one diagram

```
   Flight code ──sees only──► arduflite::device
                              (Accelerometer, Gyroscope, Barometer, Gnss,
                               RcLink, ActuatorBank, …  — one per MEASUREMENT)
                                  ▲
                              implemented by
                                  │
                              arduflite::drivers    (Mpu6500, Bmp280, CrsfLink, …)
                                  │
                              uses only
                                  ▼
                              arduflite::hal    (RegisterDevice, Uart, PwmOut, Clock, …)
                                  ▲
                              implemented by
                                  │
                    platform/esp32   ·   platform/host
                                  ▲
                     constructed once by board::Board,
                     configured by a constexpr BoardDescriptor
```

## What this fixes

| Today | After |
|---|---|
| Sensor selection = 22 preprocessor branches inside a 1345-line class, one of them a member declaration | Write one driver file, add one line to the board descriptor |
| Sensor mounting = three unexplained sign flips, compile-time only, no way to change them in the field | One `AxisMap`, overridable at runtime, with an `imu axes` CLI to find the right one |
| Board pins = five `#if BOARD_TYPE` blocks with three live pin collisions and an invalid GPIO, all undetected | One `constexpr` descriptor; those five defects become `static_assert` failures |
| CRSF and PWM receivers share no interface; the app names CRSF concretely in five places | One `RcLink` interface; protocol is a board-descriptor choice |
| Unit tests compile two production files; three test files contain hand-maintained *copies* of the logic they claim to test | Host platform; tests call the real code; real flight logs replay as a regression oracle |
| Fusion filter and PWM output are welded into `ArduFliteIMU` / `ServoManager` | `AttitudeEstimator` and `ActuatorBank` interfaces — swapping the fusion filter or moving off `ledc*()` is a one-line change in `Board.cpp` |
| Accel and gyro are inseparable; no way to express discrete parts or a second sensor | One interface per *measurement*; a 6-DOF chip implements two. Redundancy is a second descriptor line |
| No GNSS, airspeed, rangefinder or power-monitor abstraction at all | All specified; CRSF battery telemetry stops using placeholder values |
| CI compiles the sketch and runs no tests | Host tests, static layering checks, 2×2 build matrix with size budgets |

## Decisions

Settled after inspecting the installed library sources and the call graph:

* **ADR-013** — **Write our own MPU-6500 / MPU-9250 / BMP280 register drivers**;
  retire FastIMU and Adafruit_BMP280. Both bind to `TwoWire&` directly, so a driver
  wrapping them cannot go through `RegisterDevice` — which would leave the two most
  important devices untestable and outside the bus mutex. Risk retired by a bench
  cross-check harness that runs both against the same physical sensor.
* **ADR-014** — **Delete the PWM receiver.** Confirmed unreachable:
  `ArduFlitePwmReceiver` is never instantiated and `runReceiverTest_print()` is
  never called, yet both compile into every build on top of a pin table with an
  out-of-range GPIO and three collisions. `SimRcLink` is the second `RcLink`
  implementation instead.
* **ADR-017** — **Own Madgwick, but after the abstraction lands** (Phase 7, not
  Phase 2). **Done — ADR-056.** Driven by the GPL notice in
  `Adafruit_AHRS_Madgwick.cpp` inside an MIT
  project — not by overhead, which measures at ~0.3–0.8 % of the core. Phase 2 wraps
  it; the rewrite is validated by log replay against the wrapped version.
* **ADR-018** — **Drop ESP32Servo for the core LEDC API** in Phase 1. ~40 lines
  replace 1290 lines of per-chip `#ifdef`s, and AGENTS.md rule 7 favours the native
  core API over a third-party wrapper.

Settled in review:

* **ADR-019** — **One interface per measurement, not per chip.** The first draft's
  combined `ImuSensor` was a device abstraction wearing a role's name; it would have
  blocked both discrete-sensor hardware and redundancy. A `sample()`/`read()` split
  keeps the single 14-byte burst read while exposing `Accelerometer` and `Gyroscope`
  separately.
* **ADR-020** — **`ActuatorBank` must not name a transport.** The first draft leaked
  `minPulse_us`/`frameRate_hz` into the shared config — a CANopen driver would have
  had to ignore four fields and smuggle node IDs in sideways. Now: neutral config,
  `commit()` returning `Status`, optional feedback, and `CompositeActuatorBank` for
  mixed transports.

* **ADR-008** — **Option A: unit suffixes** (`gyro_dps`, `altitude_m`,
  `deadband_rad`), not strong `Radians`/`Degrees` types. Free, low-risk, and it makes
  the one genuinely mixed path visible at the call site. Option B revisited after
  Phase 6. The suffix table is in [07](07-decisions.md) ADR-008.

No decisions are currently blocking. The only judgement call left is optional and
standalone: whether to rename the `att.deadband` config key to `att.deadband_rad`,
which needs an NVS schema migration (ADR-008, last paragraph).

## Is the design complete?

**No — and that is not the goal.** The design is *ready to implement*, which is a
different and more useful property. Three things say so:

1. **Nothing here has been compiled.** Every interface in §03 is plausible C++, not
   verified C++. Phase 0 exists to find out which parts do not survive a compiler,
   and it will find some.
2. **The last three reviews each found something real** — heterogeneous actuators
   (ADR-024), the `Actuator` / `ActuatorBank` split (ADR-025), and a stale board
   descriptor that the split had invalidated. The rate of finding genuine problems
   has not dropped off.
3. **But they were found by *questions*, not by more prose.** At 5,400 lines the
   spec is past the point where writing more of it helps. A compiler and a bench will
   now find issues faster than another review pass.

**Deliberately left unspecified**, to be settled when they are written against real
call sites in Phases 3–6:

| Component | Referenced | Why deferred |
|---|---|---|
| `actuators::AirframeMixer` | 15× | Flight layer. Shape is constrained by `ControlOutputs` (§03 3.3); the geometry maths already exists and ports verbatim |
| `input::RcMapper` | 12× | Flight layer. Settled in Phase 4 against the real CRSF channel table |
| `estimation::MotionDetector` | 8× | Ports verbatim from `updateMotionSignals()` |
| `estimation::SensorSelector` | 7× | Phase 6 ships the trivial policy; the interesting version is blocked on R14 |

These are all *above* the HAL boundary. Specifying them now would guess at call sites
that do not exist yet.

## Known weak points

From [08-review.md](08-review.md), carried forward rather than hidden:

* **Sensor failover is not solved** (R14). Switching instances mid-flight hands the
  estimator a step change at the worst possible moment. Unreachable while only one
  sensor is fitted; must be designed before any redundant hardware is added.
* **Nothing has been compiled.** Every interface in §03 is plausible C++, not verified
  C++. Expect Phase 0 to force corrections — update the spec rather than letting it
  diverge from the code on day one.
* **The spec is ~3900 lines.** Its job is to be read before coding. Prefer sharpening
  over adding from here.

## Toolchain facts worth knowing up front

Both checked against the installed toolchain and your own build artefacts, because
both contradicted an assumption in an earlier draft:

* **You are already on C++20.** `esp32c3-libs/3.3.10/flags/cpp_flags` resolves to
  `-std=gnu++2a`. Host tests are C++17 — a mismatch Phase 0 fixes. This removes the
  need for a bespoke `Span`, and lets `SeqLock` enforce trivial-copyability by
  concept (§09).
* **Exceptions are enabled** (`-fexceptions`), not disabled. `Result<T>` is still
  right — unbounded unwind latency in a 500 Hz loop — but ADR-009's original
  justification was wrong and has been corrected.
* **The ESP32-C3 has no FPU.** Confirmed from `build/lolin-full/ArduFlite.ino.map`:
  `rv32imc` + soft-float ABI + `__addsf3`/`__mulsf3` linked. AGENTS.md and a comment
  in `ArduFliteIMU.cpp` both say otherwise; Phase 8 corrects them (§00 2.9b).

## Before writing any code

Record the hardware baseline described at the top of
[06-migration-plan.md](06-migration-plan.md) and commit it as `specs/hal/baseline.md`.
Phase 1's exit criterion is a comparison against it, and there is no way to
reconstruct it afterwards.

## Related documents in the repo

* [`AGENTS.md`](../../AGENTS.md) — current architecture rules. Sections "Module
  Boundaries" and "Folder Structure Overview" are rewritten in Phase 6.
* [`docs/CONFIG_REFERENCE.md`](../../docs/CONFIG_REFERENCE.md) — the runtime config
  system, which this design deliberately leaves alone (ADR-015).
* [`docs/flight_logs/`](../../docs/flight_logs/) — the CSV corpus used as the log
  replay oracle in [05](05-testing-strategy.md) §7.
