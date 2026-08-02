# 07 — Architecture Decision Records

**Status:** Draft · **Date:** 2026-08-02

Each record states the decision, the alternatives that were genuinely considered,
and why they lost. All 20 are settled. ADR-008, ADR-013 and ADR-014 were open in
the first draft; ADR-007, ADR-019 and ADR-020 **correct design errors in it** and
say so explicitly, because a spec that quietly rewrites its own history is worse
than one that shows where it was wrong.

---

## ADR-001 — Dependency injection, not a global HAL object

**Decision:** narrow interfaces passed by reference into constructors. No
`extern const HAL& hal`.

**Alternatives:**
* *AP_HAL-style global `hal` struct.* Rejected: it is the direct cause of AP_HAL's
  untestability, and it hides dependencies (§01 4.2). It is also the single thing
  the user asked to improve on.
* *Service locator / registry lookup by name.* Rejected: runtime failure mode,
  no compile-time checking, and it just relocates the global.

**Consequence:** `arduflite_init()` gets longer and more explicit — every wiring
decision is visible in one function. That is a feature.

**Status:** Accepted.

---

## ADR-002 — Virtual dispatch rather than templates/CRTP

**Decision:** pure-virtual interfaces at both tiers.

**Alternatives:**
* *CRTP / policy templates.* Zero dispatch cost, but: every consumer becomes a
  template, error messages become unreadable, compile time rises, and swapping a
  sensor at *runtime* (from the board descriptor) becomes impossible. For a project
  whose stated goal is readability, this trade is wrong.
* *C-style function-pointer vtables.* Same cost as virtual, worse ergonomics.
* *Hybrid — templates for hot paths, virtual elsewhere.* Rejected as a starting
  point (two idioms to learn), but preserved as the escape hatch: only
  `RegisterDevice` is called more than once per tick, and it can be templated later
  without changing anything above it.

**Consequence:** ~2 KB flash, ~160 B RAM, ~0.006 % CPU (§02 4.1). **Verified in
Phase 2 against a recorded baseline, not assumed.**

**Status:** Accepted, with a measurement gate.

---

## ADR-003 — Two tiers (platform / device), not one

**Decision:** `hal::` for MCU primitives, `device::` for device roles. Flight code sees
only `device::`.

**Alternatives:**
* *One tier* (flight code talks to buses). Rejected: that is the current situation.
* *Three tiers* (adding a separate "sensor backend" layer between driver and
  device, AP_InertialSensor-style). Rejected: at this scale it adds indirection
  without adding capability. Multi-sensor redundancy is handled by exposing lists of
  each measurement interface (ADR-019) — no new tier needed.

**Status:** Accepted.

---

## ADR-004 — `RegisterDevice` as the driver-facing bus abstraction

**Decision:** drivers hold a `RegisterDevice*`, not an `I2cBus*` plus an address.

**Rationale:** the same MPU-6500 driver then works over SPI on a future board with
no change. This is the one part of `AP_HAL::Device` worth copying wholesale.

**Consequence:** the bus mutex is owned by the bus and exposed via
`RegisterDevice::busLock()`, so multi-transaction atomicity is expressible without
the driver knowing which bus it is on.

**Status:** Accepted.

---

## ADR-005 — Airframe mixing moves out of the actuator layer

**Decision:** `ActuatorBank` is a flat indexed bank. `AirframeMixer` sits above it
in the control layer.

**Alternatives:**
* *Keep mixing in the output driver* (status quo — `ServoManager`). Rejected: it
  hardcodes six servos, blocks flaps/second-motor/6-surface airframes, and makes the
  mixing maths untestable, which is why `test_servo_math.cpp` contains a copy of it.

**Note on AGENTS.md:** the existing rule "ServoManager owns all geometry-specific
mixing" was written to prevent mixing logic being duplicated in the controller. That
intent is preserved — there is still exactly one place mixing happens
(`AirframeMixer`); it just is not the hardware driver. **AGENTS.md must be updated
in Phase 3, not silently violated.**

**Status:** Accepted.

---

## ADR-006 — RC channel semantics move out of the protocol driver

**Decision:** `RcLink` yields channel microseconds plus link health. Role mapping,
tri-state thresholds and callbacks live in `input::RcMapper`.

**Rationale:** today the CRSF driver directly calls `CRSFCallbacks::onArm` via a
function pointer stored in `ChannelConfig`. A protocol decoder should not know what
channel 5 means. This is also what makes CRSF and PWM genuinely interchangeable.

**Consequence:** `CSRFConfiguration.h`'s channel table becomes `RcMapper`
configuration and can later be moved to `ConfigRegistry` for field remapping.

**Status:** Accepted.

---

## ADR-007 — Sensor alignment as a signed axis map, not a rotation enum

**Decision:** a three-layer `AxisTransform` (03 §1.4): a signed `AxisMap` covering
all 48 axis-aligned mappings, an optional small-angle `AlignmentTrim`, and separate
`applyMeasurement()` / `applyAngularRate()` entry points. `Rotation` remains as a
named convenience covering the 24 proper rotations. Board descriptor supplies the
default; `ConfigRegistry` overrides it at runtime.

**Alternatives:**
* *`Rotation` enum only (24 proper rotations), as in an earlier draft of this spec.*
  **Rejected — it cannot express the transform the aircraft currently flies with.**
  `applyOrientation()`'s accel transform has determinant −1 (§00 2.3); a proper
  rotation cannot produce it. Adopting a rotation-only model would have forced a
  behaviour change on a working aircraft.
* *Free 3×3 matrix only.* Rejected as the primary interface: not `constexpr`-
  friendly to write by hand, not validatable, and it invites non-orthogonal values.
  It survives as the internal representation.
* *Keep the hand-written sign flips.* Rejected: no runtime override, no rule for a
  future reader, and the mag line is already wrong under any convention.

**Why the two apply methods.** Angular rate is measured *about* an axis, so its sign
follows the right-hand rule of the resulting frame. Under a map `M`, gyro transforms
as `det(M)·M` while per-axis measurements transform as `M`. Splitting them into two
named methods makes the distinction impossible to forget — this is precisely the
rule the current code follows correctly but never states.

**Correction to an earlier draft:** that draft claimed accel and gyro were
inconsistent and that the correct alignment therefore had to be rediscovered
empirically as a Phase 6 blocker. That was wrong — they are consistent under the
determinant rule, and the existing behaviour ports verbatim as
`{+X, −Y, +Z}`. The bench check in Phase 6 is now *verification*, not discovery,
which materially de-risks that phase.

**Still worth knowing:** det = −1 means something is genuinely mirrored. A chip
cannot be mounted mirrored, so this points at the part's internal axis sign
convention differing from the datasheet — consistent with a counterfeit module. The
design surfaces it (`isMirrored()`, boot inventory, `imu axes` CLI) rather than
hiding it, so the cause stays diagnosable.

**Status:** Accepted.

---

## ADR-008 — How units are represented — **Option A, decided**

**The problem.** Nothing in the codebase enforces units. They are conveyed by
comments and variable names, and the codebase genuinely mixes degrees and radians
along one data path:

| Stage | Unit | Where |
|---|---|---|
| Gyro out of the driver | **deg/s** | `GyroSample::rate_dps` |
| Madgwick input | **deg/s**, converted internally to rad/s (`gx *= 0.0174533f`) | `Adafruit_AHRS_Madgwick.cpp:68` |
| Euler angles out | **degrees** (`* 57.29578f`, plus `+180.0f` on yaw) | `getRoll/Pitch/Yaw()` |
| Attitude PID setpoints | **degrees** | `ArduFliteController::setAttitudeSetpoint` |
| Attitude output limits | **deg/s** | `att.*.outlimit` |
| Attitude deadband | **radians** | `CONFIG_KEY_ATT_DEADBAND`, default `0.0001`, range `0–0.01` |

That last row is a degrees-and-radians boundary in a config key, three rows away
from degrees-valued keys in the same file. A mix-up there is silent, produces a
plausible-looking number, and shows up as odd handling.

**Option A — unit suffixes only (my recommendation).** Every field and parameter
carrying a physical quantity is named with its unit: `gyro_dps`, `angle_rad`,
`pressure_pa`, `time_us`. Nothing else changes.
*Pros:* zero runtime cost, zero risk, no new idiom, and it makes a mismatch visible
at the call site — `deadband_rad = limit_deg` reads wrong.
*Cons:* the compiler still will not stop you.

**Option B — strong angle types.** A header-only `struct Radians { float value; }` /
`struct Degrees`, with explicit named conversions and arithmetic operators; the
compiler rejects a mix.
*Pros:* the bug class becomes impossible, not merely visible.
*Cons:* more verbose at every boundary; operator boilerplate to write and test; and
one target-specific caveat — on the soft-float C3 (§00 2.9b) you would want to
*confirm* the wrappers fully inline rather than assume it. A single-float-member
struct at `-Os` will, but "will" should be "did", checked in the map file.

**Decision: Option A.** Option B is deferred, not rejected — it addresses a real bug
class this codebase has, but it is *orthogonal* to the HAL (it touches the control
layer, the config schema and the estimator, none of which the HAL work needs to
modify) and would roughly double an already-large diff. The §03 interfaces are
shaped identically either way, so it stays available afterwards.

**The convention, stated so it can be applied mechanically:**

| Quantity | Suffix | Example |
|---|---|---|
| Time | `_us`, `_ms`, `_s` | `time_us`, `dt_s` |
| Frequency | `_hz` | `sampleRate_hz` |
| Angle | `_deg`, `_rad` | `roll_deg`, `deadband_rad` |
| Angular rate | `_dps` | `gyro_dps` |
| Acceleration | `_g`, `_mps2` | `accel_g` |
| Pressure | `_pa`, `_hpa` | `pressure_pa` |
| Distance / speed | `_m`, `_mps` | `altitude_m`, `climb_mps` |
| Pulse width | `_us` | `minPulse_us` |
| Percentage | `_pct` | `linkQuality_pct` |
| Signal strength | `_dbm`, `_db` | `rssi_dbm`, `snr_db` |

Dimensionless values (normalised `[-1,1]` commands, filter alphas, gains, counts)
carry no suffix — the absence is itself informative.

**Scope:** mandatory for all new code in `hal/`, `estimation/`, `actuators/`,
`input/`. Applied to existing code opportunistically as each file is touched by a
migration phase — not as a separate sweeping rename.

**Config keys are renamed too — in Phase 1.** The maintainer confirmed the stored
NVS values are disposable (the code defaults are what currently flies), which
removes the only obstacle. Scope check before committing to this:

* `tools/web_ui/src/app.js:19` keys its tabs off **prefixes only** — `rate.`,
  `att.`, `mix.`, `servo.`, `imu.`, `failsafe.`, `crsf.`, `web.`, `sys.`. Keeping the
  prefixes stable means the web UI needs **no change**.
* `ConfigObservers.cpp` subscribes via `CONFIG_KEY_*` macros, not string literals.
* `ConfigPersistence` already skips unknown keys on load with a warning
  (`ConfigPersistence.cpp:486`) and has `CONFIG_SCHEMA_VERSION` with migration
  support.

So the rename touches `ConfigKeys.h`, `ConfigSchema.h` and
`docs/CONFIG_REFERENCE.md`, and nothing else. Bump `CONFIG_SCHEMA_VERSION` to 2 so
stale entries are dropped rather than half-matched.

Renames to apply (leaf names only, prefixes unchanged):

| Now | Becomes |
|---|---|
| `att.deadband` | `att.deadband_rad` |
| `att.<axis>.outlimit` | `att.<axis>.outlimit_dps` |
| `rate.<axis>.ti` / `.td` | `rate.<axis>.ti_s` / `.td_s` |
| `att.<axis>.ti` / `.td` | `att.<axis>.ti_s` / `.td_s` |
| `mix.max_att_<axis>` | `mix.max_att_<axis>_deg` |
| `mix.max_rate_<axis>` | `mix.max_rate_<axis>_dps` |
| `servo.<ch>.min` / `.max` | `servo.<ch>.minpulse_us` / `.maxpulse_us` |
| `servo.<ch>.neutral` / `.defl` | `servo.<ch>.neutral_deg` / `.defl_deg` |
| `servo.max_deg_sec` | `servo.max_slew_dps` |
| `imu.max_gyro_dps` | *(already suffixed)* |
| `failsafe.bank` / `.pitch` | `failsafe.bank_deg` / `.pitch_deg` |

Dimensionless keys (`*.kp`, `*.alpha`, `*.headroom`, `*.outlimit` on the rate loop,
`servo.wing_design`) keep their names — the absence of a suffix is informative.

**Do this in Phase 1**, before the HAL starts adding keys, so the convention is
established rather than retrofitted.

**Status:** **Accepted — Option A**, confirmed by the maintainer. Revisit Option B
after Phase 8.

---

## ADR-009 — `Status`/`Result<T>`, no exceptions

**Decision:** value-returned errors; no exceptions, no RTTI.

**Correction:** the first draft justified this partly as "matches the ESP32 Arduino
default build (`-fno-exceptions`)". **That was wrong** — the core's `cpp_flags`
enables `-fexceptions` (§09 1.1). The decision stands; the reasoning is:

* **Unbounded latency.** A 500 Hz control loop cannot contain a path whose worst-case
  timing depends on stack unwinding.
* **Flash cost.** Unwind tables are not free, and the lite build's budget is the
  binding constraint.
* **It composes with what is underneath.** The drivers wrap C-style ESP-IDF APIs that
  already return status codes.
* Callers can distinguish `NotPresent` from `IoError` from `Corrupt`, which is
  exactly what a graceful-degradation boot path needs.

`enum class [[nodiscard]] Status` (C++20) makes dropping a status a compiler warning
— worth more here than the error type itself, given that the current codebase's
failure mode is "log it and carry on".

**Open, to measure not assume:** adding `-fno-exceptions` to `build.sh` may reclaim
meaningful flash, but the prebuilt core was compiled with exceptions on. Measure the
delta in Phase 0; adopt only if the build is clean and the aircraft flies.

**Status:** Accepted, with the rationale corrected.

---

## ADR-010 — `Board` is the one permitted singleton

**Decision:** `Board::instance()`, touched only by `arduflite_init()`.

**Rationale:** something must be the root, and static-init-order rules out a plain
global. The danger with AP_HAL's `hal` is not that it is a singleton — it is that
*everything reaches into it*. Confining that to one function, and enforcing it with
an include-direction check, keeps the benefit without the cost.

**Status:** Accepted.

---

## ADR-011 — Static storage, no heap after boot

**Decision:** all drivers are members of `Board`, constructed in place. `begin()`
may allocate; nothing after it does.

**Verification:** a host test installs a `operator new` that aborts once
`Board::begin()` has returned.

**Open sub-question:** `std::optional<T>` members for conditionally-fitted parts add
a byte of padding per part but keep construction clean. Alternative is aligned
storage + placement new, which is uglier. **Recommendation: `std::optional`.**

**Status:** Accepted.

---

## ADR-012 — Task policy in a typed table

**Decision:** `hal::Priority` enum + `TaskConfig`; `Scheduler::spawn()` takes no raw
integers.

**Rationale:** the priority ladder is currently documented in AGENTS.md prose and
implemented as magic numbers in eight `xTaskCreate` calls. Making it a type makes
the documentation and the code the same artefact.

**Status:** Accepted.

---

## ADR-013 — Own register drivers for the IMU and barometer; drop FastIMU and Adafruit_BMP280

**Decision:** write `drivers::Mpu6500`, `drivers::Mpu9250` and `drivers::Bmp280`
directly against `hal::RegisterDevice`. Retire FastIMU and Adafruit_BMP280.
Adafruit_AHRS and ESP32Servo are decided separately (ADR-017, ADR-018);
Adafruit_NeoPixel stays.

**The decisive fact — these libraries own the bus.** Verified in the installed
sources:

```cpp
// FastIMU/src/sensors/F_MPU6500.hpp:145
explicit MPU6500(TwoWire& wire = Wire) : wire(wire) {};
// :209
TwoWire& wire;
```

FastIMU binds to a `TwoWire&` and issues its own `readByteI2C` / `writeByteI2C`.
Adafruit_BMP280 does the same. **A driver wrapping either cannot go through
`RegisterDevice`**, which means:

* it cannot be tested on the host against `FakeRegisterDevice` (G4 fails for the
  single most important device in the system);
* it cannot participate in the bus mutex, so the "single I2C owner" invariant
  (§02 4.3) becomes unenforceable exactly where it matters;
* it cannot move to SPI on a future board (ADR-004's whole point);
* two independent `Wire` users appear on one bus — the arrangement that caused the
  original priority-inversion problem.

Wrapping them would produce a HAL whose IMU and baro are as untestable as today.
That is most of the value of this project, lost for the two devices that carry it.

**Three further arguments:**

1. **The register interface is small and exceptionally well documented.** MPU-6500:
   `WHO_AM_I`(0x75), `PWR_MGMT_1`(0x6B), `CONFIG`(0x1A), `GYRO_CONFIG`(0x1B),
   `ACCEL_CONFIG`(0x1C/0x1D), `SMPLRT_DIV`(0x19), and a 14-byte burst read from
   `ACCEL_XOUT_H`(0x3B). ~120 lines. BMP280's compensation is ~60 lines copied from
   the datasheet — which ships **worked reference values**, making it one of the
   easiest things in the codebase to test correctly.
2. **FastIMU's `WHO_AM_I` check is strict and silent:**
   `if (!(readByteI2C(...) == 0x70)) return -1;` — it returns −1 without reporting
   what it actually read. Given you suspect a counterfeit part, a driver that
   *logs the byte* and accepts a configured set of known-good values is directly
   useful diagnostic value you cannot get today.
3. **FastIMU carries its own `calData` calibration model**, overlapping
   `SensorCalibration`. Two calibration systems is one too many.

**Adafruit_AHRS and ESP32Servo are handled separately** — see ADR-017 and ADR-018.
Adafruit_NeoPixel stays: it lives inside `drivers/led`, a tier where Arduino
dependencies are allowed and expected, and there is no portability or licensing
argument against it.

**How the rewrite risk is retired — a cross-check harness.** In Phase 5, build a
bench firmware that runs the new register driver *and* FastIMU against the same
physical sensor and logs both sample streams. Compare offset, scale and noise over
a few minutes and through the six static orientations. This converts "did I get the
scaling right?" from a worry into a measurement. FastIMU is deleted only after the
comparison passes.

**Cost:** roughly 1–2 extra sessions in Phase 5, plus one bench session. Against
that: the IMU and baro become host-testable, SPI-capable, and diagnosable.

**Status:** Accepted (was open; decided after inspecting the installed library
sources).

---

## ADR-014 — Delete the PWM receiver

**Decision:** delete `src/receiver/pwm/` and `src/tests/ReceiverTests.*` in Phase 4.
`drivers::SimRcLink` becomes the second `RcLink` implementation that proves the
interface.

**Evidence it is dead, not merely unused:**

* `ArduFlitePwmReceiver` is **never instantiated**. Its only consumers are
  `include/ReceiverConfiguration.h` (also never included by the application) and
  `src/tests/ReceiverTests.{h,cpp}` — and `runReceiverTest_print()` is never called
  from anywhere. The whole subgraph is unreachable, yet it is compiled into every
  build.
* Its pin table (`PwmInputConfig`) contains an out-of-range GPIO and three
  collisions with live pins (§00 2.4). It cannot have been built and run on the
  current board in a long time.
* Its ISR design assumes a single instance via a `static ArduFlitePwmReceiver*
  instance` and stores per-channel state in `volatile unsigned long*` arrays — it
  would need rewriting to satisfy `RcLink` anyway.

**Alternatives:**
* *Port it to `RcLink`.* Only worthwhile if PWM RX is a real use case. Given CRSF is
  what flies and the PWM path has bit-rotted past the point of being a reference,
  porting means writing it fresh — in which case there is nothing to preserve.
* *Leave it untouched.* Rejected: dead code with actively wrong pin assignments
  consumes review attention forever and will fail the new board validation.

`SimRcLink` is needed for the test suite regardless, so it costs nothing extra and
exercises the interface more thoroughly (injectable failsafe, link-quality sweeps)
than the PWM driver would.

**Reversibility:** it is in git history. If PWM RX becomes a requirement, writing
it against `RcLink` is a day's work and the result will be correct.

**Status:** Accepted (was open; decided after confirming it is unreachable).

---

## ADR-015 — Keep `ConfigRegistry` unchanged

**Decision:** the HAL consumes `ConfigRegistry`; it does not replace or wrap it.

**Rationale:** it already does what AP_Param does, better (string keys, validation,
observers, JSON export, schema versioning). Touching it would double the size of
this work for no gain. The only related change is that IMU calibration blobs move
from raw EEPROM to `device::SettingsStore`, which is a *different* concern from
tunable parameters.

**Status:** Accepted.

---

## ADR-016 — Naming: full words, established acronyms excepted

**Decision:** namespaces and identifiers spell words out. Root namespace is
`arduflite`, not `afl`. Tiers are `arduflite::hal`, `arduflite::device`,
`arduflite::drivers`, `arduflite::board`. Macros are `ARDUFLITE_TRY`,
`ARDUFLITE_BOARD_<NAME>`.

**Exceptions — kept short because the short form *is* the name:**

* Established acronyms: `hal`, `imu`, `pwm`, `uart`, `i2c`, `spi`, `gpio`, `rc`,
  `crsf`, `nvs`, `pid`, `led`, `cli`.
* Standard unit symbols in field suffixes: `_us`, `_ms`, `_hz`, `_pa`, `_hpa`,
  `_dps`, `_deg`, `_rad`, `_g`, `_m`, `_mps`, `_pct`, `_dbm`.

**Rationale:** `afl::` saves seven characters and costs every new reader a lookup.
The acronyms above do not — nobody expands "UART" mentally. Unit suffixes are SI
notation, and `gyro_degreesPerSecond` is worse than `gyro_dps` on every axis that
matters.

**Consequence:** `arduflite::device::Accelerometer` is verbose in a declaration. In
practice a `using` at the top of an implementation file handles it, and the fully
qualified form appears mostly in this specification.

**Status:** Accepted.

---

## ADR-017 — Own Madgwick implementation, but *after* the abstraction lands

**Decision:** Phase 6 wraps `Adafruit_Madgwick` behind the `AttitudeEstimator`
interface, unchanged. A **separate, later change** replaces it with
`estimation::MadgwickEstimator`, validated against the Adafruit output by L3 log
replay on real flight data. Not bundled into Phase 6.

**Why write our own eventually:**

1. **Licensing — the strongest reason.** `Adafruit_AHRS_Madgwick.h/.cpp` carries the
   x-io header verbatim: *"Open-source resources available on this website are
   provided under the GNU General Public Licence unless an alternative licence is
   provided in source."* ArduFlite is MIT and published on GitHub. Madgwick's own
   later releases are MIT-licensed, so the situation is arguably murky rather than
   clear-cut — but an MIT project shipping a file with a GPL notice is worth
   resolving deliberately rather than by inattention. **Not legal advice; flag it,
   check it, decide it.**
2. **We use a third of it.** Of 305 lines we need `updateIMU()` + `computeAngles()`
   + `invSqrt()` ≈ 120. The mag `update()` path is unused (MPU-6500 has no mag), and
   `Adafruit_AHRS.h` also drags in Mahony and NXPFusion.
3. **Small behaviours we would rather own:** `getYaw()` returns
   `yaw * 57.29578f + 180.0f` — an undocumented offset baked into a dependency;
   the `anglesComputed` caching flag adds a branch on every getter; and dt handling
   is split across two overloads (`begin(sampleFreq)` vs the explicit-`dt` variant
   ArduFlite actually calls).

**Why *not* the reasons you might expect — being honest about the weak arguments:**

* **Overhead is not a strong case.** `updateIMU()` is roughly 90 float operations.
  At 500 Hz that is ~45 k float-ops/s; on the soft-float C3 (§00 2.9b) at ~10–30
  cycles each that is **~0.3–0.8 % of a 160 MHz core**. Real, measurable, and not a
  problem. A rewrite might halve it. That is not worth touching the estimator for on
  its own.
* **`invSqrt` is not a bug here.** It uses the Quake `0x5f3759df` hack with two
  Newton iterations. On a chip *with* an FPU that is a pessimisation versus
  `1.0f/sqrtf(x)`. On the C3, which has no FPU, it is very likely a **win**. Do not
  "fix" it reflexively — measure it on the target.
* **Testability is not a case at all.** It is pure maths depending only on
  `<math.h>`; it already compiles and runs on the host today.

**Why the sequencing matters more than the decision.** Replacing the attitude
estimator is the single highest-risk change that can be made to a flight controller.
Today there is no way to prove a replacement is equivalent. After Phase 6 there is:
the `AttitudeEstimator` interface plus L3 replay of `FL001`/`FL002` lets both
implementations be run over the same real flight data and their quaternions
compared. **The abstraction is what makes the rewrite safe, so the abstraction goes
first.** Doing both at once forfeits the oracle.

**Exit criteria for the later swap:** replay over every logged flight, quaternion
divergence within tolerance, float-op count and loop timing measured on hardware,
one bench session, one short flight.

**Status:** Accepted (wrap now, own implementation as a separate later change).

---

## ADR-018 — Drop ESP32Servo for the core LEDC API

**Decision:** `hal::esp32::Esp32PwmOut` calls the Arduino-ESP32 core directly.
ESP32Servo is removed. This lands in **Phase 3**, with all other actuator work.
`Esp32PwmOut` itself is written in Phase 2 but deliberately not wired up until
Phase 3, so the servo output stage is converted and bench-verified exactly once
(review finding R10).

**What replaces 1290 lines:** the installed core (3.3.10) exposes exactly the API
needed, in `esp32-hal-ledc.h`:

```c
bool     ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool     ledcWrite(uint8_t pin, uint32_t duty);
bool     ledcDetach(uint8_t pin);
bool     ledcOutputInvert(uint8_t pin, bool out_invert);
```

`Esp32PwmOut` becomes ~40 lines: attach at 50 Hz / 16-bit, convert microseconds to
duty, write. Microsecond→duty is integer arithmetic, so it is also cheaper than the
current path on a soft-float target.

**Why drop it:**

1. **It is 1290 lines of per-chip `#ifdef` sprawl in a dependency we do not
   control** — `CONFIG_IDF_TARGET_ESP32S2/S3/C3/C5` branches throughout `ESP32PWM.cpp`
   and `ESP32Servo.cpp`. Eliminating exactly this pattern is the point of §04.
2. **It uses `double` in setup paths** — `pow(2, timer_width)` at
   `ESP32Servo.cpp:68,98,246` and `double freq` throughout `ESP32PWM`. On the C3 that
   is software-emulated double precision. Boot-time only, so not a live problem, but
   it is a dependency working against the target.
3. **AGENTS.md rule 7 favours this, not the reverse.** *"Prefer library built-ins
   over custom code… Use native features when available."* The core LEDC API **is**
   the native feature; ESP32Servo is a third-party wrapper over it. Dropping it moves
   toward the rule.
4. **It quantises.** `Servo::write(int degrees)` is the current call path, which
   costs ~11 µs of resolution before the slew limiter ever sees the value.
   `writeMicroseconds` is the hardware's real unit.

**One thing I checked and it is *not* a problem.** ESP32Servo's C3 error message
reads *"Servo available on: 1-10,18-21"*, which would exclude your elevator on
GPIO 0. The actual guard in `ESP32PWM.h:176` is `(pin >= 0 && pin <= 10)` — GPIO 0
**is** accepted. The message is simply wrong. No live defect on your aircraft; noted
only so the discrepancy is not rediscovered later as a scare.

**Risk:** low. PWM duty arithmetic, covered by `RecordingPwmOut` host tests plus a
bench check of travel and centring on every surface before flight.

**Status:** Accepted.

---

## ADR-019 — One interface per measurement, not per chip

**Decision:** `Accelerometer`, `Gyroscope`, `Magnetometer`, `Barometer`, `Gnss`,
`Airspeed`, `RangeFinder`, `PowerMonitor`, `Thermometer` are independent interfaces.
A chip implements as many as it provides. A separate `Sensor` interface owns
`probe()` / `begin()` / `sample()` / `health()`. The board exposes **lists** of each
measurement type, not single pointers.

**This replaces the combined `ImuSensor` in the first draft**, which returned an
`ImuSample { accel, gyro, temp }` — a *device* abstraction wearing a role's name. It
would have blocked exactly the two cases the maintainer raised: custom hardware with
discrete accelerometer and gyroscope parts, and redundant sensors.

**Alternatives:**
* *Combined `ImuSensor` (first draft).* Rejected. Discrete parts would have needed a
  fake adapter presenting two chips as one, and there was no way to express two of
  them.
* *Per-measurement interfaces with `read()` hitting the bus directly.* Rejected on
  cost: the MPU-6500 returns accel + temp + gyro in one 14-byte burst from
  `ACCEL_XOUT_H`. Separate bus-touching reads would double I2C traffic at 500 Hz on a
  400 kHz bus.
* *Common base class with virtual inheritance* (`Accelerometer : virtual Sensor`).
  Rejected: a part implementing two interfaces would need virtual bases, paying
  offset lookups on every call, for no benefit. Keeping `Sensor` unrelated to
  the measurement interfaces means plain multiple inheritance of pure interfaces —
  no diamond, no vbase, no penalty.

**The key move is the `sample()` / `read()` split.** `sample()` is the only method
that touches the bus and refreshes every reading the chip provides; `read()` is
`const`, returns cached data, and can be called from anywhere. This yields
per-measurement interfaces *and* the burst read *and* a clear bus-ownership story —
the sampling task is the only caller of `sample()`.

**Consequences:**
* `AxisTransform` becomes per sensor instance, not per board — a discrete gyro and
  a discrete accelerometer can be mounted at different angles.
* Redundancy is a descriptor change: a second `sensors[]` entry at a different
  address. No code changes anywhere.
* Selection/voting is a flight-layer concern (`estimation::SensorSelector`), not a
  HAL one. Phase 6 ships the trivial policy (first healthy instance, fall back on
  `Failed`); median-of-three or full voting can be added later without touching an
  interface. The design does not *deliver* redundancy — it stops precluding it.
* A dead barometer removes one entry from one list. It does not stop the gyro from
  flying the aircraft.

**Status:** Accepted.

---

## ADR-020 — `ActuatorBank` must not name a transport

**Decision:** `ActuatorChannelConfig` carries only transport-neutral fields (role,
range, invert, trim, travel limits, slew, failsafe action). Transport specifics go
into the driver's constructor from the board descriptor. `commit()` is added as an
explicit push-to-hardware step returning `Status`. Optional `readFeedback()` and
per-channel `state()` are added. `CompositeActuatorBank` aggregates mixed transports.

**Correcting the first draft — this one was a genuine design error.** The original
`ActuatorChannelConfig` contained `minPulse_us`, `maxPulse_us`, `neutralPulse_us`
and `frameRate_hz`. That is PWM vocabulary in an interface whose entire purpose is
to hide the transport. A CANopen implementation would have had to ignore four config
fields and smuggle node IDs and object-dictionary indices in through a side channel.
The abstraction would have held right up until someone tried to use it, which is the
worst possible failure mode for an abstraction.

**Four specific defects and their fixes:**

| Defect | Fix |
|---|---|
| PWM units in the shared config | Transport config moves to the driver constructor |
| `write()` returns `void` — fine for fire-and-forget PWM, wrong for a bus where a transaction fails | `commit()` returns `Status`; `state(idx)` reports `Offline`/`Fault` |
| No feedback path — CANopen drives, serial servos and DShot ESCs all report position, current, temperature, faults | Optional `hasFeedback()` / `readFeedback()`, defaulting to `NotSupported` |
| No atomic group update — writing 5 CAN servos individually is wasteful and non-atomic | `write()` stages, `commit()` transmits the PDO group + SYNC |

**A fifth, from the maintainer's question:** one bank could not span transports.
Four PWM surfaces plus one CAN throttle is realistic, so `CompositeActuatorBank`
flattens several banks into one index space.

**Honest scoping.** The interface is sufficient for CANopen; the *work* is not
small. It needs `hal::CanBus` (specified in §03 2.3 — the ESP32-C3's TWAI controller
makes this real rather than hypothetical), a CANopen protocol stack (object
dictionary, NMT, SDO, PDO mapping, heartbeat — realistically integrating
CANopenNode rather than writing one), and a bus-timing analysis for a 500 Hz group.
What the design guarantees is that **none of that leaks upward**: the mixer,
controller, failsafe path and telemetry are untouched.

More likely near-term beneficiaries of the same interface: **DShot ESCs** and
**smart serial servo buses** (Dynamixel, FeeTech), where `ActuatorFeedback` starts
paying for itself immediately.

**Status:** Accepted.

---

## ADR-021 — C++20 baseline, matched between firmware and host tests

**Decision:** target C++20 explicitly, and raise the host test suite from C++17 to
match. Use `std::span`, concepts, `constinit`, `consteval`, designated initialisers
and `std::chrono` rather than hand-rolled equivalents.

**Why this needed deciding at all:** earlier drafts assumed C++17 and specified a
bespoke `Span<T>` "because C++17 has no `std::span`". Checking the installed
toolchain showed otherwise —
`esp32c3-libs/3.3.10/flags/cpp_flags` ends with `-std=gnu++2a`, so **the firmware
already compiles as C++20**. The bespoke type was solving a problem that does not
exist.

**The mismatch is the real finding.** `tests/unit/CMakeLists.txt` sets
`CMAKE_CXX_STANDARD 17`. Host tests compiling under different language rules than the
firmware is a latent trap: code that passes tests can fail to build for the target,
and C++20-only constructs are invisible to the test suite. Phase 0 raises it and adds
a CI check that the two stay aligned.

**What C++20 buys here, concretely:**

| Feature | Replaces | Value |
|---|---|---|
| `std::span` | bespoke `Span<T>` | one less type to write and test |
| `requires std::is_trivially_copyable_v<T>` on `SeqLock` | a comment | a seqlock over a non-trivial type now fails to compile instead of corrupting in flight |
| `enum class [[nodiscard]] Status` | convention | every dropped status is a warning |
| `constinit Board` | the deferred-init convention | static-init ordering becomes a compiler-checked property |
| `consteval isValid()` | `constexpr` | board validation *cannot* silently fall back to runtime |
| Designated initialisers | GNU extension | board descriptors are standard C++ |

**Not adopted:** `std::expected` (C++23 — the `-std=gnu++2b` in `cpp_flags` is
overridden by a later `-std=gnu++2a`); ranges (weight not justified on this target);
modules (no toolchain support in the Arduino build).

**Status:** Accepted.

---

## ADR-022 — Standard lock types, not a bespoke RAII wrapper

**Decision:** `hal::Mutex` satisfies the standard *Lockable* and *TimedLockable*
requirements (`lock()`, `try_lock()`, `unlock()`, `try_lock_for()`), so
`std::lock_guard`, `std::unique_lock` and `std::scoped_lock` work directly. Both
`SemaphoreLock` and the earlier draft's `ScopedLock` are deleted.

**Alternatives:**
* *Keep a bespoke `ScopedLock`* (earlier draft). Rejected: it reimplements
  `std::unique_lock` less well, has no multi-lock story, and teaches a project-local
  idiom for no gain.
* *Expose FreeRTOS handles and use them directly* (status quo). Rejected: puts
  `SemaphoreHandle_t` in portable signatures, violating G3.

**What it buys:**
* `std::scoped_lock a{m1, m2}` gives deadlock-free multi-lock acquisition free —
  something the current code has no answer for at all.
* `[[nodiscard]]` on `try_lock` makes an ignored acquisition failure a warning. The
  current `SemaphoreLock` requires remembering to call `.acquired()`, and nothing
  enforces it.
* `std::chrono` timeouts instead of a bare `uint32_t` that could be either
  milliseconds or ticks — a real ambiguity in `SemaphoreLock`'s current constructor,
  which special-cases `portMAX_DELAY` against millisecond values in the same
  parameter.

**Implementation note:** `try_lock_for` is templated on duration so it cannot be
virtual. It forwards to a protected virtual `try_lock_for_us(int64_t)`. The template
is header-only and inlines away.

**Status:** Accepted.

---

## ADR-023 — `Sensor` and `ActuatorBank`: why the names are asymmetric

**Decision:** rename `SensorDevice` → **`Sensor`**. Keep **`ActuatorBank`** — do
*not* rename it to `Actuator`.

**Half of this was the maintainer's suggestion and it is right.**
`device::SensorDevice` stutters — the namespace already says "device", so "Device"
carries no information. `arduflite::device::Sensor` is shorter and obvious. Applied
throughout.

**The other half would make things worse, and the reason is a real structural
asymmetry rather than a preference:**

|  | Sensors | Actuators |
|---|---|---|
| An object is… | **one chip** | **N outputs** |
| Two IMUs means | two `Sensor` objects | — |
| Five servos means | — | one `ActuatorBank` with `count() == 5` |
| How many *kinds*? | many — acceleration, rate, field, pressure, position… | one — set a normalised value |
| So the split is | lifecycle (`Sensor`) + one interface per measurement | a single interface covering the bank |

Put in one line:

> **Sensors are one chip with many kinds of measurement.
> Actuators are many outputs with one kind of actuation.**

That is why sensors need `Sensor` + `Accelerometer` + `Gyroscope` + …, and actuators
need only `ActuatorBank`. The shapes genuinely differ; matching names would hide that
rather than clarify it.

**Why a bank at all, rather than N `Actuator` objects** — three reasons, the first
load-bearing:

1. **Atomic group commit.** CANopen stages N target positions, then transmits the
   mapped PDOs plus a SYNC so every servo acts on the same control cycle. With N
   independent `Actuator` objects there is nowhere for `commit()` to live — you would
   need a separate group object anyway, and then you have both concepts.
2. **`disable()` is cross-cutting.** Failsafe must put every output into its failsafe
   action together. Per-object `disable()` leaves a window where half the surfaces are
   released and half are held.
3. **Cost.** N virtual objects and N vtable pointers for N servos, versus one bank and
   an index.

**And `Actuator` would actively mislead.** `actuator.write(idx, value)` reads as
though a single actuator has indices. `ActuatorBank` tells the reader immediately that
it is a collection — it is *more* obvious than the proposed name, not less.

**Considered and rejected:**
* *`Actuator` as the bank.* Misleading, per above.
* *`Actuator` (single output) + `ActuatorGroup` (the bank).* Two types where one
  works, and every call site would go through the group anyway to get `commit()`.
* *`ActuatorBank::channel(idx)` returning a lightweight non-owning handle*, so
  `bank.channel(kElevator).write(-0.3f)` is available for readability. Genuinely
  tempting and cheap, but it is ergonomics-only and adds a type. **Deferred** —
  revisit if call sites read badly in Phase 3.

**Caveat added later (see ADR-024).** The claim "actuators have one kind of
actuation" weakens once binary retracts and latching payload releases are in scope.
It survives — because at the *transport* level the operation really is uniform
("stage a normalised value, commit") — but the differing semantics are now carried
by `ActuatorKind` in the per-channel config rather than being pretended away.

**Status:** Accepted, with the ADR-024 caveat. `SensorDevice` → `Sensor` applied
across §01–§10.


---

## ADR-024 — Heterogeneous actuators: composite banks, honest contracts

**Context.** The maintainer asked what happens with a mix of CANopen servos, PWM
servos and "some other type of actuator too". `CompositeActuatorBank` was already in
the design, but auditing its contract found it underspecified in three
safety-relevant ways. This ADR records the fixes.

**Two orthogonal axes were being conflated:**

| Axis | Question | Handled by |
|---|---|---|
| **Transport** | how does the command reach it? | which `ActuatorBank` implementation the channel lives in |
| **Kind** | what does it physically do? | `ActuatorKind` in `ActuatorChannelConfig` |

A binary retract on CAN and a binary retract on PWM are the same *kind* on different
*transports*. Treating these as one axis is what made the original design feel
adequate when it was not.

**Decisions:**

1. **`commit()` returns `CommitResult`, not `Status`.** The original "returns the
   first failure" is not actionable: on a flying aircraft, "ailerons stale" and
   "elevator stale" demand different responses, and the flight layer cannot choose
   without knowing which. `staleMask` names the channels that did not update.

2. **A failing bank must not abort the others.** `CompositeActuatorBank::commit()`
   drives every sub-bank and ORs the results. The original wording — "returning the
   first failure" — read as though it might short-circuit, which would leave surfaces
   undriven because a *different* transport failed.

3. **`disable()` disables every bank unconditionally**, even if an earlier one
   errors. A partial disarm is worse than a failed one.

4. **`ActuatorBank::nativeRate_hz()` added**, symmetric with `Sensor::nativeRate_hz()`.
   Rate mismatch across transports (50 Hz PWM, 32 kHz DShot, PDO-budget CANopen) was
   simply unaddressed; the caller now has the data to decimate per bank.

5. **`ActuatorKind { Proportional, Binary, Latching }`** added to the channel config
   rather than splitting `ActuatorBank` into per-kind interfaces. Rationale: the
   transport-level operation stays uniform, so the differences are per-channel
   *policy* (slew or snap; what failsafe means) not per-interface *shape*. `Latching`
   carries the important constraint — a parachute or payload release is never driven
   by the mixer and **must not be actuated by `disable()`**.

**Rejected:** splitting into `ProportionalActuator` / `BinaryActuator` interfaces
mirroring the sensor side. It would give three interfaces where config suffices, and
every call site would still go through the bank for `commit()`.

**Stated as a limit rather than solved:** `commit()` cannot be atomic *across*
transports. The composite issues sub-commits back to back so the skew is bounded by
the transports themselves (~0.6 ms for a five-node CAN PDO group at 1 Mbit, against a
2 ms loop budget), but it is non-zero. §03 3.3 carries the design guidance that
follows from this: **keep the primary flight surfaces on a single bank.** Splitting
roll across a PWM aileron and a CAN aileron builds in a rolling-moment asymmetry that
only appears under bus load.

**Status:** Accepted.
