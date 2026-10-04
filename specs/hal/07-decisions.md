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

**Status:** Accepted for the `Sensor` rename. **The `ActuatorBank` half is
superseded by ADR-025**, which splits `Actuator` out as a separate interface — two
of the three arguments made here supported the *bank*, not the absence of a
per-output type, and the third did not survive arithmetic.


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

---

## ADR-025 — Split `Actuator` out of `ActuatorBank`

**Decision:** two interfaces. `Actuator` is one output; `ActuatorBank` is one
transport's worth of outputs. Flight code holds `Actuator&` resolved by role at
composition time. `ActuatorBank` keeps `commit()`, `disable()`, `begin()` and
`nativeRate_hz()`.

**This reverses part of ADR-023.** The maintainer asked whether an `ActuatorBank`
should be composed of `Actuator` objects, and whether the bank level is needed at
all. The first is right and I had it wrong. The second has a good answer, but not
the one I originally gave.

### Where my original reasoning was wrong

ADR-023 gave three reasons for a single index-based bank. Re-examined:

| Original argument | Verdict |
|---|---|
| *"Atomic group commit — nowhere for `commit()` to live with N objects."* | **Half right, and it argues for keeping the bank, not against `Actuator`.** SYNC is a property of the *bus*, not of "all outputs". So `commit()` belongs on a transport group — which is what `ActuatorBank` now is. It never argued against per-output objects |
| *"`disable()` is cross-cutting."* | **Same.** Batched disarm is a bus property. It justifies the bank, not the absence of `Actuator` |
| *"N virtual objects cost more."* | **Wrong, and I should have quantified it before asserting it.** Eight actuators is roughly 350 bytes on a 320 KB part, and five extra virtual calls per tick at 500 Hz is ~2500/s — unmeasurable. This was hand-waving |

So two of three arguments support the bank while saying nothing about `Actuator`, and
the third does not survive arithmetic.

### What the split actually buys

1. **No magic indices.** `write(2, x)` becomes `elevator.stage(x)`. An index/role
   mismatch after a board-descriptor edit is a bug class that stops existing.
2. **Per-output feedback where it is used.** `readFeedback(idx, out)` becomes
   `elevator.readFeedback(out)`.
3. **Latching actuators become structurally safe** — the strongest reason. A
   parachute release is never placed in the mixer's `ControlOutputs`, so the mixer
   *cannot* reach it. That is a compile-time guarantee replacing a runtime
   `ActuatorKind` check somebody has to remember to honour. Safety properties should
   be structural where they can be.

### Do we need the bank level at all? Yes — but it is a *transport group*

The bank is not an arbitrary collection, and that distinction is what makes it earn
its place:

* **CANopen SYNC** makes every servo on that bus act on the same control cycle.
  There is no per-output equivalent.
* **Batched disarm** — one CAN broadcast, not five SDO writes.
* **`nativeRate_hz()`** is a transport property (50 Hz PWM, 32 kHz DShot, PDO budget).

Every one of those is a property of a *bus*. Everything genuinely per-output moved to
`Actuator`. If a future transport had no batching semantics at all, its bank would be
a thin loop — which is fine, and is exactly what `PwmActuatorBank::commit()` is.

### Naming, revisited

The split resolves the tension in ADR-023 rather than deepening it. Both names now
mean exactly what they say: an `Actuator` is an actuator, an `ActuatorBank` is a bank
of them. The earlier objection — that `Actuator` would mislead because
`actuator.write(idx, value)` implies one actuator has indices — evaporates, because
`Actuator` no longer takes an index.

**Rejected:** a third level (`OutputManager` over `ActuatorBank` over `Actuator`).
`CompositeActuatorBank` is already an `ActuatorBank` by the Composite pattern, so
anything taking a bank takes the composite. A third type would add a level without
adding a capability.

**Status:** Accepted. Supersedes the single-interface actuator model in ADR-020 and
the corresponding half of ADR-023.


---

## ADR-026 — Guarantee a path for sensor failover without implementing it

**Requirement (maintainer):** failover need not exist now, but there must be a path to
adding it that is not a redesign.

**Decision:** implement nothing; add three interface hooks that make every known
failover strategy an additive change, and specify `SensorSelector` now so the trivial
and sophisticated implementations share a shape.

**Why hooks and not the feature.** Failover's hard part is not choosing a sensor — it
is the transient. Switching mid-flight hands the estimator a different bias and
possibly a different mount, and Madgwick reconverges over seconds *while the aircraft
is flying on a wrong attitude estimate, immediately after a sensor fault*. Getting
that right needs hardware to test against, which does not exist yet. Guessing at it
now would produce untested code on the most safety-critical path in the system.

**What was actually missing.** The claim "the design admits failover later" was
untested until it was audited. Three gaps would have made it a redesign:

1. **No way to re-seed the estimator.** `AttitudeEstimator` had `reset()` (zero the
   filter) but no `setOrientation()`. Re-seeding is one of the three transition
   strategies, and the capability already exists in `Adafruit_Madgwick` — only the
   interface was hiding it. Adding a method to an interface after several
   implementations exist is exactly the retrofit this project is trying to escape.
2. **A failover would have been invisible.** `ImuState` carried no record of which
   instance was live, so a switch would not reach the flash log. For a redundant
   system that is the *first* diagnostic question, and it would have been
   unanswerable.
3. **`SensorSelector` was vapour.** Referenced in four documents, specified nowhere.
   The trivial Phase 6 version and a future voting version would have had different
   shapes, which is how "we'll add it later" turns into "we'll rewrite it later".

**Consequence — the three strategies are now all additive:**

* **Crossfade** and **median-of-three** need every instance read each tick. Already
  available: the sampling loop `sample()`s all devices and only `read()` is selective,
  so this is a change inside `InertialSubsystem::tick()`.
* **Re-seed** needs `setOrientation()`. Added.
* All three need to know *when* a switch happened. `switchedThisTick()` provides it.

Adding failover later is therefore: one new `SensorSelector` implementation, one
branch in the tick, and a `SelectionPolicy` value. No interface change, nothing above
the estimation layer, no board-descriptor change.

**Cost of doing this now:** three method declarations and one struct. No runtime cost
on a single-sensor aircraft — `FirstHealthy` with a one-element span is a bounds check.

**Status:** Accepted. Path guaranteed; feature deferred with the R14 trigger intact.


---

## ADR-027 — Portability target is FreeRTOS-based platforms only

**Decision (maintainer):** the HAL targets other **FreeRTOS** platforms — STM32,
nRF, other ESP32 variants. Not Zephyr, not bare-metal, not an RTOS with different
scheduling semantics.

**What that resolves.** `hal::Scheduler` and `TaskConfig` are FreeRTOS-shaped by
design: stack sizes in bytes, small-integer priorities where **higher preempts
lower**, optional core affinity. Under this decision that is no longer a leak, it is
the contract. In particular the `Priority` enum's polarity needs no defensive
abstraction — a Zephyr port would have had to invert it (lower preempts there), and
silently getting that wrong would make the inertial task the *lowest* priority in the
system. Out of scope now, and recorded so the trap is not rediscovered.

**What it does NOT resolve** — both still bite on an STM32+FreeRTOS port:

1. **`ConfigPersistence` is not behind `hal::KeyValueStore`.** It includes
   `<Preferences.h>` directly (`ConfigPersistence.h:16`) and holds a raw
   `SemaphoreHandle_t`. `Preferences` is Arduino-ESP32, not FreeRTOS, so it does not
   travel. The `KeyValueStore` interface exists with **zero consumers**. ADR-015 was
   right to keep `ConfigRegistry` out of scope, but this consequence was not stated.
   → Added to Phase 7.
2. **The build system does not port.** `ArduFlite.ino`, `build.sh` and arduino-cli's
   recursive `src/` compilation are Arduino-specific regardless of RTOS. A port needs
   CMake or PlatformIO. Phase 8's `host_sim` board already forces a non-Arduino build,
   so that is the natural moment to add one.

**Status:** Accepted.

---

## ADR-028 — CRSF wire format lives in `src/hal/protocol/`, not in the driver

**Status:** Accepted (Phase 5)

### Context

CRSF is implemented twice in this codebase, and the two halves must agree byte
for byte:

- `drivers::CrsfParser` — decodes inbound frames. Unambiguously a driver.
- `src/telemetry/crsf/ArdufliteCRSFTelemetry` — encodes outbound frames. Maps
  `TelemetryData` (a flight-layer type) onto the wire, so it is an adapter, not
  a driver.

Phase 4 gave the encoder the link-statistics struct by having it
`#include "src/hal/drivers/rc/CrsfParser.h"`. `tools/ci/check_layering.sh` rule 5
flagged that correctly: flight code was including a driver header.

**This FAIL was live at the end of Phase 4 and was reported as passing.** The
layering script had just been extended with the burn-down metric, and the new
section's output was read instead of the exit status.

### Options

1. **Duplicate the struct in the encoder.** Two definitions of one wire format,
   in two files, that a compiler cannot check against each other. This is the
   failure mode the `static_assert(sizeof(...) == 10)` was added to prevent.
2. **Move the encoder into the driver layer.** It would then include
   `TelemetryData`, i.e. a driver depending on a flight type — the same
   violation pointing the other way.
3. **Exempt `src/telemetry/crsf/` from the rule.** An exemption whose stated
   justification is "this particular file is special" is how layering rules rot.
4. **Split the wire format into its own header.** Chosen.

### Decision

`src/hal/protocol/CrsfProtocol.h` holds the frame type IDs and the payload
structs. No bus, no pin, no register, no I/O. Both the driver and the adapter
include it.

The rule the layering check now encodes is not "nothing may include from
`src/hal/`" but the narrower and more defensible **"flight code must not depend
on a specific device driver."** A wire format is not a device: swapping the RC
receiver for another CRSF-speaking part does not change one byte of it.

### Consequences

- One definition of the CRSF wire layout, with its `static_assert` covering both
  users.
- `src/hal/protocol/` is now a documented layer. Its admission criterion is
  strict and stated in the header: **no hardware access of any kind.** Anything
  needing a bus is a driver.
- Directly applicable beyond CRSF — MAVLink and any future GNSS protocol have
  the same encode/decode split.

---

## ADR-029 — No MPU-9250 driver until a board declares one

**Status:** Accepted (Phase 5)

### Context

The Phase 5 scope in §06 listed `drivers::Mpu9250` alongside `drivers::Mpu6500`.
Neither board descriptor declares an MPU-9250, no MPU-9250 hardware is available
to test against, and the magnetometer path in `ArduFliteIMU` is compiled out
behind `#if IMU_TYPE == IMU_TYPE_MPU9250` — it has never run.

Writing it now would mean an AK8963 auxiliary-I2C bring-up sequence, a
factory-sensitivity read and a continuous-measurement mode, none of which could
be verified against anything. The `applyOrientation()` magnetometer line is also
known to be wrong (§00 2.3) and has never mattered because the code is dead.

### Decision

Not built. The `SensorPart::Mpu9250` enumerator stays in the descriptor schema,
and `Board::beginSensors()` logs "declared but no driver is built in" if a board
ever selects it.

`ArduFliteIMU.h` carries a `static_assert` that fires if `IMU_TYPE` is set to
`IMU_TYPE_MPU9250`. Without it, flipping that macro compiles and runs, fusing a
magnetometer that is never read — zeroed mag data silently dragging the yaw
estimate. A build error is the correct outcome.

### Consequences

- The MPU-9250 becomes the natural **extensibility proof** for Phase 8: adding a
  second IMU part with an extra measurement interface, against the finished HAL,
  is a far more honest test of the abstraction than writing it now alongside the
  interfaces it is supposed to be validating.
- The wrong magnetometer transform is not carried forward into a live path.

---

## ADR-030 — A driver owns its part's power-up sequence, delays included

**Status:** Accepted (Phase 5)

### Context

Both Phase 5 drivers need mandatory waits during bring-up:

- **MPU-6500** — a soft reset reloads the register file from internal defaults.
  Writes issued during that window are silently dropped. FastIMU waited 100 ms
  after reset, 100 ms after wake, 200 ms for PLL lock.
- **BMP280** — a soft reset triggers a copy of factory trimming from NVM into the
  image registers. The calibration block reads back garbage until `STATUS` bit 0
  (`im_update`) clears.

The first implementation put a 100 ms sleep in `Board::beginSensors()` between
`probe()` and `begin()`, so the driver could stay free of a scheduler dependency.
**That is not merely inelegant, it is wrong:** the delay was spent *before* the
reset it was supposed to follow. The drivers had no delays at all.

### Why this was not caught

Neither the host tests nor the firmware build could see it. A fake bus answers
instantly, and real hardware usually gets away with an under-delayed reset — the
failure is intermittent and load- and temperature-dependent. Its signature is an
IMU that comes up at the wrong range, so every reading is off by a constant
factor: an aircraft that flies, badly, in a way that reads as a tuning problem.

### Decision

**A driver's `begin()` performs the complete power-up sequence for its part,
including every delay and every readiness poll.** Drivers that need this take a
`hal::Scheduler&` alongside their `RegisterDevice&`.

Corollaries:

- **The composition root supplies capability, not knowledge.** `Board` hands over
  a scheduler; it does not know that an MPU-6500 needs 200 ms for its PLL. Any
  datasheet timing appearing in `Board.cpp` is a defect.
- **`sample()` and `read()` must never sleep.** They run inside the 500 Hz task's
  critical section. `SamplingNeverSleeps` asserts this for both drivers.
- **Poll a readiness bit in preference to a fixed delay** where the part exposes
  one, with a bounded attempt count and a hard failure on exhaustion. A part that
  never becomes ready must be reported absent, not used.

### Consequences

- `RecordingScheduler` (host fake) records sleeps instead of performing them, so
  the suite stays fast *and* the delays become assertable —
  `BeginWaitsForTheChipToSettleAfterReset` fails if they are removed or reordered.
  A mandatory wait that no test covers is indistinguishable from a missing one.
- The BMP280 gained a soft reset that Adafruit_BMP280 never performed, so a warm
  reboot no longer inherits the previous run's configuration.

---

## ADR-031 — Replacing a library means pinning its register writes first

**Status:** Accepted (Phase 5)

### Context

§06 set Phase 5's contract as "the same numbers from a different code path", and
gated it on a runtime cross-check harness. The harness could not be built as
specified (see §06 Phase 5), so the phase leaned on care instead.

Care was not enough. The first BMP280 implementation configured the part with
temperature oversampling x2 and the **hardware IIR filter at x16**.
Adafruit_BMP280's defaults — what has actually been flying — are temperature
x16, pressure x16, **filter OFF**. The IIR filter would have sat in series with
`ArduFliteIMU`'s existing altitude EMA, changing the vario's dynamics in a phase
whose stated premise was that nothing changes. It was found by diffing against
the library by hand, which is not a process.

### Decision

**When replacing a vendor driver, read the library being replaced, extract the
exact register values it writes, and pin them in a host test before cutting
over.** The test asserts bytes, not intent:

```cpp
EXPECT_EQ(dev.regs[kRegCtrlMeas], 0xB7);   // temp x16 | press x16 | normal
EXPECT_EQ(dev.regs[kRegConfig],   0x00);   // IIR filter OFF
```

Deliberate improvements are then a **separate, named change** with its own
justification and its own bench verification — never a side effect of a
refactor. Both drivers now carry a comment explaining which improvement was
declined and why, so the next reader does not "fix" it back.

### Why byte assertions rather than "sensible defaults"

A configuration divergence is invisible in every cheap check. It compiles, it
runs, the sensor responds, the numbers look plausible. It surfaces as handling
that feels slightly wrong — attributed to tuning, airframe or weather long before
anyone suspects an oversampling field.

### Consequences

- Applies directly to the remaining library replacements: `Preferences` in
  Phase 7 and `Adafruit_Madgwick` in Phase 9. The Madgwick case is the sharpest —
  its `beta` gain and its 1/sqrt approximation must be extracted and pinned, or
  the filter will behave differently in a way that reads as an airframe problem.
- Cost is small and one-directional: the values have to be read out of the old
  library anyway to write the new driver. Writing them down as assertions is
  nearly free.

---

## ADR-032 — Measurement interfaces share a base, and it carries `health()`

**Status:** Accepted (Phase 6). Supersedes the "independent, no common base" note in §03 3.1.

### Context

§03 declared `Accelerometer`, `Gyroscope`, `Magnetometer`, `Barometer` and
`Thermometer` as unrelated interfaces, with a comment stating they deliberately
share no base — an accelerometer and a barometer have nothing in common, so
inventing a base for them would be abstraction for its own sake.

`health()` lived on `Sensor`, the per-part lifecycle interface.

Implementing `SensorSelector` broke this. The selector holds *measurement*
interfaces — that is its whole job, choosing between accelerometer instances —
and has to ask "is this one trustworthy?". With health only on `Sensor`, it
could not. Recovering the `Sensor` behind an `Accelerometer*` needs a
cross-cast, and this project builds `-fno-rtti` (ADR-021), so `dynamic_cast` is
not available. The alternative — passing parallel spans of `Sensor*` and relying
on index correspondence — is exactly the kind of implicit coupling the interface
split existed to remove.

### The deeper error

The original reasoning was wrong on the merits, not merely inconvenient.
**Health is genuinely per-measurement, not per-part.** On an MPU-9250 the
magnetometer is a separate AK8963 die behind an auxiliary I2C bus; it can fail
while the accelerometer and gyroscope keep working perfectly. A single
part-level `health()` would report that device healthy and feed the estimator a
dead magnetometer.

The same shape appears in any combined part: a BME280 whose humidity channel
fails, a barometer whose temperature compensation saturates.

### Decision

```cpp
class Measurement {
public:
    virtual ~Measurement() = default;
    [[nodiscard]] virtual SensorHealth health() const = 0;
};
```

Every measurement interface derives from it. `Sensor` keeps its own `health()`
for part-level lifecycle.

A part whose measurements share a fate — the MPU-6500, where accelerometer and
gyroscope come out of one burst read — implements `health()` **once**. That
single override satisfies both `Sensor::health()` and `Measurement::health()`,
with no ambiguity at the call site and no extra storage. A part whose
measurements can fail independently overrides per-interface.

### Consequences

- `SensorSelector` works against measurement interfaces alone, as designed.
- The base is one pure virtual with no state, so it costs nothing beyond the
  vtable entries the interfaces already had.
- The "no common base" instinct was right in general and wrong here. The test is
  not "do these things resemble each other?" but "does a caller holding one of
  them need something from all of them?" — and the selector does.

---

## ADR-033 — Calibration offsets live in sensor frame, and are subtracted before the axis transform

**Status:** Accepted (Phase 6). Corrects the step ordering in §03 3.8.

### Context

The §03 tick contract listed the per-tick steps as:

```
4. apply each sensor's AxisTransform
5. subtract calibration offsets
```

The shipping code does the opposite, and the difference is not cosmetic.

`CalibrationService` averages **raw** readings — the values straight out of
`Sensor::sample()`, before any transform. So the offsets it produces, and every
offset blob already written to EEPROM on every existing aircraft, are expressed
in the **sensor's own frame**.

The axis transform for the current mounting negates three axes (`accelY`,
`gyroX`, `gyroZ`; §00 2.3). The transform is linear, so:

```
T(raw − offset) = T(raw) − T(offset)
```

Subtracting an untransformed, sensor-frame offset **after** applying `T` computes
`T(raw) − offset`, not `T(raw) − T(offset)`. On the three negated axes the
correction is applied with the wrong sign: instead of removing the bias it
**doubles** it.

### Why this was worth catching before it shipped

A gyro bias is typically a few tenths of a degree per second. Doubled rather than
removed, it produces:

- nothing visible on a bench — the aircraft sits still and the numbers look fine;
- no failed test — the arithmetic is self-consistent;
- in flight, a slow attitude drift, which presents as a trim or PID tuning
  problem and would be chased for a long time in the wrong place.

The specified order is only correct if the stored offsets are themselves
transformed on load — which would require migrating every existing calibration
blob, for no benefit.

### Decision

Offsets stay in sensor frame. The tick order is:

```
4. subtract calibration offsets   (sensor frame)
5. apply the AxisTransform
```

`CalibrationService::service()` is documented as taking sensor-frame values, the
`InertialOffsets` struct carries the warning, and the call site in the tick
states why it passes pre-transform data.

### Consequences

- Stored calibration data stays valid across the refactor; no migration.
- A future board mounting two IMUs differently is unaffected: each instance's
  offsets and each instance's transform are both per-instance, applied in this
  order, and both yield body-frame data.
- The general lesson, which is the reason this is an ADR and not a comment: **a
  step ordering in a spec is a claim about which coordinate frame each value is
  in.** Reordering steps 4 and 5 looks like a refactor and is actually a frame
  change.

---

## ADR-034 — Calibration runs inside the sampling task, not by suspending it

**Status:** Accepted (Phase 6). Implements the §03 3.8 design; closes review R6/R12.

### Context

Calibration needed the I2C bus, and the bus is owned by the IMU task. The
existing solution inverted control: `selfCalibrate()` suspended that task, took
the bus, and blocked for ten seconds.

Everything else followed from that one decision:

- `_pauseRequested` / `_taskPaused` atomics and a spin-wait handshake;
- `esp_task_wdt_reset()` called manually inside the spin, because the task that
  normally feeds the watchdog was the one being suspended;
- careful mutex ordering, so the caller would not deadlock against the task it
  had just suspended;
- `crsfTelemetry->pauseTask()` at the call site, because ten seconds of a
  suspended IMU task tripped the telemetry watchdog too;
- the control loops running on a **frozen attitude snapshot** throughout.

### Decision

The task that owns the bus is the task that calibrates. `request()` sets a flag;
step 11 of each tick contributes one sample. A ten-second run is 5000 ticks of
accumulation rather than a blocking loop.

The calling task still blocks — `waitForCalibration()` polls at 50 ms — but only
the caller waits, and it holds nothing while waiting.

### Consequences

- The entire pause protocol is deleted: both atomics, the handshake, the manual
  watchdog feeding and the telemetry pause. Platform-call burn-down 133 → 123.
- The control loops receive **live** attitude during calibration instead of a
  frozen snapshot.
- Telemetry keeps streaming, so the operator can watch the aircraft actually be
  still rather than trusting a frozen display.
- Testable for the first time. The blocking version could only be exercised on
  hardware, by hand, one ten-second run at a time; the state machine is driven
  tick-by-tick against a virtual clock, which is how the failure paths — too few
  samples, a dead barometer, a request arriving mid-run — got covered at all.
- `ArduFliteController::pauseTasks()` is **kept**, and is now clearly a separate
  concern: control surfaces should not respond to sticks while someone holds the
  airframe still. That is usability and finger safety, not a bus-ownership
  workaround.
- One asymmetry remains: `baroCalibrate()` is called from `begin()`, before the
  sampling task exists, so it drives the state machine itself in a local loop.
  That disappears when `InertialSubsystem` owns startup ordering.

---

## ADR-035 — Calibration blobs move to NVS with a CRC, keeping EEPROM as a one-way migration source

**Status:** Accepted (Phase 6)

### Context

Calibration offsets were stored as a raw struct in emulated EEPROM, guarded by a
magic number (`0xDEADBEEF`).

A magic number answers exactly one question: *has anything ever been written
here?* It cannot answer *is what was written still intact?* A single flipped bit
anywhere in the payload leaves the magic untouched, so the blob validates and the
corrupted offsets are applied — silently biasing the aircraft's idea of level,
with nothing in the log to indicate it.

### Decision

`device::SettingsStore`, implemented over NVS as `Esp32SettingsStore`, storing
payload + CRC-32 and returning `Status::Corrupt` on mismatch. A corrupt blob is
logged and **ignored**, falling through to a fresh self-calibration.

Load order on boot:

1. NVS. If present and the CRC passes, use it — NVS is authoritative.
2. If the CRC **fails**, log and recalibrate. Never fall back to EEPROM here: a
   corrupt new blob must not silently resurrect an older calibration.
3. If NVS is empty, read the legacy EEPROM blob and write it to NVS.

Writes go to NVS, falling back to EEPROM only if NVS fails — losing a
calibration the operator just spent ten seconds producing is the worse outcome.

### No migration path at all

**Revised.** The first implementation read the legacy EEPROM blob and copied it
into NVS on first boot. That has been removed at the maintainer's direction: the
flash is being erased wholesale for this refactor, so the aircraft is
recalibrated and reconfigured from scratch either way.

Deleting the migration removed the EEPROM dependency entirely — the
`StoredCalibData` struct, the magic number, the dual-write fallback and
`<EEPROM.h>` itself. Migration code that runs exactly once, on hardware nobody
will ever run it on, is pure carrying cost: it cannot be exercised by any test
after the first boot and it keeps a dead storage backend alive in the build.

The general point is worth keeping: **a migration is only worth writing when
somebody is actually going to migrate.** Asking is cheaper than assuming.

### Consequences

- Wear levelling comes free: the Arduino EEPROM shim rewrites a whole flash page
  per commit, NVS manages its own.
- `constexpr` constructor required, because `BoardStorage` is `constinit`. The
  compiler caught this immediately — `Preferences` is opened per call rather than
  held open, which keeps the object trivially constructible and is also what
  makes the store safe to use from any task.
- CRC-32 is bitwise, not table-driven: the 1 KiB a table costs buys speed that
  a boot-time read of a few dozen bytes will never need.

---

## ADR-036 — Architectural greps belong in the layering check, not in a test binary

**Status:** Accepted (Phase 6)

### Context

`test_production_contracts.cpp` contained tests that read production source files
as text and asserted on their contents. They existed because the properties they
guarded could not be reached any other way: while the sampling logic lived inside
`ArduFliteIMU::update()`, holding a mutex and feeding the watchdog, nothing about
it could be executed on a host.

The Phase 6 cutover invalidated most of them at once — every identifier they
grepped for had moved — and that forced the question of what they were actually
for.

### Decision

Split them by what kind of claim they make.

**Properties about behaviour** move to real tests against the real class, and the
grep is deleted. Seqlock ordering is covered by `test_seqlock`, decimation by
`SlowSensorsAreSampledLessOften`, climb-rate seeding by
`FirstSampleProducesNoClimbRate`. A grep was only ever a proxy for these; once
the thing is reachable, the proxy is worse than the real test in every way — it
passes when the code is renamed and fails when it is merely reformatted.

**Properties about absence** cannot be behavioural — no test can assert that
nobody created a second task — so they stay greps, but move to
`tools/ci/check_layering.sh`, which is where this project already keeps
"grep the tree for structural violations":

- `src/estimation` must not call the RTOS directly (may use `hal::Scheduler`).
- No `baroTask` anywhere.
- No `_pauseRequested` / `_taskPaused`; no `vTaskSuspend` in the sampling path.

**Properties about a value that must not drift** — the calibration magic number —
were deleted along with the thing they guarded when EEPROM went.

### Consequences

- The unit test binary tests code, and the layering script inspects structure.
  Source-scraping inside gtest was always a category error; it was tolerable only
  while there was no alternative.
- The layering checks run comments-stripped. Both new checks tripped immediately
  on prose that *names* the forbidden API in order to explain its absence —
  which is exactly the comment you want to keep.
- One check needed narrowing rather than deleting:
  `ArduFliteController::pauseTasks()` legitimately calls `vTaskSuspend`, because
  stopping the control surfaces while someone holds the airframe is a safety
  choice, not a bus-ownership workaround. The check is scoped to the sampling
  path, with that reasoning recorded next to it.

---

## ADR-037 — `EulerAngles` carries three different quantities, and that caused a real bug

**Status:** Accepted (Phase 6). Records a defect and defers the structural fix.

### The observation

`EulerAngles` is a bare `{float roll, pitch, yaw;}`. It is used for:

| Quantity | Range | Example |
|---|---|---|
| Attitude, degrees | ±180 | `attitudeSetpointDegs` |
| Angular rate, deg/s | ±maxRate, typically 100–300 | `ControlMixer::mixRate()` output |
| Normalised surface command | −1…+1 | `ControlMixer::mixManual()` output, `actuatorCmd` |

Nothing in the type distinguishes them. Which one a given value holds depends on
the flight mode, read separately.

### The bug this caused

`ArduFliteController::innerLoopTask()`:

```cpp
localMode         = controller->mode;
localRateSetpoint = controller->pilotRateSetpoint;   // deg/s in RATE_MODE
...
if (!imuHealthy) { controller->mode = MANUAL_MODE; localMode = MANUAL_MODE; }
...
if (localMode == MANUAL_MODE) { actuatorCmd = localRateSetpoint; }   // -1..+1 expected
```

On an IMU failure while in `RATE_MODE`, the mode is demoted to `MANUAL_MODE`
*after* the setpoint has been read. The setpoint is still in deg/s. It is then
assigned straight to `actuatorCmd` and handed to `AirframeMixer::mix()`, which
clamps to ±1 — so **every deflected axis goes hard over** until the next
`ControlMixer` tick refreshes the setpoint, roughly 7–20 ms later.

A full-deflection transient on all axes, at the exact moment the IMU fails and
the pilot is taking over.

Note what did *not* go wrong: mode and setpoint are read together under one lock,
so this is not a race. The code is correctly synchronised and still wrong,
because the two values are consistent with each other but the *meaning* of the
setpoint changed underneath it.

**Fixed** by neutralising the setpoint on both the demotion and the restoration.
One tick of centred surfaces is the correct handover.

### Why the type would have prevented it

`actuatorCmd = localRateSetpoint` is the whole bug, and with distinct types it
would not compile. The conversion would have to be written explicitly, and
writing it would force the question "what does a rate setpoint mean as a surface
command?" — whose answer is "nothing, use neutral".

### Decision

**The structural fix is deferred, not rejected — it is Phase 6B** (§06).
Splitting into `AttitudeDeg`, `AngularRateDps` and `SurfaceCommand`: three
distinct quantities, three types, conversions named and explicit.

It is deferred because:

- It touches ~60 sites on the **command path to the servos**, which is the one
  part of the system where a mistake is immediately dangerous.
- Every site the compiler rejects needs a human decision, not a mechanical edit —
  the `MANUAL_MODE` passthrough above is exactly such a site, and the right
  answer there was not a cast.
- Phase 6 is the estimation layer. Bundling a control-path refactor into it would
  make the bench A/B unable to attribute any change in handling.

It is its own phase, with its own bench checklist, after the estimation work has
been verified on hardware.

### The rule this settles

The project already uses unit-suffixed field names with a generic vector type
(`ImuState::accel_g`). Phase 6B introduces distinct types instead. These are not
in conflict — the deciding factor is **whether the unit is visible at the point
of use**:

- `state.accel_g` cannot be touched without reading the unit. A suffixed field
  name on a generic vector is sufficient.
- `actuatorCmd = localRateSetpoint` contains no field name at all. Both sides are
  bare locals whose meaning came from a mode flag read elsewhere. Only the type
  can carry the unit across that assignment.

**Suffixed names where values are read in place; distinct types where they cross
boundaries and get assigned.**

### Not a units library

This is not a reversal of the "unit suffixes in names, not strong types"
decision. That was about scalars carrying units in their identifiers. This is
ordinary domain modelling: three different quantities that happen to share a
representation are not the same type. The suffix convention still applies to the
members inside them.

---

## ADR-038 — The unit needs exactly one carrier

**Status:** Accepted (Phase 6). Completes the units convention begun in Phase 1.

### Context

Phase 1 put unit suffixes on every unit-bearing config **key**
(`mix.max_att_yaw_deg`, schema v2). Phase 6B introduces distinct **types** for
the three quantities on the control path. That raises an obvious question: should
variables also be suffixed — `localRateSetpoint_dps`?

### Decision

**No, when the type already says it. Yes, when nothing else does.**

The unit must be recoverable at the point of use, and it needs **exactly one**
carrier:

| Situation | Carrier | Example |
|---|---|---|
| Type is specific | the type | `AngularRateDps rateSetpoint` |
| Type is generic, value is a struct member | the member name | `ImuState::accel_g` |
| Type is generic, value is a bare scalar | the identifier | `float deadbandRads` |

`AngularRateDps localRateSetpoint_dps` states the unit twice. That is not
harmlessly redundant — it is a **second source of truth**. Change the type later
and the suffix silently becomes a lie, which is worse than never having had it,
because a wrong unit annotation is trusted.

### The gap this closed

`MixerConfig`'s members were the case where nothing carried the unit at all:

```cpp
float maxAttRoll;    // degrees, per a section comment
float maxRateRoll;   // deg/s, per a section comment
```

The keys feeding them had suffixes since Phase 1; the members mirroring them
one-to-one did not, so the unit was lost precisely at the point of use. And these
are the worst possible pair to lose it on: attitude and rate limits differ by
roughly a factor of ten, read almost identically at a call site, and
`cfg.maxAttYaw` scaling a value later consumed as deg/s is the exact ambiguity
Phase 6B must resolve.

Renamed to `maxAttRoll_deg` / `maxRateRoll_dps` etc. The cross-axis mixing
coefficients are deliberately left unsuffixed — they are dimensionless ratios,
and a suffix would imply a unit that does not exist.

### Consequences

- After Phase 6B, control-path locals like `localRateSetpoint` keep their plain
  names. The type carries the unit; the name carries the role.
- The remaining unsuffixed unit-bearing scalars are `dt` (seconds, by universal
  convention here) and a handful of trig intermediates. Left alone: `dt` is
  strong enough that suffixing every occurrence is churn without benefit.
- "Should this be suffixed?" now has a mechanical answer: **is the unit already
  recoverable from the type?** If yes, no suffix. If no, suffix.

---

## ADR-039 — A setpoint carries what it IS, not what the mode was

**Status:** Accepted (Phase 6B)

### Context

Splitting `EulerAngles` into three types (ADR-037) exposed two further defects
of the same family. Neither was the type confusion itself; both were only
visible once the types forced the question.

### Defect 1 — the mode was read twice

`ControlMixer::handleChannelInput()` read the flight mode to pick a scaling:

```cpp
EulerAngles sp = mix(s_raw, s_ctrl->getMode(), &ok);   // scales for the mode
sendSetpoint(sp);                                       // pushes to a queue
```

`CommandSystem` then read the mode **again** to pick a setter:

```cpp
if (controller->getMode() == ATTITUDE_MODE) setAttitudeSetpoint(cmd.setpoint);
else                                        setRateSetpoint(cmd.setpoint);
```

Two reads, a queue in between. A mode change across that gap delivers a
rate-scaled value to the attitude setter, or an attitude to the rate setter.

The failsafe path made it concrete: it pushes a mode command and a setpoint
command as **separate queue entries**, so the setpoint could be interpreted
under the outgoing mode.

**Fix:** the command carries a `SetpointKind` tag alongside the value. The
mixer reads the mode once, scales, and labels. `CommandSystem` dispatches on the
label. What the value *is* travels with it.

### Defect 2 — one storage slot for two quantities

`pilotRateSetpoint` held deg/s in `RATE_MODE` and a normalised −1…+1 command in
`MANUAL_MODE`. That single slot is the root of ADR-037: on an IMU-failure
demotion to `MANUAL_MODE` it still held deg/s, which reached the mixer as a
surface command.

**Fix:** `pilotManualCommand` (`AxisCommand`) is now separate from
`pilotRateSetpoint` (`AngularRateDps`). The bug is no longer *representable* —
the inner loop's manual branch reads a slot that can only ever hold a normalised
command. The neutralise-on-demotion from ADR-037 is retained as defence in
depth, because that slot can still hold a **stale** manual command from before
`RATE_MODE` was entered.

### One deliberate behaviour change

`rate_sp_*` in the flash log previously showed the manual stick positions while
in `MANUAL_MODE`, because the sticks were living in the rate slot. With separate
slots that column would instead hold whatever rate was commanded *before* manual
was entered — stale, and worse in a log than a zero.

`setManualCommand()` therefore clears the rate setpoint. **`rate_sp_*` now reads
zero in MANUAL_MODE** rather than showing stick positions. The pilot's manual
input is still recorded, via `rate_cmd_*`, which carries the resulting
`AxisCommand`.

Anything parsing `rate_sp_*` across a manual-mode segment sees different values
than before. That is the one log-format consequence of Phase 6B.

### What this says about the exercise

The type split found no bugs by itself. It found them by making the compiler ask
questions at sixty call sites, three of which turned out to be places where two
quantities were being conflated deliberately-but-silently. A mechanical sweep
that answered each with a cast would have preserved every one of them.

---

## ADR-040 — `KeyValueStore` is byte-oriented, and that is the point

**Status:** Accepted (Phase 7). Implements ADR-027.

### Context

`ConfigPersistence` included `<Preferences.h>` and used its fifteen typed
accessors — `putFloat`, `getUChar`, `putString` and so on. That header is
Arduino-ESP32 specific and does not travel even to another FreeRTOS target,
which made the configuration system — one of the least hardware-dependent parts
of the firmware — one of the least portable.

### Decision

`hal::KeyValueStore` exposes four operations over raw bytes: `read`, `write`,
`erase`, `eraseAll`. Callers serialise their own values.

**Byte-oriented deliberately.** A typed API's fifteen methods differ only in the
width they write, and every one would have to be reimplemented on every
platform. `ConfigPersistence` already had a `switch` on `ConfigType`; it now
switches on size instead of on method name, which is the same code doing an
honest job.

`eraseAll()` was added to the interface. `erase(key)` cannot express a factory
reset, because the caller does not know every key an older firmware version may
have left behind.

### What the typed API was hiding

Preferences' getters take a default and return it silently on any failure:
missing key, wrong type, corrupt entry — all indistinguishable. The byte API
returns a `Status` and an `outLen`, which surfaced two decisions the old code
was making by accident:

1. **A read whose buffer is too small must REFUSE, not truncate.** Four bytes of
   an eight-byte value is not a smaller number, it is a wrong one.
2. **A length mismatch is how a type change is detected.** A key written as a
   1-byte `bool` and read as a 4-byte `float` would otherwise return three bytes
   of adjacent garbage as a plausible configuration value. `ConfigPersistence`
   now falls back to the default in that case.

Both are pinned in `test_key_value_store.cpp` against `MemoryKeyValueStore`,
which implements the same contract.

### Consequences

- **The on-flash format changed.** Typed NVS entries became raw byte blobs, so
  existing stored configuration is not readable and every parameter falls back
  to its default. Acceptable only because the maintainer is erasing flash for
  this refactor; it would otherwise need a migration.
- Config lives in its own NVS namespace (`aflite-cfg`), separate from the
  calibration blob (`arduflite`), so a config factory-reset cannot wipe the IMU
  calibration.
- `saveAll()` now counts only successful writes. It previously incremented
  unconditionally and reported "saved all N" even when every write had failed —
  precisely when the operator most needs to know otherwise.

---

## ADR-041 — No PowerMonitor until the hardware exists; stop faking it meanwhile

**Status:** Accepted (Phase 7)

### Context

Phase 7 lists `device::PowerMonitor` plus a driver as "the first end-to-end
exercise of ADR-019's add-a-sensor path". No board has battery sensing: no
descriptor entry, no pin, no config key, and `hal::Io` has no analog input.

The maintainer intends to add sensing as an **ADC input on a custom PCB**, but
that board is a long way off. The question was whether to build the ADC path now
against the known-in-principle design.

### Decision

**Not built.** Consistent with ADR-029 (`Mpu9250`), and for a sharper reason
than "no hardware to test on": **the interface shape depends on the consumer**,
and there is no consumer yet.

`hal::AnalogIn` looks trivial until you specify its return value. Raw counts?
Millivolts? A calibrated voltage? The ESP32's ADC is markedly non-linear and
needs per-chip calibration data, so whether calibration sits in the platform
layer or the driver is a real decision — and it is decided by what the caller
needs. Designing it now means guessing, and a guessed interface with no consumer
is not validated by anything.

The divider ratio, the ADC attenuation setting and the calibration curve are all
properties of a PCB that does not exist. None can be chosen, let alone verified.

**When the PCB exists**, the shape is: `hal::AnalogIn` in the platform layer
returning calibrated millivolts; `drivers::AdcBatteryMonitor` applying the
divider ratio and cell-count arithmetic behind `device::PowerMonitor`; one
descriptor entry naming the pin and ratio. That is a genuine end-to-end
exercise of ADR-019 at the point where it can be verified.

### The defect this surfaced

Investigating the above found that `ArdufliteCRSFTelemetry::sendBattery()` was
being called every telemetry cycle with the placeholder values:

```
battery_voltage   = 0.0f;   // -> 0.0 V on the pilot's radio
battery_current   = 0.0f;
battery_remaining = 100;    // -> 100 % remaining
```

The radio was being sent fabricated numbers, indistinguishable from measured
ones, in a field pilots use to decide whether to land. Simultaneously reporting
0.0 V and 100 % remaining is not even self-consistent.

**Fixed.** `TelemetryData::battery_valid` is false while no monitor is fitted,
and the CRSF backend suppresses the battery frame entirely. A radio showing no
battery telemetry is unambiguous; one showing a confident wrong value is not.

The flash log was already correct here — it omits the battery columns with a
"hardware not yet wired" comment. Only the live link was affected.

### The general point

A placeholder that is *visible* (a TODO, an omitted column) is a note to the
developer. A placeholder that is *transmitted* is a lie to the operator. The two
look identical in the source and are not remotely the same thing.

---

## ADR-042 — `Console` is a byte stream, not a line reader

**Status:** Accepted (Phase 7). Corrects the interface in §03.

### Context

`device::Console` was specified as:

```cpp
virtual std::size_t write(const char* s, std::size_t len) = 0;
/// Returns 0 if no complete line is available. Never blocks.
[[nodiscard]] virtual std::size_t readLine(char* dst, std::size_t maxLen) = 0;
```

Neither consumer wants a line. `ArduFliteCLI` reads byte at a time because it
echoes each character and handles backspace as it goes. `CLICommandsTelemetry`
drains the buffer without interpreting it at all.

### Why `readLine` was the wrong primitive

A console that assembles lines must also decide **whether to echo**, and echo is
a property of the consumer, not the port: the CLI echoes, the telemetry drain
must not. Putting line assembly behind the interface would have forced an echo
policy into the platform layer, and then forced a way to override it back out.

### Decision

```cpp
virtual std::size_t write(const char* s, std::size_t len) = 0;
[[nodiscard]] virtual std::size_t available() const = 0;
[[nodiscard]] virtual int readByte() = 0;    // -1 when empty
```

Line assembly stays where the echo policy already lives.

### Consequences

- `Logging`'s default handler formats with `vsnprintf` into a bounded 256-byte
  stack buffer and writes bytes, rather than calling the port's own `vprintf`.
  That is what lets it depend on `device::Console` alone. Longer lines truncate
  — log lines are diagnostics, not a transport.
- `Board::begin()` opens the console **first**, before the sensor bus, because
  everything below it logs and a line emitted before the port is open is lost.
- `Serial.` no longer appears anywhere outside `src/hal/`.
- The general point, again: an interface's shape is decided by its consumers.
  `readLine` was specified from what a console *sounds like*, not from what
  either caller needed — the same error as `Measurement::health()` (ADR-032)
  and the `AnalogIn` return type that ADR-041 declined to guess.

---

## ADR-043 — The console is not a UART

**Status:** Accepted (Phase 7). Corrects the Phase 7 wording in §06.

### Context

§06 Phase 7 specified "console over `hal::Uart`". That is wrong on the hardware
this project actually targets, and the two supported boards disagree with each
other about why:

| Board | `Serial` is | |
|---|---|---|
| Lolin C3 Mini | **USB CDC** — `cdc_on_boot=1` | the ESP32-C3's USB-Serial-JTAG peripheral |
| FireBeetle 2 ESP32-E | a real **UART0** | classic ESP32, no USB peripheral |

`hal::Uart`'s interface describes a UART: baud rate, RX and TX pins, RX
inversion. On the C3 none of those mean anything — a USB CDC endpoint has no
pins to assign and no line rate to set, and `Serial.begin(115200)` there is
accepted and ignored.

### Decision

`Esp32Console` implements `device::Console` over Arduino's `Serial`, not over
`hal::Uart`.

`Serial` is the correct level of abstraction here precisely **because the
underlying transport differs per board**. One implementation covers both, and
the thing being abstracted — a bidirectional byte stream to a developer's
terminal — is what `device::Console` already describes.

`hal::Uart` remains correct for the CRSF link, which genuinely is a UART with a
baud rate, assigned pins and inverted RX.

### Consequences

- The console cannot be pointed at an arbitrary UART by changing a descriptor
  field. That is not a capability being lost: on the C3 there is no UART to
  point it at, and a board that wants a second diagnostic port can add a
  `device::Console` implementation over `hal::Uart` beside this one.
- The general point, for the fourth time in this refactor: **an interface named
  after the hardware it happens to run on today outlives that hardware badly.**
  `Console` describes the role; `Uart` describes one possible transport. The
  spec named the transport.

---

## ADR-044 — Console, logging and UART: what was changed and what was left

**Status:** Accepted (Phase 7)

A review of the console/logging/serial/UART surface after the Phase 7 migration.
Two changes made, two declined, one recorded as a latent defect.

### Changed: `Console::flushOutput()`

`LogStore` had a `flush()`; `Console` did not. On the C3 the console is USB CDC
and writes are buffered, so a watchdog reset or panic can discard whatever has
not drained — **including the line written immediately before the reset**, which
is the one worth having.

Logging now flushes after every `Error`. Deliberately not after every level:
flushing at the rate `LOG_DBG` can fire would stall the writing task on USB, and
the lines that matter for a post-mortem are the errors.

### Changed: the log handler takes a `Console&`

It previously called `Board::instance().console()` inside each log call. That
made it untestable on a host and hid a dependency the type never declared — the
reach-for-a-global pattern the HAL exists to remove (§01). Now injected;
`ConsoleLogHandler` replaces `SerialLogHandler`, which was also named after a
transport that is not a UART on half the supported boards (ADR-043).

### Left alone: `hal::Uart`

Correct as it stands. The CRSF link genuinely is a UART with a baud rate,
assigned pins and inverted RX, and nothing else uses it.

### Left alone: multiple log sinks

`Logger` holds one `LogHandler`. Logging simultaneously to console and flash is
a plausible want, but nothing needs it today, and a sink list costs an
allocation or a fixed array plus iteration on every log call at 500 Hz. Add it
when something asks.

### Fixed: telemetry no longer emits through the logger

`ArduFliteQSerialTelemetry` and `ArduFliteDebugSerialTelemetry` write their
output with `LOG()` / `LOG_N()`, at `LogLevel::Clear`. Two consequences:

1. **`setLevel(Off)` silences telemetry**, because `Clear` is filtered like any
   other level. A log-verbosity control silently doubles as a telemetry switch.
2. **A log line can corrupt a machine-parsed stream.** The Q backend emits CSV
   for a real-time 3-D viewer; an `LOG_ERR` firing mid-flight interleaves into
   that stream, and the parser sees a malformed row.

Point 2 is the serious one, and it is **not hypothetical** — the flash CSV
writer has already produced at least one malformed row
(`FL002/log_006.csv`, a `flight_state` of `0.68`), which is worth investigating
with this mechanism in mind.

**Fixed.** Both backends now write through `arduflite::ConsoleWriter`, a
printf-style wrapper straight onto `device::Console` — no level, no tag, no
filtering. `setLevel(Off)` no longer stops telemetry, and a log line can no
longer land inside a CSV row.

The split that remains is the correct one: those backends still call `LOG_WARN`
and `LOG_ERR` for genuine diagnostics ("failed to create mutex", "begin() called
more than once"). Those belong on the logger. What moved off it is the *data*.

**The distinction being drawn:** diagnostics and data are different channels
that happen to share a wire. Sending them through the same filter and the same
formatter makes the wire's owner — the logger — the arbiter of whether data
gets out.

---

## ADR-045 — The extensibility proof failed on its first test, and that is why it exists

**Status:** Accepted (Phase 8)

### The test

§06 Phase 8 requires proving the sensor abstraction landed by adding a redundant
sensor **as a data-only change**: a second MPU-6500 at 0x69 in the board
descriptor, with its own axis map. "If this needs any code change, the sensor
abstraction did not land."

### The result

The descriptor change compiled cleanly with no code change at all — and was
**wrong**.

`BoardStorage` held `std::optional<drivers::Mpu6500> imu` — singular. A second
`SensorMount` of the same part called `emplace()` a second time, which
**destroys the first driver and constructs the replacement in its place**. The
consequences, none of which the compiler could see:

- `accelList[0]` and `accelList[1]` both pointed at the same object.
- `sensorList` held that one object twice, so the tick called `sample()` on the
  same chip twice per iteration.
- The redundant part at 0x69 was configured and then never read.
- `FirstHealthySelector` would have "failed over" from an instance to itself.

Every layer above `Board` was correct. The spans, the selector, the tick's
rate-aware sampling, the per-instance axis transforms — all of it was ready for
two IMUs. The storage underneath was not, and nothing else could tell.

### The fix

`BoardStorage` holds `std::array<std::optional<drivers::Mpu6500>, kMaxPerKind>`
with a separate driver counter, and `beginSensors()` emplaces into the next free
slot. Same for the barometer.

### Why this belongs in the record

**Compiling was not the test, and it nearly passed as one.** A data-only change
that builds is exactly what the criterion asked for, and reading the diff would
have shown a green build and a descriptor entry. The defect was only visible by
asking what `emplace()` does when called twice on one optional.

The criterion in §06 should therefore be read as stronger than its wording: a
data-only change must be *correct*, not merely *compile*. The next two proofs —
the ICM-42688 driver and the `host_sim` board — should be judged the same way.

The descriptor entry is not left enabled: no second IMU is fitted, and `Board`
would log a probe failure at 0x69 on every boot. A comment records that the
change is data-only and has been verified.

---

## ADR-046 — Decimation is derived from the sensor, never configured beside it

**Status:** Accepted (Phase 8). Fixes a defect introduced in Phase 6.

### The defect

`InertialSubsystem` decimated the barometer **twice, with two different numbers
that were never reconciled**:

- The `sample()` pass used a divider derived from the part:
  `taskRate_hz / nativeRate_hz()`. For a 15 Hz BMP280 at 500 Hz that is every
  **33 ticks**.
- The `read()` in step 8 used `Config::baroDecimation`, fed from
  `BARO_DECIMATION_FACTOR` = `BARO_UPDATE_INTERVAL_MS / IMU_UPDATE_INTERVAL_MS`
  = **every 10 ticks**.

So two of every three reads returned the identical cached conversion, and the
altitude EMA was fed duplicates. Worse, the derivative:

```cpp
baroDt_s = baroDecimation / taskRate_hz;      // 10/500 = 20 ms
climbRate = (filtered - lastFiltered) / baroDt_s;
```

divided by the **read** interval (20 ms) when new data arrives at the
**conversion** interval (66 ms) — overstating vertical speed by the ratio
between them. Climb rate feeds the vario and the flight-state machine.

### Why the interfaces allowed it

`nativeRate_hz()` lived only on `device::Sensor`. Step 8 holds a
`device::Barometer*` from the selector and could not reach the part, so the rate
had to come from somewhere else — and "somewhere else" was a macro pair in a
config header, related to the truth only by a comment.

### Decision

`nativeRate_hz()` moves onto `device::Measurement`, alongside `health()`, and
the read decimation is computed from the selected barometer itself.
`Config::baroDecimation` is deleted; there is no second number to disagree.

This is the third member to land on `Measurement` for the same reason: a
consumer holding a measurement interface needs to know something about it, and
routing that through the part is either impossible or a lie once redundancy
exists (ADR-032).

### The general rule

**A rate, a range or a resolution that describes a device must be asked of the
device.** Configuring it beside the device creates two sources of truth that
drift silently — here, silently enough to inflate a flight instrument by 3.3x
while every test passed.

`BARO_UPDATE_INTERVAL_MS` and `BARO_DECIMATION_FACTOR` are gone, which also
emptied `IMUConfiguration.h`.

---

## ADR-047 — Three classes finish the Phase 2 mutex migration, and host_sim is why

**Status:** Accepted (Phase 8)

### Context

`ArduFliteRateController`, `ArduFliteAttitudeController` and `ControlMixer` each
created a raw `xSemaphoreCreateMutex()` in their constructor and used
`SemaphoreLock` at 19 sites. Phase 2 migrated `ArduFliteController` and left
these three, and nothing noticed for six phases — because on the target they
work perfectly.

`host_sim` noticed immediately: these classes are otherwise pure arithmetic, and
a raw FreeRTOS handle was the single thing keeping them on the ESP32.

### Decision

The mutex is **injected**, not created: `setMutex(hal::Mutex*)` on the two
controllers, a parameter on `ControlMixer::init()`. All three take theirs from
Board's pool, which grew from three allocations to six.

Locking moved to `std::unique_lock`, uniformly. `hal::Mutex` already satisfies
the standard Lockable requirements, so no wrapper was needed.

### Why constructing one was wrong even on the target

These objects are constructed at **static-initialisation time**, before the
FreeRTOS scheduler exists. Creating a mutex there worked only because the ESP32
port tolerates it. Injection removes the question.

### The result

All three now report **zero** platform calls, and `host_sim` runs the complete
chain on a laptop: simulated sensors → `InertialSubsystem` → attitude loop →
rate loop → `AirframeMixer`. Commanded roll tracks to 0.01 degrees and the
control loops produce a correction opposing it.

`ArduFliteController` keeps its coupling and should: it owns the tasks.

### The boundary that remains

`initFromConfig()` couples both controllers to `ConfigRegistry`, which still
holds a raw `SemaphoreHandle_t` and an Arduino `String`. So the loops are
portable but **their configuration path is not**, and `host_sim` supplies fixed
gains through a stub instead.

That stub produced the round's sharpest lesson. Its first version returned
`0.0f` for any unrecognised key — including `outlimit`, the PID's output clamp.
Every gain read back plausibly, no error was logged anywhere, and both loops
output exactly `0.000`. **A defaulted-to-zero configuration value is
indistinguishable from a correctly configured system that has nothing to do.**
The real `ConfigRegistry` has a schema with defaults for exactly this reason;
the stub had to grow one too.

It also re-ran into ADR-037: `outlimit` means deg/s for the attitude loop and a
normalised −1…+1 command for the rate loop. One key name, two quantities —
the same conflation, in configuration rather than in code.

---

## ADR-048 — `ConfigRegistry` off FreeRTOS and off Arduino `String`

**Status:** Accepted (Phase 8). Completes the coupling `host_sim` exposed.

### Context

Migrating the three controllers to `hal::Mutex` (ADR-047) made them portable but
left their *configuration* path on the target: `initFromConfig()` reaches
`ConfigRegistry`, which held a raw `SemaphoreHandle_t` and an Arduino `String`.
`host_sim` had to stub the whole registry to link, and that stub is where the
zero-defaulted-`outlimit` defect came from.

### The mutex

`ConfigRegistry` created its mutex **lazily, with a compare-exchange**, because
parameters register at static-initialisation time before the FreeRTOS scheduler
exists. That is a real problem, solved elaborately.

Injection dissolves it. `setMutex()` is called once after `Board::begin()`; a
null mutex means "before that point", which is single-threaded by construction,
so the registry runs unlocked and correctly.

All **27 lock sites were left untouched.** A small `RegistryLock` adapter
presents the same `acquired()` surface the call sites already used, so the
migration changed the lock's implementation without editing a single caller.
A null mutex reports *acquired* — deliberately, since reporting failure would
have made every early registration silently fail.

### The string

`String` was in the registry's public API: `get<String>`, `set<String>`,
`getDirtyKeys()`, the stored key, the observer list. Replaced with
`std::string`, which works identically on the target.

Four call sites converted at the boundary — `WiFiManager` and the web server
still speak Arduino `String` because they talk to Arduino APIs, and that is the
right place for the conversion: at the edge that genuinely needs it, not in a
core interface every module includes.

`<Arduino.h>` is gone from `ConfigRegistry.h`.

### Result

Zero FreeRTOS and zero Arduino types in the registry. Burn-down 91 → 90.

### The pattern, three times now

`Esp32I2cBus`, `Esp32SettingsStore` and `LittleFsLogStore` were caught by
`constinit`. These four classes — two controllers, the mixer, the registry — were
caught by `host_sim`. All six are the same mistake: **acquiring a platform
resource in a constructor that runs before the platform exists.**

The fix is the same every time — take the resource, do not make it — and the
reason it keeps recurring is that creating it in the constructor *works on the
target*, so nothing complains until something tries to run the code somewhere
else.

---

## ADR-049 — A composed key is invisible to a rename

**Status:** Accepted (Phase 8)

### What happened

Commit `5c0a907` applied ADR-008 Option A to configuration: unit suffixes on
every unit-bearing key. 122 macro renames, 45 key-string renames, 14 files.

It missed `ConfigHelpers::buildPIDConfig()`, which does not use the macros:

```cpp
snprintf(key, sizeof(key), "%s.ti", keyPrefix);
```

Grepping for `CONFIG_KEY_RATE_ROLL_TI` finds every macro use and none of these.
The string `"rate.roll.ti"` never appears in the source — it is assembled at
runtime from a prefix and a literal suffix.

Two mismatches resulted, and both were silent:

| Composed | Schema | Effect |
|---|---|---|
| `%s.ti`, `%s.td` | `…ti_s`, `…td_s` | Every PID in both loops read 0 for I and D time. All ran as pure-P |
| `%s.outlimit` | `att.…outlimit_dps` | Attitude PIDs clamped their own output to 0. **ATTITUDE_MODE commanded no rate** |

`ConfigRegistry::get()` logs a warning and returns the type's default. The PID
accepts a zero gain without complaint. Every unit test passed for seven phases.

### Why it surfaced when it did

`host_sim` originally stubbed `ConfigRegistry`, and a stub answers every key —
so it masked exactly the failure it should have exposed. Only when ADR-048 made
the real registry host-buildable, and the stub was deleted, did the warnings
appear.

There is a smaller lesson inside that one: **the first version of that stub
returned `0.0f` for unknown keys and pinned both loops at zero**, which is the
same defect the real system had. I hit it in the fake, fixed it there, and did
not think to ask whether the real one had it too.

### Decision

`buildPIDConfig()` takes the output-limit suffix per loop — the two loops
genuinely differ, and ADR-037's distinction applies to configuration keys as
much as to types.

`tests/unit/test_config_keys.cpp` asserts that **every composed key resolves to
the same value as its macro-spelled counterpart**. Value comparison, not
spelling: a zero can be legitimate (`rate.yaw.ti_s` is deliberately 0, to avoid
yaw drift without a magnetometer), so "is it non-zero" is the wrong question and
an earlier draft of the test failed on exactly that.

Verified by reverting each fix in turn; both are caught.

### The general rule

**A key that is composed rather than named cannot be found by renaming the
name.** Any string built with `snprintf`, concatenation or a prefix plus a
literal is invisible to the search that a schema migration relies on.

Where a key must be composed, a test has to assert it resolves — because the
registry's own defaulting behaviour is what turns a typo into a plausible
number instead of an error.

---

## ADR-050 — BMI323 driver, and what a 16-bit word-addressed part costs

**Status:** Accepted (Phase 8+)

### Context

The maintainer bought a DFRobot SEN0697 10-DOF module: **BMI323** (6-axis,
I2C 0x69), **BMM350** (magnetometer, 0x15), **BMP581** (barometer, 0x47) —
three Bosch parts, all I2C. This ADR covers the BMI323; the other two follow.

Register values were taken from Bosch's own driver as shipped by DFRobot
(`bmi3_defs.h`, `bmi3.c`) and the datasheets, **not from recollection**. That
mattered immediately: the first thing checked was a half-remembered "BMI270-family
parts prepend a dummy byte on I2C", and the truth is **two** bytes on I2C and
**one** on SPI — the opposite way round from what was recalled.

### Three ways this part differs from the MPU-6500

Each fails silently — wrong numbers, never an error:

| Difference | Consequence of getting it wrong |
|---|---|
| Every I2C read prepends **2 dummy bytes** | the whole burst shifts by two, yielding plausible values |
| Registers are **16-bit little-endian** | the MPU-6500 is big-endian; a swap gives a wrong-but-believable number |
| Registers address **WORDS, not bytes** | `ACC_CONF` is 0x20 and `GYR_CONF` is 0x21 — adjacent registers, not adjacent bytes |

All three are pinned by tests that fail if the convention is reversed.

### What the test fake got wrong, twice

The driver was right; the **fake** was wrong, in two different layers:

1. It modelled the dummy prefix by *storing* filler in the two register slots
   before the target. That made a read-modify-write read a different address
   than it wrote. The prefix is a **bus** artefact generated on read, so it is
   now generated on read.
2. It modelled registers as **bytes**, so a two-byte write to `ACC_CONF` (0x20)
   spilled into `GYR_CONF` (0x21) — configuring the gyroscope silently corrupted
   the accelerometer. `FakeRegisterDevice` gained a `wordAddressed` mode.

A third error was in the test's own helper: it reconstructed a 16-bit value from
the byte-indexed write log, so `registerValue(0x20)` picked up 0x21's low byte
as 0x20's high byte. Assertions now read the resulting register state, which is
the property actually of interest.

**A fake that models the wrong addressing scheme does not fail — it agrees with
a driver that models it the same wrong way.** Only the register-adjacency
collision exposed it.

### Also found: error injection was unreachable

`FakeRegisterDevice::readRegs()` returned early on the new prefix path *before*
checking `failNextRead`, so every error-injection test on a prefixed device
silently passed by never injecting anything. Caught because the BMI323's
stale-sample test expected `IoError` and got `Ok`.

### The extensibility proof, second run

Swapping the board's IMU from MPU-6500 to BMI323 is **one line** in the board
descriptor — part and address — and nothing else changes. That is what ADR-019
promised and what ADR-045's first attempt failed to deliver.

Not left enabled: the module is not wired to the aircraft yet, and `Board` would
log a probe failure at 0x69 on every boot. The descriptor records the exact
change.

---

## ADR-051 — BMP581 driver; BMM350 deferred until something reads a magnetometer

**Status:** Accepted (Phase 8+)

### BMP581 — done

Bosch's newer barometer, and much simpler than the BMP280 it replaces: it
outputs **compensated** values directly. Temperature is a signed 24-bit count
over 65536 degC, pressure an unsigned 24-bit count over 64 Pa. There is no NVM
trimming block, no `im_update` wait, and no compensation polynomial — the whole
apparatus ADR-030 and ADR-031 had to build around the BMP280 simply does not
apply.

What is left to get wrong is framing, and all of it fails silently:

- 24-bit **little**-endian, XLSB first
- **temperature before pressure** — the opposite order to the BMP280, so a
  copied offset swaps the two channels
- two different scale factors, /65536 and /64 — swapping them is a factor of
  1024, which reads as a broken sensor rather than a broken driver
- temperature is **signed**; read unsigned, −10 degC becomes about +256 degC

Each is pinned by a test. 13 in total.

### The extensibility claim, now measured

Moving the aircraft from MPU-6500 + BMP280 to the SEN0697's BMI323 + BMP581 is
**two lines** in the board descriptor — part and address on each of two entries.
No code change anywhere. Verified to build; not left enabled, because the module
is not wired yet and `Board` would log two probe failures every boot.

That is ADR-019's promise, measured rather than asserted — and worth contrasting
with ADR-045, where the first extensibility test compiled cleanly and was wrong.

### BMM350 — deliberately not written

The magnetometer is a materially bigger job AND has no consumer.

**Bigger:** unlike the other two, the BMM350 needs its factory compensation read
out of OTP word by word through a command/status handshake, then unpacked into
offset, sensitivity, TCO and TCS coefficients and applied with cross-axis
correction. Bosch's driver is ~1800 lines, most of it that.

**No consumer:** `InertialSubsystem` calls `estimator.update()` — the six-axis
path. `updateWithMagnetometer()` exists and is never called. `ImuState::mag_ut`
is published from a filter nothing feeds. A BMM350 driver would produce readings
that go nowhere.

Using it is not a driver task but a **feature**: nine-axis fusion, hard- and
soft-iron calibration, and a decision about what happens to heading when the
magnetometer fails or is disturbed. It also changes flight behaviour directly —
`rate.yaw.ti_s` is deliberately 0 today precisely because there is no heading
reference, and that choice would need revisiting.

Writing the driver first would be the ADR-029 mistake again: unverifiable code
for a capability nothing yet uses. It should be written **with** the estimation
work that consumes it, so the two can be verified together.

---

## ADR-052 — BMM350 driver and optional nine-axis fusion

**Status:** Accepted (Phase 8+). Supersedes the "deferred" half of ADR-051.

ADR-051 deferred the BMM350 on the grounds that a driver with no consumer is
unverifiable code. That still holds — so the driver and its consumer were
written together, in one change, and neither shipped alone.

### The driver

Ported from Bosch's implementation as shipped by DFRobot, not derived. The parts
that are easy to get wrong, and how each is pinned:

| Hazard | Consequence if wrong | Pinned by |
| --- | --- | --- |
| Two dummy bytes on every I2C read | Chip ID reads 0xAA | `ReadsSkipTheTwoDummyBytes` |
| OTP handshake (write address, poll status, read MSB+LSB) | Silent zero trimming | `BeginReadsEveryOtpWord` |
| Coefficients packed across shared 16-bit words | A term lands in its neighbour | `UnpacksCompensationCoefficientsBitExactly` |
| 12-bit and 8-bit sign extension | Offsets wrong by a full span | same |
| Two fixed corrections — `+0.01` on sens Y, `−0.0001` on TCS Z | Invisible at room temperature, grows with the delta | two tests |
| TCO **adds**, TCS **divides** | Dimensionally plausible, numerically close near T0 — i.e. close on the bench | two tests |
| Cross-axis solve is a 2×2 inverse, not two subtractions | Heading error that scales with tilt | two tests |
| Separate XY and Z scale factors | Horizontal projection tilts with bank angle | `ScalesZDifferentlyFromXAndY` |

28 tests. The OTP handshake is emulated in the fake rather than pinned high, so
the timeout and error-bit paths are reachable — a part that never completes must
fail the boot, not hang it, and there are 32 of these reads.

The trimming block is read once at `begin()` and the OTP array is powered down
immediately after. It is the slowest sensor bring-up on the board.

### Optional, decided per tick

The magnetometer is **not** a configured mode. `Board::magnetometers()` returns
a span that is empty on every board that does not declare one, and
`FirstHealthySelector::primaryMag()` returns null for an empty span. The tick
branches on what it actually got:

```
magValid = magnetometer != nullptr
        && health() == Ok
        && read() == Ok
        && |field|² > 1 µT²
```

Nine-axis when `magValid`, six-axis otherwise. Three properties follow, and each
is a test:

- **A board without one needs no opt-out.** No enable flag, no config key,
  nothing that can fall out of step with the descriptor. This is the same rule
  as ADR-046: derive it from the hardware, never configure it twice.
- **Failure degrades on the next tick, not at the next boot.** A magnetometer
  that dies in flight stops contributing immediately, rather than pinning the
  heading to wherever it was pointing when it failed. Deciding once at `begin()`
  would have been simpler and wrong.
- **A zero field is not a reading.** It is what an unconfigured part returns,
  and it is also what a filter divides by when it normalises the vector to a
  direction. Rejecting it here keeps that decision out of the fusion code, where
  it would depend on which filter is installed.

The low-pass is advanced **only** on valid ticks. Feeding it zeros on a failed
read would walk the field towards the origin and eventually under the rejection
threshold — turning one dropped read into a permanent loss of heading.

`ImuState::magnetometerFused` is published rather than inferred from a non-zero
`mag_ut`: it is the difference between a heading and integrated gyro drift, and
the log should say which one it recorded.

### applyMeasurement, not applyAngularRate

The field is a true vector. It shares the IMU's mount and therefore its axis
map, but a mirrored mount must **not** negate it the way it negates a gyro
reading — the determinant belongs to the pseudovector path (ADR-006). Using the
wrong one inverts the heading on exactly the boards that need the transform
most. Pinned by `AMirroredMountDoesNotNegateTheField`.

### The rate mismatch is deliberate and harmless

The part converts at 100 Hz inside a 500 Hz tick, so `sample()` is decimated by
the existing rate-aware machinery (ADR-046) and the same reading is fused five
times over. That is fine: the filter uses the field as a **direction**, so a
repeated direction applies a consistent correction rather than an accumulating
one. Contrast the barometer, where the repeat mattered because the value feeds a
derivative.

### What this does NOT do

- **No hard-iron calibration.** The OTP trimming corrects the part; it cannot
  correct the airframe's own magnetics. Until a figure-of-eight calibration
  exists, heading carries whatever bias the motor, battery and servo leads
  contribute. This is the next piece of work, not an oversight.
- **`rate.yaw.ti_s` stays 0.** ADR-051 flagged that the yaw rate loop has no
  integral term *because* there was no heading reference. There is one now on a
  board that fits the module — but changing a gain on the strength of an
  uncalibrated heading would be the wrong order. Hard-iron first, then bench
  verification, then the gain.

### Extensibility, measured again

The whole SEN0697 — BMI323 at 0x69, BMM350 at 0x15, BMP581 at 0x47 — is now
three descriptor entries and `.sensorCount = 3`. Verified to build; 28 bytes
larger than the two-part descriptor, because all three drivers were already
linked. Not left enabled: the module is not wired yet.

---

## ADR-053 — one list, two counters: the barometer span reported the wrong one

**Status:** Accepted (Phase 8+). Records a defect found while wiring ADR-052.

`BoardStorage` keeps **driver-slot** counters (`baroDriverCount`,
`baro581DriverCount`, one per concrete driver type, because each type needs its
own `std::optional` array) and **span** counters (`baroCount`, counting interface
pointers in the shared `baroList`). One part can appear in several spans, so the
two are genuinely different numbers.

`Board::barometers()` returned `baroDriverCount`. Two failures followed:

- **BMP581-only board — no barometer at all.** The BMP581 branch appended to
  `baroList` using `baroCount`, but the span was sized by `baroDriverCount`,
  which that branch never touched. `barometers()` returned an **empty span**: no
  altitude, no climb rate, and no error anywhere, because an empty span is
  exactly what a board with no barometer legitimately returns. This would have
  been the first symptom of flying the SEN0697.
- **BMP280 board — a leading null.** That branch incremented `baroDriverCount`
  and then used it *again* as the `baroList` index (`baroList[baroDriverCount++]`),
  writing to slot 1 and leaving slot 0 null. It worked only because
  `firstHealthy()` skips nulls — so the shipping boards fly, with a null in the
  span and `activeBaro` reporting 1 for the only barometer fitted.

Both fixed: every branch appends with `baroCount`, and the span reports
`baroCount`.

**The lesson, and it is a repeat.** ADR-045 recorded that a second sensor
compiled cleanly and destroyed the first. This is the same shape: two counters
that are *usually* equal, so every arrangement compiles and the common case
works. Both were found by adding a second thing of an existing kind — which is
now the standing way to test the composition root, because reading it does not
work.

**Not host-testable today.** `Board.cpp` pulls in `Esp32Mutex` and the ESP32 I2C
driver, so it is outside the host suite; the fix is verified by building all
three boards and by the SEN0697 descriptor proof above. Making the composition
root host-testable is worth doing, and is recorded as outstanding rather than
done.

---

## ADR-054 — `rate.yaw.ti_s`, and what actually happens when fusion switches

**Status:** Accepted (Phase 8+). Amends ADR-052.

Two questions, and answering the second changed the first.

### The yaw rate loop never needed a magnetometer

The schema said:

> `Yaw rate I time constant (s) - 0 disables, prevents drift without magnetometer`

That attributes the zero to the wrong loop. The **rate** loop compares a
commanded yaw rate against the gyro. No heading is involved, and no heading
reference could help it. What the comment describes belongs to the **attitude**
loop, where a drifting yaw estimate would wind up an integrator chasing a
reference that is itself moving — and `att.yaw.ti_s` is 0 as well, correctly.

The indirect argument for `rate.yaw.ti_s = 0` was real but weak: with a drifting
yaw estimate the attitude loop commands a small persistent yaw rate, and a rate
integrator will faithfully hold rudder to deliver it. Zeroing the integrator
made the aircraft *ignore* the drift instead of flying it. That masks a symptom
in the outer loop by disabling a term in the inner one.

Except — see below — the yaw estimate does not reach the attitude loop either.

**Set to 8.0 s.** In this parameterisation `Ki = Kp / Ti`, so a longer time
constant is the *conservative* direction:

| Loop | Kp | Ti (s) | Ki |
|---|---|---|---|
| `rate.roll` | 0.09 | 1.40 | 0.064 |
| `rate.pitch` | 0.04 | 5.00 | 0.008 |
| `rate.yaw` | 0.05 | **8.00** | **0.006** |

Deliberately slower than either existing axis: the rudder is a weak, laggy
control and this loop has never flown with an integrator. It is still a useful
term, not a token one — against a standing 10 °/s rate error it contributes
about 60 % of output range over ten seconds. `rate.yaw.headroom` is 0.80, so
anti-windup already protects it.

**This default only reaches an aircraft with wiped flash.** `ConfigPersistence::load()`
takes the stored NVS value when the key is present and falls back to the schema
default only when it is absent. Every board that has run the current firmware
has `0.0` written. Changing the schema and reflashing will do **nothing** until
the key is erased or set explicitly from the CLI.

### Switching between six- and nine-axis fusion: what actually happens

The estimate does **not** jump. `Adafruit_Madgwick::update()` and
`updateIMU()` integrate the same `q0..q3`; neither re-initialises anything. So
the switch is a change in which correction is applied, not a change of state.

Three real effects, in ascending order of importance:

**1. Yaw slews in — and nothing steers by it.** On engaging, the heading
estimate rotates toward magnetic north at roughly `beta` rad/s. With
`imu.madgwick.beta = 0.1` that is about **5.7 °/s**, so a 40° error takes some
seven seconds. That would matter if yaw were a control reference. It is not:

```cpp
// ArduFliteAttitudeController.cpp
rateOut.yaw = localAttitudeSetpointDegs.yaw;   // pilot passthrough
```

`pidYaw` is constructed, configured and reset, and is **never called**. `yawErr`
is computed, deadbanded, and then discarded. The pilot's yaw stick is a yaw
*rate* command in every mode. So the estimated yaw angle reaches telemetry and
the flash log and stops there — the slew is a log artefact, not a flight event.
(That `yawErr` is computed and dropped is a trap for the next reader and worth
cleaning up separately.)

**2. Roll and pitch lose some correction authority — and everything steers by
them.** This is the effect that matters, and it is not obvious from the
interface. Madgwick sums the accelerometer residual and the magnetometer
residual into one gradient and normalises them **together**:

```c
recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);   // one step, both terms
```

The corrective step is unit-length regardless of how many sensors contributed.
So a large magnetic residual does not add correction — it **takes the
accelerometer's share**. While the heading slews in, roll and pitch are
corrected more weakly and lean harder on the gyro.

The transient version of this is brief. The *persistent* version is the real
warning: an uncalibrated hard-iron offset produces a magnetic residual that
never converges, permanently starving the accelerometer term and letting roll
and pitch drift on gyro bias. **That, not the yaw error, is why hard-iron
calibration must precede trusting nine-axis fusion in flight** — it degrades the
two axes the control loops actually fly.

**3. Chatter, which had no guard at all.** The per-tick decision in ADR-052 had
no hysteresis. A magnetometer whose health flickers, or whose field hovers near
the 1 µT floor, would alternate every tick at 500 Hz — repeatedly stealing and
returning the accelerometer's correction budget.

### The fix: engage slowly, drop instantly

```
valid && streak >= taskRate_hz * 1.0s  ->  nine-axis
otherwise                              ->  six-axis, streak = 0
```

**One second** of unbroken good readings to engage; one bad tick to drop.
The asymmetry follows directly from effects 1 and 2: **losing** the magnetometer
costs a heading nothing steers by, so there is no reason to wait; **regaining**
it costs roll and pitch authority, so there is every reason to be sure first.
The low-pass is still advanced during the wait, so fusion engages against a
settled field rather than a cold one.

Why a full second rather than something reflex-fast — three arguments that all
point the same way:

- **Distinct conversions, not ticks.** The part converts at 100 Hz and `read()`
  is cached, so a tenth of a second is only about **ten** real conversions.
  Enough to rule out one dropped burst; not enough to rule out an intermittent
  fault, which is precisely the case the delay exists for. A second is ~100.
- **The filter has not settled.** `imu.mag_alpha` defaults to 0.04, so at 500 Hz
  the field low-pass has a time constant near 50 ms. Engaging at 100 ms is
  roughly 2 tau — against a field still visibly moving. One second is 20 tau.
- **The trade is one-sided.** Waiting costs a heading no control loop reads.
  Engaging early costs roll and pitch authority. There is no symmetric pressure
  to engage quickly, so the delay should be set by the slowest thing being
  ruled out, not the fastest thing being lost.

Expressed as a duration rather than a tick count, so it does not silently change
meaning with the task rate. Seven tests, including the flickering case, the
counter-saturation case (a wrap would drop fusion for a second every couple of
minutes) and the `resetFilters()` case.

### Still outstanding

Hard-iron calibration. Until it exists, nine-axis fusion should be treated as
**bench-only** — not because the heading is poor, but because of effect 2.

### Correction: heading belongs on ROLL, not on yaw

An earlier draft of this ADR proposed a heading-setpoint integrator driving
`pidYaw`. **That pairing is wrong for a fixed wing** and would have flown badly.

A fixed wing changes heading by **banking**, not by yawing:

```
psi_dot = g * tan(phi) / V
```

The rudder's job is *coordination* — killing the sideslip a turn generates, and
offsetting adverse yaw. It is not a heading actuator. Hold a heading setpoint on
the yaw axis and every aileron turn creates a heading error the rudder tries to
null with the wrong surface: it opposes the turn while the wings are banked,
which produces sideslip rather than heading change. On an airframe with dihedral
that sideslip then rolls the aircraft *against* the commanded bank. The loop does
not merely fight the turn — it uncoordinates it.

**The correct cascade adds one outer loop on roll and leaves yaw alone:**

```
heading error -> bank angle command (limited, ~25 deg)
              -> existing roll ATTITUDE loop   (deg    -> deg/s)
              -> existing roll RATE loop       (deg/s  -> surface)
```

The yaw axis keeps exactly what it does today: pilot rate passthrough plus the
`mix.yaw_from_roll` feedforward (0.10) that already compensates adverse yaw.
`pidYaw` stays unused, and if the yaw axis ever earns a controller it should be
a yaw *damper*, not a heading hold.

Placing the heading loop in the **setpoint path** — producing `sp.roll` before
`ArduFliteAttitudeController` sees it — means neither attitude nor rate
controller changes at all.

### The setpoint must track while the pilot is commanding

Independent of which axis drives it, a frozen heading setpoint is wrong:

1. pilot banks through a 90 degree turn;
2. the setpoint stays on the old heading, accumulating 90 degrees of error
   (and winding up any integral term);
3. the pilot centres the stick and the loop commands a hard bank back.

So the heading setpoint **syncs to the measured heading whenever the pilot
commands roll or yaw**, and holds only when both are centred. That is precisely
"fingers off the stick, fly straight on the current heading", and it removes the
windup case at the same time.

### A dimensional trap in the existing mixer

`mix.yaw_from_roll = 0.10` is applied as:

```cpp
sp.yaw += cfg.mixYawFromRoll * (raw.roll * cfg.maxAttYaw_deg);
```

That coefficient is tuned against `sp.yaw` meaning a **rate**. Reinterpreting
`sp.yaw` as an angle without touching the mix turns a roll-stick deflection into
a `0.10 * 180 = 18 degree` heading offset. Any change to what `sp.yaw` means must
revisit this line — it is the kind of unit reinterpretation ADR-037 and ADR-049
were both about.

### Gain scheduling, and what is acceptable without airspeed

Turn rate for a given bank scales as `1/V`, and there is no airspeed sensor, so
the heading loop's effective gain varies with speed — highest when slow. A
conservative fixed gain plus a firm bank limit is the standard answer and is
adequate here; it is worth stating explicitly rather than discovering it as
"the heading loop oscillates on slow approaches".

### Revised sequencing

1. hard-iron calibration (nothing may consume the heading before this);
2. heading-hold as an outer loop on **roll**, in the setpoint path, with
   sync-while-commanded;
3. boot-latched enable — heading hold available only when a magnetometer was
   present and healthy at init, never re-decided per tick, with a defined and
   logged fallback on sustained loss.

Heading hold is the first feature that genuinely requires the magnetometer:
gyro-integrated yaw is good enough for seconds, and drifts over a minute.

---

## ADR-055 — the magnetometer is instrumentation, not a control input

**Status:** Accepted (Phase 8+). Settles the question ADR-052 and ADR-054 left open.

### Nothing on this aircraft wants a heading

Checked rather than assumed:

- `MissionPlanner` is a **timed attitude script** — roll, pitch, duration. No
  waypoints, no navigation, no heading.
- There is **no GNSS driver**. `device::Gnss` is declared; nothing implements
  it, and nothing populates `gps_heading`.
- `pidYaw` is constructed, configured, reset — and never called. Yaw is a pilot
  rate passthrough in every mode.

So the magnetometer's only product has no consumer, and the one feature that
would use it (heading hold, ADR-054) is not built.

### And its value is smaller than it looks

Heading hold needs an *absolute* heading only in proportion to how long the
heading is held. After a good gyro calibration, residual bias runs around
0.3 deg/s — roughly 9 degrees over 30 seconds. For the straight legs an RC
glider actually flies, **the calibrated gyro is already sufficient**. The
magnetometer earns its keep on minute-plus holds and on navigation.

Against that: nine-axis fusion has a real cost even when the heading is unused.
Madgwick normalises the accelerometer and magnetometer residuals into one unit
gradient (ADR-054, effect 2), so an uncalibrated field permanently takes
correction authority away from the accelerometer and lets **roll and pitch**
drift — the two axes the control loops fly on.

**Enabled today, the magnetometer is pure downside: a real risk to the axes that
matter, in exchange for a number nothing reads.**

### And if the goal is navigation, GNSS is the better investment

For a fixed wing that is always moving, GNSS course-over-ground beats magnetic
heading as a navigation reference: it accounts for wind drift, needs no
hard-iron calibration, and is immune to motor interference. The magnetometer's
advantages — valid when stationary, high rate, no satellite lock — mostly do not
apply to an aircraft in flight. Hard-iron calibration should therefore **not**
be built speculatively ahead of a GNSS driver.

### Decision

Keep the driver. Do not fuse. Publish the data.

1. **`drivers::Bmm350` stays.** Written, 28 tests, ~4 KB. It makes the SEN0697
   fully supported and costs nothing to keep.
2. **`imu.fuse_mag` defaults to `false`.** With it false the part is still
   probed, sampled, transformed, filtered and published — it simply does not
   steer the estimate. The nine-axis path and its engage delay (ADR-054) remain
   intact and tested, ready for the reassessment.
3. **The field reaches telemetry**, so the part can be *evaluated* on a real
   airframe rather than argued about.
4. **Reassess when GNSS navigation lands**, not before.

### On the "no enable flag" principle

ADR-052 said the board descriptor is the single source of truth and an enable
flag would be a second place for the same fact. That still holds — for the fact
it was about. *Fitted* is a hardware fact the descriptor owns. *Trusted in the
fusion loop* is a judgement about calibration quality. They are two different
propositions and both must be true; this is an AND of two facts, not a duplicate
of one. The descriptor still solely decides whether a magnetometer exists.

### What was added to telemetry, and why each

| Column | Why it is there |
| --- | --- |
| `mag_x/y/z` | The raw material. A hard-iron fit is a sphere fit over these; nothing else can substitute |
| `mag_heading` | Tilt-compensated, the human-readable number |
| `mag_field` | **The diagnostic that decides everything.** See below |
| `mag_valid` | Distinguishes "no magnetometer" from "magnetometer reading zero" |
| `mag_fused` (struct only) | So a log cannot be misread as nine-axis when it was not |

**`mag_field` is the one that matters.** Earth's field is 25-65 uT and its
*magnitude* does not depend on which way the aircraft points. So:

- magnitude that moves with **orientation** -> hard-iron offset, correctable;
- magnitude that moves with **throttle** -> the motor, and **not** correctable
  by hard-iron calibration, which corrects a constant offset and not a
  throttle-dependent one.

If the field swings appreciably between idle and full throttle on this airframe,
the entire heading line of work is dead here and no calibration will revive it.
That is a ten-minute bench test and it should precede any further work.

### Placement

`magneticHeading_deg()` and `magneticFieldStrength_ut()` are free functions in
`src/estimation/MagneticHeading.h`, called from `TelemetryData::update()` — at
telemetry rate, **not** in the tick. Six transcendental calls at 500 Hz on a
part with no FPU (§00 2.9b) is real cost for a number nothing steers by.

Ten tests. The one that matters is not "north reads zero" but **heading is
invariant under roll and pitch**: an uncompensated `atan2(my, mx)` passes every
level test and then swings tens of degrees in a banked turn, which reads as a
noisy magnetometer rather than as missing maths. One test deliberately computes
the uncompensated heading on the same input to prove the invariance tests are
not vacuous.

### Log compatibility

Six columns **appended**, never inserted. The log is CSV with a named header;
`tools/data_analysis` reads it with pandas by column name and
`tools/csv_viewer` reads the header, so appending is backward compatible and
older logs still parse. Inserting would silently shift every existing parser.

---

## ADR-056 — Phase 9: own Madgwick, and the oracle that could not have worked

**Status:** Accepted (Phase 9). Implements ADR-017 and corrects part of it.

### What landed

`estimation::MadgwickEstimator` replaces `AdafruitMadgwickEstimator`, and the
Adafruit wrapper and its library dependency are **deleted** — from the firmware,
from `host_sim`, from the unit-test build and from both CI workflows. Keeping a
GPL-noticed file around "for rollback" would have left the reason for the work
unaddressed.

Written from the published algorithm (Madgwick 2010) in the paper's own
structure: build the objective `f`, build its Jacobian `J`, take the gradient as
`J^T f`. The reference implementations flatten that into one scalar expression
per quaternion component — faster, and far harder to check. Keeping the two
steps apart makes every matrix entry a single partial derivative that can be
verified by hand, which is what made the verification below possible.

**Not clean-room in the strict sense** — the Adafruit source was read during
this work. The algorithm is published and reimplementing from the paper is the
normal route, but if the licence question is the point, that is worth knowing
rather than assuming. Not legal advice.

### The oracle in ADR-017 could not have worked

ADR-017's whole argument for sequencing the rewrite after the abstraction was
that L3 replay would let both implementations run over the same flight and have
their output diffed. Building that comparison is how the following was found.

`Adafruit_AHRS_Madgwick.cpp` normalises vectors with the Quake inverse-square-
root trick, over this union:

```c
union { float f; long i; } conv = {x};
```

`long` is **4 bytes on both ESP32 targets and 8 bytes on the host** (verified:
`riscv32-esp-elf-g++` and `xtensa-esp32-elf-g++` both report
`__SIZEOF_LONG__ 4`; the host reports 8). On a 64-bit platform the union reads
four bytes of uninitialised memory, and the bit-trick operates on garbage. The
two Newton iterations drag the magnitude back — and the result comes out
**negated**. Measured relative error: exactly 2.0, i.e. `-1x`, for every input
tried.

So on a host, that filter normalises its accelerometer, its gradient and its
quaternion the wrong way round.

**The aircraft was never affected.** On a 32-bit target the union is the right
size and the routine is correct to ~3e-6. This is a host-only artefact.

But it means every host measurement this project ever took of that library was
of a filter that does not exist on the aircraft:

- the replay's "baseline mean 1.51 deg";
- the injected-defect table in `test_log_replay.cpp` (gyro sign flip 6.11, and
  the two defects recorded as NOT caught);
- the exact output fingerprint, which pinned broken behaviour as correct.

Had Phase 9 simply asserted equivalence with that baseline, a **correct**
replacement would have failed and the obvious fix would have been to bend the
new implementation towards the broken one.

**The replacement is right and the old host behaviour was wrong.** Confirmed
three ways: the gradient was checked term-by-term against Madgwick's published
simplification and is algebraically identical; a hand-computed single step
matches the new implementation exactly and the old one not at all; and over the
FL001 flight the new filter tracks the recorded attitude *better* — mean 1.51
degrees against 1.57.

### The oracle that does work

The log itself. Those roll and pitch columns were produced **on the aircraft**,
by the correct 32-bit path. `test_log_replay.cpp` now replays through the new
filter and compares against them, and the test is **unconditional** — it used to
be skipped whenever Adafruit_AHRS was absent, which made the highest-value test
in the suite the one least likely to run.

### The fingerprint had to go too

The exact-hash change detector changed when the replay helper became a template,
and changed again when one line recording results was added. Neither touches any
arithmetic. On arm64 the compiler may contract `a*b+c` into an FMA and whether it
does depends on inlining, so the hash tracked code generation as much as
behaviour — and over 833 recursive steps a one-ulp difference compounds past the
0.01 degree quantisation.

A test that fails on refactors trains the reader to update the constant, which
is exactly the habit its own comment warned against. Replaced with sampled
attitudes at a 0.05 degree tolerance: robust to float noise, ~30x below the
filter's own tracking error, and still catches anything that could matter.

### Three deliberate divergences

1. **A degenerate gradient is skipped, not divided by.** When the measurement
   exactly matches the prediction, the objective and its gradient are zero. The
   reference divided by that length and survived only because its broken
   inverse-square-root returned a huge finite number for zero rather than
   infinity. Written the obvious way, `0 * inf` is **NaN**, and a NaN quaternion
   never recovers. Rare from a real sensor, routine from a synthetic one — which
   is what `host_sim` and every unit test feed.
2. **Euler angles are computed on demand.** The reference cached them behind a
   flag `setQuaternion()` did not clear. Unobservable in the firmware.
3. **`setOrientation()` normalises.** Everything downstream assumes unit length.

The `+180` yaw offset is **not** a divergence — it is reproduced deliberately.
Every log, telemetry backend and web UI carries it, and changing a convention
inside a like-for-like replacement would have destroyed the only proof of
correctness available. Fix it separately or not at all.

### Cost

Flash **shrank 2.7 KB** on lolin-lite (643,076 -> 641,340), because the Adafruit
header also pulls in Mahony and NXPFusion. Performance was never the motivation
and was not measured on target; `1.0f/sqrtf()` is used for clarity, and whether
a correctly-sized fast inverse square root beats it on the FPU-less C3 remains
the open measurement ADR-017 flagged. Do not assume either way.

### Tests

21 new: 17 unit tests for the filter itself — degenerate input, free fall,
convergence in roll and pitch with the **sign derived rather than fitted**,
vertical pitch not producing NaN, beta monotonicity, and the magnetometer path
that no flight log can reach — plus the four replay tests.

---

## ADR-057 — guarding against the 32-bit/64-bit split

**Status:** Accepted. Follows ADR-056, which is the bug this exists to prevent.

The Adafruit defect was not "someone picked the wrong integer type". It was
**two types assumed to be the same size, unchecked, on a platform where they are
not** — and nothing in the build ever compared the two ABIs. Fixed-width types
alone would not have caught it: `union { float f; int64_t i; }` is just as
broken and just as silent.

Four layers, in the order they earn their keep.

### 1. `std::bit_cast` for every scalar reinterpretation

```cpp
// wrong: silent on the platform where the sizes differ
union { float f; long i; } conv = {x};

// right: will not compile unless sizeof matches
const auto bits = std::bit_cast<std::int32_t>(x);
```

This is the whole fix for this bug class, it is free, and it turns a silent
runtime wrong answer into a build error **on the platform that is wrong**. C++20,
which is already required (ADR-021). Union punning is also formally UB in C++ —
only the last-written member may be read — so `bit_cast` is the correct spelling
regardless of size.

Enforced by check 9 in `check_layering.sh`, which rejects any union carrying a
float or double member and any `reinterpret_cast` to a scalar type. **The check
was verified by planting the original defect and confirming it fails** — an
unverified guard proves nothing, which is the recurring lesson of this project.

### 2. Fixed-width types where the width IS the contract

Use `std::uint32_t` and friends for: wire and register formats, bit
manipulation, anything punned, anything persisted, checksums.

Do **not** reach for them where a semantic type is right: `std::size_t` for
sizes and indices, `std::ptrdiff_t` for differences, `std::chrono` for
durations, `uintptr_t` for pointer-sized integers. The rule is *fixed-width
where the width is the semantics*, not fixed-width everywhere.

And specifically: **avoid `long`.** It is 32-bit on Windows LLP64 and on
ILP32 embedded targets, 64-bit on Linux and macOS LP64 — the only mainstream
type whose size varies three ways. `int` is 32-bit everywhere this project runs;
`long long` is at least 64.

`printf`-family format specifiers are the sibling trap: `%lu` against a
`std::uint32_t` is wrong on LP64. This codebase already casts at the call site
(`(unsigned long)h.totalRetries`), which is correct and should stay; `PRIu32`
from `<cinttypes>` is the alternative. `-Wformat` catches mismatches only for
the platform being compiled for, which is exactly why layer 4 matters.

### 3. Assert the assumption where it cannot be avoided

`static_assert(sizeof(A) == sizeof(B))` next to anything that depends on it.
Free, and it fires at compile time on the platform that breaks it.

### 4. Run the tests on BOTH ABIs — the structural fix

The deepest problem was never the type. It was that host tests ran LP64 while
the firmware runs ILP32, **and nothing ever compared them**. A host-only pass is
only as trustworthy as the code's independence from that difference, and nothing
was checking that independence.

`./tests/run_tests.sh --m32` builds and runs the suite 32-bit, into a separate
build directory. CI runs both as a matrix. Any behaviour that differs between
them is an ABI dependency; on a flight controller that means the tests describe
a program that is not the one flying.

Verified locally as far as the hardware allows: Apple Silicon has no 32-bit
support, so the `--m32` path itself is exercised in CI rather than here. What
*was* verified locally is stronger for the portable core — the estimation layer,
the drivers and `hal/core` compile clean under the real `riscv32-esp-elf-g++`
at ILP32 with `-Wall -Wextra -Wconversion`.

### Local first — CI is not where these get run

CI runs on push. That is not when a mistake is made, and on this project it is
not often looked at. Two gaps followed from that, both worse than the guards
being absent would have been, because they looked like coverage:

- **`check_layering.sh` ran only in CI.** Every architectural guard in it — the
  vendor-type check, the estimation-layer RTOS ban, the burn-down, and now the
  type-punning check — was effectively dormant locally.
- **`./build.sh` is the only local ILP32 compile, and the ESP32 core passes
  `-w`.** All 186 translation units build for the target with every warning
  suppressed. The one place the code met a 32-bit compiler was the one place it
  was told to say nothing.

Both are now wired into `./tests/run_tests.sh`, which is the command actually
typed. `--no-checks` skips them for a tight edit loop; CI has no such option.

`tools/ci/check_target_warnings.sh` compiles the portable subset with the real
`riscv32-esp-elf-g++` at `-Wall -Wextra -Wconversion -Wsign-conversion
-Wshadow`. It is a compile-time check, not a behavioural one — it will not catch
a runtime ABI dependency the way running the tests 32-bit does — but it is what
runs on any machine with the toolchain, including Apple Silicon where `--m32`
cannot.

Files are **discovered, not listed**: anything under `src/` that compiles with
no Arduino or ESP-IDF headers is portable by definition. The set therefore grows
by itself as the platform-call burn-down proceeds. A floor (currently 21) stops
it silently shrinking, because a file dropping out of the portable set would
otherwise just stop being checked.

All 21 are clean today, which is the point of fixing the floor now rather than
after the next regression.

### What would have caught this specific bug, ranked honestly

1. **MemorySanitizer** — the read of uninitialised bytes is exactly what it
   detects, and it would have pointed at the line. Worth adding to CI.
2. **Dual-ABI runs** (layer 4) — would have shown the two builds disagreeing.
3. **`std::bit_cast`** (layer 1) — prevents writing it, but only in our own
   code, and this was a third-party library.

That ordering is worth keeping in mind: for a defect in a dependency, the
sanitizer and the differential build are what save you. A house style rule does
not reach code you did not write.

Both of those are Linux-only in practice — MemorySanitizer does not run on
macOS, and Apple Silicon has no 32-bit support — so on this machine they stay in
CI whether one likes it or not. That is an argument for running CI before a
flight, not for pretending the local checks cover the same ground. What the
local checks do cover is stated above, and no more than that.

---

## ADR-058 — cooperative task stop, and the burn-down it unblocked

**Status:** Accepted (post-Phase 9 cleanup).

### `Task::requestStop()` was a no-op with a confident name

`Esp32Task::requestStop()` set `_stopRequested`. Nothing read it — there was no
accessor on the interface, the scheduler never consulted it, and no task body
could see it. `isRunning()` returned `_handle != nullptr`, and `_handle` was
assigned once at spawn and never cleared, so it answered "was spawned", not
"is running".

Nothing called either, so nothing was broken. It was found while migrating the
telemetry backends **onto** it — where it would have replaced a working
`vTaskDelete()` with a silent no-op, and the tasks would simply have kept
running after teardown.

Three changes:

- **`stopRequested()` on the interface.** The flag now has somewhere to be
  observed. Bodies poll it once per loop.
- **A trampoline around every task body.** A FreeRTOS task function that returns
  takes the system down; it must call `vTaskDelete(NULL)`. Routing bodies
  through a trampoline makes returning legal, which is what makes cooperative
  stopping usable, and is where `_running` is cleared so `isRunning()` becomes
  truthful.
- **`HostTask` in the host fakes**, so the contract is testable. Five tests,
  including one that proves the polling body's iteration limit is *not* what
  ends the loop in the test that checks stopping.

Stopping stays cooperative on purpose. FreeRTOS offers a preemptive kill and it
is the wrong primitive: it can strike inside a critical section, with a mutex
held or a half-written log record, and it cannot be expressed on a host. The
cost — a body that never polls never stops — is the contract, not an oversight.

### Burn-down: 90 → 50

| Area | Was | Now |
|---|---|---|
| Debug serial telemetry | 5 | 0 |
| Q serial telemetry | 5 | 0 |
| Flash telemetry | 8 | 0 |
| CRSF telemetry | 11 | 0 |
| Buttons (base, hold, multi-tap) | 3 | 0 |
| `ArduFliteController` | 8 | 0 |

Four defects found on the way, none of which the compiler or the tests would
have reported:

**1. A 5 ms wait silently became a try-lock.** `SemaphoreLock`'s DEFAULT timeout
was `MUTEX_TIMEOUT_MS` = 5 ms, and most call sites used the default — so they
*looked* like try-locks and were not. Translating them to `std::try_to_lock`
would have dropped telemetry samples whenever publish() and the writer task
overlapped, which at 50 Hz each is routine. Now `kTelemetryLockTimeout`, named
once in `ArduFliteTelemetry.h` rather than four times by accident.

**2. A use-after-spawn.** `_task` is assigned only after `spawn()` RETURNS, but
the body may already be running — a higher-priority task preempts immediately.
Every migrated loop guards with `_task == nullptr || !_task->stopRequested()`,
because a null handle means "no stop possible yet", not "crash".

**3. A comment describing another file's behaviour.** The Q-serial loop claimed
it reused the previous iteration's copy, "stale-but-safe", on a lock timeout. It
could not have: the copy is scoped to the loop body. That is the Debug backend's
behaviour, written in the wrong file. Skipping is right for this one anyway — the
output is a CSV row for a plotter, and a repeated row is a fabricated sample.

**4. A dead member with a live comment.** `File _logFile` had already been
replaced by `device::LogStore`, but the member, its `<FS.h>` include and four
comments referring to it all remained — including one justifying a load-bearing
single-core assumption about a file that no longer existed.

### The controller pause protocol, replaced rather than wrapped

`pauseTasks()` exists for a real reason — calibration needs the airframe still,
and live surfaces responding to stick input during it are a finger hazard — so
it was not deleted. But it was implemented by finding two **foreign task handles
by name**, unsubscribing each from the watchdog so a multi-second pause could not
trip it, suspending them, and rebalancing all of it on resume, including
re-priming `lastWake` because a stale one made `sleepUntil()` return instantly
and the task busy-spin to catch up.

None of that is needed if the loops keep running and skip their body. They stay
watchdog-registered and keep feeding, so there is nothing to unsubscribe;
`dt` never grows, so there is no timing to re-prime; the PID integrators are not
advanced, so the pause cannot wind them up; and the surfaces hold their last
commanded position exactly as before, because the skipped body is what would
have called `commit()`.

`hal::Scheduler` gained nothing. Adding `suspend()`/`resume()` would have made an
interface out of a mechanism that turned out not to be needed — the same trap
ADR-029 records.

### Completed: 90 -> 1

Every remaining area followed. Four things are worth recording beyond the count.

**`SemaphoreLock` and `MUTEX_TIMEOUT_MS` are deleted.** Removing the macro fixed
something rather than merely tidying: it was `SemaphoreLock`'s DEFAULT timeout,
so a call site naming no timeout waited 5 ms while reading exactly like a
try-lock. Where the wait is load-bearing it is now named at or near the call
site (`kTelemetryLockTimeout`, `kMissionLockTimeout`), so nobody needs to know a
default to understand the code.

**`ConfigTask` lost a binary semaphore.** It had one purely to signal "task has
exited" so `stop()` could join. `Task::isRunning()` answers that directly now
that the trampoline clears it — one fewer primitive to create, take, give and
delete across every error path.

**`hal::System` gained three methods, each with a caller that already existed:**
`taskReport()` (the `tasks` CLI command, replacing a raw `vTaskList()` plus a
buffer-capacity check that now lives beside the call it protects),
`platformName()`/`sdkVersion()` (the web status endpoint), and `randomWord()`.
The last is a **security** primitive, not a convenience: the web server builds
its CSRF token from two of them, and an implementation backed by `rand()` or a
boot-seeded PRNG would make tokens predictable across reboots — a device that
always boots to the same token has no CSRF protection at all. The interface says
so, so a future port cannot satisfy it cheaply by accident.

**A contract test caught a real regression risk.** `WebSecretsFlashAndCaptiveDns`
asserts `handleFlashGet()` yields between chunks while streaming a log — without
it, a multi-megabyte download starves every other task. The assertion named the
old spelling and failed. Updated to assert the yield rather than the API, which
is what it always meant.

### The one that stays, and why

`esp_wifi_set_ps(WIFI_PS_NONE)` in `WiFiManager.cpp`. The whole file is
Arduino-WiFi — `WiFi.softAP()`, `softAPConfig()`, `DNSServer` — and none of those
match the burn-down's pattern. Routing this single call through a HAL would move
the counter without moving the coupling: the module would be exactly as portable
afterwards as before. A WiFi HAL is the real fix and is not worth inventing for
a captive portal that only runs on the ground (ADR-029).

The burn-down therefore reads **1**, and that 1 is a decision rather than a
backlog item.

---

## ADR-059 — the firmware build hides every warning, and what that was hiding

**Status:** Accepted (post-Phase 9 review).

`./build.sh` compiles all 75 sketch translation units with **`-w`**. It comes
from the ESP32 core's `cpp_flags`, not from this project, and it means the one
place the code meets the target compiler is the one place it is told to say
nothing. `check_target_warnings.sh` (ADR-057) covers the ~21 files that build
without Arduino headers; the other ~54 had no warning coverage at all.

`tools/ci/check_full_warnings.sh` closes that. It builds once, captures the
command lines arduino-cli actually used, and **replays** each with `-w` stripped
and `-Wall -Wextra` added. Replaying rather than reconstructing matters: the
include paths, defines and response files are whatever the real build used, so
there is no second description of the build to drift out of step with the first.

Two traps in writing it, both of which made the check silently pass:

- **arduino-cli preprocesses every file separately (`-E`).** Scanning those
  reports a clean tree no matter what the code says. Caught by planting an
  unused variable and finding it undetected.
- **A capture that matches nothing also reports zero warnings.** The script now
  fails if it scanned fewer than 40 units, because a clean result from an empty
  scan is worse than no check.

### What it found

| Defect | Consequence |
| --- | --- |
| `ConfigPersistence::eraseAll()` dropped its `Status` | Logged **"Config erased from NVS" whether or not the erase succeeded** |
| Two schema-version writes dropped their `Status` | A silent failure re-runs already-applied migrations on the next boot |
| `ButtonBase::begin()` dropped `setMode()`'s `Status` | A dead input reads as "nobody is pressing it" |
| Both controllers' initialiser lists were out of declaration order | Benign today; a real bug the day one member's init depends on another |
| `sendFlightMode()` bounded its array index on one side only | `flight_mode` is a plain `int`; a negative value passes `< LENGTH` and then indexes at `uint8_t(-1)` — far outside a 4-entry table |

The first four are all `[[nodiscard]] Status` being ignored — **exactly what
ADR-009 designed that type to prevent.** The attribute was doing its job; `-w`
was throwing the result away. An enforcement mechanism nobody can see is not an
enforcement mechanism, which is the same lesson as running the layering checks
only in CI (ADR-057).

The false "Config erased" is the one worth remembering. Someone erasing config
is usually trying to recover from a bad state, and a confident false
confirmation sends them looking for the fault somewhere else entirely.

### Why it is not in `run_tests.sh`

It needs a full firmware build first, so it costs minutes rather than seconds.
Run it before a release or after a broad refactor. The fast checks stay in
`run_tests.sh` where they are cheap enough to never be skipped.

### Sanitizers, and verifying the verifier

`./tests/run_tests.sh --san` builds and runs the suite under AddressSanitizer
and UndefinedBehaviorSanitizer, into its own build directory. The whole suite is
clean under both.

`UBSAN_OPTIONS=halt_on_error=1` is set deliberately: UBSan's default is to print
and continue, so a violation would scroll past inside a run that still reports
PASSED.

A clean sanitizer run is worth nothing unless the sanitizers are actually
linked, so this was verified by planting two defects — a signed-overflow and a
heap-buffer-overflow — and confirming each was reported before removing them.
That is the same discipline as the type-punning check (ADR-057) and for the same
reason: every guard added in this review was checked against the bug it exists
to catch, because a guard that cannot fail is indistinguishable from no guard.

---

## ADR-060 — two inherited scaling bugs, fixed while the flash is being wiped

**Status:** Accepted. Both were previously documented as conventions to preserve
(ADR-054 for the yaw offset, the CrsfLink header for the stick scaling). Both
turned out to be defects, and both are fixed.

### CRSF stick scaling: 20 % of stick travel was unreachable

`rawToMicroseconds()` anchored on the raw 11-bit FIELD (0 / 1024 / 2047) rather
than on the CRSF ENDPOINTS. A transmitter sends 172 at -100 %, 992 at centre and
1811 at +100 %; it only reaches 0 or 2047 with over-100 % travel configured.

| Raw | What it means | Old µs | Correct µs |
| --- | --- | --- | --- |
| 172 | stick at -100 % | 1084 | 1000 |
| 992 | stick centred | 1485 | 1500 |
| 1811 | stick at +100 % | 1885 | 2000 |

So full stick delivered **801 µs of the 1000 µs window — 80.1 %** — and centre
sat 15 µs low. The aircraft was flying with a fifth of its commanded authority
scaled away, compensated for by servo travel settings.

Now anchored on 172 / 992 / 1811, with raw values outside that range clamped so
a radio set beyond 100 % travel reaches the endpoint and stops.

**This changes flight behaviour.** Full stick now produces roughly 25 % more
surface deflection for the same `servo.*.defl` setting. Re-check travel for
binding on the bench BEFORE flying — that is a mechanical limit, not a tuning
preference.

### Yaw reported the reciprocal heading, and overflowed the CRSF field

`euler_deg().yaw` added 180 degrees. That was described as mapping yaw into
0..360, but adding 180 to a [-180, 180] range **shifts** rather than wraps: a
correct mapping is `if (deg < 0) deg += 360`. Measured across the range, the
reported value was true yaw + 180 exactly — nose-forward read 180 degrees.

Two consequences, the second of which is live:

1. Any comparison against a real heading is 180 degrees out, which would have
   bitten the moment `MagneticHeading` was used for anything.
2. **The CRSF attitude frame overflowed.** It encodes attitude as int16 radians
   x 10000, saturating at ±187.7 degrees. With yaw offset into 0..360, every
   true yaw above **7.7 degrees** wrapped the int16 — so for most of a turn the
   radio displayed a heading sweeping backwards through negative values:

   | True yaw | Reported | Radio showed |
   | --- | --- | --- |
   | 0° | 180° | 180° |
   | +45° | 225° | **-150°** |
   | +90° | 270° | **-105°** |

Yaw is now a true signed angle in -180..180, matching roll and pitch, matching
the attitude setpoint domain, and matching what CRSF expects. Nothing in the
control path reads it, so there is no flight-behaviour risk from this half.

### Why now

Both were previously justified by "the aircraft is trimmed against this, so
changing it silently re-trims it in flight". That argument was sound and is now
void: the flight controller is being erased and re-trimmed regardless. Fixing
them together, before the re-trim, means one trimming session rather than two.

The tests that pinned the old behaviour were the ones that had to change:
`test_rc_mapper.cpp`'s reference oracle now expresses the CRSF endpoints rather
than an internal formula, which is a better anchor — it is checkable against the
protocol rather than against a previous version of this codebase.

---

## ADR-061 — the two safety gates must not be able to fail to close

**Status:** Accepted (post-Phase 9 review).

`armed` and `throttleCut` were plain bools guarded by `ctrlMutex`, taken with the
same 5 ms bounded wait as everything else on that mutex. Two consequences, both
in the direction that matters:

- **`cutThrottle()` returns void and silently did nothing on a timeout.** That
  is the pilot's throttle-cut switch. Flip it, the wait expires, the motor keeps
  running, and nothing anywhere says so.
- **The inner loop kept SHADOW copies**, refreshed only on ticks that got the
  lock. A disarm therefore did not take effect until contention cleared — the
  loop kept staging surfaces and throttle from the last value it managed to
  read.

Both are now `std::atomic<bool>`, and the inner loop reads them directly rather
than through the shadow. Neither operation can fail, and neither can be delayed.

Nothing is lost by taking them out of the mutex: they are independent booleans,
not part of the setpoint group whose coherence the lock exists to protect. The
setpoints still travel together under `ctrlMutex`, still with shadow copies,
because a torn mixture of old and new setpoints IS a real hazard — whereas a
one-tick-old `armed` is simply wrong.

`isArmed()` and `isThrottleCut()` previously returned **true** when the lock
timed out. That was the safe default for their callers (the web server refuses
flash operations when armed), but it meant a diagnostic could report the
aircraft armed when it was not. They now return the real value.

Pinned by `ProductionContracts.ArmAndThrottleCutStayAtomic`, because
`ArduFliteController` cannot be instantiated on a host and "move these back
under the mutex for consistency" is a plausible refactor that looks right.

### A related false confirmation

`CRSFCallbacks::onFailsafe()` pushes three commands — cut throttle, force
ATTITUDE, command the spiral — and then logged `Throttle=CUT` unconditionally,
ignoring all three return values. The queue is 100 deep and failsafe pushes
three, so a drop is not a realistic risk; the log line was the defect. It now
reports `Failsafe INCOMPLETE` if any push was rejected.

The three commands are ONE action. A partial failsafe leaves the aircraft in a
state nobody described, and a log that claims it completed is the worst possible
thing to be reading after a crash.

---

## ADR-062 — what the file-by-file review actually found

**Status:** Record of the post-Phase-9 review. Details are in ADR-056 to ADR-061.

Fourteen defects across ~100 changed files. **None was found by reading code.**
Every one came from a tool that was either absent or switched off:

| Found by | Defects |
| --- | --- |
| Turning warnings on (`-w` was hiding them) | 4 dropped `[[nodiscard]] Status`, 2 `-Wreorder`, 1 unbounded array index |
| Running the code and measuring it | CRSF scaling costing 20 % of stick travel; yaw reporting the reciprocal heading and overflowing the CRSF int16 |
| Migrating onto an interface | `Task::requestStop()` was a no-op with a confident name |
| Reasoning about failure ordering | `cutThrottle()` silently failing; disarm delayed by lock contention |
| Reading tests as code | 4 tests for deleted behaviour, 1 tautological assertion, 1 dead mirrored formula |

### The pattern worth keeping

Almost every one was **silent in the direction of looking fine**:

- `eraseAll()` logged "Config erased from NVS" whether or not it succeeded.
- `onFailsafe()` logged "Throttle=CUT" without checking that any command queued.
- `requestStop()` set a flag nothing read; `isRunning()` meant "was spawned".
- `cutThrottle()` returned void and did nothing when its lock timed out.
- A wraparound test asserted a value equalled itself.

A system that reports success it did not achieve is worse than one that fails
loudly, because the false report sends the next person looking somewhere else.
That is the single most common shape in this list, and it is worth checking for
directly: **find every place that logs success, and ask what it checked.**

### Guards added, and why each was itself tested

| Guard | Verified by |
| --- | --- |
| `check_layering.sh` scalar type-punning check | planting the original Adafruit union |
| `check_target_warnings.sh` | a floor on files scanned, so an empty scan fails |
| `check_full_warnings.sh` | planting an unused variable — which caught that the first version scanned only `-E` preprocessing passes and could never warn |
| `--san` (ASan + UBSan) | planting a signed overflow and a heap overflow |
| `ControlLoopDtClampIsUnchanged` | changing the clamp in ONE loop, the case it exists for |
| `ArmAndThrottleCutStayAtomic` | n/a — source contract, checked by reading |

Two of those failed their first verification. The full-warnings scan reported a
clean tree while scanning nothing that could warn, and it would have been
committed as a passing check. **An unverified guard is indistinguishable from no
guard**, and on this evidence it is worse, because it is believed.

### Where the checks now live

Fast enough for every run, wired into `./tests/run_tests.sh`: unit tests,
`check_layering.sh`, `check_target_warnings.sh`. Opt-in: `--san`, `--m32`.
Minutes, so run before a release: `check_full_warnings.sh`.

CI runs all of them. That matters less than it looks: the checks that found
things were the ones that run locally, because that is when the mistake is made.

---

## ADR-063 — closing the three definition-of-done gaps

**Status:** Accepted. Items 4, 5 and 7 of §06's definition of done were not met.

### Item 5 — CI built one board out of two

The plan calls for a 2x2 build matrix. `arduino_build.yaml` built `lolin` and
`lolin lite`; **the FireBeetle was never built in CI at all**, and
`check_size.sh` budgeted only the two lolin variants. A change that broke the
FireBeetle was invisible until someone built it by hand — and the FireBeetle is
precisely the board nobody builds by habit, being the one marked `Untested`.

Now a real matrix: `board: [lolin, fire]` x `variant: ["", lite]`, with
`fail-fast: false` so one board's failure does not mask the other's.
`check_size.sh` checks all four against the same hard partition limit (both
boards use the same 0x200000 app partition) and the same two budgets.

Measured: lolin-lite 642 KB, lolin-full 1405 KB, fire-lite 565 KB,
fire-full 1284 KB. All inside budget.

### Item 7 — the README documented three deleted classes as features

`ServoManager`, `CRSFReceiver` (with per-channel callbacks "via
`CRSFConfiguration.h`") and `ArduFliteIMU` were all presented as current
features. Verified against the code, the README also had:

| Claim | Reality |
| --- | --- |
| V-Tail geometry **supported** | A stub producing NO deflection. A V-tail airframe would have had no control |
| Throttle cut via "single-lock snapshot" | Atomic since ADR-061 |
| Q-Serial at 10 Hz | Constructed at 20 Hz, and commented out |
| CRSF slow frames at 1 Hz | 200 ms, i.e. 5 Hz |
| ~90+ config parameters | 95 |
| 7 REST endpoints | 13, and five real ones were undocumented — including `/api/session`, `/api/config/reboot` and `/api/system/calibrate` |
| Web UI tabs "…, All" | No "All" tab; six others undocumented |
| Lite build ~800 KB smaller | 780 KB |

The V-Tail one is the defect that mattered: a reader with a V-tail airframe
would have built and flown it.

Rewritten against the code, with two change-narration blocks removed — the
README is not a changelog, and a reader arriving today has no previous version
to contrast against.

### Item 4 — "no mirrored formulas remain" was too absolute

Three formulas are legitimately reproduced in tests as ORACLES, because the
production code cannot be called from a host: the control loops' dt clamp (three
lines inside a task body), the RC stick shaping, and the axis transform.

The wording now permits exactly that, and only with a tie: each oracle must be
bound to its subject by a test that fails when the two drift apart. The dt clamp
has a source contract; the RC oracle asserts against the production constants
and calls the real conversion; the axis oracle is compared directly against
`AxisTransform`. A copy with no such tie remains forbidden.

That is the honest distinction. A reproduced formula is not automatically rot —
an *unanchored* one is.

---

## ADR-064 — `PeriodicTelemetryBackend`, and what the migration itself taught

**Status:** Accepted (Phase 10). Implements the plan in §06 Phase 10.

### Result

| Backend | Before | After |
| --- | --- | --- |
| Debug serial | 152 | 80 |
| Q serial | 133 | 59 |
| Flash | 617 | 568 |
| CRSF | 467 | 408 |
| Base | — | 113 + 118 header + 32 board |

Four copies of `publish()`, the idempotency guard, the mutex allocation, the
spawn-and-roll-back, the destructor and the loop guard became one. Every backend
is now its loop body and nothing else.

### The split that the compiler forced, and why it was the right answer anyway

`begin()` needs `Board::instance()`, and `Board.cpp` pulls in ESP32 — so keeping
it in the same translation unit would have made the whole class impossible to
build on a host, defeating the reason for extracting it.

`begin()` therefore lives alone in `PeriodicTelemetryBackendBoard.cpp`, and the
rest is Board-free. The seam is `beginWith(mutex, scheduler)`: production passes
the board's, tests pass a `HostMutex` and a `RecordingScheduler`.

This is the first telemetry lifecycle code in the project that any test can
reach. It is worth being precise about what that buys: the LIFECYCLE is now
tested, not the backends. Flash's log rotation and CRSF's framing are as
untestable as they were.

### Snapshot-on-timeout: one default, not one rule

`snapshot()` leaves its destination untouched and returns false when the lock
times out, so a caller that ignores the result keeps its previous copy. Debug,
Flash and CRSF do exactly that — an unbroken cadence is what their consumers
want, and a repeat is visible in the timestamp.

Q serial checks the result and skips. Its output is a CSV row for a plotter,
where a duplicate is a fabricated sample and a gap is not. Returning `bool`
rather than picking one behaviour is what lets both be right; it costs one line
at the call site and no complexity in the base.

### What stayed out

CRSF's watchdog subscribe/unsubscribe, its pause flag, its rate tiering and its
`sleepFor`; Flash's file mutex and its log operations. CRSF inherits the
lifecycle and keeps its own loop entirely. Building hooks into the base for one
subclass is the trap ADR-029 describes, and the loop bodies are where these
backends genuinely differ.

`requestTaskStop()` IS in the base, because destructors run derived-first: a
backend that closes a file has to ask the loop to stop before it starts, not
rely on `~PeriodicTelemetryBackend()` doing it afterwards.

### Two defects found in the tests, not the code

The base was verified by planting each invariant's failure and confirming the
right test caught it. Two of those attempts failed, and both were the test's
fault:

- **The null-task guard had no coverage.** Every test assigned `_task` before
  running the body, so deleting the guard passed. Reproducing spawn-preemption
  on a host means calling `runLoop()` *before* `beginWith()`. With that test
  added, removing the guard segfaults.
- **Both timeout tests were undefined behaviour.** They locked a `HostMutex` — a
  non-recursive `std::timed_mutex` — and then called into the backend on the
  same thread. That is UB; it returned false on this platform, so the tests
  looked like they worked while resting on nothing. Rewritten to hold the lock
  from a second thread.

The second is the one worth remembering: **a test that passes for the wrong
reason is indistinguishable from a test that passes.** Only planting the defect
separates them.

### On the migration order

One backend per step, full verification between each, simplest first. That
ordering earned itself twice:

- The Board/host split surfaced on the FIRST backend, when the change was two
  files rather than five.
- Flash's migration broke `ProductionContracts.FlashTelemetryResetAndDeleteAreSafe`,
  which pins the guard `!rowTruncated && _fileMutex` by source text. It failed
  because `self->` disappeared, not because the guard did — but that is the
  contract working: it noticed the code it guards had changed, and a human had
  to confirm the guard survived.

Doing all four at once would have produced the same defects with four times the
surface to search.

### Behaviour changes, all deliberate

- **Q serial** keeps its skip (unchanged).
- **CRSF's rate is now clamped** to [0.1, 200] Hz by the base; it previously
  divided by `freqHz` unguarded, so 0 Hz gave an infinite interval and an
  undefined cast. It is constructed at 10 Hz, so nothing observable changes.
- **CRSF's `pauseTask()`/`resumeTask()` no longer early-return when the task was
  never started.** They set a flag no loop is reading; harmless.
