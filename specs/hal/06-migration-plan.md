# 06 — Migration Plan

**Status:** Draft, revision 2 · **Date:** 2026-08-02

Ten phases. **Every phase ends with a flyable firmware.** No phase leaves the tree
mid-refactor. Each is one branch and one PR.

> **Revision 2 note.** Revision 1 had seven phases and three structural problems:
> actuator work was done twice (review finding R10); the two highest-risk changes —
> new sensor drivers and a new estimation layer — were bundled into one branch; and
> Phase 0 claimed "no firmware change" while containing a config rename that does
> change firmware. All three are fixed here. Phases are smaller, single-purpose, and
> ordered so **the architecture is proved on two low-risk vertical slices before it
> is used on the IMU.**

---

## Ground rules

* **Baseline is A/B, not pre-recorded.** Build sizes are already captured in
  [`baseline.md`](baseline.md) (`lolin-full` 1,418,832 B; `lolin-lite` 630,464 B) and
  need no hardware. Loop timing and stack high-water marks are compared **within a
  single Phase 2 bench session**: flash the tagged pre-HAL firmware, capture `stats`
  over 60 s and `tasks`, flash the Phase 2 firmware, capture again, compare.
  Same board, same session, same conditions — a better-controlled comparison than one
  against a recording made weeks earlier, and it needs no flying: the board on USB
  runs the IMU task and both control loops on the bench.
* **Behaviour-preserving unless stated.** Where a phase changes behaviour it is
  called out under **Behaviour change** and gets a bench gate.
* **Phase gate** = host tests green + the phase's bench checklist + one short flight.
  Then tag `hal-phase-N`.
* **Rollback** = revert the phase's merge commit. Nothing spans phases.
* **One dominant risk per phase.** If a phase has two things that could ground the
  aircraft, it is two phases.

## Sequencing rationale

Three principles drive the order:

1. **Prove the architecture on cheap subsystems first.** Phases 3 and 4 are complete
   vertical slices — flight code → device interface → driver → platform → hardware —
   through the actuator and RC paths. If the tier model is wrong, that becomes
   obvious there, on subsystems that are bench-testable in minutes, rather than
   inside the IMU rewrite.
2. **Never do the same work twice.** All actuator work is in Phase 3; all sensor
   driver work is in Phase 5. Revision 1 split actuators across Phases 1 and 3.
3. **Separate the two scariest changes.** New register drivers (Phase 5) and the new
   estimation layer (Phase 6) are independently revertible, and each has its own
   oracle — a driver cross-check for Phase 5, log replay for Phase 6. Bundled, a
   failure in either is hard to attribute.

```
P0 Foundations ──┬── P1 Config hygiene        (independent, either order)
                 │
                 └── P2 Platform layer ──┬── P3 Actuators ──┐
                                         ├── P4 RC input ───┤
                                         └── P5 Sensor drv ─┴─ P6 Estimation ─┐
                                                                              │
                                              P7 Peripherals ─── P8 Cleanup ──┴─ P9 Madgwick
```

P3, P4 and P5 are mutually independent once P2 lands. P7 is independent of P3–P6.

### Hardware availability

Confirmed available: a bench setup, and a spare board **without an IMU fitted**.
That shapes which phase can be developed where:

| Phase | Runs on the spare (no IMU)? |
|---|---|
| P0 Foundations | Host only — no board needed at all |
| P1 Config hygiene | Yes |
| P2 Platform layer | **Yes, and it is the better target** — see below |
| P3 Actuators | Yes — servos and ESC need no IMU |
| P4 RC input | Yes — receiver needs no IMU |
| P5 Sensor drivers | **No** — needs the aircraft board, and the cross-check harness runs both drivers against the same physical sensor |
| P6 Estimation | **No** — same |
| P7 Peripherals | Yes |

**The IMU-less spare is an asset, not a limitation, for Phase 2.** `Board::begin()`
is specified to log a probe failure, unbind the device and continue rather than hang
(§04 §6, replacing today's `while (1);` at `ArdufliteApp.cpp:233`). A board with no
IMU exercises that path on real hardware, for free, without desoldering anything.
**Make it an explicit Phase 2 bench check:** flash the spare, confirm the boot
inventory reports the IMU as absent, confirm the firmware reaches the CLI and does
not hang, and confirm arming is refused.

Practical consequence: **P3 and P4 can be developed and bench-tested on the spare
while the aircraft stays flyable** on the last tagged firmware. Only P5 and P6
require taking the aircraft out of service.

---

## Phase 0 — Foundations *(host only — firmware untouched)*

**Goal:** every interface in §03 exists, compiles, and is covered by host tests.
Nothing is linked into the firmware.

* `src/hal/core/`: `Status`, `Result`, `Vec3`, `AxisTransform`, `SeqLock`, `Units`,
  `NonCopyable`. **Header-only** — anything with a `.cpp` under `src/` is compiled
  into the firmware (review R20).
  *The `FliteQuaternion` rename moved to Phase 6 — it touches flight code and Phase 0
  does not need it (review R21).*
* `src/hal/platform/` and `src/hal/device/`: all interface headers, declarations only.
* `tests/unit/hal_host/`: the host platform (§05 §3). **Not** `src/hal/host/` —
  `arduino-cli` compiles `src/` recursively, so that would link `std::thread` into
  the flight firmware (review R20).
* `src/hal/board/`: `BoardDescriptor`, `BoardValidate`, descriptors for
  `lolin_c3_mini` (`Supported`) and `firebeetle_esp32e` (`Untested`) — not yet used.
* **Host test suite moves to C++20** to match the firmware (§09). It is currently
  C++17, which would let host code compile that the firmware rejects — and would
  hide `std::span`, concepts and `constinit`, all of which the design now uses.
* CI: host test job, static-check job, 2×2 build matrix with size budgets (§05 §9).
* Tests: `test_axis_transform`, `test_seqlock`, `test_result`, `test_board_descriptor`.

**Expected friction — this is the point of the phase.** Nothing in §03 has been
compiled; it is plausible C++, not verified C++. Expect corrections, and **update
§03 as you go** rather than letting spec and code diverge on day one. Transcribing
the Wemos descriptor will also trip `static_assert`s — §00 2.4's defects surfacing,
exactly as intended.

**Done when:** host suite green in CI; firmware **size-identical** to baseline
(630,464 B lite). Not byte-identical — the Arduino build embeds build metadata, so two
builds of unmodified source already differ (review R19). Size still proves nothing new
was linked in.
**Risk:** none. **Rollback:** trivial.

---

## Phase 1 — Config key hygiene *(small, standalone)*

**Goal:** establish the unit-suffix convention in config before the HAL starts adding
keys. Separated from Phase 0 because it *does* reach the running firmware.

* Unit suffixes on every unit-bearing key (ADR-008): `att.deadband` →
  `att.deadband_rad`, `servo.*.min` → `servo.*.minpulse_us`, `mix.max_rate_roll` →
  `mix.max_rate_roll_dps`, and the rest of the ADR-008 table.
* `CONFIG_SCHEMA_VERSION` → 2.
* Touches `ConfigKeys.h`, `ConfigSchema.h` and `docs/CONFIG_REFERENCE.md` only — the
  web UI keys off prefixes (`app.js:19`), which do not change, and observers use
  `CONFIG_KEY_*` macros.

**Behaviour change:** none, provided NVS is wiped after flashing. The code defaults
are the current flying values, so nothing is lost.

**Done when:** `config list` shows the new names; a bench check confirms PID gains,
servo endpoints and deflections match pre-phase values.
**Risk:** low but not zero — a silently unmigrated servo endpoint is a bent linkage.
Verify servo travel on the bench before flying.
**Rollback:** revert + wipe NVS.

---

## Phase 2 — Platform layer *(Tier 0 on hardware)*

**Goal:** Tier 0 exists and is proven on the aircraft, with every existing class
still doing its own job.

* `src/hal/esp32/`: `Esp32Clock`, `Esp32Scheduler`, `Esp32Mutex`, `Esp32Watchdog`,
  `Esp32I2cBus` (+`RegisterDevice`), `Esp32Uart`, `Esp32Gpio`, `Esp32PwmOut`,
  `Esp32Nvs`, `Esp32LittleFs`, `Esp32System`.
* `Board` split as `Board.h` / `Board_Internal.h` / `Board.cpp` so flight code cannot
  name a driver type (review R3). **Do this now** — retrofitting a pimpl after twenty
  call sites exist is far worse.
* `Board` is `constinit` (§09), removing the static-initialisation-order hazard that
  the existing `initFromConfig()` pattern exists to work around.
* Decide `I2cBus::openDevice()` storage and exhaustion (review R15): fixed handle
  array, `kMaxDevices = 8`, `Status::NoSpace` when full.
* Existing classes take `hal::Clock&` / `hal::Watchdog&` / `hal::Scheduler&`,
  replacing direct `micros()`, `esp_task_wdt_*` and `xTaskCreate`.
* `SemaphoreLock` deleted in favour of `std::unique_lock<hal::Mutex>` (§09).
* `WatchdogGuard` RAII replaces manual `esp_task_wdt_add`/`_delete` pairs.
* Task priorities move to the `Priority` enum; stack sizes audited once against the
  measured high-water marks.

**Explicitly NOT in this phase:** `Esp32PwmOut` is written and unit-tested here but
**not wired to `ServoManager`**. Revision 1 converted servo output to microseconds
here and again in Phase 3; all actuator work now happens once, in Phase 3.

**Behaviour change:** none intended. Time becomes 64-bit `std::chrono`, fixing a
latent 71-minute `micros()` wrap.

**Stack sizes: carry over verbatim.** §06 previously said "audited once against the
measured high-water marks". Without a pre-recorded baseline that audit moves into the
A/B session — but **Phase 2 must ship the existing stack sizes unchanged**. Changing
task stack sizes and the task-creation mechanism in the same phase would make an
overflow impossible to attribute. Re-size in a later, separate change once `tasks`
has been read on both firmwares.

**Done when:** the A/B comparison shows `stats` and `tasks` within noise. **This phase
validates the ADR-002 virtual-dispatch cost estimate** — if `maxDt` or
`overrunCount` regress, stop and reconsider before going further.
**Risk:** medium — touches every task's creation and timing.
**Rollback:** revert; nothing above Tier 0 has moved.

---

## Phase 3 — Actuator output *(vertical slice #1)*

**Goal:** the first complete path from flight code to hardware through all four
tiers. If the architecture is wrong, this is where it shows — cheaply.

* `drivers::PwmActuatorBank` implementing `device::ActuatorBank` on `hal::PwmOut`:
  endpoint mapping, inversion, trim, travel limits, slew, `disable()`.
* `actuators::AirframeMixer` — CONVENTIONAL / DELTA_WING / V_TAIL, extracted verbatim
  as a pure function.
* `drivers::CompositeActuatorBank` — written now rather than later, because it is
  what *proves* ADR-020's mixed-transport claim instead of merely asserting it.
* ESP32Servo dropped; `ArduFliteController` writes mixer output to the bank;
  `ServoManager` deleted.
* `test_servo_math.cpp` rewritten to call the real mixer instead of a copy of it.

**Behaviour change:** output moves from `Servo::write(int degrees)` to microseconds,
removing ~11 µs of quantisation. Pulse values change slightly.

**Done when:** `RecordingPwmOut` tests reproduce current mixing for all three
geometries; **bench-measured surface travel and centre match pre-phase values on
every axis**; no servo reaches a mechanical stop; disarm and failsafe both drive the
configured action.
**Risk:** medium-high — this layer moves the control surfaces.

---

## Phase 4 — RC input *(vertical slice #2)*

* `drivers::CrsfLink` implementing `device::RcLink` — parser extracted from
  `ArdufliteCRSFReceiver`, now testable against captured bytes.
* `drivers::SimRcLink` — the second implementation that proves the interface.
* **Delete `src/receiver/pwm/` and `src/tests/ReceiverTests.*`** (ADR-014): confirmed
  unreachable, built on a pin table with an out-of-range GPIO and three collisions.
* `input::RcMapper` — channel→role mapping, tri-state thresholds, callbacks. This is
  where `CSRFConfiguration.h`'s channel table moves.
* `PreflightCheck`, `TelemetryData`, `CommandSystem` and `ArduFliteController::arm()`
  switch from `ArdufliteCRSFReceiver*` to `device::RcLink&`.

**Shared UART note:** the CRSF telemetry backend shares UART1 with the receiver.
Today that is managed by a comment. `Board` now owns the `Uart` and hands the same
reference to both, documented in `Board.cpp`.

**Done when:** captured-frame tests pass including split frames and CRC errors;
**failsafe entry and exit verified by powering the transmitter off and on**;
arm / mode / throttle-cut switches verified.
**Risk:** medium. Failsafe is safety-critical — bench-test it deliberately.

---

## Phase 5 — Sensor drivers

**Goal:** replace FastIMU and Adafruit_BMP280 with own register drivers, leaving the
existing fusion and filtering code otherwise untouched.

* `drivers::Mpu6500` / `Mpu9250` implementing `Sensor` + `Accelerometer` +
  `Gyroscope` (+ `Magnetometer` on the 9250); `drivers::Bmp280` implementing
  `Sensor` + `Barometer` — one interface per measurement (ADR-019).
* `sample()` performs the single 14-byte burst read; each `read()` returns cached data
  and never touches the bus (review R1/R2).
* `ArduFliteIMU` keeps its filtering, fusion, motion detection and seqlock for now —
  it just gets raw samples from the new drivers instead of FastIMU.
* Tests: `FakeRegisterDevice` register sequences and error injection; the BMP280
  datasheet's worked compensation example as a numeric fixture;
  `test_sensor_threading`; `test_sample_decimation`.

**De-risking gate — the cross-check harness.** Before cutover, build a bench firmware
running the new driver *and* FastIMU against the same physical sensor, logging both
streams. Compare offset, scale and noise while static and through the six
orientations. **FastIMU is deleted only once this passes.** This turns "did I get the
scaling right?" from a hope into a measurement.

**Behaviour change:** none intended — the same numbers from a different code path,
which is exactly what the cross-check proves.
**Risk:** high, well-mitigated.

---

## Phase 6 — Estimation layer

**Goal:** decompose what remains of `ArduFliteIMU`.

* `estimation::InertialSubsystem` (§03 3.8) — the sampling task, rate-aware
  decimation, per-sensor `AxisTransform`, calibration offsets, low-pass bank, health
  validation, climb rate, `SeqLock<ImuState>` publish, with the 12-step ordering
  contract.
* `AttitudeEstimator` interface + `AdafruitMadgwickEstimator`, which **wraps
  `Adafruit_Madgwick` unchanged** (ADR-017), reproducing its quirks exactly —
  including `getYaw()`'s `+180.0f`. If the wrapper silently cleans them up, Phase 9
  has no valid baseline.
* `MotionDetector`; `SensorSelector` implementing `SelectionPolicy::FirstHealthy`
  (§03 3.8). The interface is fixed now so a future voting or crossfading policy is a
  drop-in (ADR-026); `SelectionState` is published in `ImuState` so a switch reaches
  the flash log.
* `CalibrationService` — a state machine driven *inside* the sampling task, replacing
  the `pauseTask()`/`_taskPaused` spin-wait protocol entirely (review R6).
* `ArduFliteIMU` becomes a thin façade so consumers compile unchanged, then is
  deleted and callers take `InertialSubsystem&`.
* `FlightState` leaves `ImuState` — `StateManagement` owns it and telemetry reads it
  from there (§00 2.8).

**Behaviour changes — deliberate:**
* Calibration offsets move from raw EEPROM to `SettingsStore` (NVS + CRC), with a
  one-time migration reading the old EEPROM blob. **Verify on the aircraft before
  flying** — though `calibrate imu` is a 10-second recovery.
* Sensor alignment becomes an `AxisTransform`, **ported verbatim** as
  `AxisMap{+X, −Y, +Z}` (§00 2.3, ADR-007). This is *not* a behaviour change and must
  not become one. Only the magnetometer sign changes, and it is dead code on the
  MPU-6500 build.

**L3 log replay comes online here** and is the phase's primary gate.

**Done when:** replay of `FL002/log_007.csv` matches the logged quaternion within
tolerance; the six-orientation bench check reproduces pre-phase accel/gyro signs on
every axis; calibration migration verified on the actual aircraft; `stats` at
baseline.
**Risk:** high — the estimator is the aircraft's sense of which way is up.
**Mitigation:** replay oracle plus a dedicated bench session before the test flight.

---

## Phase 7 — Peripherals and I/O

* `drivers::LittleFsLogStore`, `NvsSettingsStore`, `NeoPixelIndicator`, console over
  `hal::Uart`.
* `ArduFliteFlashTelemetry` takes `device::LogStore&`; serial backends and `Logging`
  take `device::Console&`; `StatusLED` becomes `NeoPixelIndicator`; `ButtonBase`
  takes `hal::GpioPin&`.
* `device::PowerMonitor` + a driver, replacing the placeholder battery values in
  `ArdufliteCRSFTelemetry`. Small, visible, and the first end-to-end exercise of
  ADR-019's "add a sensor" path.
* Flash-telemetry rotation, purge and full-disk tests against `MemoryFileStore`.

**Behaviour change:** CRSF battery telemetry starts reporting real values.
**Risk:** low. Independent of Phases 3–6.

---

## Phase 8 — Cleanup and proof

* Delete `include/PinConfiguration.h`, `include/IMUConfiguration.h`, and the pin
  halves of `ReceiverConfiguration.h` / `CSRFConfiguration.h`.
* Retire superseded `test_production_contracts.cpp` assertions; keep only those with
  no behavioural equivalent.
* Rewrite AGENTS.md "Module Boundaries" and "Folder Structure"; update README's
  compile-time-flags section.
* **Correct the two FPU claims the C3 build map disproves** (§00 2.9b): AGENTS.md
  "Performance Considerations" §3, and the `powf` comment at `ArduFliteIMU.cpp:405`.
* **Prove extensibility with three concrete additions:**
  1. A third IMU driver — ICM-42688 is the natural choice, SPI-capable, so it also
     exercises `RegisterDevice` over `SpiBus`.
  2. A `host_sim` board running the whole flight stack on a laptop.
  3. A redundant-sensor descriptor — a second MPU-6500 at 0x69 — as a **data-only**
     change. If this needs any code change, the sensor abstraction did not land.

  If any of the three takes more than a day, fix the abstraction before declaring
  done.

**Risk:** low. **Done when:** the §00 §4 table is answerable with "one file".

---

## Phase 9 — Own Madgwick *(optional, separate)*

Not part of the HAL work; enabled by it. `estimation::MadgwickEstimator` replaces the
Adafruit wrapper, validated by L3 replay of every logged flight against the wrapped
version (ADR-017). Motivated by the GPL notice in `Adafruit_AHRS_Madgwick.cpp` inside
an MIT project — **not** by performance, which measures at ~0.3–0.8 % of the core.

**Done when:** quaternion divergence within tolerance across `FL001` and `FL002`;
float-op count and loop timing measured on hardware; bench session; one short flight.
**Risk:** high in isolation, low with the replay oracle.
**Rollback:** one line in `Board.cpp`.

---

## Effort estimate

Rough, for one person working evenings. **A planning aid, not a commitment** —
comparable refactors run long.

| Phase | Estimate | Dominant risk |
|---|---|---|
| 0 — foundations | 3–4 sessions | none |
| 1 — config hygiene | 1 session | wrong servo endpoint after NVS wipe |
| 2 — platform layer | 4–5 sessions | loop timing regression |
| 3 — actuators | 3–4 + bench | surface travel changes |
| 4 — RC input | 3–4 + bench | failsafe behaviour |
| 5 — sensor drivers | 4–6 + bench | sensor scaling |
| 6 — estimation | 5–8 + bench | attitude estimate |
| 7 — peripherals | 2–3 sessions | none |
| 8 — cleanup and proof | 3–4 sessions | none |
| 9 — own Madgwick | 2–3 + bench | attitude estimate |

If Phase 5 or 6 runs past a week, split it across two branches rather than letting
one branch go stale against `main`.

## Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Virtual dispatch costs more than estimated | Low | High | Phase 2 measures it before anything depends on it; escape hatch is templating `RegisterDevice` (ADR-002) |
| Own register driver scales differently from FastIMU | Medium | High | Phase 5 cross-check harness, both running against the same sensor |
| Refactor changes estimator behaviour subtly | Medium | High | Phase 6 L3 log replay as an oracle |
| Calibration lost in the EEPROM→NVS migration | Medium | High | One-time migration + verify on the aircraft; `calibrate imu` is a 10 s recovery |
| Axis map ported incorrectly | Low | **Very high** | Ports verbatim as `{+X, −Y, +Z}` (ADR-007); six-orientation bench check verifies |
| Config rename leaves a wrong servo endpoint | Low | High | Phase 1 bench check of travel before flight |
| Flash growth breaks the lite build | Low | Medium | CI size budget from Phase 0 |
| §03 does not survive a compiler | **High** | Low | That is Phase 0's job; update the spec as it happens |
| Scope creep into config system or control laws | High | Medium | §01 §2 non-goals; reject in review |

## Definition of done

1. Every item in §00 §4's table is a one-file change.
2. `src/hal/platform`, `src/hal/device` and `src/hal/core` contain no vendor type,
   and no flight code names `ledc`, `Wire`, `Serial` or a driver type.
3. Exactly one `#if` selects the board.
4. Host tests exercise real production code for every driver and algorithm in §05
   §10 — no mirrored formulas remain.
5. CI runs host tests, static checks, and a 2×2 build matrix with size budgets.
6. A third IMU, a host-sim board and a redundant sensor were each added in under a
   day.
7. AGENTS.md and README describe the tier model accurately.
