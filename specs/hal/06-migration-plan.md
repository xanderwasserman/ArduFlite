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
* **Phase gate** = host tests green + the phase's bench checklist. Then tag
  `hal-phase-N`. **No test flights until the whole refactor is complete** — see
  "Flying is deferred" below for what that changes.
* **Rollback** = revert the phase's merge commit. Nothing spans phases.
* **One dominant risk per phase.** If a phase has two things that could ground the
  aircraft, it is two phases.

## Flying is deferred to the end — what that changes

The maintainer will not fly until the entire refactor is done. That is a legitimate
call, and the plan adapts rather than pretends otherwise. But the consequences should
be stated once, plainly:

**What is lost.** The original design caught errors at each phase gate, so a problem
was attributable to the twenty-odd files that phase touched. With flight deferred,
**aerodynamic-level errors accumulate silently across all ten phases** and surface
together on one flight. That is precisely the big-bang validation the phased plan
existed to avoid. Nothing below fully recovers it.

**What still works, and must therefore work harder:**

| Substitute | Catches | Now more important because |
|---|---|---|
| **Bench verification** | wrong sign, wrong travel, dead output, failsafe not firing, arming logic | it is the only hardware check left. Every phase gate's bench list is now mandatory, not advisory |
| **L3 log replay** | estimator divergence, command-chain changes | it is the **only** in-flight-behaviour oracle available. Promoted from a Phase 6 gate to a standing regression suite (below) |
| **A/B on the bench** | loop timing, stack usage, CPU regressions | unchanged in value |
| **Phase 5 cross-check harness** | sensor scaling and bias | unchanged in value |
| **Six-orientation bench check** | axis map errors | unchanged in value |

**What nothing substitutes for:** control authority and tuning, vibration coupling
into the IMU, thermal drift, real failsafe behaviour in the air, and any aerodynamic
consequence of the microsecond-resolution output change in Phase 3.

### Three adaptations

1. **Log replay is promoted to a standing gate.** It was a Phase 6 exit criterion.
   It now runs in CI from Phase 6 onward, over *every* log in `docs/flight_logs/`, and
   is a gate for Phases 6, 7, 8 and 9. It is the closest thing to a flight available.
2. **Bench checklists become mandatory phase gates**, not "should". A phase does not
   get tagged without them.
3. **The first flight is a maiden flight, not a test flight** — see the checklist at
   the end of this document. It carries the accumulated risk of ten phases, so it
   should be flown like an unproven airframe: calm conditions, height, `MANUAL_MODE`
   on a switch and a thumb on it.

**One small silver lining:** with no flights between phases there is no
"aircraft out of service" cost, so phases can be reordered or merged more freely than
the dependency graph alone requires. Tagging still matters — it is what makes a
regression bisectable once flying resumes.

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
* ~~`tests/unit/hal_host/`: the host platform~~ — **deferred, each fake to its first
  consumer.** Writing `FakeRegisterDevice` before there is a driver to drive it, or
  `RecordingPwmOut` before `PwmActuatorBank` exists, means guessing at the shape of a
  consumer that does not exist. Each fake now lands in the phase that first needs it:
  `VirtualClock`/`StepScheduler` in Phase 2, `RecordingPwmOut` in Phase 3,
  `LoopbackUart` in Phase 4, `FakeRegisterDevice` in Phase 5, `MemoryFileStore` in
  Phase 7. When they do land it is under `tests/unit/hal_host/`, **not**
  `src/hal/host/` — `arduino-cli` compiles `src/` recursively, so that would link
  `std::thread` into the flight firmware (review R20).
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
* **Only classes that SURVIVE the refactor get wired** — applying the same "never do
  the same work twice" rule that produced revision 2 (review R10). `ServoManager` is
  deleted in Phase 3 and `ArduFliteIMU` in Phase 6, so migrating their internals to
  the HAL now is throwaway work. Phase 2 wires `ArduFliteController` (27 locks, 10
  watchdog calls, 2 tasks — and the class where the loop-timing A/B gate is measured)
  and `ArduFliteCLI`'s task creation. The rest arrive with their own phases.
* Surviving classes take `hal::Clock&` / `hal::Watchdog&` / `hal::Scheduler&` /
  `hal::Mutex&`, replacing direct `micros()`, `esp_task_wdt_*` and `xTaskCreate`.
* `Board` gains a fixed **mutex pool** — flight-layer classes cannot construct an
  `Esp32Mutex` without breaking the layering rule, so the composition root hands out
  `hal::Mutex&` from static storage (ADR-011: no heap after boot).
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

**De-risking gate — the cross-check.** Revision 3, after implementation.

The original plan was a bench firmware running the new driver *and* FastIMU against
the same physical sensor simultaneously, with FastIMU deleted only once that passed.
That is not how it was done, and the reason is worth recording rather than glossing:
a dual-stack firmware would have had both libraries configuring the same registers on
the same chip. FastIMU's `init()` writes `PWR_MGMT_1`, the range fields and both LPF
fields; so does `drivers::Mpu6500::begin()`. Whichever ran second would win, and the
"comparison" would have been two readers of one configuration — precisely the thing
the harness was meant to check.

**What replaced it, in two parts:**

1. **Static equivalence, on the host, already done — and it earned its keep.**
   The register writes and scaling constants were lifted from the libraries being
   replaced and pinned byte for byte (ADR-031). This is what caught the BMP280
   configuration diverging from Adafruit's defaults — temperature x2 instead of
   x16, and the hardware IIR filter switched ON, which would have sat in series
   with the existing altitude EMA and changed the vario's dynamics. See
   `test_mpu6500.cpp`:
   `BeginReproducesTheLegacyRegisterConfiguration` asserts the exact bytes
   (`SMPLRT_DIV=2`, `DLPF_CFG=3`, `A_DLPF_CFG=3`, FCHOICE_B cleared, 500 dps, 4 g),
   and `ScalingMatchesTheLegacyValues` asserts `raw * range / 32768` at four points
   including both full-scale extremes. A scaling or configuration divergence is now a
   red test, not a flight surprise.

2. **A/B on the bench, outstanding — see the checklist below.** Same protocol as the
   Phase 2 timing A/B: flash the pre-Phase-5 firmware, record, flash Phase 5, record,
   compare. One board, one session, no dual-stack firmware needed. Git makes the
   before-image trivially recoverable.

   The equivalent BMP280 assertion is
   `BeginReproducesTheLegacyAdafruitConfiguration`: `CTRL_MEAS == 0xB7`,
   `CONFIG == 0x00`.

**Deferred to Phase 8:** `drivers::Mpu9250` (ADR-029) — no board declares one, no
hardware to verify against, and the magnetometer path it would feed has never
executed. It becomes the extensibility proof instead.

**Defects found and fixed during Phase 5 review** — recorded because each was
invisible to the tests and builds that were passing at the time:

| Defect | Why nothing caught it |
|---|---|
| Both drivers had **no power-up delays**; the 100 ms sat in `Board` *before* the reset it was meant to follow (ADR-030) | A fake bus answers instantly; hardware usually gets away with it |
| BMP280 read its **trimming during the NVM copy** — plausible but wrong coefficients, and every pressure wrong for the flight | The read succeeds; only the values are wrong |
| BMP280 **configuration diverged** from the flying one (ADR-031) | Nothing compared against the library being replaced |
| A failed I2C burst left the aircraft **fusing a frozen sample with `isHealthy()` true**, indefinitely | Stale data is finite and in range, so every validity check passed. Inherited from FastIMU, not introduced here |
| `baroCalibrate()` skipped its yield on a failed read, **spinning the CPU** for the whole calibration window | Only reachable with a dead barometer |

**Behaviour change:** none intended in the sensor data path — the same numbers from
a different code path, with configuration and scaling proven identical on the host and
**not yet confirmed against the physical sensor.**

Two behaviour changes ARE intended, both deliberate and both improvements:
1. The BMP280 is now soft-reset at boot, so a warm reboot no longer inherits the
   previous run's configuration. Adafruit never reset it.
2. A failed sensor read now marks the IMU unhealthy instead of silently fusing the
   last good sample forever. This is a **safety fix**, not a refactor artifact —
   see the defect table above — and it means a dead I2C bus in flight now surfaces
   through the existing failsafe path rather than presenting as a frozen attitude.
**Risk:** high. Mitigated on the host, with one bench measurement outstanding.

**Status:** implemented. 218 host tests pass; both builds clean. Binary shrank
~10 KB per variant as FastIMU, Adafruit_BMP280, Adafruit_BusIO and
Adafruit_Unified_Sensor left the build.

### Phase 5 bench checklist

Nothing here needs an airframe or a propeller. The board on USB is enough.

- [ ] **Boot inventory** names the right parts:
      `MPU-6500 ready (500 dps, 4 g, 333 Hz)` and `BMP280 ready`.
- [ ] **WHO_AM_I.** If the IMU is disabled at boot, the log now prints the byte the
      part actually returned. On the prototype airframe this is the one measurement
      that settles whether the IMU is counterfeit — FastIMU discarded it.
- [ ] **A/B against pre-Phase-5 firmware, one session, one board:**
      - [ ] Board level and still. Record accel XYZ and gyro XYZ on both firmwares.
            Accel should read ~1 g on one axis and ~0 on the others; gyro ~0 on all
            three. **The two firmwares must agree to within sensor noise** — a
            constant ratio between them means a scaling error, a constant offset
            means a configuration one.
      - [ ] Rotate through the six orientations, resting on each face. Confirm the
            axis that reads ±1 g and its **sign** match between firmwares. This is
            what catches a byte-order or two's-complement error, which the
            level-and-still test cannot see.
      - [ ] Rotate briskly about each axis in turn; confirm the gyro sign matches.
      - [ ] Compare baro pressure at rest. Both should read station pressure
            (~950–1030 hPa depending on altitude and weather). A wrong compensation
            coefficient shows up here as a plausible but *different* number, so
            compare against the old firmware rather than against expectation.
- [ ] **Spare board (no IMU).** Confirm it boots, logs the IMU as absent, and reaches
      a usable CLI rather than hanging or rebooting.
- [ ] **Stale-data health check.** With the board running, pull the IMU's SDA or SCL
      line (or the sensor's power) and confirm the IMU is reported UNHEALTHY within
      the failure threshold. Before Phase 5 this went undetected: the attitude simply
      froze at its last value and `isHealthy()` kept returning true. Reconnect and
      confirm it recovers.
- [ ] **Altitude drift.** Leave it running 5 minutes on the bench; altitude should
      stay within a metre or two. A sign error in the compensation shows as steady
      drift, not as a wrong constant.

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

### Status — partially implemented

Landed and verified (238 host tests, both builds clean, layering passes):

| Component | Notes |
|---|---|
| `Board` sensor **spans** | Replaced the single-pointer accessors from Phase 5. Empty span = not fitted; the interface no longer changes when a board gains a second IMU |
| `device::Measurement` base | Interface correction — ADR-032 |
| `estimation::ImuState` | Published snapshot type, `static_assert`-ed trivially copyable. `FlightState` deliberately absent |
| `AttitudeEstimator` + `AdafruitMadgwickEstimator` | Wraps the library unchanged, `getYaw()`'s `+180.0f` included, so Phase 9 keeps a valid baseline |
| `MotionDetector` | Extracted verbatim. Its tests previously exercised a **re-implementation** of the thresholds, which could pass indefinitely while the real code diverged; they now drive the real class (11 tests) |
| `FirstHealthySelector` | 8 tests covering failover, flap, all-dead and empty-span cases — none of which the current hardware can produce |
| **R18 seqlock ordering** | Fixed in place across all four sites, rather than waiting for the rewrite. See below |

| `CalibrationService` | State machine driven from step 11 of the tick. **The pause protocol is deleted** — ADR-034. 14 tests |
| Tick-order correction | Offsets are sensor-frame and must be subtracted BEFORE the axis transform. The spec had it backwards, which would have doubled the bias on three axes — ADR-033 |
| `LowPassVec3`, `AltitudeFilter` | Extracted from nine hand-written EMA lines and the altitude/vario path. 13 tests, mostly on the seeding cases that fabricate a climb rate if got wrong |
| `InertialSubsystem` | The twelve-step tick, **with no FreeRTOS in it** — 12 tests drive it with fakes and a virtual clock. Rate-aware sampling replaces `BARO_DECIMATION_FACTOR`: each part declares `nativeRate_hz()` and the loop divides |

**Cutover complete.** `ArduFliteIMU` is now a ~430-line adapter over
`InertialSubsystem`, down from 1394. It owns the task, the calibration blob and
the `ImuState` → `ImuSnapshot` translation; everything else moved to
`src/estimation/` and is covered by host tests. Consumers — controller,
telemetry, CLI, web UI, state machine — compiled unchanged.

| Component | Notes |
|---|---|
| Cutover to `InertialSubsystem` | `ArduFliteIMU` 1394 → 433 lines. Burn-down 123 → 114 |
| `FlightState` out of the estimation layer | Not in `ImuState`; the façade merges it in from an atomic that StateManagement owns. Two owners is how a value drifts |
| `Esp32SettingsStore` + `crc32` | Calibration in NVS with a CRC, one-way migration from EEPROM — ADR-035. 6 tests |
| Cross-task offset staging | `setOffsets()` is called from the CLI task while the sampling task ticks. Staged and adopted at the top of the tick; a direct write let the tick observe three new components and three old ones |
| Contract tests rewritten | The old source-grep test asserted identifiers that have all moved. Replaced with the invariants still worth grepping for — see below |

#### The contract tests that survived, and why

`ProductionContracts.ImuSnapshotAndBarometerStayRealtimeSafe` grepped
`ArduFliteIMU.cpp` for `snapshotVersion`, `_baroTickCounter`, `publishSnapshot()`
and so on. Every one of those identifiers moved, and every property they stood
in for is now covered by a *behavioural* test against the real class — seqlock
ordering by `test_seqlock`, decimation by `SlowSensorsAreSampledLessOften`, the
climb-rate seeding by `FirstSampleProducesNoClimbRate`.

Grep was a proxy for properties that could not be tested directly. Where they can
be, the proxy goes. Three invariants genuinely cannot be, and those were kept and
sharpened:

- **`BarometerHasNoTaskOfItsOwn`** — no behavioural test can assert the absence
  of a task nobody created, and a second task on the sensor mutex is what caused
  the priority inversion and dropped baro samples.
- **`EstimationLayerContainsNoRtosCalls`** — the property that makes the twelve
  step ordering testable at all. If `vTaskDelay` or `millis()` appears in
  `src/estimation/`, the host tests quietly stop meaning what they claim.
- **`CalibrationDoesNotSuspendTheSamplingTask`** — guards ADR-034 against the
  pause protocol growing back.

Plus `CalibrationBlobMagicIsUnchanged`: changing `0xDEADBEEF` does not fail
loudly, it makes every existing aircraft report "no stored calibration" and
silently recalibrate.

#### `ArduFliteIMU` — deleted

Gone. 1394 lines at the start of Phase 6, then 577, then zero. `ArdufliteApp`
now owns the estimation layer directly — an `AdafruitMadgwickEstimator`, a
`FirstHealthySelector` and an `InertialSubsystem` — and every consumer reads
`ImuState`.

Done in four stages, each independently buildable:

1. **`src/core/FlightTypes.h`.** `Vector3`, `EulerAngles`, `FlightState` and
   `MotionSignals` were living in the IMU header, which forced anything wanting
   an attitude setpoint to include a sensor class. They are not the IMU's types:
   `ControlMixer` builds an `EulerAngles` from stick input and never touches a
   sensor.
2. **`StateManagement` owns `FlightState`.** It already decided the transitions;
   the storage now lives with the authority instead of in the IMU, where it sat
   only because telemetry already had an IMU handle.
3. **Consumers migrated to `InertialSubsystem*` and `ImuState`**, compiler-driven.
4. **Deleted.**

**`Vector3` and `EulerAngles` were deliberately NOT replaced** by `Vec3f` and
`EulerAnglesDeg`. The original plan said to; looking at the actual uses, that was
wrong. `EulerAngles` appears in ~60 places as the controller's setpoint and
command vocabulary, and `setAttitudeControlSetpointRads()` uses it for RADIANS.
A `Deg` suffix would be a false claim about half its uses, and the diff would
have been enormous for no gain. They needed a home, not a replacement.

#### What the rename caught

Every field name differs between the two structs — `accel` → `accel_g`,
`climbRate` → `climbRate_mps` — so a missed call site fails to compile rather
than silently reading the wrong value. That property is what made a change this
wide safe to do without hardware.

One collision needed handling first: `ImuSnapshot::orientation` held EULER
angles while `ImuState::orientation` holds a QUATERNION. Same name, opposite
meaning. The compiler happened to catch it only because `Quaternion` has no
`.roll` member — luck, not design. `ImuState::orientation` is now
`orientation_quat`, so the collision cannot recur.

The sweep also surfaced a real defect it did not cause: the CLI's in-flight
guard read `if (cliIMU && getFlightState() == INFLIGHT)`. Once flight state
stopped coming from the IMU, that null check became a hole — on any path where
the CLI's IMU pointer was never set, a dangerous command would pass the guard
mid-flight. The guard now tests the flight state alone.

#### Task ownership, and the RTOS boundary

`InertialSubsystem` spawns its own task, through `hal::Scheduler` rather than
`xTaskCreate`. The "no RTOS in the estimation layer" invariant is about
depending on the RTOS *directly*, not about never running in a task, and
`check_layering.sh` enforces exactly that distinction.

It is also default-constructible, with `bind()` supplying dependencies after
`Board::begin()`. The controller and CLI take its address during static
initialisation, before Board's sensor spans exist — the same ordering problem
`Board` solves with `constinit` storage and a `begin()`.

### Phase 6 — COMPLETE

Every spec item implemented and verified: `InertialSubsystem`,
`AttitudeEstimator` + the Adafruit wrapper (yaw `+180` quirk reproduced),
`MotionDetector`, `FirstHealthySelector` with `SelectionState` published,
`CalibrationService`, `ArduFliteIMU` deleted, `FlightState` moved to
`StateManagement`, calibration offsets in NVS with a CRC, the axis transform
pseudovector-correct, and **L3 log replay**.

286 host tests, both builds clean, layering passes, burn-down 100.

#### L3 replay — what it proves, and what it does not

Replays FL001 (the maiden flight, 943 rows, ~59 s, roll spanning −177..+22°)
through `AdafruitMadgwickEstimator`. The FL002 logs are ground recordings —
log_007 spans two degrees of roll and would exercise nothing.

**It cannot reproduce the original filter state.** The IMU task runs at 500 Hz;
the flash log writes at a variable ~23 ms, so the real filter took ~11 updates
per replayed sample, and Madgwick is path-dependent. Exact agreement is neither
achievable nor asserted.

Measured against this log with defects injected deliberately:

| Injected defect | mean error | detected |
|---|---:|---|
| baseline | 1.51° | — |
| gyro X sign flipped | 6.11° | **yes** |
| accel X/Y swapped | 1.62° | **no** |
| gyro 10 % scale error | 1.61° | **no** |

So the tracking bound catches gross gyro sign errors and nothing subtler — the
accelerometer correction is slow at beta 0.1 and the gyro dominates across 23 ms
steps. **It is not a substitute for the six-orientation bench check**, which is
what catches an axis swap.

The second assertion is the one that will matter: a fingerprint over every
attitude produced across the flight. Any change to the fusion path moves it,
including the ones above that tracking misses. **This is Phase 9's equivalence
oracle** — when the own Madgwick lands, a matching fingerprint means bit-for-bit
equivalence over 833 real samples, and a differing one must be explained and
re-baselined deliberately.

Also noted: `docs/flight_logs/FL002_2026-05-24/log_006.csv` contains a row whose
`flight_state` column reads `0.68`. That is a malformed record — column
misalignment or a truncated write — and predates this work. Worth investigating
before the flash-telemetry format is trusted for post-incident analysis.

---

**Found during Phase 6, fixed, structural work deferred:** `EulerAngles` carries
attitude (deg), rates (deg/s) and normalised surface commands (-1..+1) with
nothing to tell them apart, and that caused a full-deflection transient on IMU
failure in RATE_MODE. The defect is fixed; splitting the type into three is
ADR-037 and needs its own phase, because it touches the command path to the
servos.

### Phase 6 bench checklist

The estimation layer is the aircraft's sense of which way is up. Nothing here
needs an airframe, but all of it needs doing before the maiden flight.

- [ ] **Calibration survives the upgrade.** Boot the new firmware on a board that
      already has a stored calibration. The log must say
      `Migrating calibration from EEPROM to NVS`, and `calibrate imu` must NOT
      run on its own. Reboot: it should now load silently from NVS.
- [ ] **Attitude matches pre-Phase-6 firmware.** A/B, one session: hold the board
      in each of the six orientations and compare roll/pitch/yaw against the old
      firmware. **Signs must match on every axis** — this is what catches the
      offsets-before-transform ordering (ADR-033) having gone in backwards.
- [ ] **Yaw still reads 0–360, not ±180.** The wrapper reproduces
      `Adafruit_Madgwick::getYaw()`'s `+180` offset deliberately; every log
      recorded so far carries it.
- [ ] **`calibrate imu` with nothing paused.** Run it and confirm: telemetry keeps
      streaming throughout, attitude keeps updating (it used to freeze), the
      progress reaches 100%, and the resulting offsets are close to the previous
      ones on the same stationary board.
- [ ] **Calibration rejects a disturbed run.** Start `calibrate imu` and move the
      board around. The offsets it produces are garbage by definition — confirm
      you can simply re-run it, and that the second run is accepted.
- [ ] **Altitude and climb rate.** At rest, climb rate should sit near zero with
      no drift. Lift the board a metre and set it down: altitude should track and
      settle. **No climb-rate spike at boot** — that is the seeding path.
- [ ] **Launch detection.** A gentle throw motion by hand should set
      `launchDetected`; sitting still for two seconds should set
      `stableDetected`. Neither should fire at boot.
- [ ] **`stats`** — snapshot retry counters should stay near zero. A rising
      `retryLimitHits` means readers are losing races with the publisher.
- [ ] **Spare board (no IMU).** Boots, reports the IMU absent, reaches the CLI.
- [ ] **IMU failure in RATE_MODE does not slam the surfaces.** Arm on the bench in
      RATE_MODE with a stick deflected, then pull SDA (or unpower the IMU). The
      surfaces must **centre** and then follow the sticks proportionally as
      MANUAL_MODE takes over. Before this fix they went hard over on every
      deflected axis for one ControlMixer period — see ADR-037. Reconnect and
      confirm the mode restores without a second transient.

#### Why tick() has no FreeRTOS in it

The single most useful structural decision in this phase. `ArduFliteIMU::update()`
took the mutex, fed the watchdog and ran the task loop's timing, so **none** of
its logic could be executed anywhere but on hardware, inside a running task. The
step ordering was therefore unverifiable — every plausible order compiles and
runs, and a wrong one produces a plausible-looking aircraft.

`tick(dt)` takes its dependencies by reference and touches nothing else, so a
host test can feed it a fake IMU, a fake barometer, a recording estimator and a
virtual clock. That is what made ADR-033 checkable: the offsets-before-transform
test asserts −0.3 where the inverted order gives −0.7.

It is also what makes **L3 log replay** possible rather than aspirational —
replay is `tick()` driven from recorded samples, with no task, no scheduler and
no board.

Still to do: `InertialSubsystem` itself, the façade and cutover, `FlightState`
moving to `StateManagement`, calibration offsets moving from EEPROM to
`SettingsStore`, and L3 log replay.

#### R18, fixed rather than deferred again

The seqlock write side marked the version odd with a **release store** and placed
no fence before the payload. A release store constrains what comes *before* it;
it does nothing to stop the payload writes below from being hoisted *above* it.
A reader could then observe an even version mid-write and accept a torn snapshot
as valid — silently, because the version check would agree.

The read side had the mirror error: acquire *loads* around the payload copy,
where acquire orders only what follows, letting the copy sink past the second
counter load and escape the check it exists to satisfy.

Both are now relaxed counter accesses with standalone fences, matching
`hal::SeqLock`. All four sites were wrong, including the *fallback* snapshot —
the one readers fall back to when the primary is contended, so tearing there
would first appear only when the system was already under stress.

This cannot manifest on the ESP32-C3: single core, in-order. It comes alive on
the dual-core ESP32 parts this HAL exists to port to, which is precisely why
leaving it until "the file gets replaced anyway" was the wrong call twice.

**Done when:** replay of `FL002/log_007.csv` matches the logged quaternion within
tolerance; the six-orientation bench check reproduces pre-phase accel/gyro signs on
every axis; calibration migration verified on the actual aircraft; `stats` at
baseline.
**Risk:** high — the estimator is the aircraft's sense of which way is up.
**Mitigation:** replay oracle plus a dedicated bench session before the test flight.

---

## Phase 6B — One type per quantity on the control path

**Goal:** make it impossible to assign a rate where a surface command is expected.

Implements ADR-037. Split out of Phase 6 deliberately: Phase 6 is the estimation
layer, this is the command path to the servos, and bundling them would leave a
bench A/B unable to attribute any change in handling.

### The problem, restated

`EulerAngles` is a bare `{float roll, pitch, yaw;}` carrying three different
quantities. Nothing distinguishes them; which one a value holds depends on the
flight mode, read separately. That already produced one full-deflection failsafe
bug (ADR-037), found by inspection rather than by any test.

### The types

| Type | Unit | Holds |
|---|---|---|
| `AttitudeDeg` | degrees | measured attitude, attitude setpoints |
| `AngularRateDps` | deg/s | gyro readings, rate setpoints, rate commands |
| `SurfaceCommand` | none, −1…+1 | mixer output, actuator commands |

Each is `{float roll, pitch, yaw;}` — same representation, distinct type. Not a
units library, no operator overloading, no dimensional analysis. Just three
names that the compiler will not silently interchange.

**No public radians type.** The only radians value in the flight layer is
`quaternionToEulerRads()`, a file-static helper whose result is converted three
lines later. Fold the conversion into it so radians never cross a function
boundary, and the type is not needed. `deadbandRads` stays a `float` — a scalar
whose identifier carries its unit is already unambiguous.

### Why types here when the estimation layer uses `Vec3f` + suffixed field names

These look inconsistent and are not. The rule is about **where the unit is
visible at the point of use**:

- `ImuState::accel_g` is read as `state.accel_g` — you cannot touch the value
  without reading the unit. A generic vector plus a suffixed field name is
  sufficient, and this is the convention the project already chose.
- `actuatorCmd = localRateSetpoint` has **no field name anywhere in it**. Both
  sides are bare locals whose meaning came from a mode flag read elsewhere. The
  type is the only thing that can carry the unit across that assignment.

So: **suffixed names where values are read in place; distinct types where they
cross boundaries and get assigned.** Phase 6B applies the second rule to the one
part of the system where the first is not enough.

### The decision this will force

`ArduFliteAttitudeController::update()` currently does:

```cpp
rateOut.yaw = localAttitudeSetpointDegs.yaw;   // angle -> rate
```

`attitudeSetpointDegs.yaw` is scaled by `maxAttYaw_deg`, an ATTITUDE limit, and is
then consumed by the rate controller as deg/s. The passthrough itself is
intentional — there is no heading reference without a magnetometer — but the
scaling is taken from the wrong config key for the way the value is used.

Under the new types this line does not compile. **This is the point.** Someone
has to decide whether the yaw stick in ATTITUDE_MODE commands a rate (in which
case it should scale by `maxRateYaw`) or something else. Do not resolve it with
a cast.

**Expect two or three more of these.** Every site the compiler rejects is a
place where the current code is relying on two quantities happening to share a
representation. Each needs a decision, not a conversion.

### Stages

Each stage builds and passes the suite on its own.

0. **Done already (Phase 6):** `MixerConfig` members carry unit suffixes
   (`maxAttYaw_deg`, `maxRateYaw_dps`) — ADR-038. The keys had them since
   Phase 1; the members did not, which is where the unit was being lost.
1. **Define the types** in `src/core/FlightTypes.h`, alongside explicit named
   conversions — `toSurfaceCommand(AngularRateDps, ...)` and friends — with each
   conversion documenting what it assumes. No call sites change yet.
2. **`SurfaceCommand` first**, from the servos backwards: `AirframeMixer::mix()`,
   `actuatorCmd`, `ControlMixer::mixManual()`. This is the smallest cluster and
   the one where a wrong value is most immediately visible on the bench.
3. **`AngularRateDps`**: rate controller, `pilotRateSetpoint`, `mixRate()`, and
   the gyro read from `ImuState`.
4. **`AttitudeDeg`**: attitude controller, `attitudeSetpointDegs`, `mixAttitude()`.
   The yaw question above lands here.
5. **`TelemetryData` and the flash log.** Field types change; the on-disk column
   order and format must NOT — existing logs stay readable and `tools/` keeps
   parsing them. Verify against a real log file, not by inspection.
6. **Delete `EulerAngles`**, and `Vector3` with it. Both went: `Vector3` was a
   second generic triple sitting alongside `Vec3f`, so every read of the IMU
   state was copied from one into the other for no gain — and the copy
   DISCARDED the unit that `Vec3f`'s source field name carried
   (`state().gyro_dps` became an untyped `gyro`). Consumers that only need a
   magnitude (PreflightCheck, TelemetryData) read `Vec3f` directly now.

### Tests

The compiler does most of the work, but it cannot check the conversions:

- A host test per named conversion, asserting scale and clamping. The
  rate→surface conversion is the one that produced the failsafe bug; pin it.
- Extend the existing mixer tests to the new type.
- A regression test for the ADR-037 failure path: entering the IMU-failure
  demotion must not produce a non-neutral `SurfaceCommand`.

### Explicitly NOT in scope

- Renaming `Vec3f`, or touching the estimation layer at all.
- Any change to PID gains, mixer limits or config keys. If the yaw decision above
  implies a config change, that is a **separate** change with its own bench
  verification — not folded into a type refactor.
- Operator overloading or arithmetic on the new types. Add it when something
  needs it.

**Behaviour change:** none intended. Every conversion the compiler forces should
reproduce exactly what the untyped code did, except where it exposes a genuine
ambiguity — and those are decisions to take deliberately and record, not to
paper over.
**Risk:** medium. The change is wide and mechanical, but it is on the servo
command path, and "mechanical" is exactly how a wrong decision gets applied
sixty times.
**Mitigation:** stage by quantity, servos-first, and treat every compiler error
as a question rather than an edit.

### Phase 6B — COMPLETE

All six stages done. `EulerAngles` is deleted; `AttitudeDeg`, `AngularRateDps`
and `AxisCommand` replace it. 294 host tests, both builds clean, layering
passes, burn-down 100. lite 633,856 / full 1,429,760 — within 700 bytes of
Phase 6.

**Verification note.** A post-completion review found the deleted
`ArduFliteIMU.{h,cpp}` present again in the working tree, referencing types that
no longer exist, so the tree did not build. The last successful build tree
contained only `FliteQuaternion` under `src/orientation/`, confirming the
deletion had taken effect at build time; the files reappeared afterwards (with
different permissions from the originals). Nothing included them — they were
orphaned, not wired back in. Removed again and both builds verified from a clean
build directory.

The lesson for the remaining phases: **"deleted" is not verified until a clean
build succeeds without the file.** A passing incremental build proves nothing
about a file that was already compiled.

**Naming correction made during execution:** the new normalised type was first
called `SurfaceCommand`, one letter from the mixer's existing
`actuators::SurfaceCommands` (aileronLeft, aileronRight, elevator, rudder). Two
types one letter apart meaning "roll/pitch/yaw demand" and "what each servo
does" is exactly the confusion this phase exists to remove. Renamed
`AxisCommand`, so the pipeline reads
`AxisCommand -> AirframeMixer::mix() -> SurfaceCommands`.

**Two further defects found, both of the ADR-037 family** — see ADR-039:

1. The flight mode was read **twice** — once by the mixer to pick a scaling,
   once by `CommandSystem` to pick a setter, with a queue in between. The
   failsafe pushes its mode and setpoint as separate entries, so a setpoint
   could be interpreted under the outgoing mode. Fixed by making the command
   carry a `SetpointKind`.
2. `pilotRateSetpoint` was **one slot for two quantities** — deg/s in RATE_MODE,
   −1…+1 in MANUAL_MODE. That slot is ADR-037's root. Split into
   `pilotManualCommand`, so the bug is no longer representable.

**One deliberate log change:** `rate_sp_*` now reads zero in MANUAL_MODE instead
of showing stick positions, because the sticks no longer live in the rate slot.
Manual input is still recorded via `rate_cmd_*`. Anything parsing `rate_sp_*`
across a manual segment sees different values than before.

The mixer's scaling arithmetic and the PID gains were **not** touched — verified
by diff. The only changes to those files are type and identifier renames.

### Phase 6B bench checklist

- [ ] **Surface travel unchanged.** Full stick deflection on each axis, all three
      modes, compared against pre-6B firmware. Same direction, same endpoints.
- [ ] **The ADR-037 failsafe still behaves.** IMU failure in RATE_MODE with a
      stick deflected: surfaces centre, then track sticks.
- [ ] **Yaw in ATTITUDE_MODE** behaves as decided above — and note in the log
      whether that is a change from before.
- [ ] **Flash log parses.** Pull a log written by 6B firmware through the
      existing `tools/` scripts unmodified.
- [ ] **PID gains are actually applied.** `config get rate.roll.*` and
      `att.roll.*` at the CLI, and confirm the boot log has NO
      "Config key not found" lines. Two keys were mismatched for seven phases
      (ADR-049): every PID ran without I or D, and ATTITUDE_MODE commanded no
      rate at all. **This is a real change in flight behaviour** — fly
      ATTITUDE_MODE on the bench first and confirm the surfaces now respond to
      attitude error, then re-check the tuning before flying.
- [ ] **`rate_sp_*` reads zero in MANUAL_MODE** and `rate_cmd_*` tracks the
      sticks. This is the one intended log change; confirm it rather than
      discovering it later in analysis.

---

## Phase 7 — Peripherals and I/O

* `drivers::LittleFsLogStore`, `NvsSettingsStore`, `NeoPixelIndicator`, and a
  console — over Arduino `Serial`, **not** `hal::Uart` as originally written.
  On the C3 `Serial` is USB CDC with no pins or baud rate; on the FireBeetle it
  is a real UART0. See ADR-043.
* `ArduFliteFlashTelemetry` takes `device::LogStore&`; serial backends and `Logging`
  take `device::Console&`; `StatusLED` becomes `NeoPixelIndicator`; `ButtonBase`
  takes `hal::GpioPin&`.
* **`ConfigPersistence` moves onto `hal::KeyValueStore`** (ADR-027). It currently
  includes `<Preferences.h>` directly, which is Arduino-ESP32 and does not travel
  even to another FreeRTOS target. The interface already exists with no consumers.
* `device::PowerMonitor` + a driver, replacing the placeholder battery values in
  `ArdufliteCRSFTelemetry`. Small, visible, and the first end-to-end exercise of
  ADR-019's "add a sensor" path.
* Flash-telemetry rotation, purge and full-disk tests against `MemoryFileStore`.

**Behaviour change:** CRSF battery telemetry starts reporting real values.
**Risk:** low. Independent of Phases 3–6.

### Status — in progress

| Item | State |
|---|---|
| `NvsSettingsStore` | **Done in Phase 6** as `Esp32SettingsStore` (ADR-035) |
| `ConfigPersistence` → `hal::KeyValueStore` | **Done** — ADR-040. `<Preferences.h>` no longer appears anywhere outside `src/hal/` |
| `Console` for `Logging` and serial backends | **Done** — ADR-042/ADR-043. The serial telemetry backends reach it transitively through `LOG_N`; none touch `Serial` directly |
| `NeoPixelIndicator` for `StatusLED` | **Done.** `StatusLED` deleted; `Colors.h` now speaks `device::Rgb`/`BlinkPattern` directly |
| `ButtonBase` → `hal::GpioPin&` | **Done.** No `digitalRead`/`pinMode` in the button classes |
| Flash-telemetry rotation/purge policy | **Done** — extracted and host-tested (14 tests). See below |
| `LittleFsLogStore` (full `device::LogStore`) | **Done.** `LittleFS` no longer appears in flight code; `MemoryLogStore` host fake, 16 tests |
| `device::PowerMonitor` + driver | **Deferred to the custom PCB — ADR-041.** Fabricated CRSF battery telemetry suppressed meanwhile |

#### Log rotation and purge — extracted and tested

The part of this item that carried the real risk is done. `LogRotationPolicy`
now holds the two rules that were buried in `ArduFliteFlashTelemetry`:

- **Index allocation** — monotonic growth as (highest + 1) until the space is
  full, then lowest-gap reuse. Deleting a mid-range log does not reclaim its
  index while room remains above, so log_007 stays reliably older than log_008.
- **Auto-purge** — how many of the oldest logs to delete to clear the free-space
  floor, with a cap against a filesystem that never reports more space.

Both are pure functions over an index list — no filesystem, no I/O — and are
covered by 14 host tests. **These paths were previously unreachable in testing:**
the purge branch requires filling a 1.9 MB partition and index exhaustion
requires a thousand files. They are also the paths that fail on a long flying
day rather than on the bench, and they fail at `startLogging()` — so the flight
simply is not recorded, and nobody finds out until they go looking.

Two behaviours are now pinned that were previously implicit:

- **The purge cap is a data-protection measure, not an optimisation.** A corrupt
  filesystem reporting free space that never rises would otherwise have the loop
  delete every log on the device chasing a threshold it can never reach.
- **A store reporting zero reclaim per log purges nothing.** Deleting flight
  history for no measurable gain is worse than failing to start the log.

The filesystem side re-measures free space after each delete rather than
trusting the policy's estimate, so it stops as soon as the real figure clears
and deletes the fewest flights.

#### `LittleFsLogStore` — done, and what it caught

`ArduFliteFlashTelemetry` no longer touches `LittleFS`. All file work goes
through `device::LogStore`, with `MemoryLogStore` as the host fake (16 tests
covering the session contract, offset streaming, and the full-medium path).

`ArduFliteWebServer`'s log endpoints — list, usage, download, delete — go
through the store too. The only remaining mentions of LittleFS in flight code
are two CLI help strings describing what `flash reset` does to the user.

Two defects surfaced during the migration, neither of which the compiler caught:

1. **`dumpLog` was silently broken.** After the file handle was removed the
   function still compiled — it printed its BEGIN and END markers with nothing
   between, because `f` was default-constructed and `f.available()` was always
   false. A build-clean, test-clean, completely non-functional `dumplog`.
2. **`readSession` had no offset**, which made streaming impossible and was the
   reason (1) could not be fixed as written. A flight log runs to hundreds of
   kilobytes; there is no buffer that size. Fixing (1) properly required
   correcting the interface — the third time in this refactor that an interface
   specified from what a thing *sounds like* did not survive its first real
   consumer (ADR-032, ADR-042).

Also added: a **latched** write-failure report. A full medium previously logged
an error per row at 50 Hz; it now says so once and marks the log truncated.

**A third gap: no `sessionSize()`.** The web listing shows each log's size, which
the interface could not answer. Added rather than worked around.

#### A security contract that had to move, not be deleted

`ProductionContracts.WebSecretsFlashAndCaptiveDnsStayProtected` failed on the
listing endpoint: it asserted `isLogFilename(name)`, which filtered a directory
walk so a stray file could not be presented to the UI as a flight log.

That filter is gone from the endpoint — because it moved **into the store**.
`listSessions()` only returns indices it parsed from `log_%03u.csv` with a range
check, and the name shown to the user is now *synthesised from that integer*
rather than echoed back from the filesystem. A non-log filename cannot reach the
response at all, which is stronger than the check it replaced.

The contract was updated to assert the new guard, with that reasoning recorded
next to it. **The failing assertion was not simply removed** — a security
contract failing because the protection moved and one failing because the
protection is gone look identical from the test output, and the difference has
to be established by reading the code, not by making the test green.

`constinit` caught a third static-init hazard here — `LittleFsLogStore` holds an
Arduino `File`, whose constructor is not constexpr, so it is held as an
`optional` and emplaced on first use. The other two were `Esp32I2cBus` and
`Esp32SettingsStore`.

#### Fixed: telemetry no longer emits through the logger

The serial telemetry backends wrote via `LOG()`/`LOG_N()`, so `setLevel(Off)`
silenced telemetry and a log line could interleave into the machine-parsed Q
stream. Both now write through `ConsoleWriter` — data straight to
`device::Console`, diagnostics still through the logger. See ADR-044.

#### PowerMonitor — deferred to the custom PCB (ADR-041)

Battery sensing will be an ADC input on a future custom PCB. Not built now: the
`hal::AnalogIn` interface's shape depends on its consumer (raw counts vs
calibrated millivolts — the ESP32 ADC is non-linear and needs per-chip
calibration), and the divider ratio and attenuation are properties of a board
that does not exist.

**Behaviour change made instead:** CRSF was transmitting 0.0 V / 100 % remaining
as if measured. That frame is now suppressed until a real monitor exists. See
ADR-041 — a placeholder that is transmitted is not a placeholder, it is a wrong
reading on the pilot's display.

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

### Status — in progress

| Item | State |
|---|---|
| Correct the two FPU claims | **Done.** AGENTS.md now states the C3 has NO FPU (rv32imc, soft-float, verified from the build map) and that the rule differs by board. Both `powf` comments were already corrected when the code moved to `AltitudeFilter` |
| Delete `BOARD_TYPE` | **Done** — and it was hiding a defect; see below |
| Redundant-sensor descriptor (proof 3) | **Done — it FAILED and was fixed.** ADR-045 |
| ICM-42688 driver (proof 1) | **Dropped**, superseded by real hardware |
| BMI323 driver (SEN0697 10-DOF) | **Done** — ADR-050. 17 tests. Swapping the board's IMU to it is a one-line descriptor change |
| BMP581 barometer (SEN0697) | **Done** — ADR-051. 13 tests |
| BMM350 magnetometer (SEN0697) | **Done** — ADR-052. 28 tests, written together with the fusion path that consumes it, as ADR-051 required |
| Optional nine-axis fusion | **Done** — ADR-052. Decided per tick from the board's magnetometer span; no enable flag |
| Barometer span counter defect | **Found and fixed** — ADR-053. A BMP581-only board reported no barometer at all |
| `host_sim` (proof 2) | **Done.** Estimation core runs on a laptop; roll tracks to 0.01 deg. Found the boundary — see below |
| Delete `PinConfiguration.h` / `IMUConfiguration.h` | **Done.** Both gone; `BOARD_TYPE`, `IMU_TYPE`, `BARO_TYPE` with them |
| Retire superseded contract assertions | **Done.** Architectural greps moved to `check_layering.sh` (ADR-036); `test_config_helpers.cpp` now calls the real header instead of re-implementing it |
| AGENTS.md / README structure sections | **Done.** Both rewritten for the HAL layout |
| File-by-file review | **Done.** Findings in ADR-057 through ADR-061. Every guard added was verified against the defect it exists to catch |
| Platform-call burn-down | **Done — 90 → 1** (ADR-058). The remaining one is `esp_wifi_set_ps` in `WiFiManager.cpp`, kept deliberately: that file is Arduino-WiFi throughout, so routing one call through a HAL would move the counter without moving the coupling |

**Moving the aircraft to the whole SEN0697 is a three-entry descriptor change** —
BMI323 at 0x69, BMM350 at 0x15, BMP581 at 0x47, `.sensorCount = 3`. Verified to
build with no code change, 28 bytes larger than the two-part descriptor because
all three drivers were already linked. Left disabled because the module is not
wired yet.

#### The magnetometer changes flight behaviour, and one piece is still missing

Nine-axis fusion now engages by itself on any board that declares a
magnetometer. Two consequences belong on the bench checklist rather than in a
changelog:

- **There is no hard-iron calibration, and the reason it matters is not the
  heading.** Madgwick normalises the accelerometer and magnetometer residuals
  into one unit gradient, so an uncalibrated magnetic offset never converges and
  permanently takes correction authority away from the accelerometer — letting
  **roll and pitch** drift on gyro bias. Those are the axes the control loops
  fly. Nine-axis fusion is therefore bench-only until hard-iron calibration
  exists. Full reasoning in ADR-054.
- **Heading hold, when it comes, goes on the ROLL axis, not yaw.** A fixed wing
  turns by banking; the rudder coordinates. A heading loop on yaw fights every
  aileron turn and uncoordinates it. See the correction section of ADR-054.
- **Resolved: the magnetometer is instrumentation, not a control input**
  (ADR-055). `imu.fuse_mag` defaults to **false**, so nothing above changes
  flight behaviour today. The field, a tilt-compensated heading and the field
  magnitude are logged so the part can be evaluated on a real airframe.
  Reassess when GNSS navigation lands.
- **`rate.yaw.ti_s` is now 8.0 s** (ADR-054), up from 0. The old rationale
  attributed the zero to the wrong loop — the yaw *rate* loop tracks the gyro
  and never needed a heading reference. 8.0 s is deliberately slower than either
  other axis. **It only takes effect on wiped flash**: a stored NVS value wins
  over the schema default, and every board that has run this firmware has 0.0
  written. Set it from the CLI or erase the key.

On every board without a magnetometer — which is all of them today — nothing
changes at all: the span is empty, `primaryMag()` returns null, and the tick
takes the same six-axis path it took before.

Nine-axis fusion engages only after **one second** of unbroken good readings and
drops on a single bad one (ADR-054). **`TelemetryData` carries no magnetometer
fields**, so today neither the flash log nor any live backend can answer "how
much of that flight was actually nine-axis?" — worth adding before the delay is
tuned against real hardware.

#### Two live config bugs, found by turning the real registry on

Once `ConfigRegistry` became host-buildable (ADR-048), `host_sim` dropped its
stub and linked the real registry and schema. It immediately logged
`Config key not found` — warnings the stub had masked, because a stub answers
every key.

Both were introduced by commit `5c0a907`, the Phase 1 unit-suffix rename, which
changed 45 key strings and missed `ConfigHelpers::buildPIDConfig()` — which
builds its keys with `snprintf` rather than the macros, and so is **invisible to
a grep-driven rename**:

1. `"%s.ti"` / `"%s.td"` stopped matching `…ti_s` / `…td_s`. Every PID in both
   loops read **0** for its integral and derivative time constants and ran as a
   pure-P controller.
2. `"%s.outlimit"` stopped matching the attitude loop's `…outlimit_dps`. The
   attitude PIDs clamped their own output to **zero** — ATTITUDE_MODE commanded
   no rate at all.

Neither failed loudly: `get()` warns and returns a default, the PID accepts it,
every test passed. **A gain of zero and a correctly configured gain are
indistinguishable except in flight.**

`buildPIDConfig()` now takes the output-limit suffix per loop, because the two
loops genuinely differ — ADR-037 in configuration rather than in code.
`tests/unit/test_config_keys.cpp` asserts that every composed key resolves to
the same value as its macro-spelled counterpart; reverting either fix fails it.

**These change flight behaviour** relative to the last time the aircraft flew,
and belong on the bench checklist: I and D terms are restored on every PID, and
ATTITUDE_MODE now actually commands rates.

#### `host_sim` — what it proved, and where it stopped

`tests/host_sim` builds and runs the real `InertialSubsystem`,
`AdafruitMadgwickEstimator`, `FirstHealthySelector`, `CalibrationService`,
`AltitudeFilter` and `AirframeMixer` on a laptop. No Arduino, no FreeRTOS, no
ESP32, no hardware. Simulated sensors implement the same `device::` interfaces
the real drivers do, so the estimation layer cannot tell the difference.

Result: commanded roll of 30 deg/s for two seconds, estimator tracks to
**0.01 degrees** — through the mirrored axis map, which correctly inverts the
sign. Altitude holds 100.00 m. The barometer takes 200 samples to the IMU's
4000, decimated by its own declared rate.

**The boundary it found.** The control loops could NOT be assembled here:

| Module | Blocker |
|---|---|
| `InertialSubsystem`, `AirframeMixer` | none — zero platform calls |
| `ArduFliteRateController` | one raw `xSemaphoreCreateMutex()` |
| `ArduFliteAttitudeController` | one raw `xSemaphoreCreateMutex()`, plus `<Arduino.h>` |
| `ControlMixer` | one raw `xSemaphoreCreateMutex()` |
| `ArduFliteController` | 13 calls — owns the FreeRTOS tasks; correctly platform-coupled |

**All fixed** — ADR-047 and ADR-048. The three controllers take an injected
`hal::Mutex`, and `ConfigRegistry` followed (it was the next layer down: the
loops were portable but `initFromConfig()` was not). `host_sim` now runs the
complete chain — simulated sensors, estimation, attitude loop, rate loop,
mixer — with zero FreeRTOS and zero Arduino below `ArduFliteController`.

`ArduFliteController` is a different case and should stay coupled: it owns the
tasks, and a task owner is exactly what a platform layer is for.

**A defect in the sim itself, worth recording.** The first run had the estimator
stall at 20 degrees against a truth of 60. The estimator was right; the
simulated accelerometer was wrong. A world vector in a frame rotating by +phi
about X transforms by `R_x(-phi)` — giving `(0, +sin phi, cos phi)`, not
`-sin`. With the wrong sign the accelerometer described a roll opposite to the
gyroscope, the two fought through the fusion filter, and the result was a
stable, plausible, wrong number. **A physically impossible sensor pair does not
look impossible from inside the filter.**

#### `BOARD_TYPE` was hiding a real defect

`#if BOARD_TYPE == BOARD_TYPE_WEMOS` gated every status-LED call. `BOARD_TYPE`
is hardcoded to `BOARD_TYPE_WEMOS` in `PinConfiguration.h`, so **those blocks
were always active on every board** — while `ArdufliteApp` constructed the
NeoPixel on a hardcoded pin 7. The FireBeetle descriptor declares
`statusLed.pin = kNoPin` because it has no pixel fitted, so that build was
driving a NeoPixel task against a pin with nothing on it.

Fixed by moving the indicator into `Board`, built from the descriptor's
`statusLed` entry, with `Board::indicator()` returning nullptr when none is
fitted. Call sites became `if (auto* led = ...->indicator())`. This is what the
board descriptor was for; the macro was answering a question the descriptor
already answers better.

---

## Phase 9 — Own Madgwick *(optional, separate)*

### Status — DONE. See ADR-056.

`estimation::MadgwickEstimator` replaces the Adafruit wrapper; the wrapper and
its library dependency are deleted from the firmware, `host_sim`, the unit-test
build and both CI workflows. 449 host tests pass; flash **shrank 2.7 KB**.

**Building the equivalence check is what found the real defect.** Adafruit's
fast inverse square root type-puns a `float` through a `long`, which is 4 bytes
on both ESP32 targets and 8 on a host — so on a host it reads uninitialised
memory and returns the **negated** result. The aircraft was never affected. But
every host measurement of that library, including this plan's "baseline mean
1.51 deg" and the replay's exact-output fingerprint, described a filter that
does not run on the aircraft. Asserting equivalence against it would have failed
a *correct* replacement and invited bending it towards the broken one.

The oracle is now the log's own attitude columns, produced on the aircraft by
the correct 32-bit path — and the replay test is **unconditional**, where it
used to be skipped whenever the library was absent.



Not part of the HAL work; enabled by it. `estimation::MadgwickEstimator` replaces the
Adafruit wrapper, validated by L3 replay of every logged flight against the wrapped
version (ADR-017). Motivated by the GPL notice in `Adafruit_AHRS_Madgwick.cpp` inside
an MIT project — **not** by performance, which measures at ~0.3–0.8 % of the core.

**Done when:** quaternion divergence within tolerance across `FL001` and `FL002`;
float-op count and loop timing measured on hardware; bench session; one short flight.
**Risk:** high in isolation, low with the replay oracle.
**Rollback:** one line in `Board.cpp`.

---

## Phase 10 — `PeriodicTelemetryBackend` *(done — ADR-064)*

Four telemetry backends each carry their own copy of the same task and mutex
lifecycle. `publish()` is byte-identical in all four; so is the idempotency
guard, the mutex allocation, the spawn-and-roll-back, the destructor, the task
guard and the snapshot.

That duplication is not theoretical. Migrating the four onto `hal::Task`
required hand-writing the `_task == nullptr` race guard four times, and the
first pass silently turned a 5 ms bounded wait into a try-lock — a change that
drops telemetry samples whenever `publish()` and the writer overlap, which at
50 Hz each is routine. One implementation makes both impossible.

### Shape

One layer between the interface and the backends:
`ArduFliteTelemetry` -> `PeriodicTelemetryBackend` -> the four.

```cpp
class PeriodicTelemetryBackend : public ArduFliteTelemetry {
public:
    void publish(const TelemetryData&) final;   // the identical body, once
    void begin() final;                          // template method
protected:
    PeriodicTelemetryBackend(const char* taskName, float frequencyHz,
                             std::uint32_t stackBytes = 4096);

    virtual bool onBegin() { return true; }      // Flash mounts its LogStore here
    virtual void runLoop() = 0;                  // the backend's own loop body

    bool  shouldRun()  const;                    // the task guard, in one place
    bool  snapshot(TelemetryData& out) const;    // bounded lock + copy
    hal::Mutex* dataMutex() const;
    float intervalMs()     const;
};
```

`begin()` becomes: idempotency guard -> `allocMutex()` -> `onBegin()` ->
`spawn()`, rolling back on any failure. The trampoline calls `runLoop()`.

### Snapshot timeout: one behaviour, not two

On a snapshot lock timeout the backends currently disagree — Debug and Flash
reuse the previous copy, QSerial skips the iteration. **Standardising on
reuse.** Flash's log wants an unbroken row cadence, and QSerial is expected to
be retired in favour of MAVLink over serial, so its skip semantics are not worth
carrying into a shared base.

`snapshot()` therefore leaves `out` untouched and returns false on timeout; the
caller keeps its previous copy. A backend that genuinely needs skip-on-stale can
check the return value.

### Deliberately NOT in the base

- **CRSF's watchdog and pause handling.** It registers and unregisters itself
  around a pause, feeds per iteration, sends rate-tiered frames, and sleeps with
  `sleepFor` rather than `sleepUntil`. Hooks existing for one subclass is the
  over-abstraction ADR-029 warns about. CRSF inherits the LIFECYCLE and keeps
  its own `runLoop()`.
- **Flash's file mutex and log operations.** `onBegin()` is seam enough.
- **The sleep call itself.** Fixed cadence versus fixed delay is a real
  per-backend decision.

### Order, with full verification between each

1. Add the base plus host tests against a fake backend. It needs only
   `hal::Mutex`, `hal::Scheduler` and `HostTask`, all of which exist — so this
   is the first telemetry lifecycle code that is host-testable at all.
2. **Debug** — simplest; proves the shape.
3. **QSerial** — adopts reuse-on-timeout, dropping its skip.
4. **Flash** — proves `onBegin()` carries the LogStore mount and second mutex.
5. **CRSF** — lifecycle only, loop untouched.

### Risks

- The base is new code in the telemetry path. Mitigated by being host-testable,
  which the backends are not — a net gain in coverage.
- **Flash is the riskiest step**: 617 lines, owns the flight log, two mutexes.
  Stopping after step 3 is a legitimate outcome if it turns awkward.
- Virtual dispatch on `runLoop()` replaces a direct call. Irrelevant at 10-50 Hz.

**Also delete:** `ArduFliteTelemetry::reset()` is never called by anything.
No point inheriting dead interface into a new base.

**Expected:** ~120-150 lines removed. The real return is that four lifecycle
invariants stop being four copies.

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
   §10. A formula may be reproduced in a test ONLY as an oracle — a reference the
   production path is asserted to agree with — and only where the production code
   cannot be called from a host. Three qualify: the control loops' dt clamp (three
   lines inside a task body), the RC stick shaping, and the axis transform. Each is
   tied to its subject by a contract test that fails if the two drift apart. A copy
   with no such tie is a mirrored formula and is not allowed.
5. CI runs host tests, static checks, and a 2×2 build matrix with size budgets.
6. A third IMU, a host-sim board and a redundant sensor were each added in under a
   day.
7. AGENTS.md and README describe the tier model accurately.


---

## Appendix — first flight after the refactor

This flight carries the accumulated risk of every phase, because none of it has been
airborne. Treat it as a maiden flight of an unproven aircraft, not as a routine sortie.

**Before leaving for the field**

- [ ] Host suite green, including full log replay over `FL001` and `FL002`.
- [ ] All ten phase bench checklists re-run **on the final firmware**, not just on the
      firmware of the phase that introduced them. Regressions between phases are the
      specific thing flight testing would have caught.
- [ ] Six-orientation IMU check: level, inverted, and on each side. Confirm accel and
      gyro signs against the pre-refactor values recorded in Phase 6.
- [ ] **Stick travel, BEFORE flying (ADR-060).** CRSF scaling now anchors on the
      protocol endpoints, so full stick produces ~25 % more surface deflection
      for the same `servo.*.defl`. Check every surface for mechanical binding at
      full stick, then re-set travel. This is a physical limit, not a tuning
      preference.
- [ ] **Yaw reads true (ADR-060).** Nose-forward should report ~0, not 180, and
      the radio's attitude display should track a turn continuously instead of
      jumping to negative values a few degrees in.
- [ ] **Magnetometer motor-interference test (ADR-055).** Wire the SEN0697, log
      on the bench, and run the motor through its full throttle range while
      watching `mag_field`. It should be **flat**. Movement with orientation is
      hard iron and is correctable; movement with throttle is the motor and is
      not. This ten-minute test decides whether heading hold is worth building
      at all, and should precede any further magnetometer work.
- [ ] **`rate.yaw.ti_s` is now 8.0 s by default (ADR-054), where it was 0.** The yaw
      rate loop has an integrator for the first time. A stored NVS value overrides the
      schema, so confirm what the aircraft is actually running with `config get
      rate.yaw.ti_s` rather than assuming the new default took. Check rudder behaviour
      in RATE_MODE before trusting it in the air.
- [ ] Servo travel and centre measured and compared against the Phase 3 numbers.
- [ ] `stats` and `tasks` compared against the Phase 2 A/B baseline.
- [ ] Boot inventory read and confirmed: every fitted part present, axis map as
      expected, no `Untested` board warning.
- [ ] Failsafe exercised on the bench: transmitter off, confirm configured bank,
      pitch and throttle; transmitter on, confirm recovery.
- [ ] Arming refused with the IMU unhealthy, throttle up, and link quality low.
- [ ] Flash logging confirmed writing, and a log pulled and opened.

**At the field**

- [ ] Calm conditions. Not a windy day, not a busy slope.
- [ ] Range check before launch.
- [ ] Launch in `MANUAL_MODE` — passthrough, no controller in the loop. Confirm the
      airframe flies and the surfaces move correctly before trusting any new code.
- [ ] Gain height. Every subsequent step happens with altitude in hand.
- [ ] Switch to `ATTITUDE_MODE` briefly, at height, with a thumb on the mode switch.
      Confirm it holds level and does not diverge. Switch straight back.
- [ ] Repeat for `RATE_MODE`.
- [ ] Only then fly the modes normally.
- [ ] Land, pull the flash log, and compare against a pre-refactor log for the same
      manoeuvres.

**Abort criteria — land immediately**

- Any surface moving in the wrong direction or not centring.
- Attitude estimate visibly disagreeing with reality on the telemetry.
- Any unexpected mode change, or arming state changing on its own.
- Anything at all that the pre-refactor aircraft did not do.
