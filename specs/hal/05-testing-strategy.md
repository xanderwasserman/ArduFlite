# 05 — Testing Strategy

**Status:** Draft · **Date:** 2026-08-02

The HAL's main practical payoff is that it makes the firmware testable. This
document says exactly what becomes testable, how, and what the tests replace.

---

## 1. Where the suite stands today

`tests/unit/CMakeLists.txt` compiles **two** production files: `pid.cpp` and
`FliteQuaternion.cpp`. The other six test files work around that by:

* **Reimplementing production logic in the test** — `test_servo_math.cpp`,
  `test_motion_signals.cpp`, `test_baro_decimation.cpp` each carry a copy of the
  algorithm and a comment telling the reader to keep it in sync by hand. These
  tests cannot fail when production code changes.
* **Asserting on source text** — `test_production_contracts.cpp` reads `.cpp` files
  and matches substrings. Useful as a guardrail; not a test of behaviour.

CI does not run the suite at all — `arduino_build.yaml` only compiles the sketch.

## 2. Test levels after the change

```
L4  Hardware-in-the-loop      bench aircraft, servos, real RX      manual, per phase gate
L3  Log replay                real flight CSVs → estimator/control  host, automatic
L2  Subsystem integration     host platform + fake devices          host, automatic
L1  Driver / unit             fake RegisterDevice, virtual clock    host, automatic
L0  Static contracts          include direction, board validation   compile + CI grep
```

L1–L3 all run under the existing `tests/run_tests.sh` and GoogleTest. No new
tooling.

## 3. The host platform (`hal/host`)

This is what unlocks everything above. Roughly 500 lines total.

| Interface | Host implementation | Capability it gives tests |
|---|---|---|
| `Clock` | `VirtualClock` | Time advances only when the test says so. Deterministic dt, no sleeps, no flake |
| `Scheduler` | `StepScheduler` | "Tasks" are callables run in priority order by `step()`. A 10-second flight runs in milliseconds |
| `Mutex` | `std::mutex` wrapper, plus a `TrackingMutex` that records the owning thread | Lets a test *assert* the single-bus-owner invariant |
| `RegisterDevice` | `FakeRegisterDevice` — a scriptable register map with injectable errors and a transaction counter | Drive an MPU-6500 driver through init, WHO_AM_I mismatch and mid-read NACK — and assert that `read()` costs zero transactions |
| `Uart` | `LoopbackUart` — inject bytes, capture writes | Feed captured CRSF frames byte-for-byte, including split frames and CRC errors |
| `PwmOut` | `RecordingPwmOut` — records every µs value with a timestamp | Assert exact servo pulses and slew-rate behaviour |
| `FileStore` | `MemoryFileStore` | Flash-telemetry rotation, purge and full-disk paths without flashing hardware |
| `KeyValueStore` | `MapKeyValueStore` | Calibration save/load, corrupt-blob handling |
| `Watchdog` | `CountingWatchdog` | Assert every long-running task actually feeds |

## 4. L0 — static contracts

Two mechanical checks, both cheap, run in CI:

**Include direction.** A script walks `src/` and enforces the table in §02 §5:
`hal/device` may not include `hal/platform`; flight code may not include
`hal/drivers` or `hal/esp32`; nothing outside `hal/` may include `Board.h` except
`ArdufliteApp.cpp`.

**No vendor types or vendor calls outside the platform implementations.** Grep
`src/hal/platform`, `src/hal/device`, `src/hal/core` **and all flight code** for
`String`, `HardwareSerial`, `Print`, `Stream`, `Servo`, `File `, `Wire`, `Serial.`,
`ledc`, `#include <Arduino.h>`. Any hit fails the build.

This is the single check that keeps G3 (portability) true over time, and it is what
makes "`ledc*()` never appears in application code" a guarantee rather than a
convention — see §03 3.3. Only `src/hal/esp32/` is exempt.

**Board validation** is `static_assert`, so it is enforced by the compiler in every
build, not just CI.

These replace most of `test_production_contracts.cpp`. The parts of that file that
pin genuine safety invariants (armed-state gates on web mutations, ground-only CLI
commands) stay until the corresponding subsystem has real tests.

## 5. L1 — driver and unit tests

New tests that were previously impossible:

| Test | What it pins |
|---|---|
| `test_mpu6500_driver` | WHO_AM_I check → `NotPresent`; range configuration writes the right registers; scaling maths (LSB → g, LSB → dps); a bus NACK mid-read returns `IoError` and leaves the sample untouched; **`sample()` issues exactly one 14-byte burst and both `read()`s hit no bus at all** (asserted by transaction count on the fake) |
| `test_sensor_threading` | The R1 contract: `sample()`/`read()` entered from a second thread trips a thread-id assertion on the fake bus. This is the test that stops the torn-read bug being reintroduced |
| `test_sample_decimation` | The R2 contract: a 25 Hz device is `sample()`d every 20th tick of a 500 Hz task, driven by `nativeRate_hz()` with no hardcoded factor; a 5 Hz device needs no new constant |
| `test_calibration_service` | State machine `Idle→Requested→Running→Complete`; accumulation happens inside the tick with no task pause; a request during flight is rejected; results round-trip through `SettingsStore` including a CRC failure |
| `test_sensor_redundancy` | Two `Accelerometer` instances in the list; `SensorSelector` picks index 0; marking it `Failed` fails over to index 1; hysteresis stops a marginal sensor flapping |
| `test_bmp280_driver` | Calibration-coefficient read; compensation formula against datasheet reference values; `nativeRate_hz()` reflects the configured oversampling |
| `test_crsf_parser` | Frame sync, length bounds, CRC8, 11-bit channel unpacking, split frames across reads, garbage resync, link-stats decode, failsafe timeout — driven by captured bytes |
| `test_axis_transform` | All 48 signed maps are orthonormal and correctly signed; `isValid()` rejects duplicate axes; `applyAngularRate()` carries the determinant while `applyMeasurement()` does not; the current aircraft map `{+X,−Y,+Z}` reproduces `applyOrientation()` exactly for accel and gyro. **The last assertion is the Phase 6 safety net** |
| `test_seqlock` | Concurrent reader/writer stress; torn reads always retry; the stale-fallback path returns coherent data; health counters are accurate |
| `test_actuator_bank` | Endpoint mapping, inversion, trim, travel limits, `OutputRange`, NaN/Inf hold-last, slew limiting against `VirtualClock`; each `FailsafeAction` on `disable()`; `commit()` propagates a transport failure as `Status`; `state()` reports `Saturated` when clipped |
| `test_actuator_disable_concurrency` | The R9 contract: `disable()` from a second thread during a `write`/`commit` loop leaves every output disabled and never interleaves destructively |
| `test_composite_actuator_bank` | Flat index space across two banks; `commit()` returns the first failure but still commits the rest; `disable()` disables all banks unconditionally even if one errors |
| `test_attitude_estimator` | Estimator contract independent of implementation: a level, static input converges to identity; a pure yaw rate integrates to the right heading; `reset()` clears state; `setGain()` changes convergence rate. Run against **both** implementations by parameterised test, so Phase 9 inherits the whole suite |
| `test_airframe_mixer` | The *real* `AirframeMixer`, replacing the copied formulas in `test_servo_math.cpp` |
| `test_motion_detector` | The *real* `MotionDetector`, replacing the harness in `test_motion_signals.cpp` |
| `test_baro_decimation` | The *real* decimation path in `InertialSubsystem`, with `nativeRate_hz()` driving the factor |
| `test_board_descriptor` | Negative tests: deliberately broken descriptors fail `validate::isValid()` (compile-time asserts can't be unit-tested, so the `constexpr` predicates are tested directly) |

Three existing test files stop being copies and start being tests.

## 6. L2 — subsystem integration

Whole subsystems assembled from host fakes:

* **Inertial subsystem.** `FakeRegisterDevice` scripted with a synthetic motion
  profile → `InertialSubsystem` → assert the published `ImuState`: rotation applied,
  offsets removed, low-pass response matches the alpha, baro decimated at the right
  cadence, climb rate correct, first sample does not spike, NaN reads do not poison
  the EMA. All of §00 2.1's tangled behaviour, verified.
* **Single-bus-owner invariant.** `TrackingMutex` asserts the sensor bus was locked
  by exactly one thread id for the whole run. This replaces the
  `expectNotContains(imuCpp, "xTaskCreate(baroTask")` grep with a real check of the
  property that actually matters.
* **Control chain.** `SimImu` → estimator → `ArduFliteController` → `AirframeMixer`
  → `RecordingPwmOut`. Assert: a level aircraft with a level setpoint produces
  neutral pulses; a 20° roll disturbance produces correctly-signed aileron
  deflection; disarm stops output; failsafe produces the configured bank and pitch.
* **RC chain.** Captured CRSF bytes → `CrsfLink` → `RcMapper` → assert the right
  commands are queued. Includes the arm/throttle-cut edge cases from
  `test_production_contracts.cpp`, as behaviour rather than as substrings.
* **Flash telemetry.** `MemoryFileStore` sized to a small disk: assert log rotation,
  auto-purge, index exhaustion, truncated-row handling, and that `stopLogging()`
  before `format()` is honoured.

## 7. L3 — flight log replay

`docs/flight_logs/FL001` and `FL002` contain real CSV telemetry from actual flights.
Once the estimator and controller are host-runnable, these become a regression
oracle:

```
tests/replay/
├── replay_main.cpp        # CSV → SimImu → estimator → controller → CSV out
└── cases/
    ├── FL002_log_007.yaml # source log, tolerances, assertions
    └── …
```

Two modes:

1. **Estimator replay.** Feed logged accel/gyro; compare the estimator's quaternion
   against the logged one. Tolerance-bounded. Any refactor that changes attitude
   output by more than the tolerance fails.
2. **Full-chain replay.** Also feed logged RC input and setpoints; compare servo
   commands. This is what makes the Phase 5 and Phase 6 refactors *provably*
   behaviour-preserving rather than "it looked fine on the bench".

This is the highest-value test in the plan and it costs one CSV reader, because the
logs already exist.

**Caveat, stated honestly:** the logged data is not a closed-loop simulation — the
aircraft's response is fixed, so replay validates the estimator and the
command-generation path, not the aerodynamics. That is exactly the scope needed to
de-risk a refactor.

## 8. L4 — hardware in the loop

Not automatable, but must be gated. Each migration phase (§06) defines its own
bench checklist. Constants across all of them:

* `stats` — inner/outer loop `avgDt`, `maxDt`, `overrunCount` compared against the
  pre-phase baseline. **Regression here blocks the phase.**
* `tasks` — stack high-water marks for every task.
* Control-surface direction and travel on every axis, in every mode.
* Failsafe: power off the transmitter, confirm the configured bank/pitch/throttle.
* Arm/disarm, throttle cut, mode switching.
* One short flight before the phase is tagged.

## 9. CI changes

`arduino_build.yaml` gains three jobs:

```yaml
host-tests:      cmake + ctest on tests/unit           # currently missing entirely
static-checks:   include-direction + no-Arduino-types + no-BOARD_TYPE greps
build-matrix:    {lolin, firebeetle} × {full, lite}    # currently one generic esp32 build
                 + flash/RAM size reported and compared against a committed budget
```

The size budget matters: §02 4.1 predicts ~2 KB of vtable growth. CI should
*measure* it rather than let the plan's estimate go unchecked.

## 10. Coverage targets

Not a percentage — a list. By the end of Phase 8, these must have real behavioural
tests: every driver in `hal/drivers`, `AirframeMixer`, `MotionDetector`,
`AttitudeEstimator` (both implementations), `InertialSubsystem`, `SensorSelector`,
`RcMapper`, `ActuatorBank`, `CompositeActuatorBank`, `SeqLock`,
`AxisTransform`, board validation predicates, and the flash log-rotation path.

Not required: the ESP32 platform implementations (thin wrappers, verified by L4),
the web server, and the CLI parser beyond what exists.
