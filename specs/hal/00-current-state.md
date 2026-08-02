# 00 — Current State Assessment

**Status:** Draft · **Date:** 2026-08-02 · **Scope:** Everything in ArduFlite that touches hardware

This document is the evidence base for the redesign. Every claim below points at a
file and line in the repository as it stands at commit `9ca8484`.

---

## 1. Where hardware is touched today

Direct use of platform APIs (`Wire`, `EEPROM`, `LittleFS`, `Serial`, `WiFi`,
`esp_task_wdt_*`, `xTaskCreate`, `micros()`, `HardwareSerial`, `ESP32Servo`,
`Adafruit_NeoPixel`) is spread across **28 source files**:

| File | What it touches directly |
|---|---|
| `src/orientation/ArduFliteIMU.cpp` | `Wire`, `EEPROM`, `FastIMU`, `Adafruit_BMP280`, `Adafruit_Madgwick`, `esp_task_wdt`, `xTaskCreate`, `micros` |
| `src/actuators/ServoManager.cpp` | `ESP32Servo`, `micros` |
| `src/receiver/crsf/ArdufliteCRSFReceiver.cpp` | `HardwareSerial`, `xTaskCreate`, `micros` |
| `src/receiver/pwm/ArduFlitePwmReceiver.cpp` | `attachInterrupt`, `pinMode`, `micros` |
| `src/telemetry/flash/ArduFliteFlashTelemetry.cpp` | `LittleFS`, `File`, `xTaskCreate` |
| `src/telemetry/crsf/ArdufliteCRSFTelemetry.cpp` | `HardwareSerial`, `xTaskCreate` |
| `src/telemetry/serial/*.cpp` | `Serial` |
| `src/controller/ArduFliteController.cpp` | `esp_task_wdt`, `xTaskCreate`, `micros` |
| `src/utils/StatusLED.cpp` | `Adafruit_NeoPixel`, `xTaskCreate` |
| `src/utils/ButtonBase.cpp` | `pinMode`, `digitalRead`, `millis` |
| `src/utils/ConfigPersistence.cpp` | `Preferences` (NVS) |
| `src/utils/Logging.cpp` | `Serial` |
| `src/web/WiFiManager.cpp`, `ArduFliteWebServer.cpp` | `WiFi`, `WebServer`, `DNSServer`, `LittleFS` |
| `ArdufliteApp.cpp` | `HardwareSerial crsfSerial(1)`, `pinMode`, `esp_task_wdt`, `esp_reset_reason`, `Serial` |

There is no layer that a port to a non-Arduino MCU could be confined to.

## 2. Concrete defects in the existing abstraction

### 2.1 `ArduFliteIMU` is nine responsibilities in one class

1345 lines (`ArduFliteIMU.cpp`) + a 609-line header. It is simultaneously:

1. the I2C bus owner (`Wire.begin`, `Wire.setClock` — `ArduFliteIMU.cpp:94`),
2. the MPU-6500/9250 driver (via FastIMU),
3. the BMP280 driver (`readBaroAltitude()`, `ArduFliteIMU.cpp:413`),
4. calibration-offset storage in raw EEPROM (`loadOffsetsFromEEPROM`, `:814`),
5. a low-pass filter bank (`applyLowPassFilters`, `:872`),
6. the Madgwick attitude estimator (`:526–550`),
7. the launch/landing motion detector (`updateMotionSignals`, `:914`),
8. a sensor-health monitor (`validateSensorData`, `:1125`),
9. a FreeRTOS task, its watchdog registration, a cooperative pause protocol, and a
   seqlock publisher (`:244–298`, `:1210–1336`).

Consequences: changing the IMU chip requires editing fusion code; changing the
fusion filter requires editing driver code; none of it can be exercised on a host.

### 2.2 Chip selection is `#ifdef` sprawl inside the class body

`IMU_TYPE` / `BARO_TYPE` / `FILTER_TYPE` / `IMU_ORIENTATION` gate code at **18
conditional blocks / 22 preprocessor branches** inside `ArduFliteIMU.{h,cpp}` alone
(`IMU_TYPE` ×5, `BARO_TYPE` ×7, `FILTER_TYPE` ×6, `IMU_ORIENTATION` ×2), including a
member declaration:

```cpp
// ArduFliteIMU.h:386
#if IMU_TYPE == IMU_TYPE_MPU9250
    MPU9250 IMU;
#else
    MPU6500 IMU;
#endif
```

Adding a third IMU means finding and editing every one of those sites. There is no
compile-time list of supported parts, and no way to have two IMUs at once.

### 2.3 Sensor mounting is hand-written sign flips with no expressible model

```cpp
// ArduFliteIMU.cpp:567 applyOrientation()
gyroX  = -gyroX;
accelY = -accelY;
gyroZ  = -gyroZ;
#if IMU_TYPE == IMU_TYPE_MPU9250
magY = -magY;
magZ = -magZ;
#endif
```

**Correction to an earlier draft of this document:** the accel and gyro transforms
here are *not* inconsistent. Written as matrices:

| Signal | Transform | Determinant |
|---|---|---|
| accel | `diag(+1, −1, +1)` | −1 |
| gyro  | `diag(−1, +1, −1)` | +1 |
| mag (MPU-9250 only) | `diag(+1, −1, −1)` | +1 |

The accel transform is a **reflection**, not a rotation (det = −1), so the relabelled
frame is left-handed. Angular rate is measured *about* an axis, so its sign depends
on the handedness of the frame — under a signed axis map `M`, gyro must transform as
`det(M)·M`. For `M = diag(+1,−1,+1)` that is `−diag(+1,−1,+1) = diag(−1,+1,−1)`,
which is **exactly** what the code does. Accel and gyro are mutually consistent.

Two real problems remain:

1. **The mag line matches neither convention.** As a per-axis measurement it should
   be `diag(+1,−1,+1)`. It is `diag(+1,−1,−1)`. This is currently dead code
   (`IMU_TYPE` is MPU-6500, which has no magnetometer, and the fusion path calls
   `filter.updateIMU()` without mag), so there is no flying behaviour to preserve —
   but it would bite the moment a 9250 or an external mag is fitted.
2. **The model is unwritten.** Nothing in the code says "this is a signed axis map
   with det = −1, and the gyro carries the determinant". A future reader — or a
   future axis change — has no rule to follow, only three sign flips that happen to
   be right. There is also no way to change it without a rebuild.

The det = −1 is itself informative: a chip cannot be *mounted* mirrored, so a
left-handed map means the part's internal axis sign convention differs from the
datasheet's — a known trait of counterfeit MPU-6500 modules. The new model should
represent this exactly and make it visible, not "fix" it.

### 2.4 Board pin tables are unvalidated, and already conflict

`include/PinConfiguration.h` uses five separate `#if BOARD_TYPE ==` blocks. For
`BOARD_TYPE_WEMOS` (Lolin C3 Mini, an ESP32-C3 with GPIO0–GPIO21 only):

| Symbol | Pin | Problem |
|---|---|---|
| `PwmInputConfig::PITCH_INPUT_PIN` | 32 | **Does not exist on ESP32-C3** (author's own `//TODO`) |
| `PwmInputConfig::THROTTLE_INPUT_PIN` | 10 | Collides with `PwmOutputConfig::THROTTLE_PIN` = 10 |
| `PwmInputConfig::ROLL_INPUT_PIN` | 6 | Collides with `CRSFPinConfig::PIN_CRSF_RX` = 6 |
| `PwmInputConfig::YAW_INPUT_PIN` | 8 | Collides with `CRSFPinConfig::PIN_CRSF_TX` = 8 |
| Status LED pin | 7 | Not in `PinConfiguration.h` at all — hardcoded `StatusLED statusLED(7, 1)` in `ArdufliteApp.cpp:128` |

These are currently *latent* (the app instantiates the CRSF receiver, not the PWM
one), but nothing detects them. `BOARD_TYPE_FIREBEETLE`'s CRSF pins are marked
`//TODO` and are simply copies of the Wemos values. A declarative board descriptor
with `static_assert` validation catches all five at compile time.

### 2.5 Hardware configuration is split across two mechanisms with no rule

`ServoManager::initFromConfig()` reads the pin from a compile-time constant and the
pulse limits from the runtime registry, in adjacent lines:

```cpp
// ServoManager.cpp:45
pitchConfig.pin      = PwmOutputConfig::PITCH_PIN;              // compile-time
pitchConfig.minPulse = config.get<int32_t>(CONFIG_KEY_SERVO_PITCH_MIN); // runtime NVS
```

AGENTS.md states the rule ("compile-time constants stay in `include/*.h`"), but the
boundary is drawn by convention only, and board identity still leaks into
application code (`#if BOARD_TYPE == BOARD_TYPE_WEMOS` in `ArdufliteApp.cpp`).

### 2.6 Two receivers, no common interface

`ArdufliteCRSFReceiver` and `ArduFlitePwmReceiver` share nothing — not a base class,
not a data type, not a failsafe concept. The application, `PreflightCheck`,
`TelemetryData::update()` and `CommandSystem::processCommands()` all name
`ArdufliteCRSFReceiver` concretely. Swapping to SBUS or PWM is a cross-cutting edit.

Worse, pilot-input *policy* lives inside the CRSF driver: `ChannelConfig` carries
`ChannelCallback` function pointers (`ArdufliteCRSFReceiver.h:78`) so the driver
directly invokes `CRSFCallbacks::onArm`. A protocol decoder should not know what
channel 5 means.

### 2.7 Actuator output conflates four concerns

`ServoManager` owns: airframe mixing (delta/V-tail/conventional), per-surface
endpoint calibration, slew-rate limiting, and the `ESP32Servo` PWM driver — with a
hardcoded set of exactly six `Servo` members (`ServoManager.h:129`). Flaps, a second
motor, or a 6-surface airframe cannot be expressed without editing the class.

### 2.8 The layer boundary is inverted for `FlightState`

`ImuSnapshot` carries a `FlightState` field that the IMU does not own:
StateManagement computes it and writes it *back* into the IMU via
`setFlightState()` (`ArduFliteIMU.cpp:986`) purely so telemetry can read everything
from one struct. A sensor object holds application state.

### 2.9 Two persistence mechanisms

IMU calibration offsets use raw EEPROM emulation at address 0 with a magic number
(`ArduFliteIMU.h:177–187`); everything else uses NVS via `ConfigPersistence`. Two
storage layers, two failure modes, one of which has no versioning.

### 2.9b The primary board has no FPU, and the docs say it does

Confirmed empirically from `build/lolin-full/ArduFlite.ino.map`: the link line is
`riscv32-esp-elf/.../rv32imc_zicsr_zifencei/ilp32/...` and the binary pulls in
`__addsf3`, `__subsf3`, `__mulsf3`, `__divsf3` (plus the `df3` double variants).

`rv32imc` has no `F` extension and `ilp32` is the soft-float ABI. **Every floating
point operation on the Lolin C3 Mini is emulated in software.** The classic ESP32
and the S3 do have a single-precision FPU; the C3 — the board that flies — does not.

Two places in the repo state otherwise:

* `AGENTS.md`, "Performance Considerations" §3: *"ESP32 has hardware FPU for
  `float`, not `double`"* — true for the ESP32/S3 targets, false for the C3.
* `ArduFliteIMU.cpp:405`: *"The ESP32 has a hardware float FPU but emulates double
  in software, so powf is markedly cheaper"* — the **conclusion is still correct**
  (single-precision soft-float is roughly 2× cheaper than double soft-float, so
  `powf` remains the right call), but the stated reason is wrong.

This does not change any current behaviour, and it does not affect the
virtual-dispatch estimate in §02 4.1 (call overhead is integer work). It matters for
two other reasons: it should be corrected before someone optimises against a false
model, and it means any judgement about "library overhead" has to be made in float
operations, not lines of code.

### 2.10 Testing is blocked, and the tests admit it

`tests/unit/CMakeLists.txt` compiles exactly **two** production files: `pid.cpp` and
`FliteQuaternion.cpp`. Everything else is host-hostile. The suite works around this
in two ways, both explicitly documented in the test files:

* **Logic is copy-pasted into the tests.** `test_servo_math.cpp:11` —
  *"Rather than mocking those, the formulas are reproduced verbatim here… Any change
  to the production mixing logic must be mirrored in this file."* Same pattern in
  `test_motion_signals.cpp` and `test_baro_decimation.cpp` ("Keep these constants in
  sync with ArduFliteIMU.h"). These tests cannot fail when production code breaks.
* **Behaviour is asserted by grepping source text.**
  `test_production_contracts.cpp` reads `.cpp` files and asserts on substrings such
  as `expectContains(imuCpp, "snapshotVersion.store(version + 1")`. This is a
  reasonable stopgap for genuinely unreachable code, but it is currently doing the
  job of real tests for the flash, web, CLI and IMU subsystems.

CI (`.github/workflows/arduino_build.yaml`) only compiles the sketch — the host test
suite is not run at all.

### 2.11 Task and timing policy is prose-only

AGENTS.md documents the priority ladder ("IMU Task (highest) → Inner Loop → Outer
Loop → Telemetry → CLI"), but each subsystem calls `xTaskCreate` with its own magic
numbers (`xTaskCreate(imuTask, "IMU Task", 4096, this, 4, …)`). Nothing enforces the
ladder, and stack sizes are unaudited.

### 2.12 Failures are logged, not returned

`begin()` methods return `bool`; the reason is only in the log. The application's
response to IMU failure is `while (1);` (`ArdufliteApp.cpp:233`), guarded by a
watchdog-recovery special case. There is no error type, so callers cannot
distinguish "not fitted" from "bus error" from "bad calibration".

---

## 3. What is already good and must be preserved

The redesign is not a rewrite of these — it should carry them forward, cleaned up:

* **The seqlock snapshot** (`ArduFliteIMU.cpp:1210–1336`) — versioned lock-free
  publish with a stale-but-coherent fallback and retry telemetry. This is a genuinely
  good pattern. It becomes a reusable `arduflite::util::SeqLock<T>`.
* **Single I2C bus owner.** The IMU task owns the bus; the baro is decimated inside
  it. This was arrived at the hard way (a separate baro task caused priority
  inversion and "sensor mutex busy" skips) and is pinned by
  `test_production_contracts.cpp`. The new design must make this structural rather
  than a rule people have to remember.
* **`ConfigRegistry`** — typed, validated, observable, NVS-backed, with hot reload.
  It stays as-is; the HAL consumes it, it does not replace it.
* **`ArduFliteTelemetry`** — already a clean pluggable interface. It is the model to
  follow, not to replace.
* **Deferred `initFromConfig()`** — the right answer to "globals constructed before
  FreeRTOS exists". The composition root formalises it.
* **Watchdog + fast-boot-to-MANUAL on WDT reset** — good safety behaviour worth
  keeping verbatim.
* **The flight log corpus** (`docs/flight_logs/FL001`, `FL002`) — real CSV telemetry
  that can be replayed as a regression oracle once the estimator is testable.

---

## 4. Summary

| Requirement | Today |
|---|---|
| Swap IMU chip | Edit 5 `#if IMU_TYPE` sites inside a 1345-line class, one of them a member declaration |
| Swap barometer | Edit 7 `#if BARO_TYPE` sites, one of them a member declaration |
| Swap fusion filter | Edit the IMU driver |
| Swap RC protocol | Cross-cutting edit across app, preflight, telemetry, commands |
| Add a board | Edit 5 `#if BOARD_TYPE` blocks + app code; no validation |
| Port to a non-Arduino MCU | Not feasible — 28 files touch the platform directly |
| Test a driver on a host | Not possible; tests copy the logic instead |
| Add a 5th control surface | Edit `ServoManager` |

That table is the specification for what §02–§04 must fix.
