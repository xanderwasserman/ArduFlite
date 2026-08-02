# 10 — Architecture Diagrams

**Status:** Draft, revision 2 (conservative Mermaid) · **Date:** 2026-08-02

Visual companion to §02 (architecture), §03 (interfaces) and §06 (migration plan).

> **Revision 2:** rewritten in a deliberately conservative Mermaid subset after the
> first version failed to render. See [Troubleshooting](#troubleshooting) at the end
> for what was actually wrong and how to verify these.

| # | Diagram | Answers |
|---|---|---|
| 1 | [Tier dependency](#1-tier-dependency) | What depends on what, and in which direction |
| 2 | [Sensor class model](#2-sensor-class-model) | How one chip provides several measurements (ADR-019) |
| 3 | [Why sample and read are separate](#3-why-sample-and-read-are-separate) | The cost that makes ADR-019 affordable |
| 4 | [Actuator class model](#4-actuator-class-model) | Actuator vs ActuatorBank; how PWM and CANopen mix (ADR-020, ADR-025) |
| 5 | [Inertial tick sequence](#5-inertial-tick-sequence) | One 500 Hz cycle, and where the bus is touched |
| 6 | [Control output path](#6-control-output-path) | Where ledc lives, and where it does not |
| 7 | [Calibration state machine](#7-calibration-state-machine) | Calibration without a task-pause protocol |
| 8 | [Board composition](#8-board-composition) | Who constructs what, once, at boot |
| 9 | [The decomposition](#9-what-happens-to-ardufliteimu) | What ArduFliteIMU becomes |
| 10 | [Phase dependencies](#10-phase-dependencies) | What can run in parallel, and what gates what |

---

## 1. Tier dependency

The single most important property: **arrows only point downward.** A tier may use
the tier below it and never the one above.

```mermaid
flowchart TD
    APP["APPLICATION<br/>ArdufliteApp, StateManagement<br/>MissionPlanner, CLI, Web"]
    FLIGHT["FLIGHT LOGIC<br/>ArduFliteController, AirframeMixer, RcMapper<br/>InertialSubsystem, AttitudeEstimator, Telemetry"]
    T2["TIER 2 - arduflite::device<br/>one interface per MEASUREMENT<br/>Accelerometer, Gyroscope, Magnetometer, Barometer<br/>Gnss, PowerMonitor, RcLink, ActuatorBank<br/>LogStore, SettingsStore, Console"]
    T1["TIER 1 - arduflite::drivers - which part<br/>Mpu6500, Mpu9250, Bmp280<br/>CrsfLink, PwmActuatorBank, LittleFsLogStore"]
    T0["TIER 0 - arduflite::hal - which MCU<br/>RegisterDevice, I2cBus, SpiBus, CanBus<br/>Uart, GpioPin, PwmOut<br/>Clock, Scheduler, Mutex, Watchdog"]
    IMPL["TIER 0 IMPLEMENTATIONS<br/>platform/esp32 - Arduino, FreeRTOS, ESP-IDF<br/>platform/host - std C++, for tests"]
    BOARD["board::Board - composition root<br/>constinit, no heap after begin"]
    DESC["constexpr BoardDescriptor<br/>pins, buses, fitted parts, axis maps<br/>validated by static_assert"]

    APP --> FLIGHT
    FLIGHT --> T2
    T2 -.->|implemented by| T1
    T1 --> T0
    T0 -.->|implemented by| IMPL
    DESC --> BOARD
    BOARD -.->|constructs| T1
    BOARD -.->|constructs| IMPL
    BOARD -.->|hands references to| FLIGHT
```

**The two rules that make this work:**

1. Flight code names only `arduflite::device` types. It never sees a bus, a pin or a
   driver.
2. Only `Board.cpp` names a concrete driver. Everywhere else holds interface
   references.

Both are checked mechanically by the include-direction job in §05 §4 — not left to
discipline.

---

## 2. Sensor class model

The part that differs most from the first draft. **A chip is not an interface.** A
6-DOF IMU implements `Accelerometer` *and* `Gyroscope`; discrete parts implement one
each; flight code cannot tell the difference.

```mermaid
classDiagram
    class Sensor {
        <<interface>>
        +probe() Status
        +begin() Status
        +sample() Status
        +nativeRate_hz() uint16
        +health() SensorHealth
    }

    class Accelerometer {
        <<interface>>
        +read(AccelSample out) Status
        +setRange_g(uint8) Status
    }

    class Gyroscope {
        <<interface>>
        +read(GyroSample out) Status
        +setRange_dps(uint16) Status
    }

    class Magnetometer {
        <<interface>>
        +read(MagSample out) Status
    }

    class Barometer {
        <<interface>>
        +read(BaroSample out) Status
    }

    class RegisterDevice {
        <<interface>>
        +readRegs(reg, dst, len) Status
        +writeReg(reg, val) Status
        +busLock() Mutex
    }

    class Mpu6500 {
        -RegisterDevice bus
        -AccelSample accelCache
        -GyroSample gyroCache
        +sample() Status
    }

    class Mpu9250
    class Bmp280
    class Adxl345
    class L3gd20

    Sensor <|.. Mpu6500
    Accelerometer <|.. Mpu6500
    Gyroscope <|.. Mpu6500
    Sensor <|.. Mpu9250
    Accelerometer <|.. Mpu9250
    Gyroscope <|.. Mpu9250
    Magnetometer <|.. Mpu9250
    Sensor <|.. Bmp280
    Barometer <|.. Bmp280
    Sensor <|.. Adxl345
    Accelerometer <|.. Adxl345
    Sensor <|.. L3gd20
    Gyroscope <|.. L3gd20
    Mpu6500 --> RegisterDevice : uses
```

Reading the diagram:

* **`sample()` is the only method that touches the bus.** Every `read()` is `const`
  and returns the cache filled by the last `sample()`.
* **`Mpu6500` realises three interfaces.** One 14-byte burst read from
  `ACCEL_XOUT_H` refreshes both caches.
* **`Adxl345` and `L3gd20` are shown deliberately.** Custom hardware with discrete
  parts produces exactly the same view of the world as one combined chip.
* **Redundancy is the same idea again** — two `Mpu6500` entries at 0x68 and 0x69
  mean `accelerometers()` returns two pointers.

---

## 3. Why sample and read are separate

```mermaid
flowchart TD
    subgraph NAIVE["NAIVE - read hits the bus"]
        B1["accel.read()"] --> B2["I2C transaction 1"]
        B3["gyro.read()"] --> B4["I2C transaction 2"]
        B4 --> B5["2 transactions per tick<br/>1000 per second at 500 Hz"]
    end

    subgraph ACTUAL["ACTUAL - sample then read"]
        G1["dev.sample()"] --> G2["ONE 14-byte burst"]
        G2 --> G3["accel cache and gyro cache"]
        G4["accel.read()"] --> G3
        G5["gyro.read()"] --> G3
        G3 --> G6["1 transaction per tick<br/>AND per-measurement interfaces"]
    end
```

This is what makes ADR-019 affordable. Without the split, per-measurement interfaces
would double I2C traffic on a 400 kHz bus at 500 Hz.

---

## 4. Actuator class model

`ActuatorBank` names no transport. Everything PWM-specific lives in the driver's
constructor, fed by the board descriptor.

```mermaid
classDiagram
    class Actuator {
        <<interface>>
        +stage(normalised) void
        +lastCommand() float
        +state() ActuatorState
        +kind() ActuatorKind
        +role() string
        +hasFeedback() bool
        +readFeedback(out) Status
    }

    class ActuatorBank {
        <<interface>>
        +begin(cfgs) Status
        +actuators() span
        +byRole(role) Actuator
        +commit() CommitResult
        +disable() Status
        +nativeRate_hz() uint16
        +transport() string
    }

    class ActuatorChannelConfig {
        +role string
        +kind ActuatorKind
        +range OutputRange
        +invert bool
        +trim float
        +maxSlew_perSec float
        +onDisable FailsafeAction
    }

    class PwmActuator
    class CanOpenActuator
    class PwmActuatorBank
    class CanOpenActuatorBank
    class CompositeActuatorBank

    Actuator <|.. PwmActuator
    Actuator <|.. CanOpenActuator
    ActuatorBank <|.. PwmActuatorBank
    ActuatorBank <|.. CanOpenActuatorBank
    ActuatorBank <|.. CompositeActuatorBank

    ActuatorBank o-- Actuator : owns
    PwmActuatorBank *-- PwmActuator
    CanOpenActuatorBank *-- CanOpenActuator
    CompositeActuatorBank o-- ActuatorBank : aggregates
    Actuator ..> ActuatorChannelConfig : configured by
```

Reading the diagram:

* **Two levels.** `Actuator` is one output — what flight code holds, by name.
  `ActuatorBank` is one *transport's* outputs — where `commit()` and `disable()`
  live, because those are batched **bus** operations (ADR-025).
* **`ActuatorChannelConfig` is transport-neutral** — no microseconds, no node IDs.
  That was the bug in the first draft (ADR-020).
* **`write()` stages, `commit()` pushes.** For CAN that is one PDO group plus SYNC,
  atomically. `commit()` returns `Status`; `write()` cannot fail.
* **`disable()` is the only method safe from any task** — the failsafe path needs it,
  and it disables every bank unconditionally.
* **`commit()` returns `CommitResult`**, whose `staleMask` names the channels that
  failed to update. "Something failed" is not actionable in flight; "the elevator is
  stale" is.
* **`ActuatorKind`** (Proportional / Binary / Latching) is *orthogonal* to transport
  — a binary retract can hang off PWM or CAN. It lives in the channel config, not in
  a separate interface (ADR-024).
* **`CompositeActuatorBank`** flattens several banks into one index space, so four
  PWM surfaces plus one CAN throttle plus a retract needs no special case — but
  `commit()` is atomic only *within* a bank. Keep primary flight surfaces on one.

---

## 5. Inertial tick sequence

The 12-step ordering contract from §03 3.8, as it runs once per 2 ms.

```mermaid
sequenceDiagram
    autonumber
    participant TASK as InertialSubsystem
    participant WDT as WatchdogGuard
    participant IMU as Mpu6500
    participant BAR as Bmp280
    participant BUS as I2cBus
    participant EST as AttitudeEstimator
    participant SL as SeqLock
    participant RDR as Control loops

    TASK->>WDT: feed()
    TASK->>IMU: sample()
    IMU->>BUS: 14-byte burst from ACCEL_XOUT_H
    BUS-->>IMU: accel, temp, gyro

    alt every 20th tick
        TASK->>BAR: sample()
        BAR->>BUS: read pressure
        BUS-->>BAR: raw pressure
    else other 19 ticks
        TASK-->>TASK: skip, decimation from nativeRate_hz
    end

    TASK->>IMU: read AccelSample - cached, no bus
    TASK->>IMU: read GyroSample - cached, no bus
    TASK->>TASK: AxisTransform applyMeasurement and applyAngularRate
    TASK->>TASK: subtract calibration offsets
    TASK->>TASK: low-pass filter bank
    TASK->>TASK: validate NaN, Inf, range then update health

    alt on baro ticks
        TASK->>TASK: altitude EMA and climb-rate derivative
    end

    TASK->>EST: update gyro_dps, accel_g, dt_s
    EST-->>TASK: quaternion and euler
    TASK->>TASK: motionDetector update
    TASK->>TASK: calibrationService serviceTick
    TASK->>SL: publish ImuState - one seqlock write, last

    Note over SL,RDR: Everyone else reads here. Never a driver.
    RDR->>SL: read or try_read
    SL-->>RDR: coherent ImuState copy
```

Two contracts are visible here:

* **Bus access is confined to the early steps.** `read()` never touches it
  (review R1).
* **The seqlock publish is last and single.** Every other task reads `ImuState`,
  never a driver — which is what makes the unsynchronised driver cache safe.

---

## 6. Control output path

Answering "where does `ledc*()` live?" precisely.

```mermaid
flowchart TD
    C["ArduFliteController<br/>inner loop 500 Hz"]
    M["actuators::AirframeMixer<br/>pure function, no state, no hardware"]
    AB["device::ActuatorBank - INTERFACE<br/>FLIGHT CODE STOPS HERE"]
    PB["drivers::PwmActuatorBank<br/>endpoints, invert, trim, slew, disable"]
    PO["hal::PwmOut - INTERFACE<br/>microseconds only"]
    EP["hal::esp32::Esp32PwmOut<br/>ledcAttach and ledcWrite live HERE<br/>about 40 lines, the only file"]
    HW["Servo or ESC"]
    FS["Failsafe, CommandSystem, CLI"]

    C -->|normalised commands| M
    M -->|float outputs by role| AB
    AB -.->|implemented by| PB
    PB -->|writeMicroseconds 1500| PO
    PO -.->|implemented by| EP
    EP --> HW
    FS -->|disable, safe from any task| AB
```

`ledcAttach` / `ledcWrite` appear in **one file, roughly 40 lines**. Application code
calls `outputs.write(kElevator, -0.3f)` and never sees `PwmOut`, let alone `ledc`.
The static-check job greps flight code for `ledc`, `Wire`, `Serial` and `Servo` and
fails the build on a hit — so the property cannot decay.

---

## 7. Calibration state machine

Replaces the current `pauseTask()` / `_taskPaused` cooperative spin-wait, which
exists only because the IMU task owns the bus that calibration needs.

```mermaid
stateDiagram-v2
    [*] --> Idle
    Idle --> Requested : request from CLI, web or button
    Requested --> Failed : rejected, aircraft armed or in flight
    Requested --> Running : accepted on next tick
    Running --> Running : accumulate one sample per tick
    Running --> Complete : sample target reached
    Running --> Failed : sensor unhealthy or timeout
    Complete --> Idle : offsets written to SettingsStore
    Failed --> Idle : lastError readable
```

* `request()` is thread-safe — it sets an atomic flag. **Step 11 of the tick does the
  work**, inside the task that already owns the bus.
* No pause, no spin-wait, no deadlock to avoid.
* A 10-second gyro calibration is 5000 ticks of accumulation, not a blocking loop, so
  control loops keep running on the last good `ImuState` and the watchdog keeps being
  fed by the normal path.

---

## 8. Board composition

Everything is constructed once, at boot, in static storage.

```mermaid
flowchart TD
    DESC["constexpr BoardDescriptor kBoard<br/>lives in flash, consteval-validated"]
    S1["1. Tier 0 - buses, UARTs, PWM pins, NVS, FS"]
    S2["2. loop over descriptor sensors"]
    S3["3. construct chip, bind to each<br/>measurement interface it provides"]
    S4["4. probe - on failure log, unbind, CONTINUE"]
    S5["5. RC link, actuator banks, LED"]
    S6["6. logInventory - the boot table"]
    STORE["BoardStorage in Board_Internal.h<br/>seen only by Board.cpp<br/>optional Mpu6500, optional Bmp280<br/>Esp32I2cBus, Esp32Uart, Esp32PwmOut"]
    LISTS["Interface lists as std span<br/>sensorDevices, accelerometers<br/>gyroscopes, barometers"]
    APP["arduflite_init - the ONLY caller of Board"]
    SUBS["InertialSubsystem, Controller<br/>RcMapper, Telemetry"]

    DESC --> S1
    S1 --> S2
    S2 --> S3
    S3 --> S4
    S4 --> S5
    S5 --> S6
    S6 --> STORE
    STORE --> LISTS
    LISTS --> APP
    APP -->|injects references into constructors| SUBS
```

Three properties worth noting:

* **`Board.h` never names a driver type.** `BoardStorage` lives in
  `Board_Internal.h`, included only by `Board.cpp` (review R3), so the layering check
  is real rather than theatre.
* **Each sensor fails independently.** A dead barometer removes one entry from one
  list; the gyro keeps flying the aircraft.
* **Nothing else calls `Board`.** `arduflite_init()` pulls references out and injects
  them. That is the difference from AP_HAL's global `hal` object.

---

## 9. What happens to ArduFliteIMU

The payoff. 1345 lines doing nine jobs becomes seven independently testable
components.

```mermaid
flowchart LR
    BEFORE["BEFORE - ArduFliteIMU, 1345 lines<br/>I2C bus owner<br/>MPU driver via FastIMU<br/>BMP280 driver<br/>EEPROM calibration store<br/>low-pass filter bank<br/>Madgwick fusion<br/>launch and landing detection<br/>health monitoring<br/>FreeRTOS task, WDT, pause protocol<br/>seqlock publisher"]
    A1["hal::Esp32I2cBus<br/>owns the bus and mutex"]
    A2["drivers::Mpu6500<br/>drivers::Bmp280"]
    A3["estimation::InertialSubsystem<br/>task, transform, filter, decimate, publish"]
    A4["estimation::AttitudeEstimator"]
    A5["estimation::MotionDetector"]
    A6["estimation::CalibrationService"]
    A7["device::SettingsStore<br/>NVS plus CRC"]
    A8["core::SeqLock of ImuState"]

    BEFORE ==>|Phases 5 and 6| A1
    A1 --> A2
    A2 --> A3
    A3 --> A4
    A3 --> A5
    A3 --> A6
    A6 --> A7
    A3 --> A8
```

Every box on the right is independently host-testable. On the left, none of it is —
which is why `test_motion_signals.cpp` and `test_baro_decimation.cpp` currently
contain hand-maintained *copies* of the logic rather than tests of it.

---

## 10. Phase dependencies

```mermaid
flowchart LR
    P0["P0 Foundations<br/>host only<br/>RISK none"]
    P1["P1 Config hygiene<br/>key rename<br/>RISK medium"]
    P2["P2 Platform layer<br/>Tier 0 plus Board<br/>GATE vtable cost<br/>RISK medium"]
    P3["P3 Actuators<br/>vertical slice 1<br/>RISK HIGH"]
    P4["P4 RC input<br/>vertical slice 2<br/>RISK medium"]
    P5["P5 Sensor drivers<br/>GATE cross-check<br/>RISK HIGH"]
    P6["P6 Estimation<br/>GATE log replay<br/>RISK HIGH"]
    P7["P7 Peripherals<br/>RISK none"]
    P8["P8 Cleanup and proof<br/>RISK none"]
    P9["P9 Own Madgwick<br/>optional<br/>RISK HIGH"]

    P0 --> P1
    P0 --> P2
    P2 --> P3
    P2 --> P4
    P2 --> P5
    P2 --> P7
    P5 --> P6
    P3 --> P8
    P4 --> P8
    P6 --> P8
    P7 --> P8
    P8 --> P9
```

**P3, P4, P5 and P7 are mutually independent** once P2 lands, so they can be
reordered around flying weather and bench availability. The one hard constraint is
**P5 then P6**: new drivers and the new estimation layer are deliberately kept in
separate branches with separate oracles, so a bad flight afterwards is attributable
to one or the other.

Actuators and RC come *before* the IMU on purpose: they are complete vertical slices
through all four tiers, bench-testable in minutes. If the tier model is wrong, that
becomes obvious there rather than inside the IMU rewrite.

---

## Troubleshooting

### What was actually wrong in revision 1

There is no Mermaid runtime on this machine, so revision 1 was written without ever
being parsed. An audit against the syntax rules found genuine bugs, not just
rendering trouble:

| Problem | Where | Why it breaks |
|---|---|---|
| `stroke-dasharray: 5 5` | diagram 5 | **Invalid.** Space after the colon and inside the value; the style parser expects `key:value` with no spaces |
| `[` and `]` inside edge labels | diagram 5 | `-->\|"roll, pitch, yaw ∈ [-1,1]"\|` — brackets in edge labels break the parser even when quoted |
| `direction LR` inside `classDiagram` | diagrams 2, 4 | Requires Mermaid **v10+**; older renderers error out |
| `note for X "..."` in `classDiagram` | 5 places | Requires Mermaid v10+ |
| `rect rgba(...)` in `sequenceDiagram` | diagram 5 | Widely supported, not universal |
| 26 `style` directives using 8-digit hex (`#1f6feb22`) | throughout | `#RRGGBBAA` is not accepted by every renderer |
| Unicode `≪ ≫ ∈ ⟵ ·` in labels | throughout | Renderer- and font-dependent |

Revision 2 removes **all** of them. What remains is the subset that has been stable
for years: `flowchart`, `classDiagram` with `<<interface>>` and `<|..`,
`sequenceDiagram` with `alt`/`else`, and `stateDiagram-v2`. Colour coding is gone —
risk levels are written into the node text instead. Once these render, styling can be
added back deliberately, one diagram at a time.

### If they still show as empty blocks

Then it is the viewer, not the syntax — and the two causes look different:

* **Empty block** — the viewer recognises ```` ```mermaid ```` and swallows the
  content, but has no Mermaid engine to render it.
* **Raw source shown as a code block** — the viewer has no Mermaid support at all.
* **Red error box** — Mermaid ran and the syntax is wrong.

To settle it in about thirty seconds, paste one diagram's body into
<https://mermaid.live>. It is the reference implementation and reports parse errors
with a line number.

Known-good viewers:

* **GitHub** — renders natively in Markdown; syntax errors appear as a red box.
* **VS Code** — the built-in preview does **not** support Mermaid. Install
  *Markdown Preview Mermaid Support* (`bierner.markdown-mermaid`).
* **Obsidian, GitLab, Typora** — native support.

If a specific diagram fails at mermaid.live, send me the error — it names the
offending line and I will fix that diagram.

### Keeping these honest

* They document the **target** design, not what exists today. §00 describes the
  current state.
* Where a diagram and the prose disagree, §02–§04 win and the diagram is a bug.
* Phase 0 will force changes to §03 as the interfaces meet a compiler for the first
  time. Diagrams 2, 4 and 5 encode interface shape — update them when that happens.
