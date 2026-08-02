# 02 — Architecture

**Status:** Draft · **Date:** 2026-08-02

---

## 1. The layer stack

```
┌──────────────────────────────────────────────────────────────────────┐
│  APPLICATION            ArdufliteApp · StateManagement · MissionPlanner
│                         CLI · Web                                     │
├──────────────────────────────────────────────────────────────────────┤
│  FLIGHT                 ArduFliteController (cascade PID)             │
│                         AirframeMixer · RcMapper · ControlMixer       │
│                         AttitudeEstimator · MotionDetector            │
│                         Telemetry backends                            │
│                                                                       │
│                         ── sees ONLY arduflite::device interfaces ──           │
├──────────────────────────────────────────────────────────────────────┤
│  TIER 2  arduflite::device       Accelerometer  Gyroscope  Magnetometer        │
│  one interface per      Barometer  Gnss  Airspeed  PowerMonitor       │
│  MEASUREMENT, not       RcLink  ActuatorBank  Indicator               │
│  per chip               LogStore  SettingsStore  Console              │
├──────────────────────────────────────────────────────────────────────┤
│  TIER 1  arduflite::drivers       Mpu6500  Mpu9250  Bmp280 …                    │
│  concrete drivers       CrsfLink  PwmRcLink                           │
│  "which part"           PwmActuatorBank  NeoPixelIndicator            │
│                         LittleFsLogStore  NvsSettingsStore            │
│                                                                       │
│                         ── sees ONLY arduflite::hal interfaces ──           │
├──────────────────────────────────────────────────────────────────────┤
│  TIER 0  arduflite::hal       RegisterDevice  I2cBus  SpiBus  Uart          │
│  platform primitives    GpioPin  PwmOut  Clock  Scheduler  Mutex      │
│  "which MCU"            Watchdog  KeyValueStore  FileStore  System    │
├──────────────────────────────────────────────────────────────────────┤
│  TIER 0 IMPL            platform/esp32/   platform/host/              │
│                         (Arduino-ESP32)   (std C++, for tests)        │
└──────────────────────────────────────────────────────────────────────┘

        ┌───────────────────────────────────────────────────────────┐
        │  BOARD   board/boards/<name>.h  →  constexpr descriptor    │
        │          board/Board.{h,cpp}    →  composition root        │
        │  Constructs every Tier-0 impl and Tier-1 driver once, in   │
        │  static storage, and hands out Tier-2 references.          │
        └───────────────────────────────────────────────────────────┘
```

**The two rules that make this work:**

1. Dependencies point strictly downward. Tier 1 never includes a Tier 2 header;
   Tier 0 never includes anything from ArduFlite except `hal/core`.
2. Only `board/Board.cpp` names a concrete driver type. Everywhere else,
   flight code holds `device::` references.

Both are mechanically checkable — see §05 §4.

## 2. Directory layout

```
src/hal/
├── core/                       # dependency-free vocabulary types (arduflite::)
│   ├── Status.h                #   Status enum, to_string
│   ├── Result.h                #   Result<T>, ARDUFLITE_TRY
│   ├── Vec3.h                  #   Vec3f + basic ops
│   ├── AxisTransform.h         #   signed axis maps, alignment trim, Rotation names
│   ├── SeqLock.h               #   lock-free versioned snapshot (from ArduFliteIMU)
│   └── Units.h                 #   named conversions, field-suffix convention
│
├── platform/                   # TIER 0 interfaces (arduflite::hal)
│   ├── Clock.h  Scheduler.h  Mutex.h  Watchdog.h  System.h
│   ├── RegisterDevice.h  I2cBus.h  SpiBus.h  CanBus.h
│   ├── Uart.h  GpioPin.h  PwmOut.h
│   └── KeyValueStore.h  FileStore.h
│
├── esp32/                      # TIER 0 impl — Arduino-ESP32 (arduflite::hal::esp32)
│   ├── Esp32Clock.*  Esp32Scheduler.*  Esp32Mutex.*  Esp32Watchdog.*
│   ├── Esp32I2cBus.*  Esp32Uart.*  Esp32Gpio.*  Esp32PwmOut.*
│   └── Esp32Nvs.*  Esp32LittleFs.*  Esp32System.*
│
├── host/                       # TIER 0 impl — host/sim (arduflite::hal::host)
│   ├── VirtualClock.*          #   time you advance by hand
│   ├── StepScheduler.*         #   deterministic cooperative task runner
│   ├── FakeRegisterDevice.*    #   scriptable register map
│   ├── LoopbackUart.*  RecordingPwmOut.*  MemoryFileStore.*  MapKeyValueStore.*
│
├── device/                     # TIER 2 interfaces (arduflite::device)
│   ├── Sensor.h                # probe/begin/sample/health — a chip
│   ├── Accelerometer.h  Gyroscope.h  Magnetometer.h  Barometer.h
│   ├── Gnss.h  Airspeed.h  RangeFinder.h  PowerMonitor.h  Thermometer.h
│   ├── RcLink.h  ActuatorBank.h  Indicator.h
│   └── LogStore.h  SettingsStore.h  Console.h
│
├── drivers/                    # TIER 1 concrete drivers (arduflite::drivers)
│   ├── imu/Mpu6500.*  imu/Mpu9250.*  imu/SimImu.*
│   ├── baro/Bmp280.*  baro/SimBaro.*
│   ├── rc/CrsfLink.*  rc/PwmRcLink.*  rc/SimRcLink.*
│   ├── out/PwmActuatorBank.*
│   ├── led/NeoPixelIndicator.*
│   └── store/LittleFsLogStore.*  store/NvsSettingsStore.*
│
└── board/
    ├── BoardDescriptor.h       # the constexpr schema
    ├── BoardValidate.h         # static_assert rules
    ├── BoardSelect.h           # THE one #if in the codebase
    ├── boards/lolin_c3_mini.h
    ├── boards/firebeetle_esp32e.h
    ├── boards/host_sim.h
    └── Board.{h,cpp}           # composition root
```

New flight-side homes for logic moving out of the HAL:

```
src/estimation/                 # split out of ArduFliteIMU
├── InertialSubsystem.*         #   the 500 Hz sampling task + SeqLock<ImuState>
├── AttitudeEstimator.h         #   interface
├── MadgwickEstimator.*         #   current behaviour, now testable
├── MotionDetector.*            #   launch / stable signals
└── SensorCalibration.*         #   offsets + SettingsStore persistence

src/actuators/
├── AirframeMixer.*             # CONVENTIONAL / DELTA_WING / V_TAIL  (was in ServoManager)
└── (ServoManager.* deleted at the end of Phase 3)

src/input/
└── RcMapper.*                  # channel→role mapping, tri-state, callbacks
                                # (was ChannelConfig inside the CRSF driver)
```

## 3. Data flow after the change

```
   Board::begin()
        │  constructs, probes, and wires everything once
        ▼
 ┌────────────────┐   RegisterDevice   ┌────────────────────────────┐
 │ Esp32I2cBus    │◄───────────────────│ drivers::Mpu6500           │
 │  (owns mutex)  │                    │  : Sensor            │
 │                │                    │  , Accelerometer           │──┐
 │                │                    │  , Gyroscope               │  │
 │                │◄───────────────────│ drivers::Bmp280 : Barometer│──┤
 └────────────────┘                    └────────────────────────────┘  │
                                        sample() once → read() is free │
                                                          ▼
                                  ┌──────────────────────────────────┐
                                  │ estimation::InertialSubsystem    │
                                  │  · one task, sole bus owner      │
                                  │  · rotate() to body frame        │
                                  │  · calibration offsets           │
                                  │  · low-pass bank                 │
                                  │  · baro decimation (from         │
                                  │    Barometer::nativeRate_hz)    │
                                  │  · AttitudeEstimator             │
                                  │  · MotionDetector                │
                                  │  └─ publishes SeqLock<ImuState>  │
                                  └──────────────┬───────────────────┘
                                                 │ lock-free reads
        ┌────────────────────────────────────────┼───────────────────────┐
        ▼                                        ▼                       ▼
 ArduFliteController                     StateManagement            Telemetry
   (outer 100 Hz / inner 500 Hz)          (FlightState)              backends
        │                                                                 │
        │ roll/pitch/yaw/throttle in [-1,1]                               │
        ▼                                                                 ▼
 actuators::AirframeMixer  ──►  device::ActuatorBank  ──►  hal::PwmOut[]   device::LogStore
        ▲                        (slew, endpoints,                      device::Console
        │                          inversion, disable)                  hal::Uart
 input::RcMapper  ◄──  device::RcLink  ◄──  hal::Uart   (drivers::CrsfLink)
```

Two layering fixes are visible in that diagram:

* `FlightState` no longer round-trips through the sensor object. `ImuState` carries
  sensor data only; StateManagement owns `FlightState` and telemetry reads it from
  StateManagement. (§00 2.8)
* The baro decimation factor is derived from `Sensor::nativeRate_hz()`
  instead of a `#define` mirrored into a test file. (§00 2.10)

## 4. Key structural decisions

### 4.1 Virtual dispatch, everywhere above the bus

All Tier 0 and Tier 2 interfaces are pure-virtual classes with a virtual destructor.

*Why it's affordable:* the hottest path is the 500 Hz inner loop and the 500 Hz IMU
task. Per tick that is roughly: 2 register reads (each 1 vcall), 1 estimator update,
1 seqlock publish, 3 PID updates, 1 mixer call, 4 PWM writes ≈ **~20 indirect calls
per tick**. At 500 Hz that is 10 000 indirect calls/sec. On a 160 MHz ESP32-C3 an
indirect call costs ~5–10 cycles more than a direct one, i.e. **~0.006 % of CPU**.
The I2C transaction it wraps costs ~100 µs.

*Cost accepted:* one vtable pointer per object (~40 objects) and one vtable per class
(~45 classes × ~8 entries × 4 B) ≈ **2 KB flash, 160 B RAM**. The lite build is
620 KB in a 2 MB partition.

*Escape hatch:* if profiling ever shows a problem, the `RegisterDevice` layer — the
only one called more than once per tick — can be made a compile-time template
parameter of the driver without touching any interface above it.

**This is measured, not assumed**: Phase 2's exit criterion is a hardware
before/after comparison of `LoopStats.overrunCount` and `maxDt` (§06).

### 4.2 No dynamic allocation after boot

Every driver and platform object is a member of the `Board` object, which itself is
a function-local `static`. `Board::begin()` may allocate; nothing after it does.
This is checkable: a CI test builds the host target with a global `operator new` that
aborts once `Board::begin()` has returned.

### 4.3 Bus ownership is structural, not a rule

`I2cBus` owns its `Mutex` and hands out `RegisterDevice` handles that lock it.
The "IMU task is the sole I2C owner" invariant (§00 §3) becomes: the
`InertialSubsystem` task is the only thing holding `RegisterDevice` handles onto
`i2c0`, and the `Board` grants them at construction. A second task cannot acquire
one without editing `Board.cpp` — which is exactly where a reviewer will see it.

The existing `test_production_contracts.cpp` grep-assertion that forbids
`xTaskCreate(baroTask` is replaced by a host test that asserts the bus sees exactly
one calling thread.

### 4.4 Task policy in one table

```cpp
namespace arduflite::hal {
enum class Priority : uint8_t {
    Cli        = 0,   Web       = 1,   Telemetry = 1,
    OuterLoop  = 2,   RcLink    = 3,   InnerLoop = 3,   Inertial = 4,
};
struct TaskConfig { const char* name; uint32_t stackBytes; Priority priority; int8_t core; };
}
```

`Scheduler::spawn()` takes a `TaskConfig`; a driver cannot pass a raw integer. The
ladder documented in AGENTS.md becomes a type. Stack sizes get audited once, here,
instead of being guessed per call site.

### 4.5 `Result<T>` and `Status`, not `bool`

`begin()`, `probe()`, `read()` return `Status`. Callers can distinguish
`NotPresent` (sensor not fitted — degrade gracefully) from `IoError` (bus fault —
retry then fail the preflight check) from `Corrupt` (bad calibration blob —
recalibrate). Today all three are `false` plus a log line.

### 4.6 Sensor mounting is data

The board descriptor carries an `AxisMap`. `InertialSubsystem` applies it
once, uniformly, to accel, gyro and mag, using a table-driven `rotate()`. The
`applyOrientation()` sign-flip block and its inconsistency (§00 2.3) disappear.
The map is also exposed as runtime config keys so a mis-mounted board — or a clone
part with odd axis signs — can be fixed
in the field without a rebuild.

## 5. What each layer may include

| Layer | May include | Must not include |
|---|---|---|
| `hal/core` | `<cstdint>`, `<cmath>`, `<atomic>` | anything else |
| `hal/platform` | `hal/core` | Arduino, FreeRTOS, ESP-IDF, `arduflite::device`, `arduflite::drivers` |
| `hal/esp32` | `hal/platform`, `hal/core`, Arduino, FreeRTOS, ESP-IDF | `arduflite::device`, `arduflite::drivers` |
| `hal/host` | `hal/platform`, `hal/core`, std C++ | Arduino, FreeRTOS |
| `hal/device` | `hal/core` | `hal/platform`, Arduino, `arduflite::drivers` |
| `hal/drivers` | `hal/core`, `hal/platform`, `hal/device`, vendor libs | Arduino globals (`Wire`, `Serial`), flight headers |
| `hal/board` | everything in `hal/` | flight headers |
| flight code | `hal/core`, `hal/device`, `hal/board/Board.h` | `hal/platform`, `hal/drivers`, `hal/esp32` |

Enforced by a CI include-direction check (§05 §4) — a legitimate use of the
source-scanning pattern already established in `test_production_contracts.cpp`.
