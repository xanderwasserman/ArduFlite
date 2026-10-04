# Agents Playbook

## Purpose
Provide concise, enforceable guidelines for any AI or human agent contributing to this repository. The goal is to keep the codebase maintainable, modular, and easy to navigate while continuously improving quality.

## Core Principles
1. **Preserve architecture boundaries.** Never reintroduce duplicated logic; extend or refactor shared helpers instead.
2. **Leave code better than you found it.** Apply boy-scout rules: clean up related smells, improve clarity, and add missing tests when practical.
3. **Favor readability over cleverness.** Small, well-named functions/modules beat large "spaghetti" blocks.
4. **Be explicit and documented.** Update relevant docs/configs whenever behavior or public APIs change.
5. **Validate changes.** Run targeted tests or provide verification steps; never assume success.
6. **Fail loudly.** If an assumption or dependency is missing, log/return early so issues are obvious.
7. **Prefer library built-ins over custom code.** Before implementing functionality, check if the library provides a built-in solution. Use native features when available.
8. **Never skip documentation updates.** Architectural changes, new modules, or API changes require updates to AGENTS.md and/or README.md before task completion.

## Workflow Checklist
- **Before coding**
  - Read the latest instructions *and* this playbook.
  - Review `git status` to understand the working tree.
  - Identify every file you expect to touch; plan reads in batches.
- **While coding**
  - Keep modules focused: orchestration vs. rendering vs. data helpers.
  - Use shared helpers instead of local fallbacks.
  - Add succinct comments only when logic is non-obvious.
  - Update or create tests/docs together with functional changes.
- **After coding**
  - Re-run relevant tests or provide precise manual verification steps.
  - Summarize changes clearly (what/why/where) and mention follow-up actions.
  - Ensure diffs are minimal and files stay formatted.

## Module Boundaries

### Core Architecture Layers
ArduFlite follows a strict layered architecture. Respect these boundaries:

1. **Configuration Layer** (`include/` + `src/utils/Config*`)
   - **Runtime-tunable parameters** use `ConfigRegistry` singleton with NVS persistence
   - **Compile-time constants** (sensor types, hardware pins) remain in `include/*.h`
   - Keys defined in `include/ConfigKeys.h`, defaults in `include/ConfigSchema.h`
   - Controllers use `initFromConfig()` pattern: default constructor + deferred init after ConfigRegistry loads
   - Hot-reload via observer pattern: `ConfigRegistry::subscribe("rate.roll.*", callback)`

2. **Control Layer** (`src/controller/`)
   - **ArduFliteController**: Top-level orchestrator managing cascade control loops
   - **ArduFliteAttitudeController**: Outer loop (attitude → rate setpoints)
   - **ArduFliteRateController**: Inner loop (rate setpoints → servo commands)
   - **PID**: Low-level PID implementation with anti-windup
   - Controllers **must not** directly access hardware — they read `ImuState`
     and emit `AxisCommand`. Both inner controllers are now free of FreeRTOS and
     Arduino entirely; they take an injected `hal::Mutex` and run in `host_sim`.

3. **Estimation and Actuator Layer** (`src/estimation/`, `src/actuators/`)
   - **InertialSubsystem**: the sampling tick — sample, select, read, offsets,
     axis transform, filter, validate, altitude, fuse, motion, calibrate,
     publish. A fixed twelve-step order; see §03 3.8 and ADR-033.
   - **AirframeMixer**: wing geometry (CONVENTIONAL, DELTA_WING, V_TAIL stub)
   - Neither owns hardware. Drivers behind `device::` interfaces do, and `Board`
     hands them out.
   - **`src/estimation/` must contain NO direct RTOS calls** — it may use
     `hal::Scheduler` and `hal::Watchdog`, never `xTaskCreate` or `millis()`.
     `tools/ci/check_layering.sh` enforces this. It is what lets the twelve-step
     order be tested at all.
   - **Sensor decimation is derived from the sensor**, via `nativeRate_hz()` on
     `device::Measurement` — never configured beside it. Two numbers disagreed
     once and inflated climb rate by 3.3x (ADR-046).
   - The IMU task remains the **sole owner of the I2C bus**. Do not add a second
     task sharing it; that reintroduces the priority inversion and the "sensor
     mutex busy" skips. `check_layering.sh` fails on any `baroTask`.

4. **Communication Layer** (`src/receiver/`, `src/telemetry/`)
   - **Receiver**: Input from pilot (CRSF/PWM) with failsafe callbacks
   - **Telemetry**: Output to ground station/transmitter (CRSF, Flash, MAVLink)
   - Each backend runs in its own FreeRTOS task
   - Use thread-safe `TelemetryData` snapshots; config queries go to `ConfigRegistry`
   - **MAVLink** (`src/telemetry/mavlink/`, `specs/mavlink/`): one
     `MavlinkEndpoint` per `hal::ByteStream` — the USB console once a ground
     station connects (or after `mavlink on`), and the board's telemetry UART. The endpoint is host-tested
     and never blocks: it checks `writable()` and the byte budget, and skips.
   - The vendored MAVLink library in `src/third_party/mavlink/` is never edited
     and is included only through `src/telemetry/mavlink/Mavlink.h`
     (`check_layering.sh` rule 11). Update it with `tools/mavlink/update_vendor.sh`

5. **Utilities Layer** (`src/utils/`)
   - **ConfigRegistry**: Singleton for runtime config with type-safe get/set, validation, observers
   - **ConfigPersistence**: NVS-backed storage with schema versioning and JSON export/import
   - **ConfigTask**: Background FreeRTOS task for periodic dirty-save and import queue
   - **ConfigHelpers**: `buildPIDConfig()` converts Ti/Td time constants to Ki/Kd gains
   - **ConfigObservers**: Bridges config changes to CommandSystem for thread-safe updates
   - **ControlMixer**: Mode-dependent scaling and mixing (Attitude/Rate/Manual)
   - **CommandSystem**: Thread-safe command queue using FreeRTOS queues
   - **Logging**: Singleton logger with pluggable handlers (`LOG_INF`, `LOG_ERR`, etc.)
   - **Button Managers**: Input handling (HoldButton, MultiTapButton)
   - **Indicator**: Visual feedback patterns, via `device::Indicator`

6. **CLI Layer** (`src/cli/`)
   - Command-line interface for runtime diagnostics and tuning
   - Uses `CommandSystem` to send thread-safe commands to other modules
   - **Never** directly modify controller state — always go through the command queue
   - Command implementations are split by concern:
     - `CLICommands.cpp`: command table and help output
     - `CLICommandsSystem.cpp`: reset/stats/tasks/setmode/calibrate
     - `CLICommandsConfig.cpp`: configuration registry commands only
     - `CLICommandsFlash.cpp`: flash log commands
     - `CLICommandsTelemetry.cpp`: serial telemetry streaming
     - `CLICommandsTests.cpp`: field-safe integration tests
   - Shared CLI parsing belongs in `CLICommandUtils.*`; shared CLI dependencies and ground-safety checks belong in `CLICommandContext.*`

7. **Web Layer** (`src/web/`)
   - **WiFiManager**: Singleton for WiFi Access Point management
   - **ArduFliteWebServer**: REST API for configuration (GET/PUT params, export/import JSON)
   - **WebUI.h**: Embedded responsive HTML/CSS/JS frontend in PROGMEM
   - Enabled via `web.enabled` config key; creates AP with configurable SSID and WPA2 password (`web.ap_pass` must be 8+ characters)
   - Full builds run captive DNS so phone/laptop captive-portal probes resolve to the Web UI
   - Mutating REST requests require the per-boot same-origin token from `/api/session`
   - REST endpoints: `/api/config`, `/api/system/status`, `/api/flash`
   - Runs in its own FreeRTOS task at priority 1 (lowest, non-blocking)
   - **Compile-time toggle**: `ENABLE_WEB_SERVER` in `include/WebConfiguration.h`
     - Full build: `./build.sh lolin` (~1.4 MB, includes WiFi/HTTP stack)
     - Lite build: `./build.sh lolin lite` (~630 KB, flight-only, no WiFi)
     - Builds use per-board/per-variant output directories (`build/lolin-full`, `build/lolin-lite`)
     - WiFi/TCP/HTTP libraries add ~780 KB; the lite build excludes them entirely

### Dependency Rules
- **Higher layers can depend on lower layers, but NOT vice versa**
- Controllers depend on `ImuState`; the estimation layer knows nothing of them
- **Flight code must not include a driver header.** Only `Board` names a
  concrete chip. Enforced by `check_layering.sh`
- Board descriptors are data: adding a sensor to a board is one array entry
- Telemetry observes flight state but **never** modifies it. The one inbound
  path, MAVLink parameter writes and reboot, goes through `ConfigRegistry::set()`
  and `CommandSystem` and the shared ground-safety rule
  (`src/state/GroundSafety.h`), exactly as the CLI does
- Use dependency injection: pass pointers to dependencies in constructors

### Thread Safety
- ArduFlite uses FreeRTOS extensively with **multiple concurrent tasks**
- **Always** protect shared state with mutexes or use FreeRTOS queues
- Lock a `hal::Mutex` with `std::unique_lock` / `std::scoped_lock`. Use the
  bounded form (`std::unique_lock lock(m, timeout)`) on any control path — an
  unbounded wait in a 500 Hz loop is a watchdog reset
- Take snapshots of data structures (like `TelemetryData`) to avoid holding locks too long
- **Never** block in ISRs or high-priority tasks
- `estimation::InertialSubsystem` publishes through a `SeqLock<ImuState>`: the
  sampling task marks the version odd while writing and even when complete, and
  readers retry if it changes mid-copy. Read it with `imu.state()` — one call,
  one coherent snapshot. Taking several separate reads defeats the point

## Folder Structure Overview

```
ArduFlite/
├── include/                          # Headers and compile-time constants
│   ├── ArduFlite.h                   # Main header, entry points
│   ├── ConfigKeys.h                  # Config key #defines (hierarchical dot notation)
│   ├── ConfigSchema.h                # Parameter registration with defaults/ranges
│   ├── ControllerTypes.h             # Shared enums (ControlLoopType)
│   ├── AircraftConfiguration.h       # Compile-time aircraft type (powered vs glider)
│   ├── MissionConfiguration.h        # Mission planner parameters
│   └── WebConfiguration.h            # ENABLE_WEB_SERVER compile-time flag
│
│   NOTE: PinConfiguration.h, IMUConfiguration.h, CSRFConfiguration.h and
│   ReceiverConfiguration.h are GONE. Hardware layout lives in one board
│   descriptor per board — see src/hal/board/boards/.
│
├── src/
│   ├── controller/                   # Cascade PID control system
│   │   ├── ArduFliteController.*     # Top-level orchestrator (Outer+Inner loops)
│   │   ├── ArduFliteAttitudeController.*  # Attitude → Rate (outer loop)
│   │   ├── ArduFliteRateController.*      # Rate → Servo (inner loop)
│   │   └── pid.*                     # Generic PID with anti-windup
│   │
│   ├── hal/                          # Hardware abstraction — see specs/hal/
│   │   ├── core/                     # Status, Result, Vec3, SeqLock, AxisTransform, Crc32
│   │   ├── platform/                 # Clock, Mutex, Scheduler, Watchdog, Buses, ByteStream, Io, Storage
│   │   ├── device/                   # Sensor, Actuator, RcLink, Peripherals (role interfaces)
│   │   ├── drivers/                  # Concrete chips: imu/, baro/, rc/, out/, log/, indicator/
│   │   ├── protocol/                 # Wire formats shared by a driver and an adapter (CRSF)
│   │   ├── esp32/                    # ESP32 platform implementations
│   │   └── board/                    # Descriptors + composition root
│   │       └── boards/               # ONE FILE PER BOARD — all hardware layout
│   │
│   ├── estimation/                   # Sensor fusion and state estimation
│   │   ├── InertialSubsystem.*       # The sampling tick; no RTOS, host-testable
│   │   ├── AttitudeEstimator.h       # Fusion behind an interface
│   │   ├── MadgwickEstimator.*      # Own gradient-descent filter (ADR-056)
│   │   ├── CalibrationService.*      # Calibration as a state machine, inside the tick
│   │   ├── SensorSelector.*          # Which instance feeds the estimator
│   │   ├── MotionDetector.*          # Launch / stable debouncing
│   │   ├── AltitudeFilter.*          # Barometric altitude and climb rate
│   │   └── ImuState.h                # The published snapshot
│   │
│   ├── core/                         # Flight-layer vocabulary, no hardware
│   │   ├── FlightTypes.h             # AttitudeDeg, AngularRateDps, AxisCommand
│   │   └── LogRotationPolicy.*       # Log index allocation and purge rules
│   │
│   ├── orientation/                  # Quaternion math
│   │   └── FliteQuaternion.*
│   │
│   ├── actuators/                    # Surface mixing and output
│   │   ├── AirframeMixer.*           # Axis demand -> per-surface commands
│   │   └── ControlOutputs.*          # Surfaces resolved by role
│   │
│   ├── telemetry/                    # Data output to ground station
│   │   ├── TelemetryData.h           # Shared data structure
│   │   ├── flash/                    # On-board flash logging
│   │   ├── crsf/                     # CRSF telemetry uplink
│   │   └── mavlink/                  # MAVLink 2 endpoints, parameters, STATUSTEXT
│   │
│   ├── third_party/                  # Vendored upstream code, never edited
│   │   └── mavlink/                  # c_library_v2 (common dialect), see VERSION
│   │
│   ├── cli/                          # Command-line interface
│   │   ├── ArduFliteCLI.*            # CLI task and command router
│   │   ├── CLICommands.*             # Command table and command declarations
│   │   ├── CLICommandContext.*       # Shared CLI dependencies and safety checks
│   │   ├── CLICommandUtils.*         # Generic CLI parsing helpers
│   │   └── CLICommands*.*            # Concern-specific command implementations
│   │
│   ├── web/                          # Web configuration interface
│   │   ├── WiFiManager.*             # WiFi Access Point singleton
│   │   ├── ArduFliteWebServer.*      # REST API and web server
│   │   └── WebUI.h                   # Embedded HTML/CSS/JS in PROGMEM
│   │
│   ├── mission_planner/              # Autonomous mission execution
│   │   └── MissionPlanner.*          # Future: waypoint navigation
│   │
│   ├── state/                        # State machines
│   │   ├── StateManagement.*         # Mode and flight state handlers
│   │   └── GroundSafety.h            # The rule for ground-only commands
│   │
│   ├── tests/                        # Test sequences
│   │   ├── AttitudeTests.*           # Wing wiggle tests
│   │   └── ReceiverTests.*           # Receiver input validation
│   │
│   └── utils/                        # Shared utilities
│       ├── ConfigRegistry.*          # Singleton config store with observers
│       ├── ConfigPersistence.*       # NVS load/save with schema versioning
│       ├── ConfigTask.*              # Background task for periodic saves
│       ├── ConfigHelpers.h           # PID config builders (Ti/Td → Ki/Kd)
│       ├── ConfigObservers.*         # Observer registrations for hot-reload
│       ├── CommandSystem.*           # Thread-safe command queue
│       ├── ControlMixer.*            # Mode-dependent input mixing
│       ├── Logging.*                 # Singleton logger with colors
│       └── Button*.*                 # Input handling (hold, multi-tap)
│
├── docs/                             # Project documentation
│   ├── CONFIG_REFERENCE.md           # Runtime parameter reference
│   └── flight_logs/                  # Chronological flight test records (FL001, FL002, ...)
│
├── tools/                            # Ground station and analysis scripts
│   ├── data_analysis/                # Python: flight data analysis
│   ├── visualisation/                # Python: 3D attitude visualization
│   └── flash_dump/                   # Python: extract flight logs from flash
│
├── ArduFlite.ino                     # Arduino entry point (calls arduflite_init/loop)
├── ArdufliteApp.cpp                  # Main application logic
└── README.md                         # Project documentation
```

### Key Design Patterns

1. **Persistent Configuration System**
   - `ConfigRegistry`: Singleton storing all runtime-tunable parameters
   - `ConfigPersistence`: NVS-backed save/load with JSON export/import
   - `ConfigSchema.h`: Static registration macros (`CONFIG_FLOAT`, `CONFIG_INT`, etc.)
   - `ConfigKeys.h`: Hierarchical keys with IDE autocomplete (e.g., `CONFIG_KEY_RATE_ROLL_KP`)
   - Components use `initFromConfig()` pattern for deferred initialization after FreeRTOS

2. **Deferred Initialization Pattern**
   - Controllers have default constructors (safe/zero values)
   - After `ConfigRegistry::init()` + `ConfigPersistence::load()`, call `initFromConfig()`
   - Enables global objects while respecting FreeRTOS startup order

3. **Manager Pattern**
   - `HoldButtonManager`, `MultiTapButtonManager`
   - Managers **own** hardware resources and provide high-level APIs
   - Airframe geometry is NOT a manager: `actuators::AirframeMixer` is a pure
     function from axis demands to surface demands, so it is host-testable

3. **Command Pattern**
   - `CommandSystem` with FreeRTOS queue for thread-safe inter-task communication
   - Commands are POD structs (`SystemCommand`) with type discriminator
   - Prevents direct state mutation across task boundaries

4. **Observer Pattern**
   - Telemetry modules observe state without modifying it
   - Use snapshot pattern: copy data under lock, then process outside lock

5. **RAII for Locks**
   - `std::unique_lock` / `std::scoped_lock` over `hal::Mutex`
   - Releases on every exit path, including early returns
   - `hal::WatchdogGuard` does the same for watchdog registration

## Ongoing Improvements

### Code Quality Guidelines

1. **Configuration Changes**
   - **Runtime-tunable parameters**: Add to `ConfigKeys.h` and `ConfigSchema.h`
   - **Compile-time constants** (hardware pins, sensor types, aircraft type): Add to appropriate `*Configuration.h`
   - Use `ConfigHelpers::buildPIDConfig()` for PID-related configs
   - Document units and ranges in comments and schema description
   - Register observers in `ConfigObservers.cpp` if hot-reload is needed

2. **Adding New Control Features**
   - Extend `ControlLoopType` enum (in `ControllerTypes.h`) and `SystemCommandType` if needed
   - Add PID configs to `ConfigSchema.h` with `CONFIG_FLOAT` macros
   - Update `ArduFliteController::processCommands()` to handle new commands
   - Add CLI commands in `CLICommands*.cpp` for runtime tuning

3. **New Telemetry Backends**
   - Inherit from base telemetry interface (if one exists, or create it)
   - Run in a separate FreeRTOS task with configurable update rate
   - Use `TelemetryData::update()` to get a consistent snapshot
   - **Never** hold locks during network I/O or slow operations

4. **New Sensor Integration**
   - Write a driver in `src/hal/drivers/` implementing the `device::` role
     interfaces it fills (`Sensor` plus `Accelerometer`, `Barometer`, ...)
   - Talk to it through `hal::RegisterDevice`, never a bus type directly — that
     is what keeps it host-testable and bus-agnostic
   - Declare it in the board descriptor and construct it in `Board::beginSensors()`
   - Do NOT give it its own task: `InertialSubsystem` samples every declared
     sensor on one tick, decimated by each part's `nativeRate_hz()`
   - Document calibration procedures in comments and README

5. **Testing**
   - Add test functions in `src/tests/` for new control modes
   - Use test sequences (like `runAttitudeTest_wiggle`) to verify hardware
   - Prefer automated validation over manual "it looks okay"
   - Update `README.md` with new test procedures

### Common Pitfalls to Avoid

1. **❌ Don't bypass the CommandSystem**
   - Bad: Directly calling `controller.setMode()` from a button callback
   - Good: Push `CMD_SET_MODE` command, let main loop process it

2. **❌ Don't hold locks during I/O**
   - Bad: Lock mutex, then `Serial.print()` or `WiFi.send()`
   - Good: Take snapshot under lock, release lock, then perform I/O

3. **❌ Don't use raw pointers without ownership clarity**
   - If a class stores a pointer, document who owns the object
   - Prefer references for mandatory non-null dependencies

4. **❌ Don't hard-code magic numbers**
   - Bad: `if (roll > 45.0f)` in source code
   - Good: `if (roll > ControlMixerConfig::MAX_ROLL_DEGREES)`

5. **❌ Don't replicate mixing logic**
   - Bad: implementing delta-wing mixing in both the controller and the output path
   - Good: `actuators::AirframeMixer` owns all geometry-specific mixing

6. **❌ Don't ignore task priorities**
   - The ladder is `hal::Priority` in `src/hal/platform/Scheduler.h`, and that
     enum is authoritative — never a number written into an xTaskCreate call
   - Inertial (4) > InnerLoop (3) > OuterLoop / RcLink (2) > everything
     background (1: CLI, Web, Config, Telemetry, Mission, Indicator)
   - Duplicate values are intentional: those tasks really do share a priority
   - Critical tasks must pre-empt slower ones to maintain loop rates

7. **❌ Don't use `Serial.print` directly**
   - Bad: `Serial.println("Debug message")`
   - Good: `LOG_DBG("Debug message")` (uses Logger singleton with colors)

### Performance Considerations

1. **Loop Timing**
   - Outer loop: ~100 Hz (10 ms period)
   - Inner loop: ~500 Hz (2 ms period)
   - Avoid dynamic allocation in control loops (pre-allocate)
   - Monitor loop stats via `LoopStats` and CLI `stats` command

2. **Memory Usage**
   - ESP32 has limited RAM (~320 KB)
   - Use `constexpr` to move data to flash when possible
   - Be mindful of FreeRTOS task stack sizes (typically 4096 bytes)

3. **Float vs. Double**
   - **The ESP32-C3 has NO FPU at all.** It is `rv32imc` with a soft-float ABI —
     verified from the build map, which links `__addsf3`, `__mulsf3` and friends.
     Both `float` and `double` are emulated in software on the primary target.
   - The classic ESP32 (FireBeetle) *does* have a single-precision FPU. So the
     rule differs by board, and the C3 is the one that flies.
   - Use `float` regardless: on the C3 it is several times cheaper than `double`
     even though both are emulated, and on the classic ESP32 it is free.
   - Use `double` only where precision genuinely demands it (e.g. GPS
     coordinates), and never in the 500 Hz control path.

### Documentation Standards

1. **File Headers**
   - All files must include copyright header with author, version, date
   - Use MIT License boilerplate

2. **Function Comments**
   - Use Doxygen-style `@brief`, `@param`, `@return`
   - Document units (seconds, degrees, radians, etc.)
   - Explain non-obvious algorithms or magic numbers

3. **Inline Comments**
   - Keep them succinct and only where logic is non-obvious
   - Prefer self-documenting code (good names) over excessive comments

4. **README and AGENTS.md**
   - Update README when adding user-facing features
   - Update AGENTS.md when changing architecture or patterns

Following this playbook is mandatory: if instructions ever conflict, pause and clarify before proceeding.
