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
   - Controllers **must not** directly access hardware — use abstractions (IMU, ServoManager)

3. **Sensor/Actuator Layer** (`src/orientation/`, `src/actuators/`)
   - **ArduFliteIMU**: Sensor fusion, flight state detection, baro auto-calibration
   - **ServoManager**: Wing geometry abstraction (CONVENTIONAL, DELTA_WING, V_TAIL)
   - These classes **own** the hardware interfaces
   - Use FreeRTOS tasks for time-critical operations (e.g., IMU @ high Hz)

4. **Communication Layer** (`src/receiver/`, `src/telemetry/`)
   - **Receiver**: Input from pilot (CRSF/PWM) with failsafe callbacks
   - **Telemetry**: Output to ground station/transmitter (Serial, CRSF, Flash)
   - Each backend runs in its own FreeRTOS task
   - Use thread-safe `TelemetryData` snapshots; config queries go to `ConfigRegistry`

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
   - **StatusLED**: Visual feedback patterns

6. **CLI Layer** (`src/cli/`)
   - Command-line interface for runtime diagnostics and tuning
   - Uses `CommandSystem` to send thread-safe commands to other modules
   - **Never** directly modify controller state — always go through the command queue

7. **Web Layer** (`src/web/`)
   - **WiFiManager**: Singleton for WiFi Access Point management
   - **ArduFliteWebServer**: REST API for configuration (GET/PUT params, export/import JSON)
   - **WebUI.h**: Embedded responsive HTML/CSS/JS frontend in PROGMEM
   - Enabled via `web.enabled` config key; creates AP with configurable SSID/password
   - REST endpoints: `/api/config`, `/api/system/status`, `/api/flash`
   - Runs in its own FreeRTOS task at priority 1 (lowest, non-blocking)
   - **Compile-time toggle**: `ENABLE_WEB_SERVER` in `include/WebConfiguration.h`
     - Full build: `./build.sh lolin` (~1.3MB, includes WiFi/HTTP stack)
     - Lite build: `./build.sh lolin lite` (~800KB, flight-only, no WiFi)
     - WiFi/TCP/HTTP libraries add ~500KB; lite build excludes them entirely

### Dependency Rules
- **Higher layers can depend on lower layers, but NOT vice versa**
- Controllers depend on IMU/ServoManager, but IMU/ServoManager are independent
- Telemetry observes state but **never** modifies it
- Use dependency injection: pass pointers to dependencies in constructors

### Thread Safety
- ArduFlite uses FreeRTOS extensively with **multiple concurrent tasks**
- **Always** protect shared state with mutexes or use FreeRTOS queues
- Use `SemaphoreLock` RAII wrapper (defined in `ArduFlite.h`) for automatic mutex management
- Take snapshots of data structures (like `TelemetryData`) to avoid holding locks too long
- **Never** block in ISRs or high-priority tasks
- `ArduFliteIMU` uses a **triple-buffer** for lock-free reads: the IMU task writes sensor data to one buffer while control loops read from another, with a third buffer ensuring readers are never caught mid-copy

## Folder Structure Overview

```
ArduFlite/
├── include/                          # Headers and compile-time constants
│   ├── ArduFlite.h                   # Main header, SemaphoreLock RAII
│   ├── ConfigKeys.h                  # Config key #defines (hierarchical dot notation)
│   ├── ConfigSchema.h                # Parameter registration with defaults/ranges
│   ├── ControllerTypes.h             # Shared enums (ControlLoopType)
│   ├── CSRFConfiguration.h           # CRSF receiver pin/channel mapping
│   ├── PinConfiguration.h            # Pin assignments (compile-time)
│   ├── ReceiverConfiguration.h       # Receiver type and failsafe config
│   ├── IMUConfiguration.h            # IMU/Baro type selection macros only
│   ├── MissionConfiguration.h        # Mission planner parameters
│   └── WebConfiguration.h            # ENABLE_WEB_SERVER compile-time flag
│
├── src/
│   ├── controller/                   # Cascade PID control system
│   │   ├── ArduFliteController.*     # Top-level orchestrator (Outer+Inner loops)
│   │   ├── ArduFliteAttitudeController.*  # Attitude → Rate (outer loop)
│   │   ├── ArduFliteRateController.*      # Rate → Servo (inner loop)
│   │   └── pid.*                     # Generic PID with anti-windup
│   │
│   ├── orientation/                  # Sensor fusion and state estimation
│   │   ├── ArduFliteIMU.*            # IMU wrapper (FastIMU + Madgwick + BMP280)
│   │   └── FliteQuaternion.*         # Quaternion math helpers
│   │
│   ├── actuators/                    # Servo output and mixing
│   │   └── ServoManager.*            # Wing geometry abstraction
│   │
│   ├── receiver/                     # Pilot input (RC link)
│   │   ├── crsf/                     # CRSF (ELRS/Crossfire) receiver
│   │   └── pwm/                      # PWM receiver (legacy)
│   │
│   ├── telemetry/                    # Data output to ground station
│   │   ├── TelemetryData.h           # Shared data structure
│   │   ├── serial/                   # Debug and quaternion serial output
│   │   ├── flash/                    # On-board flash logging
│   │   └── crsf/                     # CRSF telemetry uplink
│   │
│   ├── cli/                          # Command-line interface
│   │   ├── ArduFliteCLI.*            # CLI task and command router
│   │   └── CLICommands*.*            # Command implementations
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
│   │   └── StateManagement.*         # Mode and flight state handlers
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
│       ├── StatusLED.*               # Visual feedback patterns
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

3. **Manager Pattern****
   - `ServoManager`, `HoldButtonManager`, `MultiTapButtonManager`
   - Managers **own** hardware resources and provide high-level APIs
   - Encapsulate geometry/mixing logic (e.g., delta wing vs. conventional)

3. **Command Pattern**
   - `CommandSystem` with FreeRTOS queue for thread-safe inter-task communication
   - Commands are POD structs (`SystemCommand`) with type discriminator
   - Prevents direct state mutation across task boundaries

4. **Observer Pattern**
   - Telemetry modules observe state without modifying it
   - Use snapshot pattern: copy data under lock, then process outside lock

5. **RAII for Locks**
   - `SemaphoreLock` automatically releases mutexes on scope exit
   - Prevents deadlocks from early returns or exceptions

## Ongoing Improvements

### Code Quality Guidelines

1. **Configuration Changes**
   - **Runtime-tunable parameters**: Add to `ConfigKeys.h` and `ConfigSchema.h`
   - **Compile-time constants** (hardware pins, sensor types): Add to appropriate `*Configuration.h`
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
   - Add sensor to `ArduFliteIMU` or create a new manager class
   - Use FreeRTOS tasks for high-rate sensors
   - Provide thread-safe getter methods
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
   - Bad: Implementing delta-wing mixing in both Controller and ServoManager
   - Good: ServoManager owns all geometry-specific mixing

6. **❌ Don't ignore FreeRTOS task priorities**
   - IMU Task (highest priority) → Inner Loop → Outer Loop → Telemetry → CLI
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
   - ESP32 has hardware FPU for `float`, not `double`
   - Use `float` for performance-critical code
   - Use `double` only when precision is essential (e.g., GPS coordinates)

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
