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

1. **Configuration Layer** (`include/`)
   - Pure configuration headers with `constexpr` values and structs
   - **Never** implement logic here — only declarations and constants
   - Use namespaces (e.g., `AttitudeControllerConfig`, `RateControllerConfig`) to group related configs
   - Factory functions like `makePID_TC()` are acceptable for compile-time computation

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
   - **Telemetry**: Output to ground station/transmitter (MQTT, Serial, CRSF, Flash)
   - Each backend runs in its own FreeRTOS task
   - Use thread-safe data structures (`TelemetryData`, `ConfigData`) with snapshots

5. **Utilities Layer** (`src/utils/`)
   - **ControlMixer**: Mode-dependent scaling and mixing (Attitude/Rate/Manual)
   - **CommandSystem**: Thread-safe command queue using FreeRTOS queues
   - **Logging**: Singleton logger with pluggable handlers (`LOG_INF`, `LOG_ERR`, etc.)
   - **Button Managers**: Input handling (HoldButton, MultiTapButton)
   - **StatusLED**: Visual feedback patterns

6. **CLI Layer** (`src/cli/`)
   - Command-line interface for runtime diagnostics and tuning
   - Uses `CommandSystem` to send thread-safe commands to other modules
   - **Never** directly modify controller state — always go through the command queue

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

## Folder Structure Overview

```
ArduFlite/
├── include/                          # Configuration headers (compile-time constants)
│   ├── ArduFlite.h                   # Main header, SemaphoreLock RAII
│   ├── ControllerConfiguration.h     # PID tuning parameters (makePID_TC factory)
│   ├── ServoConfiguration.h          # Servo limits, neutral, deflection
│   ├── CSRFConfiguration.h           # CRSF receiver pin/channel mapping
│   ├── ControlMixerConfiguration.h   # Mode-dependent scaling factors
│   ├── PinConfiguration.h            # Pin assignments (I2C, PWM, Buttons)
│   ├── ReceiverConfiguration.h       # Receiver type and failsafe config
│   ├── IMUConfiguration.h            # IMU sensor selection and calibration
│   └── MissionConfiguration.h        # Mission planner parameters
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
│   │   ├── ConfigData.h              # Configuration snapshot
│   │   ├── mqtt/                     # WiFi + MQTT telemetry
│   │   ├── serial/                   # Debug and quaternion serial output
│   │   ├── flash/                    # On-board flash logging
│   │   └── crsf/                     # CRSF telemetry uplink
│   │
│   ├── cli/                          # Command-line interface
│   │   ├── ArduFliteCLI.*            # CLI task and command router
│   │   └── CLICommands*.*            # Command implementations
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
│       ├── CommandSystem.*           # Thread-safe command queue
│       ├── ControlMixer.*            # Mode-dependent input mixing
│       ├── Logging.*                 # Singleton logger with colors
│       ├── StatusLED.*               # Visual feedback patterns
│       └── Button*.*                 # Input handling (hold, multi-tap)
│
├── tools/                            # Ground station and analysis scripts
│   ├── data_logger/                  # Python: log telemetry to CSV
│   ├── visualisation/                # Python: 3D attitude visualization
│   ├── flash_dump/                   # Python: extract flight logs from flash
│   ├── mqtt_broker/                  # Docker: Mosquitto MQTT broker
│   └── xbox_to_mqtt/                 # Python: joystick → MQTT for testing
│
├── ArduFlite.ino                     # Arduino entry point (calls arduflite_init/loop)
├── ArdufliteApp.cpp                  # Main application logic
└── README.md                         # Project documentation
```

### Key Design Patterns

1. **Configuration Segregation**
   - All tunable parameters live in `include/*Configuration.h`
   - Use `constexpr` for compile-time evaluation
   - Use `namespace` to group related configs (avoid global pollution)

2. **Manager Pattern**
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
   - When adding new tunable parameters, place them in the appropriate `*Configuration.h` file
   - Use factory functions (like `makePID_TC`) for computed constants
   - Document units and ranges in comments (e.g., `// degrees/second`)
   - Prefer `constexpr` over `#define` for type safety

2. **Adding New Control Features**
   - Extend `ControlLoopType` enum and `SystemCommandType` if needed
   - Add PID configs to `ControllerConfiguration.h`
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
