# ArduFlite

ArduFlite is a highly modular and real-time flight control framework designed for small unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 platform with FreeRTOS, ArduFlite integrates sensor fusion from an IMU (such as the MPU-6500), cascade PID controllers for robust attitude and rate control, flexible servo mapping, multiple telemetry backends, a customizable command line interface (CLI) for live diagnostics and dynamic configuration, and robust failsafe behavior.

## 🚀 Key Features

### 1. Modular Control Architecture

- **Attitude Control (Outer Loop @ ~100 Hz)**  
  Converts desired Euler‐angle setpoints into angular rate commands using an IMU-driven controller.
- **Rate Control (Inner Loop @ ~500 Hz)**  
  High-bandwidth PID regulators translate rate commands into servo outputs for precise, disturbance-rejecting flight.
- **Airframe Mixing**  
  - **Conventional** and **Delta-Wing** geometries. V-Tail is reserved in the
    enum but produces no deflection — a V-tail airframe has no control until it
    is implemented.
  - `actuators::AirframeMixer` is a pure function from axis demands to surface
    demands, so the geometry is host-testable without hardware.
  - Per-output neutral, deflection, pulse limits and inversion, with a
    slew-rate limiter per surface, applied by the actuator bank.

### 2. CRSF Receiver & Failsafe

- **CrsfLink**  
  - Byte-wise parsing of Crossfire/ELRS packets over UART @ 420 kbps.
  - Channel values are scaled on the CRSF endpoints (raw 172 / 992 / 1811 to
    1000 / 1500 / 2000 µs), so full stick reaches full travel.
  - Frame parsing lives in `drivers::CrsfParser`, free of UART and FreeRTOS, so
    it is tested against captured bytes on a host.
- **RcMapper**  
  - Channel-to-role mapping and stick shaping, held outside the driver: the
    protocol decoder never calls a flight-layer callback directly.
- **Failsafe**  
  - On link loss: throttle cut, forced ATTITUDE_MODE, and a configured bank and
    pitch for a contained spiral descent. The three are one action — a partial
    failsafe is reported as an error rather than logged as success.

### 3. Estimation

- **InertialSubsystem**  
  - One sampling task drives every declared sensor, decimated by each part's own
    reported rate, and publishes a single coherent `ImuState` snapshot through a
    seqlock — readers never take a lock.
  - Attitude fusion sits behind an interface (`estimation::MadgwickEstimator`),
    so the filter can be swapped without touching the sampling loop.
- **CalibrationService**  
  - Barometric calibration captures ground-level reference pressure in situ, so
    altitude is measured from the launch site.
  - Runs for 10 seconds and requires at least 100 samples: a run that managed
    one reading during a bus fault is rejected rather than stored.
  - Accumulates inside the sampling task, which keeps ticking and feeding the
    watchdog throughout — no task is suspended to calibrate.

### 4. Throttle Cut & Arming Logic

- **ArduFliteController**  
  - `arm()`, `disarm()` and **throttle cut**. Both gates are atomic, so neither
    can fail to close under lock contention and neither is delayed by a tick
    that missed the lock.
  - Arming runs preflight checks: IMU health, gyro stability, accelerometer
    plausibility, receiver link, and throttle at minimum.
  - Clean separation of Flight Modes:  
    - **ATTITUDE_MODE** (“Assist”)  
    - **RATE_MODE** (“Stabilized”)  
    - **MANUAL_MODE** (direct passthrough)

### 5. Telemetry & CLI

- **Multiple Backends**  
  Each runs in its own task at a rate given to its constructor. The rates below
  are what `ArdufliteApp.cpp` configures; only CRSF and Flash are enabled.
  - **CRSF Telemetry** (10 Hz) — native to ELRS/Crossfire.
  - **Flash Telemetry** (50 Hz) — on-board CSV flight logging.
  - **Debug Serial** and **Q-Serial** — ground-side development backends,
    constructed but commented out.
- **CRSF Telemetry Highlights**  
  - Fast frames every loop (10 Hz): Attitude, Link-Stats, Vario.
  - Medium frames every 200 ms (5 Hz): Baro Altitude, GPS, Flight Mode.
  - The battery frame is sent only when a power monitor actually measured
    something. No board fits one, so a radio shows no battery telemetry rather
    than a confident 0.0 V.
  - GPS fields are zeroed with a satellite count of 0, which is the protocol's
    "no fix", so the frame is discoverable without reporting a position.
- **Built-in CLI**  
  - Real-time tuning: `setmode`, `calibrate imu`, `stats`, `tasks`, etc.  
  - Full access to loop-timing statistics and active FreeRTOS tasks.
### 6. Persistent Configuration System

- **ConfigRegistry Singleton**  
  - Centralized storage for 95 runtime-tunable parameters.  
  - Type-safe get/set with validation and range checking.  
  - Observer pattern for hot-reload of controller parameters.  
- **NVS Persistence**  
  - Automatic save of modified parameters to ESP32 flash (NVS).  
  - Parameters survive power cycles without recompilation.  
  - Schema versioning with migration support for firmware upgrades.  
- **JSON Export/Import**  
  - Backup and restore configuration via CLI or serial.  
  - Share tuned configs across multiple aircraft.  
- **CLI Integration**  
  - `config list`, `config get <key>`, `config set <key> <value>`.  
  - All changes take effect immediately (hot-reload) and persist automatically.
  - See [docs/CONFIG_REFERENCE.md](docs/CONFIG_REFERENCE.md) for full parameter documentation and tuning guide.

### 7. Web Configuration Interface

- **WiFi Access Point**
  - ArduFlite can create its own WiFi hotspot for configuration.
  - Enable via `config set web.enabled true` (requires reboot).
  - Customizable SSID and WPA2 password via `web.ap_ssid` and `web.ap_pass` keys (`web.ap_pass` must be at least 8 characters).
  - If `web.ap_pass` is still unset/default, firmware uses the unique AP SSID as a temporary password and logs a warning.
- **Responsive Web UI**
  - Mobile-friendly interface accessible at `http://192.168.4.1`.
  - Full builds run captive DNS so phones can also open the UI from their
    WiFi sign-in prompt or by visiting `http://arduflite.local` / any HTTP hostname.
  - Mutating WebUI actions use a per-boot same-origin session token to reject drive-by POST/PUT/DELETE requests.
  - Tabbed navigation: Rate, Attitude, Mixer, Servos, IMU, Failsafe, CRSF,
    Web, System, Other, Logs, Backup.
  - Real-time display of controller status (mode, armed state, heap usage).  
- **REST API Endpoints**  
  - `GET /api/session` — Per-boot token required by every mutating request.
  - `GET /api/config` — List all parameters with optional pattern filter.
  - `GET /api/config/:key` — Read one parameter.
  - `PUT|POST /api/config/:key` — Modify one parameter.
  - `GET /api/config/export` — Export full config as JSON.
  - `POST /api/config/import` — Import JSON configuration.
  - `POST /api/config/reset` — Restore defaults.
  - `POST /api/config/reboot` — Reboot the flight controller.
  - `GET /api/system/status` — Controller state, uptime, memory.
  - `GET /api/telemetry` — Live attitude and flight state.
  - `POST /api/system/calibrate` — Run an IMU calibration.
  - `GET /api/flash` — List flight logs.
  - `GET|DELETE /api/flash/:file` — Download or delete a log.

  Mutating endpoints are refused while armed or in flight, and require the
  session token.
- **Use Cases**  
  - Field tuning without a laptop (use phone/tablet).  
  - Bulk config changes via JSON import.  
  - Download flight logs wirelessly.

## Architecture

ArduFlite employs a cascade control structure:

1. **Attitude Controller (Outer Loop):**  
    Runs at approximately 100Hz and translates attitude errors (derived from IMU quaternions) into desired angular rate setpoints.

2. **Rate Controller (Inner Loop):**  
    Runs at approximately 500Hz to compute and output servo commands based on measured angular rates and the desired rate setpoints provided by the outer loop or direct pilot input.

3. **Mixer and Actuator Bank:**  
    `AirframeMixer` maps normalised axis demands to per-surface demands for the
    configured wing geometry; the actuator bank applies each output's calibration
    and slew limit and commits them to hardware in one batched write.

4. **Telemetry & CLI:**  
   Multiple telemetry modules are provided to suit different monitoring needs (CRSF uplink, Flash logging, and debug Serial). A dedicated CLI task supports real-time data queries, dynamic parameter adjustments, and troubleshooting commands.

## 🔧 Installation & Setup

### Hardware Requirements

- ESP32 microcontroller
- IMU sensor (e.g., MPU-6500)
- Servos for control surfaces (ailerons, elevator, rudder)
- Additional components for telemetry (optional)
- ELRS receiver (Crossfire UART interface) and transmitter

### Software Requirements

- Arduino CLI (or Arduino IDE / PlatformIO)
- ESP32 core for Arduino (includes FreeRTOS)
- Libraries: Adafruit NeoPixel, ArduinoJson
  (the IMU and barometer are driven by ArduFlite's own HAL drivers — no vendor library)

### Setup Instructions

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/xanderwasserman/ArduFlite.git
    cd ArduFlite
    ```

2. **Install Arduino CLI (if not already installed):**
    ```bash
    brew install arduino-cli
    ```

3. **Install ESP32 Board Support Package:**
    ```bash
    arduino-cli core install esp32:esp32
    ```

4. **Install Required Libraries:**
    ```bash
    arduino-cli lib install "Adafruit NeoPixel"
    arduino-cli lib install "ArduinoJson"
    ```
    
    **Note:** Some libraries (FreeRTOS, WiFi, Wire, EEPROM, Preferences, FS, LittleFS) are built into the ESP32 core and don't require separate installation.

    Servo output uses the ESP32 core's own LEDC API (`ledcAttach`/`ledcWrite`),
    confined to a single HAL file. No external servo library is needed.

5. **Verify Installation:**
    ```bash
    arduino-cli lib list
    ```

6. **Configure Hardware & Parameters:**
    - **Runtime parameters** (PID gains, limits): Use CLI `config set` command or JSON import. Parameters are defined in `include/ConfigSchema.h`.
    - **Compile-time hardware** (pins, sensors, actuators): edit the **board
      descriptor** in `src/hal/board/boards/`. One file per board, and it is
      the only place hardware layout is described — pins, I2C addresses, sensor
      parts, actuator channels, the status LED, axis orientation.

      A board is selected by a single define (`ARDUFLITE_BOARD_*`), and
      everything else follows from its descriptor. Descriptors are validated at
      compile time, so a pin outside the MCU's range or a pin assigned twice is
      a build error rather than a boot-time surprise.

    Board selection, set by `build.sh`:
    ```
    ARDUFLITE_BOARD_LOLIN_C3_MINI       src/hal/board/boards/lolin_c3_mini.h
    ARDUFLITE_BOARD_FIREBEETLE_ESP32E   src/hal/board/boards/firebeetle_esp32e.h
    ```

    Remaining build flags:
    ```
    AIRCRAFT_TYPE     - AIRCRAFT_TYPE_POWERED (0) or AIRCRAFT_TYPE_GLIDER (1)
    ENABLE_WEB_SERVER - 0 for the `lite` build (~780 KB smaller)
    ```

    Adding a board is one descriptor file plus one `#elif` in
    `src/hal/board/BoardSelect.h`. Adding a sensor to an existing board is a
    single entry in that board's `sensors` array — no code change. Both are
    validated at compile time: a pin that is input-only, reserved, or used
    twice fails the build rather than surfacing at boot.

7. **Compile and Upload:**
    ```bash
    ./build.sh lolin
    ./upload.sh lolin
    ```
    Build outputs are separated by board and variant, for example `build/lolin-full`
    and `build/lolin-lite`. The build script performs a clean compile inside
    the selected output directory to avoid stale generated Arduino files.
    Pass the same optional variant to upload:
    `./upload.sh lolin lite`.

### Usage
Once the system is running:
- Telemetry:
    Choose your telemetry method:

    - CRSF Telemetry: For sending telemetry directly to your transmitter using the ELRS receiver's uplink. Shows attitude, flight mode, battery, GPS, and link stats on your TX.

    - Flash Telemetry: For high-frequency on-board logging. Use `tools/flash_dump/` to extract flight logs after landing.

    - Debug Serial Telemetry: For low-frequency logging and debugging.

    - IMU snapshot health counters are included in Flash logs, Debug Serial output, CLI `stream`, and web status/telemetry JSON so snapshot contention can be diagnosed after field tests.

    - Q Serial Telemetry: For high-frequency, detailed real-time quaternion data output, for use with the visualiser.

- CLI Access:
    Open the Serial Monitor at 115200 baud. Type `help` to see a list of available commands.

    Example commands:

    - `help` – Lists available commands with descriptions.

    - `stats` – Displays current control loop timing statistics.

    - `tasks` – Shows FreeRTOS task statistics.

    - `setmode assist` – Switches the controller to ATTITUDE_MODE.

    - `setmode stabilized` – Switches the controller to RATE_MODE.

    - `calibrate imu` – Triggers an IMU self-calibration routine.

    - `flash list` - Lists the flight logs that have been stored in the on-board flash.

    - `config list` – Shows all configuration parameters.

    - `config get rate.roll.kp` – Gets the current value of a parameter.

    - `config set rate.roll.kp 0.12` – Sets a parameter value (persisted to flash).

    - `config export` – Exports all config as JSON for backup.

    - `config reset` – Resets all parameters to defaults.

- Control Operation:
    The attitude controller continuously computes new setpoints (from pilot input or test sequences), and the rate controller maintains stable flight even in the presence of disturbances.

# Contributing
Contributions are welcome! Whether you're adding new telemetry modules, enhancing the CLI, or improving control algorithms, please fork the repository and submit a pull request. For major enhancements, open an issue to discuss your ideas.

# License
This project is licensed under the MIT License – see the LICENSE file for details.

## Happy Flying with ArduFlite!

# References
- [FireBeetle 2 ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#target_3)

