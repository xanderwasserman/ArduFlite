# ArduFlite

ArduFlite is a highly modular and real-time flight control framework designed for small unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 platform with FreeRTOS, ArduFlite integrates sensor fusion from an IMU (such as the MPU-6500), cascade PID controllers for robust attitude and rate control, flexible servo mapping, multiple telemetry backends, a customizable command line interface (CLI) for live diagnostics and dynamic configuration, and robust failsafe behavior.

## 🚀 Key Features

### 1. Modular Control Architecture

- **Attitude Control (Outer Loop @ ~100 Hz)**  
  Converts desired Euler‐angle setpoints into angular rate commands using an IMU-driven controller.
- **Rate Control (Inner Loop @ ~500 Hz)**  
  High-bandwidth PID regulators translate rate commands into servo outputs for precise, disturbance-rejecting flight.
- **ServoManager**  
  - Supports **Conventional**, **Delta-Wing**, and **V-Tail** geometries.  
  - Adjustable neutral, deflection, pulse limits, and inversion flags.  
  - Built-in slew-rate limiter per surface to protect servos.

### 2. CRSF Receiver & Failsafe

- **CRSFReceiver**  
  - Byte-wise parsing of cross-fire/ELRS packets over UART @ 420 kbps.  
  - Configurable per-channel mapping & callbacks via `CRSFConfiguration.h`.  
- **Failsafe Callback**  
  - When link is lost, user-supplied function runs: e.g. switch into ATTITUDE_MODE, bank & descend for a safe circular landing.  

### 3. Baro-Auto-Calibrating IMU

- **ArduFliteIMU**  
  - On boot, gathers 1 s of BMP280 samples *in-situ* and stores that as `referencePressure`.  
  - Ensures accurate altitude from your actual launch site — not last-month’s desk calibration.

### 4. Throttle Cut & Arming Logic

- **ArduFliteController**  
  - `arm()`, `disarm()`, and **throttle cut** flags with single-lock snapshot in the inner loop.  
  - Clean separation of Flight Modes:  
    - **ATTITUDE_MODE** (“Assist”)  
    - **RATE_MODE** (“Stabilized”)  
    - **MANUAL_MODE** (direct passthrough)

### 5. Telemetry & CLI

- **Multiple Backends**  
  - **MQTT** (WiFi + PubSubClient)  
  - **Debug Serial** (1 Hz)  
  - **Q-Serial** (10 Hz quaternion)  
  - **CRSF Telemetry** (25 Hz burst + 1 Hz slow frames) — native to ELRS/Crossfire.  
- **CRSF Telemetry Highlights**  
  - Dedicated FreeRTOS task at user-set rate.  
  - Fast (“vital”) frames each loop: Link‐Stats, Vario, Attitude.  
  - Slow (1 Hz) frames: Battery, GPS, Flight Mode.  
  - Auto-throttling of non-vitals until we add proper `SUBSCRIBE_TELEMETRY` support.
- **Built-in CLI**  
  - Real-time tuning: `setmode`, `calibrate imu`, `stats`, `tasks`, etc.  
  - Full access to loop-timing statistics and active FreeRTOS tasks.

## Architecture

ArduFlite employs a cascade control structure:

1. **Attitude Controller (Outer Loop):**  
    Runs at approximately 100Hz and translates attitude errors (derived from IMU quaternions) into desired angular rate setpoints.

2. **Rate Controller (Inner Loop):**  
    Runs at approximately 500Hz to compute and output servo commands based on measured angular rates and the desired rate setpoints provided by the outer loop or direct pilot input.

3. **Servo Manager:**  
    Maps normalized control outputs to real servo angles, supporting various wing designs and enabling configurable control surface behavior.

4. **Telemetry & CLI:**  
   Multiple telemetry modules are provided to suit different monitoring needs (MQTT, debug Serial, and high-frequency Serial). A dedicated CLI task supports real-time data queries, dynamic parameter adjustments, and troubleshooting commands.

## 🔧 Installation & Setup

### Hardware Requirements

- ESP32 microcontroller
- IMU sensor (e.g., MPU-6500)
- Servos for control surfaces (ailerons, elevator, rudder)
- Additional components for telemetry (optional)
- ELRS receiver (Crossfire UART interface) and transmitter

### Software Requirements

- Arduino IDE or PlatformIO
- ESP32 core for Arduino (includes FreeRTOS)
- Libraries: FastIMU, ESP32Servo, Adafruit_Madgwick (or alternative sensor fusion library), PubSubClient (for MQTT), WiFiManager, etc.

### Setup Instructions

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/xanderwasserman/ArduFlite.git
    cd ArduFlite
    ```
2. Open the Project:
    Open the project in your preferred IDE (e.g., Arduino IDE or PlatformIO).

3. Configure Hardware & Parameters:
    Update pin assignments, PID gains, CRSF config, etc. in the configuration header files (e.g., ControllerConfiguration.h, CLICommandsConfig.h) as needed.

4. Upload the Code:
    Compile and upload the firmware to your ESP32.

### Usage
Once the system is running:
- Telemetry:
    Choose your telemetry method:

    - MQTT Telemetry: For remote monitoring and dashboard integration. Typically used in conjunction with a laptop or Raspberry Pi with a WiFi AP.

    - Debug Serial Telemetry: For low-frequency logging and debugging.

    - Q Serial Telemetry: For high-frequency, detailed real-time quaternion data output, for use with the visualiser.

    - CRSF Telemetry: For sending telemetry directly to your transmitter using the ELRS receiver's uplink.

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

- Control Operation:
    The attitude controller continuously computes new setpoints (from pilot input or test sequences), and the rate controller maintains stable flight even in the presence of disturbances.

# Contributing
Contributions are welcome! Whether you're adding new telemetry modules, enhancing the CLI, or improving control algorithms, please fork the repository and submit a pull request. For major enhancements, open an issue to discuss your ideas.

# License
This project is licensed under the MIT License – see the LICENSE file for details.

## Happy Flying with ArduFlite!

# References
- [FireBeetle 2 ESP32-E](https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654#target_3)
- [Adafruit_AHRS](https://github.com/adafruit/Adafruit_AHRS/tree/master)
- [FastIMU](https://github.com/LiquidCGS/FastIMU/tree/main)
