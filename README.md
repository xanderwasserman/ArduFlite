# ArduFlite

ArduFlite is a highly modular and real-time flight control framework designed for small unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 platform with FreeRTOS, ArduFlite integrates sensor fusion from an IMU (such as the MPU-6500), cascade PID controllers for robust attitude and rate control, and a customizable command line interface (CLI) for live diagnostics and dynamic configuration.

## Features

- **Modular Control Architecture**
    - **Attitude Control (Outer Loop ~100Hz):**  
        Computes desired angular rates by comparing the desired attitude (commanded via setpoints) with the current attitude derived from fused IMU data.
    - **Rate Control (Inner Loop ~500Hz):**  
        Uses high-bandwidth PID controllers to convert the desired angular rates into servo commands, accommodating both high disturbance rejection and fine setpoint tracking.
    - **Servo Manager:**  
        Configurable to support multiple wing designs—including conventional, delta wing, and V-tail configurations—with adjustable limits and inversion options.
  
- **Telemetery Modules**
  - **ArduFliteMqttTelemetry:**  
    Publishes telemetry data over MQTT, enabling remote monitoring and integration with dashboards or cloud services. It supports WiFi configuration and non-volatile storage for settings.  
  - **ArduFliteDebugSerialTelemetry:**  
    Offers a low-frequency (default 1 Hz) Serial interface for debugging purposes, ideal for simple validation without overwhelming serial output.  
  - **ArduFliteQSerialTelemetry:**  
    Provides a higher-frequency (default 10 Hz) Serial telemetry interface to deliver quaternion data for near-realtime attitude visualisation.  

  - **TelemetryData Structure:**  
    Consolidates key sensor and control information (accelerometer, gyroscope, quaternion, Euler angles, command outputs, and flight state) into a single data structure that feeds into all telemetry modules.  

- **Real-Time Scheduling & Diagnostics**
  - Uses `vTaskDelayUntil()` to enforce strict loop timings in both control loops.
  - Provides robust statistical tracking of control loop execution (rolling averages, maximum loop execution times, and overrun counts) which can be accessed via the CLI.
  - Supports a dedicated CLI for on-demand diagnostics, configuration, and telemetry retrieval.

- **Flexible and Dynamic Configuration**
  - Centralized configuration headers (e.g., `ControllerConfig.h`) allow easy tuning of PID gains and system parameters.
  - Dynamic, real-time adjustments are available through a modular CLI that supports commands like `setmode`, `calibrate`, and various telemetry queries.


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

## Installation

### Hardware Requirements

- ESP32 microcontroller
- IMU sensor (e.g., MPU-6500)
- Servos for control surfaces (ailerons, elevator, rudder)
- Additional components for telemetry (optional)

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
    Update pin assignments and PID gains in the configuration header files (e.g., ControllerConfig.h, CLICommandsConfig.h) as needed.

4. Upload the Code:
    Compile and upload the firmware to your ESP32.

### Usage
Once the system is running:
- Telemetry:
    Choose your telemetry method:

    - MQTT Telemetry: For remote monitoring and dashboard integration. Typically used in conjunction with a laptop or Raspberry Pi with a WiFi AP.

    - Debug Serial Telemetry: For low-frequency logging and debugging.

    - Q Serial Telemetry: For high-frequency, detailed real-time quaternion data output, for use with the visualiser.

- CLI Access:
    Open the Serial Monitor at 115200 baud. Type `help` to see a list of available commands.

    Example commands:

    - `help` – Lists available commands with descriptions.

    - `stats` – Displays current control loop timing statistics.

    - `tasks` – Shows FreeRTOS task statistics.

    - `setmode assist` – Switches the controller to ASSIST_MODE.

    - `setmode stabilized` – Switches the controller to STABILIZED_MODE.

    - `calibrate imu` – Triggers an IMU self-calibration routine.

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
