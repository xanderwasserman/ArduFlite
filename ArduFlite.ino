/*
 * ----------------------------------------------------------------------------
 * ArduFlite - Advanced Flight Controller Framework
 * ----------------------------------------------------------------------------
 *
 * Project: ArduFlite
 * Description: 
 *   ArduFlite is a highly modular and real-time flight control framework for 
 *   unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 and FreeRTOS,
 *   it integrates IMU sensor fusion, cascade PID control (attitude and rate loops),
 *   servo management for multiple wing designs, and dynamic telemetry (via MQTT
 *   and Serial) along with a flexible command-line interface (CLI) for live diagnostics 
 *   and configuration.
 *
 * Author: Alexander Wasserman
 * Version: 1.0
 * Date: 08 April 2025
 *
 * License: MIT License
 * ----------------------------------------------------------------------------
 */
#include <Arduino.h>

#include "include/ArduFlite.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/actuators/ServoManager.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/controller/ArduFliteController.h"
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"
#include "src/tests/AttitudeTests.h"
#include "src/tests/ReceiverTests.h"
#include "src/cli/ArduFliteCLI.h"
#include "src/receiver/ArduFlitePwmReceiver.h"

void printControlLoopStats();
void onCalibrateHold();
void onModeDoubleTap();
void onResetTripleTap();
void resetSystemCommand();

// Global telemetry objects.
TelemetryData               telemetryData;
ArduFliteMqttTelemetry      telemetry(20.0f);          // 20 Hz telemetry frequency
// ArduFliteDebugSerialTelemetry    debugTelemetry(1.0f); // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry        telemetry(20.0f);

// Create instances of the core components.
ArduFliteIMU                myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController     rateController;

// Define the servo configurations.
ServoConfig pitchCfg    = { PwmOutputConfig::PITCH_PIN,      500, 2500, 90, 70, false };
ServoConfig yawCfg      = { PwmOutputConfig::YAW_PIN,        500, 2500, 90, 70, false };
ServoConfig leftAilCfg  = { PwmOutputConfig::LEFT_AIL_PIN,   500, 2500, 90, 70, true };
ServoConfig rightAilCfg = { PwmOutputConfig::RIGHT_AIL_PIN,  500, 2500, 90, 70, false };

// Instantiate the ServoManager for a conventional wing design with dual ailerons.
ServoManager servoMgr(CONVENTIONAL, pitchCfg, yawCfg, leftAilCfg, rightAilCfg, true);
ArduFliteController arduflite(&myIMU, &attitudeController, &rateController, &servoMgr);

// Define configurations for each channel.
ReceiverChannelConfig receiverConfigs[] = {
    { PwmInputConfig::ROLL_INPUT_PIN,       1000UL, 2000UL, BIPOLAR },  // Roll channel on pin 34
    { PwmInputConfig::PITCH_INPUT_PIN,      1000UL, 2000UL, BIPOLAR },  // Pitch channel on pin 35
    { PwmInputConfig::YAW_INPUT_PIN,        1000UL, 2000UL, BIPOLAR },  // Yaw channel on pin 36
    { PwmInputConfig::THROTTLE_INPUT_PIN,   1000UL, 2000UL, UNIPOLAR }  // Throttle channel on pin 39
};
const size_t numReceiverChannels = sizeof(receiverConfigs) / sizeof(receiverConfigs[0]);

ArduFlitePwmReceiver pilotReceiver(receiverConfigs, numReceiverChannels);

ArduFliteCLI myCLI(&arduflite, &myIMU);

HoldButton calibrateButton(ButtonInputConfig::USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);
MultiTapButton modeButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 2, onModeDoubleTap, true, 30);

void setup() 
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(ButtonInputConfig::USER_BUTTON_PIN, INPUT_PULLUP);

    servoMgr.testControlSurfaces();

    telemetry.registerCalibrateCallback(onCalibrateHold);
    telemetry.registerResetCallback(resetSystemCommand);
    telemetry.begin();
    // debugTelemetry.begin();

    // Initialize the IMU.
    if (!myIMU.begin()) 
    {
        Serial.println("IMU failed to init!");
        while (1);
    }

    // Start with a level attitude (Assist mode).
    arduflite.setDesiredEulerDegs(-1.0f, 0.0f, 0.0f);
    arduflite.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);

    // Set the mode (default is ASSIST_MODE).
    arduflite.setMode(ASSIST_MODE);

    // Start the overall control tasks.
    arduflite.startTasks();

    // Start the PWM signal receiver.
    pilotReceiver.begin();

    // Start the CLI task.
    myCLI.startTask();

    // Initialize and register buttons.
    calibrateButton.begin();
    resetButton.begin();
    modeButton.begin();

    Serial.println("Available Button Functions:");

    HoldButtonManager::registerButton(calibrateButton);
    Serial.println("  3s hold - calibrate IMU.");

    MultiTapButtonManager::registerButton(resetButton);
    Serial.println("  3x tap - reset telemetry layer.");

    MultiTapButtonManager::registerButton(modeButton);
    Serial.println("  2x tap - toggle ArduFlite Mode.");

    Serial.println("ArduFlite Controller initialised.");
}

void loop() 
{
    // Update buttons.
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    // Retrieve the current flight state from the IMU.
    static FlightState lastState = UNKNOWN_STATE;
    FlightState currentState = myIMU.getFlightState();
    
    // Print transitions when they occur.
    if (currentState != lastState) 
    {
        switch (currentState) 
        {
            case PREFLIGHT:
                Serial.println("Aircraft is in PREFLIGHT state.");
                arduflite.setDesiredEulerDegs(-1.0f, 0.0f, 0.0f);
                arduflite.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);
                break;
            case INFLIGHT:
                Serial.println("Aircraft is in FLIGHT state.");
                // TODO: when entering flight, set the current heading as the yaw setpoint
                break;
            case LANDED:
                Serial.println("Aircraft has LANDED.");
                arduflite.setDesiredEulerDegs(-1.0f, 0.0f, 0.0f);
                arduflite.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);
                break;
            default:
                break;
        }
        lastState = currentState;
    }

    /* //TODO
    // Run the attitude test sequence only when in-flight.
    if (currentState == INFLIGHT) 
    {
        runAttitudeTest_wiggle(arduflite, 15, 2000);
    }
    */

    /* //TODO
    runReceiverTest_print(pilotReceiver, numReceiverChannels);
    */
        
    // Update telemetry with the latest sensor and control information.
    telemetryData.update(myIMU,
                            arduflite.getRollRateCmd(), arduflite.getPitchRateCmd(), arduflite.getYawRateCmd(),
                            arduflite.getRollCmd(), arduflite.getPitchCmd(), arduflite.getYawCmd(),
                            currentState);

    telemetry.publish(telemetryData);

    vTaskDelay(pdMS_TO_TICKS(10));
}

// Callback for calibrate button.
void onCalibrateHold() 
{
    Serial.println("Calibrating IMU...");
    arduflite.pauseTasks();     // Pause control loop tasks
    myIMU.pauseTask();           // Pause the IMU update task
    myIMU.selfCalibrate();       // Run calibration
    myIMU.resumeTask();          // Resume the IMU update task
    arduflite.resumeTasks();    // Resume control loop tasks
}

// Callback for telemetry reset button.
void onModeDoubleTap() 
{
    Serial.println("Toggling Controller Mode...");

    if (arduflite.getMode() == ASSIST_MODE) 
    {
      arduflite.setMode(STABILIZED_MODE);
    } 
    else 
    {
      arduflite.setMode(ASSIST_MODE);
    }
    
}

// Callback for telemetry reset button.
void onResetTripleTap() 
{
    Serial.println("Resetting Telemetry layer...");
    telemetry.reset();
}

void resetSystemCommand() 
{
    Serial.println("Executing reset callback...");
    ESP.restart();
}