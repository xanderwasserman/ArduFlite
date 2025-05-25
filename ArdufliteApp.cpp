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
#include "include/ReceiverConfiguration.h"
#include "include/PinConfiguration.h"
#include "include/ControllerConfiguration.h"
#include "include/ServoConfiguration.h"

#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/StatusLED.h"
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
#include "src/mission_planner/MissionPlanner.h"

void printControlLoopStats(void);
void onCalibrateHold(void);
void onModeDoubleTap(void);
void onResetTripleTap(void);
void resetSystemCommand(void);
void pauseController(void);
void resumeController(void);

// Declare system commands instnaces
CommandSystem commandSystem;

// Declare telemetry instnaces.
TelemetryData               telemetryData;
ArduFliteMqttTelemetry      telemetry(20.0f, &commandSystem);          // 20 Hz telemetry frequency
ArduFliteFlashTelemetry     flashTelemetry(50.0f);  // 50 Hz logging
// ArduFliteDebugSerialTelemetry    debugTelemetry(1.0f);               // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry        telemetry(20.0f);

// Declare instances of the core components.
ArduFliteIMU                myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController     rateController;

// Instantiate the ServoManager for a conventional wing design with dual ailerons.
ServoManager servoMgr(CONVENTIONAL, ServoSetupConfig::PITCH_CONFIG, ServoSetupConfig::YAW_CONFIG, ServoSetupConfig::LEFT_AIL_CONFIG, ServoSetupConfig::RIGHT_AIL_CONFIG, true);

// Instantiate the Controllers
ArduFliteController controller(&myIMU, &attitudeController, &rateController, &servoMgr);

// Instantiate the Receiver
ArduFlitePwmReceiver pilotReceiver(ReceiverSetupConfig::RECEIVER_CHANNELS, ReceiverSetupConfig::NR_RECEIVER_CHANNELS);

// Instantiate the CLI
ArduFliteCLI myCLI(&controller, &myIMU);

//Instantiate mission planner
MissionPlanner mission(controller);

// Instantiate the User Buttons
HoldButton calibrateButton(ButtonInputConfig::USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);
MultiTapButton modeButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 2, onModeDoubleTap, true, 30);

#if BOARD_TYPE == BOARD_TYPE_WEMOS
StatusLED statusLED(7, 1);
#endif

void arduflite_init() 
{
#if BOARD_TYPE == BOARD_TYPE_WEMOS
    statusLED.begin();
    statusLED.setPattern(Patterns::Boot);
#endif

    Serial.begin(115200);
    while (!Serial && millis() < 2000);  // wait max 2 seconds

    pinMode(ButtonInputConfig::USER_BUTTON_PIN, INPUT_PULLUP);

    telemetry.begin();
    flashTelemetry.begin();
    // debugTelemetry.begin();

    // Initialize the IMU.
    if (!myIMU.begin()) 
    {
        Serial.println("IMU failed to init!");
#if BOARD_TYPE == BOARD_TYPE_WEMOS
        statusLED.setPattern(Patterns::Error); //fast yellow blink
#endif
        while (1);
    }

    servoMgr.testControlSurfaces();

    // Start with a level attitude (Assist mode).
    controller.setDesiredEulerDegs(0.0f, 0.0f, 0.0f);
    controller.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);

    // Set the mode (default is ASSIST_MODE).
    controller.setMode(ASSIST_MODE);

    // Start the overall control tasks.
    controller.startTasks();

    // Start the PWM signal receiver.
    pilotReceiver.begin();

    // Start the CLI task.
    myCLI.startTask();

    // Start the Mission Planner
    mission.begin();

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

void arduflite_loop() 
{
    // Process any pending commands.
    commandSystem.processCommands(&controller, &myIMU);
    
    // Update buttons.
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    // Retrieve the current flight state from the IMU.
    static ArduFliteMode lastMode = UNKNOWN_MODE;
    ArduFliteMode currentMode = controller.getMode();

    if (currentMode != lastMode) 
    {
        switch (currentMode) 
        {
            case ASSIST_MODE:
                #if BOARD_TYPE == BOARD_TYPE_WEMOS
                statusLED.setPattern(Patterns::Assist);
                #endif
                break;
            case STABILIZED_MODE:
                #if BOARD_TYPE == BOARD_TYPE_WEMOS
                statusLED.setPattern(Patterns::Stabilized);
                #endif
                break;
            default:
                break;
        }
        lastMode = currentMode;
    }

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
                controller.setDesiredEulerDegs(0.0f, 0.0f, 0.0f);
                controller.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);
                controller.pauseTasks();     // Pause control loop tasks
                break;
            case INFLIGHT:
                Serial.println("Aircraft is in FLIGHT state.");
                controller.resumeTasks();    // Resume control loop tasks
                
                if (!mission.isRunning())
                {
                    mission.start();
                }
                
                break;
            case LANDED:
                Serial.println("Aircraft has LANDED.");
                if (mission.isRunning())
                {
                    mission.stop();
                }

                controller.setDesiredEulerDegs(0.0f, 0.0f, 0.0f);
                controller.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);
                controller.pauseTasks();     // Pause control loop tasks
                break;
            default:
                break;
        }
        lastState = currentState;
    }

    //TODO
    // runReceiverTest_print(pilotReceiver, numReceiverChannels);
        
    // Update telemetry with the latest sensor and control information.
    telemetryData.update(myIMU, controller);

    telemetry.publish(telemetryData);

    vTaskDelay(pdMS_TO_TICKS(10));
}

// Callback for calibrate button.
void onCalibrateHold(void) 
{
#if BOARD_TYPE == BOARD_TYPE_WEMOS
    statusLED.setPattern({0,0,255, 100,100});// fast blue blink
#endif
    Serial.println("Calibrating IMU...");
    controller.pauseTasks();     // Pause control loop tasks
    myIMU.pauseTask();           // Pause the IMU update task
    myIMU.selfCalibrate();       // Run calibration
    myIMU.resumeTask();          // Resume the IMU update task
    controller.resumeTasks();    // Resume control loop tasks
#if BOARD_TYPE == BOARD_TYPE_WEMOS
    statusLED.setPattern({0,255,0, 500,150});// slow green blink
#endif
}

// Callback for telemetry reset button.
void onModeDoubleTap(void) 
{
    Serial.println("Toggling Controller Mode...");

    if (controller.getMode() == ASSIST_MODE) 
    {
      controller.setMode(STABILIZED_MODE);
    } 
    else 
    {
      controller.setMode(ASSIST_MODE);
    }
    
}

// Callback for telemetry reset button.
void onResetTripleTap(void) 
{
    Serial.println("Resetting Telemetry layer...");
    telemetry.reset();
}