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
#include "include/ArduFlite.h"
#include "include/ReceiverConfiguration.h"
#include "include/PinConfiguration.h"
#include "include/ControllerConfiguration.h"
#include "include/ServoConfiguration.h"
#include "include/CSRFConfiguration.h"
#include "include/ControlMixerConfiguration.h"

#include "src/actuators/ServoManager.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/cli/ArduFliteCLI.h"
#include "src/mission_planner/MissionPlanner.h"
#include "src/state/StateManagement.h"

#include "src/utils/ControlMixer.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/StatusLED.h"
#include "src/utils/ButtonCallbacks.h"
#include "src/utils/Logging.h"

#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/controller/ArduFliteController.h"

#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/ConfigData.h"
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"

#include "src/receiver/pwm/ArduFlitePwmReceiver.h"
#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"

#include "src/tests/AttitudeTests.h"
#include "src/tests/ReceiverTests.h"

#include <Arduino.h>

void printControlLoopStats(void);
void onCalibrateHold(void);
void onModeDoubleTap(void);
void onResetTripleTap(void);
void resetSystemCommand(void);
void pauseController(void);
void resumeController(void);

// Serial ports for CRSF
HardwareSerial crsfSerial(1);
HardwareSerial telemSerial(2);

ArdufliteCRSFReceiver  crsfRx(crsfSerial,  CRSFPinConfig::PIN_CRSF_RX);
ArdufliteCRSFTelemetry crsfTx(telemSerial, CRSFPinConfig::PIN_CRSF_TX);

// Declare telemetry instnaces.
TelemetryData               telemetryData;
ConfigData                  configData;
ArduFliteMqttTelemetry      telemetry(20.0f);           // 20 Hz telemetry frequency
ArduFliteFlashTelemetry     flashTelemetry(50.0f);                      // 50 Hz logging
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
ArduFliteCLI myCLI(&controller, &myIMU, &flashTelemetry);

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

    ControlMixer::init(controller);

    telemetry.begin();
    flashTelemetry.begin();
    // debugTelemetry.begin();

    // Initialize the IMU.
    if (!myIMU.begin()) 
    {
        LOG_ERR("IMU failed to init!");
#if BOARD_TYPE == BOARD_TYPE_WEMOS
        statusLED.setPattern(Patterns::Error); //fast yellow blink
#endif
        while (1);
    }

    servoMgr.testControlSurfaces();

    // Start with a level attitude (Assist mode).
    EulerAngles initialSetpoint {0.0f};
    controller.setAttitudeSetpoint(initialSetpoint);
    controller.setRateSetpoint(initialSetpoint);

    // Set the mode (default is ATTITUDE_MODE).
    controller.setMode(ATTITUDE_MODE);

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

    LOG_INF("Available Button Functions:");

    HoldButtonManager::registerButton(calibrateButton);
    LOG_INF("  3s hold - calibrate IMU.");

    MultiTapButtonManager::registerButton(resetButton);
    LOG_INF("  3x tap - reset telemetry layer.");

    MultiTapButtonManager::registerButton(modeButton);
    LOG_INF("  2x tap - toggle ArduFlite Mode.");

    crsfRx.begin();
    crsfTx.begin();

    // configure every channel in one loop
    for (uint8_t ch = 0; ch < 16; ++ch) {
        crsfRx.configureChannel(ch, CRSFConfig::CRSF_CHANNEL_CONFIGS[ch]);
    }

    crsfReceiver.setFailsafeCallback(CRSFCallbacks::onFailsafe);
    crsfReceiver.setFailsafeTimeout(500);

    LOG_INF("ArduFlite Controller initialised.");
}

void arduflite_loop() 
{
    // Process any pending commands.
    CommandSystem::instance().processCommands(&controller, &myIMU);
    
    // Update buttons.
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    handleModeState();
    handleFlightState();
        
    // Update telemetry with the latest sensor and control information.
    telemetryData.update(myIMU, controller, crsfRx);
    configData.update(controller);

    crsfTx.publish(telemetryData, configData);
    telemetry.publish(telemetryData, configData);
    flashTelemetry.publish(telemetryData, configData);

    vTaskDelay(pdMS_TO_TICKS(1));
}