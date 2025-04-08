#include <Arduino.h>

#include "ArduFlite.h"
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
#include "src/cli/ArduFliteCLI.h"

void printControlLoopStats();
void onCalibrateHold();
void onModeDoubleTap();
void onResetTripleTap();

// Global telemetry objects.
TelemetryData               telemetryData;
ArduFliteMqttTelemetry      telemetry(20.0f);          // 20 Hz telemetry frequency
// ArduFliteDebugSerialTelemetry debugTelemetry(1.0f); // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry telemetry(20.0f);

// Create instances of the core components.
ArduFliteIMU                myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController     rateController;

// Define the servo configurations.
ServoConfig pitchCfg    = { PITCH_PIN,      500, 2500, 90, 70, false };
ServoConfig yawCfg      = { YAW_PIN,        500, 2500, 90, 70, false };
ServoConfig leftAilCfg  = { LEFT_AIL_PIN,   500, 2500, 90, 70, false };
ServoConfig rightAilCfg = { RIGHT_AIL_PIN,  500, 2500, 90, 70, true };

// Instantiate the ServoManager for a conventional wing design with dual ailerons.
ServoManager servoMgr(CONVENTIONAL, pitchCfg, yawCfg, leftAilCfg, rightAilCfg, true);
ArduFliteController arduflite(&myIMU, &attitudeController, &rateController, &servoMgr);

ArduFliteCLI myCLI(&arduflite);

HoldButton calibrateButton(USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);
MultiTapButton modeButton(USER_BUTTON_PIN, 1000, 2, onModeDoubleTap, true, 30);

void setup() 
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

  telemetry.begin();
  // debugTelemetry.begin();

  // Initialize the IMU.
  if (!myIMU.begin()) 
  {
      Serial.println("IMU failed to init!");
      while (1);
  }

  // Start with a level attitude (Assist mode).
  arduflite.setDesiredEulerDegs(-2.0f, 0.0f, 0.0f);
  arduflite.setPilotRateSetpoints(0.0f, 0.0f, 0.0f);

  // Set the mode (default is ASSIST_MODE).
  arduflite.setMode(ASSIST_MODE);

  // Start the overall control tasks.
  arduflite.startTasks();

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
    if (currentState != lastState) {
      switch (currentState) {
          case PREFLIGHT:
              Serial.println("Aircraft is in PREFLIGHT state.");
              break;
          case INFLIGHT:
              Serial.println("Aircraft is in FLIGHT state.");
              break;
          case LANDED:
              Serial.println("Aircraft has LANDED.");
              break;
          default:
              break;
      }
      lastState = currentState;
  }

  // Run the attitude test sequence only when in-flight.
  // if (currentState == INFLIGHT) {
  //     runAttitudeTest_wiggle(arduflite);
  // }
  runAttitudeTest_wiggle(arduflite); //TODO: for bench testing
    
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
    myIMU.selfCalibrate();
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