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

// Global telemetry objects.
TelemetryData telemetryData;
ArduFliteMqttTelemetry telemetry(20.0f);          // 20 Hz telemetry frequency
// ArduFliteDebugSerialTelemetry debugTelemetry(1.0f); // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry telemetry(20.0f);

// Create instances of the core components.
ArduFliteIMU myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController rateController;

// Define the servo configurations.
ServoConfig pitchCfg    = { PITCH_PIN, 500, 2500, 90, 70, false };
ServoConfig yawCfg      = { YAW_PIN, 500, 2500, 90, 70, false };
ServoConfig leftAilCfg  = { LEFT_AIL_PIN, 500, 2500, 90, 70, false };
ServoConfig rightAilCfg = { RIGHT_AIL_PIN, 500, 2500, 90, 70, false };

// Instantiate the ServoManager for a conventional wing design with dual ailerons.
ServoManager servoMgr(CONVENTIONAL, pitchCfg, yawCfg, leftAilCfg, rightAilCfg, true);
ArduFliteController arduflite(&myIMU, &attitudeController, &rateController, &servoMgr);

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
  arduflite.setDesiredEulerDegs(0.0, 0.0, 0.0);
  arduflite.setPilotRateSetpoints(0.0, 0.0, 0.0);

  // Set the mode (default is ASSIST_MODE).
  arduflite.setMode(ASSIST_MODE);

  // Start the overall control tasks.
  arduflite.startTasks();

  // Initialize and register buttons.
  calibrateButton.begin();
  resetButton.begin();
  modeButton.begin();

  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");

  MultiTapButtonManager::registerButton(resetButton);
  Serial.println("Triple tap the button at any time to reset the telemetry layer.");

  MultiTapButtonManager::registerButton(modeButton);
  Serial.println("Double tap the button at any time to change the ArduFlite Mode.");

  Serial.println("ArduFlite Controller initialised.");
}

void loop() 
{
  // Main loop can handle telemetry and button updates.
  HoldButtonManager::updateAll();
  MultiTapButtonManager::updateAll();

  // Update telemetry data with the latest IMU and control information.
  telemetryData.update(myIMU, arduflite.getRollRateCmd(), arduflite.getPitchRateCmd(), arduflite.getYawRateCmd(), arduflite.getRollCmd(), arduflite.getPitchCmd(), arduflite.getYawCmd());
  telemetry.publish(telemetryData);
  // debugTelemetry.publish(telemetryData);

  //TODO: update pilot setpoints via arduflite.setPilotRateSetpoints(...) or arduflite.setDesiredEulerDegs(...)

  vTaskDelay(pdMS_TO_TICKS(10));
}
