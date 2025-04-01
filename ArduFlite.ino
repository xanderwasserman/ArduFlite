#include <Arduino.h>

#include "ArduFlite.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteController.h"
#include "src/actuators/ServoManager.h"
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"

unsigned long lastMicros = 0;
float rollCmd, pitchCmd, yawCmd = 0;

TelemetryData telemetryData;
ArduFliteMqttTelemetry telemetry(20.0f); // 20 Hz telemetry frequency
ArduFliteDebugSerialTelemetry debugTelemetry(1.0f); // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry telemetry(20.0f);

ArduFliteIMU myIMU;
ArduFliteController myController;
ServoManager servoMgr(LEFT_AIL_PIN, RIGHT_AIL_PIN, PITCH_PIN, YAW_PIN);

// Callback for calibrate button
void onCalibrateHold() {
    Serial.println("Calibrating IMU...");
    myIMU.selfCalibrate();
}

// Callback for telemetry reset button
void onResetTripleTap() {
    Serial.println("Resetting Telemetry layer...");
    telemetry.reset();
}

HoldButton calibrateButton(USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

  telemetry.begin();
  debugTelemetry.begin();

  // Initialize IMU
  if (!myIMU.begin()) {
    Serial.println("IMU failed to init!");
    while (1);
  }

  myController.setDesiredEulerDegs(0.0, 0.0, 0.0);

  Serial.printf("ArduFlite Controller initialised\n");

  calibrateButton.begin();
  resetButton.begin();
  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");
  MultiTapButtonManager::registerButton(resetButton);
  Serial.println("Triple tap the button at any time to reset the telemetry layer.");

  startPrintTask();
}

void loop() 
{
    static TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the last wake time
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MILLIS); // 2 ms period for 500Hz update rate

    // Calculate delta time
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;

    // Check if dt exceeds LOOP_PERIOD_MILLIS (converted to seconds)
    if (dt > (LOOP_PERIOD_MILLIS / 1000.0f)) {
      Serial.print("Warning: Loop time exceeded! dt = ");
      Serial.print(dt, 6);
      Serial.println(" seconds");
    }

    // 1) Update all buttons in one shot
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    // 2) Update sensor data
    myIMU.update(dt);

    // get the current quaternion from the IMU
    FliteQuaternion currentQ(
      myIMU.getQw(), 
      myIMU.getQx(), 
      myIMU.getQy(), 
      myIMU.getQz()
      );

    // run the controller
    myController.update(currentQ, dt, rollCmd, pitchCmd, yawCmd);

    // send commands to servos
    servoMgr.writeCommands(rollCmd, pitchCmd, yawCmd);

    // Update telemetry data
    telemetryData.update(myIMU, rollCmd, pitchCmd, yawCmd);

    // Publish telemetry data to the telemetry tasks
    telemetry.publish(telemetryData);
    debugTelemetry.publish(telemetryData);

    // Wait until next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
}