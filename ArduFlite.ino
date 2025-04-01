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
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"

unsigned long lastMicros = 0;
float rollRateCmd, pitchRateCmd, yawRateCmd = 0;
float rollCmd, pitchCmd, yawCmd = 0;

TelemetryData telemetryData;
ArduFliteMqttTelemetry telemetry(20.0f); // 20 Hz telemetry frequency
ArduFliteDebugSerialTelemetry debugTelemetry(1.0f); // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry telemetry(20.0f);

ArduFliteIMU myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController rateController;
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

// Outer loop task (attitude control): runs at ~100Hz (10 ms period)
void OuterLoopTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10 ms -> 100Hz

  static unsigned long lastMicros = micros();

  for (;;) {
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastMicros) / 1000000.0f;
    lastMicros = currentMicros;
    if (dt < 1e-3f) dt = 1e-3f; // Avoid very small dt

    // Read current orientation from the IMU
    FliteQuaternion currentQ = myIMU.getQuaternion();

    // Update the outer (attitude) controller.
    // Here, attitudeController outputs desired angular rates (e.g., in deg/s)
    attitudeController.update(currentQ, dt, rollRateCmd, pitchRateCmd, yawRateCmd);

    // Pass the desired rates to the inner loop rate controller
    rateController.setDesiredRates(rollRateCmd, pitchRateCmd, 0); //note: no yaw control, only yaw stabilisation

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Inner loop task (rate control): runs at ~500Hz (2 ms period)
void InnerLoopTask(void *parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2 ms -> 500Hz

  static unsigned long lastMicros = micros();

  for (;;) {
      unsigned long currentMicros = micros();
      float dt = (currentMicros - lastMicros) / 1000000.0f;
      lastMicros = currentMicros;
      if (dt < 1e-3f) dt = 1e-3f; // Avoid very small dt

      // Update IMU sensor data at high frequency
      myIMU.update(dt);

      // Get measured angular rates (in deg/s)
      Vector3 gyro = myIMU.getGyro();

      // Update the inner (rate) controller, which computes the final servo commands
      rateController.update(gyro.x, gyro.y, gyro.z, dt, rollCmd, pitchCmd, yawCmd); 

      // Send final servo commands (normalized to [-1, 1])
      servoMgr.writeCommands(rollCmd, pitchCmd, yawCmd);

      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

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

  // Initialize the outer loop controller with a level orientation.
  attitudeController.setDesiredEulerDegs(0.0, 0.0, 0.0);

  Serial.println("ArduFlite Controller initialised");

  // Initialize buttons and register them
  calibrateButton.begin();
  resetButton.begin();
  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");
  MultiTapButtonManager::registerButton(resetButton);
  Serial.println("Triple tap the button at any time to reset the telemetry layer.");

  // Create FreeRTOS tasks for the outer and inner loops.
  xTaskCreate(OuterLoopTask, "OuterLoop", 4096, NULL, 2, NULL);
  xTaskCreate(InnerLoopTask, "InnerLoop", 4096, NULL, 3, NULL);
}

void loop() {
  // Main loop can be left empty as tasks handle processing.
  HoldButtonManager::updateAll();
  MultiTapButtonManager::updateAll();

  // Update telemetry data
  telemetryData.update(myIMU, rollRateCmd, pitchRateCmd, yawRateCmd, rollCmd, pitchCmd, yawCmd);

  // Publish telemetry data to the telemetry tasks
  telemetry.publish(telemetryData);
  debugTelemetry.publish(telemetryData);

  vTaskDelay(pdMS_TO_TICKS(10));
}