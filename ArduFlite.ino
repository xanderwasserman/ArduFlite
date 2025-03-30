#include <Arduino.h>

#include "ArduFlite.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/utils/PrintTask.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteController.h"
#include "src/actuators/ServoManager.h"
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"

unsigned long lastMicros = 0;
float rollCmd, pitchCmd, yawCmd = 0;

TelemetryData telemetryData;
ArduFliteMqttTelemetry telemetry(20.0f); // 20 Hz telemetry frequency
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

float getRollCmd() {
  return rollCmd;
}
float getPitchCmd() {
  return pitchCmd;
}
float getYawCmd() {
  return yawCmd;
}

HoldButton calibrateButton(USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(USER_BUTTON_PIN, INPUT_PULLUP);

  telemetry.begin();

  // Initialize IMU
  if (!myIMU.begin()) {
    Serial.println("IMU failed to init!");
    while (1);
  }

  // FliteQuaternion firstQ(1, 0, 0, 0);
  // myController.setDesiredQuaternion(firstQ);
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
  const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2 ms period for 500Hz update rate

  // Calculate delta time
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;

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
  telemetryData.accelX = myIMU.getAccelX();
  telemetryData.accelY = myIMU.getAccelY();
  telemetryData.accelZ = myIMU.getAccelZ();

  telemetryData.gyroX = myIMU.getGyroX();
  telemetryData.gyroY = myIMU.getGyroY();
  telemetryData.gyroZ = myIMU.getGyroZ();

  telemetryData.qw = myIMU.getQw();
  telemetryData.qx = myIMU.getQx();
  telemetryData.qy = myIMU.getQy();
  telemetryData.qz = myIMU.getQz();

  telemetryData.pitch = myIMU.getPitch();
  telemetryData.roll = myIMU.getRoll();
  telemetryData.yaw = myIMU.getYaw();

  telemetryData.rollCmd = rollCmd;
  telemetryData.pitchCmd = pitchCmd;
  telemetryData.yawCmd = yawCmd;

  telemetry.publish(telemetryData);

  // Wait until next cycle
  vTaskDelayUntil(&xLastWakeTime, xFrequency);
}