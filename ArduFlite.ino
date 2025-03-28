#include <Arduino.h>
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteController.h"
#include "src/actuators/ServoManager.h"
#include "src/telemetry/mqtt/ArduFliteMqttTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"

#define PRINT_EVERY_N_UPDATES 50
#define LOOP_PERIOD_MICROS    2000  // 2 ms for 500 Hz

#define LEFT_AIL_PIN    17
#define RIGHT_AIL_PIN   16
#define PITCH_PIN       4
#define YAW_PIN         12

// Calibration Button definition
#define USER_BUTTON_PIN 27
#define CALIB_HOLD_TIME 3000

TelemetryData telemetryData;
ArduFliteMqttTelemetry telemetry(10.0f); // 10 Hz telemetry frequency
// ArduFliteQSerialTelemetry telemetry(10.0f);

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

unsigned long lastMicros = 0;
int print_counter = 0;

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
  myController.setDesiredEulerDegs(5.0, 0.0, 0.0);

  Serial.printf("ArduFlite Controller initialised\n");

  calibrateButton.begin();
  resetButton.begin();
  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");
  MultiTapButtonManager::registerButton(resetButton);
  Serial.println("Triple tap the button at any time to reset the telemetry layer.");
}

void loop() 
{
  static unsigned long nextCycle = micros(); // initialize next cycle time

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
  float rollCmd, pitchCmd, yawCmd;
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

  if (print_counter++ >= PRINT_EVERY_N_UPDATES) 
  {
    
    // Serial.print("Acceleration: ");
    // Serial.print(myIMU.getAccelX());
    // Serial.print(", ");
    // Serial.print(myIMU.getAccelY());
    // Serial.print(", ");
    // Serial.println(myIMU.getAccelZ());

    // Serial.print("Gyroscope: ");
    // Serial.print(myIMU.getGyroX());
    // Serial.print(",");
    // Serial.print(myIMU.getGyroY());
    // Serial.print(",");
    // Serial.println(myIMU.getGyroZ());
    
    // Serial.print("Quarternion: ");
    // Serial.print(myIMU.getQw());
    // Serial.print(", ");
    // Serial.print(myIMU.getQx());
    // Serial.print(", ");
    // Serial.print(myIMU.getQy());
    // Serial.print(", ");
    // Serial.println(myIMU.getQz());

    Serial.print("Pitch: ");
    Serial.print(myIMU.getPitch());
    Serial.print(" Roll: ");
    Serial.print(myIMU.getRoll());
    Serial.print(" Yaw: ");
    Serial.println(myIMU.getYaw());

    Serial.print(" -> rollCmd: "); Serial.print(rollCmd);
    Serial.print(" pitchCmd: "); Serial.print(pitchCmd);
    Serial.print(" yawCmd: "); Serial.println(yawCmd);

    print_counter = 0;
  }

  // Schedule next cycle: wait until micros() reaches nextCycle.
  nextCycle += LOOP_PERIOD_MICROS;
  while (micros() < nextCycle) {
    delayMicroseconds(10);
  }
}
