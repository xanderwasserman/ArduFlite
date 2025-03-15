#include <Arduino.h>
#include "HoldButton.h"
#include "HoldButtonManager.h"
#include "ArduFliteIMU.h"
#include "ArduFliteController.h"   // quaternion-based controller
#include "ServoManager.h"          // servo driver
#include "FliteQuaternion.h"

#define PRINT_EVERY_N_UPDATES 50

ArduFliteIMU myIMU;
ArduFliteController myController;

#define LEFT_AIL_PIN    17
#define RIGHT_AIL_PIN   16
#define PITCH_PIN       4
#define YAW_PIN         12

ServoManager servoMgr(LEFT_AIL_PIN, RIGHT_AIL_PIN, PITCH_PIN, YAW_PIN);


// Callback for calibrate button
void onCalibrateHold() {
    Serial.println("Calibrating IMU...");
    myIMU.selfCalibrate();
}

// Calibration Button definition
#define CALIB_BUTTON_PIN 27
#define CALIB_HOLD_TIME 3000

HoldButton calibrateButton(CALIB_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);

unsigned long lastMicros = 0;
int print_counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  // Note: INPUT_PULLUP means the pin is HIGH when not pressed,
  // and LOW when pressed (assuming the other side of the button goes to GND).

  // Initialize IMU
  if (!myIMU.begin()) {
    Serial.println("IMU failed to init!");
    while (1);
  }

  // get the current quaternion from the IMU
  // Let the IMU filter settle for a few updates
  for (int i=0; i<10; i++) {
    myIMU.update(0.01f); // or real dt if you prefer
    delay(10);
  }
  FliteQuaternion firstQ(
    myIMU.getQw(), 
    myIMU.getQx(), 
    myIMU.getQy(), 
    myIMU.getQz()
    );

  myController.setDesiredOrientation(firstQ);

  calibrateButton.begin();
  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");
}

void loop() {

  // Calculate delta time
  unsigned long currentMicros = micros();
  float dt = (currentMicros - lastMicros) / 1000000.0f;
  lastMicros = currentMicros;

  // 1) Update all buttons in one shot
  HoldButtonManager::updateAll();

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

  if (print_counter++ >= PRINT_EVERY_N_UPDATES) {
    
    Serial.print("Acceleration: ");
    Serial.print(myIMU.getAccelX());
    Serial.print(", ");
    Serial.print(myIMU.getAccelY());
    Serial.print(", ");
    Serial.println(myIMU.getAccelZ());

    // Serial.print("Gyroscope: ");
    // Serial.print(myIMU.getGyroX());
    // Serial.print(",");
    // Serial.print(myIMU.getGyroY());
    // Serial.print(",");
    // Serial.println(myIMU.getGyroZ())
    
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
  
  delay(10); // ~100 Hz print
}
