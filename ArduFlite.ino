#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "HoldButton.h"
#include "HoldButtonManager.h"
#include "ArduFliteIMU.h"
#include "ArduFliteController.h"
#include "ServoManager.h"
#include "ArduFliteMqttTelemetry.h"
#include "TelemetryData.h"

#define MQTT_SERVER "192.168.100.14" // Your MQTT broker IP address
#define MQTT_PORT 1883
#define MQTT_TOPIC "arduflite/telemetry"

#define PRINT_EVERY_N_UPDATES 50

#define LEFT_AIL_PIN    17
#define RIGHT_AIL_PIN   16
#define PITCH_PIN       4
#define YAW_PIN         12

// Calibration Button definition
#define CALIB_BUTTON_PIN 27
#define CALIB_HOLD_TIME 3000

WiFiClient espClient;
PubSubClient mqttClient(espClient);
// 10 Hz telemetry frequency
ArduFliteMqttTelemetry telemetry(mqttClient, 10.0f); 
extern SemaphoreHandle_t telemetryMutex;
extern TelemetryData telemetryData;

ArduFliteIMU myIMU;
ArduFliteController myController;

ServoManager servoMgr(LEFT_AIL_PIN, RIGHT_AIL_PIN, PITCH_PIN, YAW_PIN);

// Callback for calibrate button
void onCalibrateHold() {
    Serial.println("Calibrating IMU...");
    myIMU.selfCalibrate();
}

HoldButton calibrateButton(CALIB_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect("ArduFliteESP32")) {
      Serial.println("connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

unsigned long lastMicros = 0;
int print_counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  // Note: INPUT_PULLUP means the pin is HIGH when not pressed,
  // and LOW when pressed (assuming the other side of the button goes to GND).

  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("ArduFliteAP")) {
    Serial.println("Failed to connect WiFi and hit timeout");
    ESP.restart();
  }

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  telemetryMutex = xSemaphoreCreateMutex();
  telemetry.begin();

  // Initialize IMU
  if (!myIMU.begin()) {
    Serial.println("IMU failed to init!");
    while (1);
  }

  FliteQuaternion firstQ(1, 0, 0, 0);

  myController.setDesiredOrientation(firstQ);

  Serial.printf("ArduFlite Controller initialised\n");

  calibrateButton.begin();
  HoldButtonManager::registerButton(calibrateButton);
  Serial.println("Press and hold the button for 3s at any time to calibrate the IMU.");
}

void loop() 
{
  if (!mqttClient.connected()) reconnect();
  mqttClient.loop();

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

  // Update telemetry data
  if (xSemaphoreTake(telemetryMutex, 5) == pdTRUE) {
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

      xSemaphoreGive(telemetryMutex);
  }

  if (print_counter++ >= PRINT_EVERY_N_UPDATES) 
  {
    
    Serial.print("Acceleration: ");
    Serial.print(myIMU.getAccelX());
    Serial.print(", ");
    Serial.print(myIMU.getAccelY());
    Serial.print(", ");
    Serial.println(myIMU.getAccelZ());

    Serial.print("Gyroscope: ");
    Serial.print(myIMU.getGyroX());
    Serial.print(",");
    Serial.print(myIMU.getGyroY());
    Serial.print(",");
    Serial.println(myIMU.getGyroZ());
    
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
  
  //print quaternion for visualiser
  // Serial.printf("%f,%f,%f,%f\n", myIMU.getQw(), myIMU.getQx(), myIMU.getQy(), myIMU.getQz());

  delay(10); // ~100 Hz print
}
