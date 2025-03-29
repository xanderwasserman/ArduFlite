#include <Arduino.h>

#include "src/orientation/ArduFliteIMU.h"

#include "PrintTask.h"

// Declare extern references for the objects whose data you want to print.
// Make sure these objects are defined in your main sketch or elsewhere.
extern ArduFliteIMU myIMU;
// You can also add additional extern declarations if you need more data.

static void printTask(void *parameter) {
  // Initialize the last wake time.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Set frequency to 1 second (1000 ms).
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);

  for (;;) {
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

    Serial.print(" -> rollCmd: "); Serial.print(getRollCmd());
    Serial.print(" pitchCmd: "); Serial.print(getPitchCmd());
    Serial.print(" yawCmd: "); Serial.println(getYawCmd());

    // Delay until the next cycle.
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void startPrintTask() {
  // Create the print task.
  xTaskCreate(printTask, "PrintTask", 4096, NULL, 1, NULL);
}
