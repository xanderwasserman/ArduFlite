/**
 * @file ArduFliteController.cpp
 * @brief Implements the overall controller for ArduFlite which combines
 *        the attitude (outer loop) and rate (inner loop) controllers.
 *
 * This class manages two FreeRTOS tasks running at different rates:
 * - The OuterLoopTask (at ~100Hz) computes desired angular rates either via the
 *   attitude controller (Assist mode) or from direct pilot input (Stabilized mode).
 * - The InnerLoopTask (at ~500Hz) updates the IMU, runs the rate controller to compute
 *   final servo commands, and sends those commands to the servos.
 *
 * It uses a mutex (ctrlMutex) to protect shared state (mode and pilot setpoints)
 * from concurrent access.
 */

 #include "src/controller/ArduFliteController.h"
 #include <Arduino.h>
 
 /**
  * @brief Constructor for ArduFliteController.
  * 
  * Initializes the controller with pointers to the shared components: IMU,
  * attitude controller, rate controller, and ServoManager. Also initializes the
  * operating mode to ASSIST_MODE and creates a mutex to protect shared state.
  *
  * @param imu Pointer to the ArduFliteIMU instance.
  * @param attitudeCtrl Pointer to the ArduFliteAttitudeController instance.
  * @param rateCtrl Pointer to the ArduFliteRateController instance.
  * @param servoMgr Pointer to the ServoManager instance.
  */
 ArduFliteController::ArduFliteController(ArduFliteIMU* imu,
                                            ArduFliteAttitudeController* attitudeCtrl,
                                            ArduFliteRateController* rateCtrl,
                                            ServoManager* servoMgr)
     : imu(imu), attitudeCtrl(attitudeCtrl), rateCtrl(rateCtrl), servoMgr(servoMgr),
       outerTaskHandle(NULL), innerTaskHandle(NULL), mode(ASSIST_MODE),
       pilotRollRateSetpoint(0.0f), pilotPitchRateSetpoint(0.0f), pilotYawRateSetpoint(0.0f)
 {
     // Create the mutex for protecting shared state.
     ctrlMutex = xSemaphoreCreateMutex();
     if (ctrlMutex == NULL) {
         Serial.println("Failed to create ArduFliteController mutex!");
     }
 }
 
 /**
  * @brief Sets the operating mode of the controller.
  * 
  * This function allows switching between ASSIST_MODE (where the pilot controls the
  * attitude setpoint and the controller computes desired angular rates) and 
  * STABILIZED_MODE (where the pilot directly provides rate setpoints).
  *
  * @param newMode The new mode to set.
  */
 void ArduFliteController::setMode(ArduFliteMode newMode) {
     xSemaphoreTake(ctrlMutex, portMAX_DELAY);
     mode = newMode;
     xSemaphoreGive(ctrlMutex);
 }
 
 /**
  * @brief Returns the current operating mode.
  * 
  * @return ArduFliteMode The current mode (ASSIST_MODE or STABILIZED_MODE).
  */
 ArduFliteMode ArduFliteController::getMode() const {
     ArduFliteMode m;
     xSemaphoreTake((SemaphoreHandle_t)ctrlMutex, portMAX_DELAY);
     m = mode;
     xSemaphoreGive((SemaphoreHandle_t)ctrlMutex);
     return m;
 }
 
 /**
  * @brief Sets the desired attitude (in Euler angles, degrees) for Assist mode.
  * 
  * In Assist mode, the attitude controller will use these values to compute the
  * desired angular rates.
  *
  * @param roll Roll angle in degrees.
  * @param pitch Pitch angle in degrees.
  * @param yaw Yaw angle in degrees.
  */
 void ArduFliteController::setDesiredEulerDegs(float roll, float pitch, float yaw) {
     // Forward the request to the attitude controller.
     attitudeCtrl->setDesiredEulerDegs(roll, pitch, yaw);
 }
 
 /**
  * @brief Sets the pilot-provided rate setpoints for Stabilized mode.
  * 
  * When operating in Stabilized mode, these values are used directly as the desired
  * angular rates.
  *
  * @param rollRate Desired roll rate setpoint (e.g., in deg/s).
  * @param pitchRate Desired pitch rate setpoint.
  * @param yawRate Desired yaw rate setpoint.
  */
 void ArduFliteController::setPilotRateSetpoints(float rollRate, float pitchRate, float yawRate) {
     xSemaphoreTake(ctrlMutex, portMAX_DELAY);
     pilotRollRateSetpoint  = rollRate;
     pilotPitchRateSetpoint = pitchRate;
     pilotYawRateSetpoint   = yawRate;
     xSemaphoreGive(ctrlMutex);
 }
 
 /**
  * @brief Starts the overall control tasks.
  * 
  * Creates two FreeRTOS tasks:
  * - OuterLoopTask: Handles attitude control (runs at ~100Hz).
  * - InnerLoopTask: Handles rate control and servo updates (runs at ~500Hz).
  */
 void ArduFliteController::startTasks() {
     xTaskCreate(OuterLoopTask, "OuterLoop", 4096, this, 2, &outerTaskHandle);
     xTaskCreate(InnerLoopTask, "InnerLoop", 4096, this, 3, &innerTaskHandle);
 }
 
 /**
  * @brief Outer loop task.
  * 
  * Runs at approximately 100Hz. Depending on the mode, it either uses the attitude
  * controller to compute desired angular rates from the IMU's quaternion (Assist mode)
  * or directly uses the pilot-provided rate setpoints (Stabilized mode).
  *
  * The computed desired rates are then passed to the rate controller.
  *
  * @param parameters Pointer to the ArduFliteController instance.
  */
 void ArduFliteController::OuterLoopTask(void* parameters) {
     ArduFliteController* controller = static_cast<ArduFliteController*>(parameters);
     TickType_t xLastWakeTime = xTaskGetTickCount();
     const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10 ms period (100Hz)
     static unsigned long lastMicros = micros();
     
     float rollRateCmd = 0, pitchRateCmd = 0, yawRateCmd = 0;
     ArduFliteMode currentMode;
     float localPilotRoll = 0, localPilotPitch = 0, localPilotYaw = 0;
     
     for (;;) {
         unsigned long currentMicros = micros();
         float dt = (currentMicros - lastMicros) / 1000000.0f;
         lastMicros = currentMicros;
         if (dt < 1e-3f) dt = 1e-3f;
 
         // Protect reading of the mode and pilot setpoints.
         xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
         currentMode = controller->mode;
         localPilotRoll  = controller->pilotRollRateSetpoint;
         localPilotPitch = controller->pilotPitchRateSetpoint;
         localPilotYaw   = controller->pilotYawRateSetpoint;
         xSemaphoreGive(controller->ctrlMutex);
 
         if (currentMode == ASSIST_MODE) {
             // In Assist mode, use the attitude controller to compute desired rates.
             FliteQuaternion currentQ = controller->imu->getQuaternion();
             controller->attitudeCtrl->update(currentQ, dt, rollRateCmd, pitchRateCmd, yawRateCmd);

            xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
            controller->lastRollRateCmd  = rollRateCmd;
            controller->lastPitchRateCmd = pitchRateCmd;
            controller->lastYawRateCmd   = yawRateCmd;
            xSemaphoreGive(controller->ctrlMutex);
         } else {
             // In Stabilized mode, use pilot-provided rate setpoints.
             rollRateCmd  = localPilotRoll;
             pitchRateCmd = localPilotPitch;
             yawRateCmd   = localPilotYaw;

            xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
            controller->lastRollRateCmd  = rollRateCmd;
            controller->lastPitchRateCmd = pitchRateCmd;
            controller->lastYawRateCmd   = yawRateCmd;
            xSemaphoreGive(controller->ctrlMutex); 
         }
         
         // Pass the desired angular rates to the rate controller.
         controller->rateCtrl->setDesiredRates(rollRateCmd, pitchRateCmd, yawRateCmd);
         vTaskDelayUntil(&xLastWakeTime, xFrequency);
     }
 }
 
 /**
  * @brief Inner loop task.
  * 
  * Runs at approximately 500Hz. This task updates the IMU sensor data, retrieves
  * the measured angular rates, runs the rate controller to compute the final servo
  * commands, and then writes these commands to the servos.
  *
  * @param parameters Pointer to the ArduFliteController instance.
  */
 void ArduFliteController::InnerLoopTask(void* parameters) {
     ArduFliteController* controller = static_cast<ArduFliteController*>(parameters);
     TickType_t xLastWakeTime = xTaskGetTickCount();
     const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2 ms period (500Hz)
     static unsigned long lastMicros = micros();
     float rollCmd, pitchCmd, yawCmd;
     
     for (;;) {
         unsigned long currentMicros = micros();
         float dt = (currentMicros - lastMicros) / 1000000.0f;
         lastMicros = currentMicros;
         if (dt < 1e-3f) dt = 1e-3f;
         
         // Update IMU data.
         controller->imu->update(dt);
         // Retrieve measured angular rates from the IMU.
         Vector3 gyro = controller->imu->getGyro();
         
         // Update the rate controller (inner loop) to compute servo commands.
         controller->rateCtrl->update(gyro.x, gyro.y, gyro.z, dt, rollCmd, pitchCmd, yawCmd);
         
         // Write the computed servo commands (normalized to [-1, 1]).
         controller->servoMgr->writeCommands(rollCmd, pitchCmd, yawCmd);

        xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
        controller->lastRollCmd  = rollCmd;
        controller->lastPitchCmd = pitchCmd;
        controller->lastYawCmd   = yawCmd;
        xSemaphoreGive(controller->ctrlMutex);

         vTaskDelayUntil(&xLastWakeTime, xFrequency);
     }
 }
 
 float ArduFliteController::getRollRateCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastRollRateCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getPitchRateCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastPitchRateCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getYawRateCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastYawRateCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getRollCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastRollCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getPitchCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastPitchCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getYawCmd() {
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastYawCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}
