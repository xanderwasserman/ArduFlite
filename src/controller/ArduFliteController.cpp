/**
 * ArduFliteController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 Aptil 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
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

#define OUTER_LOOP_DT 10
#define INNER_LOOP_DT 2
 
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
ArduFliteController::ArduFliteController(ArduFliteIMU* imu, ArduFliteAttitudeController* attitudeCtrl, ArduFliteRateController* rateCtrl, ServoManager* servoMgr)
    : imu(imu), attitudeCtrl(attitudeCtrl), rateCtrl(rateCtrl), servoMgr(servoMgr), outerTaskHandle(NULL), innerTaskHandle(NULL), mode(ASSIST_MODE), pilotRollRateSetpoint(0.0f), pilotPitchRateSetpoint(0.0f), pilotYawRateSetpoint(0.0f)
{
    // Create the mutex for protecting shared state.
    ctrlMutex = xSemaphoreCreateMutex();
    if (ctrlMutex == NULL) {
        Serial.println("Failed to create ArduFliteController mutex!");
    }

    // Create the mutex for protecting innerLoop stats.
    innerStatsMutex = xSemaphoreCreateMutex();
    if (innerStatsMutex == NULL) {
        Serial.println("Failed to create innerLoop Stats mutex!");
    }

    // Create the mutex for protecting outerLoop stats.
    outerStatsMutex = xSemaphoreCreateMutex();
    if (outerStatsMutex == NULL) {
        Serial.println("Failed to create outerLoop Stats mutex!");
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
void ArduFliteController::setMode(ArduFliteMode newMode) 
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    mode = newMode;
    xSemaphoreGive(ctrlMutex);
}
 
/**
 * @brief Returns the current operating mode.
 * 
 * @return ArduFliteMode The current mode (ASSIST_MODE or STABILIZED_MODE).
 */
ArduFliteMode ArduFliteController::getMode() const 
{
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
void ArduFliteController::setDesiredEulerDegs(float pitch, float roll, float yaw) 
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotRollAngleSetpoint  = roll;
    pilotPitchAngleSetpoint = pitch;
    pilotYawAngleSetpoint   = yaw;
    xSemaphoreGive(ctrlMutex);

    // Forward the request to the attitude controller.
    attitudeCtrl->setDesiredEulerDegs(pitch, roll, yaw);
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
void ArduFliteController::setPilotRateSetpoints(float rollRate, float pitchRate, float yawRate) 
{
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
void ArduFliteController::startTasks() 
{
    xTaskCreate(OuterLoopTask, "OuterLoop", 4096, this, 2, &outerTaskHandle);
    xTaskCreate(InnerLoopTask, "InnerLoop", 4096, this, 3, &innerTaskHandle);
}

/**
 * @brief Suspends the control tasks (outer and inner loop).
 *
 * This function suspends the FreeRTOS tasks responsible for the control loops,
 * effectively pausing the attitude and rate controllers. This is useful during
 * operations such as calibration when you want a consistent sensor reading without
 * interference from the control loops.
 */
void ArduFliteController::pauseTasks() 
{
    if (outerTaskHandle != NULL) {
        vTaskSuspend(outerTaskHandle);
    }
    if (innerTaskHandle != NULL) {
        vTaskSuspend(innerTaskHandle);
    }
    Serial.println("Control tasks paused.");
}

/**
 * @brief Resumes the control tasks (outer and inner loop).
 *
 * This function resumes the previously suspended control tasks so that the
 * attitude and rate controllers continue their normal operation.
 */
void ArduFliteController::resumeTasks() 
{
    if (outerTaskHandle != NULL) {
        vTaskResume(outerTaskHandle);
    }
    if (innerTaskHandle != NULL) {
        vTaskResume(innerTaskHandle);
    }
    Serial.println("Control tasks resumed.");
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
void ArduFliteController::OuterLoopTask(void* parameters) 
{
    ArduFliteController* controller = static_cast<ArduFliteController*>(parameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(OUTER_LOOP_DT); // 10 ms period (100Hz)
    static unsigned long lastMicros = micros();

    // Desired period in microseconds for the outer loop (10 ms = 10,000 µs)
    const unsigned long desiredPeriodOuter = OUTER_LOOP_DT*1000UL;
    
    float rollRateCmd = 0, pitchRateCmd = 0, yawRateCmd = 0;
    ArduFliteMode currentMode;
    float localPilotRoll = 0, localPilotPitch = 0, localPilotYaw = 0;
     
    for (;;) 
    {
        unsigned long currentMicros = micros();
        unsigned long dtMicro = currentMicros - lastMicros;
        lastMicros = currentMicros;

        // Update outer loop statistics.
        xSemaphoreTake(controller->outerStatsMutex, portMAX_DELAY);
        updateLoopStats(controller->outerLoopStats, dtMicro, desiredPeriodOuter);
        xSemaphoreGive(controller->outerStatsMutex);

        float dt = dtMicro / 1000000.0f;
        if (dt < 1e-3f) dt = 1e-3f;

        // Protect reading of the mode and pilot setpoints.
        xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
        currentMode = controller->mode;
        localPilotRoll  = controller->pilotRollRateSetpoint;
        localPilotPitch = controller->pilotPitchRateSetpoint;
        localPilotYaw   = controller->pilotYawRateSetpoint;
        xSemaphoreGive(controller->ctrlMutex);

        if (currentMode == ASSIST_MODE) 
        {
            // In Assist mode, use the attitude controller to compute desired rates.
            FliteQuaternion currentQ = controller->imu->getQuaternion();
            controller->attitudeCtrl->update(currentQ, dt, rollRateCmd, pitchRateCmd, yawRateCmd);

            yawRateCmd   = 0; //! This is so that the yaw is not influenced by the attitude controller, but only by the rate controller. We just want to go in a straight line for now.

            xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
            controller->lastRollRateCmd  = rollRateCmd;
            controller->lastPitchRateCmd = pitchRateCmd;
            controller->lastYawRateCmd   = yawRateCmd;
            xSemaphoreGive(controller->ctrlMutex);
        } 
        else 
        {
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
void ArduFliteController::InnerLoopTask(void* parameters) 
{
    ArduFliteController* controller = static_cast<ArduFliteController*>(parameters);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(INNER_LOOP_DT); // 2 ms period (500Hz)
    static unsigned long lastMicros = micros();

    // Desired period in microseconds for the outer loop (10 ms = 10,000 µs)
    const unsigned long desiredPeriodInner = INNER_LOOP_DT*1000UL;

    float rollCmd, pitchCmd, yawCmd;
    
    for (;;) 
    {
        unsigned long currentMicros = micros();
        unsigned long dtMicro = currentMicros - lastMicros;
        lastMicros = currentMicros;

        // Update inner loop statistics.
        xSemaphoreTake(controller->innerStatsMutex, portMAX_DELAY);
        updateLoopStats(controller->innerLoopStats, dtMicro, desiredPeriodInner);
        xSemaphoreGive(controller->innerStatsMutex);

        float dt = dtMicro / 1000000.0f;
        if (dt < 1e-3f) dt = 1e-3f;
        
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
 
float ArduFliteController::getRollRateCmd() 
{
    float value;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastRollRateCmd;
    xSemaphoreGive(ctrlMutex);

    return value;
}

float ArduFliteController::getPitchRateCmd() 
{
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastPitchRateCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getYawRateCmd() 
{
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastYawRateCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getRollCmd() 
{
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastRollCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getPitchCmd() 
{
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastPitchCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

float ArduFliteController::getYawCmd() 
{
    float value;
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastYawCmd;
    xSemaphoreGive(ctrlMutex);
    return value;
}

void ArduFliteController::updateLoopStats(LoopStats &stats, unsigned long dtMicro, unsigned long desiredPeriodMicro) 
{
    // Convert dt to milliseconds.
    float dtMs = dtMicro / 1000.0f;
    
    // Update rolling average using an exponential moving average.
    // If this is the first sample, initialize it.
    const float alpha = 0.5f; // Smoothing factor (tweak as needed)
    if (stats.sampleCount == 0) 
    {
        stats.avgDt = dtMs;
    } 
    else 
    {
        stats.avgDt = alpha * dtMs + (1.0f - alpha) * stats.avgDt;
    }
    
    // Update max dt if current dt is higher.
    if (dtMs > stats.maxDt) 
    {
        stats.maxDt = dtMs;
    }
    
    // If this dt exceeds the desired period, count it as an overrun.
    if (dtMicro > desiredPeriodMicro * 1.1f) // add 10% buffer
    {
        stats.overrunCount++;
    }
    
    stats.sampleCount++;  
}

LoopStats ArduFliteController::getOuterLoopStats() {
    LoopStats statsCopy;
    xSemaphoreTake(outerStatsMutex, portMAX_DELAY);
    statsCopy = outerLoopStats;
    xSemaphoreGive(outerStatsMutex);
    return statsCopy;
}

LoopStats ArduFliteController::getInnerLoopStats() {
    LoopStats statsCopy;
    xSemaphoreTake(innerStatsMutex, portMAX_DELAY);
    statsCopy = innerLoopStats;
    xSemaphoreGive(innerStatsMutex);
    return statsCopy;
}
