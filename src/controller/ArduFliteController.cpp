/**
 * ArduFliteController.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
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
#include "src/utils/Logging.h"

#include <Arduino.h>

#define OUTER_LOOP_DT 10
#define INNER_LOOP_DT 2
 
/**
 * @brief Constructor for ArduFliteController.
 * 
 * Initializes the controller with pointers to the shared components: IMU,
 * attitude controller, rate controller, and ServoManager. Also initializes the
 * operating mode to ATTITUDE_MODE and creates a mutex to protect shared state.
 *
 * @param imu Pointer to the ArduFliteIMU instance.
 * @param attitudeCtrl Pointer to the ArduFliteAttitudeController instance.
 * @param rateCtrl Pointer to the ArduFliteRateController instance.
 * @param servoMgr Pointer to the ServoManager instance.
 */
ArduFliteController::ArduFliteController(ArduFliteIMU* imu, ArduFliteAttitudeController* attitudeCtrl, ArduFliteRateController* rateCtrl, ServoManager* servoMgr)
    : imu(imu)
    , attitudeCtrl(attitudeCtrl)
    , rateCtrl(rateCtrl)
    , servoMgr(servoMgr)
    , outerTaskHandle(NULL)
    , innerTaskHandle(NULL)
    , mode(ATTITUDE_MODE)
    , pilotRateSetpoint{ 0.0f, 0.0f, 0.0f }
{
    // Create the mutex for protecting shared state.
    ctrlMutex = xSemaphoreCreateMutex();
    if (ctrlMutex == NULL) {
        LOG_ERR("Failed to create ArduFliteController mutex!");
    }

    // Create the mutex for protecting innerLoop stats.
    innerStatsMutex = xSemaphoreCreateMutex();
    if (innerStatsMutex == NULL) {
        LOG_ERR("Failed to create innerLoop Stats mutex!");
    }

    // Create the mutex for protecting outerLoop stats.
    outerStatsMutex = xSemaphoreCreateMutex();
    if (outerStatsMutex == NULL) {
        LOG_ERR("Failed to create outerLoop Stats mutex!");
    }
}
 
/**
 * @brief Sets the operating mode of the controller.
 * 
 * This function allows switching between ATTITUDE_MODE (where the pilot controls the
 * attitude setpoint and the controller computes desired angular rates) and 
 * RATE_MODE (where the pilot directly provides rate setpoints).
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
 * @return ArduFliteMode The current mode (ATTITUDE_MODE or RATE_MODE).
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
 * @param setpoint EulerAngles attitude setpoint in degrees.
 */
void ArduFliteController::setAttitudeSetpoint(EulerAngles setpointDeg) 
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotAttitudeSetpoint   = setpointDeg;
    xSemaphoreGive(ctrlMutex);

    // Forward the request to the attitude controller.
    attitudeCtrl->setAttitudeControlSetpoint(setpointDeg);
}

/**
 * @brief Sets the desired roll attitude (in Euler angles, degrees) for Assist mode.
 * 
 * In Assist mode, the attitude controller will use this values to compute the
 * desired angular roll rate.
 *
 * @param rollSetpointDeg Roll attitude setpoint in degrees.
 */
void ArduFliteController::setAttitudeSetpoint_roll(float rollSetpointDeg) 
{
    EulerAngles localCopy;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotAttitudeSetpoint.roll  = rollSetpointDeg;
    localCopy                   = pilotAttitudeSetpoint;
    xSemaphoreGive(ctrlMutex);

    // Forward the request to the attitude controller.
    attitudeCtrl->setAttitudeControlSetpoint(localCopy);
}

/**
 * @brief Sets the desired pitch attitude (in Euler angles, degrees) for Assist mode.
 * 
 * In Assist mode, the attitude controller will use this values to compute the
 * desired angular pitch rate.
 *
 * @param pitchSetpointDeg Pitch attitude setpoint in degrees.
 */
void ArduFliteController::setAttitudeSetpoint_pitch(float pitchSetpointDeg) 
{
    EulerAngles localCopy;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotAttitudeSetpoint.pitch = pitchSetpointDeg;
    localCopy                   = pilotAttitudeSetpoint;
    xSemaphoreGive(ctrlMutex);

    // Forward the request to the attitude controller.
    attitudeCtrl->setAttitudeControlSetpoint(localCopy);
}
 
/**
 * @brief Sets the desired yaw attitude (in Euler angles, degrees) for Assist mode.
 * 
 * In Assist mode, the attitude controller will use this values to compute the
 * desired angular yaw rate.
 *
 * @param pitchSetpointDeg Yaw attitude setpoint in degrees.
 */
void ArduFliteController::setAttitudeSetpoint_yaw(float yawSetpointDeg) 
{
    EulerAngles localCopy;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotAttitudeSetpoint.yaw   = yawSetpointDeg;
    localCopy                   = pilotAttitudeSetpoint;
    xSemaphoreGive(ctrlMutex);

    // Forward the request to the attitude controller.
    attitudeCtrl->setAttitudeControlSetpoint(localCopy);
}
 
/**
 * @brief Sets the pilot-provided rate setpoints for Stabilized mode.
 * 
 * When operating in Stabilized mode, these values are used directly as the desired
 * angular rates.
 *
 * @param setpoint EulerAngles rate setpoint in degrees/s.
 */
void ArduFliteController::setRateSetpoint(EulerAngles rateSetpoint) 
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotRateSetpoint  = rateSetpoint;
    xSemaphoreGive(ctrlMutex);
}

/**
 * @brief Sets the pilot-provided roll rate setpoint in Rate mode.
 *
 * In RATE_MODE, the pilot directly provides angular rate setpoints.
 *
 * @param rollRateSetpoint Roll rate setpoint in degrees/s.
 */
void ArduFliteController::setRateSetpoint_roll(float rollRateSetpoint)
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotRateSetpoint.roll  = rollRateSetpoint;
    xSemaphoreGive(ctrlMutex);
}

/**
 * @brief Sets the pilot-provided pitch rate setpoint in Rate mode.
 *
 * In RATE_MODE, the pilot directly provides angular rate setpoints.
 *
 * @param rollRateSetpoint Pitch rate setpoint in degrees/s.
 */
void ArduFliteController::setRateSetpoint_pitch(float pitchRateSetpoint)
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotRateSetpoint.pitch  = pitchRateSetpoint;
    xSemaphoreGive(ctrlMutex);
}

/**
 * @brief Sets the pilot-provided yaw rate setpoint in Rate mode.
 *
 * In RATE_MODE, the pilot directly provides angular rate setpoints.
 *
 * @param rollRateSetpoint Yaw rate setpoint in degrees/s.
 */
void ArduFliteController::setRateSetpoint_yaw(float yawRateSetpoint)
{
    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    pilotRateSetpoint.yaw  = yawRateSetpoint;
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
    LOG_INF("Control tasks paused.");
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
    LOG_INF("Control tasks resumed.");
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

    EulerAngles rateCommand     {0.0f};
    EulerAngles rateSetpoint    {0.0f};
    ArduFliteMode currentMode;
     
    while(1) 
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
        rateSetpoint = controller->pilotRateSetpoint;
        xSemaphoreGive(controller->ctrlMutex);

        if (currentMode == ATTITUDE_MODE) 
        {
            // In Assist mode, use the attitude controller to compute desired rates.
            FliteQuaternion currentQ = controller->imu->getQuaternion();
            controller->attitudeCtrl->update(currentQ, dt, rateCommand);

            xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
            controller->lastAttitudeCmd  = rateCommand;
            xSemaphoreGive(controller->ctrlMutex);

            // Pass the desired angular rates to the rate controller.
            controller->rateCtrl->setRateControlSetpoint(rateCommand);
        } 
        else if (currentMode == RATE_MODE)
        {
            // In Stabilized mode, use pilot-provided rate setpoints.
            rateCommand  = rateSetpoint;

            xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
            controller->lastAttitudeCmd = rateCommand;
            xSemaphoreGive(controller->ctrlMutex); 

            // Pass the desired angular rates to the rate controller.
            controller->rateCtrl->setRateControlSetpoint(rateCommand);
        }
        // else if MANUAL_MODE: do nothing here (we bypass both loops)
        
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

    EulerAngles actuatorCmd {0.0f};
    
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
        
        // Fetch current mode + pilot setpoints:
        ArduFliteMode mode;
        EulerAngles  pilot = {0,0,0};
        xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
          mode  = controller->mode;
          pilot = controller->pilotRateSetpoint;
        xSemaphoreGive(controller->ctrlMutex);

        if (mode == MANUAL_MODE) 
        {
        // Use stick values as the “actuator command”:
        actuatorCmd = pilot;

        // and send them directly:
        controller->servoMgr->writeCommands(
            actuatorCmd.roll,
            actuatorCmd.pitch,
            actuatorCmd.yaw
        );
        }
        else 
        {
            // Normal inner‐loop: run rate controller
            controller->rateCtrl->update(gyro, dt, actuatorCmd);
            controller->servoMgr->writeCommands(
                actuatorCmd.roll,
                actuatorCmd.pitch,
                actuatorCmd.yaw
            );
        }

        xSemaphoreTake(controller->ctrlMutex, portMAX_DELAY);
        controller->lastRateCmd  = actuatorCmd;
        xSemaphoreGive(controller->ctrlMutex);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

EulerAngles ArduFliteController::getAttitudeSetpoint() const
{
    EulerAngles value;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value  = pilotAttitudeSetpoint;
    xSemaphoreGive(ctrlMutex);

    return value;
}

EulerAngles ArduFliteController::getRateSetpoint() const
{
    EulerAngles value;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value  = pilotRateSetpoint;
    xSemaphoreGive(ctrlMutex);

    return value;
}

EulerAngles ArduFliteController::getRateCmd() const
{
    EulerAngles value;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastRateCmd;
    xSemaphoreGive(ctrlMutex);
    
    return value;
}
 
EulerAngles ArduFliteController::getAttitudeCmd() const
{
    EulerAngles value;

    xSemaphoreTake(ctrlMutex, portMAX_DELAY);
    value = lastAttitudeCmd;
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
