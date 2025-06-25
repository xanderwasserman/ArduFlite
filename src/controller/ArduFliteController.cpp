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
    if (ctrlMutex == NULL) 
    {
        LOG_ERR("Failed to create ArduFliteController mutex!");
    }

    // Create the mutex for protecting innerLoop stats.
    innerStatsMutex = xSemaphoreCreateMutex();
    if (innerStatsMutex == NULL) 
    {
        LOG_ERR("Failed to create innerLoop Stats mutex!");
    }

    // Create the mutex for protecting outerLoop stats.
    outerStatsMutex = xSemaphoreCreateMutex();
    if (outerStatsMutex == NULL) 
    {
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
    {
        SemaphoreLock lock(ctrlMutex);
        mode = newMode;
    }
}
 
/**
 * @brief Returns the current operating mode.
 * 
 * @return ArduFliteMode The current mode (ATTITUDE_MODE or RATE_MODE).
 */
ArduFliteMode ArduFliteController::getMode() const 
{
    ArduFliteMode m;

    {
        SemaphoreLock lock(ctrlMutex);
        m = mode;
    }

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
    {
        SemaphoreLock lock(ctrlMutex);
        pilotAttitudeSetpoint   = setpointDeg;
    }

    // Forward the request to the attitude controller.
    attitudeCtrl->setAttitudeControlSetpoint(setpointDeg);
}

/**
 * @brief Sets the desired attitude for a specified axis (in Euler angles, degrees) for ATTITUDE_MODE.
 * 
 * In ATTITUDE_MODE mode, the attitude controller will use this values to compute the
 * desired angular roll rate.
 *
 * @param axis The axis to set the setpoint for (0=roll, 1=pitch, 2=yaw)
 * @param value Roll attitude setpoint in degrees.
 */
void ArduFliteController::setAttitudeSetpointAxis(uint8_t axis, float value) 
{
    EulerAngles localCopy;

    {
        SemaphoreLock lock(ctrlMutex);
        switch(axis) {
            case 0: pilotAttitudeSetpoint.roll  = value; break;
            case 1: pilotAttitudeSetpoint.pitch = value; break;
            case 2: pilotAttitudeSetpoint.yaw   = value; break;
            default: return;
        }
    }
  attitudeCtrl->setAttitudeControlSetpoint(pilotAttitudeSetpoint);
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
    {
        SemaphoreLock lock(ctrlMutex);
        pilotRateSetpoint  = rateSetpoint;
    }
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
    {
        SemaphoreLock lock(ctrlMutex);
        pilotRateSetpoint.roll  = rollRateSetpoint;
    }
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
    {
        SemaphoreLock lock(ctrlMutex);
        pilotRateSetpoint.pitch  = pitchRateSetpoint;
    }
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
    {
        SemaphoreLock lock(ctrlMutex);
        pilotRateSetpoint.yaw  = yawRateSetpoint;
    }
}

/**
 * @brief Sets the pilot-provided throttle setpoint.
 *
 * @param throttleSetpoint Throttle setpoint in percentage/100 (0.0 - 1.0).
 */
void setThrottleSetpoint(float throttleSetpoint)
{
    {
        SemaphoreLock lock(ctrlMutex);
        pilotThrottleSetpoint  = throttleSetpoint;
    }
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
    if (xTaskCreate(OuterLoopTask, "OuterLoop", 4096, this, 2, &outerTaskHandle) != pdPASS)
    {
        LOG_ERR("OuterLoopTask creation failed!");
    }

    if (xTaskCreate(InnerLoopTask, "InnerLoop", 4096, this, 3, &innerTaskHandle) != pdPASS)
    {
        LOG_ERR("InnerLoopTask creation failed!");
    }
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

void ArduFliteController::arm() 
{
    {
        SemaphoreLock lock(ctrlMutex);
        // Reset any integrators or last‐commands here if you like:
        rateCtrl->reset();
        attitudeCtrl->reset();
        armed = true;
    }
}

void ArduFliteController::disarm() 
{
    {
        SemaphoreLock lock(ctrlMutex);
        armed = false;
        // Optionally send neutral servos immediately:
        servoMgr->writeCommands(0,0,0);
    }
}

bool ArduFliteController::isArmed() const 
{
    bool a;
    {
        SemaphoreLock lock(ctrlMutex);
        a = armed;
    }

    return a;
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
    const TickType_t xFrequency = pdMS_TO_TICKS(outerLoopMs); // 10 ms period (100Hz)
    static unsigned long lastMicros = micros();

    // Desired period in microseconds for the outer loop (10 ms = 10,000 µs)
    const unsigned long desiredPeriodOuter = outerLoopMs *1000UL;

    EulerAngles rateCommand     {0.0f};
    EulerAngles rateSetpoint    {0.0f};
    ArduFliteMode currentMode;
     
    while(1) 
    {
        unsigned long currentMicros = micros();
        unsigned long dtMicro = currentMicros - lastMicros;
        lastMicros = currentMicros;

        // Update outer loop statistics.
        {
            SemaphoreLock lock(controller->outerStatsMutex);
            updateLoopStats(controller->outerLoopStats, dtMicro, desiredPeriodOuter);
        }

        float dt = dtMicro / 1000000.0f;
        const float maxDt = 0.02f;  // 20 ms max dt

        if (dt < 1e-3f) dt = 1e-3f;
        if (dt > maxDt) dt = maxDt;

        // Protect reading of the mode and pilot setpoints.
        {
            SemaphoreLock lock(controller->ctrlMutex);
            currentMode = controller->mode;
            rateSetpoint = controller->pilotRateSetpoint;
        }

        if (currentMode == ATTITUDE_MODE) 
        {
            // In Assist mode, use the attitude controller to compute desired rates.
            FliteQuaternion currentQ = controller->imu->getQuaternion();
            controller->attitudeCtrl->update(currentQ, dt, rateCommand);

            {
                SemaphoreLock lock(controller->ctrlMutex);
                controller->lastAttitudeCmd  = rateCommand;
            }

            // Pass the desired angular rates to the rate controller.
            controller->rateCtrl->setRateControlSetpoint(rateCommand);
        } 
        else if (currentMode == RATE_MODE)
        {
            // In Stabilized mode, use pilot-provided rate setpoints.
            rateCommand  = rateSetpoint; 

            {
                SemaphoreLock lock(controller->ctrlMutex);
                controller->lastAttitudeCmd = rateCommand;
            }

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
    const TickType_t xFrequency = pdMS_TO_TICKS(innerLoopMs); // 2 ms period (500Hz)
    static unsigned long lastMicros = micros();

    // Desired period in microseconds for the outer loop (10 ms = 10,000 µs)
    const unsigned long desiredPeriodInner = innerLoopMs *1000UL;

    EulerAngles actuatorCmd {0.0f};
    
    for (;;) 
    {
        unsigned long currentMicros = micros();
        unsigned long dtMicro = currentMicros - lastMicros;
        lastMicros = currentMicros;

        // Update inner loop statistics.
        {
            SemaphoreLock lock(controller->innerStatsMutex);
            updateLoopStats(controller->innerLoopStats, dtMicro, desiredPeriodInner);
        }

        float dt = dtMicro / 1000000.0f;
        const float maxDt = 0.02f;  // 20 ms max dt

        if (dt < 1e-3f) dt = 1e-3f;
        if (dt > maxDt) dt = maxDt;
        
        // Retrieve measured angular rates from the IMU.
        Vector3 gyro = controller->imu->getGyro();
        
        // Fetch current mode + pilot setpoints:
        ArduFliteMode mode;
        EulerAngles  pilot = {0,0,0};

        {
            SemaphoreLock lock(controller->ctrlMutex);
            mode  = controller->mode;
            pilot = controller->pilotRateSetpoint;
        }

        if (mode == MANUAL_MODE) 
        {
            actuatorCmd = pilot;
        } 
        else 
        {
            controller->rateCtrl->update(gyro, dt, actuatorCmd);
        }

        // Only actually drive servos if we’re armed:
        if (controller->armed) 
        {
            controller->servoMgr->writeCommands(actuatorCmd.roll,
                                    actuatorCmd.pitch,
                                    actuatorCmd.yaw);
            controller->servoMgr->writeThrottle(pilotThrottleSetpoint);
        } 
        else 
        {
            // hold neutral
            controller->servoMgr->writeCommands(0,0,0);
            controller->servoMgr->writeThrottle(0);
        }

        {
            SemaphoreLock lock(controller->ctrlMutex);
            controller->lastRateCmd  = actuatorCmd;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

EulerAngles ArduFliteController::getAttitudeSetpoint() const
{
    EulerAngles value;

    {
        SemaphoreLock lock(ctrlMutex);
        value  = pilotAttitudeSetpoint;
    }

    return value;
}

EulerAngles ArduFliteController::getRateSetpoint() const
{
    EulerAngles value;

    {
        SemaphoreLock lock(ctrlMutex);
        value  = pilotRateSetpoint;
    }

    return value;
}

EulerAngles ArduFliteController::getRateCmd() const
{
    EulerAngles value;

    {
        SemaphoreLock lock(ctrlMutex);
        value = lastRateCmd;
    }
    
    return value;
}
 
EulerAngles ArduFliteController::getAttitudeCmd() const
{
    EulerAngles value;

    {
        SemaphoreLock lock(ctrlMutex);
        value = lastAttitudeCmd;
    }

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

LoopStats ArduFliteController::getOuterLoopStats() 
{
    LoopStats statsCopy;

    {
        SemaphoreLock lock(outerStatsMutex);
        statsCopy = outerLoopStats;
    }

    return statsCopy;
}

LoopStats ArduFliteController::getInnerLoopStats() 
{
    LoopStats statsCopy;

    {
        SemaphoreLock lock(innerStatsMutex);
        statsCopy = innerLoopStats;
    }

    return statsCopy;
}
