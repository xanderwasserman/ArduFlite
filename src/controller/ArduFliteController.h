/**
 * ArduFliteController.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef ARDU_FLITE_CONTROLLER_H
#define ARDU_FLITE_CONTROLLER_H

#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/actuators/ServoManager.h"
#include "include/ArduFlite.h"

#include <Arduino.h>

/**
 * @brief Control loop timing statistics.
 */
struct LoopStats {
    float avgDt;         // Rolling average dt (in ms)
    float maxDt;         // Maximum dt seen over the window (in ms)
    unsigned long overrunCount; // Count of dt values that exceed the desired period
    unsigned long sampleCount;  // Number of samples accumulated
};


/**
 * @brief Operating modes for the ArduFlite controller.
 *
 * The controller can operate in one of two modes:
 * - ATTITUDE_MODE: The pilot provides a desired attitude setpoint (SAFE-like).
 * - RATE_MODE: The pilot directly provides angular rate setpoints.
 */
enum ArduFliteMode 
{
    ATTITUDE_MODE   = 0,     // Pilot controls the attitude setpoint.
    RATE_MODE,              // Pilot directly controls the rate setpoints.
    MANUAL_MODE,            // Pilot directly controls the servos.
    UNKNOWN_MODE,
    FLIGHT_MODE_LENGTH
};

enum ControlLoopType
{
    ATTITUDE_ROLL_LOOP = 0,
    ATTITUDE_PITCH_LOOP,
    ATTITUDE_YAW_LOOP,
    RATE_ROLL_LOOP,
    RATE_PITCH_LOOP,
    RATE_YAW_LOOP
};

/**
 * @brief Overall controller for ArduFlite.
 *
 * This class encapsulates the complete control system for ArduFlite by
 * combining an outer loop (attitude controller) and an inner loop (rate controller).
 * It runs two FreeRTOS tasks:
 * - OuterLoopTask (approx. 100Hz): Computes desired angular rates based on the current
 *   attitude (or uses direct pilot input in RATE_MODE).
 * - InnerLoopTask (approx. 500Hz): Updates the IMU, runs the rate controller, and outputs
 *   servo commands.
 *
 * Shared state (operating mode and pilot setpoints) is protected by a mutex.
 */
class ArduFliteController 
{
public:
    /**
     * @brief Constructs the overall ArduFlite controller.
     *
     * @param imu Pointer to the ArduFliteIMU instance.
     * @param attitudeCtrl Pointer to the ArduFliteAttitudeController instance.
     * @param rateCtrl Pointer to the ArduFliteRateController instance.
     * @param servoMgr Pointer to the ServoManager instance.
     */
    ArduFliteController(ArduFliteIMU* imu,
                          ArduFliteAttitudeController* attitudeCtrl,
                          ArduFliteRateController* rateCtrl,
                          ServoManager* servoMgr);

    /**
     * @brief Starts the controller tasks.
     *
     * Creates two FreeRTOS tasks:
     * - OuterLoopTask: Runs at ~100Hz to compute desired angular rates.
     * - InnerLoopTask: Runs at ~500Hz to update sensor data, compute servo commands, and actuate.
     */
    void startTasks();

    /**
     * @brief Sets the desired orientation in Assist mode.
     *
     * In ATTITUDE_MODE, the pilot sets a desired attitude which the attitude controller
     * uses to compute the desired angular rates.
     *
     * @param setpoint EulerAngles attitude setpoint in degrees.
     */
    void setAttitudeSetpoint(EulerAngles setpointDeg);

    /**
     * @brief Sets the desired attitude for a specified axis (in Euler angles, degrees) for ATTITUDE_MODE.
     * 
     * In ATTITUDE_MODE mode, the attitude controller will use this values to compute the
     * desired angular roll rate.
     *
     * @param axis The axis to set the setpoint for (0=roll, 1=pitch, 2=yaw)
     * @param value Roll attitude setpoint in degrees.
     */
    void setAttitudeSetpointAxis(uint8_t axis, float value);

    /**
     * @brief Sets the pilot-provided rate setpoints in Rate mode.
     *
     * In RATE_MODE, the pilot directly provides angular rate setpoints.
     *
     * @param rateSetpoint EulerAngles rate setpoint in degrees/s.
     */
    void setRateSetpoint(EulerAngles rateSetpoint);

    /**
     * @brief Sets the pilot-provided roll rate setpoint in Rate mode.
     *
     * In RATE_MODE, the pilot directly provides angular rate setpoints.
     *
     * @param rollRateSetpoint Roll rate setpoint in degrees/s.
     */
    void setRateSetpoint_roll(float rollRateSetpoint);

    /**
     * @brief Sets the pilot-provided pitch rate setpoint in Rate mode.
     *
     * In RATE_MODE, the pilot directly provides angular rate setpoints.
     *
     * @param pitchRateSetpoint Pitch rate setpoint in degrees/s.
     */
    void setRateSetpoint_pitch(float pitchRateSetpoint);

    /**
     * @brief Sets the pilot-provided yaw rate setpoint in Rate mode.
     *
     * In RATE_MODE, the pilot directly provides angular rate setpoints.
     *
     * @param yawRateSetpoint Yaw rate setpoint in degrees/s.
     */
    void setRateSetpoint_yaw(float yawRateSetpoint);

    /**
     * @brief Sets the pilot-provided throttle setpoint.
     *
     * @param throttleSetpoint Throttle setpoint in percentage/100 (0.0 - 1.0).
     */
    void setThrottleSetpoint(float throttleSetpoint);

    /**
     * @brief Sets the operating mode.
     *
     * This method switches between ATTITUDE_MODE, RATE_MODE and MANUAL_MODE.
     *
     * @param mode The mode to set.
     */
    void setMode(ArduFliteMode mode);

    /**
     * @brief Gets the current operating mode.
     *
     * @return ArduFliteMode The current mode.
     */
    ArduFliteMode getMode() const;

    /**
    * @brief Suspends the control tasks (outer and inner loop).
    */
    void pauseTasks();

    /**
    * @brief Resumes the control tasks (outer and inner loop).
    */
    void resumeTasks();

    /**
     * @brief Arm the controller: enable servo outputs.
     *        Never suspends the loops—just gates them.
     */
    void arm();

    /**
     * @brief Disarm the controller: disable servo outputs immediately.
     */
    void disarm();

    /**
     * @brief Returns true if we’re currently armed.
     */
    bool isArmed() const;

    EulerAngles getAttitudeSetpoint() const;
    EulerAngles getRateSetpoint() const;

    EulerAngles getAttitudeCmd() const;
    EulerAngles getRateCmd() const;

    LoopStats getOuterLoopStats();
    LoopStats getInnerLoopStats();

private:
    ArduFliteIMU* imu;                                      //< Pointer to the IMU instance.
    ArduFliteAttitudeController* attitudeCtrl;              //< Pointer to the outer loop controller.
    ArduFliteRateController* rateCtrl;                      //< Pointer to the inner loop controller.
    ServoManager* servoMgr;                                 //< Pointer to the servo manager.

    TaskHandle_t outerTaskHandle;                           //< Handle for the outer loop task.
    TaskHandle_t innerTaskHandle;                           //< Handle for the inner loop task.

    ArduFliteMode mode;                                     //< Current operating mode.
    EulerAngles pilotRateSetpoint       {0.0f};             //< Pilot rate setpoint (deg/s) for RATE_MODE.
    EulerAngles pilotAttitudeSetpoint   {0.0f};             //< Pilot attitude setpoint for ATTITUDE_MODE.
    float       pilotThrottleSetpoint   = 0.0f;             //< Pilot throttle setpoint for all modes.

    // Shared command variables
    EulerAngles lastAttitudeCmd         {0.0f};
    EulerAngles lastRateCmd             {0.0f};

    // Statistics for the outer and inner loops.
    LoopStats outerLoopStats            = {0, 0, 0, 0};
    LoopStats innerLoopStats            = {0, 0, 0, 0};

    SemaphoreHandle_t ctrlMutex;                            //< Mutex to protect shared state.

    // Mutexes for thread-safe access to these stats.
    SemaphoreHandle_t outerStatsMutex;
    SemaphoreHandle_t innerStatsMutex;

    bool armed = false;                                     //< true once we’ve called arm()

    static constexpr TickType_t outerLoopMs = 10;
    static constexpr TickType_t innerLoopMs = 2;

    /**
     * @brief Outer loop FreeRTOS task function.
     *
     * Runs at approximately 100Hz, reads the current IMU orientation, and computes
     * desired angular rates using either the attitude controller (in ATTITUDE_MODE) or
     * direct pilot setpoints (in RATE_MODE). The computed rates are passed to
     * the rate controller.
     *
     * @param parameters Pointer to the ArduFliteController instance.
     */
    static void OuterLoopTask(void* parameters);

    /**
     * @brief Inner loop FreeRTOS task function.
     *
     * Runs at approximately 500Hz, updates the IMU, retrieves measured angular rates,
     * computes the final servo commands via the rate controller, and writes these commands
     * to the servos.
     *
     * @param parameters Pointer to the ArduFliteController instance.
     */
    static void InnerLoopTask(void* parameters);

    /**
     * @brief helper function to calculate control loop timing statistics.
     *
     * @param parameters The respective control loop's statistics struct, the change in time from the last loop execution,
     * and the desired loop timing.
     */
    static void updateLoopStats(LoopStats &stats, unsigned long dtMicro, unsigned long desiredPeriodMicro);
};

#endif // ARDU_FLITE_CONTROLLER_H
