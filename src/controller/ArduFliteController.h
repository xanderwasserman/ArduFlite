#ifndef ARDU_FLITE_CONTROLLER_H
#define ARDU_FLITE_CONTROLLER_H

#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/actuators/ServoManager.h"
#include <Arduino.h>

/**
 * @brief Operating modes for the ArduFlite controller.
 *
 * The controller can operate in one of two modes:
 * - ASSIST_MODE: The pilot provides a desired attitude setpoint (SAFE-like).
 * - STABILIZED_MODE: The pilot directly provides angular rate setpoints.
 */
enum ArduFliteMode {
    ASSIST_MODE,    ///< Pilot controls the attitude setpoint.
    STABILIZED_MODE ///< Pilot directly controls the rate setpoints.
};

/**
 * @brief Overall controller for ArduFlite.
 *
 * This class encapsulates the complete control system for ArduFlite by
 * combining an outer loop (attitude controller) and an inner loop (rate controller).
 * It runs two FreeRTOS tasks:
 * - OuterLoopTask (approx. 100Hz): Computes desired angular rates based on the current
 *   attitude (or uses direct pilot input in STABILIZED_MODE).
 * - InnerLoopTask (approx. 500Hz): Updates the IMU, runs the rate controller, and outputs
 *   servo commands.
 *
 * Shared state (operating mode and pilot setpoints) is protected by a mutex.
 */
class ArduFliteController {
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
     * In ASSIST_MODE, the pilot sets a desired attitude which the attitude controller
     * uses to compute the desired angular rates.
     *
     * @param roll Roll angle in degrees.
     * @param pitch Pitch angle in degrees.
     * @param yaw Yaw angle in degrees.
     */
    void setDesiredEulerDegs(float roll, float pitch, float yaw);

    /**
     * @brief Sets the pilot-provided rate setpoints in Stabilized mode.
     *
     * In STABILIZED_MODE, the pilot directly provides angular rate setpoints.
     *
     * @param rollRate Desired roll rate (deg/s).
     * @param pitchRate Desired pitch rate (deg/s).
     * @param yawRate Desired yaw rate (deg/s).
     */
    void setPilotRateSetpoints(float rollRate, float pitchRate, float yawRate);

    /**
     * @brief Sets the operating mode.
     *
     * This method switches between ASSIST_MODE and STABILIZED_MODE.
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

private:
    ArduFliteIMU* imu;                          ///< Pointer to the IMU instance.
    ArduFliteAttitudeController* attitudeCtrl;  ///< Pointer to the outer loop controller.
    ArduFliteRateController* rateCtrl;          ///< Pointer to the inner loop controller.
    ServoManager* servoMgr;                     ///< Pointer to the servo manager.

    TaskHandle_t outerTaskHandle;               ///< Handle for the outer loop task.
    TaskHandle_t innerTaskHandle;               ///< Handle for the inner loop task.

    volatile ArduFliteMode mode;                ///< Current operating mode.
    volatile float pilotRollRateSetpoint;       ///< Pilot roll rate setpoint (deg/s) for STABILIZED_MODE.
    volatile float pilotPitchRateSetpoint;      ///< Pilot pitch rate setpoint (deg/s) for STABILIZED_MODE.
    volatile float pilotYawRateSetpoint;        ///< Pilot yaw rate setpoint (deg/s) for STABILIZED_MODE.

    SemaphoreHandle_t ctrlMutex;                ///< Mutex to protect shared state.

    /**
     * @brief Outer loop FreeRTOS task function.
     *
     * Runs at approximately 100Hz, reads the current IMU orientation, and computes
     * desired angular rates using either the attitude controller (in ASSIST_MODE) or
     * direct pilot setpoints (in STABILIZED_MODE). The computed rates are passed to
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
};

#endif // ARDU_FLITE_CONTROLLER_H
