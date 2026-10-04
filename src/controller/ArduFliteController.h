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

#include "src/core/FlightTypes.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/state/StateManagement.h"
#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/actuators/AirframeMixer.h"
#include "src/actuators/ControlOutputs.h"
#include "src/hal/device/Actuator.h"
#include "include/ArduFlite.h"
#include "include/ControllerTypes.h"

#include <chrono>

#include "src/hal/platform/Clock.h"
#include "src/hal/platform/Mutex.h"
#include "src/hal/platform/Scheduler.h"
#include "src/hal/platform/Storage.h"
#include "src/hal/platform/Watchdog.h"

#include <atomic>

// Forward declaration
namespace arduflite::device { class RcLink; }

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
     * @brief Platform services this controller needs.
     *
     * Grouped into a struct rather than six more constructor parameters. The
     * point of ADR-001 is that dependencies are VISIBLE in the signature — if
     * this struct ever grows unwieldy, that is the design telling you the
     * controller does too much.
     */
    struct Platform
    {
        arduflite::hal::Clock*     clock           = nullptr;
        arduflite::hal::Scheduler* scheduler       = nullptr;
        arduflite::hal::Watchdog*  watchdog        = nullptr;
        arduflite::hal::System*    system          = nullptr;
        arduflite::hal::Mutex*     ctrlMutex       = nullptr;
        arduflite::hal::Mutex*     outerStatsMutex = nullptr;
        arduflite::hal::Mutex*     innerStatsMutex = nullptr;

        /// The bank owning every control surface. commit() is a batched bus
        /// operation, so it belongs here rather than on individual actuators.
        arduflite::device::ActuatorBank* outputs = nullptr;

        [[nodiscard]] bool valid() const
        {
            return clock && scheduler && watchdog && system && outputs
                && ctrlMutex && outerStatsMutex && innerStatsMutex;
        }
    };

    /**
     * @brief Inject platform services. Must be called before startTasks().
     *
     * Deferred rather than constructor-injected because the controller is a
     * global and the Board's mutex pool needs FreeRTOS running — the same reason
     * initFromConfig() exists. Pointers, not references, so the global can be
     * default-constructed.
     */
    void setPlatform(const Platform& platform);

    /// Reload airframe geometry from ConfigRegistry. Call after config load.
    void initFromConfig();

    /// Boot self-test: drive every surface to a normalised deflection and commit.
    /// Goes through the mixer and the bank, so it exercises the real output
    /// path — driving the servos directly would pass with a miswired mixer.
    void stageTestDeflection(float normalised);

    /**
     * @brief Constructs the overall ArduFlite controller.
     *
     * @param imu          the estimation subsystem to read attitude from
     * @param attitudeCtrl outer loop: attitude error -> rate setpoint
     * @param rateCtrl     inner loop: rate error -> surface command
     *
     * Platform services arrive separately via setPlatform(); see Platform.
     */
    ArduFliteController(arduflite::estimation::InertialSubsystem* imu,
                          ArduFliteAttitudeController* attitudeCtrl,
                          ArduFliteRateController* rateCtrl);

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
     * @param setpoint AttitudeDeg attitude setpoint, in degrees.
     */
    void setAttitudeSetpoint(AttitudeDeg setpointDeg);

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
     * @param rateSetpoint AngularRateDps rate setpoint, in degrees/s.
     */
    void setRateSetpoint(AngularRateDps rateSetpoint);

    /**
     * @brief Direct surface demand for MANUAL_MODE, normalised -1..+1.
     *
     * Stored separately from pilotRateSetpoint, and typed differently, so a
     * deg/s value cannot reach the mixer as a normalised demand. Sharing one
     * slot would make its meaning depend on the current mode, and a demotion to
     * MANUAL_MODE would deliver whatever the previous mode had left there.
     */
    void setManualCommand(AxisCommand command);

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
     * @brief Arm the controller after passing preflight checks.
     *
     * Runs preflight validation before arming. Will reject arm request if:
     * - IMU is unhealthy
     * - Gyro bias exceeds threshold
     * - Accelerometer not reading ~1g
     * - Receiver link quality too low
     * - Throttle not at minimum
     *
     * @param receiver Pointer to CRSF receiver for link quality check (may be nullptr)
     * @return true if arm succeeded, false if preflight check failed
     */
    bool arm(arduflite::device::RcLink* rcLink = nullptr);

    /**
     * @brief Disarm the controller: disable servo outputs immediately.
     */
    void disarm();


    /**
     * @brief Returns true if we’re currently armed.
     */
    bool isArmed() const;

    /**
     * @brief Disarm the controller: disable servo outputs immediately.
     *
     * @param value Whether the throttle should be cut or not (true -> cut, false -> enabled).
     */
    void cutThrottle(bool value);

    /**
     * @brief Returns true if the throttle cut is enabled.
     */
    bool isThrottleCut() const;

    AttitudeDeg getAttitudeSetpoint() const;
    AngularRateDps getRateSetpoint() const;

    AngularRateDps getAttitudeCmd() const;
    AxisCommand getRateCmd() const;

    LoopStats getOuterLoopStats();
    LoopStats getInnerLoopStats();

    // ─────────────────────────────────────────────────────────────────
    // Runtime Configuration Updates
    // ─────────────────────────────────────────────────────────────────

    /**
     * @brief Set the PID configuration for a rate (inner loop) axis.
     * @param loop The control loop type (RATE_ROLL_LOOP, RATE_PITCH_LOOP, RATE_YAW_LOOP)
     * @param config The PID configuration
     */
    void setRatePIDConfig(ControlLoopType loop, const PIDConfig& config);

    /**
     * @brief Set the PID configuration for an attitude (outer loop) axis.
     * @param loop The control loop type (ATTITUDE_ROLL_LOOP, ATTITUDE_PITCH_LOOP, ATTITUDE_YAW_LOOP)
     * @param config The PID configuration
     */
    void setAttitudePIDConfig(ControlLoopType loop, const PIDConfig& config);

    /**
     * @brief Set the rate controller output low-pass filter alpha.
     * @param alpha Filter alpha (0.0-1.0)
     */
    void setRateOutputAlpha(float alpha);

    /**
     * @brief Set the attitude controller deadband.
     * @param deadband Deadband in radians
     */
    void setAttitudeDeadband(float deadband);

private:
    arduflite::estimation::InertialSubsystem* imu;                                      //< Pointer to the IMU instance.
    ArduFliteAttitudeController* attitudeCtrl;              //< Pointer to the outer loop controller.
    ArduFliteRateController* rateCtrl;                      //< Pointer to the inner loop controller.
    arduflite::actuators::ControlOutputs surfaces{};        //< Resolved by role at setPlatform().
    arduflite::actuators::WingDesign      wingDesign =
        arduflite::actuators::WingDesign::Conventional;     //< From ConfigRegistry.

    /// Set by pauseTasks(), read by both control loops every iteration. The
    /// loops keep running and keep feeding the watchdog while it is set; they
    /// simply skip their body — see pauseTasks().
    std::atomic<bool> tasksPaused{ false };

    ArduFliteMode mode;                                     //< Current operating mode.
    AngularRateDps pilotRateSetpoint{};                     //< Pilot rate setpoint for RATE_MODE.
    AxisCommand    pilotManualCommand{};                    //< Pilot surface demand for MANUAL_MODE.
    AttitudeDeg pilotAttitudeSetpoint   {};                 //< Pilot attitude setpoint for ATTITUDE_MODE.
    float       pilotThrottleSetpoint   = 0.0f;             //< Pilot throttle setpoint for all modes.

    // Shared command variables
    AngularRateDps lastAttitudeCmd      {};   //< attitude loop OUTPUT: a rate setpoint
    AxisCommand lastRateCmd             {};   //< rate loop OUTPUT: normalised axis demand

    // Statistics for the outer and inner loops.
    LoopStats outerLoopStats            = {0, 0, 0, 0};
    LoopStats innerLoopStats            = {0, 0, 0, 0};

    Platform plat;                                          //< Injected platform services.

    /**
     * @brief Arm and throttle-cut state. ATOMIC, deliberately not under ctrlMutex.
     *
     * Both are safety gates and must not be able to fail to close. Behind a
     * bounded wait, cutThrottle() would do nothing on a timeout, and the control
     * loop would act on a stale copy for any tick that missed the lock — so a
     * disarm would not take effect until contention cleared.
     *
     * They are independent booleans, not part of the setpoint group's
     * coherence, so nothing is lost by keeping them out of the mutex.
     */
    std::atomic<bool> armed{ false };
    std::atomic<bool> throttleCut{ true };   //< Throttle is cut by default
    // IMU failure recovery state
    ArduFliteMode savedModeBeforeImuFailure = ATTITUDE_MODE; //< Mode to restore when IMU recovers
    bool imuFailureActive = false;                          //< True when in IMU failure MANUAL_MODE
    static constexpr std::uint32_t outerLoopMs = 10;
    static constexpr std::uint32_t innerLoopMs = 2;

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
    /// Single fatal path, so the platform reboot call appears once.
    [[noreturn]] void fatal(const char* what);

    static void updateLoopStats(LoopStats &stats, unsigned long dtMicro, unsigned long desiredPeriodMicro);

    /// Measured against the deepest call path in each loop. A stack overflow
    /// here presents as a watchdog reset with no useful trace, so change these
    /// only with a high-water-mark measurement to back it up.
    /// Bounded wait for the control loops: long enough to ride out normal
    /// contention, short enough that a held lock cannot stall a 500 Hz loop.
    static constexpr std::chrono::milliseconds kLockTimeout{ 5 };

    static constexpr std::uint32_t kOuterStackBytes = 4096;
    static constexpr std::uint32_t kInnerStackBytes = 4096;
};

#endif // ARDU_FLITE_CONTROLLER_H
