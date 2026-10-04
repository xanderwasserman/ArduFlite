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
#include "src/utils/ConfigRegistry.h"
#include "include/ConfigKeys.h"
#include "src/utils/PreflightCheck.h"
#include "src/hal/device/RcLink.h"

#include <cmath>
#include <chrono>
#include <mutex>

namespace
{
/// Non-blocking. The 500 Hz inner loop must never stall on a lock; a missed
/// acquire skips the tick and reuses the last good value. Zero maps to a plain
/// try_lock() in Esp32Mutex.
constexpr std::chrono::milliseconds kHotPathLockTimeout{ 0 };
}

/**
 * @brief Constructor for ArduFliteController.
 *
 * Initializes the controller with pointers to the shared components: IMU,
 * attitude controller, rate controller, and ServoManager. Also initializes the
 * operating mode to ATTITUDE_MODE and creates a mutex to protect shared state.
 *
 * @param imu Pointer to the arduflite::estimation::InertialSubsystem instance.
 * @param attitudeCtrl Pointer to the ArduFliteAttitudeController instance.
 * @param rateCtrl Pointer to the ArduFliteRateController instance.
 */
void ArduFliteController::fatal(const char* what)
{
    LOG_ERR("FATAL: %s - system will restart.", what);
    if (plat.system != nullptr) { plat.system->reboot(); }
    // No System injected yet: the only remaining option is to stop, so the
    // hardware watchdog resets us rather than flying on a broken controller.
    for (;;) { }
}

void ArduFliteController::setPlatform(const Platform& platform)
{
    plat = platform;

    // Resolve actuators by role ONCE. Per tick the controller then holds named
    // pointers — no index lookup, no index/role mismatch possible.
    if (plat.outputs != nullptr)
    {
        surfaces = arduflite::actuators::ControlOutputs::resolve(*plat.outputs);
        if (!surfaces.hasRequiredSurfaces())
        {
            fatal("board descriptor has no aileron_left/elevator role.");
        }
    }
}

void ArduFliteController::stageTestDeflection(float normalised)
{
    if (plat.outputs == nullptr) { return; }

    const auto mixed = arduflite::actuators::AirframeMixer::mix(
        wingDesign, normalised, normalised, normalised);
    surfaces.stageSurfaces(mixed);
    surfaces.stageThrottle(0.0f);   // never spin the motor during a surface test
    (void)plat.outputs->commit();
}

void ArduFliteController::initFromConfig()
{
    const int32_t wd = ConfigRegistry::instance().get<int32_t>(CONFIG_KEY_SERVO_WING_DESIGN);
    const auto requested = static_cast<arduflite::actuators::WingDesign>(wd);

    if (!arduflite::actuators::isImplemented(requested))
    {
        // Falling back is safer than mixing to nothing: an aircraft whose
        // surfaces never move is worse than one flying the wrong geometry,
        // because the pilot gets no feedback that anything is wrong.
        LOG_ERR("Wing design %d is NOT IMPLEMENTED - falling back to Conventional. "
                "Do not fly this airframe until it is.", static_cast<int>(wd));
        wingDesign = arduflite::actuators::WingDesign::Conventional;
    }
    else
    {
        wingDesign = requested;
    }
    LOG_INF("ArduFliteController: wing design %d", static_cast<int>(wingDesign));
}

ArduFliteController::ArduFliteController(arduflite::estimation::InertialSubsystem* imu, ArduFliteAttitudeController* attitudeCtrl, ArduFliteRateController* rateCtrl)
    : imu(imu)
    , attitudeCtrl(attitudeCtrl)
    , rateCtrl(rateCtrl)
    , mode(ATTITUDE_MODE)
    , pilotRateSetpoint{ 0.0f, 0.0f, 0.0f }
{
    // Mutexes are injected from the Board's pool — flight code cannot construct
    // an Esp32Mutex without breaking the layering rule, and the pool lives in
    // static storage (ADR-011: no heap after boot). Allocation failure is caught
    // at composition time in arduflite_init(), not here.

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
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
    ArduFliteMode m = UNKNOWN_MODE;

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return m;
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
 * @param setpoint AttitudeDeg attitude setpoint, in degrees.
 */
void ArduFliteController::setAttitudeSetpoint(AttitudeDeg setpointDeg)
{
    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
    AttitudeDeg localCopy;

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
        switch(axis) {
            case 0: pilotAttitudeSetpoint.roll  = value; break;
            case 1: pilotAttitudeSetpoint.pitch = value; break;
            case 2: pilotAttitudeSetpoint.yaw   = value; break;
            default: return;
        }
        localCopy = pilotAttitudeSetpoint;
    }
    attitudeCtrl->setAttitudeControlSetpoint(localCopy);
}

/**
 * @brief Sets the pilot-provided rate setpoints for Stabilized mode.
 *
 * When operating in Stabilized mode, these values are used directly as the desired
 * angular rates.
 *
 * @param setpoint AngularRateDps rate setpoint, in degrees/s.
 */
void ArduFliteController::setManualCommand(AxisCommand command)
{
    std::unique_lock lock(*plat.ctrlMutex, kHotPathLockTimeout);
    if (!static_cast<bool>(lock)) { return; }
    pilotManualCommand = command;

    // Clear the rate setpoint while flying manually. Before the split these
    // shared one slot, so telemetry's rate_sp_* columns showed the manual stick
    // positions; with separate slots that column would instead hold whatever
    // rate was commanded before MANUAL_MODE was entered — stale, and more
    // misleading in a log than a zero. The pilot's actual manual input is still
    // recorded, through rate_cmd_*, which carries the resulting AxisCommand.
    pilotRateSetpoint = {};
}

void ArduFliteController::setRateSetpoint(AngularRateDps rateSetpoint)
{
    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
        pilotRateSetpoint.yaw  = yawRateSetpoint;
    }
}

/**
 * @brief Sets the pilot-provided throttle setpoint.
 *
 * @param throttleSetpoint Throttle setpoint in percentage/100 (0.0 - 1.0).
 */
void ArduFliteController::setThrottleSetpoint(float throttleSetpoint)
{
    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return;
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
    if (!plat.valid())
    {
        fatal("startTasks() before setPlatform()");
    }

    using arduflite::hal::Priority;
    using arduflite::hal::TaskConfig;

    // Priorities come from the ladder enum rather than the magic 2 and 3 that
    // Stack sizes and priorities come from the header.
    const TaskConfig outerCfg{ "OuterLoop", kOuterStackBytes, Priority::OuterLoop, -1 };
    const TaskConfig innerCfg{ "InnerLoop", kInnerStackBytes, Priority::InnerLoop, -1 };

    auto outer = plat.scheduler->spawn(outerCfg, OuterLoopTask, this);
    if (!outer)
    {
        LOG_ERR("FATAL: OuterLoopTask creation failed (%s) — system will restart.",
                arduflite::toString(outer.status()));
        fatal("unrecoverable controller state");
    }

    auto inner = plat.scheduler->spawn(innerCfg, InnerLoopTask, this);
    if (!inner)
    {
        LOG_ERR("FATAL: InnerLoopTask creation failed (%s) — system will restart.",
                arduflite::toString(inner.status()));
        fatal("unrecoverable controller state");
    }

}

/**
 * @brief Stops the control loops acting, without stopping the tasks.
 *
 * Used during calibration: the airframe must be still, and live surfaces
 * responding to stick input while someone holds the aircraft is a finger
 * hazard. The loops keep running and keep feeding the watchdog; they just do
 * no work and drive no outputs.
 */
void ArduFliteController::pauseTasks()
{
    // A flag, not a task suspend. The loops keep running and skip their body,
    // which is what lets them stay watchdog-registered and keep feeding through
    // a multi-second calibration. `dt` never grows, and the PID integrators are
    // not advanced, so the pause cannot wind them up. Surfaces hold their last
    // commanded position because the skipped body is what calls commit().
    tasksPaused.store(true, std::memory_order_release);
    LOG_INF("Control tasks paused (loops idling, watchdog still fed).");
}

/**
 * @brief Resumes the control tasks (outer and inner loop).
 *
 * Lets the control loops act again, so that the
 * attitude and rate controllers continue their normal operation.
 */
void ArduFliteController::resumeTasks()
{
    tasksPaused.store(false, std::memory_order_release);
    LOG_INF("Control tasks resumed.");
}

/**
 * @brief Arm the controller after passing preflight checks.
 *
 * Runs preflight validation before arming. Will reject arm request if
 * any preflight check fails.
 *
 * @param receiver Pointer to CRSF receiver for link quality check (may be nullptr)
 * @return true if arm succeeded, false if preflight check failed
 */
bool ArduFliteController::arm(arduflite::device::RcLink* rcLink)
{
    // Detect whether the aircraft is already in flight so we can skip the
    // ground-specific preflight checks (gyro bias, 1g accel, throttle cut)
    // that will always fail on a moving, airborne airframe.
    const bool isInflight = (getFlightState() == INFLIGHT);
    const ArmContext context = isInflight
                               ? ArmContext::INFLIGHT_REARM
                               : ArmContext::GROUND_ARM;

    if (isInflight)
    {
        LOG_WARN("Arm requested INFLIGHT — using reduced preflight checks.");
    }
    else
    {
        LOG_INF("Arm requested - running full preflight checks...");
    }

    // Run appropriate preflight checks for this context
    PreflightResult preflight = PreflightCheck::runAllChecks(imu, this, rcLink, context);

    if (!preflight.allPassed())
    {
        LOG_ERR("ARM REJECTED - Preflight checks failed!");
        return false;
    }

    // Reset controllers OUTSIDE ctrlMutex to preserve lock ordering.
    // arm() must not hold ctrlMutex while entering attitudeMutex inside reset()
    // (would create an undocumented ctrlMutex → attitudeMutex ordering that could deadlock).
    rateCtrl->reset();
    attitudeCtrl->reset();

    armed.store(true, std::memory_order_release);

    LOG_INF("ARMED - System ready for flight");
    return true;
}

void ArduFliteController::disarm()
{
    // Cannot fail and cannot be delayed by contention.
    armed.store(false, std::memory_order_release);

    // disable() applies each channel's FailsafeAction and latches the bank, so
    // a control loop cannot re-drive surfaces afterwards. Held no lock while
    // doing it: I/O under ctrlMutex is forbidden (AGENTS.md, Common Pitfalls).
    if (plat.outputs) { (void)plat.outputs->disable(); }
}

bool ArduFliteController::isArmed() const
{
    return armed.load(std::memory_order_acquire);
}

void ArduFliteController::cutThrottle(bool value)
{
    throttleCut.store(value, std::memory_order_release);
}

bool ArduFliteController::isThrottleCut() const
{
    return throttleCut.load(std::memory_order_acquire);
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
    std::uint64_t lastWake = 0;   // opaque Scheduler state
    const std::chrono::milliseconds period{ outerLoopMs };   // 10 ms period (100Hz)
    auto lastTick = controller->plat.clock->now();

    // Registered for the task's whole lifetime; unregistered however it exits.
    arduflite::hal::WatchdogGuard wdt(*controller->plat.watchdog);

    // Desired period in microseconds for the outer loop (10 ms = 10,000 µs)
    const unsigned long desiredPeriodOuter = outerLoopMs *1000UL;

    AngularRateDps rateCommand{};   // attitude loop output = a RATE setpoint

    // Shadow copies - persist across iterations for fallback on mutex timeout
    AngularRateDps rateSetpoint{};
    ArduFliteMode currentMode   = ATTITUDE_MODE;

    while(1)
    {
        // BEFORE the pause check: a paused loop must still feed the watchdog,
        // or a long calibration trips it.
        wdt.feed();

        if (controller->tasksPaused.load(std::memory_order_acquire))
        {
            // Re-seeded so the first dt after resuming measures one period
            // rather than the whole pause.
            lastTick = controller->plat.clock->now();
            controller->plat.scheduler->sleepUntil(lastWake, period);
            continue;
        }

        const auto    nowTick     = controller->plat.clock->now();
        // 64-bit chrono: no 71-minute wrap. LoopStats stays in microseconds so
        // the `stats` CLI output and the Phase 2 A/B comparison are unchanged.
        const auto    elapsed     = nowTick - lastTick;
        unsigned long dtMicro     = static_cast<unsigned long>(elapsed.count());
        lastTick = nowTick;

        {
            std::unique_lock lock(*controller->plat.outerStatsMutex, kHotPathLockTimeout);
            if (static_cast<bool>(lock)) {
                updateLoopStats(controller->outerLoopStats, dtMicro, desiredPeriodOuter);
            }
            // Stats update is non-critical, skip on timeout
        }

        float dt = dtMicro / 1000000.0f;
        const float maxDt = 0.02f;  // 20 ms max dt

        if (dt < 1e-3f) dt = 1e-3f;
        if (dt > maxDt) dt = maxDt;

        // Protect reading of the mode and pilot setpoints.
        // On timeout, use shadow copies from previous iteration (fail soft)
        {
            std::unique_lock lock(*controller->plat.ctrlMutex, kHotPathLockTimeout);
            if (static_cast<bool>(lock)) {
                currentMode = controller->mode;
                rateSetpoint = controller->pilotRateSetpoint;
            }
            // If timeout, shadow copies retain values from previous iteration
        }

        // Keep attitude and rate integrators at zero during PREFLIGHT/LANDED to prevent
        // I-term windup while the aircraft is idle on the ground before launch.
        // The instant flight state transitions to INFLIGHT, integrals start from 0.
        // Cache the state once to avoid a TOCTOU race between the two getFlightState() reads.
        const FlightState fs = getFlightState();
        if (fs == PREFLIGHT || fs == LANDED)
        {
            controller->attitudeCtrl->resetIntegrals();
            controller->rateCtrl->resetIntegrals();
        }

        if (currentMode == ATTITUDE_MODE)
        {
            // In Assist mode, use the attitude controller to compute desired rates.
            // One lock-free read. Two separate ones could straddle a tick and
            // pair an attitude with a rate from a different sample.
            const arduflite::estimation::ImuState imuState = controller->imu->state();
            const FliteQuaternion currentQ(imuState.orientation_quat.w, imuState.orientation_quat.x,
                                           imuState.orientation_quat.y, imuState.orientation_quat.z);
            controller->attitudeCtrl->update(currentQ, dt, rateCommand);

            // Write back for telemetry (non-critical if missed)
            {
                std::unique_lock lock(*controller->plat.ctrlMutex, kHotPathLockTimeout);
                if (static_cast<bool>(lock)) {
                    controller->lastAttitudeCmd = rateCommand;
                }
            }

            // Pass the desired angular rates to the rate controller.
            controller->rateCtrl->setRateControlSetpoint(rateCommand);
        }
        else if (currentMode == RATE_MODE)
        {
            // In Stabilized mode, use pilot-provided rate setpoints.
            rateCommand  = rateSetpoint;

            // Write back for telemetry (non-critical if missed)
            {
                std::unique_lock lock(*controller->plat.ctrlMutex, kHotPathLockTimeout);
                if (static_cast<bool>(lock)) {
                    controller->lastAttitudeCmd = rateCommand;
                }
            }

            // Pass the desired angular rates to the rate controller.
            controller->rateCtrl->setRateControlSetpoint(rateCommand);
        }
        // else if MANUAL_MODE: do nothing here (we bypass both loops)

        controller->plat.scheduler->sleepUntil(lastWake, period);
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
    std::uint64_t lastWake = 0;   // opaque Scheduler state
    const std::chrono::milliseconds period{ innerLoopMs };   // 2 ms period (500Hz)
    auto lastTick = controller->plat.clock->now();

    // Registered for the task's whole lifetime; unregistered however it exits.
    arduflite::hal::WatchdogGuard wdt(*controller->plat.watchdog);

    // Desired period in microseconds for the inner loop (2 ms = 2,000 µs)
    const unsigned long desiredPeriodInner = innerLoopMs * 1000UL;

    // Shadow copies - persist across iterations for fallback on mutex timeout
    ArduFliteMode   localMode           = ATTITUDE_MODE;
    AngularRateDps  localRateSetpoint{};
    AxisCommand     localManualCommand{};
    float           localThrottle       = 0.0f;
    AxisCommand     actuatorCmd{};   // normalised -1..+1, straight to the mixer

    while(1)
    {
        // BEFORE the pause check — see the outer loop.
        wdt.feed();

        if (controller->tasksPaused.load(std::memory_order_acquire))
        {
            lastTick = controller->plat.clock->now();
            controller->plat.scheduler->sleepUntil(lastWake, period);
            continue;
        }

        const auto    nowTick     = controller->plat.clock->now();
        // 64-bit chrono: no 71-minute wrap. LoopStats stays in microseconds so
        // the `stats` CLI output and the Phase 2 A/B comparison are unchanged.
        const auto    elapsed     = nowTick - lastTick;
        unsigned long dtMicro     = static_cast<unsigned long>(elapsed.count());
        lastTick = nowTick;

        // Update inner loop statistics (separate mutex, low contention)
        {
            std::unique_lock lock(*controller->plat.innerStatsMutex, kHotPathLockTimeout);
            if (static_cast<bool>(lock)) {
                updateLoopStats(controller->innerLoopStats, dtMicro, desiredPeriodInner);
            }
            // Stats update is non-critical, skip on timeout
        }

        float dt = dtMicro / 1000000.0f;
        const float maxDt = 0.02f;  // 20 ms max dt

        if (dt < 1e-3f) dt = 1e-3f;
        if (dt > maxDt) dt = maxDt;

        // ─────────────────────────────────────────────────────────────────
        // Single consolidated ctrlMutex acquisition: reads all controller state
        // AND handles IMU health check logic in one critical section.
        // IMU health is sampled outside the lock (lock-free IMU API), then
        // the health state machine runs inside — prevents a second acquisition.
        // On timeout, shadow copies retain values from the previous iteration.
        // ─────────────────────────────────────────────────────────────────
        bool imuHealthy = controller->imu->healthy();  // lock-free read
        bool shouldEnterFailure = false;
        bool shouldExitFailure  = false;
        ArduFliteMode modeToRestore = MANUAL_MODE;

        // Read OUTSIDE the lock, and deliberately not shadowed: these are the
        // two safety gates, and a tick that cannot take the lock must still
        // observe a disarm or a throttle cut the moment it happens.
        const bool localArmed       = controller->armed.load(std::memory_order_acquire);
        const bool localThrottleCut = controller->throttleCut.load(std::memory_order_acquire);

        {
            std::unique_lock lock(*controller->plat.ctrlMutex, kHotPathLockTimeout);
            if (static_cast<bool>(lock)) {
                localMode           = controller->mode;
                localRateSetpoint   = controller->pilotRateSetpoint;
                localManualCommand  = controller->pilotManualCommand;
                localThrottle       = controller->pilotThrottleSetpoint;
                // IMU health state machine — inline with state read to avoid second lock
                if (!imuHealthy && !controller->imuFailureActive)
                {
                    if (controller->mode != MANUAL_MODE) {
                        controller->savedModeBeforeImuFailure = controller->mode;
                    }
                    controller->imuFailureActive = true;
                    controller->mode = MANUAL_MODE;
                    localMode = MANUAL_MODE;

                    // Retained as defence in depth. The type split (ADR-037)
                    // means MANUAL_MODE now reads its own AxisCommand slot, so a
                    // deg/s value can no longer reach the mixer at all. But that
                    // slot still holds whatever manual command was last sent,
                    // which may be stale from before RATE_MODE was entered.
                    // Centre for one tick; the next ControlMixer tick refreshes.
                    localManualCommand = {};
                    shouldEnterFailure = true;
                }
                else if (imuHealthy && controller->imuFailureActive)
                {
                    modeToRestore = controller->savedModeBeforeImuFailure;
                    controller->imuFailureActive = false;
                    controller->mode = modeToRestore;
                    localMode = modeToRestore;

                    // Mirror of the demotion above: the rate slot may hold a
                    // setpoint from before the failure. Centre for one tick.
                    localRateSetpoint = {};
                    shouldExitFailure = true;
                }
            }
        }

        if (shouldEnterFailure)
        {
            LOG_ERR("IMU FAILURE - switching to MANUAL_MODE for pilot control!");
        }
        else if (shouldExitFailure)
        {
            LOG_INF("IMU RECOVERED - restoring previous mode (%d)", modeToRestore);
        }
        // ─────────────────────────────────────────────────────────────────

        // Retrieve measured angular rates from the IMU (lock-free versioned snapshot)
        // Skip IMU read in MANUAL_MODE to avoid blocking on failed I2C bus
        AngularRateDps gyro{};
        if (localMode != MANUAL_MODE)
        {
            gyro = toAngularRateDps(controller->imu->state().gyro_dps);
        }

        if (localMode == MANUAL_MODE)
        {
            // Its own slot, already the right quantity. No conversion, and no
            // possibility of a rate arriving here (ADR-037).
            actuatorCmd = localManualCommand;
        }
        else
        {
            // Guard against NaN/Inf gyro — a failed I2C read can produce NaN which
            // would permanently corrupt the PID integrators (NaN + x = NaN forever).
            // ServoManager catches NaN at output, but integrators must stay clean.
            if (!isfinite(gyro.roll) || !isfinite(gyro.pitch) || !isfinite(gyro.yaw))
            {
                LOG_ERR("InnerLoop: NaN/Inf in gyro data — skipping rate controller update.");
            }
            else
            {
                controller->rateCtrl->update(gyro, dt, actuatorCmd);
            }
        }

        // Only actually drive servos if we’re armed:
        if (localArmed)
        {
            const auto mixed = arduflite::actuators::AirframeMixer::mix(
                controller->wingDesign,
                actuatorCmd.roll, actuatorCmd.pitch, actuatorCmd.yaw);
            controller->surfaces.stageSurfaces(mixed);

            // Only drive ESC's if the throttle cut is disabled as well
            if (!localThrottleCut)
            {
                controller->surfaces.stageThrottle(localThrottle);
            }
            else
            {
                controller->surfaces.stageThrottle(0.0f);
            }
        }
        else
        {
            // hold neutral
            controller->surfaces.stageSurfaces({});

            // hold throttle off
            controller->surfaces.stageThrottle(0.0f);
        }

        // Push every staged command to hardware in ONE batched operation.
        // stage() is purely local; this is what reaches the actuators, and for a
        // CAN bank it would be the PDO group plus SYNC. Reporting is per-channel
        // via staleMask so a stale elevator and a stale flap can be told apart.
        {
            const auto result = controller->plat.outputs->commit();
            if (!result.allOk() && localArmed)
            {
                static arduflite::hal::Clock::time_point lastWarn{};
                const auto nowT = controller->plat.clock->now();
                if (nowT - lastWarn > std::chrono::seconds{ 1 })
                {
                    LOG_ERR("Actuator commit incomplete: status=%s staleMask=0x%08lX",
                            arduflite::toString(result.status),
                            static_cast<unsigned long>(result.staleMask));
                    lastWarn = nowT;
                }
            }
        }

        // Write back actuator command (for telemetry) — non-blocking tryLock
        {
            std::unique_lock lock(*controller->plat.ctrlMutex, kHotPathLockTimeout);
            if (static_cast<bool>(lock)) {
                controller->lastRateCmd = actuatorCmd;
            }
            // Non-critical if missed — telemetry will show slightly stale data
        }

        controller->plat.scheduler->sleepUntil(lastWake, period);
    }
}

AttitudeDeg ArduFliteController::getAttitudeSetpoint() const
{
    AttitudeDeg value{};

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return value;
        value  = pilotAttitudeSetpoint;
    }

    return value;
}

AngularRateDps ArduFliteController::getRateSetpoint() const
{
    AngularRateDps value{};

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return value;
        value  = pilotRateSetpoint;
    }

    return value;
}

AxisCommand ArduFliteController::getRateCmd() const
{
    AxisCommand value{};

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return value;
        value = lastRateCmd;
    }

    return value;
}

AngularRateDps ArduFliteController::getAttitudeCmd() const
{
    AngularRateDps value{};

    {
        std::unique_lock lock(*plat.ctrlMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return value;
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
    LoopStats statsCopy{0, 0, 0, 0};

    {
        std::unique_lock lock(*plat.outerStatsMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return statsCopy;
        statsCopy = outerLoopStats;
    }

    return statsCopy;
}

LoopStats ArduFliteController::getInnerLoopStats()
{
    LoopStats statsCopy{0, 0, 0, 0};

    {
        std::unique_lock lock(*plat.innerStatsMutex, kLockTimeout);
        if (!static_cast<bool>(lock)) return statsCopy;
        statsCopy = innerLoopStats;
    }

    return statsCopy;
}

// ─────────────────────────────────────────────────────────────────
// Runtime Configuration Updates
// ─────────────────────────────────────────────────────────────────

void ArduFliteController::setRatePIDConfig(ControlLoopType loop, const PIDConfig& config)
{
    if (rateCtrl != nullptr)
    {
        rateCtrl->setPIDConfig(loop, config);
    }
}

void ArduFliteController::setAttitudePIDConfig(ControlLoopType loop, const PIDConfig& config)
{
    if (attitudeCtrl != nullptr)
    {
        attitudeCtrl->setPIDConfig(loop, config);
    }
}

void ArduFliteController::setRateOutputAlpha(float alpha)
{
    if (rateCtrl != nullptr)
    {
        rateCtrl->setOutputAlpha(alpha);
    }
}

void ArduFliteController::setAttitudeDeadband(float deadband)
{
    if (attitudeCtrl != nullptr)
    {
        attitudeCtrl->setDeadband(deadband);
    }
}
