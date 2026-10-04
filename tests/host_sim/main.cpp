/**
 * main.cpp — host_sim
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief The flight stack's estimation core, running on a laptop.
 *
 * Extensibility proof 2 (§06 Phase 8). No Arduino, no FreeRTOS, no ESP32, no
 * hardware — the same InertialSubsystem, MadgwickEstimator,
 * FirstHealthySelector, CalibrationService and AirframeMixer that fly, driven
 * by simulated sensors through the same device:: interfaces.
 *
 * What it proves: nothing in the estimation layer is coupled to the target.
 * What it does NOT prove: that the whole application runs on a host. It does
 * not — see the note at the end of main(), and §06 Phase 8.
 */
#include <cstdio>
#include <cmath>

#include "hal_host/HostPlatform.h"
#include "src/actuators/AirframeMixer.h"
#include "src/estimation/MadgwickEstimator.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/estimation/SensorSelector.h"
#include "src/utils/ConfigRegistry.h"
#include "tests/host_sim/SimSensors.h"

using namespace arduflite;
using namespace arduflite::estimation;

int main()
{
    // ── Platform: all host implementations ──────────────────────────────────
    hal::host::VirtualClock             clock;
    hal::host::RecordingScheduler       scheduler;
    hal::host::NullWatchdog             watchdog;
    hal::host::MemorySettingsStore      settings;

    // ── Sensors ─────────────────────────────────────────────────────────────
    sim::SimImu       imu;
    sim::SimBarometer baro;

    std::array<device::Sensor*, 2>        devices{ &imu, &baro };
    std::array<device::Accelerometer*, 1> accels { &imu };
    std::array<device::Gyroscope*, 1>     gyros  { &imu };
    std::array<device::Barometer*, 1>     baros  { &baro };

    FirstHealthySelector      selector{ accels, gyros, baros };
    MadgwickEstimator estimator;

    InertialSubsystem subsystem{ InertialSubsystem::Dependencies{
        devices, selector, estimator, clock, scheduler, watchdog, settings } };

    InertialSubsystem::Config config;
    config.taskRate_hz = 500;
    config.accelAlpha  = 0.2f;
    config.gyroAlpha   = 0.2f;
    config.altiAlpha   = 0.2f;
    // Mounted the way the prototype is: a reflection, det = -1.
    config.axes = AxisMap{ SignedAxis::PlusX, SignedAxis::MinusY, SignedAxis::PlusZ };
    subsystem.configure(config);

    estimator.begin(static_cast<float>(config.taskRate_hz));
    estimator.setBeta(0.1f);

    printf("=== ArduFlite host_sim ===\n");
    printf("No Arduino, no FreeRTOS, no ESP32.\n\n");

    // ── Settle, then fly a roll ─────────────────────────────────────────────
    baro.altitude_m = 100.0f;

    constexpr float kDt = 1.0f / 500.0f;
    imu.dt_s = kDt;

    for (int i = 0; i < 2000; ++i) { clock.advanceMs(2); subsystem.tick(kDt); }
    subsystem.resetFilters();

    printf("after settle:   roll=%7.2f deg  (truth %7.2f)  alt=%7.2f m\n",
           subsystem.state().euler_deg.roll, imu.roll_deg(), subsystem.state().altitude_m);

    imu.rollRate_dps = 30.0f;                       // roll right at 30 deg/s
    for (int i = 0; i < 1000; ++i) { clock.advanceMs(2); subsystem.tick(kDt); }
    imu.rollRate_dps = 0.0f;
    for (int i = 0; i < 1000; ++i) { clock.advanceMs(2); subsystem.tick(kDt); }

    const ImuState state = subsystem.state();

    // The estimator sees the body-frame gyro through the axis map, which negates
    // roll rate for this mount — so the reported sign is inverted relative to
    // the simulated truth. That is the transform doing its job, not an error.
    printf("after roll:     roll=%7.2f deg  (truth %7.2f, sign flipped by the axis map)\n",
           state.euler_deg.roll, imu.roll_deg());
    printf("                alt=%7.2f m   climb=%6.2f m/s   healthy=%s\n",
           state.altitude_m, state.climbRate_mps, state.healthy ? "yes" : "no");
    printf("                imu samples=%d  baro samples=%d  (baro decimated by its own rate)\n",
           imu.sampleCount, baro.sampleCount);

    bool ok = true;

    // ── The control loops, on the laptop ────────────────────────────────────
    // Assembling these on a laptop works because they take an injected
    // hal::Mutex rather than creating one — here, a std::mutex behind the same
    // interface.
    hal::host::HostMutex attitudeMutex;
    hal::host::HostMutex rateMutex;

    ArduFliteAttitudeController attitude;
    ArduFliteRateController     rate;
    attitude.setMutex(&attitudeMutex);
    rate.setMutex(&rateMutex);

    // Drain the schema's static registrations. On the target this happens in
    // arduflite_init() once FreeRTOS is up; here there is no FreeRTOS, and the
    // registry no longer needs any (ADR-048).
    ConfigRegistry::instance().init();

    // Gains are the aircraft's real defaults, from src/utils/ConfigSchema.cpp.
    attitude.initFromConfig();
    rate.initFromConfig();

    // Ask for wings level and let the loops fly it back.
    attitude.setAttitudeControlSetpoint(AttitudeDeg{ 0.0f, 0.0f, 0.0f });

    AngularRateDps rateCommand{};
    AxisCommand    surfaceCommand{};

    for (int i = 0; i < 500; ++i)
    {
        const ImuState s = subsystem.state();
        const FliteQuaternion q(s.orientation_quat.w, s.orientation_quat.x,
                                s.orientation_quat.y, s.orientation_quat.z);

        attitude.update(q, kDt, rateCommand);          // outer loop -> a RATE
        rate.setRateControlSetpoint(rateCommand);
        rate.update(toAngularRateDps(s.gyro_dps), kDt, surfaceCommand);
    }

    printf("control loops:  rateCmd roll=%6.2f dps   surfaceCmd roll=%6.3f\n",
           rateCommand.roll, surfaceCommand.roll);

    // Rolled left, so the loops must command a roll back to the right.
    if (surfaceCommand.roll <= 0.0f)
    {
        printf("FAIL: control output does not oppose the roll\n");
        ok = false;
    }

    // ── Through the control surface mixer ───────────────────────────────────
    const auto surfaces = actuators::AirframeMixer::mix(
        actuators::WingDesign::Conventional,
        state.euler_deg.roll / 45.0f, state.euler_deg.pitch / 45.0f, 0.0f);
    printf("mixed surfaces: ailL=%5.2f ailR=%5.2f elev=%5.2f rud=%5.2f\n\n",
           surfaces.aileronLeft, surfaces.aileronRight, surfaces.elevator, surfaces.rudder);

    // ── Sanity gates ────────────────────────────────────────────────────────
    if (!state.healthy)                                  { printf("FAIL: unhealthy\n");        ok = false; }
    if (std::fabs(state.altitude_m - 100.0f) > 5.0f)     { printf("FAIL: altitude drift\n");   ok = false; }
    if (std::fabs(state.euler_deg.roll) < 20.0f)         { printf("FAIL: roll not tracked\n"); ok = false; }
    if (baro.sampleCount >= imu.sampleCount)             { printf("FAIL: baro not decimated\n"); ok = false; }

    printf("%s\n", ok ? "host_sim OK" : "host_sim FAILED");

    // What is NOT here, and why: ArduFliteController owns the FreeRTOS tasks,
    // and ArduFliteAttitudeController / ArduFliteRateController / ControlMixer
    // still create raw xSemaphoreCreateMutex() handles rather than taking a
    // hal::Mutex. Those three were never migrated in Phase 2. Until they are,
    // the control loops cannot be assembled here — see §06 Phase 8.
    return ok ? 0 : 1;
}
