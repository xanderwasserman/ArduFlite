/*
 * ----------------------------------------------------------------------------
 * ArduFlite - Advanced Flight Controller Framework
 * ----------------------------------------------------------------------------
 *
 * Project: ArduFlite
 * Description:
 *   ArduFlite is a highly modular and real-time flight control framework for
 *   unmanned aerial vehicles (UAVs) and gliders. Built on the ESP32 and FreeRTOS,
 *   it integrates IMU sensor fusion, cascade PID control (attitude and rate loops),
 *   servo management for multiple wing designs, and telemetry (via CRSF and Flash)
 *   along with a flexible command-line interface (CLI) for live diagnostics
 *   and configuration.
 *
 * Author: Alexander Wasserman
 * Version: 1.0
 * Date: 08 April 2025
 *
 * License: MIT License
 * ----------------------------------------------------------------------------
 */
#include "include/ArduFlite.h"

#include <esp_task_wdt.h>  // ESP32 hardware watchdog
#include <esp_system.h>    // For esp_reset_reason()

#include "src/hal/drivers/out/PwmActuatorBank.h"
#include "src/actuators/AirframeMixer.h"
#include "src/core/FlightTypes.h"
#include "src/estimation/MadgwickEstimator.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/estimation/SensorSelector.h"
#include <optional>
#include "src/cli/ArduFliteCLI.h"
#include "src/mission_planner/MissionPlanner.h"
#include "src/state/StateManagement.h"

#include "src/utils/ControlMixer.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/Colors.h"
#include "src/utils/ButtonCallbacks.h"
#include "src/utils/Logging.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/ConfigTask.h"
#include "src/utils/ConfigObservers.h"
#include "include/ConfigKeys.h"

#include "src/hal/board/Board.h"

#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/controller/ArduFliteController.h"

#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"

#include "src/hal/drivers/rc/CrsfLink.h"
#include "src/receiver/crsf/ArdufliteCRSFCallbacks.h"
#include "src/input/RcMapper.h"

#include "include/WebConfiguration.h"
#if ENABLE_WEB_SERVER
#include "src/web/WiFiManager.h"
#include "src/web/ArduFliteWebServer.h"
#endif

#include <Arduino.h>
#include <string_view>

void printControlLoopStats(void);
void onCalibrateHold(void);
void onModeDoubleTap(void);
void onResetTripleTap(void);
void resetSystemCommand(void);
void pauseController(void);
void resumeController(void);

// ─────────────────────────────────────────────────────────────────
// Watchdog Recovery Detection
// ─────────────────────────────────────────────────────────────────
// Returns true if the system reset due to a watchdog timeout.
// In this case, we need fast recovery: skip servo tests and go
// directly to MANUAL_MODE so the pilot has immediate control.
static bool isWatchdogRecovery() {
    esp_reset_reason_t reason = esp_reset_reason();
    return (reason == ESP_RST_TASK_WDT ||
            reason == ESP_RST_INT_WDT ||
            reason == ESP_RST_WDT);
}

// CRSF receive and telemetry SHARE one UART — the ESP32-C3 has only UART0 and
// UART1, and UART0 is the USB CDC console. Board owns the port and hands the
// same hal::Uart& to both, which makes the sharing explicit rather than a
// comment. Constructed in arduflite_init() once the Board is up.
static arduflite::drivers::CrsfLink*    g_rcLink = nullptr;
static arduflite::input::RcMapper       g_rcMapper;
static ArdufliteCRSFTelemetry*          g_crsfTx = nullptr;

// Declare telemetry instances.
TelemetryData               telemetryData;
ArduFliteFlashTelemetry     flashTelemetry(50.0f);                      // 50 Hz logging
// ArduFliteDebugSerialTelemetry    debugTelemetry(1.0f);               // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry        telemetry(20.0f);

// Declare instances of the core components.
//
// The estimation layer, assembled here rather than hidden behind a façade.
// myIMU is default-constructed at static-init time so the controller and CLI
// can take its address below; its dependencies come from the Board and are
// bound in arduflite_init(), once Board::begin() has run.
arduflite::estimation::MadgwickEstimator                   g_estimator;
static std::optional<arduflite::estimation::FirstHealthySelector> g_selector;
arduflite::estimation::InertialSubsystem                  myIMU;

/// Handed to ControlMixer::init() once Board's pool is available.
static arduflite::hal::Mutex* g_mixerConfigMutex = nullptr;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController     rateController;

// Instantiate the Controllers. Platform services are injected in
// arduflite_init() via setPlatform() — the Board's mutex pool needs FreeRTOS
// running, the same reason initFromConfig() is deferred.
ArduFliteController controller(&myIMU, &attitudeController, &rateController);

// PWM actuator bank. Storage is file-scope (ADR-011: no heap after boot); the
// pins come from the Board, the tuning and per-channel config from ConfigRegistry.
static arduflite::hal::PwmOut*                        g_pwmPins[8]{};
static arduflite::drivers::PwmChannelTuning           g_pwmTuning[8]{};
static arduflite::device::ActuatorChannelConfig       g_pwmCfg[8]{};
static arduflite::drivers::PwmActuatorBank*           g_outputs = nullptr;

// Instantiate the CLI
ArduFliteCLI myCLI(&controller, &myIMU, &flashTelemetry);

//Instantiate mission planner
MissionPlanner mission(controller);

// Instantiate the User Buttons.
//
// All three watch the SAME physical pin — one button, three gestures (hold,
// triple-tap, double-tap). Each holds a reference to the Board's GpioPin rather
// than a pin number; the object lives in BoardStorage, which is constinit and
// therefore already constructed at static-initialisation time. Each will call
// setMode() on it during begin(), which is idempotent.
HoldButton     calibrateButton(arduflite::board::Board::instance().userButton(),
                               arduflite::board::Board::instance().clock(), CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton    (arduflite::board::Board::instance().userButton(),
                               arduflite::board::Board::instance().clock(), 1000, 3, onResetTripleTap, true, 30);
MultiTapButton modeButton     (arduflite::board::Board::instance().userButton(),
                               arduflite::board::Board::instance().clock(), 1000, 2, onModeDoubleTap, true, 30);

// The status LED lives in Board now, built from the descriptor's statusLed
// entry — pin, pixel count and brightness. It was constructed here on a
// hardcoded pin 7, which meant the FireBeetle build (statusLed.pin == kNoPin)
// still drove a pixel on a pin that has none.


// ─────────────────────────────────────────────────────────────────────────────
// Actuator configuration assembly
// ─────────────────────────────────────────────────────────────────────────────
// Pins and roles come from the compile-time board descriptor; endpoints, trim,
// inversion and slew come from ConfigRegistry. This is the ONE place the two
// mechanisms meet — see specs/hal/04-board-descriptors.md §7. Keeping it here
// stops the split leaking back into a driver, which is what ServoManager did.
static uint8_t buildActuatorConfig(arduflite::board::Board& board,
                                   arduflite::hal::PwmOut** pins,
                                   arduflite::drivers::PwmChannelTuning* tuning,
                                   arduflite::device::ActuatorChannelConfig* cfgs)
{
    using arduflite::device::ActuatorKind;
    using arduflite::device::FailsafeAction;
    using arduflite::device::OutputRange;

    auto& reg = ConfigRegistry::instance();
    const auto& desc = board.descriptor();

    // Per-role config keys, in the servo.<name>.* namespace.
    struct RoleKeys { const char* role; const char* minUs; const char* maxUs;
                      const char* neutralDeg; const char* deflDeg; const char* invert; };
    static const RoleKeys kKeys[] = {
        { "elevator",      CONFIG_KEY_SERVO_PITCH_MIN_US, CONFIG_KEY_SERVO_PITCH_MAX_US,
                           CONFIG_KEY_SERVO_PITCH_NEUTRAL_DEG, CONFIG_KEY_SERVO_PITCH_DEFL_DEG,
                           CONFIG_KEY_SERVO_PITCH_INV },
        { "rudder",        CONFIG_KEY_SERVO_YAW_MIN_US,   CONFIG_KEY_SERVO_YAW_MAX_US,
                           CONFIG_KEY_SERVO_YAW_NEUTRAL_DEG,   CONFIG_KEY_SERVO_YAW_DEFL_DEG,
                           CONFIG_KEY_SERVO_YAW_INV },
        { "aileron_left",  CONFIG_KEY_SERVO_LAIL_MIN_US,  CONFIG_KEY_SERVO_LAIL_MAX_US,
                           CONFIG_KEY_SERVO_LAIL_NEUTRAL_DEG,  CONFIG_KEY_SERVO_LAIL_DEFL_DEG,
                           CONFIG_KEY_SERVO_LAIL_INV },
        { "aileron_right", CONFIG_KEY_SERVO_RAIL_MIN_US,  CONFIG_KEY_SERVO_RAIL_MAX_US,
                           CONFIG_KEY_SERVO_RAIL_NEUTRAL_DEG,  CONFIG_KEY_SERVO_RAIL_DEFL_DEG,
                           CONFIG_KEY_SERVO_RAIL_INV },
    };

    const float slewDps  = reg.get<float>(CONFIG_KEY_SERVO_MAX_SLEW_DPS);
    const float thrSlew  = reg.get<float>(CONFIG_KEY_SERVO_MAX_THR_SLEW_PER_S);

    uint8_t n = 0;
    for (uint8_t b = 0; b < desc.bankCount && n < 8; ++b)
    {
        const auto& bank = desc.actuatorBanks[b];
        if (bank.transport != arduflite::board::ActuatorTransport::Pwm) { continue; }

        for (uint8_t i = 0; i < bank.outputCount && n < 8; ++i)
        {
            const char* role = bank.outputs[i].role;
            arduflite::hal::PwmOut* pin = board.pwmOutput(n);
            if (pin == nullptr) { break; }

            pins[n]  = pin;
            cfgs[n]  = {};
            cfgs[n].role = role;
            cfgs[n].kind = bank.outputs[i].kind;

            if (std::string_view{ role } == "throttle")
            {
                tuning[n] = { static_cast<uint16_t>(reg.get<int32_t>(CONFIG_KEY_SERVO_THR_MIN_US)),
                              static_cast<uint16_t>(reg.get<int32_t>(CONFIG_KEY_SERVO_THR_MIN_US)),
                              static_cast<uint16_t>(reg.get<int32_t>(CONFIG_KEY_SERVO_THR_MAX_US)),
                              50 };
                cfgs[n].range          = OutputRange::Unipolar;
                cfgs[n].minOutput      = 0.0f;
                cfgs[n].maxOutput      = 1.0f;
                cfgs[n].maxSlew_perSec = thrSlew;
                // Throttle must go to zero on disarm, never hold.
                cfgs[n].onDisable      = FailsafeAction::Neutral;
            }
            else
            {
                const RoleKeys* k = nullptr;
                for (const auto& kk : kKeys)
                {
                    if (std::string_view{ kk.role } == role) { k = &kk; break; }
                }
                if (k == nullptr) { continue; }

                const int32_t minUs   = reg.get<int32_t>(k->minUs);
                const int32_t maxUs   = reg.get<int32_t>(k->maxUs);
                const int32_t neutral = reg.get<int32_t>(k->neutralDeg);
                const int32_t defl    = reg.get<int32_t>(k->deflDeg);

                // Config stores servo travel in DEGREES over a 0-180 sweep;
                // the actuator bank works in microseconds. Convert the neutral
                // and deflection angles into the equivalent pulse window.
                const float usPerDeg = static_cast<float>(maxUs - minUs) / 180.0f;
                tuning[n].minPulse_us     = static_cast<uint16_t>(minUs);
                tuning[n].maxPulse_us     = static_cast<uint16_t>(maxUs);
                tuning[n].neutralPulse_us =
                    static_cast<uint16_t>(static_cast<float>(minUs) + neutral * usPerDeg);
                tuning[n].frameRate_hz    = 50;

                cfgs[n].range     = OutputRange::Bipolar;
                cfgs[n].invert    = reg.get<bool>(k->invert);
                // Travel limit expressed as a fraction of the half-sweep.
                const float travel = static_cast<float>(defl) / 90.0f;
                cfgs[n].minOutput = -travel;
                cfgs[n].maxOutput =  travel;
                // Slew is configured in deg/s over the same 90 deg half-sweep.
                cfgs[n].maxSlew_perSec = slewDps / 90.0f;
                cfgs[n].onDisable      = FailsafeAction::Neutral;
            }
            ++n;
        }
    }
    return n;
}

// ─────────────────────────────────────────────────────────────────────────────
// RC link task
// ─────────────────────────────────────────────────────────────────────────────
// Drains the UART, parses frames and dispatches role callbacks. Priority::RcLink
// is 2: level with the outer loop, deliberately below the 500 Hz inner loop.
static void rcLinkTask(void* arg)
{
    (void)arg;
    auto& board = arduflite::board::Board::instance();
    arduflite::hal::WatchdogGuard wdt(board.watchdog());

    std::uint64_t lastWake = 0;
    const std::chrono::milliseconds period{ 2 };   // 500 Hz, as before

    arduflite::device::RcFrame frame{};
    bool wasFailsafe = false;

    for (;;)
    {
        wdt.feed();

        g_rcLink->poll();

        if (g_rcLink->readFrame(frame))
        {
            g_rcMapper.apply(frame);
        }

        // Failsafe edges. The link decides IF, the app decides WHAT: the
        // driver never calls a flight-layer callback itself (ADR-006).
        const bool nowFailsafe = g_rcLink->isFailsafe();
        if (nowFailsafe != wasFailsafe)
        {
            if (nowFailsafe) { CRSFCallbacks::onFailsafe(); }
            else             { CRSFCallbacks::onFailsafeExit(); }
            wasFailsafe = nowFailsafe;
        }

        board.scheduler().sleepUntil(lastWake, period);
    }
}

void arduflite_init()
{
    // ─────────────────────────────────────────────────────────────────
    // Watchdog Recovery Detection
    // ─────────────────────────────────────────────────────────────────
    // Check FIRST before any other initialization. If this is a watchdog
    // reset during flight, we need to recover as fast as possible.
    const bool watchdogRecovery = isWatchdogRecovery();

    if (auto* led = arduflite::board::Board::instance().indicator())
    {
        led->setPattern(watchdogRecovery ? Patterns::Error : Patterns::Boot);
    }

    // Serial is opened by Board::begin() now — see Esp32Console.
    while (!Serial && millis() < 2000);  // wait max 2 seconds

    // ─────────────────────────────────────────────────────────────────
    // HAL Tier 0
    // ─────────────────────────────────────────────────────────────────
    // Board::begin() brings up the buses and binds pins from the compile-time
    // BoardDescriptor. Nothing consumes it yet — the existing classes still own
    // their own hardware until later phases move them across. What it does give
    // us now is the boot inventory, which becomes the first thing in every
    // flight log and makes post-crash triage possible without knowing which
    // build flags were used.
    {
        const arduflite::Status boardStatus = arduflite::board::Board::instance().begin();
        if (boardStatus != arduflite::Status::Ok)
        {
            LOG_ERR("HAL Board::begin() failed: %s", arduflite::toString(boardStatus));
        }
        arduflite::board::Board::instance().logInventory();
    }

    if (watchdogRecovery) {
        LOG_WARN("!!! WATCHDOG RECOVERY - Fast boot to MANUAL_MODE !!!");
    }

    // ─────────────────────────────────────────────────────────────────
    // Initialize ESP32 Hardware Watchdog
    // ─────────────────────────────────────────────────────────────────
    // 1 second timeout, panic (reset) on timeout. Every registered task must
    // call hal::Watchdog::feed() within that period or the system resets.
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true
    };
    esp_err_t wdtResult = esp_task_wdt_init(&wdt_config);
    if (wdtResult == ESP_ERR_INVALID_STATE)
    {
        wdtResult = esp_task_wdt_reconfigure(&wdt_config);
    }
    if (wdtResult == ESP_OK)
    {
        if (!watchdogRecovery) {
            LOG_INF("Hardware watchdog configured (1s timeout).");
        }
    }
    else
    {
        LOG_ERR("Hardware watchdog configuration failed: %s", esp_err_to_name(wdtResult));
    }
    // ─────────────────────────────────────────────────────────────────

    // Button pin mode is set by Board::begin() from the descriptor.

    // ─────────────────────────────────────────────────────────────────
    // Initialize Persistent Configuration System
    // ─────────────────────────────────────────────────────────────────
    // ConfigRegistry: In-memory config store with validation and observers
    // ConfigPersistence: NVS storage layer
    // ConfigTask: Background task for periodic saves
    ConfigRegistry::instance().init();
    ConfigPersistence::begin();
    ConfigPersistence::load();
    ConfigTask::start();
    ConfigObservers::registerAll();
    LOG_INF("Configuration system initialized.");

    // ─────────────────────────────────────────────────────────────────
    // Initialize Web Configuration Server (if enabled at compile & runtime)
    // ─────────────────────────────────────────────────────────────────
#if ENABLE_WEB_SERVER
    if (ConfigRegistry::instance().get<bool>(CONFIG_KEY_WEB_ENABLED)) {
        if (WiFiManager::instance().begin()) {
            LOG_INF("WiFi AP started: %s", WiFiManager::instance().getSSID().c_str());
            ArduFliteWebServer::instance().begin(&controller, &myIMU, &flashTelemetry);
            LOG_INF("Web server started at http://%s",
                    WiFiManager::instance().getIP().toString().c_str());
        } else {
            LOG_ERR("WiFi AP failed to start!");
        }
    }
#else
    LOG_INF("Web server disabled at compile-time (ENABLE_WEB_SERVER=0)");
#endif

    // ─────────────────────────────────────────────────────────────────
    // Actuator bank
    // ─────────────────────────────────────────────────────────────────
    // Pins from the board descriptor, calibration from ConfigRegistry. This is
    // the one place the two config mechanisms meet — see specs/hal/04 §7.
    {
        auto& board = arduflite::board::Board::instance();
        const uint8_t n = buildActuatorConfig(board, g_pwmPins, g_pwmTuning, g_pwmCfg);

        static arduflite::drivers::PwmActuatorBank bank(
            { g_pwmPins, n }, { g_pwmTuning, n }, board.clock());

        const arduflite::Status s = bank.begin({ g_pwmCfg, n });
        if (s != arduflite::Status::Ok)
        {
            LOG_ERR("FATAL: actuator bank begin failed: %s", arduflite::toString(s));
            ESP.restart();
        }
        g_outputs = &bank;
    }

    // ─────────────────────────────────────────────────────────────────
    // Inject HAL services into the controller
    // ─────────────────────────────────────────────────────────────────
    // Three mutexes come from the Board's fixed pool. A failure here means the
    // pool is too small — a compile-time-fixable bug, not a runtime condition —
    // so it is fatal rather than degraded.
    {
        auto& board = arduflite::board::Board::instance();
        auto m1 = board.allocMutex();
        auto m2 = board.allocMutex();
        auto m3 = board.allocMutex();

        // The inner controllers and the mixer take theirs from the same pool.
        // Injected rather than self-created: these objects are constructed at
        // static-init time, before the scheduler exists, and creating a mutex
        // there tied three otherwise-portable classes to the target.
        auto m4 = board.allocMutex();
        auto m5 = board.allocMutex();
        auto m6 = board.allocMutex();
        auto m7 = board.allocMutex();   // ConfigRegistry

        if (!m1 || !m2 || !m3 || !m4 || !m5 || !m6 || !m7)
        {
            LOG_ERR("FATAL: controller mutex allocation failed — raise kMutexPoolSize.");
            ESP.restart();
        }

        attitudeController.setMutex(m4.value());
        rateController.setMutex(m5.value());
        g_mixerConfigMutex = m6.value();

        // Until now the registry ran unlocked, which was correct — the only
        // callers were static-init registrations, on one thread. From here on
        // the CLI, web server and config task all reach it.
        ConfigRegistry::instance().setMutex(m7.value());

        ArduFliteController::Platform plat{};
        plat.clock           = &board.clock();
        plat.scheduler       = &board.scheduler();
        plat.watchdog        = &board.watchdog();
        plat.system          = &board.system();
        plat.ctrlMutex       = m1.value();
        plat.outerStatsMutex = m2.value();
        plat.innerStatsMutex = m3.value();
        plat.outputs         = g_outputs;
        controller.setPlatform(plat);
        myCLI.setScheduler(board.scheduler());
    }

    // Initialize controllers from ConfigRegistry (must be after config load)
    attitudeController.initFromConfig();
    rateController.initFromConfig();
    controller.initFromConfig();

    ControlMixer::init(controller, g_mixerConfigMutex);

    flashTelemetry.begin();
    // debugTelemetry.begin();

    // ── Estimation layer ────────────────────────────────────────────────────
    {
        auto& board = arduflite::board::Board::instance();
        // The magnetometer span is empty on every board that does not declare
        // one, and the subsystem falls back to six-axis fusion on its own —
        // there is no separate enable to keep in step with the descriptor.
        g_selector.emplace(board.accelerometers(), board.gyroscopes(),
                           board.barometers(), board.magnetometers());

        myIMU.bind(arduflite::estimation::InertialSubsystem::Dependencies{
            board.sensors(), *g_selector, g_estimator, board.clock(),
            board.scheduler(), board.watchdog(), board.settings() });

        auto& config = ConfigRegistry::instance();

        arduflite::estimation::InertialSubsystem::Config imuConfig;
        // taskRate_hz keeps its default (500 Hz) — the single source of truth
        // is InertialSubsystem::Config, not a macro in a config header.
        imuConfig.accelAlpha     = config.get<float>(CONFIG_KEY_IMU_ACCEL_ALPHA);
        imuConfig.gyroAlpha      = config.get<float>(CONFIG_KEY_IMU_GYRO_ALPHA);
        imuConfig.magAlpha       = config.get<float>(CONFIG_KEY_IMU_MAG_ALPHA);
        imuConfig.fuseMagnetometer = config.get<bool>(CONFIG_KEY_IMU_FUSE_MAG);
        imuConfig.altiAlpha      = config.get<float>(CONFIG_KEY_IMU_ALTI_ALPHA);
        imuConfig.maxAccel_g     = config.get<float>(CONFIG_KEY_IMU_MAX_ACCEL_G);
        imuConfig.maxGyro_dps    = config.get<float>(CONFIG_KEY_IMU_MAX_GYRO_DPS);
        imuConfig.failThreshold  = config.get<uint8_t>(CONFIG_KEY_IMU_FAIL_THRESHOLD);

        // How the IMU is physically mounted: accelY, gyroX and gyroZ negated,
        // i.e. the map {+X, -Y, +Z}. A REFLECTION, determinant -1 — which is why
        // the gyroscope goes through applyAngularRate() and the accelerometer
        // through applyMeasurement() (§00 2.3).
        imuConfig.axes = arduflite::AxisMap{ arduflite::SignedAxis::PlusX,
                                             arduflite::SignedAxis::MinusY,
                                             arduflite::SignedAxis::PlusZ };

        myIMU.configure(imuConfig);
        g_estimator.begin(static_cast<float>(imuConfig.taskRate_hz));
        g_estimator.setBeta(config.get<float>(CONFIG_KEY_IMU_MADGWICK_BETA));
    }

    // Initialize the IMU.
    if (!myIMU.begin())
    {
        LOG_ERR("IMU failed to init!");
        if (auto* led = arduflite::board::Board::instance().indicator())
        {
            led->setPattern(Patterns::Error);
        }
        // On watchdog recovery, don't hang - try to give pilot passthrough control anyway
        if (!watchdogRecovery) {
            while (1);
        }
        LOG_WARN("IMU failed but continuing in MANUAL_MODE for pilot control.");
    }

    // ─────────────────────────────────────────────────────────────────
    // Servo Test - SKIP on watchdog recovery!
    // ─────────────────────────────────────────────────────────────────
    // During normal boot, test control surfaces.
    // After watchdog reset, skip this to avoid erratic servo movement during flight.
    if (!watchdogRecovery) {
        // Surface self-test via the bank. Same sequence as
        // ServoManager::testControlSurfaces(), now through the mixer so it
        // exercises the real output path rather than a parallel one.
        for (int cycle = 0; cycle < 2; ++cycle)
        {
            for (const float v : { 1.0f, -1.0f, 0.0f })
            {
                controller.stageTestDeflection(v);
                delay(500);
            }
        }
    } else {
        // Immediately command neutral positions to ensure clean PWM output
        // This prevents any servo glitches from residual PWM state
        (void)g_outputs->disable();   // neutral pulses, no glitch on WDT recovery
        LOG_WARN("Skipping servo test - watchdog recovery. Servos neutralized.");
    }

    // Start with a level attitude (Assist mode).
    // Two zeroes, not one shared value: an attitude of zero degrees and a rate
    // of zero deg/s are different quantities that happen to have the same
    // numeric value. Sharing one variable between them conflates the two.
    controller.setAttitudeSetpoint(AttitudeDeg{});
    controller.setRateSetpoint(AngularRateDps{});

    // ─────────────────────────────────────────────────────────────────
    // Mode Selection Based on Recovery State
    // ─────────────────────────────────────────────────────────────────
    if (watchdogRecovery) {
        // CRITICAL: After watchdog reset, go directly to MANUAL_MODE
        // This gives the pilot immediate passthrough control.
        controller.setMode(MANUAL_MODE);
        LOG_WARN("MANUAL_MODE active - pilot has direct control.");
    } else {
        // Normal boot: default is ATTITUDE_MODE.
        controller.setMode(ATTITUDE_MODE);
    }

    // Start the overall control tasks.
    controller.startTasks();

    // Start the CLI task.
    myCLI.startTask();

    // Start the Mission Planner
    mission.begin();

    // Initialize and register buttons.
    calibrateButton.begin();
    resetButton.begin();
    modeButton.begin();

    LOG_INF("Available Button Functions:");

    HoldButtonManager::registerButton(calibrateButton);
    LOG_INF("  3s hold - calibrate IMU.");

    MultiTapButtonManager::registerButton(resetButton);
    LOG_INF("  3x tap - reset telemetry layer.");

    MultiTapButtonManager::registerButton(modeButton);
    LOG_INF("  2x tap - toggle ArduFlite Mode.");

    // ─────────────────────────────────────────────────────────────────
    // RC link
    // ─────────────────────────────────────────────────────────────────
    {
        auto& board = arduflite::board::Board::instance();

        static arduflite::drivers::CrsfLink link(board.rcUart(), board.clock());
        static ArdufliteCRSFTelemetry       tx(board.rcUart(), 10.0f);
        g_rcLink = &link;
        g_crsfTx = &tx;

        const arduflite::Status s = link.begin();
        if (s != arduflite::Status::Ok)
        {
            LOG_ERR("RC link begin failed: %s", arduflite::toString(s));
        }
        link.setFailsafeTimeout(std::chrono::milliseconds{ 500 });

        // Channel -> role mapping. Kept out of the driver so the protocol
        // decoder never calls a flight-layer callback directly (ADR-006).
        using arduflite::input::ChannelMap;
        using arduflite::input::ChannelShape;
        const float triLow  = ConfigRegistry::instance().get<float>(CONFIG_KEY_CRSF_TRI_LOW);
        const float triHigh = ConfigRegistry::instance().get<float>(CONFIG_KEY_CRSF_TRI_HIGH);

        g_rcMapper.configure(0, { ChannelShape::DualThrow,   0, 0, CRSFCallbacks::onRoll });
        g_rcMapper.configure(1, { ChannelShape::DualThrow,   0, 0, CRSFCallbacks::onPitch });
        g_rcMapper.configure(2, { ChannelShape::SingleThrow, 0, 0, CRSFCallbacks::onThrottle });
        g_rcMapper.configure(3, { ChannelShape::DualThrow,   0, 0, CRSFCallbacks::onYaw });
        g_rcMapper.configure(4, { ChannelShape::Boolean,     0, 0, CRSFCallbacks::onArm });
        g_rcMapper.configure(5, { ChannelShape::Boolean,     0, 0, CRSFCallbacks::onActivateMission });
        g_rcMapper.configure(6, { ChannelShape::TriState, triLow, triHigh,
                                  CRSFCallbacks::onModeSwitch });
        g_rcMapper.configure(7, { ChannelShape::Boolean,     0, 0, CRSFCallbacks::onThrottleCut });

        tx.begin();

        const arduflite::hal::TaskConfig rcCfg{
            "CRSFRecv", 4096, arduflite::hal::Priority::RcLink, -1
        };
        auto task = board.scheduler().spawn(rcCfg, rcLinkTask, nullptr);
        if (!task)
        {
            LOG_ERR("FATAL: RC link task creation failed: %s",
                    arduflite::toString(task.status()));
            ESP.restart();
        }
    }

    if (watchdogRecovery) {
        LOG_WARN("!!! WATCHDOG RECOVERY COMPLETE - FLY TO SAFETY !!!");
    } else {
        LOG_INF("ArduFlite Controller initialised.");
    }
}

void arduflite_loop()
{
    // Process any pending commands (pass receiver for preflight checks, telemetry for calibration pause).
    CommandSystem::instance().processCommands(&controller, &myIMU, g_rcLink);

    // Update buttons.
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    handleModeState();
    handleFlightState();

    // Update telemetry with the latest sensor and control information.
    telemetryData.update(myIMU, controller, *g_rcLink);

    if (g_crsfTx) { g_crsfTx->publish(telemetryData); }
    flashTelemetry.publish(telemetryData);

    vTaskDelay(pdMS_TO_TICKS(1));
}
