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
#include "include/ReceiverConfiguration.h"
#include "include/PinConfiguration.h"
#include "include/CSRFConfiguration.h"

#include <esp_task_wdt.h>  // ESP32 hardware watchdog
#include <esp_system.h>    // For esp_reset_reason()

#include "src/actuators/ServoManager.h"
#include "src/orientation/ArduFliteIMU.h"
#include "src/cli/ArduFliteCLI.h"
#include "src/mission_planner/MissionPlanner.h"
#include "src/state/StateManagement.h"

#include "src/utils/ControlMixer.h"
#include "src/utils/HoldButton.h"
#include "src/utils/HoldButtonManager.h"
#include "src/utils/MultiTapButton.h"
#include "src/utils/MultiTapButtonManager.h"
#include "src/utils/CommandSystem.h"
#include "src/utils/StatusLED.h"
#include "src/utils/ButtonCallbacks.h"
#include "src/utils/Logging.h"
#include "src/utils/ConfigRegistry.h"
#include "src/utils/ConfigPersistence.h"
#include "src/utils/ConfigTask.h"
#include "src/utils/ConfigObservers.h"
#include "include/ConfigKeys.h"

#include "src/controller/ArduFliteAttitudeController.h"
#include "src/controller/ArduFliteRateController.h"
#include "src/controller/ArduFliteController.h"

#include "src/telemetry/ArduFliteTelemetry.h"
#include "src/telemetry/serial/ArduFliteQSerialTelemetry.h"
#include "src/telemetry/serial/ArduFliteDebugSerialTelemetry.h"
#include "src/telemetry/flash/ArduFliteFlashTelemetry.h"
#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"

#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"

#include "include/WebConfiguration.h"
#if ENABLE_WEB_SERVER
#include "src/web/WiFiManager.h"
#include "src/web/ArduFliteWebServer.h"
#endif

#include "src/tests/AttitudeTests.h"

#include <Arduino.h>

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

// Single UART for CRSF (ESP32-C3 only has UART0 and UART1; UART0 is used by USB CDC)
// We must share UART1 between RX and TX with separate pins
HardwareSerial crsfSerial(1);

ArdufliteCRSFReceiver  crsfRx(crsfSerial, CRSFPinConfig::PIN_CRSF_RX, CRSFPinConfig::PIN_CRSF_TX);
ArdufliteCRSFTelemetry crsfTx(crsfSerial, 10.0f); // 10 Hz telemetry (uses already-initialized serial)

// Declare telemetry instances.
TelemetryData               telemetryData;
ArduFliteFlashTelemetry     flashTelemetry(50.0f);                      // 50 Hz logging
// ArduFliteDebugSerialTelemetry    debugTelemetry(1.0f);               // 1 Hz telemetry frequency
// ArduFliteQSerialTelemetry        telemetry(20.0f);

// Declare instances of the core components.
ArduFliteIMU                myIMU;
ArduFliteAttitudeController attitudeController;
ArduFliteRateController     rateController;

// Instantiate the ServoManager (deferred init - servos attached after ConfigRegistry loads)
ServoManager servoMgr;

// Instantiate the Controllers
ArduFliteController controller(&myIMU, &attitudeController, &rateController, &servoMgr);

// Instantiate the CLI
ArduFliteCLI myCLI(&controller, &myIMU, &flashTelemetry);

//Instantiate mission planner
MissionPlanner mission(controller);

// Instantiate the User Buttons
HoldButton calibrateButton(ButtonInputConfig::USER_BUTTON_PIN, CALIB_HOLD_TIME, onCalibrateHold, true, false, 50);
MultiTapButton resetButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 3, onResetTripleTap, true, 30);
MultiTapButton modeButton(ButtonInputConfig::USER_BUTTON_PIN, 1000, 2, onModeDoubleTap, true, 30);

#if BOARD_TYPE == BOARD_TYPE_WEMOS
StatusLED statusLED(7, 1);
#endif

void arduflite_init() 
{
    // ─────────────────────────────────────────────────────────────────
    // Watchdog Recovery Detection
    // ─────────────────────────────────────────────────────────────────
    // Check FIRST before any other initialization. If this is a watchdog
    // reset during flight, we need to recover as fast as possible.
    const bool watchdogRecovery = isWatchdogRecovery();

#if BOARD_TYPE == BOARD_TYPE_WEMOS
    statusLED.begin();
    statusLED.setPattern(watchdogRecovery ? Patterns::Error : Patterns::Boot);
#endif

    Serial.begin(115200);
    while (!Serial && millis() < 2000);  // wait max 2 seconds

    if (watchdogRecovery) {
        LOG_WARN("!!! WATCHDOG RECOVERY - Fast boot to MANUAL_MODE !!!");
    }

    // ─────────────────────────────────────────────────────────────────
    // Initialize ESP32 Hardware Watchdog
    // ─────────────────────────────────────────────────────────────────
    // 1 second timeout, panic (reset) on timeout
    // Critical tasks (control loops, CRSF receiver) must call esp_task_wdt_reset()
    // within this period or the system will reset.
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    if (!watchdogRecovery) {
        LOG_INF("Hardware watchdog initialized (1s timeout).");
    }
    // ─────────────────────────────────────────────────────────────────

    pinMode(ButtonInputConfig::USER_BUTTON_PIN, INPUT_PULLUP);

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

    // Initialize controllers and servos from ConfigRegistry (must be after config load)
    attitudeController.initFromConfig();
    rateController.initFromConfig();
    servoMgr.initFromConfig();
    myIMU.initFromConfig();

    ControlMixer::init(controller);

    flashTelemetry.begin();
    // debugTelemetry.begin();

    // Initialize the IMU.
    if (!myIMU.begin()) 
    {
        LOG_ERR("IMU failed to init!");
#if BOARD_TYPE == BOARD_TYPE_WEMOS
        statusLED.setPattern(Patterns::Error); //fast yellow blink
#endif
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
        servoMgr.testControlSurfaces();
    } else {
        // Immediately command neutral positions to ensure clean PWM output
        // This prevents any servo glitches from residual PWM state
        servoMgr.writeCommands(0.0f, 0.0f, 0.0f);
        servoMgr.writeThrottle(0.0f);
        LOG_WARN("Skipping servo test - watchdog recovery. Servos neutralized.");
    }

    // Start with a level attitude (Assist mode).
    EulerAngles initialSetpoint {0.0f};
    controller.setAttitudeSetpoint(initialSetpoint);
    controller.setRateSetpoint(initialSetpoint);

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

    // Initialize shared CRSF UART with both RX and TX pins
    crsfRx.begin();
    crsfTx.begin();  // Telemetry uses already-initialized serial

    // configure every channel in one loop
    for (uint8_t ch = 0; ch < 16; ++ch) {
        crsfRx.configureChannel(ch, CRSFConfig::CRSF_CHANNEL_CONFIGS[ch]);
    }

    crsfRx.setFailsafeCallback(CRSFCallbacks::onFailsafe);
    crsfRx.setFailsafeExitCallback(CRSFCallbacks::onFailsafeExit);
    crsfRx.setFailsafeTimeout(500);

    if (watchdogRecovery) {
        LOG_WARN("!!! WATCHDOG RECOVERY COMPLETE - FLY TO SAFETY !!!");
    } else {
        LOG_INF("ArduFlite Controller initialised.");
    }
}

void arduflite_loop() 
{
    // Process any pending commands (pass receiver for preflight checks, telemetry for calibration pause).
    CommandSystem::instance().processCommands(&controller, &myIMU, &crsfRx, &crsfTx);
    
    // Update buttons.
    HoldButtonManager::updateAll();
    MultiTapButtonManager::updateAll();

    handleModeState();
    handleFlightState();
        
    // Update telemetry with the latest sensor and control information.
    telemetryData.update(myIMU, controller, crsfRx);

    crsfTx.publish(telemetryData);
    flashTelemetry.publish(telemetryData);

    vTaskDelay(pdMS_TO_TICKS(1));
}