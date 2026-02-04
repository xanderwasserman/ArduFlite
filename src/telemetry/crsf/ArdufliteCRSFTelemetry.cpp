/**
 * @file      ArdufliteCRSFTelemetry.cpp
 * @brief     Implementation of CRSF telemetry over a dedicated FreeRTOS task.
 * @author    Alexander Wasserman
 * @version   1.1
 * @date      07 June 2025
 *
 * @copyright MIT License
 *
 * This module serializes telemetry data into CRSF frames and sends them
 * at a fixed rate over a HardwareSerial port (e.g. to an ELRS or Crossfire
 * receiver). Uses a FreeRTOS task and mutex to decouple from the FC's
 * main control loop.
 */

#include "src/telemetry/crsf/ArdufliteCRSFTelemetry.h"
#include "src/controller/ArduFliteController.h"
#include "src/utils/Logging.h"
#include "include/ArduFlite.h"

#include <esp_task_wdt.h>  // Hardware watchdog
#include <math.h> 
#include <cstring> 

/// @brief Constructor.
/// @param[in] ser     Reference to a HardwareSerial port for CRSF downlink.
/// @param[in] txPin   GPIO pin to use as UART TX line.
/// @param[in] freqHz  Frame-send frequency, in Hz (default 25Hz).
ArdufliteCRSFTelemetry::ArdufliteCRSFTelemetry(HardwareSerial& ser,
                                               int            txPin,
                                               float          freqHz)
  : _serial(ser)
  , _txPin(txPin)
  , _intervalMs(1000.0f / freqHz)
{
    _lock = xSemaphoreCreateMutex();
    if (!_lock) 
    {
        LOG_ERR("CRSF Telemetry: failed to create mutex");
    } 
    else 
    {
        LOG_DBG("CRSF Telemetry: mutex created");
    }
}

/// @brief Destructor: stops the telemetry task and deletes the mutex.
ArdufliteCRSFTelemetry::~ArdufliteCRSFTelemetry()
{
    if (_taskHandle) 
    {
        vTaskDelete(_taskHandle);
        LOG_INF("CRSF Telemetry: task deleted");
    }
    if (_lock) 
    {
        vSemaphoreDelete(_lock);
        LOG_INF("CRSF Telemetry: mutex deleted");
    }
}

/// @brief Initializes the UART and starts the telemetry FreeRTOS task.
/// @note  Baud = 420000, 8N1. TX only.
/// @see   xTaskCreate()
void ArdufliteCRSFTelemetry::begin()
{
    LOG_INF("CRSF Telemetry: starting on TX pin %d @ 420000 baud", _txPin);
    _serial.begin(420000, SERIAL_8N1, /*rxPin=*/-1, _txPin);

    if (_taskHandle) 
    {
        LOG_WARN("CRSF Telemetry: begin() called but task already running");
        return;  ///< already running
    }

    BaseType_t res = xTaskCreate(
        telemetryTask,
        "CrsfTelem",
        /*stack depth*/ 4096,
        this,
        tskIDLE_PRIORITY + 1,
        &_taskHandle
    );
    if (res != pdPASS) 
    {
        LOG_ERR("CRSF Telemetry: failed to start task (pdPASS=%d)", pdPASS);
    } 
    else 
    {
        LOG_INF("CRSF Telemetry: task started (interval %.1f ms)", _intervalMs);
    }

    LOG_INF("Started CRSF Telemetry task.");
}

/// @brief Publishes new telemetry & config snapshots.
/// @param[in] telem  Fresh sensor & state data.
/// @param[in] cfg    Configuration data (not used in this version).
/// @note  Uses a mutex to safely swap in the pending buffers.
void ArdufliteCRSFTelemetry::publish(const TelemetryData& telem,
                                     const ConfigData&    cfg)
{
    if (!_lock) return;

    {
        SemaphoreLock lock(_lock);
        _pendingData   = telem;
        _pendingConfig = cfg;
    }
}

/// @brief FreeRTOS task entrypoint.
/// @param[in] pv  Pointer to the ArdufliteCRSFTelemetry instance.
void ArdufliteCRSFTelemetry::telemetryTask(void* pv)
{
    auto* self = static_cast<ArdufliteCRSFTelemetry*>(pv);
    LOG_INF("CRSF Telemetry: task entry");
    self->run();
}

/// @brief Main loop: pulls the latest buffer, sends all frames, then delays.
/// @details
///   Frame rate tiering for optimal bandwidth usage:
///   - Fast (every loop ~10Hz): Attitude, Link Stats, Vario
///   - Medium (~5Hz): Battery, Baro Altitude
///   - Slow (~1Hz): GPS, Flight Mode
void ArdufliteCRSFTelemetry::run()
{
    // Rate tiering intervals (milliseconds)
    constexpr uint32_t MEDIUM_INTERVAL_MS = 200;   // ~5 Hz
    constexpr uint32_t SLOW_INTERVAL_MS   = 1000;  // ~1 Hz

    LOG_DBG("CRSF Telemetry: run() loop begin");

    // Register this task with hardware watchdog
    esp_task_wdt_add(NULL);  // NULL = current task

    // Rate tiering: track last send times
    uint32_t lastMediumTime = millis();
    uint32_t lastSlowTime   = millis();

    while (true) 
    {
        // Reset hardware watchdog - proves this task is alive
        esp_task_wdt_reset();

        uint32_t now = millis();

        // 1) grab a snapshot
        TelemetryData td;
        ConfigData    cd;
        if (_lock) 
        {
            SemaphoreLock lock(_lock);
            td = _pendingData;
            cd = _pendingConfig;
        }

        // 2) FAST frames: send every loop (~10 Hz)
        //    These are critical for real-time display
        sendAttitude (td);
        sendLinkStats(td);
        sendVario    (td);

        // 3) MEDIUM frames: send at ~5 Hz
        if (now - lastMediumTime >= MEDIUM_INTERVAL_MS) 
        {
            sendBattery(td);
            sendBaroAlt(td);
            lastMediumTime = now;
        }

        // 4) SLOW frames: send at ~1 Hz
        //    GPS and flight mode don't change rapidly
        if (now - lastSlowTime >= SLOW_INTERVAL_MS) 
        {
            LOG_DBG("CRSF Telemetry: sending 1 Hz frames");
            sendGps       (td);
            sendFlightMode(td);
            lastSlowTime = now;
        }

        // 5) wait until next burst
        vTaskDelay(pdMS_TO_TICKS(_intervalMs));
    }
}


/// @brief Computes the standard CRC-8 for CRSF (poly 0xD5).
/// @param[in] data  Byte array to checksum (type byte + payload).
/// @param[in] len   Number of bytes in @p data.
/// @returns 8-bit CRC.
uint8_t ArdufliteCRSFTelemetry::crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; ++i) {
            crc = (crc & 0x80)
                  ? (uint8_t)((crc << 1) ^ 0xD5u)
                  : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/// @brief Sends a generic CRSF frame.
/// @param[in] type     CRSF frame type byte (e.g. T_ATT).
/// @param[in] payload  Pointer to the payload bytes.
/// @param[in] len      Number of payload bytes.
/// @note  Prepends sync byte (0xC8) and length, appends CRC8(type|payload).
void ArdufliteCRSFTelemetry::sendFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    // Length = type (1) + payload (len) + CRC (1)
    uint8_t flen = len + 2;
    LOG_DBG("CRSF Telemetry: frame 0x%02X, payload %u bytes, flen %u", type, len, flen);

    // Build CRC buffer: [type, payload...]
    // CRC is computed over type + payload, NOT the sync byte or length
    uint8_t crcBuf[64];  // Max CRSF payload is ~62 bytes
    crcBuf[0] = type;
    if (len > 0 && len < sizeof(crcBuf) - 1) {
        memcpy(crcBuf + 1, payload, len);
    }
    uint8_t crc = crc8(crcBuf, len + 1);

    // Send frame: [sync=0xC8] [length] [type] [payload...] [crc]
    _serial.write(AddrFC);  // 0xC8 sync byte for telemetry from FC
    _serial.write(flen);
    _serial.write(type);
    _serial.write(payload, len);
    _serial.write(crc);
}

/// @brief Sends the 6-byte attitude frame (pitch, roll, yaw).
/// @param[in] t  Latest telemetry (angles converted to radians).
void ArdufliteCRSFTelemetry::sendAttitude(const TelemetryData& t)
{
    // Convert from degrees → radians, then scale
    constexpr float DEG2RAD = M_PI / 180.0f;
    constexpr float SCALE     = 10000.0f;
    
    int16_t p = int16_t((t.orientation.pitch * DEG2RAD) * SCALE);
    int16_t r = int16_t((t.orientation.roll  * DEG2RAD) * SCALE);
    int16_t y = int16_t((t.orientation.yaw   * DEG2RAD) * SCALE);

    // CRSF uses big-endian byte order (MSB first)
    uint8_t buf[6];
    buf[0] = uint8_t((p >> 8) & 0xFF);  // pitch MSB
    buf[1] = uint8_t(p & 0xFF);          // pitch LSB
    buf[2] = uint8_t((r >> 8) & 0xFF);  // roll MSB
    buf[3] = uint8_t(r & 0xFF);          // roll LSB
    buf[4] = uint8_t((y >> 8) & 0xFF);  // yaw MSB
    buf[5] = uint8_t(y & 0xFF);          // yaw LSB

    LOG_DBG("CRSF Telemetry: Attitude P=%d R=%d Y=%d", p, r, y);
    sendFrame(T_ATT, buf, sizeof(buf));
}


/// @brief Sends the null-terminated flight mode string.
/// @param[in] t  Latest telemetry (mode index).
void ArdufliteCRSFTelemetry::sendFlightMode(const TelemetryData& t)
{
    // EdgeTX-compatible flight mode strings:
    // - '*' prefix = disarmed
    // - '!' prefix = failsafe/warning
    // - No prefix  = armed and normal
    // Mode names recognized by EdgeTX for icons:
    //   STAB/ANGL = Stabilized/Angle mode
    //   ACRO      = Acrobatic/Rate mode
    //   MANU      = Manual passthrough
    static const char* modeNames[] = {
        "STAB",   // ATTITUDE_MODE (0) - Stabilized/Angle
        "ACRO",   // RATE_MODE (1)     - Acrobatic
        "MANU",   // MANUAL_MODE (2)   - Manual passthrough
        "????"    // UNKNOWN_MODE (3)
    };
    
    uint8_t idx = (t.flight_mode < FLIGHT_MODE_LENGTH) ? uint8_t(t.flight_mode) : UNKNOWN_MODE;
    const char* modeName = modeNames[idx];
    
    // Build mode string with prefix based on state
    char modeStr[16];
    if (t.in_failsafe) {
        // Failsafe active - show warning prefix
        snprintf(modeStr, sizeof(modeStr), "!FS!");
    } else if (!t.armed) {
        // Disarmed - show asterisk prefix
        snprintf(modeStr, sizeof(modeStr), "*%s", modeName);
    } else {
        // Armed and normal - no prefix
        snprintf(modeStr, sizeof(modeStr), "%s", modeName);
    }

    LOG_DBG("CRSF Telemetry: FlightMode \"%s\"", modeStr);
    sendFrame(T_MODE, reinterpret_cast<const uint8_t*>(modeStr), uint8_t(strlen(modeStr) + 1));
}

/// @brief Sends battery status (voltage/current/capacity/%).
/// @param[in] t  Latest telemetry (battery fields populated).
void ArdufliteCRSFTelemetry::sendBattery(const TelemetryData& t)
{
    uint8_t buf[8];
    uint16_t v   = uint16_t(t.battery_voltage * 10.0f);  // CRSF: decivolts (0.1V units)
    uint16_t i   = uint16_t(t.battery_current * 10.0f);  // CRSF: deciamps (0.1A units)
    uint32_t cap = t.battery_consumed;

    // CRSF uses big-endian byte order (MSB first)
    buf[0] = uint8_t((v >> 8) & 0xFF);  // voltage MSB
    buf[1] = uint8_t(v & 0xFF);          // voltage LSB
    buf[2] = uint8_t((i >> 8) & 0xFF);  // current MSB
    buf[3] = uint8_t(i & 0xFF);          // current LSB
    // capacity is 24-bit big-endian:
    buf[4] = uint8_t((cap >> 16) & 0xFF);
    buf[5] = uint8_t((cap >>  8) & 0xFF);
    buf[6] = uint8_t( cap        & 0xFF);
    buf[7] = t.battery_remaining;

    LOG_DBG("CRSF Telemetry: Battery V=%u mV I=%u mA C=%u%%", v, i, t.battery_remaining);
    sendFrame(T_BATTERY, buf, sizeof(buf));
}

/// @brief Sends GPS fix, speed, heading, altitude, sats.
/// @param[in] t  Latest telemetry (GPS fields populated).
void ArdufliteCRSFTelemetry::sendGps(const TelemetryData& t)
{
    uint8_t buf[15];
    int32_t lat = int32_t(t.gps_lat * 1e7);
    int32_t lon = int32_t(t.gps_lon * 1e7);
    uint16_t gs = uint16_t(t.gps_groundspeed * 10.0f);   // CRSF: km/h * 10
    uint16_t hd = uint16_t(t.gps_heading * 100.0f);       // CRSF: degrees * 100
    uint16_t al = uint16_t(t.gps_alt + 1000.0f);          // CRSF: meters + 1000 offset

    // CRSF uses big-endian byte order (MSB first)
    // Latitude: 32-bit signed, big-endian
    buf[0] = uint8_t((lat >> 24) & 0xFF);
    buf[1] = uint8_t((lat >> 16) & 0xFF);
    buf[2] = uint8_t((lat >>  8) & 0xFF);
    buf[3] = uint8_t( lat        & 0xFF);
    // Longitude: 32-bit signed, big-endian
    buf[4] = uint8_t((lon >> 24) & 0xFF);
    buf[5] = uint8_t((lon >> 16) & 0xFF);
    buf[6] = uint8_t((lon >>  8) & 0xFF);
    buf[7] = uint8_t( lon        & 0xFF);
    // Ground speed: 16-bit, big-endian
    buf[8] = uint8_t((gs >> 8) & 0xFF);
    buf[9] = uint8_t(gs & 0xFF);
    // Heading: 16-bit, big-endian
    buf[10] = uint8_t((hd >> 8) & 0xFF);
    buf[11] = uint8_t(hd & 0xFF);
    // Altitude: 16-bit, big-endian
    buf[12] = uint8_t((al >> 8) & 0xFF);
    buf[13] = uint8_t(al & 0xFF);
    // Satellites
    buf[14] = t.gps_sats;

    LOG_DBG("CRSF Telemetry: GPS lat=%ld lon=%ld gs=%u hd=%u al=%u sats=%u", lat, lon, gs, hd, al, t.gps_sats);
    sendFrame(T_GPS, buf, sizeof(buf));
}

/// @brief Sends vertical speed (vario) in cm/s.
/// @param[in] t  Latest telemetry (climb_rate in m/s).
void ArdufliteCRSFTelemetry::sendVario(const TelemetryData& t)
{
    int16_t vs = int16_t(t.climb_rate * 10.0f);  // CRSF: dm/s (0.1 m/s units)
    // CRSF uses big-endian byte order (MSB first)
    uint8_t buf[2] = { uint8_t((vs >> 8) & 0xFF), uint8_t(vs & 0xFF) };

    LOG_DBG("CRSF Telemetry: Vario %d dm/s", vs);
    sendFrame(T_VARIO, buf, sizeof(buf));
}

/// @brief Sends barometric altitude.
/// @param[in] t  Latest telemetry (altitude in meters).
void ArdufliteCRSFTelemetry::sendBaroAlt(const TelemetryData& t)
{
    // CRSF barometric altitude: 16-bit signed, decimeters (0.1m units)
    // Range: -3276.8m to +3276.7m - clamp to prevent overflow
    constexpr float MAX_ALT_M = 3276.7f;
    constexpr float MIN_ALT_M = -3276.8f;
    float clampedAlt = constrain(t.altitude, MIN_ALT_M, MAX_ALT_M);
    int16_t alt_dm = int16_t(clampedAlt * 10.0f);
    
    // CRSF uses big-endian byte order (MSB first)
    uint8_t buf[2] = { uint8_t((alt_dm >> 8) & 0xFF), uint8_t(alt_dm & 0xFF) };

    LOG_DBG("CRSF Telemetry: BaroAlt %d dm (%.1f m)", alt_dm, t.altitude);
    sendFrame(T_BARO_ALT, buf, sizeof(buf));
}

/// @brief Sends full link statistics (10-byte struct).
/// @param[in] t  Latest telemetry (link fields populated).
void ArdufliteCRSFTelemetry::sendLinkStats(const TelemetryData& t)
{
    crsfLinkStatistics_t s = {
        .uplink_RSSI_1         = uint8_t(t.link_rssi1),
        .uplink_RSSI_2         = uint8_t(t.link_rssi2),
        .uplink_Link_quality   = t.link_quality,
        .uplink_SNR            = t.link_snr,
        .active_antenna        = t.link_antenna,
        .rf_Mode               = t.link_rf_mode,
        .uplink_TX_Power       = t.link_tx_power,
        .downlink_RSSI         = uint8_t(t.dl_rssi),
        .downlink_Link_quality = t.dl_quality,
        .downlink_SNR          = t.dl_snr
    };

    LOG_DBG("CRSF Telemetry: Link RSSI1=%d RSSI2=%d Q=%u SNR=%d ANT=%u", s.uplink_RSSI_1, s.uplink_RSSI_2, s.uplink_Link_quality, s.uplink_SNR, s.active_antenna);
    sendFrame(T_LINK, reinterpret_cast<const uint8_t*>(&s), sizeof(s));
}
