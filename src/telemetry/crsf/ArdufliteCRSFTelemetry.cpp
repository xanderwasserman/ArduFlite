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
#include "src/hal/protocol/CrsfProtocol.h"
#include "src/controller/ArduFliteController.h"
#include "src/utils/Logging.h"

#include <mutex>

#include "src/hal/board/Board.h"
#include <math.h> 
#include <cstring> 

/// @brief Constructor.
/// @param[in] ser     Reference to a HardwareSerial port for CRSF (shared with receiver).
/// @param[in] freqHz  Frame-send frequency, in Hz (default 10Hz).
/// @note  UART is configured by receiver's begin() - this class uses it for TX.
ArdufliteCRSFTelemetry::ArdufliteCRSFTelemetry(arduflite::hal::Uart& ser,
                                               float          freqHz)
  : PeriodicTelemetryBackend("CrsfTelem", freqHz)
  , _serial(ser)
{
    // No platform resource here: this may be constructed before Board::begin()
    // has run. The mutex is taken in begin() (ADR-030).
}

/// @brief Destructor: stops the telemetry task and deletes the mutex.
ArdufliteCRSFTelemetry::~ArdufliteCRSFTelemetry()
{
    requestTaskStop();
    LOG_INF("CRSF Telemetry: task stop requested");
}

/// @brief Pause the telemetry task (unsubscribes from WDT).
void ArdufliteCRSFTelemetry::pauseTask()
{
    // Only the flag is set here; the TASK changes its own watchdog
    // subscription when it notices. Unsubscribing another task from here would
    // race with that task feeding a subscription that just vanished.
    _paused.store(true, std::memory_order_release);

    LOG_INF("CRSF Telemetry: pause requested");
}

/// @brief Resume the telemetry task (re-subscribes to WDT).
void ArdufliteCRSFTelemetry::resumeTask()
{
    _paused.store(false, std::memory_order_release);

    LOG_INF("CRSF Telemetry: resume requested");
}

/// @brief Main loop: pulls the latest buffer, sends all frames, then delays.
/// @details
///   Frame rate tiering for optimal bandwidth usage:
///   - Fast (every loop ~10Hz): Attitude, Link Stats, Vario
///   - Medium (~5Hz): Battery, Baro Altitude
///   - Slow (~1Hz): GPS, Flight Mode
void ArdufliteCRSFTelemetry::runLoop()
{
    // Rate tiering interval (milliseconds)
    constexpr uint32_t MEDIUM_INTERVAL_MS = 200;   // ~5 Hz

    LOG_DBG("CRSF Telemetry: run() loop begin");

    auto&       board      = arduflite::board::Board::instance();
    auto&       scheduler  = board.scheduler();
    auto&       watchdog   = board.watchdog();
    const auto& boardClock = board.clock();

    // Registration happens here, from INSIDE the task, because that is the only
    // place "current task" means this one.
    (void)watchdog.registerCurrentTask();
    bool watched = true;

    const auto nowMs = [&]() -> std::uint32_t {
        return static_cast<std::uint32_t>(
            boardClock.now().time_since_epoch().count() / 1000);
    };

    // Rate tiering: track last send time
    std::uint32_t lastMediumTime = nowMs();
    TelemetryData td{};

    while (shouldRun())
    {
        // Paused during calibration, which can outlast the watchdog window, so
        // the task unsubscribes itself while idle.
        if (_paused.load(std::memory_order_acquire))
        {
            if (watched)
            {
                (void)watchdog.unregisterCurrentTask();
                watched = false;
            }
            scheduler.sleepFor(std::chrono::milliseconds{ 10 });
            continue;
        }

        if (!watched)
        {
            (void)watchdog.registerCurrentTask();
            watched = true;
        }

        // Proves this task is alive.
        watchdog.feed();

        const std::uint32_t now = nowMs();

        // 1) grab a snapshot
        // Reuse the previous frame on a timeout: the radio wants a continuous
        // stream, and a repeated attitude is better than a gap it reads as a
        // lost sensor.
        (void)snapshot(td);

        // 2) FAST frames: send every loop (~10 Hz)
        //    These are critical for real-time display
        sendAttitude (td);
        sendLinkStats(td);
        sendVario    (td);

        // 3) MEDIUM frames: send at ~5 Hz
        if (now - lastMediumTime >= MEDIUM_INTERVAL_MS) 
        {
            // Only when something actually measured it. Sending 0.0 V and
            // 100 % remaining — which is what the placeholders produced — puts
            // fabricated numbers on the pilot's battery display, in a field
            // pilots use to decide whether to land. A radio showing no battery
            // telemetry is unambiguous; one showing a confident wrong value is
            // not. Restore this call when device::PowerMonitor exists.
            if (td.battery_valid) { sendBattery(td); }
            sendBaroAlt   (td);
            sendGps       (td);  // Moved from 1Hz - prevents sensor lost warnings
            sendFlightMode(td);
            lastMediumTime = now;
        }

        // 5) wait until next burst
        scheduler.sleepFor(
            std::chrono::milliseconds{ static_cast<std::int64_t>(intervalMs()) });
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

    // Send frame: [sync=0xEA] [length] [type] [payload...] [crc]
    // hal::Uart writes byte buffers, not single bytes: assemble the frame then
    // emit it in one call. That is also better on a shared UART — a partial
    // frame interleaved with the receiver's traffic would confuse the link.
    std::uint8_t frame[64];
    std::size_t  n = 0;
    frame[n++] = AddrRX;   // 0xEA = receiver address for FC->RX telemetry
    frame[n++] = flen;
    frame[n++] = type;
    for (std::size_t i = 0; i < len && n < sizeof(frame) - 1; ++i) { frame[n++] = payload[i]; }
    frame[n++] = crc;
    (void)_serial.write(frame, n);
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
    
    // Bounded on BOTH sides. flight_mode is a plain int, so a negative value
    // would pass a "< LENGTH" test and then index the array at uint8_t(-1).
    // Nothing produces one today; the cost of ruling it out is one comparison,
    // and the cost of being wrong is a read far outside a 4-entry table.
    const int modeIndex = (t.flight_mode >= 0 && t.flight_mode < FLIGHT_MODE_LENGTH)
                              ? t.flight_mode
                              : static_cast<int>(UNKNOWN_MODE);
    const char* modeName = modeNames[modeIndex];
    
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
/// @note  Always sends frame for sensor discovery; shows zeros when no fix.
void ArdufliteCRSFTelemetry::sendGps(const TelemetryData& t)
{
    uint8_t buf[15];
    
    // When no GPS fix, send safe defaults (Sats=0 tells pilot there's no fix)
    // Always send so EdgeTX can discover sensors during desk setup
    int32_t lat = (t.gps_sats > 0) ? int32_t(t.gps_lat * 1e7) : 0;
    int32_t lon = (t.gps_sats > 0) ? int32_t(t.gps_lon * 1e7) : 0;
    uint16_t gs = (t.gps_sats > 0) ? uint16_t(t.gps_groundspeed * 10.0f) : 0;  // km/h * 10
    uint16_t hd = (t.gps_sats > 0) ? uint16_t(t.gps_heading * 100.0f) : 0;      // degrees * 100
    uint16_t al = (t.gps_sats > 0) ? uint16_t(t.gps_alt + 1000.0f) : 1000;      // +1000 offset, 0m when no fix

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
    arduflite::drivers::CrsfLinkStatistics s = {
        .uplinkRssi1         = uint8_t(t.link_rssi1),
        .uplinkRssi2         = uint8_t(t.link_rssi2),
        .uplinkLinkQuality   = t.link_quality,
        .uplinkSnr           = t.link_snr,
        .activeAntenna       = t.link_antenna,
        .rfMode              = t.link_rf_mode,
        .uplinkTxPower       = t.link_tx_power,
        .downlinkRssi        = uint8_t(t.dl_rssi),
        .downlinkLinkQuality = t.dl_quality,
        .downlinkSnr         = t.dl_snr
    };

    LOG_DBG("CRSF Telemetry: Link RSSI1=%d RSSI2=%d Q=%u SNR=%d ANT=%u", s.uplinkRssi1, s.uplinkRssi2, s.uplinkLinkQuality, s.uplinkSnr, s.activeAntenna);
    sendFrame(T_LINK, reinterpret_cast<const uint8_t*>(&s), sizeof(s));
}
