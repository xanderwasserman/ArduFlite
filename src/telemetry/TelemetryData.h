/**
 * TelemetryData.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 08 April 2025
 *
 * Licensed under the MIT License. See LICENSE file for details.
 */
#ifndef TELEMETRY_DATA_H
#define TELEMETRY_DATA_H

#include "src/core/FlightTypes.h"
#include "src/estimation/InertialSubsystem.h"
#include "src/estimation/MagneticHeading.h"
#include "src/orientation/FliteQuaternion.h"
#include "src/state/StateManagement.h"
#include "src/controller/ArduFliteController.h"
#include "src/hal/device/RcLink.h"


struct TelemetryData 
{
    arduflite::Vec3f accel;    ///< g, body frame
    arduflite::Vec3f gyro;     ///< deg/s, body frame
    FliteQuaternion quat;
    // The log column names are historical and one stage off: att_cmd_* is the
    // ATTITUDE loop's output, which is a rate, and rate_cmd_* is the RATE
    // loop's output, which is a normalised axis demand. The types say so now;
    // the column names are left alone so existing logs stay parseable.
    AttitudeDeg     orientation;
    AttitudeDeg     attitudeSetpoint;
    AngularRateDps  rateSetpoint;
    AngularRateDps  attitudeCmd;
    AxisCommand     rateCmd;
    float           altitude;
    int             flight_state;
    int             flight_mode;

    // Magnetometer — instrumentation only. Nothing in the control path reads a
    // heading (ADR-055); these exist so the part can be EVALUATED on a real
    // airframe before anything is built on top of it.
    /// True when a magnetometer produced a usable reading this snapshot. False
    /// on every board that fits none, which is all of them today.
    bool                mag_valid;
    /// Body frame, microtesla, calibrated and filtered. Logged as the raw
    /// material for an offline hard-iron fit — heading and strength below are
    /// both derivable from it, but having them saves doing so by hand.
    arduflite::Vec3f    mag;
    /// Tilt-compensated, 0 = magnetic north, clockwise. Will NOT match the
    /// `yaw` column: that carries the Adafruit +180 offset, this does not.
    float               mag_heading;    // degrees
    /// The diagnostic that decides whether any of this is usable. Earth's field
    /// is 25-65 uT and its magnitude is attitude-independent, so drift here
    /// with ORIENTATION means hard iron, and drift with THROTTLE means the
    /// motor — which no hard-iron calibration can correct.
    float               mag_field;      // microtesla
    /// Whether the estimator actually fused it. Off unless `imu.fuse_mag` is
    /// set; logged so a flight cannot be misread as nine-axis when it was not.
    bool                mag_fused;

    // battery
    /**
     * @brief Whether any of the battery fields below were actually measured.
     *
     * False whenever no power monitor is fitted, which is currently always: no
     * board declares one (see §06 Phase 7). The CRSF backend suppresses the
     * battery frame entirely when this is false, rather than transmitting
     * zeroes that the radio would display as a measurement.
     */
    bool            battery_valid;
    float           battery_voltage;    // Volts
    float           battery_current;    // Amps
    uint32_t        battery_consumed;  // mAh
    uint8_t         battery_remaining;  // %

    // GPS
    double          gps_lat;            // degrees
    double          gps_lon;            // degrees
    float           gps_alt;            // meters
    float           gps_groundspeed;    // km/h
    float           gps_heading;        // degrees
    uint8_t         gps_sats;           // count

    // vario
    float           climb_rate;         // m/s

    // System status
    bool            armed;              // true if controller is armed
    bool            in_failsafe;        // true if in RC failsafe
    bool            imu_healthy;        // the estimation layer's health verdict
    float           throttle;           // commanded, 0..1; 0 when disarmed or cut

    // IMU snapshot read health
    uint32_t        imu_snapshot_retries;          // cumulative retries since boot
    uint32_t        imu_snapshot_max_retries;      // max retries for one snapshot read
    uint32_t        imu_snapshot_retry_limit_hits; // stale fallback returns since boot

    // Link statistics (populated by CRSFReceiver)
    int8_t               link_rssi1;
    int8_t               link_rssi2;
    uint8_t              link_quality;
    int8_t               link_snr;
    uint8_t              link_antenna;
    uint8_t              link_rf_mode;
    uint8_t              link_tx_power;
    int8_t               dl_rssi;
    uint8_t              dl_quality;
    int8_t               dl_snr;

    // Pull fresh values from IMU & Controller every cycle
    void update(const arduflite::estimation::InertialSubsystem &imu, const ArduFliteController &myController, const arduflite::device::RcLink &rcLink) 
    {
        // Update data from the estimation layer. One lock-free read: taking
        // several would let fields from different ticks land in one telemetry
        // frame, which is exactly what the seqlock exists to prevent.
        const arduflite::estimation::ImuState imuState = imu.state();

        accel               = imuState.accel_g;
        gyro                = imuState.gyro_dps;
        quat                = FliteQuaternion(imuState.orientation_quat.w, imuState.orientation_quat.x,
                                              imuState.orientation_quat.y, imuState.orientation_quat.z);
        orientation         = toAttitudeDeg(imuState.euler_deg);

        // Update data from ArduFlite Controller
        attitudeSetpoint    = myController.getAttitudeSetpoint();
        rateSetpoint        = myController.getRateSetpoint();
        attitudeCmd         = myController.getAttitudeCmd();
        rateCmd             = myController.getRateCmd();

        // Flight state comes from its owner now, not from the IMU snapshot.
        flight_state        = static_cast<int>(getFlightState());
        flight_mode         = static_cast<int>( myController.getMode());

        // Update additional flight data
        altitude            = imuState.altitude_m;

        // Derived here rather than in the tick: this runs at telemetry rate,
        // and the trig is not worth 500 Hz on a chip with no FPU for a number
        // nothing steers by.
        mag                 = imuState.mag_ut;
        mag_field           = arduflite::estimation::magneticFieldStrength_ut(mag);
        mag_valid           = mag_field > 0.0f;
        mag_heading         = arduflite::estimation::magneticHeading_deg(mag, orientation);
        mag_fused           = imuState.magnetometerFused;
        // No power monitor exists on any board yet. These stay zero and
        // battery_valid stays false, so nothing downstream reports them as
        // measurements. Set battery_valid when a device::PowerMonitor lands.
        battery_valid       = false;
        battery_voltage     = 0.0f;
        battery_current     = 0.0f;
        battery_consumed    = 0;
        battery_remaining   = 0;

        // GPS — no driver exists (ADR-055). `gps_sats == 0` is the contract for
        // "no fix", and every consumer gates on it: the CRSF backend still
        // sends the frame so the radio can discover the sensor, but zeroes each
        // field rather than transmitting a position of 0,0. Populate these only
        // together with a real sats count, or a radio will show the aircraft
        // off the coast of Africa with apparent confidence.
        gps_lat             = 0.0f;
        gps_lon             = 0.0f;
        gps_alt             = 0.0f;
        gps_groundspeed     = 0.0f;
        gps_heading         = 0.0f;
        gps_sats            = 0;

        // vario
        climb_rate          = imuState.climbRate_mps;

        // System status
        armed               = myController.isArmed();
        in_failsafe         = rcLink.isFailsafe();
        imu_healthy         = imuState.healthy;
        throttle            = (armed && !myController.isThrottleCut())
                                  ? myController.getThrottleSetpoint() : 0.0f;

        const auto snapshotHealth = imu.snapshotHealth();
        imu_snapshot_retries          = snapshotHealth.totalRetries;
        imu_snapshot_max_retries      = snapshotHealth.maxRetries;
        imu_snapshot_retry_limit_hits = snapshotHealth.retryLimitHits;

        // Update link statistics
        const arduflite::device::RcLinkStats stats = rcLink.stats();

        link_rssi1          = stats.uplinkRssi1_dbm;
        link_rssi2          = stats.uplinkRssi2_dbm;
        link_quality        = stats.uplinkQuality_pct;
        link_snr            = stats.uplinkSnr_db;
        link_antenna        = stats.activeAntenna;
        link_rf_mode        = stats.rfMode;
        link_tx_power       = stats.txPower;
        dl_rssi             = stats.downlinkRssi_dbm;
        dl_quality          = stats.downlinkQuality_pct;
        dl_snr              = stats.downlinkSnr_db;
    }
};

#endif // TELEMETRY_DATA_H
