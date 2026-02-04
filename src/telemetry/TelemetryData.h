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

#include "src/orientation/ArduFliteIMU.h"
#include "src/controller/ArduFliteController.h"
#include "src/receiver/crsf/ArdufliteCRSFReceiver.h"

#include <Arduino.h>

struct TelemetryData 
{
    Vector3         accel;
    Vector3         gyro;
    FliteQuaternion quat;
    EulerAngles     orientation;
    EulerAngles     attitudeSetpoint;
    EulerAngles     rateSetpoint;
    EulerAngles     attitudeCmd;
    EulerAngles     rateCmd;
    float           altitude;
    int             flight_state;
    int             flight_mode;

    // battery
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
    void update(const ArduFliteIMU &myIMU, const ArduFliteController &myController, const ArdufliteCRSFReceiver &crsfReceiver) 
    {
        // Update data from ArduFlite IMU
        accel               = myIMU.getAcceleration();
        gyro                = myIMU.getGyro();
        quat                = myIMU.getQuaternion();
        orientation         = myIMU.getOrientation();

        // Update data from ArduFlite Controller
        attitudeSetpoint    = myController.getAttitudeSetpoint();
        rateSetpoint        = myController.getRateSetpoint();
        attitudeCmd         = myController.getAttitudeCmd();
        rateCmd             = myController.getRateCmd();

        // Update flight state and mode
        flight_state        = static_cast<int>(myIMU.getFlightState());
        flight_mode         = static_cast<int>( myController.getMode());

        // Update additional flight data
        altitude            = myIMU.getAltitude();
        battery_voltage     = 0.0f; //TODO
        battery_current     = 0.0f; //TODO
        battery_consumed    = 0;    //TODO
        battery_remaining   = 100;  //TODO

        // GPS
        gps_lat             = 0.0f; //TODO
        gps_lon             = 0.0f; //TODO
        gps_alt             = 0.0f; //TODO
        gps_groundspeed     = 0.0f; //TODO
        gps_heading         = 0.0f; //TODO
        gps_sats            = 0;    //TODO

        // vario
        climb_rate          = myIMU.getClimbRate();

        // System status
        armed               = myController.isArmed();
        in_failsafe         = crsfReceiver.isInFailsafe();

        // Update link statistics
        crsfLinkStatistics_t stats{};
        crsfReceiver.getLinkStats(stats);

        link_rssi1          = stats.uplink_RSSI_1;
        link_rssi2          = stats.uplink_RSSI_2;
        link_quality        = stats.uplink_Link_quality;
        link_snr            = stats.uplink_SNR;
        link_antenna        = stats.active_antenna;
        link_rf_mode        = stats.rf_Mode;
        link_tx_power       = stats.uplink_TX_Power;
        dl_rssi             = stats.downlink_RSSI;
        dl_quality          = stats.downlink_Link_quality;
        dl_snr              = stats.downlink_SNR;
    }
};

#endif // TELEMETRY_DATA_H
