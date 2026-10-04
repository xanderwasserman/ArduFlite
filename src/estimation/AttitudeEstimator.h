/**
 * AttitudeEstimator.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Sensor fusion behind an interface, so the filter is swappable.
 *
 * Exists so that Phase 9 can swap Adafruit_Madgwick for an own implementation
 * against a fixed contract, rather than editing fusion arithmetic in place
 * inside the sampling task (ADR-014, ADR-017).
 */
#ifndef ARDUFLITE_ESTIMATION_ATTITUDE_ESTIMATOR_H
#define ARDUFLITE_ESTIMATION_ATTITUDE_ESTIMATOR_H

#include "src/estimation/ImuState.h"
#include "src/hal/core/NonCopyable.h"
#include "src/hal/core/Vec3.h"

namespace arduflite::estimation {

class AttitudeEstimator : private NonCopyable
{
public:
    virtual ~AttitudeEstimator() = default;

    /// @param rate_hz the nominal tick rate. Some filters derive a gain from it.
    virtual void begin(float rate_hz) = 0;

    /// Gyro-and-accelerometer update (no magnetometer).
    /// @param gyro_dps  degrees per second, body frame
    /// @param accel_g   g, body frame
    /// @param dt_s      seconds since the previous update
    virtual void update(const Vec3f& gyro_dps, const Vec3f& accel_g, float dt_s) = 0;

    /// Nine-axis update. Implementations without magnetometer support may
    /// delegate to update(); the caller is responsible for only passing a
    /// magnetometer reading when one exists.
    virtual void updateWithMagnetometer(const Vec3f& gyro_dps, const Vec3f& accel_g,
                                        const Vec3f& mag_ut, float dt_s) = 0;

    [[nodiscard]] virtual Quaternion     orientation() const = 0;
    [[nodiscard]] virtual EulerAnglesDeg euler_deg()   const = 0;

    /**
     * @brief Force the filter's state to a known orientation.
     *
     * Not used yet. It exists because it is the enabling primitive for the
     * "re-seed" failover strategy (ADR-026): on a sensor switch the estimator
     * must be able to adopt the new instance's attitude instead of slewing to it
     * over several seconds. Retro-fitting this into a filter later is much
     * harder than declaring it now.
     *
     * @warning Implementations are NOT required to make euler_deg() reflect the
     *          new orientation before the next update() — the Adafruit-backed
     *          one does not. Read orientation() if you need it immediately.
     */
    virtual void setOrientation(const Quaternion& q) = 0;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_ATTITUDE_ESTIMATOR_H
