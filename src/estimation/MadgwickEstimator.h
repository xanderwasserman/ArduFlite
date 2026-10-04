/**
 * MadgwickEstimator.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Gradient-descent orientation filter. The one the aircraft flies with.
 *
 * Own implementation for LICENSING reasons, not performance: Adafruit_AHRS
 * carries the x-io GPL notice, and this project is MIT. Fusion cost measures at
 * 0.3-0.8 % of the core either way, so do not swap back for speed (ADR-017).
 *
 * Written from the published algorithm — S. Madgwick, "An efficient orientation
 * filter for inertial and inertial/magnetic sensor arrays" (2010) — in the
 * paper's own structure: build the objective function `f`, build its Jacobian
 * `J`, take the gradient as `J^T f`. The reference C implementations flatten all
 * of that into one scalar expression per quaternion component, which is faster
 * to execute and far harder to check. Here the two steps stay separate and each
 * matrix entry is one partial derivative you can verify by hand.
 *
 * Agreement with the Adafruit-backed estimator is not argued from the code —
 * it is measured, by replaying a real flight through both and comparing
 * (tests/unit/test_log_replay.cpp). Keep that test working.
 *
 * @note NOT a clean-room reimplementation in the strict sense: the Adafruit
 *       source was read during this work. The algorithm is published and
 *       reimplementing from the paper is the normal route, but if the licence
 *       question is the point, that is worth knowing rather than assuming.
 */
#ifndef ARDUFLITE_ESTIMATION_MADGWICK_ESTIMATOR_H
#define ARDUFLITE_ESTIMATION_MADGWICK_ESTIMATOR_H

#include "src/estimation/AttitudeEstimator.h"

namespace arduflite::estimation {

class MadgwickEstimator final : public AttitudeEstimator
{
public:
    /// Matches the Adafruit default, so a board that switches implementations
    /// without touching config behaves the same.
    static constexpr float kDefaultBeta = 0.1f;

    /**
     * @param rate_hz accepted for interface compatibility and otherwise unused.
     *
     * The reference filter kept a nominal sample period from begin() for its
     * dt-less overload. ArduFlite has always passed dt explicitly — the tick
     * measures it and clamps it — so there is no second source of timing to
     * disagree with the first.
     */
    void begin(float rate_hz) override;

    void update(const Vec3f& gyro_dps, const Vec3f& accel_g, float dt_s) override;
    void updateWithMagnetometer(const Vec3f& gyro_dps, const Vec3f& accel_g,
                                const Vec3f& mag_ut, float dt_s) override;

    [[nodiscard]] Quaternion     orientation() const override { return _q; }
    [[nodiscard]] EulerAnglesDeg euler_deg()   const override;

    /**
     * @brief Force the filter to a known orientation.
     *
     * The argument is normalised, and euler_deg() reflects it immediately.
     */
    void setOrientation(const Quaternion& q) override;

    /// Gyro/accelerometer trust balance. Higher trusts the accelerometer more:
    /// faster convergence, more noise.
    void  setBeta(float beta) { _beta = beta; }
    [[nodiscard]] float beta() const { return _beta; }

private:
    /// Integrate the gyro, apply a normalised correction, renormalise.
    /// @param gradient  unnormalised dF/dq; ignored if it is degenerate
    void integrate(const Vec3f& gyro_dps, const float gradient[4], float dt_s);

    Quaternion _q{ 1.0f, 0.0f, 0.0f, 0.0f };
    float      _beta = kDefaultBeta;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_MADGWICK_ESTIMATOR_H
