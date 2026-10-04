/**
 * MadgwickEstimator.cpp
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * THINGS TO BE AWARE OF before editing the arithmetic below.
 *
 * 1. **A degenerate gradient is skipped, never divided by.** When the measured
 *    acceleration exactly matches the predicted gravity, the objective function
 *    is the zero vector and so is its gradient. Normalising it would be
 *    `0 * inf` = **NaN**, and a NaN quaternion never recovers. Unlikely with a
 *    noisy sensor, and immediately reachable with the synthetic input that
 *    host_sim and the unit tests feed.
 *
 * 2. **Euler angles are computed on demand**, so they can never lag the
 *    quaternion.
 *
 * 3. **setOrientation() normalises**, because every step below assumes a unit
 *    quaternion.
 * ─────────────────────────────────────────────────────────────────────────
 */
#include "src/estimation/MadgwickEstimator.h"

#include <cmath>

#include "src/hal/core/Units.h"

namespace arduflite::estimation {

namespace {

/**
 * @brief Below this, a vector has no usable direction.
 *
 * Applied to squared lengths, so it is (1e-12)^2 in the quantities involved.
 * The gradient and the sensor vectors are all normalised before use, and
 * normalising something this short produces noise, not a direction.
 */
constexpr float kNegligibleSquared = 1e-24f;

/// @return false when the vector is too short to have a direction, leaving
///         `out` untouched. Callers treat that as "no measurement".
bool normalised(const Vec3f& in, Vec3f& out)
{
    const float lengthSquared = in.x * in.x + in.y * in.y + in.z * in.z;
    if (lengthSquared < kNegligibleSquared) { return false; }

    const float scale = 1.0f / std::sqrt(lengthSquared);
    out = Vec3f{ in.x * scale, in.y * scale, in.z * scale };
    return true;
}

} // namespace

void MadgwickEstimator::begin(float /*rate_hz*/)
{
    _q = Quaternion{ 1.0f, 0.0f, 0.0f, 0.0f };
}

void MadgwickEstimator::setOrientation(const Quaternion& q)
{
    const float lengthSquared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    if (lengthSquared < kNegligibleSquared)
    {
        _q = Quaternion{ 1.0f, 0.0f, 0.0f, 0.0f };
        return;
    }

    const float scale = 1.0f / std::sqrt(lengthSquared);
    _q = Quaternion{ q.w * scale, q.x * scale, q.y * scale, q.z * scale };
}

void MadgwickEstimator::integrate(const Vec3f& gyro_dps, const float gradient[4], float dt_s)
{
    const float gx = gyro_dps.x * units::kDegToRad;
    const float gy = gyro_dps.y * units::kDegToRad;
    const float gz = gyro_dps.z * units::kDegToRad;

    // Quaternion rate from the gyroscope alone: qDot = 0.5 * q (x) (0, omega).
    float rate[4] = {
        0.5f * (-_q.x * gx - _q.y * gy - _q.z * gz),
        0.5f * ( _q.w * gx + _q.y * gz - _q.z * gy),
        0.5f * ( _q.w * gy - _q.x * gz + _q.z * gx),
        0.5f * ( _q.w * gz + _q.x * gy - _q.y * gx),
    };

    // Steepest descent, normalised so beta sets the correction RATE in rad/s
    // independently of how large the error currently is.
    const float gradientLengthSquared = gradient[0] * gradient[0] + gradient[1] * gradient[1] +
                                        gradient[2] * gradient[2] + gradient[3] * gradient[3];
    if (gradientLengthSquared >= kNegligibleSquared)
    {
        const float scale = 1.0f / std::sqrt(gradientLengthSquared);
        for (int i = 0; i < 4; ++i) { rate[i] -= _beta * gradient[i] * scale; }
    }
    // else: the estimate already satisfies the measurement. Divergence 1 above.

    Quaternion integrated{
        _q.w + rate[0] * dt_s,
        _q.x + rate[1] * dt_s,
        _q.y + rate[2] * dt_s,
        _q.z + rate[3] * dt_s,
    };

    const float lengthSquared = integrated.w * integrated.w + integrated.x * integrated.x +
                                integrated.y * integrated.y + integrated.z * integrated.z;
    if (lengthSquared < kNegligibleSquared) { return; }   // keep the last good attitude

    const float scale = 1.0f / std::sqrt(lengthSquared);
    _q = Quaternion{ integrated.w * scale, integrated.x * scale,
                     integrated.y * scale, integrated.z * scale };
}

void MadgwickEstimator::update(const Vec3f& gyro_dps, const Vec3f& accel_g, float dt_s)
{
    Vec3f a{};
    if (!normalised(accel_g, a))
    {
        // Free fall, or a dead sensor. Integrate the gyro and correct nothing:
        // an accelerometer reading no gravity carries no attitude information.
        const float noCorrection[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        integrate(gyro_dps, noCorrection, dt_s);
        return;
    }

    const float qw = _q.w, qx = _q.x, qy = _q.y, qz = _q.z;

    // ── Objective: where the filter thinks "down" is, minus where the
    //    accelerometer says it is. Zero when the estimate is consistent with
    //    the measurement. This is the third row of the body-to-earth rotation
    //    matrix, which is the earth-frame down axis expressed in body axes.
    const float f[3] = {
        2.0f * (qx * qz - qw * qy)          - a.x,
        2.0f * (qw * qx + qy * qz)          - a.y,
        1.0f - 2.0f * (qx * qx + qy * qy)   - a.z,
    };

    // ── Jacobian df/dq. Each entry is one partial derivative of the three
    //    lines above; written out so it can be checked term by term.
    const float J[3][4] = {
        { -2.0f * qy,  2.0f * qz, -2.0f * qw,  2.0f * qx },
        {  2.0f * qx,  2.0f * qw,  2.0f * qz,  2.0f * qy },
        {  0.0f,      -4.0f * qx, -4.0f * qy,  0.0f      },
    };

    // ── Gradient of the squared error: J^T f.
    float gradient[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (int component = 0; component < 4; ++component)
    {
        for (int row = 0; row < 3; ++row)
        {
            gradient[component] += J[row][component] * f[row];
        }
    }

    integrate(gyro_dps, gradient, dt_s);
}

void MadgwickEstimator::updateWithMagnetometer(const Vec3f& gyro_dps, const Vec3f& accel_g,
                                               const Vec3f& mag_ut, float dt_s)
{
    Vec3f a{};
    Vec3f m{};
    if (!normalised(mag_ut, m) || !normalised(accel_g, a))
    {
        // No usable field, or no usable gravity. Six-axis is strictly better
        // than feeding the nine-axis solve a direction that does not exist.
        update(gyro_dps, accel_g, dt_s);
        return;
    }

    const float qw = _q.w, qx = _q.x, qy = _q.y, qz = _q.z;

    // ── Earth-frame reference direction for the measured field.
    //    Rotate the measurement into the earth frame, then collapse its
    //    horizontal part onto a single axis. That is what makes the correction
    //    a HEADING correction: the filter is told the field's inclination by
    //    the measurement itself, so an inclination error cannot fight gravity.
    const float hx = m.x * (1.0f - 2.0f * (qy * qy + qz * qz)) +
                     m.y * 2.0f * (qx * qy - qw * qz) +
                     m.z * 2.0f * (qx * qz + qw * qy);
    const float hy = m.x * 2.0f * (qx * qy + qw * qz) +
                     m.y * (1.0f - 2.0f * (qx * qx + qz * qz)) +
                     m.z * 2.0f * (qy * qz - qw * qx);
    const float hz = m.x * 2.0f * (qx * qz - qw * qy) +
                     m.y * 2.0f * (qy * qz + qw * qx) +
                     m.z * (1.0f - 2.0f * (qx * qx + qy * qy));

    const float bx = std::sqrt(hx * hx + hy * hy);
    const float bz = hz;

    // ── Objective: gravity (rows 0-2) stacked with the magnetic field
    //    (rows 3-5). One solve over both, which is why a large magnetic
    //    residual takes correction authority from gravity — see ADR-054.
    const float f[6] = {
        2.0f * (qx * qz - qw * qy)        - a.x,
        2.0f * (qw * qx + qy * qz)        - a.y,
        1.0f - 2.0f * (qx * qx + qy * qy) - a.z,

        2.0f * bx * (0.5f - qy * qy - qz * qz) + 2.0f * bz * (qx * qz - qw * qy) - m.x,
        2.0f * bx * (qx * qy - qw * qz)        + 2.0f * bz * (qw * qx + qy * qz) - m.y,
        2.0f * bx * (qw * qy + qx * qz)        + 2.0f * bz * (0.5f - qx * qx - qy * qy) - m.z,
    };

    const float J[6][4] = {
        { -2.0f * qy, 2.0f * qz, -2.0f * qw, 2.0f * qx },
        {  2.0f * qx, 2.0f * qw,  2.0f * qz, 2.0f * qy },
        {  0.0f,     -4.0f * qx, -4.0f * qy, 0.0f      },

        { -2.0f * bz * qy,
           2.0f * bz * qz,
          -4.0f * bx * qy - 2.0f * bz * qw,
          -4.0f * bx * qz + 2.0f * bz * qx },
        { -2.0f * bx * qz + 2.0f * bz * qx,
           2.0f * bx * qy + 2.0f * bz * qw,
           2.0f * bx * qx + 2.0f * bz * qz,
          -2.0f * bx * qw + 2.0f * bz * qy },
        {  2.0f * bx * qy,
           2.0f * bx * qz - 4.0f * bz * qx,
           2.0f * bx * qw - 4.0f * bz * qy,
           2.0f * bx * qx },
    };

    float gradient[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    for (int component = 0; component < 4; ++component)
    {
        for (int row = 0; row < 6; ++row)
        {
            gradient[component] += J[row][component] * f[row];
        }
    }

    integrate(gyro_dps, gradient, dt_s);
}

EulerAnglesDeg MadgwickEstimator::euler_deg() const
{
    const float qw = _q.w, qx = _q.x, qy = _q.y, qz = _q.z;

    EulerAnglesDeg euler{};
    euler.roll  = std::atan2(qw * qx + qy * qz, 0.5f - qx * qx - qy * qy) * units::kRadToDeg;

    // asin's domain is [-1, 1]; a quaternion one ulp off unit length can push
    // the argument outside it and produce NaN at exactly 90 degrees of pitch.
    const float pitchArgument = -2.0f * (qx * qz - qw * qy);
    euler.pitch = std::asin(pitchArgument < -1.0f ? -1.0f
                          : (pitchArgument > 1.0f ? 1.0f : pitchArgument)) * units::kRadToDeg;

    // Signed, like roll and pitch, and like the attitude setpoint domain.
    euler.yaw = std::atan2(qx * qy + qw * qz, 0.5f - qy * qy - qz * qz) * units::kRadToDeg;

    return euler;
}

} // namespace arduflite::estimation
