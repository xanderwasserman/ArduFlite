/**
 * LowPassBank.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief First-order exponential moving average over a Vec3f.
 *
 * One alpha per bank rather than nine
 * hand-written copies of the same line — three axes for each of accelerometer,
 * gyroscope and magnetometer.
 */
#ifndef ARDUFLITE_ESTIMATION_LOW_PASS_BANK_H
#define ARDUFLITE_ESTIMATION_LOW_PASS_BANK_H

#include "src/hal/core/Vec3.h"

namespace arduflite::estimation {

class LowPassVec3
{
public:
    /// @param alpha 0..1. Higher follows the input faster; 1.0 is no filtering.
    void setAlpha(float alpha) { _alpha = alpha; }
    [[nodiscard]] float alpha() const { return _alpha; }

    /**
     * @brief Filter one sample.
     *
     * The first sample after construction or reset() is adopted OUTRIGHT rather
     * than blended toward from zero. Blending from zero would drag the first
     * second of every flight toward the origin — visible on the accelerometer as
     * a gravity vector that fades in, and on the estimator as an attitude that
     * settles from level regardless of how the aircraft is actually sitting.
     */
    Vec3f update(const Vec3f& input)
    {
        if (!_seeded)
        {
            _value  = input;
            _seeded = true;
            return _value;
        }

        const float beta = 1.0f - _alpha;
        _value = Vec3f{ _alpha * input.x + beta * _value.x,
                        _alpha * input.y + beta * _value.y,
                        _alpha * input.z + beta * _value.z };
        return _value;
    }

    [[nodiscard]] const Vec3f& value() const { return _value; }

    /// Drop the history. The next sample is adopted outright.
    void reset() { _seeded = false; _value = Vec3f{}; }

    [[nodiscard]] bool seeded() const { return _seeded; }

private:
    Vec3f _value{};
    float _alpha  = 1.0f;
    bool  _seeded = false;
};

} // namespace arduflite::estimation

#endif // ARDUFLITE_ESTIMATION_LOW_PASS_BANK_H
