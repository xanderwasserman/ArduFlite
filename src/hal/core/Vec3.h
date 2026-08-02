/**
 * Vec3.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Three-component float vector. Trivially copyable — SeqLock depends on it.
 */
#ifndef ARDUFLITE_HAL_CORE_VEC3_H
#define ARDUFLITE_HAL_CORE_VEC3_H

#include <cmath>

namespace arduflite {

struct Vec3f
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3f operator+(const Vec3f& o) const noexcept { return { x + o.x, y + o.y, z + o.z }; }
    constexpr Vec3f operator-(const Vec3f& o) const noexcept { return { x - o.x, y - o.y, z - o.z }; }
    constexpr Vec3f operator*(float s)        const noexcept { return { x * s,   y * s,   z * s   }; }
    constexpr Vec3f operator-()               const noexcept { return { -x, -y, -z }; }

    constexpr bool operator==(const Vec3f& o) const noexcept
    {
        return x == o.x && y == o.y && z == o.z;
    }

    /// Prefer this in hot paths — no sqrt.
    [[nodiscard]] constexpr float magnitudeSquared() const noexcept { return x * x + y * y + z * z; }
    [[nodiscard]] float           magnitude()        const noexcept { return std::sqrt(magnitudeSquared()); }

    /// One place for the NaN/Inf guard that is currently duplicated per axis.
    [[nodiscard]] bool isFinite() const noexcept
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

static_assert(sizeof(Vec3f) == 12, "Vec3f must stay packed — it is logged and seqlocked");

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_VEC3_H
