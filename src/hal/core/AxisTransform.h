/**
 * AxisTransform.h
 *
 * ArduFlite - Advanced Flight Controller Framework
 * Author: Alexander Wasserman | Version: 1.0 | 02 August 2026
 *
 * Licensed under the MIT License. See LICENSE file for details.
 *
 * @brief Sensor-to-body alignment. Replaces ArduFliteIMU::applyOrientation().
 *
 * A signed axis MAP rather than a rotation, because the aircraft currently flies
 * with a transform whose determinant is -1 (see specs/hal/00-current-state.md 2.3),
 * which no proper rotation can express.
 *
 * The critical rule, and the reason applyMeasurement() and applyAngularRate() are
 * separate methods:
 *
 *   - A per-axis MEASUREMENT (acceleration, magnetic field) transforms as  M.
 *   - An ANGULAR RATE transforms as  det(M) * M,  because the right-hand rule
 *     flips when the map produces a left-handed frame.
 *
 * Getting that wrong is the classic mirrored-mount bug. Two named methods make it
 * impossible to forget.
 */
#ifndef ARDUFLITE_HAL_CORE_AXISTRANSFORM_H
#define ARDUFLITE_HAL_CORE_AXISTRANSFORM_H

#include <cmath>
#include <cstdint>

#include "src/hal/core/Vec3.h"

namespace arduflite {

// ─────────────────────────────────────────────────────────────────────────────
// Signed axis
// ─────────────────────────────────────────────────────────────────────────────

/// Which sensor axis, and with what sign. Magnitude 1/2/3 == X/Y/Z.
enum class SignedAxis : std::int8_t
{
    PlusX  =  1, MinusX = -1,
    PlusY  =  2, MinusY = -2,
    PlusZ  =  3, MinusZ = -3,
};

/// 0, 1 or 2 for X, Y, Z.
constexpr int axisIndex(SignedAxis a) noexcept
{
    const int v = static_cast<int>(a);
    return (v < 0 ? -v : v) - 1;
}

constexpr int axisSign(SignedAxis a) noexcept
{
    return static_cast<int>(a) < 0 ? -1 : 1;
}

constexpr const char* toString(SignedAxis a) noexcept
{
    switch (a)
    {
        case SignedAxis::PlusX:  return "+X";
        case SignedAxis::MinusX: return "-X";
        case SignedAxis::PlusY:  return "+Y";
        case SignedAxis::MinusY: return "-Y";
        case SignedAxis::PlusZ:  return "+Z";
        case SignedAxis::MinusZ: return "-Z";
    }
    return "??";
}

// ─────────────────────────────────────────────────────────────────────────────
// Axis map — all 48 axis-aligned mappings (24 proper rotations + 24 reflections)
// ─────────────────────────────────────────────────────────────────────────────

struct AxisMap
{
    /// Each member names the SENSOR axis that feeds that BODY axis.
    /// e.g. y == MinusY  =>  body.y = -sensor.y
    SignedAxis x = SignedAxis::PlusX;
    SignedAxis y = SignedAxis::PlusY;
    SignedAxis z = SignedAxis::PlusZ;

    /// True when each of X, Y, Z is used exactly once. A map that fails this is
    /// singular and would silently destroy an axis.
    [[nodiscard]] constexpr bool isValid() const noexcept
    {
        const int ix = axisIndex(x);
        const int iy = axisIndex(y);
        const int iz = axisIndex(z);
        return ix != iy && iy != iz && ix != iz;
    }

    /// +1 for a proper rotation, -1 for a reflection (a mirrored frame).
    /// @note Meaningless unless isValid().
    [[nodiscard]] constexpr int determinant() const noexcept
    {
        int m[3][3] = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };
        m[0][axisIndex(x)] = axisSign(x);
        m[1][axisIndex(y)] = axisSign(y);
        m[2][axisIndex(z)] = axisSign(z);

        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    [[nodiscard]] constexpr bool isMirrored() const noexcept { return determinant() < 0; }

    constexpr bool operator==(const AxisMap& o) const noexcept
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

/// The SignedAxis feeding body axis i (0=X, 1=Y, 2=Z).
constexpr SignedAxis axisAt(const AxisMap& m, int i) noexcept
{
    return (i == 0) ? m.x : ((i == 1) ? m.y : m.z);
}

/// Apply `inner` first, then `outer`.
constexpr AxisMap compose(const AxisMap& outer, const AxisMap& inner) noexcept
{
    const auto composeOne = [&](SignedAxis outerAxis) constexpr -> SignedAxis
    {
        const SignedAxis innerAxis = axisAt(inner, axisIndex(outerAxis));
        const bool negate = (axisSign(outerAxis) * axisSign(innerAxis)) < 0;
        const int  magnitude = axisIndex(innerAxis) + 1;
        return static_cast<SignedAxis>(negate ? -magnitude : magnitude);
    };
    return { composeOne(outer.x), composeOne(outer.y), composeOne(outer.z) };
}

// ─────────────────────────────────────────────────────────────────────────────
// Named rotations — the 24 proper ones, for the common case
// ─────────────────────────────────────────────────────────────────────────────

/// Naming follows ArduPilot so datasheets and community mounting advice translate.
enum class Rotation : std::uint8_t
{
    None = 0,      Yaw90,            Yaw180,            Yaw270,
    Roll180,       Roll180Yaw90,     Roll180Yaw180,     Roll180Yaw270,
    Roll90,        Roll90Yaw90,      Roll90Yaw180,      Roll90Yaw270,
    Roll270,       Roll270Yaw90,     Roll270Yaw180,     Roll270Yaw270,
    Pitch90,       Pitch90Yaw90,     Pitch90Yaw180,     Pitch90Yaw270,
    Pitch270,      Pitch270Yaw90,    Pitch270Yaw180,    Pitch270Yaw270,
    Count
};

namespace detail {

// Sensor-to-body maps for a sensor physically rotated by the named amount.
inline constexpr AxisMap kIdentity { SignedAxis::PlusX,  SignedAxis::PlusY,  SignedAxis::PlusZ  };
inline constexpr AxisMap kYaw90    { SignedAxis::MinusY, SignedAxis::PlusX,  SignedAxis::PlusZ  };
inline constexpr AxisMap kYaw180   { SignedAxis::MinusX, SignedAxis::MinusY, SignedAxis::PlusZ  };
inline constexpr AxisMap kYaw270   { SignedAxis::PlusY,  SignedAxis::MinusX, SignedAxis::PlusZ  };
inline constexpr AxisMap kRoll90   { SignedAxis::PlusX,  SignedAxis::MinusZ, SignedAxis::PlusY  };
inline constexpr AxisMap kRoll180  { SignedAxis::PlusX,  SignedAxis::MinusY, SignedAxis::MinusZ };
inline constexpr AxisMap kRoll270  { SignedAxis::PlusX,  SignedAxis::PlusZ,  SignedAxis::MinusY };
inline constexpr AxisMap kPitch90  { SignedAxis::PlusZ,  SignedAxis::PlusY,  SignedAxis::MinusX };
inline constexpr AxisMap kPitch270 { SignedAxis::MinusZ, SignedAxis::PlusY,  SignedAxis::PlusX  };

constexpr AxisMap baseFor(std::uint8_t group) noexcept
{
    switch (group)
    {
        case 0:  return kIdentity;
        case 1:  return kRoll180;
        case 2:  return kRoll90;
        case 3:  return kRoll270;
        case 4:  return kPitch90;
        default: return kPitch270;
    }
}

constexpr AxisMap yawFor(std::uint8_t step) noexcept
{
    switch (step)
    {
        case 0:  return kIdentity;
        case 1:  return kYaw90;
        case 2:  return kYaw180;
        default: return kYaw270;
    }
}

} // namespace detail

/// Rotation is a named subset of AxisMap — every one of these has determinant +1.
constexpr AxisMap toAxisMap(Rotation r) noexcept
{
    const auto raw   = static_cast<std::uint8_t>(r);
    const auto group = static_cast<std::uint8_t>(raw / 4u);
    const auto yaw   = static_cast<std::uint8_t>(raw % 4u);
    return compose(detail::yawFor(yaw), detail::baseFor(group));
}

// ─────────────────────────────────────────────────────────────────────────────
// Fine alignment trim
// ─────────────────────────────────────────────────────────────────────────────

/// Small Euler offsets for a board that is square-ish but not exactly square in
/// the fuselage. Composed into the transform once, at construction.
struct AlignmentTrim
{
    float roll_deg  = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg   = 0.0f;

    [[nodiscard]] constexpr bool isZero() const noexcept
    {
        return roll_deg == 0.0f && pitch_deg == 0.0f && yaw_deg == 0.0f;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// The composed transform
// ─────────────────────────────────────────────────────────────────────────────

class AxisTransform
{
public:
    constexpr AxisTransform() noexcept = default;

    explicit AxisTransform(AxisMap map, AlignmentTrim trim = {}) noexcept
        : _map(map), _trim(trim), _det(static_cast<float>(map.determinant()))
    {
        buildMatrix();
    }

    /// Per-axis measurements: acceleration, magnetic field.
    [[nodiscard]] Vec3f applyMeasurement(const Vec3f& v) const noexcept
    {
        return { _m[0] * v.x + _m[1] * v.y + _m[2] * v.z,
                 _m[3] * v.x + _m[4] * v.y + _m[5] * v.z,
                 _m[6] * v.x + _m[7] * v.y + _m[8] * v.z };
    }

    /// Angular rate. Carries det(map): rotation *about* an axis changes sign when
    /// the map produces a left-handed frame.
    [[nodiscard]] Vec3f applyAngularRate(const Vec3f& v) const noexcept
    {
        const Vec3f m = applyMeasurement(v);
        return (_det < 0.0f) ? -m : m;
    }

    [[nodiscard]] constexpr AxisMap       map()        const noexcept { return _map; }
    [[nodiscard]] constexpr AlignmentTrim trim()       const noexcept { return _trim; }
    [[nodiscard]] constexpr bool          isMirrored() const noexcept { return _det < 0.0f; }

private:
    void buildMatrix() noexcept
    {
        // Row-major 3x3 for the signed axis map.
        float mapM[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        mapM[0 * 3 + axisIndex(_map.x)] = static_cast<float>(axisSign(_map.x));
        mapM[1 * 3 + axisIndex(_map.y)] = static_cast<float>(axisSign(_map.y));
        mapM[2 * 3 + axisIndex(_map.z)] = static_cast<float>(axisSign(_map.z));

        if (_trim.isZero())
        {
            // Exact +/-1 and 0 entries — bit-identical to a sign flip, which is
            // what the tests assert against the legacy applyOrientation().
            for (int i = 0; i < 9; ++i) { _m[i] = mapM[i]; }
            return;
        }

        constexpr float kDegToRad = 0.017453292519943295f;
        const float cr = std::cos(_trim.roll_deg  * kDegToRad);
        const float sr = std::sin(_trim.roll_deg  * kDegToRad);
        const float cp = std::cos(_trim.pitch_deg * kDegToRad);
        const float sp = std::sin(_trim.pitch_deg * kDegToRad);
        const float cy = std::cos(_trim.yaw_deg   * kDegToRad);
        const float sy = std::sin(_trim.yaw_deg   * kDegToRad);

        // Rz(yaw) * Ry(pitch) * Rx(roll), row-major.
        const float trimM[9] = {
            cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
            sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
              -sp,    cp * sr,                 cp * cr
        };

        // _m = trimM * mapM
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
            {
                _m[r * 3 + c] = trimM[r * 3 + 0] * mapM[0 * 3 + c]
                              + trimM[r * 3 + 1] * mapM[1 * 3 + c]
                              + trimM[r * 3 + 2] * mapM[2 * 3 + c];
            }
        }
    }

    AxisMap       _map{};
    AlignmentTrim _trim{};
    float         _m[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    float         _det  = 1.0f;
};

} // namespace arduflite

#endif // ARDUFLITE_HAL_CORE_AXISTRANSFORM_H
