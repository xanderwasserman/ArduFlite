#include "FliteQuaternion.h"
#include <math.h>

// Constructor
FliteQuaternion::FliteQuaternion(float _w, float _x, float _y, float _z)
    : w(_w), x(_x), y(_y), z(_z)
{
}

// Quaternion multiplication (Hamilton product)
FliteQuaternion FliteQuaternion::operator*(const FliteQuaternion &other) const {
    return FliteQuaternion(
        w * other.w - x * other.x - y * other.y - z * other.z,
        w * other.x + x * other.w + y * other.z - z * other.y,
        w * other.y - x * other.z + y * other.w + z * other.x,
        w * other.z + x * other.y - y * other.x + z * other.w
    );
}

// Conjugate: (w, -x, -y, -z)
FliteQuaternion FliteQuaternion::conjugate() const {
    return FliteQuaternion(w, -x, -y, -z);
}

// Norm squared of the quaternion
float FliteQuaternion::normSq() const {
    return (w*w + x*x + y*y + z*z);
}

// Inverse: q^-1 = conjugate(q) / normSq(q)
FliteQuaternion FliteQuaternion::inverse() const {
    float ns = normSq();
    if (ns < 1e-9f) {
        // If norm is too small, return identity as a safe fallback.
        return FliteQuaternion();
    }
    float inv = 1.0f / ns;
    return FliteQuaternion(w * inv, -x * inv, -y * inv, -z * inv);
}

// Convert quaternion to axis-angle representation.
void FliteQuaternion::toAxisAngle(float &rx, float &ry, float &rz, float &outAngle) const {
    // First, ensure the quaternion is normalized.
    FliteQuaternion qnorm = *this;
    float n = sqrtf(normSq());
    if (fabs(n - 1.0f) > 1e-3f) {
        qnorm.w /= n;
        qnorm.x /= n;
        qnorm.y /= n;
        qnorm.z /= n;
    }

    // The rotation angle is given by: angle = 2 * acos(w)
    outAngle = 2.0f * acosf(qnorm.w);
    float s = sqrtf(1.0f - qnorm.w * qnorm.w);
    if (s < 1e-4f) {
        // If s is nearly zero, the axis is not well-defined; choose an arbitrary axis.
        rx = 1.0f;
        ry = 0.0f;
        rz = 0.0f;
    } else {
        rx = qnorm.x / s;
        ry = qnorm.y / s;
        rz = qnorm.z / s;
    }
}

// Normalize the quaternion in place.
void FliteQuaternion::normalize() {
    float norm = sqrt(w*w + x*x + y*y + z*z);
    if (norm > 1e-6f) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    } else {
        // If the norm is too small, reset to identity quaternion.
        w = 1.0f;
        x = y = z = 0.0f;
    }
}
