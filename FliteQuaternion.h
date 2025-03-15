// Quaternion.h
#ifndef MY_QUATERNION_H
#define MY_QUATERNION_H

#include <math.h>

class FliteQuaternion {
public:
    // Components
    float w, x, y, z;

    // Default: identity quaternion
    FliteQuaternion(float _w=1.0f, float _x=0.0f, float _y=0.0f, float _z=0.0f)
      : w(_w), x(_x), y(_y), z(_z) {}

    // Quaternion multiplication (q1 * q2)
    FliteQuaternion operator*(const FliteQuaternion &other) const {
        // Hamilton product
        return FliteQuaternion(
          w*other.w - x*other.x - y*other.y - z*other.z,
          w*other.x + x*other.w + y*other.z - z*other.y,
          w*other.y - x*other.z + y*other.w + z*other.x,
          w*other.z + x*other.y - y*other.x + z*other.w
        );
    }

    // Returns the conjugate (w, -x, -y, -z)
    FliteQuaternion conjugate() const {
        return FliteQuaternion(w, -x, -y, -z);
    }

    // Returns the norm squared
    float normSq() const {
        return (w*w + x*x + y*y + z*z);
    }

    // Inverse: q^-1 = conjugate / (norm^2)
    FliteQuaternion inverse() const {
        float ns = normSq();
        if (ns < 1e-9) {
            // Degenerate, return identity
            return FliteQuaternion();
        }
        float inv = 1.0f / ns;
        return FliteQuaternion(w*inv, -x*inv, -y*inv, -z*inv);
    }

    // Convert to axis-angle. 
    //  axis (rx, ry, rz) is normalized. 
    //  angle in radians is returned via outAngle.
    void toAxisAngle(float &rx, float &ry, float &rz, float &outAngle) const {
        // if w>1.0, normalize first 
        FliteQuaternion qnorm = *this;
        float n = sqrtf(normSq());
        if (fabs(n - 1.0f) > 1e-3) {
            qnorm.w /= n;
            qnorm.x /= n;
            qnorm.y /= n;
            qnorm.z /= n;
        }

        // angle = 2 * acos(w)
        outAngle = 2.0f * acosf(qnorm.w);
        float s = sqrtf(1.0f - qnorm.w*qnorm.w);
        if (s < 1e-4) {
            // axis is arbitrary if angle is small
            rx = 1.0f; ry = 0.0f; rz = 0.0f;
        } else {
            rx = qnorm.x / s;
            ry = qnorm.y / s;
            rz = qnorm.z / s;
        }
    }
};

#endif // MY_QUATERNION_H
