#ifndef FLITE_QUATERNION_H
#define FLITE_QUATERNION_H

#include <math.h>

class FliteQuaternion {
public:
    float w, x, y, z;

    // Constructor: defaults to the identity quaternion.
    FliteQuaternion(float _w = 1.0f, float _x = 0.0f, float _y = 0.0f, float _z = 0.0f);

    // Quaternion multiplication (Hamilton product): q1 * q2.
    FliteQuaternion operator*(const FliteQuaternion &other) const;

    // Returns the conjugate (w, -x, -y, -z).
    FliteQuaternion conjugate() const;

    // Returns the norm squared.
    float normSq() const;

    // Returns the inverse: q^-1 = conjugate(q)/normSq(q).
    FliteQuaternion inverse() const;

    // Converts the quaternion to axis-angle representation.
    // The axis (rx, ry, rz) is normalized and outAngle is in radians.
    void toAxisAngle(float &rx, float &ry, float &rz, float &outAngle) const;

    // Normalize the quaternion in place.
    void normalize();
};

#endif // FLITE_QUATERNION_H
