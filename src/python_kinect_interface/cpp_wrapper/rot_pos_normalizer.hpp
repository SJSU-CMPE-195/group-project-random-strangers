#pragma once

// Rotate all joint orientations so that the base of the spine stays in a
// consistent orientation. Assumes input quaternions are normalized.

#include "kinect_wrapper.h"

/////////////////////////////////////////////////
//             QUATERNION ROTATIONS            //
/////////////////////////////////////////////////

inline quaternionC multiply(const quaternionC& q_a, const quaternionC& q_b) {
    return quaternionC {
        q_a.w * q_b.x + q_a.x * q_b.w + q_a.y * q_b.z - q_a.z * q_b.y,
        q_a.w * q_b.y - q_a.x * q_b.z + q_a.y * q_b.w + q_a.z * q_b.x,
        q_a.w * q_b.z + q_a.x * q_b.y - q_a.y * q_b.x + q_a.z * q_b.w,
        q_a.w * q_b.w - q_a.x * q_b.x - q_a.y * q_b.y - q_a.z * q_b.z
    };
}

// The inverse of a unit quaternion is its conjugate.
inline quaternionC get_conjugate_quaternion(const quaternionC& q) {
    return quaternionC {-q.x, -q.y, -q.z, q.w};
}

// Rotates a quaternion with respect to the global axis.
inline quaternionC rotate_quaternion_global(const quaternionC& q_rotatee, const quaternionC& q_rotator) {
    // For global-axis rotation, multiply q_rotator * q_rotatee. Order matters.
    return multiply(q_rotator, q_rotatee);
}

// Finds the rotation necessary for a quaternion to match the target quaternion.
inline quaternionC get_rotator(const quaternionC& q, const quaternionC& q_target) {
    const quaternionC q_conjugate = get_conjugate_quaternion(q);
    return multiply(q_target, q_conjugate);
}

/////////////////////////////////////////////////
//                VECTOR ROTATIONS             //
/////////////////////////////////////////////////

inline coordinate_3dC cross(const coordinate_3dC& a, const coordinate_3dC& b) {
    return coordinate_3dC {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

inline float dot(const coordinate_3dC& a, const coordinate_3dC& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline coordinate_3dC operator*(const float& a, const coordinate_3dC& b) {
    return coordinate_3dC {a * b.x, a * b.y, a * b.z};
}

inline coordinate_3dC operator+(const coordinate_3dC& a, const coordinate_3dC& b) {
    return coordinate_3dC {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Rotates a point by a quaternion on the global axis.
inline coordinate_3dC rotate_point(const coordinate_3dC& v, const quaternionC& q) {
    const coordinate_3dC q_vector {q.x, q.y, q.z};
    const float q_w = q.w;

    const coordinate_3dC t = 2.0f * cross(q_vector, v);
    const coordinate_3dC rotation_path = cross(q_vector, t);
    const coordinate_3dC scaled_t = q_w * t;

    return v + scaled_t + rotation_path;
}
