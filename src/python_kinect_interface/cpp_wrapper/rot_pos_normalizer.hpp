//this file will rotate all joint orientations so that the base of the spine always stays at the same orientation
//assumes all quaternions are normalized

#include "kinect_wrapper.h"
#include "kinect.hpp"


    /////////////////////////////////////////////////
    //             QUATERNION ROTATIONS            //    
    /////////////////////////////////////////////////               

quaternionC multiply(const quaternionC& q_a, const quaternionC& q_b){
    return quaternionC {
        .x = q_a.w * q_b.x + q_a.x * q_b.w + q_a.y * q_b.z - q_a.z * q_b.y,
        .y = q_a.w * q_b.y - q_a.x * q_b.z + q_a.y * q_b.w + q_a.z * q_b.x,
        .z = q_a.w * q_b.z + q_a.x * q_b.y - q_a.y * q_b.x + q_a.z * q_b.w,
        .w = q_a.w * q_b.w - q_a.x * q_b.x - q_a.y * q_b.y - q_a.z * q_b.z
    };
}

//the inverse of a quaterneon is the rotation required to make that quaternion into the identity quaternion
inline quaternionC get_conjugate_quaternion(const quaternionC& q){
    return quaternionC {-q.x, -q.y, -q.z, q.w};
} 

extern "C" { //ABI for Quaternions

    //rotates a quaternion with respect to the global axis
    quaternionC rotate_quaternion_global(const quaternionC& q_rotatee, const quaternionC& q_rotator){
        //to achieve rotation with respect to the global axis, multiply a * q_b (order matters)
        return multiply(q_rotator, q_rotatee);
    } 

    //finds the rotation necessary for the quaterion to orient directly in the +y direciton
    quaternionC get_rotator(const quaternionC& q, const quaternionC& q_target){
        quaternionC q_conjugate = get_conjugate_quaternion(q);

        return multiply(q_target, q_conjugate);
    }
}

    /////////////////////////////////////////////////
    //             VECTOR ROTATIONS            //    
    /////////////////////////////////////////////////   

//get the cross product of two vectors
inline coordinate_3dC cross(const coordinate_3dC& a, const coordinate_3dC& b){
    return coordinate_3dC{
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };
}

//get the dot product of two vectors
inline float dot(const coordinate_3dC& a, const coordinate_3dC& b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

//overload multiplication operator for vectors
inline coordinate_3dC operator*(const float& a, const coordinate_3dC& b){
    return coordinate_3dC {
        .x = a * b.x,
        .y = a * b.y,
        .z = a * b.z
    };
}

//overload addition operator for vectors
inline coordinate_3dC operator+(const coordinate_3dC& a, const coordinate_3dC& b){
    return coordinate_3dC {
        .x = a.x + b.x,
        .y = a.y + b.y,
        .z = a.z + b.z
    };
}

extern "C"{ //ABI for vectors

    //rotates a point by a quaternion on the global axis
    coordinate_3dC rotate_point(const coordinate_3dC& v, const quaternionC& q){
        //uses algorithm from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        
        //get the vector component of the quaternion
        const coordinate_3dC q_vector {
            .x = q.x,
            .y = q.y,
            .z = q.z
        };

        //get the scalar component of the quaternion
        const float q_w = q.w;

        //get the rotation direction vector
        const coordinate_3dC t = 2 * cross(q_vector, v);

        //generate rotation path
        const coordinate_3dC rotation_path = cross(q_vector, t);

        //scale
        const coordinate_3dC scaled_t = q_w * t;

        //combine
        const coordinate_3dC v_prime = v + scaled_t + rotation_path;

        return v_prime;
    }
}