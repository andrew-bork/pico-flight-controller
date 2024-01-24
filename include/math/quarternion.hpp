#pragma once

#include <math/vector.hpp>

namespace math {
    struct quarternion{
        float w, x, y, z;
        bool unit;

        quarternion();
        quarternion(float x, float y, float z);
        quarternion(float w, float x, float y, float z);
        quarternion(float w, float x, float y, float z, bool unit);

        quarternion operator+ (const quarternion& r);
        quarternion operator* (const quarternion& r);

        static quarternion inverse(const quarternion& n);
        static quarternion conjugate(const quarternion& n);
        static quarternion rotate(float theta, const vector& axis);
        static quarternion from_euler_ZYX(const vector& euler);
        
        static vector to_euler(const quarternion& q);
        static vector to_magnitude_axis(const quarternion& q);
        static vector rotate_vector(math::quarternion& q, math::vector& in);
    };
    
    float length(const quarternion& n);

};
