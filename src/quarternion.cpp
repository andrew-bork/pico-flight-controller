#include <math/quarternion.hpp>
#include <cmath>

math::quarternion::quarternion(){
    w = x = y = z = 0;
    unit = false;
}

math::quarternion::quarternion(float x, float y, float z){
    w = 0;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}
math::quarternion::quarternion(float w, float x, float y, float z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}

math::quarternion::quarternion(float w, float x, float y, float z, bool unit){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->unit = unit;
}



math::quarternion math::quarternion::operator*(const math::quarternion& r){
    quarternion res;
    res.w = w*r.w - x*r.x - y*r.y - z*r.z;
    res.x = r.w*x + r.x*w + r.y*z - r.z*y;
    res.y = r.w*y + r.y*w + r.z*x - r.x*z;
    res.z = r.w*z + r.z*w + r.x*y - r.y*x;
    return res;
}

math::quarternion math::quarternion::operator+(const math::quarternion& r){
    quarternion res;
    res.w = r.w + w;
    res.x = r.x + x;
    res.y = r.y + y;
    res.z = r.z + z;
    return res;
    
}

math::quarternion math::quarternion::conjugate(const math::quarternion& q){
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
    
}

float math::length(const math::quarternion& q){
    return sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
}

float math::length(const math::vector& v){
    return sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
}

math::quarternion math::quarternion::inverse(const math::quarternion& q){
    float len;
    if(q.unit) len = 1;
    else len = length(q);
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}
math::quarternion math::quarternion::rotate(float theta, const math::vector& axis){
    quarternion res;
    res.w = cos(theta/2);
    res.x = axis.x*sin(theta/2);
    res.y = axis.y*sin(theta/2);
    res.z = axis.z*sin(theta/2);
    return res;
}

math::quarternion math::quarternion::from_euler_ZYX(const math::vector& euler){

    float cy = cos(euler.z*0.5);
    float sy = sin(euler.z*0.5);
    float cp = cos(euler.y*0.5);
    float sp = sin(euler.y*0.5);
    float cr = cos(euler.x*0.5);
    float sr = sin(euler.x*0.5);

    quarternion res;
    res.w = cr*cp*cy + sr*sp*sy;
    res.x = sr*cp*cy - cr*sp*sy;
    res.y = cr*sp*cy + sr*cp*sy;
    res.z = cr*cp*sy - sr*sp*cy;
    return res;
}

math::vector math::quarternion::to_euler(const math::quarternion& q){
    float  sin_p = 2*(q.w*q.y - q.z*q.x);
    if(sin_p >=1){
    vector res(-2*atan2(q.x, q.w), 1.570796326794897, 0);
    return res;
    }else if(sin_p <= -1){
    vector res(2 *atan2(q.x, q.w), -1.570796326794897,  0);
    return res;
    }
    //asin(2*(q.w*q.y - q.z*q.x));
    vector res(atan2(2*(q.w*q.x+q.y*q.z), 1 - 2* (q.x*q.x + q.y*q.y)), asin(sin_p), atan2(2*(q.w*q.z+q.x*q.y), 1 - 2 * (q.y*q.y+q.z*q.z)));
    return res;
}
/*



    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;



*/
math::vector math::quarternion::to_magnitude_axis(const math::quarternion& q){
    float s = 1 - q.w*q.w;
    float mag = acos(q.w)*2;

    if(q.w > -0.00001 && q.w < 0.00001){
        vector res(0,0,0);
        return res;
    }
    vector res(q.x*mag/s, q.y*mag/s,q.z*mag/s);
    return res;
}


math::vector math::quarternion::rotate_vector(math::quarternion& q, math::vector& r){
    quarternion temp(0, r.x, r.y, r.z);
    temp = q * temp * quarternion::conjugate(q);
    vector out = math::vector(temp.x, temp.y, temp.z);
    return out;
}