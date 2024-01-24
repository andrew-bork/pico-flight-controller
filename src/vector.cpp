#include <math/vector.hpp>

math::vector::vector (){
    x = y = z = 0;
}
math::vector::vector (float xa){
    x = xa;
    y = z = 0;
}
math::vector::vector (float xa, float ya){
    x = xa;
    y = ya;
    z = 0;
}
math::vector::vector (float xa, float ya, float za){
    x = xa;
    y = ya;
    z = za;
}

math::vector math::vector::operator+ (const math::vector& r) const{
    vector res(x+r.x,y+r.y,z+r.z);
    return res;
}
math::vector math::vector::operator- (const math::vector& r) const{
    vector res(x-r.x,y-r.y,z-r.z);
    return res;
}

math::vector math::vector::operator*(const float& s) const{
    vector res(x*s, y*s, z*s);
    return res;
}