#ifndef COWPHYS_VEC4_H
#define COWPHYS_VEC4_H

#include "Vec3.h"

namespace cp {

template<class T>
class Vec4 {

public:

    Vec4() : x(0), y(0), z(0), w(0) {

    }

    Vec4(T v) : x(v), y(v), z(v), w(v) {

    }

    Vec4(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {

    }

    Vec4(Vec3<T> v, T w) : x(v.x), y(v.y), z(v.z), w(w) {

    }

    Vec3<T> xyz() {
        return {x, y, z};
    }

    T dot(const Vec4 &other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    T x, y, z, w;

};

typedef Vec4<double> Vec4d;
typedef Vec4<float> Vec4f;

}

#endif //COWPHYS_VEC4_H
