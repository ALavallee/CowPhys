#ifndef COWPHYS_QUAT_H
#define COWPHYS_QUAT_H

#include <cmath>
#include "Vec3.h"

namespace cp {

template<class T>
struct Quat {

    T x, y, z, w;

    static Quat<T> fromEuler(T xRad, T yRad, T zRad) {
        T cx = std::cos(xRad * 0.5f);
        T sx = std::sin(xRad * 0.5f);
        T cy = std::cos(yRad * 0.5f);
        T sy = std::sin(yRad * 0.5f);
        T cz = std::cos(zRad * 0.5f);
        T sz = std::sin(zRad * 0.5f);

        T qw = cx * cy * cz + sx * sy * sz;
        T qx = sx * cy * cz - cx * sy * sz;
        T qy = cx * sy * cz + sx * cy * sz;
        T qz = cx * cy * sz - sx * sy * cz;

        return {qx, qy, qz, qw};
    }

    static Quat<T> fromEuler(Vec3<T> rad) {
        return fromEuler(rad.x, rad.y, rad.z);
    }

    static Quat<T> identity() {
        return {0, 0, 0, 1};
    }

    Quat() : x(0), y(0), z(0), w(0) {

    }

    Quat(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {

    }

    Quat operator+(const Quat &other) const {
        return {x + other.x, y + other.y, z + other.z, w + other.w};
    }

    Quat operator*(const Quat &other) const {
        return {
                w * other.x + x * other.w + y * other.z - z * other.y,
                w * other.y - x * other.z + y * other.w + z * other.x,
                w * other.z + x * other.y - y * other.x + z * other.w,
                w * other.w - x * other.x - y * other.y - z * other.z
        };
    }

    Quat operator*(T v) {
        return {x * v, y * v, z * v, w * v};
    }

    Vec3d rotate(const Vec3d &v) const {
        // Quaternion rotation implementation
        Vec3d qVec(x, y, z);
        Vec3d uv = qVec.cross(v);
        Vec3d uuv = qVec.cross(uv);
        uv = uv * (2.0 * w);
        uuv = uuv * 2.0;
        return v + uv + uuv;
    }

    Quat<T> normalize() {
        Quat<T> result = *this;
        T len = std::sqrt(x * x + y * y + z * z + w * w);
        if (len > 0.0) {
            result.x /= len;
            result.y /= len;
            result.z /= len;
            result.w /= len;
        }
        return result;
    }

    T dot(const Quat<T> &b) const {
        return x * b.x + y * b.y + z * b.z + w * b.w;
    }

    T norm() const {
        return std::sqrt(dot(*this));
    }

    template<class C>
    Quat<C> to() {
        return {static_cast<C>(x), static_cast<C>(y), static_cast<C>(z), static_cast<C>(w)};
    }

    std::string toStr() {
        return "( " + std::to_string(x) + " , " + std::to_string(y) +
               " , " + std::to_string(z) + " , " + std::to_string(w) + " )";
    }

};

typedef Quat<float> Quatf;
typedef Quat<double> Quatd;

}

#endif //COWPHYS_QUAT_H
