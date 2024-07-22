#ifndef COWPHYS_VEC3_H
#define COWPHYS_VEC3_H

#include <cmath>
#include <string>

namespace cp {

template<class T>
class Vec3 {
public:
    constexpr Vec3() : x(0), y(0), z(0) {

    }

    constexpr explicit Vec3(T v) : x(v), y(v), z(v) {

    }

    constexpr Vec3(T x, T y, T z) : x(x), y(y), z(z) {

    }

    bool isZero() {
        return *this == Vec3<T>();
    }

    Vec3 operator+(const Vec3 &rhs) const {
        return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    Vec3 operator-(const Vec3 &rhs) const {
        return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    Vec3 operator*(const Vec3 &rhs) const {
        return Vec3(x * rhs.x, y * rhs.y, z * rhs.z);
    }

    Vec3 operator*(T scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    Vec3 operator/(T scalar) const {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3 operator/(Vec3<T> scalar) const {
        return Vec3(x / scalar.x, y / scalar.y, z / scalar.z);
    }

    bool operator==(const Vec3 &rhs) const {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    bool operator!=(const Vec3 &rhs) const {
        return (*this == rhs) == false;
    }

    Vec3 operator-() const {
        return Vec3(-x, -y, -z);
    }

    T distance(const Vec3 &rhs) const {
        return std::sqrt(std::pow(x - rhs.x, 2) + std::pow(y - rhs.y, 2) + std::pow(z - rhs.z, 2));
    }

    T length() const {
        return static_cast<T>(std::sqrt(static_cast<T>(x) * static_cast<T>(x) +
                                        static_cast<T>(y) * static_cast<T>(y) +
                                        static_cast<T>(z) * static_cast<T>(z)));
    }

    T lengthSquared() const {
        return x * x + y * y + z * z;
    }

    // Linear interpolation between this vector and another vector
    Vec3<T> lerp(const Vec3<T> &rhs, T t) const {
        return Vec3<T>(x + (rhs.x - x) * t, y + (rhs.y - y) * t, z + (rhs.z - z) * t);
    }

    Vec3 normalize() const {
        T length = std::sqrt(x * x + y * y + z * z);
        if (length > 0.0) {
            return Vec3(x / length, y / length, z / length);
        } else {
            // Return the original vector if the length is 0 to avoid division by zero
            return *this;
        }
    }

    Vec3<T> cross(const Vec3 &rhs) const {
        return Vec3(y * rhs.z - z * rhs.y,
                    z * rhs.x - x * rhs.z,
                    x * rhs.y - y * rhs.x);
    }

    T dot(const Vec3 &rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    Vec3<T> align(T alignTo) {
        Vec3<T> result;
        result.x = std::round(x * alignTo) / alignTo;
        result.y = std::round(y * alignTo) / alignTo;
        result.z = std::round(z * alignTo) / alignTo;
        return result;
    }

    Vec3<T> min(const Vec3<T> &rhs) const {
        return Vec3<T>(std::min(x, rhs.x), std::min(y, rhs.y), std::min(z, rhs.z));
    }

    Vec3<T> max(const Vec3<T> &rhs) const {
        return Vec3<T>(std::max(x, rhs.x), std::max(y, rhs.y), std::max(z, rhs.z));
    }

    template<class A>
    Vec3<A> to() {
        return Vec3<A>(static_cast<A>(x), static_cast<A>(y), static_cast<A>(z));
    }

    std::string toStr() {
        return "( " + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + " )";
    }

    T operator[](int index) {
        switch (index) {
            case 0:
                return x;
            case 1 :
                return y;
            default:
                return z;
        }
    }

    T x;
    T y;
    T z;
};

typedef Vec3<int> Vec3i;
typedef Vec3<float> Vec3f;
typedef Vec3<double> Vec3d;

}

#endif //COWPHYS_VEC3_H
