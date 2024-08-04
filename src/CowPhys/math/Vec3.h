#ifndef COWPHYS_VEC3_H
#define COWPHYS_VEC3_H

#include <cmath>
#include <string>
#include <numeric>
#include "Unit.h"

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

    Vec3 operator%(T scalar) const {
        return Vec3(x % scalar, y % scalar, z % scalar);
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

    Vec3 normalize() const {
        int maxComponent = std::max(std::abs(x), std::max(std::abs(y), std::abs(z)));
        if (maxComponent == 0) {
            return {0, 0, 0}; // Handle the zero vector case
        }
        return {x / maxComponent, y / maxComponent, z / maxComponent};
    }

    Vec3<T> cross(const Vec3 &rhs) const {
        return Vec3(y * rhs.z - z * rhs.y,
                    z * rhs.x - x * rhs.z,
                    x * rhs.y - y * rhs.x);
    }

    T dot(const Vec3 &rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    void rotate(const Vec3 &euler) {
        // Convert Euler angles from [0, 512] range to radians [0, 2*pi]
        constexpr double factor = 2.0 * M_PI / 512.0;
        double radX = euler.x * factor;
        double radY = euler.y * factor;
        double radZ = euler.z * factor;

        double cx = std::cos(radX);
        double sx = std::sin(radX);
        double cy = std::cos(radY);
        double sy = std::sin(radY);
        double cz = std::cos(radZ);
        double sz = std::sin(radZ);

        // Rotation matrices for each axis
        double tempX = x;
        double tempY = y * cx - z * sx;
        double tempZ = y * sx + z * cx;

        double tempX2 = tempX * cy + tempZ * sy;
        double tempY2 = tempY;
        double tempZ2 = -tempX * sy + tempZ * cy;

        double newX = tempX2 * cz - tempY2 * sz;
        double newY = tempX2 * sz + tempY2 * cz;
        double newZ = tempZ2;

        // Assign rotated values back to integers
        x = static_cast<T>(std::round(newX));
        y = static_cast<T>(std::round(newY));
        z = static_cast<T>(std::round(newZ));
    }

    Vec3<T> round() {
        return {std::round(x), std::round(y), std::round(z)};
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

    T &operator[](int index) {
        switch (index) {
            case 0:
                return x;
            case 1:
                return y;
            default:
                return z;
        }
    }

    T x;
    T y;
    T z;
};

typedef Vec3<Unit> Vec3U;
typedef Vec3<SmallUnit> Vec3Small;
typedef Vec3<TinyUnit> Vec3Tiny;
typedef Vec3<MicroUnit> Vec3Micro;

}

#endif //COWPHYS_VEC3_H
