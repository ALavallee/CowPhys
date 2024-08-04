#ifndef COWPHYS_SPHERE_H
#define COWPHYS_SPHERE_H

#include "Vec3.h"

namespace cp {

template<class T>
class Sphere {

public:

    Sphere() : mPosition(), mRadius(0) {

    }

    Sphere(Vec3<T> pos, T radius) : mPosition(pos), mRadius(radius) {

    }

    bool collides(const Sphere<T> &other) {
        return mPosition.distance(other.mPosition) < mRadius + other.mRadius;
    }

    T penetration(const Sphere<T> &other) {
        auto distance = mPosition.distance(other.mPosition);
        return std::abs(distance - (mRadius + other.mRadius));
    }

    void rotateBy(const Vec3<T> &euler) {
        mPosition.rotate(euler);
    }

    void moveBy(const Vec3<T> &by) {
        mPosition = mPosition + by;
    }

    Vec3<T> getPosition() {
        return mPosition;
    }

    T getRadius() {
        return mRadius;
    }

    bool raycast(const Vec3<T> &origin, const Vec3<T> &dir, T &t) const {
        Vec3<T> oc = origin - mPosition;
        T a = dir.dot(dir);
        T b = 2 * oc.dot(dir);
        T c = oc.dot(oc) - mRadius * mRadius;
        T discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return false;
        } else {
            t = (-b - std::sqrt(discriminant)) / (2.0 * a);
            return true;
        }
    }

private:

    Vec3<T> mPosition;
    T mRadius;


};

typedef Sphere<Unit> SphereU;


}

#endif //COWPHYS_SPHERE_H
