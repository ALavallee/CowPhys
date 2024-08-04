#ifndef COWPHYS_TRIANGLE_H
#define COWPHYS_TRIANGLE_H

#include <array>
#include "Vec3.h"

namespace cp {

template<class T>
class Triangle {

public:
    Triangle() = default;

    Triangle(Vec3<T> p0, Vec3<T> p1, Vec3<T> p2) : p0(p0), p1(p1), p2(p2) {}

    std::array<Vec3<T>, 3> getVertices() {
        return {p0, p1, p2};
    }

    Vec3<T> p0;
    Vec3<T> p1;
    Vec3<T> p2;

};

typedef Triangle<Unit> TriangleU;

}

#endif //OESERVER_TRIANGLE_H
