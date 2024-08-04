#ifndef COWPHYS_MESHSHAPE_H
#define COWPHYS_MESHSHAPE_H

#include <utility>
#include <vector>
#include "Shape.h"
#include "CowPhys/math/Triangle.h"

namespace cp {

class MeshShape : public Shape {

public:

    MeshShape() {}

    MeshShape(std::vector<TriangleU> triangles) : mTriangles(std::move(triangles)) {
        toCounterWise();
    }

    const std::vector<TriangleU> &getTriangles() {
        return mTriangles;
    }

private:

    void toCounterWise() {
        for (auto &triangle: mTriangles) {
            if (isClockwise(triangle)) {
                std::swap(triangle.p1, triangle.p2);
            }
        }
    }

    bool isClockwise(const TriangleU &triangle) const {
        const auto &a = triangle.p0;
        const auto &b = triangle.p1;
        const auto &c = triangle.p2;
        auto ab = b - a;
        auto ac = c - a;
        auto crossProduct = ab.cross(ac);
        return crossProduct.z > 0;
    }

    std::vector<TriangleU> mTriangles;

};


}

#endif //COWPHYS_MESHSHAPE_H
