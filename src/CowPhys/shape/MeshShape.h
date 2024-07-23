#ifndef COWPHYS_MESHSHAPE_H
#define COWPHYS_MESHSHAPE_H

#include <utility>
#include <vector>
#include "Shape.h"
#include "CowPhys/math/Triangle.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/sat/SatInfo.h"

namespace cp {

class MeshShape : public Shape {

public:

    MeshShape() {}

    MeshShape(std::vector<Triangle<double>> triangles) : mTriangles(std::move(triangles)) {
        toCounterWise();
    }

    void addTriangle(Triangle<double> triangle) {
        mTriangles.emplace_back(triangle);
    }

    SATInfo collision(Vec3d pos, Quatf rotation, Shape *other, Vec3d otherPos, Quatf otherRot) override;


    const std::vector<Triangle<double>> &getTriangles() {
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

    bool isClockwise(const Triangle<double> &triangle) const {
        const auto &a = triangle.p0;
        const auto &b = triangle.p1;
        const auto &c = triangle.p2;
        auto ab = b - a;
        auto ac = c - a;
        auto crossProduct = ab.cross(ac);
        return crossProduct.z > 0.0;
    }

    std::vector<Triangle<double>> mTriangles;

};


}

#endif //COWPHYS_MESHSHAPE_H
