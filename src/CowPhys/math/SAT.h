#ifndef COWPHYS_SAT_H
#define COWPHYS_SAT_H

#include <vector>
#include "CowPhys/CowPhys.h"
#include "Vec3.h"
#include "Box.h"

namespace cp {

struct SATInfo {
    bool isColliding;
    Vec3d normal;
    double depth;
    Vec3d resolution;
    Vec3d contact;
};

class SAT {
public:

    static SATInfo satNoCollision() {
        SATInfo info;
        info.isColliding = false;
        return info;
    }


    static SATInfo SATCollision(Boxd &boxA, Boxd &boxB) {
        auto verticesA = boxA.getVertices();
        auto verticesB = boxB.getVertices();

        std::vector<Vec3d> axes = {
                boxA.rotation.rotate({1, 0, 0}).normalize(),
                boxA.rotation.rotate({0, 1, 0}).normalize(),
                boxA.rotation.rotate({0, 0, 1}).normalize(),
                boxB.rotation.rotate({1, 0, 0}).normalize(),
                boxB.rotation.rotate({0, 1, 0}).normalize(),
                boxB.rotation.rotate({0, 0, 1}).normalize()
        };

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Vec3d crossAxis = axes[i].cross(axes[j + 3]);
                if (crossAxis.length() > 1e-8) {
                    axes.push_back(crossAxis.normalize());
                }
            }
        }

        double minPenetrationDepth = std::numeric_limits<double>::max();
        Vec3d smallestAxis;

        for (auto &axis: axes) {
            double minA, maxA, minB, maxB;
            if (!overlapOnAxis(verticesA, verticesB, axis, minA, maxA, minB, maxB)) {
                return {false, {}, 0.0, {}, {}};
            }

            double overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minPenetrationDepth) {
                minPenetrationDepth = overlap;
                smallestAxis = axis;
            }
        }

        Vec3d resolution = smallestAxis * minPenetrationDepth;
        Vec3d contact = computeContactPoint(boxA, boxB, smallestAxis, minPenetrationDepth);
        return {true, smallestAxis, minPenetrationDepth, resolution, contact};
    }

private:
    static bool
    overlapOnAxis(const std::array<Vec3d, 8> &verticesA, const std::array<Vec3d, 8> &verticesB, const Vec3d &axis,
                  double &minA, double &maxA, double &minB, double &maxB) {
        auto project = [](const Vec3d &v, const Vec3d &axis) {
            return v.dot(axis);
        };

        minA = maxA = project(verticesA[0], axis);
        for (const auto &vertex: verticesA) {
            double projection = project(vertex, axis);
            if (projection < minA) minA = projection;
            if (projection > maxA) maxA = projection;
        }

        minB = maxB = project(verticesB[0], axis);
        for (const auto &vertex: verticesB) {
            double projection = project(vertex, axis);
            if (projection < minB) minB = projection;
            if (projection > maxB) maxB = projection;
        }

        return !(maxA < minB || maxB < minA);
    }

    static Vec3d computeContactPoint(Boxd &boxA, Boxd &boxB, const Vec3d &axis, double depth) {
        // Placeholder for a more accurate contact point calculation
        Vec3d centerA = boxA.pos;
        Vec3d centerB = boxB.pos;
        Vec3d midpoint = (centerA + centerB) / 2;
        return midpoint + axis * depth / 2;
    }
};

}

#endif //COWPHYS_SAT_H
