#ifndef COWPHYS_SATTRIBOX_H
#define COWPHYS_SATTRIBOX_H

#include <vector>
#include <array>
#include <limits>
#include <algorithm>
#include "SatInfo.h"
#include "CowPhys/math/Triangle.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/Box.h"

namespace cp {

class SATTriBox {

public:

    static SATInfo sat(const std::vector<Triangle<double>> &triangles, Vec3d pos, Quatf rot, Boxd &box) {

        for (auto triangle: triangles) {
            auto poop = Triangle<double>(rot.rotate(triangle.p0) + pos,
                                         rot.rotate(triangle.p1) + pos,
                                         rot.rotate(triangle.p2) + pos);

            auto triSat = sat(poop, box);
            if (triSat.isColliding) {
                return triSat;
            }
        }

        return SATInfo::noCollision();
    }


    static SATInfo sat(Triangle<double> &triangle, Boxd &box) {

        std::vector<Vec3d> axes;

        // Box axes
        Vec3d boxAxes[3] = {
                box.rotation.rotate(Vec3d(1, 0, 0)),
                box.rotation.rotate(Vec3d(0, 1, 0)),
                box.rotation.rotate(Vec3d(0, 0, 1))
        };

        // Triangle axes
        Vec3d triangleEdges[3] = {
                triangle.p1 - triangle.p0,
                triangle.p2 - triangle.p1,
                triangle.p0 - triangle.p2
        };

        // Add face normals of the box
        axes.push_back(boxAxes[0]);
        axes.push_back(boxAxes[1]);
        axes.push_back(boxAxes[2]);

        // Add face normal of the triangle
        Vec3d triangleNormal = (triangleEdges[0].cross(triangleEdges[1])).normalize();
        axes.push_back(triangleNormal);

        // Add edge cross products
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                axes.push_back(boxAxes[i].cross(triangleEdges[j]).normalize());
            }
        }

        double minOverlap = std::numeric_limits<double>::max();
        Vec3d smallestAxis;

        // Check all axes for overlap
        for (auto &axis: axes) {
            if (axis.isZero())
                continue;

            auto [boxMin, boxMax] = projectBox(box, axis);
            auto [triMin, triMax] = projectTriangle(triangle, axis);

            double overlap = getOverlap(boxMin, boxMax, triMin, triMax);
            if (overlap <= 0) {
                return SATInfo::noCollision();
            }

            if (overlap < minOverlap) {
                minOverlap = overlap;
                smallestAxis = axis;
            }
        }

        SATInfo result;
        result.isColliding = true;
        result.depth = minOverlap;
        result.normal = smallestAxis;
        return result;
    }

private:
    static std::pair<double, double> projectBox(Boxd &box, const Vec3d &axis) {
        std::array<Vec3d, 8> vertices = box.getVertices();
        double min = vertices[0].dot(axis);
        double max = min;

        for (const auto &vertex: vertices) {
            double projection = vertex.dot(axis);
            if (projection < min) {
                min = projection;
            }
            if (projection > max) {
                max = projection;
            }
        }

        return {min, max};
    }

    static std::pair<double, double> projectTriangle(const Triangle<double> &triangle, const Vec3d &axis) {
        std::array<double, 3> projections = {
                triangle.p0.dot(axis),
                triangle.p1.dot(axis),
                triangle.p2.dot(axis)
        };

        double min = *std::min_element(projections.begin(), projections.end());
        double max = *std::max_element(projections.begin(), projections.end());

        return {min, max};
    }

    static double getOverlap(double minA, double maxA, double minB, double maxB) {
        const double epsilon = 1e-6; // a small tolerance
        return std::max(0.0, std::min(maxA, maxB) - std::max(minA, minB) + epsilon);
    }
};

};

#endif //COWPHYS_SATTRIBOX_H
