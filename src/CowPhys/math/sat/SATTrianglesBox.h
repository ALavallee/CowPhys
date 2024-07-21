#ifndef COWPHYS_SATTRIANGLESBOX_H
#define COWPHYS_SATTRIANGLESBOX_H

#include <vector>
#include "SatInfo.h"
#include "CowPhys/math/Triangle.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/Box.h"


namespace cp {

class SATTrianglesBox {

public:
    static SATInfo sat(const std::vector<Triangle<double>> &triangles, Vec3d trianglesPos,
                       Quatf trianglesRotation, Boxd &box) {
        auto verticesBox = box.getVertices();
        std::vector<Vec3d> axes;

        // Add Box face normals to axes
        for (const auto &dir: {Vec3d{1, 0, 0}, Vec3d{0, 1, 0}, Vec3d{0, 0, 1}}) {
            axes.push_back(box.rotation.rotate(dir).normalize());
        }

        // Add Triangle normals and edges to axes
        for (const auto &triangle: triangles) {
            Vec3d p0 = trianglesRotation.rotate(triangle.p0) + trianglesPos;
            Vec3d p1 = trianglesRotation.rotate(triangle.p1) + trianglesPos;
            Vec3d p2 = trianglesRotation.rotate(triangle.p2) + trianglesPos;

            // Calculate the normal vector
            Vec3d normal = (p1 - p0).cross(p2 - p0).normalize();

            // Check winding order and flip normal if necessary
            if ((p1 - p0).cross(p2 - p0).dot(normal) < 0) {
                normal = -normal;
            }

            axes.push_back(normal);
            for (const auto &edge: {(p1 - p0), (p2 - p1), (p0 - p2)}) {
                axes.push_back(edge.normalize());
            }

            // Cross products of box edges and triangle edges
            for (const auto &boxEdge: {box.rotation.rotate({1, 0, 0}), box.rotation.rotate({0, 1, 0}),
                                       box.rotation.rotate({0, 0, 1})}) {
                for (const auto &triEdge: {(p1 - p0).normalize(), (p2 - p1).normalize(), (p0 - p2).normalize()}) {
                    Vec3d crossAxis = boxEdge.cross(triEdge);
                    if (crossAxis.length() > 1e-8) {
                        axes.push_back(crossAxis.normalize());
                    }
                }
            }
        }

        double minPenetrationDepth = std::numeric_limits<double>::max();
        Vec3d smallestAxis;
        for (const auto &axis: axes) {
            double minA, maxA, minB, maxB;
            if (!overlapOnAxis(verticesBox, triangles, axis, trianglesPos, trianglesRotation, minA, maxA, minB, maxB)) {
                return SATInfo::noCollision();
            }

            double overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < minPenetrationDepth) {
                minPenetrationDepth = overlap;
                smallestAxis = axis;
            }
        }

        // Ensure smallestAxis points from box to triangle
        Vec3d boxCenter = box.pos;
        Vec3d triangleCenter = trianglesPos; // Assuming the average position of triangle vertices
        if ((triangleCenter - boxCenter).dot(smallestAxis) < 0) {
            smallestAxis = -smallestAxis;
        }

        Vec3d resolution = smallestAxis * minPenetrationDepth;
        Vec3d contact = computeContactPoint(verticesBox, triangles, smallestAxis, minPenetrationDepth);
        return {true, smallestAxis, minPenetrationDepth, resolution, contact};
    }

private:

    static bool
    overlapOnAxis(const std::array<Vec3d, 8> &verticesBox, const std::vector<Triangle<double>> &triangles,
                  const Vec3d &axis, Vec3d trianglesPos, Quatf trianglesRotation,
                  double &minBox, double &maxBox, double &minTri, double &maxTri) {
        auto project = [](const Vec3d &v, const Vec3d &axis) {
            return v.dot(axis);
        };

        minBox = maxBox = project(verticesBox[0], axis);
        for (const auto &vertex: verticesBox) {
            double projection = project(vertex, axis);
            if (projection < minBox) minBox = projection;
            if (projection > maxBox) maxBox = projection;
        }

        bool first = true;
        for (const auto &triangle: triangles) {
            std::array<Vec3d, 3> verticesTri = {triangle.p0, triangle.p1, triangle.p2};
            for (auto vertex: verticesTri) {
                vertex = trianglesRotation.rotate(vertex) + trianglesPos;
                double projection = project(vertex, axis);
                if (first) {
                    minTri = maxTri = projection;
                    first = false;
                } else {
                    if (projection < minTri) minTri = projection;
                    if (projection > maxTri) maxTri = projection;
                }
            }
        }

        return !(maxBox < minTri || maxTri < minBox);
    }

    static Vec3d
    computeContactPoint(const std::array<Vec3d, 8> &verticesBox, const std::vector<Triangle<double>> &triangles,
                        const Vec3d &axis, double depth) {
        // Placeholder for a more accurate contact point calculation
        Vec3d midpoint(0, 0, 0);
        for (const auto &vertex: verticesBox) {
            midpoint = midpoint + vertex;
        }
        midpoint = midpoint / static_cast<double>(verticesBox.size());

        for (const auto &triangle: triangles) {
            midpoint = midpoint + (triangle.p0 + triangle.p1 + triangle.p2) / 3.0;
        }
        midpoint = midpoint / static_cast<double>((triangles.size() + 1));

        return midpoint + axis * depth / 2;
    }
};

};

#endif //COWPHYS_SATTRIANGLESBOX_H
