#ifndef COWPHYS_GJK_H
#define COWPHYS_GJK_H

#include <vector>
#include <limits>
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Triangle.h"
#include "CowPhys/math/Simplex.h"

namespace cp {


class GJK {

public:

    static bool gjk(std::vector<Vec3d> &verticesA, std::vector<Vec3d> &verticesB) {
        Vec3d support = sup(verticesA, verticesB, Vec3d(1, 0, 0));

        Simplex points;
        points.push_front(support);
        Vec3d direction = -support;

        while (true) {
            support = sup(verticesA, verticesB, direction);

            if (support.dot(direction) <= 0) {
                return false; // no collision
            }

            points.push_front(support);

            if (nextSimplex(points, direction)) {
                return true;
            }
        }
    }

    static Vec3d sup(std::vector<Vec3d> &verticeA, std::vector<Vec3d> &verticesB, Vec3d direction) {
        return findFurthestPoint(verticeA, direction) - findFurthestPoint(verticesB, -direction);
    }

    static Vec3d findFurthestPoint(std::vector<Vec3d> &vertices, Vec3d direction) {
        Vec3d maxPoint;
        double maxDistance = std::numeric_limits<double>::max();

        for (auto vertex: vertices) {
            double distance = vertex.dot(direction);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxPoint = vertex;
            }
        }

        return maxPoint;
    }


private:

    static bool nextSimplex(Simplex &points, Vec3d &direction) {
        switch (points.size()) {
            case 2:
                return line(points, direction);
            case 3:
                return triangle(points, direction);
            case 4:
                return tetrahedron(points, direction);
        }

        // never should be here
        return false;
    }

    static bool sameDirection(const Vec3d &direction, Vec3d &ao) {
        return direction.dot(ao) > 0.0;
    }

    static bool line(Simplex &points, Vec3d &direction) {
        Vec3d a = points[0];
        Vec3d b = points[1];

        Vec3d ab = b - a;
        Vec3d ao = -a;

        if (sameDirection(ab, ao)) {
            direction = ab.cross(ao).cross(ab);
        } else {
            points = {a};
            direction = ao;
        }

        return false;
    }

    static bool triangle(Simplex &points, Vec3d &direction) {
        Vec3d a = points[0];
        Vec3d b = points[1];
        Vec3d c = points[2];

        Vec3d ab = b - a;
        Vec3d ac = c - a;
        Vec3d ao = -a;

        Vec3d abc = ab.cross(ac);

        if (sameDirection(abc.cross(ac), ao)) {
            if (sameDirection(ac, ao)) {
                points = {a, c};
                direction = ac.cross(ao).cross(ac);
            } else {
                return line(points = {a, b}, direction);
            }
        } else {
            if (sameDirection(ab.cross(abc), ao)) {
                return line(points = {a, b}, direction);
            } else {
                if (sameDirection(abc, ao)) {
                    direction = abc;
                } else {
                    points = {a, c, b};
                    direction = -abc;
                }
            }
        }

        return false;
    }

    static bool tetrahedron(Simplex &points, Vec3d &direction) {
        Vec3d a = points[0];
        Vec3d b = points[1];
        Vec3d c = points[2];
        Vec3d d = points[3];

        Vec3d ab = b - a;
        Vec3d ac = c - a;
        Vec3d ad = d - a;
        Vec3d ao = -a;

        Vec3d abc = ab.cross(ac);
        Vec3d acd = ac.cross(ad);
        Vec3d adb = ad.cross(ab);

        if (sameDirection(abc, ao)) {
            return triangle(points = {a, b, c}, direction);
        }

        if (sameDirection(acd, ao)) {
            return triangle(points = {a, c, d}, direction);
        }

        if (sameDirection(adb, ao)) {
            return triangle(points = {a, d, b}, direction);
        }

        return true;
    }

};


}

#endif //COWPHYS_GJK_H
