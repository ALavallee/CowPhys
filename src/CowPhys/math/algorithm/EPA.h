#ifndef COWPHYS_EPA_H
#define COWPHYS_EPA_H

#include <vector>
#include <algorithm>
#include <limits>
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Vec4.h"
#include "CowPhys/math/Simplex.h"

namespace cp {

struct CollisionPoint {
    bool collides;
    double depth;
    Vec3d normal;
    Vec3d contactPoint;
    Vec3d resolution;
};

class EPA {

public:
    static CollisionPoint
    collisionPoint(Simplex &simplex, std::vector<Vec3d> &verticesA, std::vector<Vec3d> &verticesB,
                   int maxIterations = 100) {
        std::vector<Vec3d> polytope(simplex.begin(), simplex.end());
        std::vector<size_t> faces = {
                0, 1, 2,
                0, 3, 1,
                0, 2, 3,
                1, 3, 2
        };

        auto [normals, minFace] = getFaceNormals(polytope, faces);

        Vec3d minNormal;
        double minDistance = std::numeric_limits<double>::max();
        int iterations = 0;

        while (iterations < maxIterations) {
            minNormal = normals[minFace].xyz();
            minDistance = normals[minFace].w;

            Vec3d support = sup(verticesA, verticesB, minNormal);
            double sDistance = minNormal.dot(support);

            if (std::abs(sDistance - minDistance) > 0.001f) {
                minDistance = std::numeric_limits<double>::max();
                std::vector<std::pair<size_t, size_t>> uniqueEdges;

                for (size_t i = 0; i < normals.size(); i++) {
                    if (sameDirection(normals[i], support)) {
                        size_t f = i * 3;

                        addIfUniqueEdge(uniqueEdges, faces, f, f + 1);
                        addIfUniqueEdge(uniqueEdges, faces, f + 1, f + 2);
                        addIfUniqueEdge(uniqueEdges, faces, f + 2, f);

                        faces[f + 2] = faces.back();
                        faces.pop_back();
                        faces[f + 1] = faces.back();
                        faces.pop_back();
                        faces[f] = faces.back();
                        faces.pop_back();

                        normals[i] = normals.back();
                        normals.pop_back();

                        i--;
                    }
                }

                std::vector<size_t> newFaces;
                for (auto [edgeIndex1, edgeIndex2]: uniqueEdges) {
                    newFaces.push_back(edgeIndex1);
                    newFaces.push_back(edgeIndex2);
                    newFaces.push_back(polytope.size());
                }

                polytope.push_back(support);

                auto [newNormals, newMinFace] = getFaceNormals(polytope, newFaces);
                double oldMinDistance = std::numeric_limits<double>::max();
                for (size_t i = 0; i < normals.size(); i++) {
                    if (normals[i].w < oldMinDistance) {
                        oldMinDistance = normals[i].w;
                        minFace = i;
                    }
                }

                if (newNormals[newMinFace].w < oldMinDistance) {
                    minFace = newMinFace + normals.size();
                }

                faces.insert(faces.end(), newFaces.begin(), newFaces.end());
                normals.insert(normals.end(), newNormals.begin(), newNormals.end());
            } else {
                break;
            }

            iterations++;
        }

        CollisionPoint points;
        if (iterations == maxIterations) {
            points.collides = false;
        } else {
            points.normal = minNormal;
            points.depth = minDistance + 0.001f;
            points.collides = true;

            // Compute contact point
            Vec3d contactPoint = Vec3d(0.0, 0.0, 0.0);
            size_t faceIndex = minFace * 3;
            for (size_t i = 0; i < 3; ++i) {
                contactPoint = contactPoint + polytope[faces[faceIndex + i]];
            }
            contactPoint = contactPoint / 3.0;
            points.contactPoint = contactPoint;

            // Compute resolution vector
            points.resolution = minNormal * points.depth;
        }

        return points;
    }

private:

    static std::pair<std::vector<Vec4d>, size_t> getFaceNormals(
            const std::vector<Vec3d> &polytope,
            const std::vector<size_t> &faces) {
        std::vector<Vec4d> normals;
        size_t minTriangle = 0;
        double minDistance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < faces.size(); i += 3) {
            Vec3d a = polytope[faces[i]];
            Vec3d b = polytope[faces[i + 1]];
            Vec3d c = polytope[faces[i + 2]];

            Vec3d normal = (b - a).cross(c - a).normalize();
            double distance = normal.dot(a);

            if (distance < 0) {
                normal = normal * -1;
                distance *= -1;
            }

            normals.emplace_back(normal, distance);

            if (distance < minDistance) {
                minTriangle = i / 3;
                minDistance = distance;
            }
        }

        return {normals, minTriangle};
    }

    static void
    addIfUniqueEdge(std::vector<std::pair<size_t, size_t>> &edges, const std::vector<size_t> &faces, size_t a,
                    size_t b) {
        auto reverse = std::find(
                edges.begin(),
                edges.end(),
                std::make_pair(faces[b], faces[a])
        );

        if (reverse != edges.end()) {
            edges.erase(reverse);
        } else {
            edges.emplace_back(faces[a], faces[b]);
        }
    }

    static Vec3d sup(std::vector<Vec3d> &verticeA, std::vector<Vec3d> &verticesB, Vec3d direction) {
        return findFurthestPoint(verticeA, direction) - findFurthestPoint(verticesB, -direction);
    }

    static Vec3d findFurthestPoint(std::vector<Vec3d> &vertices, Vec3d direction) {
        Vec3d maxPoint;
        double maxDistance = -std::numeric_limits<double>::max();

        for (auto vertex: vertices) {
            double distance = vertex.dot(direction);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxPoint = vertex;
            }
        }

        return maxPoint;
    }

    static bool sameDirection(const Vec4d &direction, Vec3d &ao) {
        return direction.dot(Vec4d(ao, 0)) > 0.0;
    }

};

}

#endif //COWPHYS_EPA_H
