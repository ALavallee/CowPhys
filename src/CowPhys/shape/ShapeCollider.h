#ifndef COWPHYS_SHAPECOLLIDER_H
#define COWPHYS_SHAPECOLLIDER_H

#include <limits>
#include "CowPhys/math/Vec3.h"
#include "Shape.h"

namespace cp {

struct ShapeToCollide {

    ShapeToCollide(Shape *shape, Vec3d pos, Quatf rotation) : shape(shape), pos(pos), rotation(rotation) {}

    Shape *shape;
    Vec3d pos;
    Quatf rotation;
};

struct ShapeCollision {
    bool hasCollision;
    Vec3d contactPoint;
    Vec3d resolution;
    Vec3d normal;
    double depth;
};

class ShapeCollider {

public:

    static ShapeCollision collision(ShapeToCollide left, ShapeToCollide right) {
        ShapeCollision collision;
        collision.hasCollision = false;
        collision.depth = 0;
        collision.normal = Vec3d();
        collision.contactPoint = Vec3d();
        collision.resolution = Vec3d();

        double minDepth = std::numeric_limits<double>::max();
        double maxDepth = 0;

        const Vec3i halfGrid = left.shape->getGridSize() / 2;
        for (int x = -halfGrid.x; x < halfGrid.x; ++x) {
            for (int y = -halfGrid.y; y < halfGrid.y; ++y) {
                for (int z = -halfGrid.z; z < halfGrid.z; ++z) {
                    if (left.shape->getGridDimension(x, y, z)) {
                        Vec3d coord = Vec3i(x, y, z).to<double>() + Vec3d(.5);
                        Vec3d pointPos = left.pos + left.rotation.to<double>().rotate(coord * Shape::GridSize);
                        Vec3d localPointPos = right.rotation.to<double>().inverse().rotate(pointPos - right.pos);
                        Vec3i otherGrid = (localPointPos.round() / Shape::GridSize).to<int>();

                        if (right.shape->getGridDimension(otherGrid.x, otherGrid.y, otherGrid.z)) {
                            Vec3d distanceVec = pointPos - right.pos;
                            double depth = distanceVec.length();
                            if (depth > maxDepth) {
                                collision.hasCollision = true;
                                collision.contactPoint = pointPos;
                                collision.normal = (pointPos - right.pos).normalize();
                            }

                            if (depth < minDepth) {
                                minDepth = depth;
                            }
                        }
                    }
                }
            }
        }

        if (collision.hasCollision) {
            collision.depth = maxDepth - minDepth;
            collision.resolution = collision.normal * collision.depth;
        }

        return collision;
    }


private:


};

}

#endif //COWPHYS_SHAPECOLLIDER_H
