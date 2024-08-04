#ifndef COWPHYS_COLLISIONCHECKER_H
#define COWPHYS_COLLISIONCHECKER_H

#include <limits>
#include "body/Body.h"

namespace cp {

struct CollisionInfo {
    bool collision;
    Unit depth;
    Vec3U normal;
    Vec3U contact;
};

class CollisionChecker {


public:

    static CollisionInfo checkCollision(Body *left, Body *right) {
        CollisionInfo info;
        info.collision = false;
        info.depth = std::numeric_limits<Unit>::min();
        for (auto leftSphere: left->getShape()->getSpheres()) {
            leftSphere.rotateBy(left->getRotation().to<Unit>());
            leftSphere.moveBy(left->getPos());
            for (auto rightSphere: right->getShape()->getSpheres()) {
                rightSphere.rotateBy(right->getRotation().to<Unit>());
                rightSphere.moveBy(right->getPos());

                if (leftSphere.collides(rightSphere)) {
                    Unit depth = leftSphere.penetration(rightSphere);
                    if (depth > info.depth) {
                        info.collision = true;
                        info.depth = depth;
                        info.contact = rightSphere.getPosition();
                        info.normal = (rightSphere.getPosition() - leftSphere.getPosition()).normalize();
                    }
                }
            }
        }

        return info;
    }


private:


};


}

#endif //COWPHYS_COLLISIONCHECKER_H
