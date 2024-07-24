#ifndef COWPHYS_PHYSWORLD_H
#define COWPHYS_PHYSWORLD_H

#include <vector>
#include "body/Body.h"
#include "CowPhys/body/DynBody.h"
#include "CowPhys/body/StaticBody.h"
#include "WorldRaycast.h"
#include "CowPhys/math/algorithm/EPA.h"
#include "WorldRaycast.h"

namespace cp {

class PhysWorld {

public:

    void update(double deltaTime);

    WorldRaycast raycast(Vec3d pos, Vec3d to, double maxRange);

    DynBody *createDynBody(Shape *shape, Vec3d pos) {
        auto newBody = new DynBody(shape);
        newBody->setPos(pos);
        mDynBodies.push_back(newBody);
        return newBody;
    }

    std::vector<DynBody *> &getDynBodies() {
        return mDynBodies;
    }

    StaticBody *createStaticBody(Shape *shape, Vec3d pos) {
        auto newBody = new StaticBody(shape);
        newBody->setPos(pos);
        mStaticBodies.push_back(newBody);
        return newBody;
    }

    std::vector<StaticBody *> &getStaticBodies() {
        return mStaticBodies;
    }

private:
    void resolveCollision(DynBody *bodyA, DynBody *bodyB, CollisionPoint &collision, double deltaTime);

    void resolveCollision(DynBody *bodyA, StaticBody *bodyB, CollisionPoint &collision, double deltaTime);

    std::vector<DynBody *> mDynBodies;
    std::vector<StaticBody *> mStaticBodies;


};


}

#endif //COWPHYS_PHYSWORLD_H
