#ifndef COWPHYS_PHYSWORLD_H
#define COWPHYS_PHYSWORLD_H

#include <vector>
#include "body/Body.h"
#include "CowPhys/body/DynBody.h"
#include "CowPhys/body/StaticBody.h"

namespace cp {

class PhysWorld {

public:

    void update(double deltaTime);

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
    void resolveCollision(DynBody *bodyA, DynBody *bodyB, SATInfo &collision, double deltaTime);

    void resolveCollision(DynBody *bodyA, StaticBody *bodyB, SATInfo &collision, double deltaTime);

    std::vector<DynBody *> mDynBodies;
    std::vector<StaticBody *> mStaticBodies;


};


}

#endif //COWPHYS_PHYSWORLD_H
