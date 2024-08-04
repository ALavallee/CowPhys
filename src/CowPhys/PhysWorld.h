#ifndef COWPHYS_PHYSWORLD_H
#define COWPHYS_PHYSWORLD_H

#include <vector>
#include "CollisionChecker.h"
#include "body/Body.h"
#include "CowPhys/body/DynBody.h"
#include "CowPhys/body/StaticBody.h"
#include "interface/ContactListener.h"
#include "interface/MovementListener.h"

namespace cp {

struct WorldRaycast {
    Unit distance;
    Vec3U contact;
    Shape *shape;
    Body *body;
};

class PhysWorld {

public:

    PhysWorld();

    ~PhysWorld();

    void update();

    WorldRaycast raycast(Vec3U pos, Vec3U dir, Body *bodyToIgnore = nullptr);

    void applyForceToAllDynBodies(Vec3U force) {
        for (auto &body: mDynBodies) {
            body->applyForce(force);
        }
    }

    DynBody *createDynBody(Shape *shape, Vec3U pos) {
        auto newBody = new DynBody(shape);
        newBody->setPos(pos);
        mDynBodies.push_back(newBody);
        return newBody;
    }

    std::vector<DynBody *> &getDynBodies() {
        return mDynBodies;
    }

    StaticBody *createStaticBody(Shape *shape, Vec3U pos) {
        auto newBody = new StaticBody(shape);
        newBody->setPos(pos);
        mStaticBodies.push_back(newBody);
        return newBody;
    }

    std::vector<StaticBody *> &getStaticBodies() {
        return mStaticBodies;
    }

    void setContactListener(ContactListener *contactListener) {
        mContactListener = contactListener;
    }

    void setMovementListener(MovementListener *movementListener) {
        mMovementListener = movementListener;
    }

private:
    void resolveCollision(DynBody *bodyA, DynBody *bodyB, CollisionInfo &collision);

    void resolveCollision(DynBody *bodyA, StaticBody *bodyB, CollisionInfo &collision);

    ContactListener *mContactListener;
    MovementListener *mMovementListener;

    std::vector<DynBody *> mDynBodies;
    std::vector<StaticBody *> mStaticBodies;


};


}

#endif //COWPHYS_PHYSWORLD_H
