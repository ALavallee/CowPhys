#include "PhysWorld.h"

namespace cp {

PhysWorld::PhysWorld() : mContactListener(nullptr), mMovementListener(nullptr) {

}

PhysWorld::~PhysWorld() {
    delete mContactListener;
    delete mMovementListener;
}

void PhysWorld::update() {

    for (auto body: mDynBodies) {
        auto oldPos = body->getPos();
        auto oldRotation = body->getRotation();
        body->update();

        if (mMovementListener != nullptr) {
            if (body->getPos() != oldPos) {
                mMovementListener->onMove(body, oldPos);
            }
            if (body->getRotation() != oldRotation) {
                mMovementListener->onRotate(body, oldRotation);
            }
        }
    }

    for (auto body: mDynBodies) {

        for (auto other: mDynBodies) { ;
            if (body != other && body < other) { // this check is there to only check that collision once
                auto collision = CollisionChecker::checkCollision(body, other);
                if (collision.collision) {
                    resolveCollision(body, other, collision);
                }
            }
        }

        for (auto other: mStaticBodies) {
            auto collision = CollisionChecker::checkCollision(body, other);
            if (collision.collision) {
                resolveCollision(body, other, collision);
            }
        }
    }
}

WorldRaycast PhysWorld::raycast(Vec3U pos, Vec3U dir, Body *bodyToIgnore) {
    WorldRaycast raycast;
    raycast.body = nullptr;
    raycast.shape = nullptr;
    raycast.distance = std::numeric_limits<Unit>::max();

    for (auto body: mDynBodies) {
        if (body != bodyToIgnore) {
            if (body->raycast(pos, dir, raycast.distance)) {
                raycast.body = body;
            }
        }
    }

    for (auto body: mStaticBodies) {
        if (body != bodyToIgnore) {
            if (body->raycast(pos, dir, raycast.distance)) {
                raycast.body = body;
            }
        }
    }

    return raycast;
}

void PhysWorld::resolveCollision(DynBody *bodyA, DynBody *bodyB, CollisionInfo &collision) {

    auto impulse = collision.normal;

    bodyA->applyForceAt(-impulse, collision.contact);
    bodyB->applyForceAt(impulse, collision.contact);

    Vec3U mtv = collision.normal * collision.depth;
    bodyA->setPos(bodyA->getPos() - (mtv / 2));
    bodyB->setPos(bodyB->getPos() + (mtv / 2));
}

void PhysWorld::resolveCollision(DynBody *bodyA, StaticBody *bodyB, CollisionInfo &collision) {
    auto impulse = collision.normal * -bodyA->getVelocity();

    bodyA->applyForceAt(-impulse, collision.contact);

    Vec3U mtv = collision.normal * collision.depth;
    bodyA->setPos(bodyA->getPos() - mtv);
}

}