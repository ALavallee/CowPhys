#include "PhysWorld.h"

namespace cp {

void PhysWorld::update(double deltaTime) {

    for (auto body: mDynBodies) {
        body->applyFriction(deltaTime);
        body->applyDamping(deltaTime);
        body->update(deltaTime);
    }

    for (auto body: mDynBodies) {

        for (auto other: mDynBodies) {
            if (body != other && body < other) { // this check so there is only one collision
                auto sat = body->collides(other);
                if (sat.isColliding) {
                    resolveCollision(body, other, sat, deltaTime);
                }
            }
        }

        for (auto other: mStaticBodies) {
            auto sat = body->collides(other);
            if (sat.isColliding) {
                resolveCollision(body, other, sat, deltaTime);
            }
        }
    }

}

void PhysWorld::resolveCollision(DynBody *bodyA, DynBody *bodyB, SATInfo &collision, double deltaTime) {

    Vec3d normal = collision.normal;

    Vec3f relativeVelocity = bodyB->getVelocity() - bodyA->getVelocity();
    float velocityAlongNormal = relativeVelocity.dot(normal.to<float>());

    // Calculate the impulse scalar
    float restitution = std::min(bodyA->getRestitution(), bodyB->getRestitution());
    float impulseScalar = -(1.0f + restitution) * velocityAlongNormal;
    impulseScalar /= bodyA->getMassInverse() + bodyB->getMassInverse();
    Vec3f impulse = normal.to<float>() * impulseScalar;

    Vec3f pointOfCollision = collision.contact.to<float>();
    bodyA->applyForceAt(-impulse, pointOfCollision);
    bodyB->applyForceAt(impulse, pointOfCollision);


    // now move them outside of one and another
    Vec3d MTV = collision.resolution;
    Vec3d collisionNormal = MTV.normalize();

    Vec3d centerA = bodyA->getPos();
    Vec3d centerB = bodyB->getPos();

    Vec3d centerToCenter = centerB - centerA;
    double dotProduct = collisionNormal.dot(centerToCenter);

    if (dotProduct > 0.0) {
        bodyA->setPos(bodyA->getPos() - MTV * 0.5);
        bodyB->setPos(bodyB->getPos() + MTV * 0.5);
    } else {
        bodyA->setPos(bodyA->getPos() + MTV * 0.5);
        bodyB->setPos(bodyB->getPos() - MTV * 0.5);
    }
}

void PhysWorld::resolveCollision(DynBody *bodyA, StaticBody *bodyB, SATInfo &collision, double deltaTime) {
    const float staticMass = 1000000;

    Vec3d normal = collision.normal;
    Vec3f relativeVelocity = -bodyA->getVelocity();
    float velocityAlongNormal = relativeVelocity.dot(normal.to<float>());

    // Calculate the impulse scalar
    float restitution = std::min(bodyA->getRestitution(), bodyB->getRestitution());
    float impulseScalar = -(1.0f + restitution) * velocityAlongNormal;
    impulseScalar /= bodyA->getMassInverse() + (1.0f / staticMass);
    Vec3f normalImpulse = normal.to<float>() * impulseScalar;

    // Apply the normal impulse
    Vec3f pointOfCollision = collision.contact.to<float>();
    bodyA->applyForceAt(-normalImpulse, pointOfCollision);

    // Now move them outside of one and another
    Vec3d MTV = collision.resolution;
    Vec3d collisionNormal = MTV.normalize();

    Vec3d centerA = bodyA->getPos();
    Vec3d centerB = bodyB->getPos();

    Vec3d centerToCenter = centerB - centerA;
    double dotProduct = collisionNormal.dot(centerToCenter);

    if (dotProduct > 0.0) {
        bodyA->setPos(bodyA->getPos() - MTV);
    } else {
        bodyA->setPos(bodyA->getPos() + MTV);
    }
}

}