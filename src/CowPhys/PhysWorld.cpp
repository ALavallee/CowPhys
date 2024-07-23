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

WorldRaycast PhysWorld::raycast(cp::Vec3d pos, cp::Vec3d to, double maxRange) {
    WorldRaycast worldRaycast;
    Ray ray;
    ray.pos = pos;
    ray.dir = (to - pos).normalize();
    for (auto body: mDynBodies) {
        auto bodyRay = body->raycast(ray);
        if (bodyRay.hit && bodyRay.distance < maxRange) {
            if (worldRaycast.raycast.distance > bodyRay.distance || worldRaycast.raycast.hit == false) {
                worldRaycast.raycast = bodyRay;
                worldRaycast.body_hit = body;
            }
        }
    }

    return worldRaycast;
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
    const float largeMass = 1e5f; // Still large, but slightly lower
    const float staticFriction = 0.7f; // Add static friction

    Vec3d normal = collision.normal;
    Vec3f relativeVelocity = -bodyA->getVelocity();
    float velocityAlongNormal = relativeVelocity.dot(normal.to<float>());

    // Calculate impulse with potential angular effects)
    float impulseScalar = -(1.0f + std::min(bodyA->getRestitution(), bodyB->getRestitution())) * velocityAlongNormal;
    impulseScalar /= bodyA->getMassInverse() + (1.0f / largeMass);
    Vec3f impulse = normal.to<float>() * impulseScalar;

    // Apply impulse at center of mass
    bodyA->applyForceAt(-impulse, bodyA->getPos().to<float>());

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