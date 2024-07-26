#include "PhysWorld.h"

namespace cp {

void PhysWorld::update(double deltaTime) {

    for (auto body: mDynBodies) {
        body->applyFriction(deltaTime);
        body->applyDamping(deltaTime);
        body->update(deltaTime);
    }

    for (auto body: mDynBodies) {

        auto bodyCollide = ShapeToCollide(body->getShape(), body->getPos(), body->getRotation());

        for (auto other: mDynBodies) {
            auto otherCollide = ShapeToCollide(other->getShape(), other->getPos(), other->getRotation());
            if (body != other && body < other) { // this check so there is only one collision
                auto collision = ShapeCollider::collision(bodyCollide, otherCollide);
                if (collision.hasCollision) {
                    resolveCollision(body, other, collision, deltaTime);
                }
            }
        }

        for (auto other: mStaticBodies) {
            auto otherCollide = ShapeToCollide(other->getShape(), other->getPos(), other->getRotation());
            auto collision = ShapeCollider::collision(bodyCollide, otherCollide);
            if (collision.hasCollision) {
                resolveCollision(body, other, collision, deltaTime);
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

void PhysWorld::resolveCollision(DynBody *bodyA, DynBody *bodyB, ShapeCollision &collision, double deltaTime) {

    Vec3d normal = collision.normal;

    Vec3f relativeVelocity = bodyB->getVelocity() - bodyA->getVelocity();
    float velocityAlongNormal = relativeVelocity.dot(normal.to<float>());

    // Calculate the impulse scalar
    float restitution = std::min(bodyA->getRestitution(), bodyB->getRestitution());
    float impulseScalar = -(1.0f + restitution) * velocityAlongNormal;
    impulseScalar /= bodyA->getMassInverse() + bodyB->getMassInverse();
    Vec3f impulse = normal.to<float>() * impulseScalar;

    Vec3f pointOfCollision = collision.contactPoint.to<float>();
    bodyA->applyForceAt(-impulse, pointOfCollision);
    bodyB->applyForceAt(impulse, pointOfCollision);

    // now move them outside of one and another
    Vec3d MTV = collision.resolution;
    bodyA->setPos(bodyA->getPos() - MTV * 0.5);
    bodyB->setPos(bodyB->getPos() + MTV * 0.5);
}

void PhysWorld::resolveCollision(DynBody *bodyA, StaticBody *bodyB, ShapeCollision &collision, double deltaTime) {
    const float largeMass = 1e5f; // Still large, but slightly lower

    Vec3d normal = collision.normal;
    Vec3f relativeVelocity = -bodyA->getVelocity();
    float velocityAlongNormal = relativeVelocity.dot(normal.to<float>());

    // Calculate impulse with potential angular effects
    float impulseScalar = -(1.0f + std::min(bodyA->getRestitution(), bodyB->getRestitution())) * velocityAlongNormal;
    impulseScalar /= bodyA->getMassInverse() + (1.0f / largeMass);
    Vec3f impulse = normal.to<float>() * impulseScalar;

    // Apply normal impulse at center of mass
    bodyA->applyForceAt(-impulse, bodyA->getPos().to<float>());

    // Calculate friction
    Vec3f tangent = (relativeVelocity - (normal.to<float>() * velocityAlongNormal)).normalize();
    float velocityAlongTangent = relativeVelocity.dot(tangent);

    // Friction coefficient
    float frictionCoefficient = std::min(bodyA->getFriction(), bodyB->getFriction());

    // Calculate friction impulse scalar
    float frictionImpulseScalar = -velocityAlongTangent / (bodyA->getMassInverse() + (1.0f / largeMass));

    // Clamp friction impulse to the Coulomb's law
    float maxFrictionImpulse = impulseScalar * frictionCoefficient;
    frictionImpulseScalar = std::clamp(frictionImpulseScalar, -maxFrictionImpulse, maxFrictionImpulse);

    // Apply friction impulse
    Vec3f frictionImpulse = tangent * frictionImpulseScalar;
    bodyA->applyForceAt(-frictionImpulse, bodyA->getPos().to<float>());

    // Adjust position to resolve penetration
    bodyA->setPos(bodyA->getPos() - collision.resolution);
}

}