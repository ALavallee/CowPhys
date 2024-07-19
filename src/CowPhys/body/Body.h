#ifndef COWPHYS_BODY_H
#define COWPHYS_BODY_H

#include <vector>
#include <iostream>
#include "Collision.h"
#include "CowPhys/CowPhys.h"
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Box.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/SAT.h"
#include "CowPhys/math/Mat3.h"
#include "CowPhys/math/AABB.h"
#include "CowPhys/shape/Shape.h"
#include "CowPhys/shape/BoxShape.h"

namespace cp {

class Body {
public:
    explicit Body(Shape *shape) : mShape(shape), mMass(1), mRestitution(0.2),
                                  mFriction(0.6), mRotation(Quatf::identity()) {
    }

    SATInfo collides(Body *other) {
        auto boxLeft = dynamic_cast<BoxShape *>(mShape);
        auto boxRight = dynamic_cast<BoxShape *>(other->getShape());
        if (boxLeft != nullptr && boxRight != nullptr) {
            auto boxA = boxLeft->getBox<double>(getPos(), getRotation().to<double>());
            auto boxB = boxRight->getBox<double>(other->getPos(), other->getRotation().to<double>());
            return SAT::SATCollision(boxA, boxB);
        }

        return SAT::satNoCollision();
    }

    Shape *getShape() {
        return mShape;
    }

    void setPos(const Vec3<Coord> &pos) {
        mPosition = pos;
    }

    Vec3<Coord> getPos() const {
        return mPosition;
    }

    void setMass(float mass) {
        mMass = mass;
    }

    float getMass() const {
        return mMass;
    }

    float getMassInverse() const {
        return 1.0f / mMass;
    }

    void setRotation(const Quatf &rotation) {
        mRotation = rotation;
    }

    Quatf getRotation() const {
        return mRotation;
    }

    void addCollision(const Collision &collision) {
        mCollisions.emplace_back(collision);
    }

    const std::vector<Collision> &getCollisions() const {
        return mCollisions;
    }

    void clearCollisions() {
        mCollisions.clear();
    }

    void setRestitution(float restitution) {
        mRestitution = restitution;
    }

    float getRestitution() const {
        return mRestitution;
    }

    float getFriction() const {
        return mFriction;
    }


private:

    void updateInertiaTensor() {

    }

    float mMass;
    float mRestitution;
    float mFriction;
    Vec3<Coord> mPosition;
    Quatf mRotation;
    Shape *mShape;
    std::vector<Collision> mCollisions;
};

} // namespace cp

#endif // COWPHYS_BODY_H
