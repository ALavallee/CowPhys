#ifndef COWPHYS_BODY_H
#define COWPHYS_BODY_H

#include <vector>
#include <iostream>
#include "Collision.h"
#include "CowPhys/CowPhys.h"
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Box.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/sat/SATTrianglesBox.h"
#include "CowPhys/math/sat/SATBoxBox.h"
#include "CowPhys/math/Mat3.h"
#include "CowPhys/math/AABB.h"
#include "CowPhys/shape/Shape.h"
#include "CowPhys/shape/BoxShape.h"
#include "CowPhys/shape/MeshShape.h"

namespace cp {

class Body {
public:
    explicit Body(Shape *shape) : mShape(shape), mMass(1), mRestitution(0.2),
                                  mFriction(0.6), mRotation(Quatf::identity()) {
    }

    SATInfo collides(Body *other) {
        auto boxLeft = dynamic_cast<BoxShape *>(mShape);
        if (boxLeft != nullptr) {
            auto boxA = boxLeft->getBox<double>(getPos(), getRotation().to<double>());

            auto boxRight = dynamic_cast<BoxShape *>(other->getShape());
            if (boxRight != nullptr) {
                auto boxB = boxRight->getBox<double>(other->getPos(), other->getRotation().to<double>());
                return SATBoxBox::sat(boxA, boxB);
            }

            auto meshRight = dynamic_cast<MeshShape *>(other->getShape());
            if (meshRight != nullptr) {
                return SATTrianglesBox::sat(meshRight->getTriangles(), other->getPos(), other->getRotation(), boxA);
            }
        }

        auto meshLeft = dynamic_cast<MeshShape *>(getShape());
        if (meshLeft != nullptr) {
            auto boxRight = dynamic_cast<BoxShape *>(other->getShape());
            if (boxRight != nullptr) {
                auto box = boxRight->getBox<double>(getPos(), getRotation().to<double>());
                return SATTrianglesBox::sat(meshLeft->getTriangles(), getPos(), getRotation(), box);
            }
        }

        return SATInfo::noCollision();
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

    void setUserData(void *userData) {
        mUserData = userData;
    }

    void *getUserData() {
        return mUserData;
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
    void *mUserData;
};

} // namespace cp

#endif // COWPHYS_BODY_H
