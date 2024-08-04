#ifndef COWPHYS_BODY_H
#define COWPHYS_BODY_H

#include <vector>
#include <iostream>
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/AABB.h"
#include "CowPhys/shape/Shape.h"
#include "CowPhys/shape/BoxShape.h"
#include "CowPhys/shape/MeshShape.h"
#include "CowPhys/shape/CompShape.h"
#include "Collision.h"

namespace cp {

class Body {
public:

    explicit Body(Shape *shape) : mShape(shape), mMass(1), mRestitution(1),
                                  mFriction(1), mRotation(), mUserData(nullptr) {
    }

    virtual void update() {
        for (auto &collision: mCollisions) {
            collision.update();
        }
    }

    Shape *getShape() {
        return mShape;
    }

    void setPos(const Vec3U &pos) {
        mPosition = pos;
    }

    Vec3U getPos() const {
        return mPosition;
    }

    void setMass(SmallUnit mass) {
        mMass = mass;
    }

    SmallUnit getMass() const {
        return mMass;
    }

    void setRotation(const Vec3Small &rotation) {
        mRotation = rotation;
    }

    Vec3Small getRotation() const {
        return mRotation;
    }

    void setRestitution(SmallUnit restitution) {
        mRestitution = restitution;
    }

    SmallUnit getRestitution() const {
        return mRestitution;
    }

    SmallUnit getFriction() const {
        return mFriction;
    }

    void setUserData(void *userData) {
        mUserData = userData;
    }

    void *getUserData() {
        return mUserData;
    }

    bool raycast(Vec3U pos, Vec3U dir, Unit &t) {
        bool found = false;
        for (auto sphere: mShape->getSpheres()) {
            sphere.rotateBy(getRotation().to<Unit>());
            sphere.moveBy(getPos());
            Unit current;
            if (sphere.raycast(pos, dir, current)) {
                if (current < t) {
                    t = current;
                }
                found = true;
            }
        }

        return found;

    }

    bool hasCollisionWith(Body *body) {
        for (auto &collision: mCollisions) {
            if (collision.getCollided() == body) {
                return true;
            }
        }

        return false;
    }

    void addCollision(Body *body) {
        if (!hasCollisionWith(body)) {
            mCollisions.emplace_back(body);
        }
    }

    const std::vector<Collision> &getCollisions() {
        return mCollisions;
    }

private:

    Vec3U mPosition;
    Vec3Small mRotation;
    Shape *mShape;
    SmallUnit mMass;
    SmallUnit mRestitution;
    SmallUnit mFriction;
    std::vector<Collision> mCollisions;
    void *mUserData;
};

} // namespace cp

#endif // COWPHYS_BODY_H
