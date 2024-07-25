#ifndef COWPHYS_BODY_H
#define COWPHYS_BODY_H

#include <vector>
#include <iostream>
#include "CowPhys/CowPhys.h"
#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Box.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/Mat3.h"
#include "CowPhys/math/AABB.h"
#include "CowPhys/shape/Shape.h"
#include "CowPhys/shape/BoxShape.h"
#include "CowPhys/shape/MeshShape.h"
#include "CowPhys/shape/CompShape.h"

namespace cp {

class Body {
public:
    explicit Body(Shape *shape) : mShape(shape), mMass(1), mRestitution(0.2),
                                  mFriction(0.8), mRotation(Quatf::identity()) {
    }

    RaycastInfo raycast(Ray ray) {
        return mShape->raycast(ray, getPos(), getRotation());
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
    void *mUserData;
};

} // namespace cp

#endif // COWPHYS_BODY_H
