#ifndef COWPHYS_DYNBODY_H
#define COWPHYS_DYNBODY_H

#include "Body.h"

namespace cp {

class DynBody : public Body {

    static int constexpr VelocityToPosition = 8;

public:

    explicit DynBody(Shape *shape) : Body(shape), mAllowRotation(true) {
    }

    void update() override {
        Body::update();
        setPos(getPos() + mVelocity / VelocityToPosition);
        setRotation(getRotation() + mAngularVelocity.to<SmallUnit>() / VelocityToPosition);
        applyFriction();
    }

    void applyForce(const Vec3U &force) {
        Vec3U acceleration = force / getMass();
        mVelocity = mVelocity + acceleration;
    }

    void applyForceAt(const Vec3U &force, const Vec3U &at) {
        applyForce(force);
        if (mAllowRotation) {
            auto relative = (at - getPos()).normalize();
            auto torque = relative.cross(force);
            mAngularVelocity = mAngularVelocity + torque;
        }
    }

    Vec3U getVelocity() const {
        return mVelocity;
    }

    void setVelocity(const Vec3U &velocity) {
        mVelocity = velocity;
    }

    void setAngularVelocity(const Vec3U &angular) {
        mAngularVelocity = angular;
    }

    Vec3U getAngularVelocity() const {
        return mAngularVelocity;
    }

    void setRotationAllowed(bool allowed) {
        mAllowRotation = allowed;
    }

    bool isRotationAllowed() const {
        return mAllowRotation;
    }

private:

    void applyFriction() {
        auto friction = getFriction();
        for (int i = 0; i < 3; ++i) {
            if (mVelocity[i] >= friction) {
                mVelocity[i] = mVelocity[i] - friction;
            } else if (mVelocity[i] <= -friction) {
                mVelocity[i] = mVelocity[i] + friction;
            } else {
                mVelocity[i] = 0;
            }
        }
    }

    Vec3U mVelocity;
    Vec3U mAngularVelocity;
    bool mAllowRotation;

};


}

#endif //COWPHYS_DYNBODY_H
