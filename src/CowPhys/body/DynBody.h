#ifndef COWPHYS_DYNBODY_H
#define COWPHYS_DYNBODY_H

#include "CowPhys/math/Quat.h"
#include "CowPhys/math/Mat3.h"
#include "Body.h"

namespace cp {

class DynBody : public Body {

public:

    explicit DynBody(Shape *shape) : Body(shape) {
        updateInertiaTensor();
    }

    void update(double deltaTime) {
        updateInertiaTensor();
        setPos(getPos() + mVelocity.to<double>() * deltaTime);
        auto angularVelocityQuat = Quatf(mAngularVelocity.x, mAngularVelocity.y, mAngularVelocity.z, 0);
        Quat deltaRotation = angularVelocityQuat * getRotation() * (0.5f * static_cast<float>(deltaTime));
        setRotation((getRotation() + deltaRotation).normalize());
    }

    void applyFriction(double deltaTime) {
        mVelocity = mVelocity * (1.f - getFriction() * static_cast<float>(deltaTime));
    }

    void applyDamping(double deltaTime) {
        mAngularVelocity = mAngularVelocity * static_cast<float>(std::pow(1.0 - 0.98, deltaTime));
    }

    void applyForce(const Vec3f &force) {
        Vec3f acceleration = force / getMass();
        mVelocity = mVelocity + acceleration;
    }

    void applyForceAt(const Vec3f &force, const Vec3f &pos) {
        applyForce(force);
        Vec3f relativePos = pos - getPos().to<float>();
        Vec3f torque = relativePos.cross(force);
        Vec3f angularAcceleration = mInertiaTensor.inverse() * torque;
        mAngularVelocity = mAngularVelocity + angularAcceleration;
    }

    Vec3f getVelocity() const {
        return mVelocity;
    }

    void setVelocity(const Vec3f &velocity) {
        mVelocity = velocity;
    }

    void setAngularVelocity(const Vec3f &angular) {
        mAngularVelocity = angular;
    }

    Vec3f getAngularVelocity() const {
        return mAngularVelocity;
    }

private:

    void updateInertiaTensor() {
        auto shape = getShape();
        auto box = dynamic_cast<BoxShape *>(shape);
        if (box != nullptr) {
            float width = box->getHalfSize().x * 2;
            float height = box->getHalfSize().y * 2;
            float depth = box->getHalfSize().z * 2;
            float massDiv12 = getMass() / 12.0f;
            mInertiaTensor = Mat3::diagonal(massDiv12 * (height * height + depth * depth),
                                            massDiv12 * (width * width + depth * depth),
                                            massDiv12 * (width * width + height * height));
        }
    }

    Vec3f mVelocity;
    Vec3f mAngularVelocity;
    Mat3 mInertiaTensor;

};


}

#endif //COWPHYS_DYNBODY_H
