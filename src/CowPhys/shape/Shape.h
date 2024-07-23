#ifndef COWPHYS_SHAPE_H
#define COWPHYS_SHAPE_H

#include "CowPhys/math/Mat3.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/sat/SatInfo.h"
#include "CowPhys/math/Raycast.h"

namespace cp {

class Shape {

public:

    Shape() : mUserData(nullptr) {
    }

    virtual ~Shape() = default;

    void *getUserData() {
        return mUserData;
    }

    void setUserData(void *userData) {
        mUserData = userData;
    }

    virtual SATInfo collision(Vec3d pos, Quatf rotation, Shape *other, Vec3d otherPos, Quatf otherRot) {
        return SATInfo::noCollision();
    }

    virtual RaycastInfo raycast(Ray ray, Vec3d pos, Quatf rotation) {
        return RaycastInfo::noHit();
    }


private:
    void *mUserData;

};


}

#endif //COWPHYS_SHAPE_H
