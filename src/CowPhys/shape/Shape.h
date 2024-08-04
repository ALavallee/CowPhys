#ifndef COWPHYS_SHAPE_H
#define COWPHYS_SHAPE_H

#include <vector>
#include "CowPhys/math/Sphere.h"

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

    void addSphere(SphereU sphere) {
        mSpheres.emplace_back(sphere);
    }

    const std::vector<SphereU> &getSpheres() {
        return mSpheres;
    }

private:
    std::vector<SphereU> mSpheres;
    void *mUserData;

};


}

#endif //COWPHYS_SHAPE_H
