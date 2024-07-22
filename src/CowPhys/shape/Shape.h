#ifndef COWPHYS_SHAPE_H
#define COWPHYS_SHAPE_H

#include "CowPhys/math/Mat3.h"

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


private:
    void *mUserData;

};


}

#endif //COWPHYS_SHAPE_H
