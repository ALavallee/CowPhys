#ifndef COWPHYS_BOXSHAPE_H
#define COWPHYS_BOXSHAPE_H

#include "CowPhys/math/Vec3.h"
#include "Shape.h"

namespace cp {

class BoxShape : public Shape {

public:

    BoxShape() : mHalfSize(1) {

    }

    BoxShape(Vec3f halfSize) : mHalfSize(halfSize) {

    }

    BoxShape(float halfX, float halfY, float halfZ) : mHalfSize(halfX, halfY, halfZ) {}

    Vec3f getHalfSize() {
        return mHalfSize;
    }

    template<class T>
    Box<T> getBox(Vec3<T> pos, Quat<T> quat) {
        return Box<T>(pos, mHalfSize.to<T>(), quat);
    }

private:

    Vec3f mHalfSize;


};


}

#endif //COWPHYS_BOXSHAPE_H
