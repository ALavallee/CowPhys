#ifndef COWPHYS_SATINFO_H
#define COWPHYS_SATINFO_H

#include "CowPhys/math/Vec3.h"

namespace cp {

struct SATInfo {
    bool isColliding;
    Vec3d normal;
    double depth;
    Vec3d resolution;
    Vec3d contact;

    static SATInfo noCollision() {
        SATInfo info;
        info.isColliding = false;
        return info;
    }
};


}

#endif //COWPHYS_SATINFO_H
