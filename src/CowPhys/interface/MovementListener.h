#ifndef COWPHYS_MOVEMENTLISTENER_H
#define COWPHYS_MOVEMENTLISTENER_H

#include "CowPhys/body/Body.h"

namespace cp {


class MovementListener {

public:


    virtual void onMove(Body *left, Vec3<Unit> oldPos) {

    }

    virtual void onRotate(Body *left, Vec3<SmallUnit> oldRotation) {

    }


};


}

#endif //COWPHYS_MOVEMENTLISTENER_H
