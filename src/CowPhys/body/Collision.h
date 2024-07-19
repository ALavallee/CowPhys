#ifndef COWPHYS_COLLISION_H
#define COWPHYS_COLLISION_H

#include "CowPhys/math/SAT.h"
#include "CowPhys/math/Vec3.h"

namespace cp {

class Body;

class Collision {

public:

    Collision(Body *a, Body *b, SATInfo sat) : mBodyA(a), mBodyB(b), mSat(sat) {

    }

    Body *getBodyA() {
        return mBodyA;
    }

    Body *getBodyB() {
        return mBodyB;
    }

    SATInfo &getSAT() {
        return mSat;
    }

private:
    Body *mBodyA;
    Body *mBodyB;
    SATInfo mSat;

};


}

#endif //COWPHYS_COLLISION_H
