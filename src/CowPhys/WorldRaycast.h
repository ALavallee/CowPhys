#ifndef COWPHYS_WORLDRAYCAST_H
#define COWPHYS_WORLDRAYCAST_H

#include <limits>
#include "math/Raycast.h"
#include "body/Body.h"
#include "shape/Shape.h"

namespace cp {

struct WorldRaycast {
    RaycastInfo raycast;
    Body *body_hit;

    WorldRaycast() : raycast(RaycastInfo::noHit()), body_hit(nullptr) {
        raycast.distance = std::numeric_limits<double>::max();
    }
};


}

#endif //COWPHYS_WORLDRAYCAST_H
