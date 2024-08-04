#ifndef COWPHYS_AABB_H
#define COWPHYS_AABB_H

#include "Vec3.h"

namespace cp {

template<class T>
class AABB {
public:

    AABB(Vec3<T> pos, Vec3<T> halfSize) : pos(pos), halfSize(halfSize) {}

    Vec3<T> pos;
    Vec3<T> halfSize;

    bool collides(AABB<T> &other) {
        Vec3<T> delta = pos - other.pos;
        return delta.x <= (halfSize.x + other.halfSize.x) &&
               delta.y <= (halfSize.y + other.halfSize.y) &&
               delta.z <= (halfSize.z + other.halfSize.z);
    }

};

}

#endif //COWPHYS_AABB_H
