#ifndef COWPHYS_AABB_H
#define COWPHYS_AABB_H

#include "Vec3.h"
#include "Quat.h"

namespace cp {

template<class T>
class AABB {
public:

    AABB(Vec3<T> pos, Vec3<T> halfSize) : pos(pos), halfSize(halfSize) {}

    Vec3<T> pos;
    Vec3<T> halfSize;

    bool collides(AABB<T> &other) {
        Vec3<T> delta = pos - other.pos;
        return std::abs(delta.x) <= (halfSize.x + other.halfSize.x) &&
               std::abs(delta.y) <= (halfSize.y + other.halfSize.y) &&
               std::abs(delta.z) <= (halfSize.z + other.halfSize.z);
    }

};

}

#endif //COWPHYS_AABB_H
