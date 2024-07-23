#ifndef COWPHYS_RAYCAST_H
#define COWPHYS_RAYCAST_H

#include <algorithm>
#include "Vec3.h"
#include "Box.h"

namespace cp {

struct Ray {
    Vec3d pos;
    Vec3d dir;
};

struct RaycastInfo {
    bool hit;
    double distance;
    Vec3d hitpoint;

    static RaycastInfo noHit() {
        RaycastInfo info;
        info.hit = false;
        return info;
    }
};

class Raycast {
public:
    static RaycastInfo raycastBox(Ray ray, Boxd box) {
        RaycastInfo info;
        info.hit = false;
        info.distance = std::numeric_limits<double>::infinity();

        // Transform the ray into the local space of the box
        Quat invRotation = box.rotation.conjugate();
        Vec3d localRayPos = invRotation * (ray.pos - box.pos);
        Vec3d localRayDir = invRotation * ray.dir;

        Vec3d invDir = {1.0 / localRayDir.x, 1.0 / localRayDir.y, 1.0 / localRayDir.z};
        Vec3d t1 = (box.halfSize - localRayPos) * invDir;
        Vec3d t2 = (-box.halfSize - localRayPos) * invDir;

        Vec3d tMinVec = t1.min(t2);
        Vec3d tMaxVec = t1.max(t2);

        double tMin = std::max({tMinVec.x, tMinVec.y, tMinVec.z});
        double tMax = std::min({tMaxVec.x, tMaxVec.y, tMaxVec.z});

        if (tMax >= std::max(tMin, 0.0)) {
            info.hit = true;
            info.distance = tMin;
            info.hitpoint = ray.pos + ray.dir * tMin;
        }

        return info;
    }



};


}

#endif //COWPHYS_RAYCAST_H
