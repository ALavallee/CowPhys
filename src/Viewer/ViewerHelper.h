
#ifndef COWPHYS_VIEWERHELPER_H
#define COWPHYS_VIEWERHELPER_H

#include <raylib.h>
#include "CowPhys/math/Vec3.h"

namespace viewer {

class ViewerHelper {

public:

    static Vector3 vec3ToVec3(cp::Vec3U v) {
        Vector3 result;
        result.x = static_cast<float>(v.x) / 100.f;
        result.y = static_cast<float>(v.y) / 100.f;
        result.z = static_cast<float>(v.z) / 100.f;
        return result;
    }

    static cp::Vec3U vec3ToVec3(Vector3 v) {
        return {v.x * 100, v.y * 100, v.z * 100};
    }


};

}

#endif //COWPHYS_VIEWERHELPER_H
