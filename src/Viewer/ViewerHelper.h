
#ifndef COWPHYS_VIEWERHELPER_H
#define COWPHYS_VIEWERHELPER_H

#include <raylib.h>
#include "CowPhys/math/Vec3.h"

namespace viewer {

class ViewerHelper {

public:

    static Vector3 vec3ToVec3(cp::Vec3d v) {
        Vector3 result;
        result.x = static_cast<float>(v.x);
        result.y = static_cast<float>(v.y);
        result.z = static_cast<float>(v.z);
        return result;
    }


};

}

#endif //COWPHYS_VIEWERHELPER_H
