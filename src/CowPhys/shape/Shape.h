#ifndef COWPHYS_SHAPE_H
#define COWPHYS_SHAPE_H

#include <vector>
#include "CowPhys/math/Mat3.h"
#include "CowPhys/math/Quat.h"
#include "CowPhys/math/Raycast.h"

namespace cp {

class Shape {

public:

    static constexpr double GridSize = 0.1;

    Shape() : mUserData(nullptr) {
    }

    virtual ~Shape() = default;

    void *getUserData() {
        return mUserData;
    }

    void setUserData(void *userData) {
        mUserData = userData;
    }

    virtual RaycastInfo raycast(Ray ray, Vec3d pos, Quatf rotation) {
        return RaycastInfo::noHit();
    }

    virtual std::vector<Vec3d> getVertices(Vec3d pos, Quatf rotation) {
        return {};
    }

    bool isInGridRange(int x, int y, int z) const {
        x += mGridsSize.x / 2;
        y += mGridsSize.y / 2;
        z += mGridsSize.z / 2;
        return x >= 0 && y >= 0 && z >= 0 &&
               x < mGridsSize.x && y < mGridsSize.y && z < mGridsSize.z;
    }

    Vec3i getGridSize() {
        return mGridsSize;
    }

    bool getGridDimension(int x, int y, int z) {
        if (!isInGridRange(x, y, z)) {
            return false;
        }

        x += mGridsSize.x / 2;
        y += mGridsSize.y / 2;
        z += mGridsSize.z / 2;

        size_t index = x + y * mGridsSize.x + z * mGridsSize.x * mGridsSize.y;
        return mGrids[index];
    }


protected:

    void setGridDimension(int halfX, int halfY, int halfZ) {
        mGridsSize = Vec3i(halfX * 2, halfY * 2, halfZ * 2);
        int totalSize = (halfX * 2) * (halfY * 2) * (halfZ * 2);
        mGrids.resize(totalSize, false);
    }

    void fillGrid(bool value) {
        for (auto &&mGrid: mGrids) {
            mGrid = value;
        }
    }

private:
    Vec3i mGridsSize;
    std::vector<bool> mGrids;
    void *mUserData;

};


}

#endif //COWPHYS_SHAPE_H
