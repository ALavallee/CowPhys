#ifndef COWPHYS_BOXSHAPE_H
#define COWPHYS_BOXSHAPE_H

#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Box.h"
#include "Shape.h"
#include "MeshShape.h"
#include "CompShape.h"
#include "CowPhys/math/Raycast.h"

namespace cp {

class BoxShape : public Shape {

public:

    BoxShape() : mHalfSize(1) {
        setupGrid();
    }

    BoxShape(Vec3f halfSize) : mHalfSize(halfSize) {
        setupGrid();
    }

    BoxShape(float halfX, float halfY, float halfZ) : mHalfSize(halfX, halfY, halfZ) {
        setupGrid();
    }

    Vec3f getHalfSize() {
        return mHalfSize;
    }

    template<class T>
    Box<T> getBox(Vec3<T> pos, Quat<T> quat) {
        return Box<T>(pos, mHalfSize.to<T>(), quat);
    }

    RaycastInfo raycast(Ray ray, Vec3d pos, Quatf rotation) override {
        return Raycast::raycastBox(ray, getBox(pos, rotation.to<double>()));
    }

    std::vector<Vec3d> getVertices(Vec3d pos, Quatf rotation) override {
        auto vertices = getBox(pos, rotation.to<double>()).getVertices();
        return {vertices.begin(), vertices.end()};
    }
private:

    void setupGrid() {
        setGridDimension(static_cast<int>(mHalfSize.x / Shape::GridSize),
                         static_cast<int>(mHalfSize.y / Shape::GridSize),
                         static_cast<int>(mHalfSize.z / Shape::GridSize));
        fillGrid(true);
    }

    Vec3f mHalfSize;

};


}

#endif //COWPHYS_BOXSHAPE_H
