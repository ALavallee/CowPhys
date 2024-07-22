#ifndef COWPHYS_COMPSHAPE_H
#define COWPHYS_COMPSHAPE_H

#include <vector>
#include "Shape.h"
#include "CowPhys/math/Quat.h"

namespace cp {

struct ShapeComposition {

    ShapeComposition(Shape *shape, Vec3d pos, Quatf rotation) : shape(shape), position(pos), rotation(rotation) {

    }

    Shape *shape;
    Vec3d position;
    Quatf rotation;
};

class CompShape : public Shape {

public:

    CompShape() {

    }

    void addShape(Shape *shape, Vec3d pos, Quatf rotation) {
        mComposites.emplace_back(shape, pos, rotation);
    }

    ShapeComposition getShape(size_t index) {
        return mComposites[index];
    }

    size_t size() {
        return mComposites.size();
    }


private:
    std::vector<ShapeComposition> mComposites;

};


}

#endif //COWPHYS_COMPSHAPE_H
