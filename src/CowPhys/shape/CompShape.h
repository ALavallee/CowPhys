#ifndef COWPHYS_COMPSHAPE_H
#define COWPHYS_COMPSHAPE_H

#include <utility>
#include <vector>
#include "Shape.h"
#include "CowPhys/math/Quat.h"

namespace cp {

struct ShapeComposition {

    ShapeComposition(Shape *shape, Vec3d pos, Quatf rotation = Quatf::identity()) : shape(shape), position(pos),
                                                                                    rotation(rotation) {}

    Shape *shape;
    Vec3d position;
    Quatf rotation;
};

class CompShape : public Shape {

public:

    CompShape() = default;

    explicit CompShape(std::vector<ShapeComposition> compositions) : mComposites(std::move(compositions)) {

    }

    SATInfo collision(Vec3d pos, Quatf rotation, Shape *other, Vec3d otherPos, Quatf otherRot) override {
        for (auto &comp: mComposites) {
            auto shape = comp.shape;
            auto sat = shape->collision(pos + comp.position, rotation + comp.rotation, other, otherPos, otherRot);
            if (sat.isColliding) {
                return sat;
            }
        }

        return SATInfo::noCollision();
    }

    void addShape(Shape *shape, Vec3d pos, Quatf rotation = Quatf::identity()) {
        mComposites.emplace_back(shape, pos, rotation);
    }

    ShapeComposition getShape(size_t index) {
        return mComposites[index];
    }

    size_t getShapesCount() {
        return mComposites.size();
    }


private:
    std::vector<ShapeComposition> mComposites;

};


}

#endif //COWPHYS_COMPSHAPE_H
