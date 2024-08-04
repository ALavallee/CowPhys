#ifndef COWPHYS_COMPSHAPE_H
#define COWPHYS_COMPSHAPE_H

#include <utility>
#include <vector>
#include "Shape.h"

namespace cp {

struct Comp {
    Shape *shape;
    Vec3U position;
};

class CompShape : public Shape {

public:

    explicit CompShape() {
    }

    void addShape(Shape *shape, Vec3U pos) {
        auto comp = Comp();
        comp.shape = shape;
        comp.position = pos;
        mCompositions.push_back(comp);

        for (auto sphere: shape->getSpheres()) {
            sphere.moveBy(pos);
            addSphere(sphere);
        }
    }

    const std::vector<Comp> &getComposition() {
        return mCompositions;
    }


private:
    std::vector<Comp> mCompositions;


};


}

#endif //COWPHYS_COMPSHAPE_H
