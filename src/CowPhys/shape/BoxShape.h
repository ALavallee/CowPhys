#ifndef COWPHYS_BOXSHAPE_H
#define COWPHYS_BOXSHAPE_H

#include "CowPhys/math/Vec3.h"
#include "CowPhys/math/Box.h"
#include "CowPhys/math/sat/SATBoxBox.h"
#include "Shape.h"
#include "MeshShape.h"
#include "CompShape.h"
#include "CowPhys/math/Raycast.h"

namespace cp {

class BoxShape : public Shape {

public:

    BoxShape() : mHalfSize(1) {

    }

    BoxShape(Vec3f halfSize) : mHalfSize(halfSize) {

    }

    BoxShape(float halfX, float halfY, float halfZ) : mHalfSize(halfX, halfY, halfZ) {}

    Vec3f getHalfSize() {
        return mHalfSize;
    }

    template<class T>
    Box<T> getBox(Vec3<T> pos, Quat<T> quat) {
        return Box<T>(pos, mHalfSize.to<T>(), quat);
    }

    SATInfo collision(Vec3d pos, Quatf rotation, Shape *other, Vec3d otherPos, Quatf otherRot) override {
        auto boxA = getBox<double>(pos, rotation.to<double>());
        auto rightBox = dynamic_cast<BoxShape *>(other);
        if (rightBox != nullptr) {
            auto boxB = rightBox->getBox<double>(otherPos, otherRot.to<double>());
            return SATBoxBox::sat(boxA, boxB);
        }

        auto rightMesh = dynamic_cast<MeshShape *>(other);
        if (rightMesh != nullptr) {
            return rightMesh->collision(otherPos, otherRot, this, pos, rotation);
        }

        auto rightComp = dynamic_cast<CompShape *>(other);
        if (rightComp != nullptr) {
            return rightComp->collision(otherPos, otherRot, this, pos, rotation);
        }


        return SATInfo::noCollision();
    }

    RaycastInfo raycast(Ray ray, Vec3d pos, Quatf rotation) override {
        return Raycast::raycastBox(ray, getBox(pos, rotation.to<double>()));
    }

    std::vector<Vec3d> getVertices(Vec3d pos, Quatf rotation) override {
        auto vertices = getBox(pos, rotation.to<double>()).getVertices();
        return {vertices.begin(), vertices.end()};
    }

private:

    Vec3f mHalfSize;


};


}

#endif //COWPHYS_BOXSHAPE_H
