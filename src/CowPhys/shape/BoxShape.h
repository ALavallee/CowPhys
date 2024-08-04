#ifndef COWPHYS_BOXSHAPE_H
#define COWPHYS_BOXSHAPE_H

#include "CowPhys/math/Vec3.h"
#include "Shape.h"
#include "MeshShape.h"
#include "CompShape.h"

namespace cp {

class BoxShape : public Shape {

public:

    BoxShape() : BoxShape(Vec3U(1)) {
    }

    explicit BoxShape(Vec3U halfSize) : mHalfSize(halfSize) {
        setupSpheres();
    }

    BoxShape(Unit halfX, Unit halfY, Unit halfZ) : BoxShape(Vec3U(halfX, halfY, halfZ)) {
    }

    Vec3U getHalfSize() {
        return mHalfSize;
    }

private:

    void setupSpheres() {
        auto radius = std::min(std::min(mHalfSize.x, mHalfSize.y), mHalfSize.z);

        auto countX = static_cast<Unit>((mHalfSize.x * 2) / radius);
        auto countY = static_cast<Unit>((mHalfSize.y * 2) / radius);
        auto countZ = static_cast<Unit>((mHalfSize.z * 2) / radius);

        for (Unit i = 0; i <= countX; ++i) {
            for (Unit j = 0; j <= countY; ++j) {
                for (Unit k = 0; k <= countZ; ++k) {
                    auto pos = Vec3U(
                            i * radius - mHalfSize.x,
                            j * radius - mHalfSize.y,
                            k * radius - mHalfSize.z
                    );
                    addSphere(SphereU(pos, radius));
                }
            }
        }
    }

    Vec3U mHalfSize;

};


}

#endif //COWPHYS_BOXSHAPE_H
