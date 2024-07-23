#include "MeshShape.h"
#include "BoxShape.h"
#include "CowPhys/math/sat/SATTriBox.h"

namespace cp {

SATInfo MeshShape::collision(Vec3d pos, Quatf rotation, Shape *other, Vec3d otherPos, Quatf otherRot) {
    auto rightBox = dynamic_cast<BoxShape *>(other);
    if (rightBox != nullptr) {
        auto box = rightBox->getBox(otherPos, otherRot.to<double>());
        return SATTriBox::sat(mTriangles, pos, rotation, box);
    }

    return SATInfo::noCollision();
}


}