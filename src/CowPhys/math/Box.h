#ifndef COWPHYS_BOX_H
#define COWPHYS_BOX_H

#include <array>
#include "Vec3.h"
#include "Quat.h"

namespace cp {

template<class T>
class Box {
public:

    Box() {

    }

    Box(Vec3<T> pos, Vec3<T> halfSize) : pos(pos), halfSize(halfSize), rotation(Quat<T>::identity()) {}

    Box(Vec3<T> pos, Vec3<T> halfSize, Quat<T> rotation) : pos(pos), halfSize(halfSize),
                                                           rotation(rotation) {}

    Vec3<T> max() {
        return pos + (halfSize / static_cast<T>(2));
    }

    Vec3<T> min() {
        return pos - (halfSize / static_cast<T>(2));
    }

    template<class C>
    Box<C> to() {
        Box<C> box;
        box.pos = pos.template to<C>();
        box.size = halfSize.template to<C>();
        return box;
    }

    std::array<Vec3<T>, 8> getVertices() {
        std::array<Vec3<T>, 8> vertices;
        Vec3<T> localVertices[8] = {
                Vec3<T>(-halfSize.x, -halfSize.y, -halfSize.z),
                Vec3<T>(halfSize.x, -halfSize.y, -halfSize.z),
                Vec3<T>(-halfSize.x, halfSize.y, -halfSize.z),
                Vec3<T>(halfSize.x, halfSize.y, -halfSize.z),
                Vec3<T>(-halfSize.x, -halfSize.y, halfSize.z),
                Vec3<T>(halfSize.x, -halfSize.y, halfSize.z),
                Vec3<T>(-halfSize.x, halfSize.y, halfSize.z),
                Vec3<T>(halfSize.x, halfSize.y, halfSize.z),
        };

        for (int i = 0; i < 8; ++i) {
            vertices[i] = pos + rotation.rotate(localVertices[i]);
        }

        return vertices;
    }

    Vec3<T> pos;
    Vec3<T> halfSize;
    Quat<T> rotation;

};


typedef Box<float> Boxf;
typedef Box<double> Boxd;

}

#endif //COWPHYS_BOX_H
