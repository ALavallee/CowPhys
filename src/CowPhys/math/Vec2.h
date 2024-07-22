#ifndef COWPHYS_VEC2_H
#define COWPHYS_VEC2_H

namespace cp {

template<class T>
class Vec2 {
public:

    T x;
    T y;

};

typedef Vec2<double> Vec2f;
typedef Vec2<double> Vec2d;

}

#endif //COWPHYS_VEC2_H
