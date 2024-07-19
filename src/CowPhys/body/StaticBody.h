#ifndef COWPHYS_STATICBODY_H
#define COWPHYS_STATICBODY_H

#include "Body.h"

namespace cp {
class StaticBody : public Body {

public:

    explicit StaticBody(Shape *shape) : Body(shape) {

    }


private:

};

}

#endif //COWPHYS_STATICBODY_H
