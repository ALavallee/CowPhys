#ifndef COWPHYS_CONTACTLISTENER_H
#define COWPHYS_CONTACTLISTENER_H

#include "CowPhys/body/Body.h"

namespace cp {


class ContactListener {

public:

    virtual void onContactBegin(Body *left, Body *right) {

    }

    virtual void onContactEnd(Body *left, Body *right) {

    }

};


}

#endif //COWPHYS_CONTACTLISTENER_H
