#ifndef COWPHYS_VIEWER_H
#define COWPHYS_VIEWER_H

#include <raylib.h>
#include "CowPhys/PhysWorld.h"

namespace viewer {

class Viewer {

public:

    Viewer();

    void run();


private:

    void update();

    void draw();

    void drawBody(cp::Body *body, Color color);


    cp::PhysWorld mWorld;
    Camera3D mCamera;

};

}

#endif //COWPHYS_VIEWER_H
