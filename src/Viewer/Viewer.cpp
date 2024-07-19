#include "Viewer.h"
#include "rlgl.h"
#include "raymath.h"
#include "CowPhys/shape/BoxShape.h"

namespace viewer {

Viewer::Viewer() : mWorld() {
    InitWindow(800, 600, "CowPhys viewer");
    mCamera.position = (Vector3) {0, 0.6, -10};
    mCamera.target = (Vector3) {0, 0, 0};
    mCamera.up = (Vector3) {0, 1, 0};
    mCamera.fovy = 45.f;
    mCamera.projection = CAMERA_PERSPECTIVE;
    SetTargetFPS(60);

    auto floor = mWorld.createStaticBody(new cp::BoxShape(10, 0.2f, 10), cp::Vec3d());
    floor->setMass(1000);

    //auto a = mWorld.createDynBody(new cp::BoxShape(.5, .5, .5), cp::Vec3d(0, 2, 0));
    //a->setMass(10);
    //a->applyForceAt(cp::Vec3f(1, -2, 1), cp::Vec3f(1, 2, 1));
}

void Viewer::run() {
    while (!WindowShouldClose()) {
        UpdateCamera(&mCamera, CAMERA_CUSTOM);
        update();
        draw();
    }
}

void Viewer::update() {

    for (auto body: mWorld.getDynBodies()) {
        body->applyForce(cp::Vec3f(0, -9.81 * (1.0 / 60.0), 0));
    }


    mWorld.update(1.0 / 60.0);

    if (IsKeyPressed(KEY_Q)) {
        auto body = mWorld.createDynBody(new cp::BoxShape(cp::Vec3f(0.5)),
                                         cp::Vec3d(-5, 3, .2));
        body->applyForce(cp::Vec3f(10, 0, 0));
    }

    if (IsKeyPressed(KEY_W)) {
        auto body = mWorld.createDynBody(new cp::BoxShape(cp::Vec3f(0.5)),
                                         cp::Vec3d(5, 3.1, 0));
        body->applyForce(cp::Vec3f(-10, 0, 0));
    }
}

void Viewer::draw() {
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(mCamera);

    Color colors[] = {RED, BLUE, GREEN, PURPLE};

    int current = 0;

    for (auto body: mWorld.getDynBodies()) {
        drawBody(body, colors[++current % 4]);
    }

    current = 0;

    for (auto body: mWorld.getStaticBodies()) {
        drawBody(body, colors[++current % 4]);
    }

    EndMode3D();
    EndDrawing();
}

void Viewer::drawBody(cp::Body *body, Color color) {
    color.a = 100;

    rlPushMatrix();

    // Translate to the body's position
    rlTranslatef(body->getPos().x, body->getPos().y, body->getPos().z);

    // Apply the rotation
    auto quat = body->getRotation();
    Quaternion rotation = (Quaternion) {quat.x, quat.y, quat.z, quat.w};
    rlMultMatrixf(MatrixToFloat(QuaternionToMatrix(rotation)));

    // Draw the cube at the origin (since we already translated to the body's position)

    cp::Shape *shape = body->getShape();
    auto boxShape = dynamic_cast<cp::BoxShape *>(shape);

    if (boxShape != nullptr) {
        DrawCube((Vector3) {0, 0, 0},
                 boxShape->getHalfSize().x * 2.0,
                 boxShape->getHalfSize().y * 2.0,
                 boxShape->getHalfSize().z * 2.0, color);
    }

    rlPopMatrix();

}

}