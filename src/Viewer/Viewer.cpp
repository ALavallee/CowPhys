#include "Viewer.h"
#include "rlgl.h"
#include "raymath.h"
#include "CowPhys/shape/BoxShape.h"
#include "CowPhys/math/Triangle.h"
#include "ViewerHelper.h"
#include "CowPhys/shape/CompShape.h"

namespace viewer {

Viewer::Viewer() : mWorld() {
    InitWindow(800, 600, "CowPhys viewer");
    mCamera.position = (Vector3) {0, 0.6, -10};
    mCamera.target = (Vector3) {0, 0, 0};
    mCamera.up = (Vector3) {0, 1, 0};
    mCamera.fovy = 45.f;
    mCamera.projection = CAMERA_PERSPECTIVE;
    SetTargetFPS(60);

    auto t0 = cp::Triangle<double>(cp::Vec3d(0, 0, 0), cp::Vec3d(10, 0, 0), cp::Vec3d(10, 0, 10));
    auto t1 = cp::Triangle<double>(cp::Vec3d(10, 0, 10), cp::Vec3d(0, 0, 10), cp::Vec3d(0, 0, 0));
    auto test = mWorld.createStaticBody(new cp::BoxShape(10, 0.1, 10), cp::Vec3d(0, 0, 0));

    auto a = mWorld.createDynBody(new cp::BoxShape(.5, .5, .5), cp::Vec3d(0, 2, 0));
    a->setRotation(cp::Quatf::fromEuler(0, 3.1415 / 4.0, 3.1415 / 4.0));
    //auto b = mWorld.createDynBody(new cp::BoxShape(.5, .5, .5), cp::Vec3d(0.01, 4, 0));


    auto subOne = new cp::BoxShape(.2, .2, .2);
    auto subTwo = new cp::BoxShape(.2, .2, .2);
    auto comp = new cp::CompShape();
    comp->addShape(subOne, cp::Vec3d());
    comp->addShape(subTwo, cp::Vec3d(1, 1, 1));

    auto c = mWorld.createDynBody(comp, cp::Vec3d(0, 3, 0));


}

void Viewer::run() {
    while (!WindowShouldClose()) {
        UpdateCamera(&mCamera, CAMERA_ORBITAL);
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
                                         cp::Vec3d(-3, 3, 0));
        body->applyForce(cp::Vec3f(10, 0, 0));
    }

    if (IsKeyPressed(KEY_W)) {
        auto body = mWorld.createDynBody(new cp::BoxShape(cp::Vec3f(0.5)),
                                         cp::Vec3d(3, 3, 0));
        body->applyForce(cp::Vec3f(-10, 0, 0));
    }
}

void Viewer::draw() {

    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(mCamera);


    Ray ray = GetScreenToWorldRay(GetMousePosition(), mCamera);
    auto raycast = mWorld.raycast(ViewerHelper::vec3ToVec3(ray.position),
                                  ViewerHelper::vec3ToVec3(ray.direction), 1000);

    Color colors[] = {RED, BLUE, GREEN, PURPLE};

    int current = 0;

    for (auto body: mWorld.getDynBodies()) {
        if (raycast.body_hit != body) {
            drawBody(body, colors[++current % 4]);
        } else {
            drawBody(body, BLACK);
        }
    }

    current = 0;

    for (auto body: mWorld.getStaticBodies()) {
        if (raycast.body_hit != body) {
            drawBody(body, colors[++current % 4]);
        } else {
            drawBody(body, BLACK);
        }
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

    // Draw at the origin (since we already translated to the body's position)

    cp::Shape *shape = body->getShape();
    auto boxShape = dynamic_cast<cp::BoxShape *>(shape);
    if (boxShape != nullptr) {
        DrawCube((Vector3) {0, 0, 0},
                 boxShape->getHalfSize().x * 2.0,
                 boxShape->getHalfSize().y * 2.0,
                 boxShape->getHalfSize().z * 2.0, color);
    }

    auto meshShape = dynamic_cast<cp::MeshShape *>(shape);
    if (meshShape != nullptr) {
        for (auto triangle: meshShape->getTriangles()) {
            Vector3 p0 = ViewerHelper::vec3ToVec3(triangle.p0);
            Vector3 p1 = ViewerHelper::vec3ToVec3(triangle.p1);
            Vector3 p2 = ViewerHelper::vec3ToVec3(triangle.p2);
            DrawTriangle3D(p2, p1, p0, color);
        }
    }

    auto compShape = dynamic_cast<cp::CompShape *>(shape);
    if (compShape != nullptr) {
        for (size_t i = 0; i < compShape->getShapesCount(); ++i) {
            auto comp = compShape->getShape(i);
            auto subBox = dynamic_cast<cp::BoxShape *>(comp.shape);
            if (subBox != nullptr) {
                DrawCube(ViewerHelper::vec3ToVec3(comp.position),
                         subBox->getHalfSize().x * 2.0,
                         subBox->getHalfSize().y * 2.0,
                         subBox->getHalfSize().z * 2.0, color);
            }
        }
    }

    rlPopMatrix();

}

}