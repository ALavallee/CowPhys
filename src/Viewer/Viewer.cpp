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

    /*auto t0 = cp::Triangle<double>(cp::Vec3d(0, 0, 0), cp::Vec3d(10, 0, 0), cp::Vec3d(10, 0, 10));
    auto t1 = cp::Triangle<double>(cp::Vec3d(10, 0, 10), cp::Vec3d(0, 0, 10), cp::Vec3d(0, 0, 0));
    auto test = mWorld.createStaticBody(new cp::BoxShape(10, 0.1, 10), cp::Vec3d(0, 0, 0));*/

    //auto a = mWorld.createDynBody(new cp::BoxShape(50, 50, 50), cp::Vec3U(0, 2, 0));
    //auto b = mWorld.createDynBody(new cp::BoxShape(.5, .5, .5), cp::Vec3d(0.01, 4, 0));
    auto c = mWorld.createStaticBody(new cp::BoxShape(500, 50, 500), cp::Vec3U());


    /*auto subOne = new cp::BoxShape(.2, .2, .2);
    auto subTwo = new cp::BoxShape(.2, .2, .2);
    auto comp = new cp::CompShape();
    comp->addShape(subOne, cp::Vec3d());
    comp->addShape(subTwo, cp::Vec3d(1, 1, 1));*/

    //auto c = mWorld.createDynBody(comp, cp::Vec3d(0, 3, 0));


}

void Viewer::run() {
    while (!WindowShouldClose()) {
        UpdateCamera(&mCamera, CAMERA_ORBITAL);
        update();
        draw();
    }
}

void Viewer::update() {

    mWorld.applyForceToAllDynBodies(cp::Vec3U(0, -8, 0));
    mWorld.update();

    if (IsKeyPressed(KEY_Q)) {
        auto body = mWorld.createDynBody(new cp::BoxShape(50, 50, 50), cp::Vec3U(-300, 150, 0));
        body->applyForce(cp::Vec3U(70, 0, 0));
    }

    if (IsKeyPressed(KEY_W)) {
        auto body = mWorld.createDynBody(new cp::BoxShape(50, 50, 50), cp::Vec3U(300, 150, 0));
        body->setRotation(cp::Vec3Small(0, 40, 40));
        body->applyForce(cp::Vec3U(-70, 0, 0));
    }

    if (IsKeyPressed(KEY_E)) {
        auto subOne = new cp::BoxShape(20, 20, 20);
        auto subTwo = new cp::BoxShape(20, 20, 20);
        auto comp = new cp::CompShape();
        comp->addShape(subOne, cp::Vec3U());
        comp->addShape(subTwo, cp::Vec3U(100, 100, 100));
        mWorld.createDynBody(comp, cp::Vec3U(0, 300, 0));
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

    if (IsKeyDown(KEY_LEFT_SHIFT)) {
        for (auto sphere: body->getShape()->getSpheres()) {
            sphere.rotateBy(body->getRotation().to<cp::Unit>());
            sphere.moveBy(body->getPos());
            DrawSphere(ViewerHelper::vec3ToVec3(sphere.getPosition()),
                       static_cast<float>(sphere.getRadius()) / 100.f, BLUE);
        }

        return;
    }

    Vector2 mousePos = GetMousePosition();
    auto ray = GetScreenToWorldRay(mousePos, mCamera);

    auto cameraPos = ViewerHelper::vec3ToVec3(ray.position);
    auto cameraDir = ViewerHelper::vec3ToVec3(ray.direction);
    auto worldRay = mWorld.raycast(cameraPos, cameraDir);
    if (worldRay.body == body) {
        color = WHITE;
    }

    color.a = 100;

    rlPushMatrix();

    // Translate to the body's position
    Vector3 pos = ViewerHelper::vec3ToVec3(body->getPos());
    rlTranslatef(pos.x, pos.y, pos.z);

    // Apply the rotation
    auto rotation = body->getRotation();
    rotation = rotation % 512;
    float rotX = ((2.f * 3.1415f) * static_cast<float>(rotation.x)) / 512.f;
    float rotY = ((2.f * 3.1415f) * static_cast<float>(rotation.y)) / 512.f;
    float rotZ = ((2.f * 3.1415f) * static_cast<float>(rotation.z)) / 512.f;
    auto quat = QuaternionFromEuler(rotX, rotY, rotZ);
    rlMultMatrixf(MatrixToFloat(QuaternionToMatrix(quat)));

    // Draw at the origin (since we already translated to the body's position)

    cp::Shape *shape = body->getShape();
    auto boxShape = dynamic_cast<cp::BoxShape *>(shape);
    if (boxShape != nullptr) {
        Vector3 halfSize = ViewerHelper::vec3ToVec3(boxShape->getHalfSize());
        DrawCube((Vector3) {0, 0, 0},
                 halfSize.x * 2,
                 halfSize.y * 2,
                 halfSize.z * 2, color);
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
        for (auto comp: compShape->getComposition()) {
            auto subBox = dynamic_cast<cp::BoxShape *>(comp.shape);
            if (subBox != nullptr) {
                Vector3 halfSize = ViewerHelper::vec3ToVec3(subBox->getHalfSize());
                DrawCube(ViewerHelper::vec3ToVec3(comp.position),
                         halfSize.x * 2,
                         halfSize.y * 2,
                         halfSize.z * 2, color);
            }
        }
    }

    rlPopMatrix();
}

}