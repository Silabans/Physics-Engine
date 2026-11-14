#include "game.hpp"
#include "raylib.h"
#include <cmath>


Game::Game() {
    world = PhysicsWorld({0.0f, 900.0f});
    qt = QuadTree((AABB){400.0f, 300.0f, 400.0f, 300.0f});
    slingshot = Slingshot({100.0f, 400.0f}, 80.0f, 0.15f);
    SpawnPigs();
}



void Game::Update(float dt) {
    if (!isRunning) return;

    slingshot.Update(world, dt);
    world.Step(dt, 120);

    //CleanupDeadBirds();
    CheckWinStatus();
}


void Game::DrawGround() {

}


void Game::Draw() {
    BeginDrawing();
    ClearBackground(BLACK);

    DrawGround();
    slingshot.Draw();
    DrawObjects();
    //DrawBirdPreview();

    //DrawUI();

    EndDrawing();
}

// Automatically handled by the Slingshot class instance
void Game::SpawnBirds() {
    remainingBirds--;
}


void Game::SpawnPigs() {
    auto pigShape = std::make_unique<Circle>(30.0f);
    auto pigBody = std::make_unique<RigidBody>(
        600.0f, 300.0f, 1.0f, std::move(pigShape), GREEN
    );
    pigBody->setVelocity({0.0f, 0.0f});

    RigidBody* pig = pigBody.get();
    world.CreateCircle({pigBody->getPositionX(), pigBody->getPositionY()}, 15.0f, 1.0f);

    objects.push_back(GameObject(pig, GREEN));

}

