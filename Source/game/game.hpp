#pragma once
#include "GameObject.hpp"
#include "rendering.hpp"
#include "QuadTree.hpp"
#include "Slingshot.hpp"


class Game {
private:
    PhysicsWorld world;
    QuadTree qt;
    Slingshot slingshot;
    std::vector<GameObject> objects;

    bool isRunning = true;
    int remainingBirds = 3;

public:
    Game();

    void Update(float dt);
    void Draw();

    void DrawGround();
    void DrawObjects();

    void SpawnBirds();
    void SpawnPigs();
    void CheckWinStatus();
};