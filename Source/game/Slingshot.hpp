#pragma once
#include "PhysicsWorld.hpp"
#include <raylib.h>

// State Machine for the Drag-and-Launch mechanics of the slingshot
enum class SlingshotState {
    IDLE,
    DRAGGING,
    LAUNCHED,
    COOLDOWN
};


class Slingshot {
private:
    Vector2D anchor; // centre coordinates of the slingshot
    float startRadius;
    float maxRadius; // max stretch distance of the slingshot
    float launchForceScalar; // The scalar multiplier mapping distance of stretch to initial physics impulse

    SlingshotState state;
    Vector2D birdPos;
    RigidBody* activeBirdBody;

    float cooldownMax;
    float cooldownTimer;

public:
    Slingshot(Vector2D anchor_pos, float max_radius = 80.0f, float dforce_scalar);

    SlingshotState GetState() { return state; }

    void Update(PhysicsWorld& world, float dt);
    void Draw(); // Renders the bands and the Bird
};