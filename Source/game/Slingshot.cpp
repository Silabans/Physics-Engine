#include "Slingshot.hpp"
#include <cmath>


Slingshot::Slingshot(Vector2D anchor_pos, float max_radius, float force_scalar)
    : anchor(anchor_pos),
    startRadius(30.0f),
    maxRadius(max_radius),
    launchForceScalar(force_scalar),
    state(SlingshotState::IDLE),
    birdPos(anchor_pos),
    activeBirdBody(nullptr),
    cooldownMax(4.0f),
    cooldownTimer(0.0f) {}


void Slingshot::Update(PhysicsWorld& world, float dt) {
    Vector2 rayMouse = GetMousePosition();
    Vector2D mousePos { rayMouse.x, rayMouse.y };

    switch (state) {
        case SlingshotState::IDLE: {
            birdPos = anchor;

            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                Vector2D diff = anchor - mousePos;
                if (diff.x * diff.x + diff.y * diff.y <= (startRadius * startRadius)) {
                    state = SlingshotState::DRAGGING;
                }
            }
            break;
        }

        case SlingshotState::DRAGGING: {
            Vector2D dragVector = mousePos - anchor;
            float dragDist = std::sqrt(dragVector.x * dragVector.x + dragVector.y * dragVector.y);

            if (dragDist > maxRadius) {
                birdPos = anchor + (normalized(dragVector) * maxRadius);
            } else {
                birdPos = mousePos;
            }

            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
                Vector2D displacement = anchor - mousePos;
                Vector2D launchVel = displacement * launchForceScalar;

                float birdRadius = 15.0f;
                float birdMass = 3.0f;
                activeBirdBody = world.CreateCircle(birdPos, birdRadius, birdMass);

                activeBirdBody->setVelocity(launchVel);

                state = SlingshotState::LAUNCHED;
                cooldownTimer = 0.0f;
            }
            break;
        }

        case SlingshotState::LAUNCHED: {
            cooldownTimer += dt;

            bool birdStopped = false;
            if (activeBirdBody) {
                Vector2D vel = activeBirdBody->getVelocity();
                float speedSquared = vel.x * vel.x + vel.y * vel.y;
                if (speedSquared < 10.0f && cooldownTimer > 1.5f) {
                    birdStopped = true;
                }
            }

            // If the bird has stopped or the cooldown threshold has been surpassed,
            // end the cooldown period
            if (birdStopped || cooldownTimer >= cooldownMax) {
                state = SlingshotState::COOLDOWN;
            }
            break;
        }

        case SlingshotState::COOLDOWN: {
            // Reset the slingshot for the next launch
            activeBirdBody = nullptr;
            birdPos = anchor;
            state = SlingshotState::IDLE;
        }
        break;
    }
}


void Slingshot::Draw() {
    
    if (state == SlingshotState::DRAGGING || state == SlingshotState::IDLE) {
        // Draw Back Elastic Band
        DrawLineEx({ anchor.x - 10.0f, anchor.y }, { birdPos.x, birdPos.y }, 5.0f, BEIGE);

        // Draw Front Elastic Band
        DrawLineEx({ anchor.x + 10.0f, anchor.y }, { birdPos.x, birdPos.y }, 6.0f, BROWN);

        // Draw Bird preview while dragging
        DrawCircle((int)(birdPos.x), (int)(birdPos.y), 15.0f, RED);
    }

    if (state == SlingshotState::DRAGGING) {
        Vector2D displacement = anchor - birdPos;
        Vector2D initialVel = displacement * launchForceScalar;
        Vector2D gravity = (Vector2D){0.0f, 900.0f};

        for (int i = 0; i <= 15; ++i) {
            float t = i * 0.05f; // 0.05 s increments
            Vector2D trajPos = birdPos + (initialVel * t) + ( gravity * 0.5f * t * t);
            DrawCircle((int)(trajPos.x), (int)(trajPos.y), 3.0f, RAYWHITE);
        }
    }

    DrawRectangle((int)(anchor.x - 5.0f), (int)(anchor.y), 10, 50, DARKBROWN);
}
