#pragma once
#include "raylib.h"
#include "PhysicsWorld.hpp"
#include <random>

Color colors[7] = {RED, BLUE, YELLOW, GREEN, WHITE, PURPLE, ORANGE};

// reusable random device seeding
std::mt19937& random_engine() {
    // initialised once in the first function call (static var)
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return gen;
}


void render(std::vector<std::unique_ptr<RigidBody>>& bodies) {
    BeginDrawing();
        // ClearBackground(BLACK);
        DrawRectangle(0, 0, 800, 600, Fade(BLACK, 0.75f)); // faded trails

        for (const auto& body : bodies) {
            const Shape* shape = body->getShape(); // Shape, not ShapeType

            if (shape->type == ShapeType::CIRCLE) {
                const Circle* circle = static_cast<const Circle*>(shape); // convert shape to circle
                DrawCircle(
                    static_cast<int>(body->getPositionX()),
                    static_cast<int>(body->getPositionY()),
                    static_cast<int>(circle->radius),
                    body->getColor()
                );
            }
            else if (shape->type == ShapeType::BOX) {
                const Box* box = static_cast<const Box*>(shape); // convert shape to circle

                Rectangle values = { body->getPositionX(), body->getPositionY(), box->width, box->height};
                Vector2 origin = { box->halfExtents.x, box->halfExtents.y}; // centre of the box (point of rotation)
                float angleDegrees = box->angle * (180.0f / PI); // raylib requires degrees

                DrawRectanglePro(
                    values,
                    origin,
                    angleDegrees,
                    body->getColor()
                );
            }
        }
        slingshot.
        DrawFPS(10, 10);

    EndDrawing();
}
