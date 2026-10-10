#include "slingshot.hpp"

int main() {
    const int screenWidth = 800;
    const int screenHeight = 600;
    SetTargetFPS(120);
    InitWindow(screenWidth, screenHeight, "Slingshot Game");

    PhysicsWorld world({0.0f, 900.0f});
    Slingshot slingshot({150.0f, 450.0f});
    
    // Create Static Ground
    world.CreateBox({400.0f, 500.0f}, 800.0f, 40.0f, 0.0f, 0.0f);

    // This acts as a time middleman, allowing physics calculation to happen with the precision
    // of the time delta (dt) without having the frame rate to be as precise
    // For example, in 0.20s, 20 physics updates happen but the frame may only update once or twice
    // by aggregating the final outcome of the 20 updates
    double accumulator = 0.0;
    const float dt = 0.01f;
    int impulseIterations = 10;

    while (!WindowShouldClose()) { // runs while window is open
        float frameTime = GetFrameTime(); // delta time in seconds
        if (frameTime > 0.20) {
            frameTime = 0.20;
        }
        accumulator += frameTime;

        while (accumulator > dt) {
            //userInteraction(bodies, dt);
            slingshot.Update(world, frameTime);
            world.Step(frameTime);
        }

        BeginDrawing();
            // ClearBackground(BLACK);
            DrawRectangle(0, 0, 800, 600, Fade(BLACK, 0.75f)); // faded trails

            for (const auto& body : world.GetBodies()) {
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
            slingshot.Draw();
            DrawFPS(10, 10);

        EndDrawing();
    }
    CloseWindow();
    return 0;
}

