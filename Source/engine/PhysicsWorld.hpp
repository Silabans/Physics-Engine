#pragma once
#include "rigidbody.hpp"
#include <vector>
#include <memory>

class PhysicsWorld {
private:
    // inside logic
    std::vector<std::shared_ptr<RigidBody>> m_bodies;
    Vector2D m_gravity{0.0f, 900.0f};
    int m_impulseIterations{10};

    void ResolveCollisions();
    void ApplyExternalForces();

public:
    PhysicsWorld(Vector2D gravity = {0.0f, 900.0f});
    ~PhysicsWorld() = default;

    // Game management
    RigidBody* CreateCircle(Vector2D position, float radius, float mass);
    RigidBody* CreateBox(Vector2D position, float width, float height, float mass, float angle);

    void DestroyBody(RigidBody* body);
    void Clear();

    // Advances the simulation every time increment dt, displaying it with the stated fps
    void Step(float dt = 0.01, int fps = 120);

    // Read-only access for the renderer or other game logic loops
    const std::vector<std::shared_ptr<RigidBody>>& GetBodies() const { return m_bodies; }
    const Vector2D GetGravity() const { return m_gravity; }
};