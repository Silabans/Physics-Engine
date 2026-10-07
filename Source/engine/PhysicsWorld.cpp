#include "PhysicsWorld.hpp"
#include "Collision.hpp"

PhysicsWorld::PhysicsWorld(Vector2D gravity) : m_gravity(gravity) {}

RigidBody* PhysicsWorld::CreateBox(Vector2D position, float width, float height, float mass, float angle) {
    auto shape = std::make_unique<Box>(width, height, angle);
    auto body = std::make_unique<RigidBody>(position.x, position.y, mass, std::move(shape));

    RigidBody* rawPtr = body.get(); // get the raw pointer to the body
    m_bodies.push_back(std::move(body));
    return rawPtr;
}

RigidBody* PhysicsWorld::CreateCircle(Vector2D position, float radius, float mass) {
    auto shape = std::make_unique<Circle>(radius);
    auto body = std::make_unique<RigidBody>(position.x, position.y, mass, std::move(shape));

    RigidBody* rawPtr = body.get();
    m_bodies.push_back(std::move(body));
    return rawPtr;
}

void PhysicsWorld::Step(float dt, int fps) {
    for (auto& body : m_bodies) {
        if (body->getInverseMass() > 0.0f) {
            body->add_forces(m_gravity * body->getMass());
        }
    }

    for (int i = 0; i < m_impulseIterations; ++i) {
        resolve_all_collisions(m_bodies);
    }

    for (auto& body : m_bodies) {
        body->calculate_velocity(dt);
        body->integrate_pos(dt, fps);
        body->reset_forces();
    }
}