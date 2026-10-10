#include "PhysicsWorld.hpp"
#include "Collision.hpp"
#include "QuadTree.hpp"

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

    AABB worldBounds{400.0f, 300.0f, 400.0f, 300.0f};
    QuadTree tree(worldBounds);

    // insert all bodies to the tree
    for (auto& body : m_bodies) {
        tree.insert(Point{ body->getPositionX(), body->getPositionY(), body.get()});
    }

    for (int iter = 0; iter < m_impulseIterations; ++iter) {
        for (auto& body : m_bodies) {
            float searchRadius = 100.0f;
            AABB searchRange{ body->getPositionX(), body->getPositionY(), searchRadius, searchRadius };

            std::vector<Point> nearbyPoints;
            tree.query(searchRange, nearbyPoints); // push all nearby bodies to nearbyPoints

            for (auto& point : nearbyPoints) {
                // Compare the memory address of the current body to a nearby one.
                // This ensures the pair is resolved once only (A + B only, not B + A)
                // to prevent redundancies
                if (body.get() > point.body) {
                    Collision::ResolvePair(*body, *point.body);
                }
            }
        }
    }

    for (auto& body : m_bodies) {
        body->calculate_velocity(dt);
        body->integrate_pos(dt, fps);
        body->reset_forces();
    }
}
