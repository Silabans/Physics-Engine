#pragma once
#include "PhysicsWorld.hpp"
#include "rigidbody.hpp"
#include <raylib.h>


struct GameObject {
    RigidBody* body;
    Color color;

    GameObject(Rigidbody* b, Color c) : body(b), color(c) {}
};