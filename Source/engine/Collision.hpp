#pragma once


class Collision {
private:
    static Vector2D CalculateVrel(const RigidBody& a, const RigidBody& b);
    static void ResolveCircleCollision(RigidBody& a, RigidBody& b);
    static void ResolveBoxCollision(RigidBody& a, RigidBody& b, SATresult sat);
    static void ResolveBoxCircleCollision(RigidBody& a, RigidBody& b);
public:
    static void ResolvePair(RigidBody& a, RigidBody& b);
};


// no longer in use
void resolve_boundaries(RigidBody& body) {
    if (body.getInverseMass() == 0.0f) return;

    // boundary collisions
    const Shape* shape = body.getShape();
    float halfW; float halfH;

    if (shape->type == ShapeType::CIRCLE) {
        const Circle* circle = static_cast<const Circle*>(shape);
        halfW = halfH = circle->radius;
    }
    else if (shape->type == ShapeType::BOX) {
        const Box* box = static_cast<const Box*>(shape);
        
        // find the extreme points along the x and y axes
        std::array<Vector2D, 4> verts = box->get_vertices(body.getPosition());
        float minX = verts[0].x; float maxX = verts[0].x;
        float minY = verts[0].y; float maxY = verts[0].y;

        for (int i = 1; i < 4; ++i) { // from the second to fourth vertex (the first is the default vertex)
            minX = std::min(minX, verts[i].x);
            minY = std::min(minY, verts[i].y);
            maxX = std::max(maxX, verts[i].x);
            maxY = std::max(maxY, verts[i].y);
        }

        // measure how far the furthest point is (along the x or y axis) from the centre of the box
        halfH = maxY - body.getPositionY(); 
        halfW = maxX - body.getPositionX();
    }


    float vx = body.getVelocityX();
    float vy = body.getVelocityY();

    int x = body.getPositionX();
    int y = body.getPositionY();

    float restitution = 0.7f;
    float restThreshold = 50.0f;

    if (x + halfW > 800.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(800.0f - halfW);
        } else body.setVelocityX(-std::abs(vx) * restitution);

    } else if (x - halfW < 0.0f) {
        if (std::abs(vx) < restThreshold) {
            body.setVelocityX(0.0f);
            body.setPositionX(halfW);
        } else body.setVelocityX(std::abs(vx) * restitution);
    }

    if (y - halfH < 0.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(halfH);
        } else body.setVelocityY(std::abs(vy) * restitution);

    } else if (y + halfH > 600.0f) {
        if (std::abs(vy) < restThreshold) {
            body.setVelocityY(0.0f);
            body.setPositionY(600.0f - halfH);
        } else body.setVelocityY(-std::abs(vy) * restitution);

        float surfaceFriction = 0.95f;
        // angular vel decreases due to frictional forces
        body.setAngularVel(body.getAngularVel() * surfaceFriction);
    }
}