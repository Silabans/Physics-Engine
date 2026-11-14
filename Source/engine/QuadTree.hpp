#include <vector>
#include <memory>
#include "rigidbody.hpp"


struct Point {
    float x, y;
    RigidBody* body = nullptr;
};


struct AABB {
    float x, y; // Centre
    float hw, hh; // half-width and half-height (from the centre)

    // Check if a point is within a bounded box
    bool contains(const Point& p) const {
        return (p.x >= x - hw && p.x <= x + hw
        && p.y >= y - hh && p.y <= y + hh);
    }

    // Check if another box intersects the current box
    bool intersects(const AABB& other) const {
        return !(other.x - other.hw > x + hw ||
                other.x + other.hw < x - hw ||
                other.y - other.hh > y + hh ||
                other.y + other.hh < y - hh);
    }
};

class QuadTree {
private:
    static constexpr int CAPACITY = 4;

    AABB boundary;
    std::vector<Point> points; // all the points where there are game objects / bodies
    bool divided; // keeps track on whether this layer has been divided into four smaller boundaries

    // Smart pointers to all four quadrants (north-west, north-east, south-west, south-east)
    std::unique_ptr<QuadTree> nw;
    std::unique_ptr<QuadTree> ne;
    std::unique_ptr<QuadTree> sw;
    std::unique_ptr<QuadTree> se;

    // helper method of insert()
    // returns true if a point is successfully inserted into one of the quadrants, otherwise false
    bool insertToChildren(const Point& p);

public:
    // initialise the QuadTree
    QuadTree(AABB boundary) : boundary(boundary) {
        points.reserve(CAPACITY); // pre-allocates memory for the vector (enough for 4 elements)
    }

    // divides the current tree into 4 quadrants (can be used recursively)
    void subdivide();

    bool insert(const Point& point);

    #
    void query(const AABB& range, std::vector<Point>& found) const;
};