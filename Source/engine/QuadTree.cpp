#include "QuadTree.hpp"


QuadTree::QuadTree(AABB boundary) : boundary(boundary) {};

bool QuadTree::insertToChildren(const Point& p) {
    if (nw->insert(p)) return true;
    if (ne->insert(p)) return true;
    if (sw->insert(p)) return true;
    if (se->insert(p)) return true;
    return false;
}

void QuadTree::subdivide() {
    // Centre coordinates of the parent
    float x = boundary.x;
    float y = boundary.y;
    float subHw = boundary.hw * 0.5f;
    float subHh = boundary.hh * 0.5f;

    AABB nwBox{x - subHw, y - subHh, subHw, subHh};
    AABB neBox{x + subHw, y - subHh, subHw, subHh};
    AABB swBox{x - subHw, y + subHh, subHw, subHh};
    AABB seBox{x + subHw, y + subHh, subHw, subHh};

    // instantiate the quadrants
    nw = std::make_unique<QuadTree>(nwBox);
    ne = std::make_unique<QuadTree>(neBox);
    sw = std::make_unique<QuadTree>(swBox);
    se = std::make_unique<QuadTree>(seBox);

    divided = true;

    // distribute the stored points to the child quadrants
    for (const auto& p : points) {
        insertToChildren(p);
    }
    points.clear(); // the parent no longer holds the points (delegated to its children)
}

bool QuadTree::insert(const Point& point) {
    if (!boundary.contains(point)) {
        return false;
    }

    // if the boundary is not full (number of points < CAPACITY) and the boundary has not
    // been divided into quadrants, store the new point in the points vector
    if (points.size() < CAPACITY && !divided) {
        points.push_back(point);
        return true;
    }

    if (!divided) {
        subdivide();
    }

    // if current boundary is full, delegate the new point to a child quadrant
    return insertToChildren(point);
}

void QuadTree::query(const AABB& range, std::vector<Point>& found) const {
    if (!boundary.intersects(range)) return;

    for (const auto& p : points) {
        if (range.contains(p)) {
            found.push_back(p);
        }
    }

    if (divided) {
        nw->query(range, found);
        ne->query(range, found);
        sw->query(range, found);
        se->query(range, found);
    }
};
