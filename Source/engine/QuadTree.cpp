#include "QuadTree.hpp"


QuadTree::QuadTree(AABB boundary) : boundary(boundary) {};

// Insert the point inside a box to its most appropriate child 
// (where the point lies)
bool QuadTree::insertToChildren(const Point& p) {
    if (nw->insert(p)) return true;
    if (ne->insert(p)) return true;
    if (sw->insert(p)) return true;
    if (se->insert(p)) return true;
    return false;
}

// Divides the current box (the parent) into 4 quadrants (its children),
// then allocates the points that the parent contains into its children
// by calling QuadTree::insertToChildren
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

// Insert a point into the current box if its capacity has not been exceeded.
// Otherwise, the point will be inserted to one of the box's children
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

// Locate and determine where a point lies (which box), 
// then push the point into a 'found' vector
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
