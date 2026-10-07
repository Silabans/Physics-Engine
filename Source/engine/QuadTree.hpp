#include <vector>
#include <memory>


struct Point {
    float x, y;
    void* userData = nullptr;
};


struct AABB {
    float x, y; // Centre
    float hw, hh; // half-width and half-height (from the centre)

    bool contains(const Point& p) const {
        return (p.x >= x - hw && p.x <= x + hw
        && p.y >= y - hh && p.y <= y + hh);
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
    bool insertToChildren(const Point& p) {
        if (nw->insert(p)) return true;
        if (ne->insert(p)) return true;
        if (sw->insert(p)) return true;
        if (se->insert(p)) return true;
        return false;
    }

public:
    // initialise the QuadTree
    QuadTree(AABB boundary) : boundary(boundary) {
        points.reserve(CAPACITY); // pre-allocates memory for the vector (enough for 4 elements)
    }

    // divides the current tree into 4 quadrants (can be used recursively)
    void subdivide() {
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

    bool insert(const Point& point) {
        if (!boundary.contains(point)) {
            return false;
        }

        // if the boundary is not full (number of points < CAPACITY) and the boundary has not
        // been divided into quadrants, store the new point in the points vector
        if (points.size() < CAPACITY & !divided) {
            points.push_back(point);
            return true;
        }

        if (!divided) {
            subdivide();
        }

        // if current boundary is full, delegate the new point to a child quadrant
        return insertToChildren(point);
    }

};