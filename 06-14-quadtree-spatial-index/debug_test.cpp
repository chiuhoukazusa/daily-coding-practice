#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>
#include <cfloat>
#include <queue>
#include <iomanip>
#include <cassert>
#include <memory>
#include <functional>

struct Point {
    float x, y;
    Point() : x(0), y(0) {}
    Point(float x, float y) : x(x), y(y) {}
    float dist(const Point& other) const {
        float dx = x - other.x, dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

struct AABB {
    float cx, cy;
    float half;
    AABB(float cx, float cy, float half) : cx(cx), cy(cy), half(half) {}
    bool contains(const Point& p) const {
        return p.x >= cx - half && p.x <= cx + half &&
               p.y >= cy - half && p.y <= cy + half;
    }
    bool intersectsCircle(const Point& center, float radius) const {
        float closestX = std::max(cx - half, std::min(center.x, cx + half));
        float closestY = std::max(cy - half, std::min(center.y, cy + half));
        float dx = center.x - closestX;
        float dy = center.y - closestY;
        return dx*dx + dy*dy <= radius*radius;
    }
    float sqDist(const Point& p) const {
        float dx = std::max(0.0f, std::abs(p.x - cx) - half);
        float dy = std::max(0.0f, std::abs(p.y - cy) - half);
        return dx*dx + dy*dy;
    }
};

class Quadtree {
public:
    static constexpr int MAX_POINTS = 4;
    static constexpr int MAX_DEPTH = 8;
private:
    AABB boundary;
    std::vector<Point> points;
    std::unique_ptr<Quadtree> nw, ne, sw, se;
    int depth;
    bool divided;
    
    void subdivide() {
        float qh = boundary.half * 0.5f;
        nw = std::make_unique<Quadtree>(AABB(boundary.cx - qh, boundary.cy + qh, qh), depth + 1);
        ne = std::make_unique<Quadtree>(AABB(boundary.cx + qh, boundary.cy + qh, qh), depth + 1);
        sw = std::make_unique<Quadtree>(AABB(boundary.cx - qh, boundary.cy - qh, qh), depth + 1);
        se = std::make_unique<Quadtree>(AABB(boundary.cx + qh, boundary.cy - qh, qh), depth + 1);
        divided = true;
        
        std::vector<Point> keep;
        for (const auto& p : points) {
            bool inserted = false;
            if (nw->boundary.contains(p)) { nw->insert(p); inserted = true; }
            else if (ne->boundary.contains(p)) { ne->insert(p); inserted = true; }
            else if (sw->boundary.contains(p)) { sw->insert(p); inserted = true; }
            else if (se->boundary.contains(p)) { se->insert(p); inserted = true; }
            if (!inserted) keep.push_back(p);
        }
        points = std::move(keep);
    }
    
    bool insertIntoChild(const Point& p) {
        if (nw->boundary.contains(p)) { nw->insert(p); return true; }
        if (ne->boundary.contains(p)) { ne->insert(p); return true; }
        if (sw->boundary.contains(p)) { sw->insert(p); return true; }
        if (se->boundary.contains(p)) { se->insert(p); return true; }
        return false;
    }
    
public:
    Quadtree(const AABB& b, int d = 0) : boundary(b), depth(d), divided(false) {}
    
    bool insert(const Point& p) {
        if (!boundary.contains(p)) return false;
        if (!divided && (int)points.size() < MAX_POINTS) {
            points.push_back(p);
            return true;
        }
        if (!divided && depth < MAX_DEPTH) {
            subdivide();
        }
        if (divided) {
            return insertIntoChild(p);
        }
        points.push_back(p);
        return true;
    }
    
    void rangeQuery(const Point& center, float radius, std::vector<Point>& result) const {
        if (!boundary.intersectsCircle(center, radius)) return;
        for (const auto& p : points) {
            if (p.dist(center) <= radius) result.push_back(p);
        }
        if (divided) {
            nw->rangeQuery(center, radius, result);
            ne->rangeQuery(center, radius, result);
            sw->rangeQuery(center, radius, result);
            se->rangeQuery(center, radius, result);
        }
    }
    
    int countAll() const {
        int c = (int)points.size();
        if (divided) c += nw->countAll() + ne->countAll() + sw->countAll() + se->countAll();
        return c;
    }
};

int main() {
    // Reproduce failing test
    std::mt19937 rng(123);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    std::vector<Point> valPoints;
    for (int i = 0; i < 500; i++) {
        valPoints.emplace_back(dist(rng), dist(rng));
    }
    
    AABB world(0.5f, 0.5f, 0.5f);
    Quadtree qt(world);
    for (const auto& p : valPoints) qt.insert(p);
    
    std::cout << "Total points: " << valPoints.size() << std::endl;
    std::cout << "Quadtree count: " << qt.countAll() << std::endl;
    
    if (qt.countAll() != (int)valPoints.size()) {
        std::cout << "MISSING POINTS! Lost " << (valPoints.size() - qt.countAll()) << " points\n";
    }
    
    // Test specific failing case
    Point c(0.18f, 0.21f);
    float r = 0.12f;
    std::vector<Point> qtRes;
    qt.rangeQuery(c, r, qtRes);
    
    std::vector<Point> bruteRes;
    for (const auto& p : valPoints) {
        if (p.dist(c) <= r) bruteRes.push_back(p);
    }
    
    std::cout << "Range query: qt=" << qtRes.size() << " brute=" << bruteRes.size() << std::endl;
    
    return 0;
}
