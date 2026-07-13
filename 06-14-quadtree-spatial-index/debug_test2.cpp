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
    
    int getNodeCount() const {
        int count = 1;
        if (divided) count += nw->getNodeCount() + ne->getNodeCount() + sw->getNodeCount() + se->getNodeCount();
        return count;
    }
};

// Copy of validation from main
std::vector<Point> bruteRangeQuery(const std::vector<Point>& allPoints, const Point& center, float radius) {
    std::vector<Point> result;
    for (const auto& p : allPoints) {
        if (p.dist(center) <= radius) result.push_back(p);
    }
    return result;
}

bool validateResults(const std::vector<Point>& qt, const std::vector<Point>& brute) {
    if (qt.size() != brute.size()) return false;
    for (size_t i = 0; i < qt.size(); i++) {
        if (std::abs(qt[i].x - brute[i].x) > 0.001f || std::abs(qt[i].y - brute[i].y) > 0.001f) return false;
    }
    return true;
}

int main() {
    std::mt19937 rng2(123);
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    std::vector<Point> valPoints;
    for (int i = 0; i < 500; i++) valPoints.emplace_back(dist(rng2), dist(rng2));
    
    AABB world(0.5f, 0.5f, 0.5f);
    Quadtree qtVal(world);
    for (const auto& p : valPoints) qtVal.insert(p);
    
    std::cout << "Counts: val=" << valPoints.size() << " qt=" << qtVal.countAll() << std::endl;
    std::cout << "Nodes: " << qtVal.getNodeCount() << std::endl;
    
    // Test all 20 range queries
    bool allPass = true;
    for (int i = 0; i < 20; i++) {
        Point c(dist(rng2), dist(rng2));
        float r = dist(rng2) * 0.3f;
        std::vector<Point> qtRes, bruteRes;
        qtVal.rangeQuery(c, r, qtRes);
        bruteRes = bruteRangeQuery(valPoints, c, r);
        
        // Validate by size first
        if (qtRes.size() != bruteRes.size()) {
            allPass = false;
            std::cout << "FAIL size: i=" << i << " center=(" << c.x << "," << c.y 
                      << ") r=" << r << " qt=" << qtRes.size() << " brute=" << bruteRes.size() << std::endl;
            
            // Find missing/extra points
            if (qtRes.size() < bruteRes.size()) {
                // Some points missing - which ones?
                for (const auto& bp : bruteRes) {
                    bool found = false;
                    for (const auto& qp : qtRes) {
                        if (std::abs(qp.x-bp.x)<0.001f && std::abs(qp.y-bp.y)<0.001f) { found=true; break; }
                    }
                    if (!found) {
                        std::cout << "  Missing: (" << bp.x << "," << bp.y << ") dist=" << bp.dist(c) << std::endl;
                    }
                }
            } else {
                // Extra points
                for (const auto& qp : qtRes) {
                    bool found = false;
                    for (const auto& bp : bruteRes) {
                        if (std::abs(qp.x-bp.x)<0.001f && std::abs(qp.y-bp.y)<0.001f) { found=true; break; }
                    }
                    if (!found) {
                        std::cout << "  Extra: (" << qp.x << "," << qp.y << ") dist=" << qp.dist(c) << std::endl;
                    }
                }
            }
            break;
        }
    }
    std::cout << "All passing after fix: " << (allPass ? "yes" : "no") << std::endl;
    return 0;
}
