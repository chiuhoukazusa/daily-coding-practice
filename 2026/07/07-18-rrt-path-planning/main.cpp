/**
 * RRT (Rapidly-exploring Random Tree) Path Planning
 * 
 * Algorithm: Sample-based motion planning in continuous 2D space
 * - Builds a tree from start to goal by random sampling
 * - Each iteration: sample random point, find nearest tree node,
 *   extend toward sample (steer), add new node if collision-free
 * - Connects to goal when within reach
 * - Backtracking extracts the final path
 * 
 * Features:
 * - Multiple circular and rectangular obstacles
 * - Configurable step size and goal sampling bias
 * - Path smoothing via shortcut pruning
 * - PPM visualization with obstacles, tree, and final path
 * - Quantitative verification of path validity
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <fstream>
#include <cassert>
#include <limits>
#include <cstdlib>

// ============================================================
// Vector2D
// ============================================================
struct Vec2 {
    double x, y;
    Vec2(double x=0, double y=0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& o) const { return Vec2(x+o.x, y+o.y); }
    Vec2 operator-(const Vec2& o) const { return Vec2(x-o.x, y-o.y); }
    Vec2 operator*(double s) const { return Vec2(x*s, y*s); }
    double dot(const Vec2& o) const { return x*o.x + y*o.y; }
    double len() const { return std::sqrt(x*x + y*y); }
    double lenSq() const { return x*x + y*y; }
    Vec2 normalized() const { double l = len(); return l > 1e-9 ? Vec2(x/l, y/l) : Vec2(0,0); }
    double distTo(const Vec2& o) const { return (*this - o).len(); }
    double distToSq(const Vec2& o) const { return (*this - o).lenSq(); }
};

// ============================================================
// Obstacles
// ============================================================
struct CircleObstacle {
    Vec2 center;
    double radius;
};

struct RectObstacle {
    Vec2 min, max;  // axis-aligned
};

// ============================================================
// Collision detection
// ============================================================
bool pointInCircle(const Vec2& p, const CircleObstacle& c) {
    return p.distToSq(c.center) < c.radius * c.radius;
}

bool pointInRect(const Vec2& p, const RectObstacle& r) {
    return p.x >= r.min.x && p.x <= r.max.x && p.y >= r.min.y && p.y <= r.max.y;
}

// Check if a line segment (a to b) collides with a circle
bool segmentCollidesCircle(const Vec2& a, const Vec2& b, const CircleObstacle& c, double margin=0.0) {
    Vec2 d = b - a;
    Vec2 f = a - c.center;
    double r = c.radius + margin;
    
    double A = d.dot(d);
    double B = 2.0 * f.dot(d);
    double C = f.dot(f) - r * r;
    
    double discriminant = B*B - 4*A*C;
    if (discriminant < 0) return false;
    
    double sqrtD = std::sqrt(discriminant);
    double t1 = (-B - sqrtD) / (2*A);
    double t2 = (-B + sqrtD) / (2*A);
    
    // Intersection with segment if any t in [0,1]
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1) || (t1 < 0 && t2 > 1);
}

// Check if line segment collides with any rectangle
bool segmentCollidesRect(const Vec2& a, const Vec2& b, const RectObstacle& r, double margin=0.0) {
    RectObstacle expanded = {
        Vec2(r.min.x - margin, r.min.y - margin),
        Vec2(r.max.x + margin, r.max.y + margin)
    };
    
    // Both endpoints inside
    if (a.x >= expanded.min.x && a.x <= expanded.max.x && a.y >= expanded.min.y && a.y <= expanded.max.y) return true;
    if (b.x >= expanded.min.x && b.x <= expanded.max.x && b.y >= expanded.min.y && b.y <= expanded.max.y) return true;
    
    // Check intersection with 4 edges
    // Horizontal edges
    for (double y : {expanded.min.y, expanded.max.y}) {
        if (std::abs(a.y - b.y) > 1e-9) {
            double t = (y - a.y) / (b.y - a.y);
            if (t >= 0 && t <= 1) {
                double x = a.x + t * (b.x - a.x);
                if (x >= expanded.min.x && x <= expanded.max.x) return true;
            }
        }
    }
    // Vertical edges
    for (double xv : {expanded.min.x, expanded.max.x}) {
        if (std::abs(a.x - b.x) > 1e-9) {
            double t = (xv - a.x) / (b.x - a.x);
            if (t >= 0 && t <= 1) {
                double y = a.y + t * (b.y - a.y);
                if (y >= expanded.min.y && y <= expanded.max.y) return true;
            }
        }
    }
    return false;
}

bool isCollisionFree(const Vec2& a, const Vec2& b,
                     const std::vector<CircleObstacle>& circles,
                     const std::vector<RectObstacle>& rects,
                     double margin = 0.1) {
    for (const auto& c : circles)
        if (segmentCollidesCircle(a, b, c, margin)) return false;
    for (const auto& r : rects)
        if (segmentCollidesRect(a, b, r, margin)) return false;
    return true;
}

bool isPointCollisionFree(const Vec2& p,
                          const std::vector<CircleObstacle>& circles,
                          const std::vector<RectObstacle>& rects,
                          double margin = 0.1) {
    for (const auto& c : circles)
        if (pointInCircle(p, c)) return false;
    for (const auto& r : rects)
        if (pointInRect(p, r)) return false;
    // Check margin
    for (const auto& c : circles)
        if (p.distTo(c.center) < c.radius + margin) return false;
    for (const auto& r : rects) {
        RectObstacle exp = {
            Vec2(r.min.x - margin, r.min.y - margin),
            Vec2(r.max.x + margin, r.max.y + margin)
        };
        if (pointInRect(p, exp)) return false;
    }
    return true;
}

// ============================================================
// RRT Node
// ============================================================
struct RRTNode {
    Vec2 pos;
    int parent;
    RRTNode(Vec2 p, int parent=-1) : pos(p), parent(parent) {}
};

// ============================================================
// RRT Algorithm
// ============================================================
struct RRTResult {
    std::vector<RRTNode> tree;
    std::vector<int> path;  // indices in tree, from start to goal
    bool success;
    int iterations;
};

RRTResult rrt(const Vec2& start, const Vec2& goal,
              const std::vector<CircleObstacle>& circles,
              const std::vector<RectObstacle>& rects,
              double xmin, double xmax, double ymin, double ymax,
              double stepSize = 0.5,
              double goalBias = 0.05,
              double goalRadius = 0.5,
              int maxIterations = 5000) {
    
    std::mt19937 rng(42);  // fixed seed for reproducibility
    std::uniform_real_distribution<double> distX(xmin, xmax);
    std::uniform_real_distribution<double> distY(ymin, ymax);
    std::uniform_real_distribution<double> dist01(0.0, 1.0);
    
    RRTResult result;
    result.tree.push_back(RRTNode(start, -1));
    result.success = false;
    
    for (int iter = 0; iter < maxIterations; iter++) {
        // Sample random point (with goal bias)
        Vec2 sample;
        if (dist01(rng) < goalBias) {
            sample = goal;
        } else {
            sample = Vec2(distX(rng), distY(rng));
        }
        
        // Find nearest node
        int nearest = 0;
        double bestDist = sample.distToSq(result.tree[0].pos);
        for (size_t i = 1; i < result.tree.size(); i++) {
            double d = sample.distToSq(result.tree[i].pos);
            if (d < bestDist) {
                bestDist = d;
                nearest = (int)i;
            }
        }
        
        // Steer toward sample
        Vec2 nearPos = result.tree[nearest].pos;
        Vec2 dir = sample - nearPos;
        double dist = dir.len();
        if (dist < 1e-9) continue;
        
        Vec2 newPos;
        if (dist > stepSize) {
            newPos = nearPos + dir.normalized() * stepSize;
        } else {
            newPos = sample;
        }
        
        // Collision check
        if (!isPointCollisionFree(newPos, circles, rects)) continue;
        if (!isCollisionFree(nearPos, newPos, circles, rects)) continue;
        
        // Add node
        int newNodeIdx = (int)result.tree.size();
        result.tree.push_back(RRTNode(newPos, nearest));
        
        // Check if we reached the goal
        if (newPos.distTo(goal) < goalRadius) {
            // Connect to goal
            if (isCollisionFree(newPos, goal, circles, rects)) {
                result.tree.push_back(RRTNode(goal, newNodeIdx));
                result.success = true;
                result.iterations = iter + 1;
                
                // Extract path (backtrack)
                int idx = (int)result.tree.size() - 1;
                while (idx >= 0) {
                    result.path.push_back(idx);
                    idx = result.tree[idx].parent;
                }
                std::reverse(result.path.begin(), result.path.end());
                return result;
            }
        }
    }
    
    result.iterations = maxIterations;
    return result;
}

// ============================================================
// Path Smoothing (shortcut pruning)
// ============================================================
std::vector<Vec2> smoothPath(const std::vector<Vec2>& path,
                              const std::vector<CircleObstacle>& circles,
                              const std::vector<RectObstacle>& rects,
                              int maxAttempts = 100) {
    if (path.size() <= 2) return path;
    
    std::mt19937 rng(123);
    std::vector<Vec2> smoothed = path;
    
    for (int attempt = 0; attempt < maxAttempts; attempt++) {
        if (smoothed.size() <= 2) break;
        
        // Pick two random non-adjacent indices
        int n = (int)smoothed.size();
        std::uniform_int_distribution<int> dist(0, n-1);
        int i = dist(rng);
        int j = dist(rng);
        if (i > j) std::swap(i, j);
        if (j - i <= 1) continue;  // must be non-adjacent
        
        // Check if direct connection is collision-free
        if (isCollisionFree(smoothed[i], smoothed[j], circles, rects)) {
            // Remove intermediate points
            smoothed.erase(smoothed.begin() + i + 1, smoothed.begin() + j);
        }
    }
    
    return smoothed;
}

// ============================================================
// PPM Output
// ============================================================
void writePPM(const std::string& filename, int w, int h,
              const std::vector<Vec2>& treeNodes, const std::vector<int>& treeParents,
              const std::vector<Vec2>& path,
              const std::vector<Vec2>& smoothedPath,
              const Vec2& start, const Vec2& goal,
              const std::vector<CircleObstacle>& circles,
              const std::vector<RectObstacle>& rects,
              double xmin, double xmax, double ymin, double ymax) {
    
    // Background: white
    std::vector<unsigned char> img(w * h * 3, 255);
    
    auto setPixel = [&](int px, int py, unsigned char r, unsigned char g, unsigned char b) {
        if (px < 0 || px >= w || py < 0 || py >= h) return;
        int idx = (py * w + px) * 3;
        img[idx] = r; img[idx+1] = g; img[idx+2] = b;
    };
    
    // Coordinate transform: world -> pixel (y flipped for PPM)
    auto worldToPixel = [&](const Vec2& p) -> std::pair<int,int> {
        int px = (int)((p.x - xmin) / (xmax - xmin) * w);
        int py = (int)((1.0 - (p.y - ymin) / (ymax - ymin)) * h);
        return {px, py};
    };
    
    // Draw obstacles
    for (const auto& c : circles) {
        auto [cx, cy] = worldToPixel(c.center);
        double crPixels = c.radius / (xmax - xmin) * w;
        int ir = (int)std::ceil(crPixels);
        for (int dy = -ir; dy <= ir; dy++) {
            for (int dx = -ir; dx <= ir; dx++) {
                if (dx*dx + dy*dy <= crPixels*crPixels) {
                    setPixel(cx+dx, cy+dy, 100, 100, 120);
                }
            }
        }
    }
    
    for (const auto& r : rects) {
        auto [rx1, ry1] = worldToPixel(r.min);
        auto [rx2, ry2] = worldToPixel(r.max);
        int x1 = std::min(rx1, rx2), x2 = std::max(rx1, rx2);
        int y1 = std::min(ry1, ry2), y2 = std::max(ry1, ry2);
        for (int y = y1; y <= y2; y++)
            for (int x = x1; x <= x2; x++)
                setPixel(x, y, 100, 100, 120);
    }
    
    // Draw tree edges
    for (size_t i = 1; i < treeNodes.size(); i++) {
        int parent = treeParents[i];
        if (parent < 0) continue;
        auto [x1, y1] = worldToPixel(treeNodes[i]);
        auto [x2, y2] = worldToPixel(treeNodes[parent]);
        
        // Bresenham
        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        int cx = x1, cy = y1;
        while (true) {
            setPixel(cx, cy, 200, 200, 220);
            if (cx == x2 && cy == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
    }
    
    // Draw original path (blue)
    for (size_t i = 1; i < path.size(); i++) {
        auto [x1, y1] = worldToPixel(path[i-1]);
        auto [x2, y2] = worldToPixel(path[i]);
        
        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        int cx = x1, cy = y1;
        while (true) {
            setPixel(cx-1, cy, 0, 0, 200);
            setPixel(cx, cy, 30, 50, 200);
            setPixel(cx+1, cy, 0, 0, 200);
            if (cx == x2 && cy == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
    }
    
    // Draw smoothed path (red, on top)
    for (size_t i = 1; i < smoothedPath.size(); i++) {
        auto [x1, y1] = worldToPixel(smoothedPath[i-1]);
        auto [x2, y2] = worldToPixel(smoothedPath[i]);
        
        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;
        int cx = x1, cy = y1;
        while (true) {
            setPixel(cx-1, cy, 220, 0, 0);
            setPixel(cx, cy, 255, 30, 30);
            setPixel(cx+1, cy, 220, 0, 0);
            if (cx == x2 && cy == y2) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; cx += sx; }
            if (e2 <= dx) { err += dx; cy += sy; }
        }
    }
    
    // Draw start and goal
    auto [sx, sy] = worldToPixel(start);
    auto [gx, gy] = worldToPixel(goal);
    for (int dy = -5; dy <= 5; dy++)
        for (int dx = -5; dx <= 5; dx++)
            if (dx*dx + dy*dy <= 25)
                setPixel(sx+dx, sy+dy, 0, 200, 0);
    for (int dy = -5; dy <= 5; dy++)
        for (int dx = -5; dx <= 5; dx++)
            if (dx*dx + dy*dy <= 25)
                setPixel(gx+dx, gy+dy, 255, 100, 0);
    
    // Write PPM
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << w << " " << h << "\n255\n";
    out.write((const char*)img.data(), img.size());
}

// ============================================================
// Main
// ============================================================
int main() {
    // World bounds
    double xmin = -1.0, xmax = 14.0;
    double ymin = -1.0, ymax = 12.0;
    
    // Start and goal
    Vec2 start(0.0, 0.5);
    Vec2 goal(13.0, 10.5);
    
    // Obstacles
    std::vector<CircleObstacle> circles = {
        {Vec2(3.0, 3.0), 1.2},
        {Vec2(5.5, 7.0), 1.5},
        {Vec2(8.0, 2.0), 1.0},
        {Vec2(10.0, 5.5), 0.8},
        {Vec2(2.0, 8.0), 1.1},
        {Vec2(7.0, 9.0), 1.3},
        {Vec2(12.0, 8.0), 0.9},
    };
    
    std::vector<RectObstacle> rects = {
        {Vec2(4.0, 0.5), Vec2(5.5, 2.0)},
        {Vec2(9.0, 7.0), Vec2(11.0, 9.0)},
    };
    
    // Run RRT
    std::cout << "=== RRT Path Planning ===" << std::endl;
    std::cout << "Start: (" << start.x << ", " << start.y << ")" << std::endl;
    std::cout << "Goal:  (" << goal.x << ", " << goal.y << ")" << std::endl;
    std::cout << "Obstacles: " << circles.size() << " circles, " << rects.size() << " rectangles" << std::endl;
    
    auto result = rrt(start, goal, circles, rects, xmin, xmax, ymin, ymax);
    
    std::cout << "\n=== RRT Results ===" << std::endl;
    std::cout << "Success: " << (result.success ? "YES" : "NO") << std::endl;
    std::cout << "Iterations: " << result.iterations << std::endl;
    std::cout << "Tree nodes: " << result.tree.size() << std::endl;
    std::cout << "Path length: " << result.path.size() << " nodes" << std::endl;
    
    if (!result.success) {
        std::cerr << "ERROR: RRT failed to find a path!" << std::endl;
        return 1;
    }
    
    // Extract path as Vec2
    std::vector<Vec2> rawPath;
    for (int idx : result.path) {
        rawPath.push_back(result.tree[idx].pos);
    }
    
    // Compute path length
    double rawLen = 0;
    for (size_t i = 1; i < rawPath.size(); i++)
        rawLen += rawPath[i].distTo(rawPath[i-1]);
    
    // Smooth path
    auto smoothed = smoothPath(rawPath, circles, rects, 200);
    double smoothLen = 0;
    for (size_t i = 1; i < smoothed.size(); i++)
        smoothLen += smoothed[i].distTo(smoothed[i-1]);
    
    std::cout << "\n=== Path Metrics ===" << std::endl;
    std::cout << "Original path length: " << rawLen << std::endl;
    std::cout << "Smoothed path length: " << smoothLen << std::endl;
    std::cout << "Improvement: " << (1.0 - smoothLen/rawLen)*100 << "% shorter" << std::endl;
    std::cout << "Smoothed path nodes: " << smoothed.size() << " (from " << rawPath.size() << ")" << std::endl;
    
    // Quantitative verification
    std::cout << "\n=== Verification ===" << std::endl;
    
    // 1. Start and end match
    double startErr = smoothed.front().distTo(start);
    double goalErr = smoothed.back().distTo(goal);
    std::cout << "Start error: " << startErr << " (must be 0)" << std::endl;
    std::cout << "Goal error: " << goalErr << " (must be 0)" << std::endl;
    assert(startErr < 1e-6);
    assert(goalErr < 1e-6);
    std::cout << "  ✓ Start/goal position correct" << std::endl;
    
    // 2. Every segment is collision-free
    bool allFree = true;
    for (size_t i = 1; i < smoothed.size(); i++) {
        if (!isCollisionFree(smoothed[i-1], smoothed[i], circles, rects, 0.0)) {
            std::cout << "  ✗ Segment " << i-1 << "->" << i << " collides!" << std::endl;
            allFree = false;
        }
    }
    assert(allFree);
    std::cout << "  ✓ All " << (smoothed.size()-1) << " path segments collision-free" << std::endl;
    
    // 3. Path is monotonic toward goal (rough check: euclidean distance decreases)
    double startDist = start.distTo(goal);
    assert(smoothLen >= startDist);
    std::cout << "  ✓ Path length (" << smoothLen << ") >= direct distance (" << startDist << ")" << std::endl;
    
    // 4. Tree has reasonable coverage (tree nodes > 0)
    assert(result.tree.size() > 10);
    std::cout << "  ✓ Tree has sufficient nodes (" << result.tree.size() << ")" << std::endl;
    
    // 5. Path doesn't go through obstacles (point check every 0.1 units)
    bool allPointsFree = true;
    for (size_t i = 1; i < smoothed.size() && allPointsFree; i++) {
        Vec2 dir = smoothed[i] - smoothed[i-1];
        double len = dir.len();
        Vec2 step = dir.normalized() * 0.05;
        for (double d = 0; d <= len; d += 0.05) {
            Vec2 p = smoothed[i-1] + step * (d / 0.05);
            if (!isPointCollisionFree(p, circles, rects, 0.0)) {
                std::cout << "  ✗ Point (" << p.x << "," << p.y << ") inside obstacle!" << std::endl;
                allPointsFree = false;
                break;
            }
        }
    }
    assert(allPointsFree);
    std::cout << "  ✓ All interpolated path points collision-free" << std::endl;
    
    std::cout << "\n=== ALL VERIFICATIONS PASSED ===" << std::endl;
    
    // Extract tree node positions and parents for rendering
    std::vector<Vec2> treeNodes;
    std::vector<int> treeParents;
    for (const auto& node : result.tree) {
        treeNodes.push_back(node.pos);
        treeParents.push_back(node.parent);
    }
    
    // Render
    int w = 1400, h = 1200;
    writePPM("rrt_output.ppm", w, h, treeNodes, treeParents, rawPath, smoothed,
             start, goal, circles, rects, xmin, xmax, ymin, ymax);
    std::cout << "\nRendered rrt_output.ppm (" << w << "x" << h << ")" << std::endl;
    
    return 0;
}
