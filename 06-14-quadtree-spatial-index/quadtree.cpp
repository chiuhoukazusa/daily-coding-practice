/**
 * 每日编程实践 2026-06-14: Quadtree 空间分区索引
 * 
 * 技术点:
 * - Quadtree数据结构 (区域四叉树)
 * - 点插入 (最大深度/容量限制分裂)
 * - 范围查询 (圆形区域内的点搜索)
 * - 最近邻搜索 (K-Nearest Neighbors)
 * - 量化性能验证: 与暴力搜索的速度对比
 * 
 * 量化验证指标:
 * - 不同点数下的范围查询耗时对比
 * - 不同点数下的KNN查询耗时对比
 * - 平均加速比
 * - 查询结果正确性验证
 */

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

// ============================ Point ============================
struct Point {
    float x, y;
    Point() : x(0), y(0) {}
    Point(float x, float y) : x(x), y(y) {}
    
    float dist(const Point& other) const {
        float dx = x - other.x, dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

// ============================ AABB ============================
struct AABB {
    float cx, cy;  // center
    float half;    // half-size (square)
    
    AABB() : cx(0), cy(0), half(0) {}
    AABB(float cx, float cy, float half) : cx(cx), cy(cy), half(half) {}
    
    bool contains(const Point& p) const {
        return p.x >= cx - half && p.x <= cx + half &&
               p.y >= cy - half && p.y <= cy + half;
    }
    
    bool intersects(const AABB& other) const {
        return !(std::abs(cx - other.cx) > half + other.half ||
                 std::abs(cy - other.cy) > half + other.half);
    }
    
    // Check if circle (center, radius) intersects this AABB
    bool intersectsCircle(const Point& center, float radius) const {
        float closestX = std::max(cx - half, std::min(center.x, cx + half));
        float closestY = std::max(cy - half, std::min(center.y, cy + half));
        float dx = center.x - closestX;
        float dy = center.y - closestY;
        return dx*dx + dy*dy <= radius*radius;
    }
    
    // Squared distance from point to AABB
    float sqDist(const Point& p) const {
        float dx = std::max(0.0f, std::abs(p.x - cx) - half);
        float dy = std::max(0.0f, std::abs(p.y - cy) - half);
        return dx*dx + dy*dy;
    }
};

// ============================ Quadtree ============================
class Quadtree {
public:
    static constexpr int MAX_POINTS = 4;       // Capacity before split
    static constexpr int MAX_DEPTH = 8;         // Max subdivision depth
    
private:
    AABB boundary;
    std::vector<Point> points;
    std::unique_ptr<Quadtree> nw, ne, sw, se;  // children
    int depth;
    bool divided;
    
    void subdivide() {
        float qh = boundary.half * 0.5f;
        nw = std::make_unique<Quadtree>(AABB(boundary.cx - qh, boundary.cy + qh, qh), depth + 1);
        ne = std::make_unique<Quadtree>(AABB(boundary.cx + qh, boundary.cy + qh, qh), depth + 1);
        sw = std::make_unique<Quadtree>(AABB(boundary.cx - qh, boundary.cy - qh, qh), depth + 1);
        se = std::make_unique<Quadtree>(AABB(boundary.cx + qh, boundary.cy - qh, qh), depth + 1);
        divided = true;
        
        // Re-insert existing points into children; keep points that span boundaries
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
        // Try each child in order, keep here if cross-boundary
        if (nw->boundary.contains(p)) { nw->insert(p); return true; }
        if (ne->boundary.contains(p)) { ne->insert(p); return true; }
        if (sw->boundary.contains(p)) { sw->insert(p); return true; }
        if (se->boundary.contains(p)) { se->insert(p); return true; }
        return false;
    }
    
public:
    Quadtree(const AABB& b, int d = 0) 
        : boundary(b), depth(d), divided(false) {}
    
    bool insert(const Point& p) {
        if (!boundary.contains(p)) return false;
        
        if (!divided && points.size() < MAX_POINTS) {
            points.push_back(p);
            return true;
        }
        
        if (!divided && depth < MAX_DEPTH) {
            subdivide();
        }
        
        if (divided) {
            return insertIntoChild(p);
        }
        
        // Can't subdivide further, store here
        points.push_back(p);
        return true;
    }
    
    // ==================== Range Query ====================
    void rangeQuery(const Point& center, float radius, std::vector<Point>& result) const {
        if (!boundary.intersectsCircle(center, radius)) return;
        
        for (const auto& p : points) {
            if (p.dist(center) <= radius) {
                result.push_back(p);
            }
        }
        
        if (divided) {
            nw->rangeQuery(center, radius, result);
            ne->rangeQuery(center, radius, result);
            sw->rangeQuery(center, radius, result);
            se->rangeQuery(center, radius, result);
        }
    }
    
    int rangeQueryCount(const Point& center, float radius) const {
        if (!boundary.intersectsCircle(center, radius)) return 0;
        
        int count = 0;
        for (const auto& p : points) {
            if (p.dist(center) <= radius) count++;
        }
        
        if (divided) {
            count += nw->rangeQueryCount(center, radius);
            count += ne->rangeQueryCount(center, radius);
            count += sw->rangeQueryCount(center, radius);
            count += se->rangeQueryCount(center, radius);
        }
        return count;
    }
    
    // ==================== KNN Query ====================
    struct KnnCandidate {
        Point point;
        float dist;
        bool operator<(const KnnCandidate& other) const {
            return dist < other.dist;
        }
    };
    
    void knnQuery(const Point& target, int k, std::vector<Point>& result) const {
        // Max-heap for top-k
        std::priority_queue<KnnCandidate> heap;
        float searchRadius = FLT_MAX;
        
        // Sort children by distance for better pruning
        knnRecursive(target, k, heap, searchRadius);
        
        // Extract results in ascending order
        result.clear();
        std::vector<KnnCandidate> sorted;
        while (!heap.empty()) {
            sorted.push_back(heap.top());
            heap.pop();
        }
        std::reverse(sorted.begin(), sorted.end());
        for (const auto& c : sorted) {
            result.push_back(c.point);
        }
    }
    
private:
    void knnRecursive(const Point& target, int k, 
                      std::priority_queue<KnnCandidate>& heap,
                      float& bestDist) const {
        // Prune: if AABB is farther than current k-th, skip
        if (!heap.empty() && boundary.sqDist(target) > bestDist * bestDist) {
            return;
        }
        
        // Check points in this node
        for (const auto& p : points) {
            float d = p.dist(target);
            if (heap.size() < (size_t)k) {
                heap.push({p, d});
                if (heap.size() == (size_t)k) bestDist = heap.top().dist;
            } else if (d < heap.top().dist) {
                heap.pop();
                heap.push({p, d});
                bestDist = heap.top().dist;
            }
        }
        
        if (divided) {
            // Sort children by distance to target for better pruning
            struct ChildDist {
                Quadtree* qt;
                float sd;
                bool operator<(const ChildDist& o) const { return sd < o.sd; }
            };
            std::vector<ChildDist> children;
            children.push_back({nw.get(), nw->boundary.sqDist(target)});
            children.push_back({ne.get(), ne->boundary.sqDist(target)});
            children.push_back({sw.get(), sw->boundary.sqDist(target)});
            children.push_back({se.get(), se->boundary.sqDist(target)});
            std::sort(children.begin(), children.end());
            
            for (const auto& cd : children) {
                cd.qt->knnRecursive(target, k, heap, bestDist);
            }
        }
    }
    
public:
    int getNodeCount() const {
        int count = 1;
        if (divided) {
            count += nw->getNodeCount() + ne->getNodeCount() + 
                     sw->getNodeCount() + se->getNodeCount();
        }
        return count;
    }
    
    int getLeafCount() const {
        if (!divided) return 1;
        return nw->getLeafCount() + ne->getLeafCount() + 
               sw->getLeafCount() + se->getLeafCount();
    }
};

// ============================ Brute Force ============================
std::vector<Point> bruteRangeQuery(const std::vector<Point>& allPoints,
                                    const Point& center, float radius) {
    std::vector<Point> result;
    for (const auto& p : allPoints) {
        if (p.dist(center) <= radius) {
            result.push_back(p);
        }
    }
    return result;
}

std::vector<Point> bruteKnnQuery(const std::vector<Point>& allPoints,
                                  const Point& target, int k) {
    std::vector<std::pair<float, Point>> dists;
    for (const auto& p : allPoints) {
        dists.push_back({p.dist(target), p});
    }
    std::sort(dists.begin(), dists.end(), 
              [](const auto& a, const auto& b) { return a.first < b.first; });
    
    std::vector<Point> result;
    for (int i = 0; i < k && i < (int)dists.size(); i++) {
        result.push_back(dists[i].second);
    }
    return result;
}

// ============================ PPM Output ============================
void renderToPPM(const char* filename, Quadtree& qt, 
                 const std::vector<Point>& allPoints,
                 int width, int height) {
    std::vector<unsigned char> img(width * height * 3, 255);
    
    // Draw quadtree boundaries (light gray grid)
    std::function<void(const Quadtree&)> drawBounds = [&](const Quadtree& node) {
        // We need to recurse - but Quadtree doesn't expose children publicly
        // Use reflection via the Quadtree API
    };
    
    // Instead draw points and a sampling grid showing query efficiency
    AABB world(0.5f, 0.5f, 0.5f);  // Normalized [0,1]
    
    // Color the background with a heatmap showing nearest neighbor distance
    // as a proxy for spatial distribution
    std::vector<float> nnDist(width * height, 0);
    
    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            Point query(px / (float)width, (height - 1 - py) / (float)height);
            
            // Find nearest neighbor distance
            float minDist = 1e10f;
            for (const auto& p : allPoints) {
                float d = p.dist(query);
                if (d < minDist) minDist = d;
            }
            nnDist[py * width + px] = std::min(minDist * 3.0f, 1.0f);
        }
    }
    
    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            int idx = (py * width + px) * 3;
            float v = nnDist[py * width + px];
            // Cool (blue) to warm (red) based on density
            unsigned char r = (unsigned char)(v * 255);
            unsigned char b = (unsigned char)((1.0f - v) * 255);
            unsigned char g = (unsigned char)((1.0f - std::abs(v - 0.5f) * 2.0f) * 128);
            img[idx + 0] = r;
            img[idx + 1] = g;
            img[idx + 2] = b;
        }
    }
    
    // Draw points as small circles
    auto drawPoint = [&](float x, float y, unsigned char r, unsigned char g, unsigned char b, int rad) {
        int cx = (int)(x * width);
        int cy = (int)((1.0f - y) * height);
        for (int dy = -rad; dy <= rad; dy++) {
            for (int dx = -rad; dx <= rad; dx++) {
                if (dx*dx + dy*dy > rad*rad) continue;
                int px2 = cx + dx, py2 = cy + dy;
                if (px2 < 0 || px2 >= width || py2 < 0 || py2 >= height) continue;
                int idx = (py2 * width + px2) * 3;
                img[idx+0] = r; img[idx+1] = g; img[idx+2] = b;
            }
        }
    };
    
    for (const auto& p : allPoints) {
        drawPoint(p.x, p.y, 255, 255, 255, 3);
    }
    
    // Draw range query circle
    Point qCenter(0.5f, 0.5f);
    int angleSteps = 360;
    float qRadius = 0.15f;
    for (int i = 0; i <= angleSteps; i++) {
        float a = i * M_PI * 2.0f / angleSteps;
        float cx = qCenter.x + qRadius * cosf(a);
        float cy = qCenter.y + qRadius * sinf(a);
        drawPoint(cx, cy, 0, 255, 0, 1);
    }
    
    // Highlight points inside query radius
    std::vector<Point> queryResult;
    qt.rangeQuery(qCenter, qRadius, queryResult);
    for (const auto& p : queryResult) {
        drawPoint(p.x, p.y, 0, 255, 0, 5);
    }
    
    // Draw KNN points
    std::vector<Point> knnResult;
    qt.knnQuery(qCenter, 5, knnResult);
    for (size_t i = 0; i < knnResult.size(); i++) {
        drawPoint(knnResult[i].x, knnResult[i].y, 255, 255, 0, 6);
        // Draw line to center
        int cx = (int)(qCenter.x * width);
        int cy = (int)((1.0f - qCenter.y) * height);
        int px2 = (int)(knnResult[i].x * width);
        int py2 = (int)((1.0f - knnResult[i].y) * height);
        // Simple Bresenham line approximation
        float steps = std::max(std::abs(px2 - cx), std::abs(py2 - cy));
        for (float s = 0; s <= steps; s += 1.0f) {
            float t = s / steps;
            int lx = (int)(cx + (px2 - cx) * t);
            int ly = (int)(cy + (py2 - cy) * t);
            if (lx >= 0 && lx < width && ly >= 0 && ly < height) {
                int idx = (ly * width + lx) * 3;
                img[idx+0] = 255; img[idx+1] = 255; img[idx+2] = 0;
            }
        }
    }
    
    // Write PPM
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);
    std::cout << "  PPM written: " << filename << std::endl;
}

// ============================ Benchmark ============================
struct Benchmark {
    double quadtreeTime;
    double bruteTime;
    double speedup;
    int quadtreeVisited;  // Not tracked explicitly, but useful
    int resultCount;
};

double timeMs() {
    auto now = std::chrono::high_resolution_clock::now();
    static auto epoch = now;
    return std::chrono::duration<double, std::milli>(now - epoch).count();
}

Benchmark benchmarkRangeQuery(Quadtree& qt, const std::vector<Point>& allPoints,
                               const Point& center, float radius, int iterations) {
    Benchmark bm = {0, 0, 0, 0, 0};
    
    // Warmup
    for (int i = 0; i < 10; i++) {
        std::vector<Point> r1;
        qt.rangeQuery(center, radius, r1);
        auto r2 = bruteRangeQuery(allPoints, center, radius);
    }
    
    // Benchmark quadtree
    double t0 = timeMs();
    for (int i = 0; i < iterations; i++) {
        std::vector<Point> r;
        qt.rangeQuery(center, radius, r);
        if (i == 0) bm.resultCount = r.size();
    }
    bm.quadtreeTime = (timeMs() - t0) / iterations;
    
    // Benchmark brute force
    t0 = timeMs();
    for (int i = 0; i < iterations; i++) {
        auto r = bruteRangeQuery(allPoints, center, radius);
        if (i == 0) assert(r.size() == (size_t)bm.resultCount);
    }
    bm.bruteTime = (timeMs() - t0) / iterations;
    
    bm.speedup = bm.bruteTime / bm.quadtreeTime;
    return bm;
}

Benchmark benchmarkKnnQuery(Quadtree& qt, const std::vector<Point>& allPoints,
                             const Point& target, int k, int iterations) {
    Benchmark bm = {0, 0, 0, 0, 0};
    
    // Warmup
    for (int i = 0; i < 10; i++) {
        std::vector<Point> r1;
        qt.knnQuery(target, k, r1);
        auto r2 = bruteKnnQuery(allPoints, target, k);
    }
    
    // Benchmark quadtree
    double t0 = timeMs();
    for (int i = 0; i < iterations; i++) {
        std::vector<Point> r;
        qt.knnQuery(target, k, r);
        if (i == 0) bm.resultCount = r.size();
    }
    bm.quadtreeTime = (timeMs() - t0) / iterations;
    
    // Benchmark brute force
    t0 = timeMs();
    for (int i = 0; i < iterations; i++) {
        auto r = bruteKnnQuery(allPoints, target, k);
        if (i == 0) assert(r.size() == (size_t)bm.resultCount);
    }
    bm.bruteTime = (timeMs() - t0) / iterations;
    
    bm.speedup = bm.bruteTime / bm.quadtreeTime;
    return bm;
}

// ============================ Validation ============================
bool validateResults(const std::vector<Point>& qt, const std::vector<Point>& brute) {
    if (qt.size() != brute.size()) return false;
    // Results may be in different order - use set-based comparison
    for (const auto& qp : qt) {
        bool found = false;
        for (const auto& bp : brute) {
            if (std::abs(qp.x - bp.x) < 0.001f && std::abs(qp.y - bp.y) < 0.001f) {
                found = true; break;
            }
        }
        if (!found) return false;
    }
    return true;
}

bool validateKnn(const std::vector<Point>& qt, const std::vector<Point>& brute, 
                 const Point& target) {
    if (qt.size() != brute.size()) return false;
    // KNN order may differ for ties, so just check set membership
    for (const auto& p : qt) {
        bool found = false;
        for (const auto& bp : brute) {
            if (std::abs(p.x - bp.x) < 0.001f && std::abs(p.y - bp.y) < 0.001f) {
                found = true; break;
            }
        }
        if (!found) return false;
    }
    return true;
}

// ============================ Main ============================
int main() {
    std::cout << "========================================\n";
    std::cout << "  Quadtree Spatial Partitioning Index\n";
    std::cout << "  量化性能验证\n";
    std::cout << "========================================\n\n";
    
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    // Test with different point counts
    std::vector<int> pointCounts = {500, 1000, 5000, 10000, 50000};
    
    std::cout << std::left << std::setw(12) << "Points"
              << std::setw(12) << "Qt Nodes"
              << std::setw(12) << "Qt Leaves"
              << std::setw(14) << "Range(Qt ms)"
              << std::setw(14) << "Range(Brute)"
              << std::setw(10) << "Speedup"
              << std::setw(14) << "KNN(Qt ms)"
              << std::setw(14) << "KNN(Brute)"
              << std::setw(10) << "Speedup"
              << std::endl;
    std::cout << std::string(120, '-') << std::endl;
    
    // For visualization: use the largest dataset
    Quadtree* vizQt = nullptr;
    std::vector<Point>* vizPoints = nullptr;
    
    for (int n : pointCounts) {
        // Generate random points
        std::vector<Point> points;
        points.reserve(n);
        for (int i = 0; i < n; i++) {
            points.emplace_back(dist(rng), dist(rng));
        }
        
        // Build quadtree
        AABB world(0.5f, 0.5f, 0.5f);
        Quadtree qt(world);
        for (const auto& p : points) {
            qt.insert(p);
        }
        
        // Range query: 10% of world area
        Point qCenter(0.5f, 0.5f);
        float qRadius = 0.05f + n * 0.000002f;  // Slightly grows with n
        
        int iters = std::max(1, 100000 / n);  // more iters for smaller n
        auto rangeBm = benchmarkRangeQuery(qt, points, qCenter, qRadius, iters);
        
        // KNN query
        int k = 10;
        auto knnBm = benchmarkKnnQuery(qt, points, qCenter, k, iters);
        
        std::cout << std::left << std::setw(12) << n
                  << std::setw(12) << qt.getNodeCount()
                  << std::setw(12) << qt.getLeafCount()
                  << std::setw(14) << std::fixed << std::setprecision(4) << rangeBm.quadtreeTime
                  << std::setw(14) << rangeBm.bruteTime
                  << std::setw(10) << std::setprecision(2) << rangeBm.speedup << "x"
                  << std::setw(14) << std::setprecision(4) << knnBm.quadtreeTime
                  << std::setw(14) << knnBm.bruteTime
                  << std::setw(10) << std::setprecision(2) << knnBm.speedup << "x"
                  << std::endl;
        
        // Keep the 5000-point dataset for visualization
        if (n == 5000) {
            vizQt = new Quadtree(world);
            vizPoints = new std::vector<Point>(points);
            for (const auto& p : points) vizQt->insert(p);
        }
    }
    
    std::cout << std::endl;
    
    // ==================== Correctness Validation ====================
    std::cout << "========================================\n";
    std::cout << "  正确性验证\n";
    std::cout << "========================================\n";
    
    std::vector<Point> valPoints;
    std::mt19937 rng2(123);
    for (int i = 0; i < 500; i++) {
        valPoints.emplace_back(dist(rng2), dist(rng2));
    }
    
    AABB world(0.5f, 0.5f, 0.5f);
    Quadtree qtVal(world);
    for (const auto& p : valPoints) qtVal.insert(p);
    
    // Test range query correctness
    bool allRangePass = true;
    for (int i = 0; i < 20; i++) {
        Point c(dist(rng2), dist(rng2));
        float r = dist(rng2) * 0.3f;
        std::vector<Point> qtRes, bruteRes;
        qtVal.rangeQuery(c, r, qtRes);
        bruteRes = bruteRangeQuery(valPoints, c, r);
        if (!validateResults(qtRes, bruteRes)) {
            allRangePass = false;
            std::cout << "  ❌ Range query mismatch at (" << c.x << "," << c.y 
                      << ") r=" << r << std::endl;
            break;
        }
    }
    std::cout << "  Range Query: " << (allRangePass ? "✅ PASS" : "❌ FAIL") 
              << " (20 random tests)\n";
    
    // Test KNN correctness
    bool allKnnPass = true;
    for (int i = 0; i < 20; i++) {
        Point c(dist(rng2), dist(rng2));
        int k2 = 5 + (i % 10);
        std::vector<Point> qtRes, bruteRes;
        qtVal.knnQuery(c, k2, qtRes);
        bruteRes = bruteKnnQuery(valPoints, c, k2);
        if (!validateKnn(qtRes, bruteRes, c)) {
            allKnnPass = false;
            std::cout << "  ❌ KNN query mismatch at (" << c.x << "," << c.y 
                      << ") k=" << k2 << std::endl;
            break;
        }
    }
    std::cout << "  KNN Query:    " << (allKnnPass ? "✅ PASS" : "❌ FAIL") 
              << " (20 random tests)\n\n";
    
    // Check for exact match by sorting both results and comparing distances
    // More rigorous check
    std::cout << "  Rigorous KNN distance check...\n";
    bool rigorousPass = true;
    for (int i = 0; i < 20; i++) {
        Point c(dist(rng2), dist(rng2));
        int k2 = 5 + (i % 10);
        std::vector<Point> qtRes, bruteRes;
        qtVal.knnQuery(c, k2, qtRes);
        bruteRes = bruteKnnQuery(valPoints, c, k2);
        
        // Check that all distances match
        float qtDist = 0, bruteDist = 0;
        for (const auto& p : qtRes) qtDist += p.dist(c);
        for (const auto& p : bruteRes) bruteDist += p.dist(c);
        
        if (std::abs(qtDist - bruteDist) > 0.01f) {
            rigorousPass = false;
            std::cout << "  ❌ Distance mismatch: qt=" << qtDist 
                      << " brute=" << bruteDist << std::endl;
            break;
        }
    }
    std::cout << "  Sum-distance check: " << (rigorousPass ? "✅ PASS" : "❌ FAIL") 
              << " (20 random tests)\n\n";
    
    // ==================== Visualization ====================
    std::cout << "========================================\n";
    std::cout << "  可视化输出\n";
    std::cout << "========================================\n";
    
    if (vizQt && vizPoints) {
        renderToPPM("quadtree_output.ppm", *vizQt, *vizPoints, 800, 800);
        delete vizQt;
        delete vizPoints;
    }
    
    std::cout << "\n✅ 每日编程实践 2026-06-14 完成！\n";
    
    return 0;
}
