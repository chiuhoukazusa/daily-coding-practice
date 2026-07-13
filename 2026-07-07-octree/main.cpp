/**
 * Octree Spatial Partitioning — Daily Coding Practice 2026-07-07
 *
 * Features:
 *  - 3D point insertion with adaptive subdivision
 *  - Point & sphere range queries
 *  - K-Nearest Neighbor search (KNN)
 *  - Brute-force baseline for speedup validation
 *  - PPM visualization (2D slice projection)
 *
 * Validation:
 *  - Quantitative speedup vs brute force (range query & KNN)
 *  - Correctness: Octree results must match brute-force results
 *  - Memory: report node count & depth
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

// ============================================================
// 3D Vector
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    float dist(const Vec3& o) const {
        float dx = x - o.x, dy = y - o.y, dz = z - o.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    bool operator<(const Vec3& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
    bool operator==(const Vec3& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

class Octree;

// ============================================================
// Octree Node
// ============================================================
struct OctreeNode {
    Vec3 center;
    float halfSize;
    std::vector<Vec3> points;  // stored points (for leaf nodes)
    OctreeNode* children[8];
    bool isLeaf;
    Octree* owner;

    OctreeNode(Vec3 c, float hs, Octree* o)
        : center(c), halfSize(hs), isLeaf(true), owner(o) {
        for (int i = 0; i < 8; i++) children[i] = nullptr;
    }

    ~OctreeNode() {
        for (int i = 0; i < 8; i++) delete children[i];
    }

    bool contains(const Vec3& p) const {
        return p.x >= center.x - halfSize && p.x <= center.x + halfSize &&
               p.y >= center.y - halfSize && p.y <= center.y + halfSize &&
               p.z >= center.z - halfSize && p.z <= center.z + halfSize;
    }

    bool intersectsSphere(const Vec3& sphereCenter, float radius) const {
        float dx = std::max(0.0f, std::fabs(sphereCenter.x - center.x) - halfSize);
        float dy = std::max(0.0f, std::fabs(sphereCenter.y - center.y) - halfSize);
        float dz = std::max(0.0f, std::fabs(sphereCenter.z - center.z) - halfSize);
        return dx*dx + dy*dy + dz*dz <= radius * radius;
    }

    int getOctant(const Vec3& p) const {
        int idx = 0;
        if (p.x >= center.x) idx |= 1;
        if (p.y >= center.y) idx |= 2;
        if (p.z >= center.z) idx |= 4;
        return idx;
    }

    void subdivide();
    void insertPoint(const Vec3& p, int depth);
};

// ============================================================
// Octree
// ============================================================
class Octree {
public:
    OctreeNode* root;
    int nodeCount;
    int maxDepth;
    int maxPointsPerLeaf;
    float minSize;

    Octree(Vec3 center, float halfSize, int maxPts = 4, float minSz = 0.01f)
        : nodeCount(1), maxDepth(0), maxPointsPerLeaf(maxPts), minSize(minSz) {
        root = new OctreeNode(center, halfSize, this);
    }

    ~Octree() { delete root; }

    void insert(const Vec3& p) {
        root->insertPoint(p, 0);
    }

    std::vector<Vec3> rangeQuery(const Vec3& center, float radius) const {
        std::vector<Vec3> result;
        rangeQueryRecursive(root, center, radius, result);
        return result;
    }

    std::vector<std::pair<float, Vec3>> knnQuery(const Vec3& query, int k) const {
        std::vector<std::pair<float, Vec3>> best;
        knnSearch(root, query, k, best);
        return best;
    }

    int countNodes() const { return countNodesRecursive(root); }
    int getMaxDepth() const {
        int d = 0;
        getDepthRecursive(root, 0, d);
        return d;
    }

private:
    void rangeQueryRecursive(const OctreeNode* node, const Vec3& center, float radius,
                             std::vector<Vec3>& result) const {
        if (!node->intersectsSphere(center, radius)) return;

        // Check all points in this node (both leaves and internal nodes may have points
        // if subdivision was stopped by minSize)
        if (node->isLeaf || !node->points.empty()) {
            for (const auto& p : node->points) {
                if (p.dist(center) <= radius) {
                    result.push_back(p);
                }
            }
        }

        if (node->isLeaf) return;

        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                rangeQueryRecursive(node->children[i], center, radius, result);
            }
        }
    }

    static void knnSearch(const OctreeNode* node, const Vec3& query, int k,
                          std::vector<std::pair<float, Vec3>>& best) {
        if (!node) return;

        // Check all points in this node
        if (node->isLeaf || !node->points.empty()) {
            for (const auto& p : node->points) {
                float d = p.dist(query);
                best.push_back({d, p});
                std::sort(best.begin(), best.end());
                if ((int)best.size() > k) best.resize(k);
            }
        }

        if (node->isLeaf) return;

        // Compute distance to each child's center for pruning
        struct ChildDist { int idx; float dist; };
        std::vector<ChildDist> cds;
        for (int i = 0; i < 8; i++) {
            if (node->children[i]) {
                cds.push_back({i, node->children[i]->center.dist(query)});
            }
        }

        std::sort(cds.begin(), cds.end(),
                  [](const ChildDist& a, const ChildDist& b) { return a.dist < b.dist; });

        for (auto& cd : cds) {
            float bestDist = ((int)best.size() >= k) ? best[k-1].first
                                                      : std::numeric_limits<float>::max();
            float childMinDist = cd.dist - node->children[cd.idx]->halfSize * 1.732f;
            if ((int)best.size() >= k && childMinDist > bestDist) continue;
            knnSearch(node->children[cd.idx], query, k, best);
        }
    }

    int countNodesRecursive(const OctreeNode* node) const {
        if (!node) return 0;
        int count = 1;
        for (int i = 0; i < 8; i++) count += countNodesRecursive(node->children[i]);
        return count;
    }

    void getDepthRecursive(const OctreeNode* node, int depth, int& maxD) const {
        if (!node) return;
        maxD = std::max(maxD, depth);
        for (int i = 0; i < 8; i++) getDepthRecursive(node->children[i], depth + 1, maxD);
    }
};

// ============================================================
// OctreeNode methods (need full Octree definition)
// ============================================================
void OctreeNode::subdivide() {
    float quarter = halfSize * 0.5f;
    for (int i = 0; i < 8; i++) {
        float cx = center.x + ((i & 1) ? quarter : -quarter);
        float cy = center.y + ((i & 2) ? quarter : -quarter);
        float cz = center.z + ((i & 4) ? quarter : -quarter);
        children[i] = new OctreeNode(Vec3(cx, cy, cz), quarter, owner);
        owner->nodeCount += 1; // actually we count on construction
    }
    isLeaf = false;

    // Re-insert stored points into children
    for (const auto& pt : points) {
        int idx = getOctant(pt);
        children[idx]->insertPoint(pt, 0); // depth reset for the child
    }
    points.clear();
}

void OctreeNode::insertPoint(const Vec3& p, int depth) {
    if (!contains(p)) return;

    owner->maxDepth = std::max(owner->maxDepth, depth);

    if (isLeaf) {
        points.push_back(p);
        owner->nodeCount++;

        // Subdivide if we have too many points and aren't at min size
        if ((int)points.size() > owner->maxPointsPerLeaf && halfSize > owner->minSize) {
            subdivide();
        }
        return;
    }

    // Not a leaf - delegate to child
    int octant = getOctant(p);
    children[octant]->insertPoint(p, depth + 1);
}

// ============================================================
// Brute-force baseline
// ============================================================
std::vector<Vec3> bruteForceRangeQuery(const std::vector<Vec3>& points,
                                        const Vec3& center, float radius) {
    std::vector<Vec3> result;
    for (const auto& p : points) {
        if (p.dist(center) <= radius) result.push_back(p);
    }
    return result;
}

std::vector<std::pair<float, Vec3>> bruteForceKNN(const std::vector<Vec3>& points,
                                                     const Vec3& query, int k) {
    std::vector<std::pair<float, Vec3>> dists;
    for (const auto& p : points) {
        dists.push_back({p.dist(query), p});
    }
    std::sort(dists.begin(), dists.end());
    if ((int)dists.size() > k) dists.resize(k);
    return dists;
}

// ============================================================
// PPM Image output (2D slice visualization)
// ============================================================
void savePPM(const std::string& filename, const std::vector<Vec3>& points,
             const Octree& tree, int width, int height) {
    std::vector<uint8_t> image(width * height * 3, 255);

    // Grid
    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++) {
            int idx = (j * width + i) * 3;
            if (i % 40 == 0 || j % 40 == 0)
                image[idx] = image[idx+1] = image[idx+2] = 230;
        }

    auto worldToScreen = [&](float wx, float wy) -> std::pair<int, int> {
        int sx = (int)((wx + 5.0f) / 10.0f * width);
        int sy = (int)((5.0f - wy) / 10.0f * height);
        return {sx, sy};
    };

    // Draw octree boundaries
    std::function<void(const OctreeNode*)> drawCells = [&](const OctreeNode* node) {
        if (!node) return;
        float cx = node->center.x, cy = node->center.y;
        float hs = node->halfSize;
        auto [x1, y1] = worldToScreen(cx - hs, cy - hs);
        auto [x2, y2] = worldToScreen(cx + hs, cy + hs);
        x1 = std::max(0, std::min(width-1, x1));
        x2 = std::max(0, std::min(width-1, x2));
        y1 = std::max(0, std::min(height-1, y1));
        y2 = std::max(0, std::min(height-1, y2));

        if ((x2-x1) >= 2 && (y2-y1) >= 2) {
            for (int x = x1; x <= x2; x++) {
                if (x >= 0 && x < width && y1 >= 0 && y1 < height) {
                    int i1 = (y1*width+x)*3; image[i1]=image[i1+1]=image[i1+2]=180;
                }
                if (x >= 0 && x < width && y2 >= 0 && y2 < height) {
                    int i2 = (y2*width+x)*3; image[i2]=image[i2+1]=image[i2+2]=180;
                }
            }
            for (int y = y1; y <= y2; y++) {
                if (y >= 0 && y < height && x1 >= 0 && x1 < width) {
                    int i1 = (y*width+x1)*3; image[i1]=image[i1+1]=image[i1+2]=180;
                }
                if (y >= 0 && y < height && x2 >= 0 && x2 < width) {
                    int i2 = (y*width+x2)*3; image[i2]=image[i2+1]=image[i2+2]=180;
                }
            }
        }
        for (int i = 0; i < 8; i++) drawCells(node->children[i]);
    };
    drawCells(tree.root);

    // Draw points (projected XY, color by Z)
    for (const auto& p : points) {
        auto [sx, sy] = worldToScreen(p.x, p.y);
        if (sx < 1 || sx >= width-1 || sy < 1 || sy >= height-1) continue;
        float t = std::max(0.0f, std::min(1.0f, (p.z + 5.0f) / 10.0f));
        uint8_t r = (uint8_t)(255*(1.0f-t)), b = (uint8_t)(255*t);
        uint8_t g = (uint8_t)(128*(1.0f-std::fabs(t-0.5f)*2.0f));
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int px = sx+dx, py = sy+dy;
                if (px>=0 && px<width && py>=0 && py<height) {
                    int idx = (py*width+px)*3;
                    image[idx]=r; image[idx+1]=g; image[idx+2]=b;
                }
            }
    }

    // Legend bar
    int lY = height - 35;
    for (int i = 0; i < 12; i++) {
        float t = i / 11.0f;
        uint8_t r = (uint8_t)(255*(1.0f-t)), b = (uint8_t)(255*t);
        int x0 = 30 + i*35;
        for (int dy=0; dy<15; dy++)
            for (int dx=0; dx<25; dx++) {
                int idx = ((lY+dy)*width+(x0+dx))*3;
                image[idx]=r; image[idx+1]=0; image[idx+2]=b;
            }
    }

    FILE* f = fopen(filename.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", width, height);
    fwrite(image.data(), 1, image.size(), f);
    fclose(f);
}

// ============================================================
// Benchmark helper
// ============================================================
struct BResult {
    int octCnt, bruteCnt;
    double octMs, bruteMs, speedup;
    bool correct;
};

BResult benchRange(const Octree& tree, const std::vector<Vec3>& pts,
                   const Vec3& c, float r, int reps=50) {
    BResult res = {};
    auto o1 = tree.rangeQuery(c, r);
    auto b1 = bruteForceRangeQuery(pts, c, r);

    clock_t start = clock();
    for (int i = 0; i < reps; i++) { volatile auto x = tree.rangeQuery(c, r); (void)x; }
    res.octMs = 1000.0*(clock()-start)/CLOCKS_PER_SEC/reps;

    start = clock();
    for (int i = 0; i < reps; i++) { volatile auto x = bruteForceRangeQuery(pts, c, r); (void)x; }
    res.bruteMs = 1000.0*(clock()-start)/CLOCKS_PER_SEC/reps;

    res.speedup = res.octMs > 0 ? res.bruteMs/res.octMs : 0;
    res.octCnt = (int)o1.size(); res.bruteCnt = (int)b1.size();
    res.correct = (res.octCnt == res.bruteCnt) && (res.octCnt > 0);
    return res;
}

BResult benchKNN(const Octree& tree, const std::vector<Vec3>& pts,
                 const Vec3& q, int k, int reps=30) {
    BResult res = {};
    auto o1 = tree.knnQuery(q, k);
    auto b1 = bruteForceKNN(pts, q, k);

    clock_t start = clock();
    for (int i = 0; i < reps; i++) { volatile auto x = tree.knnQuery(q, k); (void)x; }
    res.octMs = 1000.0*(clock()-start)/CLOCKS_PER_SEC/reps;

    start = clock();
    for (int i = 0; i < reps; i++) { volatile auto x = bruteForceKNN(pts, q, k); (void)x; }
    res.bruteMs = 1000.0*(clock()-start)/CLOCKS_PER_SEC/reps;

    res.speedup = res.octMs > 0 ? res.bruteMs/res.octMs : 0;
    res.octCnt = (int)o1.size(); res.bruteCnt = (int)b1.size();

    res.correct = ((int)o1.size() == (int)b1.size() && (int)o1.size() == k);
    if (res.correct) {
        for (int i = 0; i < k; i++)
            if (std::fabs(o1[i].first - b1[i].first) > 0.001f) { res.correct = false; break; }
    }
    return res;
}

// ============================================================
// Main
// ============================================================
int main() {
    printf("=== Octree Spatial Partitioning ===\n");
    printf("Daily Coding Practice 2026-07-07\n\n");

    const int N = 10000;
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-5.0f, 5.0f);

    std::vector<Vec3> pts;
    pts.reserve(N);
    for (int i = 0; i < N; i++)
        pts.push_back(Vec3(dist(rng), dist(rng), dist(rng)));

    printf("Building Octree with %d points (maxPointsPerLeaf=4, minSize=0.05)...\n", N);
    Octree tree(Vec3(0,0,0), 5.0f, 4, 0.05f);

    clock_t start = clock();
    for (const auto& p : pts) tree.insert(p);
    double buildMs = 1000.0*(clock()-start)/CLOCKS_PER_SEC;

    printf("  Build: %.3f ms  Nodes: %d  MaxDepth: %d\n\n", buildMs, tree.countNodes(), tree.getMaxDepth());

    // Verify total points stored
    int storedCount = 0;
    std::function<void(const OctreeNode*)> countPts = [&](const OctreeNode* n) {
        if (!n) return;
        storedCount += (int)n->points.size();
        for (int i=0; i<8; i++) countPts(n->children[i]);
    };
    countPts(tree.root);
    printf("  Points stored in octree: %d / %d\n", storedCount, N);

    // Range tests
    printf("\n=== Range Query Benchmarks ===\n");
    struct { const char* name; Vec3 c; float r; } rTests[] = {
        {"Small (0.5)",  Vec3(1.5f,-2.0f,0.5f), 0.5f},
        {"Medium (1.5)", Vec3(-3.0f,1.0f,-1.0f), 1.5f},
        {"Large (3.0)",  Vec3(0.f,0.f,0.f), 3.0f},
        {"Huge (5.0)",   Vec3(2.f,3.f,-2.f), 5.0f},
        {"Edge",          Vec3(4.5f,4.5f,4.5f), 1.0f},
    };
    int rPass = 0;
    for (auto& t : rTests) {
        auto r = benchRange(tree, pts, t.c, t.r);
        printf("  %-16s → oct=%d brute=%d %s  %.4fms vs %.4fms (%.1fx)\n",
               t.name, r.octCnt, r.bruteCnt, r.correct?"✅":"❌", r.octMs, r.bruteMs, r.speedup);
        if (r.correct) rPass++;
    }

    // KNN tests
    printf("\n=== KNN Query Benchmarks ===\n");
    struct { const char* name; Vec3 q; int k; } kTests[] = {
        {"Small k,center",    Vec3(0,0,0), 5},
        {"Med k,corner",      Vec3(4,4,4), 10},
        {"Large k,sparse",    Vec3(-4.5f,-4.5f,-4.5f), 20},
        {"Small k,random",    Vec3(2.3f,-3.1f,1.7f), 8},
        {"Med k,center",      Vec3(0.5f,-0.5f,0.2f), 15},
    };
    int kPass = 0;
    for (auto& t : kTests) {
        auto r = benchKNN(tree, pts, t.q, t.k);
        printf("  %-16s k=%2d → %s  %.4fms vs %.4fms (%.1fx)\n",
               t.name, t.k, r.correct?"✅":"❌", r.octMs, r.bruteMs, r.speedup);
        if (r.correct) kPass++;
    }

    printf("\n=== Summary ===\n");
    printf("  Range: %d/5  KNN: %d/5  Nodes: %d  Depth: %d  Build: %.3fms\n",
           rPass, kPass, tree.countNodes(), tree.getMaxDepth(), buildMs);
    bool ok = (rPass==5 && kPass==5);
    printf("  VERDICT: %s\n", ok ? "✅ ALL PASSED" : "❌ FAILURES");

    printf("\nSaving octree_output.ppm...\n");
    savePPM("octree_output.ppm", pts, tree, 800, 800);
    printf("  Saved (800x800)\n");

    return ok ? 0 : 1;
}
