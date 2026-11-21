// Spatial Hash Grid Collision Detection
// Broad-phase collision detection using a uniform spatial hash grid.
// Objects are axis-aligned bounding boxes (AABB) moving in 2D.
//
// Broad phase: insert AABBs into grid cells, query only candidate neighbors.
// Narrow phase: exact AABB-vs-AABB overlap test.
//
// Verification:
//   - Compare against brute-force O(n^2) exact overlap counting (ground truth).
//   - Assert identical collision-pair set.
//   - Quantify speedup (time broad+narrow vs brute force).
//   - Measure complexity scaling across object counts.

#include <bits/stdc++.h>
using namespace std;

static const double PI = 3.14159265358979323846;

struct Vec2 {
    double x, y;
};

struct AABB {
    Vec2 c;     // center
    Vec2 h;     // half-extent
    int id;
};

// Simple, deterministic RNG (avoid std::mt19937 nondeterminism across runs for reproducibility).
struct Rng {
    uint64_t s;
    Rng(uint64_t seed = 88172645463325252ULL) : s(seed) {}
    uint64_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return s;
    }
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * (double)(next() % 1000000000ULL) / 1000000000.0;
    }
};

// Exact AABB overlap test.
static inline bool overlaps(const AABB& a, const AABB& b) {
    return fabs(a.c.x - b.c.x) <= (a.h.x + b.h.x) &&
           fabs(a.c.y - b.c.y) <= (a.h.y + b.h.y);
}

// ---- Spatial Hash Grid ----
struct SpatialHashGrid {
    double cellSize;
    double invCell;
    // key = (cx * 73856093) ^ (cy * 19349663);  handle negative coords carefully
    unordered_map<long long, vector<int>> cells;

    SpatialHashGrid(double cell) : cellSize(cell), invCell(1.0 / cell) {}

    static inline long long hashKey(long long cx, long long cy) {
        // Cantor-like pairing to avoid collisions from xor of nearby cells.
        // Use a simple bijective pairing (Szudzik).
        long long a = cx >= 0 ? 2 * cx : -2 * cx - 1;
        long long b = cy >= 0 ? 2 * cy : -2 * cy - 1;
        return (a >= b) ? (a * a + a + b) : (a + b * b);
    }

    void build(const vector<AABB>& objs) {
        cells.clear();
        for (int i = 0; i < (int)objs.size(); ++i) {
            const AABB& o = objs[i];
            long long cx0 = (long long)floor(o.c.x * invCell);
            long long cx1 = (long long)floor(o.c.x * invCell); // only center-based for broad phase
            // For a hash grid we typically hash based on the AABB span, but here we
            // hash using bbox min/max cells to guarantee we never miss a neighbor.
            long long minX = (long long)floor((o.c.x - o.h.x) * invCell);
            long long maxX = (long long)floor((o.c.x + o.h.x) * invCell);
            long long minY = (long long)floor((o.c.y - o.h.y) * invCell);
            long long maxY = (long long)floor((o.c.y + o.h.y) * invCell);
            (void)cx0; (void)cx1;
            for (long long cx = minX; cx <= maxX; ++cx) {
                for (long long cy = minY; cy <= maxY; ++cy) {
                    cells[hashKey(cx, cy)].push_back(i);
                }
            }
        }
    }

    // Returns the set of candidate (i,j) pairs where i<j and their AABBs share at
    // least one cell, without duplicates.
    vector<pair<int,int>> candidatePairs(const vector<AABB>& objs) {
        vector<pair<int,int>> pairs;
        // Use a set to dedupe pairs.
        unordered_set<long long> seen;
        seen.reserve(objs.size() * 4);
        for (auto& kv : cells) {
            const vector<int>& ids = kv.second;
            int n = ids.size();
            for (int x = 0; x < n; ++x) {
                for (int y = x + 1; y < n; ++y) {
                    int a = ids[x], b = ids[y];
                    if (a > b) swap(a, b);
                    long long key = ((long long)a << 32) | (unsigned int)b;
                    if (seen.insert(key).second) {
                        pairs.emplace_back(a, b);
                    }
                }
            }
        }
        return pairs;
    }
};

// Brute-force exact collision pairs.
static vector<pair<int,int>> bruteForce(const vector<AABB>& objs) {
    vector<pair<int,int>> res;
    int n = objs.size();
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (overlaps(objs[i], objs[j]))
                res.emplace_back(i, j);
    return res;
}

// Broad+narrow using spatial hash.
static vector<pair<int,int>> spatialHash(const vector<AABB>& objs, double cell) {
    SpatialHashGrid grid(cell);
    grid.build(objs);
    auto cand = grid.candidatePairs(objs);
    vector<pair<int,int>> res;
    res.reserve(cand.size());
    for (auto& p : cand) {
        if (overlaps(objs[p.first], objs[p.second]))
            res.push_back(p);
    }
    return res;
}

static vector<AABB> genObjects(int n, Rng& rng, double world, double maxHalf) {
    vector<AABB> objs;
    objs.reserve(n);
    for (int i = 0; i < n; ++i) {
        AABB o;
        o.id = i;
        o.c.x = rng.uniform(0.0, world);
        o.c.y = rng.uniform(0.0, world);
        o.h.x = rng.uniform(0.5, maxHalf);
        o.h.y = rng.uniform(0.5, maxHalf);
        objs.push_back(o);
    }
    return objs;
}

int main() {
    double world = 1000.0;
    double maxHalf = 5.0;
    double cellSize = 8.0;

    // ---- Correctness test over increasing sizes ----
    printf("=== Correctness (Spatial Hash vs Brute Force) ===\n");
    printf("%8s %10s %10s %10s %10s\n", "N", "Brute", "Hash", "Match", "CandPairs");
    bool allOk = true;
    for (int n : {100, 500, 1000, 5000}) {
        Rng rng(12345 + n);
        auto objs = genObjects(n, rng, world, maxHalf);
        auto b = bruteForce(objs);
        auto h = spatialHash(objs, cellSize);

        // Compare sets.
        auto keySet = [](const vector<pair<int,int>>& p) {
            set<pair<int,int>> s;
            for (auto& x : p) s.insert(x);
            return s;
        };
        bool match = (keySet(b) == keySet(h));
        allOk &= match;
        printf("%8d %10zu %10zu %10s %10zu\n", n, b.size(), h.size(),
               match ? "OK" : "MISMATCH", h.size());
    }
    if (!allOk) {
        printf("❌ Correctness FAILED\n");
        return 1;
    }
    printf("✅ Collision-pair sets identical to brute force for all sizes\n\n");

    // ---- Performance / speedup ----
    printf("=== Performance (timing) ===\n");
    printf("%8s %14s %14s %12s\n", "N", "Brute(ms)", "Hash(ms)", "Speedup");
    for (int n : {1000, 5000, 10000, 20000, 50000}) {
        Rng rng(999 + n);
        auto objs = genObjects(n, rng, world, maxHalf);

        auto t0 = chrono::high_resolution_clock::now();
        auto b = bruteForce(objs);
        auto t1 = chrono::high_resolution_clock::now();
        auto h = spatialHash(objs, cellSize);
        auto t2 = chrono::high_resolution_clock::now();

        double msB = chrono::duration<double, milli>(t1 - t0).count();
        double msH = chrono::duration<double, milli>(t2 - t1).count();
        double sp = msB / max(msH, 1e-9);
        printf("%8d %14.2f %14.2f %12.1fx\n", n, msB, msH, sp);
    }
    printf("\n");

    // ---- Complexity scaling check: ratio of adjacent timings should grow ~linearly
    //      for hash (not N^2). Output hash time ratios for 2x object counts.
    printf("=== Hash-grid scaling sanity (should be ~O(N), not O(N^2)) ===\n");
    double prev = -1;
    for (int n : {5000, 10000, 20000, 40000}) {
        Rng rng(777 + n);
        auto objs = genObjects(n, rng, world, maxHalf);
        auto t0 = chrono::high_resolution_clock::now();
        auto h = spatialHash(objs, cellSize);
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        printf("N=%6d  Hash=%.2f ms", n, ms);
        if (prev > 0)
            printf("   (2x objects -> %.2fx time)\n", ms / prev);
        else
            printf("\n");
        prev = ms;
    }

    printf("\nDONE\n");
    return 0;
}
