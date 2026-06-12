// A* Pathfinding on a 2D grid, with Dijkstra & BFS cross-validation.
//
// Goal: implement A* search with an admissible heuristic and PROVE its
// correctness quantitatively (not by eyeballing the picture):
//   1. A* path cost == Dijkstra path cost (A* is optimal)
//   2. On a uniform-cost grid, BFS path length == A* path length
//   3. A* expands <= Dijkstra nodes (heuristic guidance pays off)
//   4. The returned path is connected, obstacle-free, and endpoint-correct
//
// Output: a PPM visualization (obstacles, explored set, final path).
//
// Build: g++ main.cpp -o astar -std=c++17 -O2 -Wall -Wextra

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <queue>
#include <random>
#include <tuple>
#include <vector>

struct Vec2i {
    int x, y;
    bool operator==(const Vec2i& o) const { return x == o.x && y == o.y; }
};

struct Grid {
    int w, h;
    std::vector<uint8_t> blocked;  // 1 = obstacle
    Grid(int w_, int h_) : w(w_), h(h_), blocked(static_cast<size_t>(w_) * h_, 0) {}
    int idx(int x, int y) const { return y * w + x; }
    bool inBounds(int x, int y) const { return x >= 0 && x < w && y >= 0 && y < h; }
    bool isBlocked(int x, int y) const { return blocked[idx(x, y)] != 0; }
    void block(int x, int y) { blocked[idx(x, y)] = 1; }
};

// 4-connected neighbours (uniform unit cost) => BFS optimality is comparable.
static const std::array<Vec2i, 4> kNeighbors = {{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

static int manhattan(Vec2i a, Vec2i b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

struct SearchResult {
    bool found = false;
    int pathCost = -1;            // number of steps (edges)
    long expanded = 0;            // number of nodes popped/settled
    std::vector<Vec2i> path;      // start..goal inclusive
};

// Reconstruct path from parent pointers.
static std::vector<Vec2i> reconstruct(const std::vector<int>& parent, const Grid& g,
                                       Vec2i start, Vec2i goal) {
    std::vector<Vec2i> path;
    int cur = g.idx(goal.x, goal.y);
    int startIdx = g.idx(start.x, start.y);
    while (cur != -1) {
        path.push_back({cur % g.w, cur / g.w});
        if (cur == startIdx) break;
        cur = parent[cur];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ---------------- A* ----------------
static SearchResult astar(const Grid& g, Vec2i start, Vec2i goal) {
    SearchResult r;
    const int N = g.w * g.h;
    std::vector<int> gScore(N, INT32_MAX);
    std::vector<int> parent(N, -1);
    std::vector<uint8_t> closed(N, 0);

    // priority queue of (fScore, gScore, idx)
    using Node = std::tuple<int, int, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    int s = g.idx(start.x, start.y);
    gScore[s] = 0;
    open.push({manhattan(start, goal), 0, s});

    while (!open.empty()) {
        auto [f, gc, cur] = open.top();
        open.pop();
        (void)f;
        if (closed[cur]) continue;  // stale entry
        closed[cur] = 1;
        r.expanded++;

        Vec2i cp{cur % g.w, cur / g.w};
        if (cp == goal) {
            r.found = true;
            r.pathCost = gScore[cur];
            r.path = reconstruct(parent, g, start, goal);
            return r;
        }
        for (auto d : kNeighbors) {
            int nx = cp.x + d.x, ny = cp.y + d.y;
            if (!g.inBounds(nx, ny) || g.isBlocked(nx, ny)) continue;
            int ni = g.idx(nx, ny);
            if (closed[ni]) continue;
            int tentative = gScore[cur] + 1;
            if (tentative < gScore[ni]) {
                gScore[ni] = tentative;
                parent[ni] = cur;
                int h = manhattan({nx, ny}, goal);
                open.push({tentative + h, tentative, ni});
            }
        }
    }
    return r;
}

// ---------------- Dijkstra (h == 0) ----------------
static SearchResult dijkstra(const Grid& g, Vec2i start, Vec2i goal) {
    SearchResult r;
    const int N = g.w * g.h;
    std::vector<int> dist(N, INT32_MAX);
    std::vector<int> parent(N, -1);
    std::vector<uint8_t> closed(N, 0);
    using Node = std::pair<int, int>;  // (dist, idx)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    int s = g.idx(start.x, start.y);
    dist[s] = 0;
    open.push({0, s});
    while (!open.empty()) {
        auto [d, cur] = open.top();
        open.pop();
        if (closed[cur]) continue;
        closed[cur] = 1;
        r.expanded++;
        Vec2i cp{cur % g.w, cur / g.w};
        if (cp == goal) {
            r.found = true;
            r.pathCost = dist[cur];
            r.path = reconstruct(parent, g, start, goal);
            return r;
        }
        for (auto dir : kNeighbors) {
            int nx = cp.x + dir.x, ny = cp.y + dir.y;
            if (!g.inBounds(nx, ny) || g.isBlocked(nx, ny)) continue;
            int ni = g.idx(nx, ny);
            if (closed[ni]) continue;
            int tentative = dist[cur] + 1;
            if (tentative < dist[ni]) {
                dist[ni] = tentative;
                parent[ni] = cur;
                open.push({tentative, ni});
            }
        }
        (void)d;
    }
    return r;
}

// ---------------- BFS (unit cost shortest path) ----------------
static SearchResult bfs(const Grid& g, Vec2i start, Vec2i goal) {
    SearchResult r;
    const int N = g.w * g.h;
    std::vector<int> dist(N, -1);
    std::vector<int> parent(N, -1);
    std::queue<int> q;
    int s = g.idx(start.x, start.y);
    dist[s] = 0;
    q.push(s);
    while (!q.empty()) {
        int cur = q.front();
        q.pop();
        r.expanded++;
        Vec2i cp{cur % g.w, cur / g.w};
        if (cp == goal) {
            r.found = true;
            r.pathCost = dist[cur];
            r.path = reconstruct(parent, g, start, goal);
            return r;
        }
        for (auto dir : kNeighbors) {
            int nx = cp.x + dir.x, ny = cp.y + dir.y;
            if (!g.inBounds(nx, ny) || g.isBlocked(nx, ny)) continue;
            int ni = g.idx(nx, ny);
            if (dist[ni] == -1) {
                dist[ni] = dist[cur] + 1;
                parent[ni] = cur;
                q.push(ni);
            }
        }
    }
    return r;
}

// ---------------- PPM visualization ----------------
static void writePPM(const std::string& fname, const Grid& g,
                     const std::vector<Vec2i>& path,
                     const std::vector<uint8_t>& explored,
                     Vec2i start, Vec2i goal, int scale) {
    int W = g.w * scale, H = g.h * scale;
    std::vector<uint8_t> img(static_cast<size_t>(W) * H * 3, 0);

    auto setCell = [&](int cx, int cy, uint8_t r, uint8_t gg, uint8_t b) {
        for (int dy = 0; dy < scale; ++dy)
            for (int dx = 0; dx < scale; ++dx) {
                int px = cx * scale + dx, py = cy * scale + dy;
                // 1px grid line on cell border for readability/detail.
                bool border = (dx == 0 || dy == 0);
                uint8_t rr = border ? static_cast<uint8_t>(r * 0.75) : r;
                uint8_t gc = border ? static_cast<uint8_t>(gg * 0.75) : gg;
                uint8_t bb = border ? static_cast<uint8_t>(b * 0.75) : b;
                size_t o = (static_cast<size_t>(py) * W + px) * 3;
                img[o] = rr; img[o + 1] = gc; img[o + 2] = bb;
            }
    };

    for (int y = 0; y < g.h; ++y)
        for (int x = 0; x < g.w; ++x) {
            if (g.isBlocked(x, y))            setCell(x, y, 30, 30, 40);     // obstacle
            else if (explored[g.idx(x, y)])   setCell(x, y, 70, 110, 160);   // explored
            else                              setCell(x, y, 235, 235, 235);  // free
        }
    for (auto p : path) setCell(p.x, p.y, 230, 70, 60);                      // path (red)
    setCell(start.x, start.y, 40, 200, 60);                                  // start (green)
    setCell(goal.x, goal.y, 250, 200, 30);                                   // goal (yellow)

    std::ofstream f(fname, std::ios::binary);
    f << "P6\n" << W << " " << H << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), img.size());
}

int main() {
    const int W = 80, H = 60;
    Grid g(W, H);
    Vec2i start{2, 2}, goal{W - 3, H - 3};

    // Deterministic obstacle field: random scatter + a few maze walls with gaps.
    std::mt19937 rng(20260612u);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x)
            if (uni(rng) < 0.20) g.block(x, y);

    // Vertical walls with a single gap each (forces the search to route around).
    for (int wallX : {20, 40, 60}) {
        int gap = 5 + static_cast<int>(uni(rng) * (H - 10));
        for (int y = 0; y < H; ++y)
            if (y != gap && y != gap + 1) g.block(wallX, y);
    }
    // Keep endpoints clear.
    g.blocked[g.idx(start.x, start.y)] = 0;
    g.blocked[g.idx(goal.x, goal.y)] = 0;

    SearchResult a = astar(g, start, goal);
    SearchResult d = dijkstra(g, start, goal);
    SearchResult b = bfs(g, start, goal);

    printf("Grid: %dx%d  start=(%d,%d) goal=(%d,%d)\n", W, H, start.x, start.y, goal.x, goal.y);
    printf("A*       : found=%d  cost=%d  expanded=%ld  pathLen=%zu\n",
           a.found, a.pathCost, a.expanded, a.path.size());
    printf("Dijkstra : found=%d  cost=%d  expanded=%ld  pathLen=%zu\n",
           d.found, d.pathCost, d.expanded, d.path.size());
    printf("BFS      : found=%d  cost=%d  expanded=%ld  pathLen=%zu\n",
           b.found, b.pathCost, b.expanded, b.path.size());

    // Build explored mask from A* for visualization.
    std::vector<uint8_t> explored(static_cast<size_t>(W) * H, 0);
    {
        // Re-run a lightweight A* mark by flooding via closed set proxy:
        // simplest: mark all cells whose Dijkstra dist <= a.pathCost are reachable.
        // For an honest "explored" view we re-run A* recording closed nodes.
    }
    // Honest explored set: run A* again but capture closed nodes.
    {
        const int N = W * H;
        std::vector<int> gScore(N, INT32_MAX);
        std::vector<uint8_t> closed(N, 0);
        using Node = std::tuple<int, int, int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        int s = g.idx(start.x, start.y);
        gScore[s] = 0;
        open.push({manhattan(start, goal), 0, s});
        while (!open.empty()) {
            auto [f, gc, cur] = open.top(); open.pop(); (void)f; (void)gc;
            if (closed[cur]) continue;
            closed[cur] = 1;
            explored[cur] = 1;
            Vec2i cp{cur % W, cur / W};
            if (cp == goal) break;
            for (auto dir : kNeighbors) {
                int nx = cp.x + dir.x, ny = cp.y + dir.y;
                if (!g.inBounds(nx, ny) || g.isBlocked(nx, ny)) continue;
                int ni = g.idx(nx, ny);
                if (closed[ni]) continue;
                int t = gScore[cur] + 1;
                if (t < gScore[ni]) {
                    gScore[ni] = t;
                    open.push({t + manhattan({nx, ny}, goal), t, ni});
                }
            }
        }
    }

    writePPM("astar_output.ppm", g, a.path, explored, start, goal, 14);
    printf("Wrote astar_output.ppm\n");

    // ---------------- Quantitative verification ----------------
    int failures = 0;
    auto check = [&](bool cond, const char* msg) {
        printf("%s %s\n", cond ? "[PASS]" : "[FAIL]", msg);
        if (!cond) failures++;
    };

    check(a.found && d.found && b.found, "all three algorithms found a path");
    check(a.pathCost == d.pathCost, "A* cost == Dijkstra cost (A* optimal)");
    check(a.pathCost == b.pathCost, "A* cost == BFS cost (unit-cost shortest)");
    check(a.expanded <= d.expanded, "A* expands <= Dijkstra (heuristic helps)");
    check(static_cast<int>(a.path.size()) == a.pathCost + 1, "path length == cost+1 (no gaps)");

    // Path integrity: endpoints, adjacency, obstacle-free.
    bool endpointsOk = !a.path.empty() && a.path.front() == start && a.path.back() == goal;
    check(endpointsOk, "path starts at start and ends at goal");

    bool connected = true, clean = true;
    for (size_t i = 0; i < a.path.size(); ++i) {
        if (g.isBlocked(a.path[i].x, a.path[i].y)) clean = false;
        if (i > 0) {
            int md = manhattan(a.path[i - 1], a.path[i]);
            if (md != 1) connected = false;
        }
    }
    check(connected, "every consecutive path step is 4-adjacent");
    check(clean, "no path cell lies on an obstacle");

    printf("\n%s  (%d failure(s))\n", failures == 0 ? "ALL CHECKS PASSED" : "VERIFICATION FAILED",
           failures);
    return failures == 0 ? 0 : 1;
}
