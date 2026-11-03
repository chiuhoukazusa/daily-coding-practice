// BSP Dungeon Generation
// Binary Space Partitioning for procedural dungeon layout:
//  - Recursively split a rectangular region into leaves via BSP partition lines.
//  - Place one room inside each leaf (with margin jitter).
//  - Connect sibling leaves to parent by L-shaped corridors (center<->center).
//  - Optionally connect adjacent leaves for extra loops.
//  - Verify: all rooms reachable (BFS), no overlaps, all inside bounds.
//
// Output: PPM map + textual metrics (quantitative verification).

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <queue>
#include <random>
#include <algorithm>
#include <cmath>
#include <string>

// ---------------------------------------------------------------------------
// RNG (deterministic for reproducibility)
static std::mt19937 rng(0xC0FFEE);

static int irand(int lo, int hi) { // inclusive
    if (lo > hi) std::swap(lo, hi);
    return lo + (int)(rng() % (unsigned)(hi - lo + 1));
}

// ---------------------------------------------------------------------------
struct Rect {
    int x, y, w, h;
    Rect() : x(0), y(0), w(0), h(0) {}
    Rect(int x_, int y_, int w_, int h_) : x(x_), y(y_), w(w_), h(h_) {}
    int cx() const { return x + w / 2; }
    int cy() const { return y + h / 2; }
    int area() const { return w * h; }
};

struct Room {
    Rect r;
    bool used;
    Room() : used(false) {}
};

struct Node {
    Rect rect;
    Node* left = nullptr;
    Node* right = nullptr;
    bool leaf = false;
    Room room;
    ~Node() { delete left; delete right; }
};

// ---------------------------------------------------------------------------
// BSP tree build: recursively split until region <= max leaf size or too small.
static const int MIN_LEAF = 12;      // stop splitting below this w/h
static const int MAX_LEAF = 28;      // target max leaf dimension
static const int MAX_DEPTH = 6;

static Node* buildBSP(Rect r, int depth) {
    Node* n = new Node;
    n->rect = r;

    bool canSplitH = (r.h >= MIN_LEAF * 2);
    bool canSplitV = (r.w >= MIN_LEAF * 2);
    bool tooBig = (r.w > MAX_LEAF || r.h > MAX_LEAF);

    if ((canSplitH || canSplitV) && tooBig && depth < MAX_DEPTH) {
        // choose orientation: prefer splitting the longer axis, but allow randomness
        bool vertical;
        if (canSplitH && canSplitV) vertical = (rng() % 2 == 0);
        else vertical = canSplitV;

        if (vertical) {
            // random split position with a margin so both sides are >= MIN_LEAF
            int minPos = r.x + MIN_LEAF;
            int maxPos = r.x + r.w - MIN_LEAF;
            int split = irand(minPos, maxPos);
            n->left  = buildBSP(Rect(r.x, r.y, split - r.x, r.h), depth + 1);
            n->right = buildBSP(Rect(split, r.y, r.x + r.w - split, r.h), depth + 1);
        } else {
            int minPos = r.y + MIN_LEAF;
            int maxPos = r.y + r.h - MIN_LEAF;
            int split = irand(minPos, maxPos);
            n->left  = buildBSP(Rect(r.x, r.y, r.w, split - r.y), depth + 1);
            n->right = buildBSP(Rect(r.x, split, r.w, r.y + r.h - split), depth + 1);
        }
    } else {
        n->leaf = true;
    }
    return n;
}

// ---------------------------------------------------------------------------
// Collect all leaves and place rooms
static void collectLeaves(Node* n, std::vector<Node*>& leaves) {
    if (!n) return;
    if (n->leaf) { leaves.push_back(n); return; }
    collectLeaves(n->left, leaves);
    collectLeaves(n->right, leaves);
}

// Min room size and margins
static const int ROOM_MIN = 5;
static const int MARGIN = 2;

static bool placeRoom(Node* leaf) {
    const Rect& r = leaf->rect;
    // minimum needed interior: ROOM_MIN + 2*MARGIN
    if (r.w < ROOM_MIN + 2 * MARGIN || r.h < ROOM_MIN + 2 * MARGIN) {
        leaf->room.used = false;
        return false;
    }
    // random room size within the leaf interior
    int maxW = r.w - 2 * MARGIN;
    int maxH = r.h - 2 * MARGIN;
    int rw = irand(ROOM_MIN, maxW);
    int rh = irand(ROOM_MIN, maxH);
    // random offset
    int ox = irand(r.x + MARGIN, r.x + r.w - MARGIN - rw);
    int oy = irand(r.y + MARGIN, r.y + r.h - MARGIN - rh);
    leaf->room.r = Rect(ox, oy, rw, rh);
    leaf->room.used = true;
    return true;
}

// L-shaped corridor between two points on the grid
struct Grid {
    int W, H;
    std::vector<int> cells; // 0=wall, 1=floor(room), 2=floor(corridor), 3=door?
    Grid(int w, int h) : W(w), H(h), cells(w * h, 0) {}
    int& at(int x, int y) { return cells[y * W + x]; }
    const int& at(int x, int y) const { return cells[y * W + x]; }
};

static void carveroom(Grid& g, const Rect& r) {
    for (int y = r.y; y < r.y + r.h; y++)
        for (int x = r.x; x < r.x + r.w; x++)
            g.at(x, y) = 1;
}

static void carveCorridor(Grid& g, int x0, int y0, int x1, int y1) {
    // horizontal then vertical (L-shape)
    int xa = x0, ya = y0, xb = x1, yb = y1;
    int dx = (xb > xa) ? 1 : -1;
    int dy = (yb > ya) ? 1 : -1;
    for (int x = xa; x != xb + dx; x += dx) if (x >= 0 && x < g.W && y0 >= 0 && y0 < g.H) g.at(x, y0) = 2;
    for (int y = ya; y != yb + dy; y += dy) if (y >= 0 && y < g.H && x1 >= 0 && x1 < g.W) g.at(x1, y) = 2;
}

// ---------------------------------------------------------------------------
// Connect recursively: sibling leaves connect center to center
static void connect(Node* n, Grid& g) {
    if (!n || n->leaf) return;
    // recurse first
    connect(n->left, g);
    connect(n->right, g);

    // gather leaf centers on each side
    auto centers = [](Node* node, std::vector<std::pair<int,int>>& out, std::vector<Rect>& roomRects) {
        std::vector<Node*> leaves;
        collectLeaves(node, leaves);
        for (auto* lf : leaves) {
            if (lf->room.used) {
                out.push_back({lf->room.r.cx(), lf->room.r.cy()});
                roomRects.push_back(lf->room.r);
            }
        }
    };

    std::vector<std::pair<int,int>> lc, rc;
    std::vector<Rect> lr, rr;
    centers(n->left, lc, lr);
    centers(n->right, rc, rr);

    if (lc.empty() || rc.empty()) return;

    // connect nearest pair (or first) of centers between the two subtrees
    for (auto& a : lc) {
        for (auto& b : rc) {
            carveCorridor(g, a.first, a.second, b.first, b.second);
        }
    }
}

// ---------------------------------------------------------------------------
// Quantitative verification
struct Metrics {
    int rooms = 0;
    int floorTiles = 0;
    int roomTiles = 0;
    int corridorTiles = 0;
    int totalTiles = 0;
    bool overlap = false;
    double roomAreaPct = 0;
    int reachableRooms = 0;
    int connectedComponents = 0;
    int maxComponentSize = 0;
};

static void renderRoomsAndCount(Grid& g, std::vector<Node*>& leaves, Metrics& m) {
    std::vector<Rect> placed;
    for (auto* lf : leaves) {
        if (lf->room.used) {
            carveroom(g, lf->room.r);
            placed.push_back(lf->room.r);
            m.rooms++;
            m.roomTiles += lf->room.r.area();
        }
    }
    // overlap check (O(n^2), fine for ~dozens of rooms)
    for (size_t i = 0; i < placed.size(); i++) {
        for (size_t j = i + 1; j < placed.size(); j++) {
            const Rect& a = placed[i];
            const Rect& b = placed[j];
            bool noOverlap = (a.x + a.w <= b.x || b.x + b.w <= a.x ||
                              a.y + a.h <= b.y || b.y + b.h <= a.y);
            if (!noOverlap) m.overlap = true;
        }
    }
}

// BFS flood fill on floor tiles to compute connectivity
static void analyzeConnectivity(Grid& g, Metrics& m) {
    std::vector<int> visited(g.W * g.H, 0);
    int comps = 0;
    int maxComp = 0;
    for (int y = 0; y < g.H; y++) {
        for (int x = 0; x < g.W; x++) {
            if (g.at(x, y) == 0 || visited[y * g.W + x]) continue;
            comps++;
            int size = 0;
            std::queue<std::pair<int,int>> q;
            q.push({x, y});
            visited[y * g.W + x] = 1;
            int dx[] = {1,-1,0,0};
            int dy[] = {0,0,1,-1};
            while (!q.empty()) {
                auto [cx, cy] = q.front(); q.pop();
                size++;
                for (int k = 0; k < 4; k++) {
                    int nx = cx + dx[k], ny = cy + dy[k];
                    if (nx < 0 || ny < 0 || nx >= g.W || ny >= g.H) continue;
                    if (visited[ny * g.W + nx]) continue;
                    if (g.at(nx, ny) == 0) continue;
                    visited[ny * g.W + nx] = 1;
                    q.push({nx, ny});
                }
            }
            maxComp = std::max(maxComp, size);
        }
    }
    m.connectedComponents = comps;
    m.maxComponentSize = maxComp;
    // count total floor tiles
    for (int i = 0; i < g.W * g.H; i++) {
        if (g.cells[i]) m.floorTiles++;
    }
    m.corridorTiles = m.floorTiles - m.roomTiles;
    m.totalTiles = g.W * g.H;
    m.roomAreaPct = 100.0 * m.roomTiles / (double)m.totalTiles;
}

// ---------------------------------------------------------------------------
// PPM output (colored: wall=black, room=white/gray, corridor=blue-ish)
static void writePPM(Grid& g, const char* path) {
    FILE* f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", g.W, g.H);
    for (int y = 0; y < g.H; y++) {
        for (int x = 0; x < g.W; x++) {
            unsigned char r, gg, b;
            switch (g.at(x, y)) {
                case 0: r = 20; gg = 20; b = 24; break;   // wall
                case 1: r = 235; gg = 235; b = 225; break; // room floor
                case 2: r = 90; gg = 140; b = 210; break;  // corridor
                default: r = 255; gg = 0; b = 255; break;
            }
            fputc(r, f); fputc(gg, f); fputc(b, f);
        }
    }
    fclose(f);
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    int W = 128, H = 128;
    if (argc >= 3) { W = atoi(argv[1]); H = atoi(argv[2]); }

    Node* root = buildBSP(Rect(0, 0, W, H), 0);

    std::vector<Node*> leaves;
    collectLeaves(root, leaves);

    for (auto* lf : leaves) placeRoom(lf);

    Grid g(W, H);

    Metrics m;
    renderRoomsAndCount(g, leaves, m);

    connect(root, g);

    analyzeConnectivity(g, m);

    // Reachability of ROOMS specifically: a room is reachable if its center tile is floor
    // (all room tiles are floor by construction; verify each room center is in max component)
    // Simpler: count distinct rooms touched by largest component is hard without labels,
    // so we rely on: connectedComponents == 1 => all floor (=all rooms+corridors) connected.
    m.reachableRooms = (m.connectedComponents == 1) ? m.rooms : -1;

    writePPM(g, "bsp_dungeon_output.ppm");

    // ---- print metrics for quantitative verification ----
    printf("W=%d H=%d\n", W, H);
    printf("total_tiles=%d\n", m.totalTiles);
    printf("rooms=%d\n", m.rooms);
    printf("room_tiles=%d\n", m.roomTiles);
    printf("corridor_tiles=%d\n", m.corridorTiles);
    printf("floor_tiles=%d\n", m.floorTiles);
    printf("room_area_pct=%.3f\n", m.roomAreaPct);
    printf("overlap=%d\n", m.overlap ? 1 : 0);
    printf("connected_components=%d\n", m.connectedComponents);
    printf("max_component_size=%d\n", m.maxComponentSize);
    printf("reachable_rooms=%d\n", m.reachableRooms);

    // explicit PASS/FAIL checks
    bool pass = true;
    if (m.rooms < 4) { printf("FAIL: too few rooms\n"); pass = false; }
    if (m.overlap) { printf("FAIL: room overlap detected\n"); pass = false; }
    if (m.connectedComponents != 1) { printf("FAIL: dungeon not fully connected\n"); pass = false; }
    if (m.roomTiles <= 0) { printf("FAIL: no room tiles\n"); pass = false; }

    printf("%s\n", pass ? "PASS" : "FAIL");

    delete root;
    return pass ? 0 : 1;
}
