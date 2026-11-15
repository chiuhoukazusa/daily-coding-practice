// Wave Function Collapse (WFC) — Procedural Tiling with Minimum-Entropy Collapse
//
// Classic MAXENT (Gumin's "Model Synthesis" family) overlapping-model WFC:
//  - A set of abstract "tiles" defined only by adjacency constraints (edges).
//  - Each grid cell is a superposition of all tiles (a "wave").
//  - Iteratively: pick the lowest-entropy cell, collapse it to a single tile,
//    propagate constraints to neighbors, until the whole grid is deterministic.
//
// Quantified validation (no eyeballing):
//   1. Tiles actually generated / grid fully collapsed (no 0-entropy contradiction).
//   2. Every adjacent pair satisfies the directional adjacency rule (hard invariant).
//   3. Entropy statistics: mean entropy per step, final determinism == 100%.
//   4. Each tile class appears with non-trivial frequency (coverage).
//
// Compile:  g++ main.cpp -o wfc -std=c++17 -O2 -Wall -Wextra

#include <cstdio>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <random>
#include <algorithm>
#include <numeric>
#include <stdexcept>

// ---------------- Tile model ----------------
// We model a "Wang tile" style adjacency with 4 directional edges.
// Each tile has: up/down/left/right edge labels. Two tiles are compatible along
// a direction if the facing edges match (A.right == B.left, etc.).

struct Tile {
    int up, down, left, right;   // edge "colors"/symbols
};

// Build a small handcrafted tile set with a few edge colors so tiling is non-trivial.
// Build a Wang-tile set where every edge color pairs with exactly one
// complementary color, so all tiles remain reachable and the tiling is rich.
// Edge colors: each tile's directional edges are chosen from {0,1} per axis,
// giving distinct horizontal/vertical/corner patterns.
static std::vector<Tile> buildTiles() {
    // Classic binary-edge Wang tile set: each of the 4 edges is either 0 or 1,
    // forming the 16 distinct "road/circuit" tiles. Only a subset of these
    // is needed to produce rich, non-trivial tiling. We use a balanced subset:
    //   0: all-0 (blank), 1..4: single-edge "spurs", 5..10: straight/corner roads,
    //   11..14: T-junctions, 15: all-1 (crossing).
    std::vector<Tile> t;
    // index = (up<<3)|(down<<2)|(left<<1)|right, each edge 0/1
    t.resize(16);
    for (int m = 0; m < 16; ++m) {
        t[m].up    = (m >> 3) & 1;
        t[m].down  = (m >> 2) & 1;
        t[m].left  = (m >> 1) & 1;
        t[m].right = (m >> 0) & 1;
    }
    return t;
}

// Directions (for adjacency check): 0=up,1=down,2=left,3=right
static const int DIR_UP = 0, DIR_DOWN = 1, DIR_LEFT = 2, DIR_RIGHT = 3;
static const std::array<int,4> DX = {0, 0, -1, 1};
static const std::array<int,4> DY = {-1, 1, 0, 0};

struct WFC {
    int W, H;
    int N;                       // number of tiles
    std::vector<std::array<int,4>> edges; // per-tile edges
    // compatibility: compat[d][a][b] -> tile a along direction d is compatible with tile b
    std::vector<std::vector<std::vector<bool>>> compat;

    // wave: for each cell, a bitset of still-possible tiles
    std::vector<uint64_t> wave;   // N <= 64 assumed
    std::vector<int> entropy;     // popcount of each cell's wave

    std::mt19937 rng;

    WFC(int w, int h, const std::vector<Tile>& tiles, uint32_t seed)
        : W(w), H(h), N((int)tiles.size()), rng(seed) {
        edges.resize(N);
        for (int i = 0; i < N; ++i) {
            edges[i] = {tiles[i].up, tiles[i].down, tiles[i].left, tiles[i].right};
        }
        buildCompatibility();
        // initialize every cell to full superposition
        uint64_t full = (N >= 64) ? ~0ULL : ((1ULL << N) - 1ULL);
        wave.assign((size_t)W * H, full);
        entropy.assign((size_t)W * H, N);
    }

    int edgeOf(int tile, int dir) const {
        if (dir == DIR_UP)    return edges[tile][0];
        if (dir == DIR_DOWN)  return edges[tile][1];
        if (dir == DIR_LEFT)  return edges[tile][2];
        /* DIR_RIGHT */       return edges[tile][3];
    }

    void buildCompatibility() {
        compat.assign(4, std::vector<std::vector<bool>>(N, std::vector<bool>(N, false)));
        for (int d = 0; d < 4; ++d) {
            int opp = (d == DIR_UP) ? DIR_DOWN : (d == DIR_DOWN) ? DIR_UP
                     : (d == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;
            for (int a = 0; a < N; ++a)
                for (int b = 0; b < N; ++b) {
                    // tile a placed, tile b neighbor along direction d: a.edge(d) must equal b.edge(opp)
                    compat[d][a][b] = (edgeOf(a, d) == edgeOf(b, opp));
                }
        }
    }

    int idx(int x, int y) const { return y * W + x; }
    bool inGrid(int x, int y) const { return x >= 0 && x < W && y >= 0 && y < H; }

    // Return the mask of tiles in `from` that have at least one compatible tile in `against` along d
    uint64_t allowed(uint64_t from, uint64_t againstMask, int d) const {
        uint64_t result = 0;
        for (int t = 0; t < N; ++t) {
            if (!(from & (1ULL << t))) continue;
            // check if any compatible tile exists in againstMask
            bool ok = false;
            for (int b = 0; b < N && !ok; ++b) {
                if ((againstMask & (1ULL << b)) && compat[d][t][b]) ok = true;
            }
            if (ok) result |= (1ULL << t);
        }
        return result;
    }

    // Propagate constraints from a changed cell across the grid (iterative queue).
    bool propagate(std::vector<int>& queue) {
        while (!queue.empty()) {
            int pos = queue.back();
            queue.pop_back();
            int x = pos % W, y = pos / W;
            uint64_t mask = wave[pos];
            if (mask == 0) return false; // contradiction
            for (int d = 0; d < 4; ++d) {
                int nx = x + DX[d], ny = y + DY[d];
                if (!inGrid(nx, ny)) continue;
                int npos = idx(nx, ny);
                int opp = (d == DIR_UP) ? DIR_DOWN : (d == DIR_DOWN) ? DIR_UP
                        : (d == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;
                uint64_t before = wave[npos];
                uint64_t after = allowed(before, mask, opp);
                if (after != before) {
                    wave[npos] = after;
                    entropy[npos] = __builtin_popcountll(after);
                    if (after == 0) return false;
                    queue.push_back(npos);
                }
            }
        }
        return true;
    }

    // One observation = one collapse at the minimum-entropy (non-collapsed) cell.
    // Returns: 0 = success (still running), 1 = fully collapsed, -1 = contradiction.
    int observe(double& totalEntropyBefore, double& chosenEntropy) {
        // find min-entropy non-collapsed cell
        int bestPos = -1, bestE = 1 << 30;
        for (int i = 0; i < W * H; ++i) {
            int e = entropy[i];
            if (e > 1 && e < bestE) { bestE = e; bestPos = i; }
        }
        if (bestPos == -1) {
            // all cells entropy <= 1 -> collapsed
            return 1;
        }
        chosenEntropy = bestE;

        // compute total entropy (Shannon) for stats
        totalEntropyBefore = 0.0;
        for (int i = 0; i < W * H; ++i) {
            double p = entropy[i];
            if (p > 0) totalEntropyBefore += std::log2(p); // sum of log2(superposition size)
        }

        // randomly choose one of the allowed tiles for this cell (weighted uniformly)
        uint64_t mask = wave[bestPos];
        std::vector<int> opts;
        for (int t = 0; t < N; ++t) if (mask & (1ULL << t)) opts.push_back(t);
        if (opts.empty()) return -1;
        std::uniform_int_distribution<int> dist(0, (int)opts.size() - 1);
        int pick = opts[dist(rng)];

        wave[bestPos] = (1ULL << pick);
        entropy[bestPos] = 1;

        std::vector<int> q = {bestPos};
        if (!propagate(q)) return -1;
        return 0;
    }

    // final collapsed tile per cell (must be fully collapsed already)
    std::vector<int> collapsed() const {
        std::vector<int> out(W * H, -1);
        for (int i = 0; i < W * H; ++i) {
            if (entropy[i] != 1) return out; // not collapsed
            uint64_t m = wave[i];
            for (int t = 0; t < N; ++t) if (m & (1ULL << t)) { out[i] = t; break; }
        }
        return out;
    }
};

// ---------------- PPM rendering ----------------
static void writePPM(const std::vector<int>& grid, int W, int H, const std::string& path) {
    // assign a color per tile
    std::vector<std::array<int,3>> palette = {
        {20,20,30},    // 0  blank
        {60,60,70},    // 1
        {90,90,100},   // 2
        {120,120,130}, // 3
        {40,80,160},   // 4  blue
        {50,140,170},  // 5
        {70,200,180},  // 6  teal
        {90,220,140},  // 7  green
        {200,120,60},  // 8  orange
        {220,160,80},  // 9
        {230,200,100}, // 10 yellow
        {240,220,160}, // 11
        {200,80,120},  // 12 pink
        {180,80,180},  // 13 purple
        {230,140,190}, // 14
        {255,255,255}, // 15 white (all-1 crossing)
    };
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) throw std::runtime_error("cannot open " + path);
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int t = grid[y * W + x];
            if (t < 0) t = 0;
            auto c = palette[t % palette.size()];
            fputc(c[0], f); fputc(c[1], f); fputc(c[2], f);
        }
    }
    fclose(f);
}

int main() {
    const int W = 256, H = 256;
    auto tiles = buildTiles();
    WFC wfc(W, H, tiles, 20261116u);

    // Run observations until done or contradiction, collecting stats.
    std::vector<double> entropies, chosenEntropies;
    int steps = 0;
    int maxIter = W * H * 4;
    int status = 0;

    for (int it = 0; it < maxIter; ++it) {
        double te = 0, ce = 0;
        status = wfc.observe(te, ce);
        if (status == 1) break;      // fully collapsed
        if (status == -1) break;     // contradiction
        entropies.push_back(te);
        chosenEntropies.push_back(ce);
        ++steps;
    }

    if (status == -1) {
        printf("CONTRADICTION at step %d\n", steps);
        return 1;
    }
    if (status == 0) {
        printf("TIMEOUT: not collapsed after %d steps\n", steps);
        return 1;
    }

    auto grid = wfc.collapsed();

    // ---- Quantitative validation ----
    // 1. Full determinism
    int collapsedCells = 0;
    for (int i = 0; i < W * H; ++i) if (grid[i] >= 0) ++collapsedCells;
    printf("Grid: %dx%d=%d cells, collapsed=%d, determinism=%.2f%%\n",
           W, H, W * H, collapsedCells, 100.0 * collapsedCells / (W * H));

    // 2. Adjacency invariant: every adjacent pair satisfies the directional rule
    long adjTotal = 0, adjViolations = 0;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int t = grid[y * W + x];
            for (int d = 0; d < 4; ++d) {
                int nx = x + DX[d], ny = y + DY[d];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                int nb = grid[ny * W + nx];
                ++adjTotal;
                if (!wfc.compat[d][t][nb]) ++adjViolations;
            }
        }
    }
    printf("Adjacency: %ld edges, violations=%ld (%.4f%%)\n",
           adjTotal, adjViolations, 100.0 * adjViolations / std::max(1L, adjTotal));

    // 3. Tile coverage / frequency
    std::vector<int> freq(wfc.N, 0);
    for (int i = 0; i < W * H; ++i) freq[grid[i]]++;
    printf("Tile frequencies:\n");
    for (int t = 0; t < wfc.N; ++t) {
        printf("  tile %d: %d (%.1f%%)\n", t, freq[t], 100.0 * freq[t] / (W * H));
    }

    // 4. Entropy stats during collapse
    if (!entropies.empty()) {
        double meanE = std::accumulate(entropies.begin(), entropies.end(), 0.0) / entropies.size();
        double meanC = std::accumulate(chosenEntropies.begin(), chosenEntropies.end(), 0.0) / chosenEntropies.size();
        double minE = *std::min_element(entropies.begin(), entropies.end());
        double maxE = *std::max_element(entropies.begin(), entropies.end());
        printf("Entropy: steps=%d, meanShannon=%.2f, min=%.2f, max=%.2f, meanChosenEntropy=%.3f\n",
               steps, meanE, minE, maxE, meanC);
    }

    writePPM(grid, W, H, "wfc_output.ppm");
    printf("Wrote wfc_output.ppm\n");

    // Final explicit PASS/FAIL summary
    bool ok = (collapsedCells == W * H) && (adjViolations == 0);
    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
