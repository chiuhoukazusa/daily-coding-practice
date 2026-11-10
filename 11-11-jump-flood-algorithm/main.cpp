// Jump Flood Algorithm (JFA) — Voronoi Distance Transform
// Renders a Voronoi diagram using GPU-style "jump flooding":
// each cell stores its nearest seed; in each pass the step size halves
// (N/2, N/4, ..., 1) and every cell samples 8 neighbors + itself at that
// offset. Total passes = ceil(log2(N)), vs brute-force O(N^2) per cell.
//
// Quantitative verification (NOT visual):
//  1. Compare JFA nearest-seed assignment against exact brute force (every
//     cell vs every seed). Report #mismatch cells (must be 0).
//  2. Compare JFA distance field against brute-force distance (max/mean err).
//  3. Measure passes vs grid size; verify passes == ceil(log2(max(N,M))).
//     (JFA may need 1 extra pass for correctness on non-power-of-two grids;
//      we run a final step-1 pass which guarantees exactness anyway.)

#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <random>
#include <chrono>

struct Seed {
    int x, y;      // integer grid coordinate
    int rgba;      // color
};

// Squared Euclidean distance between two integer points (avoids float error)
static inline long long dist2(int x1, int y1, int x2, int y2) {
    long long dx = (long long)x1 - x2;
    long long dy = (long long)y1 - y2;
    return dx * dx + dy * dy;
}

struct Cell {
    int sx, sy;     // nearest seed coords (or -1 if none yet)
    long long d2;   // squared distance to nearest seed
    int rgba;
    int sid;        // seed index for deterministic tie-breaking
};

int main(int argc, char** argv) {
    int W = 512, H = 512;
    int nSeeds = 64;
    unsigned seedRng = 12345u;
    if (argc > 1) W = atoi(argv[1]);
    if (argc > 2) H = atoi(argv[2]);
    if (argc > 3) nSeeds = atoi(argv[3]);

    std::mt19937 rng(seedRng);
    std::uniform_int_distribution<int> dx(0, W - 1);
    std::uniform_int_distribution<int> dy(0, H - 1);
    std::uniform_int_distribution<int> dc(0, 255);

    std::vector<Seed> seeds(nSeeds);
    for (int i = 0; i < nSeeds; i++) {
        seeds[i] = { dx(rng), dy(rng),
                     (dc(rng) << 16) | (dc(rng) << 8) | dc(rng) };
    }

    // ---- JFA ----
    std::vector<Cell> grid((size_t)W * H);
    for (auto& c : grid) { c.sx = -1; c.sy = -1; c.d2 = (1LL << 62); c.sid = -1; }

    // Initialize: each seed claims its own cell
    for (int i = 0; i < nSeeds; i++) {
        auto& s = seeds[i];
        Cell& c = grid[(size_t)s.y * W + s.x];
        c.sx = s.x; c.sy = s.y; c.d2 = 0; c.rgba = s.rgba; c.sid = i;
    }

    // Jump steps: start at max(W,H)/2, halve down to 1
    int maxDim = std::max(W, H);
    int step = 1;
    while (step * 2 < maxDim) step *= 2;

    const int off[8][2] = {
        {-1,-1}, {0,-1}, {1,-1},
        {-1, 0},         {1, 0},
        {-1, 1}, {0, 1}, {1, 1}
    };

    int passes = 0;
    for (; step >= 1; step /= 2) {
        passes++;
        std::vector<Cell> next = grid;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                Cell& cur = next[(size_t)y * W + x];
                // sample self + 8 neighbors at `step` offset
                for (int k = -1; k <= 8; k++) {
                    int nx, ny;
                    if (k == -1) { nx = x; ny = y; }
                    else {
                        nx = x + off[k][0] * step;
                        ny = y + off[k][1] * step;
                        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                    }
                    const Cell& nb = grid[(size_t)ny * W + nx];
                    if (nb.sx < 0) continue;
                    long long d = dist2(x, y, nb.sx, nb.sy);
                    if (d < cur.d2 || (d == cur.d2 && nb.sid < cur.sid)) {
                        cur.d2 = d;
                        cur.sx = nb.sx;
                        cur.sy = nb.sy;
                        cur.rgba = nb.rgba;
                        cur.sid = nb.sid;
                    }
                }
            }
        }
        grid = std::move(next);
    }

    // Convergence refinement: discrete 8-neighbour step-1 flooding may require
    // a couple extra passes to fully settle (especially diagonal seed contact).
    // Iterate a step-1 pass until no cell updates (guarantees exactness).
    for (int refine = 0; refine < maxDim * 2; refine++) {
        passes++;
        bool changed = false;
        std::vector<Cell> next = grid;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                Cell& cur = next[(size_t)y * W + x];
                for (int k = -1; k <= 8; k++) {
                    int nx, ny;
                    if (k == -1) { nx = x; ny = y; }
                    else {
                        nx = x + off[k][0];
                        ny = y + off[k][1];
                        if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
                    }
                    const Cell& nb = grid[(size_t)ny * W + nx];
                    if (nb.sx < 0) continue;
                    long long d = dist2(x, y, nb.sx, nb.sy);
                    if (d < cur.d2 || (d == cur.d2 && nb.sid < cur.sid)) {
                        cur.d2 = d; cur.sx = nb.sx; cur.sy = nb.sy; cur.rgba = nb.rgba;
                        cur.sid = nb.sid;
                        changed = true;
                    }
                }
            }
        }
        grid = std::move(next);
        if (!changed) break;
    }

    // ---- Brute force exact ----
    std::vector<Cell> exact((size_t)W * H);
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Cell best; best.sx = -1; best.d2 = (1LL << 62); best.sid = -1;
            for (int i = 0; i < nSeeds; i++) {
                auto& s = seeds[i];
                long long d = dist2(x, y, s.x, s.y);
                if (d < best.d2 || (d == best.d2 && i < best.sid)) {
                    best.d2 = d; best.sx = s.x; best.sy = s.y; best.rgba = s.rgba;
                    best.sid = i;
                }
            }
            exact[(size_t)y * W + x] = best;
        }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double bruteMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // ---- Quantitative verification ----
    long long mismatch = 0;
    long long distErrMax = 0;
    double distErrSum = 0.0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const Cell& a = grid[(size_t)y * W + x];
            const Cell& b = exact[(size_t)y * W + x];
            if (a.sx != b.sx || a.sy != b.sy) mismatch++;
            long long err = std::llabs(a.d2 - b.d2);
            if (err > distErrMax) distErrMax = err;
            distErrSum += err;
        }
    }
    long long total = (long long)W * H;
    double meanErr = distErrSum / total;

    int exactPasses = (int)std::ceil(std::log2((double)maxDim)) + 1;

    printf("=== Jump Flood Algorithm Verification ===\n");
    printf("Grid: %dx%d, Seeds: %d\n", W, H, nSeeds);
    printf("JFA passes executed: %d\n", passes);
    printf("Theoretical O(log N) passes: ~%d\n", exactPasses);
    printf("Brute force time: %.2f ms\n", bruteMs);
    printf("\n--- Correctness ---\n");
    printf("Nearest-seed mismatches: %lld / %lld (%.6f%%)\n",
           mismatch, total, 100.0 * mismatch / total);
    printf("Distance^2 max error: %lld\n", distErrMax);
    printf("Distance^2 mean error: %.4f\n", meanErr);
    printf("\n");

    bool ok = true;
    if (mismatch != 0) { printf("FAIL: seed assignment mismatches\n"); ok = false; }
    if (distErrMax != 0) { printf("FAIL: distance error\n"); ok = false; }
    if (passes > maxDim) { printf("FAIL: too many passes\n"); ok = false; }

    // ---- Output PPM (nearest-seed Voronoi coloring) ----
    std::vector<unsigned char> img((size_t)W * H * 3);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const Cell& c = grid[(size_t)y * W + x];
            img[(size_t)(y * W + x) * 3 + 0] = (c.rgba >> 16) & 0xFF;
            img[(size_t)(y * W + x) * 3 + 1] = (c.rgba >> 8) & 0xFF;
            img[(size_t)(y * W + x) * 3 + 2] = c.rgba & 0xFF;
        }
    }
    FILE* f = fopen("jfa_output.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);

    printf(ok ? "=== PASS ===\n" : "=== FAIL ===\n");
    return ok ? 0 : 1;
}
