#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <queue>
#include <functional>
#include <cmath>
#include <random>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace std;

typedef vector<int> VI;
typedef vector<VI> VVI;

// ---------------- Graph generation ----------------
struct Graph {
    int n;
    VVI adj;
    Graph(int n_) : n(n_), adj(n_) {}
    void addEdge(int u, int v) { adj[u].push_back(v); adj[v].push_back(u); }
};

// Random graph with given edge probability
Graph randomGraph(int n, double p, mt19937& rng) {
    Graph g(n);
    uniform_real_distribution<double> d(0.0, 1.0);
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++)
            if (d(rng) < p) g.addEdge(i, j);
    return g;
}

// Complete graph K_n (chromatic number = n)
Graph completeGraph(int n) {
    Graph g(n);
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) g.addEdge(i, j);
    return g;
}

// Cycle graph C_n (chromatic number 2 if even, 3 if odd)
Graph cycleGraph(int n) {
    Graph g(n);
    for (int i = 0; i < n; i++) g.addEdge(i, (i + 1) % n);
    return g;
}

// Random bipartite graph (chromatic number <= 2)
Graph bipartiteGraph(int n, double p, mt19937& rng) {
    Graph g(n);
    int half = n / 2;
    uniform_real_distribution<double> d(0.0, 1.0);
    for (int i = 0; i < half; i++)
        for (int j = half; j < n; j++)
            if (d(rng) < p) g.addEdge(i, j);
    return g;
}

// ---------------- Welsh-Powell greedy coloring ----------------
VI welshPowell(const Graph& g) {
    int n = g.n;
    VI order(n);
    for (int i = 0; i < n; i++) order[i] = i;
    // sort by degree descending
    sort(order.begin(), order.end(), [&](int a, int b) {
        return g.adj[a].size() > g.adj[b].size();
    });
    VI color(n, -1);
    int maxColor = -1;
    for (int u : order) {
        vector<bool> used(n, false);
        for (int v : g.adj[u]) if (color[v] >= 0) used[color[v]] = true;
        int c = 0;
        while (used[c]) c++;
        color[u] = c;
        maxColor = max(maxColor, c);
    }
    return color;
}

// ---------------- Exact coloring via backtracking (DANTZIG / DSATUR-ish) ----------------
// Finds the minimum number of colors (chromatic number) and returns the coloring
bool canColor(int v, int c, const VI& color, const Graph& g) {
    for (int u : g.adj[v]) if (color[u] == c) return false;
    return true;
}

bool backtrack(int idx, const VI& order, VI& color, int maxColors, const Graph& g) {
    if (idx == g.n) return true;
    int v = order[idx];
    for (int c = 0; c < maxColors; c++) {
        if (canColor(v, c, color, g)) {
            color[v] = c;
            if (backtrack(idx + 1, order, color, maxColors, g)) return true;
            color[v] = -1;
        }
    }
    return false;
}

// Compute chromatic number by trying 1, 2, 3, ... colors
int chromaticNumber(const Graph& g, VI& outColor) {
    int n = g.n;
    // Order vertices by degree descending (good branching heuristic)
    VI order(n);
    for (int i = 0; i < n; i++) order[i] = i;
    sort(order.begin(), order.end(), [&](int a, int b) {
        return g.adj[a].size() > g.adj[b].size();
    });
    // Lower bound: clique size approximation (max degree + 1 upper bound, clique lower bound)
    for (int k = 1; k <= n; k++) {
        VI color(n, -1);
        if (backtrack(0, order, color, k, g)) {
            outColor = color;
            return k;
        }
    }
    outColor.assign(n, 0);
    return n;
}

// ---------------- Proper coloring verifier ----------------
bool isProperColoring(const Graph& g, const VI& color) {
    for (int u = 0; u < g.n; u++)
        for (int v : g.adj[u])
            if (color[u] == color[v]) return false;
    return true;
}

int countColors(const VI& color) {
    int mx = -1;
    for (int c : color) mx = max(mx, c);
    return mx + 1;
}

// ---------------- Bipartite detection (2-colorability) ----------------
bool isBipartite(const Graph& g, VI& twoColor) {
    int n = g.n;
    twoColor.assign(n, -1);
    queue<int> q;
    for (int s = 0; s < n; s++) {
        if (twoColor[s] != -1) continue;
        twoColor[s] = 0;
        q.push(s);
        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int v : g.adj[u]) {
                if (twoColor[v] == -1) {
                    twoColor[v] = 1 - twoColor[u];
                    q.push(v);
                } else if (twoColor[v] == twoColor[u]) {
                    return false;
                }
            }
        }
    }
    return true;
}

// ---------------- PPM visualization ----------------
void writePPM(const char* path, const Graph& g, const VI& color) {
    // Lay out vertices on a circle
    int S = 800;
    int W = S, H = S;
    vector<unsigned char> img(W * H * 3, 255);
    int cx = S / 2, cy = S / 2, R = S / 2 - 50;

    // Palette (distinct colors)
    unsigned char palette[][3] = {
        {230, 57, 70}, {241, 196, 15}, {46, 204, 113}, {52, 152, 219},
        {155, 89, 182}, {230, 126, 34}, {26, 188, 156}, {231, 76, 60},
        {149, 165, 166}, {243, 156, 18}, {39, 174, 96}, {41, 128, 185},
        {142, 68, 173}, {211, 84, 0}, {127, 140, 141}, {192, 57, 43}
    };

    int n = g.n;
    vector<pair<double,double>> pos(n);
    for (int i = 0; i < n; i++) {
        double ang = 2.0 * M_PI * i / n - M_PI / 2;
        pos[i] = {cx + R * cos(ang), cy + R * sin(ang)};
    }

    auto setPixel = [&](int x, int y, unsigned char r, unsigned char gg, unsigned char b) {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        int idx = (y * W + x) * 3;
        img[idx] = r; img[idx+1] = gg; img[idx+2] = b;
    };

    // Draw edges
    for (int u = 0; u < n; u++) {
        for (int v : g.adj[u]) {
            if (v <= u) continue; // draw once
            double x0 = pos[u].first, y0 = pos[u].second;
            double x1 = pos[v].first, y1 = pos[v].second;
            int steps = max(1, (int)hypot(x1-x0, y1-y0));
            for (int s = 0; s <= steps; s++) {
                double t = (double)s / steps;
                int x = (int)(x0 + t*(x1-x0));
                int y = (int)(y0 + t*(y1-y0));
                setPixel(x, y, 180, 180, 180);
            }
        }
    }

    // Draw vertices as filled disks colored by their color
    for (int i = 0; i < n; i++) {
        int x = (int)pos[i].first, y = (int)pos[i].second;
        int r = 18;
        unsigned char* pal = palette[color[i] % 16];
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++)
                if (dx*dx + dy*dy <= r*r)
                    setPixel(x+dx, y+dy, pal[0], pal[1], pal[2]);
    }

    stbi_write_png(path, W, H, 3, img.data(), W * 3);
}

// ---------------- Main ----------------
int main() {
    mt19937 rng(12345);

    printf("=== Graph Coloring: Welsh-Powell + Backtracking ===\n\n");

    struct TestResult {
        string name;
        int n;
        int wpColors;
        int exactColors;
        bool proper;
        bool bipartite;
        bool bipartiteCorrect;
    };
    vector<TestResult> results;

    // Helper to run a test
    auto runTest = [&](const string& name, const Graph& g, bool expectBipartite) {
        VI wp = welshPowell(g);
        VI exact;
        int chi = chromaticNumber(g, exact);
        bool proper = isProperColoring(g, wp);
        int wpColors = countColors(wp);
        VI two;
        bool bip = isBipartite(g, two);

        results.push_back({name, g.n, wpColors, chi, proper, bip, expectBipartite == bip});

        printf("[%s] n=%d  Welsh-Powell=%d colors  chromatic=%d  proper=%s  bipartite=%s (expect=%s)\n",
               name.c_str(), g.n, wpColors, chi, proper ? "YES" : "NO",
               bip ? "YES" : "NO", expectBipartite ? "YES" : "NO");
    };

    // --- Deterministic correctness tests ---
    printf("--- Deterministic graphs ---\n");
    runTest("K5 (complete)", completeGraph(5), false);      // chi=5
    runTest("C5 (odd cycle)", cycleGraph(5), false);         // chi=3
    runTest("C8 (even cycle)", cycleGraph(8), true);         // chi=2
    runTest("K3,3 bipartite", bipartiteGraph(6, 1.0, rng), true); // chi=2
    // Petersen graph (chromatic number = 3, not bipartite)
    {
        Graph pet(10);
        int outer[5] = {0,1,2,3,4};
        int inner[5] = {5,6,7,8,9};
        for (int i = 0; i < 5; i++) {
            pet.addEdge(outer[i], outer[(i+1)%5]);
            pet.addEdge(inner[i], inner[(i+2)%5]);
            pet.addEdge(outer[i], inner[i]);
        }
        VI wp = welshPowell(pet);
        VI exact; int chi = chromaticNumber(pet, exact);
        VI two; bool bip = isBipartite(pet, two);
        results.push_back({"Petersen graph", 10, countColors(wp), chi, isProperColoring(pet, wp), bip, false == bip});
        printf("[Petersen graph] n=10  Welsh-Powell=%d  chromatic=%d  proper=%s  bipartite=%s (expect=NO)\n",
               countColors(wp), chi, isProperColoring(pet,wp)?"YES":"NO", bip?"YES":"NO");
    }

    printf("\n--- Random graphs ---\n");
    runTest("Random n=20 p=0.3", randomGraph(20, 0.3, rng), false);
    runTest("Random n=20 p=0.5", randomGraph(20, 0.5, rng), false);
    runTest("Random bipartite n=24", bipartiteGraph(24, 0.4, rng), true);

    // --- Quantitative verification ---
    printf("\n=== QUANTITATIVE VERIFICATION ===\n");
    bool allPass = true;
    int passCount = 0, total = 0;

    // 1. All WP colorings must be proper
    printf("\n[1] Proper coloring check (all graphs):\n");
    for (auto& r : results) {
        total++;
        printf("  %-22s proper=%s\n", r.name.c_str(), r.proper ? "PASS" : "FAIL");
        if (r.proper) passCount++; else allPass = false;
    }

    // 2. WP colors >= chromatic number (lower bound)
    printf("\n[2] Welsh-Powell >= chromatic number (upper-bound valid):\n");
    for (auto& r : results) {
        total++;
        bool ok = r.wpColors >= r.exactColors;
        printf("  %-22s wp=%d chi=%d -> %s\n", r.name.c_str(), r.wpColors, r.exactColors, ok ? "PASS" : "FAIL");
        if (ok) passCount++; else allPass = false;
    }

    // 3. Known chromatic numbers match exactly
    printf("\n[3] Known chromatic number correctness:\n");
    struct Known { string name; int expectedChi; };
    vector<Known> known = {{"K5 (complete)", 5}, {"C5 (odd cycle)", 3}, {"C8 (even cycle)", 2},
                           {"K3,3 bipartite", 2}, {"Petersen graph", 3}};
    for (auto& k : known) {
        for (auto& r : results) {
            if (r.name == k.name) {
                total++;
                bool ok = r.exactColors == k.expectedChi;
                printf("  %-22s chi=%d expect=%d -> %s\n", r.name.c_str(), r.exactColors, k.expectedChi, ok ? "PASS" : "FAIL");
                if (ok) passCount++; else allPass = false;
                break;
            }
        }
    }

    // 4. Bipartite detection correctness
    printf("\n[4] Bipartite detection correctness:\n");
    for (auto& r : results) {
        total++;
        bool ok = r.bipartiteCorrect;
        printf("  %-22s bipartite=%s expect_correct=%s -> %s\n", r.name.c_str(),
               r.bipartite ? "YES" : "NO", ok ? "PASS" : "FAIL", ok ? "PASS" : "FAIL");
        if (ok) passCount++; else allPass = false;
    }

    // 5. Clique lower bound: chromatic >= size of largest clique (K5 -> chi>=5)
    printf("\n[5] Chromatic >= clique lower bound:\n");
    {
        // For K5, largest clique = 5, so chi must be >= 5
        total++;
        bool ok = results[0].exactColors == 5; // K5
        printf("  K5 chromatic=5 (clique=5) -> %s\n", ok ? "PASS" : "FAIL");
        if (ok) passCount++; else allPass = false;
    }

    printf("\n=== RESULT: %d/%d checks passed ===\n", passCount, total);
    if (!allPass) { printf("❌ SOME CHECKS FAILED\n"); return 1; }
    printf("✅ ALL QUANTITATIVE CHECKS PASSED\n");

    // --- Visualization: render the random graph with exact coloring ---
    Graph viz = randomGraph(18, 0.35, rng);
    VI vc = welshPowell(viz);
    writePPM("graph_coloring_output.png", viz, vc);
    printf("\nVisualization written to graph_coloring_output.png (Welsh-Powell, %d colors)\n", countColors(vc));

    return 0;
}
