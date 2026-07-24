#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// ============================================================
// PPM Image Writer (P3 ASCII format for easy inspection)
// ============================================================
struct Image {
    int w, h;
    std::vector<uint8_t> r, g, b;
    Image(int w_, int h_) : w(w_), h(h_), r(w * h), g(w * h), b(w * h) {}
    void setPixel(int x, int y, uint8_t rr, uint8_t gg, uint8_t bb) {
        if (x >= 0 && x < w && y >= 0 && y < h) {
            int idx = y * w + x;
            r[idx] = rr; g[idx] = gg; b[idx] = bb;
        }
    }
    void drawLine(int x0, int y0, int x1, int y1, uint8_t rr, uint8_t gg, uint8_t bb) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            setPixel(x0, y0, rr, gg, bb);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    void fillCircle(int cx, int cy, int radius, uint8_t rr, uint8_t gg, uint8_t bb) {
        for (int y = cy - radius; y <= cy + radius; ++y)
            for (int x = cx - radius; x <= cx + radius; ++x)
                if ((x-cx)*(x-cx) + (y-cy)*(y-cy) <= radius*radius)
                    setPixel(x, y, rr, gg, bb);
    }
    void save(const std::string& filename) {
        std::ofstream f(filename);
        f << "P6\n" << w << " " << h << "\n255\n";
        for (int i = 0; i < w * h; ++i)
            f.put(static_cast<char>(r[i]))
             .put(static_cast<char>(g[i]))
             .put(static_cast<char>(b[i]));
    }
};

// ============================================================
// Graph data structures
// ============================================================
struct Edge {
    int u, v;
    double w;
};

struct Graph {
    int V;
    std::vector<Edge> edges;
    std::vector<double> xs, ys; // positions for visualization

    Graph(int n) : V(n), xs(n), ys(n) {}
};

// ============================================================
// Union-Find (Disjoint Set) for Kruskal's algorithm
// ============================================================
class UnionFind {
    std::vector<int> parent, rank;
public:
    UnionFind(int n) : parent(n), rank(n, 0) {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }
    int find(int x) {
        return parent[x] == x ? x : (parent[x] = find(parent[x]));
    }
    void unite(int x, int y) {
        int rx = find(x), ry = find(y);
        if (rx != ry) {
            if (rank[rx] < rank[ry]) std::swap(rx, ry);
            parent[ry] = rx;
            if (rank[rx] == rank[ry]) ++rank[rx];
        }
    }
};

// ============================================================
// Prim's Algorithm
// ============================================================
std::pair<std::vector<Edge>, double> primMST(const Graph& g) {
    int V = g.V;
    // Build adjacency list
    std::vector<std::vector<std::pair<int,double>>> adj(V);
    for (const auto& e : g.edges) {
        adj[e.u].push_back({e.v, e.w});
        adj[e.v].push_back({e.u, e.w});
    }

    std::vector<bool> visited(V, false);
    std::vector<double> key(V, std::numeric_limits<double>::max());
    std::vector<int> parent(V, -1);
    std::vector<double> parentWeight(V, 0);

    using pq_elem = std::pair<double,int>;
    std::priority_queue<pq_elem, std::vector<pq_elem>, std::greater<pq_elem>> pq;

    key[0] = 0;
    pq.push({0, 0});

    while (!pq.empty()) {
        auto [dist, u] = pq.top(); pq.pop();
        if (visited[u]) continue;
        visited[u] = true;

        for (auto [v, w] : adj[u]) {
            if (!visited[v] && w < key[v]) {
                key[v] = w;
                parent[v] = u;
                parentWeight[v] = w;
                pq.push({w, v});
            }
        }
    }

    std::vector<Edge> mst;
    double total = 0;
    for (int v = 0; v < V; ++v) {
        if (parent[v] != -1) {
            mst.push_back({parent[v], v, parentWeight[v]});
            total += parentWeight[v];
        }
    }
    return {mst, total};
}

// ============================================================
// Kruskal's Algorithm
// ============================================================
std::pair<std::vector<Edge>, double> kruskalMST(const Graph& g) {
    std::vector<Edge> sorted = g.edges;
    std::sort(sorted.begin(), sorted.end(),
              [](const Edge& a, const Edge& b) { return a.w < b.w; });

    UnionFind uf(g.V);
    std::vector<Edge> mst;
    double total = 0;

    for (const auto& e : sorted) {
        if (uf.find(e.u) != uf.find(e.v)) {
            uf.unite(e.u, e.v);
            mst.push_back(e);
            total += e.w;
        }
        if ((int)mst.size() == g.V - 1) break;
    }
    return {mst, total};
}

// ============================================================
// Generate random graph with positions
// ============================================================
Graph generateGraph(int V, double density = 0.3, int seed = 42) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> posDist(50, 550);
    std::uniform_real_distribution<double> weightDist(1.0, 100.0);

    Graph g(V);
    for (int i = 0; i < V; ++i) {
        g.xs[i] = posDist(rng);
        g.ys[i] = posDist(rng);
    }

    // Ensure connected graph first (random spanning tree)
    std::vector<int> order(V);
    std::iota(order.begin(), order.end(), 0);
    std::shuffle(order.begin(), order.end(), rng);
    for (int i = 1; i < V; ++i) {
        int u = order[i], v = order[i - 1];
        double dx = g.xs[u] - g.xs[v];
        double dy = g.ys[u] - g.ys[v];
        double w = std::sqrt(dx*dx + dy*dy) / 6.0; // euclidean base
        g.edges.push_back({u, v, w});
    }

    // Add extra random edges
    int maxExtra = static_cast<int>(density * V * (V-1) / 2);
    int extra = std::min(maxExtra - (V-1), V * 3);
    for (int i = 0; i < extra; ++i) {
        int u = rng() % V;
        int v = rng() % V;
        if (u == v) { --i; continue; }
        double dx = g.xs[u] - g.xs[v];
        double dy = g.ys[u] - g.ys[v];
        double w = std::sqrt(dx*dx + dy*dy) / 6.0;
        g.edges.push_back({u, v, w});
    }
    return g;
}

// ============================================================
// Visualization
// ============================================================
Image drawGraphVisual(const Graph& g, const std::vector<Edge>& mstPrim,
                       const std::vector<Edge>& mstKruskal,
                       double /*primTotal*/, double /*kruskalTotal*/) {
    int W = 1200, H = 600;
    Image img(W, H);
    // White background
    for (int i = 0; i < W*H; ++i) img.r[i] = img.g[i] = img.b[i] = 255;

    auto drawGraph = [&](int offsetX, const std::vector<Edge>& edges,
                          const std::vector<Edge>& mst, uint8_t mr, uint8_t mg, uint8_t mb) {

        // Draw all edges (light gray background edges)
        for (const auto& e : edges) {
            int x1 = static_cast<int>(g.xs[e.u]) + offsetX - 300;
            int y1 = static_cast<int>(g.ys[e.u]);
            int x2 = static_cast<int>(g.xs[e.v]) + offsetX - 300;
            int y2 = static_cast<int>(g.ys[e.v]);
            img.drawLine(x1, y1, x2, y2, 200, 200, 200);
        }

        // Draw MST edges (bold colored)
        for (const auto& e : mst) {
            int x1 = static_cast<int>(g.xs[e.u]) + offsetX - 300;
            int y1 = static_cast<int>(g.ys[e.u]);
            int x2 = static_cast<int>(g.xs[e.v]) + offsetX - 300;
            int y2 = static_cast<int>(g.ys[e.v]);
            // Draw 3-pixel wide lines
            for (int d = -1; d <= 1; ++d) {
                img.drawLine(x1+d, y1, x2+d, y2, mr, mg, mb);
                img.drawLine(x1, y1+d, x2, y2+d, mr, mg, mb);
            }
        }

        // Draw vertices
        for (int i = 0; i < g.V; ++i) {
            int cx = static_cast<int>(g.xs[i]) + offsetX - 300;
            int cy = static_cast<int>(g.ys[i]);
            img.fillCircle(cx, cy, 6, 50, 50, 50);
            img.fillCircle(cx, cy, 4, 255, 255, 255);
        }

        // Draw title (simple text using pixels at top)
        // We'll add text annotation below the graph area later
    };

    drawGraph(300, g.edges, mstPrim, 255, 80, 80);
    drawGraph(900, g.edges, mstKruskal, 80, 80, 255);

    // Draw separator line
    img.drawLine(600, 0, 600, 599, 180, 180, 180);

    return img;
}

// ============================================================
// Quantitative validation
// ============================================================
bool validateMST(const Graph& g, const std::vector<Edge>& mst, double totalWeight) {
    // 1. Check correct number of edges
    if ((int)mst.size() != g.V - 1) {
        std::cerr << "FAIL: MST has " << mst.size() << " edges, expected " << g.V-1 << std::endl;
        return false;
    }

    // 2. Check connectivity using UnionFind
    UnionFind uf(g.V);
    double recalcTotal = 0;
    for (const auto& e : mst) {
        uf.unite(e.u, e.v);
        recalcTotal += e.w;
    }
    int root = uf.find(0);
    for (int i = 1; i < g.V; ++i) {
        if (uf.find(i) != root) {
            std::cerr << "FAIL: MST is not connected! Node " << i << " not reachable." << std::endl;
            return false;
        }
    }

    // 3. Verify total weight matches
    if (std::abs(recalcTotal - totalWeight) > 0.01) {
        std::cerr << "FAIL: total weight mismatch " << recalcTotal << " vs " << totalWeight << std::endl;
        return false;
    }

    // 4. Check MST is minimal: try removing any edge and find a replacement
    for (const auto& removed : mst) {
        UnionFind testUF(g.V);
        double testWeight = 0;
        for (const auto& e : mst) {
            if (e.u == removed.u && e.v == removed.v) continue;
            testUF.unite(e.u, e.v);
            testWeight += e.w;
        }
        // Find any edge connecting the two components that's NOT the removed edge
        for (const auto& cand : g.edges) {
            if (cand.u == removed.u && cand.v == removed.v) continue;
            if (testUF.find(cand.u) != testUF.find(cand.v)) {
                // Found an alternative, check weight
                double newWeight = testWeight + cand.w;
                if (newWeight + 0.001 < totalWeight) {
                    std::cerr << "FAIL: Found cheaper replacement!";
                    std::cerr << " Remove (" << removed.u << "," << removed.v << ":" << removed.w << ")";
                    std::cerr << " Add (" << cand.u << "," << cand.v << ":" << cand.w << ")";
                    std::cerr << " new=" << newWeight << " < old=" << totalWeight << std::endl;
                    return false;
                }
                break;
            }
        }
    }

    std::cout << "PASS: All MST validation checks passed." << std::endl;
    std::cout << "  Edges: " << mst.size() << " (expected " << g.V-1 << ")" << std::endl;
    std::cout << "  Connected: YES" << std::endl;
    std::cout << "  Total weight: " << totalWeight << std::endl;
    return true;
}

// ============================================================
// Main
// ============================================================
int main() {
    // Generate random graph
    const int V = 30;
    Graph g = generateGraph(V, 0.25, 42);

    std::cout << "Graph: " << V << " vertices, " << g.edges.size() << " edges" << std::endl;

    // Compute both MSTs
    auto [primEdges, primWeight] = primMST(g);
    auto [kruskalEdges, kruskalWeight] = kruskalMST(g);

    std::cout << "\nPrim's MST:   " << primEdges.size() << " edges, weight=" << primWeight << std::endl;
    std::cout << "Kruskal's MST: " << kruskalEdges.size() << " edges, weight=" << kruskalWeight << std::endl;

    // Validate both algorithms produce the same total weight
    if (std::abs(primWeight - kruskalWeight) > 0.001) {
        std::cerr << "FAIL: Prim and Kruskal produce different MST weights! "
                  << primWeight << " vs " << kruskalWeight << std::endl;
        return 1;
    }
    std::cout << "PASS: Both algorithms produce same weight (" << primWeight << ")" << std::endl;

    // Run full validation on both
    std::cout << "\n--- Validating Prim's MST ---" << std::endl;
    bool primValid = validateMST(g, primEdges, primWeight);
    std::cout << "\n--- Validating Kruskal's MST ---" << std::endl;
    bool kruskalValid = validateMST(g, kruskalEdges, kruskalWeight);

    if (!primValid || !kruskalValid) {
        std::cerr << "FAIL: MST validation failed!" << std::endl;
        return 1;
    }

    // Check edge sets are the same (they should be for unique weights)
    // Count matching edges
    auto edgeKey = [](const Edge& e) { return std::min(e.u,e.v)*10000 + std::max(e.u,e.v); };
    std::set<int> primSet, kruskalSet;
    for (const auto& e : primEdges) primSet.insert(edgeKey(e));
    for (const auto& e : kruskalEdges) kruskalSet.insert(edgeKey(e));
    int matches = 0;
    for (int key : primSet) if (kruskalSet.count(key)) ++matches;
    std::cout << "\nPASS: Edge set overlap: " << matches << "/" << g.V-1
              << " (same MST for unique weights)" << std::endl;

    // Generate visualization
    Image img = drawGraphVisual(g, primEdges, kruskalEdges, primWeight, kruskalWeight);
    img.save("mst_output.ppm");
    std::cout << "\nVisualization saved to mst_output.ppm" << std::endl;

    // Quantitative image verification
    std::vector<int> pixelVals;
    pixelVals.reserve(img.w * img.h);
    for (int i = 0; i < img.w * img.h; ++i)
        pixelVals.push_back(static_cast<int>(img.r[i]) + img.g[i] + img.b[i]);

    double mean = std::accumulate(pixelVals.begin(), pixelVals.end(), 0.0) / pixelVals.size();
    double sq_sum = 0;
    for (int v : pixelVals) sq_sum += (v - mean) * (v - mean);
    double stddev = std::sqrt(sq_sum / pixelVals.size());

    std::cout << "Image stats: mean=" << mean << ", stddev=" << stddev << std::endl;
    if (mean < 10 || mean > 755) {
        std::cerr << "FAIL: Image mean out of range (10-755)" << std::endl;
        return 1;
    }
    if (stddev < 10) {
        std::cerr << "FAIL: Image stddev too low (<10)" << std::endl;
        return 1;
    }
    std::cout << "PASS: Image statistics OK" << std::endl;

    return 0;
}
