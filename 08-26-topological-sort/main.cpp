// Topological Sort: Kahn's Algorithm + DFS Post-Order, with cycle detection.
// Quantitatively validates:
//   1) Ordering is a valid topological order (all edges u->v satisfy pos[u] < pos[v])
//   2) Kahn == DFS result length and validity
//   3) Cycle detection correctness on graphs with/without cycles
//   4) Randomized DAG stress test (many trials)

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <bits/stdc++.h>
#include "stb_image_write.h"
using namespace std;

// ---------- Kahn's Algorithm (BFS, in-degree based) ----------
// Returns topological order; empty vector if graph has a cycle.
vector<int> kahn(int n, const vector<pair<int,int>>& edges) {
    vector<vector<int>> adj(n);
    vector<int> indeg(n, 0);
    for (auto& e : edges) {
        adj[e.first].push_back(e.second);
        indeg[e.second]++;
    }
    queue<int> q;
    for (int i = 0; i < n; i++) if (indeg[i] == 0) q.push(i);
    vector<int> order;
    order.reserve(n);
    while (!q.empty()) {
        int u = q.front(); q.pop();
        order.push_back(u);
        for (int v : adj[u]) {
            if (--indeg[v] == 0) q.push(v);
        }
    }
    if ((int)order.size() != n) return {}; // cycle detected
    return order;
}

// ---------- DFS Post-Order ----------
// Returns reverse-postorder topological order; empty if cycle.
vector<int> dfs_topo(int n, const vector<pair<int,int>>& edges) {
    vector<vector<int>> adj(n);
    for (auto& e : edges) adj[e.first].push_back(e.second);
    vector<int> color(n, 0); // 0=white 1=gray 2=black
    vector<int> order;
    bool hasCycle = false;
    function<void(int)> dfs = [&](int u) {
        color[u] = 1;
        for (int v : adj[u]) {
            if (color[v] == 1) hasCycle = true;        // back edge -> cycle
            else if (color[v] == 0) dfs(v);
        }
        color[u] = 2;
        order.push_back(u);
    };
    for (int i = 0; i < n; i++) if (color[i] == 0) dfs(i);
    if (hasCycle) return {};
    reverse(order.begin(), order.end());
    return order;
}

// ---------- Validation helpers ----------
// Check that order is a valid topological ordering of edges.
bool isValidTopo(const vector<int>& order, const vector<pair<int,int>>& edges) {
    if (order.empty()) return false;
    unordered_map<int,int> pos;
    for (int i = 0; i < (int)order.size(); i++) pos[order[i]] = i;
    if ((int)pos.size() != (int)order.size()) return false; // duplicates
    for (auto& e : edges) {
        if (pos[e.first] >= pos[e.second]) return false;
    }
    return true;
}

bool hasCycle(int n, const vector<pair<int,int>>& edges) {
    return kahn(n, edges).empty();
}

// ---------------- PNG visualization ----------------
// Draw a DAG laid out left-to-right in layers by topological rank.
// Vertices colored by a gradient from early (red) to late (blue) in the order.
void writePNG(const char* path, int n, const vector<pair<int,int>>& edges,
              const vector<int>& order) {
    int S = 800, W = S, H = S;
    vector<unsigned char> img(W * H * 3, 255);

    // Rank of each vertex in the topological order (0 = earliest).
    vector<int> rank(n, 0);
    for (int i = 0; i < (int)order.size(); i++) rank[order[i]] = i;

    // Layer (x) = rank; y = spread within same layer to reduce overlap.
    // Simple assignment: x proportional to rank/n, y = pseudo-random-ish offset
    // based on vertex id so same-layer vertices don't overlap.
    mt19937 layout(7);
    double marginX = 70, marginY = 70;
    double usableW = W - 2 * marginX;
    double usableH = H - 2 * marginY;
    int maxRank = max(1, n - 1);

    vector<pair<double,double>> pos(n);
    for (int i = 0; i < n; i++) {
        double tx = (double)rank[i] / maxRank;
        double x = marginX + tx * usableW;
        double y = marginY + ((double)(layout() % 1000) / 1000.0) * usableH;
        pos[i] = {x, y};
    }

    auto setPixel = [&](int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        int idx = (y * W + x) * 3;
        img[idx] = r; img[idx+1] = g; img[idx+2] = b;
    };

    auto drawLine = [&](double x0, double y0, double x1, double y1,
                        unsigned char r, unsigned char g, unsigned char b) {
        int steps = max(1, (int)hypot(x1-x0, y1-y0));
        for (int s = 0; s <= steps; s++) {
            double t = (double)s / steps;
            setPixel((int)(x0 + t*(x1-x0)), (int)(y0 + t*(y1-y0)), r, g, b);
        }
    };

    // Draw directed edges (light gray) with arrowheads at the target end.
    for (auto& e : edges) {
        int u = e.first, v = e.second;
        double x0 = pos[u].first, y0 = pos[u].second;
        double x1 = pos[v].first, y1 = pos[v].second;
        // Shorten the line so it stops at the vertex boundary (radius r=16).
        double len = hypot(x1-x0, y1-y0);
        if (len < 1e-9) continue;
        double ux = (x1-x0)/len, uy = (y1-y0)/len;
        double stopx = x1 - ux * 18, stopy = y1 - uy * 18;
        drawLine(x0, y0, stopx, stopy, 150, 150, 150);
        // Arrowhead triangle at (stopx, stopy) pointing along (ux, uy).
        double ah = 9, aw = 6;
        double nx = -uy, ny = ux;
        double bx = stopx - ux*ah, by = stopy - uy*ah;
        double lx = bx + nx*aw, ly = by + ny*aw;
        double rx = bx - nx*aw, ry = by - ny*aw;
        drawLine(stopx, stopy, lx, ly, 120, 120, 120);
        drawLine(stopx, stopy, rx, ry, 120, 120, 120);
        drawLine(lx, ly, rx, ry, 120, 120, 120);
    }

    // Draw vertices as filled disks, gradient color by rank.
    for (int i = 0; i < n; i++) {
        int x = (int)pos[i].first, y = (int)pos[i].second;
        int r = 16;
        double t = maxRank == 0 ? 0.0 : (double)rank[i] / maxRank;
        // Red (early) -> blue (late) gradient.
        unsigned char cr = (unsigned char)(230 - 150 * t);
        unsigned char cg = (unsigned char)(57 + 60 * t);
        unsigned char cb = (unsigned char)(70 + 180 * t);
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++)
                if (dx*dx + dy*dy <= r*r)
                    setPixel(x+dx, y+dy, cr, cg, cb);
    }

    stbi_write_png(path, W, H, 3, img.data(), W * 3);
}

// Generate a random DAG: edges only point from lower index to higher index.
vector<pair<int,int>> randomDAG(mt19937& rng, int n, int maxEdges) {
    vector<pair<int,int>> edges;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if ((int)edges.size() >= maxEdges) break;
            if (rng() % 100 < 25) edges.push_back({i, j});
        }
        if ((int)edges.size() >= maxEdges) break;
    }
    return edges;
}

int main() {
    // ---------- Fixed test cases ----------
    struct Case { string name; int n; vector<pair<int,int>> edges; bool expectCycle; };
    vector<Case> cases = {
        {"chain",        5, {{0,1},{1,2},{2,3},{3,4}}, false},
        {"diamond",      6, {{0,1},{0,2},{1,3},{2,3},{3,4},{3,5}}, false},
        {"course-sched", 6, {{1,0},{2,0},{3,1},{3,2}}, false}, // classic 207
        {"disconnected", 6, {{0,1},{2,3},{4,5}}, false},
        {"self-loop",    3, {{0,1},{1,2},{1,1}}, true},
        {"triangle",     3, {{0,1},{1,2},{2,0}}, true},
        {"two-cycle",    4, {{0,1},{1,0},{2,3}}, true},
    };

    int pass = 0, totalChecks = 0;

    for (auto& c : cases) {
        auto k = kahn(c.n, c.edges);
        auto d = dfs_topo(c.n, c.edges);
        bool kCyc = k.empty();
        bool dCyc = d.empty();

        totalChecks++;
        if (kCyc == c.expectCycle) pass++;
        totalChecks++;
        if (dCyc == c.expectCycle) pass++;

        if (!c.expectCycle) {
            totalChecks++;
            if (isValidTopo(k, c.edges)) pass++;
            totalChecks++;
            if (isValidTopo(d, c.edges)) pass++;
            totalChecks++;
            if (k.size() == (size_t)c.n && d.size() == (size_t)c.n) pass++;
        }
        printf("[%s] kahn_cyc=%d dfs_cyc=%d expected_cyc=%d  kahn_len=%zu dfs_len=%zu\n",
               c.name.c_str(), kCyc, dCyc, c.expectCycle, k.size(), d.size());
    }

    // ---------- Random DAG stress test ----------
    mt19937 rng(12345);
    int trials = 2000;
    int dPass = 0;
    for (int t = 0; t < trials; t++) {
        int n = 2 + (int)(rng() % 30);
        auto edges = randomDAG(rng, n, 200);
        auto k = kahn(n, edges);
        auto d = dfs_topo(n, edges);
        if (!k.empty() && !d.empty() &&
            isValidTopo(k, edges) && isValidTopo(d, edges) &&
            k.size() == (size_t)n && d.size() == (size_t)n) {
            dPass++;
        }
    }
    printf("\nRandomDAG stress: %d/%d trials produced valid topo orders\n", dPass, trials);
    bool stressOK = (dPass == trials);
    // Random graph with a guaranteed cycle -> must be detected.
    // Build a DAG, pick a pair i<j that has a path i->j (guaranteed reachability),
    // then add the back edge j->i to create a real cycle.
    int cycDetected = 0, cycTrials = 500;
    for (int t = 0; t < cycTrials; t++) {
        int n = 3 + (int)(rng() % 30);
        auto edges = randomDAG(rng, n, 200);
        // Build reachability adjacency from the DAG edges (i<j by construction).
        vector<vector<bool>> reach(n, vector<bool>(n, false));
        for (auto& e : edges) reach[e.first][e.second] = true;
        for (int k = 0; k < n; k++)
            for (int i = 0; i < n; i++)
                if (reach[i][k])
                    for (int j = 0; j < n; j++)
                        if (reach[k][j]) reach[i][j] = true;
        // Find i<j with reach[i][j], add back edge j->i => guaranteed cycle.
        int bi = -1, bj = -1;
        for (int i = 0; i < n && bi < 0; i++)
            for (int j = i + 1; j < n; j++)
                if (reach[i][j]) { bi = i; bj = j; break; }
        if (bi < 0) { // no path — force a guaranteed 2-node cycle
            edges.push_back({n-1, n-2});
            edges.push_back({n-2, n-1});
        } else {
            edges.push_back({bj, bi});
        }
        if (hasCycle(n, edges)) cycDetected++;
    }
    printf("Cycle detection: %d/%d planted-cycle graphs correctly flagged\n", cycDetected, cycTrials);
    printf("\nTOTAL CHECKS: %d, PASSED: %d\n", totalChecks, pass);
    printf("stressOK=%d cycOK=%d\n", stressOK ? 1 : 0, cycDetected == cycTrials ? 1 : 0);

    // ---------- Visualization: draw a fixed example DAG ----------
    // A course-schedule style DAG with 10 nodes for a recognizable diagram.
    int vn = 10;
    vector<pair<int,int>> vedges = {
        {0,2},{0,3},{1,3},{1,4},{2,5},{3,5},{3,6},{4,6},
        {5,7},{6,7},{6,8},{7,9},{8,9},{2,8},{4,8}
    };
    vector<int> vorder = kahn(vn, vedges);
    if (!vorder.empty()) {
        writePNG("topological_sort_output.png", vn, vedges, vorder);
        printf("\nVisualization written to topological_sort_output.png (10-node DAG)\n");
    }

    bool allOK = (pass == totalChecks) && stressOK && (cycDetected == cycTrials);
    printf("\nRESULT: %s\n", allOK ? "ALL_PASS" : "FAIL");
    return allOK ? 0 : 1;
}
