// Johnson's Algorithm for All-Pairs Shortest Paths
// Technique: reweight edges using Bellman-Ford potentials h[v],
//             then run Dijkstra from every source with Dijkstra+potentials.
// Supports negative edges (but no negative cycle). Best for sparse graphs.
// Verification:
//   1. Correctness vs Floyd-Warshall (exact distance matrix match) on random graphs
//      including negative edges.
//   2. Correctness vs Bellman-Ford (from every source) as an independent baseline.
//   3. Negative-cycle detection equality (Johnson's BF detects same as FW/BF).
//   4. Performance speedup on sparse graphs versus Floyd-Warshall (O(VE log V) vs O(V^3)).

#include <bits/stdc++.h>
using namespace std;

const long long INF = 4e18;

struct Edge { int u, v; long long w; };

// Bellman-Ford from virtual source with edges already including 0->v (w=0).
// Returns true if NO negative cycle, fills h[0..n]. h has size n+1 (node 0 is virtual).
bool bellman_ford_potentials(int n, const vector<Edge>& edges, vector<long long>& h) {
    h.assign(n + 1, INF);
    h[n] = 0; // virtual node = n (real nodes are 0..n-1)
    for (int i = 0; i <= n; i++) {
        bool changed = false;
        for (auto &e : edges) {
            if (h[e.u] != INF && h[e.u] + e.w < h[e.v]) {
                h[e.v] = h[e.u] + e.w;
                changed = true;
                // relax at most n+1 times; if changed on n+1-th pass => negative cycle
                if (i == n) return false;
            }
        }
        if (!changed) break;
    }
    return true;
}

// Standard Dijkstra with binary heap on original adjacency (reweighted).
vector<long long> dijkstra(int n, int s, const vector<vector<pair<int,long long>>>& adj) {
    vector<long long> dist(n, INF);
    dist[s] = 0;
    priority_queue<pair<long long,int>, vector<pair<long long,int>>, greater<>> pq;
    pq.push({0, s});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        for (auto &[v, w] : adj[u]) {
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

// Johnson: returns dist matrix (n x n) and whether negative cycle exists.
// dist[u][v] = actual shortest distance, or INF if unreachable.
bool johnson(int n, const vector<Edge>& edges, vector<vector<long long>>& dist) {
    // Build edge list with virtual node 0 -> every v, weight 0.
    vector<Edge> aug = edges;
    for (int v = 0; v < n; v++) aug.push_back({n, v, 0}); // virtual node n -> v

    vector<long long> h;
    if (!bellman_ford_potentials(n, aug, h)) return false; // negative cycle

    // Reweight: w'(u,v) = w(u,v) + h[u] - h[v], guaranteed >= 0.
    // Build adjacency of reweighted graph (node indices 0..n-1).
    vector<vector<pair<int,long long>>> adj(n);
    for (auto &e : edges) {
        long long nw = e.w + h[e.u] - h[e.v];
        adj[e.u].push_back({e.v, nw});
    }

    dist.assign(n, vector<long long>(n, INF));
    for (int s = 0; s < n; s++) {
        vector<long long> d = dijkstra(n, s, adj);
        for (int t = 0; t < n; t++) {
            if (d[t] != INF) {
                dist[s][t] = d[t] - h[s] + h[t]; // undo reweighting
            }
        }
    }
    return true;
}

// Floyd-Warshall baseline.
vector<vector<long long>> floyd_warshall(int n, const vector<Edge>& edges) {
    vector<vector<long long>> d(n, vector<long long>(n, INF));
    for (int i = 0; i < n; i++) d[i][i] = 0;
    for (auto &e : edges) d[e.u][e.v] = min(d[e.u][e.v], e.w);
    for (int k = 0; k < n; k++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                if (d[i][k] != INF && d[k][j] != INF)
                    d[i][j] = min(d[i][j], d[i][k] + d[k][j]);
    return d;
}

// Bellman-Ford from single source (baseline, supports negative edges).
vector<long long> bellman_ford(int n, int s, const vector<Edge>& edges) {
    vector<long long> dist(n, INF);
    dist[s] = 0;
    for (int i = 0; i < n - 1; i++) {
        bool changed = false;
        for (auto &e : edges) {
            if (dist[e.u] != INF && dist[e.u] + e.w < dist[e.v]) {
                dist[e.v] = dist[e.u] + e.w;
                changed = true;
            }
        }
        if (!changed) break;
    }
    return dist;
}

uint64_t rd() { static uint64_t x = 123456789; x ^= x << 13; x ^= x >> 7; x ^= x << 17; return x; }

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    vector<pair<int,int>> test_cases; // {n, expected_edge_count}
    // Randomized tests across sizes and densities (including negative weights)
    vector<int> sizes = {8, 16, 32, 64, 128};
    long long passed = 0, total = 0;
    vector<string> failures;

    for (int n : sizes) {
        int trials = (n <= 16) ? 20 : (n <= 64 ? 8 : 4);
        for (int tr = 0; tr < trials; tr++) {
            vector<Edge> edges;
            // Random directed graph: density ~20%
            int target = n * (n - 1) * 20 / 100;
            set<pair<int,int>> seen;
            for (int i = 0; i < target; i++) {
                int u = rd() % n, v = rd() % n;
                if (u == v || seen.count({u,v})) continue;
                seen.insert({u,v});
                // 40% negative edges, weight in [-5, 20]
                long long w = (rd() % 26) - 5;
                edges.push_back({u, v, w});
            }
            // ensure at least some edges
            if (edges.empty()) { edges.push_back({0, 1, 3}); }

            // Johnson
            vector<vector<long long>> dj;
            bool j_ok = johnson(n, edges, dj);

            // Floyd-Warshall baseline
            vector<vector<long long>> dfw = floyd_warshall(n, edges);

            // Check negative-cycle detection: FW detects neg cycle iff some d[i][i] < 0
            bool fw_negcycle = false;
            for (int i = 0; i < n; i++) if (dfw[i][i] < 0) fw_negcycle = true;

            if (fw_negcycle) {
                // If negative cycle, Johnson should report false.
                if (j_ok) failures.push_back("Johnson missed negative cycle at n=" + to_string(n));
                total++;
                passed += (!j_ok);
                continue;
            }

            if (!j_ok) { failures.push_back("Johnson false negative cycle at n=" + to_string(n)); total++; continue; }

            // Compare distance matrices
            bool match = true;
            for (int i = 0; i < n && match; i++)
                for (int j = 0; j < n && match; j++)
                    if (dj[i][j] != dfw[i][j]) match = false;
            if (!match) failures.push_back("Distance mismatch vs FW at n=" + to_string(n));
            total++;
            passed += match;
        }
    }

    cout << "========================================\n";
    cout << "Johnson vs Floyd-Warshall correctness\n";
    cout << "========================================\n";
    cout << "Random graph cases passed: " << passed << " / " << total << "\n";
    if (passed == total) cout << "RESULT: PASS (all distance matrices + neg-cycle detection match Floyd-Warshall)\n";
    else { cout << "RESULT: FAIL\n"; for (auto &s : failures) cout << "  - " << s << "\n"; }

    // Independent baseline: Bellman-Ford from every source
    int n2 = 40;
    vector<Edge> edges2;
    set<pair<int,int>> seen2;
    int target2 = n2 * (n2 - 1) * 25 / 100;
    for (int i = 0; i < target2; i++) {
        int u = rd() % n2, v = rd() % n2;
        if (u == v || seen2.count({u,v})) continue;
        seen2.insert({u,v});
        long long w = (rd() % 20) - 6;
        edges2.push_back({u,v,w});
    }
    vector<vector<long long>> dj2; bool j2 = johnson(n2, edges2, dj2);
    vector<vector<long long>> dfw2 = floyd_warshall(n2, edges2);
    bool fw_neg2 = false; for (int i=0;i<n2;i++) if (dfw2[i][i] < 0) fw_neg2 = true;

    cout << "\n========================================\n";
    cout << "Johnson vs Bellman-Ford (independent)\n";
    cout << "========================================\n";
    if (fw_neg2) {
        cout << "Graph has negative cycle; Johnson result: " << (j2 ? "false-positive (BAD)" : "correctly rejected") << "\n";
    } else if (!j2) {
        cout << "Johnson wrongly rejected (no negative cycle). FAIL\n";
    } else {
        bool ok = true;
        for (int s = 0; s < n2; s++) {
            auto bf = bellman_ford(n2, s, edges2);
            for (int t = 0; t < n2; t++) if (bf[t] != dj2[s][t]) { ok = false; s = n2; break; }
        }
        cout << "Full distance match vs Bellman-Ford: " << (ok ? "PASS" : "FAIL") << "\n";
    }

    // Performance: sparse graph speedup vs Floyd-Warshall
    cout << "\n========================================\n";
    cout << "Performance (sparse graph, density 2%)\n";
    cout << "========================================\n";
    for (int n : {200, 400, 800, 1600}) {
        vector<Edge> e;
        set<pair<int,int>> s;
        int target = n * (n - 1) * 2 / 100;
        for (int i = 0; i < target; i++) {
            int u = rd() % n, v = rd() % n;
            if (u == v || s.count({u,v})) continue;
            s.insert({u,v});
            long long w = (rd() % 10) + 1; // positive weights for clean timing
            e.push_back({u,v,w});
        }
        auto t0 = chrono::high_resolution_clock::now();
        vector<vector<long long>> dj; johnson(n, e, dj);
        auto t1 = chrono::high_resolution_clock::now();
        vector<vector<long long>> df = floyd_warshall(n, e);
        auto t2 = chrono::high_resolution_clock::now();
        double tj = chrono::duration<double>(t1 - t0).count();
        double tf = chrono::duration<double>(t2 - t1).count();
        printf("n=%5d  edges=%6d  Johnson=%7.4fs  FloydWarshall=%7.4fs  speedup=%6.1fx\n",
               n, (int)e.size(), tj, tf, tf / max(tj, 1e-9));
    }

    cout << "\nALL CHECKS COMPLETE\n";
    return 0;
}
