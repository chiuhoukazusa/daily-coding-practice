// Bellman-Ford Single-Source Shortest Path
// - Handles negative edge weights (unlike Dijkstra)
// - Detects negative cycles reachable from source
// - Dynamic-programming relaxation over at most V-1 passes
// Verification strategy (quantitative, not visual):
//   1. Correctness vs Floyd-Warshall (guaranteed correct O(V^3) APSP) on
//      random directed graphs WITH negative edges -> compare dist matrix.
//   2. Negative-cycle detection on graphs with injected negative cycles.
//   3. Runtime scaling: O(V*E) empirically measured.

#include <bits/stdc++.h>
using namespace std;

const long long INF = 4e18;

struct Edge { int u, v; long long w; };

// Returns {dist, hasNegCycleReachable}. dist[i] = INF if unreachable.
// If a negative cycle is reachable from src, hasNegCycleReachable = true
// and dist for affected nodes is not meaningful.
pair<vector<long long>, bool> bellman_ford(int n, const vector<Edge>& edges, int src) {
    vector<long long> dist(n, INF);
    dist[src] = 0;

    // V-1 relaxation passes
    for (int pass = 0; pass < n - 1; ++pass) {
        bool changed = false;
        for (auto& e : edges) {
            if (dist[e.u] != INF && dist[e.u] + e.w < dist[e.v]) {
                dist[e.v] = dist[e.u] + e.w;
                changed = true;
            }
        }
        if (!changed) break; // early termination optimization
    }

    // Nth pass: detect negative cycles
    bool negCycle = false;
    for (auto& e : edges) {
        if (dist[e.u] != INF && dist[e.u] + e.w < dist[e.v]) {
            negCycle = true;
            break;
        }
    }
    return {dist, negCycle};
}

// Floyd-Warshall for guaranteed-correct reference on negative-edge graphs
// Returns dist matrix; diagonal negative => negative cycle somewhere.
vector<vector<long long>> floyd_warshall(int n, const vector<Edge>& edges) {
    vector<vector<long long>> d(n, vector<long long>(n, INF));
    for (int i = 0; i < n; ++i) d[i][i] = 0;
    for (auto& e : edges) {
        if (e.w < d[e.u][e.v]) d[e.u][e.v] = e.w;
    }
    for (int k = 0; k < n; ++k)
        for (int i = 0; i < n; ++i)
            if (d[i][k] != INF)
                for (int j = 0; j < n; ++j)
                    if (d[k][j] != INF)
                        d[i][j] = min(d[i][j], d[i][k] + d[k][j]);
    return d;
}

// Generate a random directed graph with negative edges (no negative cycle by
// construction unless requested). We build a random DAG-shape with weights
// potentially negative, ensuring no negative cycle via topological ordering.
mt19937 rng;

struct TestResult {
    int n, m, src;
    double bf_ms, fw_ms;
    bool bf_matches_fw;
    long long max_err; // max distance error (should be 0)
};

// Build graph with no negative cycle: assign each node a random "potential"
// and set edge weight = potential[v] - potential[u] + random_nonnegative,
// so every path weight >= potential[t]-potential[s] (no negative cycle).
TestResult run_correctness_test(int n, int m) {
    vector<Edge> edges;
    // potential values
    vector<long long> pot(n);
    for (int i = 0; i < n; ++i) pot[i] = (long long)(rng() % 2001) - 1000;

    set<pair<int,int>> used;
    while ((int)edges.size() < m) {
        int u = rng() % n;
        int v = rng() % n;
        if (u == v) continue;
        if (used.count({u, v})) continue;
        used.insert({u, v});
        long long base = (long long)(rng() % 500); // non-negative part
        long long w = pot[v] - pot[u] + base;       // no negative cycle
        edges.push_back({u, v, w});
    }
    m = edges.size();
    int src = rng() % n;

    auto t0 = chrono::high_resolution_clock::now();
    auto bf = bellman_ford(n, edges, src);
    auto t1 = chrono::high_resolution_clock::now();

    auto t2 = chrono::high_resolution_clock::now();
    auto fw = floyd_warshall(n, edges);
    auto t3 = chrono::high_resolution_clock::now();

    bool match = true;
    long long max_err = 0;
    for (int i = 0; i < n; ++i) {
        long long a = bf.first[i];
        long long b = fw[src][i];
        long long err = (a == INF || b == INF) ? (a == b ? 0 : 1) : llabs(a - b);
        if (a != b) match = false;
        max_err = max(max_err, err);
    }

    double bf_ms = chrono::duration<double, milli>(t1 - t0).count();
    double fw_ms = chrono::duration<double, milli>(t3 - t2).count();
    return {n, m, src, bf_ms, fw_ms, match, max_err};
}

// Negative cycle detection test: inject a directed cycle with negative total weight.
bool run_negative_cycle_test() {
    // Simple graph: 0->1 (w=1), 1->2 (w=1), 2->0 (w=-3) => total -1 cycle.
    int n = 3;
    vector<Edge> edges = {{0,1,1},{1,2,1},{2,0,-3}};
    auto res = bellman_ford(n, edges, 0);
    return res.second == true; // should detect
}
bool run_no_cycle_sanity() {
    int n = 4;
    vector<Edge> edges = {{0,1,1},{1,2,2},{0,2,10},{2,3,1}};
    auto res = bellman_ford(n, edges, 0);
    if (res.second) return false;
    vector<long long> expect = {0,1,3,4};
    for (int i = 0; i < n; ++i) if (res.first[i] != expect[i]) return false;
    return true;
}

int main() {
    rng.seed(20260824);
    bool all_ok = true;

    printf("=== Bellman-Ford Quantitative Verification ===\n\n");

    // Sanity checks
    bool nc = run_negative_cycle_test();
    bool no = run_no_cycle_sanity();
    printf("[Sanity] Negative cycle detected correctly: %s\n", nc ? "PASS" : "FAIL");
    printf("[Sanity] No-cycle basic dist correct:      %s\n", no ? "PASS" : "FAIL");
    all_ok &= nc && no;

    // Random correctness tests vs Floyd-Warshall (negative edges, no neg cycle)
    printf("\n--- Correctness vs Floyd-Warshall (negative edges, no neg cycle) ---\n");
    vector<pair<int,int>> configs = {{20,60},{50,300},{100,800},{150,1800}};
    int passes = 0, total = 0;
    long long total_max_err = 0;
    for (auto& cfg : configs) {
        int ok = 0;
        for (int rep = 0; rep < 5; ++rep) {
            auto r = run_correctness_test(cfg.first, cfg.second);
            total++;
            if (r.bf_matches_fw) { ok++; passes++; }
            total_max_err = max(total_max_err, r.max_err);
        }
        printf("  n=%4d m=%4d : %d/5 match Floyd-Warshall\n", cfg.first, cfg.second, ok);
        all_ok &= (ok == 5);
    }
    printf("  Total: %d/%d tests match. Max dist error = %lld\n",
           passes, total, total_max_err);

    // Runtime scaling O(V*E)
    printf("\n--- Runtime scaling (expect roughly O(V*E)) ---\n");
    for (auto& cfg : configs) {
        int n = cfg.first, m = cfg.second;
        vector<Edge> edges;
        vector<long long> pot(n);
        for (int i = 0; i < n; ++i) pot[i] = (long long)(rng()%2001)-1000;
        set<pair<int,int>> used;
        while ((int)edges.size() < m) {
            int u = rng()%n, v = rng()%n;
            if (u==v || used.count({u,v})) continue;
            used.insert({u,v});
            edges.push_back({u,v, pot[v]-pot[u] + (long long)(rng()%500)});
        }
        int src = 0;
        auto t0 = chrono::high_resolution_clock::now();
        bellman_ford(n, edges, src);
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1-t0).count();
        printf("  n=%4d m=%4d V*E=%9d : %.3f ms\n", n, m, n*(int)edges.size(), ms);
    }

    printf("\n=== RESULT: %s ===\n", all_ok ? "ALL PASS" : "SOME FAILED");
    return all_ok ? 0 : 1;
}
