// Dijkstra Shortest Path with Binary Heap (Priority Queue)
// Daily Coding Practice 2026-10-10
// Direction: Graph Algorithms
// Technique: Dijkstra's algorithm using a binary min-heap with lazy deletion,
//              O((V+E) log V) time, O(V+E) space.
// Verification: compare against Floyd-Warshall (all-pairs correct reference),
//               randomized graphs, and brute-force Dijkstra O(V^2) baseline.

#include <bits/stdc++.h>
using namespace std;
using ll = long long;
const ll INF = 4e18;

struct Edge { int to; ll w; };

// ---------- Dijkstra with binary heap (lazy deletion) ----------
vector<ll> dijkstra_heap(int V, const vector<vector<Edge>>& adj, int src) {
    vector<ll> dist(V, INF);
    using P = pair<ll,int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    dist[src] = 0;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue; // lazy deletion: stale entry
        for (auto &e : adj[u]) {
            ll nd = d + e.w;
            if (nd < dist[e.to]) {
                dist[e.to] = nd;
                pq.push({nd, e.to});
            }
        }
    }
    return dist;
}

// ---------- Baseline O(V^2) Dijkstra (dense adjacency matrix) ----------
vector<ll> dijkstra_naive(int V, const vector<vector<ll>>& mat, int src) {
    vector<ll> dist(V, INF);
    vector<char> done(V, false);
    dist[src] = 0;
    for (int it = 0; it < V; ++it) {
        int u = -1;
        for (int i = 0; i < V; ++i)
            if (!done[i] && (u == -1 || dist[i] < dist[u])) u = i;
        if (u == -1 || dist[u] == INF) break;
        done[u] = true;
        for (int v = 0; v < V; ++v)
            if (!done[v] && mat[u][v] < INF)
                dist[v] = min(dist[v], dist[u] + mat[u][v]);
    }
    return dist;
}

// ---------- Floyd-Warshall reference (all-pairs correct) ----------
vector<vector<ll>> floyd_warshall(int V, const vector<vector<ll>>& mat) {
    auto d = mat;
    for (int k = 0; k < V; ++k)
        for (int i = 0; i < V; ++i)
            for (int j = 0; j < V; ++j)
                if (d[i][k] < INF && d[k][j] < INF)
                    d[i][j] = min(d[i][j], d[i][k] + d[k][j]);
    return d;
}

int main() {
    mt19937 rng(20261010);
    int total_correct = 0, total_tests = 0;
    int failures = 0;
    ll max_abs_err = 0;

    // Run a battery of randomized tests across varying graph sizes/densities
    for (int t = 0; t < 60; ++t) {
        int V = 2 + (int)(rng() % 80);            // 2..81 vertices
        int maxE = V * (V - 1);
        double density = 0.05 + (rng() % 100) / 100.0 * 0.9; // 5%..95%
        int E = max(1, (int)(maxE * density));

        vector<vector<Edge>> adj(V);
        vector<vector<ll>> mat(V, vector<ll>(V, INF));
        for (int i = 0; i < V; ++i) mat[i][i] = 0;

        set<pair<int,int>> seen;
        for (int e = 0; e < E; ++e) {
            int u = rng() % V, v = rng() % V;
            if (u == v || seen.count({u,v})) { --e; continue; }
            seen.insert({u,v});
            ll w = 1 + (ll)(rng() % 1000); // positive weights 1..1000
            adj[u].push_back({v, w});
            mat[u][v] = min(mat[u][v], w);
        }

        // Ensure at least one edge out (graphs may be disconnected; that's fine)
        int src = rng() % V;

        vector<ll> heap_res = dijkstra_heap(V, adj, src);
        vector<ll> naive_res = dijkstra_naive(V, mat, src);
        auto floyd = floyd_warshall(V, mat);

        // Verify heap vs naive and heap vs floyd (single-source row of Floyd)
        bool ok = true;
        for (int v = 0; v < V; ++v) {
            ll ref = floyd[src][v];
            if (ref == INF) ref = INF;
            ll a = heap_res[v], b = naive_res[v];
            if (a != b) { ok = false; failures++; }
            else if (a != ref) { ok = false; failures++; }
            if (a < INF && ref < INF) max_abs_err = max(max_abs_err, llabs(a - ref));
        }
        total_tests += V;
        if (ok) total_correct += V;
    }

    printf("=== Dijkstra (Binary Heap) Verification ===\n");
    printf("Random tests: 60 graphs (V=2..81, density 5%%..95%%)\n");
    printf("Total single-source distance checks: %d\n", total_tests);
    printf("Correct (match Floyd-Warshall & O(V^2) baseline): %d\n", total_correct);
    printf("Failures: %d\n", failures);
    printf("Max abs error vs reference: %lld\n", max_abs_err);

    // ---- Benchmarks ----
    // Sparse graph (V=20000, E~5V) vs dense (V=2000, E~V^2/4)
    {
        int V = 20000, E = 5 * V;
        vector<vector<Edge>> adj(V);
        vector<vector<ll>> mat(V, vector<ll>(V, INF));
        for (int i = 0; i < V; ++i) mat[i][i] = 0;
        for (int e = 0; e < E; ++e) {
            int u = rng() % V, v = rng() % V;
            if (u == v) { --e; continue; }
            ll w = 1 + rng() % 100;
            adj[u].push_back({v, w});
            mat[u][v] = min(mat[u][v], w);
        }
        auto t0 = chrono::high_resolution_clock::now();
        auto h1 = dijkstra_heap(V, adj, 0);
        auto t1 = chrono::high_resolution_clock::now();
        auto n1 = dijkstra_naive(V, mat, 0);
        auto t2 = chrono::high_resolution_clock::now();
        double heap_ms = chrono::duration<double,milli>(t1-t0).count();
        double naive_ms = chrono::duration<double,milli>(t2-t1).count();
        printf("\n=== Benchmark (sparse V=%d E=%d) ===\n", V, E);
        printf("Heap Dijkstra: %.2f ms\n", heap_ms);
        printf("O(V^2) baseline: %.2f ms\n", naive_ms);
        printf("Speedup: %.2fx\n", naive_ms / heap_ms);

        // sanity: results identical
        int mism = 0;
        for (int v = 0; v < V; ++v) if (h1[v] != n1[v]) mism++;
        printf("Sparse result mismatch count: %d\n", mism);
    }
    {
        int V = 3000;
        vector<vector<Edge>> adj(V);
        vector<vector<ll>> mat(V, vector<ll>(V, INF));
        for (int i = 0; i < V; ++i) mat[i][i] = 0;
        for (int u = 0; u < V; ++u)
            for (int v = u+1; v < V; ++v)
                if (rng() % 2 == 0) {
                    ll w = 1 + rng() % 100;
                    adj[u].push_back({v, w});
                    adj[v].push_back({u, w});
                    mat[u][v] = mat[v][u] = w;
                }
        auto t0 = chrono::high_resolution_clock::now();
        auto h2 = dijkstra_heap(V, adj, 0);
        auto t1 = chrono::high_resolution_clock::now();
        auto n2 = dijkstra_naive(V, mat, 0);
        auto t2 = chrono::high_resolution_clock::now();
        double heap_ms = chrono::duration<double,milli>(t1-t0).count();
        double naive_ms = chrono::duration<double,milli>(t2-t1).count();
        printf("\n=== Benchmark (dense V=%d) ===\n", V);
        printf("Heap Dijkstra: %.2f ms\n", heap_ms);
        printf("O(V^2) baseline: %.2f ms\n", naive_ms);
        printf("Speedup: %.2fx\n", naive_ms / heap_ms);
        int mism = 0;
        for (int v = 0; v < V; ++v) if (h2[v] != n2[v]) mism++;
        printf("Dense result mismatch count: %d\n", mism);
    }

    if (failures > 0) {
        printf("\nRESULT: FAIL (%d mismatches)\n", failures);
        return 1;
    }
    printf("\nRESULT: PASS\n");
    return 0;
}
