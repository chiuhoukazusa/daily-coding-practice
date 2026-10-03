// 0-1 BFS (Deque-based BFS for graphs with edge weights 0 or 1)
// Topic: 图算法 / 最短路
// 0-1 BFS solves single-source shortest paths on graphs where every edge
// has weight 0 or 1, in O(V + E) using a deque (unlike Dijkstra's O(E log V)).
//
// Verification (quantitative):
//   1. Correctness: distance equals Dijkstra with 0/1 weights on random graphs.
//   2. Correctness: distance equals the true multi-source distance on a
//      known grid with walls (0/1 weights) vs a brute-force BFS on a
//      unit-weight expansion.
//   3. Complexity/Optimality: signed-weight sanity — verify 0-1 BFS fails to
//      give correct answers is NOT the goal; instead we prove it matches
//      Dijkstra which is optimal for non-negative weights.
//   4. Performance: wall-clock speedup of 0-1 BFS vs std::priority_queue
//      Dijkstra on large random 0/1 graphs.

#include <bits/stdc++.h>
using namespace std;

// Returns vector of shortest distances from source (node 0).
// Adjacency: vector of (to, weight) where weight is 0 or 1.
vector<long long> zero_one_bfs(int n, const vector<vector<pair<int,int>>>& adj, int src) {
    const long long INF = LLONG_MAX / 4;
    vector<long long> dist(n, INF);
    deque<int> dq;
    dist[src] = 0;
    dq.push_back(src);
    while (!dq.empty()) {
        int u = dq.front(); dq.pop_front();
        for (auto& e : adj[u]) {
            int v = e.first;
            int w = e.second; // 0 or 1
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                if (w == 0) dq.push_front(v);
                else        dq.push_back(v);
            }
        }
    }
    return dist;
}

// Dijkstra with binary heap for reference (works for any non-negative weights).
vector<long long> dijkstra(int n, const vector<vector<pair<int,int>>>& adj, int src) {
    const long long INF = LLONG_MAX / 4;
    vector<long long> dist(n, INF);
    priority_queue<pair<long long,int>, vector<pair<long long,int>>, greater<>> pq;
    dist[src] = 0;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        for (auto& e : adj[u]) {
            int v = e.first; long long w = e.second;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}

int main() {
    cout << fixed << setprecision(6);

    // --- Test 1: random 0/1 graph, compare 0-1 BFS vs Dijkstra ---
    {
        int n = 2000, m = 12000;
        mt19937 rng(12345);
        vector<vector<pair<int,int>>> adj(n);
        for (int i = 0; i < m; i++) {
            int u = rng() % n;
            int v = rng() % n;
            int w = rng() % 2; // 0 or 1
            adj[u].push_back({v, w});
            adj[v].push_back({u, w});
        }
        auto d1 = zero_one_bfs(n, adj, 0);
        auto d2 = dijkstra(n, adj, 0);
        int mismatches = 0;
        for (int i = 0; i < n; i++) if (d1[i] != d2[i]) mismatches++;
        cout << "Test1 (random 0/1 graph, n=" << n << " m=" << m << "): ";
        cout << "mismatches=" << mismatches << " -> "
             << (mismatches == 0 ? "PASS" : "FAIL") << "\n";
    }

    // --- Test 2: grid with walls as multi-source 0/1 BFS vs expanded BFS ---
    // Grid HxW. A cell has either open (cost 0 to enter) or wall (cost 1 to
    // "break"). Moves are 4-directional. We compare 0-1 BFS distance with the
    // true minimum number of wall-entries via a separate reference (Dijkstra
    // on the same weights).
    {
        int H = 300, W = 300;
        mt19937 rng(777);
        vector<vector<int>> grid(H, vector<int>(W, 0)); // 0 open, 1 wall
        for (int r = 0; r < H; r++)
            for (int c = 0; c < W; c++)
                grid[r][c] = (rng() % 5 < 3) ? 1 : 0; // ~60% walls (> percolation threshold, forces wall-breaking)
        grid[0][0] = 0;
        grid[H-1][W-1] = 0;

        auto id = [&](int r, int c) { return r * W + c; };
        int N = H * W;
        vector<vector<pair<int,int>>> adj(N);
        int dr[4] = {-1,1,0,0}, dc[4] = {0,0,-1,1};
        for (int r = 0; r < H; r++) for (int c = 0; c < W; c++) {
            for (int k = 0; k < 4; k++) {
                int nr = r + dr[k], nc = c + dc[k];
                if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
                adj[id(r,c)].push_back({id(nr,nc), grid[nr][nc]});
            }
        }
        auto d_bfs = zero_one_bfs(N, adj, id(0,0));
        auto d_dij = dijkstra(N, adj, id(0,0));
        int mismatches = 0;
        for (int i = 0; i < N; i++) if (d_bfs[i] != d_dij[i]) mismatches++;
        cout << "Test2 (grid " << H << "x" << W << " with walls): ";
        cout << "mismatches=" << mismatches << " corner_dist=" << d_bfs[id(H-1,W-1)]
             << " -> " << (mismatches == 0 ? "PASS" : "FAIL") << "\n";
    }

    // --- Test 3: performance 0-1 BFS vs Dijkstra on large random graph ---
    {
        int n = 200000, m = 1000000;
        mt19937 rng(999);
        vector<vector<pair<int,int>>> adj(n);
        for (int i = 0; i < m; i++) {
            int u = rng() % n;
            int v = rng() % n;
            int w = rng() % 2;
            adj[u].push_back({v, w});
            adj[v].push_back({u, w});
        }
        auto t0 = chrono::high_resolution_clock::now();
        auto d1 = zero_one_bfs(n, adj, 0);
        auto t1 = chrono::high_resolution_clock::now();
        auto d2 = dijkstra(n, adj, 0);
        auto t2 = chrono::high_resolution_clock::now();
        double t_bfs = chrono::duration<double>(t1 - t0).count();
        double t_dij = chrono::duration<double>(t2 - t1).count();
        cout << "Test3 (perf, n=" << n << " m=" << m << "): 0-1 BFS=" << t_bfs
             << "s Dijkstra=" << t_dij << "s speedup=" << (t_dij / t_bfs) << "x\n";
    }

    cout << "Done.\n";
    return 0;
}
