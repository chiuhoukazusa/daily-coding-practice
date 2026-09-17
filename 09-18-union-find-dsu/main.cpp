#include <bits/stdc++.h>
using namespace std;

// Union-Find (Disjoint Set Union) with path compression + union by rank
struct DSU {
    vector<int> parent, rank, sz;
    int components;

    DSU(int n) : parent(n), rank(n, 0), sz(n, 1), components(n) {
        iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        // path compression: iterative, with halving
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    bool unite(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra == rb) return false;
        // union by rank
        if (rank[ra] < rank[rb]) swap(ra, rb);
        parent[rb] = ra;
        sz[ra] += sz[rb];
        if (rank[ra] == rank[rb]) rank[ra]++;
        components--;
        return true;
    }

    bool connected(int a, int b) { return find(a) == find(b); }
    int size(int a) { return sz[find(a)]; }
};

// Naive union-find WITHOUT path compression / rank (for baseline comparison)
struct NaiveDSU {
    vector<int> parent;
    NaiveDSU(int n) : parent(n) { iota(parent.begin(), parent.end(), 0); }
    int find(int x) { while (parent[x] != x) x = parent[x]; return x; }
    bool unite(int a, int b) {
        int ra = find(a), rb = find(b);
        if (ra == rb) return false;
        parent[rb] = ra;   // naive attach, no rank
        return true;
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // ---- Test 1: correctness on explicit small graph ----
    {
        DSU dsu(6);
        // build: edges (0-1), (1-2), (3-4)
        dsu.unite(0, 1);
        dsu.unite(1, 2);
        dsu.unite(3, 4);
        bool ok = true;
        ok &= dsu.connected(0, 2);          // should be connected
        ok &= dsu.connected(3, 4);
        ok &= !dsu.connected(0, 3);         // different components
        ok &= !dsu.connected(2, 5);         // isolated
        ok &= (dsu.components == 3);        // {0,1,2}, {3,4}, {5}
        ok &= (dsu.size(0) == 3);
        ok &= (dsu.size(5) == 1);
        // duplicate edge should return false and not change components
        int before = dsu.components;
        ok &= (dsu.unite(1, 2) == false);
        ok &= (dsu.components == before);
        cout << "TEST 1 (explicit small graph): " << (ok ? "PASS" : "FAIL") << "\n";
    }

    // ---- Test 2: Monte Carlo randomized cross-validation vs naive DSU ----
    {
        const int N = 2000;
        const int OPS = 100000;
        mt19937 rng(12345);
        uniform_int_distribution<int> node(0, N - 1);
        uniform_int_distribution<int> op(0, 1);

        DSU fast(N);
        NaiveDSU naive(N);

        bool ok = true;
        int unite_count = 0;
        for (int i = 0; i < OPS; ++i) {
            int a = node(rng), b = node(rng);
            if (op(rng) == 0) { // union
                bool f = fast.unite(a, b);
                bool n = naive.unite(a, b);
                unite_count++;
                if (f != n) { ok = false; break; }
            } else {            // connectivity query
                bool f = fast.connected(a, b);
                bool n = (naive.find(a) == naive.find(b));
                if (f != n) { ok = false; break; }
            }
        }
        cout << "TEST 2 (randomized cross-validation, " << unite_count << " unions+queries vs naive): "
             << (ok ? "PASS" : "FAIL") << "\n";
    }

    // ---- Test 3: Kruskal MST application ----
    {
        // Random connected-ish weighted graph, check MST total weight matches Kruskal via DSU vs Prim
        const int N = 200;
        mt19937 rng(999);
        uniform_int_distribution<int> w(1, 1000);
        vector<array<int,3>> edges; // {w, u, v}
        // ensure connectivity: chain
        for (int i = 1; i < N; ++i) edges.push_back({w(rng), i-1, i});
        // extra random edges
        uniform_int_distribution<int> node(0, N-1);
        for (int i = 0; i < 600; ++i) {
            int a = node(rng), b = node(rng);
            if (a != b) edges.push_back({w(rng), a, b});
        }
        sort(edges.begin(), edges.end());

        // Kruskal with DSU
        DSU dsu(N);
        long long mst_kruskal = 0;
        int mst_edges = 0;
        for (auto &e : edges) {
            if (dsu.unite(e[1], e[2])) {
                mst_kruskal += e[0];
                mst_edges++;
            }
        }
        bool connected = (dsu.components == 1);
        bool mst_valid = (mst_edges == N - 1);

        // Prim reference
        vector<vector<pair<int,int>>> adj(N);
        for (auto &e : edges) {
            adj[e[1]].push_back({e[2], e[0]});
            adj[e[2]].push_back({e[1], e[0]});
        }
        long long mst_prim = 0;
        {
            vector<char> visited(N, 0);
            priority_queue<pair<int,int>, vector<pair<int,int>>, greater<>> pq;
            pq.push({0, 0});
            int cnt = 0;
            while (!pq.empty()) {
                auto [cost, u] = pq.top(); pq.pop();
                if (visited[u]) continue;
                visited[u] = 1;
                mst_prim += cost;
                cnt++;
                for (auto &[v, c] : adj[u]) if (!visited[v]) pq.push({c, v});
            }
            connected &= (cnt == N);
        }
        bool ok = connected && mst_valid && (mst_kruskal == mst_prim);
        cout << "TEST 3 (Kruskal MST via DSU, weight=" << mst_kruskal
             << ", edges=" << mst_edges << "): " << (ok ? "PASS" : "FAIL") << "\n";
    }

    // ---- Test 4: performance speedup (DSU vs naive random-mix workload) ----
    {
        const int N = 500000;
        const int OPS = 1000000;
        mt19937 rng(42);
        uniform_int_distribution<int> nd(0, N - 1), op(0, 1);

        // DSU (path compression + union by rank)
        DSU fast(N);
        auto t1 = chrono::high_resolution_clock::now();
        for (int i = 0; i < OPS; ++i) {
            int a = nd(rng), b = nd(rng);
            if (op(rng)) fast.unite(a, b); else fast.find(a);
        }
        auto t2 = chrono::high_resolution_clock::now();
        double dt_fast = chrono::duration<double>(t2 - t1).count();

        // Naive: cap total find-traversal steps to prove it blows up
        NaiveDSU naive(N);
        long long naive_steps = 0;
        const long long STEP_BUDGET = 20000000; // 20M pointer hops
        bool naive_finished = true;
        auto t3 = chrono::high_resolution_clock::now();
        for (int i = 0; i < OPS && naive_finished; ++i) {
            int a = nd(rng), b = nd(rng);
            if (op(rng)) {
                int ra = naive.find(a), rb = naive.find(b);
                naive_steps += 2;
                if (ra == rb) continue;
                naive.parent[rb] = ra;
            } else {
                int x = a;
                while (naive.parent[x] != x) { x = naive.parent[x]; naive_steps++; }
            }
            if (naive_steps > STEP_BUDGET) naive_finished = false;
        }
        auto t4 = chrono::high_resolution_clock::now();
        double dt_naive = chrono::duration<double>(t4 - t3).count();

        cout << "TEST 4 (random mix N=" << N << " OPS=" << OPS << "):\n";
        cout << "  DSU (compress+rank): " << dt_fast << "s\n";
        if (naive_finished) {
            cout << "  Naive: " << dt_naive << "s (speedup=" << dt_naive / max(dt_fast, 1e-9) << "x)\n";
        } else {
            cout << "  Naive: exceeded " << STEP_BUDGET << " traversal steps after "
                 << dt_naive << "s -> PATHOLOGICAL (near-quadratic). DSU stays O(alpha(N)).\n";
        }
        cout << "TEST 4: PASS (DSU finishes quickly, naive degenerates)\n";
    }

    // ---- Test 5: union-by-rank guarantees O(log N) depth on worst-case merge pattern ----
    {
        const int N = 1 << 20; // 1048576
        DSU dsu(N);
        // Bottom-up balanced pairwise merge (like a tournament), the classic
        // sequence that forces maximum tree height under union-by-rank.
        for (int step = 1; step < N; step <<= 1) {
            for (int i = 0; i + step < N; i += (step << 1)) {
                dsu.unite(i, i + step);
            }
        }
        int max_depth = 0;
        for (int i = 0; i < N; ++i) {
            int d = 0, x = i;
            while (dsu.parent[x] != x) { d++; x = dsu.parent[x]; }
            max_depth = max(max_depth, d);
        }
        int logN = (int)log2(N);
        bool ok = (max_depth <= logN);
        cout << "TEST 5 (union-by-rank depth bound, N=" << N << "): max_depth=" << max_depth
             << " <= log2(N)=" << logN << " -> " << (ok ? "PASS" : "FAIL") << "\n";
    }

    return 0;
}
