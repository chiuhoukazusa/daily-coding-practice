// Edmonds' Blossom Algorithm — General Graph Maximum Matching
// Implementation following the classic "blossom" augmentation (E-Maxx style).
// Uses recursive find_path with blossom contraction. Verified vs brute force.
#include <bits/stdc++.h>
using namespace std;

struct Blossom {
    int n;
    vector<vector<int>> g;
    vector<int> match, p, base, used, blossom;

    Blossom(int n_) : n(n_), g(n_), match(n_, -1), p(n_), base(n_), used(n_), blossom(n_) {}

    void add_edge(int u, int v) { g[u].push_back(v); g[v].push_back(u); }

    int lca(int a, int b) {
        vector<int> mark(n, 0);
        while (true) {
            a = base[a];
            mark[a] = 1;
            if (match[a] == -1) break;
            a = p[match[a]];
        }
        while (true) {
            b = base[b];
            if (mark[b]) return b;
            b = p[match[b]];
        }
    }

    void mark_path(int v, int b, int child) {
        while (base[v] != b) {
            blossom[base[v]] = blossom[base[match[v]]] = b;
            p[v] = child;
            child = match[v];
            v = p[match[v]];
        }
    }

    int find_path(int root) {
        used.assign(n, 0);
        p.assign(n, -1);
        iota(base.begin(), base.end(), 0);
        deque<int> q{root};
        used[root] = 1;

        while (!q.empty()) {
            int v = q.front(); q.pop_front();
            for (int to : g[v]) {
                if (base[v] == base[to] || match[v] == to) continue;
                if (to == root || (match[to] != -1 && p[match[to]] != -1)) {
                    int curbase = lca(v, to);
                    blossom.assign(n, -1);
                    mark_path(v, curbase, to);
                    mark_path(to, curbase, v);
                    for (int i = 0; i < n; ++i)
                        if (blossom[base[i]] != -1) {
                            base[i] = curbase;
                            if (!used[i]) { used[i] = 1; q.push_back(i); }
                        }
                } else if (p[to] == -1) {
                    p[to] = v;
                    if (match[to] == -1) {
                        int cur = to;
                        while (cur != -1) {
                            int nxt = match[p[cur]];
                            match[cur] = p[cur];
                            match[p[cur]] = cur;
                            cur = nxt;
                        }
                        return 1;
                    }
                    used[match[to]] = 1;
                    q.push_back(match[to]);
                }
            }
        }
        return 0;
    }

    int max_matching() {
        int res = 0;
        for (int v = 0; v < n; ++v)
            if (match[v] == -1)
                for (int to : g[v])
                    if (match[to] == -1) { match[v] = to; match[to] = v; ++res; break; }
        for (int v = 0; v < n; ++v)
            if (match[v] == -1)
                res += find_path(v);
        return res;
    }
};

int brute_max_matching(int n, const vector<pair<int,int>>& edges) {
    (void)n;
    int m = edges.size();
    int best = 0;
    for (long long mask = 0; mask < (1LL << m); ++mask) {
        if ((int)__builtin_popcountll(mask) <= best) continue;
        int used[64] = {0}; bool ok = true; int cnt = 0;
        for (int i = 0; i < m && ok; ++i) if ((mask >> i) & 1LL) {
            int u = edges[i].first, v = edges[i].second;
            if (used[u] || used[v]) { ok = false; break; }
            used[u] = used[v] = 1; cnt++;
        }
        if (ok) best = max(best, cnt);
    }
    return best;
}

int main() {
    mt19937 rng(20261023);
    int total = 0, mismatches = 0, small_pass = 0, dense_pass = 0;

    auto run_case = [&](int n, vector<pair<int,int>> edges, const char* tag) {
        Blossom bl(n);
        for (auto& e : edges) bl.add_edge(e.first, e.second);
        int a = bl.max_matching();
        int b = brute_max_matching(n, edges);
        total++;
        if (a != b) { mismatches++; if (mismatches <= 8) printf("MISMATCH[%s] n=%d blossom=%d brute=%d\n", tag, n, a, b); }
        return a == b;
    };

    for (int t = 0; t < 300; ++t) {
        int n = 2 + (rng() % 9);
        int m = rng() % (min(18, n*(n-1)/2) + 1);
        vector<pair<int,int>> edges; set<pair<int,int>> seen;
        while ((int)edges.size() < m) {
            int u = rng()%n, v = rng()%n; if (u==v) continue;
            if (u>v) swap(u,v);
            if (seen.count({u,v})) continue;
            seen.insert({u,v}); edges.push_back({u,v});
        }
        if (run_case(n, edges, "small")) small_pass++;
    }
    for (int t = 0; t < 150; ++t) {
        int n = 12, m = 18;
        vector<pair<int,int>> edges; set<pair<int,int>> seen;
        while ((int)edges.size() < m) {
            int u = rng()%n, v = rng()%n; if (u==v) continue;
            if (u>v) swap(u,v);
            if (seen.count({u,v})) continue;
            seen.insert({u,v}); edges.push_back({u,v});
        }
        if (run_case(n, edges, "dense")) dense_pass++;
    }

    run_case(5, {{0,1},{1,2},{2,3},{3,4},{4,0}}, "C5");
    run_case(7, {{0,1},{1,2},{2,3},{3,4},{4,5},{5,6},{6,0}}, "C7");
    run_case(3, {{0,1},{1,2},{2,0}}, "C3");
    run_case(5, {{0,1},{1,2},{2,0},{2,3},{3,4},{4,2}}, "2tri");
    run_case(5, {{0,1},{1,2},{2,0},{3,4}}, "tri+edge");
    // sunflower: center 0 connected to k petals, each petal is a triangle (0,i,i+1)
    // This forces many overlapping blossoms. Known: with 2k+1 vertices in k triangles,
    // center can only be matched once, so max matching = k (each remaining triangle
    // contributes one matched edge among its two non-center vertices... actually
    // center in one triangle leaves k-1 triangles each giving 1 edge = k total).
    // We just verify vs brute force.
    for (int k = 2; k <= 4; ++k) {
        // build k disjoint triangles all sharing center 0
        vector<pair<int,int>> e;
        vector<pair<int,int>> cx;
        for (int i = 0; i < k; ++i) {
            int a = 1 + 2*i, b = 2 + 2*i;
            e.push_back({0, a}); e.push_back({0, b}); e.push_back({a, b});
        }
        int n = 1 + 2*k;
        run_case(n, e, "sunflower");
    }
    for (int K : {4,5,6,7}) {
        vector<pair<int,int>> e;
        for (int i=0;i<K;i++) for (int j=i+1;j<K;j++) e.push_back({i,j});
        run_case(K, e, "Kn");
    }

    {
        int n = 30; vector<pair<int,int>> edges;
        for (int u=0;u<n;u++) for (int v=u+1;v<n;v++) if (rng()%100<15) edges.push_back({u,v});
        Blossom bl(n); for (auto& e : edges) bl.add_edge(e.first, e.second);
        int a = bl.max_matching(); bool valid = true; int covered = 0;
        for (int v=0;v<n;v++) { if (bl.match[v]==-1) continue; int u = bl.match[v];
            if (u<0||u>=n||u==v||bl.match[u]!=v) valid=false;
            covered++; }
        total++; if (!valid || covered/2 != a) mismatches++;
        printf("medium n=%d blossom=%d covered=%d valid=%d\n", n, a, covered, valid?1:0);
    }
    {
        int n = 200; vector<pair<int,int>> edges;
        for (int u=0;u<n;u++) for (int v=u+1;v<n;v++) if (rng()%100<5) edges.push_back({u,v});
        Blossom bl(n); for (auto& e : edges) bl.add_edge(e.first, e.second);
        auto t0 = chrono::high_resolution_clock::now();
        int a = bl.max_matching();
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double,milli>(t1-t0).count();
        bool valid = true;
        for (int v=0;v<n;v++) { if (bl.match[v]==-1) continue; if (bl.match[bl.match[v]]!=v) valid=false; }
        total++; if (!valid) mismatches++;
        printf("large n=%d edges=%zu blossom=%d valid=%d time=%.2fms\n", n, edges.size(), a, valid?1:0, ms);
    }

    printf("\n=== SUMMARY ===\n");
    printf("small_random_pass=%d/300 dense_pass=%d/150\n", small_pass, dense_pass);
    printf("total_graphs=%d mismatches=%d\n", total, mismatches);
    printf("RESULT=%s\n", mismatches==0 ? "PASS" : "FAIL");
    return mismatches==0 ? 0 : 1;
}
