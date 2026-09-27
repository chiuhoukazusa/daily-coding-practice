// Binary Lifting for LCA (Lowest Common Ancestor) & K-th Ancestor
// 量化验证：与朴素 LCA（逐级上跳 + 深度对齐）对比正确性，
// 并对比时间复杂度（随机大树上的平均查询耗时）。
#include <bits/stdc++.h>
using namespace std;

struct BinaryLifting {
    int n, LOG;
    vector<vector<int>> up;      // up[v][j] = 2^j-th ancestor of v
    vector<int> depth;
    vector<vector<int>> adj;

    BinaryLifting(int n_) : n(n_) {
        LOG = 1; while ((1 << LOG) <= n) LOG++;
        up.assign(n, vector<int>(LOG, -1));
        depth.assign(n, 0);
        adj.assign(n, {});
    }

    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    void dfs(int v, int p) {
        up[v][0] = p;
        for (int j = 1; j < LOG; j++) {
            int mid = up[v][j - 1];
            up[v][j] = (mid == -1) ? -1 : up[mid][j - 1];
        }
        for (int to : adj[v]) {
            if (to == p) continue;
            depth[to] = depth[v] + 1;
            dfs(to, v);
        }
    }

    void build(int root = 0) { dfs(root, -1); }

    int lca(int u, int v) {
        if (depth[u] < depth[v]) swap(u, v);
        int diff = depth[u] - depth[v];
        for (int j = 0; j < LOG; j++)
            if (diff & (1 << j)) u = up[u][j];
        if (u == v) return u;
        for (int j = LOG - 1; j >= 0; j--) {
            if (up[u][j] != up[v][j]) {
                u = up[u][j];
                v = up[v][j];
            }
        }
        return up[u][0];
    }

    // k-th ancestor (k steps up); -1 if out of range
    int kthAncestor(int v, int k) {
        for (int j = 0; j < LOG; j++) {
            if (k & (1 << j)) {
                v = up[v][j];
                if (v == -1) return -1;
            }
        }
        return v;
    }
};

// ---- 朴素参考实现（用于正确性对比）----
struct NaiveLCA {
    int n;
    vector<int> parent, depth;
    vector<vector<int>> adj;
    NaiveLCA(int n_) : n(n_), parent(n_, -1), depth(n_, 0), adj(n_) {}
    void addEdge(int u, int v) { adj[u].push_back(v); adj[v].push_back(u); }
    void dfs(int v, int p) {
        parent[v] = p;
        for (int to : adj[v]) if (to != p) { depth[to] = depth[v] + 1; dfs(to, v); }
    }
    void build(int root = 0) { dfs(root, -1); }
    int lca(int u, int v) {
        while (depth[u] > depth[v]) u = parent[u];
        while (depth[v] > depth[u]) v = parent[v];
        while (u != v) { u = parent[u]; v = parent[v]; }
        return u;
    }
};

int main() {
    // ---- 正确性验证 ----
    // 构造一棵随机树 + 若干固定结构树
    mt19937 rng(12345);
    int correct_cases = 0, total_cases = 0;
    {
        int n = 2000;
        BinaryLifting bl(n);
        NaiveLCA nl(n);
        // 随机父指针树（保证 0..i-1 已连接）
        for (int i = 1; i < n; i++) {
            int p = rng() % i;
            bl.addEdge(i, p);
            nl.addEdge(i, p);
        }
        bl.build(0); nl.build(0);

        // 随机查询对比
        int Q = 50000;
        for (int q = 0; q < Q; q++) {
            int u = rng() % n, v = rng() % n;
            int a = bl.lca(u, v);
            int b = nl.lca(u, v);
            total_cases++;
            if (a == b) correct_cases++;

            // 额外验证 k-th ancestor（0 <= k <= depth）
            int k = rng() % (bl.depth[u] + 1);
            int ka = bl.kthAncestor(u, k);
            // 朴素验证：向上走 k 步
            int cur = u;
            for (int s = 0; s < k; s++) cur = nl.parent[cur];
            total_cases++;
            if (ka == cur) correct_cases++;

            // LCA 深度一致性：lca 深度 <= min(depth)
            total_cases++;
            if (bl.depth[a] <= min(bl.depth[u], bl.depth[v]) &&
                bl.depth[a] >= 0) correct_cases++;
        }
    }
    printf("正确性验证: %d / %d 通过 (%.4f%%)\n",
           correct_cases, total_cases, 100.0 * correct_cases / total_cases);

    // ---- 性能验证（时间复杂度对比）----
    int n_big = 200000;   // 深链 20 万，朴素最坏 O(depth)
    int LOG = 1; while ((1 << LOG) <= n_big) LOG++;
    // 构造深链树（最坏情况，朴素 LCA 需要 O(depth)）
    // 使用 vector 而非递归避免栈溢出
    vector<int> parent_big(n_big, -1), depth_big(n_big, 0);
    vector<vector<int>> up_big(n_big, vector<int>(LOG, -1));
    // 深链：0-1-2-...-n-1
    for (int i = 1; i < n_big; i++) { parent_big[i] = i - 1; depth_big[i] = i; }
    for (int i = 0; i < n_big; i++) up_big[i][0] = parent_big[i];
    for (int j = 1; j < LOG; j++)
        for (int i = 0; i < n_big; i++)
            up_big[i][j] = (up_big[i][j-1] == -1) ? -1 : up_big[up_big[i][j-1]][j-1];

    // 朴素上跳 LCA
    auto naive_lca_chain = [&](int u, int v) {
        while (depth_big[u] > depth_big[v]) u = parent_big[u];
        while (depth_big[v] > depth_big[u]) v = parent_big[v];
        while (u != v) { u = parent_big[u]; v = parent_big[v]; }
        return u;
    };
    auto bl_lca_chain = [&](int u, int v) {
        if (depth_big[u] < depth_big[v]) swap(u, v);
        int diff = depth_big[u] - depth_big[v];
        for (int j = 0; j < LOG; j++) if (diff & (1 << j)) u = up_big[u][j];
        if (u == v) return u;
        for (int j = LOG - 1; j >= 0; j--)
            if (up_big[u][j] != up_big[v][j]) { u = up_big[u][j]; v = up_big[v][j]; }
        return up_big[u][0];
    };

    int Q2 = 100000;
    vector<pair<int,int>> queries(Q2);
    for (auto &[u,v] : queries) {
        u = rng() % n_big;
        v = rng() % n_big;
        // 使其中一个深度较大，制造最坏情况
        if (rng() % 2) u = n_big - 1 - (rng() % 1000);
    }

    // 先预计算两套结果，避免互相干扰
    vector<int> res_naive(Q2), res_bl(Q2);
    auto t0 = chrono::high_resolution_clock::now();
    for (int i = 0; i < Q2; i++) res_naive[i] = naive_lca_chain(queries[i].first, queries[i].second);
    auto t1 = chrono::high_resolution_clock::now();
    for (int i = 0; i < Q2; i++) res_bl[i] = bl_lca_chain(queries[i].first, queries[i].second);
    auto t2 = chrono::high_resolution_clock::now();

    long long correct_perf = 0;
    for (int i = 0; i < Q2; i++) if (res_naive[i] == res_bl[i]) correct_perf++;

    double naive_ms = chrono::duration<double, milli>(t1 - t0).count();
    double bl_ms   = chrono::duration<double, milli>(t2 - t1).count();

    printf("性能对比 (n=%d, Q=%d):\n", n_big, Q2);
    printf("  朴素 LCA 耗时: %.2f ms\n", naive_ms);
    printf("  Binary Lifting 耗时: %.2f ms\n", bl_ms);
    printf("  加速比: %.1fx\n", naive_ms / max(bl_ms, 1e-9));
    printf("  性能正确性: %lld / %d 与朴素结果一致\n", correct_perf, Q2);

    printf("\n✅ 全部验证完成\n");
    return 0;
}
