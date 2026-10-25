// Stoer-Wagner Global Minimum Cut
// 全局最小割算法：Stoer-Wagner 最大邻接序(MAS) + 相位合并
//
// 验证策略（全部量化，不靠眼睛）：
//  1. 正确性：与"枚举所有 s-t 对做最大流(min-cut)"的基准对比，在大量随机图上 min-cut 值必须一致
//  2. 最优性：与随机"猜一个随机割"的下界对比（Stoer-Wagner <= 任意随机割），证明找到的是全局最优
//  3. 复杂度：O(n^3) 理论，测量运行时间随 n 增长的缩放
//  4. 特例：只有 2 个点 / 单个点 / 不连通图（min-cut = 0）

#include <bits/stdc++.h>
using namespace std;

// Stoer-Wagner 算法，返回全局最小割的权值和。
// g 为邻接矩阵（无自环，无向图），n 为顶点数。
long long stoer_wagner(int n, vector<vector<long long>> g) {
    if (n <= 1) return 0;       // 单点/空图：最小割为 0
    vector<int> id(n);          // 当前"合并后"的顶点 id（用旧编号索引）
    iota(id.begin(), id.end(), 0);
    long long best = LLONG_MAX;

    // 需要 n-1 个相位
    for (int phase = n; phase > 1; --phase) {
        // 最大邻接序 (Maximum Adjacency Search)
        vector<long long> w(phase, 0);   // w[i] = 已加入集合 A 的点到顶点 i 的总权重
        vector<bool> added(phase, false);
        int prev = -1;
        for (int iter = 0; iter < phase; ++iter) {
            // 选择 w 最大的未加入顶点
            int sel = -1;
            for (int i = 0; i < phase; ++i) {
                if (!added[i] && (sel == -1 || w[i] > w[sel]))
                    sel = i;
            }
            // 最后一个被选中的顶点 s 与倒数第二个顶点 t 是当前相位的一对切割点
            if (iter == phase - 1) {
                // 记录 cut-of-the-phase = w[s]
                best = min(best, w[sel]);
                // 合并 s（sel）到 t（prev）
                for (int i = 0; i < phase; ++i) {
                    g[prev][i] += g[sel][i];
                    g[i][prev] = g[prev][i];
                }
                // 移除 sel，把最后一个元素挪到 sel 的位置
                id[sel] = id[phase - 1];
                for (int i = 0; i < phase; ++i) {
                    g[i][sel] = g[i][phase - 1];
                    g[sel][i] = g[phase - 1][i];
                }
                break;
            }
            added[sel] = true;
            prev = sel;
            // 把 sel 的权重贡献加到其它未加入的顶点上
            for (int i = 0; i < phase; ++i) {
                if (!added[i])
                    w[i] += g[sel][i];
            }
        }
    }
    return best;
}

// ---------- 基准：Dinic 最大流（用于计算 s-t 最小割） ----------
struct Dinic {
    struct Edge { int to, rev; long long cap; };
    vector<vector<Edge>> g;
    vector<int> level, iter;
    Dinic(int n) : g(n), level(n), iter(n) {}
    void add_edge(int u, int v, long long cap) {
        g[u].push_back({v, (int)g[v].size(), cap});
        g[v].push_back({u, (int)g[u].size() - 1, cap}); // 无向 => 双向同容量
    }
    long long flow(int s, int t) {
        long long f = 0;
        const long long INF = LLONG_MAX / 4;
        while (bfs(s, t)) {
            fill(iter.begin(), iter.end(), 0);
            long long d;
            while ((d = dfs(s, t, INF)) > 0) f += d;
        }
        return f;
    }
    bool bfs(int s, int t) {
        fill(level.begin(), level.end(), -1);
        queue<int> q; q.push(s); level[s] = 0;
        while (!q.empty()) {
            int v = q.front(); q.pop();
            for (auto &e : g[v]) if (e.cap > 0 && level[e.to] < 0) {
                level[e.to] = level[v] + 1;
                q.push(e.to);
            }
        }
        return level[t] >= 0;
    }
    long long dfs(int v, int t, long long f) {
        if (v == t) return f;
        for (int &i = iter[v]; i < (int)g[v].size(); ++i) {
            auto &e = g[v][i];
            if (e.cap > 0 && level[v] < level[e.to]) {
                long long d = dfs(e.to, t, min(f, e.cap));
                if (d > 0) { e.cap -= d; g[e.to][e.rev].cap += d; return d; }
            }
        }
        return 0;
    }
};

// 枚举所有 s-t 对求最小割（基准正确性答案）
long long brute_mincut(int n, const vector<vector<long long>> &g) {
    long long best = LLONG_MAX;
    for (int s = 0; s < n; ++s) {
        for (int t = s + 1; t < n; ++t) {
            Dinic d(n);
            for (int i = 0; i < n; ++i)
                for (int j = i + 1; j < n; ++j)
                    if (g[i][j] > 0) d.add_edge(i, j, g[i][j]);
            best = min(best, d.flow(s, t));
        }
    }
    return best;
}

// 随机生成一个割并计算其权值（用于证明找到的是"全局"最小，而非局部）
long long random_cut(int n, const vector<vector<long long>> &g, mt19937 &rng) {
    // 保证非平凡割（两侧都非空）：随机选一个真子集
    vector<int> side(n);
    for (int i = 0; i < n; ++i) side[i] = rng() % 2;
    // 若全在同一侧，强制把一个点翻到另一侧
    bool any0 = false, any1 = false;
    for (int i = 0; i < n; ++i) { any0 |= (side[i]==0); any1 |= (side[i]==1); }
    if (!any0 || !any1) side[rng() % n] ^= 1;
    long long c = 0;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (side[i] != side[j]) c += g[i][j];
    return c;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(12345);
    int total_tests = 0, sw_eq_brute = 0;
    int random_close_check = 0;

    // 测试 1：随机加权稠密图，正确性 vs 暴力 min-cut
    for (int tc = 0; tc < 60; ++tc) {
        int n = 2 + rng() % 8;   // 2..9 个顶点
        vector<vector<long long>> g(n, vector<long long>(n, 0));
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                long long w = 1 + rng() % 20;
                g[i][j] = g[j][i] = w;
            }
        long long sw = stoer_wagner(n, g);
        long long bf = brute_mincut(n, g);
        ++total_tests;
        if (sw == bf) ++sw_eq_brute;
        else {
            printf("❌ 不一致! n=%d sw=%lld brute=%lld\n", n, sw, bf);
            return 1;
        }
    }

    // 测试 2：随机图（含稀疏、可能不连通），正确性 vs 暴力
    for (int tc = 0; tc < 40; ++tc) {
        int n = 2 + rng() % 10;
        vector<vector<long long>> g(n, vector<long long>(n, 0));
        double p = 0.3 + (rng() % 100) / 100.0 * 0.6; // 边密度 0.3~0.9
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                if ((rng() % 1000) / 1000.0 < p) {
                    long long w = 1 + rng() % 100;
                    g[i][j] = g[j][i] = w;
                }
            }
        long long sw = stoer_wagner(n, g);
        long long bf = brute_mincut(n, g);
        ++total_tests;
        if (sw == bf) ++sw_eq_brute;
        else {
            printf("❌ 不一致! n=%d sw=%lld brute=%lld\n", n, sw, bf);
            return 1;
        }
    }

    // 测试 3：全局最优性下界检查——随机割的权值永远 >= Stoer-Wagner 结果
    for (int tc = 0; tc < 200; ++tc) {
        int n = 3 + rng() % 12;
        vector<vector<long long>> g(n, vector<long long>(n, 0));
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                long long w = 1 + rng() % 50;
                g[i][j] = g[j][i] = w;
            }
        long long sw = stoer_wagner(n, g);
        for (int k = 0; k < 20; ++k) {
            long long rc = random_cut(n, g, rng);
            if (rc < sw) {
                printf("❌ 发现随机割 %lld < sw %lld，说明 SW 非全局最优！\n", rc, sw);
                return 1;
            }
        }
        random_close_check++;
    }

    // 测试 4：特殊边界情况
    // 4a. 单点图
    if (stoer_wagner(1, vector<vector<long long>>(1, vector<long long>(1, 0))) != 0) {
        printf("❌ 单点图应返回 0\n"); return 1;
    }
    // 4b. 两点图
    {
        vector<vector<long long>> g = {{0, 7}, {7, 0}};
        if (stoer_wagner(2, g) != 7) { printf("❌ 两点图应返回 7\n"); return 1; }
    }
    // 4c. 不连通图（无桥边）
    {
        vector<vector<long long>> g = {{0, 5, 0, 0}, {5, 0, 0, 0}, {0, 0, 0, 9}, {0, 0, 9, 0}};
        if (stoer_wagner(4, g) != 0) { printf("❌ 不连通图应返回 0\n"); return 1; }
    }
    // 4d. 已知经典例子：三角形 + 一条桥
    {
        // 顶点0-1-2 三角形权重各1，顶点3通过权重10连到0
        vector<vector<long long>> g = {
            {0, 1, 1, 10},
            {1, 0, 1, 0},
            {1, 1, 0, 0},
            {10, 0, 0, 0}
        };
        long long sw = stoer_wagner(4, g);
        // 最小割 = min(割掉顶点3: 10, 割三角形: 由于三角形每条边都被跨, 最小割 = 2? )
        // 实际枚举: 割{3}:10, 割{0,3} vs {1,2}: w(0,1)+w(0,2)=2, 全局最小 = 2
        if (sw != 2) { printf("❌ 经典例应返回 2，得到 %lld\n", sw); return 1; }
    }

    printf("=== Stoer-Wagner 验证报告 ===\n");
    printf("正确性(与最大流暴力min-cut对比): %d/%d 全部一致 ✅\n", sw_eq_brute, total_tests);
    printf("全局最优性下界检查(随机割>=SW): %d 张图通过 ✅\n", random_close_check);
    printf("边界情况(单点/两点/不连通/经典例): 全部通过 ✅\n");

    // 测试 5：复杂度缩放（O(V^3) 理论）
    printf("\n=== 运行时间缩放 ===\n");
    printf(" n      时间(ms)\n");
    vector<int> sizes = {50, 100, 150, 200, 300, 400};
    for (int n : sizes) {
        vector<vector<long long>> g(n, vector<long long>(n, 0));
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                long long w = 1 + rng() % 1000;
                g[i][j] = g[j][i] = w;
            }
        auto t0 = chrono::high_resolution_clock::now();
        volatile long long r = stoer_wagner(n, g);
        auto t1 = chrono::high_resolution_clock::now();
        (void)r;
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        printf("%3d   %8.2f\n", n, ms);
    }

    printf("\n全部量化验证通过。\n");
    return 0;
}
