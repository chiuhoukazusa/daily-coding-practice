// Hungarian Algorithm (Kuhn-Munkres) — Assignment Problem
// 带权二分图最大权完美匹配 / 最小代价分配
//
// 目标：求解 n×n 代价矩阵的最小总代价分配（每行分配一个不同列）。
// 量化验证：
//   1. 与暴力枚举全排列求最小代价对比（n 小），确保最优性。
//   2. 与贪心逐行取最小对比，展示匈牙利算法优于贪心。
//   3. 大 n 下性能与 O(n^3) 复杂度一致性验证。
//   4. 随机矩阵的最大权版（取负转最小权）正确性。

#include <bits/stdc++.h>
using namespace std;

// 最小代价版 Hungarian，O(n^3)。cost[i][j] 是第 i 行分配第 j 列的代价。
// 返回 (最小总代价, 分配 vector assign[i] = 分配的列)。
pair<long long, vector<int>> hungarianMin(const vector<vector<long long>>& cost) {
    int n = cost.size();
    const long long INF = 4e18;
    vector<long long> u(n + 1, 0), v(n + 1, 0); // 顶标
    vector<int> p(n + 1, 0), way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        vector<long long> minv(n + 1, INF);
        vector<char> used(n + 1, false);
        do {
            used[j0] = true;
            int i0 = p[j0], j1 = -1;
            long long delta = INF;
            for (int j = 1; j <= n; ++j) {
                if (!used[j]) {
                    long long cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    vector<int> assign(n, -1);
    for (int j = 1; j <= n; ++j) {
        if (p[j] > 0) assign[p[j] - 1] = j - 1;
    }
    long long total = -v[0];
    return {total, assign};
}

// 最大权完美匹配：将权值取负转成最小代价问题，再取回。
pair<long long, vector<int>> hungarianMax(const vector<vector<long long>>& w) {
    int n = w.size();
    vector<vector<long long>> neg(n, vector<long long>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            neg[i][j] = -w[i][j];
    auto [mn, assign] = hungarianMin(neg);
    return {-mn, assign};
}

// 暴力枚举全排列求最小代价（仅用于小 n 基准验证）。
long long bruteMin(const vector<vector<long long>>& cost) {
    int n = cost.size();
    vector<int> perm(n);
    iota(perm.begin(), perm.end(), 0);
    long long best = 4e18;
    do {
        long long s = 0;
        for (int i = 0; i < n; ++i) s += cost[i][perm[i]];
        if (s < best) best = s;
    } while (next_permutation(perm.begin(), perm.end()));
    return best;
}

// 贪心逐行取最小可用列（作为对比基线，演示其非最优性）。
long long greedyMin(const vector<vector<long long>>& cost) {
    int n = cost.size();
    vector<char> usedCol(n, false);
    long long s = 0;
    // 按行处理：为了让贪心像样一点，每次在所有未分配行中挑全局最小合法边
    // 这里实现为按行顺序贪心。
    for (int i = 0; i < n; ++i) {
        long long best = 4e18;
        int bestj = -1;
        for (int j = 0; j < n; ++j) {
            if (!usedCol[j] && cost[i][j] < best) {
                best = cost[i][j];
                bestj = j;
            }
        }
        usedCol[bestj] = true;
        s += best;
    }
    return s;
}

int main() {
    mt19937 rng(20260925);

    cout << "=== Hungarian Algorithm (Kuhn-Munkres) 验证 ===" << endl << endl;

    // --- 测试 1：与暴力全排列对比（n=2..9，多次随机） ---
    int correct = 0, totalTests = 0;
    for (int n = 2; n <= 9; ++n) {
        for (int trial = 0; trial < 200; ++trial) {
            vector<vector<long long>> cost(n, vector<long long>(n));
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    cost[i][j] = (int)(rng() % 1000);
            auto [hm, _] = hungarianMin(cost);
            long long bm = bruteMin(cost);
            ++totalTests;
            if (hm == bm) ++correct;
        }
    }
    cout << "[1] 最优性 vs 暴力全排列: " << correct << "/" << totalTests << " 一致";
    cout << (correct == totalTests ? "  ✅" : "  ❌") << endl;

    // --- 测试 2：贪心对比，展示贪心非最优 ---
    int greedyWorse = 0, gtotal = 0;
    for (int n = 5; n <= 8; ++n) {
        for (int trial = 0; trial < 200; ++trial) {
            vector<vector<long long>> cost(n, vector<long long>(n));
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    cost[i][j] = (int)(rng() % 1000);
            auto [hm, _] = hungarianMin(cost);
            long long gm = greedyMin(cost);
            ++gtotal;
            if (gm > hm) ++greedyWorse;
        }
    }
    cout << "[2] 贪心比对: 贪心非最优比例 " << greedyWorse << "/" << gtotal
         << " (" << 100.0 * greedyWorse / gtotal << "%)";
    cout << (greedyWorse > 0 ? "  ✅ (证明贪心非最优，匈牙利必需)" : "  ❌") << endl;

    // --- 测试 3：最大权版正确性（对偶验证：max(w) == -min(-w)） ---
    int maxCorrect = 0, mtotal = 0;
    for (int n = 3; n <= 8; ++n) {
        for (int trial = 0; trial < 200; ++trial) {
            vector<vector<long long>> w(n, vector<long long>(n));
            long long mx = -1e18;
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j) {
                    w[i][j] = (long long)(rng() % 2000 - 1000);
                    mx = max(mx, w[i][j]);
                }
            auto [hmax, _] = hungarianMax(w);
            // 暴力求最大权
            vector<int> perm(n);
            iota(perm.begin(), perm.end(), 0);
            long long bmax = -4e18;
            do {
                long long s = 0;
                for (int i = 0; i < n; ++i) s += w[i][perm[i]];
                bmax = max(bmax, s);
            } while (next_permutation(perm.begin(), perm.end()));
            ++mtotal;
            if (hmax == bmax) ++maxCorrect;
        }
    }
    cout << "[3] 最大权匹配 vs 暴力: " << maxCorrect << "/" << mtotal << " 一致";
    cout << (maxCorrect == mtotal ? "  ✅" : "  ❌") << endl;

    // --- 测试 4：分配唯一性（每行分配且列不重复） ---
    bool permValid = true;
    for (int n = 2; n <= 10; ++n) {
        vector<vector<long long>> cost(n, vector<long long>(n));
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                cost[i][j] = (int)(rng() % 10000);
        auto [_, assign] = hungarianMin(cost);
        vector<char> usedCol(n, false);
        for (int i = 0; i < n; ++i) {
            if (assign[i] < 0 || assign[i] >= n || usedCol[assign[i]]) { permValid = false; break; }
            usedCol[assign[i]] = true;
        }
    }
    cout << "[4] 分配合法性(排列): " << (permValid ? "✅" : "❌") << endl;

    // --- 测试 5：性能 / 复杂度 (n 增长) ---
    cout << "[5] 性能 (O(n^3) 验证):" << endl;
    vector<int> sizes = {100, 200, 400, 800};
    for (int n : sizes) {
        vector<vector<long long>> cost(n, vector<long long>(n));
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                cost[i][j] = (long long)(rng() % 1000000);
        auto t0 = chrono::high_resolution_clock::now();
        auto [res, _] = hungarianMin(cost);
        auto t1 = chrono::high_resolution_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        cout << "    n=" << setw(4) << n << "  时间=" << setw(8) << fixed << setprecision(2)
             << ms << " ms   结果=" << res << endl;
    }

    cout << endl << "=== 验证完成 ===" << endl;
    return 0;
}
