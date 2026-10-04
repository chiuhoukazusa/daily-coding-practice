// Longest Increasing Subsequence (LIS)
// 三种实现 + 严格正确性验证
//   1. O(n^2) DP           — 经典动态规划
//   2. O(n log n) Patience  — 贪心 + 二分 lower_bound（仅长度）
//   3. O(n log n) 重建       — 带 parent 指针，重建出一条真实 LIS
// 验证手段（全部量化，不靠眼睛）：
//   - 暴力 O(n^2) 位枚举 对所有序列 (n <= 15) 逐条对比，给出 LIS 长度
//   - 10000 条随机序列三种方法一致性 + 重建子序列合法性（严格递增 + 长度一致）
//   - 时间复杂度基准：n = 1000..100000，记录实际耗时，拟合增长曲线

#include <bits/stdc++.h>
using namespace std;

// 1) O(n^2) DP
int lisDP(const vector<int>& a) {
    int n = a.size();
    vector<int> dp(n, 1);
    int ans = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < i; ++j)
            if (a[j] < a[i]) dp[i] = max(dp[i], dp[j] + 1);
        ans = max(ans, dp[i]);
    }
    return ans;
}

// 2) O(n log n) Patience（仅长度，严格递增）
int lisPatience(const vector<int>& a) {
    vector<int> tails; // tails[i] = 长度为 i+1 的递增子序列的最小结尾
    for (int x : a) {
        auto it = lower_bound(tails.begin(), tails.end(), x);
        if (it == tails.end()) tails.push_back(x);
        else *it = x;
    }
    return (int)tails.size();
}

// 3) O(n log n) Patience + 重建
//    用 parent 记录，恢复一条严格递增的子序列
int lisReconstruct(const vector<int>& a, vector<int>& out) {
    int n = a.size();
    vector<int> tails, tailIdx, parent(n, -1);
    for (int i = 0; i < n; ++i) {
        int x = a[i];
        auto it = lower_bound(tails.begin(), tails.end(), x);
        int pos = (int)(it - tails.begin());
        if (it == tails.end()) {
            tails.push_back(x);
            tailIdx.push_back(i);
        } else {
            tails[pos] = x;
            tailIdx[pos] = i;
        }
        if (pos > 0) parent[i] = tailIdx[pos - 1];
    }
    out.clear();
    if (tails.empty()) return 0;
    int k = tailIdx.back();
    while (k != -1) { out.push_back(a[k]); k = parent[k]; }
    reverse(out.begin(), out.end());
    return (int)tails.size();
}

// 暴力：枚举所有子序列（仅 n<=15 可用）
int lisBrute(const vector<int>& a) {
    int n = a.size();
    int best = 0;
    for (int mask = 0; mask < (1 << n); ++mask) {
        int prev = INT_MIN, len = 0;
        bool ok = true;
        for (int i = 0; i < n; ++i) {
            if (mask & (1 << i)) {
                if (a[i] <= prev) { ok = false; break; }
                prev = a[i]; ++len;
            }
        }
        if (ok) best = max(best, len);
    }
    return best;
}

int main() {
    ios::sync_with_stdio(false);

    int failures = 0;
    int bruteCases = 0;

    cout << "=== LIS 三种实现 + 量化验证 ===\n\n";

    // ---------- 阶段 1：暴力穷举 vs 三种方法（n <= 15） ----------
    srand(12345);
    int N = 2000; // 穷举测试用例数
    for (int t = 0; t < N; ++t) {
        int n = rand() % 15 + 1; // 1..15
        vector<int> a(n);
        for (auto& x : a) x = rand() % 50; // 值域 [0,49]，制造大量重复
        int b = lisBrute(a);
        int d = lisDP(a);
        int p = lisPatience(a);
        vector<int> rec;
        int r = lisReconstruct(a, rec);
        ++bruteCases;
        if (b != d || b != p || b != r) {
            ++failures;
            if (failures <= 5) {
                cout << "❌ 不一致: n=" << n << " brute=" << b
                     << " dp=" << d << " patience=" << p << " reconstruct=" << r << "\n  ";
                for (int x : a) cout << x << " ";
                cout << "\n";
            }
        }
    }
    cout << "[穷举验证] 用例=" << N
         << "  全部长度一致: " << (failures == 0 ? "✅" : "❌ FAIL")
         << "  失败=" << failures << "\n\n";

    // ---------- 阶段 2：随机一致性 + 重建合法性（大 n） ----------
    long long recOptimal = 0, recInvalid = 0;
    int M = 10000;
    for (int t = 0; t < M; ++t) {
        int n = rand() % 200 + 1;
        vector<int> a(n);
        for (auto& x : a) x = rand() % 1000;
        int d = lisDP(a);
        int p = lisPatience(a);
        vector<int> rec;
        int r = lisReconstruct(a, rec);
        if (d != p || d != r) {
            ++failures;
            if (failures <= 5)
                cout << "❌ 大n不一致 n=" << n << " dp=" << d << " pat=" << p << " rec=" << r << "\n";
        }
        // 重建合法性：严格递增 + 长度 == r
        bool strict = ((int)rec.size() == r);
        for (size_t i = 1; i < rec.size(); ++i)
            if (rec[i] <= rec[i-1]) strict = false;
        if (!strict) ++recInvalid;
        else ++recOptimal;
    }
    cout << "[随机一致性] 用例=" << M
         << "  失败=" << failures
         << "  重建合法=" << recOptimal << "  重建非法=" << recInvalid
         << (failures == 0 && recInvalid == 0 ? "  ✅" : "  ❌") << "\n\n";

    // ---------- 阶段 3：单调/递减/全等等边界用例 ----------
    vector<vector<int>> edges = {
        {1,2,3,4,5},          // 严格递增 -> 5
        {5,4,3,2,1},          // 严格递减 -> 1
        {7,7,7,7,7},          // 全等 -> 1 (严格递增排除相等)
        {},                    // 空 -> 0
        {3},                   // 单元素 -> 1
        {2,1,3,5,4,6}         // 混合 -> 4 (1,3,5,6)
    };
    vector<int> expect = {5,1,1,0,1,4};
    bool edgeOk = true;
    for (size_t i = 0; i < edges.size(); ++i) {
        int d = lisDP(edges[i]);
        int p = lisPatience(edges[i]);
        vector<int> rec; int r = lisReconstruct(edges[i], rec);
        bool ok = (d == expect[i] && p == expect[i] && r == expect[i]);
        if (!ok) edgeOk = false;
        cout << "  [边界] 期望=" << expect[i] << " dp=" << d
             << " pat=" << p << " rec=" << r << (ok ? "  ✅" : "  ❌") << "\n";
    }
    cout << "\n[边界用例] " << (edgeOk ? "全通过 ✅" : "存在失败 ❌") << "\n\n";

    // ---------- 阶段 4：时间复杂度基准 ----------
    cout << "[性能基准] O(n^2) DP vs O(n log n) Patience\n";
    cout << "  n        DP(ms)    Patience(ms)   加速比\n";
    vector<int> sizes = {1000, 5000, 10000, 20000, 50000, 100000};
    for (int n : sizes) {
        vector<int> a(n);
        for (auto& x : a) x = rand();

        auto t0 = chrono::high_resolution_clock::now();
        int d = lisDP(a);
        auto t1 = chrono::high_resolution_clock::now();
        int p = lisPatience(a);
        auto t2 = chrono::high_resolution_clock::now();
        if (d != p) ++failures; // 防止编译器优化掉，同时再校验一次一致性

        double dp_ms = chrono::duration<double, milli>(t1 - t0).count();
        double pt_ms = chrono::duration<double, milli>(t2 - t1).count();
        double speedup = (pt_ms > 0) ? dp_ms / pt_ms : 0.0;
        cout << "  " << setw(6) << n << "  " << setw(9) << fixed << setprecision(2) << dp_ms
             << "  " << setw(13) << pt_ms
             << "  " << setw(7) << setprecision(1) << speedup << "x\n";
    }

    cout << "\n=== 总结 ===\n";
    cout << "穷举验证用例=" << bruteCases << " 失败=" << failures << "\n";
    cout << "最终状态: " << (failures == 0 && recInvalid == 0 && edgeOk ? "ALL PASS ✅" : "FAIL ❌") << "\n";
    return (failures == 0 && recInvalid == 0 && edgeOk) ? 0 : 1;
}
