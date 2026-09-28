// Sparse Table RMQ (Range Minimum Query)
// 稀疏表：静态数组区间最值查询，O(n log n) 预处理，O(1) 查询
// 量化验证：与朴素 O(n) 循环基准对比正确性 + 性能加速比

#include <bits/stdc++.h>
using namespace std;

struct SparseTable {
    int n;
    vector<int> log2_;
    vector<vector<int>> st;

    SparseTable(const vector<int>& a) {
        n = a.size();
        int K = 1;
        while ((1 << K) <= n) K++;
        st.assign(K, vector<int>(n));
        st[0] = a;
        for (int k = 1; k < K; k++) {
            int len = 1 << k;
            for (int i = 0; i + len <= n; i++) {
                st[k][i] = min(st[k - 1][i], st[k - 1][i + (1 << (k - 1))]);
            }
        }
        // 预计算 log2
        log2_.assign(n + 1, 0);
        for (int i = 2; i <= n; i++) log2_[i] = log2_[i / 2] + 1;
    }

    // 查询 [l, r] 闭区间最小值，O(1)
    int query(int l, int r) const {
        int len = r - l + 1;
        int k = log2_[len];
        return min(st[k][l], st[k][r - (1 << k) + 1]);
    }
};

// 朴素 O(n) 循环求最小值（基准）
int naive_min(const vector<int>& a, int l, int r) {
    int m = a[l];
    for (int i = l + 1; i <= r; i++) m = min(m, a[i]);
    return m;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(12345);

    // ---- 正确性验证：随机数组 + 随机查询，逐条对比朴素结果 ----
    int n = 50000;
    vector<int> arr(n);
    uniform_int_distribution<int> val(-1000000, 1000000);
    for (int& x : arr) x = val(rng);

    SparseTable st(arr);

    int Q = 200000;
    int ok = 0, fail = 0;
    uniform_int_distribution<int> idx(0, n - 1);
    for (int q = 0; q < Q; q++) {
        int l = idx(rng);
        int r = idx(rng);
        if (l > r) swap(l, r);
        int got = st.query(l, r);
        int exp = naive_min(arr, l, r);
        if (got == exp) ok++;
        else fail++;
    }

    // 特殊边界验证
    bool edge_ok = true;
    // 单元素区间
    for (int i = 0; i < 100; i++) {
        int p = idx(rng);
        if (st.query(p, p) != arr[p]) edge_ok = false;
    }
    // 整个数组
    if (st.query(0, n - 1) != naive_min(arr, 0, n - 1)) edge_ok = false;
    // 已知固定数组
    vector<int> fixed = {7, 2, 9, 1, 8, 5, 3, 6, 4};
    SparseTable stf(fixed);
    if (stf.query(1, 3) != 1 || stf.query(0, 8) != 1 || stf.query(2, 5) != 1 || stf.query(0, 0) != 7 || stf.query(8, 8) != 4)
        edge_ok = false;

    // ---- 性能基准：对比朴素 O(n) ----
    auto t0 = chrono::high_resolution_clock::now();
    long long sink1 = 0;
    uniform_int_distribution<int> idx2(0, n - 1);
    for (int q = 0; q < Q; q++) {
        int l = idx2(rng);
        int r = idx2(rng);
        if (l > r) swap(l, r);
        sink1 += st.query(l, r);
    }
    auto t1 = chrono::high_resolution_clock::now();
    double st_time = chrono::duration<double>(t1 - t0).count();

    auto t2 = chrono::high_resolution_clock::now();
    long long sink2 = 0;
    for (int q = 0; q < Q; q++) {
        int l = idx2(rng);
        int r = idx2(rng);
        if (l > r) swap(l, r);
        sink2 += naive_min(arr, l, r);
    }
    auto t3 = chrono::high_resolution_clock::now();
    double naive_time = chrono::duration<double>(t3 - t2).count();

    // ---- 输出报告 ----
    cout << "=== Sparse Table RMQ 量化验证报告 ===\n";
    cout << "数组大小 n = " << n << "\n";
    cout << "随机查询次数 Q = " << Q << "\n\n";

    cout << "[正确性验证]\n";
    cout << "随机查询匹配: " << ok << "/" << Q << " (错误 " << fail << ")\n";
    cout << "边界测试(单元素/全区间/固定数组): " << (edge_ok ? "全部通过" : "存在失败") << "\n\n";

    cout << "[性能基准]\n";
    cout << "Sparse Table 查询总耗时: " << st_time << " s (sink=" << sink1 << ")\n";
    cout << "朴素 O(n) 循环耗时:      " << naive_time << " s (sink=" << sink2 << ")\n";
    cout << "加速比: " << naive_time / st_time << "x\n";
    cout << "单次查询均摊(稀疏表): " << (st_time / Q * 1e6) << " us\n";

    // 断言判断
    bool correctness = (fail == 0) && edge_ok;
    bool speedup = (st_time < naive_time);
    cout << "\n[结论]\n";
    cout << "正确性: " << (correctness ? "PASS (100% 匹配朴素基准)" : "FAIL") << "\n";
    cout << "加速验证: " << (speedup ? "PASS (显著快于朴素 O(n))" : "FAIL") << "\n";

    return 0;
}
