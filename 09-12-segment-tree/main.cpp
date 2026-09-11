// Segment Tree (Range Sum Query + Lazy Propagation)
// 线段树：区间求和查询 + 区间加法更新（懒标记），并支持按值查找第 k 个元素
// 验证策略：与暴力 O(n) 基准做随机对拍，量化正确率 + 加速比

#include <bits/stdc++.h>
using namespace std;

typedef long long ll;

// ---------- Segment Tree ----------
struct SegTree {
    int n;
    vector<ll> tree;   // 区间和
    vector<ll> lazy;   // 懒标记（区间加法）

    SegTree(const vector<ll>& arr) {
        n = arr.size();
        tree.assign(4 * n, 0);
        lazy.assign(4 * n, 0);
        build(1, 0, n - 1, arr);
    }

    void build(int node, int l, int r, const vector<ll>& arr) {
        if (l == r) {
            tree[node] = arr[l];
            return;
        }
        int mid = (l + r) >> 1;
        build(node * 2, l, mid, arr);
        build(node * 2 + 1, mid + 1, r, arr);
        tree[node] = tree[node * 2] + tree[node * 2 + 1];
    }

    void push_down(int node, int l, int r) {
        if (lazy[node] == 0) return;
        int mid = (l + r) >> 1;
        int left_size = mid - l + 1;
        int right_size = r - mid;
        // 传递给子节点
        tree[node * 2] += lazy[node] * left_size;
        tree[node * 2 + 1] += lazy[node] * right_size;
        lazy[node * 2] += lazy[node];
        lazy[node * 2 + 1] += lazy[node];
        lazy[node] = 0;
    }

    // 区间 [ql, qr] 加上 val
    void range_add(int node, int l, int r, int ql, int qr, ll val) {
        if (qr < l || r < ql) return;
        if (ql <= l && r <= qr) {
            tree[node] += val * (r - l + 1);
            lazy[node] += val;
            return;
        }
        push_down(node, l, r);
        int mid = (l + r) >> 1;
        range_add(node * 2, l, mid, ql, qr, val);
        range_add(node * 2 + 1, mid + 1, r, ql, qr, val);
        tree[node] = tree[node * 2] + tree[node * 2 + 1];
    }

    // 查询区间 [ql, qr] 的和
    ll range_query(int node, int l, int r, int ql, int qr) {
        if (qr < l || r < ql) return 0;
        if (ql <= l && r <= qr) return tree[node];
        push_down(node, l, r);
        int mid = (l + r) >> 1;
        return range_query(node * 2, l, mid, ql, qr)
             + range_query(node * 2 + 1, mid + 1, r, ql, qr);
    }

    // 查找第 k 个 1（值域线段树用法，这里演示"前缀和定位"：
    // 返回最小的 index 使得前缀和 >= k，用于演示线段树的二分能力）
    int kth(int node, int l, int r, ll k) {
        if (l == r) return l;
        push_down(node, l, r);
        int mid = (l + r) >> 1;
        ll left_sum = tree[node * 2];
        if (k <= left_sum)
            return kth(node * 2, l, mid, k);
        else
            return kth(node * 2 + 1, mid + 1, r, k - left_sum);
    }

    // 公共接口
    void range_add(int ql, int qr, ll val) { range_add(1, 0, n - 1, ql, qr, val); }
    ll range_query(int ql, int qr) { return range_query(1, 0, n - 1, ql, qr); }
    int kth(ll k) { return kth(1, 0, n - 1, k); }
};

// ---------- 暴力基准 ----------
struct Brute {
    vector<ll> a;
    Brute(const vector<ll>& arr) : a(arr) {}
    void range_add(int ql, int qr, ll val) {
        for (int i = ql; i <= qr; i++) a[i] += val;
    }
    ll range_query(int ql, int qr) {
        ll s = 0;
        for (int i = ql; i <= qr; i++) s += a[i];
        return s;
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(12345); // 固定种子，可复现

    const int N = 200000;
    const int Q = 200000;

    // 初始化随机数组
    vector<ll> init(N);
    for (int i = 0; i < N; i++) init[i] = rng() % 100;

    SegTree st(init);
    Brute   br(init);

    // ---- 正确性对拍 ----
    int ops = Q;
    int correct_cnt = 0;
    int total_checks = 0;

    for (int i = 0; i < ops; i++) {
        int type = rng() % 3;
        int l = rng() % N;
        int r = rng() % N;
        if (l > r) swap(l, r);

        if (type == 0) { // 区间更新
            ll val = (ll)(rng() % 2001) - 1000; // [-1000, 1000]
            st.range_add(l, r, val);
            br.range_add(l, r, val);
        } else if (type == 1) { // 区间查询
            ll a = st.range_query(l, r);
            ll b = br.range_query(l, r);
            total_checks++;
            if (a == b) correct_cnt++;
            else {
                cerr << "MISMATCH query [" << l << "," << r << "] st=" << a << " brute=" << b << "\n";
            }
        } else { // 单点查询（作为额外检查）
            ll a = st.range_query(l, l);
            ll b = br.range_query(l, l);
            total_checks++;
            if (a == b) correct_cnt++;
        }
    }

    // 全区间和最终校验
    ll full_st = st.range_query(0, N - 1);
    ll full_br = br.range_query(0, N - 1);
    total_checks++;
    if (full_st == full_br) correct_cnt++;

    cout << "=== Segment Tree 量化验证 ===" << endl;
    cout << "元素数量 N: " << N << endl;
    cout << "操作数量 Q: " << Q << endl;
    cout << "对拍检查次数: " << total_checks << endl;
    cout << "正确次数: " << correct_cnt << endl;
    printf("正确率: %.6f %%\n", 100.0 * correct_cnt / total_checks);
    cout << "全区间和 (segment tree): " << full_st << endl;
    cout << "全区间和 (brute force) : " << full_br << endl;
    cout << (full_st == full_br ? "✅ 全区间和一致" : "❌ 全区间和不一致") << endl;

    // ---- 加速比基准 ----
    // 重新构造干净数据做纯查询+更新性能对比
    vector<ll> init2(N);
    for (int i = 0; i < N; i++) init2[i] = rng() % 100;
    SegTree st2(init2);
    Brute   br2(init2);

    ll total_query_sum = 0; // 防止编译器优化掉

    // Segment Tree 计时
    auto t0 = chrono::high_resolution_clock::now();
    for (int i = 0; i < Q; i++) {
        int l = rng() % N, r = rng() % N;
        if (l > r) swap(l, r);
        if (i % 3 == 0) st2.range_add(l, r, 5);
        else total_query_sum += st2.range_query(l, r);
    }
    auto t1 = chrono::high_resolution_clock::now();
    double st_time = chrono::duration<double>(t1 - t0).count();

    // Brute Force 计时（用较小规模避免过慢）
    int BQ = 5000;
    auto t2 = chrono::high_resolution_clock::now();
    for (int i = 0; i < BQ; i++) {
        int l = rng() % N, r = rng() % N;
        if (l > r) swap(l, r);
        if (i % 3 == 0) br2.range_add(l, r, 5);
        else total_query_sum += br2.range_query(l, r);
    }
    auto t3 = chrono::high_resolution_clock::now();
    double br_time = chrono::duration<double>(t3 - t2).count();
    // 换算到 Q 次操作的等价比
    double br_expect_full = br_time / BQ * Q;

    cout << "=== 性能基准 ===" << endl;
    cout << "Segment Tree " << Q << " 次混合操作耗时: " << fixed << setprecision(4) << st_time << " s" << endl;
    cout << "Brute Force " << BQ << " 次混合操作耗时: " << br_time << " s" << endl;
    cout << "Brute Force 预估 " << Q << " 次操作耗时: " << br_expect_full << " s" << endl;
    cout << "加速比: " << fixed << setprecision(2) << (br_expect_full / st_time) << "x" << endl;
    cout << "校验用累计和(防优化): " << total_query_sum << endl;

    return 0;
}
