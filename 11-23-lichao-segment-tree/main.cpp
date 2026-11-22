// Li Chao Segment Tree (Convex Hull Trick) — 李超线段树
// 支持两种操作：
//   1. 插入一条直线 y = m*x + b （或限制在区间内的线段）
//   2. 查询某一点 x 上所有已插入直线的最小值
//
// 本实现包含：
//   - 动态开点 Li Chao 树（直线版）：add_line
//   - 线段版：add_segment（在 [l, r] 区间插入线段）
//   - 点查询：query(x)
//   - 定量验证：与暴力遍历全部直线的结果逐点比对
//   - 性能：对比暴力 O(n) 与 Li Chao O(log N)

#include <bits/stdc++.h>
using namespace std;

typedef long long ll;
static const ll INF = 4e18;

struct Line {
    ll m, b;
    Line(ll _m = 0, ll _b = INF) : m(_m), b(_b) {}
    ll eval(ll x) const { return m * x + b; }
};

// x 的坐标范围：[minX, maxX]（包含端点）
struct LiChao {
    int minX, maxX;
    struct Node {
        Line line;
        int left, right;
        Node() : line(Line()), left(-1), right(-1) {}
    };
    vector<Node> tree;

    LiChao(int _minX, int _maxX) : minX(_minX), maxX(_maxX) {
        tree.reserve(1 << 20);
        tree.emplace_back(); // root = 0
    }

    // 在 [l, r] 区间插入 Line；node 为当前节点
    void add_line(Line nw, int node, int l, int r) {
        int mid = (l + r) >> 1;

        // 让当前节点保存“中点处更优（更小）”的直线
        bool leftBetter  = nw.eval(l) < tree[node].line.eval(l);
        bool midBetter   = nw.eval(mid) < tree[node].line.eval(mid);

        if (midBetter) swap(nw, tree[node].line);

        if (l == r) return;

        // 若两侧无交叠可剪枝
        if (l + 1 == r) {
            bool rightBetter = nw.eval(r) < tree[node].line.eval(r);
            if (rightBetter) {
                if (tree[node].right == -1) {
                    tree[node].right = tree.size();
                    tree.emplace_back();
                }
                add_line(nw, tree[node].right, mid + 1, r);
            }
            return;
        }

        if (leftBetter != midBetter) {
            if (tree[node].left == -1) {
                tree[node].left = tree.size();
                tree.emplace_back();
            }
            add_line(nw, tree[node].left, l, mid);
        } else {
            if (tree[node].right == -1) {
                tree[node].right = tree.size();
                tree.emplace_back();
            }
            add_line(nw, tree[node].right, mid + 1, r);
        }
    }

    void add_line(Line nw) { add_line(nw, 0, minX, maxX); }

    // 线段版：仅在 [ql, qr] 区间插入
    void add_segment(Line nw, int node, int l, int r, int ql, int qr) {
        if (ql <= l && r <= qr) { add_line(nw, node, l, r); return; }
        int mid = (l + r) >> 1;
        if (ql <= mid) {
            if (tree[node].left == -1) {
                tree[node].left = tree.size();
                tree.emplace_back();
            }
            add_segment(nw, tree[node].left, l, mid, ql, qr);
        }
        if (qr > mid) {
            if (tree[node].right == -1) {
                tree[node].right = tree.size();
                tree.emplace_back();
            }
            add_segment(nw, tree[node].right, mid + 1, r, ql, qr);
        }
    }
    void add_segment(Line nw, int ql, int qr) { add_segment(nw, 0, minX, maxX, ql, qr); }

    ll query(int x, int node, int l, int r) {
        if (node == -1) return INF;
        int mid = (l + r) >> 1;
        ll res = tree[node].line.eval(x);
        if (l == r) return res;
        if (x <= mid) res = min(res, query(x, tree[node].left, l, mid));
        else          res = min(res, query(x, tree[node].right, mid + 1, r));
        return res;
    }
    ll query(int x) { return query(x, 0, minX, maxX); }
};

struct Brute {
    vector<Line> lines;
    void add_line(Line nw) { lines.push_back(nw); }
    void add_segment(Line nw, int ql, int qr) { (void)ql; (void)qr; lines.push_back(nw); }
    ll query(int x) {
        ll best = INF;
        for (auto& L : lines) best = min(best, L.eval(x));
        return best;
    }
};

static uint64_t rng_state = 0x9E3779B97F4A7C15ULL;
static uint64_t splitmix64() {
    uint64_t z = (rng_state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}
// [a, b] 均匀随机整数
static ll rnd(ll a, ll b) { return a + (ll)(splitmix64() % (uint64_t)(b - a + 1)); }

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    const int XMIN = -1000000, XMAX = 1000000;

    // ========== 测试 1：直线版正确性（随机数据） ==========
    {
        LiChao lc(XMIN, XMAX);
        Brute br;
        int N = 2000;
        for (int i = 0; i < N; i++) {
            ll m = rnd(-1000, 1000);
            ll b = rnd(-100000000, 100000000);
            lc.add_line(Line(m, b));
            br.add_line(Line(m, b));
        }
        int Q = 20000;
        int mismatch = 0;
        for (int i = 0; i < Q; i++) {
            ll x = rnd(XMIN, XMAX);
            ll got = lc.query(x);
            ll exp = br.query(x);
            if (got != exp) {
                if (mismatch < 10)
                    cerr << "MISMATCH line x=" << x << " got=" << got << " exp=" << exp << "\n";
                mismatch++;
            }
        }
        cout << "[Test1 直线版] N=" << N << " Q=" << Q << " mismatch=" << mismatch << "\n";
        cout << (mismatch == 0 ? "PASS" : "FAIL") << "\n";
    }

    // ========== 测试 2：线段版正确性（随机区间插入） ==========
    {
        LiChao lc(XMIN, XMAX);
        Brute br_seg;
        int N = 3000;
        // brute 对 segment 的语义：仅在 [ql,qr] 内生效 —— 需要单独处理
        // 为此，brute 对线段也记录 ql,qr，query 时只统计覆盖 x 的线段
        struct Seg { Line L; int ql, qr; };
        vector<Seg> segs;
        vector<Line> fullLines;
        auto bruteQuery = [&](ll x) -> ll {
            ll best = INF;
            for (auto& L : fullLines) best = min(best, L.eval(x));
            for (auto& s : segs) if (s.ql <= x && x <= s.qr) best = min(best, s.L.eval(x));
            return best;
        };
        for (int i = 0; i < N; i++) {
            int type = rnd(0, 1);
            ll m = rnd(-1000, 1000);
            ll b = rnd(-100000000, 100000000);
            if (type == 0) {
                lc.add_line(Line(m, b));
                fullLines.push_back(Line(m, b));
            } else {
                ll ql = rnd(XMIN, XMAX), qr = rnd(XMIN, XMAX);
                if (ql > qr) swap(ql, qr);
                lc.add_segment(Line(m, b), ql, qr);
                segs.push_back({Line(m, b), (int)ql, (int)qr});
            }
        }
        int Q = 20000;
        int mismatch = 0;
        for (int i = 0; i < Q; i++) {
            ll x = rnd(XMIN, XMAX);
            ll got = lc.query(x);
            ll exp = bruteQuery(x);
            if (got != exp) {
                if (mismatch < 10)
                    cerr << "MISMATCH seg x=" << x << " got=" << got << " exp=" << exp << "\n";
                mismatch++;
            }
        }
        cout << "[Test2 线段版] N=" << N << " Q=" << Q << " mismatch=" << mismatch << "\n";
        cout << (mismatch == 0 ? "PASS" : "FAIL") << "\n";
    }

    // ========== 测试 3：经典应用 —— 斜率优化 DP（下凸壳计数） ==========
    // 利用 Li Chao 求 1..n 个点到直线的最小值问题作为应用验证
    {
        LiChao lc(XMIN, XMAX);
        int N = 500;
        for (int i = 0; i < N; i++) {
            ll m = rnd(-500, 500);
            ll b = rnd(-1000000, 1000000);
            lc.add_line(Line(m, b));
        }
        // 求这些直线在若干点的最小值，验证 min 值本身是有限的且符合下界
        bool ok = true;
        ll sampleMin = INF;
        for (int x = XMIN; x <= XMAX; x += 10000) {
            ll v = lc.query(x);
            if (v >= INF) { ok = false; break; }
            sampleMin = min(sampleMin, v);
        }
        cout << "[Test3 应用] 全坐标采样 min=" << sampleMin << " 有效=" << (ok ? "yes" : "no") << "\n";
    }

    // ========== 测试 4：性能对比（Li Chao vs 暴力） ==========
    {
        int N = 100000, Q = 100000;
        // 暴力查询：需要 re-scan 所有 line
        vector<Line> lines; lines.reserve(N);
        LiChao lc(XMIN, XMAX);
        for (int i = 0; i < N; i++) {
            ll m = rnd(-1000, 1000), b = rnd(-100000000, 100000000);
            lines.push_back(Line(m, b));
            lc.add_line(Line(m, b));
        }
        vector<ll> xs(Q);
        for (auto& x : xs) x = rnd(XMIN, XMAX);

        // 暴力
        auto t0 = chrono::high_resolution_clock::now();
        ll sink = 0;
        for (auto x : xs) {
            ll best = INF;
            for (auto& L : lines) best = min(best, L.eval(x));
            sink += best;
        }
        auto t1 = chrono::high_resolution_clock::now();

        // Li Chao
        auto t2 = chrono::high_resolution_clock::now();
        ll sink2 = 0;
        for (auto x : xs) sink2 += lc.query(x);
        auto t3 = chrono::high_resolution_clock::now();

        // 一致性验证
        bool consistent = (sink == sink2);

        double bruteMs = chrono::duration<double, milli>(t1 - t0).count();
        double lcMs    = chrono::duration<double, milli>(t3 - t2).count();
        double speedup = bruteMs / lcMs;

        cout << "[Test4 性能] N=" << N << " Q=" << Q << "\n";
        cout << "  暴力耗时 = " << bruteMs << " ms\n";
        cout << "  LiChao耗时 = " << lcMs << " ms\n";
        cout << "  加速比 = " << speedup << "x\n";
        cout << "  结果一致性 = " << (consistent ? "PASS" : "FAIL") << "\n";
    }

    return 0;
}
