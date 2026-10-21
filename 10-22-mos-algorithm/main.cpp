#include <bits/stdc++.h>
using namespace std;

// Mo's Algorithm -- offline range query processing with sqrt decomposition.
//
// Problem: given array a[0..n-1], answer q "sum of absolute frequencies"
// queries of the form [l, r) -> sum over distinct values v of (cnt[v])^2.
// (The "power of array" problem, Codeforces 86D.) This metric lets us
// add/remove elements in O(1) and gives an objective, verifiable number.
//
// Mo orders the queries so that the two pointers l, r move a total of
// O((n+q) * sqrt(n)) steps, instead of O(q*n) for the naive re-scan.

using ll = long long;

static int n, q;
static vector<int> a;
static vector<int> freq;
static ll cur = 0;

static inline void add(int idx) {
    int v = a[idx];
    cur -= 1LL * freq[v] * freq[v];
    freq[v]++;
    cur += 1LL * freq[v] * freq[v];
}

static inline void remove(int idx) {
    int v = a[idx];
    cur -= 1LL * freq[v] * freq[v];
    freq[v]--;
    cur += 1LL * freq[v] * freq[v];
}

struct Query {
    int l, r, idx;
};

// Naive baseline: re-count whole [l, r) for every query in O(len).
static ll naive(const vector<int>& arr, int l, int r) {
    unordered_map<int,int> c;
    c.reserve(r - l);
    for (int i = l; i < r; ++i) c[arr[i]]++;
    ll s = 0;
    for (auto& kv : c) s += 1LL * kv.second * kv.second;
    return s;
}

int main() {
    mt19937 rng(12345);
    n = 200000;
    q = 100000;
    const int VAL_RANGE = 1000;

    a.resize(n);
    for (int i = 0; i < n; ++i) a[i] = (int)(rng() % VAL_RANGE);

    // Generate random queries [l, r) with r > l.
    vector<Query> queries(q);
    vector<pair<int,int>> qpairs(q);
    for (int i = 0; i < q; ++i) {
        int l = (int)(rng() % n);
        int r = (int)(rng() % n);
        if (l > r) swap(l, r);
        if (l == r) r = min(n, l + 1);
        queries[i] = {l, r, i};
        qpairs[i] = {l, r};
    }

    // ---- Naive baseline (sample a subset to keep runtime reasonable) ----
    int naiveSubset = min(q, 2000);
    vector<ll> naiveAns(naiveSubset);
    auto t0 = chrono::high_resolution_clock::now();
    for (int i = 0; i < naiveSubset; ++i) {
        naiveAns[i] = naive(a, qpairs[i].first, qpairs[i].second);
    }
    auto t1 = chrono::high_resolution_clock::now();
    double naiveMs = chrono::duration<double, milli>(t1 - t0).count();
    double naivePerQuery = naiveMs / naiveSubset;
    double naiveFullEst = naivePerQuery * q; // projected total for all q

    // ---- Mo's Algorithm ----
    int block = max(1, (int)sqrt(n));
    auto cmp = [&](const Query& x, const Query& y) {
        int bx = x.l / block, by = y.l / block;
        if (bx != by) return bx < by;
        return (bx & 1) ? (x.r > y.r) : (x.r < y.r); // alternating direction
    };
    sort(queries.begin(), queries.end(), cmp);

    freq.assign(VAL_RANGE, 0);
    cur = 0;
    vector<ll> moAns(q);

    auto t2 = chrono::high_resolution_clock::now();
    int curL = 0, curR = 0;
    for (auto& qry : queries) {
        // extend / shrink to [qry.l, qry.r)
        while (curL > qry.l) add(--curL);
        while (curR < qry.r) add(curR++);
        while (curL < qry.l) remove(curL++);
        while (curR > qry.r) remove(--curR);
        moAns[qry.idx] = cur;
    }
    auto t3 = chrono::high_resolution_clock::now();
    double moMs = chrono::duration<double, milli>(t3 - t2).count();

    // ---- Correctness: compare Mo vs naive on the sampled subset ----
    bool allMatch = true;
    int mismatches = 0;
    for (int i = 0; i < naiveSubset; ++i) {
        if (moAns[i] != naiveAns[i]) {
            allMatch = false;
            ++mismatches;
        }
    }

    // ---- Aggregate numeric verification ----
    ll moSum = 0;
    for (ll v : moAns) moSum += v;

    cout << fixed << setprecision(3);
    cout << "=== Mo's Algorithm (Power-of-Array Range Queries) ===\n";
    cout << "n = " << n << ", q = " << q << ", value_range = " << VAL_RANGE << ", block = " << block << "\n";
    cout << "naive_subset = " << naiveSubset << "\n";
    cout << "naive_time_ms = " << naiveMs << " (subset) -> projected full " << naiveFullEst << " ms\n";
    cout << "mo_time_ms = " << moMs << "\n";
    cout << "speedup_vs_projected_naive = " << (naiveFullEst / max(moMs, 1e-9)) << "x\n";
    cout << "correctness_matches = " << (naiveSubset - mismatches) << " / " << naiveSubset
         << (allMatch ? " (ALL MATCH)" : " (MISMATCH!)") << "\n";
    cout << "mo_total_answer_sum = " << moSum << "\n";

    bool pass = allMatch && (mismatches == 0);
    cout << "VERDICT: " << (pass ? "PASS" : "FAIL") << "\n";
    return pass ? 0 : 1;
}
