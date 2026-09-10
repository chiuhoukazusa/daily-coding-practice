#include <bits/stdc++.h>
using namespace std;

// Fenwick Tree (Binary Indexed Tree) — 1-indexed
// Supports: point update O(log n), prefix sum O(log n),
//           range sum O(log n), inversion count.
struct Fenwick {
    int n;
    vector<long long> bit;
    Fenwick(int n_) : n(n_), bit(n_ + 1, 0) {}

    void add(int idx, long long delta) {
        for (; idx <= n; idx += idx & (-idx))
            bit[idx] += delta;
    }

    // sum of [1..idx]
    long long prefix(int idx) const {
        long long s = 0;
        for (; idx > 0; idx -= idx & (-idx))
            s += bit[idx];
        return s;
    }

    // sum of [l..r] (1-indexed)
    long long range(int l, int r) const {
        return prefix(r) - prefix(l - 1);
    }
};

// Brute-force prefix array for correctness checking
struct Brute {
    vector<long long> a;
    Brute(int n) : a(n + 1, 0) {}
    void add(int idx, long long d) { a[idx] += d; }
    long long prefix(int idx) const {
        long long s = 0;
        for (int i = 1; i <= idx; ++i) s += a[i];
        return s;
    }
    long long range(int l, int r) const {
        long long s = 0;
        for (int i = l; i <= r; ++i) s += a[i];
        return s;
    }
};

// Inversion count via Fenwick (coordinate-compressed)
long long inversionCountBIT(const vector<int>& arr) {
    // coordinate compression
    vector<int> sorted = arr;
    sort(sorted.begin(), sorted.end());
    sorted.erase(unique(sorted.begin(), sorted.end()), sorted.end());
    int m = sorted.size();
    Fenwick fw(m);
    long long inv = 0;
    for (int i = (int)arr.size() - 1; i >= 0; --i) {
        int rank = lower_bound(sorted.begin(), sorted.end(), arr[i]) - sorted.begin() + 1;
        inv += fw.prefix(rank - 1);
        fw.add(rank, 1);
    }
    return inv;
}

// Brute inversion count O(n^2)
long long inversionCountBrute(const vector<int>& arr) {
    long long inv = 0;
    int n = arr.size();
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (arr[i] > arr[j]) ++inv;
    return inv;
}

int main() {
    // Use deterministic RNG for reproducibility
    mt19937 rng(12345);
    const int N = 200000;

    // ---- Test 1: correctness vs brute force on point updates + prefix/range queries ----
    const int N_SMALL = 5000;
    const int Q_SMALL = 20000;
    Fenwick fw(N_SMALL);
    Brute br(N_SMALL);
    long long mismatches = 0;
    for (int q = 0; q < Q_SMALL; ++q) {
        int op = rng() % 3;
        int idx = rng() % N_SMALL + 1;
        long long delta = (rng() % 2001) - 1000; // [-1000, 1000]
        if (op == 0) {
            fw.add(idx, delta);
            br.add(idx, delta);
        } else if (op == 1) {
            long long a = fw.prefix(idx);
            long long b = br.prefix(idx);
            if (a != b) mismatches++;
        } else {
            int l = rng() % N_SMALL + 1;
            int r = rng() % N_SMALL + 1;
            if (l > r) swap(l, r);
            long long a = fw.range(l, r);
            long long b = br.range(l, r);
            if (a != b) mismatches++;
        }
    }

    // ---- Test 2: inversion count correctness vs brute force ----
    vector<int> arr(N_SMALL);
    for (auto &x : arr) x = rng() % 100000;
    long long invBIT = inversionCountBIT(arr);
    long long invBrute = inversionCountBrute(arr);

    // ---- Test 3: performance benchmark ----
    // Build large array and measure
    vector<int> big(N);
    for (auto &x : big) x = rng();

    auto t0 = chrono::high_resolution_clock::now();
    long long invBigBIT = inversionCountBIT(big);
    auto t1 = chrono::high_resolution_clock::now();
    double msBIT = chrono::duration<double, milli>(t1 - t0).count();

    // Brute force on a smaller N to extrapolate / measure ratio directly
    const int NB = 20000;
    vector<int> bruteArr(big.begin(), big.begin() + NB);
    auto t2 = chrono::high_resolution_clock::now();
    long long invBigBrute = inversionCountBrute(bruteArr);
    auto t3 = chrono::high_resolution_clock::now();
    double msBrute = chrono::duration<double, milli>(t3 - t2).count();

    // ---- Output ----
    cout << fixed << setprecision(6);
    cout << "== Fenwick Tree (BIT) 验证报告 ==" << endl;
    cout << "Test1 点更新/前缀和/区间查询 失配数 = " << mismatches
         << " (应为 0)" << endl;
    cout << (mismatches == 0 ? "Test1 PASS" : "Test1 FAIL") << endl;

    cout << "Test2 逆序对 BIT=" << invBIT << " 暴力=" << invBrute
         << " (N=" << N_SMALL << ")" << endl;
    cout << (invBIT == invBrute ? "Test2 PASS" : "Test2 FAIL") << endl;

    cout << "Test3a BIT 逆序对  N=" << N << " 时间=" << msBIT << "ms 结果=" << invBigBIT << endl;
    cout << "Test3b 暴力逆序对 N=" << NB << " 时间=" << msBrute << "ms 结果=" << invBigBrute << endl;
    double speedup = msBrute / max(msBIT, 1e-9) * ((double)N / NB) * ((double)N / NB);
    cout << "Estimated speedup (extrapolated O(n^2) vs O(n log n)) = " << speedup << "x" << endl;

    return 0;
}
