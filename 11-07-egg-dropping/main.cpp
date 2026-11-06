#include <bits/stdc++.h>
using namespace std;

// Egg Dropping Problem: given k eggs and n floors, find minimum number of
// drops (trials) in the worst case to determine the critical floor F such that
// an egg breaks if dropped from floor >= F, and does not break below F.

// O(n*k^2) classic DP (naive baseline)
static int eggDropNaive(int k, int n) {
    // dp[e][f] = min trials with e eggs and f floors
    vector<vector<int>> dp(k+1, vector<int>(n+1, 0));
    // 1 egg: need to try all floors one by one
    for (int f = 1; f <= n; ++f) dp[1][f] = f;
    // 0 floors: 0 trials
    for (int e = 1; e <= k; ++e) dp[e][0] = 0;
    for (int e = 2; e <= k; ++e) {
        for (int f = 1; f <= n; ++f) {
            dp[e][f] = INT_MAX;
            for (int x = 1; x <= f; ++x) {
                int res = 1 + max(dp[e-1][x-1], dp[e][f-x]); // break / not break
                dp[e][f] = min(dp[e][f], res);
            }
        }
    }
    return dp[k][n];
}

// O(n*k) DP using monotonicity of break/not-break (optimized)
static int eggDropOptimized(int k, int n) {
    // dp[e][f] = min trials; optimize the inner x loop via the fact that
    // dp[e-1][x-1] increases with x and dp[e][f-x] decreases with x
    vector<vector<int>> dp(k+1, vector<int>(n+1, 0));
    for (int f = 1; f <= n; ++f) dp[1][f] = f;
    for (int e = 2; e <= k; ++e) {
        int x = 1; // optimal split point is monotonic in f
        for (int f = 1; f <= n; ++f) {
            while (x < f && dp[e-1][x-1] < dp[e][f-x]) ++x;
            dp[e][f] = 1 + dp[e-1][x-1];
        }
    }
    return dp[k][n];
}

// O(k*logn) "number of floors with m moves" DP (optimal known solution)
static int eggDropLog(int k, int n) {
    // dp[m][e] = max floors testable with m moves and e eggs
    // recurrence: dp[m][e] = dp[m-1][e-1] + dp[m-1][e] + 1
    vector<vector<long long>> dp(n+1, vector<long long>(k+1, 0));
    for (int m = 1; m <= n; ++m) {
        for (int e = 1; e <= k; ++e) {
            dp[m][e] = dp[m-1][e-1] + dp[m-1][e] + 1;
        }
        if (dp[m][k] >= n) return m; // first m where we can cover n floors
    }
    return n;
}

// Analytical lower bound: log2(n+1) for infinite eggs
static long long ceilLog2(long long n) {
    long long v = n + 1, l = 0;
    while ((1LL << l) < v) ++l;
    return l;
}

bool checkConsistent(int k, int n) {
    long long a = eggDropOptimized(k, n);
    long long b = eggDropLog(k, n);
    long long c = eggDropNaive(k, n);
    return a == b && b == c;
}

int main() {
    cout << fixed << setprecision(0);
    // Test cases: (eggs, floors)
    vector<pair<int,int>> tests = {
        {1, 100}, {2, 10}, {2, 36}, {2, 100}, {3, 14},
        {3, 100}, {4, 50}, {5, 100}, {10, 100}, {100, 100},
        {3, 200}, {2, 200}, {8, 500}, {2, 1000}, {3, 1000}
    };

    // Famous result: 2 eggs 100 floors -> 14 trials; 2 eggs 36 floors -> 8
    map<pair<int,int>,int> known = {
        {{2,100},14}, {{2,36},8}, {{1,100},100}
    };

    int passed = 0, total = 0;
    for (auto &tc : tests) {
        int k = tc.first, n = tc.second;
        int naive = eggDropNaive(k, n);
        int opt   = eggDropOptimized(k, n);
        int lo    = eggDropLog(k, n);
        long long lb = ceilLog2(n);

        total++;
        bool ok = (naive == opt) && (opt == lo);
        if (ok) passed++;
        // known reference check
        bool knownOk = true;
        if (known.count({k,n})) knownOk = (known[{k,n}] == opt);

        cout << "k=" << setw(3) << k << " n=" << setw(4) << n
             << "  naive=" << setw(4) << naive
             << "  opt=" << setw(4) << opt
             << "  logDP=" << setw(4) << lo
             << "  ceilLog2=" << setw(3) << lb
             << "  " << (ok ? "OK" : "MISMATCH")
             << (knownOk ? "" : "  [KNOWN-REF-FAIL]")
             << "\n";
    }
    cout << "\n一致性验证: " << passed << "/" << total << " 通过"
         << (passed==total ? " ✅" : " ❌") << "\n";

    // Known references explicitly
    cout << "\n参考值校验:\n";
    for (auto &kv : known) {
        int v = eggDropOptimized(kv.first.first, kv.first.second);
        cout << "  k=" << kv.first.first << " n=" << kv.first.second
             << " 期望=" << kv.second << " 实际=" << v
             << (v==kv.second ? " ✅" : " ❌") << "\n";
    }

    // Performance comparison on large input (naive vs optimized)
    // Skip naive for huge n to avoid O(n*k^2) too slow
    int kL = 50, nL = 2000;
    cout << "\n性能 (k=50, n=2000):\n";
    cout << "  二分DP O(k*logn)  = " << eggDropLog(kL, nL) << " 次\n";
    cout << "  单调DP O(n*k)     = " << eggDropOptimized(kL, nL) << " 次\n";
    // naive O(n*k^2) would be ~50*2000^2 = 2e8, run it once for a timing sample on smaller
    auto t0 = chrono::steady_clock::now();
    int naiveResult = eggDropNaive(10, 500);
    auto t1 = chrono::steady_clock::now();
    auto tNaive = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    t0 = chrono::steady_clock::now();
    int optResult = eggDropOptimized(10, 500);
    t1 = chrono::steady_clock::now();
    auto tOpt = chrono::duration_cast<chrono::milliseconds>(t1-t0).count();
    cout << "  naive(10,500)=" << naiveResult << " (" << tNaive << "ms)  "
         << "opt(10,500)=" << optResult << " (" << tOpt << "ms)\n";

    // Monotonicity sanity checks (quantitative invariants)
    cout << "\n不变量验证:\n";
    // 1) more eggs never needs more trials
    bool mono1 = true;
    for (int f = 1; f <= 60; ++f) {
        int prev = eggDropOptimized(1, f);
        for (int e = 2; e <= 20; ++e) {
            int cur = eggDropOptimized(e, f);
            if (cur > prev) { mono1 = false; }
            prev = cur;
        }
    }
    cout << "  [单调性1] 蛋数增加, 试验次数单调不增: " << (mono1?"✅":"❌") << "\n";
    // 2) more floors never needs fewer trials (for fixed eggs)
    bool mono2 = true;
    for (int e = 1; e <= 20; ++e) {
        int prev = eggDropOptimized(e, 0);
        for (int f = 1; f <= 100; ++f) {
            int cur = eggDropOptimized(e, f);
            if (cur < prev) mono2 = false;
            prev = cur;
        }
    }
    cout << "  [单调性2] 楼层增加, 试验次数单调不减: " << (mono2?"✅":"❌") << "\n";
    // 3) with >= ceil(log2(n+1)) eggs, trials == ceil(log2(n+1)) (binary search achieves lower bound)
    bool mono3 = true;
    for (int f = 1; f <= 500; ++f) {
        int withMany = eggDropLog(1000, f);
        if (withMany != ceilLog2(f)) mono3 = false;
    }
    cout << "  [最优下界] 蛋数充足时 == ceil(log2(n+1)) 二分下界: " << (mono3?"✅":"❌") << "\n";

    return 0;
}
