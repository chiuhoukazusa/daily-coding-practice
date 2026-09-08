#include <bits/stdc++.h>
using namespace std;

// ============ Prefix Doubling Suffix Array (O(n log n)) ============
// Builds suffix array of string s (0-indexed). Returns vector<int> where
// sa[i] = starting index of i-th lexicographically smallest suffix.
vector<int> build_suffix_array(const string& s) {
    int n = (int)s.size();
    vector<int> sa(n), rnk(n), tmp(n);

    // Initial sort by first character using counting sort over character values (0..255)
    int maxv = 256;
    vector<int> cnt(max(maxv, n), 0);
    for (int i = 0; i < n; i++) cnt[(unsigned char)s[i]]++;
    for (int i = 1; i < maxv; i++) cnt[i] += cnt[i - 1];
    for (int i = n - 1; i >= 0; i--) sa[--cnt[(unsigned char)s[i]]] = i;

    rnk[sa[0]] = 0;
    int classes = 1;
    for (int i = 1; i < n; i++) {
        if (s[sa[i]] != s[sa[i - 1]]) classes++;
        rnk[sa[i]] = classes - 1;
    }

    // Doubling: sort by pair (rnk[i], rnk[i + k])
    for (int k = 1; k < n; k <<= 1) {
        // Step 1: build positions sorted by second key (i + k)
        // Iterate suffixes in increasing order of i+k, i.e., from n-k..n-1 (their +k is past end = rank -1)
        vector<int> pn(n);
        int cur = 0;
        // suffixes with i+k >= n have second key = -1 (smallest)
        for (int i = n - k; i < n; i++) pn[cur++] = i;
        // remaining suffixes ordered by current sa (which is sorted by rank, hence +k rank order)
        for (int i = 0; i < n; i++) {
            if (sa[i] >= k) pn[cur++] = sa[i] - k;
        }
        // Step 2: counting sort pn by first key rnk (stable)
        fill(cnt.begin(), cnt.begin() + classes, 0);
        for (int i = 0; i < n; i++) cnt[rnk[pn[i]]]++;
        for (int i = 1; i < classes; i++) cnt[i] += cnt[i - 1];
        for (int i = n - 1; i >= 0; i--) sa[--cnt[rnk[pn[i]]]] = pn[i];

        // Step 3: recompute ranks by comparing adjacent pairs
        vector<int> newRnk(n);
        newRnk[sa[0]] = 0;
        int nc = 1;
        for (int i = 1; i < n; i++) {
            int prev = sa[i - 1], now = sa[i];
            bool diff = (rnk[prev] != rnk[now]) ||
                        ((prev + k < n ? rnk[prev + k] : -1) != (now + k < n ? rnk[now + k] : -1));
            if (diff) nc++;
            newRnk[now] = nc - 1;
        }
        rnk = newRnk;
        classes = nc;
        if (classes == n) break; // fully sorted
    }
    return sa;
}

// ============ Kasai LCP (O(n)) ============
// Builds LCP array where lcp[i] = LCP(sa[i], sa[i+1]) for i in [0, n-2].
vector<int> build_lcp(const string& s, const vector<int>& sa) {
    int n = (int)s.size();
    vector<int> rank(n), lcp(n - 1, 0);
    for (int i = 0; i < n; i++) rank[sa[i]] = i;
    int h = 0;
    for (int i = 0; i < n; i++) {
        int r = rank[i];
        if (r == n - 1) { h = 0; continue; }
        int j = sa[r + 1];
        while (i + h < n && j + h < n && s[i + h] == s[j + h]) h++;
        lcp[r] = h;
        if (h > 0) h--;
    }
    return lcp;
}

// ============ Brute-force baseline (O(n^2 log n)) ============
vector<int> brute_suffix_array(const string& s) {
    int n = (int)s.size();
    vector<int> sa(n);
    iota(sa.begin(), sa.end(), 0);
    sort(sa.begin(), sa.end(), [&](int a, int b) {
        return s.compare(a, n - a, s, b, n - b) < 0;
    });
    return sa;
}

vector<int> brute_lcp(const string& s, const vector<int>& sa) {
    int n = (int)s.size();
    vector<int> lcp(n - 1, 0);
    auto lcp_of = [&](int a, int b) {
        int h = 0;
        while (a + h < n && b + h < n && s[a + h] == s[b + h]) h++;
        return h;
    };
    for (int i = 0; i < n - 1; i++) lcp[i] = lcp_of(sa[i], sa[i + 1]);
    return lcp;
}

// random string generator over given alphabet
string random_string(int n, int alpha, mt19937& rng) {
    uniform_int_distribution<int> dist(0, alpha - 1);
    string s(n, 'a');
    for (int i = 0; i < n; i++) s[i] = (char)('a' + dist(rng));
    return s;
}

int main() {
    mt19937 rng(12345);

    // ---- Part 1: correctness across varied inputs ----
    vector<pair<string,int>> tests = {
        {"banana", 1},
        {"abracadabra", 1},
        {"aaaaa", 1},
        {"a", 1},
        {"abababab", 1},
        {"mississippi", 1},
    };
    bool all_ok = true;
    int total_corr_checks = 0;

    // small random strings (tiny alphabet -> many repeated suffixes, stress LCP)
    for (int t = 0; t < 200; t++) {
        int n = 1 + (int)(rng() % 40);
        int alpha = 2 + (int)(rng() % 5); // 2..6 distinct chars
        tests.push_back({random_string(n, alpha, rng), 0});
    }
    // medium random strings (larger alphabet)
    for (int t = 0; t < 20; t++) {
        int n = 100 + (int)(rng() % 400);
        int alpha = 20 + (int)(rng() % 100);
        tests.push_back({random_string(n, alpha, rng), 0});
    }

    for (auto& [s, _] : tests) {
        auto sa_fast = build_suffix_array(s);
        auto sa_brut = brute_suffix_array(s);
        if (sa_fast != sa_brut) {
            all_ok = false;
            printf("❌ SA MISMATCH on string len=%d\n", (int)s.size());
            break;
        }
        auto lcp_fast = build_lcp(s, sa_fast);
        auto lcp_brut = brute_lcp(s, sa_brut);
        if (lcp_fast != lcp_brut) {
            all_ok = false;
            printf("❌ LCP MISMATCH on string len=%d\n", (int)s.size());
            break;
        }
        total_corr_checks++;
    }
    printf("Correctness: %s (%d strings checked, SA & LCP all match brute force)\n",
           all_ok ? "PASS ✅" : "FAIL ❌", total_corr_checks);

    // ---- Part 2: LCP bound verification (Kasai invariant) ----
    // For every pair of adjacent suffixes, lcp[i] <= min(len(sa[i]), len(sa[i+1]))
    // and total sum of LCP = sum of LCP over all adjacent pairs (already true by def).
    // We verify: 0 <= lcp[i] <= n - max(sa[i], sa[i+1]).
    string big = random_string(2000, 26, rng);
    auto sa_big = build_suffix_array(big);
    auto lcp_big = build_lcp(big, sa_big);
    int n_big = (int)big.size();
    bool bound_ok = true;
    int lcp_sum = 0;
    for (int i = 0; i < n_big - 1; i++) {
        int remaining = n_big - max(sa_big[i], sa_big[i + 1]);
        if (lcp_big[i] < 0 || lcp_big[i] > remaining) { bound_ok = false; break; }
        lcp_sum += lcp_big[i];
    }
    printf("LCP bounds: %s (sum=%d over %d adj pairs, each within [0, remaining-suffix-len])\n",
           bound_ok ? "PASS ✅" : "FAIL ❌", lcp_sum, n_big - 1);

    // ---- Part 3: performance / speedup vs brute force ----
    auto time_ms = [](auto&& fn) {
        auto t0 = chrono::high_resolution_clock::now();
        fn();
        auto t1 = chrono::high_resolution_clock::now();
        return chrono::duration<double, milli>(t1 - t0).count();
    };

    int perf_n = 5000;
    string perf_s = random_string(perf_n, 26, rng);
    vector<int> sa_fast, sa_brut;

    double t_fast = time_ms([&] { sa_fast = build_suffix_array(perf_s); });
    double t_brut = time_ms([&] { sa_brut = brute_suffix_array(perf_s); });
    double speedup = t_brut / t_fast;

    printf("\nPerformance (n=%d, alphabet=26):\n", perf_n);
    printf("  brute force:  %.2f ms\n", t_brut);
    printf("  prefix doubling: %.2f ms\n", t_fast);
    printf("  speedup:      %.2fx\n", speedup);
    printf("  SA equality:  %s\n", sa_fast == sa_brut ? "PASS ✅" : "FAIL ❌");

    // ---- Part 4: LCP useful application — count distinct substrings ----
    // number of distinct substrings = n*(n+1)/2 - sum(lcp)
    long long total_sub = 1LL * n_big * (n_big + 1) / 2;
    long long distinct = total_sub - lcp_sum;
    long long brute_distinct = 0;
    {
        set<string> subs;
        for (int i = 0; i < n_big; i++) {
            string cur;
            for (int j = i; j < n_big; j++) {
                cur += big[j];
                subs.insert(cur);
            }
        }
        brute_distinct = subs.size();
    }
    printf("\nDistinct substrings: SA/LCP=%lld, brute count=%lld  %s\n",
           distinct, brute_distinct,
           distinct == brute_distinct ? "MATCH ✅" : "MISMATCH ❌");

    // ---- summary ----
    printf("\n=== SUMMARY ===\n");
    printf("correctness_checks=%d\n", total_corr_checks);
    printf("lcp_bound_ok=%s\n", bound_ok ? "true" : "false");
    printf("speedup=%.2fx\n", speedup);
    printf("distinct_substring_correct=%s\n", distinct == brute_distinct ? "true" : "false");

    return all_ok && bound_ok && (sa_fast == sa_brut) && (distinct == brute_distinct) ? 0 : 1;
}
