#include <bits/stdc++.h>
using namespace std;

// Rabin-Karp with double rolling hash (base 911382323, mods 1e9+7 / 1e9+9)
// to eliminate collisions. Also implements single-hash variant to
// demonstrate collision handling.

static const long long MOD1 = 1000000007LL;
static const long long MOD2 = 1000000009LL;
static const long long BASE = 911382323LL;

struct PairHash {
    long long h1, h2;
    bool operator==(const PairHash &o) const { return h1 == o.h1 && h2 == o.h2; }
};

struct PairHashHasher {
    size_t operator()(const PairHash &p) const {
        return (size_t)(p.h1 * 1000000007ULL + p.h2);
    }
};

// Compute (base^exp) mod m
long long modpow(long long base, long long exp, long long m) {
    long long r = 1;
    base %= m;
    while (exp > 0) {
        if (exp & 1) r = r * base % m;
        base = base * base % m;
        exp >>= 1;
    }
    return r;
}

// Returns all starting indices where pattern occurs in text (0-based).
vector<int> rabin_karp_double(const string &text, const string &pat) {
    vector<int> res;
    int n = text.size(), m = pat.size();
    if (m == 0 || m > n) return res;

    long long ph1 = 0, ph2 = 0, th1 = 0, th2 = 0;
    for (int i = 0; i < m; i++) {
        ph1 = (ph1 * BASE + pat[i]) % MOD1;
        ph2 = (ph2 * BASE + pat[i]) % MOD2;
        th1 = (th1 * BASE + text[i]) % MOD1;
        th2 = (th2 * BASE + text[i]) % MOD2;
    }

    // Precompute BASE^(m-1) for rolling
    long long base_pow1 = modpow(BASE, m - 1, MOD1);
    long long base_pow2 = modpow(BASE, m - 1, MOD2);

    for (int i = 0; i <= n - m; i++) {
        if (ph1 == th1 && ph2 == th2) {
            // Verify to be 100% sure (although double hash collisions are astronomically rare)
            if (text.compare(i, m, pat) == 0)
                res.push_back(i);
        }
        if (i < n - m) {
            // Slide window: remove text[i], add text[i+m]
            th1 = ((th1 - text[i] * base_pow1 % MOD1 + MOD1) % MOD1) * BASE % MOD1;
            th1 = (th1 + text[i + m]) % MOD1;
            th2 = ((th2 - text[i] * base_pow2 % MOD2 + MOD2) % MOD2) * BASE % MOD2;
            th2 = (th2 + text[i + m]) % MOD2;
        }
    }
    return res;
}

// Single-hash variant (just MOD1) - may report false positives on hash collision
vector<int> rabin_karp_single(const string &text, const string &pat) {
    vector<int> res;
    int n = text.size(), m = pat.size();
    if (m == 0 || m > n) return res;

    long long ph = 0, th = 0;
    for (int i = 0; i < m; i++) {
        ph = (ph * BASE + pat[i]) % MOD1;
        th = (th * BASE + text[i]) % MOD1;
    }
    long long base_pow = modpow(BASE, m - 1, MOD1);
    for (int i = 0; i <= n - m; i++) {
        if (ph == th) res.push_back(i); // no verify step -> may include false positives
        if (i < n - m) {
            th = ((th - text[i] * base_pow % MOD1 + MOD1) % MOD1) * BASE % MOD1;
            th = (th + text[i + m]) % MOD1;
        }
    }
    return res;
}

vector<int> naive_search(const string &text, const string &pat) {
    vector<int> res;
    int n = text.size(), m = pat.size();
    if (m == 0) return res; // empty pattern: no matches (consistent with RK)
    for (int i = 0; i + m <= n; i++) {
        if (text.compare(i, m, pat) == 0) res.push_back(i);
    }
    return res;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int tests = 0, failures = 0;

    // ---- Test 1: basic correctness on small strings ----
    {
        struct Case { string t, p; };
        vector<Case> cases = {
            {"hello world", "lo"},
            {"aaaaa", "aa"},
            {"aabaaab", "aa"},
            {"mississippi", "issi"},
            {"abcdef", "xyz"},
            {"abc", "abc"},
            {"", "a"},   // empty text
            {"abc", ""}, // empty pattern
            {"a", "a"},
            {"the quick brown fox", "quick"},
        };
        for (auto &c : cases) {
            auto d = rabin_karp_double(c.t, c.p);
            auto nv = naive_search(c.t, c.p);
            tests++;
            if (d != nv) {
                failures++;
                cerr << "FAIL basic: text='" << c.t << "' pat='" << c.p << "'\n";
            }
        }
        cout << "basic_correctness_tests=" << cases.size()
             << " failures=" << failures << "\n";
    }

    // ---- Test 2: single vs double hash correctness (double must be exact) ----
    {
        // Random tests
        mt19937 rng(12345);
        int mismatches = 0;
        for (int iter = 0; iter < 5000; iter++) {
            int n = 1 + rng() % 200;
            int m = 1 + rng() % min(n, 30);
            string t(n, 'a'), p(m, 'a');
            string alpha = "ab";
            for (int i = 0; i < n; i++) t[i] = alpha[rng() % alpha.size()];
            for (int i = 0; i < m; i++) p[i] = alpha[rng() % alpha.size()];
            auto d = rabin_karp_double(t, p);
            auto nv = naive_search(t, p);
            tests++;
            if (d != nv) { failures++; mismatches++; }
        }
        cout << "random_correctness_tests=5000 mismatches=" << mismatches << "\n";
    }

    // ---- Test 3: adversarial collision test (single vs double) ----
    // Construct strings that cause single-hash false positives by brute-forcing
    // small strings with equal single-hash but different content.
    {
        // Find two distinct 2-char patterns with same MOD1 hash under BASE.
        bool found = false;
        long long exemplar_h = -1;
        string s1, s2;
        for (int a = 'a'; a <= 'z' && !found; a++) {
            for (int b = 'a'; b <= 'z'; b++) {
                string s(1, (char)a); s += (char)b;
                long long h = ((long long)a % MOD1 * BASE % MOD1 + b) % MOD1;
                if (exemplar_h < 0) { exemplar_h = h; s1 = s; }
                else if (h == exemplar_h && s != s1) { s2 = s; found = true; break; }
            }
        }
        cout << "single_hash_collision_pair_found=" << (found ? "yes" : "no");
        if (found) {
            cout << " pair=(" << s1 << "," << s2 << ")";
            // Demonstrate: text = s2, pattern = s1 -> single hash reports match (false positive),
            // double hash correctly reports no match.
            string t = s2, pat = s1;
            auto single = rabin_karp_single(t, pat);
            auto dbl = rabin_karp_double(t, pat);
            auto nv = naive_search(t, pat);
            cout << " single_hash_matches=" << single.size()
                 << " double_hash_matches=" << dbl.size()
                 << " naive_matches=" << nv.size();
        }
        cout << "\n";
    }

    // ---- Test 4: performance benchmark (large random text) ----
    {
        // Worst-case for naive: text all 'a', pattern = "aaaa...b" (n matches required)
        // forces naive to compare m chars at nearly every position.
        int n = 2000000;
        int m = 20;
        string t(n, 'a');
        string p(m, 'a');
        p[m - 1] = 'b'; // pattern never matches, but naive still compares ~m chars per pos

        auto t0 = chrono::high_resolution_clock::now();
        auto d = rabin_karp_double(t, p);
        auto t1 = chrono::high_resolution_clock::now();
        auto nv = naive_search(t, p);
        auto t2 = chrono::high_resolution_clock::now();

        double rk_ms = chrono::duration<double, milli>(t1 - t0).count();
        double nv_ms = chrono::duration<double, milli>(t2 - t1).count();

        cout << "benchmark_text_len=" << n << " pattern_len=" << m << "\n";
        cout << "rk_matches=" << d.size() << " naive_matches=" << nv.size() << "\n";
        cout << "rk_time_ms=" << fixed << setprecision(3) << rk_ms
             << " naive_time_ms=" << nv_ms << "\n";
        cout << "speedup=" << fixed << setprecision(2) << (nv_ms / rk_ms) << "x\n";
    }

    // ---- Test 5: multi-pattern matching (RK's real strength) ----
    // Rolling hash computed once per text position can be reused for many patterns.
    {
        mt19937 rng(777);
        int n = 1000000;
        string t(n, 'a');
        string alpha = "abcdefghijklmnopqrstuvwxyz";
        uniform_int_distribution<int> di(0, 25);
        for (int i = 0; i < n; i++) t[i] = alpha[di(rng)];
        int P = 200; // number of patterns to search simultaneously
        int m = 12;
        vector<string> pats(P, string(m, 'a'));
        for (int k = 0; k < P; k++)
            for (int i = 0; i < m; i++) pats[k][i] = alpha[di(rng)];

        // RK multi: compute prefix rolling hashes once, then check each pattern
        auto t0 = chrono::high_resolution_clock::now();
        long long total_rk = 0;
        {
            // Precompute hash of all length-m windows via rolling incrementally.
            // We compute pattern hashes individually and slide text once.
            unordered_map<PairHash, vector<int>, PairHashHasher> pat_hashes;
            pat_hashes.reserve(P * 2);
            for (int k = 0; k < P; k++) {
                long long ph1 = 0, ph2 = 0;
                for (int i = 0; i < m; i++) {
                    ph1 = (ph1 * BASE + pats[k][i]) % MOD1;
                    ph2 = (ph2 * BASE + pats[k][i]) % MOD2;
                }
                pat_hashes[{ph1, ph2}].push_back(k);
            }
            // slide text once
            long long th1 = 0, th2 = 0;
            for (int i = 0; i < m; i++) { th1 = (th1 * BASE + t[i]) % MOD1; th2 = (th2 * BASE + t[i]) % MOD2; }
            long long bp1 = modpow(BASE, m - 1, MOD1), bp2 = modpow(BASE, m - 1, MOD2);
            for (int i = 0; i <= n - m; i++) {
                auto it = pat_hashes.find({th1, th2});
                if (it != pat_hashes.end()) {
                    total_rk += it->second.size();
                }
                if (i < n - m) {
                    th1 = ((th1 - t[i] * bp1 % MOD1 + MOD1) % MOD1) * BASE % MOD1; th1 = (th1 + t[i + m]) % MOD1;
                    th2 = ((th2 - t[i] * bp2 % MOD2 + MOD2) % MOD2) * BASE % MOD2; th2 = (th2 + t[i + m]) % MOD2;
                }
            }
        }
        auto t1 = chrono::high_resolution_clock::now();

        // Naive multi: search each pattern independently
        long long total_nv = 0;
        for (int k = 0; k < P; k++) total_nv += naive_search(t, pats[k]).size();
        auto t2 = chrono::high_resolution_clock::now();

        double rk_ms = chrono::duration<double, milli>(t1 - t0).count();
        double nv_ms = chrono::duration<double, milli>(t2 - t1).count();

        cout << "multi_pattern_count=" << P << " text_len=" << n << " pat_len=" << m << "\n";
        cout << "multi_rk_matches=" << total_rk << " multi_naive_matches=" << total_nv << "\n";
        cout << "multi_rk_time_ms=" << fixed << setprecision(3) << rk_ms
             << " multi_naive_time_ms=" << nv_ms
             << " multi_speedup=" << fixed << setprecision(2) << (nv_ms / rk_ms) << "x\n";
    }

    cout << "TOTAL_TESTS=" << tests << " TOTAL_FAILURES=" << failures << "\n";
    cout << (failures == 0 ? "ALL_PASS" : "SOME_FAIL") << "\n";
    return failures == 0 ? 0 : 1;
}
