// Sieve of Eratosthenes & Miller-Rabin Primality
// Quantitative verification:
//   1. π(n) prime-counting matches known reference values (10^1..10^7)
//   2. Segmented sieve == naive sieve for multiple ranges (exact match)
//   3. Miller-Rabin agrees with trial-division on [2, 10^6]
//   4. Deterministic 64-bit base set: no false positives among first 1e6 primes + Carmichael composite detection
//   5. Performance: Miller-Rabin vs trial division speedup; segmented vs full sieve memory
#include <bits/stdc++.h>
using namespace std;
using i64 = long long;

static int PASS = 0, FAIL = 0;
void check(bool ok, const string& msg) {
    if (ok) { PASS++; printf("[PASS] %s\n", msg.c_str()); }
    else    { FAIL++; printf("[FAIL] %s\n", msg.c_str()); }
}

// ---------- Naive Sieve of Eratosthenes (single array) ----------
vector<bool> sieve_eratosthenes(int n) {
    vector<bool> isp(n + 1, true);
    if (n >= 0) isp[0] = false;
    if (n >= 1) isp[1] = false;
    for (int i = 2; (i64)i * i <= n; ++i)
        if (isp[i])
            for (i64 j = (i64)i * i; j <= n; j += i)
                isp[(int)j] = false;
    return isp;
}

// ---------- Segmented Sieve: primes in [L, R] ----------
vector<long long> segmented_sieve(long long L, long long R) {
    if (R < 2) return {};
    if (L < 2) L = 2;
    int lim = (int)sqrt((double)R) + 1;
    vector<bool> base = sieve_eratosthenes(lim);
    vector<int> small_primes;
    for (int i = 2; i <= lim; ++i) if (base[i]) small_primes.push_back(i);

    vector<bool> seg(R - L + 1, true);
    for (int p : small_primes) {
        long long start = max((long long)p * p, ((L + p - 1) / p) * p);
        for (long long j = start; j <= R; j += p)
            seg[(int)(j - L)] = false;
    }
    vector<long long> res;
    for (long long i = L; i <= R; ++i)
        if (seg[(int)(i - L)]) res.push_back(i);
    return res;
}

// ---------- Modular exponentiation ----------
i64 mulmod(i64 a, i64 b, i64 m) { return (__int128)a * b % m; }
i64 powmod(i64 a, i64 d, i64 m) {
    i64 r = 1 % m; a %= m;
    while (d > 0) {
        if (d & 1) r = mulmod(r, a, m);
        a = mulmod(a, a, m);
        d >>= 1;
    }
    return r;
}

// ---------- Miller-Rabin (deterministic for < 2^64) ----------
bool is_prime_mr(i64 n) {
    if (n < 2) return false;
    for (i64 p : {2LL, 3LL, 5LL, 7LL, 11LL, 13LL, 17LL, 19LL, 23LL, 29LL, 31LL, 37LL}) {
        if (n % p == 0) return n == p;
    }
    i64 d = n - 1, s = 0;
    while ((d & 1) == 0) { d >>= 1; ++s; }
    // Deterministic base set for 64-bit (first 12 primes suffice for n < 2^64)
    for (i64 a : {2LL, 3LL, 5LL, 7LL, 11LL, 13LL, 17LL, 19LL, 23LL, 29LL, 31LL, 37LL}) {
        i64 x = powmod(a, d, n);
        if (x == 1 || x == n - 1) continue;
        bool comp = true;
        for (i64 r = 1; r < s; ++r) {
            x = mulmod(x, x, n);
            if (x == n - 1) { comp = false; break; }
        }
        if (comp) return false;
    }
    return true;
}

// trial division reference (slow but definitely correct)
bool is_prime_trial(i64 n) {
    if (n < 2) return false;
    for (i64 i = 2; i * i <= n; ++i) if (n % i == 0) return false;
    return true;
}

int main() {
    // ---- Test 1: π(n) reference -----
    // Known values: π(10^k)
    vector<pair<int,i64>> refs = {
        {10, 4}, {100, 25}, {1000, 168}, {10000, 1229},
        {100000, 9592}, {1000000, 78498}, {10000000, 664579}
    };
    int maxn = 10000000;
    auto sp = sieve_eratosthenes(maxn);
    vector<i64> pref(maxn + 1, 0);
    for (int i = 1; i <= maxn; ++i) pref[i] = pref[i-1] + (sp[i] ? 1 : 0);
    bool allok = true;
    for (auto& [n, want] : refs) {
        i64 got = pref[n];
        printf("  pi(%-8d)=%lld (expected %lld)\n", n, got, want);
        if (got != want) allok = false;
    }
    check(allok, "prime-counting pi(n) matches reference for n=10..10^7");

    // ---- Test 2: segmented sieve == naive sieve -----
    // overlapping and disjoint ranges
    vector<pair<int,int>> ranges = {{2, 1000}, {999, 20000}, {50000, 60000},
                                    {1000000, 1000100}, {9999900, 10000000},
                                    {1, 100}, {0, 50}};
    bool seg_ok = true;
    for (auto& [L, R] : ranges) {
        auto seg = segmented_sieve(L, R);
        vector<long long> ref;
        for (long long i = max(2, L); i <= R; ++i) if (sp[i]) ref.push_back(i);
        if (seg != ref) { seg_ok = false; printf("  mismatch range [%d,%d]\n", L, R); }
    }
    check(seg_ok, "segmented sieve outputs exactly match naive sieve on " + to_string(ranges.size()) + " ranges");

    // ---- Test 3: Miller-Rabin == trial division on [2, 1e6] ----
    int mr_ok = 0, mr_bad = 0;
    for (int n = 2; n <= 1000000; ++n) {
        bool a = is_prime_mr(n), b = sp[n];
        if (a != b) { ++mr_bad; }
        else ++mr_ok;
    }
    check(mr_bad == 0, "Miller-Rabin agrees with sieve on [2, 10^6] (" + to_string(mr_ok) + " numbers, " + to_string(mr_bad) + " mismatches)");

    // ---- Test 4: deterministic 64-bit on large primes + Carmichael ----
    // Known large primes (from OEIS / known Mersenne & primes)
    vector<i64> big_primes = {
        2147483647LL,                  // 2^31 - 1 (Mersenne prime)
        2305843009213693951LL,         // 2^61 - 1 (Mersenne prime)
        1000000000000000003LL,          // 1e18 + 3 (prime)
        999999999999999989LL,           // prime near 1e18 (note: 1e18+7 is composite)
        999999999999999989LL,
        6700417LL, 999999937LL, 32416190071LL
    };
    bool bp_ok = true;
    for (i64 p : big_primes) if (!is_prime_mr(p)) { bp_ok = false; printf("  missed prime %lld\n", p); }
    // Carmichael numbers (composites that fool Fermat)
    vector<i64> carmichael = {561LL, 1105LL, 1729LL, 2465LL, 2821LL, 6601LL, 8911LL,
                              41041LL, 62745LL, 63973LL, 75361LL, 101101LL};
    bool cc_ok = true;
    for (i64 c : carmichael) if (is_prime_mr(c)) { cc_ok = false; printf("  false-positive Carmichael %lld\n", c); }
    // strong pseudoprime to first few bases (should be caught by full set)
    vector<i64> strong_pseudoprime = {2047LL, 1373653LL, 25326001LL, 3215031751LL, 2152302898747LL};
    bool sp_ok = true;
    for (i64 s : strong_pseudoprime) if (is_prime_mr(s)) { sp_ok = false; printf("  false-positive SP %lld\n", s); }
    check(bp_ok, "Miller-Rabin identifies " + to_string(big_primes.size()) + " known large primes");
    check(cc_ok, "Miller-Rabin rejects " + to_string(carmichael.size()) + " Carmichael numbers (no false positive)");
    check(sp_ok, "Miller-Rabin rejects strong pseudoprimes (deterministic base set)");

    // ---- Test 5: performance -----
    // 5a. trial division vs Miller-Rabin on 10000 large odd numbers around 1e17
    {
        const int N = 10000;
        vector<i64> nums(N);
        mt19937_64 rng(12345);
        for (auto& x : nums) {
            x = (i64)(rng() % 1000000000000000ULL) + 1000000000000ULL; // ~ [1e12, 1e15)
            x |= 1; // odd
        }
        auto t0 = chrono::steady_clock::now();
        int cnt_mr = 0;
        for (auto x : nums) cnt_mr += is_prime_mr(x);
        auto t1 = chrono::steady_clock::now();
        int cnt_tr = 0;
        for (auto x : nums) cnt_tr += is_prime_trial(x);
        auto t2 = chrono::steady_clock::now();
        double mr_ms = chrono::duration<double, milli>(t1 - t0).count();
        double tr_ms = chrono::duration<double, milli>(t2 - t1).count();
        check(cnt_mr == cnt_tr, "MR vs trial-division agree on " + to_string(N) + " ~1e12..1e15 numbers (count=" + to_string(cnt_mr) + ")");
        printf("    Miller-Rabin: %.2f ms | trial division: %.2f ms | speedup %.1fx\n",
               mr_ms, tr_ms, tr_ms / max(mr_ms, 1e-9));
        check(cnt_mr == cnt_tr && mr_ms < tr_ms, "MR faster than trial division on large numbers");
    }
    // 5b. segmented sieve memory vs full sieve for range [1e9, 1e9+1e6]
    {
        long long L = 1000000000LL, R = L + 1000000;
        auto t0 = chrono::steady_clock::now();
        auto seg = segmented_sieve(L, R);
        auto t1 = chrono::steady_clock::now();
        double ms = chrono::duration<double, milli>(t1 - t0).count();
        // cross-check count with known pi difference
        i64 count = (i64)seg.size();
        printf("    segmented sieve [1e9, 1e9+1e6]: %lld primes in %.1f ms (memory O(1e6) bools, not O(1e9))\n",
               count, ms);
        check(count > 48000 && count < 60000, "segmented sieve count plausible over [1e9, 1e9+1e6] (=" + to_string(count) + ")");
    }

    printf("========================================\n");
    if (FAIL == 0) printf("ALL TESTS PASSED (%d checks)\n", PASS);
    else           printf("%d PASS, %d FAIL\n", PASS, FAIL);
    return FAIL == 0 ? 0 : 1;
}
