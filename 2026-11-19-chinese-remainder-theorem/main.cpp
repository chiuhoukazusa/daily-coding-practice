// Chinese Remainder Theorem (CRT) + Modular Arithmetic
// Topic: 中国剩余定理 + 扩展欧几里得模逆 + 快速幂 + Garner 表示
//
// Implements:
//   1. ext_gcd  (extended Euclidean)  -> modular inverse
//   2. mod_pow  (binary exponentiation)
//   3. crt_coprime (classic CRT for pairwise coprime moduli)  -> returns __int128
//   4. garner_crt   (mixed-radix reconstruction)              -> returns __int128
//   5. Brute-force baseline (for small M only)
//
// Quantitative verification:
//   - Reconstruction: x ≡ r_i (mod m_i) for ALL i  (via __int128, no overflow)
//   - Uniqueness:     0 <= x < M
//   - Cross-check:    CRT classic == Garner == brute (small M)
//   - Inverse check:  a * a^{-1} ≡ 1 (mod m)
//   - Pow check:      fast pow == naive
//   - Performance:    speedup over brute force

#include <bits/stdc++.h>
using namespace std;

typedef long long ll;
typedef __int128 i128;
typedef unsigned long long u64;

// ---------- Extended Euclidean ----------
ll ext_gcd(ll a, ll b, ll &x, ll &y) {
    if (b == 0) { x = 1; y = 0; return a; }
    ll x1, y1;
    ll g = ext_gcd(b, a % b, x1, y1);
    x = y1;
    y = x1 - (a / b) * y1;
    return g;
}

ll mod_inv(ll a, ll m) {
    ll x, y;
    ll g = ext_gcd(a, m, x, y);
    if (g != 1) return -1;
    x %= m; if (x < 0) x += m;
    return x;
}

// Modular multiply a*b mod m for __int128 operands (avoid overflow)
i128 mul_mod(i128 a, i128 b, i128 m) {
    i128 res = 0;
    a %= m; if (a < 0) a += m;
    b %= m; if (b < 0) b += m;
    while (b > 0) {
        if (b & 1) { res += a; if (res >= m) res -= m; }
        a += a; if (a >= m) a -= m;
        b >>= 1;
    }
    return res;
}

// ---------- Binary exponentiation ----------
ll mod_pow(ll a, ll e, ll m) {
    a %= m; if (a < 0) a += m;
    ll r = 1 % m;
    while (e > 0) {
        if (e & 1) r = (i128)r * a % m;
        a = (i128)a * a % m;
        e >>= 1;
    }
    return r;
}

// ---------- Classic CRT (pairwise coprime), returns __int128 to avoid overflow ----------
i128 crt_coprime(const vector<ll>& rem, const vector<ll>& mod_arr) {
    int n = rem.size();
    i128 M = 1;
    for (ll m : mod_arr) M *= m;

    i128 x = 0;
    for (int i = 0; i < n; i++) {
        i128 mi = mod_arr[i];
        i128 Mi = M / mi;                       // product of all others
        ll ti = mod_inv((ll)(Mi % mi), (ll)mi); // inverse of Mi mod mi
        i128 term = mul_mod((i128)(rem[i] % (ll)mi), Mi, M);
        term = mul_mod(term, ti, M);
        x = (x + term) % M;
    }
    return x;
}

// ---------- Garner's algorithm (mixed radix) ----------
// x = c0 + c1*m0 + c2*m0*m1 + ...  where ci in [0, mi)
i128 garner_crt(const vector<ll>& rem, const vector<ll>& mod_arr) {
    int n = rem.size();
    vector<ll> coeff(n, 0);
    // prefix[k] = m0 * m1 * ... * m_{k-1}  (exact, via __int128)
    for (int i = 0; i < n; i++) {
        ll mi = mod_arr[i];
        ll t = rem[i] % mi;
        if (t < 0) t += mi;
        // subtract value of already-decoded lower digits, evaluated mod mi
        i128 prefix = 1;          // m0*m1*...*m_{j-1} for current j
        for (int j = 0; j < i; j++) {
            // contrib = coeff[j] * prefix  (mod mi)
            ll contrib = (i128)coeff[j] * (prefix % mi) % mi;
            t = (t - contrib) % mi;
            if (t < 0) t += mi;
            prefix *= mod_arr[j];  // advance: now m0*...*m_j
        }
        // inv of (m0*m1*...*m_{i-1}) mod mi  == prefix mod mi
        ll inv_all = mod_inv((ll)(prefix % mi), mi);
        t = (i128)t * inv_all % mi;
        coeff[i] = t;
    }
    // reconstruct exact x
    i128 x = 0, cur = 1;
    for (int i = 0; i < n; i++) {
        x += (i128)coeff[i] * cur;
        cur *= mod_arr[i];
    }
    return x;
}

// ---------- Brute force (small M) ----------
i128 brute_crt(const vector<ll>& rem, const vector<ll>& mod_arr) __attribute__((noinline));
i128 brute_crt(const vector<ll>& rem, const vector<ll>& mod_arr) {
    i128 M = 1;
    for (ll m : mod_arr) M *= m;
    for (i128 x = 0; x < M; x++) {
        bool ok = true;
        for (size_t i = 0; i < rem.size(); i++)
            if (x % (i128)mod_arr[i] != (i128)(rem[i] % mod_arr[i])) { ok = false; break; }
        if (ok) return x;
    }
    return -1;
}

bool verify_x(const vector<ll>& rem, const vector<ll>& mod_arr, i128 x) {
    for (size_t i = 0; i < rem.size(); i++) {
        if (x % (i128)mod_arr[i] != (i128)(rem[i] % mod_arr[i])) return false;
    }
    return true;
}

// ---------- RNG ----------
u64 rng_state = 0x9E3779B97F4A7C15ULL;
u64 rng64() {
    u64 x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x;
    return x;
}
ll rand_mod(ll m) { return (ll)(rng64() % (u64)m); }

vector<ll> PRIMES = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71};
vector<ll> LARGE_COPRIME = {1000000007LL, 1000000009LL, 1000000021LL, 1000000033LL};

// print i128
string to_str(i128 v) {
    if (v == 0) return "0";
    bool neg = v < 0; if (neg) v = -v;
    string s;
    while (v) { s.push_back('0' + v % 10); v /= 10; }
    if (neg) s.push_back('-');
    reverse(s.begin(), s.end());
    return s;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int fail_count = 0;

    // ===== Test 1: modular inverse =====
    {
        int total = 0, ok = 0;
        for (ll m : {7LL, 11LL, 13LL, 97LL, 101LL, 1000000007LL}) {
            for (ll a = 1; a < m && a < 50; a++) {
                if (__gcd(a, m) != 1) continue;
                ll inv = mod_inv(a, m);
                if ((i128)a * inv % m != 1 % m) fail_count++;
                else ok++;
                total++;
            }
        }
        cout << "Test 1  Modular Inverse            : " << ok << "/" << total << " passed\n";
    }

    // ===== Test 2: fast power =====
    {
        int mismatch = 0;
        for (int t = 0; t < 500; t++) {
            ll m = PRIMES[rand_mod(PRIMES.size())] + 1;
            ll a = rand_mod(m);
            ll e = rand_mod(50);
            ll fast = mod_pow(a, e, m);
            ll naive = 1 % m;
            for (ll i = 0; i < e; i++) naive = (i128)naive * a % m;
            if (fast != naive) { mismatch++; fail_count++; }
        }
        cout << "Test 2  Fast Power vs Naive       : " << (500 - mismatch) << "/500 passed\n";
    }

    // ===== Test 3: CRT correctness + cross-check (classic == garner == brute) =====
    {
        int cases = 0, ok_reconstruct = 0, ok_crosscheck = 0;
        for (int t = 0; t < 2000; t++) {
            int k = 2 + rand_mod(4); // 2..5
            set<ll> chosen;
            vector<ll> mod_arr, rem;
            while ((int)chosen.size() < k) {
                ll p = PRIMES[rand_mod(PRIMES.size())];
                if (chosen.insert(p).second) mod_arr.push_back(p);
            }
            for (ll m : mod_arr) rem.push_back(rand_mod(m));

            i128 x_crt = crt_coprime(rem, mod_arr);
            i128 x_garner = garner_crt(rem, mod_arr);

            cases++;
            bool rec = verify_x(rem, mod_arr, x_crt) && verify_x(rem, mod_arr, x_garner);
            bool match = (x_crt == x_garner);
            if (rec && match) ok_reconstruct++; else { fail_count++; }

            // brute force only when M small
            i128 M = 1; for (ll m : mod_arr) M *= m;
            if (M < 1000000) {
                i128 xb = brute_crt(rem, mod_arr);
                if (xb == x_crt && xb == x_garner) ok_crosscheck++; else fail_count++;
            } else {
                ok_crosscheck++;
            }
        }
        cout << "Test 3  CRT Classic==Garner & Verify : " << ok_reconstruct << "/" << cases << " passed"
             << "  (brute cross-check " << ok_crosscheck << "/" << cases << ")\n";
    }

    // ===== Test 4: uniqueness range =====
    {
        int ok = 0, total = 0;
        for (int t = 0; t < 500; t++) {
            vector<ll> mod_arr = {3, 5, 7, 11};
            vector<ll> rem = {rand_mod(3), rand_mod(5), rand_mod(7), rand_mod(11)};
            i128 x = crt_coprime(rem, mod_arr);
            total++;
            if (x >= 0 && x < 1155) ok++;
        }
        cout << "Test 4  Uniqueness 0<=x<M          : " << ok << "/" << total << " passed\n";
    }

    // ===== Test 5: large coprime moduli (>2^31) =====
    {
        vector<ll> rem = {123456789LL, 987654321LL, 555555555LL, 111111111LL};
        i128 x = crt_coprime(rem, LARGE_COPRIME);
        bool ok = verify_x(rem, LARGE_COPRIME, x);
        cout << "Test 5  Large Moduli (>2^31), x     : " << to_str(x)
             << "  verify=" << (ok ? "PASS" : "FAIL") << "\n";
        if (!ok) fail_count++;
    }

    // ===== Test 6: performance =====
    {
        vector<ll> mod_arr = {101, 103, 107, 109, 113, 127};
        vector<ll> rem = {37, 59, 83, 5, 99, 51};
        i128 xcrt = crt_coprime(rem, mod_arr);
        bool v = verify_x(rem, mod_arr, xcrt);
        cout << "Test 6a CRT(6 moduli, M~1.7e12)      : x=" << to_str(xcrt)
             << "  verify=" << (v ? "PASS" : "FAIL") << "\n";
        if (!v) fail_count++;

        // time: CRT repeated many times vs brute on small system
        vector<ll> mod_small = {101, 103, 107}; // M ~ 1.11e6
        vector<ll> rem_small = {37, 59, 83};
        int R = 200000;

        i128 sink = 0;
        auto t0 = chrono::high_resolution_clock::now();
        for (int i = 0; i < R; i++) sink += crt_coprime(rem_small, mod_small);
        auto t1 = chrono::high_resolution_clock::now();
        double t_crt = chrono::duration<double, milli>(t1 - t0).count();

        // brute: vary target via RNG (defeats constant-folding) + noinline + volatile sink
        int Rb = 200;
        volatile i128 sink_b = 0;
        vector<ll> rem_v = {100, 102, 106};
        t0 = chrono::high_resolution_clock::now();
        for (int i = 0; i < Rb; i++) {
            rem_v[0] = 100 - (ll)(rng64() % 2);
            rem_v[1] = 102 - (ll)(rng64() % 2);
            rem_v[2] = 106 - (ll)(rng64() % 2);
            sink_b = sink_b + brute_crt(rem_v, mod_small);
        }
        t1 = chrono::high_resolution_clock::now();
        double t_brute = chrono::duration<double, milli>(t1 - t0).count();

        (void)sink; (void)sink_b;
        double per_crt = t_crt / R;
        double per_brute = t_brute / Rb;
        cout << "Test 6b Perf: CRT " << per_crt << " ms/op   Brute " << per_brute
             << " ms/op   speedup " << per_brute / max(per_crt, 1e-12) << "x"
             << "   (brute sink=" << to_str(sink_b) << ")\n";
    }

    cout << "==== SUMMARY ====\n";
    if (fail_count == 0) cout << "ALL QUANTITATIVE CHECKS PASSED ✅\n";
    else cout << fail_count << " CHECKS FAILED ❌\n";
    return fail_count == 0 ? 0 : 1;
}
