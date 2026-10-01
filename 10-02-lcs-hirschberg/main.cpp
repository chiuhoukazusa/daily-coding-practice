#include <bits/stdc++.h>
using namespace std;

// ============================================================
// LCS (Longest Common Subsequence)
// Three implementations compared:
//   1. Naive O(n*m) DP with full table + traceback  (reference)
//   2. Space-optimized O(min(n,m)) DP for length only
//   3. Hirschberg's linear-space O(min(n,m)) divide-and-conquer
//      with traceback reconstruction
// Verification is quantitative: cross-check lengths, verify the
// reconstructed subsequence is actually common to both strings,
// and verify the exact LCS length against the reference.
// ============================================================

static const long long MOD_BASE = 0; // unused placeholder

// ---------- 1. Naive full-table DP (reference) ----------
int lcs_naive_len(const string& a, const string& b) {
    int n = a.size(), m = b.size();
    vector<vector<int>> dp(n + 1, vector<int>(m + 1, 0));
    for (int i = 1; i <= n; ++i)
        for (int j = 1; j <= m; ++j)
            if (a[i-1] == b[j-1]) dp[i][j] = dp[i-1][j-1] + 1;
            else dp[i][j] = max(dp[i-1][j], dp[i][j-1]);
    return dp[n][m];
}

string lcs_naive(const string& a, const string& b) {
    int n = a.size(), m = b.size();
    vector<vector<int>> dp(n + 1, vector<int>(m + 1, 0));
    for (int i = 1; i <= n; ++i)
        for (int j = 1; j <= m; ++j)
            if (a[i-1] == b[j-1]) dp[i][j] = dp[i-1][j-1] + 1;
            else dp[i][j] = max(dp[i-1][j], dp[i][j-1]);
    // traceback
    string res;
    int i = n, j = m;
    while (i > 0 && j > 0) {
        if (a[i-1] == b[j-1]) { res.push_back(a[i-1]); --i; --j; }
        else if (dp[i-1][j] >= dp[i][j-1]) --i;
        else --j;
    }
    reverse(res.begin(), res.end());
    return res;
}

// ---------- 2. Space-optimized DP (length only) ----------
int lcs_space_len(const string& a, const string& b) {
    // ensure a is shorter for O(min(n,m)) space
    if (a.size() > b.size()) return lcs_space_len(b, a);
    int n = a.size(), m = b.size();
    vector<int> prev(m + 1, 0), cur(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j) {
            if (a[i-1] == b[j-1]) cur[j] = prev[j-1] + 1;
            else cur[j] = max(prev[j], cur[j-1]);
        }
        swap(prev, cur);
    }
    return prev[m];
}

// ---------- 3. Hirschberg (linear space + reconstruction) ----------
// Compute last row of DP between a[0..n) and b[0..m) using O(m) space.
static vector<int> hirschberg_last_row(const string& a, const string& b, int n, int m) {
    vector<int> prev(m + 1, 0), cur(m + 1, 0);
    for (int i = 1; i <= n; ++i) {
        for (int j = 1; j <= m; ++j)
            cur[j] = (a[i-1] == b[j-1]) ? prev[j-1] + 1 : max(prev[j], cur[j-1]);
        swap(prev, cur);
    }
    return prev;
}

// Hirschberg recursive reconstruction.
// Works on substrings a[alo..ahi), b[blo..bhi).
static void hirschberg_rec(const string& a, int alo, int ahi,
                           const string& b, int blo, int bhi,
                           string& out) {
    int n = ahi - alo;
    int m = bhi - blo;
    if (n == 0) return;
    if (n == 1) {
        char c = a[alo];
        for (int j = blo; j < bhi; ++j)
            if (b[j] == c) { out.push_back(c); return; }
        return;
    }
    int amid = alo + n / 2;
    // forward last-row from [alo..amid) against [blo..bhi)
    vector<int> fwd = hirschberg_last_row(a.substr(alo, amid - alo), b.substr(blo, bhi - blo),
                                          amid - alo, bhi - blo);
    // backward last-row from reversed [amid..ahi) against reversed [blo..bhi)
    string ar = a.substr(amid, ahi - amid), br = b.substr(blo, bhi - blo);
    reverse(ar.begin(), ar.end());
    reverse(br.begin(), br.end());
    vector<int> bwd = hirschberg_last_row(ar, br, ahi - amid, bhi - blo);
    // find split point k maximizing fwd[k] + bwd[m-k]
    int best = -1, bestk = 0;
    for (int k = 0; k <= m; ++k) {
        int val = fwd[k] + bwd[m - k];
        if (val > best) { best = val; bestk = k; }
    }
    hirschberg_rec(a, alo, amid, b, blo, blo + bestk, out);
    hirschberg_rec(a, amid, ahi, b, blo + bestk, bhi, out);
}

string lcs_hirschberg(const string& a, const string& b) {
    string out;
    hirschberg_rec(a, 0, a.size(), b, 0, b.size(), out);
    return out;
}

// ---------- helpers ----------
// verify s is a subsequence of t
static bool is_subseq(const string& s, const string& t) {
    int i = 0;
    for (char c : t) {
        if (i < (int)s.size() && s[i] == c) ++i;
    }
    return i == (int)s.size();
}

static string random_string(int len, int alphabet, mt19937& rng) {
    uniform_int_distribution<int> dist(0, alphabet - 1);
    string s;
    s.reserve(len);
    for (int i = 0; i < len; ++i) s.push_back('a' + dist(rng));
    return s;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    mt19937 rng(12345);

    struct Case { string a, b; int expected_len; };
    vector<Case> cases = {
        {"", "", 0},
        {"ABCBDAB", "BDCABA", 4},   // classic: LCS = "BCBA" or "BCAB", len 4
        {"abcdef", "abcdef", 6},    // identical
        {"aaaaa", "aaa", 3},        // repeated — all 'a'
        {"abc", "xyz", 0},          // disjoint
        {"AGGTAB", "GXTXAYB", 4},   // classic: "GTAB"
    };

    // random cases
    for (int i = 0; i < 200; ++i) {
        string a = random_string(rng() % 60, 4, rng);
        string b = random_string(rng() % 60, 4, rng);
        cases.push_back({a, b, lcs_naive_len(a, b)});
    }

    int passed = 0, total = cases.size();

    for (auto& c : cases) {
        int naive_len = lcs_naive_len(c.a, c.b);
        int space_len = lcs_space_len(c.a, c.b);
        string hb = lcs_hirschberg(c.a, c.b);
        int hb_len = (int)hb.size();

        bool ok = true;
        if (naive_len != c.expected_len) ok = false;
        if (space_len != naive_len) ok = false;
        if (hb_len != naive_len) ok = false;
        if (!is_subseq(hb, c.a) || !is_subseq(hb, c.b)) ok = false;

        if (ok) ++passed;
        else {
            cout << "FAIL case a=\"" << c.a << "\" b=\"" << c.b << "\"\n";
            cout << "  naive_len=" << naive_len << " space_len=" << space_len
                 << " hb_len=" << hb_len << " expected=" << c.expected_len << "\n";
        }
    }

    cout << "=== LCS Verification Report ===" << endl;
    cout << "Total cases: " << total << endl;
    cout << "Passed:      " << passed << endl;
    cout << "Failed:      " << (total - passed) << endl;

    // Print a few concrete examples
    cout << "\n--- Examples ---" << endl;
    {
        string a = "ABCBDAB", b = "BDCABA";
        cout << "a=\"" << a << "\" b=\"" << b << "\"" << endl;
        cout << "  LCS length (naive)  = " << lcs_naive_len(a, b) << endl;
        cout << "  LCS length (space)  = " << lcs_space_len(a, b) << endl;
        cout << "  LCS (hirschberg)    = \"" << lcs_hirschberg(a, b) << "\"" << endl;
    }
    cout << endl;

    // Performance comparison on a moderately large random case
    {
        string a = random_string(2000, 4, rng);
        string b = random_string(2000, 4, rng);
        auto t0 = chrono::steady_clock::now();
        volatile int l1 = lcs_naive_len(a, b);
        auto t1 = chrono::steady_clock::now();
        volatile int l2 = lcs_space_len(a, b);
        auto t2 = chrono::steady_clock::now();
        string hb = lcs_hirschberg(a, b);
        auto t3 = chrono::steady_clock::now();

        auto ms = [](auto s, auto e){ return chrono::duration<double, milli>(e-s).count(); };
        cout << "--- Performance (length 2000, alphabet 4) ---" << endl;
        cout << "  naive O(nm) table:   " << ms(t0,t1) << " ms  (len=" << l1 << ")" << endl;
        cout << "  space O(min) length: " << ms(t1,t2) << " ms  (len=" << l2 << ")" << endl;
        cout << "  hirschberg rec:      " << ms(t2,t3) << " ms  (len=" << hb.size() << ")" << endl;
        cout << "  hirschberg==naive?   " << ((int)hb.size()==l1 ? "YES":"NO") << endl;
    }

    if (total - passed == 0) {
        cout << "\nRESULT: PASS" << endl;
        return 0;
    } else {
        cout << "\nRESULT: FAIL" << endl;
        return 1;
    }
}
