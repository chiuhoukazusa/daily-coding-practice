// Matrix Chain Multiplication — Optimal Parenthesization via Interval DP
//
// Given a chain of matrices A0 A1 ... A_{n-1} with dimensions d[0] x d[1],
// d[1] x d[2], ..., d[n-1] x d[n], we want the parenthesization that minimizes
// the total number of scalar multiplications when evaluating the product.
//
// Core idea: interval DP. Let m[i][j] = min cost to multiply matrices i..j.
//   m[i][j] = min_{i<=k<j} ( m[i][k] + m[k+1][j] + d[i]*d[k+1]*d[j+1] )
//   m[i][i] = 0
// Reconstruct the optimal split via a split[][] table, then emit the
// parenthesization recursively.
//
// Verification strategy (fully quantitative, no eyeballing):
//   1. Brute force: enumerate ALL Catalan(n-1) parenthesizations via recursion
//      computing exact scalar-multiplication cost. Compare DP's minimum to the
//      exact brute-force minimum — must match exactly for every test case.
//   2. Sanity: optimal cost must equal the cost of evaluating the DP-reconstructed
//      parenthesization (re-evaluated independently), proving the emitted
//      parenthesization is self-consistent and actually achieves the reported min.
//   3. Speedup: measure DP vs brute force on random chains. Brute force cost is
//      super-exponential (Catalan ~4^n / n^{3/2}); DP is O(n^3). Report ratio.

#include <bits/stdc++.h>
using namespace std;

static const long long INF = LLONG_MAX / 4;

// ---- Bottom-up interval DP returning min cost + split table ----
long long matrixChainDP(const vector<int>& d,
                        vector<vector<int>>& split) {
    int n = (int)d.size() - 1;          // number of matrices
    vector<vector<long long>> m(n + 1, vector<long long>(n + 1, 0));
    split.assign(n + 1, vector<int>(n + 1, -1));

    for (int len = 2; len <= n; ++len) {          // chain length
        for (int i = 0; i + len - 1 < n; ++i) {
            int j = i + len - 1;
            m[i][j] = INF;
            for (int k = i; k < j; ++k) {
                // cost = m[i][k] + m[k+1][j] + d[i]*d[k+1]*d[j+1]
                long long cost = m[i][k] + m[k + 1][j]
                               + (long long)d[i] * d[k + 1] * d[j + 1];
                if (cost < m[i][j]) {
                    m[i][j] = cost;
                    split[i][j] = k;
                }
            }
        }
    }
    return m[0][n - 1];
}

// ---- Reconstruct parenthesization string from split table ----
void buildParens(const vector<vector<int>>& split,
                 int i, int j, string& out) {
    if (i == j) {
        out += "A" + to_string(i);
        return;
    }
    out += "(";
    int k = split[i][j];
    buildParens(split, i, k, out);
    out += " x ";
    buildParens(split, k + 1, j, out);
    out += ")";
}

// ---- Independent evaluation of a parenthesization string ----
// Parses "((A0 x A1) x A2)" and computes the exact multiplication cost,
// tracking the resulting matrix dimensions.
struct EvalResult { long long cost; int rows, cols; };
EvalResult evalParensString(const string& s, const vector<int>& d,
                           bool* ok) {
    *ok = true;
    // We parse recursively using a token index.
    int pos = 0;
    function<EvalResult()> parse = [&]() -> EvalResult {
        if (pos >= (int)s.size()) { *ok = false; return {0,0,0}; }
        char c = s[pos];
        if (c == '(') {
            ++pos; // consume '('
            EvalResult left = parse();
            // expect ' x '
            if (!(pos + 2 < (int)s.size() && s[pos]==' ' && s[pos+1]=='x' && s[pos+2]==' ')) {
                *ok = false; return {0,0,0};
            }
            pos += 3; // consume " x "
            EvalResult right = parse();
            // expect ')'
            if (pos >= (int)s.size() || s[pos] != ')') { *ok = false; return {0,0,0}; }
            ++pos; // consume ')'
            if (left.cols != right.rows) { *ok = false; return {0,0,0}; }
            long long c = left.cost + right.cost
                        + (long long)left.rows * left.cols * right.cols;
            return {c, left.rows, right.cols};
        } else if (c == 'A') {
            // parse "A<num>"
            int num = 0, t = pos + 1;
            while (t < (int)s.size() && isdigit(s[t])) { num = num*10 + (s[t]-'0'); ++t; }
            pos = t;
            int idx = num;
            if (idx < 0 || idx >= (int)d.size()) { *ok=false; return {0,0,0}; }
            return {0, d[idx], d[idx+1]};
        } else {
            *ok = false; return {0,0,0};
        }
    };
    EvalResult r = parse();
    if (pos != (int)s.size()) { *ok = false; }
    return r;
}

// ---- Brute force: enumerate all parenthesizations and find min cost ----
long long bruteForce(const vector<int>& d, int i, int j,
                     vector<int>& bestPath) {
    if (i == j) return 0;
    long long best = INF;
    for (int k = i; k < j; ++k) {
        vector<int> lp, rp;
        long long lc = bruteForce(d, i, k, lp);
        long long rc = bruteForce(d, k + 1, j, rp);
        long long cost = lc + rc + (long long)d[i] * d[k+1] * d[j+1];
        if (cost < best) {
            best = cost;
            bestPath = {k};
            bestPath.insert(bestPath.end(), lp.begin(), lp.end());
            bestPath.insert(bestPath.end(), rp.begin(), rp.end());
        }
    }
    return best;
}

// ---- Count Catalan parenthesizations (for reporting brute-force work) ----
long long catalanParenthesizations(int n) {
    // number of ways to parenthesize n matrices = Catalan(n-1)
    long long c = 1;
    int N = n - 1;
    for (int i = 0; i < N; ++i) {
        c = c * (2 * (2*i + 1)) / (i + 2);
    }
    return c;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    // Fixed deterministic test cases (dimension chains), each designed to be
    // small enough for exact brute force but varied in shape.
    vector<vector<int>> cases = {
        {10, 20, 30, 40, 30},               // n=4, classic CLRS-style
        {5, 10, 3, 12, 5, 50, 6},           // n=6, classic problem
        {30, 35, 15, 5, 10, 20, 25},        // n=6 (textbook example)
        {2, 3, 4, 5},                       // n=3
        {7, 6, 5, 4, 3, 2},                 // n=5, decreasing
        {100, 1, 100, 1, 100},              // n=4, extreme skew
        {1, 100, 1, 100, 1, 100, 1},        // n=6, alternating
    };

    bool allCorrect = true;
    int passed = 0;

    cout << "=== Matrix Chain Multiplication — Quantitative Verification ===\n";
    cout << fixed << setprecision(0);

    for (size_t c = 0; c < cases.size(); ++c) {
        const vector<int>& d = cases[c];
        int n = (int)d.size() - 1;

        vector<vector<int>> split;
        long long dpCost = matrixChainDP(d, split);

        // Reconstruct parenthesization and re-evaluate it independently.
        string parens;
        buildParens(split, 0, n - 1, parens);
        bool ok = false;
        auto evaled = evalParensString(parens, d, &ok);

        // Brute force exact minimum.
        vector<int> bp;
        long long bfCost = bruteForce(d, 0, n - 1, bp);

        bool selfConsistent = ok && (evaled.cost == dpCost)
                              && (evaled.rows == d[0])
                              && (evaled.cols == d[n]);
        bool matchesBF = (dpCost == bfCost);

        bool pass = selfConsistent && matchesBF;
        if (pass) ++passed; else allCorrect = false;

        cout << "\n[Case " << (c+1) << "] dims=[";
        for (size_t vi = 0; vi < d.size(); ++vi)
            cout << d[vi] << (vi + 1 == d.size() ? "" : ",");
        cout << "]  n=" << n << "\n";
        cout << "  DP min cost          = " << dpCost << "\n";
        cout << "  Brute-force min cost = " << bfCost << "\n";
        cout << "  Parenthesization     = " << parens << "\n";
        cout << "  Re-eval cost         = " << evaled.cost
             << "  (dims " << evaled.rows << "x" << evaled.cols << ")\n";
        cout << "  Self-consistent      = " << (selfConsistent ? "PASS" : "FAIL")
             << "   Matches brute-force = " << (matchesBF ? "PASS" : "FAIL")
             << "   => " << (pass ? "✅" : "❌") << "\n";
    }

    cout << "\n=== Correctness summary: " << passed << "/" << cases.size()
         << " cases passed ===\n\n";

    // ---- Performance / speedup benchmark on random chains ----
    cout << "=== Performance comparison (DP O(n^3) vs Brute-force Catalan) ===\n";
    mt19937 rng(12345);
    uniform_int_distribution<int> dist(5, 120);
    vector<int> nList = {8, 10, 12, 14, 16, 18, 20};
    cout << "  n  |       DP_time(us) |      BF_time(us) |  Catalan(pars) | speedup\n";
    cout << "-----|-------------------|------------------|----------------|---------\n";
    for (int n : nList) {
        vector<int> d(n + 1);
        for (int& v : d) v = dist(rng);

        // DP timing
        auto t0 = chrono::high_resolution_clock::now();
        vector<vector<int>> sp;
        long long dpc = matrixChainDP(d, sp);
        auto t1 = chrono::high_resolution_clock::now();
        double dpUs = chrono::duration<double, micro>(t1 - t0).count();

        // Brute-force timing (may be expensive for larger n)
        auto t2 = chrono::high_resolution_clock::now();
        vector<int> bp;
        long long bfc = bruteForce(d, 0, n - 1, bp);
        auto t3 = chrono::high_resolution_clock::now();
        double bfUs = chrono::duration<double, micro>(t3 - t2).count();

        long long cat = catalanParenthesizations(n);
        double speedup = (bfUs > 0.0) ? bfUs / dpUs : -1.0;

        cout << setw(5) << n << " | "
             << setw(17) << (long long)dpUs << " | "
             << setw(16) << (long long)bfUs << " | "
             << setw(14) << cat << " | "
             << setw(8) << setprecision(1) << speedup << "x\n";
        cout << setprecision(0);

        // consistency of cost between DP and BF for random cases too
        if (dpc != bfc) {
            cout << "  ⚠️ MISMATCH dp=" << dpc << " bf=" << bfc << "\n";
            allCorrect = false;
        }
    }

    cout << "\n=== Overall: " << (allCorrect ? "ALL CHECKS PASSED ✅" : "SOME CHECKS FAILED ❌")
         << " ===\n";

    return allCorrect ? 0 : 1;
}
