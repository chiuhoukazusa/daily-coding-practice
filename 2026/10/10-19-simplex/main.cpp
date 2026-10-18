// Simplex Linear Programming Solver
// Solves linear programs in standard form:
//   maximize c^T x
//   subject to A x <= b, x >= 0
// Uses the two-phase simplex method (Phase I finds feasibility, Phase II optimizes).
// Verification: brute-force enumeration of all basic feasible solutions (vertices)
// compares optimal objective value with the simplex result.

#include <bits/stdc++.h>
using namespace std;

static const double EPS = 1e-9;

struct SimplexResult {
    bool feasible;
    bool bounded;
    double value;
    vector<double> x;
};

// High-level solve using an explicit two-phase tableau.
// Columns order (Phase I): [0..n-1 original][n..n+m-1 slacks][n+m..n+m+m-1 artificials][RHS]
SimplexResult solveLP(const vector<vector<double>>& A,
                      const vector<double>& b,
                      const vector<double>& c) {
    SimplexResult res;
    res.feasible = true; res.bounded = true;
    int n = c.size(), m = b.size();

    int nvar = n + m;
    int total = nvar + m + 1;
    vector<vector<double>> T(m + 1, vector<double>(total, 0.0));
    vector<int> basis(m);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) T[i][j] = A[i][j];
        T[i][n + i] = 1.0;           // slack
        T[i][nvar + i] = 1.0;        // artificial
        T[i][total - 1] = b[i];      // RHS
        basis[i] = nvar + i;         // artificial basic
    }
    // Phase I objective: minimize sum of artificials (w). Row m holds coefficients.
    for (int i = 0; i < m; i++) T[m][nvar + i] = 1.0;
    // Zero out objective coefficients of basic (artificial) columns.
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < total; j++) T[m][j] -= T[i][j];
    }

    // Phase I simplex loop
    while (true) {
        int q = -1;
        for (int j = 0; j < nvar + m; j++)
            if (T[m][j] < -EPS) { q = j; break; }
        if (q == -1) break; // phase I optimal
        int p = -1; double best = 1e18;
        for (int i = 0; i < m; i++) {
            if (T[i][q] > EPS) {
                double r = T[i][total - 1] / T[i][q];
                if (r < best - EPS) { best = r; p = i; }
            }
        }
        if (p == -1) { res.feasible = false; return res; }
        double piv = T[p][q];
        for (auto& v : T[p]) v /= piv;
        for (int i = 0; i <= m; i++) {
            if (i == p) continue;
            double f = T[i][q];
            if (fabs(f) < EPS) continue;
            for (int j = 0; j < total; j++) T[i][j] -= f * T[p][j];
        }
        basis[p] = q;
    }
    double wval = T[m][total - 1];
    if (fabs(wval) > 1e-7) { res.feasible = false; return res; }

    // Remove artificial columns.
    for (int i = 0; i <= m; i++) {
        vector<double> row;
        for (int j = 0; j < nvar; j++) row.push_back(T[i][j]);
        row.push_back(T[i][total - 1]);
        T[i] = row;
    }
    int t2 = nvar + 1;

    // Phase II: restore original objective (maximize c^T x).
    fill(T[m].begin(), T[m].end(), 0.0);
    for (int j = 0; j < n; j++) T[m][j] = -c[j];
    for (int i = 0; i < m; i++) {
        int bcol = basis[i];
        if (bcol >= nvar) continue;
        double f = T[m][bcol];
        if (fabs(f) < EPS) continue;
        for (int j = 0; j < t2; j++) T[m][j] -= f * T[i][j];
    }

    // Phase II simplex loop
    while (true) {
        int q = -1;
        for (int j = 0; j < nvar; j++)
            if (T[m][j] < -EPS) { q = j; break; }
        if (q == -1) break; // optimal
        int p = -1; double best = 1e18;
        for (int i = 0; i < m; i++) {
            if (T[i][q] > EPS) {
                double r = T[i][t2 - 1] / T[i][q];
                if (r < best - EPS) { best = r; p = i; }
            }
        }
        if (p == -1) { res.bounded = false; return res; }
        double piv = T[p][q];
        for (auto& v : T[p]) v /= piv;
        for (int i = 0; i <= m; i++) {
            if (i == p) continue;
            double f = T[i][q];
            if (fabs(f) < EPS) continue;
            for (int j = 0; j < t2; j++) T[i][j] -= f * T[p][j];
        }
        basis[p] = q;
    }

    res.value = T[m][t2 - 1];
    res.x.assign(n, 0.0);
    for (int i = 0; i < m; i++)
        if (basis[i] < n) res.x[basis[i]] = T[i][t2 - 1];
    return res;
}

// --- Brute force vertex enumeration for verification -----------------------
// Enumerate all subsets of m basic variables out of n+m, solve the linear system
// for each, keep those with all variables >= 0 (BFS), record max objective.
double bruteForceOptimal(const vector<vector<double>>& A,
                         const vector<double>& b,
                         const vector<double>& c) {
    int n = c.size(), m = b.size();
    int N = n + m;
    double best = -1e18;
    bool found = false;

    vector<vector<double>> M(m, vector<double>(N, 0.0));
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) M[i][j] = A[i][j];
        M[i][n + i] = 1.0;
    }

    vector<int> comb(m);
    function<void(int,int)> rec = [&](int start, int k) {
        if (k == m) {
            vector<vector<double>> S(m, vector<double>(m, 0.0));
            for (int i = 0; i < m; i++)
                for (int j = 0; j < m; j++)
                    S[i][j] = M[i][comb[j]];
            vector<double> rhs = b;
            bool singular = false;
            for (int col_i = 0; col_i < m; col_i++) {
                int piv = -1;
                for (int r = col_i; r < m; r++)
                    if (fabs(S[r][col_i]) > EPS) { piv = r; break; }
                if (piv == -1) { singular = true; break; }
                swap(S[col_i], S[piv]);
                swap(rhs[col_i], rhs[piv]);
                double d = S[col_i][col_i];
                for (int j = col_i; j < m; j++) S[col_i][j] /= d;
                rhs[col_i] /= d;
                for (int r = 0; r < m; r++) {
                    if (r == col_i) continue;
                    double f = S[r][col_i];
                    if (fabs(f) < EPS) continue;
                    for (int j = col_i; j < m; j++) S[r][j] -= f * S[col_i][j];
                    rhs[r] -= f * rhs[col_i];
                }
            }
            if (singular) return;
            for (int i = 0; i < m; i++)
                if (rhs[i] < -1e-7) return; // infeasible vertex (negative basic)
            double obj = 0.0;
            for (int i = 0; i < m; i++)
                if (comb[i] < n) obj += c[comb[i]] * rhs[i];
            best = max(best, obj);
            found = true;
            return;
        }
        for (int i = start; i < N; i++) {
            comb[k] = i;
            rec(i + 1, k + 1);
        }
    };
    rec(0, 0);
    return found ? best : -1e18;
}

// --- Random LP generator ----------------------------------------------------
mt19937 rng(12345);

struct LP {
    vector<vector<double>> A;
    vector<double> b;
    vector<double> c;
};

LP genLP(int n, int m) {
    LP lp;
    lp.A.assign(m + n, vector<double>(n, 0.0));
    lp.b.assign(m + n, 0.0);
    lp.c.assign(n, 0.0);
    uniform_real_distribution<double> coef(-5.0, 5.0);
    uniform_real_distribution<double> rhs(1.0, 20.0);
    uniform_real_distribution<double> ub(2.0, 10.0);
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) lp.A[i][j] = coef(rng);
        lp.b[i] = rhs(rng); // b >= 0 => x=0 feasible
    }
    // Add upper bounds x_j <= ub_j => bounded feasible polytope (nonempty, contains 0).
    for (int j = 0; j < n; j++) {
        lp.A[m + j][j] = 1.0;
        lp.b[m + j] = ub(rng);
    }
    for (int j = 0; j < n; j++) lp.c[j] = coef(rng);
    return lp;
}

int main() {
    int passed = 0, total = 0;
    vector<double> diffs;

    cout << "=== Simplex LP Solver Verification ===\n";
    cout << "Comparing simplex optimal value vs brute-force vertex enumeration\n\n";

    vector<pair<int,int>> configs = {{2,2},{2,3},{3,2},{3,3},{3,4},{4,4},{4,5}};
    int testsPerConfig = 20;

    for (auto [n, m] : configs) {
        cout << "Config n=" << n << " m=" << m << " : ";
        int ok = 0, cnt = 0;
        for (int t = 0; t < testsPerConfig; t++) {
            LP lp = genLP(n, m);
            auto res = solveLP(lp.A, lp.b, lp.c);
            double brute = bruteForceOptimal(lp.A, lp.b, lp.c);
            total++; cnt++;
            if (!res.feasible) { cout << " [INFEASIBLE!]"; continue; }
            if (!res.bounded) { cout << " [UNBOUNDED!]"; continue; }
            double diff = fabs(res.value - brute);
            diffs.push_back(diff);
            if (diff < 1e-5) ok++;
        }
        cout << " passed " << ok << "/" << cnt << "\n";
        passed += ok;
    }

    cout << "\n=== Summary ===\n";
    cout << "Total tests: " << total << "\n";
    cout << "Passed (|simplex - brute| < 1e-5): " << passed << "\n";

    if (!diffs.empty()) {
        double maxDiff = *max_element(diffs.begin(), diffs.end());
        double meanDiff = accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
        cout << "Max |diff|: " << maxDiff << "\n";
        cout << "Mean |diff|: " << meanDiff << "\n";
    }

    // --- Layer 2: feasibility check on returned solution -----------------
    cout << "\n=== Feasibility Check (A x <= b, x >= 0) ===\n";
    int feasPass = 0, feasTotal = 0;
    double maxViol = 0.0;
    for (auto [nn, mm] : configs) {
        for (int t = 0; t < testsPerConfig; t++) {
            LP lp = genLP(nn, mm);
            auto res = solveLP(lp.A, lp.b, lp.c);
            if (!res.feasible || !res.bounded) continue;
            feasTotal++;
            double viol = 0.0;
            for (double xj : res.x) viol = max(viol, max(0.0, -xj)); // x >= 0
            for (int i = 0; i < (int)lp.b.size(); i++) {
                double lhs = 0.0;
                for (int j = 0; j < (int)lp.c.size(); j++) lhs += lp.A[i][j] * res.x[j];
                viol = max(viol, lhs - lp.b[i]); // A x <= b
            }
            maxViol = max(maxViol, viol);
            if (viol < 1e-6) feasPass++;
        }
    }
    cout << "Feasible solutions: " << feasPass << "/" << feasTotal << "\n";
    cout << "Max constraint violation: " << maxViol << "\n";

    // --- Layer 3: analytical textbook case -------------------------------
    cout << "\n=== Analytical Textbook Case ===\n";
    // maximize 3x + 2y  s.t.  x + y <= 4, 2x + y <= 5,  x,y >= 0
    // Vertices: (0,0)=0, (0,4)=8, (2.5,0)=7.5, intersect x+y=4 & 2x+y=5 -> x=1,y=3 -> 3+6=9
    {
        vector<vector<double>> A = {{1,1},{2,1}};
        vector<double> b = {4,5};
        vector<double> c = {3,2};
        auto r = solveLP(A, b, c);
        double expected = 9.0;
        cout << "optimal value = " << r.value << " (expected " << expected << ")\n";
        cout << "solution x = [" << r.x[0] << ", " << r.x[1] << "]\n";
        cout << "(expected x=[1,3])\n";
        bool ana = fabs(r.value - expected) < 1e-6 && fabs(r.x[0]-1.0)<1e-6 && fabs(r.x[1]-3.0)<1e-6;
        cout << (ana ? "✅ ANALYTICAL CASE PASSED" : "❌ ANALYTICAL CASE FAILED") << "\n";
        if (!ana) passed--;
    }

    cout << "\n" << (passed == total ? "✅ ALL TESTS PASSED" : "❌ SOME TESTS FAILED") << "\n";
    return (passed == total) ? 0 : 1;
}
// (verification helpers appended in a separate test below)
