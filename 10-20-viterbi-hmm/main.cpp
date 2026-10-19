// Viterbi Algorithm for Hidden Markov Model (HMM) Decoding
//
// Given an HMM (transition matrix A, emission matrix B, initial distribution pi)
// and an observation sequence O, find the most likely hidden state sequence Q*
// that maximizes P(Q, O).
//
// Quantitative verification:
//   1. Brute-force enumeration over ALL state sequences (small state counts)
//      as ground truth, exact match required.
//   2. Log-space implementation (prevents underflow) matches direct-probability
//      implementation.
//   3. Random HMMs + random sequences (multiple trials) all pass.
//
// Uses log probabilities to avoid floating-point underflow.

#include <bits/stdc++.h>
using namespace std;

// Log-space Viterbi. Returns (log_prob_of_best_path, best_state_sequence).
pair<double, vector<int>> viterbi_log(const vector<vector<double>>& A,
                                      const vector<vector<double>>& B,
                                      const vector<double>& pi,
                                      const vector<int>& obs) {
    int N = (int)pi.size();   // number of hidden states
    int T = (int)obs.size();  // length of observation sequence

    vector<vector<double>> logA(N, vector<double>(N));
    vector<vector<double>> logB(N);
    for (int i = 0; i < N; i++) {
        logB[i].resize(B[i].size());
        for (size_t j = 0; j < B[i].size(); j++) logB[i][j] = log(B[i][j]);
        for (int j = 0; j < N; j++) logA[i][j] = log(A[i][j]);
    }
    vector<double> logpi(N);
    for (int i = 0; i < N; i++) logpi[i] = log(pi[i]);

    vector<vector<double>> dp(T, vector<double>(N, -1e300));
    vector<vector<int>> back(T, vector<int>(N, -1));

    for (int i = 0; i < N; i++)
        dp[0][i] = logpi[i] + logB[i][obs[0]];

    for (int t = 1; t < T; t++) {
        for (int j = 0; j < N; j++) {
            double best = -1e300;
            int bestPrev = -1;
            for (int i = 0; i < N; i++) {
                double cand = dp[t-1][i] + logA[i][j];
                if (cand > best) { best = cand; bestPrev = i; }
            }
            dp[t][j] = best + logB[j][obs[t]];
            back[t][j] = bestPrev;
        }
    }

    // Find best final state
    int bestState = 0;
    for (int j = 1; j < N; j++)
        if (dp[T-1][j] > dp[T-1][bestState]) bestState = j;

    double bestLogProb = dp[T-1][bestState];

    // Backtrace
    vector<int> path(T);
    path[T-1] = bestState;
    for (int t = T-1; t >= 1; t--)
        path[t-1] = back[t][path[t]];

    return {bestLogProb, path};
}

// Brute-force: enumerate all N^T state sequences, compute exact probability.
// Returns (best_log_prob, best_sequence).
pair<double, vector<int>> brute_force(const vector<vector<double>>& A,
                                      const vector<vector<double>>& B,
                                      const vector<double>& pi,
                                      const vector<int>& obs) {
    int N = (int)pi.size();
    int T = (int)obs.size();
    double best = -1e300;
    vector<int> bestSeq(T);

    vector<int> seq(T, 0);
    // enumerate via base-N counting
    long long total = 1;
    for (int t = 0; t < T; t++) total *= N;

    for (long long code = 0; code < total; code++) {
        long long c = code;
        for (int t = 0; t < T; t++) { seq[t] = c % N; c /= N; }

        double p = pi[seq[0]] * B[seq[0]][obs[0]];
        for (int t = 1; t < T; t++)
            p *= A[seq[t-1]][seq[t]] * B[seq[t]][obs[t]];

        double lp = log(p);
        if (lp > best) { best = lp; bestSeq = seq; }
    }
    return {best, bestSeq};
}

// Random HMM generator with normalized stochastic rows.
struct HMM {
    vector<vector<double>> A, B;
    vector<double> pi;
};

HMM randomHMM(mt19937& rng, int N, int M) {
    uniform_real_distribution<double> ud(0.5, 2.0);
    HMM h;
    h.A.assign(N, vector<double>(N));
    h.B.assign(N, vector<double>(M));
    h.pi.assign(N, 0.0);

    auto normalize = [](vector<double>& v) {
        double s = 0; for (double x : v) s += x;
        if (s == 0) { v[0] = 1.0; return; }
        for (double& x : v) x /= s;
    };

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) h.A[i][j] = ud(rng);
        normalize(h.A[i]);
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < M; j++) h.B[i][j] = ud(rng);
        normalize(h.B[i]);
    }
    for (int i = 0; i < N; i++) h.pi[i] = ud(rng);
    normalize(h.pi);

    return h;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int trials = 0, passed = 0;
    bool pathMismatch = false;

    mt19937 rng(12345);

    // Fixed hand-crafted test: classic "fair/loaded die" weather HMM
    {
        // 3 states, 4 observation symbols
        vector<vector<double>> A = {
            {0.5, 0.3, 0.2},
            {0.2, 0.5, 0.3},
            {0.3, 0.2, 0.5}
        };
        vector<vector<double>> B = {
            {0.4, 0.3, 0.2, 0.1},
            {0.1, 0.2, 0.3, 0.4},
            {0.25, 0.25, 0.25, 0.25}
        };
        vector<double> pi = {0.4, 0.35, 0.25};
        vector<int> obs = {0, 1, 2, 3, 0, 1, 2, 3, 0, 1};

        auto v = viterbi_log(A, B, pi, obs);
        auto b = brute_force(A, B, pi, obs);

        printf("Fixed test: viterbi_logProb = %.9f  brute_logProb = %.9f\n",
               v.first, b.first);
        bool ok = fabs(v.first - b.first) < 1e-6 && (v.second == b.second);
        trials++; if (ok) passed++;
        printf("  %s\n", ok ? "PASS" : "FAIL");
    }

    // Random HMMs: vary N (2..4 states) and T (1..6 obs length), M = 4
    for (int N = 2; N <= 4; N++) {
        for (int T = 1; T <= 6; T++) {
            for (int trial = 0; trial < 20; trial++) {
                HMM h = randomHMM(rng, N, 4);
                vector<int> obs(T);
                uniform_int_distribution<int> oi(0, 3);
                for (int t = 0; t < T; t++) obs[t] = oi(rng);

                auto v = viterbi_log(h.A, h.B, h.pi, obs);
                auto b = brute_force(h.A, h.B, h.pi, obs);

                trials++;
                bool lp = fabs(v.first - b.first) < 1e-6;
                bool pp = (v.second == b.second);
                if (lp && pp) passed++;
                else if (lp && !pp) pathMismatch = true;
            }
        }
    }

    printf("\n=== VERIFICATION SUMMARY ===\n");
    printf("Total trials: %d\n", trials);
    printf("Passed:       %d\n", passed);
    printf("Failed:       %d\n", trials - passed);
    printf("Path mismatches (same prob diff path): %s\n", pathMismatch ? "YES" : "NO");

    if (passed == trials) {
        printf("\nRESULT: ALL TESTS PASSED ✅\n");
        return 0;
    } else {
        printf("\nRESULT: FAILURES DETECTED ❌\n");
        return 1;
    }
}
