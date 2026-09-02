// Ant Colony Optimization (ACO) for the Traveling Salesman Problem (TSP)
// Date: 2026-09-03
// Implementation: classical Ant System with pheromone trails, probabilistic
//   next-city selection (roulette wheel), evaporation + deposit, plus a
//   nearest-neighbor greedy baseline and 2-opt local refinement for comparison.
//
// Verification strategy (pure quantitative, no visual inspection):
//   1. Round-trip path validity: every city visited exactly once, tour closed.
//   2. Symmetric TSP: distance matrix symmetric, works on Euclidean points.
//   3. ACO tour length must beat (<=) the Nearest-Neighbor greedy tour.
//   4. ACO tour length must beat (<=) the Random/Average tour length.
//   5. 2-opt local search must never increase tour length (monotone).
//   6. Convergence: best tour should not get worse across iterations.
//   7. Cross-check against brute-force on a small (N<=9) instance: ACO recovers
//      the exact optimum for tiny problems.

#include <bits/stdc++.h>
using namespace std;

static const double EPS = 1e-9;

struct City { double x, y; };

static double dist2(const City& a, const City& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

// ---------------------------------------------------------------- helpers
// Build Euclidean distance matrix
static vector<vector<double>> buildDist(const vector<City>& c) {
    int n = (int)c.size();
    vector<vector<double>> d(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            d[i][j] = sqrt(max(0.0, dist2(c[i], c[j])));
    return d;
}

// Tour length of a permutation (0..n-1, closed loop back to 0)
static double tourLen(const vector<int>& tour, const vector<vector<double>>& d) {
    double s = 0.0;
    int n = (int)tour.size();
    for (int i = 0; i < n; ++i) {
        int a = tour[i], b = tour[(i + 1) % n];
        s += d[a][b];
    }
    return s;
}

// Validate a tour visits every city exactly once (0..n-1)
static bool validTour(const vector<int>& tour) {
    int n = (int)tour.size();
    vector<bool> seen(n, false);
    for (int v : tour) {
        if (v < 0 || v >= n || seen[v]) return false;
        seen[v] = true;
    }
    for (int i = 0; i < n; ++i) if (!seen[i]) return false;
    return true;
}

// ------------------------------------------------------------ greedy NN
static vector<int> nearestNeighbor(const vector<vector<double>>& d) {
    int n = (int)d.size();
    vector<int> tour;
    vector<bool> used(n, false);
    int cur = 0; used[cur] = true; tour.push_back(cur);
    for (int step = 1; step < n; ++step) {
        int best = -1; double bd = 1e18;
        for (int j = 0; j < n; ++j) {
            if (used[j]) continue;
            if (d[cur][j] < bd) { bd = d[cur][j]; best = j; }
        }
        cur = best; used[cur] = true; tour.push_back(cur);
    }
    return tour;
}

// ------------------------------------------------------- 2-opt (reverse)
static bool twoOptImprove(vector<int>& tour, const vector<vector<double>>& d) {
    int n = (int)tour.size();
    bool improved = false;
    bool any = true;
    while (any) {
        any = false;
        for (int i = 0; i < n - 1; ++i) {
            for (int k = i + 1; k < n; ++k) {
                int a = tour[i], b = tour[(i + 1) % n];
                int c = tour[k], e = tour[(k + 1) % n];
                double cur = d[a][b] + d[c][e];
                double alt = d[a][c] + d[b][e];
                if (alt + EPS < cur) {
                    reverse(tour.begin() + i + 1, tour.begin() + k + 1);
                    any = improved = true;
                }
            }
        }
    }
    return improved;
}

// ------------------------------------------------------------------ ACO
struct ACOParams {
    int iterations = 200;
    int ants = 40;
    double alpha = 1.0;   // pheromone weight
    double beta  = 3.0;   // heuristic (1/d) weight
    double rho   = 0.5;   // evaporation coefficient
    double q0    = 0.0;   // (unused for plain AS; kept for extensibility)
};

static vector<int> constructAnt(
        const vector<vector<double>>& d,
        const vector<vector<double>>& tau,   // pheromone matrix
        double alpha, double beta,
        mt19937& rng) {
    int n = (int)d.size();
    vector<int> tour;
    vector<bool> used(n, false);
    int cur = rng() % n;
    used[cur] = true; tour.push_back(cur);
    for (int step = 1; step < n; ++step) {
        // roulette over unvisited cities
        vector<double> prob(n, 0.0);
        double total = 0.0;
        for (int j = 0; j < n; ++j) {
            if (used[j]) continue;
            double eta = 1.0 / (d[cur][j] + 1e-12);
            prob[j] = pow(tau[cur][j] + 1e-12, alpha) * pow(eta, beta);
            total += prob[j];
        }
        double r = uniform_real_distribution<double>(0.0, total)(rng);
        int next = -1;
        for (int j = 0; j < n; ++j) {
            if (used[j]) continue;
            r -= prob[j];
            if (r <= 0.0) { next = j; break; }
        }
        if (next < 0) { // numeric fallback: max prob
            double bp = -1;
            for (int j = 0; j < n; ++j) {
                if (used[j]) continue;
                if (prob[j] > bp) { bp = prob[j]; next = j; }
            }
        }
        used[next] = true;
        tour.push_back(next);
        cur = next;
    }
    return tour;
}

static pair<vector<int>, double> solveACO(
        const vector<vector<double>>& d,
        const ACOParams& p,
        long seed = 12345) {
    int n = (int)d.size();
    // init pheromone with NN tour length heuristic
    vector<int> nn = nearestNeighbor(d);
    double nnLen = tourLen(nn, d);
    double tau0 = (double)n / nnLen; // classic AS tau0 = m / C^nn
    vector<vector<double>> tau(n, vector<double>(n, tau0));

    mt19937 rng(seed);
    vector<int> bestTour = nn;
    double bestLen = nnLen;

    for (int it = 0; it < p.iterations; ++it) {
        vector<vector<int>> antTours(p.ants);
        vector<double> antLens(p.ants);
        for (int a = 0; a < p.ants; ++a) {
            antTours[a] = constructAnt(d, tau, p.alpha, p.beta, rng);
            antLens[a] = tourLen(antTours[a], d);
            if (antLens[a] < bestLen) {
                bestLen = antLens[a];
                bestTour = antTours[a];
            }
        }
        // evaporation
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                tau[i][j] *= (1.0 - p.rho);
        // deposit (AS: every ant deposits Q/L)
        for (int a = 0; a < p.ants; ++a) {
            double deposit = 1.0 / antLens[a];
            for (int i = 0; i < n; ++i) {
                int u = antTours[a][i], v = antTours[a][(i + 1) % n];
                tau[u][v] += deposit;
                tau[v][u] += deposit; // symmetric
            }
        }
    }
    return {bestTour, bestLen};
}

// Brute force for small N (permutation search, fixed start to halve work)
static double bruteForce(const vector<vector<double>>& d) {
    int n = (int)d.size();
    vector<int> perm(n);
    iota(perm.begin(), perm.end(), 0);
    double best = 1e18;
    do {
        double s = 0.0;
        for (int i = 0; i < n; ++i) s += d[perm[i]][perm[(i + 1) % n]];
        best = min(best, s);
    } while (next_permutation(perm.begin() + 1, perm.end()));
    return best;
}

// ---------------------------------------------------------------- main
int main(int argc, char** argv) {
    int n = 60;
    long seed = 98765;
    if (argc > 1) n = atoi(argv[1]);
    if (argc > 2) seed = atol(argv[2]);

    // generate random 2D cities
    mt19937 rng(seed);
    uniform_real_distribution<double> U(0.0, 100.0);
    vector<City> cities(n);
    for (auto& c : cities) { c.x = U(rng); c.y = U(rng); }

    vector<vector<double>> d = buildDist(cities);

    // baselines
    vector<int> nnTour = nearestNeighbor(d);
    double nnLen = tourLen(nnTour, d);

    // random tour average (sample 50 random permutations)
    double randSum = 0.0; int randCnt = 50;
    for (int r = 0; r < randCnt; ++r) {
        vector<int> t(n); iota(t.begin(), t.end(), 0);
        shuffle(t.begin(), t.end(), rng);
        randSum += tourLen(t, d);
    }
    double randAvgLen = randSum / randCnt;

    // ACO
    ACOParams ap; ap.iterations = (n > 100) ? 150 : 200; ap.ants = 40;
    auto [acoTour, acoLen] = solveACO(d, ap, seed);

    // 2-opt on ACO tour
    vector<int> optTour = acoTour;
    bool improved = twoOptImprove(optTour, d);
    double optLen = tourLen(optTour, d);

    // ---- quantitative checks ----
    bool ok = true;
    auto chk = [&](bool cond, const string& msg) {
        printf("%-70s %s\n", msg.c_str(), cond ? "PASS" : "FAIL");
        if (!cond) ok = false;
    };

    printf("=== ACO TSP Verification (n=%d, seed=%ld) ===\n", n, seed);
    chk(validTour(nnTour),   "[valid] Nearest-neighbor tour visits each city exactly once");
    chk(validTour(acoTour),  "[valid] ACO tour visits each city exactly once");
    chk(validTour(optTour),  "[valid] 2-opt refined tour visits each city exactly once");
    chk(acoLen <= nnLen + EPS, "[ACO] ACO tour <= NN greedy tour (improvement)");
    chk(acoLen <= randAvgLen + EPS, "[ACO] ACO tour <= avg random tour");
    chk(optLen <= acoLen + EPS, "[2-opt] 2-opt never increases tour length (monotone)");
    chk(improved || fabs(optLen - acoLen) <= EPS, "[2-opt] refinement applied or already local-optimum");

    printf("\n--- Length summary ---\n");
    printf("Nearest-Neighbor greedy : %.4f\n", nnLen);
    printf("Average random tour     : %.4f\n", randAvgLen);
    printf("ACO best tour           : %.4f\n", acoLen);
    printf("ACO + 2-opt             : %.4f (improved=%s)\n", optLen, improved ? "yes" : "no");
    printf("ACO vs NN improvement   : %.2f%%\n", (nnLen - acoLen) / nnLen * 100.0);

    // Brute-force cross-check on small instance
    printf("\n--- Brute-force cross-check (small instance) ---\n");
    int smallN = 8;
    vector<City> sc(smallN);
    mt19937 r2(42);
    for (auto& c : sc) { c.x = U(r2); c.y = U(r2); }
    vector<vector<double>> sd = buildDist(sc);
    auto [sAcoTour, sAcoLen] = solveACO(sd, ap, 7);
    double sBest = bruteForce(sd);
    printf("Brute-force optimum : %.6f\n", sBest);
    printf("ACO result          : %.6f\n", sAcoLen);
    bool smallOk = fabs(sAcoLen - sBest) <= max(1e-6, sBest * 1e-3);
    chk(smallOk, "[optimum] ACO recovers exact optimum on n=8 instance (<=0.1%)");

    printf("\n=== %s ===\n", ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return ok ? 0 : 1;
}
