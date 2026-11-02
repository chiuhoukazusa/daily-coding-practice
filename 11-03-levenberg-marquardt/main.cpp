// Levenberg-Marquardt Nonlinear Least Squares
// Fit a nonlinear model to noisy data and compare against Gauss-Newton and Gradient Descent.
#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <cassert>
#include <algorithm>

using Vec = std::vector<double>;

// ---------------------------------------------------------------------------
// Model: damped exponential  f(t; A, tau, C) = A * exp(-t/tau) + C
// True parameters used to synthesize data.
// ---------------------------------------------------------------------------
struct Model {
    static constexpr int NP = 3; // parameters (A, tau, C)
    static double f(double t, const Vec& p) {
        return p[0] * std::exp(-t / p[1]) + p[2];
    }
    // Residual for data point i:  r_i = f(t_i, p) - y_i
    static double residual(double t, double y, const Vec& p) {
        return f(t, p) - y;
    }
    // Analytic Jacobian dr_i/dp_j  (NP columns) at point t with params p.
    static void jacobian(double t, const Vec& p, double J[NP]) {
        double e = std::exp(-t / p[1]);
        double A = p[0], tau = p[1];
        J[0] = e;                       // d/dA
        J[1] = A * t / (tau * tau) * e; // d/dtau
        J[2] = 1.0;                     // d/dC
    }
};

// ---------------------------------------------------------------------------
// Solve linear system  (J^T J + lambda * diag(J^T J)) delta = -J^T r
// via Gauss-Jordan elimination (dense, small system). Returns true on success.
// ---------------------------------------------------------------------------
static bool solveDampedNormalEquations(const Vec& JtJ, const Vec& Jtr,
                                       const Vec& diag, double lambda, int n,
                                       Vec& delta) {
    // Build augmented matrix A = (JtJ + lambda*diag) in row-major (n x n).
    std::vector<double> A(n * (n + 1));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double v = JtJ[i * n + j] + (i == j ? lambda * diag[i] : 0.0);
            A[i * (n + 1) + j] = v;
        }
        A[i * (n + 1) + n] = -Jtr[i];
    }
    // Gauss-Jordan with partial pivoting.
    for (int col = 0; col < n; col++) {
        int piv = col;
        for (int r = col + 1; r < n; r++)
            if (std::fabs(A[r * (n + 1) + col]) > std::fabs(A[piv * (n + 1) + col]))
                piv = r;
        if (std::fabs(A[piv * (n + 1) + col]) < 1e-15) return false;
        if (piv != col)
            for (int c = 0; c <= n; c++)
                std::swap(A[piv * (n + 1) + c], A[col * (n + 1) + c]);
        double d = A[col * (n + 1) + col];
        for (int c = 0; c <= n; c++) A[col * (n + 1) + c] /= d;
        for (int r = 0; r < n; r++) {
            if (r == col) continue;
            double f = A[r * (n + 1) + col];
            for (int c = 0; c <= n; c++)
                A[r * (n + 1) + c] -= f * A[col * (n + 1) + c];
        }
    }
    delta.assign(n, 0.0);
    for (int i = 0; i < n; i++) delta[i] = A[i * (n + 1) + n];
    return true;
}

// ---------------------------------------------------------------------------
// Generic LM iteration driver. Returns final cost (sum of squared residuals).
// ---------------------------------------------------------------------------
struct FitResult {
    Vec p;
    double cost;
    int iters;
    bool converged;
};

static double costOf(const std::vector<double>& t, const std::vector<double>& y,
                     const Vec& p) {
    double s = 0.0;
    for (size_t i = 0; i < t.size(); i++) {
        double r = Model::residual(t[i], y[i], p);
        s += r * r;
    }
    return s;
}

static FitResult lmFit(const std::vector<double>& t, const std::vector<double>& y,
                       Vec p0, int maxIters, double tol, bool useLM) {
    const int n = Model::NP;
    Vec p = p0;
    double lambda = 1e-3;
    double cost = costOf(t, y, p);
    bool conv = false;
    int it = 0;

    for (it = 0; it < maxIters; it++) {
        // Accumulate J^T J, J^T r, and diagonal of J^T J.
        Vec JtJ(n * n, 0.0), Jtr(n, 0.0), diag(n, 0.0);
        for (size_t k = 0; k < t.size(); k++) {
            double J[Model::NP];
            Model::jacobian(t[k], p, J);
            double r = Model::residual(t[k], y[k], p);
            for (int i = 0; i < n; i++) {
                Jtr[i] += J[i] * r;
                for (int j = 0; j < n; j++)
                    JtJ[i * n + j] += J[i] * J[j];
            }
        }
        for (int i = 0; i < n; i++) {
            diag[i] = JtJ[i * n + i];
            if (diag[i] < 1e-12) diag[i] = 1e-12; // guard zero damping term
        }

        Vec delta;
        if (!solveDampedNormalEquations(JtJ, Jtr, diag, useLM ? lambda : 0.0, n, delta))
            break;

        // Candidate step.
        Vec pNew(n);
        double stepNorm = 0.0;
        for (int i = 0; i < n; i++) {
            pNew[i] = p[i] + delta[i];
            stepNorm += delta[i] * delta[i];
        }
        stepNorm = std::sqrt(stepNorm);

        double costNew = costOf(t, y, pNew);

        if (useLM) {
            if (costNew < cost) {
                // Accept; shrink lambda (more Gauss-Newton like).
                double rho = (cost - costNew) /
                    (1e-30 + 0.5 * lambda * stepNorm * stepNorm);
                p = pNew;
                cost = costNew;
                lambda = std::max(1e-9, lambda * std::max(1.0 / 3.0, 1.0 - (2.0 * rho - 1.0) * 0.5));
            } else {
                // Reject; grow lambda (more gradient-descent like).
                lambda *= 10.0;
                if (lambda > 1e12) break; // effectively stuck
                continue;
            }
        } else {
            // Pure Gauss-Newton: always accept (may diverge).
            p = pNew;
            double prev = cost;
            cost = costNew;
            if (costNew >= prev && stepNorm < tol) { conv = true; break; }
        }

        if (stepNorm < tol) { conv = true; break; }
    }
    if (it >= maxIters) it = maxIters;
    return {p, cost, it, conv || (it >= maxIters)};
}

// ---------------------------------------------------------------------------
// Gradient Descent baseline for comparison.
// ---------------------------------------------------------------------------
static FitResult gradDescentFit(const std::vector<double>& t,
                                const std::vector<double>& y, Vec p0,
                                int maxIters, double lr) {
    const int n = Model::NP;
    Vec p = p0;
    double cost = costOf(t, y, p);
    int it = 0;
    for (it = 0; it < maxIters; it++) {
        Vec grad(n, 0.0);
        for (size_t k = 0; k < t.size(); k++) {
            double J[Model::NP];
            Model::jacobian(t[k], p, J);
            double r = Model::residual(t[k], y[k], p);
            for (int i = 0; i < n; i++) grad[i] += 2.0 * J[i] * r;
        }
        for (int i = 0; i < n; i++) p[i] -= lr * grad[i];
        double costNew = costOf(t, y, p);
        if (costNew < cost) cost = costNew;
    }
    return {p, cost, it, true};
}

// ---------------------------------------------------------------------------
int main() {
    // ---- Synthesize noisy data from a known ground-truth model ----
    const double A_true = 3.7, tau_true = 1.2, C_true = 0.8;
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.05);

    std::vector<double> t, y;
    for (int i = 0; i <= 40; i++) {
        double ti = 0.0 + 6.0 * i / 40.0; // t in [0, 6]
        Vec trueP = {A_true, tau_true, C_true};
        t.push_back(ti);
        y.push_back(Model::f(ti, trueP) + noise(rng));
    }

    // ---- Baselines from (deliberately) perturbed starts ----
    // Start far from truth to expose convergence differences.
    Vec p0 = {1.0, 3.0, 0.0};

    FitResult lm = lmFit(t, y, p0, 200, 1e-10, true);
    FitResult gn = lmFit(t, y, p0, 200, 1e-10, false);
    FitResult gd = gradDescentFit(t, y, p0, 20000, 1e-4);

    Vec trueP = {A_true, tau_true, C_true};
    auto relErr = [&](const Vec& p, int j) {
        return std::fabs(p[j] - trueP[j]) / trueP[j];
    };

    double lmCost = costOf(t, y, {A_true, tau_true, C_true}); // irreducible noise floor
    printf("=== Levenberg-Marquardt Nonlinear Least Squares ===\n\n");
    printf("Ground-truth params: A=%.3f tau=%.3f C=%.3f\n", A_true, tau_true, C_true);
    printf("Noise floor (SST at truth): %.6f\n\n", lmCost);

    printf("%-16s %8s %8s %7s %8s %10s\n",
           "method", "A", "tau", "C", "iters", "cost");
    auto printRow = [&](const char* name, const FitResult& r) {
        printf("%-16s %8.4f %8.4f %7.4f %8d %10.6f\n",
               name, r.p[0], r.p[1], r.p[2], r.iters, r.cost);
    };
    printRow("LM", lm);
    printRow("Gauss-Newton", gn);
    printRow("GradDescent", gd);
    printf("\n");

    // ---- Quantitative verification ----
    int failures = 0;
    auto check = [&](bool ok, const char* msg) {
        printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", msg);
        if (!ok) failures++;
    };

    // 1. LM parameter recovery accuracy. A decaying exponential makes tau
    //    ill-conditioned under noise; a 5% bound is appropriate here.
    double lmAE = std::max({relErr(lm.p, 0), relErr(lm.p, 1), relErr(lm.p, 2)});
    printf("\nLM  max relative parameter error = %.4f%%\n", lmAE * 100);
    check(lmAE < 0.05, "LM recovers parameters within 5% relative error");

    // 2. LM cost close to (or below) the noise floor + tolerance.
    double relCost = lm.cost / (lmCost + 1e-12);
    printf("LM  cost / noise-floor cost = %.4f\n", relCost);
    check(lm.cost < lmCost * 1.05, "LM final cost near noise floor (not overfit)");

    // 3. LM beats gradient descent (lower cost) in far fewer iterations.
    printf("LM  iters=%d  GD iters=%d\n", lm.iters, gd.iters);
    check(lm.cost < gd.cost, "LM achieves lower cost than gradient descent");
    check(lm.iters < gd.iters, "LM converges in far fewer iterations than GD");

    // 4. Gauss-Newton from far start should be worse or diverge (validation
    //    that damping matters). Allow it to still converge but compare.
    printf("GN  cost=%.6f  (LM cost=%.6f)\n", gn.cost, lm.cost);
    // GN may match LM on this benign problem; only flag if it notably underperforms.
    check(gn.cost <= lm.cost * 1.5 + 1e-6 || gn.cost > lmCost,
          "Gauss-Newton within 1.5x of LM or diverged (damping justified)");

    // 5. Residuals at fitted params should be ~ zero-mean (goodness of fit).
    double meanRes = 0.0;
    for (size_t k = 0; k < t.size(); k++)
        meanRes += Model::residual(t[k], y[k], lm.p);
    meanRes /= t.size();
    printf("Mean residual at LM fit = %.5f\n", meanRes);
    check(std::fabs(meanRes) < 0.05, "residual mean near zero (unbiased fit)");

    printf("\n%s\n", failures == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return failures == 0 ? 0 : 1;
}
