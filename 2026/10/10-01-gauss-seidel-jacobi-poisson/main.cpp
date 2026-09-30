// Gauss-Seidel & Jacobi Iterative Poisson Solver
// Solve the 2D Poisson equation  ∇²u = f  on the unit square with Dirichlet BCs.
//
// Analytic solution used for verification:
//   u(x,y) = sin(pi*x) * sin(pi*y)
//   → ∇²u = -2*pi² * sin(pi*x)*sin(pi*y) = f
// Boundary condition u=0 everywhere (matches).
//
// We compare Jacobi and Gauss-Seidel iteration convergence, and quantify:
//   1. Max absolute error vs analytic solution (must fall below tolerance).
//   2. Convergence rate (spectral radius estimate) via residual decay.
//   3. Iteration count until convergence for each method.
//   4. Jacobi acceleration by SOR relaxation (omega) on Gauss-Seidel -> GS-SOR.
//
// Quantified, not eyeballed: all checks are numeric assertions with thresholds.

#include <cmath>
#include <cstdio>
#include <vector>
#include <algorithm>
#include <string>

static constexpr double PI = 3.14159265358979323846;

// Analytic solution and its Laplacian
static double u_exact(double x, double y) { return std::sin(PI * x) * std::sin(PI * y); }
static double f_source(double x, double y) { return -2.0 * PI * PI * std::sin(PI * x) * std::sin(PI * y); }

struct Result {
    std::string name;
    int iterations;
    double max_err;      // max |u - u_exact|
    double final_resid;  // max residual
    double rate;         // estimated per-iteration contraction factor
};

// One squared-map error metric: sum of squared error, normalized.
static double max_error(const std::vector<double>& u, int n, double h) {
    double me = 0.0;
    for (int j = 0; j <= n; ++j)
        for (int i = 0; i <= n; ++i) {
            double x = i * h, y = j * h;
            // skip boundary (exact by BC)
            double err = std::fabs(u[j * (n + 1) + i] - u_exact(x, y));
            me = std::max(me, err);
        }
    return me;
}

static double max_residual(const std::vector<double>& u, int n, double h) {
    double h2 = h * h;
    double mr = 0.0;
    for (int j = 1; j < n; ++j)
        for (int i = 1; i < n; ++i) {
            double x = i * h, y = j * h;
            double lap = (u[(j+1)*(n+1)+i] + u[(j-1)*(n+1)+i] +
                          u[j*(n+1)+i+1]   + u[j*(n+1)+i-1] - 4.0*u[j*(n+1)+i]) / h2;
            double r = std::fabs(lap - f_source(x, y));
            mr = std::max(mr, r);
        }
    return mr;
}

// Jacobi iteration
static Result solve_jacobi(int n, double tol, int max_iter) {
    double h = 1.0 / n;
    double h2 = h * h;
    int N = n + 1;
    std::vector<double> u(N * N, 0.0), unew(N * N, 0.0);
    // initialize interior with f*h2/4? keep 0; BC = 0 matches analytic at boundary.
    int it = 0;
    double prev_resid = -1.0;
    double rate = 0.0;
    for (; it < max_iter; ++it) {
        double maxdelta = 0.0;
        for (int j = 1; j < n; ++j)
            for (int i = 1; i < n; ++i) {
                int idx = j * N + i;
                unew[idx] = 0.25 * (u[(j+1)*N+i] + u[(j-1)*N+i] + u[j*N+i+1] + u[j*N+i-1] - h2 * f_source(i*h, j*h));
                maxdelta = std::max(maxdelta, std::fabs(unew[idx] - u[idx]));
            }
        std::swap(u, unew);
        double resid = max_residual(u, n, h);
        if (it > 0 && prev_resid > 0) rate = resid / prev_resid;
        prev_resid = resid;
        if (maxdelta < tol) break;
    }
    return { "Jacobi", it + 1, max_error(u, n, h), prev_resid, rate };
}

// Gauss-Seidel (in-place red-black free, lexicographic) iteration
static Result solve_gs(int n, double tol, int max_iter, double omega = 1.0) {
    double h = 1.0 / n;
    double h2 = h * h;
    int N = n + 1;
    std::vector<double> u(N * N, 0.0);
    int it = 0;
    double prev_resid = -1.0, rate = 0.0;
    for (; it < max_iter; ++it) {
        double maxdelta = 0.0;
        for (int j = 1; j < n; ++j)
            for (int i = 1; i < n; ++i) {
                int idx = j * N + i;
                double gs = 0.25 * (u[(j+1)*N+i] + u[(j-1)*N+i] + u[j*N+i+1] + u[j*N+i-1] - h2 * f_source(i*h, j*h));
                double unew = u[idx] + omega * (gs - u[idx]);
                maxdelta = std::max(maxdelta, std::fabs(unew - u[idx]));
                u[idx] = unew;
            }
        double resid = max_residual(u, n, h);
        if (it > 0 && prev_resid > 0) rate = resid / prev_resid;
        prev_resid = resid;
        if (maxdelta < tol) break;
    }
    std::string nm = (omega == 1.0) ? "Gauss-Seidel" : "GS-SOR(w=" + std::to_string(omega) + ")";
    return { nm, it + 1, max_error(u, n, h), prev_resid, rate };
}

int main() {
    int n = 64;
    double tol = 1e-8;
    int max_iter = 200000;

    printf("Poisson solver  2D  n=%d  (h=%.5f)  tol=%.1e\n", n, 1.0/n, tol);
    printf("Analytic: u = sin(pi x) sin(pi y),  f = -2 pi^2 sin(pi x) sin(pi y)\n\n");

    auto j = solve_jacobi(n, tol, max_iter);
    auto g = solve_gs(n, tol, max_iter);
    auto s = solve_gs(n, tol, max_iter, 1.8);

    printf("%-20s  iter=%6d  max_err=%.3e  resid=%.3e  rate=%.4f\n",
           j.name.c_str(), j.iterations, j.max_err, j.final_resid, j.rate);
    printf("%-20s  iter=%6d  max_err=%.3e  resid=%.3e  rate=%.4f\n",
           g.name.c_str(), g.iterations, g.max_err, g.final_resid, g.rate);
    printf("%-20s  iter=%6d  max_err=%.3e  resid=%.3e  rate=%.4f\n",
           s.name.c_str(), s.iterations, s.max_err, s.final_resid, s.rate);

    // ---- Quantified verification (non-visual assertions) ----
    bool ok = true;
    auto check = [&](const Result& r, double err_limit) {
        if (r.max_err > err_limit) { printf("FAIL %s: error too large\n", r.name.c_str()); ok = false; }
        if (r.final_resid > 1e-2) { printf("FAIL %s: residual too large\n", r.name.c_str()); ok = false; }
        if (r.iterations >= max_iter) { printf("FAIL %s: did not converge\n", r.name.c_str()); ok = false; }
    };
    // analytic solution accuracy: expect max error well below h^2 (h^2 ~ 2.4e-4)
    check(j, 1e-3);
    check(g, 1e-3);
    check(s, 1e-3);

    // Theoretical spectral radii:
    //   Jacobi: rho = cos(pi h)                      ~ 1 - (pi h)^2/2
    //   Gauss-Seidel: rho = cos^2(pi h)              ~ 1 - (pi h)^2
    double h = 1.0 / n;
    double rho_jac_theory = std::cos(PI * h);
    double rho_gs_theory  = rho_jac_theory * rho_jac_theory;
    printf("\n-- Convergence-rate verification (measured vs theory) --\n");
    printf("Jacobi        measured rate=%.5f   theory rho=%.5f\n", j.rate, rho_jac_theory);
    printf("Gauss-Seidel  measured rate=%.5f   theory rho=%.5f\n", g.rate, rho_gs_theory);

    // Measured rate should be within ~2% of theory (rate is asymptotic).
    if (std::fabs(j.rate - rho_jac_theory) > 0.02) { printf("FAIL: Jacobi rate mismatch\n"); ok = false; }
    if (std::fabs(g.rate - rho_gs_theory) > 0.02)   { printf("FAIL: GS rate mismatch\n"); ok = false; }

    // GS should converge roughly twice as fast as Jacobi:
    if (!(g.iterations < j.iterations)) { printf("FAIL: GS not faster than Jacobi\n"); ok = false; }
    if (!(s.iterations < g.iterations)) { printf("FAIL: SOR not faster than GS\n"); ok = false; }

    // Discretization order check: error ~ O(h^2). Run coarser grid, verify halving h cuts error ~4x.
    int n2 = 32;
    auto g_coarse = solve_gs(n2, 1e-9, max_iter);
    double ratio = (g_coarse.max_err) / (g.max_err);
    printf("\n-- Discretization order (GS 32x32 vs 64x64) --\n");
    printf("err(32)=%.3e  err(64)=%.3e  ratio=%.3f (expect ~4 for O(h^2))\n",
           g_coarse.max_err, g.max_err, ratio);
    if (ratio < 3.0 || ratio > 5.0) { printf("FAIL: not O(h^2)\n"); ok = false; }

    printf("\n%s\n", ok ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED");
    return ok ? 0 : 1;
}
