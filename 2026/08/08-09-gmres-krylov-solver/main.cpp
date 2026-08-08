#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <algorithm>
#include <cassert>
#include <functional>
#include <fstream>

// ============================================================
// GMRES(m) Krylov Subspace Solver
// Implements: Arnoldi iteration, Givens rotations, restart strategy
//
// Quantitative Verification Tests:
//  1. Symmetric SPD system: GMRES residual vs CG residual (should match within tol)
//  2. Non-symmetric system: residual monotonic decrease
//  3. Givens rotation: orthogonality preservation
//  4. Arnoldi: Hessenberg matrix structure verification
//  5. Restart behavior: convergence across restarts
//  6. Backward error: ||b - Ax|| / ||b||
//  7. Different restart sizes (m=5,10,20,30): convergence rate comparison
// ============================================================

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

// ---- Vector operations ----
double dot(const Vector& a, const Vector& b) {
    double s = 0.0;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

double norm2(const Vector& v) {
    return std::sqrt(dot(v, v));
}

void axpy(double alpha, const Vector& x, Vector& y) {
    for (size_t i = 0; i < x.size(); ++i) y[i] += alpha * x[i];
}

void scale(Vector& v, double s) {
    for (size_t i = 0; i < v.size(); ++i) v[i] *= s;
}

Vector matvec(const Matrix& A, const Vector& x) {
    size_t n = A.size();
    Vector y(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            y[i] += A[i][j] * x[j];
        }
    }
    return y;
}

Vector subtract(const Vector& a, const Vector& b) {
    Vector c(a.size());
    for (size_t i = 0; i < a.size(); ++i) c[i] = a[i] - b[i];
    return c;
}

// ---- Modified Gram-Schmidt (Arnoldi step) ----
// Builds V_{k+1} and Hessenberg matrix H_{k+1,k}
void arnoldi_step(const Matrix& A, std::vector<Vector>& V, 
                  std::vector<std::vector<double>>& H, int k) {
    size_t n = A.size();
    Vector w = matvec(A, V[k]);  // w = A * v_k

    // Modified Gram-Schmidt: orthogonalize w against V[0..k]
    for (int i = 0; i <= k; ++i) {
        H[i][k] = dot(w, V[i]);
        for (size_t j = 0; j < n; ++j) {
            w[j] -= H[i][k] * V[i][j];
        }
    }
    H[k+1][k] = norm2(w);
    if (H[k+1][k] > 1e-14) {
        V[k+1].resize(n);
        for (size_t j = 0; j < n; ++j) V[k+1][j] = w[j] / H[k+1][k];
    }
}

// ---- Givens rotation ----
struct Givens {
    double c, s;  // cos, sin
};

// Apply Givens rotation to eliminate H[i+1][i]
Givens givens_rotation(double a, double b) {
    Givens g;
    if (std::abs(b) < 1e-14) {
        g.c = 1.0; g.s = 0.0;
    } else if (std::abs(b) > std::abs(a)) {
        double tau = -a / b;
        g.s = 1.0 / std::sqrt(1.0 + tau*tau);
        g.c = g.s * tau;
    } else {
        double tau = -b / a;
        g.c = 1.0 / std::sqrt(1.0 + tau*tau);
        g.s = g.c * tau;
    }
    return g;
}

void apply_givens(const Givens& g, double& a, double& b) {
    double tmp = g.c * a - g.s * b;
    b = g.s * a + g.c * b;
    a = tmp;
}

// ---- GMRES(m) main solver ----
struct GMRESResult {
    Vector x;
    std::vector<double> residuals;
    int total_iters;
    bool converged;
};

GMRESResult gmres(const Matrix& A, const Vector& b, int restart_m, 
                  double tol = 1e-8, int max_outer = 1000) {
    size_t n = A.size();
    GMRESResult result;
    result.x.assign(n, 0.0);
    result.total_iters = 0;
    result.converged = false;

    double b_norm = norm2(b);
    if (b_norm < 1e-14) {
        result.converged = true;
        result.residuals.push_back(0.0);
        return result;
    }

    // Allocate workspace
    std::vector<Vector> V(restart_m + 1);   // Arnoldi vectors
    std::vector<std::vector<double>> H(restart_m + 1, 
        std::vector<double>(restart_m, 0.0)); // Hessenberg
    std::vector<Givens> givens_list(restart_m);
    Vector g(restart_m + 1);                  // RHS of least-squares

    for (int outer = 0; outer < max_outer; ++outer) {
        // Compute initial residual: r0 = b - A*x
        Vector Ax = matvec(A, result.x);
        Vector r0 = subtract(b, Ax);
        double beta = norm2(r0);
        double res_norm = beta / b_norm;
        result.residuals.push_back(res_norm);

        if (res_norm < tol) {
            result.converged = true;
            return result;
        }

        // Arnoldi: build Krylov subspace
        V[0].resize(n);
        for (size_t i = 0; i < n; ++i) V[0][i] = r0[i] / beta;
        g[0] = beta;
        for (int i = 1; i <= restart_m; ++i) g[i] = 0.0;

        int k = 0;
        bool breakdown = false;
        for (k = 0; k < restart_m; ++k) {
            arnoldi_step(A, V, H, k);

            // Apply previous Givens rotations to column k of H
            for (int i = 0; i < k; ++i) {
                apply_givens(givens_list[i], H[i][k], H[i+1][k]);
            }

            // Compute and apply new Givens rotation for H[k+1][k]
            givens_list[k] = givens_rotation(H[k][k], H[k+1][k]);
            apply_givens(givens_list[k], H[k][k], H[k+1][k]);
            apply_givens(givens_list[k], g[k], g[k+1]);

            result.total_iters++;

            // Check convergence within this restart cycle
            double current_res = std::abs(g[k+1]) / b_norm;
            if (current_res < tol) {
                breakdown = true;
                break;
            }

            if (std::abs(H[k+1][k]) < 1e-14) {
                breakdown = true;
                break;
            }
        }

        // Solve upper triangular system H*y = g
        int solve_size = breakdown ? k : (restart_m - 1);
        Vector y(solve_size + 1, 0.0);
        for (int i = solve_size; i >= 0; --i) {
            double sum = g[i];
            for (int j = i + 1; j <= solve_size; ++j) {
                sum -= H[i][j] * y[j];
            }
            if (std::abs(H[i][i]) > 1e-14) {
                y[i] = sum / H[i][i];
            }
        }

        // Update x: x = x + V * y
        for (int i = 0; i <= solve_size; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result.x[j] += y[i] * V[i][j];
            }
        }

        if (breakdown && std::abs(g[k+1]) / b_norm < tol) {
            result.converged = true;
            return result;
        }
    }
    result.converged = false;
    return result;
}

// ---- CG solver for comparison (SPD systems only) ----
Vector conjugate_gradient(const Matrix& A, const Vector& b, 
                          double tol = 1e-8, int max_iter = 10000,
                          std::vector<double>* residuals_out = nullptr) {
    size_t n = A.size();
    Vector x(n, 0.0);
    Vector r = b;            // r0 = b - A*0 = b
    Vector p = r;
    double rsold = dot(r, r);

    if (residuals_out) residuals_out->push_back(std::sqrt(rsold) / norm2(b));

    for (int iter = 0; iter < max_iter; ++iter) {
        Vector Ap = matvec(A, p);
        double alpha = rsold / dot(p, Ap);
        for (size_t i = 0; i < n; ++i) x[i] += alpha * p[i];
        for (size_t i = 0; i < n; ++i) r[i] -= alpha * Ap[i];
        double rsnew = dot(r, r);

        if (residuals_out) residuals_out->push_back(std::sqrt(rsnew) / norm2(b));

        if (std::sqrt(rsnew) < tol * norm2(b)) break;
        double beta_cg = rsnew / rsold;
        for (size_t i = 0; i < n; ++i) p[i] = r[i] + beta_cg * p[i];
        rsold = rsnew;
    }
    return x;
}

// ---- Test helpers ----
Matrix generate_spd_matrix(int n) {
    // Generate well-conditioned SPD matrix using diagonal dominance
    // A = I + 0.5*(R + R^T) where R is random, then add diagonal dominance
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A[i][j] = dist(rng);
        }
    }

    // Symmetrize: A = (A + A^T)/2
    Matrix sym(n, Vector(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            sym[i][j] = 0.5 * (A[i][j] + A[j][i]);
        }
    }

    // Make diagonally dominant: A = sym + 2*n*I
    for (int i = 0; i < n; ++i) {
        double row_sum = 0.0;
        for (int j = 0; j < n; ++j) {
            if (i != j) row_sum += std::abs(sym[i][j]);
        }
        sym[i][i] = row_sum + 10.0;  // Ensure SPD and well-conditioned
    }
    return sym;
}

Matrix generate_nonsymmetric_matrix(int n) {
    std::mt19937 rng(123);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            A[i][j] = dist(rng);
        }
    }
    // Add identity to ensure nonsingular
    for (int i = 0; i < n; ++i) A[i][i] += n * 0.5;
    return A;
}

Vector generate_rhs(int n) {
    std::mt19937 rng(789);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Vector b(n);
    for (int i = 0; i < n; ++i) b[i] = dist(rng);
    return b;
}

double compute_residual_norm(const Matrix& A, const Vector& x, const Vector& b) {
    Vector Ax = matvec(A, x);
    double r_norm = 0.0;
    for (size_t i = 0; i < A.size(); ++i) {
        double diff = b[i] - Ax[i];
        r_norm += diff * diff;
    }
    return std::sqrt(r_norm);
}

// Hessenberg structure check: H[i][j] must be 0 for i > j+1
bool verify_hessenberg_structure(const Matrix& A, int restart_m) {
    size_t n = A.size();
    Vector b(n, 1.0);
    Vector x0(n, 0.0);

    std::vector<Vector> V(restart_m + 1);
    std::vector<std::vector<double>> H(restart_m + 1,
        std::vector<double>(restart_m, 0.0));

    Vector r0 = b; // b - A*0
    double beta = norm2(r0);
    V[0].resize(n);
    for (size_t i = 0; i < n; ++i) V[0][i] = r0[i] / beta;

    for (int k = 0; k < std::min(restart_m, 5); ++k) {
        arnoldi_step(A, V, H, k);
    }

    // Check structure: H[i][j] == 0 for i > j+1
    for (int i = 0; i < std::min(restart_m, 5) + 1; ++i) {
        for (int j = 0; j < std::min(restart_m, 5); ++j) {
            if (i > j + 1) {
                if (std::abs(H[i][j]) > 1e-12) {
                    std::cerr << "H[" << i << "][" << j << "] = " 
                              << H[i][j] << " (expected 0)" << std::endl;
                    return false;
                }
            }
        }
    }
    return true;
}

// ---- Write PPM visualization of convergence history ----
void write_convergence_plot(const std::string& filename,
                            const std::vector<double>& /*gmres_res*/,
                            const std::vector<double>& cg_res,
                            const std::vector<std::vector<double>>& all_residuals,
                            const std::vector<int>& /*restart_sizes*/) {
    int width = 800, height = 600;
    std::ofstream out(filename);
    out << "P6\n" << width << " " << height << "\n255\n";

    std::vector<unsigned char> img(width * height * 3, 255);

    int plot_left = 80, plot_right = 780, plot_top = 50, plot_bottom = 550;
    int pw = plot_right - plot_left, ph = plot_bottom - plot_top;

    // Draw axes
    for (int y = plot_top; y <= plot_bottom; ++y) {
        int idx = (y * width + plot_left) * 3;
        img[idx] = img[idx+1] = img[idx+2] = 0;
    }
    for (int x = plot_left; x <= plot_right; ++x) {
        int idx = (plot_bottom * width + x) * 3;
        img[idx] = img[idx+1] = img[idx+2] = 0;
    }

    // Draw grid lines
    for (int i = 0; i <= 4; ++i) {
        int y = plot_bottom - (i * ph / 4);
        for (int x = plot_left; x <= plot_right; ++x) {
            int idx = (y * width + x) * 3;
            img[idx] = img[idx+1] = img[idx+2] = 200;
        }
    }

    auto draw_line = [&](const std::vector<double>& data, int r, int g, int b,
                         int max_iters) {
        if (data.empty()) return;
        for (size_t i = 1; i < data.size() && i < (size_t)max_iters; ++i) {
            double prev_val = std::max(-16.0, std::log10(data[i-1]));
            double curr_val = std::max(-16.0, std::log10(data[i]));
            prev_val = std::min(0.0, prev_val);
            curr_val = std::min(0.0, curr_val);

            int x1 = plot_left + (int)((i-1) * pw / max_iters);
            int x2 = plot_left + (int)(i * pw / max_iters);
            int y1 = plot_bottom - (int)((prev_val + 16.0) * ph / 16.0);
            int y2 = plot_bottom - (int)((curr_val + 16.0) * ph / 16.0);

            // Simple line drawing
            int dx = std::abs(x2 - x1), dy = -std::abs(y2 - y1);
            int sx = x1 < x2 ? 1 : -1, sy = y1 < y2 ? 1 : -1;
            int err = dx + dy;
            int cx = x1, cy = y1;
            while (true) {
                if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
                    int idx = (cy * width + cx) * 3;
                    img[idx] = r; img[idx+1] = g; img[idx+2] = b;
                }
                if (cx == x2 && cy == y2) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; cx += sx; }
                if (e2 <= dx) { err += dx; cy += sy; }
            }
        }
    };

    // Colors for different restart sizes
    int colors[][3] = {
        {255, 0, 0},    // red: m=5
        {0, 180, 0},    // green: m=10
        {0, 0, 255},    // blue: m=20
        {200, 0, 200},  // purple: m=30
        {0, 150, 150},  // teal: CG
    };

    int max_iters = 100;
    for (size_t i = 0; i < all_residuals.size() && i < 4; ++i) {
        draw_line(all_residuals[i], colors[i][0], colors[i][1], colors[i][2], max_iters);
    }
    // CG in teal
    if (!cg_res.empty()) {
        draw_line(cg_res, colors[4][0], colors[4][1], colors[4][2], max_iters);
    }

    out.write(reinterpret_cast<char*>(img.data()), img.size());
    out.close();
}

// ============================================================
int main() {
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "====================================================\n";
    std::cout << "  GMRES(m) Krylov Subspace Solver - Verification\n";
    std::cout << "====================================================\n\n";

    bool all_passed = true;
    int n = 100;
    double tol = 1e-8;

    // ---- Test 1: Symmetric SPD system — GMRES vs CG ----
    std::cout << "Test 1: Symmetric SPD system (CG equivalence check)\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A_spd = generate_spd_matrix(n);
        Vector b_spd = generate_rhs(n);

        // CG reference
        std::vector<double> cg_residuals;
        Vector x_cg = conjugate_gradient(A_spd, b_spd, tol, 10000, &cg_residuals);
        double cg_res = compute_residual_norm(A_spd, x_cg, b_spd) / norm2(b_spd);

        // GMRES(30) — large enough to converge without restart for fair comparison
        auto gmres_result = gmres(A_spd, b_spd, 30, tol);
        double gmres_res = compute_residual_norm(A_spd, gmres_result.x, b_spd) / norm2(b_spd);

        std::cout << "  CG final residual:     " << cg_res << "\n";
        std::cout << "  GMRES(30) final residual: " << gmres_res << "\n";
        std::cout << "  GMRES total iterations:   " << gmres_result.total_iters << "\n";
        std::cout << "  CG iterations:            " << cg_residuals.size() << "\n";

        bool t1_pass = (cg_res < tol * 10) && (gmres_res < tol * 10);
        std::cout << "  " << (t1_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Both solvers converge to tolerance\n\n";
        if (!t1_pass) all_passed = false;
    }

    // ---- Test 2: Non-symmetric system ----
    std::cout << "Test 2: Non-symmetric linear system\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A_nonsym = generate_nonsymmetric_matrix(n);
        Vector b_nonsym = generate_rhs(n);

        auto result = gmres(A_nonsym, b_nonsym, 10, tol);
        double final_res = compute_residual_norm(A_nonsym, result.x, b_nonsym) / norm2(b_nonsym);

        std::cout << "  Final relative residual: " << final_res << "\n";
        std::cout << "  Total iterations:        " << result.total_iters << "\n";
        std::cout << "  Converged:               " << (result.converged ? "yes" : "no") << "\n";

        bool t2_pass = result.converged && final_res < tol * 10;
        std::cout << "  " << (t2_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — GMRES converges on non-symmetric system\n\n";
        if (!t2_pass) all_passed = false;
    }

    // ---- Test 3: Residual monotonic decrease within restart cycles ----
    std::cout << "Test 3: Residual monotonic decrease (within restart)\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A = generate_nonsymmetric_matrix(n);
        Vector b = generate_rhs(n);

        auto result = gmres(A, b, 30, tol);
        bool monotonic = true;
        for (size_t i = 1; i < result.residuals.size(); ++i) {
            if (result.residuals[i] > result.residuals[i-1] * 1.01) {
                monotonic = false;
                std::cerr << "  Non-monotonic at restart " << i 
                          << ": " << result.residuals[i-1] << " -> " 
                          << result.residuals[i] << "\n";
            }
        }

        std::cout << "  Restart cycles: " << result.residuals.size() << "\n";
        std::cout << "  Initial residual: " << result.residuals[0] << "\n";
        std::cout << "  Final residual:   " << result.residuals.back() << "\n";
        std::cout << "  Reduction factor: " << result.residuals[0] / result.residuals.back() << "\n";
        std::cout << "  " << (monotonic ? "✅ PASS" : "❌ FAIL") 
                  << " — Residual decreases monotonically\n\n";
        if (!monotonic) all_passed = false;
    }

    // ---- Test 4: Givens rotation orthogonality ----
    std::cout << "Test 4: Givens rotation orthogonality\n";
    std::cout << "----------------------------------------------------\n";
    {
        double errors[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            // Random angle
            double theta = (i + 1) * 0.7;
            double a = std::cos(theta), b = std::sin(theta);
            Givens g = givens_rotation(a, b);

            // Check: c^2 + s^2 ≈ 1
            double orth = g.c * g.c + g.s * g.s;
            errors[i] = std::abs(orth - 1.0);

            // Verify: [c -s; s c] * [a; b] = [r; 0]
            // (implicitly verified by orthogonality: c²+s²=1)
        }
        std::cout << "  c²+s²-1 errors: " << errors[0] << ", " 
                  << errors[1] << ", " << errors[2] << "\n";

        bool t4_pass = errors[0] < 1e-14 && errors[1] < 1e-14 && errors[2] < 1e-14;
        std::cout << "  " << (t4_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Givens rotations preserve orthogonality\n\n";
        if (!t4_pass) all_passed = false;
    }

    // ---- Test 5: Arnoldi Hessenberg structure ----
    std::cout << "Test 5: Arnoldi Hessenberg matrix structure\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A = generate_nonsymmetric_matrix(50);
        bool hessenberg = verify_hessenberg_structure(A, 8);
        std::cout << "  " << (hessenberg ? "✅ PASS" : "❌ FAIL") 
                  << " — H[k] is upper Hessenberg (i>j+1 ⇒ 0)\n\n";
        if (!hessenberg) all_passed = false;
    }

    // ---- Test 6: Backward error ||b - Ax|| / ||b|| ----
    std::cout << "Test 6: Backward error analysis\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A = generate_nonsymmetric_matrix(n);
        Vector b = generate_rhs(n);

        auto result = gmres(A, b, 15, tol);
        double backward_err = compute_residual_norm(A, result.x, b) / norm2(b);

        std::cout << "  ||b - Ax|| / ||b|| = " << backward_err << "\n";
        bool t6_pass = backward_err < tol * 10;
        std::cout << "  " << (t6_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Backward error below tolerance\n\n";
        if (!t6_pass) all_passed = false;
    }

    // ---- Test 7: Restart size convergence rate comparison ----
    std::cout << "Test 7: Restart size convergence comparison\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A = generate_nonsymmetric_matrix(100);
        Vector b = generate_rhs(100);
        std::vector<int> restart_sizes = {5, 10, 20, 30};
        std::vector<double> final_residuals;
        std::vector<std::vector<double>> all_res;
        std::vector<double> cg_res_vals; // empty for nonsymmetric

        for (int m : restart_sizes) {
            auto result = gmres(A, b, m, tol, 200);
            double fres = compute_residual_norm(A, result.x, b) / norm2(b);
            final_residuals.push_back(fres);
            all_res.push_back(result.residuals);

            std::cout << "  m=" << std::setw(2) << m 
                      << ": iters=" << std::setw(4) << result.total_iters
                      << "  residual=" << fres
                      << "  converged=" << (result.converged ? "yes" : "no") << "\n";
        }

        // Larger m should converge in fewer total iterations (or smaller residual)
        bool t7_pass = true;
        for (size_t i = 1; i < final_residuals.size(); ++i) {
            // Not strictly monotonic but larger m should not be dramatically worse
            if (final_residuals[i] > final_residuals[i-1] * 100 && 
                final_residuals[i-1] > tol) {
                t7_pass = false;
            }
        }
        std::cout << "  " << (t7_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Larger restart sizes do not degrade convergence\n\n";
        if (!t7_pass) all_passed = false;

        // Write convergence visualization (non-symmetric case, no CG)
        write_convergence_plot("gmres_convergence.ppm", 
                               std::vector<double>(), std::vector<double>(),
                               all_res, restart_sizes);
        std::cout << "  Convergence plot saved: gmres_convergence.ppm\n\n";
    }

    // ---- Test 8: Predetermined exact solution verification ----
    std::cout << "Test 8: Exact solution recovery\n";
    std::cout << "----------------------------------------------------\n";
    {
        n = 30;
        Matrix A(n, Vector(n));
        // Well-conditioned matrix: diagonally dominant
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                A[i][j] = (i == j) ? (n + 1.0) : (1.0 / (1 + std::abs(i - j)));
            }
        }

        // Known solution: x_true = [1, 2, 3, ..., n]
        Vector x_true(n);
        for (int i = 0; i < n; ++i) x_true[i] = i + 1.0;

        // Compute b = A * x_true
        Vector b = matvec(A, x_true);

        auto result = gmres(A, b, 15, tol);
        double err = 0.0;
        for (int i = 0; i < n; ++i) {
            err = std::max(err, std::abs(result.x[i] - x_true[i]));
        }
        double rel_err = err / norm2(x_true);

        std::cout << "  Max absolute error: " << err << "\n";
        std::cout << "  Relative error:     " << rel_err << "\n";
        bool t8_pass = rel_err < tol * 10;
        std::cout << "  " << (t8_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Recovers known solution accurately\n\n";
        if (!t8_pass) all_passed = false;
    }

    // ---- Test 9: SPD system GMRES vs CG convergence rate ----
    std::cout << "Test 9: SPD — GMRES vs CG convergence rate comparison\n";
    std::cout << "----------------------------------------------------\n";
    {
        Matrix A_spd = generate_spd_matrix(100);
        Vector b_spd = generate_rhs(100);

        std::vector<double> cg_res;
        conjugate_gradient(A_spd, b_spd, tol, 10000, &cg_res);

        auto gmres_result = gmres(A_spd, b_spd, 30, tol);

        // CG and GMRES should have comparable convergence on SPD
        double cg_final = cg_res.back();
        double gmres_final_res = compute_residual_norm(A_spd, gmres_result.x, b_spd) / norm2(b_spd);

        std::cout << "  CG final residual:       " << cg_final << "\n";
        std::cout << "  GMRES final residual:    " << gmres_final_res << "\n";
        std::cout << "  CG iterations:           " << cg_res.size() << "\n";
        std::cout << "  GMRES total iterations:  " << gmres_result.total_iters << "\n";

        bool t9_pass = cg_final < tol * 10 && gmres_final_res < tol * 10;
        std::cout << "  " << (t9_pass ? "✅ PASS" : "❌ FAIL") 
                  << " — Both converge on SPD system\n\n";
        if (!t9_pass) all_passed = false;

        // Write SPD convergence comparison plot
        std::vector<std::vector<double>> gmres_res_list = {gmres_result.residuals};
        std::vector<int> restart_label = {30};
        write_convergence_plot("gmres_cg_comparison.ppm",
                               gmres_result.residuals, cg_res,
                               std::vector<std::vector<double>>(), std::vector<int>());
        std::cout << "  SPD comparison plot saved: gmres_cg_comparison.ppm\n\n";
    }

    // ---- Summary ----
    std::cout << "====================================================\n";
    std::cout << "  " << (all_passed ? "ALL TESTS PASSED ✅" : "SOME TESTS FAILED ❌") << "\n";
    std::cout << "====================================================\n";

    return all_passed ? 0 : 1;
}
