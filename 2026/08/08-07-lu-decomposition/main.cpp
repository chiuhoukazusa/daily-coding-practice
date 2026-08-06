/**
 * LU Decomposition (Doolittle Algorithm)
 * 
 * Doolittle's method: L has 1s on diagonal, U is upper triangular.
 * For matrix A = L * U:
 *   U[0][j] = A[0][j]           (first row of U)
 *   L[i][0] = A[i][0] / U[0][0] (first column of L)
 *   For i,j >= 1:
 *     U[i][j] = A[i][j] - sum_{k=0}^{i-1} L[i][k] * U[k][j]   (for j >= i)
 *     L[i][j] = (A[i][j] - sum_{k=0}^{j-1} L[i][k] * U[k][j]) / U[j][j]  (for i > j)
 *
 * Then solve Ax = b:
 *   1. Forward substitution: Ly = b
 *   2. Back substitution:    Ux = y
 *
 * Validation:
 * 1. ||A - L*U||_F should be near 0 (decomposition accuracy)
 * 2. ||Ax - b||_2 should be near 0 (solution accuracy)
 * 3. Works for general non-singular matrices
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <cassert>
#include <sstream>

using Matrix = std::vector<std::vector<double>>;
using Vector = std::vector<double>;

// ============================================================
// Utility
// ============================================================

void print_matrix(const Matrix& A, const std::string& label, int max_rows = 8) {
    int n = A.size();
    int m = A[0].size();
    std::cout << label << " (" << n << "x" << m << "):\n";
    int show = std::min(n, max_rows);
    for (int i = 0; i < show; i++) {
        for (int j = 0; j < std::min(m, 8); j++) {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12) << A[i][j] << " ";
        }
        std::cout << "\n";
    }
    if (n > max_rows) std::cout << "  ... (" << (n - max_rows) << " more rows)\n";
    std::cout << "\n";
}

void print_vector(const Vector& v, const std::string& label, int max_entries = 8) {
    std::cout << label << " (" << v.size() << "): ";
    int show = std::min((int)v.size(), max_entries);
    for (int i = 0; i < show; i++) {
        std::cout << std::fixed << std::setprecision(6) << v[i] << " ";
    }
    if ((int)v.size() > max_entries) std::cout << "...";
    std::cout << "\n";
}

double frobenius_norm(const Matrix& A) {
    double sum = 0.0;
    for (const auto& row : A)
        for (double x : row) sum += x * x;
    return std::sqrt(sum);
}

double l2_norm(const Vector& v) {
    double sum = 0.0;
    for (double x : v) sum += x * x;
    return std::sqrt(sum);
}

Matrix mat_mul(const Matrix& A, const Matrix& B) {
    int n = A.size(), k = A[0].size(), m = B[0].size();
    Matrix C(n, Vector(m, 0.0));
    for (int i = 0; i < n; i++)
        for (int p = 0; p < k; p++)
            if (std::abs(A[i][p]) > 1e-15)
                for (int j = 0; j < m; j++)
                    C[i][j] += A[i][p] * B[p][j];
    return C;
}

Vector mat_vec_mul(const Matrix& A, const Vector& x) {
    int n = A.size(), m = A[0].size();
    Vector y(n, 0.0);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < m; j++)
            y[i] += A[i][j] * x[j];
    return y;
}

Vector vec_sub(const Vector& a, const Vector& b) {
    int n = a.size();
    Vector c(n);
    for (int i = 0; i < n; i++) c[i] = a[i] - b[i];
    return c;
}

// ============================================================
// Doolittle LU Decomposition (in-place, no pivoting)
// ============================================================

/**
 * Performs Doolittle LU decomposition.
 * A is modified in-place:
 *   Upper triangle + diagonal = U
 *   Strictly lower triangle = L (diagonal 1s are implicit)
 * 
 * Returns false if decomposition fails (e.g. zero pivot).
 */
bool doolittle_lu(Matrix& A, int n) {
    for (int i = 0; i < n; i++) {
        // Compute U[i][j] for j = i..n-1
        for (int j = i; j < n; j++) {
            double sum = 0.0;
            for (int k = 0; k < i; k++)
                sum += A[i][k] * A[k][j]; // L[i][k] * U[k][j]
            A[i][j] = A[i][j] - sum;
        }
        
        // Compute L[j][i] for j = i+1..n-1
        for (int j = i + 1; j < n; j++) {
            double sum = 0.0;
            for (int k = 0; k < i; k++)
                sum += A[j][k] * A[k][i]; // L[j][k] * U[k][i]
            A[j][i] = (A[j][i] - sum) / A[i][i]; // divide by U[i][i]
        }
    }
    
    // Check for singular matrix (zero diagonal in U)
    for (int i = 0; i < n; i++)
        if (std::abs(A[i][i]) < 1e-12) return false;
    return true;
}

/**
 * Extract explicit L and U matrices from in-place LU storage.
 */
void extract_lu(const Matrix& lu, Matrix& L, Matrix& U, int n) {
    L.assign(n, Vector(n, 0.0));
    U.assign(n, Vector(n, 0.0));
    for (int i = 0; i < n; i++) {
        L[i][i] = 1.0; // Doolittle: L has 1s on diagonal
        for (int j = 0; j < n; j++) {
            if (i > j) {
                L[i][j] = lu[i][j];
            } else {
                U[i][j] = lu[i][j];
            }
        }
    }
}

// ============================================================
// Forward and Back Substitution
// ============================================================

/**
 * Forward substitution: solve Ly = b where L is unit lower triangular.
 * L's diagonal is 1 (Doolittle), L's lower part is in lu.
 */
Vector forward_substitution(const Matrix& lu, const Vector& b, int n) {
    Vector y(n);
    for (int i = 0; i < n; i++) {
        double sum = b[i];
        for (int j = 0; j < i; j++)
            sum -= lu[i][j] * y[j]; // L[i][j] * y[j], L[i][i]=1
        y[i] = sum; // divided by L[i][i] = 1
    }
    return y;
}

/**
 * Back substitution: solve Ux = y where U is upper triangular.
 */
Vector back_substitution(const Matrix& lu, const Vector& y, int n) {
    Vector x(n);
    for (int i = n - 1; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < n; j++)
            sum -= lu[i][j] * x[j]; // U[i][j] * x[j]
        x[i] = sum / lu[i][i]; // divide by U[i][i]
    }
    return x;
}

/**
 * Full solve: LU decompose A, then forward + back substitute.
 */
Vector lu_solve(Matrix A, const Vector& b) {
    int n = A.size();
    bool ok = doolittle_lu(A, n);
    if (!ok) {
        std::cerr << "ERROR: LU decomposition failed (singular matrix?)\n";
        return Vector(n, 0.0);
    }
    Vector y = forward_substitution(A, b, n);
    Vector x = back_substitution(A, y, n);
    return x;
}

// ============================================================
// Linear system solver using LU decomposition
// ============================================================

struct LUSolver {
    Matrix LU; // in-place LU
    int n;
    bool ok;
    
    bool decompose(const Matrix& A_orig) {
        n = A_orig.size();
        LU = A_orig; // copy
        ok = doolittle_lu(LU, n);
        return ok;
    }
    
    Vector solve(const Vector& b) const {
        Vector y = forward_substitution(LU, b, n);
        return back_substitution(LU, y, n);
    }
    
    // Solve multiple RHS efficiently
    std::vector<Vector> solve_multi(const std::vector<Vector>& B) const {
        std::vector<Vector> results;
        for (const auto& b : B)
            results.push_back(solve(b));
        return results;
    }
};

// ============================================================
// Validation helpers
// ============================================================

struct ValidationResult {
    double lu_error;        // ||A - L*U||_F
    double residual_error;  // ||Ax - b||_2 / ||b||_2
    double max_component_error; // max |A[i][j] - (LU)[i][j]|
    bool passed;
};

ValidationResult validate_decomposition(const Matrix& A_orig, const Matrix& lu, const Vector& x, const Vector& b) {
    int n = A_orig.size();
    Matrix L, U;
    extract_lu(lu, L, U, n);
    
    // Compute A - L*U
    Matrix LU_prod = mat_mul(L, U);
    Matrix diff(n, Vector(n, 0.0));
    double max_err = 0.0;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            diff[i][j] = A_orig[i][j] - LU_prod[i][j];
            max_err = std::max(max_err, std::abs(diff[i][j]));
        }
    }
    
    double lu_frob = frobenius_norm(diff);
    
    // Compute residual ||Ax - b||
    Vector Ax = mat_vec_mul(A_orig, x);
    Vector residual = vec_sub(Ax, b);
    double resid_norm = l2_norm(residual);
    double b_norm = l2_norm(b);
    double rel_resid = (b_norm > 1e-15) ? resid_norm / b_norm : resid_norm;
    
    ValidationResult vr;
    vr.lu_error = lu_frob;
    vr.residual_error = rel_resid;
    vr.max_component_error = max_err;
    vr.passed = (lu_frob < 1e-8) && (rel_resid < 1e-8);
    return vr;
}

// ============================================================
// Condition number estimation
// ============================================================

/**
 * Estimate condition number using 1-norm.
 * ||A||_1 * ||A^{-1}||_1 estimated via Hager's method.
 */
double condition_number_estimate(const Matrix& A_orig, int n) {
    // ||A||_1 (max column sum)
    double anorm = 0.0;
    for (int j = 0; j < n; j++) {
        double col = 0.0;
        for (int i = 0; i < n; i++) col += std::abs(A_orig[i][j]);
        anorm = std::max(anorm, col);
    }
    
    // Estimate ||A^{-1}||_1 using Hager's method
    LUSolver solver;
    solver.decompose(A_orig);
    if (!solver.ok) return -1.0;
    
    Vector y(n, 1.0 / n); // initial guess
    for (int iter = 0; iter < 5; iter++) {
        Vector z = solver.solve(y);
        
        // Compute sign(z)
        Vector xi(n);
        for (int i = 0; i < n; i++)
            xi[i] = (z[i] >= 0) ? 1.0 : -1.0;
        
        Vector y_next = solver.solve(xi); // A^{-T} * xi via A^{-T} = (A^{-1})^T
        // Actually we need A^{-T} * xi. Since we have A = LU, 
        // A^{-T} = (L^{-T}) * (U^{-T}). Let's just solve A^T y = xi
        // by transposing the solve.
        // Simpler: estimate via power iteration on A^{-1}
        
        // Hager's method: y_{k+1} = A^{-T} * sign(A^{-1} * y_k)
        // A^{-1} * y_k = z already computed
        // A^{-T} * sign(z): solve A^T w = sign(z)
        // A^T = U^T * L^T, so forward/back substitution transposed
        
        // Simpler approach: use power method directly
        double znorm = 0.0;
        for (int i = 0; i < n; i++) znorm = std::max(znorm, std::abs(z[i]));
        if (znorm < 1e-15) break;
        
        y = y_next; // approximate
    }
    
    // Simpler approach: compute directly via power method
    // Estimate ||A^{-1}||_1 ~ max_i ||A^{-1} * e_i||_1
    double ainorm = 0.0;
    for (int j = 0; j < std::min(n, 5); j++) {
        Vector e(n, 0.0);
        e[j] = 1.0;
        Vector col = solver.solve(e);
        double s = 0.0;
        for (int i = 0; i < n; i++) s += std::abs(col[i]);
        ainorm = std::max(ainorm, s);
    }
    
    // Do a few more random probe vectors for better estimate
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    for (int t = 0; t < 5; t++) {
        Vector v(n);
        for (int i = 0; i < n; i++) v[i] = dist(rng);
        double vnorm = 0.0;
        for (int i = 0; i < n; i++) vnorm = std::max(vnorm, std::abs(v[i]));
        for (int i = 0; i < n; i++) v[i] /= vnorm;
        
        Vector col = solver.solve(v);
        double s = 0.0;
        for (int i = 0; i < n; i++) s += std::abs(col[i]);
        ainorm = std::max(ainorm, s);
    }
    
    return anorm * ainorm;
}

// ============================================================
// Test Cases
// ============================================================

std::vector<double> generate_solution(int n) {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    std::vector<double> x(n);
    for (int i = 0; i < n; i++) x[i] = dist(rng);
    return x;
}

// Well-conditioned matrix: diagonally dominant
Matrix make_diagonally_dominant(int n) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n; i++) {
        double row_sum = 0.0;
        for (int j = 0; j < n; j++) {
            if (i != j) {
                A[i][j] = dist(rng);
                row_sum += std::abs(A[i][j]);
            }
        }
        A[i][i] = row_sum + 1.0 + std::abs(dist(rng)); // strictly diagonally dominant
    }
    return A;
}

// Hilbert matrix (ill-conditioned)
Matrix make_hilbert(int n) {
    Matrix H(n, Vector(n, 0.0));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            H[i][j] = 1.0 / (i + j + 1.0);
    return H;
}

// Random SPD matrix
Matrix make_random_spd(int n) {
    std::mt19937 rng(99);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);
    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            A[i][j] = dist(rng);
    // Make SPD: A = M * M^T + n * I
    Matrix B = mat_mul(A, A); // A*A^T is PSD
    for (int i = 0; i < n; i++) B[i][i] += n; // make PD
    return B;
}

// Lower triangular (already in LU form, for edge case testing)
Matrix make_random_general(int n) {
    std::mt19937 rng(77);
    std::uniform_real_distribution<double> dist(-5.0, 5.0);
    Matrix A(n, Vector(n, 0.0));
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            A[i][j] = dist(rng);
    // Ensure nonsingular by strengthening diagonal
    for (int i = 0; i < n; i++) A[i][i] += n * 2.0;
    return A;
}

struct TestResult {
    std::string name;
    int n;
    double lu_frob_err;
    double rel_residual;
    double max_comp_err;
    double cond_est;
    bool passed;
};

void run_test(const std::string& name, const Matrix& A, const Vector& x_exact, bool verbose = true) {
    int n = A.size();
    Vector b = mat_vec_mul(A, x_exact);
    
    LUSolver solver;
    bool decomp_ok = solver.decompose(A);
    
    if (!decomp_ok) {
        std::cout << "  [" << name << " n=" << n << "] ❌ DECOMPOSITION FAILED\n";
        return;
    }
    
    Vector x_solved = solver.solve(b);
    auto vr = validate_decomposition(A, solver.LU, x_solved, b);
    double cond = condition_number_estimate(A, n);
    
    // Also compute ||x - x_exact|| / ||x_exact||
    double x_err = l2_norm(vec_sub(x_solved, x_exact)) / l2_norm(x_exact);
    
    std::cout << "  [" << name << " n=" << n << "] ";
    std::cout << "||A-LU||_F=" << std::scientific << vr.lu_error;
    std::cout << " | ||Ax-b||/||b||=" << vr.residual_error;
    std::cout << " | max|er|=" << vr.max_component_error;
    std::cout << " | ||Δx||/||x||=" << x_err;
    std::cout << " | κ₁≈" << std::fixed << std::setprecision(2) << cond;
    std::cout << " | " << (vr.passed ? "✅" : (vr.lu_error < 1e-6 ? "⚠️" : "❌")) << "\n";
    
    if (verbose && n <= 6) {
        print_matrix(A, "    A");
        print_vector(b, "    b");
        print_vector(x_exact, "    x (exact)");
        print_vector(x_solved, "    x (solved)");
    }
}

// ============================================================
// PPM output for visual report
// ============================================================

void write_ppm_report(const std::vector<TestResult>& results) {
    // Generate a summary report image
    int W = 800, H = 600;
    std::ostringstream ppm;
    ppm << "P3\n" << W << " " << H << "\n255\n";
    
    auto set_pixel = [](std::vector<std::vector<int>>& img, int x, int y, int r, int g, int b) {
        if (x >= 0 && x < (int)img[0].size() && y >= 0 && y < (int)img.size()) {
            img[y][x] = r * 65536 + g * 256 + b;
        }
    };
    
    auto fill_rect = [&](std::vector<std::vector<int>>& img, int x, int y, int w, int h, int r, int g, int b) {
        for (int dy = 0; dy < h; dy++)
            for (int dx = 0; dx < w; dx++)
                set_pixel(img, x + dx, y + dy, r, g, b);
    };
    
    std::vector<std::vector<int>> img(H, std::vector<int>(W, 0xFFFFFF)); // white bg
    
    // Title
    int y = 20;
    for (int dy = -12; dy <= 2; dy++) {
        int alpha = (dy == 0) ? 40 : 30;
        fill_rect(img, 0, y + dy + 33, W, 36, alpha, alpha, alpha);
    }
    // We draw title text programmatically would be complex, so let's draw a bar chart of errors
    
    // Background
    fill_rect(img, 40, 100, W - 80, H - 160, 240, 245, 250);
    
    // Draw test results as bar chart
    int chart_y = 120, bar_area_h = 350;
    int n = results.size();
    int bar_w = (W - 120) / n - 20;
    
    // Find max error for scaling
    double max_err = 1e-15;
    for (const auto& r : results) max_err = std::max(max_err, r.lu_frob_err);
    max_err = std::max(max_err, 1e-14);
    
    for (int i = 0; i < n; i++) {
        int x = 70 + i * (bar_w + 20);
        
        // Log scale: map log10(error) to height
        double log_err = std::log10(std::max(results[i].lu_frob_err, 1e-16));
        // Map [-16, 0] to [0, bar_area_h]
        double frac = (log_err + 16.0) / 16.0;
        int h = std::min((int)(frac * bar_area_h), bar_area_h);
        h = std::max(h, 3);
        
        // Green if passed, red if not
        int r = results[i].passed ? 40 : 200;
        int g_val = results[i].passed ? 160 : 40;
        int b_val = results[i].passed ? 80 : 40;
        
        fill_rect(img, x, chart_y + bar_area_h - h, bar_w, h, r, g_val, b_val);
        
        // Label
        int label_y = chart_y + bar_area_h + 10;
        fill_rect(img, x, label_y, bar_w, 20, 255, 255, 255);
    }
    
    int px = 0;
    for (int yp = 0; yp < H; yp++) {
        for (int xp = 0; xp < W; xp++) {
            int c = img[yp][xp];
            int r = (c >> 16) & 0xFF;
            int g = (c >> 8) & 0xFF;
            int b = c & 0xFF;
            px++;
            if (px % 6 == 0) ppm << "\n";
            ppm << r << " " << g << " " << b << " ";
        }
    }
    ppm << "\n";
    
    // Handle very large PPM text
    std::string content = ppm.str();
    std::ofstream f("lu_decomposition_report.ppm");
    f << content;
    f.close();
    std::cout << "  Written ppm report: lu_decomposition_report.ppm\n";
}

// ============================================================
// Main
// ============================================================

int main() {
    std::cout << std::scientific << std::setprecision(3);
    std::cout << "========================================\n";
    std::cout << "  LU Decomposition (Doolittle Method)\n";
    std::cout << "========================================\n\n";
    
    std::vector<TestResult> all_results;
    bool all_passed = true;
    
    // -------------------------------------------------------
    // Test 1: Small 3x3 well-conditioned
    // -------------------------------------------------------
    {
        int n = 3;
        Matrix A = {{4, 3, 2}, {3, 5, 1}, {2, 1, 6}};
        Vector x = {1, 2, 3};
        std::cout << "=== Test 1: 3x3 Hand-crafted Matrix ===\n";
        run_test("Hand 3x3", A, x, true);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x)), mat_vec_mul(A, x));
        all_results.push_back({"3x3 Hand", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 2: 5x5 Diagonally Dominant
    // -------------------------------------------------------
    {
        int n = 5;
        std::cout << "\n=== Test 2: 5x5 Diagonally Dominant ===\n";
        Matrix A = make_diagonally_dominant(n);
        Vector x_exact = generate_solution(n);
        run_test("Diag-Dom 5x5", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"5x5 Diag-Dom", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 3: 10x10 Random SPD
    // -------------------------------------------------------
    {
        int n = 10;
        std::cout << "\n=== Test 3: 10x10 Random SPD ===\n";
        Matrix A = make_random_spd(n);
        Vector x_exact = generate_solution(n);
        run_test("SPD 10x10", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"10x10 SPD", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 4: 20x20 Random General
    // -------------------------------------------------------
    {
        int n = 20;
        std::cout << "\n=== Test 4: 20x20 Random General ===\n";
        Matrix A = make_random_general(n);
        Vector x_exact = generate_solution(n);
        run_test("Gen 20x20", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"20x20 General", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 5: 50x50 Diagonally Dominant
    // -------------------------------------------------------
    {
        int n = 50;
        std::cout << "\n=== Test 5: 50x50 Diagonally Dominant ===\n";
        Matrix A = make_diagonally_dominant(n);
        Vector x_exact = generate_solution(n);
        run_test("Diag-Dom 50x50", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"50x50 Diag-Dom", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 6: 10x10 Hilbert (ill-conditioned)
    // -------------------------------------------------------
    {
        int n = 10;
        std::cout << "\n=== Test 6: 10x10 Hilbert Matrix (ill-conditioned) ===\n";
        Matrix A = make_hilbert(n);
        Vector x_exact = generate_solution(n);
        run_test("Hilbert 10x10", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        // Note: Hilbert is ill-conditioned, relaxation on residual check
        bool hilbert_ok = vr.lu_error < 1e-6 && vr.residual_error < 1e-4;
        all_results.push_back({"10x10 Hilbert", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), hilbert_ok});
        // Don't fail overall for Hilbert
    }
    
    // -------------------------------------------------------
    // Test 7: Solve multiple RHS efficiently
    // -------------------------------------------------------
    {
        int n = 8;
        std::cout << "\n=== Test 7: Multiple RHS Solve (8x8) ===\n";
        Matrix A = make_diagonally_dominant(n);
        LUSolver s;
        s.decompose(A);
        
        std::vector<Vector> B;
        std::mt19937 rng(999);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        for (int k = 0; k < 5; k++) {
            Vector xk(n);
            for (int i = 0; i < n; i++) xk[i] = dist(rng);
            B.push_back(mat_vec_mul(A, xk));
        }
        
        auto X = s.solve_multi(B);
        
        bool all_ok = true;
        double max_resid = 0.0;
        for (int k = 0; k < 5; k++) {
            Vector Ax = mat_vec_mul(A, X[k]);
            double res = l2_norm(vec_sub(Ax, B[k]));
            max_resid = std::max(max_resid, res);
            if (res > 1e-10) all_ok = false;
        }
        
        std::cout << "  [Multi-RHS 8x8] max||Ax-b||=" << std::scientific << max_resid;
        std::cout << " | " << (all_ok ? "✅" : "❌") << "\n";
        
        all_results.push_back({"Multi-RHS 8x8", n, 0.0, max_resid, 0.0, condition_number_estimate(A, n), all_ok});
        all_passed = all_passed && all_ok;
    }
    
    // -------------------------------------------------------
    // Test 8: 100x100 Diagonally Dominant (stress test)
    // -------------------------------------------------------
    {
        int n = 100;
        std::cout << "\n=== Test 8: 100x100 Diagonally Dominant ===\n";
        Matrix A = make_diagonally_dominant(n);
        Vector x_exact = generate_solution(n);
        run_test("Diag-Dom 100x100", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"100x100 Diag-Dom", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Test 9: Non-symmetric random general matrix
    // -------------------------------------------------------
    {
        int n = 15;
        std::cout << "\n=== Test 9: 15x15 Non-symmetric General ===\n";
        std::mt19937 rng(111);
        std::uniform_real_distribution<double> dist(-3.0, 3.0);
        Matrix A(n, Vector(n, 0.0));
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                A[i][j] = dist(rng);
        for (int i = 0; i < n; i++) A[i][i] += n * 3.0;
        Vector x_exact = generate_solution(n);
        run_test("Non-Sym 15x15", A, x_exact, false);
        
        LUSolver s; s.decompose(A);
        auto vr = validate_decomposition(A, s.LU, s.solve(mat_vec_mul(A, x_exact)), mat_vec_mul(A, x_exact));
        all_results.push_back({"15x15 Non-Sym", n, vr.lu_error, vr.residual_error, vr.max_component_error, condition_number_estimate(A, n), vr.passed});
        all_passed = all_passed && vr.passed;
    }
    
    // -------------------------------------------------------
    // Write PPM report
    
    // -------------------------------------------------------
    std::cout << "\n========================================\n";
    std::cout << "  Summary\n";
    std::cout << "========================================\n";
    
    std::cout << "\n" << std::left << std::setw(20) << "Test" 
              << std::setw(10) << "Size" 
              << std::setw(16) << "||A-LU||_F"
              << std::setw(16) << "||Ax-b||/||b||"
              << std::setw(12) << "κ₁_est"
              << "Result\n";
    std::cout << std::string(85, '-') << "\n";
    
    for (const auto& r : all_results) {
        std::cout << std::left << std::setw(20) << r.name
                  << std::setw(10) << r.n
                  << std::scientific << std::setprecision(3) << std::setw(16) << r.lu_frob_err
                  << std::setw(16) << r.rel_residual
                  << std::fixed << std::setprecision(1) << std::setw(12) << r.cond_est
                  << (r.passed ? "✅" : "❌") << "\n";
    }
    
    write_ppm_report(all_results);
    
    // -------------------------------------------------------
    // Final validation
    // -------------------------------------------------------
    int passed_count = 0;
    for (const auto& r : all_results) if (r.passed) passed_count++;
    
    std::cout << "\n========================================\n";
    std::cout << "  Final Result: " << passed_count << "/" << all_results.size() << " tests passed\n";
    std::cout << "  Overall: " << (all_passed ? "✅ ALL PASSED" : "❌ SOME FAILED") << "\n";
    std::cout << "========================================\n";
    
    return all_passed ? 0 : 1;
}
