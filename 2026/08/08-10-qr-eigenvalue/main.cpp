/**
 * Jacobi Eigenvalue Decomposition
 * 08-10-2026: Daily Coding Practice
 *
 * Implements the Jacobi eigenvalue algorithm for real symmetric matrices.
 * Jacobi iteration applies Givens rotations to systematically zero out
 * off-diagonal elements, converging to the diagonal eigenvalue matrix.
 *
 * Verification (quantitative, not visual):
 * 1. Trace(A) = sum(eigenvalues) — invariant
 * 2. ||A||_F^2 = sum(eigenvalue^2) — Frobenius norm invariance
 * 3. det(A) = product(eigenvalues) — invariant
 * 4. Comparison against analytical solutions for known matrices
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>

using Matrix = std::vector<std::vector<double>>;

// ===================== Jacobi Eigenvalue Algorithm =====================
// For symmetric matrices, Jacobi iteration is remarkably robust.
// It applies Givens rotations to zero out off-diagonal elements.

std::vector<double> jacobi_eigenvalues(const Matrix& A_orig, double tol = 1e-12, int max_sweeps = 200) {
    int n = A_orig.size();
    Matrix A = A_orig;

    auto off_sq = [&]() {
        double s = 0.0;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                s += A[i][j] * A[i][j];
        return 2.0 * s;  // off-diagonal norm squared
    };

    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        double off_norm_sq = off_sq();
        if (off_norm_sq < tol * tol) break;

        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double apq = A[p][q];
                if (std::abs(apq) < 1e-18) continue;

                double app = A[p][p];
                double aqq = A[q][q];

                // Compute Jacobi rotation angle to zero out A[p][q]
                // tan(2φ) = 2*apq / (app - aqq)
                // Using the numerically stable formula:
                double theta = (aqq - app) / (2.0 * apq);
                double t = (theta >= 0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
                double c = 1.0 / std::sqrt(1.0 + t * t);
                double s = c * t;

                // Apply rotation: A = J(p,q,θ)^T * A * J(p,q,θ)
                // This only affects rows/cols p and q
                // For symmetry, we update both upper and lower triangles

                // Row updates: A[i][p], A[i][q] for i != p,q
                for (int i = 0; i < n; ++i) {
                    if (i == p || i == q) continue;
                    double aip = A[i][p], aiq = A[i][q];
                    A[i][p] = A[p][i] = c * aip - s * aiq;
                    A[i][q] = A[q][i] = s * aip + c * aiq;
                }

                // Diagonal and cross-term updates
                A[p][p] = c * c * app - 2.0 * c * s * apq + s * s * aqq;
                A[q][q] = s * s * app + 2.0 * c * s * apq + c * c * aqq;
                A[p][q] = A[q][p] = 0.0;
            }
        }
    }

    std::vector<double> ev(n);
    for (int i = 0; i < n; ++i) ev[i] = A[i][i];
    std::sort(ev.begin(), ev.end());
    return ev;
}

// ===================== Verification infrastructure =====================

void print_check(const std::string& label, bool passed, double value, double expected, double tol) {
    std::cout << "  " << std::left << std::setw(42) << label
              << " | val=" << std::scientific << std::setprecision(3) << std::setw(12) << value
              << " exp=" << std::scientific << std::setprecision(3) << std::setw(12) << expected
              << " tol=" << std::scientific << std::setprecision(1) << std::setw(8) << tol
              << " | " << (passed ? "PASS" : "FAIL") << "\n";
}

struct TestResult {
    std::string name;
    int n;
    bool all_pass;
    double max_eig_err;
    double trace_err;
    double det_err;
    double frob_err;
};

TestResult test_matrix(const std::string& name, const Matrix& A,
                       const std::vector<double>& expected = {}) {
    int n = A.size();
    std::cout << "\n" << std::string(72, '=') << "\n";
    std::cout << "Test: " << name << " (" << n << "x" << n << ")\n";
    std::cout << std::string(72, '=') << "\n";

    auto eigvals = jacobi_eigenvalues(A);

    std::cout << "  Eigenvalues: ";
    for (double v : eigvals) std::cout << std::fixed << std::setprecision(8) << v << " ";
    std::cout << "\n";

    if (!expected.empty()) {
        std::cout << "  Expected:    ";
        for (double v : expected) std::cout << std::fixed << std::setprecision(8) << v << " ";
        std::cout << "\n";
    }

    // 1. Trace check: tr(A) = sum(λ_i)
    double trace_A = 0.0;
    for (int i = 0; i < n; ++i) trace_A += A[i][i];
    double trace_eig = 0.0;
    for (double v : eigvals) trace_eig += v;
    double trace_err = std::abs(trace_A - trace_eig);
    bool trace_ok = trace_err < 1e-10 * std::max(1.0, std::abs(trace_A));

    // 2. Frobenius norm check: ||A||_F^2 = sum(λ_i^2)
    double frob2_A = 0.0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            frob2_A += A[i][j] * A[i][j];
    double frob2_eig = 0.0;
    for (double v : eigvals) frob2_eig += v * v;
    double frob_err = std::abs(frob2_A - frob2_eig);
    bool frob_ok = frob_err < 1e-8 * std::max(1.0, frob2_A);

    // 3. Determinant check (via Gaussian elimination with partial pivoting)
    double det_A = 1.0;
    {
        auto A_copy = A;
        for (int i = 0; i < n; ++i) {
            // Find pivot
            int pivot = i;
            for (int r = i + 1; r < n; ++r)
                if (std::abs(A_copy[r][i]) > std::abs(A_copy[pivot][i]))
                    pivot = r;
            if (pivot != i) {
                std::swap(A_copy[i], A_copy[pivot]);
                det_A = -det_A;
            }
            if (std::abs(A_copy[i][i]) < 1e-15) {
                det_A = 0.0; break;
            }
            det_A *= A_copy[i][i];
            for (int r = i + 1; r < n; ++r) {
                double fac = A_copy[r][i] / A_copy[i][i];
                for (int j = i; j < n; ++j)
                    A_copy[r][j] -= fac * A_copy[i][j];
            }
        }
    }
    double det_eig = 1.0;
    for (double v : eigvals) det_eig *= v;
    double det_err = std::abs(det_A - det_eig);
    bool det_ok = (std::abs(det_A) < 1e-10) ? (det_err < 1e-8) : (det_err < 1e-6 * std::max(1.0, std::abs(det_A)));

    // 4. Comparison with analytical
    double max_eig_err = 0.0;
    if (!expected.empty()) {
        for (size_t i = 0; i < expected.size() && i < eigvals.size(); ++i)
            max_eig_err = std::max(max_eig_err, std::abs(eigvals[i] - expected[i]));
    }
    bool expected_ok = expected.empty() || max_eig_err < 1e-8;

    print_check("Trace(A) = sum(lambda)", trace_ok, trace_err, 0.0, 1e-10);
    print_check("||A||_F^2 = sum(lambda^2)", frob_ok, frob_err, 0.0, 1e-8);
    print_check("det(A) = prod(lambda)", det_ok, det_err, 0.0, 1e-6);
    if (!expected.empty() && expected.size() == eigvals.size())
        print_check("lambda_i vs analytical", expected_ok, max_eig_err, 0.0, 1e-8);

    bool all_ok = trace_ok && frob_ok && det_ok && expected_ok;
    std::cout << "  -> " << (all_ok ? "ALL PASSED" : "SOME FAILED") << "\n";

    return {name, n, all_ok, max_eig_err, trace_err, det_err, frob_err};
}

// ===================== Random SPD matrix generator =====================

Matrix random_spd(int n, int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    Matrix A(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j <= i; ++j)
            A[i][j] = A[j][i] = dist(rng);

    // A = A^T * A + n*I to make SPD
    Matrix SPD(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            double s = 0.0;
            for (int k = 0; k < n; ++k) s += A[k][i] * A[k][j];
            SPD[i][j] = s;
        }
    for (int i = 0; i < n; ++i) SPD[i][i] += n;

    return SPD;
}

// ===================== Main =====================

int main() {
    std::cout << std::string(72, '=') << "\n";
    std::cout << "JACOBI EIGENVALUE DECOMPOSITION — NUMERICAL VERIFICATION\n";
    std::cout << "Date: 2026-08-10\n";
    std::cout << std::string(72, '=') << "\n";

    std::vector<TestResult> results;

    // Test 1: 2x2 symmetric
    results.push_back(test_matrix("2x2 Symmetric",
        {{4.0, 1.0}, {1.0, 3.0}},
        {2.381966011250105, 4.618033988749895}));

    // Test 2: 3x3 diagonal
    results.push_back(test_matrix("3x3 Diagonal",
        {{5.0, 0.0, 0.0}, {0.0, -2.0, 0.0}, {0.0, 0.0, 3.0}},
        {-2.0, 3.0, 5.0}));

    // Test 3: 3x3 tridiagonal
    results.push_back(test_matrix("3x3 Tridiagonal",
        {{2.0, 1.0, 0.0}, {1.0, 2.0, 1.0}, {0.0, 1.0, 2.0}},
        {2.0 - std::sqrt(2.0), 2.0, 2.0 + std::sqrt(2.0)}));

    // Test 4: 3x3 repeated eigenvalue
    results.push_back(test_matrix("3x3 Repeated lambda",
        {{4.0, 1.0, 1.0}, {1.0, 4.0, 1.0}, {1.0, 1.0, 4.0}},
        {3.0, 3.0, 6.0}));

    // Test 5: 4x4 Hilbert matrix (notoriously ill-conditioned)
    {
        Matrix H(4, std::vector<double>(4));
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                H[i][j] = 1.0 / (i + j + 1);
        // Known eigenvalues for H_4 (trace = 1+1/3+1/5+1/7 ≈ 1.676190476)
        // Reference values with enough precision for 1e-8 tol comparison:
        // 9.670e-5, 6.73827e-3, 1.69141e-1, 1.50021428
        // Note: Hilbert eigenvalues are numerically computed, not closed-form
        // Hilbert H_4 computed eigenvalues (verified by trace=1.676190, frob, det invariants)
        results.push_back(test_matrix("4x4 Hilbert (ill-cond)", H,
            {9.66970e-5, 6.73827e-3, 1.6914122e-1, 1.50021428}));
    }

    // Test 6: 5x5 near-diagonal
    {
        Matrix W(5, std::vector<double>(5, 0.0));
        for (int i = 0; i < 5; ++i) W[i][i] = i + 1.0;
        W[0][4] = W[4][0] = 1e-10;
        results.push_back(test_matrix("5x5 Near-Diagonal", W,
            {1.0, 2.0, 3.0, 4.0, 5.0}));
    }

    // Test 7: 6x6 random SPD
    results.push_back(test_matrix("6x6 Random SPD", random_spd(6, 123)));

    // Test 8: 8x8 random SPD
    results.push_back(test_matrix("8x8 Random SPD", random_spd(8, 456)));

    // Test 9: 15x15 random SPD (stress test)
    results.push_back(test_matrix("15x15 Random SPD (stress)", random_spd(15, 789)));

    // Test 10: 20x20 random SPD (larger stress test)
    results.push_back(test_matrix("20x20 Random SPD (stress)", random_spd(20, 101)));

    // Summary
    std::cout << "\n" << std::string(72, '=') << "\n";
    std::cout << "SUMMARY\n";
    std::cout << std::string(72, '=') << "\n";
    std::cout << std::left << std::setw(32) << "  Test"
              << std::setw(8) << "N"
              << std::setw(14) << "Trace err"
              << std::setw(14) << "Frob err"
              << std::setw(14) << "Det err"
              << std::setw(14) << "MaxLamErr"
              << "Result\n";
    std::cout << std::string(96, '-') << "\n";

    int pass = 0;
    for (auto& r : results) {
        std::cout << "  " << std::left << std::setw(30) << r.name
                  << std::setw(8) << std::to_string(r.n) + "x" + std::to_string(r.n)
                  << std::scientific << std::setprecision(1) << std::setw(14) << r.trace_err
                  << std::scientific << std::setprecision(1) << std::setw(14) << r.frob_err
                  << std::scientific << std::setprecision(1) << std::setw(14) << r.det_err
                  << std::scientific << std::setprecision(1) << std::setw(14) << r.max_eig_err
                  << (r.all_pass ? "PASS" : "FAIL") << "\n";
        if (r.all_pass) pass++;
    }

    std::cout << "\n  " << pass << "/" << results.size() << " tests passed\n";
    bool all_ok = (pass == (int)results.size());
    std::cout << "  -> " << (all_ok ? "ALL TESTS PASSED" : std::to_string(results.size() - pass) + " TEST(S) FAILED") << "\n";

    return all_ok ? 0 : 1;
}
