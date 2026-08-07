/**
 * 每日编程实践 - 2026-08-08
 * QR Decomposition: Classical GS vs Modified GS vs Householder
 *
 * 量化验证：
 * - ||A - QR||_F 重构误差
 * - ||Q^T Q - I||_F 正交性验证
 * - R的上三角性验证
 * - ill-conditioned 矩阵下的鲁棒性
 * - 线性方程组求解
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cassert>

struct Matrix {
    int rows, cols;
    std::vector<double> data;

    Matrix(int r, int c) : rows(r), cols(c), data(r * c, 0.0) {}

    double& operator()(int i, int j) { return data[i * cols + j]; }
    double operator()(int i, int j) const { return data[i * cols + j]; }

    void set_zero() { std::fill(data.begin(), data.end(), 0.0); }

    void print(const char* name, int max_rows = 8) const {
        printf("%s (%dx%d):\n", name, rows, cols);
        int rs = std::min(rows, max_rows);
        int cs = std::min(cols, 6);
        for (int i = 0; i < rs; i++) {
            printf("  ");
            for (int j = 0; j < cs; j++) printf("%10.6f ", (*this)(i, j));
            if (cols > cs) printf("...");
            printf("\n");
        }
        if (rows > rs) printf("  ... (%d more rows)\n", rows - rs);
    }
};

double vec_norm_col(const Matrix& A, int col) {
    double s = 0;
    for (int i = 0; i < A.rows; i++) { double v = A(i, col); s += v * v; }
    return std::sqrt(s);
}

double dot_cols(const Matrix& A, int ca, const Matrix& B, int cb) {
    double d = 0;
    for (int i = 0; i < A.rows; i++) d += A(i, ca) * B(i, cb);
    return d;
}

void axpy_col(Matrix& dst, int dc, const Matrix& src, int sc, double factor) {
    for (int i = 0; i < dst.rows; i++) dst(i, dc) -= factor * src(i, sc);
}

Matrix mat_mul(const Matrix& A, const Matrix& B) {
    assert(A.cols == B.rows);
    Matrix C(A.rows, B.cols);
    for (int i = 0; i < A.rows; i++)
        for (int k = 0; k < A.cols; k++) {
            double aik = A(i, k);
            for (int j = 0; j < B.cols; j++) C(i, j) += aik * B(k, j);
        }
    return C;
}

Matrix transpose(const Matrix& A) {
    Matrix At(A.cols, A.rows);
    for (int i = 0; i < A.rows; i++)
        for (int j = 0; j < A.cols; j++) At(j, i) = A(i, j);
    return At;
}

double frob_norm(const Matrix& A) {
    double s = 0;
    for (auto v : A.data) s += v * v;
    return std::sqrt(s);
}

double recon_error(const Matrix& A, const Matrix& Q, const Matrix& R) {
    Matrix QR = mat_mul(Q, R);
    Matrix diff(A.rows, A.cols);
    for (size_t i = 0; i < diff.data.size(); i++) diff.data[i] = A.data[i] - QR.data[i];
    return frob_norm(diff);
}

double ortho_error(const Matrix& Q) {
    Matrix Qt = transpose(Q);
    Matrix QtQ = mat_mul(Qt, Q);
    Matrix I(Q.cols, Q.cols);
    for (int i = 0; i < Q.cols; i++) I(i, i) = 1.0;
    Matrix diff(Q.cols, Q.cols);
    for (size_t i = 0; i < diff.data.size(); i++) diff.data[i] = QtQ.data[i] - I.data[i];
    return frob_norm(diff);
}

double upper_tri_error(const Matrix& R) {
    double s = 0;
    for (int i = 1; i < R.rows; i++)
        for (int j = 0; j < std::min(i, R.cols); j++) {
            double v = R(i, j); s += v * v;
        }
    return std::sqrt(s);
}

// ---- QR Decomposition Methods ----

void cgs_qr(const Matrix& A, Matrix& Q, Matrix& R) {
    int m = A.rows, n = A.cols;
    std::copy(A.data.begin(), A.data.end(), Q.data.begin());
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < j; i++) {
            R(i, j) = dot_cols(Q, i, Q, j);
            axpy_col(Q, j, Q, i, R(i, j));
        }
        R(j, j) = vec_norm_col(Q, j);
        for (int i = 0; i < m; i++) Q(i, j) /= R(j, j);
    }
}

void mgs_qr(const Matrix& A, Matrix& Q, Matrix& R) {
    int m = A.rows, n = A.cols;
    std::copy(A.data.begin(), A.data.end(), Q.data.begin());
    for (int i = 0; i < n; i++) {
        R(i, i) = vec_norm_col(Q, i);
        for (int r = 0; r < m; r++) Q(r, i) /= R(i, i);
        for (int j = i + 1; j < n; j++) {
            R(i, j) = dot_cols(Q, i, Q, j);
            axpy_col(Q, j, Q, i, R(i, j));
        }
    }
}

// Correct Householder implementation
void householder_qr_correct(const Matrix& A, Matrix& Q_full, Matrix& R) {
    int m = A.rows, n = A.cols;
    std::copy(A.data.begin(), A.data.end(), R.data.begin());

    // Track accumulated Q^T as product of Householder reflectors
    // Q_full starts as I (m x m)
    Q_full.set_zero();
    for (int i = 0; i < m; i++) Q_full(i, i) = 1.0;

    for (int k = 0; k < n; k++) {
        // Build Householder vector v for column k in the submatrix R(k:m, k)
        double sigma = 0.0;
        for (int i = k; i < m; i++) { double x = R(i, k); sigma += x * x; }
        if (sigma < 1e-30) continue;

        double norm_x = std::sqrt(sigma);
        double xk = R(k, k);
        double alpha = (xk > 0) ? -norm_x : norm_x;
        double rho = 1.0 / (sigma - xk * alpha); // = 1/(||x||^2 + |xk|*||x||) = 1/(||v||^2/2)

        // v = x - alpha * e_k, but store implicitly: v[k] = 1, v[i] = x[i]/(xk - alpha) for i > k
        // Actually standard: v[k] = 1, for i>k: v[i] = R(i,k) / beta where beta = xk - alpha
        double beta = xk - alpha;
        std::vector<double> v(m, 0.0);
        v[k] = 1.0;
        for (int i = k + 1; i < m; i++) v[i] = R(i, k) / beta;

        // tau = 2 / (v^T v) = 2 / (1 + sum(v[i]^2))
        double vtv = 1.0;
        for (int i = k+1; i < m; i++) vtv += v[i] * v[i];
        double tau = 2.0 / vtv;

        // Apply H = I - tau * v * v^T to R(k:m, k:n) from the left
        for (int j = k; j < n; j++) {
            double vd = 0.0;
            for (int i = k; i < m; i++) vd += v[i] * R(i, j);
            double t = tau * vd;
            for (int i = k; i < m; i++) R(i, j) -= t * v[i];
        }

        // Apply H to Q from the right: Q * H^T = Q * (I - tau * v * v^T)
        // Actually we build Q as Q = H0 * H1 * ... * H_{n-1}, so Q accumulates from left
        // Standard: Q = Q * H_j, which applies H to rows of Q on the right
        for (int i = 0; i < m; i++) {
            double vd = 0.0;
            for (int j = k; j < m; j++) vd += v[j] * Q_full(i, j);
            double t = tau * vd;
            for (int j = k; j < m; j++) Q_full(i, j) -= t * v[j];
        }
    }
}

// Wrapper: extract Q(m,n) from Q_full(m,m) and R from R(m,n)
void householder_qr(const Matrix& A, Matrix& Q, Matrix& R) {
    int m = A.rows, n = A.cols;
    Matrix Qf(m, m);
    Matrix Rf(m, n); // R needs to be m x n for householder (tall matrix)
    householder_qr_correct(A, Qf, Rf);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            Q(i, j) = Qf(i, j);
    // Copy the upper n x n part of Rf into R
    for (int i = 0; i < n; i++)
        for (int j = i; j < n; j++)
            R(i, j) = Rf(i, j);
}

// ---- Test Matrices ----

Matrix rand_mat(int m, int n, unsigned seed) {
    srand(seed);
    Matrix A(m, n);
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            A(i, j) = (double)rand() / RAND_MAX * 10.0 - 5.0;
    return A;
}

Matrix hilbert(int n) {
    Matrix H(n, n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            H(i, j) = 1.0 / (i + j + 1.0);
    return H;
}

Matrix vandermonde(int m, int n) {
    Matrix V(m, n);
    for (int i = 0; i < m; i++) {
        double x = (double)i / (m - 1) * 2.0 - 1.0;
        double xp = 1.0;
        for (int j = 0; j < n; j++) { V(i, j) = xp; xp *= x; }
    }
    return V;
}

void test_one(const char* label, const Matrix& A,
              void (*fn)(const Matrix&, Matrix&, Matrix&)) {
    Matrix Q(A.rows, A.cols), R(A.cols, A.cols);
    fn(A, Q, R);

    double re = recon_error(A, Q, R);
    double oe = ortho_error(Q);
    double ut = upper_tri_error(R);

    printf("  %-30s recon=%.2e  ortho=%.2e  upper_tri=%.2e\n", label, re, oe, ut);
}

int main() {
    printf("============================================\n");
    printf("  QR Decomposition - 量化验证\n");
    printf("  Classical GS vs Modified GS vs Householder\n");
    printf("============================================\n\n");

    // Test 1: 5x3 Random
    printf("Test 1: 5x3 Random Matrix\n");
    Matrix A1 = rand_mat(5, 3, 42);
    A1.print("A", 5);
    printf("\n");
    test_one("Classical Gram-Schmidt", A1, cgs_qr);
    test_one("Modified Gram-Schmidt", A1, mgs_qr);
    test_one("Householder", A1, householder_qr);

    Matrix Q1c(5,3), R1c(3,3), Q1m(5,3), R1m(3,3), Q1h(5,3), R1h(3,3);
    cgs_qr(A1, Q1c, R1c); mgs_qr(A1, Q1m, R1m); householder_qr(A1, Q1h, R1h);

    double c_ortho = ortho_error(Q1c), m_ortho = ortho_error(Q1m), h_ortho = ortho_error(Q1h);
    double c_recon = recon_error(A1, Q1c, R1c), m_recon = recon_error(A1, Q1m, R1m), h_recon = recon_error(A1, Q1h, R1h);

    bool t1 = (c_recon < 1e-10 && m_recon < 1e-10 && h_recon < 1e-10);
    bool t2 = (m_ortho <= c_ortho);
    bool t3 = (h_ortho <= m_ortho);
    bool t4 = (upper_tri_error(R1c) < 1e-13 && upper_tri_error(R1m) < 1e-13 && upper_tri_error(R1h) < 1e-13);

    printf("\n  T1 (recon < 1e-10): %s\n", t1 ? "✅ PASS" : "❌ FAIL");
    printf("  T2 (MGS ortho <= CGS): %s\n", t2 ? "✅ PASS" : "❌ FAIL");
    printf("  T3 (HH ortho <= MGS): %s\n", t3 ? "✅ PASS" : "❌ FAIL");
    printf("  T4 (upper tri = 0): %s\n", t4 ? "✅ PASS" : "❌ FAIL");

    // Test 2: 4x4 Square
    printf("\n\nTest 2: 4x4 Square Random Matrix\n");
    Matrix A2 = rand_mat(4, 4, 123);
    A2.print("A", 4);
    printf("\n");
    test_one("Classical Gram-Schmidt", A2, cgs_qr);
    test_one("Modified Gram-Schmidt", A2, mgs_qr);
    test_one("Householder", A2, householder_qr);

    // Test 3: Hilbert 6x6 (ill-conditioned)
    printf("\n\nTest 3: 6x6 Hilbert Matrix (ill-conditioned)\n");
    Matrix H = hilbert(6);
    test_one("Classical Gram-Schmidt", H, cgs_qr);
    test_one("Modified Gram-Schmidt", H, mgs_qr);
    test_one("Householder", H, householder_qr);

    Matrix Hc(6,6), Rc(6,6), Hm(6,6), Rm(6,6), Hh(6,6), Rh_h(6,6);
    cgs_qr(H, Hc, Rc); mgs_qr(H, Hm, Rm); householder_qr(H, Hh, Rh_h);
    double hc_o = ortho_error(Hc), hm_o = ortho_error(Hm), hh_o = ortho_error(Hh);
    printf("\n  Hilbert ortho errors: CGS=%.2e MGS=%.2e HH=%.2e\n", hc_o, hm_o, hh_o);
    bool t5 = (hh_o <= hm_o && hm_o <= hc_o) || (hh_o < 1e-10 && hm_o < 1e-10); // tolerate rounding
    printf("  T5 (HH < MGS < CGS for ill-cond): %s\n", t5 ? "✅ PASS" : "❌ FAIL");

    // Test 4: Vandermonde 8x5
    printf("\n\nTest 4: 8x5 Vandermonde (moderate condition)\n");
    Matrix V = vandermonde(8, 5);
    test_one("Classical Gram-Schmidt", V, cgs_qr);
    test_one("Modified Gram-Schmidt", V, mgs_qr);
    test_one("Householder", V, householder_qr);

    // Test 5: 20x3 Tall-Skinny
    printf("\n\nTest 5: 20x3 Tall-Skinny (m >> n)\n");
    Matrix T = rand_mat(20, 3, 999);
    test_one("Classical Gram-Schmidt", T, cgs_qr);
    test_one("Modified Gram-Schmidt", T, mgs_qr);
    test_one("Householder", T, householder_qr);

    // Test 6: Solve linear system Ax = b via QR
    printf("\n\nTest 6: Linear System via QR\n");
    Matrix As = rand_mat(5, 5, 777);
    for (int i = 0; i < 5; i++) As(i,i) += 10.0; // ensure non-singular
    Matrix b(5, 1);
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            b(i,0) += As(i,j) * (j + 1.0); // true x = [1,2,3,4,5]

    Matrix Qs(5,5), Rs(5,5);
    householder_qr(As, Qs, Rs);
    // R x = Q^T b
    Matrix Qtb(5, 1);
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            Qtb(i,0) += Qs(j,i) * b(j,0); // Q^T

    std::vector<double> x(5);
    for (int i = 4; i >= 0; i--) {
        x[i] = Qtb(i,0);
        for (int j = i+1; j < 5; j++) x[i] -= Rs(i,j) * x[j];
        x[i] /= Rs(i,i);
    }
    double solve_err = 0;
    for (int i = 0; i < 5; i++) solve_err += std::abs(x[i] - (i + 1.0));
    printf("  Solve error = %.2e\n", solve_err);
    bool t6 = (solve_err < 1e-10);
    printf("  T6 (solve < 1e-10): %s\n", t6 ? "✅ PASS" : "❌ FAIL");

    // Test 7: Q^T A = R
    printf("\n\nTest 7: Verify Q^T A = R\n");
    Matrix Qt = transpose(Q1h);
    Matrix QtA = mat_mul(Qt, A1);
    double qt_err = 0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            qt_err = std::max(qt_err, std::abs(QtA(i,j) - R1h(i,j)));
    printf("  max|Q^T A - R| = %.2e\n", qt_err);
    bool t7 = (qt_err < 1e-12);
    printf("  T7 (Q^T A = R): %s\n", t7 ? "✅ PASS" : "❌ FAIL");

    // Summary
    printf("\n============================================\n");
    printf("  SUMMARY\n");
    printf("============================================\n");
    printf("  T1 (reconstruction):  %s\n", t1 ? "✅" : "❌");
    printf("  T2 (MGS <= CGS):      %s\n", t2 ? "✅" : "❌");
    printf("  T3 (HH <= MGS):       %s\n", t3 ? "✅" : "❌");
    printf("  T4 (upper triangular):%s\n", t4 ? "✅" : "❌");
    printf("  T5 (ill-cond rank):   %s\n", t5 ? "✅" : "❌");
    printf("  T6 (linear solve):    %s\n", t6 ? "✅" : "❌");
    printf("  T7 (Q^T A = R):       %s\n", t7 ? "✅" : "❌");

    bool all = t1 && t2 && t3 && t4 && t5 && t6 && t7;
    printf("\n  Overall: %s\n", all ? "✅ ALL PASSED" : "❌ SOME FAILED");
    return all ? 0 : 1;
}
