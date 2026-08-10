#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <iomanip>
#include <random>
#include <fstream>
#include <sstream>
#include <limits>

// ========================= Data Types =========================
using Matrix = std::vector<std::vector<double>>;
const double EPS = 1e-14;

// ========================= Utilities =========================
double dot(const std::vector<double>& a, const std::vector<double>& b) {
    double s = 0;
    for (size_t i = 0; i < a.size(); i++) s += a[i] * b[i];
    return s;
}

double norm(const std::vector<double>& v) {
    return std::sqrt(dot(v, v));
}

double frobeniusNorm(const Matrix& A) {
    double sum = 0;
    for (auto& row : A)
        for (double v : row) sum += v * v;
    return std::sqrt(sum);
}

Matrix transpose(const Matrix& A) {
    int m = (int)A.size(), n = (int)A[0].size();
    Matrix AT(n, std::vector<double>(m));
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            AT[j][i] = A[i][j];
    return AT;
}

// ========================= Jacobi Eigenvalue Decomposition =========================
// Classic Jacobi iteration for real symmetric matrices
// Returns eigenvalues and eigenvectors sorted descending
void jacobiEigen(const Matrix& A, std::vector<double>& evals, Matrix& evecs) {
    int n = (int)A.size();
    Matrix V(n, std::vector<double>(n, 0));
    for (int i = 0; i < n; i++) V[i][i] = 1;

    Matrix M = A;  // working copy

    for (int iter = 0; iter < 500; iter++) {
        // Find largest off-diagonal element
        int p = 0, q = 1;
        double maxVal = 0;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double v = std::abs(M[i][j]);
                if (v > maxVal) { maxVal = v; p = i; q = j; }
            }
        }
        if (maxVal < EPS) break;

        // Compute rotation angle
        double theta;
        if (std::abs(M[p][p] - M[q][q]) < EPS) {
            theta = (M[p][q] > 0) ? M_PI / 4.0 : -M_PI / 4.0;
        } else {
            theta = 0.5 * std::atan2(2.0 * M[p][q], M[p][p] - M[q][q]);
        }
        double c = std::cos(theta), s = std::sin(theta);

        // Apply J^T * M * J (classic Jacobi rotation)
        // Step 1: J^T * M (left multiply by J^T) — J^T = [[c,s],[-s,c]]
        std::vector<double> oldRowP = M[p], oldRowQ = M[q];
        for (int j = 0; j < n; j++) {
            M[p][j] = c * oldRowP[j] + s * oldRowQ[j];
            M[q][j] = -s * oldRowP[j] + c * oldRowQ[j];
        }
        // Step 2: (J^T*M) * J (right multiply by J) — J = [[c,-s],[s,c]]
        for (int i = 0; i < n; i++) {
            double old_mip = M[i][p];
            double old_miq = M[i][q];
            M[i][p] = c * old_mip + s * old_miq;
            M[i][q] = -s * old_mip + c * old_miq;
        }

        // Update eigenvector matrix V = V * J
        for (int i = 0; i < n; i++) {
            double vip = V[i][p], viq = V[i][q];
            V[i][p] = c * vip + s * viq;
            V[i][q] = -s * vip + c * viq;
        }
    }

    // Extract eigenvalues from diagonal
    evals.resize(n);
    for (int i = 0; i < n; i++) evals[i] = M[i][i];

    // Sort eigenvalues descending (and eigenvectors accordingly)
    std::vector<int> idx(n);
    for (int i = 0; i < n; i++) idx[i] = i;
    std::sort(idx.begin(), idx.end(), [&](int a, int b) { return evals[a] > evals[b]; });

    std::vector<double> sortedEvals(n);
    evecs.assign(n, std::vector<double>(n));
    for (int i = 0; i < n; i++) {
        sortedEvals[i] = evals[idx[i]];
        for (int j = 0; j < n; j++)
            evecs[j][i] = V[j][idx[i]];
    }
    evals = sortedEvals;
}

// ========================= SVD: Two-Step Method =========================
// Step 1: Compute A^T*A, find eigenpairs (using Jacobi) -> V, S^2
// Step 2: Compute U = A * V * S^{-1}
// For m > p = min(m,n), extend U with Gram-Schmidt
void svd(const Matrix& A, Matrix& U, std::vector<double>& S, Matrix& V) {
    int m = (int)A.size(), n = (int)A[0].size();
    int p = std::min(m, n);

    // --- Step 1: A^T * A (n x n) ---
    Matrix ATA(n, std::vector<double>(n, 0));
    for (int i = 0; i < n; i++) {
        for (int j = i; j < n; j++) {
            double sum = 0;
            for (int k = 0; k < m; k++) sum += A[k][i] * A[k][j];
            ATA[i][j] = ATA[j][i] = sum;
        }
    }

    // Eigen decomposition of ATA
    std::vector<double> evals;
    jacobiEigen(ATA, evals, V);

    // Singular values = sqrt(eigenvalues)
    S.resize(p);
    for (int i = 0; i < p; i++) {
        S[i] = (evals[i] > 0) ? std::sqrt(evals[i]) : 0.0;
    }

    // --- Step 2: Compute U ---
    // For each singular value > 0: u_i = A * v_i / s_i
    U.assign(m, std::vector<double>(p, 0));
    for (int i = 0; i < p; i++) {
        if (S[i] > 1e-12) {
            double invS = 1.0 / S[i];
            for (int row = 0; row < m; row++) {
                double sum = 0;
                for (int col = 0; col < n; col++)
                    sum += A[row][col] * V[col][i];
                U[row][i] = sum * invS;
            }
        }
    }

    // Extend U to m × m if m > p, AND fill zero columns from zero singular values
    if (m > p || true) {  // always normalize U, filling zero-columns
        Matrix Uext(m, std::vector<double>(m, 0));
        for (int i = 0; i < p; i++)
            for (int row = 0; row < m; row++)
                Uext[row][i] = U[row][i];

        for (int col = 0; col < m; col++) {
            // Check if column is valid (non-zero enough)
            double existingNorm = 0;
            for (int row = 0; row < m; row++)
                existingNorm += Uext[row][col] * Uext[row][col];

            if (existingNorm > 1e-10) continue;  // already good

            // Try identity vectors until we find a good one
            for (int trial = 0; trial < m + 10; trial++) {
                std::vector<double> vec(m, 0);
                if (trial < m) vec[trial] = 1.0;
                else {
                    // Random fallback
                    std::mt19937 rng(trial * 131 + col * 271);
                    std::uniform_real_distribution<double> dist(-1, 1);
                    for (int row = 0; row < m; row++) vec[row] = dist(rng);
                }

                // Modified Gram-Schmidt against ALL previous valid columns
                for (int pass = 0; pass < 3; pass++) {
                    for (int j = 0; j < col; j++) {
                        double nj = 0;
                        for (int row = 0; row < m; row++)
                            nj += Uext[row][j] * Uext[row][j];
                        if (nj < 1e-10) continue;

                        double proj = 0;
                        for (int row = 0; row < m; row++)
                            proj += vec[row] * Uext[row][j];
                        for (int row = 0; row < m; row++)
                            vec[row] -= proj * Uext[row][j];
                    }
                }

                double colNorm = 0;
                for (int row = 0; row < m; row++)
                    colNorm += vec[row] * vec[row];
                colNorm = std::sqrt(colNorm);

                if (colNorm > 1e-10) {
                    for (int row = 0; row < m; row++)
                        Uext[row][col] = vec[row] / colNorm;
                    break;
                }
            }
        }
        U = Uext;
    }
    
    // Also extend S to length max(m,n) for reconstruction compatibility
    // (reconstructionError only uses first p, this is for eigenvalueMatch)
}

// ========================= Verification Functions =========================

// Relative reconstruction error: ||A - U*S*V^T||_F / ||A||_F
// Uses only p = min(m,n) components of S for reconstruction
double reconstructionError(const Matrix& A, const Matrix& U, const std::vector<double>& S, const Matrix& V) {
    int m = (int)A.size(), n = (int)A[0].size();
    int p = (int)S.size();  // p = min(m,n)

    Matrix USV(m, std::vector<double>(n, 0));
    for (int i = 0; i < m; i++) {
        for (int k = 0; k < p; k++) {
            if (S[k] > EPS) {
                double uik_s = U[i][k] * S[k];
                for (int j = 0; j < n; j++)
                    USV[i][j] += uik_s * V[j][k];
            }
        }
    }

    double err = 0;
    for (int i = 0; i < m; i++)
        for (int j = 0; j < n; j++)
            err += (A[i][j] - USV[i][j]) * (A[i][j] - USV[i][j]);

    return std::sqrt(err) / frobeniusNorm(A);
}

// Orthogonality check: max |U^T*U - I|
double orthogonalityError(const Matrix& X) {
    int rows = (int)X.size(), cols = (int)X[0].size();
    double maxErr = 0;
    for (int i = 0; i < cols; i++) {
        for (int j = 0; j < cols; j++) {
            double d = 0;
            for (int k = 0; k < rows; k++) d += X[k][i] * X[k][j];
            double expected = (i == j) ? 1.0 : 0.0;
            maxErr = std::max(maxErr, std::abs(d - expected));
        }
    }
    return maxErr;
}

// Verify SVD property: for non-zero S[k], A*V[:,k] = U[:,k] * S[k]
// For zero S[k], verify A*V[:,k] ≈ 0
double svdPropertyError(const Matrix& A, const Matrix& U, const std::vector<double>& S, const Matrix& V) {
    int m = (int)A.size(), n = (int)A[0].size(), p = (int)S.size();
    double maxErr = 0;
    for (int k = 0; k < p; k++) {
        if (S[k] > 1e-10) {
            for (int i = 0; i < m; i++) {
                double av = 0;
                for (int jj = 0; jj < n; jj++) av += A[i][jj] * V[jj][k];
                maxErr = std::max(maxErr, std::abs(av - U[i][k] * S[k]));
            }
        } else {
            // For zero singular values, verify A*V_k ≈ 0
            for (int i = 0; i < m; i++) {
                double av = 0;
                for (int jj = 0; jj < n; jj++) av += A[i][jj] * V[jj][k];
                maxErr = std::max(maxErr, std::abs(av));
            }
        }
    }
    return maxErr;
}

// Singular value monotonicity: S[0] >= S[1] >= ... >= S[p-1]
bool singularValuesMonotonic(const std::vector<double>& S) {
    for (size_t i = 1; i < S.size(); i++)
        if (S[i] > S[i-1] + EPS) return false;
    return true;
}

// Verify singular values match the square root of eigenvalues of ATA and AAT
double eigenvalueMatchCheck(const Matrix& A, const std::vector<double>& S) {
    int m = (int)A.size(), n = (int)A[0].size(), p = (int)S.size();

    // Compute AAT (m×m) eigenvalues using Jacobi
    Matrix AAT(m, std::vector<double>(m, 0));
    for (int i = 0; i < m; i++)
        for (int j = i; j < m; j++) {
            double sum = 0;
            for (int k = 0; k < n; k++) sum += A[i][k] * A[j][k];
            AAT[i][j] = AAT[j][i] = sum;
        }

    std::vector<double> evalsAAT;
    Matrix dummy;
    jacobiEigen(AAT, evalsAAT, dummy);

    // Compare: first p eigenvalues of AAT should equal S^2
    // (AAT is m×m, with m eigenvalues; only first p = min(m,n) are non-zero)
    int compareCount = std::min(p, m);
    double maxDiff = 0;
    for (int i = 0; i < compareCount; i++) {
        maxDiff = std::max(maxDiff, std::abs(S[i] * S[i] - evalsAAT[i]));
    }
    // Also verify that remaining AAT eigenvalues are ~0
    for (int i = compareCount; i < m; i++) {
        maxDiff = std::max(maxDiff, evalsAAT[i]);  // should be ~0
    }
    return maxDiff;
}

// ========================= Visualization =========================

void generateVisualization(const Matrix& A, const Matrix& U, const std::vector<double>& S,
                           const Matrix& V, const std::string& filename) {
    int m = (int)A.size(), n = (int)A[0].size();
    int p = (int)S.size();

    // Color mapping: divergent blue-white-red
    double gmin = 1e30, gmax = -1e30;
    auto update = [&](double v) {
        if (std::isfinite(v)) { gmin = std::min(gmin, v); gmax = std::max(gmax, v); }
    };
    for (auto& row : A) for (double v : row) update(v);
    for (auto& row : U) for (double v : row) update(v);
    for (double v : S) update(v);
    for (auto& row : V) for (double v : row) update(v);
    if (gmax - gmin < EPS) gmax = gmin + 1;

    // Center zero for divergent colormap
    double absMax = std::max(std::abs(gmin), std::abs(gmax));

    auto colorMap = [&](double v) -> int {
        double t = v / (absMax + EPS);  // in [-1, 1]
        int r, g, b;
        if (t < 0) {
            double s = 1 + t;  // [0, 1]
            r = (int)(255 * s);
            g = (int)(255 * s);
            b = 255;
        } else {
            double s = 1 - t;
            r = 255;
            g = (int)(255 * s);
            b = (int)(255 * s);
        }
        return (r << 16) | (g << 8) | b;
    };

    int cw = 30, ch = 30, pad = 10, gap = 5;

    // Layout: [A m×n] [U m×m] [S m×n] [V n×n]
    int imgW = (n + m + n + n) * cw + 3 * gap + 2 * pad;
    int imgH = std::max({m, m, m, n}) * ch + 2 * pad;

    std::vector<int> img(imgW * imgH, 0xFFFFFF);

    auto draw = [&](const Matrix& mat, int ox, int oy) {
        for (int i = 0; i < (int)mat.size(); i++)
            for (int j = 0; j < (int)mat[0].size(); j++) {
                int c = colorMap(mat[i][j]);
                for (int dy = 0; dy < ch; dy++)
                    for (int dx = 0; dx < cw; dx++)
                        img[(oy + i * ch + dy) * imgW + (ox + j * cw + dx)] = c;
            }
    };

    auto drawDiag = [&](int ox, int oy) {
        for (int i = 0; i < m; i++)
            for (int j = 0; j < n; j++) {
                int c = (i == j && i < p) ? colorMap(S[i]) : 0xFFFFFF;
                for (int dy = 0; dy < ch; dy++)
                    for (int dx = 0; dx < cw; dx++)
                        img[(oy + i * ch + dy) * imgW + (ox + j * cw + dx)] = c;
            }
    };

    int x = pad, y = pad;
    draw(A, x, y); x += n * cw + gap;
    draw(U, x, y); x += m * cw + gap;
    drawDiag(x, y); x += n * cw + gap;
    draw(V, x, y);

    // PPM output
    std::ofstream out(filename);
    out << "P3\n" << imgW << " " << imgH << "\n255\n";
    for (int py = 0; py < imgH; py++)
        for (int px = 0; px < imgW; px++) {
            int c = img[py * imgW + px];
            out << ((c >> 16) & 0xFF) << " " << ((c >> 8) & 0xFF) << " " << (c & 0xFF) << " ";
        }
    out.close();
}

// ========================= Main Test Suite =========================

void runTest(const std::string& name, const Matrix& A) {
    int m = (int)A.size(), n = (int)A[0].size();
    std::cout << "\n--- " << name << " (" << m << "x" << n << ") ---\n";

    // Print matrix
    std::cout << "A:\n";
    for (auto& row : A) {
        for (double v : row) std::cout << std::setw(8) << std::fixed << std::setprecision(3) << v;
        std::cout << "\n";
    }

    Matrix U, V;
    std::vector<double> S;
    svd(A, U, S, V);

    std::cout << "Singular values (descending): ";
    for (double s : S) std::cout << std::setprecision(6) << s << " ";
    std::cout << "\n";

    // Quantitative verifications
    double recErr = reconstructionError(A, U, S, V);
    double orthErrU = orthogonalityError(U);
    double orthErrV = orthogonalityError(V);
    double propErr = svdPropertyError(A, U, S, V);
    bool monotonic = singularValuesMonotonic(S);
    double eigenMatch = eigenvalueMatchCheck(A, S);

    std::cout << "\n--- Verification Results ---\n";
    std::cout << "Reconstruction error (rel): " << std::scientific << recErr << "\n";
    std::cout << "U orthogonality error:        " << std::scientific << orthErrU << "\n";
    std::cout << "V orthogonality error:        " << std::scientific << orthErrV << "\n";
    std::cout << "AV=US property error:         " << std::scientific << propErr << "\n";
    std::cout << "Singular values monotonic:     " << (monotonic ? "PASS" : "FAIL") << "\n";
    std::cout << "Eigenvalue match (S^2 vs AAT): " << std::scientific << eigenMatch << "\n";

    // Pass/fail criteria (adaptive thresholds for rank-deficient matrices)
    int rank = 0;
    for (double s : S) if (s > 1e-6) rank++;
    double rtol = (rank < (int)S.size()) ? 1e-6 : 1e-10;  // relaxed for rank-deficient
    
    bool pass = true;
    auto check = [&](double val, double threshold, const char* label) {
        bool ok = (val < threshold);
        std::cout << "  [" << (ok ? "PASS" : "FAIL") << "] " << label << " < " << std::scientific << threshold << " : " << val << "\n";
        if (!ok) pass = false;
    };

    check(recErr, rtol, "Reconstruction error");
    check(orthErrU, 1e-10, "U orthogonality");
    check(orthErrV, 1e-10, "V orthogonality");
    check(propErr, rtol, "AV=US property");
    check(eigenMatch, 1e-8, "S^2 vs AAT match");

    if (!monotonic) { std::cout << "  [FAIL] Singular values monotonic\n"; pass = false; }

    std::cout << "\nOverall: " << (pass ? "PASS" : "FAIL") << "\n";
}

int main() {
    std::cout << "====================================================\n";
    std::cout << "  SVD — Singular Value Decomposition\n";
    std::cout << "  Two-step method (ATA Jacobi + AV = US)\n";
    std::cout << "====================================================\n";

    // ===== Test 1: Small square matrix (3x3) =====
    Matrix A1 = {
        {4.0, 2.0, 1.0},
        {2.0, 5.0, 3.0},
        {1.0, 3.0, 6.0}
    };
    runTest("Square 3x3 (symmetric positive definite)", A1);

    // ===== Test 2: Tall rectangular matrix (6x4) =====
    Matrix A2 = {
        { 1.0,  2.0,  3.0,  4.0},
        { 5.0,  6.0,  7.0,  8.0},
        { 9.0, 10.0, 11.0, 12.0},
        {13.0, 14.0, 15.0, 16.0},
        {17.0, 18.0, 19.0, 20.0},
        {21.0, 22.0, 23.0, 24.0}
    };
    runTest("Tall 6x4 (rank-deficient, nearly)", A2);

    // ===== Test 3: Wide rectangular (3x5) =====
    Matrix A3 = {
        {2.0, -1.0,  3.0,  0.0,  1.0},
        {1.0,  0.0, -2.0,  4.0, -1.0},
        {3.0,  2.0,  1.0, -1.0,  2.0}
    };
    runTest("Wide 3x5", A3);

    // ===== Test 4: Random 8x6 matrix =====
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-5, 5);
    Matrix A4(8, std::vector<double>(6));
    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 6; j++)
            A4[i][j] = dist(rng);
    runTest("Random 8x6", A4);

    // ===== Test 5: Rank-1 matrix (known SVD) =====
    Matrix A5 = {
        {2.0, 4.0, 6.0},
        {1.0, 2.0, 3.0},
        {3.0, 6.0, 9.0},
        {-1.0, -2.0, -3.0}
    };
    runTest("Rank-1 4x3", A5);

    // ===== Visualization =====
    std::cout << "\n--- Generating visualization ---\n";
    // Use a 10x7 random matrix for nice visualization
    Matrix Aviz(10, std::vector<double>(7));
    std::mt19937 rng2(123);
    std::uniform_real_distribution<double> dist2(-3, 3);
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 7; j++)
            Aviz[i][j] = dist2(rng2);

    Matrix Uviz, Vviz;
    std::vector<double> Sviz;
    svd(Aviz, Uviz, Sviz, Vviz);
    generateVisualization(Aviz, Uviz, Sviz, Vviz, "svd_visualization.ppm");
    std::cout << "Visualization saved to svd_visualization.ppm\n";

    std::cout << "\n=== All tests completed ===\n";
    return 0;
}
