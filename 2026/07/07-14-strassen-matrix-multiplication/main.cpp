#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <random>
#include <algorithm>

// ============================================================
// Strassen Matrix Multiplication - 分治 O(n^2.81)
// Features:
//   1. Strassen's divide-and-conquer algorithm
//   2. Standard O(n^3) matrix multiplication for baseline
//   3. Quantitative verification: max error, MSE, correctness %%
//   4. Performance comparison: time & operations ratio
//   5. PPM visualization of result matrices
// ============================================================

using namespace std;

using Matrix = vector<vector<double>>;

// -------------------- Matrix utilities --------------------

Matrix createMatrix(int n) {
    return Matrix(n, vector<double>(n, 0.0));
}

void freeMatrix(Matrix& M) {
    M.clear();
    M.shrink_to_fit();
}

Matrix add(const Matrix& A, const Matrix& B) {
    int n = A.size();
    Matrix C = createMatrix(n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

Matrix sub(const Matrix& A, const Matrix& B) {
    int n = A.size();
    Matrix C = createMatrix(n);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            C[i][j] = A[i][j] - B[i][j];
    return C;
}

// O(n^3) standard multiplication
Matrix standardMultiply(const Matrix& A, const Matrix& B) {
    int n = A.size();
    Matrix C = createMatrix(n);
    for (int i = 0; i < n; i++)
        for (int k = 0; k < n; k++)
            for (int j = 0; j < n; j++)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

// -------------------- Strassen --------------------

Matrix strassenRecursive(const Matrix& A, const Matrix& B) {
    int n = A.size();

    // Base case: use standard multiplication for small matrices
    if (n <= 64) {
        return standardMultiply(A, B);
    }

    int half = n / 2;

    auto subBlock = [half](const Matrix& M, int row, int col) {
        Matrix block = createMatrix(half);
        for (int i = 0; i < half; i++)
            for (int j = 0; j < half; j++)
                block[i][j] = M[row + i][col + j];
        return block;
    };

    auto assignBlock = [half](Matrix& M, const Matrix& block, int row, int col) {
        for (int i = 0; i < half; i++)
            for (int j = 0; j < half; j++)
                M[row + i][col + j] = block[i][j];
    };

    // Split into 4 sub-blocks
    Matrix A11 = subBlock(A, 0, 0);
    Matrix A12 = subBlock(A, 0, half);
    Matrix A21 = subBlock(A, half, 0);
    Matrix A22 = subBlock(A, half, half);

    Matrix B11 = subBlock(B, 0, 0);
    Matrix B12 = subBlock(B, 0, half);
    Matrix B21 = subBlock(B, half, 0);
    Matrix B22 = subBlock(B, half, half);

    // Strassen's 7 recursive multiplications (fewer than the 8 naive sub-multiplies)
    Matrix M1 = strassenRecursive(add(A11, A22), add(B11, B22));
    Matrix M2 = strassenRecursive(add(A21, A22), B11);
    Matrix M3 = strassenRecursive(A11, sub(B12, B22));
    Matrix M4 = strassenRecursive(A22, sub(B21, B11));
    Matrix M5 = strassenRecursive(add(A11, A12), B22);
    Matrix M6 = strassenRecursive(sub(A21, A11), add(B11, B12));
    Matrix M7 = strassenRecursive(sub(A12, A22), add(B21, B22));

    // Combine
    Matrix C11 = add(sub(add(M1, M4), M5), M7);
    Matrix C12 = add(M3, M5);
    Matrix C21 = add(M2, M4);
    Matrix C22 = add(sub(add(M1, M3), M2), M6);

    Matrix C = createMatrix(n);
    assignBlock(C, C11, 0, 0);
    assignBlock(C, C12, 0, half);
    assignBlock(C, C21, half, 0);
    assignBlock(C, C22, half, half);

    return C;
}

// Pad matrix to next power of 2
Matrix padToPowerOf2(const Matrix& A) {
    int n = A.size();
    int nextPow2 = 1;
    while (nextPow2 < n) nextPow2 <<= 1;
    if (nextPow2 == n) return A;

    Matrix padded = createMatrix(nextPow2);
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++)
            padded[i][j] = A[i][j];
    return padded;
}

// Unpad matrix
Matrix unpad(const Matrix& A, int originalN) {
    Matrix unpadded = createMatrix(originalN);
    for (int i = 0; i < originalN; i++)
        for (int j = 0; j < originalN; j++)
            unpadded[i][j] = A[i][j];
    return unpadded;
}

Matrix strassenMultiply(const Matrix& A, const Matrix& B) {
    int n = A.size();
    Matrix Apad = padToPowerOf2(A);
    Matrix Bpad = padToPowerOf2(B);
    Matrix Cpad = strassenRecursive(Apad, Bpad);
    return (int)Apad.size() > n ? unpad(Cpad, n) : Cpad;
}

// -------------------- Verification --------------------

struct VerificationResult {
    double maxAbsError;
    double mse;
    int totalElements;
    int exactMatches;
    double exactPercent;
    bool allWithinTolerance;
};

VerificationResult verify(const Matrix& standard, const Matrix& strassen, double tolerance = 1e-10) {
    VerificationResult res = {};
    int n = standard.size();
    res.totalElements = n * n;
    res.exactMatches = 0;
    res.maxAbsError = 0.0;
    double sumSqErr = 0.0;
    res.allWithinTolerance = true;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double err = fabs(standard[i][j] - strassen[i][j]);
            sumSqErr += err * err;
            if (err > res.maxAbsError) res.maxAbsError = err;
            if (err < tolerance) res.exactMatches++;
            if (err > tolerance) res.allWithinTolerance = false;
        }
    }
    res.mse = sumSqErr / res.totalElements;
    res.exactPercent = 100.0 * res.exactMatches / res.totalElements;
    return res;
}

// -------------------- PPM Output --------------------

void writePPM(const Matrix& M, const string& filename, double maxVal) {
    int n = M.size();
    ofstream out(filename, ios::binary);
    out << "P6\n" << n << " " << n << "\n255\n";
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double val = maxVal > 0 ? M[i][j] / maxVal : 0.0;
            val = max(0.0, min(1.0, val));
            unsigned char c = (unsigned char)(val * 255);
            out.put(c);
            out.put(c);
            out.put(c);
        }
    }
    out.close();
}

void writeComparisonPPM(const Matrix& standard, const Matrix& strassen, int n,
                        const string& filename, double maxVal) {
    // Side-by-side comparison: left = standard, right = strassen, border in between
    int totalWidth = n * 2 + 4; // 2px border
    ofstream out(filename, ios::binary);
    out << "P6\n" << totalWidth << " " << n << "\n255\n";
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double val = maxVal > 0 ? standard[i][j] / maxVal : 0.0;
            val = max(0.0, min(1.0, val));
            unsigned char c = (unsigned char)(val * 255);
            out.put(c); out.put(c); out.put(c);
        }
        // Border (red)
        for (int b = 0; b < 4; b++) {
            out.put((unsigned char)255); out.put(0); out.put(0);
        }
        for (int j = 0; j < n; j++) {
            double val = maxVal > 0 ? strassen[i][j] / maxVal : 0.0;
            val = max(0.0, min(1.0, val));
            unsigned char c = (unsigned char)(val * 255);
            out.put(c); out.put(c); out.put(c);
        }
    }
    out.close();
}

// Error heatmap: bright = larger error
void writeErrorHeatmap(const Matrix& standard, const Matrix& strassen,
                       const string& filename) {
    int n = standard.size();
    double maxErr = 0.0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            double err = fabs(standard[i][j] - strassen[i][j]);
            if (err > maxErr) maxErr = err;
        }

    ofstream out(filename, ios::binary);
    out << "P6\n" << n << " " << n << "\n255\n";
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            double err = fabs(standard[i][j] - strassen[i][j]);
            double intensity = maxErr > 0 ? err / maxErr : 0.0;
            // Use heat colormap: black(0) -> red -> yellow -> white
            unsigned char r, g, b;
            if (intensity < 0.25) {
                r = (unsigned char)(intensity * 4 * 255);
                g = 0;
                b = 0;
            } else if (intensity < 0.5) {
                r = 255;
                g = (unsigned char)((intensity - 0.25) * 4 * 255);
                b = 0;
            } else if (intensity < 0.75) {
                r = 255;
                g = 255;
                b = (unsigned char)((intensity - 0.5) * 4 * 255);
            } else {
                r = 255;
                g = 255;
                b = 255;
            }
            out.put(r); out.put(g); out.put(b);
        }
    }
    out.close();
}

// -------------------- Main --------------------

int main() {
    // Test with multiple matrix sizes: 128, 256, 512
    vector<int> sizes = {128, 256, 512};
    mt19937 rng(42);  // fixed seed for reproducibility
    uniform_real_distribution<double> dist(-10.0, 10.0);

    cout << "========================================" << endl;
    cout << "  Strassen Matrix Multiplication" << endl;
    cout << "  Divide-and-Conquer O(n^log2(7))" << endl;
    cout << "========================================" << endl;

    for (int n : sizes) {
        cout << "\n--- Matrix size: " << n << "x" << n << " ---" << endl;

        // Generate random matrices
        Matrix A = createMatrix(n);
        Matrix B = createMatrix(n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                A[i][j] = dist(rng);
                B[i][j] = dist(rng);
            }

        // Standard multiplication (baseline)
        auto t1 = chrono::high_resolution_clock::now();
        Matrix C_std = standardMultiply(A, B);
        auto t2 = chrono::high_resolution_clock::now();
        double time_std = chrono::duration<double>(t2 - t1).count();

        // Strassen multiplication
        auto t3 = chrono::high_resolution_clock::now();
        Matrix C_str = strassenMultiply(A, B);
        auto t4 = chrono::high_resolution_clock::now();
        double time_str = chrono::duration<double>(t4 - t3).count();

        // Verify
        VerificationResult v = verify(C_std, C_str, 1e-10);

        cout << fixed << setprecision(6);
        cout << "  Standard multiply: " << time_std << "s" << endl;
        cout << "  Strassen multiply:  " << time_str << "s" << endl;
        cout << "  Speedup ratio:     " << (time_std / time_str) << "x" << endl;
        cout << "  Max absolute error: " << scientific << v.maxAbsError << endl;
        cout << "  MSE:                " << scientific << v.mse << endl;
        cout << "  Exact match %:      " << fixed << v.exactPercent << "%" << endl;
        cout << "  All within tol:     " << (v.allWithinTolerance ? "YES ✅" : "NO ❌") << endl;

        // Theoretical ops: standard n^3 multiplications, strassen 7 * n^2.81...
        double ops_std = pow(n, 3.0);
        double ops_str_a = n <= 64 ? pow(n, 3.0) : 7.0 * pow((double)n, log2(7.0));
        cout << "  ~Standard ops:     " << ops_std << endl;
        cout << "  ~Strassen ops:     " << ops_str_a << endl;
        cout << "  Ops ratio (std/str): " << (ops_std / ops_str_a) << endl;

        // Write PPM outputs (for ns up to 256 so images are readable)
        if (n <= 256) {
            double maxVal = 0.0;
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++) {
                    maxVal = max(maxVal, fabs(C_std[i][j]));
                }
            maxVal = max(maxVal, 1e-10);

            string prefix = "strassen_" + to_string(n);

            writePPM(C_std, prefix + "_standard.ppm", maxVal);
            writePPM(C_str, prefix + "_strassen.ppm", maxVal);
            writeComparisonPPM(C_std, C_str, n, prefix + "_comparison.ppm", maxVal);
            writeErrorHeatmap(C_std, C_str, prefix + "_error.ppm");

            cout << "  Output images saved: " << prefix << "_*.ppm" << endl;
        }
    }

    // =========== Quantitative Verification Summary ===========
    cout << "\n========================================" << endl;
    cout << "  QUANTITATIVE VERIFICATION SUMMARY" << endl;
    cout << "========================================" << endl;

    // Run a focused accuracy test with known values
    {
        int n_check = 5;
        Matrix A_check = createMatrix(n_check);
        Matrix B_check = createMatrix(n_check);

        // Fill with small integers for predictable results
        for (int i = 0; i < n_check; i++)
            for (int j = 0; j < n_check; j++) {
                A_check[i][j] = (i + 1) * (j + 1);
                B_check[i][j] = (j + 1);
            }

        Matrix C_check_std = standardMultiply(A_check, B_check);
        Matrix C_check_str = strassenMultiply(A_check, B_check);

        cout << "\nKnown-value check (" << n_check << "x" << n_check << "):" << endl;
        bool manualPass = true;
        for (int i = 0; i < n_check; i++) {
            for (int j = 0; j < n_check; j++) {
                double dif = fabs(C_check_std[i][j] - C_check_str[i][j]);
                if (dif > 1e-12) {
                    cout << "  [" << i << "][" << j << "] std=" << C_check_std[i][j]
                         << " str=" << C_check_str[i][j] << " diff=" << dif << endl;
                    manualPass = false;
                }
            }
        }
        if (manualPass) cout << "  All values match exactly ✅" << endl;
    }

    cout << "\nAll tests completed." << endl;
    return 0;
}
