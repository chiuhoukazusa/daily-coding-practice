#include <bits/stdc++.h>
using namespace std;

// Cholesky Decomposition: A = L * L^T for symmetric positive definite (SPD) matrix A.
// Solves A x = b via forward/back substitution.

typedef vector<vector<double>> Mat;
typedef vector<double> Vec;

static double rand01() {
    return (double)rand() / RAND_MAX;
}

// Build a random SPD matrix: A = M * M^T + n*I (ensures positive definiteness).
Mat makeSPD(int n, int seed) {
    srand(seed);
    Mat M(n, Vec(n));
    for (int i=0;i<n;i++) for (int j=0;j<n;j++) M[i][j] = rand01()*2.0 - 1.0;
    Mat A(n, Vec(n, 0.0));
    // M * M^T  (note M[j][k], not M[k][j]) => guaranteed symmetric
    for (int i=0;i<n;i++) for (int j=0;j<n;j++)
        for (int k=0;k<n;k++) A[i][j] += M[i][k]*M[j][k];
    for (int i=0;i<n;i++) A[i][i] += n; // add n*I -> strictly SPD, well-conditioned
    return A;
}

// Cholesky factorization: returns L (lower triangular), A = L*L^T.
// Returns false if A is not SPD (negative pivot encountered).
bool cholesky(const Mat& A, Mat& L) {
    int n = A.size();
    L.assign(n, Vec(n, 0.0));
    for (int i=0;i<n;i++) {
        for (int j=0;j<=i;j++) {
            double sum = A[i][j];
            for (int k=0;k<j;k++) sum -= L[i][k]*L[j][k];
            if (i == j) {
                if (sum <= 1e-12) return false; // not positive definite
                L[i][j] = sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }
    return true;
}

Mat transpose(const Mat& M) {
    int n=M.size(), m=M[0].size();
    Mat T(m, Vec(n));
    for(int i=0;i<n;i++) for(int j=0;j<m;j++) T[j][i]=M[i][j];
    return T;
}

Mat matmul(const Mat& A, const Mat& B) {
    int n=A.size(), m=B[0].size(), k=B.size();
    Mat C(n, Vec(m, 0.0));
    for(int i=0;i<n;i++) for(int j=0;j<m;j++)
        for(int t=0;t<k;t++) C[i][j]+=A[i][t]*B[t][j];
    return C;
}

Vec matvec(const Mat& A, const Vec& x) {
    int n=A.size();
    Vec y(n,0.0);
    for(int i=0;i<n;i++) for(int j=0;j<n;j++) y[i]+=A[i][j]*x[j];
    return y;
}

// Solve A x = b given L (A = L L^T).
// Forward: L y = b ; Backward: L^T x = y.
Vec solve(const Mat& L, const Vec& b) {
    int n = L.size();
    Vec y(n,0.0), x(n,0.0);
    for (int i=0;i<n;i++) {
        double s = b[i];
        for (int j=0;j<i;j++) s -= L[i][j]*y[j];
        y[i] = s / L[i][i];
    }
    for (int i=n-1;i>=0;i--) {
        double s = y[i];
        for (int j=i+1;j<n;j++) s -= L[j][i]*x[j];
        x[i] = s / L[i][i];
    }
    return x;
}

double frobeniusNorm(const Mat& A) {
    double s=0.0;
    for (auto& r:A) for (double v:r) s+=v*v;
    return sqrt(s);
}

double vecNorm(const Vec& v){ double s=0; for(double x:v) s+=x*x; return sqrt(s); }
Vec vecSub(const Vec& a,const Vec& b){ Vec c(a.size()); for(size_t i=0;i<a.size();i++)c[i]=a[i]-b[i]; return c; }

int main() {
    cout << fixed << setprecision(10);

    // ---- Test 1: reconstruction accuracy A = L * L^T across sizes ----
    cout << "=== Test 1: Factorization reconstruction (A - L*L^T) ===\n";
    vector<int> sizes = {4, 16, 64, 128};
    double maxRecErr = 0.0;
    for (int n : sizes) {
        Mat A = makeSPD(n, n*7+3);
        Mat L;
        bool ok = cholesky(A, L);
        if (!ok) { cout << "  FAIL: factorization failed for n="<<n<<"\n"; return 1; }
        Mat R = matmul(L, transpose(L));
        // error matrix
        Mat E(n, Vec(n));
        for (int i=0;i<n;i++) for (int j=0;j<n;j++) E[i][j]=A[i][j]-R[i][j];
        double err = frobeniusNorm(E);
        maxRecErr = max(maxRecErr, err);
        cout << "  n=" << setw(3) << n << "  ||A - L*L^T||_F = " << err << "\n";
    }
    cout << "  MAX reconstruction error = " << maxRecErr << "\n";

    // ---- Test 2: solve A x = b, verify residual ||A x - b|| ----
    cout << "\n=== Test 2: Linear solve residual ||A x - b|| ===\n";
    double maxRes = 0.0;
    for (int n : sizes) {
        Mat A = makeSPD(n, n*11+5);
        Vec b(n); srand(n*13+1);
        for (int i=0;i<n;i++) b[i] = rand01()*10.0 - 5.0;
        Mat L; cholesky(A, L);
        Vec x = solve(L, b);
        Vec Ax = matvec(A, x);
        double res = vecNorm(vecSub(Ax, b));
        maxRes = max(maxRes, res);
        cout << "  n=" << setw(3) << n << "  residual = " << res << "\n";
    }
    cout << "  MAX residual = " << maxRes << "\n";

    // ---- Test 3: non-SPD detection (should return false) ----
    cout << "\n=== Test 3: Non-SPD detection ===\n";
    {
        // indefinite matrix [[1,2],[2,1]] (eigenvalues -1, 3)
        Mat A = {{1.0,2.0},{2.0,1.0}};
        Mat L;
        bool ok = cholesky(A, L);
        cout << "  indefinite matrix -> decomposition returned: " << (ok?"true (BAD)":"false (correct)") << "\n";
        if (ok) { cout << "  FAIL: should not decompose indefinite matrix\n"; return 1; }
    }
    {
        // negative definite [[-4,0],[0,-4]]
        Mat A = {{-4.0,0.0},{0.0,-4.0}};
        Mat L;
        bool ok = cholesky(A, L);
        cout << "  negative definite matrix -> returned: " << (ok?"true (BAD)":"false (correct)") << "\n";
        if (ok) { cout << "  FAIL: should not decompose negative definite\n"; return 1; }
    }

    // ---- Test 4: known exact case (A=[[4,2],[2,3]], L=[[2,0],[1,sqrt2]]) ----
    cout << "\n=== Test 4: Known exact decomposition ===\n";
    {
        Mat A = {{4.0,2.0},{2.0,3.0}};
        Mat L; cholesky(A, L);
        double l00=2.0, l10=1.0, l11=sqrt(2.0);
        cout << "  L[0][0]="<<L[0][0]<<" (expect 2.0)\n";
        cout << "  L[1][0]="<<L[1][0]<<" (expect 1.0)\n";
        cout << "  L[1][1]="<<L[1][1]<<" (expect "<<sqrt(2.0)<<")\n";
        bool good = fabs(L[0][0]-l00)<1e-9 && fabs(L[1][0]-l10)<1e-9 && fabs(L[1][1]-l11)<1e-9;
        if (!good) { cout << "  FAIL: exact case mismatch\n"; return 1; }
        cout << "  exact case OK\n";
    }

    // ---- Summary (machine-readable for downstream verification) ----
    cout << "\n=== SUMMARY ===\n";
    cout << "max_reconstruction_error=" << maxRecErr << "\n";
    cout << "max_solve_residual=" << maxRes << "\n";
    bool pass = (maxRecErr < 1e-6) && (maxRes < 1e-8);
    cout << "PASS=" << (pass ? "true" : "false") << "\n";
    return pass ? 0 : 1;
}
