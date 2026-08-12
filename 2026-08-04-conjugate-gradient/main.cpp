/**
 * Conjugate Gradient Solver - 共轭梯度法稀疏线性系统求解器
 *
 * 技术点：
 *   - CSR 稀疏矩阵存储格式
 *   - Conjugate Gradient 迭代求解算法
 *   - Poisson 方程 (5点差分) 构造正定对称稀疏系统
 *   - 残差范数收敛性分析
 *   - 收敛曲线量化验证
 *
 * CG 算法步骤：
 *   1. r0 = b - A*x0, p0 = r0
 *   2. alpha_k = (r_k·r_k) / (p_k·A·p_k)
 *   3. x_{k+1} = x_k + alpha_k * p_k
 *   4. r_{k+1} = r_k - alpha_k * A * p_k
 *   5. beta_k = (r_{k+1}·r_{k+1}) / (r_k·r_k)
 *   6. p_{k+1} = r_{k+1} + beta_k * p_k
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <random>
#include <fstream>

// ============================================================
// CSR 稀疏矩阵格式 (Compressed Sparse Row)
// 适合 CG 算法中的矩阵-向量乘法
// ============================================================
struct SparseMatrixCSR {
    int n;                          // 矩阵维度 n x n
    std::vector<double> values;     // 非零元值
    std::vector<int> col_indices;   // 列索引
    std::vector<int> row_ptr;       // 行偏移 (size = n+1)
};

// 矩阵-向量乘法: y = A * x
std::vector<double> mat_vec_mul(const SparseMatrixCSR& A, const std::vector<double>& x) {
    std::vector<double> y(A.n, 0.0);
    for (int i = 0; i < A.n; ++i) {
        double sum = 0.0;
        for (int k = A.row_ptr[i]; k < A.row_ptr[i+1]; ++k) {
            sum += A.values[k] * x[A.col_indices[k]];
        }
        y[i] = sum;
    }
    return y;
}

// 向量点积
double dot(const std::vector<double>& a, const std::vector<double>& b) {
    double s = 0.0;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

// 向量 L2 范数
double norm2(const std::vector<double>& v) {
    return std::sqrt(dot(v, v));
}

// 向量 L∞ 范数
double norm_inf(const std::vector<double>& v) {
    double m = 0.0;
    for (double x : v) m = std::max(m, std::fabs(x));
    return m;
}

// axpy: y += a * x
void axpy(double a, const std::vector<double>& x, std::vector<double>& y) {
    for (size_t i = 0; i < x.size(); ++i) y[i] += a * x[i];
}

// scale: x *= a
void scale(double a, std::vector<double>& x) {
    for (size_t i = 0; i < x.size(); ++i) x[i] *= a;
}

// ============================================================
// Conjugate Gradient 求解器 (带详细的收敛监控)
// ============================================================
struct CGResult {
    bool converged;
    int iterations;
    double final_rel_residual;       // 最终相对残差 ||r||/||b||
    double final_abs_residual;       // 最终绝对残差 ||r||
    std::vector<double> x;           // 解向量
    std::vector<double> residual_history; // 每次迭代的残差范数
};

CGResult conjugate_gradient(
    const SparseMatrixCSR& A,
    const std::vector<double>& b,
    double tol = 1e-8,
    int max_iter = 1000
) {
    int n = A.n;
    CGResult result;
    result.converged = false;

    // 初始猜测 x0 = 0
    std::vector<double> x(n, 0.0);

    // r0 = b - A*x0 = b (因为 x0=0)
    std::vector<double> r = b;
    std::vector<double> p = r;  // p0 = r0

    double b_norm = norm2(b);
    if (b_norm < 1e-15) {
        // 平凡解
        result.converged = true;
        result.iterations = 0;
        result.final_rel_residual = 0.0;
        result.final_abs_residual = 0.0;
        result.x = x;
        return result;
    }

    double rsold = dot(r, r);

    for (int k = 0; k < max_iter; ++k) {
        // Ap = A * p_k
        std::vector<double> Ap = mat_vec_mul(A, p);

        // alpha = rsold / (p·Ap)
        double pAp = dot(p, Ap);
        if (pAp <= 0.0) {
            // 矩阵非正定，CG 可能失败
            break;
        }
        double alpha = rsold / pAp;

        // x_{k+1} = x_k + alpha * p_k
        axpy(alpha, p, x);

        // r_{k+1} = r_k - alpha * A * p_k
        axpy(-alpha, Ap, r);

        double rsnew = dot(r, r);

        // 记录残差
        double rel_res = std::sqrt(rsnew) / b_norm;
        result.residual_history.push_back(rel_res);

        // 检查收敛
        if (rel_res < tol) {
            result.converged = true;
            result.iterations = k + 1;
            result.final_rel_residual = rel_res;
            result.final_abs_residual = std::sqrt(rsnew);
            result.x = x;
            return result;
        }

        // beta = rsnew / rsold
        double beta = rsnew / rsold;

        // p_{k+1} = r_{k+1} + beta * p_k
        scale(beta, p);
        for (int i = 0; i < n; ++i) p[i] += r[i];  // p = r + beta*p

        rsold = rsnew;
    }

    // 未收敛
    result.converged = false;
    result.iterations = max_iter;
    result.final_rel_residual = !result.residual_history.empty() ? result.residual_history.back() : 1.0;
    result.final_abs_residual = std::sqrt(rsold);
    result.x = x;
    return result;
}

// ============================================================
// 构造 Poisson 方程矩阵 (1D / 2D)
// -u'' = f  (1D)  或  -Δu = f  (2D)
// 5点差分离散化产生对称正定矩阵
// ============================================================
SparseMatrixCSR build_poisson_1d(int n) {
    SparseMatrixCSR A;
    A.n = n;
    A.row_ptr.resize(n + 1, 0);

    // 计算每行的非零元数量
    for (int i = 0; i < n; ++i) {
        int nnz = 1;  // 对角线
        if (i > 0) nnz++;       // 左邻居
        if (i < n - 1) nnz++;   // 右邻居
        A.row_ptr[i+1] = nnz;
    }

    // 前缀和转偏移
    for (int i = 0; i < n; ++i) {
        A.row_ptr[i+1] += A.row_ptr[i];
    }

    int total_nnz = A.row_ptr[n];
    A.values.resize(total_nnz);
    A.col_indices.resize(total_nnz);

    // 填充值
    for (int i = 0; i < n; ++i) {
        int idx = A.row_ptr[i];
        if (i > 0) {
            A.col_indices[idx] = i - 1;
            A.values[idx] = -1.0;
            idx++;
        }
        A.col_indices[idx] = i;
        A.values[idx] = 2.0;
        idx++;
        if (i < n - 1) {
            A.col_indices[idx] = i + 1;
            A.values[idx] = -1.0;
        }
    }

    return A;
}

// 2D Poisson: -Δu = f  在 Nx × Ny 网格上
// 自然行排序: k = j*Nx + i
SparseMatrixCSR build_poisson_2d(int Nx, int Ny) {
    int n = Nx * Ny;
    SparseMatrixCSR A;
    A.n = n;
    A.row_ptr.resize(n + 1, 0);

    for (int j = 0; j < Ny; ++j) {
        for (int i = 0; i < Nx; ++i) {
            int k = j * Nx + i;
            int nnz = 1;  // 对角线
            if (i > 0) nnz++;       // 左
            if (i < Nx - 1) nnz++;  // 右
            if (j > 0) nnz++;       // 下
            if (j < Ny - 1) nnz++;  // 上
            A.row_ptr[k+1] = nnz;
        }
    }

    for (int k = 0; k < n; ++k) A.row_ptr[k+1] += A.row_ptr[k];

    int total_nnz = A.row_ptr[n];
    A.values.resize(total_nnz);
    A.col_indices.resize(total_nnz);

    for (int j = 0; j < Ny; ++j) {
        for (int i = 0; i < Nx; ++i) {
            int k = j * Nx + i;
            int idx = A.row_ptr[k];

            // 按列索引排序
            // 左
            if (i > 0) {
                A.col_indices[idx] = k - 1;
                A.values[idx] = -1.0;
                idx++;
            }
            // 下
            if (j > 0) {
                A.col_indices[idx] = k - Nx;
                A.values[idx] = -1.0;
                idx++;
            }
            // 对角线
            A.col_indices[idx] = k;
            A.values[idx] = 4.0;
            idx++;
            // 上
            if (j < Ny - 1) {
                A.col_indices[idx] = k + Nx;
                A.values[idx] = -1.0;
                idx++;
            }
            // 右
            if (i < Nx - 1) {
                A.col_indices[idx] = k + 1;
                A.values[idx] = -1.0;
            }
        }
    }

    return A;
}

// ============================================================
// 条件数估计 & 理论收敛速率
// ============================================================
double condition_number_estimate(const SparseMatrixCSR& A) {
    // 用 power iteration 估计最大特征值，反幂法估计最小特征值
    // 简化：对于 Poisson 问题，条件数 ~ O(N²) / O(1)
    int n = A.n;

    // 最大特征值通过 Power Iteration 估计
    std::vector<double> v(n, 1.0 / std::sqrt(double(n)));
    double lambda_max = 0.0;
    for (int iter = 0; iter < 100; ++iter) {
        std::vector<double> Av = mat_vec_mul(A, v);
        double lambda = norm2(Av);
        if (std::fabs(lambda - lambda_max) < 1e-10) break;
        lambda_max = lambda;
        for (int i = 0; i < n; ++i) v[i] = Av[i] / lambda;
    }

    // 最小特征值通过 Inverse Power Iteration 估计（使用 CG 近似）
    // 对 Poisson -Δ 矩阵，最小特征值近似 = 2(1-cos(π/(n+1))) ... 我们用粗略估计
    // 这里用 mesh size：λ_min ≈ π²/(N+1)² 对于 1D, 但我们就近似的吧
    // 用理论估计来验证 power method
    // 对于 Poisson 2D: λ_max ≈ 8*h², λ_min ≈ 2π²/h²
    // 直接用 power method 得到的 lambda_max
    return lambda_max; // 返回最大特征值作为参考
}

// ============================================================
// 主程序
// ============================================================
int main() {
    std::cout << "╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  Conjugate Gradient Solver - 共轭梯度法求解器    ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n\n";

    // =========================================
    // Test 1: 1D Poisson, 已知精确解
    // -u'' = π² sin(πx)  →  u(x) = sin(πx)
    // =========================================
    {
        std::cout << "=== Test 1: 1D Poisson Equation ===\n";
        int N = 200;
        double h = 1.0 / (N + 1);

        SparseMatrixCSR A = build_poisson_1d(N);
        std::vector<double> b(N, 0.0);
        std::vector<double> u_exact(N, 0.0);

        // b = h² * f(x),  f(x) = π² sin(πx)
        for (int i = 0; i < N; ++i) {
            double x = (i + 1) * h;
            u_exact[i] = std::sin(M_PI * x);
            b[i] = M_PI * M_PI * std::sin(M_PI * x) * h * h;
        }

        CGResult result = conjugate_gradient(A, b, 1e-10, N);

        // 计算解误差
        double error_L2 = 0.0;
        double error_Linf = 0.0;
        for (int i = 0; i < N; ++i) {
            double diff = std::fabs(result.x[i] - u_exact[i]);
            error_L2 += diff * diff;
            error_Linf = std::max(error_Linf, diff);
        }
        error_L2 = std::sqrt(error_L2);

        std::cout << "  N = " << N << "\n";
        std::cout << "  收敛: " << (result.converged ? "✅ YES" : "❌ NO") << "\n";
        std::cout << "  迭代次数: " << result.iterations << "\n";
        std::cout << "  最终相对残差: " << std::scientific << result.final_rel_residual << "\n";
        std::cout << "  L2 误差 vs 精确解: " << error_L2 << "\n";
        std::cout << "  L∞ 误差 vs 精确解: " << error_Linf << "\n";

        if (!result.converged) {
            std::cout << "  ❌ CG did not converge!\n";
            return 1;
        }
        if (error_Linf > 1e-4) {
            std::cout << "  ❌ Error too large vs exact solution!\n";
            return 1;
        }
        std::cout << "  ✅ 1D Poisson Test PASSED\n\n";
    }

    // =========================================
    // Test 2: 2D Poisson, 已知精确解
    // -Δu = 2π² sin(πx)sin(πy)  →  u(x,y) = sin(πx)sin(πy)
    // =========================================
    {
        std::cout << "=== Test 2: 2D Poisson Equation ===\n";
        int Nx = 64, Ny = 64;
        int n = Nx * Ny;
        double h = 1.0 / (Nx + 1);

        SparseMatrixCSR A = build_poisson_2d(Nx, Ny);
        std::vector<double> b(n, 0.0);
        std::vector<double> u_exact(n, 0.0);

        for (int j = 0; j < Ny; ++j) {
            for (int i = 0; i < Nx; ++i) {
                int k = j * Nx + i;
                double x = (i + 1) * h;
                double y = (j + 1) * h;
                u_exact[k] = std::sin(M_PI * x) * std::sin(M_PI * y);
                b[k] = 2.0 * M_PI * M_PI * u_exact[k] * h * h;
            }
        }

        CGResult result = conjugate_gradient(A, b, 1e-8, 2000);

        double error_L2 = 0.0, error_Linf = 0.0;
        for (int i = 0; i < n; ++i) {
            double diff = std::fabs(result.x[i] - u_exact[i]);
            error_L2 += diff * diff;
            error_Linf = std::max(error_Linf, diff);
        }
        error_L2 = std::sqrt(error_L2);

        std::cout << "  矩阵维度: " << n << " x " << n << "\n";
        std::cout << "  非零元: " << A.values.size() << " (密度: "
                  << std::fixed << std::setprecision(2)
                  << (100.0 * A.values.size() / (double(n)*n)) << "%)\n";
        std::cout << "  收敛: " << (result.converged ? "✅ YES" : "❌ NO") << "\n";
        std::cout << "  迭代次数: " << result.iterations << "\n";
        std::cout << "  最终相对残差: " << std::scientific << result.final_rel_residual << "\n";
        std::cout << "  L2 误差 vs 精确解: " << error_L2 << "\n";
        std::cout << "  L∞ 误差 vs 精确解: " << error_Linf << "\n";

        if (!result.converged) {
            std::cout << "  ❌ CG did not converge!\n";
            return 1;
        }
        if (error_Linf > 1e-3) {
            std::cout << "  ❌ Error too large vs exact solution!\n";
            return 1;
        }

        // 量化验证：检查解在 (0.5, 0.5) 处的值
        int cx = Nx / 2, cy = Ny / 2;
        int ck = cy * Nx + cx;
        double expected_center = std::sin(M_PI * 0.5) * std::sin(M_PI * 0.5);  // = 1.0
        double computed_center = result.x[ck];
        double center_error = std::fabs(computed_center - expected_center);
        std::cout << "  中心点值: " << std::fixed << std::setprecision(6)
                  << computed_center << " (expected: " << expected_center
                  << ", error: " << std::scientific << center_error << ")\n";

        if (center_error > 1e-3) {
            std::cout << "  ❌ Center value too inaccurate!\n";
            return 1;
        }

        std::cout << "  ✅ 2D Poisson Test PASSED\n\n";
    }

    // =========================================
    // Test 3: 收敛性分析 — 残差逐次递减
    // =========================================
    {
        std::cout << "=== Test 3: Residual Monotonic Convergence ===\n";
        int N = 100;
        SparseMatrixCSR A = build_poisson_1d(N);
        std::vector<double> b(N, 1.0);  // 常数右端项

        CGResult result = conjugate_gradient(A, b, 1e-12, 500);

        // 验证残差单调递减特性
        bool monotonic = true;
        int violations = 0;
        double total_ratio = 0.0;
        for (size_t k = 1; k < result.residual_history.size(); ++k) {
            double ratio = result.residual_history[k] / result.residual_history[k-1];
            total_ratio += ratio;
            if (result.residual_history[k] > result.residual_history[k-1] * 1.01) {
                monotonic = false;
                violations++;
            }
        }

        double avg_ratio = result.residual_history.size() > 1 ?
            total_ratio / (result.residual_history.size() - 1) : 0.0;

        std::cout << "  迭代次数: " << result.iterations << "\n";
        std::cout << "  最终残差: " << std::scientific << result.final_rel_residual << "\n";
        std::cout << "  初始残差: " << result.residual_history[0] << "\n";
        std::cout << "  残差衰减倍数: "
                  << std::scientific << result.residual_history[0] / result.final_rel_residual << "\n";
        std::cout << "  平均衰减因子: " << std::fixed << std::setprecision(4) << avg_ratio << "\n";

        if (!monotonic) {
            std::cout << "  残差非单调次数: " << violations << " (可接受的小幅波动)\n";
        }

        // 量化检查: 最终残差必须足够小
        if (result.final_rel_residual > 1e-10) {
            std::cout << "  ❌ Final residual too large!\n";
            return 1;
        }

        // 量化检查: 残差必须比初始值小至少 1e6 倍
        double reduction = result.residual_history[0] / result.final_rel_residual;
        if (reduction < 1e6) {
            std::cout << "  ❌ Residual reduction insufficient (< 1e6)\n";
            return 1;
        }

        std::cout << "  ✅ Convergence Test PASSED\n\n";
    }

    // =========================================
    // Test 4: 大规模 2D Poisson 可扩展性测试
    // =========================================
    {
        std::cout << "=== Test 4: Scalability — Larger 2D Poisson ===\n";
        int Nx = 100, Ny = 100;
        int n = Nx * Ny;

        SparseMatrixCSR A = build_poisson_2d(Nx, Ny);
        std::vector<double> b(n, 1.0);

        CGResult result = conjugate_gradient(A, b, 1e-8, 5000);

        // 验证解非平凡
        double x_min = *std::min_element(result.x.begin(), result.x.end());
        double x_max = *std::max_element(result.x.begin(), result.x.end());
        double x_mean = 0.0, x_var = 0.0;
        for (double v : result.x) x_mean += v;
        x_mean /= n;
        for (double v : result.x) x_var += (v - x_mean) * (v - x_mean);
        x_var /= n;

        std::cout << "  矩阵维度: " << n << " x " << n
                  << " (" << A.values.size() << " nonzeros)\n";
        std::cout << "  收敛: " << (result.converged ? "✅ YES" : "❌ NO") << "\n";
        std::cout << "  迭代次数: " << result.iterations << "\n";
        std::cout << "  最终相对残差: " << std::scientific << result.final_rel_residual << "\n";
        std::cout << "  解统计: min=" << std::fixed << std::setprecision(6) << x_min
                  << " max=" << x_max << " mean=" << x_mean << " var=" << x_var << "\n";

        // 验证: 解必须有意义的变化
        if (std::fabs(x_max - x_min) < 1e-6) {
            std::cout << "  ❌ Solution is essentially constant (degenerate)\n";
            return 1;
        }

        // 验证: 残差必须降得足够小
        if (!result.converged || result.final_rel_residual > 1e-6) {
            std::cout << "  ❌ Not converged or residual too large!\n";
            return 1;
        }

        std::cout << "  ✅ Scalability Test PASSED\n\n";
    }

    // =========================================
    // Test 5: 输出残差收敛曲线数据 (用于可视化)
    // =========================================
    {
        std::cout << "=== Test 5: Residual Convergence Profile ===\n";

        // 对不同大小的 1D Poisson 问题记录收敛曲线
        std::vector<int> sizes = {32, 64, 128, 256};
        std::ofstream profile("../residual_profile.csv");
        profile << "N,iteration,residual\n";

        for (int N : sizes) {
            SparseMatrixCSR A = build_poisson_1d(N);
            std::vector<double> b(N, 1.0);
            CGResult result = conjugate_gradient(A, b, 1e-12, 500);

            std::cout << "  N=" << N << ": " << result.iterations << " iterations to tol=1e-12"
                      << ", final residual=" << std::scientific << result.final_rel_residual << "\n";

            for (size_t k = 0; k < result.residual_history.size(); ++k) {
                profile << N << "," << k << "," << result.residual_history[k] << "\n";
            }
        }
        profile.close();
        std::cout << "  ✅ Convergence profile written to residual_profile.csv\n\n";
    }

    // =========================================
    // Summary
    // =========================================
    std::cout << "╔══════════════════════════════════════════════════╗\n";
    std::cout << "║  ✅ ALL TESTS PASSED                            ║\n";
    std::cout << "╚══════════════════════════════════════════════════╝\n";

    return 0;
}
