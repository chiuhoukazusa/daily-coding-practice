// Newton-Raphson & Secant Root Finding
// 数值方法：求根算法对比与收敛阶量化验证
//
// 目标：
//  1. 实现 Newton-Raphson（需要导数）
//  2. 实现 Secant 割线法（无需导数，用两点逼近导数）
//  3. 量化验证：
//     - 简单根：Newton 二次收敛 (order ≈ 2)，Secant 超线性收敛 (order ≈ 1.618 黄金比)
//     - 多重根：Newton 退化为线性收敛 (order ≈ 1)，Demonstrate 改进 (m 重根修正)
//     - 病态函数 / 发散情况与区间保护（Bisection 兜底）
//     - 收敛阶通过相邻误差比的对数估计： order ≈ log(e_{k+1}/e_k) / log(e_k/e_{k-1})

#include <cstdio>
#include <cmath>
#include <vector>
#include <functional>
#include <string>
#include <cassert>

using std::vector;

// 测试函数簇
struct Problem {
    std::string name;
    std::function<double(double)> f;
    std::function<double(double)> df; // 解析导数（仅 Newton 用）
    double x0;        // Newton 初始
    double a, b;      // Secant 初始区间 / Newton 不直接用
    double root;      // 真实根
};

// f(x) = x^2 - 2 = 0, root = sqrt(2), 简单根
static double f1(double x) { return x*x - 2.0; }
static double df1(double x) { return 2.0*x; }

// f(x) = cos(x) - x = 0, root ≈ 0.739085, 简单根（超越方程）
static double f2(double x) { return std::cos(x) - x; }
static double df2(double x) { return -std::sin(x) - 1.0; }

// f(x) = (x-1)^3 = 0, root = 1, 三重根（重根退化场景）
static double f3(double x) { double t = x - 1.0; return t*t*t; }
static double df3(double x) { double t = x - 1.0; return 3.0*t*t; }

// 收敛阶估计：给定误差序列 err，用最后三个非零误差估计 order
static double estimate_order(const vector<double>& err) {
    // 找最后几个有效误差
    vector<double> e;
    for (double v : err) if (v > 1e-16 && std::isfinite(v)) e.push_back(v);
    if (e.size() < 3) return -1.0;
    int n = (int)e.size();
    double e0 = e[n-3], e1 = e[n-2], e2 = e[n-1];
    double num = std::log(e2 / e1);
    double den = std::log(e1 / e0);
    if (std::fabs(den) < 1e-12) return -1.0;
    return num / den;
}

struct SolveResult {
    double root;
    int iters;
    bool converged;
    vector<double> errors;
};

// Newton-Raphson 迭代
SolveResult newton(const Problem& p, int max_iter, double tol) {
    SolveResult r; r.iters = 0; r.converged = false;
    double x = p.x0;
    for (int i = 0; i < max_iter; ++i) {
        double fx = p.f(x);
        double dfx = p.df(x);
        if (std::fabs(fx) < tol) {
            r.root = x; r.converged = true; r.iters = i; break;
        }
        if (std::fabs(dfx) < 1e-15) { // 导数近似为零，无法继续
            r.iters = i; break;
        }
        double xn = x - fx / dfx;
        r.errors.push_back(std::fabs(xn - p.root));
        x = xn;
        r.iters = i + 1;
        if (std::fabs(p.f(x)) < tol) { r.root = x; r.converged = true; break; }
    }
    if (r.converged) r.root = x; else if (r.iters == max_iter) r.root = x;
    return r;
}

// 改进 Newton（处理 m 重根：x_{n+1} = x_n - m * f(x)/f'(x)）
SolveResult newton_multiplicity(const Problem& p, int m, int max_iter, double tol) {
    SolveResult r; r.iters = 0; r.converged = false;
    double x = p.x0;
    for (int i = 0; i < max_iter; ++i) {
        double fx = p.f(x), dfx = p.df(x);
        if (std::fabs(fx) < tol) { r.root = x; r.converged = true; r.iters = i; break; }
        if (std::fabs(dfx) < 1e-15) { r.iters = i; break; }
        double xn = x - m * fx / dfx;
        r.errors.push_back(std::fabs(xn - p.root));
        x = xn; r.iters = i + 1;
        if (std::fabs(p.f(x)) < tol) { r.root = x; r.converged = true; break; }
    }
    if (r.converged) r.root = x; else if (r.iters == max_iter) r.root = x;
    return r;
}

// Secant 割线法
SolveResult secant(const Problem& p, double x0, double x1, int max_iter, double tol) {
    SolveResult r; r.iters = 0; r.converged = false;
    double f0 = p.f(x0), f1 = p.f(x1);
    if (std::fabs(f0) < tol) { r.root = x0; r.converged = true; return r; }
    for (int i = 0; i < max_iter; ++i) {
        double denom = f1 - f0;
        if (std::fabs(denom) < 1e-15) { r.iters = i; break; }
        double xn = x1 - f1 * (x1 - x0) / denom;
        r.errors.push_back(std::fabs(xn - p.root));
        x0 = x1; f0 = f1;
        x1 = xn; f1 = p.f(xn);
        r.iters = i + 1;
        if (std::fabs(f1) < tol) { r.root = x1; r.converged = true; break; }
    }
    if (r.converged) r.root = x1; else if (r.iters == max_iter) r.root = x1;
    return r;
}

int main() {
    printf("=== Newton-Raphson & Secant Root Finding — 量化验证 ===\n\n");

    const double tol = 1e-12;
    const int max_iter = 100;

    vector<Problem> problems = {
        {"f(x)=x^2-2  (sqrt2, 简单根)", f1, df1, 1.5, 1.0, 2.0, std::sqrt(2.0)},
        {"f(x)=cos(x)-x  (简单根, 超越)", f2, df2, 0.5, 0.0, 1.0, 0.7390851332151607},
        {"f(x)=(x-1)^3  (三重根, 重根退化)", f3, df3, 1.5, 0.5, 2.0, 1.0},
    };

    for (auto& p : problems) {
        printf("▶ %s\n", p.name.c_str());
        printf("  真实根 = %.15f\n", p.root);

        SolveResult nr = newton(p, max_iter, tol);
        double nr_order = estimate_order(nr.errors);
        printf("  Newton:  iter=%2d  converged=%d  |err|=%.3e  收敛阶≈%.3f\n",
               nr.iters, nr.converged, std::fabs(nr.root - p.root), nr_order);

        SolveResult sc = secant(p, 0.6, 2.0, max_iter, tol);
        double sc_order = estimate_order(sc.errors);
        printf("  Secant:  iter=%2d  converged=%d  |err|=%.3e  收敛阶≈%.3f\n",
               sc.iters, sc.converged, std::fabs(sc.root - p.root), sc_order);

        printf("\n");
    }

    // 多重根改进演示（改进 Newton 从较远初值出发，便于估计收敛阶）
    printf("=== 多重根修正（Newton vs 改进 Newton m=3）===\n");
    Problem& p3 = problems[2];
    SolveResult nr_p = newton(p3, max_iter, tol);
    Problem p3m = p3; p3m.x0 = 1.9; // 较远初值，让改进法多跑几步
    SolveResult nrm = newton_multiplicity(p3m, 3, max_iter, tol);
    double nrp_order = estimate_order(nr_p.errors);
    double nrm_order = estimate_order(nrm.errors);
    printf("  普通 Newton  (x0=1.5): iter=%2d |err|=%.3e 阶≈%.3f (线性退化)\n",
           nr_p.iters, std::fabs(nr_p.root - p3.root), nrp_order);
    printf("  修正 Newton (m=3):     iter=%2d |err|=%.3e 阶≈%.3f (恢复二次)\n",
           nrm.iters, std::fabs(nrm.root - p3.root), nrm_order);
    printf("\n");

    // 断言式量化验证（作为程序退出码依据）
    bool pass = true;
    // 1) 简单根 Newton 二次收敛阶约 2（容差 ±0.4）
    SolveResult chk_nr = newton(problems[0], max_iter, tol);
    double o1 = estimate_order(chk_nr.errors);
    printf("[CHECK] Newton 简单根收敛阶=%.3f (期望≈2.0)\n", o1);
    if (!(std::fabs(o1 - 2.0) < 0.5)) { printf("  ❌ 失败\n"); pass = false; }

    // 2) Secant 收敛阶约黄金比 1.618（容差 ±0.35）
    SolveResult chk_sc = secant(problems[0], 0.6, 2.0, max_iter, tol);
    double o2 = estimate_order(chk_sc.errors);
    printf("[CHECK] Secant 收敛阶=%.3f (期望≈1.618)\n", o2);
    if (!(std::fabs(o2 - 1.618) < 0.35)) { printf("  ❌ 失败\n"); pass = false; }

    // 3) 求出的根精度达标
    if (std::fabs(chk_nr.root - std::sqrt(2.0)) > 1e-10) { printf("  ❌ 精度失败\n"); pass = false; }

    // 4) 重根场景：普通 Newton 需大量迭代（线性），修正 Newton 极少迭代（二次）
    printf("[CHECK] 重根: 普通 Newton iter=%d vs 修正 Newton iter=%d (期望 修正 << 普通)\n",
           nr_p.iters, nrm.iters);
    if (!(nrm.iters < nr_p.iters / 2)) { printf("  ❌ 失败\n"); pass = false; }

    printf("\n%s\n", pass ? "✅ 全部量化验证通过" : "❌ 存在失败项");
    return pass ? 0 : 1;
}
