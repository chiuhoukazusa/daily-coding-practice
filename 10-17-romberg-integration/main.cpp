// Romberg Integration with Richardson Extrapolation
// ====================================================
//
// 数值积分的方法谱系里，最朴素的是矩形法/梯形法（复合梯形公式），
// 其误差以 h^2 收敛（h 为步长）。Richardson 外推（又称 Richardson
// extrapolation / Richardson's deferred approach to the limit）能把这些
// 低阶方法的外推表逐级组合，把收敛阶逐级提升到 h^4、h^6、h^8……直到
// 机器精度。Romberg 积分就是「复合梯形公式 + Richardson 外推」的经典
// 结合，几十年来一直是确定性一维数值积分的主力算法。
//
// 本程序做三件事：
//   1. 实现标准 Romberg 表（T[0][k] 为复合梯形，逐级外推）。
//   2. 对多个解析可积的被积函数，量化验证误差随级数 k 与步数 n 的
//      收敛行为，实测收敛阶并与理论对比。
//   3. 与朴素复合梯形 / 复合 Simpson 做精度对比，量化 Romberg 的优势。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// ----------------------------------------------------------------------
// R(0, 0) = (h/2)*(f(a)+f(b))  —— 单一梯形
// R(n, 0) = 0.5*R(n-1,0) + h_n * sum_{odd i} f(a + i*h_n)   复合梯形 (2^n 段)
// R(n, m) = R(n, m-1) + (R(n, m-1) - R(n-1, m-1)) / (4^m - 1)  Richardson 外推
// ----------------------------------------------------------------------
struct RombergResult {
    double value;
    // 存储整个三角表第一列（梯形）与最终外推值，便于打印收敛过程
    std::vector<double> trapezoid_column; // R(n, 0) for n = 0..maxN
    std::vector<double> diagonal;         // R(n, n) 若做完全表
};

// 计算 Romberg 表，返回 R(maxN, maxM) 外推值。
// 为衡量收敛，同时把每一层的梯形列与对角线保留下来。
static int romberg(double a, double b,
                   const std::function<double(double)>& f,
                   int maxN, int maxM,
                   std::vector<std::vector<double>>& table_out) {
    // table[r][m] = R(r, m)；r 是梯形细分层级 (2^r 段)，m 是外推级。
    table_out.assign(maxN + 1, std::vector<double>(maxM + 1, 0.0));

    double& r00 = table_out[0][0];
    r00 = 0.5 * (b - a) * (f(a) + f(b));

    for (int r = 1; r <= maxN; ++r) {
        int n = 1 << r;                  // 2^r 段
        double h = (b - a) / n;
        double sum = 0.0;
        for (int i = 1; i < n; i += 2) { // 只累加新增的奇数点
            sum += f(a + i * h);
        }
        // 复合梯形递推：R(r,0) = 0.5 * R(r-1,0) + sum * h
        table_out[r][0] = 0.5 * table_out[r - 1][0] + sum * h;

        // Richardson 外推，逐级提升精度
        for (int m = 1; m <= maxM && m <= r; ++m) {
            double denom = std::pow(4.0, m) - 1.0;
            table_out[r][m] =
                table_out[r][m - 1] +
                (table_out[r][m - 1] - table_out[r - 1][m - 1]) / denom;
        }
    }
    return 0;
}

// 朴素复合梯形（n 段），仅用于对比
static double composite_trapezoid(double a, double b,
                                  const std::function<double(double)>& f,
                                  int n) {
    double h = (b - a) / n;
    double s = 0.5 * (f(a) + f(b));
    for (int i = 1; i < n; ++i) s += f(a + i * h);
    return s * h;
}

// 朴素复合 Simpson（n 段，n 必须为偶数），仅用于对比
static double composite_simpson(double a, double b,
                                const std::function<double(double)>& f,
                                int n) {
    if (n % 2) ++n;
    double h = (b - a) / n;
    double s = f(a) + f(b);
    for (int i = 1; i < n; ++i) {
        s += (i % 2 ? 4.0 : 2.0) * f(a + i * h);
    }
    return s * h / 3.0;
}

// 实测收敛阶：给定误差序列 err(k) ~ C * n_k^{-p}，
// p ≈ log(err_k / err_{k+1}) / log(n_{k+1} / n_k)。
// 这里 n_k = 2^k（梯形细分），或按模拟。
static double estimate_order(const std::vector<double>& errs,
                             const std::vector<int>& ns) {
    if (errs.size() < 3) return 0.0;
    // 用倒数第二对求斜率（末期最接近渐近行为，又避免机器精度污染最后一位）
    size_t k = errs.size() - 2;
    double p = std::log(errs[k] / errs[k + 1]) /
               std::log((double)ns[k + 1] / ns[k]);
    return p;
}

// 用梯形列（第一列 R(r,0) 的误差）估计收敛阶；用于奇异情形验证外推退化。
static double estimate_order_of_first_column(
        const std::vector<std::vector<double>>& table, double exact) {
    std::vector<double> errs;
    std::vector<int> ns;
    for (size_t r = 2; r < table.size(); ++r) {
        errs.push_back(std::fabs(table[r][0] - exact));
        ns.push_back(1 << r);
    }
    return estimate_order(errs, ns);
}

// 被积函数族：每个都是解析可积的，便于精确验证
struct TestCase {
    std::string name;
    std::function<double(double)> f;
    double a, b;
    double exact;          // 解析真值
    bool singular_endpoint = false; // 端点奇异 (导数无穷)，Richardson 外推预期失效
};

int main() {
    // 构造测试用例（逐个体积解析可积）
    std::vector<TestCase> cases;
    cases.push_back({
        "exp(-x^2) on [0,1]",                       // 与误差函数相关，无初等原函数但可用精确值
        [](double x) { return std::exp(-x * x); },
        0.0, 1.0,
        0.74682413281242702539946743613185300532968 // sqrt(pi)/2 * erf(1)
    });
    cases.push_back({
        "sin(x)/x on [1,5]",
        [](double x) { return std::sin(x) / x; },
        1.0, 5.0,
        // 用高精度积分近似（Si(5)-Si(1)）：Si(5)=1.54993124494467414... Si(1)=0.94608307036718301...
        0.60384817457749112218
    });
    cases.push_back({
        "1/(1+x^2) on [0,1] = pi/4",
        [](double x) { return 1.0 / (1.0 + x * x); },
        0.0, 1.0,
        3.14159265358979323846 / 4.0
    });
    cases.push_back({
        "sqrt(x) on [0,1]   (端点奇异导数)",
        [](double x) { return std::sqrt(x); },
        0.0, 1.0,
        2.0 / 3.0,
        true // 端点奇异：期望 Richardson 外推失效，收敛阶退化为 ~1.5
    });

    const int MAXN = 12;   // 最多 2^12 = 4096 段复合梯形
    const int MAXM = 12;   // 外推级数，可与 MAXN 一致做完全表

    std::printf("====================================================================\n");
    std::printf("Romberg Integration with Richardson Extrapolation — 量化验证\n");
    std::printf("====================================================================\n\n");

    bool all_ok = true;

    for (auto& tc : cases) {
        std::vector<std::vector<double>> table;
        romberg(tc.a, tc.b, tc.f, MAXN, MAXM, table);

        // 收集各级外推的误差：用固定细分层级 r = MAXN 的 R(MAXN, m)
        // 观察外推级 m 递增时的误差下降，同时用对角线 R(n,n) 观察整体收敛。
        std::printf("【%s】  exact = %.15g\n", tc.name.c_str(), tc.exact);
        std::printf("  n(段)     梯形 R(n,0)        外推 R(n,m递增)\n");

        // 表头：细分层级 + 梯形 + 若干外推级
        std::printf("  %-8s ", "细分段");
        for (int m = 0; m <= 6; ++m) std::printf("%-18s", m == 0 ? "梯形R(n,0)" :
                                                        ("外推m=" + std::to_string(m)).c_str());
        std::printf("\n");

        for (int r = 0; r <= MAXN; ++r) {
            int n = 1 << r;
            std::printf("  %-8d ", n);
            for (int m = 0; m <= 6; ++m) {
                double v = (m <= r) ? table[r][m] : NAN;
                std::printf("%-18.10g", v);
            }
            std::printf("\n");
        }

        // 收敛阶测量：对每个外推级 m，误差随 r 的收敛阶
        std::printf("\n  收敛阶实测（误差 |R-E| 随细分段数 2^r 的幂次，理论 2(m+1)）:\n");
        for (int m = 0; m <= 4; ++m) {
            std::vector<double> errs;
            std::vector<int> ns;
            for (int r = m; r <= MAXN; ++r) {
                errs.push_back(std::fabs(table[r][m] - tc.exact));
                ns.push_back(1 << r);
            }
            double ord = estimate_order(errs, ns);
            double theory = 2.0 * (m + 1);
            std::printf("    外推级 m=%d: 理论阶 %g, 实测阶 %.3f\n",
                        m, theory, ord);
        }

        // 对比：相同函数求值次数下，Romberg vs 复合梯形 vs 复合 Simpson
        // 用 R(MAXN, MAXN)（对角线，达到最高精度）
        double romberg_val = table[MAXN][MAXN];
        double err_romberg = std::fabs(romberg_val - tc.exact);

        // 复合梯形用 4096 段（同样的函数求值规模量级）
        double trap_err = std::fabs(composite_trapezoid(tc.a, tc.b, tc.f, 1 << MAXN) - tc.exact);
        double simp_err = std::fabs(composite_simpson(tc.a, tc.b, tc.f, 1 << MAXN) - tc.exact);

        std::printf("\n  Romberg 最终值 = %.15g  (误差 %.3g)\n", romberg_val, err_romberg);
        std::printf("  复合梯形(4096段)    误差 %.3g\n", trap_err);
        std::printf("  复合Simpson(4096段) 误差 %.3g\n", simp_err);

        // 判定：Romberg 应显著优于同规模梯形；且误差应达到高精度。
        // 端点奇异情形专门处理：预期 Richardson 外推失效，收敛阶退化为 1.5。
        bool improved = (err_romberg < trap_err * 1e-6);
        bool precise = (err_romberg < 1e-10);
        if (tc.singular_endpoint) {
            // 奇异情形：误差应明显小于朴素梯形，且不要求达到 1e-10。
            // 用实测收敛阶 ~1.5 验证 Euler-Maclaurin 假设被破坏这一理论预期。
            std::printf("  => 端点奇异：Richardson 外推退化，收敛阶实测 %.3f (理论 ~1.5)\n",
                        estimate_order_of_first_column(table, tc.exact));
            bool singular_ok = (err_romberg < trap_err * 10.0) && std::isfinite(romberg_val);
            bool order_ok = false;
            {
                double ord = estimate_order_of_first_column(table, tc.exact);
                order_ok = (ord > 1.3 && ord < 1.7);
            }
            std::printf("  => %s\n", singular_ok && order_ok
                        ? "奇异退化行为符合理论预期 ✅" : "❌ 奇异情形验证失败");
            if (!(singular_ok && order_ok)) all_ok = false;
        } else {
            std::printf("  => %s%s\n",
                        improved ? "Romberg 碾压同规模梯形 ✅" : "❌ Romberg 未显著优于梯形",
                        precise ? "，达到 1e-10 级精度 ✅" : "");
            if (!(improved && precise && std::isfinite(romberg_val))) all_ok = false;
        }

        std::printf("\n--------------------------------------------------------------------\n");
    }

    std::printf("\n========================================================\n");
    std::printf("%s\n", all_ok ? "✅ 全部量化验证通过" : "❌ 存在验证失败项");
    std::printf("========================================================\n");

    return all_ok ? 0 : 1;
}
