// Adaptive Simpson Integration + Gauss-Legendre Quadrature
// 数值积分：自适应 Simpson 积分（递归细分）与高斯-勒让德求积对比
// 量化验证：对比解析解，报告误差与收敛阶

#include <cstdio>
#include <cmath>
#include <functional>
#include <vector>
#include <algorithm>
#include <cassert>
#include <string>

using f64 = double;

// ---- 测试函数 ----
// 1. f(x)=sin(x)  in [0, pi]  => ∫ = 2
// 2. f(x)=exp(-x^2) in [0, 3] => √π/2 * erf(3) ≈ 0.88622692545275801365
// 3. f(x)=1/(1+x^2) in [0,1] => π/4 ≈ 0.785398163397448...
// 4. f(x)=sqrt(x) in [0,1]   => 2/3 (尖点，自适应细分发挥作用)
// 5. f(x)=x^10 in [0,1]      => 1/11 ≈ 0.0909090909...  (高阶多项式)

struct TestCase {
    const char* name;
    std::function<f64(f64)> f;
    f64 a, b;
    f64 exact;
};

// ---- 自适应 Simpson 积分 ----
// 复合 Simpson: S(a,b) = (b-a)/6 * [f(a)+4f(mid)+f(b)]
// 自适应: 若 |S(a,mid)+S(mid,b) - S(a,b)|/15 <= eps，接受，否则细分

f64 simpson(f64 a, f64 b, const std::function<f64(f64)>& f) {
    f64 mid = 0.5 * (a + b);
    return (b - a) / 6.0 * (f(a) + 4.0 * f(mid) + f(b));
}

int eval_count = 0;

f64 adaptiveSimpson(f64 a, f64 b, f64 eps, f64 whole,
                    const std::function<f64(f64)>& f, int depth) {
    f64 mid = 0.5 * (a + b);
    f64 left  = simpson(a, mid, f);
    f64 right = simpson(mid, b, f);
    f64 delta = left + right - whole;
    if (depth <= 0 || std::fabs(delta) <= 15.0 * eps) {
        // 结果带误差修正（经典 Richardson 外推）
        return left + right + delta / 15.0;
    }
    return adaptiveSimpson(a, mid, eps / 2.0, left, f, depth - 1)
         + adaptiveSimpson(mid, b, eps / 2.0, right, f, depth - 1);
}

f64 integrateAdaptiveSimpson(f64 a, f64 b, f64 eps,
                             const std::function<f64(f64)>& f) {
    f64 whole = simpson(a, b, f);
    return adaptiveSimpson(a, b, eps, whole, f, 40);
}

// ---- 高斯-勒让德求积（Gauss-Legendre Quadrature）----
// 使用 n 点求积，节点/权重通过 Newton 迭代计算 Legendre 多项式的根
// 这里为了简单，硬编码常见阶数的节点与权重（n=5, n=10）

// 5-point Gauss-Legendre on [-1,1]
const f64 gl5_x[5] = {-0.9061798459386640, -0.5384693101056831, 0.0,
                       0.5384693101056831,  0.9061798459386640};
const f64 gl5_w[5] = {0.2369268850561891, 0.4786286704993665, 0.5688888888888889,
                       0.4786286704993665, 0.2369268850561891};

// 10-point Gauss-Legendre on [-1,1]
const f64 gl10_x[10] = {
    -0.9739065285171717, -0.8650633666889845, -0.6794095682990244, -0.4333953941292472, -0.1488743389816312,
     0.1488743389816312,  0.4333953941292472,  0.6794095682990244,  0.8650633666889845,  0.9739065285171717};
const f64 gl10_w[10] = {
    0.0666713443086881, 0.1494513491505806, 0.2190863625159820, 0.2692667193099963, 0.2955242247147529,
    0.2955242247147529, 0.2692667193099963, 0.2190863625159820, 0.1494513491505806, 0.0666713443086881};

f64 integrateGL(f64 a, f64 b, const std::function<f64(f64)>& f,
                const f64* xs, const f64* ws, int n) {
    f64 half = 0.5 * (b - a);
    f64 mid  = 0.5 * (a + b);
    f64 sum = 0.0;
    for (int i = 0; i < n; ++i) {
        sum += ws[i] * f(mid + half * xs[i]);
    }
    return half * sum;
}

int main() {
    std::vector<TestCase> tests;
    tests.push_back({
        "sin(x) in [0,pi]",
        [](f64 x){ return std::sin(x); }, 0.0, M_PI, 2.0});
    tests.push_back({
        "exp(-x^2) in [0,3]",
        [](f64 x){ return std::exp(-x*x); }, 0.0, 3.0,
        0.8862073482595210});
    tests.push_back({
        "1/(1+x^2) in [0,1]",
        [](f64 x){ return 1.0/(1.0+x*x); }, 0.0, 1.0,
        0.78539816339744830962});
    tests.push_back({
        "sqrt(x) in [0,1]  (singular derivative)",
        [](f64 x){ return std::sqrt(x); }, 0.0, 1.0,
        2.0/3.0});
    tests.push_back({
        "x^10 in [0,1]",
        [](f64 x){ f64 r=1.0; for(int i=0;i<10;++i) r*=x; return r; }, 0.0, 1.0,
        1.0/11.0});

    printf("=====================================================\n");
    printf("数值积分验证：自适应 Simpson vs 高斯-勒让德 vs 解析解\n");
    printf("=====================================================\n");

    int failures = 0;
    for (auto& t : tests) {
        f64 eps = 1e-10;
        f64 as = integrateAdaptiveSimpson(t.a, t.b, eps, t.f);
        f64 g5  = integrateGL(t.a, t.b, t.f, gl5_x,  gl5_w,  5);
        f64 g10 = integrateGL(t.a, t.b, t.f, gl10_x, gl10_w, 10);

        f64 e_as  = std::fabs(as  - t.exact);
        f64 e_g5  = std::fabs(g5  - t.exact);
        f64 e_g10 = std::fabs(g10 - t.exact);

        printf("\n%-38s  解析解 = %.12f\n", t.name, t.exact);
        printf("  AdaptiveSimpson = %.12f  err=%.3e\n", as,  e_as);
        printf("  Gauss-Leg(5)    = %.12f  err=%.3e\n", g5,  e_g5);
        printf("  Gauss-Leg(10)   = %.12f  err=%.3e\n", g10, e_g10);

        bool isSqrt = std::string(t.name).find("sqrt") != std::string::npos;
        // 自适应 Simpson 必须处处高精度（<1e-8）
        // 高斯-勒让德仅对光滑函数要求高精度；对 sqrt(x) 尖点函数，
        // 其退化正是教学重点（非光滑函数上多项式求积能力下降）
        f64 glTol = isSqrt ? 1e-3 : 1e-8;
        if (e_as > 1e-8 || e_g10 > glTol) {
            printf("  ❌ 超出容差\n");
            failures++;
            continue;
        }
        printf("  ✅ 通过\n");
    }

    // ---- 收敛阶验证（自适应 Simpson 在 sin 上）----
    printf("\n----- 自适应 Simpson 收敛阶验证 (sin(x) in [0,pi]) -----\n");
    printf("%-12s %-18s %-12s %s\n", "eps", "error", "ratio", "approx order");
    f64 prev_err = 0.0;
    for (f64 e : {1e-3, 1e-5, 1e-7, 1e-9, 1e-11}) {
        f64 val = integrateAdaptiveSimpson(0.0, M_PI, e,
                    [](f64 x){ return std::sin(x); });
        f64 err = std::fabs(val - 2.0);
        if (prev_err > 0.0) {
            // 每次 eps 缩小 100 倍，误差减小 ratio 倍
            double ratio = prev_err / err;
            double order = std::log10(ratio) / 2.0; // eps 缩小 10^2
            printf("%-12.0e %-18.6e %-12.2f %.4f\n", e, err, ratio, order);
        } else {
            printf("%-12.0e %-18.6e %-12s %s\n", e, err, "-", "-");
        }
        prev_err = err;
    }

    printf("\n=====================================================\n");
    if (failures == 0) {
        printf("✅ 全部测试通过，数值积分实现正确\n");
    } else {
        printf("❌ %d 个测试未通过\n", failures);
    }
    printf("=====================================================\n");

    return failures == 0 ? 0 : 1;
}
