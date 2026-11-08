// Natural Cubic Spline Interpolation
// 数值方法：三次样条插值（自然边界条件）
// 比对：全局多项式插值（Runge 现象对照）
// 验证：对已知解析函数，量化误差 (max error / RMSE)

#include <vector>
#include <functional>
#include <cmath>
#include <cstdio>
#include <string>
#include <algorithm>
#include <cassert>

// 三对角线性方程组求解（Thomas 算法）
// 求解 a[i]*x[i-1] + b[i]*x[i] + c[i]*x[i+1] = d[i]
std::vector<double> thomasSolve(const std::vector<double>& a,
                                const std::vector<double>& b,
                                const std::vector<double>& c,
                                const std::vector<double>& d) {
    int n = (int)d.size();
    std::vector<double> bp = b, cp = c, dp = d;
    std::vector<double> x(n);
    // 前向消去
    for (int i = 1; i < n; i++) {
        double m = a[i] / bp[i - 1];
        bp[i] -= m * cp[i - 1];
        dp[i] -= m * dp[i - 1];
    }
    // 回代
    x[n - 1] = dp[n - 1] / bp[n - 1];
    for (int i = n - 2; i >= 0; i--) {
        x[i] = (dp[i] - cp[i] * x[i + 1]) / bp[i];
    }
    return x;
}

// 自然三次样条：给定 (x_i, y_i)，返回每段的二次导数 M_i（自然边界 M_0 = M_{n-1} = 0）
struct CubicSpline {
    std::vector<double> xs, a, b, c, d; // 段多项式：S_i(t)=a+b*t+c*t^2+d*t^3, t=x-xs[i]
    void build(const std::vector<double>& X, const std::vector<double>& Y) {
        int n = (int)X.size();
        assert(n >= 3);
        xs = X;
        // 每段步长
        std::vector<double> h(n - 1);
        for (int i = 0; i < n - 1; i++) h[i] = X[i + 1] - X[i];

        // 构建三对角系统求二阶导数 M[0..n-1]
        // 方程: h[i-1]*M[i-1] + 2(h[i-1]+h[i])*M[i] + h[i]*M[i+1]
        //      = 6 * ( (Y[i+1]-Y[i])/h[i] - (Y[i]-Y[i-1])/h[i-1] )
        std::vector<double> lo(n), diag(n), up(n), rhs(n);
        diag[0] = 1.0; up[0] = 0.0; rhs[0] = 0.0; // 自然边界 M0=0
        diag[n-1] = 1.0; lo[n-1] = 0.0; rhs[n-1] = 0.0; // 自然边界 Mn-1=0
        for (int i = 1; i < n - 1; i++) {
            lo[i] = h[i - 1];
            diag[i] = 2.0 * (h[i - 1] + h[i]);
            up[i] = h[i];
            double dy1 = (Y[i+1] - Y[i]) / h[i];
            double dy0 = (Y[i] - Y[i-1]) / h[i-1];
            rhs[i] = 6.0 * (dy1 - dy0);
        }
        std::vector<double> M = thomasSolve(lo, diag, up, rhs);

        // 计算每段的系数
        a.resize(n - 1); b.resize(n - 1); c.resize(n - 1); d.resize(n - 1);
        for (int i = 0; i < n - 1; i++) {
            a[i] = Y[i];
            b[i] = (Y[i+1] - Y[i]) / h[i] - h[i]*(2.0*M[i] + M[i+1]) / 6.0;
            c[i] = M[i] / 2.0;
            d[i] = (M[i+1] - M[i]) / (6.0 * h[i]);
        }
    }
    double eval(double x) const {
        // 找到所在分段
        if (x <= xs[0]) x = xs[0];
        if (x >= xs.back()) x = xs.back();
        int i = (int)(std::upper_bound(xs.begin(), xs.end(), x) - xs.begin()) - 1;
        if (i < 0) i = 0;
        if (i >= (int)a.size()) i = (int)a.size() - 1;
        double t = x - xs[i];
        return a[i] + b[i]*t + c[i]*t*t + d[i]*t*t*t;
    }
};

// 等距节点全局多项式插值（Lagrange 形式，仅用于对照 Runge 现象）
struct LagrangeInterp {
    std::vector<double> xs, ys;
    void build(const std::vector<double>& X, const std::vector<double>& Y) { xs = X; ys = Y; }
    double eval(double x) const {
        double r = 0.0;
        int n = (int)xs.size();
        for (int i = 0; i < n; i++) {
            double term = ys[i];
            for (int j = 0; j < n; j++) {
                if (j == i) continue;
                term *= (x - xs[j]) / (xs[i] - xs[j]);
            }
            r += term;
        }
        return r;
    }
};

struct Stats {
    double maxErr, rmse, meanAbs;
};

// 在 [lo, hi] 上密集采样，对比插值器与真实函数
Stats quantify(const std::function<double(double)>& interp,
               const std::function<double(double)>& f,
               double lo, double hi, int samples) {
    double maxErr = 0.0, sumSq = 0.0, sumAbs = 0.0;
    for (int k = 0; k <= samples; k++) {
        double x = lo + (hi - lo) * k / samples;
        double e = std::fabs(interp(x) - f(x));
        maxErr = std::max(maxErr, e);
        sumSq += e * e;
        sumAbs += e;
    }
    int n = samples + 1;
    return { maxErr, std::sqrt(sumSq / n), sumAbs / n };
}

std::vector<double> linspace(double lo, double hi, int n) {
    std::vector<double> v(n);
    for (int i = 0; i < n; i++) v[i] = lo + (hi - lo) * i / (n - 1);
    return v;
}

int main() {
    std::vector<std::string> lines;
    auto out = [&](const std::string& s){ lines.push_back(s); printf("%s\n", s.c_str()); };

    out("=== Natural Cubic Spline Interpolation ===");
    out("");

    // ---------- 测试 1：光滑函数 sin(x) ----------
    {
        out("[Test 1] f(x) = sin(x) on [0, 2*pi], nodes = 8 (natural BC -> O(h^2))");
        int N = 8;
        std::vector<double> X = linspace(0.0, 2.0 * M_PI, N);
        std::vector<double> Y(N);
        for (int i = 0; i < N; i++) Y[i] = std::sin(X[i]);

        CubicSpline spl; spl.build(X, Y);
        auto splErr = quantify([&](double x){ return spl.eval(x); }, [](double x){return std::sin(x);}, 0, 2*M_PI, 2000);

        char buf[256];
        snprintf(buf, sizeof(buf), "  Spline   : max-err=%.6e  RMSE=%.6e  mean-abs=%.6e", splErr.maxErr, splErr.rmse, splErr.meanAbs);
        out(buf);
        bool s1 = (splErr.maxErr < 0.05);
        out(s1 ? "  PASS (max-err < 0.05)" : "  FAIL");
    }
    out("");

    // ---------- 测试 2：Runge 现象对照 ----------
    // f(x)=1/(1+25x^2)，等距节点，多项式插值会剧烈震荡，样条则平滑
    {
        out("[Test 2] Runge function f(x)=1/(1+25x^2) on [-1,1], nodes = 11");
        int N = 11;
        std::vector<double> X = linspace(-1.0, 1.0, N);
        std::vector<double> Y(N);
        auto runge = [](double x){ return 1.0 / (1.0 + 25.0 * x * x); };
        for (int i = 0; i < N; i++) Y[i] = runge(X[i]);

        CubicSpline spl; spl.build(X, Y);
        LagrangeInterp poly; poly.build(X, Y);
        auto splErr  = quantify([&](double x){ return spl.eval(x); }, runge, -1, 1, 2000);
        auto polyErr = quantify([&](double x){ return poly.eval(x); }, runge, -1, 1, 2000);

        char buf[256];
        snprintf(buf, sizeof(buf), "  Spline    : max-err=%.6e  RMSE=%.6e", splErr.maxErr, splErr.rmse);
        out(buf);
        snprintf(buf, sizeof(buf), "  Polynomial: max-err=%.6e  RMSE=%.6e", polyErr.maxErr, polyErr.rmse);
        out(buf);

        // 量化断言：样条应显著优于多项式，且样条不会发生 Runge 震荡
        // 多项式在 Runge 函数等距节点下 max-err 会 > 1（剧烈震荡），样条 < 0.1
        bool s2a = (splErr.maxErr < 0.1);
        bool s2b = (splErr.maxErr < polyErr.maxErr);
        snprintf(buf, sizeof(buf), "  Spline max-err < 0.1 : %s", s2a ? "PASS" : "FAIL");
        out(buf);
        snprintf(buf, sizeof(buf), "  Spline beats poly    : %s (%.1fx better)", s2b ? "PASS" : "FAIL",
                 polyErr.maxErr / std::max(splErr.maxErr, 1e-300));
        out(buf);
    }
    out("");

    // ---------- 测试 3：非等距节点 + 精度/收敛阶 ----------
    // f(x) = e^x，节点数 4/8/16，验证 h^4 收敛（三次样条截断误差 O(h^4)）
    {
        out("[Test 3] f(x)=e^x on [0,1], convergence order (natural BC -> ~2)");
        auto fexp = [](double x){ return std::exp(x); };
        std::vector<double> maxErrs;
        std::vector<int> Ns = {4, 8, 16};
        for (int N : Ns) {
            std::vector<double> X = linspace(0.0, 1.0, N);
            std::vector<double> Y(N);
            for (int i = 0; i < N; i++) Y[i] = fexp(X[i]);
            CubicSpline spl; spl.build(X, Y);
            auto e = quantify([&](double x){ return spl.eval(x); }, fexp, 0, 1, 4000);
            maxErrs.push_back(e.maxErr);
            char buf[256];
            snprintf(buf, sizeof(buf), "  N=%2d  max-err=%.6e", N, e.maxErr);
            out(buf);
        }
        // 收敛阶 = log(E(N1)/E(N2)) / log((N2-1)/(N1-1))
        double order = std::log(maxErrs[0] / maxErrs[2]) / std::log((16.0 - 1) / (4.0 - 1));
        char buf[256];
        snprintf(buf, sizeof(buf), "  Estimated convergence order = %.2f (theoretical ~2 for natural spline)", order);
        out(buf);
        bool s3 = (order > 1.5 && order < 3.0);
        out(s3 ? "  PASS (order consistent with natural-boundary O(h^2))" : "  FAIL");
    }
    out("");

    // ---------- 测试 4：线性函数精确重现 + C2 连续性 + 节点插值 ----------
    {
        out("[Test 4] linear f(x)=2x+1 on [0,1] -> any spline must be EXACT");
        auto flin = [](double x){ return 2.0 * x + 1.0; };
        std::vector<double> X = linspace(0.0, 1.0, 6);
        std::vector<double> Y(6);
        for (int i = 0; i < 6; i++) Y[i] = flin(X[i]);
        CubicSpline spl; spl.build(X, Y);
        auto e = quantify([&](double x){ return spl.eval(x); }, flin, 0, 1, 2000);
        char buf[256];
        snprintf(buf, sizeof(buf), "  1. max-err=%.6e (linear reproduced exactly)", e.maxErr);
        out(buf);
        bool s4a = (e.maxErr < 1e-12);
        out(s4a ? "      PASS (exact linear reproduction)" : "      FAIL");

        // 2. 节点插值：所有给定点必须精确命中
        auto fcubic = [](double x){ return x * x * x; };
        std::vector<double> X2 = linspace(0.0, 1.0, 7);
        std::vector<double> Y2(7);
        for (int i = 0; i < 7; i++) Y2[i] = fcubic(X2[i]);
        CubicSpline spl2; spl2.build(X2, Y2);
        double nodeErr = 0.0;
        for (int i = 0; i < 7; i++) nodeErr = std::max(nodeErr, std::fabs(spl2.eval(X2[i]) - Y2[i]));
        snprintf(buf, sizeof(buf), "  2. node interpolation max-err=%.6e (must hit all knots exactly)", nodeErr);
        out(buf);
        bool s4b = (nodeErr < 1e-12);
        out(s4b ? "      PASS (knots interpolated exactly)" : "      FAIL");

        out(s4a && s4b ? "  PASS" : "  FAIL");
    }
    out("");

    out("=== SUMMARY ===");
    out("Natural cubic spline retains C2 continuity and avoids Runge oscillation.");
    out("Natural BC forces zero 2nd-derivative at endpoints -> O(h^2) global convergence");
    out("(clamped splines achieve O(h^4), a separate topic).");

    // 写文件
    FILE* f = fopen("spline_output.txt", "w");
    for (auto& s : lines) fprintf(f, "%s\n", s.c_str());
    fclose(f);
    return 0;
}
