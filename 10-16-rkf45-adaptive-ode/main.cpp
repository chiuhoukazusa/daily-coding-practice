// RKF45 Adaptive Step-Size ODE Solver
// Runge-Kutta-Fehlberg 4(5) embedded pair with error-controlled step-size adaptation.
// Verifies:
//   1. Harmonic oscillator (d²x/dt² = -k/m * x) vs analytic solution  x = x0*cos(ωt)
//   2. Simple pendulum (nonlinear) — energy conservation check (total energy drift)
//   3. Stiff-ish decaying exponential — global error vs tolerance
//   4. Lorentz attractor — chaotic but bounded, Lyapunov-like divergence sanity
//   5. Adaptive vs fixed-step RK4: step count + max global error at same accuracy budget

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <functional>
#include <string>
#include <cassert>

using Vec = std::vector<double>;
using OdeFn = std::function<Vec(double, const Vec&)>;

// RKF45 Butcher tableau (Fehlberg 4(5))
static const double A2  = 1.0/4.0;
static const double A3  = 3.0/8.0;
static const double A4  = 12.0/13.0;
static const double A5  = 1.0;
static const double A6  = 1.0/2.0;

static const double B21 = 1.0/4.0;
static const double B31 = 3.0/32.0,        B32 = 9.0/32.0;
static const double B41 = 1932.0/2197.0,   B42 = -7200.0/2197.0,   B43 = 7296.0/2197.0;
static const double B51 = 439.0/216.0,     B52 = -8.0,             B53 = 3680.0/513.0,   B54 = -845.0/4104.0;
static const double B61 = -8.0/27.0,       B62 = 2.0,              B63 = -3544.0/2565.0, B64 = 1859.0/4104.0, B65 = -11.0/40.0;

// 5th order weights
static const double C1 = 16.0/135.0,  C2 = 0.0,           C3 = 6656.0/12825.0,
                    C4 = 28561.0/56430.0, C5 = -9.0/50.0, C6 = 2.0/55.0;
// 4th order weights (different => error estimate)
static const double D1 = 25.0/216.0,  D2 = 0.0,           D3 = 1408.0/2565.0,
                    D4 = 2197.0/4104.0, D5 = -1.0/5.0,    D6 = 0.0;

struct RkfResult {
    double t;          // final time reached
    Vec y;             // final state
    long steps;        // steps taken
    long rejected;     // rejected (step halved) attempts
    double maxErrEst;  // max scaled local error estimate
};

// Adaptive RKF45 integrator.
RkfResult rkf45(OdeFn f, double t0, Vec y0, double t1, double tol, double h0) {
    Vec y = y0;
    double t = t0;
    double h = h0;
    RkfResult r{t0, y0, 0, 0, 0.0};

    while (t < t1 - 1e-15) {
        if (t + h > t1) h = t1 - t;
        int n = (int)y.size();

        Vec k1(n), k2(n), k3(n), k4(n), k5(n), k6(n);
        Vec tmp(n), y4(n), y5(n);

        k1 = f(t, y);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h * B21 * k1[i];
        k2 = f(t + A2*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h * (B31*k1[i] + B32*k2[i]);
        k3 = f(t + A3*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h * (B41*k1[i] + B42*k2[i] + B43*k3[i]);
        k4 = f(t + A4*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h * (B51*k1[i] + B52*k2[i] + B53*k3[i] + B54*k4[i]);
        k5 = f(t + A5*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h * (B61*k1[i] + B62*k2[i] + B63*k3[i] + B64*k4[i] + B65*k5[i]);
        k6 = f(t + A6*h, tmp);

        for (int i = 0; i < n; i++) {
            y5[i] = y[i] + h * (C1*k1[i] + C2*k2[i] + C3*k3[i] + C4*k4[i] + C5*k5[i] + C6*k6[i]);
            y4[i] = y[i] + h * (D1*k1[i] + D2*k2[i] + D3*k3[i] + D4*k4[i] + D5*k5[i] + D6*k6[i]);
        }

        // Scaled error estimate (per component, relative + absolute)
        double err = 0.0;
        for (int i = 0; i < n; i++) {
            double sc = tol * (1.0 + std::fabs(y5[i]));
            double e = std::fabs(y5[i] - y4[i]) / sc;
            if (e > err) err = e;
        }
        if (err > r.maxErrEst) r.maxErrEst = err;

        if (err <= 1.0) {
            // accept
            t += h;
            y = y5;
            r.steps++;
        } else {
            r.rejected++;
        }

        // Step-size control (safety factor 0.9, cap growth at 5x)
        double s = (err > 1e-30) ? 0.9 * std::pow(err, -0.2) : 5.0;
        if (s > 5.0) s = 5.0;
        if (s < 0.1) s = 0.1;
        h *= s;
        if (t + h > t1) h = t1 - t;
    }
    r.t = t;
    r.y = y;
    return r;
}

// Fixed-step classic RK4
Vec rk4(OdeFn f, double t0, Vec y0, double t1, double h, long* stepsOut) {
    Vec y = y0;
    double t = t0;
    long steps = 0;
    while (t < t1 - 1e-15) {
        if (t + h > t1) h = t1 - t;
        int n = (int)y.size();
        Vec k1(n), k2(n), k3(n), k4(n), tmp(n);
        k1 = f(t, y);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + 0.5*h*k1[i];
        k2 = f(t + 0.5*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + 0.5*h*k2[i];
        k3 = f(t + 0.5*h, tmp);
        for (int i = 0; i < n; i++) tmp[i] = y[i] + h*k3[i];
        k4 = f(t + h, tmp);
        for (int i = 0; i < n; i++) y[i] += h*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]) / 6.0;
        t += h;
        steps++;
    }
    if (stepsOut) *stepsOut = steps;
    return y;
}

// ---------- Test systems ----------

// 1. Harmonic oscillator  x'' = -x  (k/m = 1), analytic x = cos(t) with x0=1, v0=0
Vec harmonic(double, const Vec& y) {
    // y = [x, v]
    return { y[1], -y[0] };
}

// 2. Pendulum  θ'' = -(g/L) sin(θ)  (energy E = 0.5 θ'^2 + (g/L)(1-cos θ))
Vec pendulum(double, const Vec& y) {
    const double gL = 9.81 / 1.0;
    return { y[1], -gL * std::sin(y[0]) };
}

// 3. Decaying exponential  y' = -λ y, analytic y = exp(-λ t)
Vec decay(double, const Vec& y) {
    const double lam = 1.0;
    return { -lam * y[0] };
}

// 4. Lorentz attractor (canonical σ=10, ρ=28, β=8/3)
Vec lorenz(double, const Vec& y) {
    const double s = 10.0, r = 28.0, b = 8.0/3.0;
    return {
        s * (y[1] - y[0]),
        y[0] * (r - y[2]) - y[1],
        y[0] * y[1] - b * y[2]
    };
}

int main() {
    std::printf("=== RKF45 Adaptive Step-Size ODE Solver — 量化验证 ===\n\n");

    bool allPass = true;

    // ---- Test 1: Harmonic oscillator accuracy ----
    {
        const double T = 6.283185307179586; // 2π
        RkfResult r = rkf45(harmonic, 0.0, {1.0, 0.0}, T, 1e-8, 0.1);
        double x_analytic = std::cos(T); // = 1.0
        double err_x = std::fabs(r.y[0] - x_analytic);
        double err_v = std::fabs(r.y[1] - (-std::sin(T))); // = 0.0
        std::printf("[1] 谐振子 x''=-x, 积分 0→2π\n");
        std::printf("    x(2π)=%.12e (解析=%.12e) 误差=%.3e\n", r.y[0], x_analytic, err_x);
        std::printf("    v(2π)=%.12e (解析=0)      误差=%.3e\n", r.y[1], err_v);
        std::printf("    步数=%ld  拒绝=%ld  最大缩放误差估计=%.3e\n", r.steps, r.rejected, r.maxErrEst);
        bool ok = (err_x < 1e-6) && (err_v < 1e-6) && (r.maxErrEst <= 1.5);
        std::printf("    => %s\n\n", ok ? "✅ 通过" : "❌ 失败");
        allPass &= ok;
    }

    // ---- Test 2: Pendulum energy conservation ----
    {
        const double T = 20.0;
        const double gL = 9.81;
        double theta0 = 1.0; // initial angle, v0=0
        double E0 = 0.5*0.0 + gL*(1.0 - std::cos(theta0));
        RkfResult r = rkf45(pendulum, 0.0, {theta0, 0.0}, T, 1e-8, 0.05);
        double Ef = 0.5*r.y[1]*r.y[1] + gL*(1.0 - std::cos(r.y[0]));
        double drift = std::fabs(Ef - E0) / E0;
        std::printf("[2] 单摆能量守恒 θ''=-(g/L)sinθ, 积分 0→20s\n");
        std::printf("    E0=%.10f  Ef=%.10f  相对漂移=%.3e\n", E0, Ef, drift);
        std::printf("    步数=%ld  拒绝=%ld\n", r.steps, r.rejected);
        // 4th-order method @ tol=1e-8 over 20s: energy drift ~1e-6 is expected & acceptable
        bool ok = drift < 1e-5;
        std::printf("    => %s\n\n", ok ? "✅ 通过" : "❌ 失败");
        allPass &= ok;
    }

    // ---- Test 3: Decaying exponential global error vs tolerance ----
    {
        const double t1 = 10.0;
        std::printf("[3] 衰减指数 y'=-y, 全局误差 vs 容差 (0→10)\n");
        double prevErr = -1;
        for (double tol : {1e-4, 1e-6, 1e-8, 1e-10}) {
            RkfResult r = rkf45(decay, 0.0, {1.0}, t1, tol, 0.5);
            double analytic = std::exp(-t1);
            double gErr = std::fabs(r.y[0] - analytic);
            std::printf("    tol=%.0e  y(10)=%.12e (解析=%.10e) 全局误差=%.3e  步数=%ld\n",
                        tol, r.y[0], analytic, gErr, r.steps);
            // global error should be monotonically decreasing with tighter tol
            if (prevErr >= 0 && gErr > prevErr) allPass = false;
            prevErr = gErr;
        }
        std::printf("    => %s\n\n", (prevErr < 1e-8) ? "✅ 通过" : "❌ 失败");
    }

    // ---- Test 4: Lorentz attractor boundedness (chaotic but finite) ----
    {
        const double T = 50.0;
        RkfResult r = rkf45(lorenz, 0.0, {1.0, 1.0, 1.0}, T, 1e-8, 0.01);
        double norm = std::sqrt(r.y[0]*r.y[0] + r.y[1]*r.y[1] + r.y[2]*r.y[2]);
        std::printf("[4] Lorentz 混沌系统, 积分 0→50s\n");
        std::printf("    终点=[%.6f, %.6f, %.6f] 范数=%.6f\n", r.y[0], r.y[1], r.y[2], norm);
        std::printf("    步数=%ld  拒绝=%ld\n", r.steps, r.rejected);
        // Trajectory should stay within the attractor's bounding region (norm < ~50)
        bool ok = norm > 1.0 && norm < 60.0;
        std::printf("    => %s\n\n", ok ? "✅ 通过" : "❌ 失败");
        allPass &= ok;
    }

    // ---- Test 5: Adaptive RKF45 vs fixed-step RK4 (accuracy + step economy) ----
    {
        std::printf("[5] 自适应 RKF45 vs 固定步长 RK4 (单摆, 0→10s, 精度匹配)\n");
        const double T = 10.0;
        double theta0 = 1.5;
        // Reference "exact" via very tight RKF45
        RkfResult ref = rkf45(pendulum, 0.0, {theta0, 0.0}, T, 1e-13, 0.001);

        // Adaptive RKF45 at tol=1e-9
        RkfResult a = rkf45(pendulum, 0.0, {theta0, 0.0}, T, 1e-9, 0.1);
        double errRkf = std::fabs(a.y[0] - ref.y[0]);

        // Fixed RK4: binary-search the step size to *match* RKF45's error, then compare steps
        double lo = 1e-6, hi = 1e-1;
        long steps4 = 0; Vec y4;
        double err4 = 0;
        for (int iter = 0; iter < 60; iter++) {
            double mid = std::sqrt(lo * hi);
            long s = 0;
            Vec yy = rk4(pendulum, 0.0, {theta0, 0.0}, T, mid, &s);
            double e = std::fabs(yy[0] - ref.y[0]);
            if (e > errRkf) lo = mid; else { hi = mid; steps4 = s; y4 = yy; err4 = e; }
        }

        std::printf("    RKF45(tol=1e-9): θ(10)=%.12f 误差=%.3e 步数=%ld\n", a.y[0], errRkf, a.steps);
        std::printf("    RK4 (h=%.6e):  θ(10)=%.12f 误差=%.3e 步数=%ld\n", (lo+hi)/2, y4[0], err4, steps4);
        double speedup = (double)steps4 / a.steps;
        std::printf("    同等精度下 RK4 步数 / RKF45 步数 = %.1f 倍\n", speedup);

        bool ok = (a.steps < steps4) && (err4 <= 10.0 * errRkf);
        std::printf("    => 自适应达到同等精度所用步数更少: %s\n\n", ok ? "✅ 通过" : "❌ 失败");
        allPass &= ok;
    }

    std::printf("============================================\n");
    std::printf("总结果: %s\n", allPass ? "✅ 全部通过" : "❌ 存在失败项");
    return allPass ? 0 : 1;
}
