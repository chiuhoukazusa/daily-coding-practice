// Brent's Method Root Finding
// 数值方法: Brent算法 = 二分法(Bisection) 保证收敛 + 逆二次插值(Inverse Quadratic) + 割线法(Secant) 加速
// 量化验证:
//   1. 对已知根的多组函数, 验证找到的根与解析根误差 < 1e-9
//   2. 验证返回结果满足 |f(x)| 极小
//   3. 对比 二分法 / 割线法 / Newton-Raphson 的迭代次数与收敛性(含重根/病态情况)
//   4. 验证 Brent 在病态函数(割线法发散)下仍能收敛(鲁棒性)
#include <bits/stdc++.h>
#include <cstdint>
#include <functional>
#include <iomanip>
using namespace std;

// 通用单变量函数类型
using Fn = function<double(double)>;

// ---- Brent 方法 ----
// 返回 {根, 迭代次数, 收敛标志}
struct Result {
    double root;
    int iters;
    bool converged;
};

Result brent(Fn f, double a, double b, double tol = 1e-12, int max_iter = 200) {
    double fa = f(a), fb = f(b);
    if (fa * fb > 0) return {NAN, 0, false}; // 端点同号, 无保证根

    // 确保 |f(b)| <= |f(a)|
    double c = a, fc = fa;
    double d = b - a; // d: 上一次步长
    if (fabs(fc) < fabs(fb)) { swap(a, b); swap(fa, fb); c = a; fc = fa; }
    // 注意: 调整后 a 是"最新最好", b 是上一次
    bool mflag = true;
    double s, fs;
    for (int i = 0; i < max_iter; ++i) {
        if (fb == 0.0 || (fabs(b - a) < tol)) {
            return {b, i, true};
        }

        // 尝试三种插值, 选择能给出"落在区间内且缩进足够"的候选 s
        if (fa != fc && fb != fc) {
            // 逆二次插值 (IQI)
            s = a * fb * fc / ((fa - fb) * (fa - fc))
              + b * fa * fc / ((fb - fa) * (fb - fc))
              + c * fa * fb / ((fc - fa) * (fc - fb));
        } else {
            // 割线法
            s = b - fb * (b - a) / (fb - fa);
        }

        // 判断 s 是否可用, 否则退回二分
        bool cond1 = (s < (3.0 * a + b) / 4.0 || s > b);
        bool cond2 = (mflag && fabs(s - b) >= fabs(b - c) / 2.0);
        bool cond3 = (!mflag && fabs(s - b) >= fabs(c - d) / 2.0);
        bool cond4 = (mflag && fabs(b - c) < tol);
        bool cond5 = (!mflag && fabs(c - d) < tol);
        if (cond1 || cond2 || cond3 || cond4 || cond5) {
            s = (a + b) / 2.0; // 二分
            mflag = true;
        } else {
            mflag = false;
        }

        fs = f(s);
        d = c; c = b; fc = fb;
        // 更新区间
        if (fa * fs < 0) {
            b = s; fb = fs;
        } else {
            a = s; fa = fs;
        }
        // 确保 |f(b)| <= |f(a)|, 保证 b 是更好的近似
        if (fabs(fa) < fabs(fb)) {
            swap(a, b); swap(fa, fb);
        }
    }
    return {b, max_iter, false};
}

// ---- 对比方法 ----
int bisection(Fn f, double a, double b, double tol, double &root) {
    double fa = f(a);
    for (int i = 0; i < 10000; ++i) {
        double m = (a + b) / 2.0, fm = f(m);
        if (fm == 0.0 || (b - a) < 2 * tol) { root = m; return i + 1; }
        if (fa * fm < 0) b = m; else { a = m; fa = fm; }
    }
    root = (a + b) / 2.0; return 10000;
}

int secant(Fn f, double a, double b, double tol, double &root, int max_iter = 200) {
    double fa = f(a), fb = f(b);
    for (int i = 0; i < max_iter; ++i) {
        if (fb == fa) { root = b; return i; }
        double s = b - fb * (b - a) / (fb - fa);
        double fs = f(s);
        if (fabs(fs) < tol) { root = s; return i + 1; }
        a = b; fa = fb; b = s; fb = fs;
    }
    root = b; return max_iter;
}

int newton(Fn f, Fn df, double x0, double tol, double &root, int max_iter = 200) {
    double x = x0;
    for (int i = 0; i < max_iter; ++i) {
        double fx = f(x), dfx = df(x);
        if (dfx == 0) { root = x; return i; }
        double nx = x - fx / dfx;
        if (fabs(nx - x) < tol) { root = nx; return i + 1; }
        x = nx;
    }
    root = x; return max_iter;
}

// ---- 测试函数集合 ----
struct TestCase {
    string name;
    Fn f;
    Fn df;      // 导数(供 Newton)
    double a, b; // 初始区间(供 Brent/二分/割线)
    double x0;   // Newton 初始点
    double trueRoot;
};

int main() {
    vector<TestCase> cases = {
        // 简单: 单根
        {"f1=(x-2)(x+3) 根=2",  [](double x){return (x-2)*(x+3);}, [](double x){return 2*x+1;}, 0, 5, 2.5, 2.0},
        // 三角函数
        {"f2=cos(x)-x 根≈0.739085", [](double x){return cos(x)-x;}, [](double x){return -sin(x)-1;}, 0, M_PI/2, 0.5, 0.7390851332151607},
        // 指数
        {"f3=e^x-5 根=ln5", [](double x){return exp(x)-5;}, [](double x){return exp(x);}, 0, 3, 1.0, log(5.0)},
        // 多项式多根, 取区间内
        {"f4=x^3-2x-5 根≈2.094551", [](double x){return x*x*x-2*x-5;}, [](double x){return 3*x*x-2;}, 2.0, 3.0, 2.5, 2.0945514815423265},
        // 重根(在 x=1 处穿过, 构造有符号变号的三重根)
        {"f5=(x-1)^3 三重根(变号) x=1", [](double x){return (x-1)*(x-1)*(x-1);}, [](double x){return 3*(x-1)*(x-1);}, 0.0, 2.0, 1.5, 1.0},
        // 病态: 割线法在接近区间端点时发散但仍可处理的函数
        {"f6=x-tanh(x)*4 x=0", [](double x){return x - 4.0*tanh(x);}, [](double x){return 1 - 4.0*(1-tanh(x)*tanh(x));}, -2.0, 2.0, 1.8, 0.0},
        // 平坦函数(变号): x^3 在 [-1,2] 穿零
        {"f7=x^3 x=0 三重根", [](double x){return x*x*x;}, [](double x){return 3*x*x;}, -1.0, 2.0, 1.5, 0.0},
    };

    int totalTests = 0, passed = 0;
    cout << fixed << setprecision(12);
    cout << "=== Brent's Method 量化验证报告 ===\n\n";

    for (auto &tc : cases) {
        cout << "----- " << tc.name << " -----\n";
        Result br = brent(tc.f, tc.a, tc.b);
        double bRoot, sRoot, nRoot;
        int bIt = bisection(tc.f, tc.a, tc.b, 1e-12, bRoot);
        int sIt = secant(tc.f, tc.a, tc.b, 1e-12, sRoot);
        int nIt = newton(tc.f, tc.df, tc.x0, 1e-12, nRoot);

        double br_err = fabs(br.root - tc.trueRoot);
        double f_at_root = fabs(tc.f(br.root));
        cout << "  Brent    根=" << br.root << " 迭代=" << br.iters
             << " |根误差|=" << br_err << " |f(根)|=" << f_at_root << "\n";
        cout << "  Bisection 迭代=" << bIt << " 根误差=" << fabs(bRoot - tc.trueRoot) << "\n";
        cout << "  Secant    迭代=" << sIt << " 根误差=" << fabs(sRoot - tc.trueRoot)
             << (sIt >= 200 ? " (不收敛!)" : "") << "\n";
        cout << "  Newton    迭代=" << nIt << " 根误差=" << fabs(nRoot - tc.trueRoot)
             << (nIt >= 200 ? " (不收敛!)" : "") << "\n";

        totalTests += 2;
        if (br.converged) passed++;
        else cout << "  ❌ Brent 未收敛\n";
        if (br_err < 1e-9 && f_at_root < 1e-9) passed++;
        else cout << "  ❌ Brent 根精度不足\n";

        // 鲁棒性检查: 即使重根/病态, Brent 也应收敛且 f 值极小
        if (f_at_root < 1e-6) passed++;
        else cout << "  ❌ Brent f(根) 不够小\n";
        totalTests++;
    }

    // 汇总
    cout << "\n=== 汇总 ===\n";
    cout << "通过 " << passed << "/" << totalTests << " 项检查\n";

    // 性能对比汇总 (平均迭代次数, 取前4个良态函数)
    double avgBrent = 0, avgBis = 0, avgSec = 0, avgNew = 0;
    int nGood = 0;
    for (int i = 0; i < 4; ++i) {
        auto &tc = cases[i];
        avgBrent += brent(tc.f, tc.a, tc.b).iters;
        double p; avgBis += bisection(tc.f, tc.a, tc.b, 1e-12, p);
        avgSec += secant(tc.f, tc.a, tc.b, 1e-12, p);
        avgNew += newton(tc.f, tc.df, tc.x0, 1e-12, p);
        nGood++;
    }
    cout << "良态函数平均迭代次数: Brent=" << avgBrent/nGood
         << " 二分=" << avgBis/nGood << " 割线=" << avgSec/nGood
         << " Newton=" << avgNew/nGood << "\n";
    cout << "Brent 相对二分的加速比 = " << (avgBis/avgBrent) << "x\n";
    cout << "Brent 相对割线的加速比 = " << (avgSec/avgBrent) << "x\n";

    // 鲁棒性结论
    cout << "\n=== 结论 ===\n";
    // 检查 f6 (病态) Newton 是否发散、f7 (三重根) 割线是否失精
    auto &heavy = cases[6]; // f7 x^3 三重根
    double pr; int pIt = secant(heavy.f, heavy.a, heavy.b, 1e-12, pr);
    Result hb = brent(heavy.f, heavy.a, heavy.b);
    cout << "三重根(x^3)情形: Secant 迭代=" << pIt << " 根误差=" << fabs(pr-heavy.trueRoot)
         << " vs Brent 迭代=" << hb.iters << " 根误差=" << fabs(hb.root-heavy.trueRoot) << "\n";

    bool allPass = (passed == totalTests);
    cout << (allPass ? "✅ ALL PASS" : "❌ SOME FAIL") << "\n";
    return allPass ? 0 : 1;
}
