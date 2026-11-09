// RANSAC Robust Model Fitting — Daily Coding Practice 2026-11-10
// 方向：算法 / 计算机视觉 / 计算几何
// 技术点：RANSAC（随机采样一致性）、直线拟合、圆拟合、内点/外点判定、
//        最小二乘（OLS）基准对比、鲁棒性量化验证
//
// 目标：展示 RANSAC 在含大量离群点（outliers）数据下，仍能正确估计模型参数，
//       并量化对比最小二乘法在同样数据下的退化表现。

#include <bits/stdc++.h>
using namespace std;

// ---------------------------------------------------------------------------
// PRNG：可复现的随机数（用于数据生成与 RANSAC 采样）
// ---------------------------------------------------------------------------
struct RNG {
    uint64_t s;
    explicit RNG(uint64_t seed) : s(seed) {}
    uint64_t next() {           // xorshift64*
        uint64_t x = s;
        x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
        s = x;
        return x * 0x2545F4914F6CDD1DULL;
    }
    double uniform() { return (double)(next() >> 11) / (double)(1ULL << 53); } // [0,1)
    double gauss() { // Box-Muller
        double u1 = uniform(), u2 = uniform();
        return sqrt(-2.0 * log(u1 + 1e-12)) * cos(2.0 * M_PI * u2);
    }
};

// ---------------------------------------------------------------------------
// 点云生成：真实直线 y = ax + b，叠加高斯噪声 + 离群点(均匀散布)
// ---------------------------------------------------------------------------
struct Data {
    vector<double> x, y;
};

Data gen_line_data(size_t n_inlier, size_t n_outlier, double a, double b,
                   double noise_sigma, double outlier_range, uint64_t seed) {
    RNG rng(seed);
    Data d;
    size_t total = n_inlier + n_outlier;
    d.x.resize(total); d.y.resize(total);
    vector<size_t> idx(total);
    iota(idx.begin(), idx.end(), 0);
    for (size_t i = 0; i < total; ++i) swap(idx[i], idx[rng.next() % total]); // 打乱
    for (size_t k = 0; k < total; ++k) {
        size_t i = idx[k];
        double xx = (rng.uniform() - 0.5) * 20.0;      // x 范围 [-10, 10]
        if (k < n_inlier) {
            double yy = a * xx + b + noise_sigma * rng.gauss();
            d.x[i] = xx; d.y[i] = yy;
        } else {
            d.x[i] = xx; d.y[i] = (rng.uniform() - 0.5) * outlier_range;
        }
    }
    return d;
}

Data gen_circle_data(size_t n_inlier, size_t n_outlier, double cx, double cy,
                     double r, double noise_sigma, double outlier_range, uint64_t seed) {
    RNG rng(seed);
    Data d;
    size_t total = n_inlier + n_outlier;
    d.x.resize(total); d.y.resize(total);
    vector<size_t> idx(total);
    iota(idx.begin(), idx.end(), 0);
    for (size_t i = 0; i < total; ++i) swap(idx[i], idx[rng.next() % total]);
    for (size_t k = 0; k < total; ++k) {
        size_t i = idx[k];
        if (k < n_inlier) {
            double theta = rng.uniform() * 2.0 * M_PI;
            double rr = r + noise_sigma * rng.gauss();
            d.x[i] = cx + rr * cos(theta);
            d.y[i] = cy + rr * sin(theta);
        } else {
            d.x[i] = (rng.uniform() - 0.5) * outlier_range;
            d.y[i] = (rng.uniform() - 0.5) * outlier_range;
        }
    }
    return d;
}

// ---------------------------------------------------------------------------
// 最小二乘直线拟合（用作基准对比）
// ---------------------------------------------------------------------------
struct Line { double a, b; };   // y = a x + b

Line ols_line(const Data& d) {
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    size_t n = d.x.size();
    for (size_t i = 0; i < n; ++i) {
        sx += d.x[i]; sy += d.y[i];
        sxx += d.x[i] * d.x[i];
        sxy += d.x[i] * d.y[i];
    }
    double denom = n * sxx - sx * sx;
    double a = (n * sxy - sx * sy) / denom;
    double b = (sy - a * sx) / n;
    return {a, b};
}

// 最小二乘圆拟合（Kasa 线性化法，用作基准对比）
struct Circle { double cx, cy, r; };

Circle ols_circle(const Data& d) {
    double sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0;
    size_t n = d.x.size();
    for (size_t i = 0; i < n; ++i) {
        double x = d.x[i], y = d.y[i];
        double z = x * x + y * y;
        sx += x; sy += y; sxx += x*x; syy += y*y; sxy += x*y;
        sxz += x*z; syz += y*z;
    }
    // 线性系统： [sxx sxy sx] [c1]   与 [sxz]
    //          [sxy syy sy] [c2]  =    [syz]
    //          [sx  sy  n ] [c3]      [zsum]
    // 解 2x2 子问题（利用第三个方程消元）：实际用 3x3 直接解
    double zsum = 0;
    for (size_t i = 0; i < n; ++i) zsum += d.x[i]*d.x[i] + d.y[i]*d.y[i];
    // 构造并解 3x3 线性系统 via 高斯消元
    double A[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, (double)n}};
    double B[3] = {sxz, syz, zsum};
    for (int col = 0; col < 3; ++col) {
        int piv = col;
        for (int r = col + 1; r < 3; ++r)
            if (fabs(A[r][col]) > fabs(A[piv][col])) piv = r;
        swap(A[col], A[piv]); swap(B[col], B[piv]);
        double d0 = A[col][col];
        for (int r = 0; r < 3; ++r) {
            if (r == col) continue;
            double f = A[r][col] / d0;
            for (int c = col; c < 3; ++c) A[r][c] -= f * A[col][c];
            B[r] -= f * B[col];
        }
    }
    double b1 = B[0] / A[0][0], b2 = B[1] / A[1][1], b3 = B[2] / A[2][2];
    double cx = b1 / 2.0, cy = b2 / 2.0;
    double r = sqrt(b3 + cx*cx + cy*cy);
    return {cx, cy, r};
}

// ---------------------------------------------------------------------------
// RANSAC 直线拟合
// ---------------------------------------------------------------------------
Line ransac_line(const Data& d, double inlier_thresh, int max_iter, uint64_t seed,
                 size_t* out_inliers = nullptr) {
    RNG rng(seed);
    size_t n = d.x.size();
    size_t best_count = 0;
    Line best{0, 0};

    for (int it = 0; it < max_iter; ++it) {
        size_t i1 = rng.next() % n, i2 = rng.next() % n;
        if (i1 == i2) continue;
        double dx = d.x[i2] - d.x[i1];
        if (fabs(dx) < 1e-9) continue;
        double a = (d.y[i2] - d.y[i1]) / dx;
        double b = d.y[i1] - a * d.x[i1];

        // 统计内点
        size_t cnt = 0;
        for (size_t j = 0; j < n; ++j) {
            double dist = fabs(a * d.x[j] - d.y[j] + b) / sqrt(a*a + 1.0);
            if (dist < inlier_thresh) cnt++;
        }
        if (cnt > best_count) { best_count = cnt; best = {a, b}; }
    }
    // 用最佳内点集合重新做 OLS（精修）
    vector<double> ix, iy;
    for (size_t j = 0; j < n; ++j) {
        double dist = fabs(best.a * d.x[j] - d.y[j] + best.b) / sqrt(best.a*best.a + 1.0);
        if (dist < inlier_thresh) { ix.push_back(d.x[j]); iy.push_back(d.y[j]); }
    }
    Data ref; ref.x = ix; ref.y = iy;
    Line refined = ols_line(ref);
    if (out_inliers) *out_inliers = ix.size();
    return refined;
}

// ---------------------------------------------------------------------------
// RANSAC 圆拟合（三点定圆）
// ---------------------------------------------------------------------------
Circle ransac_circle(const Data& d, double inlier_thresh, int max_iter, uint64_t seed,
                     size_t* out_inliers = nullptr) {
    RNG rng(seed);
    size_t n = d.x.size();
    size_t best_count = 0;
    Circle best{0, 0, 1};

    auto circle_from3 = [](double x1,double y1,double x2,double y2,double x3,double y3,
                           Circle& c) {
        double d0 = 2*(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2));
        if (fabs(d0) < 1e-9) return false;
        double ux = ((x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2)) / d0;
        double uy = ((x1*x1+y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3+y3*y3)*(x2-x1)) / d0;
        double r = sqrt((x1-ux)*(x1-ux) + (y1-uy)*(y1-uy));
        c = {ux, uy, r};
        return true;
    };

    for (int it = 0; it < max_iter; ++it) {
        size_t i1 = rng.next() % n, i2 = rng.next() % n, i3 = rng.next() % n;
        if (i1==i2 || i1==i3 || i2==i3) continue;
        Circle c;
        if (!circle_from3(d.x[i1],d.y[i1],d.x[i2],d.y[i2],d.x[i3],d.y[i3], c)) continue;
        size_t cnt = 0;
        for (size_t j = 0; j < n; ++j) {
            double dist = fabs(hypot(d.x[j]-c.cx, d.y[j]-c.cy) - c.r);
            if (dist < inlier_thresh) cnt++;
        }
        if (cnt > best_count) { best_count = cnt; best = c; }
    }
    // 内点精修（重新 OLS 拟合圆）
    vector<double> ix, iy;
    for (size_t j = 0; j < n; ++j) {
        double dist = fabs(hypot(d.x[j]-best.cx, d.y[j]-best.cy) - best.r);
        if (dist < inlier_thresh) { ix.push_back(d.x[j]); iy.push_back(d.y[j]); }
    }
    Data ref; ref.x = ix; ref.y = iy;
    Circle refined = ols_circle(ref);
    if (out_inliers) *out_inliers = ix.size();
    return refined;
}

// ---------------------------------------------------------------------------
// 工具：均方误差 / 内点率
// ---------------------------------------------------------------------------
double line_mse(const Data& d, Line l, double* rmse_out = nullptr) {
    double s = 0; size_t n = d.x.size();
    for (size_t i = 0; i < n; ++i) {
        double dy = l.a * d.x[i] + l.b - d.y[i];
        s += dy * dy;
    }
    double mse = s / n;
    if (rmse_out) *rmse_out = sqrt(mse);
    return mse;
}

double circle_geo_error(const Data& d, Circle c, double* rmse_out = nullptr) {
    double s = 0; size_t n = d.x.size();
    for (size_t i = 0; i < n; ++i) {
        double dist = hypot(d.x[i]-c.cx, d.y[i]-c.cy) - c.r;
        s += dist * dist;
    }
    double mse = s / n;
    if (rmse_out) *rmse_out = sqrt(mse);
    return mse;
}

int main() {
    cout << fixed << setprecision(6);
    cout << "================================================================" << endl;
    cout << "RANSAC 鲁棒模型拟合 — 量化验证报告" << endl;
    cout << "日期: 2026-11-10" << endl;
    cout << "================================================================" << endl;

    bool all_pass = true;

    // =====================================================================
    // 实验 1：直线拟合，不同离群点比例
    // =====================================================================
    cout << "\n[实验1] 直线拟合 (真值 y = 3.0x + 1.0)" << endl;
    cout << "  离群点比例 | 方法        | 估计 a     | 估计 b     | 内点数 | RMSE(全数据)" << endl;
    vector<double> outlier_ratios = {0.0, 0.2, 0.4, 0.6, 0.8};
    const double TRUTH_A = 3.0, TRUTH_B = 1.0;
    for (double ratio : outlier_ratios) {
        size_t n_in = 100;
        size_t n_out = (size_t)(n_in * ratio / (1.0 - ratio));
        Data d = gen_line_data(n_in, n_out, TRUTH_A, TRUTH_B, 0.3, 30.0, 12345 + (uint64_t)(ratio*1000));

        size_t inliers;
        Line r = ransac_line(d, 1.0, 1000, 42, &inliers);
        Line o = ols_line(d);
        double rmse_r, rmse_o;
        line_mse(d, r, &rmse_r);
        line_mse(d, o, &rmse_o);

        cout << "  " << setw(10) << (int)(ratio*100) << "% | RANSAC     | "
             << setw(10) << r.a << " | " << setw(10) << r.b << " | "
             << setw(6) << inliers << " | " << setw(10) << rmse_r << endl;
        cout << "  " << setw(10) << "" << "  | OLS(最小二乘) | "
             << setw(10) << o.a << " | " << setw(10) << o.b << " | "
             << setw(6) << "-" << " | " << setw(10) << rmse_o << endl;

        // 验证：RANSAC 应比 OLS 更接近真值（参数误差更小）
        double err_r = hypot(r.a - TRUTH_A, r.b - TRUTH_B);
        double err_o = hypot(o.a - TRUTH_A, o.b - TRUTH_B);
        cout << "      -> 参数误差: RANSAC=" << err_r << "  OLS=" << err_o
             << (err_r <= err_o ? "  ✅" : "  ❌") << endl;
        if (ratio > 0.1 && err_r > err_o) all_pass = false;
    }

    // =====================================================================
    // 实验 2：圆拟合 (真值圆心 (2,-1) 半径 5)
    // =====================================================================
    cout << "\n[实验2] 圆拟合 (真值 圆心(2,-1) r=5)" << endl;
    cout << "  离群点比例 | 方法        | 估计cx     | 估计cy     | 估计r      | 内点数 | RMSE" << endl;
    const double TCX = 2.0, TCY = -1.0, TR = 5.0;
    for (double ratio : {0.2, 0.5, 0.7}) {
        size_t n_in = 100;
        size_t n_out = (size_t)(n_in * ratio / (1.0 - ratio));
        Data d = gen_circle_data(n_in, n_out, TCX, TCY, TR, 0.3, 30.0, 777 + (uint64_t)(ratio*1000));

        size_t inliers;
        Circle r = ransac_circle(d, 1.0, 2000, 99, &inliers);
        Circle o = ols_circle(d);
        double rmse_r, rmse_o;
        circle_geo_error(d, r, &rmse_r);
        circle_geo_error(d, o, &rmse_o);

        cout << "  " << setw(10) << (int)(ratio*100) << "% | RANSAC     | "
             << setw(10) << r.cx << " | " << setw(10) << r.cy << " | " << setw(10) << r.r
             << " | " << setw(6) << inliers << " | " << setw(10) << rmse_r << endl;
        cout << "  " << setw(10) << "" << "  | OLS(最小二乘) | "
             << setw(10) << o.cx << " | " << setw(10) << o.cy << " | " << setw(10) << o.r
             << " | " << setw(6) << "-" << " | " << setw(10) << rmse_o << endl;

        double err_r = hypot(hypot(r.cx - TCX, r.cy - TCY), r.r - TR);
        double err_o = hypot(hypot(o.cx - TCX, o.cy - TCY), o.r - TR);
        cout << "      -> 参数误差: RANSAC=" << err_r << "  OLS=" << err_o
             << (err_r <= err_o ? "  ✅" : "  ❌") << endl;
        if (err_r > err_o) all_pass = false;
    }

    // =====================================================================
    // 实验 3：内点率验证（RANSAC 识别的内点数应 ≈ 真实内点数）
    // =====================================================================
    cout << "\n[实验3] 内点率验证 (真实内点=100)" << endl;
    {
        Data d = gen_line_data(100, 100, TRUTH_A, TRUTH_B, 0.3, 30.0, 2024);
        size_t inliers;
        ransac_line(d, 1.0, 1000, 7, &inliers);
        double recall = (double)inliers / 100.0;
        cout << "  识别内点数 = " << inliers << " / 100, 召回率 = " << recall * 100 << "%" << endl;
        cout << (recall > 0.85 ? "  ✅ 召回率 > 85%" : "  ❌ 召回率不足") << endl;
        if (recall <= 0.85) all_pass = false;
    }

    // =====================================================================
    // 实验 4：零离群点时 RANSAC 与 OLS 应一致
    // =====================================================================
    cout << "\n[实验4] 无离群点一致性验证" << endl;
    {
        Data d = gen_line_data(100, 0, TRUTH_A, TRUTH_B, 0.1, 30.0, 555);
        Line r = ransac_line(d, 1.0, 1000, 3);
        Line o = ols_line(d);
        double diff = hypot(r.a - o.a, r.b - o.b);
        cout << "  RANSAC=(" << r.a << "," << r.b << ")  OLS=(" << o.a << "," << o.b << ") 差异=" << diff << endl;
        cout << (diff < 0.05 ? "  ✅ 两者一致" : "  ❌ 不一致") << endl;
        if (diff >= 0.05) all_pass = false;
    }

    cout << "\n================================================================" << endl;
    cout << (all_pass ? "✅ 全部验证通过" : "❌ 存在失败项") << endl;
    cout << "================================================================" << endl;
    return all_pass ? 0 : 1;
}
