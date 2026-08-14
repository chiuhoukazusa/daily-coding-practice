// NURBS Curve Renderer
// Non-Uniform Rational B-Spline: exact circle/ellipse via weights,
// Cox-de Boor basis evaluation, weight control (curve pull), knot-multiplicity cusps.
// Quantitative verification: circle radius error, conic exactness, endpoint interpolation.

#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <iomanip>

struct Vec2 { double x, y; };
Vec2 operator+(Vec2 a, Vec2 b) { return {a.x+b.x, a.y+b.y}; }
Vec2 operator-(Vec2 a, Vec2 b) { return {a.x-b.x, a.y-b.y}; }
Vec2 operator*(Vec2 a, double s) { return {a.x*s, a.y*s}; }

// Cox-de Boor basis (handles u == U.back() via left-inclusive convention)
double Basis(int i, int p, double u, const std::vector<double>& U) {
    if (p == 0) {
        if (i+1 >= (int)U.size()) return 0.0;
        if (u >= U[i] && u < U[i+1]) return 1.0;
        // inclusive handle at the very last knot: u == U.back() belongs to last span
        if (u == U[i+1] && U[i+1] == U.back()) return 1.0;
        return 0.0;
    }
    double left = U[i+p] - U[i];
    double A = (left != 0.0) ? ((u - U[i]) / left) * Basis(i, p-1, u, U) : 0.0;
    double right = U[i+p+1] - U[i+1];
    double B = (right != 0.0) ? ((U[i+p+1] - u) / right) * Basis(i+1, p-1, u, U) : 0.0;
    return A + B;
}

Vec2 NURBSPoint(const std::vector<Vec2>& P, const std::vector<double>& w,
                const std::vector<double>& U, int p, double u) {
    int n = (int)P.size() - 1;
    Vec2 num = {0,0};
    double den = 0.0;
    for (int i = 0; i <= n; i++) {
        double b = Basis(i, p, u, U);
        double wb = w[i] * b;
        num = num + P[i] * wb;
        den += wb;
    }
    if (den == 0.0) return P[0];
    return num * (1.0 / den);
}

// Open clamped uniform knot vector: n = index of last ctrl pt, p = degree.
// Produces n+p+2 knot values (indices 0 .. n+p+1).
std::vector<double> MakeClampedUniform(int n, int p) {
    int m = n + p + 1;                 // last knot index
    std::vector<double> U(m + 1);
    int internal = m - 2*p;            // number of internal knot-spans' boundaries
    for (int i = 0; i <= m; i++) {
        if (i <= p) U[i] = 0.0;
        else if (i >= m - p) U[i] = 1.0;
        else U[i] = (double)(i - p) / (double)internal;
    }
    return U;
}

void writePPM(const std::string& fn, int W, int H, const std::vector<unsigned char>& rgb) {
    FILE* f = fopen(fn.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    fwrite(rgb.data(), 1, rgb.size(), f);
    fclose(f);
}
void drawPixel(std::vector<unsigned char>& img, int W, int H, int x, int y, int r, int g, int b) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int idx = (y * W + x) * 3;
    img[idx+0]=r; img[idx+1]=g; img[idx+2]=b;
}
void drawLine(std::vector<unsigned char>& img, int W, int H, int x0, int y0, int x1, int y1, int r, int g, int b) {
    int dx = std::abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -std::abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy, e2;
    while (true) {
        drawPixel(img, W, H, x0, y0, r, g, b);
        if (x0==x1 && y0==y1) break;
        e2 = 2*err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
void drawCircle(std::vector<unsigned char>& img, int W, int H, int cx, int cy, int rad, int r, int g, int b) {
    for (int y = -rad; y <= rad; y++)
        for (int x = -rad; x <= rad; x++) {
            int d2 = x*x + y*y;
            if (d2 <= rad*rad && d2 >= (rad-1)*(rad-1))
                drawPixel(img, W, H, cx+x, cy+y, r, g, b);
        }
}

int main() {
    const int W = 800, H = 800;
    std::vector<unsigned char> img(W*H*3, 255);
    const int p = 3; // cubic

    // ===== 1. NURBS circle (top-left): QUADRATIC rational arcs, weights 1 & sqrt(2)/2 =====
    {
        double cx = 200, cy = 180, R = 90;
        double w = std::sqrt(2.0)/2.0;
        // Full circle = 4 quarter-circle quadratic arcs sharing 9 control points.
        // Each quarter arc has 3 ctrl pts (endpoint, shoulder, endpoint), shoulder weight = w.
        // Circle centered (cx,cy), radius R, starting from angle -180..180 around squares.
        std::vector<Vec2> P;
        std::vector<double> wts;
        // 4 arcs: angles 0..90, 90..180, 180..270, 270..360
        for (int k = 0; k < 4; k++) {
            double a0 = k * M_PI/2.0;
            double a1 = (k+1) * M_PI/2.0;
            Vec2 e0 = {cx + R*std::cos(a0), cy + R*std::sin(a0)};
            Vec2 e1 = {cx + R*std::cos(a1), cy + R*std::sin(a1)};
            Vec2 sh = {cx + R/w * std::cos((a0+a1)/2.0), cy + R/w * std::sin((a0+a1)/2.0)};
            if (k == 0) { P.push_back(e0); wts.push_back(1.0); }
            P.push_back(sh); wts.push_back(w);
            P.push_back(e1); wts.push_back(1.0);
        }
        int n = (int)P.size() - 1; // 8
        int pp = 2; // quadratic
        // 4 arcs joined at shared endpoints: interior boundary knots have multiplicity 2
        std::vector<double> U = {0,0,0, 0.25,0.25, 0.5,0.5, 0.75,0.75, 1,1,1};

        for (int i = 0; i < n; i++) drawLine(img, W, H, (int)P[i].x, (int)P[i].y, (int)P[i+1].x, (int)P[i+1].y, 200,200,200);
        for (int i = 0; i <= n; i++) drawCircle(img, W, H, (int)P[i].x, (int)P[i].y, 3, 180,180,180);

        std::vector<double> errs;
        for (int s = 0; s <= 1000; s++) {
            double u = (double)s/1000.0;
            Vec2 pt = NURBSPoint(P, wts, U, pp, u);
            drawCircle(img, W, H, (int)std::lround(pt.x), (int)std::lround(pt.y), 1, 220, 40, 40);
            double dist = std::sqrt((pt.x-cx)*(pt.x-cx)+(pt.y-cy)*(pt.y-cy));
            errs.push_back(std::abs(dist - R));
        }
        double maxerr = *std::max_element(errs.begin(), errs.end());
        double meanerr = 0; for (double e : errs) meanerr += e; meanerr /= errs.size();
        std::cout << std::setprecision(6);
        std::cout << "[Circle quadratic] max radius err = " << maxerr << " px, mean = " << meanerr << " px\n";
    }

    // ===== 2. Weight pull (top-right): same control polygon, w=1 vs w=6 =====
    {
        std::vector<Vec2> P = {
            {460, 260}, {520, 100}, {640, 100}, {700, 260},
        };
        std::vector<double> uw = {1,1,1,1};
        std::vector<double> hw = {1,1,6,1}; // bump weight of P[2]
        int n = 3;
        std::vector<double> U = MakeClampedUniform(n, p);

        for (int i = 0; i < n; i++) drawLine(img, W, H, (int)P[i].x, (int)P[i].y, (int)P[i+1].x, (int)P[i+1].y, 200,200,200);
        for (int i = 0; i <= n; i++) drawCircle(img, W, H, (int)P[i].x, (int)P[i].y, 3, 180,180,180);

        // w=1 (blue)
        double d_uniform = 1e9;
        for (int s = 0; s <= 400; s++) {
            double u = (double)s/400.0;
            Vec2 pt = NURBSPoint(P, uw, U, p, u);
            drawCircle(img, W, H, (int)std::lround(pt.x), (int)std::lround(pt.y), 1, 30, 90, 220);
            double d = std::sqrt((pt.x-P[2].x)*(pt.x-P[2].x)+(pt.y-P[2].y)*(pt.y-P[2].y));
            d_uniform = std::min(d_uniform, d);
        }
        // w=6 (green), pulled toward P[2]
        double d_heavy = 1e9;
        for (int s = 0; s <= 400; s++) {
            double u = (double)s/400.0;
            Vec2 pt = NURBSPoint(P, hw, U, p, u);
            drawCircle(img, W, H, (int)std::lround(pt.x), (int)std::lround(pt.y), 1, 40, 160, 40);
            double d = std::sqrt((pt.x-P[2].x)*(pt.x-P[2].x)+(pt.y-P[2].y)*(pt.y-P[2].y));
            d_heavy = std::min(d_heavy, d);
        }
        std::cout << "[WeightPull] min dist to P[2]: w=1 -> " << d_uniform << " px, w=6 -> " << d_heavy << " px "
                  << (d_heavy < d_uniform ? "(✅ pulled closer)" : "(❌ wrong)") << "\n";
    }

    // ===== 3. Knot multiplicity cusp (bottom-left): triple interior knot =====
    {
        // cubic, 7 control points -> n=6, p=3 -> knots count = n+p+2 = 11
        int n = 6;
        // clamped cubic with ONE triple interior knot at 0.5 => cusp (C0 discontinuity)
        std::vector<double> U = {0,0,0,0, 0.5,0.5,0.5, 1,1,1,1}; // 11 values
        std::vector<Vec2> P = {
            {90, 600}, {150, 480}, {240, 660}, {320, 520}, {360, 600}, {300, 700}, {210, 660},
        };
        std::vector<double> wts(n+1, 1.0);

        for (int i = 0; i < n; i++) drawLine(img, W, H, (int)P[i].x, (int)P[i].y, (int)P[i+1].x, (int)P[i+1].y, 200,200,200);
        for (int i = 0; i <= n; i++) drawCircle(img, W, H, (int)P[i].x, (int)P[i].y, 3, 180,180,180);

        for (int s = 0; s <= 900; s++) {
            double u = (double)s/900.0;
            Vec2 pt = NURBSPoint(P, wts, U, p, u);
            drawCircle(img, W, H, (int)std::lround(pt.x), (int)std::lround(pt.y), 1, 160, 60, 200);
        }

        Vec2 p0 = NURBSPoint(P, wts, U, p, 0.0);
        Vec2 pn = NURBSPoint(P, wts, U, p, 1.0);
        double e0 = std::sqrt((p0.x-P[0].x)*(p0.x-P[0].x)+(p0.y-P[0].y)*(p0.y-P[0].y));
        double en = std::sqrt((pn.x-P[n].x)*(pn.x-P[n].x)+(pn.y-P[n].y)*(pn.y-P[n].y));
        std::cout << "[Endpoint] err P0 = " << e0 << ", Pn = " << en << " (expect ~0, clamped)\n";

        // verify C0 cusp: point exactly at u=0.5 equals P[3] (the knot-insertion control point)
        Vec2 mid = NURBSPoint(P, wts, U, p, 0.5);
        double dm = std::sqrt((mid.x-P[3].x)*(mid.x-P[3].x)+(mid.y-P[3].y)*(mid.y-P[3].y));
        std::cout << "[Cusp] dist(curve(u=0.5), P3) = " << dm << " (expect ~0 => passes through P3 = cusp)\n";
    }

    // ===== 4. Rational quadratic exact conic (bottom-right) =====
    {
        double cx = 580, cy = 600, R = 100;
        double theta = M_PI / 2.0;
        double wm = std::cos(theta/2.0);
        std::vector<Vec2> P = {
            {cx + R, cy},                                                                // angle 0
            {cx + R/wm * std::cos(theta/2.0), cy + R/wm * std::sin(theta/2.0)},          // shoulder
            {cx, cy + R},                                                                // angle 90
        };
        std::vector<double> wts = {1, wm, 1};
        int n = 2;
        std::vector<double> U = {0,0,0, 1,1,1}; // quadratic clamped, 6 knots

        for (int i = 0; i < n; i++) drawLine(img, W, H, (int)P[i].x, (int)P[i].y, (int)P[i+1].x, (int)P[i+1].y, 200,200,200);
        for (int i = 0; i <= n; i++) drawCircle(img, W, H, (int)P[i].x, (int)P[i].y, 3, 180,180,180);

        std::vector<double> errs;
        double ang_min = 1e9, ang_max = -1e9;
        for (int s = 0; s <= 1000; s++) {
            double u = (double)s/1000.0;
            Vec2 pt = NURBSPoint(P, wts, U, 2, u);
            drawCircle(img, W, H, (int)std::lround(pt.x), (int)std::lround(pt.y), 1, 220, 140, 20);
            double dist = std::sqrt((pt.x-cx)*(pt.x-cx)+(pt.y-cy)*(pt.y-cy));
            errs.push_back(std::abs(dist - R));
            double a = std::atan2(pt.y-cy, pt.x-cx);
            ang_min = std::min(ang_min, a); ang_max = std::max(ang_max, a);
        }
        double maxerr = *std::max_element(errs.begin(), errs.end());
        double meanerr = 0; for (double e : errs) meanerr += e; meanerr /= errs.size();
        std::cout << "[ConicArc] max radius err = " << maxerr << " px, mean = " << meanerr << " px\n";
        std::cout << "[ConicArc] angular span = [" << ang_min << ", " << ang_max << "] rad (expect [0, pi/2])\n";
    }

    writePPM("nurbs_output.ppm", W, H, img);
    std::cout << "✅ wrote nurbs_output.ppm (" << W << "x" << H << ")\n";
    return 0;
}
