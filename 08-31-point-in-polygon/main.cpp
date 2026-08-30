// Point-in-Polygon Test: Ray Casting + Winding Number
// Quantitative verification via Monte Carlo area vs Shoelace exact area
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <random>

struct Pt { double x, y; };
typedef std::vector<Pt> Poly;

static double cross(const Pt& o, const Pt& a, const Pt& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Ray casting: cast ray to +x infinity, count edge crossings (even-odd rule)
bool rayCastInside(const Poly& p, const Pt& q) {
    bool inside = false;
    int n = (int)p.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Pt& a = p[i];
        const Pt& b = p[j];
        if ((a.y > q.y) != (b.y > q.y)) {
            double xint = a.x + (q.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (q.x < xint) inside = !inside;
        }
    }
    return inside;
}

// Winding number: count signed crossings
bool windingInside(const Poly& p, const Pt& q) {
    int wn = 0;
    int n = (int)p.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Pt& a = p[i];
        const Pt& b = p[j];
        if (a.y <= q.y) {
            if (b.y > q.y && cross(a, b, q) > 0) wn++;
        } else {
            if (b.y <= q.y && cross(a, b, q) < 0) wn--;
        }
    }
    return wn != 0;
}

// Shoelace exact signed area (x2)
static double shoelaceArea(const Poly& p) {
    double a = 0;
    int n = (int)p.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        a += p[i].x * p[j].y - p[j].x * p[i].y;
    }
    return std::fabs(a) * 0.5;
}

int main() {
    const int W = 800, H = 600;
    // Visual world bounds
    const double XMIN = -10, XMAX = 10, YMIN = -10, YMAX = 10;

    // Test polygons
    struct Named { const char* name; Poly poly; };
    std::vector<Named> polys = {
        {"convex",  {{-6,-4},{6,-4},{6,4},{-6,4}}},  // square (also convex)
        {"triangle",{{0,6},{-5,-4},{5,-4}}},
        {"concave", {{-6,-6},{0,6},{6,-6},{0,0}}},   // dart/concave
        {"star",    {{0,8},{-2.5,2},{-8,2},{-3.5,-2},{-5,-7},{0,-4},{5,-7},{3.5,-2},{8,2},{2.5,2}}}, // self? actually simple-ish star
    };

    // Output buffers for visualization
    std::vector<unsigned char> img(W * H * 3, 40); // dark bg

    // Results summary
    printf("=== Point-in-Polygon Verification ===\n");
    bool allOk = true;

    for (auto& named : polys) {
        const Poly& poly = named.poly;
        double exactArea = shoelaceArea(poly);

        // Cross-validation + Monte Carlo area
        std::mt19937 rng(12345 + std::hash<std::string>{}(named.name));
        std::uniform_real_distribution<double> dx(XMIN, XMAX);
        std::uniform_real_distribution<double> dy(YMIN, YMAX);

        int N = 400000;
        int agree = 0, disagree = 0;
        int insideCount = 0;
        for (int k = 0; k < N; k++) {
            Pt q{dx(rng), dy(rng)};
            bool rc = rayCastInside(poly, q);
            bool wn = windingInside(poly, q);
            if (rc == wn) agree++; else disagree++;
            if (rc) insideCount++;
        }

        // Monte Carlo area: bounding box area * fraction inside
        double bboxArea = (XMAX - XMIN) * (YMAX - YMIN);
        double mcArea = bboxArea * insideCount / N;
        double areaErrPct = std::fabs(mcArea - exactArea) / exactArea * 100.0;

        printf("\n[%s] 顶点数=%zu  精确面积(shoelace)=%.4f\n",
               named.name, poly.size(), exactArea);
        printf("  蒙特卡洛面积估算=%.4f  误差=%.3f%% (N=%d)\n", mcArea, areaErrPct, N);
        printf("  RayCast vs Winding 一致性: %d / %d (%.4f%%)  不一致=%d\n",
               agree, N, agree * 100.0 / N, disagree);
        if (disagree != 0) { printf("  ❌ 两算法不一致!\n"); allOk = false; }
        if (areaErrPct > 5.0) { printf("  ❌ 蒙特卡洛面积误差超5%%!\n"); allOk = false; }

        // Visualization: fill inside points as color per polygon (via ray casting)
        unsigned char color[3];
        if (std::string(named.name) == "convex")   { color[0]=220; color[1]=80;  color[2]=80; }
        else if (std::string(named.name)=="triangle"){ color[0]=80; color[1]=220; color[2]=80; }
        else if (std::string(named.name)=="concave") { color[0]=80; color[1]=80;  color[2]=220; }
        else                                          { color[0]=220; color[1]=220; color[2]=80; }

        for (int py = 0; py < H; py++) {
            double y = YMAX - (py + 0.5) * (YMAX - YMIN) / H;
            for (int px = 0; px < W; px++) {
                double x = XMIN + (px + 0.5) * (XMAX - XMIN) / W;
                Pt q{x, y};
                if (rayCastInside(poly, q)) {
                    int idx = (py * W + px) * 3;
                    img[idx+0] = (img[idx+0] + color[0]) / 2;
                    img[idx+1] = (img[idx+1] + color[1]) / 2;
                    img[idx+2] = (img[idx+2] + color[2]) / 2;
                }
            }
        }

        // Draw polygon outline (white)
        int n = (int)poly.size();
        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            Pt a = poly[i], b = poly[j];
            int steps = 200;
            for (int s = 0; s <= steps; s++) {
                double t = (double)s / steps;
                double x = a.x + (b.x - a.x) * t;
                double y = a.y + (b.y - a.y) * t;
                int px = (int)((x - XMIN) / (XMAX - XMIN) * (W - 1));
                int py = (int)((YMAX - y) / (YMAX - YMIN) * (H - 1));
                if (px >= 0 && px < W && py >= 0 && py < H) {
                    int idx = (py * W + px) * 3;
                    img[idx+0] = 255; img[idx+1] = 255; img[idx+2] = 255;
                }
            }
        }
    }

    // Write PPM
    FILE* f = fopen("pip_output.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    fwrite(img.data(), 1, img.size(), f);
    fclose(f);

    printf("\n%s\n", allOk ? "✅ 全部验证通过" : "❌ 存在失败项");
    return allOk ? 0 : 1;
}
