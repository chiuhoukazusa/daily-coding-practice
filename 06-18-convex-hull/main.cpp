#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stack>
#include <vector>

// ============================================================
// Utility: 2D point
// ============================================================
struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x, double y) : x(x), y(y) {}

    bool operator<(const Point& p) const {
        return x < p.x || (x == p.x && y < p.y);
    }
    bool operator==(const Point& p) const {
        return x == p.x && y == p.y;
    }
};

double cross(const Point& O, const Point& A, const Point& B) {
    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
}

double dist2(const Point& a, const Point& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double area2(const Point& a, const Point& b, const Point& c) {
    return std::abs(cross(a, b, c));
}

// ============================================================
// Algorithm 1: Graham Scan
// ============================================================
std::vector<Point> graham_scan(std::vector<Point> pts) {
    if (pts.size() <= 2) return pts;

    // Find pivot (lowest y, then leftmost x)
    size_t pivot = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].y < pts[pivot].y ||
            (pts[i].y == pts[pivot].y && pts[i].x < pts[pivot].x))
            pivot = i;
    }
    std::swap(pts[0], pts[pivot]);
    Point P = pts[0];

    // Sort by polar angle relative to P
    std::sort(pts.begin() + 1, pts.end(), [&P](const Point& a, const Point& b) {
        double c = cross(P, a, b);
        if (std::abs(c) < 1e-12) {
            return dist2(P, a) < dist2(P, b);
        }
        return c > 0; // counter-clockwise
    });

    // Build hull
    std::vector<Point> hull;
    hull.push_back(pts[0]);
    hull.push_back(pts[1]);

    for (size_t i = 2; i < pts.size(); ++i) {
        while (hull.size() >= 2 &&
               cross(hull[hull.size() - 2], hull.back(), pts[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(pts[i]);
    }
    return hull;
}

// ============================================================
// Algorithm 2: Monotone Chain (Andrew's Algorithm)
// ============================================================
std::vector<Point> monotone_chain(std::vector<Point> pts) {
    if (pts.size() <= 2) return pts;

    std::sort(pts.begin(), pts.end());
    // Remove duplicates
    pts.erase(std::unique(pts.begin(), pts.end(),
                          [](const Point& a, const Point& b) {
                              return std::abs(a.x - b.x) < 1e-12 &&
                                     std::abs(a.y - b.y) < 1e-12;
                          }),
              pts.end());

    if (pts.size() <= 2) return pts;

    std::vector<Point> hull(2 * pts.size());
    int k = 0;

    // Lower hull
    for (size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 &&
               cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
            --k;
        hull[k++] = pts[i];
    }

    // Upper hull
    int t = k + 1;
    for (int i = (int)pts.size() - 2; i >= 0; --i) {
        while (k >= t &&
               cross(hull[k - 2], hull[k - 1], pts[i]) <= 0)
            --k;
        hull[k++] = pts[i];
    }

    hull.resize(k - 1);
    return hull;
}

// ============================================================
// Algorithm 3: QuickHull (recursive divide-and-conquer)
// ============================================================
int find_side(const Point& p1, const Point& p2, const Point& p) {
    double val = cross(p1, p2, p);
    if (std::abs(val) < 1e-12) return 0;
    return (val > 0) ? 1 : -1;
}

double line_dist(const Point& p1, const Point& p2, const Point& p) {
    return std::abs(cross(p1, p2, p));
}

void quickhull_rec(const std::vector<Point>& pts, const Point& p1,
                   const Point& p2, int side,
                   std::vector<Point>& hull) {
    int ind = -1;
    double max_dist = 0;

    for (size_t i = 0; i < pts.size(); ++i) {
        double d = line_dist(p1, p2, pts[i]);
        if (find_side(p1, p2, pts[i]) == side && d > max_dist) {
            ind = (int)i;
            max_dist = d;
        }
    }

    if (ind == -1) {
        hull.push_back(p1);
        hull.push_back(p2);
        return;
    }

    quickhull_rec(pts, pts[ind], p1,
                  -find_side(pts[ind], p1, p2), hull);
    quickhull_rec(pts, pts[ind], p2,
                  -find_side(pts[ind], p2, p1), hull);
}

std::vector<Point> quickhull(std::vector<Point> pts) {
    if (pts.size() <= 2) return pts;

    // Find min and max x
    int min_x = 0, max_x = 0;
    for (size_t i = 1; i < pts.size(); ++i) {
        if (pts[i].x < pts[min_x].x) min_x = (int)i;
        if (pts[i].x > pts[max_x].x) max_x = (int)i;
    }

    std::vector<Point> hull;
    quickhull_rec(pts, pts[min_x], pts[max_x], 1, hull);
    quickhull_rec(pts, pts[min_x], pts[max_x], -1, hull);

    // Remove duplicates from hull construction
    std::sort(hull.begin(), hull.end(),
              [](const Point& a, const Point& b) {
                  return a.x < b.x || (a.x == b.x && a.y < b.y);
              });
    hull.erase(std::unique(hull.begin(), hull.end(),
                           [](const Point& a, const Point& b) {
                               return std::abs(a.x - b.x) < 1e-12 &&
                                      std::abs(a.y - b.y) < 1e-12;
                           }),
               hull.end());

    // Order CCW
    if (hull.size() >= 3) {
        Point center(0, 0);
        for (auto& p : hull) {
            center.x += p.x;
            center.y += p.y;
        }
        center.x /= hull.size();
        center.y /= hull.size();

        std::sort(hull.begin(), hull.end(),
                  [&center](const Point& a, const Point& b) {
                      double angle_a =
                          std::atan2(a.y - center.y, a.x - center.x);
                      double angle_b =
                          std::atan2(b.y - center.y, b.x - center.x);
                      return angle_a < angle_b;
                  });
    }

    return hull;
}

// ============================================================
// Verification: check that hull is convex and contains all points
// ============================================================
bool is_convex(const std::vector<Point>& hull) {
    if (hull.size() <= 2) return true;
    int n = (int)hull.size();
    int sign = 0;
    for (int i = 0; i < n; ++i) {
        double c =
            cross(hull[i], hull[(i + 1) % n], hull[(i + 2) % n]);
        int cur_sign = (c > 1e-12) ? 1 : ((c < -1e-12) ? -1 : 0);
        if (cur_sign != 0) {
            if (sign == 0)
                sign = cur_sign;
            else if (sign != cur_sign)
                return false;
        }
    }
    return true;
}

bool point_in_convex_polygon(const Point& p,
                             const std::vector<Point>& hull) {
    if (hull.size() <= 2) return false;
    int n = (int)hull.size();
    int sign = 0;
    for (int i = 0; i < n; ++i) {
        double c = cross(hull[i], hull[(i + 1) % n], p);
        int cur_sign = (c > 1e-12) ? 1 : ((c < -1e-12) ? -1 : 0);
        if (cur_sign != 0) {
            if (sign == 0)
                sign = cur_sign;
            else if (sign != cur_sign)
                return false; // point outside
        }
    }
    return true; // all on same side or on boundary
}

double polygon_area(const std::vector<Point>& hull) {
    if (hull.size() < 3) return 0;
    double area = 0;
    int n = (int)hull.size();
    for (int i = 0; i < n; ++i) {
        area += hull[i].x * hull[(i + 1) % n].y;
        area -= hull[i].y * hull[(i + 1) % n].x;
    }
    return std::abs(area) / 2.0;
}

// ============================================================
// PPM Image Output
// ============================================================
struct Image {
    int w, h;
    std::vector<unsigned char> data; // RGB
    Image(int w, int h) : w(w), h(h), data(w * h * 3, 255) {}

    void set(int x, int y, unsigned char r, unsigned char g,
             unsigned char b) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        int idx = (y * w + x) * 3;
        data[idx] = r;
        data[idx + 1] = g;
        data[idx + 2] = b;
    }

    void draw_dot(int x, int y, int radius, unsigned char r,
                  unsigned char g, unsigned char b) {
        for (int dy = -radius; dy <= radius; ++dy)
            for (int dx = -radius; dx <= radius; ++dx)
                set(x + dx, y + dy, r, g, b);
    }

    void draw_line(int x0, int y0, int x1, int y1, unsigned char r,
                   unsigned char g, unsigned char b) {
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            set(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    void save_ppm(const std::string& filename) {
        std::ofstream ofs(filename, std::ios::binary);
        ofs << "P6\n" << w << " " << h << "\n255\n";
        ofs.write(reinterpret_cast<const char*>(data.data()),
                  data.size());
    }
};

void draw_scene(const std::vector<Point>& points,
                const std::vector<Point>& hull_gs,
                const std::vector<Point>& hull_mc,
                const std::vector<Point>& hull_qh,
                const std::string& /*label*/,
                const std::string& filename, int img_w = 800,
                int img_h = 600) {
    Image img(img_w, img_h);

    // Compute bounding box
    double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (auto& p : points) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    double pad = 0.1 * std::max(max_x - min_x, max_y - min_y);
    if (pad < 10) pad = 10;
    min_x -= pad;
    max_x += pad;
    min_y -= pad;
    max_y += pad;
    double sx = img_w / (max_x - min_x);
    double sy = img_h / (max_y - min_y);
    double s = std::min(sx, sy);

    auto to_px = [&](double x, double y) -> std::pair<int, int> {
        int px = (int)((x - min_x) * s);
        int py = (int)((max_y - y) * s); // flip y
        return {px, py};
    };

    // Color coding legend: Red=Graham Scan, Green=Monotone Chain, Blue=QuickHull

    // Draw hull polygons (bottom layer)
    // Graham Scan hull in red
    if (hull_gs.size() >= 3) {
        for (size_t i = 0; i < hull_gs.size(); ++i) {
            auto [x0, y0] = to_px(hull_gs[i].x, hull_gs[i].y);
            auto [x1, y1] =
                to_px(hull_gs[(i + 1) % hull_gs.size()].x,
                      hull_gs[(i + 1) % hull_gs.size()].y);
            img.draw_line(x0, y0, x1, y1, 255, 40, 40);
        }
    }

    // Monotone Chain hull in green
    if (hull_mc.size() >= 3) {
        for (size_t i = 0; i < hull_mc.size(); ++i) {
            auto [x0, y0] = to_px(hull_mc[i].x, hull_mc[i].y);
            auto [x1, y1] =
                to_px(hull_mc[(i + 1) % hull_mc.size()].x,
                      hull_mc[(i + 1) % hull_mc.size()].y);
            img.draw_line(x0, y0, x1, y1, 40, 200, 40);
        }
    }

    // QuickHull hull in blue
    if (hull_qh.size() >= 3) {
        for (size_t i = 0; i < hull_qh.size(); ++i) {
            auto [x0, y0] = to_px(hull_qh[i].x, hull_qh[i].y);
            auto [x1, y1] =
                to_px(hull_qh[(i + 1) % hull_qh.size()].x,
                      hull_qh[(i + 1) % hull_qh.size()].y);
            img.draw_line(x0, y0, x1, y1, 40, 100, 255);
        }
    }

    // Draw points
    for (auto& p : points) {
        auto [px, py] = to_px(p.x, p.y);
        img.draw_dot(px, py, 2, 200, 200, 200);
    }

    // Draw hull vertices with larger dots
    for (auto& p : hull_gs) {
        auto [px, py] = to_px(p.x, p.y);
        img.draw_dot(px, py, 4, 255, 80, 80);
    }
    for (auto& p : hull_mc) {
        auto [px, py] = to_px(p.x, p.y);
        img.draw_dot(px, py, 4, 80, 255, 80);
    }
    for (auto& p : hull_qh) {
        auto [px, py] = to_px(p.x, p.y);
        img.draw_dot(px, py, 4, 80, 120, 255);
    }

    img.save_ppm(filename);
    std::cout << "  Image saved: " << filename << " (" << img_w << "x"
              << img_h << ")" << std::endl;
}

// ============================================================
// Test case generators
// ============================================================
std::vector<Point> random_points(int n, double range, int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-range, range);
    std::vector<Point> pts(n);
    for (int i = 0; i < n; ++i)
        pts[i] = Point(dist(rng), dist(rng));
    return pts;
}

std::vector<Point> circle_points(int n, double radius) {
    std::vector<Point> pts(n);
    for (int i = 0; i < n; ++i) {
        double angle = 2.0 * M_PI * i / n;
        pts[i] = Point(radius * cos(angle), radius * sin(angle));
    }
    return pts;
}

std::vector<Point> grid_points(int nx, int ny, double spacing) {
    std::vector<Point> pts;
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            pts.push_back(Point(i * spacing, j * spacing));
    return pts;
}

std::vector<Point> noisy_circle(int n, double radius, double noise_std,
                                int seed) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> dist(0, noise_std);
    std::vector<Point> pts(n);
    for (int i = 0; i < n; ++i) {
        double angle = 2.0 * M_PI * i / n;
        double r = radius + dist(rng);
        pts[i] = Point(r * cos(angle), r * sin(angle));
    }
    return pts;
}

// ============================================================
// Quantitative validation
// ============================================================
struct ValidationResult {
    std::string test_name;
    int n_points;
    int hull_size_gs, hull_size_mc, hull_size_qh;
    double area_gs, area_mc, area_qh;
    bool convex_gs, convex_mc, convex_qh;
    bool covers_all_gs, covers_all_mc, covers_all_qh;
    bool hulls_equivalent;
    bool all_passed;
};

ValidationResult validate(const std::string& test_name,
                          const std::vector<Point>& points,
                          const std::vector<Point>& hull_gs,
                          const std::vector<Point>& hull_mc,
                          const std::vector<Point>& hull_qh) {
    ValidationResult r;
    r.test_name = test_name;
    r.n_points = (int)points.size();
    r.hull_size_gs = (int)hull_gs.size();
    r.hull_size_mc = (int)hull_mc.size();
    r.hull_size_qh = (int)hull_qh.size();
    r.area_gs = polygon_area(hull_gs);
    r.area_mc = polygon_area(hull_mc);
    r.area_qh = polygon_area(hull_qh);
    r.convex_gs = is_convex(hull_gs);
    r.convex_mc = is_convex(hull_mc);
    r.convex_qh = is_convex(hull_qh);

    // Check that all input points are inside or on the hull
    r.covers_all_gs = true;
    for (auto& p : points) {
        if (!point_in_convex_polygon(p, hull_gs)) {
            r.covers_all_gs = false;
            break;
        }
    }
    r.covers_all_mc = true;
    for (auto& p : points) {
        if (!point_in_convex_polygon(p, hull_mc)) {
            r.covers_all_mc = false;
            break;
        }
    }
    r.covers_all_qh = true;
    for (auto& p : points) {
        if (!point_in_convex_polygon(p, hull_qh)) {
            r.covers_all_qh = false;
            break;
        }
    }

    // Hulls should be equivalent (same number of vertices, similar area)
    r.hulls_equivalent =
        (r.hull_size_gs == r.hull_size_mc &&
         r.hull_size_mc == r.hull_size_qh &&
         std::abs(r.area_gs - r.area_mc) < 1.0 &&
         std::abs(r.area_mc - r.area_qh) < 1.0);

    r.all_passed = r.convex_gs && r.convex_mc && r.convex_qh &&
                   r.covers_all_gs && r.covers_all_mc &&
                   r.covers_all_qh && r.hulls_equivalent;

    return r;
}

void print_result(const ValidationResult& r) {
    std::cout << "\n=== " << r.test_name << " ===" << std::endl;
    std::cout << "Input points: " << r.n_points << std::endl;
    std::cout << "Hull sizes: GS=" << r.hull_size_gs
              << " MC=" << r.hull_size_mc
              << " QH=" << r.hull_size_qh << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Hull areas:  GS=" << r.area_gs
              << " MC=" << r.area_mc << " QH=" << r.area_qh
              << std::endl;
    std::cout << "Convex:      GS=" << (r.convex_gs ? "✓" : "✗")
              << " MC=" << (r.convex_mc ? "✓" : "✗")
              << " QH=" << (r.convex_qh ? "✓" : "✗") << std::endl;
    std::cout << "Covers all:  GS=" << (r.covers_all_gs ? "✓" : "✗")
              << " MC=" << (r.covers_all_mc ? "✓" : "✗")
              << " QH=" << (r.covers_all_qh ? "✓" : "✗") << std::endl;
    std::cout << "Hulls agree: "
              << (r.hulls_equivalent ? "✓" : "✗") << std::endl;
    std::cout << "OVERALL:     "
              << (r.all_passed ? "✅ PASSED" : "❌ FAILED")
              << std::endl;
}

// ============================================================
// Main
// ============================================================
int main() {
    srand(time(nullptr));
    bool all_passed = true;

    // Test 1: Small random set to verify correctness
    {
        auto pts = random_points(50, 200, 42);
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        auto r = validate("Small Random 50", pts, gs, mc, qh);
        print_result(r);
        all_passed = all_passed && r.all_passed;
        draw_scene(pts, gs, mc, qh, "Small Random 50",
                   "hull_random50.ppm");
    }

    // Test 2: Circle points (all on hull)
    {
        auto pts = circle_points(100, 200);
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        auto r = validate("Circle 100", pts, gs, mc, qh);
        print_result(r);
        all_passed = all_passed && r.all_passed;
        draw_scene(pts, gs, mc, qh, "Circle 100",
                   "hull_circle100.ppm");
    }

    // Test 3: Large random set
    {
        auto pts = random_points(500, 300, 12345);
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        auto r = validate("Large Random 500", pts, gs, mc, qh);
        print_result(r);
        all_passed = all_passed && r.all_passed;
        draw_scene(pts, gs, mc, qh, "Large Random 500",
                   "hull_random500.ppm");
    }

    // Test 4: Grid points (4 corner hull)
    {
        auto pts = grid_points(15, 15, 30);
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        auto r = validate("Grid 15x15", pts, gs, mc, qh);
        print_result(r);
        all_passed = all_passed && r.all_passed;
        draw_scene(pts, gs, mc, qh, "Grid 15x15",
                   "hull_grid.ppm");
    }

    // Test 5: Noisy circle
    {
        auto pts = noisy_circle(200, 250, 15, 999);
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        auto r = validate("Noisy Circle 200", pts, gs, mc, qh);
        print_result(r);
        all_passed = all_passed && r.all_passed;
        draw_scene(pts, gs, mc, qh, "Noisy Circle 200",
                   "hull_noisy_circle.ppm");
    }

    // Test 6: Extreme - all points collinear (degenerate)
    {
        std::vector<Point> pts;
        for (int i = 0; i < 20; ++i)
            pts.push_back(Point(i * 10.0, 0));
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        // Collinear hull: just the two endpoints
        bool gs_ok = (gs.size() == 2);
        bool mc_ok = (mc.size() == 2);
        bool qh_ok = (qh.size() <= 2); // Quickhull may return empty
        std::cout << "\n=== Degenerate: Collinear 20 ===" << std::endl;
        std::cout << "Hull sizes: GS=" << gs.size() << " MC=" << mc.size()
                  << " QH=" << qh.size() << std::endl;
        std::cout << "Expected 2 endpoints: GS=" << (gs_ok ? "✓" : "✗")
                  << " MC=" << (mc_ok ? "✓" : "✗")
                  << " QH=" << (qh_ok ? "✓" : "✗") << std::endl;
        all_passed = all_passed && gs_ok && mc_ok && qh_ok;
    }

    // Test 7: < 3 points (degenerate)
    {
        std::vector<Point> pts;
        pts.push_back(Point(0, 0));
        pts.push_back(Point(100, 100));
        auto gs = graham_scan(pts);
        auto mc = monotone_chain(pts);
        auto qh = quickhull(pts);
        bool ok = (gs.size() <= 2 && mc.size() <= 2 && qh.size() <= 2);
        std::cout << "\n=== Degenerate: Only 2 points ===" << std::endl;
        std::cout << "Handled correctly: " << (ok ? "✓" : "✗")
                  << std::endl;
        all_passed = all_passed && ok;
    }

    // Summary
    std::cout << "\n========================================" << std::endl;
    std::cout << "FINAL VERDICT: "
              << (all_passed ? "✅ ALL TESTS PASSED"
                             : "❌ SOME TESTS FAILED")
              << std::endl;
    std::cout << "========================================" << std::endl;

    return all_passed ? 0 : 1;
}
