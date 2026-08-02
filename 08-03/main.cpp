/**
 * Rotating Calipers - Minimum Area Bounding Box
 *
 * Algorithm:
 *   1. Compute convex hull (Monotone Chain, O(n log n))
 *   2. Use rotating calipers to find the minimum-area oriented bounding box (O(n))
 *   3. Visualize in PPM format with quantitative verification
 *
 * Theory:
 *   - For any convex polygon, the minimum-area bounding box has at least one
 *     edge flush with an edge of the polygon.
 *   - Rotating calipers iterates through all such candidate orientations by
 *     testing each edge as a bounding box side.
 */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}

    Point operator-(const Point& o) const { return Point(x - o.x, y - o.y); }
    Point operator+(const Point& o) const { return Point(x + o.x, y + o.y); }
    Point operator*(double s) const { return Point(x * s, y * s); }
    double cross(const Point& o) const { return x * o.y - y * o.x; }
    double dot(const Point& o) const { return x * o.x + y * o.y; }
    double len2() const { return x * x + y * y; }
    double len() const { return std::sqrt(len2()); }
    Point normalized() const {
        double l = len();
        if (l < 1e-12) return Point(1,0);
        return Point(x/l, y/l);
    }
};

// --- Convex Hull: Monotone Chain ---
std::vector<Point> convexHull(std::vector<Point> pts) {
    if (pts.size() <= 1) return pts;
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    std::vector<Point> hull;
    for (const auto& p : pts) {
        while (hull.size() >= 2) {
            Point a = hull[hull.size() - 2];
            Point b = hull.back();
            if ((b - a).cross(p - a) <= 0)
                hull.pop_back();
            else
                break;
        }
        hull.push_back(p);
    }
    size_t lower = hull.size();
    for (int i = (int)pts.size() - 1; i >= 0; --i) {
        const auto& p = pts[i];
        while (hull.size() > lower) {
            Point a = hull[hull.size() - 2];
            Point b = hull.back();
            if ((b - a).cross(p - a) <= 0)
                hull.pop_back();
            else
                break;
        }
        hull.push_back(p);
    }
    hull.pop_back(); // remove duplicate start point
    return hull;
}

// --- Rotating Calipers: Minimum Area Bounding Box ---
struct BoundingBox {
    Point center;
    Point u, v;   // orthonormal axes: u = edge direction, v = perpendicular
    double w, h;  // width (along u), height (along v)
    double area;
};

/**
 * Compute the minimum-area bounding box of a convex polygon using rotating calipers.
 *
 * For each edge of the convex hull:
 *   - Align one axis (u) to the edge direction
 *   - Find extreme projections along u and v
 *   - Compute the bounding box area for this orientation
 */
BoundingBox minAreaBoundingBox(const std::vector<Point>& hull) {
    size_t n = hull.size();
    if (n < 2) {
        BoundingBox bb;
        bb.center = hull.empty() ? Point() : hull[0];
        bb.u = Point(1,0); bb.v = Point(0,1);
        bb.w = bb.h = bb.area = 0;
        return bb;
    }

    double minArea = std::numeric_limits<double>::max();
    BoundingBox best;
    best.area = minArea;

    // For each hull edge, compute the bounding box aligned to that edge
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        Point edge = hull[j] - hull[i];
        double edgeLen = edge.len();
        if (edgeLen < 1e-12) continue;

        // Local coordinate axes: u along edge, v perpendicular (CCW 90 deg)
        Point u = Point(edge.x / edgeLen, edge.y / edgeLen);
        Point v = Point(-u.y, u.x);

        // Project all hull vertices onto u and v
        double umin = std::numeric_limits<double>::max();
        double umax = -std::numeric_limits<double>::max();
        double vmin = std::numeric_limits<double>::max();
        double vmax = -std::numeric_limits<double>::max();

        for (const auto& p : hull) {
            double up = p.dot(u);
            double vp = p.dot(v);
            umin = std::min(umin, up);
            umax = std::max(umax, up);
            vmin = std::min(vmin, vp);
            vmax = std::max(vmax, vp);
        }

        double w = umax - umin;
        double h = vmax - vmin;
        double area = w * h;

        if (area < minArea) {
            minArea = area;

            // Compute box center in world coordinates
            double cu = (umin + umax) * 0.5;
            double cv = (vmin + vmax) * 0.5;
            Point center(cu * u.x + cv * v.x, cu * u.y + cv * v.y);

            best.u = u;
            best.v = v;
            best.w = w;
            best.h = h;
            best.area = area;
            best.center = center;
        }
    }

    return best;
}

void getBoxCorners(const BoundingBox& bb, Point corners[4]) {
    Point hw = bb.u * (bb.w * 0.5);
    Point hh = bb.v * (bb.h * 0.5);
    corners[0] = bb.center - hw - hh;
    corners[1] = bb.center + hw - hh;
    corners[2] = bb.center + hw + hh;
    corners[3] = bb.center - hw + hh;
}

// --- PPM Image helpers ---
inline void drawPixel(std::vector<uint8_t>& img, int w, int h, int x, int y,
                      uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    size_t idx = 3 * ((h - 1 - y) * w + x);
    img[idx] = r; img[idx+1] = g; img[idx+2] = b;
}

void drawLine(std::vector<uint8_t>& img, int w, int h,
              int x0, int y0, int x1, int y1,
              uint8_t r, uint8_t g, uint8_t b) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        drawPixel(img, w, h, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void drawFilledCircle(std::vector<uint8_t>& img, int w, int h,
                      int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b) {
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            if (dx*dx + dy*dy <= radius*radius)
                drawPixel(img, w, h, cx + dx, cy + dy, r, g, b);
        }
    }
}

void writePPM(const std::string& fname, const std::vector<uint8_t>& img,
              int w, int h) {
    std::ofstream f(fname, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), img.size());
}

// --- Quantitative Verification ---
struct VerificationResult {
    bool containmentOk;
    bool areaLeAABB;
    bool areaPositive;
    bool hullLeBB;
    bool axesOrthonormal;
    double aabbArea;
    double bboxArea;
    double hullArea;
    double improvement;
    double maxOutside;
};

VerificationResult verifyBoundingBox(const std::vector<Point>& hull, const BoundingBox& bb) {
    VerificationResult vr{};
    vr.bboxArea = bb.area;

    // 1. Containment: all hull points must be inside the bounding box
    vr.containmentOk = true;
    vr.maxOutside = 0;
    for (const auto& p : hull) {
        Point local = p - bb.center;
        double uProj = local.dot(bb.u);
        double vProj = local.dot(bb.v);
        double uDist = fabs(uProj) - bb.w * 0.5;
        double vDist = fabs(vProj) - bb.h * 0.5;
        double outside = std::max(0.0, std::max(uDist, vDist));
        if (outside > 1e-8) {
            vr.containmentOk = false;
            vr.maxOutside = std::max(vr.maxOutside, outside);
        }
    }

    // 2. Area <= AABB area
    double xmin = std::numeric_limits<double>::max();
    double xmax = -std::numeric_limits<double>::max();
    double ymin = std::numeric_limits<double>::max();
    double ymax = -std::numeric_limits<double>::max();
    for (const auto& p : hull) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    vr.aabbArea = (xmax - xmin) * (ymax - ymin);
    vr.areaLeAABB = vr.bboxArea <= vr.aabbArea + 1e-6;
    vr.improvement = vr.aabbArea > 0 ? (vr.aabbArea - vr.bboxArea) / vr.aabbArea * 100.0 : 0;

    // 3. Area > 0
    vr.areaPositive = vr.bboxArea > 1e-6;

    // 4. Hull area <= bounding box area
    double hullArea = 0;
    for (size_t i = 0; i < hull.size(); ++i) {
        size_t j = (i + 1) % hull.size();
        hullArea += hull[i].cross(hull[j]);
    }
    vr.hullArea = fabs(hullArea) * 0.5;
    vr.hullLeBB = vr.hullArea <= vr.bboxArea + 1e-6;

    // 5. Axes orthonormal
    double dotUV = fabs(bb.u.dot(bb.v));
    double lenU = bb.u.len2();
    double lenV = bb.v.len2();
    vr.axesOrthonormal = (dotUV < 1e-8) && (fabs(lenU - 1.0) < 1e-8) && (fabs(lenV - 1.0) < 1e-8);

    return vr;
}

void printVerification(const char* name, const VerificationResult& vr) {
    std::cout << "=== " << name << " ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  AABB area:   " << vr.aabbArea << std::endl;
    std::cout << "  BBox area:   " << vr.bboxArea << std::endl;
    std::cout << "  Hull area:   " << vr.hullArea << std::endl;
    std::cout << "  Improvement:  " << vr.improvement << "%" << std::endl;
    std::cout << std::endl;

    bool allPass = true;
    auto check = [&](bool cond, const char* /*label*/, const char* pass, const char* fail) {
        if (cond) {
            std::cout << "  ✅ PASS: " << pass << std::endl;
        } else {
            std::cout << "  ❌ FAIL: " << fail << std::endl;
            allPass = false;
        }
    };

    check(vr.containmentOk, "Containment",
          "All hull points are inside the bounding box",
          "Some hull points are outside the bounding box");

    check(vr.areaLeAABB, "Area <= AABB",
          "Oriented bbox area is <= AABB area",
          "Oriented bbox area > AABB area (should never happen for correct algorithm)");

    check(vr.areaPositive, "Area positive",
          "Bounding box area is positive",
          "Bounding box area is zero or negative");

    check(vr.hullLeBB, "Hull <= BBox",
          "Hull area <= bounding box area",
          "Hull area > bounding box area (impossible for correct algorithm)");

    check(vr.axesOrthonormal, "Axes orthonormal",
          "BBox axes are orthonormal (|u|=1, |v|=1, u·v=0)",
          "BBox axes are not orthonormal");

    if (allPass) {
        std::cout << std::endl << "  🏆 ALL " << (vr.improvement > 5 ? "5" : "") << " verifications PASSED!" << std::endl;
    } else {
        std::cout << std::endl << "  ❌ Some verifications FAILED!" << std::endl;
    }
    std::cout << std::endl;
}

// --- Main ---
int main() {
    const int W = 800, H = 600;
    std::vector<uint8_t> img(W * H * 3, 30); // dark background

    // --- Generate the test point set ---
    // A 30-degree rotated rectangle (thin and long) for clear improvement
    srand(12345);
    std::vector<Point> pts;

    double angle = 30.0 * M_PI / 180.0; // 30 degrees
    double cosA = cos(angle);
    double sinA = sin(angle);

    // Generate points inside a rotated rectangle: width=500, height=80
    for (int i = 0; i < 200; ++i) {
        double u = (rand() % 5000) / 10.0;     // [0, 500] along local x
        double v = (rand() % 800) / 10.0 - 40; // [-40, 40] along local y
        double x = 150 + u * cosA - v * sinA;
        double y = 200 + u * sinA + v * cosA;
        pts.push_back(Point(x, y));
    }

    // Add scattered points around
    for (int i = 0; i < 60; ++i) {
        double ax = 50 + rand() % 650;
        double ay = 50 + rand() % 450;
        pts.push_back(Point(ax, ay));
    }

    // --- Compute ---
    auto hull = convexHull(pts);
    auto bb   = minAreaBoundingBox(hull);
    auto vr   = verifyBoundingBox(hull, bb);

    printVerification("Rotating Calipers Min BBox", vr);

    // --- Visualization ---
    // Coordinate mapping: points are roughly in [0, 700] x [0, 550]
    auto toScreen = [](double x, double y) -> std::pair<int,int> {
        int sx = (int)(20 + x * (760.0 / 700.0));
        int sy = (int)(20 + y * (560.0 / 550.0));
        return {sx, sy};
    };

    // Grid
    for (int x = 0; x < W; x += 50)
        for (int y = 0; y < H; ++y) drawPixel(img, W, H, x, y, 50, 50, 50);
    for (int y = 0; y < H; y += 50)
        for (int x = 0; x < W; ++x) drawPixel(img, W, H, x, y, 50, 50, 50);

    // Input points (gray dots)
    for (const auto& p : pts) {
        auto [sx, sy] = toScreen(p.x, p.y);
        drawPixel(img, W, H, sx, sy, 100, 100, 100);
    }

    // Convex hull (blue outline)
    for (size_t i = 0; i < hull.size(); ++i) {
        size_t j = (i + 1) % hull.size();
        auto [x0, y0] = toScreen(hull[i].x, hull[i].y);
        auto [x1, y1] = toScreen(hull[j].x, hull[j].y);
        drawLine(img, W, H, x0, y0, x1, y1, 80, 140, 255);
    }

    // Hull vertices (red dots)
    for (const auto& p : hull) {
        auto [sx, sy] = toScreen(p.x, p.y);
        drawFilledCircle(img, W, H, sx, sy, 3, 255, 50, 50);
    }

    // Minimum bounding box (green)
    Point corners[4];
    getBoxCorners(bb, corners);
    std::pair<int,int> sc[4];
    for (int i = 0; i < 4; ++i) sc[i] = toScreen(corners[i].x, corners[i].y);
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        drawLine(img, W, H, sc[i].first, sc[i].second, sc[j].first, sc[j].second, 0, 220, 60);
    }

    // BBox center (green dot)
    auto [csx, csy] = toScreen(bb.center.x, bb.center.y);
    drawFilledCircle(img, W, H, csx, csy, 4, 0, 180, 0);

    writePPM("output.ppm", img, W, H);
    std::cout << "Visualization saved to output.ppm (" << W << "x" << H << ")" << std::endl;

    // --- Additional verification: generate another random test for robustness ---
    std::cout << "\n=== Additional Random Test ===\n";
    srand(99999);
    std::vector<Point> pts2;
    for (int i = 0; i < 150; ++i) {
        double angle = (rand() % 6283) / 1000.0; // 0 to 2*pi
        double r = 100 + (rand() % 300);
        pts2.push_back(Point(350 + r * cos(angle), 300 + r * sin(angle)));
    }
    auto hull2 = convexHull(pts2);
    auto bb2   = minAreaBoundingBox(hull2);
    auto vr2   = verifyBoundingBox(hull2, bb2);
    printVerification("Random scatter", vr2);

    // Verify the output image with quantitative pixel check
    std::cout << "=== Image Verification ===" << std::endl;
    double imgMean = 0, imgVar = 0;
    for (size_t i = 0; i < img.size(); ++i) imgMean += img[i];
    imgMean /= img.size();
    for (size_t i = 0; i < img.size(); ++i) {
        double d = img[i] - imgMean;
        imgVar += d * d;
    }
    imgVar /= img.size();
    double imgStd = std::sqrt(imgVar);
    std::cout << "  Pixel mean: " << imgMean << std::endl;
    std::cout << "  Pixel std:  " << imgStd << std::endl;
    if (imgMean < 5) {
        std::cout << "  ❌ FAIL: Image too dark" << std::endl;
    } else if (imgMean > 250) {
        std::cout << "  ❌ FAIL: Image too bright" << std::endl;
    } else if (imgStd < 5) {
        std::cout << "  ❌ FAIL: Image has no variation" << std::endl;
    } else {
        std::cout << "  ✅ PASS: Image pixel statistics normal" << std::endl;
    }

    // File size check
    std::ifstream f("output.ppm", std::ios::binary | std::ios::ate);
    std::streamsize fsize = f.tellg();
    std::cout << "  File size: " << fsize << " bytes" << std::endl;
    if (fsize > 10240) {
        std::cout << "  ✅ PASS: File size > 10KB" << std::endl;
    } else {
        std::cout << "  ❌ FAIL: File too small" << std::endl;
    }

    return 0;
}
