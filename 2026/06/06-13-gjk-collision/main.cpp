/// GJK Collision Detection Visualizer
/// Implements the Gilbert-Johnson-Keerthi algorithm for convex polygon collision,
/// plus EPA (Expanding Polytope Algorithm) for penetration depth.
/// Renders random convex polygons and visualizes collision states.
///
/// Reference: https://blog.hamaluik.ca/posts/building-a-collision-engine-part-1-2d-gjk-collision-detection/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

// ===== Math types =====
struct Vec2 {
    double x, y;
    Vec2() : x(0), y(0) {}
    Vec2(double x_, double y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    double cross(const Vec2& o) const { return x * o.y - y * o.x; }
    double len2() const { return x*x + y*y; }
    double len() const { return std::sqrt(len2()); }
    Vec2 normalized() const { double l = len(); return l > 1e-12 ? (*this)/l : Vec2(); }
    Vec2 perp() const { return {-y, x}; }
};

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
};

// ===== Convex polygon =====
using Polygon = std::vector<Vec2>;

// ===== GJK types =====
struct Simplex {
    Vec2 a, b, c;
    int count;        // 1-3
    Vec2 A[3], B[3];  // original points on A and B for EPA
};

// ===== Utility =====
Vec2 support(const Polygon& A, const Polygon& B, Vec2 dir) {
    // Support point of Minkowski difference A-B in direction dir
    auto furthest = [](const Polygon& p, Vec2 d) -> Vec2 {
        double best = -std::numeric_limits<double>::max();
        Vec2 bestPt;
        for (const auto& v : p) {
            double dp = d.dot(v);
            if (dp > best) { best = dp; bestPt = v; }
        }
        return bestPt;
    };
    return furthest(A, dir) - furthest(B, dir * -1.0);
}

// ===== GJK core =====
// Return true if origin (0,0) is in simplex.
// Updates simplex with closest feature.
bool handleSimplex(Simplex& s, Vec2& dir) {
    if (s.count == 2) {
        // Line case: find region closest to origin
        Vec2 a = s.a;
        Vec2 b = s.b;
        Vec2 ab = b - a;
        Vec2 ao = a * -1.0;

        if (ab.dot(ao) > 0) {
            // Origin is on AB side of A: project onto AB
            dir = ab.perp().cross(ab) < 0 ? ab.perp() : ab.perp()*-1;
            // More standard: use triple cross product
            dir = Vec2(ab.x * ao.x + ab.y * ao.y, ab.x * ao.y - ab.y * ao.x);
            dir = Vec2(dir.y * ab.x - dir.x * ab.y, -dir.y * ab.y - dir.x * ab.x);
            // Actually use standard approach: n x AB x AB, where n is perpendicular
            Vec2 abPerp = {-ab.y, ab.x};
            double crossVal = ab.cross(ao);
            dir = abPerp * (crossVal < 0 ? -1.0 : 1.0);
        } else {
            // Closest to A
            s.count = 1;
            s.a = a;
            s.A[0] = s.A[0]; // keep A[0]
            s.B[0] = s.B[0]; // keep B[0]
            dir = ao;
        }
        return false;
    }

    if (s.count == 3) {
        Vec2 a = s.a;
        Vec2 b = s.b;
        Vec2 c = s.c;
        Vec2 ao = a * -1.0;

        Vec2 ab = b - a;
        Vec2 ac = c - a;

        // Check regions
        double abPerpA  = ab.dot(ao);      // alt: abPerp sign
        double acPerpA  = ac.dot(ao);

        // Use barycentric-ish edge checks
        if (abPerpA <= 0 && acPerpA <= 0) {
            // Closest to A
            s.count = 1;
            s.a = a;
            s.A[0] = s.A[0]; s.B[0] = s.B[0];
            dir = ao;
        } else {
            Vec2 bo = b * -1.0;
            Vec2 ba = a - b;
            Vec2 bc = c - b;
            double baPerpB = ba.dot(bo);
            double bcPerpB = bc.dot(bo);
            if (baPerpB >= 0 && bcPerpB <= 0) {
                // Closest to B
                s.count = 1;
                s.a = b;
                s.A[0] = s.A[1]; s.B[0] = s.B[1];
                dir = bo;
            } else {
                Vec2 co = c * -1.0;
                Vec2 ca = a - c;
                Vec2 cb = b - c;
                double caPerpC = ca.dot(co);
                double cbPerpC = cb.dot(co);
                if (caPerpC >= 0 && cbPerpC <= 0) {
                    // Closest to C
                    s.count = 1;
                    s.a = c;
                    s.A[0] = s.A[2]; s.B[0] = s.B[2];
                    dir = co;
                } else {
                    // Closest to triangle interior — origin inside!
                    return true;
                }
            }
        }
        // Try edge closest: check AB edge
        Vec2 abPerp = {-ab.y, ab.x};
        if (abPerp.dot(ao) > 0) {
            s.b = a; s.c = b;
            s.A[1] = s.A[0]; s.B[1] = s.B[0];
            s.count = 2;
            dir = Vec2(ao.x * ab.y - ao.y * ab.x, ab.x * ao.y - ab.y * ao.x);
            // Standard: (AB x AO) x AB
            double crossVal = ab.cross(ao);
            dir = Vec2{-ab.y * crossVal, ab.x * crossVal};
            return false;
        }
        // Check AC edge
        Vec2 acPerp = {-ac.y, ac.x};
        if (acPerp.dot(ao) < 0) {
            s.b = c; s.c = a;
            s.A[0] = s.A[2]; s.B[0] = s.B[2];
            s.A[1] = s.A[1]; s.B[1] = s.B[1];
            s.count = 2;
            double crossVal = ac.cross(ao);
            dir = Vec2{-ac.y * crossVal, ac.x * crossVal};
            return false;
        }

        return true; // origin inside triangle
    }

    return false;
}

bool gjk(const Polygon& A, const Polygon& B) {
    // Initial direction: from centroid A to centroid B
    Vec2 ca(0,0), cb(0,0);
    for (auto& v : A) ca = ca + v;
    for (auto& v : B) cb = cb + v;
    ca = ca / (int)A.size();
    cb = cb / (int)B.size();
    Vec2 dir = (cb - ca).len() > 1e-6 ? (cb - ca) : Vec2(1,0);

    Simplex s;
    s.a = support(A, B, dir);
    s.A[0] = s.a + support(A, B, Vec2(0,0)); // dummy — fixed below
    s.count = 1;

    dir = s.a * -1.0;

    for (int iter = 0; iter < 64; iter++) {
        Vec2 p = support(A, B, dir);
        if (p.dot(dir) < 0) return false;  // no intersection possible

        if (s.count == 1) {
            s.b = s.a;
            s.A[1] = s.A[0]; s.B[1] = s.B[0];
            s.a = p;
            s.A[0] = p + Vec2(0,0); // we need original A,B points
            s.count = 2;
        } else if (s.count == 2) {
            s.c = s.b;
            s.A[2] = s.A[1]; s.B[2] = s.B[1];
            s.b = s.a;
            s.A[1] = s.A[0]; s.B[1] = s.B[0];
            s.a = p;
            s.A[0] = p + Vec2(0,0);
            s.count = 3;
        }

        if (handleSimplex(s, dir)) return true;
    }
    return false;
}

// ===== Rewritten GJK with correct support point tracking =====
// The original support function returns a point in Minkowski difference space.
// We need to track which points on A and B produced it for EPA.

struct SupportPoint {
    Vec2 mink;  // point in A-B
    Vec2 a;     // point on A
    Vec2 b;     // point on B
};

SupportPoint support2(const Polygon& A, const Polygon& B, Vec2 dir) {
    auto furthest = [](const Polygon& p, Vec2 d) -> Vec2 {
        double best = -std::numeric_limits<double>::max();
        Vec2 bestPt;
        for (const auto& v : p) {
            double dp = d.dot(v);
            if (dp > best) { best = dp; bestPt = v; }
        }
        return bestPt;
    };
    Vec2 pa = furthest(A, dir);
    Vec2 pb = furthest(B, dir * -1.0);
    return {pa - pb, pa, pb};
}

// Proper GJK with EPA tracking
struct Simplex2 {
    std::vector<SupportPoint> pts;
};

Vec2 tripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {
    // (a x b) x c = b*(a·c) - a*(b·c)
    double dotAC = a.dot(c);
    double dotBC = b.dot(c);
    return {b.x * dotAC - a.x * dotBC, b.y * dotAC - a.y * dotBC};
}

bool nearestSimplex(Simplex2& s, Vec2& dir) {
    if (s.pts.size() == 2) {
        Vec2 a = s.pts[1].mink;
        Vec2 b = s.pts[0].mink;
        Vec2 ao = a * -1.0;
        Vec2 ab = b - a;

        if (ab.dot(ao) > 0) {
            // Origin projected onto AB
            dir = tripleProduct(ab, ao, ab);
        } else {
            // Closest to A
            s.pts.erase(s.pts.begin()); // keep a
            dir = ao;
        }
        return false;
    }

    if (s.pts.size() == 3) {
        Vec2 a = s.pts[2].mink;
        Vec2 b = s.pts[1].mink;
        Vec2 c = s.pts[0].mink;
        Vec2 ao = a * -1.0;
        Vec2 ab = b - a;
        Vec2 ac = c - a;

        // Check if origin passes Voronoi region of A
        if (ab.dot(ao) <= 0 && ac.dot(ao) <= 0) {
            s.pts = {s.pts[2]}; // keep A
            dir = ao;
            return false;
        }

        Vec2 bo = b * -1.0;
        Vec2 ba = a - b;
        Vec2 bc = c - b;
        if (ba.dot(bo) <= 0 && bc.dot(bo) <= 0) {
            s.pts = {s.pts[1]}; // keep B
            dir = bo;
            return false;
        }

        Vec2 co = c * -1.0;
        Vec2 ca = a - c;
        Vec2 cb = b - c;
        if (ca.dot(co) <= 0 && cb.dot(co) <= 0) {
            s.pts = {s.pts[0]}; // keep C
            dir = co;
            return false;
        }

        // Check AB edge
        Vec2 abPerp = {-ab.y, ab.x};
        if (abPerp.dot(ao) > 0 && ab.dot(ao) > 0) {
            s.pts = {s.pts[1], s.pts[2]}; // keep B, A
            dir = tripleProduct(ab, ao, ab);
            return false;
        }

        // Check AC edge
        Vec2 acPerp = {-ac.y, ac.x};
        if (acPerp.dot(ao) < 0 && ac.dot(ao) > 0) {
            s.pts = {s.pts[0], s.pts[2]}; // keep C, A
            dir = tripleProduct(ac, ao, ac);
            return false;
        }

        // Origin inside triangle — collision!
        return true;
    }

    // Single point
    dir = s.pts[0].mink * -1.0;
    return false;
}

bool gjk2(const Polygon& A, const Polygon& B, std::vector<SupportPoint>* outSimplex = nullptr) {
    Vec2 ca(0,0), cb(0,0);
    for (auto& v : A) ca = ca + v;
    for (auto& v : B) cb = cb + v;
    ca = ca / (int)A.size();
    cb = cb / (int)B.size();
    Vec2 dir = (cb - ca).len() > 1e-6 ? (cb - ca) : Vec2(1,0);

    Simplex2 s;
    s.pts.push_back(support2(A, B, dir));
    dir = s.pts[0].mink * -1.0;

    for (int iter = 0; iter < 64; iter++) {
        auto sp = support2(A, B, dir);
        if (sp.mink.dot(dir) <= 0) return false;

        s.pts.push_back(sp);
        if (nearestSimplex(s, dir)) {
            if (outSimplex) {
                // Ensure CCW winding for EPA
                if (s.pts.size() == 3) {
                    Vec2 ab = s.pts[1].mink - s.pts[0].mink;
                    Vec2 ac = s.pts[2].mink - s.pts[0].mink;
                    if (ab.cross(ac) < 0) {
                        std::swap(s.pts[1], s.pts[2]);
                    }
                }
                *outSimplex = s.pts;
            }
            return true;
        }
    }
    return false;
}

// ===== EPA for penetration depth =====
// Builds convex hull of Minkowski difference points, then finds closest feature to origin
struct EPAResult {
    double depth = 0;
    Vec2 normal;
    Vec2 contactA, contactB;
    bool valid = false;
};

// Simple monotone chain for convex hull of Minkowski points
std::vector<SupportPoint> convexHull(std::vector<SupportPoint> pts) {
    if (pts.size() <= 2) return pts;
    std::sort(pts.begin(), pts.end(), [](const SupportPoint& a, const SupportPoint& b) {
        return a.mink.x < b.mink.x || (a.mink.x == b.mink.x && a.mink.y < b.mink.y);
    });
    auto cross = [](Vec2 o, Vec2 a, Vec2 b) -> double {
        return (a.x-o.x)*(b.y-o.y) - (a.y-o.y)*(b.x-o.x);
    };
    std::vector<SupportPoint> hull;
    for (int phase = 0; phase < 2; phase++) {
        size_t start = hull.size();
        for (auto& p : pts) {
            while (hull.size() >= start + 2 &&
                   cross(hull[hull.size()-2].mink, hull.back().mink, p.mink) <= 0)
                hull.pop_back();
            hull.push_back(p);
        }
        hull.pop_back(); // remove last (duplicate with start of next phase)
        std::reverse(pts.begin(), pts.end());
    }
    return hull;
}

EPAResult epa(const Polygon& A, const Polygon& B, const std::vector<SupportPoint>& simplex) {
    EPAResult result;
    if (simplex.size() < 3) return result;

    std::vector<SupportPoint> polytope = simplex;

    for (int iter = 0; iter < 32; iter++) {
        int n = (int)polytope.size();

        // Find closest edge to origin
        double minDist = std::numeric_limits<double>::max();
        int bestIdx = 0;
        Vec2 bestNormal;

        for (int i = 0; i < n; i++) {
            Vec2 a = polytope[i].mink;
            Vec2 b = polytope[(i+1)%n].mink;
            Vec2 ab = b - a;

            // Outward normal for CCW polygon = rotate ab 90° clockwise
            Vec2 outward = {ab.y, -ab.x};
            double len = outward.len();
            if (len < 1e-12) continue;
            Vec2 normal = outward / len;

            double dist = normal.dot(a);
            if (dist <= 0 || dist >= minDist) continue;

            minDist = dist;
            bestIdx = i;
            bestNormal = normal;
        }

        if (minDist >= 1e9) {
            // No valid edge found — polytope may have wrong winding
            return result;
        }

        // Support in normal direction
        auto sp = support2(A, B, bestNormal);
        double dp = sp.mink.dot(bestNormal);
        double error = dp - minDist;
        if (error < 1e-3) {
            result.depth = minDist;
            result.normal = bestNormal;
            int i1 = bestIdx, i2 = (bestIdx+1) % n;
            double t = 0.5;
            result.contactA = (polytope[i1].a + polytope[i2].a) * t;
            result.contactB = (polytope[i1].b + polytope[i2].b) * t;
            result.valid = true;
            return result;
        }

        // Add new support point and rebuild convex hull
        polytope.push_back(sp);
        polytope = convexHull(polytope);
        if (polytope.size() < 3) return result;
    }

    return result;
}

// ===== Polygon generation =====
Polygon randomConvexPolygon(Vec2 center, double radius, int n) {
    // Generate by random angles and sort
    std::vector<double> angles;
    for (int i = 0; i < n; i++) {
        angles.push_back((double)rand()/RAND_MAX * 2 * M_PI);
    }
    std::sort(angles.begin(), angles.end());

    Polygon poly;
    for (int i = 0; i < n; i++) {
        double r = radius * (0.6 + 0.4 * (double)rand()/RAND_MAX);
        poly.push_back({center.x + r * cos(angles[i]), center.y + r * sin(angles[i])});
    }
    return poly;
}

// ===== PPM output =====
void drawCircle(std::vector<Vec3>& img, int w, int h, Vec2 c, double r, Vec3 color) {
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx*dx + dy*dy > r*r) continue;
            int px = (int)(c.x + dx);
            int py = (int)(c.y + dy);
            if (px >= 0 && px < w && py >= 0 && py < h)
                img[py*w+px] = color;
        }
    }
}

void drawLine(std::vector<Vec3>& img, int w, int h, Vec2 a, Vec2 b, Vec3 color) {
    int x0 = (int)a.x, y0 = (int)a.y, x1 = (int)b.x, y1 = (int)b.y;
    int dx = abs(x1-x0), dy = -abs(y1-y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) img[y0*w+x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2*err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void drawPolygon(std::vector<Vec3>& img, int w, int h, const Polygon& p, Vec3 color) {
    for (int i = 0; i < (int)p.size(); i++) {
        drawLine(img, w, h, p[i], p[(i+1)%p.size()], color);
    }
}

void fillPolygon(std::vector<Vec3>& img, int w, int h, const Polygon& p, Vec3 color) {
    // Simple scanline fill
    if (p.size() < 3) return;
    double minY = 1e9, maxY = -1e9;
    for (auto& v : p) {
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }
    for (int y = std::max(0, (int)minY); y <= std::min(h-1, (int)maxY); y++) {
        std::vector<double> xs;
        for (int i = 0; i < (int)p.size(); i++) {
            Vec2 a = p[i], b = p[(i+1)%p.size()];
            if ((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) {
                double t = (y - a.y) / (b.y - a.y);
                xs.push_back(a.x + t * (b.x - a.x));
            }
        }
        std::sort(xs.begin(), xs.end());
        for (int i = 0; i+1 < (int)xs.size(); i += 2) {
            int xi = std::max(0, (int)xs[i]);
            int xe = std::min(w-1, (int)xs[i+1]);
            for (int x = xi; x <= xe; x++) img[y*w+x] = color;
        }
    }
}

// ===== Main =====
int main() {
    srand(time(0));

    const int W = 1200, H = 800;
    const int cols = 4, rows = 3;
    const int cellW = W / cols, cellH = H / rows;

    std::vector<Vec3> image(W*H, Vec3(20,20,30)); // dark background

    // Statistics for validation
    int totalPairs = 0;
    int collisionCount = 0;
    std::vector<double> allDepths;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            int cx = col * cellW + cellW/2;
            int cy = row * cellH + cellH/2;
            double cellR = std::min(cellW, cellH) * 0.38;

            // Generate two random convex polygons that may or may not overlap
            double offsetX = ((double)rand()/RAND_MAX * 2 - 1) * cellR * 0.8;
            double offsetY = ((double)rand()/RAND_MAX * 2 - 1) * cellR * 0.8;
            int n1 = 4 + rand() % 4; // 4-7 vertices
            int n2 = 4 + rand() % 4;

            Polygon A = randomConvexPolygon({double(cx) - offsetX*0.5, double(cy) - offsetY*0.5}, cellR*0.5, n1);
            Polygon B = randomConvexPolygon({double(cx) + offsetX*0.5, double(cy) + offsetY*0.5}, cellR*0.5, n2);

            totalPairs++;

            std::vector<SupportPoint> simplex;
            bool collision = gjk2(A, B, &simplex);

            Vec3 colorA, colorB;
            if (collision) {
                collisionCount++;
                colorA = Vec3(220, 60, 60);    // red
                colorB = Vec3(220, 100, 60);   // orange

                // Run EPA for penetration depth
                EPAResult epaResult = epa(A, B, simplex);
                if (epaResult.valid) {
                    allDepths.push_back(epaResult.depth);
                    // Draw contact normal
                    drawLine(image, W, H, epaResult.contactA, epaResult.contactA + epaResult.normal * epaResult.depth,
                             Vec3(255, 255, 0));
                    // Draw contact points
                    drawCircle(image, W, H, epaResult.contactA, 4, Vec3(0, 255, 0));
                    drawCircle(image, W, H, epaResult.contactB, 4, Vec3(0, 255, 0));
                }
            } else {
                colorA = Vec3(60, 180, 60);    // green
                colorB = Vec3(60, 100, 220);   // blue
            }

            // Fill polygons
            double alpha = collision ? 0.25 : 0.22;
            fillPolygon(image, W, H, A, colorA * alpha);
            fillPolygon(image, W, H, B, colorB * alpha);

            // Draw polygon outlines
            drawPolygon(image, W, H, A, colorA);
            drawPolygon(image, W, H, B, colorB);

            // Draw cell border
            drawLine(image, W, H, {(double)(col*cellW), (double)(row*cellH)},
                     {(double)(col*cellW), (double)((row+1)*cellH)}, Vec3(80,80,80));
            drawLine(image, W, H, {(double)(col*cellW), (double)(row*cellH)},
                     {(double)((col+1)*cellW), (double)(row*cellH)}, Vec3(80,80,80));

            // Draw a small matching label
            char label[4];
            snprintf(label, 4, "%c%c", (char)(row*cols+col)*2 % 26 + 'A',
                     (char)(row*cols+col)*2 % 26 + 'a');
        }
    }

    // ===== Quantified validation =====
    std::cout << "=== GJK Collision Detection Results ===" << std::endl;
    std::cout << "Total polygon pairs: " << totalPairs << std::endl;
    std::cout << "Collision pairs: " << collisionCount << std::endl;
    std::cout << "Non-collision pairs: " << (totalPairs - collisionCount) << std::endl;

    std::cout << "\n--- Penetration Depths (EPA) ---" << std::endl;
    std::cout << "Depths: ";
    for (double d : allDepths) {
        std::cout << d << " ";
    }
    std::cout << std::endl;

    // Compute stats
    if (!allDepths.empty()) {
        double sum = 0, minD = 1e9, maxD = 0;
        for (double d : allDepths) {
            sum += d;
            minD = std::min(minD, d);
            maxD = std::max(maxD, d);
        }
        double mean = sum / allDepths.size();
        double var = 0;
        for (double d : allDepths) var += (d - mean)*(d - mean);
        var /= allDepths.size();
        double stdDev = std::sqrt(var);
        std::cout << "Mean depth: " << mean << std::endl;
        std::cout << "Std dev: " << stdDev << std::endl;
        std::cout << "Range: [" << minD << ", " << maxD << "]" << std::endl;
    }

    // Save PPM
    std::ofstream f("gjk_output.ppm", std::ios::binary);
    f << "P6\n" << W << " " << H << "\n255\n";
    for (auto& p : image) {
        unsigned char r = (unsigned char)std::max(0.0, std::min(255.0, p.x));
        unsigned char g = (unsigned char)std::max(0.0, std::min(255.0, p.y));
        unsigned char b = (unsigned char)std::max(0.0, std::min(255.0, p.z));
        f << r << g << b;
    }
    f.close();

    // Image statistics (using the image we just wrote)
    std::cout << "\n--- Image Pixel Statistics ---" << std::endl;
    double pixSum = 0, pixMax = 0, pixMin = 255;
    int pixCount = W*H;
    for (auto& p : image) {
        double val = (p.x + p.y + p.z) / 3.0;
        pixSum += val;
        pixMax = std::max(pixMax, val);
        pixMin = std::min(pixMin, val);
    }
    double pixMean = pixSum / pixCount;
    double pixVar = 0;
    for (auto& p : image) {
        double val = (p.x + p.y + p.z) / 3.0;
        pixVar += (val - pixMean)*(val - pixMean);
    }
    pixVar /= pixCount;
    double pixStd = std::sqrt(pixVar);
    std::cout << "Pixel mean: " << pixMean << std::endl;
    std::cout << "Pixel std dev: " << pixStd << std::endl;
    std::cout << "Pixel range: [" << pixMin << ", " << pixMax << "]" << std::endl;

    // Validation assertions
    bool pass = true;

    // Must have both collision and non-collision cases
    if (collisionCount == 0) {
        std::cout << "❌ FAIL: No collisions detected — unrealistic for 12 random polygon pairs" << std::endl;
        pass = false;
    }
    if (totalPairs - collisionCount == 0) {
        std::cout << "❌ FAIL: All pairs in collision — unrealistic" << std::endl;
        pass = false;
    }
    if (collisionCount > 0 && allDepths.empty()) {
        std::cout << "❌ FAIL: EPA should produce depths for all collision pairs" << std::endl;
        pass = false;
    }

    // Pixel validation
    if (pixMean < 10) {
        std::cout << "❌ FAIL: Image too dark (" << pixMean << ")" << std::endl;
        pass = false;
    }
    if (pixMean > 245) {
        std::cout << "❌ FAIL: Image too bright (" << pixMean << ")" << std::endl;
        pass = false;
    }
    if (pixStd < 5) {
        std::cout << "❌ FAIL: Image has no content variation (" << pixStd << ")" << std::endl;
        pass = false;
    }

    if (allDepths.size() >= 2) {
        double minDv = 1e9, maxDv = 0;
        for (double d : allDepths) { minDv = std::min(minDv, d); maxDv = std::max(maxDv, d); }
        if (minDv == maxDv) {
            std::cout << "❌ FAIL: All penetration depths identical — suspicious" << std::endl;
            pass = false;
        }
    }

    if (pass) {
        std::cout << "\n✅ ALL VALIDATION CHECKS PASSED" << std::endl;
    } else {
        std::cout << "\n❌ SOME VALIDATION CHECKS FAILED" << std::endl;
    }

    return pass ? 0 : 1;
}
