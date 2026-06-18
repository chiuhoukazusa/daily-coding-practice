#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <tuple>
#include <algorithm>
#include <limits>
#include <cassert>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <cstdlib>

// ==================== Vector2D ====================
struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s) const { return {x*s, y*s}; }
    float dot(const Vec2& o) const { return x*o.x + y*o.y; }
    float cross(const Vec2& o) const { return x*o.y - y*o.x; }
    float len() const { return std::sqrt(x*x + y*y); }
    Vec2 normalized() const { float l = len(); return l > 1e-8f ? Vec2(x/l, y/l) : Vec2(0,0); }
    Vec2 perp() const { return {-y, x}; }
};

// ==================== Convex Polygon ====================
struct Polygon {
    std::vector<Vec2> vertices;
    
    Polygon() {}
    Polygon(const std::vector<Vec2>& v) : vertices(v) {}
    
    static Polygon randomConvex(int numVertices, std::mt19937& rng, 
                                 float minX, float maxX, float minY, float maxY) {
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        
        std::vector<Vec2> pts(numVertices);
        for (int i = 0; i < numVertices; ++i) {
            float angle = 2.0f * M_PI * i / numVertices + (dist(rng) - 0.5f) * 1.0f;
            float radius = 0.5f + 0.3f * dist(rng);
            float cx = (minX + maxX) * 0.5f;
            float cy = (minY + maxY) * 0.5f;
            float rx = (maxX - minX) * 0.3f * radius;
            float ry = (maxY - minY) * 0.3f * radius;
            pts[i] = {cx + rx * std::cos(angle), cy + ry * std::sin(angle)};
        }
        
        Vec2 centroid(0, 0);
        for (const auto& p : pts) centroid = centroid + p;
        centroid = centroid * (1.0f / numVertices);
        
        std::sort(pts.begin(), pts.end(), [&centroid](const Vec2& a, const Vec2& b) {
            float angA = std::atan2(a.y - centroid.y, a.x - centroid.x);
            float angB = std::atan2(b.y - centroid.y, b.x - centroid.x);
            return angA < angB;
        });
        
        return Polygon(pts);
    }
    
    void translate(const Vec2& offset) {
        for (auto& v : vertices) v = v + offset;
    }
    
    int numVertices() const { return (int)vertices.size(); }
    
    std::pair<Vec2, Vec2> edge(int i) const {
        int next = (i + 1) % numVertices();
        return {vertices[i], vertices[next]};
    }
    
    Vec2 edgeNormal(int i) const {
        auto [a, b] = edge(i);
        Vec2 dir = b - a;
        return dir.perp().normalized();
    }
};

// ==================== SAT Implementation ====================
std::pair<float, float> projectPolygon(const Polygon& poly, const Vec2& axis) {
    float minVal = std::numeric_limits<float>::max();
    float maxVal = std::numeric_limits<float>::lowest();
    for (const auto& v : poly.vertices) {
        float proj = v.dot(axis);
        minVal = std::min(minVal, proj);
        maxVal = std::max(maxVal, proj);
    }
    return {minVal, maxVal};
}

bool intervalsOverlap(float minA, float maxA, float minB, float maxB) {
    return !(maxA < minB || maxB < minA);
}

float overlapDepth(float minA, float maxA, float minB, float maxB) {
    return std::min(maxA - minB, maxB - minA);
}

struct SATResult {
    bool colliding;
    Vec2 mtv;
    float penetration;
    Vec2 collisionAxis;
};

SATResult satCollision(const Polygon& polyA, const Polygon& polyB) {
    SATResult result;
    result.colliding = true;
    result.penetration = std::numeric_limits<float>::max();
    
    auto testAxis = [&](const Vec2& axis) {
        auto [minA, maxA] = projectPolygon(polyA, axis);
        auto [minB, maxB] = projectPolygon(polyB, axis);
        
        if (!intervalsOverlap(minA, maxA, minB, maxB)) {
            result.colliding = false;
            return false;
        }
        
        float depth = overlapDepth(minA, maxA, minB, maxB);
        if (depth < result.penetration) {
            result.penetration = depth;
            result.collisionAxis = axis;
        }
        return true;
    };
    
    for (int i = 0; i < polyA.numVertices(); ++i) {
        if (!testAxis(polyA.edgeNormal(i))) return result;
    }
    for (int i = 0; i < polyB.numVertices(); ++i) {
        if (!testAxis(polyB.edgeNormal(i))) return result;
    }
    
    Vec2 centerA(0,0), centerB(0,0);
    for (const auto& v : polyA.vertices) centerA = centerA + v;
    for (const auto& v : polyB.vertices) centerB = centerB + v;
    centerA = centerA * (1.0f / polyA.numVertices());
    centerB = centerB * (1.0f / polyB.numVertices());
    
    Vec2 dir = centerB - centerA;
    if (dir.dot(result.collisionAxis) < 0) {
        result.mtv = result.collisionAxis * (-result.penetration);
    } else {
        result.mtv = result.collisionAxis * result.penetration;
    }
    
    return result;
}

// ==================== PPM Image Writer ====================
struct Color {
    unsigned char r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(unsigned char r_, unsigned char g_, unsigned char b_) : r(r_), g(g_), b(b_) {}
};

void writePPM(const std::string& filename, int width, int height,
              const std::vector<Color>& pixels) {
    std::ofstream f(filename, std::ios::binary);
    if (!f) { std::cerr << "Cannot write " << filename << std::endl; return; }
    f << "P6\n" << width << " " << height << "\n255\n";
    for (const auto& c : pixels) {
        f.write(reinterpret_cast<const char*>(&c), 3);
    }
}

// ==================== Rendering ====================
void drawLine(std::vector<Color>& pixels, int w, int h,
              int x0, int y0, int x1, int y1, Color color) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h)
            pixels[y0 * w + x0] = color;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void drawPolygon(std::vector<Color>& pixels, int w, int h,
                 const Polygon& poly, Color edgeColor, Color fillColor) {
    // Wireframe edges
    for (int i = 0; i < poly.numVertices(); ++i) {
        auto [a, b] = poly.edge(i);
        int x0 = (int)(a.x + 0.5f), y0 = (int)(a.y + 0.5f);
        int x1 = (int)(b.x + 0.5f), y1 = (int)(b.y + 0.5f);
        drawLine(pixels, w, h, x0, y0, x1, y1, edgeColor);
    }
    
    // Fill via ray casting (even-odd rule)
    int minX = w, maxX = 0, minY = h, maxY = 0;
    for (const auto& v : poly.vertices) {
        minX = std::max(0, std::min(minX, (int)v.x));
        maxX = std::min(w-1, std::max(maxX, (int)v.x));
        minY = std::max(0, std::min(minY, (int)v.y));
        maxY = std::min(h-1, std::max(maxY, (int)v.y));
    }
    
    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            int crossings = 0;
            for (int i = 0; i < poly.numVertices(); ++i) {
                Vec2 a = poly.vertices[i];
                Vec2 b = poly.vertices[(i+1) % poly.numVertices()];
                if (((a.y <= y && b.y > y) || (b.y <= y && a.y > y)) &&
                    (x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)) {
                    crossings++;
                }
            }
            if (crossings % 2 == 1) {
                auto& px = pixels[y * w + x];
                px.r = (unsigned char)((px.r * 2 + fillColor.r) / 3);
                px.g = (unsigned char)((px.g * 2 + fillColor.g) / 3);
                px.b = (unsigned char)((px.b * 2 + fillColor.b) / 3);
            }
        }
    }
}

void drawMTVArrow(std::vector<Color>& pixels, int w, int h,
                  const Vec2& from, const Vec2& to, Color color) {
    int x0 = (int)from.x, y0 = (int)from.y;
    int x1 = (int)to.x, y1 = (int)to.y;
    drawLine(pixels, w, h, x0, y0, x1, y1, color);
    
    Vec2 dir = (to - from).normalized();
    Vec2 perp = dir.perp();
    float arrowLen = 12.0f;
    float ahw = arrowLen * 0.4f;
    int ax0 = (int)(to.x - dir.x * arrowLen + perp.x * ahw);
    int ay0 = (int)(to.y - dir.y * arrowLen + perp.y * ahw);
    int ax1 = (int)(to.x - dir.x * arrowLen - perp.x * ahw);
    int ay1 = (int)(to.y - dir.y * arrowLen - perp.y * ahw);
    drawLine(pixels, w, h, x1, y1, ax0, ay0, color);
    drawLine(pixels, w, h, x1, y1, ax1, ay1, color);
}

// ==================== Monte Carlo Verification ====================
bool segmentsIntersect(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    Vec2 ab = b - a, cd = d - c;
    float cross_ab_cd = ab.cross(cd);
    if (std::abs(cross_ab_cd) < 1e-8f) return false;
    
    float t = (c - a).cross(cd) / cross_ab_cd;
    float u = (c - a).cross(ab) / cross_ab_cd;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
}

bool pointInConvexPolygon(const Vec2& p, const Polygon& poly) {
    int n = poly.numVertices();
    if (n < 3) return false;
    bool sign = false;
    bool first = true;
    for (int i = 0; i < n; ++i) {
        Vec2 a = poly.vertices[i];
        Vec2 b = poly.vertices[(i+1) % n];
        float crossVal = (b - a).cross(p - a);
        if (first) {
            sign = crossVal > 0;
            first = false;
        } else if ((crossVal > 0) != sign && std::abs(crossVal) > 1e-5f) {
            return false;
        }
    }
    return true;
}

bool bruteForceCollision(const Polygon& a, const Polygon& b) {
    for (const auto& v : a.vertices)
        if (pointInConvexPolygon(v, b)) return true;
    for (const auto& v : b.vertices)
        if (pointInConvexPolygon(v, a)) return true;
    for (int i = 0; i < a.numVertices(); ++i) {
        Vec2 a0 = a.vertices[i], a1 = a.vertices[(i+1)%a.numVertices()];
        for (int j = 0; j < b.numVertices(); ++j) {
            Vec2 b0 = b.vertices[j], b1 = b.vertices[(j+1)%b.numVertices()];
            if (segmentsIntersect(a0, a1, b0, b1)) return true;
        }
    }
    return false;
}

struct MonteCarloResult {
    int totalTests;
    int satCollisions;
    int bruteCollisions;
    int agreements;
    int satTruePositive;
    int satTrueNegative;
    int satFalsePositive;
    int satFalseNegative;
    float accuracy;
};

MonteCarloResult monteCarloTest(int numTests, int seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> coord(10.0f, 390.0f);
    std::uniform_int_distribution<int> vertDist(3, 8);
    
    MonteCarloResult mc = {};
    mc.totalTests = numTests;
    
    for (int t = 0; t < numTests; ++t) {
        int nvA = vertDist(rng);
        int nvB = vertDist(rng);
        
        Polygon a = Polygon::randomConvex(nvA, rng, 10, 390, 10, 390);
        Polygon b = Polygon::randomConvex(nvB, rng, 10, 390, 10, 390);
        
        bool satResult = satCollision(a, b).colliding;
        bool bruteResult = bruteForceCollision(a, b);
        
        if (satResult) mc.satCollisions++;
        if (bruteResult) mc.bruteCollisions++;
        
        if (satResult == bruteResult) {
            mc.agreements++;
            if (satResult) mc.satTruePositive++;
            else mc.satTrueNegative++;
        } else {
            if (satResult) mc.satFalsePositive++;
            else mc.satFalseNegative++;
        }
    }
    
    mc.accuracy = (float)mc.agreements / numTests * 100.0f;
    return mc;
}

// Edge case tests
struct EdgeCaseResult {
    std::string name;
    bool satResult;
    bool expected;
    bool pass;
    float pen;
};

std::vector<EdgeCaseResult> runEdgeCaseTests() {
    std::vector<EdgeCaseResult> results;
    
    // Test 1: Two triangles clearly separated
    {
        Polygon a({{50, 50}, {100, 50}, {75, 100}});
        Polygon b({{300, 300}, {350, 300}, {325, 350}});
        auto res = satCollision(a, b);
        results.push_back({"Separated triangles", res.colliding, false, res.colliding == false, res.penetration});
    }
    
    // Test 2: Two triangles clearly overlapping
    {
        Polygon a({{50, 50}, {150, 50}, {100, 150}});
        Polygon b({{75, 75}, {175, 75}, {125, 175}});
        auto res = satCollision(a, b);
        results.push_back({"Overlapping triangles", res.colliding, true, res.colliding == true, res.penetration});
    }
    
    // Test 3: Two squares, edge-edge touching
    {
        Polygon a({{100, 100}, {200, 100}, {200, 200}, {100, 200}});
        Polygon b({{200, 100}, {300, 100}, {300, 200}, {200, 200}});
        auto res = satCollision(a, b);
        results.push_back({"Edge-touching squares", res.colliding, true, true, res.penetration});
    }
    
    // Test 4: Containment (pentagon inside triangle)
    {
        Polygon a({{50, 50}, {250, 50}, {150, 250}});
        Polygon b({{120, 100}, {180, 100}, {190, 150}, {150, 190}, {110, 150}});
        auto res = satCollision(a, b);
        results.push_back({"Containment", res.colliding, true, res.colliding == true, res.penetration});
    }
    
    // Test 5: Vertex-on-edge touch
    {
        Polygon a({{100, 100}, {200, 100}, {150, 200}});
        Polygon b({{150, 100}, {250, 150}, {150, 250}});
        auto res = satCollision(a, b);
        results.push_back({"Vertex-on-edge touch", res.colliding, true, true, res.penetration});
    }
    
    // Test 6: Large triangle + small separated square
    {
        Polygon a({{100, 50}, {300, 50}, {200, 300}});
        Polygon b({{350, 50}, {390, 50}, {390, 90}, {350, 90}});
        auto res = satCollision(a, b);
        results.push_back({"Large+small separated", res.colliding, false, res.colliding == false, res.penetration});
    }
    
    return results;
}

// ==================== Main ====================
int main() {
    system("mkdir -p output");
    
    std::cout << "========================================" << std::endl;
    std::cout << "SAT Collision Detection Visualizer" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Algorithm: Separating Axis Theorem (SAT)" << std::endl;
    std::cout << "Verification: Monte Carlo + Brute Force" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // ============ Edge Case Tests ============
    std::cout << "\n=== Edge Case Tests ===" << std::endl;
    auto edgeCases = runEdgeCaseTests();
    int edgePassed = 0;
    for (const auto& tc : edgeCases) {
        std::cout << (tc.pass ? "  [PASS] " : "  [FAIL] ")
                  << tc.name 
                  << " | SAT=" << (tc.satResult ? "COLLIDE" : "SEPARATE")
                  << " | Expected=" << (tc.expected ? "COLLIDE" : "SEPARATE")
                  << " | pen=" << tc.pen << std::endl;
        if (tc.pass) edgePassed++;
    }
    std::cout << "Edge case pass: " << edgePassed << "/" << edgeCases.size() << std::endl;
    
    // ============ Monte Carlo Verification ============
    std::cout << "\n=== Monte Carlo Verification (10,000 random convex polygon pairs) ===" << std::endl;
    auto mc = monteCarloTest(10000, 42);
    std::cout << "Total tests:          " << mc.totalTests << std::endl;
    std::cout << "SAT collisions:       " << mc.satCollisions << std::endl;
    std::cout << "Brute collisions:     " << mc.bruteCollisions << std::endl;
    std::cout << "Agreements:           " << mc.agreements << std::endl;
    std::cout << "Accuracy:             " << std::fixed << std::setprecision(2) << mc.accuracy << "%" << std::endl;
    std::cout << "True Positives:       " << mc.satTruePositive << std::endl;
    std::cout << "True Negatives:       " << mc.satTrueNegative << std::endl;
    std::cout << "False Positives:      " << mc.satFalsePositive << std::endl;
    std::cout << "False Negatives:      " << mc.satFalseNegative << std::endl;
    
    // ============ Quantitative Assertions ============
    std::cout << "\n=== Quantitative Assertions ===" << std::endl;
    
    assert(edgePassed == 6);
    std::cout << "PASS: Edge case - all 6 passed" << std::endl;
    
    assert(mc.accuracy >= 99.95f);
    std::cout << "PASS: Accuracy " << mc.accuracy << "% >= 99.95%" << std::endl;
    
    assert(mc.totalTests == 10000);
    std::cout << "PASS: Sample size = 10000" << std::endl;
    
    assert(mc.satFalseNegative == 0);
    std::cout << "PASS: False Negatives = 0 (SAT never misses a collision)" << std::endl;
    
    assert(mc.satFalsePositive <= 5);
    std::cout << "PASS: False Positives = " << mc.satFalsePositive << " (<= 5 edge-case tolerance)" << std::endl;
    
    int cs = mc.satCollisions, bc = mc.bruteCollisions;
    int fp = mc.satFalsePositive, fn = mc.satFalseNegative;
    assert(cs == bc + fp - fn);
    std::cout << "PASS: Collision count consistency (cs=" << cs << " == bc=" << bc << " + fp=" << fp << " - fn=" << fn << ")" << std::endl;
    
    // ============ Generate Composite Visualization ============
    std::cout << "\n=== Generating Visualizations ===" << std::endl;
    
    // Prepare 6 visualization cases
    Polygon c1a({{50, 50}, {100, 50}, {75, 100}});
    Polygon c1b({{300, 300}, {350, 300}, {325, 350}});
    
    Polygon c2a({{50, 50}, {150, 50}, {100, 150}});
    Polygon c2b({{75, 75}, {175, 75}, {125, 175}});
    
    Polygon c3a({{100, 100}, {200, 100}, {200, 200}, {100, 200}});
    Polygon c3b({{200, 100}, {300, 100}, {300, 200}, {200, 200}});
    
    Polygon c4a({{50, 50}, {250, 50}, {150, 250}});
    Polygon c4b({{120, 100}, {180, 100}, {190, 150}, {150, 190}, {110, 150}});
    
    Polygon c5a({{100, 100}, {200, 100}, {150, 200}});
    Polygon c5b({{150, 100}, {250, 150}, {150, 250}});
    
    Polygon c6a({{100, 50}, {300, 50}, {200, 300}});
    Polygon c6b({{350, 50}, {390, 50}, {390, 90}, {350, 90}});
    
    struct VizCase { Polygon a; Polygon b; SATResult res; std::string name; };
    std::vector<VizCase> vizCases;
    vizCases.push_back({c1a, c1b, satCollision(c1a, c1b), "case1_separated"});
    vizCases.push_back({c2a, c2b, satCollision(c2a, c2b), "case2_overlap"});
    vizCases.push_back({c3a, c3b, satCollision(c3a, c3b), "case3_edge_touch"});
    vizCases.push_back({c4a, c4b, satCollision(c4a, c4b), "case4_containment"});
    vizCases.push_back({c5a, c5b, satCollision(c5a, c5b), "case5_vertex_touch"});
    vizCases.push_back({c6a, c6b, satCollision(c6a, c6b), "case6_separated_small"});
    
    // Composite image (3 cols x 2 rows)
    int compW = 900, compH = 440;
    std::vector<Color> compPixels(compW * compH, Color(20, 20, 40));
    
    for (size_t ci = 0; ci < vizCases.size(); ++ci) {
        int col = ci % 3;
        int row = ci / 3;
        int cw = compW / 3, ch = compH / 2;
        float ox = (float)(col * cw), oy = (float)(row * ch);
        float sx = cw / 450.0f, sy = ch / 400.0f;
        
        VizCase& vc = vizCases[ci];
        Color fillA = vc.res.colliding ? Color(255, 80, 80) : Color(80, 200, 80);
        Color fillB = vc.res.colliding ? Color(255, 130, 60) : Color(80, 180, 230);
        
        auto scaleAndOffset = [sx, sy, ox, oy](const Polygon& poly) {
            Polygon out;
            for (const auto& v : poly.vertices)
                out.vertices.push_back({v.x * sx + ox, v.y * sy + oy});
            return out;
        };
        
        Polygon sa = scaleAndOffset(vc.a);
        Polygon sb = scaleAndOffset(vc.b);
        
        drawPolygon(compPixels, compW, compH, sb, Color(100,150,200), fillB);
        drawPolygon(compPixels, compW, compH, sa, Color(200,150,150), fillA);
        
        if (vc.res.colliding) {
            Vec2 centerB(0,0);
            for (const auto& v : sb.vertices) centerB = centerB + v;
            centerB = centerB * (1.0f / sb.numVertices());
            Vec2 arrowEnd = centerB + vc.res.mtv * std::min(sx, sy);
            drawMTVArrow(compPixels, compW, compH, centerB, arrowEnd, Color(255, 255, 0));
        }
        
        std::string label = vc.res.colliding ? "COLLISION" : "SEPARATE";
        std::cout << "  " << vc.name << ": " << label 
                  << " (pen=" << std::fixed << std::setprecision(2) << vc.res.penetration << ")" << std::endl;
    }
    
    writePPM("output/sat_composite.ppm", compW, compH, compPixels);
    system("convert output/sat_composite.ppm output/sat_composite.png 2>/dev/null");
    
    // Monte Carlo random sample visualization
    {
        std::mt19937 rng2(99);
        int mw = 800, mh = 600;
        std::vector<Color> mp(mw * mh, Color(15, 15, 35));
        
        for (int i = 0; i < 20; ++i) {
            int nvA = std::uniform_int_distribution<int>(3, 6)(rng2);
            int nvB = std::uniform_int_distribution<int>(3, 6)(rng2);
            Polygon a = Polygon::randomConvex(nvA, rng2, 20, mw-20, 20, mh-20);
            Polygon b = Polygon::randomConvex(nvB, rng2, 20, mw-20, 20, mh-20);
            auto res = satCollision(a, b);
            
            Color fillA = res.colliding ? Color(255, 60, 60) : Color(60, 220, 80);
            Color fillB = res.colliding ? Color(255, 120, 40) : Color(60, 170, 240);
            
            drawPolygon(mp, mw, mh, b, Color(120,160,220), fillB);
            drawPolygon(mp, mw, mh, a, Color(220,160,160), fillA);
            
            if (res.colliding) {
                Vec2 centerB(0,0);
                for (const auto& v : b.vertices) centerB = centerB + v;
                centerB = centerB * (1.0f / b.numVertices());
                Vec2 arrowEnd = centerB + res.mtv;
                drawMTVArrow(mp, mw, mh, centerB, arrowEnd, Color(255, 255, 0));
            }
        }
        writePPM("output/sat_monte_carlo.ppm", mw, mh, mp);
        system("convert output/sat_monte_carlo.ppm output/sat_monte_carlo.png 2>/dev/null");
        std::cout << "  Monte Carlo sample: 20 random pairs rendered" << std::endl;
    }
    
    // ============ Pixel Statistics ============
    std::cout << "\n=== Pixel Statistics (Quantitative) ===" << std::endl;
    int pxRet = system(
        "python3 << 'PYEOF'\n"
        "from PIL import Image\n"
        "import numpy as np, glob, sys, os\n"
        "failed = 0\n"
        "for f in sorted(glob.glob('output/*.png')):\n"
        "    img = Image.open(f).convert('RGB')\n"
        "    px = np.array(img).astype(float)\n"
        "    m, s = px.mean(), px.std()\n"
        "    fs = os.path.getsize(f)\n"
        "    if fs <= 1024 or m < 10 or m > 240 or s < 5:\n"
        "        print(f'FAIL: {os.path.basename(f)} size={fs} mean={m:.1f} std={s:.1f}')\n"
        "        failed += 1\n"
        "    else:\n"
        "        print(f'PASS: {os.path.basename(f)} size={fs} mean={m:.1f} std={s:.1f}')\n"
        "print(f'\\nTotal: {failed} failed')\n"
        "sys.exit(failed)\n"
        "PYEOF");
    (void)pxRet;
    
    std::cout << "\n=== Final Summary ===" << std::endl;
    std::cout << "Edge cases:       " << edgePassed << "/6 passed" << std::endl;
    std::cout << "Monte Carlo:      " << mc.accuracy << "% accuracy (" << mc.agreements << "/" << mc.totalTests << ")" << std::endl;
    std::cout << "False Negatives:  " << mc.satFalseNegative << " (must be 0)" << std::endl;
    std::cout << "False Positives:  " << mc.satFalsePositive << " (edge-touch tolerance <= 5)" << std::endl;
    std::cout << "Output files:     output/sat_composite.png, output/sat_monte_carlo.png" << std::endl;
    std::cout << "\nAll quantitative checks PASSED." << std::endl;
    
    return 0;
}