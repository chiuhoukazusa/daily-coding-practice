/**
 * Ear Clipping Polygon Triangulation
 * 
 * Classic O(n^2) polygon triangulation algorithm.
 * Quantitative verification:
 *   1. All triangles are non-degenerate (area > epsilon)
 *   2. Sum of triangle areas == original polygon area (within FP tolerance)
 *   3. Each triangle has counter-clockwise winding
 *   4. All vertices appear in exactly one triangle (partition completeness)
 *   5. Handles convex, concave, and star-shaped polygons
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>
#include <cassert>
#include <tuple>

const double EPS = 1e-9;

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
    
    Point operator-(const Point& p) const { return Point(x - p.x, y - p.y); }
    Point operator+(const Point& p) const { return Point(x + p.x, y + p.y); }
    bool operator==(const Point& p) const { return std::abs(x - p.x) < EPS && std::abs(y - p.y) < EPS; }
};

// 2D cross product
double cross(const Point& a, const Point& b) {
    return a.x * b.y - a.y * b.x;
}

double cross(const Point& a, const Point& b, const Point& c) {
    return cross(b - a, c - a);
}

double dot(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y;
}

// Polygon area (signed, positive for CCW)
double polygonArea(const std::vector<Point>& poly) {
    double area = 0;
    int n = (int)poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += cross(poly[i], poly[j]);
    }
    return area * 0.5;
}

// Triangle area
double triangleArea(const Point& a, const Point& b, const Point& c) {
    return std::abs(cross(a, b, c)) * 0.5;
}

// Check if point p is inside triangle (a, b, c) - using barycentric technique
bool pointInTriangle(const Point& p, const Point& a, const Point& b, const Point& c) {
    double d1 = cross(b - a, p - a);
    double d2 = cross(c - b, p - b);
    double d3 = cross(a - c, p - c);
    bool has_neg = (d1 < -EPS) || (d2 < -EPS) || (d3 < -EPS);
    bool has_pos = (d1 > EPS) || (d2 > EPS) || (d3 > EPS);
    return !(has_neg && has_pos);
}

// Check if vertex at index i is an "ear" (convex + no other vertex inside)
bool isEar(const std::vector<Point>& poly, int i, const std::vector<int>& indices) {
    int n = (int)indices.size();
    int prev = indices[(i - 1 + n) % n];
    int curr = indices[i];
    int next = indices[(i + 1) % n];
    
    const Point& A = poly[prev];
    const Point& B = poly[curr];
    const Point& C = poly[next];
    
    // Must be convex (interior angle < 180): cross(BC, BA) > 0 for CCW poly
    if (cross(C - B, A - B) <= EPS) {
        return false;
    }
    
    // No other vertex inside triangle (A, B, C)
    for (int j = 0; j < n; j++) {
        int idx = indices[j];
        if (idx == prev || idx == curr || idx == next) continue;
        const Point& P = poly[idx];
        if (pointInTriangle(P, A, B, C)) {
            return false;
        }
    }
    
    return true;
}

struct Triangle {
    int a, b, c;
};

std::vector<Triangle> earClippingTriangulation(const std::vector<Point>& poly) {
    int n = (int)poly.size();
    if (n < 3) return {};
    
    // Ensure CCW winding
    double area = polygonArea(poly);
    std::vector<int> indices(n);
    for (int i = 0; i < n; i++) indices[i] = i;
    
    if (area < 0) {
        // Reverse to CCW
        std::reverse(indices.begin() + 1, indices.end());
    }
    
    std::vector<Triangle> triangles;
    int remaining = n;
    int safety = 0;
    
    while (remaining > 3 && safety < n * n) {
        safety++;
        bool foundEar = false;
        
        for (int i = 0; i < remaining; i++) {
            if (isEar(poly, i, indices)) {
                int prev = indices[(i - 1 + remaining) % remaining];
                int curr = indices[i];
                int next = indices[(i + 1) % remaining];
                
                triangles.push_back({prev, curr, next});
                indices.erase(indices.begin() + i);
                remaining--;
                foundEar = true;
                break;
            }
        }
        
        if (!foundEar) {
            std::cerr << "ERROR: No ear found! Remaining: " << remaining << std::endl;
            break;
        }
    }
    
    // Last triangle
    if (remaining == 3) {
        triangles.push_back({indices[0], indices[1], indices[2]});
    }
    
    return triangles;
}

// --- QUANTITATIVE VERIFICATION ---

struct VerificationResult {
    bool allNonDegenerate;
    bool areaMatches;
    bool allCCW;
    bool completePartition;
    double totalTriArea;
    double polyArea;
    double areaError;
    int degenerateCount;
    std::string summary;
};

VerificationResult verifyTriangulation(
    const std::vector<Point>& poly,
    const std::vector<Triangle>& triangles)
{
    VerificationResult res = {};
    res.allNonDegenerate = true;
    res.areaMatches = true;
    res.allCCW = true;
    res.completePartition = true;
    res.totalTriArea = 0;
    res.polyArea = std::abs(polygonArea(poly));
    res.degenerateCount = 0;
    res.areaError = 0;
    
    std::vector<int> vertexCount(poly.size(), 0);
    
    for (size_t t = 0; t < triangles.size(); t++) {
        const auto& tri = triangles[t];
        const Point& a = poly[tri.a];
        const Point& b = poly[tri.b];
        const Point& c = poly[tri.c];
        
        // 1. Non-degenerate check
        double area = triangleArea(a, b, c);
        res.totalTriArea += area;
        if (area < EPS) {
            res.allNonDegenerate = false;
            res.degenerateCount++;
        }
        
        // 2. CCW winding check
        double signedArea = cross(b - a, c - a);
        if (signedArea <= EPS) {
            res.allCCW = false;
        }
        
        // 3. Vertex coverage
        vertexCount[tri.a]++;
        vertexCount[tri.b]++;
        vertexCount[tri.c]++;
    }
    
    // 4. Area match
    res.areaError = std::abs(res.totalTriArea - res.polyArea);
    res.areaMatches = (res.areaError < 1e-6);
    
    // 5. Complete partition: every vertex appears at least once
    for (size_t i = 0; i < vertexCount.size(); i++) {
        if (vertexCount[i] == 0) {
            res.completePartition = false;
        }
    }
    
    // Build summary
    std::ostringstream oss;
    oss << "=== VERIFICATION RESULTS ===\n";
    oss << "Polygon vertices: " << poly.size() << "\n";
    oss << "Triangles generated: " << triangles.size() << "\n";
    oss << "Expected triangle count (n-2): " << (poly.size() - 2) << "\n\n";
    
    oss << "1. Non-degenerate triangles: " << (res.allNonDegenerate ? "PASS" : "FAIL")
        << " (" << res.degenerateCount << " degenerate)\n";
    oss << "2. Area sum matches original: " << (res.areaMatches ? "PASS" : "FAIL")
        << " (poly=" << std::fixed << std::setprecision(6) << res.polyArea
        << ", sum=" << res.totalTriArea
        << ", error=" << res.areaError << ")\n";
    oss << "3. All CCW winding: " << (res.allCCW ? "PASS" : "FAIL") << "\n";
    oss << "4. Complete vertex coverage: " << (res.completePartition ? "PASS" : "FAIL") << "\n\n";
    
    bool allPass = res.allNonDegenerate && res.areaMatches && res.allCCW && res.completePartition;
    oss << "OVERALL: " << (allPass ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << "\n";
    
    if (triangles.size() == poly.size() - 2) {
        oss << "Triangle count check: PASS (exactly n-2)\n";
    } else {
        oss << "Triangle count check: WARN (got " << triangles.size()
            << " expected " << (poly.size() - 2) << ")\n";
    }
    
    res.summary = oss.str();
    return res;
}

// --- PPM OUTPUT ---

void writePPM(const std::string& filename, int w, int h,
              const std::vector<Point>& poly,
              const std::vector<Triangle>& triangles,
              const std::vector<Point>& extraPoints = {})
{
    std::ofstream out(filename);
    out << "P3\n" << w << " " << h << "\n255\n";
    
    // Color palette for triangles
    auto triColor = [](int idx, double bary) -> std::tuple<int,int,int> {
        int r, g, b;
        switch(idx % 7) {
            case 0: r=255;g=100;b=100; break; // red
            case 1: r=100;g=255;b=100; break; // green
            case 2: r=100;g=100;b=255; break; // blue
            case 3: r=255;g=255;b=100; break; // yellow
            case 4: r=255;g=100;b=255; break; // magenta
            case 5: r=100;g=255;b=255; break; // cyan
            case 6: r=255;g=200;b=100; break; // orange
            default: r=200;g=200;b=200;
        }
        // Darken based on barycentric (closer to center = brighter)
        double f = 0.6 + 0.4 * (1.0 - std::abs(bary));
        return {(int)(r*f), (int)(g*f), (int)(b*f)};
    };
    
    // Bounding box for viewport transform
    double minX = poly[0].x, maxX = poly[0].x;
    double minY = poly[0].y, maxY = poly[0].y;
    for (auto& p : poly) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    double margin = 20.0;
    double scaleX = (w - 2*margin) / (maxX - minX + EPS);
    double scaleY = (h - 2*margin) / (maxY - minY + EPS);
    double scale = std::min(scaleX, scaleY);
    double offsetX = margin - minX * scale;
    double offsetY = margin - minY * scale;
    
    auto toScreen = [&](const Point& p) -> std::pair<int,int> {
        int sx = (int)(p.x * scale + offsetX);
        int sy = (int)(h - 1 - (p.y * scale + offsetY)); // flip Y
        return {sx, sy};
    };
    
    // Rasterize triangles
    std::vector<std::vector<int>> colorBuf(w, std::vector<int>(h, -1));
    
    for (int t = 0; t < (int)triangles.size(); t++) {
        const auto& tri = triangles[t];
        auto [ax, ay] = toScreen(poly[tri.a]);
        auto [bx, by] = toScreen(poly[tri.b]);
        auto [cx, cy] = toScreen(poly[tri.c]);
        
        int minpx = std::max(0, std::min({ax, bx, cx}));
        int maxpx = std::min(w-1, std::max({ax, bx, cx}));
        int minpy = std::max(0, std::min({ay, by, cy}));
        int maxpy = std::min(h-1, std::max({ay, by, cy}));
        
        for (int py = minpy; py <= maxpy; py++) {
            for (int px = minpx; px <= maxpx; px++) {
                // Barycentric test using integer coordinates
                double d0 = (double)(bx - ax)*(cy - ay) - (double)(by - ay)*(cx - ax);
                if (std::abs(d0) < EPS) continue;
                double w1 = ((double)(by - cy)*(px - cx) + (double)(cx - bx)*(py - cy)) / d0;
                double w2 = ((double)(cy - ay)*(px - cx) + (double)(ax - cx)*(py - cy)) / d0;
                double w3 = 1.0 - w1 - w2;
                if (w1 >= -EPS && w2 >= -EPS && w3 >= -EPS) {
                    if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
                        colorBuf[px][py] = t;
                    }
                }
            }
        }
    }
    
    // Write pixels
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int t = colorBuf[x][y];
            if (t >= 0) {
                auto [r,g,b] = triColor(t, 0.5);
                out << r << " " << g << " " << b << " ";
            } else {
                out << "30 30 30 "; // dark background
            }
        }
        out << "\n";
    }
    out.close();
}

// --- TEST POLYGONS ---

std::vector<Point> makeConvexPolygon(int sides, double radius = 200.0) {
    std::vector<Point> pts;
    for (int i = 0; i < sides; i++) {
        double angle = 2.0 * M_PI * i / sides - M_PI / 2.0;
        pts.push_back(Point(radius * cos(angle) + 250, radius * sin(angle) + 250));
    }
    return pts;
}

std::vector<Point> makeConcavePolygon() {
    // Star-like concave shape
    return {
        {250, 50},   // top
        {350, 150},
        {450, 80},
        {400, 200},
        {480, 280},
        {350, 260},
        {300, 400},
        {230, 300},
        {100, 420},
        {180, 250},
        {50,  250},
        {170, 180},
        {100, 80},
        {210, 160},
    };
}

std::vector<Point> makeArrowShape() {
    return {
        {200, 100},  // top tip
        {250, 200},
        {450, 200},  // right tip
        {350, 280},
        {380, 400},  // bottom tip
        {280, 320},
        {200, 420},
        {200, 300},
        {50,  300},  // left tip
        {150, 220},
    };
}

std::vector<Point> makeLShape() {
    return {
        {100, 100},
        {100, 400},
        {200, 400},
        {200, 200},
        {400, 200},
        {400, 100},
    };
}

int main() {
    struct TestCase {
        std::string name;
        std::vector<Point> poly;
    };
    
    std::vector<TestCase> tests = {
        {"Convex Hexagon", makeConvexPolygon(6)},
        {"Concave Star", makeConcavePolygon()},
        {"Arrow Shape", makeArrowShape()},
        {"L-Shape", makeLShape()},
        {"Convex Octagon", makeConvexPolygon(8, 180.0)},
    };
    
    bool allTestsPassed = true;
    
    for (size_t testIdx = 0; testIdx < tests.size(); testIdx++) {
        const auto& tc = tests[testIdx];
        std::cout << "\n========================================\n";
        std::cout << "TEST " << (testIdx+1) << ": " << tc.name << "\n";
        std::cout << "========================================\n";
        
        auto triangles = earClippingTriangulation(tc.poly);
        auto result = verifyTriangulation(tc.poly, triangles);
        
        std::cout << result.summary;
        
        // Generate PPM
        std::ostringstream fname;
        fname << "triangulation_" << (testIdx+1) << "_" << tc.name << ".ppm";
        // Replace spaces
        std::string fn = fname.str();
        std::replace(fn.begin(), fn.end(), ' ', '_');
        writePPM(fn, 512, 512, tc.poly, triangles);
        std::cout << "Output: " << fn << "\n";
        
        if (!result.allNonDegenerate || !result.areaMatches || 
            !result.allCCW || !result.completePartition) {
            allTestsPassed = false;
        }
    }
    
    // Final summary
    std::cout << "\n========================================\n";
    std::cout << "FINAL RESULT: " << (allTestsPassed ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << "\n";
    std::cout << "========================================\n";
    
    return allTestsPassed ? 0 : 1;
}
