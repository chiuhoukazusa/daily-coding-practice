/**
 * Scanline Polygon Fill Algorithm
 * 
 * Implements the classic scanline polygon filling algorithm for rendering
 * arbitrary 2D polygons onto a framebuffer (PPM output).
 * 
 * Key data structures:
 * - Edge Table (ET): bucketed by min-y for all polygon edges
 * - Active Edge Table (AET): edges intersecting current scanline
 * 
 * Algorithm:
 * 1. Build Edge Table from polygon edges
 * 2. For each scanline y from ymin to ymax:
 *    a. Move edges from ET[y] to AET
 *    b. Sort AET by x intersection
 *    c. Fill pixels between pairs (even-odd rule)
 *    d. Update AET entries (x += dx/dy), remove stale edges
 * 
 * This implementation handles:
 * - Convex polygons
 * - Concave polygons
 * - Self-intersecting polygons (via even-odd rule)
 * - Multiple disjoint polygons
 * 
 * Quantifiable verification:
 * - Pixel fill count vs theoretical area
 * - Bounding box area coverage ratio
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <limits>
#include <iomanip>

const int WIDTH = 800;
const int HEIGHT = 600;
const int CHANNELS = 3;

struct Point2D {
    float x, y;
    Point2D(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
};

struct Edge {
    float y_min, y_max;      // y range of edge
    float x;                  // current x intersection with scanline
    float dx;                 // 1/slope: change in x per scanline step
    float dy;                 // total y delta (for normalization)
    
    Edge() {}
    Edge(const Point2D& p1, const Point2D& p2) {
        if (p1.y < p2.y) {
            y_min = p1.y;
            y_max = p2.y;
            x = p1.x;
        } else {
            y_min = p2.y;
            y_max = p1.y;
            x = p2.x;
        }
        dy = y_max - y_min;
        if (dy > 0) {
            dx = (p2.x - p1.x) / (p2.y - p1.y);
        } else {
            dx = 0; // horizontal edge, will be skipped
        }
    }
    
    // For sorting AET by x
    bool operator<(const Edge& other) const {
        return x < other.x;
    }
};

// Framebuffer class for PPM output
class Framebuffer {
public:
    std::vector<unsigned char> data;
    
    Framebuffer() : data(WIDTH * HEIGHT * CHANNELS, 0) {}
    
    void clear(unsigned char r, unsigned char g, unsigned char b) {
        for (int i = 0; i < WIDTH * HEIGHT; i++) {
            data[i * CHANNELS + 0] = r;
            data[i * CHANNELS + 1] = g;
            data[i * CHANNELS + 2] = b;
        }
    }
    
    void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
        int idx = (y * WIDTH + x) * CHANNELS;
        data[idx + 0] = r;
        data[idx + 1] = g;
        data[idx + 2] = b;
    }
    
    void drawLine(int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b) {
        // Bresenham's line algorithm
        int dx = std::abs(x1 - x0);
        int dy = -std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;
        
        while (true) {
            setPixel(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
    void drawPolygonOutline(const std::vector<Point2D>& poly, unsigned char r, unsigned char g, unsigned char b) {
        for (size_t i = 0; i < poly.size(); i++) {
            size_t j = (i + 1) % poly.size();
            drawLine(
                static_cast<int>(poly[i].x), static_cast<int>(poly[i].y),
                static_cast<int>(poly[j].x), static_cast<int>(poly[j].y),
                r, g, b
            );
        }
    }
    
    int countNonBlackPixels() const {
        int count = 0;
        for (int i = 0; i < WIDTH * HEIGHT; i++) {
            if (data[i * CHANNELS] > 0 || data[i * CHANNELS + 1] > 0 || data[i * CHANNELS + 2] > 0) {
                count++;
            }
        }
        return count;
    }
    
    int countPixelsOfColor(unsigned char r, unsigned char g, unsigned char b) const {
        int count = 0;
        for (int i = 0; i < WIDTH * HEIGHT; i++) {
            if (data[i * CHANNELS] == r && data[i * CHANNELS + 1] == g && data[i * CHANNELS + 2] == b) {
                count++;
            }
        }
        return count;
    }
    
    void savePPM(const std::string& filename) {
        // Save as P6 (binary) PPM
        std::ofstream file(filename, std::ios::binary);
        file << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
        file.write(reinterpret_cast<const char*>(data.data()), data.size());
        file.close();
        std::cout << "Saved: " << filename << " (" << data.size() << " bytes)" << std::endl;
    }
    
    void savePPM_ASCII(const std::string& filename) {
        // Save as P3 (ASCII) PPM for debugging
        std::ofstream file(filename);
        file << "P3\n" << WIDTH << " " << HEIGHT << "\n255\n";
        for (int i = 0; i < WIDTH * HEIGHT; i++) {
            file << (int)data[i * CHANNELS] << " "
                 << (int)data[i * CHANNELS + 1] << " "
                 << (int)data[i * CHANNELS + 2] << "\n";
        }
        file.close();
    }
};

/**
 * Compute the area of a polygon using the shoelace formula.
 * Returns the absolute area.
 */
double polygonArea(const std::vector<Point2D>& poly) {
    double area = 0.0;
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        area += poly[i].x * poly[j].y;
        area -= poly[j].x * poly[i].y;
    }
    return std::abs(area) * 0.5;
}

/**
 * Build the Edge Table (ET) from a polygon's edges.
 * ET is bucketed by y_min.
 * Horizontal edges are skipped (they don't cross scanlines).
 */
std::vector<std::vector<Edge>> buildEdgeTable(const std::vector<Point2D>& poly, int height) {
    std::vector<std::vector<Edge>> et(height);
    
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        Edge edge(poly[i], poly[j]);
        
        // Skip horizontal edges
        if (edge.dy <= 0) continue;
        
        int bucket = static_cast<int>(edge.y_min);
        if (bucket < 0) bucket = 0;
        if (bucket >= height) continue;
        
        et[bucket].push_back(edge);
    }
    
    return et;
}

/**
 * Scanline fill a single polygon into the framebuffer.
 * Uses even-odd rule for filling.
 * Returns the number of pixels filled.
 */
int scanlineFillSingle(Framebuffer& fb, const std::vector<Point2D>& poly,
                       unsigned char r, unsigned char g, unsigned char b) {
    std::vector<std::vector<Edge>> et = buildEdgeTable(poly, HEIGHT);
    std::vector<Edge> aet;
    
    // Find scanline bounds
    int y_min = HEIGHT, y_max = 0;
    for (auto& p : poly) {
        int y = static_cast<int>(p.y);
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
    }
    if (y_min < 0) y_min = 0;
    if (y_max >= HEIGHT) y_max = HEIGHT - 1;
    
    int pixels_filled = 0;
    
    for (int y = y_min; y <= y_max; y++) {
        // Step a: Add edges from ET[y] to AET
        for (auto& e : et[y]) {
            aet.push_back(e);
        }
        
        // Step b: Remove edges where y >= y_max
        aet.erase(
            std::remove_if(aet.begin(), aet.end(),
                [y](const Edge& e) { return y >= static_cast<int>(e.y_max); }),
            aet.end()
        );
        
        // Step c: Sort AET by x
        std::sort(aet.begin(), aet.end());
        
        // Step d: Fill between pairs (even-odd rule)
        for (size_t i = 0; i + 1 < aet.size(); i += 2) {
            int x_start = static_cast<int>(std::ceil(aet[i].x));
            int x_end   = static_cast<int>(std::floor(aet[i + 1].x));
            
            if (x_start < 0) x_start = 0;
            if (x_end >= WIDTH) x_end = WIDTH - 1;
            
            for (int x = x_start; x <= x_end; x++) {
                fb.setPixel(x, y, r, g, b);
                pixels_filled++;
            }
        }
        
        // Step e: Update x positions for next scanline
        for (auto& e : aet) {
            e.x += e.dx;
        }
    }
    
    return pixels_filled;
}

/**
 * Generate a star-shaped polygon centered at (cx, cy).
 */
std::vector<Point2D> generateStar(float cx, float cy, float outer_r, float inner_r, int points) {
    std::vector<Point2D> poly;
    for (int i = 0; i < points * 2; i++) {
        float angle = M_PI * i / points - M_PI / 2;
        float r = (i % 2 == 0) ? outer_r : inner_r;
        poly.push_back(Point2D(cx + r * cos(angle), cy + r * sin(angle)));
    }
    return poly;
}

/**
 * Generate a regular convex polygon.
 */
std::vector<Point2D> generateRegularPolygon(float cx, float cy, float radius, int sides, float rot = 0) {
    std::vector<Point2D> poly;
    for (int i = 0; i < sides; i++) {
        float angle = 2.0 * M_PI * i / sides + rot - M_PI / 2;
        poly.push_back(Point2D(cx + radius * cos(angle), cy + radius * sin(angle)));
    }
    return poly;
}

int main() {
    Framebuffer fb;
    fb.clear(20, 20, 40); // Dark background
    
    // ========================================
    // Test 1: Simple Triangle (convex)
    // ========================================
    std::vector<Point2D> triangle = {
        Point2D(100, 100),
        Point2D(200, 300),
        Point2D(50, 250)
    };
    
    int tri_pixels = scanlineFillSingle(fb, triangle, 255, 80, 80);
    fb.drawPolygonOutline(triangle, 255, 200, 200);
    double tri_area = polygonArea(triangle);
    std::cout << "Triangle: filled=" << tri_pixels << " area=" << tri_area << std::endl;
    
    // ========================================
    // Test 2: Concave Polygon (arrow shape)
    // ========================================
    std::vector<Point2D> concave = {
        Point2D(350, 100),
        Point2D(450, 100),
        Point2D(450, 200),
        Point2D(520, 200),
        Point2D(400, 350),
        Point2D(280, 200),
        Point2D(350, 200)
    };
    
    int concave_pixels = scanlineFillSingle(fb, concave, 80, 200, 80);
    fb.drawPolygonOutline(concave, 150, 255, 150);
    double concave_area = polygonArea(concave);
    std::cout << "Concave: filled=" << concave_pixels << " area=" << concave_area << std::endl;
    
    // ========================================
    // Test 3: Star polygon (self-intersecting via even-odd)
    // ========================================
    std::vector<Point2D> star = generateStar(650, 200, 80, 35, 5);
    
    int star_pixels = scanlineFillSingle(fb, star, 255, 200, 50);
    fb.drawPolygonOutline(star, 255, 240, 150);
    double star_area = polygonArea(star);
    std::cout << "Star: filled=" << star_pixels << " area=" << star_area 
              << " (note: area uses shoelace which gives different result for self-intersecting)" << std::endl;
    
    // ========================================
    // Test 4: Regular hexagon (convex)
    // ========================================
    std::vector<Point2D> hexagon = generateRegularPolygon(200, 480, 80, 6);
    int hex_pixels = scanlineFillSingle(fb, hexagon, 80, 80, 255);
    fb.drawPolygonOutline(hexagon, 180, 180, 255);
    double hex_area = polygonArea(hexagon);
    std::cout << "Hexagon: filled=" << hex_pixels << " area=" << hex_area << std::endl;
    
    // ========================================
    // Test 5: Octagon
    // ========================================
    std::vector<Point2D> octagon = generateRegularPolygon(450, 480, 70, 8);
    int oct_pixels = scanlineFillSingle(fb, octagon, 255, 150, 50);
    fb.drawPolygonOutline(octagon, 255, 200, 120);
    double oct_area = polygonArea(octagon);
    std::cout << "Octagon: filled=" << oct_pixels << " area=" << oct_area << std::endl;
    
    // ========================================
    // Test 6: Thin polygon (stress test for edge cases)
    // ========================================
    std::vector<Point2D> thin_poly = {
        Point2D(620, 400),
        Point2D(720, 405),
        Point2D(760, 550),
        Point2D(640, 555)
    };
    int thin_pixels = scanlineFillSingle(fb, thin_poly, 200, 100, 255);
    fb.drawPolygonOutline(thin_poly, 240, 180, 255);
    double thin_area = polygonArea(thin_poly);
    std::cout << "Thin quad: filled=" << thin_pixels << " area=" << thin_area << std::endl;
    
    // ========================================
    // Save output
    // ========================================
    fb.savePPM("scanline_output.ppm");
    
    // ========================================
    // Summary statistics
    // ========================================
    int total_pixels = fb.countNonBlackPixels();
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Total non-background pixels: " << total_pixels << std::endl;
    std::cout << "Total pixels: " << (WIDTH * HEIGHT) << std::endl;
    std::cout << "Coverage: " << std::fixed << std::setprecision(2) 
              << (100.0 * total_pixels / (WIDTH * HEIGHT)) << "%" << std::endl;
    
    // Quantitative validation data
    std::cout << "\n=== Quantitative Validation ===" << std::endl;
    std::cout << "Polygon     | Shoelace Area | Pixels Filled | Area/Pixel Ratio" << std::endl;
    std::cout << "------------|---------------|---------------|-----------------" << std::endl;
    
    auto print_row = [](const char* name, double area, int pixels) {
        double ratio = (pixels > 0) ? area / pixels : 0;
        std::cout << std::left << std::setw(12) << name << "| "
                  << std::right << std::setw(13) << std::fixed << std::setprecision(1) << area << " | "
                  << std::right << std::setw(13) << pixels << " | "
                  << std::right << std::setw(15) << std::fixed << std::setprecision(4) << ratio << std::endl;
    };
    
    print_row("Triangle", tri_area, tri_pixels);
    print_row("Concave", concave_area, concave_pixels);
    print_row("Star", star_area, star_pixels);
    print_row("Hexagon", hex_area, hex_pixels);
    print_row("Octagon", oct_area, oct_pixels);
    print_row("Thin quad", thin_area, thin_pixels);
    
    return 0;
}
