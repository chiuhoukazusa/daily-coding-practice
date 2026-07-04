// Sutherland-Hodgman Polygon Clipping Algorithm
// Clips a polygon against a rectangular clip window using the Sutherland-Hodgman algorithm.
// 
// The algorithm processes each polygon edge against each clip boundary (left, right, bottom, top),
// outputting vertices for the clipped polygon at each stage.
//
// Output: PPM image with original polygon in blue, clip window in white, clipped polygon in red.

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstdio>
#include <string>
#include <limits>
#include <cstdlib>

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
};

// Compute intersection of line segment p1->p2 with a clip boundary edge.
// The boundary edge is defined by a point `edge_p` and inward normal direction.
// Returns the intersection point.
Point intersect(const Point& p1, const Point& p2, const Point& edge_p1, const Point& edge_p2) {
    // Parametric intersection of two line segments
    double x1 = p1.x, y1 = p1.y;
    double x2 = p2.x, y2 = p2.y;
    double x3 = edge_p1.x, y3 = edge_p1.y;
    double x4 = edge_p2.x, y4 = edge_p2.y;

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(denom) < 1e-12) {
        // Parallel lines - return the endpoint (shouldn't happen in normal clipping)
        return p2;
    }

    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    (void)0; /* u removed - unused variable */

    // Clamp to segment bounds for numerical stability
    t = std::max(0.0, std::min(1.0, t));

    return Point(x1 + t * (x2 - x1), y1 + t * (y2 - y1));
}

// Check if a point is inside a clip boundary edge.
// The boundary edges go clockwise around the clip window.
// "Inside" means to the RIGHT of the directed edge.
// cross > 0 means LEFT; cross < 0 means RIGHT.
// So inside = cross <= 0.
bool inside(const Point& p, const Point& edge_p1, const Point& edge_p2) {
    // Cross product: (edge direction) × (point relative to edge start)
    double cross = (edge_p2.x - edge_p1.x) * (p.y - edge_p1.y) 
                 - (edge_p2.y - edge_p1.y) * (p.x - edge_p1.x);
    // Clockwise edges: "inside" is to the RIGHT, so cross <= 0.
    return cross <= 1e-12;
}

// Clip a polygon against a single clip edge
std::vector<Point> clipEdge(const std::vector<Point>& polygon,
                            const Point& edge_p1, const Point& edge_p2) {
    std::vector<Point> output;
    if (polygon.empty()) return output;

    for (size_t i = 0; i < polygon.size(); i++) {
        const Point& current = polygon[i];
        const Point& next = polygon[(i + 1) % polygon.size()];

        bool curr_inside = inside(current, edge_p1, edge_p2);
        bool next_inside = inside(next, edge_p1, edge_p2);

        if (curr_inside && next_inside) {
            // Case 1: Both inside → output next vertex only
            output.push_back(next);
        } else if (curr_inside && !next_inside) {
            // Case 2: Inside → Outside → output intersection point
            output.push_back(intersect(current, next, edge_p1, edge_p2));
        } else if (!curr_inside && next_inside) {
            // Case 3: Outside → Inside → output intersection point + next vertex
            output.push_back(intersect(current, next, edge_p1, edge_p2));
            output.push_back(next);
        }
        // Case 4: Both outside → output nothing (implicit)
    }

    return output;
}

// Sutherland-Hodgman polygon clipping against a rectangular window
std::vector<Point> sutherlandHodgman(const std::vector<Point>& polygon,
                                      double xmin, double ymin, double xmax, double ymax) {
    // Define the four clip boundaries in clockwise order
    // Each edge: p1 -> p2, where "inside" is to the right of the edge direction
    // Left edge: bottom-left (xmin, ymin) -> top-left (xmin, ymax)
    // Top edge: top-left (xmin, ymax) -> top-right (xmax, ymax)
    // Right edge: top-right (xmax, ymax) -> bottom-right (xmax, ymin)
    // Bottom edge: bottom-right (xmax, ymin) -> bottom-left (xmin, ymin)
    
    struct Edge { Point p1, p2; };
    Edge edges[4] = {
        { Point(xmin, ymin), Point(xmin, ymax) },  // Left
        { Point(xmin, ymax), Point(xmax, ymax) },  // Top
        { Point(xmax, ymax), Point(xmax, ymin) },  // Right
        { Point(xmax, ymin), Point(xmin, ymin) }   // Bottom
    };

    std::vector<Point> output = polygon;
    for (int i = 0; i < 4; i++) {
        output = clipEdge(output, edges[i].p1, edges[i].p2);
        if (output.empty()) break;
    }

    return output;
}

// Generate various test polygons
std::vector<Point> generateStarPolygon(double cx, double cy, double outer_r, double inner_r, int points) {
    std::vector<Point> poly;
    double angle_step = M_PI / points;
    for (int i = 0; i < 2 * points; i++) {
        double r = (i % 2 == 0) ? outer_r : inner_r;
        double angle = i * angle_step - M_PI / 2; // Start from top
        poly.push_back(Point(cx + r * cos(angle), cy + r * sin(angle)));
    }
    return poly;
}

std::vector<Point> generateRegularPolygon(double cx, double cy, double r, int sides, double start_angle = -M_PI/2) {
    std::vector<Point> poly;
    for (int i = 0; i < sides; i++) {
        double angle = start_angle + 2 * M_PI * i / sides;
        poly.push_back(Point(cx + r * cos(angle), cy + r * sin(angle)));
    }
    return poly;
}

// Draw a line on the PPM buffer using Bresenham's algorithm
void drawLine(std::vector<std::vector<uint8_t>>& r,
              std::vector<std::vector<uint8_t>>& g,
              std::vector<std::vector<uint8_t>>& b,
              int x1, int y1, int x2, int y2,
              uint8_t cr, uint8_t cg, uint8_t cb,
              int width, int height) {
    int dx = std::abs(x2 - x1);
    int dy = std::abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x1 >= 0 && x1 < width && y1 >= 0 && y1 < height) {
            r[y1][x1] = cr;
            g[y1][x1] = cg;
            b[y1][x1] = cb;
        }
        if (x1 == x2 && y1 == y2) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x1 += sx; }
        if (e2 < dx)  { err += dx; y1 += sy; }
    }
}

// Fill a polygon using scanline algorithm
void fillPolygon(std::vector<std::vector<uint8_t>>& r,
                 std::vector<std::vector<uint8_t>>& g,
                 std::vector<std::vector<uint8_t>>& b,
                 const std::vector<Point>& poly,
                 uint8_t cr, uint8_t cg_color, uint8_t cb_val,
                 int width, int height,
                 double alpha = 1.0) {
    if (poly.size() < 3) return;

    // Find y range
    double ymin = poly[0].y, ymax = poly[0].y;
    for (const auto& p : poly) {
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }

    int y_start = std::max(0, (int)std::floor(ymin));
    int y_end = std::min(height - 1, (int)std::ceil(ymax));

    for (int y = y_start; y <= y_end; y++) {
        // Find all intersections of scanline with polygon edges
        std::vector<double> x_intersections;
        for (size_t i = 0; i < poly.size(); i++) {
            const Point& p1 = poly[i];
            const Point& p2 = poly[(i + 1) % poly.size()];

            if ((p1.y <= y && p2.y > y) || (p2.y <= y && p1.y > y)) {
                double t = (y - p1.y) / (p2.y - p1.y);
                double x = p1.x + t * (p2.x - p1.x);
                x_intersections.push_back(x);
            }
        }

        // Sort intersections
        std::sort(x_intersections.begin(), x_intersections.end());

        // Fill between pairs of intersections
        for (size_t i = 0; i + 1 < x_intersections.size(); i += 2) {
            int x_start = std::max(0, (int)std::ceil(x_intersections[i]));
            int x_end = std::min(width - 1, (int)std::floor(x_intersections[i + 1]));

            for (int x = x_start; x <= x_end; x++) {
                if (x >= 0 && x < width && y >= 0 && y < height) {
                    if (alpha >= 1.0) {
                        r[y][x] = cr;
                        g[y][x] = cg_color;
                        b[y][x] = cb_val;
                    } else {
                        r[y][x] = (uint8_t)(r[y][x] * (1 - alpha) + cr * alpha);
                        g[y][x] = (uint8_t)(g[y][x] * (1 - alpha) + cg_color * alpha);
                        b[y][x] = (uint8_t)(b[y][x] * (1 - alpha) + cb_val * alpha);
                    }
                }
            }
        }
    }
}

// Fill the clip window with a semi-transparent overlay
void fillRect(std::vector<std::vector<uint8_t>>& r,
              std::vector<std::vector<uint8_t>>& g,
              std::vector<std::vector<uint8_t>>& b,
              double xmin, double ymin, double xmax, double ymax,
              uint8_t cr, uint8_t cg_color, uint8_t cb_val,
              int width, int height,
              double alpha = 1.0) {
    int x1 = std::max(0, (int)xmin);
    int y1 = std::max(0, (int)ymin);
    int x2 = std::min(width - 1, (int)xmax);
    int y2 = std::min(height - 1, (int)ymax);

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            if (alpha >= 1.0) {
                r[y][x] = cr;
                g[y][x] = cg_color;
                b[y][x] = cb_val;
            } else {
                r[y][x] = (uint8_t)(r[y][x] * (1 - alpha) + cr * alpha);
                g[y][x] = (uint8_t)(g[y][x] * (1 - alpha) + cg_color * alpha);
                b[y][x] = (uint8_t)(b[y][x] * (1 - alpha) + cb_val * alpha);
            }
        }
    }
}

// Write PPM file (P6 binary format)
void writePPM(const std::string& filename,
              const std::vector<std::vector<uint8_t>>& r,
              const std::vector<std::vector<uint8_t>>& g,
              const std::vector<std::vector<uint8_t>>& b,
              int width, int height) {
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << width << " " << height << "\n255\n";
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            out.put((char)r[y][x]);
            out.put((char)g[y][x]);
            out.put((char)b[y][x]);
        }
    }
    out.close();
}

// Compute polygon area using the shoelace formula (for coverage verification)
double polygonArea(const std::vector<Point>& poly) {
    if (poly.size() < 3) return 0.0;
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); i++) {
        const Point& a = poly[i];
        const Point& b = poly[(i + 1) % poly.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return std::abs(area) * 0.5;
}

// Compute bounding box area
double boundingBoxArea(const std::vector<Point>& poly) {
    if (poly.empty()) return 0.0;
    double xmin = poly[0].x, xmax = poly[0].x;
    double ymin = poly[0].y, ymax = poly[0].y;
    for (const auto& p : poly) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    return (xmax - xmin) * (ymax - ymin);
}

// Count how many vertices of the clipped polygon are exactly on the clip boundary
int countBoundaryVertices(const std::vector<Point>& poly, 
                          double xmin, double ymin, double xmax, double ymax,
                          double eps = 1.0) {
    int count = 0;
    for (const auto& p : poly) {
        bool on_left   = std::abs(p.x - xmin) < eps;
        bool on_right  = std::abs(p.x - xmax) < eps;
        bool on_bottom = std::abs(p.y - ymin) < eps;
        bool on_top    = std::abs(p.y - ymax) < eps;
        if (on_left || on_right || on_bottom || on_top) count++;
    }
    return count;
}

int main() {
    const int WIDTH = 1200;
    const int HEIGHT = 900;

    // Clip window: centered, 300x200
    const double CX = WIDTH / 2.0;
    const double CY = HEIGHT / 2.0;
    const double CW = 400.0;  // clip window width
    const double CH = 300.0;  // clip window height
    const double XMIN = CX - CW / 2;
    const double YMIN = CY - CH / 2;
    const double XMAX = CX + CW / 2;
    const double YMAX = CY + CH / 2;
    const double clip_area = CW * CH;

    // Image buffers
    std::vector<std::vector<uint8_t>> r(HEIGHT, std::vector<uint8_t>(WIDTH, 20));
    std::vector<std::vector<uint8_t>> g(HEIGHT, std::vector<uint8_t>(WIDTH, 20));
    std::vector<std::vector<uint8_t>> b(HEIGHT, std::vector<uint8_t>(WIDTH, 30));

    // Test scenes:
    // Scene 1: Star polygon that extends beyond the clip window
    // Scene 2: Regular hexagon partially outside
    // Scene 3: Convex polygon that straddles the clip window
    // We'll show all 3 in a grid layout
    
    struct TestCase {
        std::string name;
        std::vector<Point> polygon;
        double offset_x, offset_y;
    };

    std::vector<TestCase> test_cases = {
        {"Star", generateStarPolygon(CX, CY, 220, 100, 5), 0, 0},
        {"Hexagon", generateRegularPolygon(CX, CY, 160, 6), -80, -80},
        {"Triangle", generateRegularPolygon(CX, CY, 200, 3, -M_PI/6), 120, 120},
        {"Concave", std::vector<Point>{
            {420, 300}, {520, 270}, {580, 350}, {540, 430}, {480, 450}, {460, 370}, {400, 410}, {380, 330}
        }, 0, 0},
    };

    // For each test case, clip and verify
    std::vector<std::string> verification_results;
    bool all_passed = true;

    for (size_t t = 0; t < test_cases.size(); t++) {
        auto& tc = test_cases[t];
        
        // Apply offset
        std::vector<Point> adjusted;
        for (const auto& p : tc.polygon) {
            adjusted.push_back(Point(p.x + tc.offset_x, p.y + tc.offset_y));
        }

        // Clip the polygon
        std::vector<Point> clipped = sutherlandHodgman(adjusted, XMIN, YMIN, XMAX, YMAX);

        // --- Verification ---
        double orig_area = polygonArea(adjusted);
        double clipped_area = polygonArea(clipped);
        double orig_bbox = boundingBoxArea(adjusted);

        bool verify_ok = true;
        std::string status = "PASS";

        // Check 1: Clipped polygon must be within clip window
        for (const auto& p : clipped) {
            if (p.x < XMIN - 1.0 || p.x > XMAX + 1.0 || 
                p.y < YMIN - 1.0 || p.y > YMAX + 1.0) {
                status = "FAIL: vertex outside clip window";
                verify_ok = false;
                break;
            }
        }

        // Check 2: Clipped area must be <= clip window area
        if (verify_ok && clipped_area > clip_area + 1.0) {
            status = "FAIL: clipped area exceeds clip window";
            verify_ok = false;
        }

        // Check 3: Clipped area must be <= original area
        if (verify_ok && clipped_area > orig_area + 1.0) {
            status = "FAIL: clipped area exceeds original area";
            verify_ok = false;
        }

        // Check 4: Clipped polygon should not be empty unless completely outside
        if (verify_ok && clipped.size() < 3 && orig_bbox > 0) {
            // Check if polygon overlaps clip window at all
            bool any_inside = false;
            for (const auto& p : adjusted) {
                if (p.x >= XMIN && p.x <= XMAX && p.y >= YMIN && p.y <= YMAX) {
                    any_inside = true;
                    break;
                }
            }
            if (any_inside) {
                status = "FAIL: polygon has vertices inside but clipped is empty";
                verify_ok = false;
            }
        }

        if (!verify_ok) all_passed = false;

        char buf[512];
        snprintf(buf, sizeof(buf),
            "%-10s | Orig area: %9.1f | Clipped area: %9.1f | Vertices: %3zu→%3zu | %s",
            tc.name.c_str(), orig_area, clipped_area, adjusted.size(), clipped.size(), status.c_str());
        verification_results.push_back(buf);

        // --- Visualize ---
        // Draw clip window as a semi-transparent white overlay
        fillRect(r, g, b, XMIN, YMIN, XMAX, YMAX, 50, 50, 50, WIDTH, HEIGHT, 0.15);

        // Draw clip window border in white
        int x0 = (int)XMIN, y0 = (int)YMIN;
        int x1 = (int)XMAX, y1 = (int)YMAX;
        drawLine(r, g, b, x0, y0, x1, y0, 255, 255, 255, WIDTH, HEIGHT);
        drawLine(r, g, b, x1, y0, x1, y1, 255, 255, 255, WIDTH, HEIGHT);
        drawLine(r, g, b, x1, y1, x0, y1, 255, 255, 255, WIDTH, HEIGHT);
        drawLine(r, g, b, x0, y1, x0, y0, 255, 255, 255, WIDTH, HEIGHT);

        // Draw original polygon border in blue
        for (size_t i = 0; i < adjusted.size(); i++) {
            const Point& pa = adjusted[i];
            const Point& pb = adjusted[(i + 1) % adjusted.size()];
            drawLine(r, g, b, (int)pa.x, (int)pa.y, (int)pb.x, (int)pb.y, 
                     100, 140, 255, WIDTH, HEIGHT);
        }

        // Fill original polygon in semi-transparent blue
        fillPolygon(r, g, b, adjusted, 50, 70, 180, WIDTH, HEIGHT, 0.3);

        // Fill clipped polygon in semi-transparent red
        if (clipped.size() >= 3) {
            fillPolygon(r, g, b, clipped, 220, 40, 40, WIDTH, HEIGHT, 0.6);

            // Draw clipped polygon border in bright red
            for (size_t i = 0; i < clipped.size(); i++) {
                const Point& pa = clipped[i];
                const Point& pb = clipped[(i + 1) % clipped.size()];
                drawLine(r, g, b, (int)pa.x, (int)pa.y, (int)pb.x, (int)pb.y,
                         255, 60, 60, WIDTH, HEIGHT);
            }
        }
    }

    // Write output
    writePPM("sutherland_hodgman_output.ppm", r, g, b, WIDTH, HEIGHT);

    // Convert to PNG using ImageMagick if available
    int ret = system("command -v convert > /dev/null 2>&1 && convert sutherland_hodgman_output.ppm sutherland_hodgman_output.png 2>/dev/null");
    const char* output_file = (ret == 0) ? "sutherland_hodgman_output.png" : "sutherland_hodgman_output.ppm";

    // Print verification results
    printf("=== Sutherland-Hodgman Polygon Clipping Verification ===\n");
    printf("Clip window: [%.0f, %.0f] × [%.0f, %.0f] (area=%.0f)\n", XMIN, YMIN, XMAX, YMAX, clip_area);
    printf("%-10s | %-22s | %-22s | %-14s | %s\n", "Test", "Original", "Clipped", "Vertex Change", "Status");
    printf("----------+------------------------+------------------------+----------------+--------\n");
    for (const auto& line : verification_results) {
        printf("%s\n", line.c_str());
    }
    printf("\n");

    // Overall pixel verification on the output image
    printf("=== Pixel Verification ===\n");
    if (ret == 0) {
        // Use ImageMagick identify for verification
        printf("Output file: %s\n", output_file);
        // Check image dimensions with ImageMagick
        FILE* f = popen("identify sutherland_hodgman_output.png 2>/dev/null", "r");
        if (f) {
            char img_info[256];
            if (fgets(img_info, sizeof(img_info), f)) {
                printf("Image info: %s", img_info);
            }
            pclose(f);
        }
    } else {
        printf("Output file: sutherland_hodgman_output.ppm\n");
        printf("(ImageMagick not available, PPM output only)\n");
    }

    if (all_passed) {
        printf("\n✅ ALL TESTS PASSED\n");
    } else {
        printf("\n❌ SOME TESTS FAILED\n");
        return 1;
    }

    return 0;
}
