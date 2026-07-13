/**
 * Cohen-Sutherland Line Clipping Algorithm
 * 
 * Clips 2D line segments against a rectangular viewport using region codes (outcodes).
 * 
 * The algorithm divides the plane into 9 regions using a 4-bit outcode:
 *   bit 3 (8): top    - above the viewport
 *   bit 2 (4): bottom - below the viewport
 *   bit 1 (2): right  - to the right of the viewport
 *   bit 0 (1): left   - to the left of the viewport
 * 
 * Features:
 *   - Cohen-Sutherland clipping with outcodes
 *   - Random line generation (100 lines)
 *   - Benchmark: simple brute-force endpoint clipping for comparison
 *   - PPM output visualization
 *   - Quantitative verification of clipping correctness
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <cmath>
#include <cstdint>
#include <algorithm>

// ============ Region Codes ============
const int INSIDE = 0; // 0000
const int LEFT   = 1; // 0001
const int RIGHT  = 2; // 0010
const int BOTTOM = 4; // 0100
const int TOP    = 8; // 1000

// ============ Clipping Window ============
struct ClipRect {
    double xmin, ymin, xmax, ymax;
    
    int computeCode(double x, double y) const {
        int code = INSIDE;
        if (x < xmin) code |= LEFT;
        else if (x > xmax) code |= RIGHT;
        if (y < ymin) code |= BOTTOM;
        else if (y > ymax) code |= TOP;
        return code;
    }
};

// ============ Line Segment ============
struct LineSeg {
    double x0, y0, x1, y1;
};

// ============ Cohen-Sutherland Algorithm ============
// Returns true if any part of the line is visible, false if completely rejected.
// On return, the endpoints are updated to the clipped segment.
bool cohenSutherlandClip(LineSeg& line, const ClipRect& rect) {
    double x0 = line.x0, y0 = line.y0;
    double x1 = line.x1, y1 = line.y1;
    
    int code0 = rect.computeCode(x0, y0);
    int code1 = rect.computeCode(x1, y1);
    
    while (true) {
        // Trivial accept: both endpoints inside
        if ((code0 | code1) == 0) {
            line.x0 = x0; line.y0 = y0;
            line.x1 = x1; line.y1 = y1;
            return true;
        }
        
        // Trivial reject: both endpoints on same outside half-plane
        if (code0 & code1) {
            return false;
        }
        
        // Pick the endpoint outside the clip rectangle
        int codeOut = (code0 != 0) ? code0 : code1;
        double x, y;
        
        // Find intersection point
        // y = y0 + slope * (x - x0), x = x0 + (1/slope) * (y - y0)
        if (codeOut & TOP) {
            x = x0 + (x1 - x0) * (rect.ymax - y0) / (y1 - y0);
            y = rect.ymax;
        } else if (codeOut & BOTTOM) {
            x = x0 + (x1 - x0) * (rect.ymin - y0) / (y1 - y0);
            y = rect.ymin;
        } else if (codeOut & RIGHT) {
            y = y0 + (y1 - y0) * (rect.xmax - x0) / (x1 - x0);
            x = rect.xmax;
        } else { // LEFT
            y = y0 + (y1 - y0) * (rect.xmin - x0) / (x1 - x0);
            x = rect.xmin;
        }
        
        // Replace the outside endpoint with the intersection point
        if (codeOut == code0) {
            x0 = x; y0 = y;
            code0 = rect.computeCode(x0, y0);
        } else {
            x1 = x; y1 = y;
            code1 = rect.computeCode(x1, y1);
        }
    }
}

// ============ Naive brute-force clipping for comparison ============
// Clips by computing the parametric intersection with all 4 edges
bool naiveClip(LineSeg& line, const ClipRect& rect) {
    double x0 = line.x0, y0 = line.y0;
    double x1 = line.x1, y1 = line.y1;
    
    // Liang-Barsky algorithm as reference (simpler param approach)
    double dx = x1 - x0;
    double dy = y1 - y0;
    
    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - rect.xmin, rect.xmax - x0, y0 - rect.ymin, rect.ymax - y0};
    
    double t0 = 0.0, t1 = 1.0;
    
    for (int i = 0; i < 4; i++) {
        if (std::abs(p[i]) < 1e-12) {
            // parallel to edge
            if (q[i] < 0) return false;
        } else {
            double t = q[i] / p[i];
            if (p[i] < 0) {
                if (t > t1) return false;
                if (t > t0) t0 = t;
            } else {
                if (t < t0) return false;
                if (t < t1) t1 = t;
            }
        }
    }
    
    if (t0 > t1) return false;
    line.x0 = x0 + dx * t0;
    line.y0 = y0 + dy * t0;
    line.x1 = x0 + dx * t1;
    line.y1 = y0 + dy * t1;
    return true;
}

// ============ PPM Image ============
struct PPM {
    int w, h;
    std::vector<uint8_t> data; // RGB
    
    PPM(int width, int height) : w(width), h(height), data(width * height * 3, 255) {}
    
    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        int idx = (y * w + x) * 3;
        data[idx] = r; data[idx+1] = g; data[idx+2] = b;
    }
    
    void drawLine(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
        // Bresenham
        int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;
        while (true) {
            setPixel(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    
    void drawRect(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
        drawLine(x0, y0, x1, y0, r, g, b);
        drawLine(x1, y0, x1, y1, r, g, b);
        drawLine(x1, y1, x0, y1, r, g, b);
        drawLine(x0, y1, x0, y0, r, g, b);
    }
    
    bool save(const char* filename) {
        std::ofstream f(filename, std::ios::binary);
        if (!f) return false;
        f << "P6\n" << w << " " << h << "\n255\n";
        f.write(reinterpret_cast<const char*>(data.data()), data.size());
        return true;
    }
};

// ============ Coordinate Mapping ============
struct Viewport {
    int imgW, imgH;
    double worldXmin, worldYmin, worldXmax, worldYmax;
    
    int toScreenX(double wx) const {
        return static_cast<int>((wx - worldXmin) * imgW / (worldXmax - worldXmin));
    }
    int toScreenY(double wy) const {
        // Y flips: world Y↑, screen Y↓
        return imgH - 1 - static_cast<int>((wy - worldYmin) * imgH / (worldYmax - worldYmin));
    }
};

// ============ Main ============
int main() {
    const int IMG_W = 800;
    const int IMG_H = 600;
    const ClipRect clipRect = { -8.0, -6.0, 8.0, 6.0 };
    const Viewport viewport = { IMG_W, IMG_H, -10.0, -8.0, 10.0, 8.0 };
    
    PPM img(IMG_W, IMG_H);
    
    // Draw clipping rectangle
    int cx0 = viewport.toScreenX(clipRect.xmin);
    int cy0 = viewport.toScreenY(clipRect.ymin);
    int cx1 = viewport.toScreenX(clipRect.xmax);
    int cy1 = viewport.toScreenY(clipRect.ymax);
    img.drawRect(cx0, cy0, cx1, cy1, 255, 255, 0); // Yellow clipping rect
    
    // Generate random lines
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-10.0, 10.0);
    
    const int NUM_LINES = 100;
    int cs_visible = 0, cs_rejected = 0;
    int naive_visible = 0, naive_rejected = 0;
    int agreement = 0, disagreement = 0;
    
    std::vector<LineSeg> original_lines;
    
    for (int i = 0; i < NUM_LINES; i++) {
        LineSeg orig = { dist(rng), dist(rng), dist(rng), dist(rng) };
        original_lines.push_back(orig);
        
        LineSeg cs = orig;
        bool cs_vis = cohenSutherlandClip(cs, clipRect);
        if (cs_vis) cs_visible++; else cs_rejected++;
        
        LineSeg naive = orig;
        bool naive_vis = naiveClip(naive, clipRect);
        if (naive_vis) naive_visible++; else naive_rejected++;
        
        if (cs_vis == naive_vis) {
            agreement++;
            if (cs_vis) {
                // Draw the clipped line in green
                int sx0 = viewport.toScreenX(cs.x0), sy0 = viewport.toScreenY(cs.y0);
                int sx1 = viewport.toScreenX(cs.x1), sy1 = viewport.toScreenY(cs.y1);
                img.drawLine(sx0, sy0, sx1, sy1, 0, 255, 0);
                
                // Verify endpoints are inside the clip rect
                bool p0_inside = (cs.x0 >= clipRect.xmin - 1e-9 && cs.x0 <= clipRect.xmax + 1e-9 &&
                                  cs.y0 >= clipRect.ymin - 1e-9 && cs.y0 <= clipRect.ymax + 1e-9);
                bool p1_inside = (cs.x1 >= clipRect.xmin - 1e-9 && cs.x1 <= clipRect.xmax + 1e-9 &&
                                  cs.y1 >= clipRect.ymin - 1e-9 && cs.y1 <= clipRect.ymax + 1e-9);
                if (!p0_inside || !p1_inside) {
                    disagreement++;
                }
            } else {
                // Draw original line in dim red (rejected)
                int sx0 = viewport.toScreenX(orig.x0), sy0 = viewport.toScreenY(orig.y0);
                int sx1 = viewport.toScreenX(orig.x1), sy1 = viewport.toScreenY(orig.y1);
                img.drawLine(sx0, sy0, sx1, sy1, 100, 100, 100);
            }
        } else {
            disagreement++;
            // Draw in red for disagreement
            int sx0 = viewport.toScreenX(orig.x0), sy0 = viewport.toScreenY(orig.y0);
            int sx1 = viewport.toScreenX(orig.x1), sy1 = viewport.toScreenY(orig.y1);
            img.drawLine(sx0, sy0, sx1, sy1, 255, 0, 0);
        }
    }
    
    // Save image
    img.save("cohen_sutherland_output.ppm");
    
    // ============ Quantitative Results ============
    std::cout << "=== Cohen-Sutherland Line Clipping ===" << std::endl;
    std::cout << "Lines generated: " << NUM_LINES << std::endl;
    std::cout << "Clip window: [" << clipRect.xmin << ", " << clipRect.ymin << "] to [" 
              << clipRect.xmax << ", " << clipRect.ymax << "]" << std::endl;
    std::cout << std::endl;
    std::cout << "Cohen-Sutherland: " << cs_visible << " visible, " << cs_rejected << " rejected" << std::endl;
    std::cout << "Naive (Liang-Barsky): " << naive_visible << " visible, " << naive_rejected << " rejected" << std::endl;
    std::cout << std::endl;
    std::cout << "Algorithm agreement: " << agreement << "/" << NUM_LINES 
              << " (" << (100.0 * agreement / NUM_LINES) << "%)" << std::endl;
    std::cout << "Disagreements: " << disagreement << std::endl;
    std::cout << std::endl;
    
    if (disagreement == 0) {
        std::cout << "✅ PASS: Both algorithms produce identical accept/reject results" << std::endl;
    } else {
        std::cout << "❌ FAIL: Algorithms disagree on " << disagreement << " lines" << std::endl;
    }
    
    // Quantify line lengths inside clip region
    double total_inside_len = 0;
    int inside_count = 0;
    for (auto& orig : original_lines) {
        LineSeg cs = orig;
        if (cohenSutherlandClip(cs, clipRect)) {
            double dx = cs.x1 - cs.x0;
            double dy = cs.y1 - cs.y0;
            total_inside_len += std::sqrt(dx*dx + dy*dy);
            inside_count++;
        }
    }
    double avg_len = inside_count > 0 ? total_inside_len / inside_count : 0;
    std::cout << "Average clipped segment length: " << avg_len << std::endl;
    std::cout << "Inside segments: " << inside_count << "/" << NUM_LINES << std::endl;
    
    return (disagreement == 0) ? 0 : 1;
}
