/**
 * Liang-Barsky Line Clipping Algorithm
 *
 * Liang-Barsky uses parametric line equation P(t) = P0 + t*(P1-P0), t ∈ [0,1]
 * and clips against rectangle boundaries using inequality tests:
 *   x_min <= x0 + t*dx <= x_max
 *   y_min <= y0 + t*dy <= y_max
 *
 * This is more efficient than Cohen-Sutherland because it computes
 * intersection parameters directly instead of using region codes and
 * potentially performing intersection calculations.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <random>
#include <cassert>

const int WIDTH = 800;
const int HEIGHT = 600;
const int CLIP_XMIN = 150;
const int CLIP_YMIN = 100;
const int CLIP_XMAX = 650;
const int CLIP_YMAX = 500;

struct AABB {
    double xmin, ymin, xmax, ymax;
};

// Liang-Barsky line clipping algorithm
// Returns true if any part of the line is visible, false if completely outside.
// The clipped endpoints are stored in x0,y0,x1,y1.
bool liang_barsky_clip(double& x0, double& y0, double& x1, double& y1,
                        double xmin, double ymin, double xmax, double ymax) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double t_min = 0.0;
    double t_max = 1.0;

    double p[4] = {-dx, dx, -dy, dy};
    double q[4] = {x0 - xmin, xmax - x0, y0 - ymin, ymax - y0};

    for (int i = 0; i < 4; i++) {
        if (p[i] == 0) {
            if (q[i] < 0) return false;
        } else {
            double t = q[i] / p[i];
            if (p[i] < 0) {
                t_min = std::max(t_min, t);
            } else {
                t_max = std::min(t_max, t);
            }
        }
        if (t_min > t_max) return false;
    }

    double new_x0 = x0 + t_min * dx;
    double new_y0 = y0 + t_min * dy;
    double new_x1 = x0 + t_max * dx;
    double new_y1 = y0 + t_max * dy;

    x0 = new_x0; y0 = new_y0;
    x1 = new_x1; y1 = new_y1;
    return true;
}

// Bresenham line drawing
void draw_line(std::vector<uint8_t>& buffer, int x0, int y0, int x1, int y1,
               uint8_t r, uint8_t g, uint8_t b) {
    int dx = abs(x1 - x0);
    int dy = -abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (x0 >= 0 && x0 < WIDTH && y0 >= 0 && y0 < HEIGHT) {
            int idx = (y0 * WIDTH + x0) * 3;
            buffer[idx] = r;
            buffer[idx + 1] = g;
            buffer[idx + 2] = b;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; if (x0 == x1 && e2 <= dx) break; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Draw clipping rectangle border
void draw_rect(std::vector<uint8_t>& buffer, int xmin, int ymin, int xmax, int ymax,
               uint8_t r, uint8_t g, uint8_t b) {
    for (int x = xmin; x <= xmax; x++) {
        if (x >= 0 && x < WIDTH) {
            int t = (ymin * WIDTH + x) * 3;
            buffer[t] = r; buffer[t+1] = g; buffer[t+2] = b;
            t = (ymax * WIDTH + x) * 3;
            buffer[t] = r; buffer[t+1] = g; buffer[t+2] = b;
        }
    }
    for (int y = ymin; y <= ymax; y++) {
        int l = (y * WIDTH + xmin) * 3;
        buffer[l] = r; buffer[l+1] = g; buffer[l+2] = b;
        int rt = (y * WIDTH + xmax) * 3;
        buffer[rt] = r; buffer[rt+1] = g; buffer[rt+2] = b;
    }
}

// Cohen-Sutherland implementation for comparison verification
const int INSIDE = 0, LEFT = 1, RIGHT = 2, BOTTOM = 4, TOP = 8;

int compute_code(double x, double y, double xmin, double ymin, double xmax, double ymax) {
    int code = INSIDE;
    if (x < xmin) code |= LEFT;
    if (x > xmax) code |= RIGHT;
    if (y < ymin) code |= BOTTOM;
    if (y > ymax) code |= TOP;
    return code;
}

bool cohen_sutherland_clip(double& x0, double& y0, double& x1, double& y1,
                           double xmin, double ymin, double xmax, double ymax) {
    int code0 = compute_code(x0, y0, xmin, ymin, xmax, ymax);
    int code1 = compute_code(x1, y1, xmin, ymin, xmax, ymax);
    while (true) {
        if ((code0 | code1) == 0) return true;
        if ((code0 & code1) != 0) return false;
        int code_out = (code0 != 0) ? code0 : code1;
        double x, y;
        if (code_out & TOP) {
            x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
            y = ymax;
        } else if (code_out & BOTTOM) {
            x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
            y = ymin;
        } else if (code_out & RIGHT) {
            y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
            x = xmax;
        } else {
            y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
            x = xmin;
        }
        if (code_out == code0) {
            x0 = x; y0 = y;
            code0 = compute_code(x0, y0, xmin, ymin, xmax, ymax);
        } else {
            x1 = x; y1 = y;
            code1 = compute_code(x1, y1, xmin, ymin, xmax, ymax);
        }
    }
}

struct VerifyResult {
    int total_lines, both_visible, both_rejected, mismatches;
    double max_endpoint_diff;
    int visible_lb, visible_cs;
};

VerifyResult verify_algorithms(int num_lines, double xmin, double ymin,
                                double xmax, double ymax) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> pos_dist(-200.0, WIDTH + 200.0);

    VerifyResult result = {num_lines, 0, 0, 0, 0.0, 0, 0};

    for (int i = 0; i < num_lines; i++) {
        double lb_x0 = pos_dist(rng), lb_y0 = pos_dist(rng);
        double lb_x1 = pos_dist(rng), lb_y1 = pos_dist(rng);
        double cs_x0 = lb_x0, cs_y0 = lb_y0;
        double cs_x1 = lb_x1, cs_y1 = lb_y1;

        bool lb_v = liang_barsky_clip(lb_x0, lb_y0, lb_x1, lb_y1, xmin, ymin, xmax, ymax);
        bool cs_v = cohen_sutherland_clip(cs_x0, cs_y0, cs_x1, cs_y1, xmin, ymin, xmax, ymax);

        if (lb_v) result.visible_lb++;
        if (cs_v) result.visible_cs++;

        if (lb_v == cs_v) {
            if (lb_v) {
                result.both_visible++;
                double d0 = (lb_x0-cs_x0)*(lb_x0-cs_x0)+(lb_y0-cs_y0)*(lb_y0-cs_y0);
                double d1 = (lb_x1-cs_x1)*(lb_x1-cs_x1)+(lb_y1-cs_y1)*(lb_y1-cs_y1);
                result.max_endpoint_diff = std::max(result.max_endpoint_diff,
                    std::sqrt(std::max(d0, d1)));
            } else {
                result.both_rejected++;
            }
        } else {
            result.mismatches++;
        }
    }
    return result;
}

void write_ppm(const std::string& filename, const std::vector<uint8_t>& buf) {
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    out.write(reinterpret_cast<const char*>(buf.data()), buf.size());
}

void ppm_to_png(const std::string& ppm, const std::string& png) {
    std::string cmd = "python3 -c \"from PIL import Image; img = Image.open('"
        + ppm + "'); img.save('" + png + "')\"";
    system(cmd.c_str());
}

int main() {
    std::vector<uint8_t> buffer(WIDTH * HEIGHT * 3, 10); // dark background

    // Draw clipping rectangle in cyan
    draw_rect(buffer, CLIP_XMIN, CLIP_YMIN, CLIP_XMAX, CLIP_YMAX, 0, 220, 220);

    // Generate test lines
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> dist(-100.0, WIDTH + 100.0);

    struct { double x0, y0, x1, y1; } lines[50];
    for (int i = 0; i < 50; i++) {
        lines[i] = {dist(rng), dist(rng), dist(rng), dist(rng)};
    }

    int visible_count = 0, rejected_count = 0;

    for (const auto& l : lines) {
        double x0 = l.x0, y0 = l.y0, x1 = l.x1, y1 = l.y1;

        // Draw original full line faintly (dark gray)
        draw_line(buffer, (int)x0, (int)y0, (int)x1, (int)y1, 50, 50, 50);

        bool visible = liang_barsky_clip(x0, y0, x1, y1,
                                          CLIP_XMIN, CLIP_YMIN, CLIP_XMAX, CLIP_YMAX);
        if (visible) {
            visible_count++;
            // Draw clipped segment in bright orange
            draw_line(buffer, (int)std::round(x0), (int)std::round(y0),
                      (int)std::round(x1), (int)std::round(y1), 255, 140, 30);
        } else {
            rejected_count++;
            // Mark rejected lines midpoint with gray dot
            int mx = (int)((l.x0 + l.x1) / 2);
            int my = (int)((l.y0 + l.y1) / 2);
            if (mx >= 0 && mx < WIDTH && my >= 0 && my < HEIGHT) {
                for (int dy = -1; dy <= 1; dy++)
                    for (int dx = -1; dx <= 1; dx++) {
                        int px = mx + dx, py = my + dy;
                        if (px >= 0 && px < WIDTH && py >= 0 && py < HEIGHT) {
                            buffer[(py*WIDTH+px)*3] = 180;
                            buffer[(py*WIDTH+px)*3+1] = 60;
                            buffer[(py*WIDTH+px)*3+2] = 60;
                        }
                    }
            }
        }
    }

    write_ppm("liang_barsky_output.ppm", buffer);
    ppm_to_png("liang_barsky_output.ppm", "liang_barsky_output.png");

    // ===== Quantitative Verification =====
    std::cout << "========================================" << std::endl;
    std::cout << "Liang-Barsky Line Clipping - Verification" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Viewport: [" << CLIP_XMIN << "," << CLIP_YMIN << "] to ["
              << CLIP_XMAX << "," << CLIP_YMAX << "]" << std::endl;
    std::cout << "Test lines: " << 50 << std::endl;
    std::cout << "Visible after clip: " << visible_count << std::endl;
    std::cout << "Rejected: " << rejected_count << std::endl;
    std::cout << std::endl;

    VerifyResult vr = verify_algorithms(100000, CLIP_XMIN, CLIP_YMIN,
                                         CLIP_XMAX, CLIP_YMAX);

    std::cout << "--- Correctness vs Cohen-Sutherland (100K random lines) ---" << std::endl;
    std::cout << "Both visible: " << vr.both_visible << std::endl;
    std::cout << "Both rejected: " << vr.both_rejected << std::endl;
    std::cout << "Mismatches: " << vr.mismatches << std::endl;
    std::cout << "Max endpoint diff: " << vr.max_endpoint_diff << std::endl;
    std::cout << "LB visible: " << vr.visible_lb << std::endl;
    std::cout << "CS visible: " << vr.visible_cs << std::endl << std::endl;

    bool all_correct = true;

    if (vr.mismatches > 0) {
        std::cout << "❌ FAIL: " << vr.mismatches << " mismatches vs CS!" << std::endl;
        all_correct = false;
    } else std::cout << "✅ PASS: Zero mismatches vs Cohen-Sutherland" << std::endl;

    if (vr.max_endpoint_diff > 1.0) {
        std::cout << "❌ FAIL: Endpoint diff " << vr.max_endpoint_diff << " > 1.0" << std::endl;
        all_correct = false;
    } else std::cout << "✅ PASS: Max endpoint diff " << vr.max_endpoint_diff << " <= 1.0" << std::endl;

    if (vr.both_visible + vr.both_rejected != vr.total_lines) {
        std::cout << "❌ FAIL: Incomplete classification" << std::endl;
        all_correct = false;
    } else std::cout << "✅ PASS: All 100K lines classified consistently" << std::endl;

    // Pixel stats
    std::cout << std::endl << "--- Pixel Statistics ---" << std::endl;
    double sum = 0, sq_sum = 0;
    int n = WIDTH * HEIGHT * 3;
    for (int i = 0; i < n; i++) { double v = buffer[i]; sum += v; sq_sum += v*v; }
    double mean = sum / n;
    double stddev = std::sqrt(sq_sum / n - mean * mean);
    std::cout << "Mean: " << mean << "  Stddev: " << stddev << std::endl;

    if (mean < 10) { std::cout << "❌ FAIL: Too dark" << std::endl; all_correct = false; }
    else if (mean > 240) { std::cout << "❌ FAIL: Too bright" << std::endl; all_correct = false; }
    else std::cout << "✅ PASS: Mean in [10,240]" << std::endl;

    if (stddev < 5) { std::cout << "❌ FAIL: Uniform" << std::endl; all_correct = false; }
    else std::cout << "✅ PASS: Stddev > 5" << std::endl;

    // Rect border presence
    int rect_px = 0;
    for (int y = CLIP_YMIN; y <= CLIP_YMAX; y++)
        for (int x = CLIP_XMIN; x <= CLIP_XMAX; x++) {
            int i = (y*WIDTH+x)*3;
            if (buffer[i]==0 && buffer[i+1]==220 && buffer[i+2]==220) rect_px++;
        }
    std::cout << "Rect border pixels: " << rect_px << std::endl;
    if (rect_px < 1000) { std::cout << "❌ FAIL: Rect not rendered" << std::endl; all_correct = false; }
    else std::cout << "✅ PASS: Clipping rectangle visible" << std::endl;

    // File size
    std::ifstream pf("liang_barsky_output.png", std::ios::binary | std::ios::ate);
    size_t sz = pf.tellg();
    std::cout << "PNG size: " << sz << " bytes" << std::endl;
    if (sz < 10240) { std::cout << "❌ FAIL: PNG too small" << std::endl; all_correct = false; }
    else std::cout << "✅ PASS: PNG > 10KB" << std::endl;

    std::cout << std::endl;
    if (all_correct) std::cout << "🎉 ALL VERIFICATIONS PASSED" << std::endl;
    else { std::cout << "❌ SOME FAILED" << std::endl; return 1; }
    return 0;
}
