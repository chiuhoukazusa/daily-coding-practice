#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <iomanip>
#include <sstream>

// ============================================================
// Halton Sequence Quasi-Monte Carlo
// ============================================================
// Generates Halton low-discrepancy sequences and compares
// them against pseudo-random sequences:
// 1. Visualization (PPM images)
// 2. Star discrepancy computation
// 3. Monte Carlo integration comparison (π estimation)
// ============================================================

const int WIDTH  = 512;
const int HEIGHT = 512;
const int NUM_POINTS = 1024;

// ---- Halton sequence generator ----
double halton(int index, int base) {
    double result = 0.0;
    double f = 1.0 / static_cast<double>(base);
    int i = index + 1;  // 1-indexed
    while (i > 0) {
        result += f * (i % base);
        i /= base;
        f /= static_cast<double>(base);
    }
    return result;
}

// ---- Pseudo-random number generator ----
std::mt19937 rng(42);  // fixed seed for reproducibility

// ---- Write a PPM image ----
void write_ppm(const std::string &filename,
               const std::vector<double> &x,
               const std::vector<double> &y,
               int r, int g, int b,
               bool draw_grid = true) {
    // Create buffer
    std::vector<unsigned char> img(WIDTH * HEIGHT * 3, 255);
    
    // Draw grid lines
    if (draw_grid) {
        for (int i = 0; i < WIDTH; i++) {
            for (int gy = 0; gy <= 10; gy++) {
                int py = gy * HEIGHT / 10;
                if (py < HEIGHT) {
                    int idx = (py * WIDTH + i) * 3;
                    img[idx] = 220; img[idx+1] = 220; img[idx+2] = 220;
                }
            }
            for (int gx = 0; gx <= 10; gx++) {
                int px = gx * WIDTH / 10;
                if (px < WIDTH) {
                    int idx = (i * WIDTH + px) * 3;
                    img[idx] = 220; img[idx+1] = 220; img[idx+2] = 220;
                }
            }
        }
    }
    
    // Draw points (3x3 pixels each for visibility)
    int n = std::min((int)x.size(), NUM_POINTS);
    for (int i = 0; i < n; i++) {
        int px = static_cast<int>(x[i] * (WIDTH - 1));
        int py = static_cast<int>((1.0 - y[i]) * (HEIGHT - 1)); // flip Y
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int sx = px + dx;
                int sy = py + dy;
                if (sx >= 0 && sx < WIDTH && sy >= 0 && sy < HEIGHT) {
                    int idx = (sy * WIDTH + sx) * 3;
                    img[idx] = static_cast<unsigned char>(r);
                    img[idx+1] = static_cast<unsigned char>(g);
                    img[idx+2] = static_cast<unsigned char>(b);
                }
            }
        }
    }
    
    // Write PPM
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), img.size());
    f.close();
}

// ---- L2 star discrepancy (simplified) ----
// Measures how uniformly points are distributed in [0,1]^2
// Smaller value = more uniform
double l2_discrepancy(const std::vector<double> &x,
                      const std::vector<double> &y,
                      int n) {
    // Use a grid of sub-rectangles to approximate L2 discrepancy
    // L2 discrepancy measures deviation from uniform distribution
    double result = 0.0;
    int grid = 50; // 50x50 grid of sub-rectangles
    
    for (int gx = 0; gx < grid; gx++) {
        for (int gy = 0; gy < grid; gy++) {
            double vol = ((gx + 1.0) / grid) * ((gy + 1.0) / grid);
            int count = 0;
            double bx = (gx + 1.0) / grid;
            double by = (gy + 1.0) / grid;
            for (int i = 0; i < n; i++) {
                if (x[i] <= bx && y[i] <= by) {
                    count++;
                }
            }
            double frac = static_cast<double>(count) / n;
            double diff = frac - vol;
            result += diff * diff;
        }
    }
    return std::sqrt(result / (grid * grid));
}

// ---- Star discrepancy (simplified approximation) ----
// Star discrepancy is sup|A(B)/N - vol(B)| over axis-aligned boxes [0,a]x[0,b]
double star_discrepancy_approx(const std::vector<double> &x,
                               const std::vector<double> &y,
                               int n) {
    double max_diff = 0.0;
    int grid = 100; // finer grid for better approximation
    
    for (int gx = 1; gx <= grid; gx++) {
        for (int gy = 1; gy <= grid; gy++) {
            double a = static_cast<double>(gx) / grid;
            double b = static_cast<double>(gy) / grid;
            double vol = a * b;
            int count = 0;
            for (int i = 0; i < n; i++) {
                if (x[i] <= a && y[i] <= b) {
                    count++;
                }
            }
            double diff = std::abs(static_cast<double>(count) / n - vol);
            if (diff > max_diff) max_diff = diff;
        }
    }
    return max_diff;
}

// ---- Monte Carlo π estimation ----
double estimate_pi_mc(const std::vector<double> &x,
                      const std::vector<double> &y,
                      int n) {
    int inside = 0;
    for (int i = 0; i < n; i++) {
        double dx = x[i] - 0.5;
        double dy = y[i] - 0.5;
        if (dx*dx + dy*dy <= 0.25) {
            inside++;
        }
    }
    return 4.0 * static_cast<double>(inside) / n;
}

// ---- Composite test function integration ----
// ∫_0^1∫_0^1 sin(π*x)*cos(π*y) dx dy = 4/π² ≈ 0.405284734...
double integrate_test_func(const std::vector<double> &x,
                           const std::vector<double> &y,
                           int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += std::sin(M_PI * x[i]) * std::cos(M_PI * y[i]);
    }
    return sum / n;
}

// ---- Binning uniformity check ----
// Compute chi-squared statistic for uniformity
double chi_squared_uniformity(const std::vector<double> &x,
                               const std::vector<double> &y,
                               int n, int bins) {
    std::vector<int> counts(bins * bins, 0);
    double expected = static_cast<double>(n) / (bins * bins);
    
    for (int i = 0; i < n; i++) {
        int bx = std::min(static_cast<int>(x[i] * bins), bins - 1);
        int by = std::min(static_cast<int>(y[i] * bins), bins - 1);
        counts[by * bins + bx]++;
    }
    
    double chi2 = 0.0;
    for (int c : counts) {
        double diff = c - expected;
        chi2 += diff * diff / expected;
    }
    return chi2;
}

// ---- Gap / clustering metric ----
double nearest_neighbor_mean(const std::vector<double> &x,
                              const std::vector<double> &y,
                              int n) {
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        double min_dist = 1e9;
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            double dx = x[i] - x[j];
            double dy = y[i] - y[j];
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < min_dist && dist > 0) min_dist = dist;
        }
        sum += min_dist;
    }
    return sum / n;
}

// ---- Generate combined comparison image ----
void write_comparison_image(const std::vector<double> &halton_x,
                            const std::vector<double> &halton_y,
                            const std::vector<double> &random_x,
                            const std::vector<double> &random_y,
                            int n) {
    // Left: Halton (blue), Right: Random (red)
    // The image is WIDTH*2 wide, HEIGHT tall
    std::vector<unsigned char> img(WIDTH * 2 * HEIGHT * 3, 255);
    
    // Draw grid on both halves
    for (int half = 0; half < 2; half++) {
        int offset_x = half * WIDTH;
        for (int i = 0; i < WIDTH; i++) {
            for (int gy = 0; gy <= 10; gy++) {
                int py = gy * HEIGHT / 10;
                if (py < HEIGHT) {
                    int idx = (py * WIDTH * 2 + offset_x + i) * 3;
                    img[idx] = 230; img[idx+1] = 230; img[idx+2] = 230;
                }
            }
            for (int gx = 0; gx <= 10; gx++) {
                int px = gx * WIDTH / 10;
                if (px < WIDTH) {
                    int idx = (i * WIDTH * 2 + offset_x + px) * 3;
                    img[idx] = 230; img[idx+1] = 230; img[idx+2] = 230;
                }
            }
        }
    }
    
    // Draw Halton (left half, blue)
    for (int i = 0; i < n; i++) {
        int px = static_cast<int>(halton_x[i] * (WIDTH - 1));
        int py = static_cast<int>((1.0 - halton_y[i]) * (HEIGHT - 1));
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int sx = px + dx;
                int sy = py + dy;
                if (sx >= 0 && sx < WIDTH && sy >= 0 && sy < HEIGHT) {
                    int idx = (sy * WIDTH * 2 + sx) * 3;
                    img[idx] = 40; img[idx+1] = 80; img[idx+2] = 220;
                }
            }
        }
    }
    
    // Draw Random (right half, red)
    for (int i = 0; i < n; i++) {
        int px = static_cast<int>(random_x[i] * (WIDTH - 1)) + WIDTH;
        int py = static_cast<int>((1.0 - random_y[i]) * (HEIGHT - 1));
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                int sx = px + dx;
                int sy = py + dy;
                if (sx >= 0 && sx < WIDTH * 2 && sy >= 0 && sy < HEIGHT) {
                    int idx = (sy * WIDTH * 2 + sx) * 3;
                    img[idx] = 220; img[idx+1] = 60; img[idx+2] = 60;
                }
            }
        }
    }
    
    std::ofstream f("comparison.ppm", std::ios::binary);
    f << "P6\n" << (WIDTH * 2) << " " << HEIGHT << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data()), img.size());
    f.close();
}

int main() {
    std::cout << "================================================" << std::endl;
    std::cout << "Halton Sequence Quasi-Monte Carlo" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "Number of points: " << NUM_POINTS << std::endl;
    std::cout << std::endl;
    
    // Generate Halton sequences (base 2 and base 3 for 2D)
    std::vector<double> halton_x(NUM_POINTS);
    std::vector<double> halton_y(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        halton_x[i] = halton(i, 2);
        halton_y[i] = halton(i, 3);
    }
    
    // Generate pseudo-random sequences
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    std::vector<double> random_x(NUM_POINTS);
    std::vector<double> random_y(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        random_x[i] = dist(rng);
        random_y[i] = dist(rng);
    }
    
    // ---- Metrics ----
    std::cout << "--- Discrepancy Metrics (lower = more uniform) ---" << std::endl;
    
    double halton_l2 = l2_discrepancy(halton_x, halton_y, NUM_POINTS);
    double random_l2 = l2_discrepancy(random_x, random_y, NUM_POINTS);
    std::cout << "L2 Discrepancy:" << std::endl;
    std::cout << "  Halton: " << std::setprecision(6) << halton_l2 << std::endl;
    std::cout << "  Random: " << std::setprecision(6) << random_l2 << std::endl;
    std::cout << "  Halton improvement: " << std::setprecision(2)
              << (random_l2 / halton_l2) << "x better" << std::endl;
    
    double halton_star = star_discrepancy_approx(halton_x, halton_y, NUM_POINTS);
    double random_star = star_discrepancy_approx(random_x, random_y, NUM_POINTS);
    std::cout << "Star Discrepancy (approx):" << std::endl;
    std::cout << "  Halton: " << std::setprecision(6) << halton_star << std::endl;
    std::cout << "  Random: " << std::setprecision(6) << random_star << std::endl;
    std::cout << "  Halton improvement: " << std::setprecision(2)
              << (random_star / halton_star) << "x better" << std::endl;
    
    std::cout << std::endl;
    std::cout << "--- Uniformity Metrics ---" << std::endl;
    double halton_chi2 = chi_squared_uniformity(halton_x, halton_y, NUM_POINTS, 10);
    double random_chi2 = chi_squared_uniformity(random_x, random_y, NUM_POINTS, 10);
    std::cout << "Chi-squared uniformity (10x10 bins):" << std::endl;
    std::cout << "  Halton: " << std::setprecision(3) << halton_chi2 << std::endl;
    std::cout << "  Random: " << std::setprecision(3) << random_chi2 << std::endl;
    std::cout << "  (Halton should be LOWER than Random for better uniformity)" << std::endl;
    
    double halton_nn = nearest_neighbor_mean(halton_x, halton_y, NUM_POINTS);
    double random_nn = nearest_neighbor_mean(random_x, random_y, NUM_POINTS);
    std::cout << "Mean nearest-neighbor distance:" << std::endl;
    std::cout << "  Halton: " << std::setprecision(6) << halton_nn << std::endl;
    std::cout << "  Random: " << std::setprecision(6) << random_nn << std::endl;
    std::cout << "  (Halton should be HIGHER = less clustering)" << std::endl;
    
    std::cout << std::endl;
    std::cout << "--- Monte Carlo Integration Test ---" << std::endl;
    std::cout << "True π = 3.141592653589793" << std::endl;
    double halton_pi = estimate_pi_mc(halton_x, halton_y, NUM_POINTS);
    double random_pi = estimate_pi_mc(random_x, random_y, NUM_POINTS);
    std::cout << "π via Halton QMC: " << std::setprecision(8) << halton_pi
              << " (error: " << std::setprecision(6) << std::abs(halton_pi - M_PI) << ")" << std::endl;
    std::cout << "π via Random MC: " << std::setprecision(8) << random_pi
              << " (error: " << std::setprecision(6) << std::abs(random_pi - M_PI) << ")" << std::endl;
    std::cout << "  Halton error improvement: " << std::setprecision(2)
              << (std::abs(random_pi - M_PI) / std::max(std::abs(halton_pi - M_PI), 1e-12)) << "x better" << std::endl;
    
    std::cout << std::endl;
    std::cout << "--- Test Function Integration ---" << std::endl;
    double true_val = 4.0 / (M_PI * M_PI);
    std::cout << "True ∫₀¹∫₀¹ sin(πx)·cos(πy) dxdy = " << std::setprecision(8) << true_val << std::endl;
    double halton_int = integrate_test_func(halton_x, halton_y, NUM_POINTS);
    double random_int = integrate_test_func(random_x, random_y, NUM_POINTS);
    std::cout << "Halton QMC: " << std::setprecision(8) << halton_int
              << " (error: " << std::setprecision(6) << std::abs(halton_int - true_val) << ")" << std::endl;
    std::cout << "Random MC:  " << std::setprecision(8) << random_int
              << " (error: " << std::setprecision(6) << std::abs(random_int - true_val) << ")" << std::endl;
    std::cout << "  Halton error improvement: " << std::setprecision(2)
              << (std::abs(random_int - true_val) / std::max(std::abs(halton_int - true_val), 1e-12)) << "x better" << std::endl;
    
    // ---- Generate images ----
    write_ppm("halton_sequence.ppm", halton_x, halton_y, 40, 80, 200, true);
    std::cout << "Generated halton_sequence.ppm" << std::endl;
    
    write_ppm("random_sequence.ppm", random_x, random_y, 200, 40, 40, true);
    std::cout << "Generated random_sequence.ppm" << std::endl;
    
    write_comparison_image(halton_x, halton_y, random_x, random_y, NUM_POINTS);
    std::cout << "Generated comparison.ppm" << std::endl;
    
    // ---- Automated quantitative verification ----
    std::cout << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "AUTOMATED VERIFICATION" << std::endl;
    std::cout << "================================================" << std::endl;
    
    bool all_pass = true;
    
    // Check 1: Halton should have lower L2 discrepancy
    std::cout << "Check 1: L2 Discrepancy (Halton < Random)? ";
    if (halton_l2 < random_l2) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 2: Halton should have lower star discrepancy
    std::cout << "Check 2: Star Discrepancy (Halton < Random)? ";
    if (halton_star < random_star) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 3: Halton should have lower chi-squared (more uniform bin distribution)
    std::cout << "Check 3: Chi-squared (Halton < Random)? ";
    if (halton_chi2 < random_chi2) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 4: Halton should have larger mean NN distance (less clustering)
    std::cout << "Check 4: Mean NN distance (Halton > Random)? ";
    if (halton_nn > random_nn) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 5: Halton π estimation should be more accurate
    std::cout << "Check 5: π estimation accuracy (Halton < Random error)? ";
    double halton_err = std::abs(halton_pi - M_PI);
    double random_err = std::abs(random_pi - M_PI);
    if (halton_err < random_err) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL (Halton error: " << halton_err << ", Random error: " << random_err << ")" << std::endl;
        all_pass = false;
    }
    
    // Check 6: Halton test function integration should be more accurate
    std::cout << "Check 6: Test function accuracy (Halton < Random error)? ";
    double halton_ferr = std::abs(halton_int - true_val);
    double random_ferr = std::abs(random_int - true_val);
    if (halton_ferr < random_ferr) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 7: Image files exist and have reasonable size
    std::cout << "Check 7: Image file generation? ";
    std::ifstream f1("halton_sequence.ppm", std::ios::binary | std::ios::ate);
    std::ifstream f2("random_sequence.ppm", std::ios::binary | std::ios::ate);
    std::ifstream f3("comparison.ppm", std::ios::binary | std::ios::ate);
    auto s1 = f1.tellg(); auto s2 = f2.tellg(); auto s3 = f3.tellg();
    std::cout << "(sizes: " << s1 << ", " << s2 << ", " << s3 << " bytes) ";
    if (s1 > 1000 && s2 > 1000 && s3 > 1000) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    // Check 8: Halton mean should be close to 0.5
    double hx_mean = 0.0, hy_mean = 0.0;
    for (int i = 0; i < NUM_POINTS; i++) { hx_mean += halton_x[i]; hy_mean += halton_y[i]; }
    hx_mean /= NUM_POINTS; hy_mean /= NUM_POINTS;
    std::cout << "Check 8: Halton mean ≈ 0.5? (x=" << std::setprecision(4) << hx_mean
              << ", y=" << hy_mean << ") ";
    if (std::abs(hx_mean - 0.5) < 0.05 && std::abs(hy_mean - 0.5) < 0.05) {
        std::cout << "✅ PASS" << std::endl;
    } else {
        std::cout << "❌ FAIL" << std::endl;
        all_pass = false;
    }
    
    std::cout << "================================================" << std::endl;
    if (all_pass) {
        std::cout << "RESULT: ✅ ALL CHECKS PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "RESULT: ❌ SOME CHECKS FAILED" << std::endl;
        return 1;
    }
}
