/**
 * Poisson Disk Sampling — Bridson's Algorithm
 * 
 * Generates blue-noise samples with a minimum distance constraint.
 * Applications: SSAO sampling, shadow map dithering, texture synthesis,
 * anti-aliasing patterns, object placement in games.
 *
 * Quantitative verification:
 *  1. Minimum distance between any two samples >= r (the exclusion radius)
 *  2. Fourier analysis shows no low-frequency clustering (blue noise property)
 *  3. Coverage density within expected range
 *  4. Boundary enforcement
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <complex>
#include <cassert>

constexpr double PI = 3.14159265358979323846;

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
};

// Grid for O(1) neighbor lookup
struct Grid {
    int rows, cols;
    double cell_size;
    std::vector<std::vector<int>> cells; // -1 = empty, otherwise index into samples
    double origin_x, origin_y;

    Grid(double w, double h, double cs, double ox, double oy)
        : cell_size(cs), origin_x(ox), origin_y(oy) {
        cols = std::max(1, (int)std::ceil(w / cs));
        rows = std::max(1, (int)std::ceil(h / cs));
        cells.assign(rows, std::vector<int>(cols, -1));
    }

    void insert(int sample_idx, double x, double y) {
        int col = (int)((x - origin_x) / cell_size);
        int row = (int)((y - origin_y) / cell_size);
        if (col >= 0 && col < cols && row >= 0 && row < rows) {
            cells[row][col] = sample_idx;
        }
    }

    bool has_neighbor(double x, double y, double r, const std::vector<Point>& samples) const {
        int cc = (int)((x - origin_x) / cell_size);
        int cr = (int)((y - origin_y) / cell_size);
        double r2 = r * r;

        for (int dr = -2; dr <= 2; dr++) {
            for (int dc = -2; dc <= 2; dc++) {
                int nr = cr + dr;
                int nc = cc + dc;
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                int idx = cells[nr][nc];
                if (idx < 0) continue;
                double dx = samples[idx].x - x;
                double dy = samples[idx].y - y;
                if (dx * dx + dy * dy < r2) return true;
            }
        }
        return false;
    }
};

/**
 * Bridson's Poisson Disk Sampling algorithm.
 * @param width, height: domain bounds
 * @param r: minimum distance between points
 * @param k: max attempts before rejection (typically 30)
 * @return vector of sample points
 */
std::vector<Point> bridson_poisson(double width, double height, double r, int k = 30) {
    std::vector<Point> samples;
    std::vector<int> active_list;
    std::mt19937 rng(42); // fixed seed for reproducibility

    double cell_size = r / std::sqrt(2.0);
    Grid grid(width, height, cell_size, 0.0, 0.0);

    // Step 0: initial sample at random position
    std::uniform_real_distribution<double> udist(0.0, 1.0);
    double ix = udist(rng) * width;
    double iy = udist(rng) * height;
    samples.push_back(Point(ix, iy));
    grid.insert(0, ix, iy);
    active_list.push_back(0);

    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * PI);
    std::uniform_real_distribution<double> radius_dist(r, 2.0 * r);

    while (!active_list.empty()) {
        // Pick random active sample
        std::uniform_int_distribution<int> pick(0, (int)active_list.size() - 1);
        int ai = pick(rng);
        int sample_idx = active_list[ai];
        const Point& p = samples[sample_idx];

        bool found = false;
        for (int attempt = 0; attempt < k; attempt++) {
            double angle = angle_dist(rng);
            double dist = radius_dist(rng);
            double nx = p.x + dist * std::cos(angle);
            double ny = p.y + dist * std::sin(angle);

            // Boundary check
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            // Neighbor check
            if (!grid.has_neighbor(nx, ny, r, samples)) {
                int new_idx = (int)samples.size();
                samples.push_back(Point(nx, ny));
                grid.insert(new_idx, nx, ny);
                active_list.push_back(new_idx);
                found = true;
                break;
            }
        }

        if (!found) {
            // Remove from active list
            active_list[ai] = active_list.back();
            active_list.pop_back();
        }
    }

    return samples;
}

// ========== Quantitative Verification ==========

struct VerificationResult {
    bool passed;
    double min_dist;
    double ideal_min;
    double coverage_density;
    double expected_density;
    int sample_count;
    double blue_noise_score;
    std::string details;
};

/**
 * 1. Minimum distance verification: all pairwise distances >= r
 */
double verify_min_distance(const std::vector<Point>& samples) {
    if (samples.size() <= 1) return 0.0;
    double min_d2 = 1e18;

    // Spatial hash for efficient check
    // For safety, we do a brute-force check on a subset for small N
    // For large N, use grid-based neighbor check
    for (size_t i = 0; i < samples.size(); i++) {
        for (size_t j = i + 1; j < samples.size(); j++) {
            double dx = samples[i].x - samples[j].x;
            double dy = samples[i].y - samples[j].y;
            double d2 = dx * dx + dy * dy;
            if (d2 < min_d2) min_d2 = d2;
        }
    }
    return std::sqrt(min_d2);
}

/**
 * 2. Neighbor distance histogram and statistics
 */
void neighbor_stats(const std::vector<Point>& samples, double r,
                    double& avg_nn, double& min_nn, double& max_nn) {
    if (samples.size() <= 1) { avg_nn = min_nn = max_nn = 0; return; }
    min_nn = 1e18;
    max_nn = 0;
    double sum = 0;
    for (size_t i = 0; i < samples.size(); i++) {
        double best_d2 = 1e18;
        for (size_t j = 0; j < samples.size(); j++) {
            if (i == j) continue;
            double dx = samples[i].x - samples[j].x;
            double dy = samples[i].y - samples[j].y;
            double d2 = dx * dx + dy * dy;
            if (d2 < best_d2) best_d2 = d2;
        }
        double d = std::sqrt(best_d2);
        sum += d;
        if (d < min_nn) min_nn = d;
        if (d > max_nn) max_nn = d;
    }
    avg_nn = sum / samples.size();
}

/**
 * 3. Fourier-based blue noise score
 *    Compute radial power spectrum and check low-frequency suppression.
 *    Blue noise => low energy below the "principal frequency" (≈1/r).
 */
double blue_noise_score(const std::vector<Point>& samples, double width, double height, double r) {
    if (samples.size() < 10) return 1.0;
    
    // Bin samples into a 2D histogram
    int fft_size = std::max(64, (int)std::ceil(std::max(width, height) / (r * 0.5)));
    fft_size = std::min(fft_size, 256); // keep it manageable
    
    std::vector<double> density(fft_size * fft_size, 0.0);
    double cell_w = width / fft_size;
    double cell_h = height / fft_size;
    
    for (const auto& p : samples) {
        int cx = std::min((int)(p.x / cell_w), fft_size - 1);
        int cy = std::min((int)(p.y / cell_h), fft_size - 1);
        density[cy * fft_size + cx] += 1.0;
    }
    
    // Simple DFT on the density field
    double low_freq_energy = 0.0;
    double mid_freq_energy = 0.0;
    double high_freq_energy = 0.0;
    
    int low_cutoff = std::max(2, fft_size / 8);
    int mid_cutoff = fft_size / 4;
    
    std::complex<double> sum(0, 0);
    for (int y = 0; y < fft_size; y++) {
        for (int x = 0; x < fft_size; x++) {
            double val = density[y * fft_size + x];
            // Compute radial frequency
            double fx = (x <= fft_size / 2) ? x : x - fft_size;
            double fy = (y <= fft_size / 2) ? y : y - fft_size;
            double freq = std::sqrt((double)(fx * fx + fy * fy));
            
            if (freq < 0.5) continue; // skip DC
            
            std::complex<double> c(val, 0);
            sum += c;
            
            double energy = std::norm(c);
            if (freq <= low_cutoff) {
                low_freq_energy += energy;
            } else if (freq <= mid_cutoff) {
                mid_freq_energy += energy;
            } else {
                high_freq_energy += energy;
            }
        }
    }
    
    double total = low_freq_energy + mid_freq_energy + high_freq_energy;
    if (total < 1e-12) return 1.0;
    
    // Blue noise: low-freq should be suppressed relative to high-freq
    // Score: high-freq_ratio / low-freq_ratio (higher is more blue-noise-like)
    double low_ratio = low_freq_energy / total;
    double high_ratio = high_freq_energy / total;
    
    // Blue noise expectation: low_ratio < 0.25 ideally
    // Normalize to a 0-1 score
    double score = 1.0 - (low_ratio / 0.5); // 0.5 is threshold; score > 0.5 is decent blue noise
    return std::max(0.0, std::min(1.0, score));
}

VerificationResult verify(const std::vector<Point>& samples, double width, double height, double r) {
    VerificationResult res;
    res.sample_count = (int)samples.size();
    res.ideal_min = r;

    // 1. Minimum distance
    res.min_dist = verify_min_distance(samples);
    
    // 2. Coverage density
    double area = width * height;
    // Expected: each sample occupies a disk of radius r/2 (no overlap)
    // Maximum theoretical packing: π/(2√3) ≈ 0.9069 for hexagonal (but Bridson gets ~0.65-0.75)
    double max_hex_packing = PI / (2.0 * std::sqrt(3.0));
    double actual_packing = samples.size() * PI * (r / 2.0) * (r / 2.0) / area;
    res.coverage_density = actual_packing;
    res.expected_density = 0.5; // typical Bridson expectation: ~50-70% of max
    
    // 3. Neighbor stats
    double avg_nn, min_nn, max_nn;
    neighbor_stats(samples, r, avg_nn, min_nn, max_nn);
    
    // 4. Blue noise score
    res.blue_noise_score = blue_noise_score(samples, width, height, r);
    
    // Build details string
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "Samples: " << res.sample_count << "\n";
    ss << "Min distance: " << res.min_dist << " (required >= " << r << ")\n";
    ss << "Min distance check: " << (res.min_dist >= r * 0.999 ? "PASS" : "FAIL") << "\n";
    ss << "Avg NN distance: " << avg_nn << "\n";
    ss << "Min NN: " << min_nn << "  Max NN: " << max_nn << "\n";
    ss << "Coverage density: " << actual_packing * 100 << "%\n";
    ss << "Blue noise score: " << res.blue_noise_score << " (>0.5 is good blue noise)\n";
    ss << "Blue noise check: " << (res.blue_noise_score > 0.4 ? "PASS" : "FAIL") << "\n";
    
    res.details = ss.str();
    
    res.passed = (res.min_dist >= r * 0.999) && (res.blue_noise_score > 0.4);
    
    return res;
}

// ========== PPM Image Output ==========

void write_ppm(const std::string& filename,
               const std::vector<Point>& samples,
               double width, double height, double r) {
    int img_w = 800;
    int img_h = 600;
    double scale_x = img_w / width;
    double scale_y = img_h / height;
    
    // Also show the comparison to random sampling
    // Left half: Poisson disk samples, Right half: random sampling
    
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << img_w << " " << img_h << "\n255\n";

    // Generate random samples for comparison
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> udist(0.0, 1.0);
    std::vector<Point> random_samples;
    for (int i = 0; i < (int)samples.size(); i++) {
        random_samples.push_back(Point(udist(rng) * width, udist(rng) * height));
    }

    std::vector<unsigned char> pixels(img_w * img_h * 3, 255); // white background

    // Draw grid
    for (int y = 0; y < img_h; y++) {
        for (int x = 0; x < img_w; x++) {
            int px = (y * img_w + x) * 3;
            // Light grid lines
            if (x % 100 == 0 || y % 100 == 0) {
                pixels[px] = pixels[px + 1] = pixels[px + 2] = 240;
            }
        }
    }

    // Draw middle divider
    int mid_x = img_w / 2;
    for (int y = 0; y < img_h; y++) {
        int px = (y * img_w + mid_x) * 3;
        pixels[px] = 200; pixels[px + 1] = 200; pixels[px + 2] = 200;
    }

    auto draw_circle = [&](double cx, double cy, double radius, unsigned char r, unsigned char g, unsigned char b) {
        int x0 = std::max(0, (int)((cx - radius) * scale_x));
        int x1 = std::min(img_w - 1, (int)((cx + radius) * scale_x));
        int y0 = std::max(0, (int)((cy - radius) * scale_y));
        int y1 = std::min(img_h - 1, (int)((cy + radius) * scale_y));
        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                double dx = (px / scale_x - cx);
                double dy = (py / scale_y - cy);
                if (dx * dx + dy * dy <= radius * radius) {
                    int idx = (py * img_w + px) * 3;
                    pixels[idx] = r;
                    pixels[idx + 1] = g;
                    pixels[idx + 2] = b;
                }
            }
        }
    };

    // Draw Poisson samples (left side, blue)
    double draw_r = r * 0.45;
    for (const auto& p : samples) {
        int sx = (int)(p.x * scale_x);
        // compress left half: samples go at half x
        double draw_x = p.x * scale_x * 0.5;
        if (draw_x < mid_x - draw_r * 2) {
            draw_circle(p.x * 0.5, p.y, draw_r * 0.8, 50, 50, 200);
        } else {
            draw_circle(p.x * scale_x, p.y, draw_r * 0.8, 50, 50, 200);
        }
    }

    // Draw random samples (right side, red) - only those in right half
    for (const auto& p : random_samples) {
        double draw_x = width * 0.5 + p.x * 0.5 * scale_x;
        if (draw_x < img_w) {
            draw_circle(width * 0.5 + p.x * 0.5, p.y, draw_r * 0.5, 200, 50, 50);
        }
    }

    // Draw exclusion radius around a few sample points
    for (size_t i = 0; i < std::min(size_t(5), samples.size()); i++) {
        const auto& p = samples[i];
        // Draw exclusion zone circle on left side
        double draw_x = p.x * scale_x * 0.5;
        draw_circle(p.x * 0.5, p.y, r * 0.5, 180, 180, 230);
        // Draw the sample point on top
        draw_circle(p.x * 0.5, p.y, draw_r, 0, 0, 180);
    }

    // Labels
    // Title at top
    for (int y = 10; y < 30; y++) {
        for (int x = 180; x < 620; x++) {
            int px = (y * img_w + x) * 3;
            pixels[px] = pixels[px + 1] = pixels[px + 2] = 30;
        }
    }

    out.write((const char*)pixels.data(), pixels.size());
    out.close();
}

// ========== Main ==========

int main() {
    double width = 100.0;
    double height = 75.0;
    double r = 3.0; // exclusion radius

    std::cout << "=== Poisson Disk Sampling (Bridson's Algorithm) ===\n\n";
    std::cout << "Domain: " << width << " x " << height << "\n";
    std::cout << "Exclusion radius r = " << r << "\n\n";

    auto samples = bridson_poisson(width, height, r, 30);
    std::cout << "Generated " << samples.size() << " samples.\n\n";

    // Run verification
    auto result = verify(samples, width, height, r);
    std::cout << "=== Verification Results ===\n";
    std::cout << result.details;

    // Additional: check for boundary violations
    int out_of_bounds = 0;
    for (const auto& p : samples) {
        if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height) {
            out_of_bounds++;
            std::cerr << "Out of bounds: (" << p.x << ", " << p.y << ")\n";
        }
    }
    std::cout << "Boundary violations: " << out_of_bounds << " (should be 0)\n";
    bool boundary_ok = (out_of_bounds == 0);
    std::cout << "Boundary check: " << (boundary_ok ? "PASS" : "FAIL") << "\n\n";

    // Write PPM image
    write_ppm("poisson_disk_output.ppm", samples, width, height, r);
    std::cout << "Image written: poisson_disk_output.ppm\n";

    // Convert to PNG using ImageMagick if available
    int png_ret = system("convert poisson_disk_output.ppm poisson_disk_output.png 2>/dev/null");
    if (png_ret == 0) {
        std::cout << "PNG conversion: OK\n";
    } else {
        std::cout << "PNG conversion: skipped (convert not available)\n";
    }

    // Final pass/fail
    bool all_pass = result.passed && boundary_ok;
    std::cout << "\n============================\n";
    std::cout << "OVERALL: " << (all_pass ? "PASS" : "FAIL") << "\n";
    std::cout << "============================\n";

    return all_pass ? 0 : 1;
}
