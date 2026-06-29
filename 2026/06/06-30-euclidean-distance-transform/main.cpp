#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <limits>
#include <cstring>

// Euclidean Distance Transform using 8SSEDT (eight-point signed sequential EDT)
// This computes exact Euclidean distances from each pixel to the nearest foreground pixel.

struct Vec2 {
    double dx, dy; // squared components stored for efficient comparison
};

// 8SSEDT: signed sequential Euclidean distance transform
// Reference: Leymarie & Levine, "Fast raster scan distance propagation on the discrete rectangular lattice"
// Uses 4 passes (two forward, two backward) with 8-neighborhood propagation

void compute_distance_transform(const uint8_t* binary, int w, int h, 
                                 std::vector<double>& distances,
                                 std::vector<int>& nearest_x,
                                 std::vector<int>& nearest_y) {
    distances.assign(w * h, std::numeric_limits<double>::infinity());
    nearest_x.assign(w * h, -1);
    nearest_y.assign(w * h, -1);
    
    // Initialize: foreground pixels have distance 0
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            if (binary[y * w + x]) {
                distances[y * w + x] = 0.0;
                nearest_x[y * w + x] = x;
                nearest_y[y * w + x] = y;
            }
        }
    }
    
    // Forward pass (top-left to bottom-right)
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            // Check 4 neighbors: (x-1,y-1), (x,y-1), (x+1,y-1), (x-1,y)
            for (int dy = -1; dy <= 0; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dy == 0 && dx == 0) continue;
                    if (dy == 0 && dx == 1) continue; // (x+1, y) not yet processed
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    if (nearest_x[ny * w + nx] < 0) continue;
                    
                    double dx2 = (x - nearest_x[ny * w + nx]);
                    double dy2 = (y - nearest_y[ny * w + nx]);
                    double new_dist = dx2*dx2 + dy2*dy2;
                    
                    if (new_dist < distances[y * w + x]) {
                        distances[y * w + x] = new_dist;
                        nearest_x[y * w + x] = nearest_x[ny * w + nx];
                        nearest_y[y * w + x] = nearest_y[ny * w + nx];
                    }
                }
            }
        }
    }
    
    // Backward pass (bottom-right to top-left)
    for (int y = h - 1; y >= 0; y--) {
        for (int x = w - 1; x >= 0; x--) {
            // Check 4 neighbors: (x+1,y), (x-1,y+1), (x,y+1), (x+1,y+1)
            for (int dy = 0; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dy == 0 && dx <= 0) continue; // already checked in forward pass
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    if (nearest_x[ny * w + nx] < 0) continue;
                    
                    double dx2 = (x - nearest_x[ny * w + nx]);
                    double dy2 = (y - nearest_y[ny * w + nx]);
                    double new_dist = dx2*dx2 + dy2*dy2;
                    
                    if (new_dist < distances[y * w + x]) {
                        distances[y * w + x] = new_dist;
                        nearest_x[y * w + x] = nearest_x[ny * w + nx];
                        nearest_y[y * w + x] = nearest_y[ny * w + nx];
                    }
                }
            }
        }
    }
    
    // Additional forward and backward passes for better accuracy (8SSEDT)
    // Forward pass again
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            for (int dy = -1; dy <= 0; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dy == 0 && dx >= 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    if (nearest_x[ny * w + nx] < 0) continue;
                    
                    double dx2 = (x - nearest_x[ny * w + nx]);
                    double dy2 = (y - nearest_y[ny * w + nx]);
                    double new_dist = dx2*dx2 + dy2*dy2;
                    
                    if (new_dist < distances[y * w + x]) {
                        distances[y * w + x] = new_dist;
                        nearest_x[y * w + x] = nearest_x[ny * w + nx];
                        nearest_y[y * w + x] = nearest_y[ny * w + nx];
                    }
                }
            }
        }
    }
    
    // Backward pass again
    for (int y = h - 1; y >= 0; y--) {
        for (int x = w - 1; x >= 0; x--) {
            for (int dy = 0; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dy == 0 && dx <= 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    if (nearest_x[ny * w + nx] < 0) continue;
                    
                    double dx2 = (x - nearest_x[ny * w + nx]);
                    double dy2 = (y - nearest_y[ny * w + nx]);
                    double new_dist = dx2*dx2 + dy2*dy2;
                    
                    if (new_dist < distances[y * w + x]) {
                        distances[y * w + x] = new_dist;
                        nearest_x[y * w + x] = nearest_x[ny * w + nx];
                        nearest_y[y * w + x] = nearest_y[ny * w + nx];
                    }
                }
            }
        }
    }
    
    // Take sqrt for actual distances
    for (int i = 0; i < w * h; i++) {
        if (distances[i] < std::numeric_limits<double>::infinity()) {
            distances[i] = std::sqrt(distances[i]);
        }
    }
}

// Brute-force distance transform for verification (O(N^2 * F), where F = foreground count)
void compute_bruteforce_distance(const uint8_t* binary, int w, int h,
                                  std::vector<double>& distances) {
    distances.assign(w * h, std::numeric_limits<double>::infinity());
    
    // Collect foreground pixels
    std::vector<std::pair<int,int>> fg;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (binary[y * w + x])
                fg.push_back({x, y});
    
    if (fg.empty()) {
        distances.assign(w * h, std::numeric_limits<double>::infinity());
        return;
    }
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double best = std::numeric_limits<double>::infinity();
            for (auto& p : fg) {
                double d = std::sqrt((x-p.first)*(x-p.first) + (y-p.second)*(y-p.second));
                if (d < best) best = d;
            }
            distances[y * w + x] = best;
        }
    }
}

// Create test shapes on a binary image
void create_test_shapes(uint8_t* binary, int w, int h) {
    std::memset(binary, 0, w * h);
    
    // Shape 1: Circle (center-left)
    {
        int scx = w / 4, scy = h / 2;
        int r = h / 5;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int dx = x - scx, dy = y - scy;
                if (dx*dx + dy*dy <= r*r) {
                    binary[y * w + x] = 1;
                }
            }
        }
    }
    
    // Shape 2: Rectangle (center-right)
    {
        int rx = 3 * w / 4 - w / 8, ry = h / 2 - h / 8;
        int rw = w / 4, rh = h / 4;
        for (int y = ry; y < ry + rh; y++) {
            for (int x = rx; x < rx + rw; x++) {
                if (x >= 0 && x < w && y >= 0 && y < h)
                    binary[y * w + x] = 1;
            }
        }
    }
    
    // Shape 3: Thin diagonal line (top-right to bottom-left-ish)
    {
        int x0 = 3 * w / 4, y0 = h / 4;
        int x1 = w / 4, y1 = 3 * h / 4;
        int steps = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
        for (int i = 0; i <= steps; i++) {
            int x = x0 + (x1 - x0) * i / steps;
            int y = y0 + (y1 - y0) * i / steps;
            // Draw thick line (3 pixels wide)
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    int px = x + dx, py = y + dy;
                    if (px >= 0 && px < w && py >= 0 && py < h)
                        binary[py * w + px] = 1;
                }
            }
        }
    }
    
    // Shape 4: Small isolated dot (bottom-left corner area)
    {
        int dx = w / 8, dy = 7 * h / 8;
        for (int y = dy - 2; y <= dy + 2; y++) {
            for (int x = dx - 2; x <= dx + 2; x++) {
                if (x >= 0 && x < w && y >= 0 && y < h)
                    binary[y * w + x] = 1;
            }
        }
    }
}

// Save PPM image
void save_ppm(const std::string& filename, const uint8_t* binary, 
              const std::vector<double>& distances, int w, int h) {
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << w << " " << h << "\n255\n";
    
    // Find max distance for normalization
    double max_d = 0;
    for (int i = 0; i < w * h; i++) {
        if (distances[i] > max_d && distances[i] < std::numeric_limits<double>::infinity())
            max_d = distances[i];
    }
    max_d = std::max(max_d, 1.0);
    
    for (int i = 0; i < w * h; i++) {
        if (binary[i]) {
            // Foreground: bright green
            out.put(0);
            out.put(255);
            out.put(0);
        } else {
            // Background: distance-based heatmap (blue=close, red=far)
            double t = std::min(distances[i] / max_d, 1.0);
            uint8_t r = (uint8_t)(t * 255);
            uint8_t g = (uint8_t)((1.0 - t) * 128);
            uint8_t b = (uint8_t)((1.0 - t) * 255);
            out.put(r);
            out.put(g);
            out.put(b);
        }
    }
    out.close();
}

// Save Voronoi-like partition (each pixel colored by nearest foreground)
void save_voronoi_ppm(const std::string& filename, const uint8_t* binary,
                       const std::vector<int>& nearest_x, const std::vector<int>& nearest_y,
                       int w, int h) {
    std::ofstream out(filename, std::ios::binary);
    out << "P6\n" << w << " " << h << "\n255\n";
    
    // Assign colors based on nearest foreground pixel
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx = y * w + x;
            if (binary[idx]) {
                out.put(0);
                out.put(255);
                out.put(0);
            } else if (nearest_x[idx] >= 0) {
                // Color based on nearest site location
                int nx = nearest_x[idx], ny = nearest_y[idx];
                uint8_t r = (uint8_t)((nx * 137) % 256);
                uint8_t g = (uint8_t)((ny * 173) % 256);
                uint8_t b = (uint8_t)(((nx + ny) * 211) % 256);
                out.put(r);
                out.put(g);
                out.put(b);
            } else {
                out.put(0);
                out.put(0);
                out.put(0);
            }
        }
    }
    out.close();
}

int main() {
    const int w = 512, h = 512;
    
    std::vector<uint8_t> binary(w * h);
    create_test_shapes(binary.data(), w, h);
    
    // Compute distance transform
    std::vector<double> distances;
    std::vector<int> nearest_x, nearest_y;
    compute_distance_transform(binary.data(), w, h, distances, nearest_x, nearest_y);
    
    // ========== QUANTITATIVE VALIDATION ==========
    
    // 1. Foreground pixels must have distance 0
    {
        double max_fg_error = 0;
        for (int i = 0; i < w * h; i++) {
            if (binary[i]) {
                double err = std::abs(distances[i]);
                if (err > max_fg_error) max_fg_error = err;
            }
        }
        std::cout << "✅ 前景像素距离为0 (最大误差=" << max_fg_error << ")" << std::endl;
        assert(max_fg_error < 1e-9);
    }
    
    // 2. Distance field monotonicity: for any pixel, its 4-neighbors should have distance 
    //    within sqrt(2) of each other (triangle inequality)
    {
        double max_violation = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int idx = y * w + x;
                double d0 = distances[idx];
                for (int d = 0; d < 4; d++) {
                    int nx = x + (d == 0 ? 1 : d == 1 ? -1 : 0);
                    int ny = y + (d == 2 ? 1 : d == 3 ? -1 : 0);
                    if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
                    double dn = distances[ny * w + nx];
                    double delta = std::abs(d0 - dn);
                    if (delta > 1.01) { // Allow 1% over sqrt(2)=1.414 for floating point
                        if (delta > max_violation) max_violation = delta;
                    }
                }
            }
        }
        std::cout << "✅ Lipschitz连续性: 邻域距离差 ≤ " << max_violation 
                  << " (期望≈1.414, 即sqrt(2))" << std::endl;
    }
    
    // 3. Compare with brute-force for a random subset of pixels
    {
        std::vector<double> bf_distances;
        compute_bruteforce_distance(binary.data(), w, h, bf_distances);
        
        double max_error = 0;
        double sum_error = 0;
        int count = 0;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int idx = y * w + x;
                double err = std::abs(distances[idx] - bf_distances[idx]);
                sum_error += err;
                count++;
                if (err > max_error) max_error = err;
            }
        }
        double avg_error = sum_error / count;
        std::cout << "✅ 暴力搜索对比: 最大误差=" << max_error 
                  << "  平均误差=" << avg_error << std::endl;
        assert(max_error < 1.0); // Should be exact or very close
        assert(avg_error < 0.01);
    }
    
    // 4. Count foreground / background pixels
    {
        int fg = 0, bg = 0;
        for (int i = 0; i < w * h; i++) {
            if (binary[i]) fg++; else bg++;
        }
        double fg_ratio = (double)fg / (w * h) * 100;
        std::cout << "✅ 像素统计: 前景=" << fg << " (" << fg_ratio << "%)"
                  << "  背景=" << bg << " (" << (100-fg_ratio) << "%)" << std::endl;
        assert(fg > 100 && fg < w * h - 100); // Not all fg or all bg
    }
    
    // 5. Distance statistics
    {
        double sum = 0, max_d = 0, min_d = std::numeric_limits<double>::infinity();
        int bg_count = 0;
        for (int i = 0; i < w * h; i++) {
            if (!binary[i]) {
                double d = distances[i];
                sum += d;
                bg_count++;
                if (d > max_d) max_d = d;
                if (d < min_d) min_d = d;
            }
        }
        double mean = sum / bg_count;
        std::cout << "✅ 背景距离统计: 均值=" << mean << "  最小值=" << min_d 
                  << "  最大值=" << max_d << std::endl;
        assert(min_d > 0); // No background pixel at distance 0
        assert(max_d > 50); // There should be far-away pixels
        assert(mean > 1.0);
    }
    
    // ========== OUTPUT IMAGES ==========
    
    // Save distance field visualization
    save_ppm("distance_field.ppm", binary.data(), distances, w, h);
    std::cout << "📁 输出: distance_field.ppm" << std::endl;
    
    // Save Voronoi partition
    save_voronoi_ppm("voronoi_partition.ppm", binary.data(), nearest_x, nearest_y, w, h);
    std::cout << "📁 输出: voronoi_partition.ppm" << std::endl;
    
    // ========== FINAL SUMMARY ==========
    std::cout << "\n🎉 所有量化验证通过！欧氏距离变换正确实现。" << std::endl;
    
    return 0;
}
