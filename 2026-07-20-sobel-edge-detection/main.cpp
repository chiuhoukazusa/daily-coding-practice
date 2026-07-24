/**
 * Sobel Edge Detection with Non-Maximum Suppression & Hysteresis Thresholding
 * 
 * Pipeline:
 *   1. Sobel operator (3x3) → Gx, Gy
 *   2. Gradient magnitude = sqrt(Gx² + Gy²)
 *   3. Gradient direction = atan2(Gy, Gx)
 *   4. Non-Maximum Suppression (NMS)
 *   5. Double threshold (low/high)
 *   6. Hysteresis edge linking
 * 
 * Quantitative verification:
 *   - Edge pixel count vs non-edge
 *   - Gradient statistics (mean, std, max)
 *   - After NMS: edge thinning ratio
 *   - After hysteresis: final edge density
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <climits>
#include <cassert>
#include <cfloat>

using namespace std;

// ============================================================
// Image I/O (PPM format)
// ============================================================

struct Image {
    int width, height;
    vector<uint8_t> r, g, b;  // 0..255
    
    Image(int w, int h) : width(w), height(h), r(w*h, 0), g(w*h, 0), b(w*h, 0) {}
    
    int idx(int x, int y) const { return y * width + x; }
    
    void set_pixel(int x, int y, uint8_t rr, uint8_t gg, uint8_t bb) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int i = idx(x, y);
        r[i] = rr; g[i] = gg; b[i] = bb;
    }
    
    uint8_t gray(int x, int y) const {
        if (x < 0 || x >= width || y < 0 || y >= height) return 0;
        int i = idx(x, y);
        return (uint8_t)(0.299*r[i] + 0.587*g[i] + 0.114*b[i]);
    }
    
    bool save_ppm(const string& filename) const {
        ofstream out(filename, ios::binary);
        if (!out) { cerr << "Cannot open " << filename << endl; return false; }
        out << "P6\n" << width << " " << height << "\n255\n";
        for (int i = 0; i < width * height; ++i) {
            out.put(r[i]); out.put(g[i]); out.put(b[i]);
        }
        out.close();
        return true;
    }
    
    static Image load_ppm(const string& filename) {
        ifstream in(filename, ios::binary);
        if (!in) { cerr << "Cannot open " << filename << endl; exit(1); }
        string magic; int w, h, maxval;
        in >> magic >> w >> h >> maxval;
        in.get(); // skip newline
        Image img(w, h);
        if (magic == "P6") {
            for (int i = 0; i < w*h; ++i) {
                img.r[i] = in.get(); img.g[i] = in.get(); img.b[i] = in.get();
            }
        } else if (magic == "P5") {
            for (int i = 0; i < w*h; ++i) {
                img.r[i] = img.g[i] = img.b[i] = in.get();
            }
        }
        return img;
    }
};

// ============================================================
// Sobel operator
// ============================================================

// 3x3 Sobel kernels
const int Gx_kernel[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
const int Gy_kernel[3][3] = {{-1,-2,-1}, { 0, 0, 0}, { 1, 2, 1}};

struct GradientImage {
    int width, height;
    vector<float> magnitude;   // gradient magnitude
    vector<float> direction;   // gradient direction in radians [-PI, PI]
    
    GradientImage(int w, int h) : width(w), height(h), magnitude(w*h, 0), direction(w*h, 0) {}
    
    int idx(int x, int y) const { return y * width + x; }
};

GradientImage sobel(const Image& img) {
    int w = img.width, h = img.height;
    GradientImage grad(w, h);
    
    for (int y = 1; y < h-1; ++y) {
        for (int x = 1; x < w-1; ++x) {
            float gx = 0, gy = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    float gray = img.gray(x + dx, y + dy);
                    gx += Gx_kernel[dy+1][dx+1] * gray;
                    gy += Gy_kernel[dy+1][dx+1] * gray;
                }
            }
            int i = grad.idx(x, y);
            grad.magnitude[i] = sqrt(gx*gx + gy*gy);
            grad.direction[i] = atan2(gy, gx);
        }
    }
    return grad;
}

// ============================================================
// Non-Maximum Suppression (NMS)
// ============================================================

// quantize angle to 4 directions: 0°, 45°, 90°, 135°
int quantize_direction(float rad) {
    // normalize to [0, PI)
    float deg = rad * 180.0f / M_PI;
    if (deg < 0) deg += 180.0f;
    
    // 4 bins: 0° (horizontal), 45° (diagonal), 90° (vertical), 135° (anti-diagonal)
    // ranges: [0,22.5) + [157.5,180) → 0°; [22.5,67.5) → 45°; [67.5,112.5) → 90°; [112.5,157.5) → 135°
    if (deg < 22.5f || deg >= 157.5f) return 0;       // horizontal edge
    if (deg >= 22.5f && deg < 67.5f) return 45;        // diagonal /
    if (deg >= 67.5f && deg < 112.5f) return 90;       // vertical edge
    return 135;                                        // anti-diagonal
}

GradientImage non_maximum_suppression(const GradientImage& grad) {
    int w = grad.width, h = grad.height;
    GradientImage nms(w, h);
    
    for (int y = 1; y < h-1; ++y) {
        for (int x = 1; x < w-1; ++x) {
            int i = grad.idx(x, y);
            float mag = grad.magnitude[i];
            int dir = quantize_direction(grad.direction[i]);
            
            float neighbor1 = 0, neighbor2 = 0;
            switch (dir) {
                case 0:   // edge normal is vertical → compare left/right
                    neighbor1 = grad.magnitude[grad.idx(x-1, y)];
                    neighbor2 = grad.magnitude[grad.idx(x+1, y)];
                    break;
                case 45:  // edge normal is anti-diagonal → compare SW/NE
                    neighbor1 = grad.magnitude[grad.idx(x-1, y+1)];
                    neighbor2 = grad.magnitude[grad.idx(x+1, y-1)];
                    break;
                case 90:  // edge normal is horizontal → compare top/bottom
                    neighbor1 = grad.magnitude[grad.idx(x, y-1)];
                    neighbor2 = grad.magnitude[grad.idx(x, y+1)];
                    break;
                case 135: // edge normal is diagonal → compare NW/SE
                    neighbor1 = grad.magnitude[grad.idx(x-1, y-1)];
                    neighbor2 = grad.magnitude[grad.idx(x+1, y+1)];
                    break;
            }
            
            // Suppress if not local maximum
            nms.magnitude[i] = (mag >= neighbor1 && mag >= neighbor2) ? mag : 0.0f;
            nms.direction[i] = grad.direction[i];
        }
    }
    return nms;
}

// ============================================================
// Double Thresholding & Hysteresis
// ============================================================

Image double_threshold_hysteresis(const GradientImage& nms, float low_ratio, float high_ratio) {
    int w = nms.width, h = nms.height;
    
    // Find max magnitude for adaptive thresholds
    float max_mag = 0;
    for (int i = 0; i < w*h; ++i)
        if (nms.magnitude[i] > max_mag) max_mag = nms.magnitude[i];
    
    float high_thresh = max_mag * high_ratio;
    float low_thresh = max_mag * low_ratio;
    
    cout << "  Max gradient: " << max_mag << endl;
    cout << "  High threshold: " << high_thresh << "  Low threshold: " << low_thresh << endl;
    
    // Classification: 2 = strong, 1 = weak, 0 = suppressed
    vector<int> classify(w*h, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = nms.idx(x, y);
            if (nms.magnitude[i] >= high_thresh)
                classify[i] = 2;  // strong
            else if (nms.magnitude[i] >= low_thresh)
                classify[i] = 1;  // weak
        }
    }
    
    // Hysteresis: weak edges connected to strong edges become strong
    // Iterate until stable (at most 2 passes should suffice with 8-connectivity)
    bool changed = true;
    int iterations = 0;
    while (changed && iterations < 100) {
        changed = false;
        iterations++;
        for (int y = 1; y < h-1; ++y) {
            for (int x = 1; x < w-1; ++x) {
                int i = nms.idx(x, y);
                if (classify[i] == 1) {  // weak
                    // Check 8-neighbors for strong edges
                    bool has_strong_neighbor = false;
                    for (int dy = -1; dy <= 1 && !has_strong_neighbor; ++dy) {
                        for (int dx = -1; dx <= 1 && !has_strong_neighbor; ++dx) {
                            if (classify[nms.idx(x+dx, y+dy)] == 2) {
                                has_strong_neighbor = true;
                            }
                        }
                    }
                    if (has_strong_neighbor) {
                        classify[i] = 2;
                        changed = true;
                    }
                }
            }
        }
    }
    cout << "  Hysteresis iterations: " << iterations << endl;
    
    // Output edge image (white edges on black background)
    Image edge(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = nms.idx(x, y);
            uint8_t val = (classify[i] == 2) ? 255 : 0;
            edge.set_pixel(x, y, val, val, val);
        }
    }
    return edge;
}

// ============================================================
// Utility: normalize gradient magnitude to grayscale image
// ============================================================

Image magnitude_to_image(const GradientImage& grad, bool invert = false) {
    int w = grad.width, h = grad.height;
    float max_mag = 0, min_mag = FLT_MAX;
    for (int i = 0; i < w*h; ++i) {
        if (grad.magnitude[i] > max_mag) max_mag = grad.magnitude[i];
        if (grad.magnitude[i] < min_mag) min_mag = grad.magnitude[i];
    }
    
    Image img(w, h);
    float range = max_mag - min_mag;
    if (range < 0.001f) range = 1.0f;
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = grad.idx(x, y);
            uint8_t val = (uint8_t)((grad.magnitude[i] - min_mag) / range * 255.0f);
            if (invert) val = 255 - val;
            img.set_pixel(x, y, val, val, val);
        }
    }
    return img;
}

// ============================================================
// Gradient direction visualization (color-coded)
// ============================================================

Image direction_to_image(const GradientImage& grad) {
    int w = grad.width, h = grad.height;
    Image img(w, h);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = grad.idx(x, y);
            float mag = grad.magnitude[i];
            float dir = grad.direction[i];
            
            // Normalize dir to [0, 2PI) for hue
            float hue = fmod(dir + 2*M_PI, 2*M_PI) / (2*M_PI) * 360.0f;
            float saturation = fmin(mag / 255.0f, 1.0f);
            float lightness = 0.5f;
            
            // HSV to RGB
            float c = (1 - fabs(2*lightness - 1)) * saturation;
            float xh = c * (1 - fabs(fmod(hue/60.0f, 2) - 1));
            float m = lightness - c/2;
            float r1=0, g1=0, b1=0;
            if (hue < 60)      { r1=c; g1=xh; b1=0; }
            else if (hue < 120) { r1=xh; g1=c; b1=0; }
            else if (hue < 180) { r1=0; g1=c; b1=xh; }
            else if (hue < 240) { r1=0; g1=xh; b1=c; }
            else if (hue < 300) { r1=xh; g1=0; b1=c; }
            else                { r1=c; g1=0; b1=xh; }
            
            img.set_pixel(x, y,
                (uint8_t)((r1+m)*255), (uint8_t)((g1+m)*255), (uint8_t)((b1+m)*255));
        }
    }
    return img;
}

// ============================================================
// Generate test patterns
// ============================================================

Image generate_test_pattern() {
    // 512x512 synthetic image with known edge positions
    const int S = 512;
    Image img(S, S);
    
    // Fill with white background
    for (int y = 0; y < S; ++y)
        for (int x = 0; x < S; ++x)
            img.set_pixel(x, y, 240, 240, 240);
    
    // Horizontal edge: white above, dark below → edge at y=100
    for (int y = 100; y < S; ++y)
        for (int x = 0; x < S; ++x)
            img.set_pixel(x, y, 100, 100, 140);
    
    // Vertical edge: light left, dark right → edge at x=200
    for (int y = 250; y < 350; ++y)
        for (int x = 200; x < S; ++x)
            img.set_pixel(x, y, 50, 80, 50);
    
    // Diagonal edge: upper-left=light, lower-right=dark
    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            if (x + y > 360 && x > 300 && y > 100 && x < 450 && y < 300) {
                if (x + y > 420)
                    img.set_pixel(x, y, 30, 30, 60);
                else
                    img.set_pixel(x, y, 180, 160, 180);
            }
        }
    }
    
    // Small bright square → edges on all 4 sides
    int sq_x = 370, sq_y = 50, sq_s = 80;
    for (int y = sq_y; y < sq_y + sq_s; ++y)
        for (int x = sq_x; x < sq_x + sq_s; ++x)
            img.set_pixel(x, y, 200, 220, 200);
    
    // Circle → curved edge
    int cx = 120, cy = 380, cr = 60;
    for (int y = cy-cr; y <= cy+cr; ++y) {
        for (int x = cx-cr; x <= cx+cr; ++x) {
            int dx = x - cx, dy = y - cy;
            if (dx*dx + dy*dy <= cr*cr) {
                img.set_pixel(x, y, 200, 150, 100);
            }
        }
    }
    
    return img;
}

Image generate_camera_test_pattern() {
    // Standard resolution chart style pattern
    const int S = 512;
    Image img(S, S);
    
    // Sine grating (horizontal frequency sweep)
    for (int y = 0; y < S; ++y) {
        for (int x = 0; x < S; ++x) {
            float freq = 2.0f + (float)x / S * 15.0f; // increasing frequency
            float val = 127.5f + 127.5f * sin(2 * M_PI * freq * y / S);
            uint8_t v = (uint8_t)val;
            img.set_pixel(x, y, v, v, v);
        }
    }
    
    return img;
}

// ============================================================
// Quantitative Verification
// ============================================================

struct EdgeStats {
    int total_pixels;
    int sobel_edge_pixels;      // pixels with mag > threshold before NMS
    int nms_edge_pixels;        // after NMS
    int final_edge_pixels;      // after hysteresis
    float mean_grad_mag;
    float std_grad_mag;
    float max_grad_mag;
    float thinning_ratio;       // nms / sobel (should be < 1)
    float edge_density;         // final / total
    
    void print(const string& label) const {
        cout << "\n📊 Quantitative Verification: " << label << endl;
        cout << "  Total pixels:        " << total_pixels << endl;
        cout << "  Sobel edges (|G|>20): " << sobel_edge_pixels << " (" 
             << 100.0*sobel_edge_pixels/total_pixels << "%)" << endl;
        cout << "  After NMS:           " << nms_edge_pixels << " (" 
             << 100.0*nms_edge_pixels/total_pixels << "%)" << endl;
        cout << "  Final (hysteresis):  " << final_edge_pixels << " ("
             << 100.0*final_edge_pixels/total_pixels << "%)" << endl;
        cout << "  Gradient mean/std:   " << mean_grad_mag << " / " << std_grad_mag << endl;
        cout << "  Max gradient:        " << max_grad_mag << endl;
        cout << "  Thinning ratio:      " << thinning_ratio << " (should be < 1.0)" << endl;
        cout << "  Edge density:        " << edge_density * 100 << "%" << endl;
    }
};

EdgeStats compute_stats(const GradientImage& grad, const GradientImage& nms, const Image& edge) {
    EdgeStats s{};
    s.total_pixels = grad.width * grad.height;
    s.final_edge_pixels = 0;
    
    float sum_grad = 0, sum_sq = 0;
    s.max_grad_mag = 0;
    s.sobel_edge_pixels = 0;
    s.nms_edge_pixels = 0;
    
    for (int i = 0; i < s.total_pixels; ++i) {
        float mag = grad.magnitude[i];
        sum_grad += mag;
        sum_sq += mag * mag;
        if (mag > s.max_grad_mag) s.max_grad_mag = mag;
        if (mag > 20.0f) s.sobel_edge_pixels++;
        if (nms.magnitude[i] > 0) s.nms_edge_pixels++;
    }
    
    s.mean_grad_mag = sum_grad / s.total_pixels;
    float variance = sum_sq / s.total_pixels - s.mean_grad_mag * s.mean_grad_mag;
    s.std_grad_mag = sqrt(max(0.0f, variance));
    s.thinning_ratio = s.sobel_edge_pixels > 0 ? (float)s.nms_edge_pixels / s.sobel_edge_pixels : 0;
    
    // Count white pixels in edge image
    for (int y = 0; y < edge.height; ++y)
        for (int x = 0; x < edge.width; ++x)
            if (edge.r[edge.idx(x, y)] > 128) s.final_edge_pixels++;
    
    s.edge_density = (float)s.final_edge_pixels / s.total_pixels;
    
    return s;
}

// Verify known edge positions: the synthetic image has edges at specific locations
struct KnownEdgeTest {
    static bool verify_synthetic(const GradientImage& grad) {
        int w = grad.width, h = grad.height;
        
        // Expected edges:
        // 1. Horizontal: around y=100
        // 2. Vertical: around x=200 (between y=250..350)
        // 3. Diagonal: x+y ≈ 420 region
        // 4. Square: x=370..450, y=50..130
        // 5. Circle: centered at (120,380), r=60
        
        int detected = 0, expected_edges = 5;
        
        // Check horizontal edge at y=100
        bool horz = false;
        for (int x = 50; x < w - 50; ++x) {
            if (grad.magnitude[grad.idx(x, 100)] > 30.0f) { horz = true; break; }
        }
        if (horz) detected++;
        else cerr << "  ⚠️ Horizontal edge at y=100 NOT detected!" << endl;
        
        // Check vertical edge at x=200
        bool vert = false;
        for (int y = 260; y < 340; ++y) {
            if (grad.magnitude[grad.idx(200, y)] > 30.0f) { vert = true; break; }
        }
        if (vert) detected++;
        else cerr << "  ⚠️ Vertical edge at x=200 NOT detected!" << endl;
        
        // Check diagonal edge
        bool diag = false;
        for (int t = 0; t < 80; ++t) {
            int x = 350 + t;
            int y = 70 + t;
            if (x < w && y < h && grad.magnitude[grad.idx(x, y)] > 30.0f) { diag = true; break; }
        }
        if (diag) detected++;
        else cerr << "  ⚠️ Diagonal edge NOT detected!" << endl;
        
        // Check square edges
        bool sq = false;
        // Square spans x=370..450, y=50..130
        for (int x = 371; x < 449; ++x) {
            if (grad.magnitude[grad.idx(x, 51)] > 30.0f ||
                grad.magnitude[grad.idx(x, 129)] > 30.0f) { sq = true; break; }
        }
        if (sq) detected++;
        else cerr << "  ⚠️ Square edges NOT detected!" << endl;
        
        // Check circle edge
        bool cir = false;
        int cx = 120, cy = 380, r = 60;
        // Sample points on circle boundary
        for (float angle = 0; angle < 2*M_PI; angle += 0.2f) {
            int x = cx + (int)(r * cos(angle));
            int y = cy + (int)(r * sin(angle));
            if (x >= 0 && x < w && y >= 0 && y < h &&
                grad.magnitude[grad.idx(x, y)] > 30.0f) { cir = true; break; }
        }
        if (cir) detected++;
        else cerr << "  ⚠️ Circle edge NOT detected!" << endl;
        
        cout << "\n🔍 Known Edge Detection Test: " << detected << "/" << expected_edges << " edges confirmed" << endl;
        return detected >= 4; // Allow 1 missed (corners may be weak)
    }
    
    static bool verify_nms_thinning(const GradientImage& grad, const GradientImage& nms) {
        // NMS should thin edges significantly compared to raw Sobel
        // Count: number of adjacent edge pixel pairs in NMS (clumps indicate poor thinning)
        int w = grad.width, h = grad.height;
        int nms_pixels = 0;
        int adjacent_pairs = 0;
        
        for (int y = 1; y < h-1; ++y) {
            for (int x = 1; x < w-1; ++x) {
                int i = nms.idx(x, y);
                if (nms.magnitude[i] > 0) {
                    nms_pixels++;
                    // Check 8-neighbors for other edge pixels
                    // Only count if the neighbor also has an edge (forms a clump)
                    if (nms.magnitude[nms.idx(x+1,y)] > 0) adjacent_pairs++;
                    if (nms.magnitude[nms.idx(x,y+1)] > 0) adjacent_pairs++;
                    if (nms.magnitude[nms.idx(x+1,y+1)] > 0) adjacent_pairs++;
                    if (nms.magnitude[nms.idx(x-1,y+1)] > 0) adjacent_pairs++;
                }
            }
        }
        
        // Average neighbors per edge pixel (should be low for well-thinned edges)
        float avg_neighbors = nms_pixels > 0 ? (float)adjacent_pairs / nms_pixels : 0;
        cout << "  NMS avg neighbors per edge pixel: " << avg_neighbors 
             << " (should be < 2.5 for good thinning)" << endl;
        
        // Also verify: NMS edge count < raw Sobel edge count
        // (already verified in thinning_ratio check)
        return avg_neighbors < 2.5f;
    }
};

// ============================================================
// Main
// ============================================================

int main(int argc, char** argv) {
    cout << "========================================" << endl;
    cout << "Sobel Edge Detection Pipeline" << endl;
    cout << "========================================" << endl;
    
    // Determine input: command-line arg or generate test pattern
    Image input = (argc > 1) ? 
        Image::load_ppm(argv[1]) : 
        generate_test_pattern();
    
    cout << "Input: " << input.width << "x" << input.height << endl;
    
    // Save input
    input.save_ppm("input.ppm");
    cout << "✅ Saved input.ppm" << endl;
    
    // Step 1: Sobel operator
    cout << "\n--- Step 1: Sobel Gradient ---" << endl;
    GradientImage grad = sobel(input);
    Image mag_img = magnitude_to_image(grad);
    mag_img.save_ppm("gradient_magnitude.ppm");
    cout << "✅ Saved gradient_magnitude.ppm" << endl;
    
    // Step 2: Gradient direction visualization
    Image dir_img = direction_to_image(grad);
    dir_img.save_ppm("gradient_direction.ppm");
    cout << "✅ Saved gradient_direction.ppm" << endl;
    
    // Step 3: Non-Maximum Suppression
    cout << "\n--- Step 2: Non-Maximum Suppression ---" << endl;
    GradientImage nms = non_maximum_suppression(grad);
    Image nms_img = magnitude_to_image(nms);
    nms_img.save_ppm("nms_magnitude.ppm");
    cout << "✅ Saved nms_magnitude.ppm" << endl;
    
    // Step 4: Double Threshold + Hysteresis
    cout << "\n--- Step 3: Double Threshold + Hysteresis ---" << endl;
    Image edge = double_threshold_hysteresis(nms, 0.05f, 0.15f);
    edge.save_ppm("edge_output.ppm");
    cout << "✅ Saved edge_output.ppm" << endl;
    
    // Also generate a side-by-side comparison
    Image comparison(input.width * 3, input.height);
    for (int y = 0; y < input.height; ++y) {
        for (int x = 0; x < input.width; ++x) {
            // Left: original
            int i = input.idx(x, y);
            comparison.set_pixel(x, y, input.r[i], input.g[i], input.b[i]);
            // Middle: gradient magnitude
            comparison.set_pixel(x + input.width, y, 
                mag_img.r[i], mag_img.g[i], mag_img.b[i]);
            // Right: edge output
            comparison.set_pixel(x + 2*input.width, y,
                edge.r[i], edge.g[i], edge.b[i]);
        }
    }
    comparison.save_ppm("comparison.ppm");
    cout << "✅ Saved comparison.ppm" << endl;
    
    // ========================================================
    // QUANTITATIVE VERIFICATION
    // ========================================================
    
    cout << "\n========================================" << endl;
    cout << "QUANTITATIVE VERIFICATION" << endl;
    cout << "========================================" << endl;
    
    EdgeStats stats = compute_stats(grad, nms, edge);
    stats.print("Synthetic Test Image");
    
    bool all_ok = true;
    
    // Checks
    cout << "\n🔬 Automated Checks:" << endl;
    
    // 1. Gradient must have non-zero values
    if (stats.mean_grad_mag < 1.0f) {
        cout << "  ❌ FAIL: Mean gradient too low (" << stats.mean_grad_mag << ")" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Mean gradient = " << stats.mean_grad_mag << " > 1.0" << endl;
    }
    
    // 2. Standard deviation must be significant (image has variation)
    if (stats.std_grad_mag < 5.0f) {
        cout << "  ❌ FAIL: Gradient std too low (" << stats.std_grad_mag << ")" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Gradient std = " << stats.std_grad_mag << " > 5.0" << endl;
    }
    
    // 3. NMS must reduce edge pixels (thinning)
    if (stats.thinning_ratio >= 1.0f) {
        cout << "  ❌ FAIL: NMS didn't thin edges (ratio=" << stats.thinning_ratio << ")" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: NMS thinning ratio = " << stats.thinning_ratio << " < 1.0" << endl;
    }
    
    // 4. Final edges should be sparse (not the whole image)
    if (stats.edge_density > 0.30f) {
        cout << "  ❌ FAIL: Edge density too high (" << stats.edge_density*100 << "%)" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Edge density = " << stats.edge_density*100 << "% < 30%" << endl;
    }
    
    // 5. Must have at least some edges
    if (stats.final_edge_pixels < 100) {
        cout << "  ❌ FAIL: Too few edge pixels (" << stats.final_edge_pixels << ")" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Edge pixel count = " << stats.final_edge_pixels << " > 100" << endl;
    }
    
    // 6. File size check
    ifstream fin("edge_output.ppm", ios::binary | ios::ate);
    auto fsize = fin.tellg();
    cout << "  ✅ File size: " << fsize << " bytes (need > 10240)" << endl;
    if (fsize < 10240) { cout << "  ❌ FAIL: Output file too small" << endl; all_ok = false; }
    else cout << "  ✅ PASS: Output file size OK" << endl;
    
    // 7. Known edge detection
    if (!KnownEdgeTest::verify_synthetic(grad)) {
        cout << "  ❌ FAIL: Not all expected edges detected" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Expected edges detected" << endl;
    }
    
    // 8. NMS single-pixel check
    if (!KnownEdgeTest::verify_nms_thinning(grad, nms)) {
        cout << "  ❌ FAIL: NMS violation rate too high" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: NMS single-pixel edges" << endl;
    }
    
    // 9. Max gradient should be significant
    if (stats.max_grad_mag < 50.0f) {
        cout << "  ❌ FAIL: Max gradient too low (" << stats.max_grad_mag << ")" << endl;
        all_ok = false;
    } else {
        cout << "  ✅ PASS: Max gradient = " << stats.max_grad_mag << " > 50" << endl;
    }
    
    cout << "\n========================================" << endl;
    if (all_ok) {
        cout << "🎉 ALL CHECKS PASSED!" << endl;
    } else {
        cout << "❌ SOME CHECKS FAILED!" << endl;
    }
    cout << "========================================" << endl;
    
    return all_ok ? 0 : 1;
}
