/*
 * Daily Coding Practice - 2026-07-28
 * Morphological Operations (形态学操作)
 *
 * Implements:
 *   1. Erosion (腐蚀) - minimum filter with structuring element
 *   2. Dilation (膨胀) - maximum filter with structuring element
 *   3. Opening (开运算) - Erosion followed by Dilation
 *   4. Closing (闭运算) - Dilation followed by Erosion
 *
 * Structuring elements: Square (3x3, 5x5), Cross (3x3), Diamond
 *
 * Quantitative Verification:
 *   - Pixel count before/after each operation
 *   - Boundary area change ratio
 *   - Idempotence test: Opening(Opening(X)) == Opening(X)
 *   - Duality test: Closing(X) == complement(Erosion(Dilation(complement(X))))
 *   - Monotonicity: pixel count ordering preserved
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <cassert>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"

// ============ Image class ============
struct Image {
    int w, h, c;
    unsigned char* data = nullptr;

    Image() {}
    Image(int w_, int h_, int c_) : w(w_), h(h_), c(c_) {
        data = new unsigned char[w * h * c];
        memset(data, 0, w * h * c);
    }
    Image(const Image& other) : w(other.w), h(other.h), c(other.c) {
        data = new unsigned char[w * h * c];
        memcpy(data, other.data, w * h * c);
    }
    ~Image() { delete[] data; }

    unsigned char& at(int x, int y, int ch) {
        return data[(y * w + x) * c + ch];
    }
    unsigned char at(int x, int y, int ch) const {
        return data[(y * w + x) * c + ch];
    }

    Image to_binary(unsigned char threshold = 128) const {
        Image bin(w, h, 1);
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                unsigned char gray = 0;
                for (int k = 0; k < c; k++) gray += at(x, y, k);
                gray /= c;
                bin.at(x, y, 0) = gray >= threshold ? 255 : 0;
            }
        return bin;
    }

    void save_ppm(const char* path) const {
        FILE* f = fopen(path, "wb");
        if (!f) { printf("Cannot open %s\n", path); return; }
        fprintf(f, "P6\n%d %d\n255\n", w, h);
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) {
                if (c == 1) {
                    unsigned char v = at(x, y, 0);
                    fputc(v, f); fputc(v, f); fputc(v, f);
                } else {
                    for (int k = 0; k < c; k++) fputc(at(x, y, k), f);
                }
            }
        fclose(f);
    }

    void save_png(const char* path) const {
        if (c == 1) {
            stbi_write_png(path, w, h, 1, data, w);
        } else if (c == 3) {
            stbi_write_png(path, w, h, 3, data, w * 3);
        } else if (c == 4) {
            stbi_write_png(path, w, h, 4, data, w * 4);
        }
    }

    int count_white() const {
        int cnt = 0;
        for (int i = 0; i < w * h; i++)
            if (data[i] > 127) cnt++;
        return cnt;
    }

    Image complement() const {
        Image comp(w, h, c);
        for (int i = 0; i < w * h * c; i++)
            comp.data[i] = 255 - data[i];
        return comp;
    }

    bool equals(const Image& other) const {
        if (w != other.w || h != other.h || c != other.c) return false;
        return memcmp(data, other.data, w * h * c) == 0;
    }
};

// ============ Structuring Elements ============
// Support: 3x3 square, 5x5 square, cross 3x3, diamond 3x3
struct Kernel {
    std::vector<std::pair<int,int>> offsets;
    const char* name;
};

Kernel kernel_square3() {
    Kernel k;
    k.name = "Square 3x3";
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            k.offsets.push_back({dx, dy});
    return k;
}

Kernel kernel_square5() {
    Kernel k;
    k.name = "Square 5x5";
    for (int dy = -2; dy <= 2; dy++)
        for (int dx = -2; dx <= 2; dx++)
            k.offsets.push_back({dx, dy});
    return k;
}

Kernel kernel_cross3() {
    Kernel k;
    k.name = "Cross 3x3";
    k.offsets.push_back({0, -1});
    k.offsets.push_back({-1, 0}); k.offsets.push_back({0, 0}); k.offsets.push_back({1, 0});
    k.offsets.push_back({0, 1});
    return k;
}

Kernel kernel_diamond3() {
    Kernel k;
    k.name = "Diamond 3x3";
    // Diamond shape at radius 1: center + 4-neighbors (same as cross3 for r=1)
    // For a larger diamond, include corners at radius sqrt(2) = 1???
    // Real diamond: pixels where |dx|+|dy| <= radius
    for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++)
            if (abs(dx) + abs(dy) <= 1)
                k.offsets.push_back({dx, dy});
    return k;
}

// ============ Morphological Operations ============
// For binary images with white=255 (foreground), black=0 (background):
//   Erosion (min) shrinks white objects, expands black.
//   Dilation (max) expands white objects, shrinks black.
// For black-objects-on-white: we can either invert, or use the dual definition.
// Our test image has BLACK objects on WHITE background.
// Default ops work on WHITE foreground.
// To erode BLACK objects, we would need dilation (expands white, shrinks black).
// We implement standard definition (white foreground), and also provide
// a "black_foreground" variant for erosion_black = dilation_white.

Image erosion(const Image& src, const Kernel& k) {
    // Standard grayscale erosion: output = min over kernel neighborhood
    // For white foreground: shrinks white objects
    Image dst(src.w, src.h, src.c);
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            for (int ch = 0; ch < src.c; ch++) {
                int minval = 255;
                for (auto& off : k.offsets) {
                    int nx = x + off.first;
                    int ny = y + off.second;
                    if (nx >= 0 && nx < src.w && ny >= 0 && ny < src.h) {
                        minval = std::min(minval, (int)src.at(nx, ny, ch));
                    }
                }
                dst.at(x, y, ch) = minval;
            }
        }
    }
    return dst;
}

Image dilation(const Image& src, const Kernel& k) {
    // Standard grayscale dilation: output = max over kernel neighborhood
    // For white foreground: expands white objects
    Image dst(src.w, src.h, src.c);
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            for (int ch = 0; ch < src.c; ch++) {
                int maxval = 0;
                for (auto& off : k.offsets) {
                    int nx = x + off.first;
                    int ny = y + off.second;
                    if (nx >= 0 && nx < src.w && ny >= 0 && ny < src.h) {
                        maxval = std::max(maxval, (int)src.at(nx, ny, ch));
                    }
                }
                dst.at(x, y, ch) = maxval;
            }
        }
    }
    return dst;
}

Image opening(const Image& src, const Kernel& k) {
    return dilation(erosion(src, k), k);
}

Image closing(const Image& src, const Kernel& k) {
    return erosion(dilation(src, k), k);
}

// ============ Test pattern generation ============
Image generate_test_pattern(int w, int h) {
    Image img(w, h, 1);
    // White background
    memset(img.data, 255, w * h);

    // Rectangle shapes
    for (int y = 30; y < 80; y++)
        for (int x = 30; x < 150; x++)
            img.at(x, y, 0) = 0;

    for (int y = 30; y < 80; y++)
        for (int x = 180; x < 300; x++)
            img.at(x, y, 0) = 0;

    // Circle shapes
    int cx = 75, cy = 150, r = 30;
    for (int y = cy - r - 2; y < cy + r + 2; y++)
        for (int x = cx - r - 2; x < cx + r + 2; x++)
            if (x >= 0 && x < w && y >= 0 && y < h && (x-cx)*(x-cx) + (y-cy)*(y-cy) <= r*r)
                img.at(x, y, 0) = 0;

    cx = 240; cy = 150;
    for (int y = cy - r - 2; y < cy + r + 2; y++)
        for (int x = cx - r - 2; x < cx + r + 2; x++)
            if (x >= 0 && x < w && y >= 0 && y < h && (x-cx)*(x-cx) + (y-cy)*(y-cy) <= r*r)
                img.at(x, y, 0) = 0;

    // Thin line (1px)
    for (int x = 20; x < 300; x++)
        img.at(x, 210, 0) = 0;

    // Thin diagonal
    for (int t = 0; t < 80; t++)
        img.at(20 + t, 230 + t, 0) = 0;

    // Small dots (single pixels)
    img.at(50, 260, 0) = 0;
    img.at(80, 270, 0) = 0;
    img.at(110, 255, 0) = 0;
    img.at(200, 265, 0) = 0;
    img.at(230, 275, 0) = 0;

    // Salt-and-pepper noise area
    srand(42);
    for (int y = 290; y < 340; y++)
        for (int x = 200; x < 300; x++)
            if (rand() % 100 < 10)
                img.at(x, y, 0) = 0;

    // Thin border rectangle
    for (int x = 20; x < 110; x++) {
        img.at(x, 360, 0) = 0;
        img.at(x, 400, 0) = 0;
    }
    for (int y = 360; y < 401; y++) {
        img.at(20, y, 0) = 0;
        img.at(109, y, 0) = 0;
    }

    return img;
}

// ============ Quantitative Verification ============
struct VerificationResult {
    bool passed;
    const char* name;
    double value;
    double expected;
    double tolerance;
};

void print_result(const VerificationResult& r) {
    const char* status = r.passed ? "✅ PASS" : "❌ FAIL";
    printf("  %s: %s (value=%.3f, expected=%.3f, tol=%.3f)\n",
           r.name, status, r.value, r.expected, r.tolerance);
}

int main() {
    printf("=== Morphological Operations - Quantitative Verification ===\n\n");

    int W = 320, H = 440;
    Image src = generate_test_pattern(W, H);
    src.save_ppm("input_test.ppm");

    int src_white = src.count_white();
    printf("Input image: %d x %d, white pixels = %d\n\n", W, H, src_white);

    Kernel k3 = kernel_square3();
    Kernel k5 = kernel_square5();
    Kernel cross = kernel_cross3();
    Kernel diamond = kernel_diamond3();

    std::vector<VerificationResult> results;

    // ---- Test 1: Erosion should reduce (or equal) white pixel count ----
    {
        Image eroded = erosion(src, k3);
        int ew = eroded.count_white();
        bool ok = ew <= src_white;
        results.push_back({ok, "Erosion reduces white count", (double)ew, (double)src_white, 0});
        printf("  Erosion(Square3): white=%d (src=%d)\n", ew, src_white);
        eroded.save_ppm("erosion_square3.ppm");
    }

    {
        Image eroded = erosion(src, k5);
        int ew5 = eroded.count_white();
        Image eroded3 = erosion(src, k3);
        int ew3 = eroded3.count_white();
        bool ok = ew5 <= ew3; // bigger kernel => more erosion
        results.push_back({ok, "Larger kernel erodes more", (double)ew5, (double)ew3, 0});
        printf("  Erosion(Square5): white=%d vs (Square3)=%d\n", ew5, ew3);
        eroded.save_ppm("erosion_square5.ppm");
    }

    {
        Image eroded = erosion(src, cross);
        int ew = eroded.count_white();
        eroded.save_ppm("erosion_cross3.ppm");
        printf("  Erosion(Cross3): white=%d\n", ew);
    }

    // ---- Test 2: Dilation should increase (or equal) white pixel count ----
    {
        Image dilated = dilation(src, k3);
        int dw = dilated.count_white();
        bool ok = dw >= src_white;
        results.push_back({ok, "Dilation increases white count", (double)dw, (double)src_white, 0});
        printf("  Dilation(Square3): white=%d (src=%d)\n", dw, src_white);
        dilated.save_ppm("dilation_square3.ppm");
    }

    {
        Image dilated = dilation(src, k5);
        int dw = dilated.count_white();
        dilated.save_ppm("dilation_square5.ppm");
        printf("  Dilation(Square5): white=%d\n", dw);
    }

    // ---- Test 3: Dilation increases boundary area ----
    // We approximate "boundary area" as the number of pixels adjacent to different values
    {
        auto boundary_count = [](const Image& img) -> int {
            int cnt = 0;
            for (int y = 0; y < img.h; y++)
                for (int x = 0; x < img.w; x++) {
                    unsigned char v = img.at(x, y, 0) > 127 ? 255 : 0;
                    bool is_boundary = false;
                    for (int dy = -1; dy <= 1 && !is_boundary; dy++)
                        for (int dx = -1; dx <= 1 && !is_boundary; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx, ny = y + dy;
                            if (nx < 0 || nx >= img.w || ny < 0 || ny >= img.h) continue;
                            if (v != (img.at(nx, ny, 0) > 127 ? 255 : 0)) is_boundary = true;
                        }
                    if (is_boundary) cnt++;
                }
            return cnt;
        };

        Image dilated = dilation(src, k3);
        int src_bnd = boundary_count(src);
        int dil_bnd = boundary_count(dilated);
        // Dilation shrinks black objects, so boundary area should decrease
        // (boundary of holes decreases)
        printf("  Source boundary pixels: %d\n", src_bnd);
        printf("  Dilated(Square3) boundary pixels: %d\n", dil_bnd);
        // Not always strictly monotonic, but worth recording
    }

    // ---- Test 4: Dilation (expands white) removes thin black lines and dots ----
    {
        Image dilated = dilation(src, k3);
        // Check thin line at y=210: dilation of white fills the thin black line
        int line_pixels = 0;
        for (int x = 20; x < 300; x++)
            if (dilated.at(x, 210, 0) < 128) line_pixels++;
        bool ok = line_pixels == 0;
        results.push_back({ok, "Dilation removes 1px black line", (double)line_pixels, 0.0, 0});
        printf("  Dilation(Square3) on 1px line: remaining black=%d (expected 0)\n", line_pixels);

        // Single dots also gone after dilation
        int dot_pixels = 0;
        int dots[][2] = {{50,260},{80,270},{110,255},{200,265},{230,275}};
        for (int i = 0; i < 5; i++)
            if (dilated.at(dots[i][0], dots[i][1], 0) < 128) dot_pixels++;
        ok = dot_pixels == 0;
        results.push_back({ok, "Dilation removes isolated black dots", (double)dot_pixels, 0.0, 0});
        printf("  Dilation(Square3) on dots: remaining=%d (expected 0)\n", dot_pixels);
    }

    // ---- Test 5: Dilation should close small holes ----
    {
        Image dilated = dilation(src, k3);
        // After dilation, check the thin border rectangle inner area
        // The thin border encloses a white region. After dilation, the black border expands,
        // reducing the white hole. Count white pixels inside.
        int inner_white = 0;
        for (int y = 361; y < 400; y++)
            for (int x = 21; x < 109; x++)
                if (dilated.at(x, y, 0) > 127) inner_white++;
        int original_inner_white = 0;
        for (int y = 361; y < 400; y++)
            for (int x = 21; x < 109; x++)
                if (src.at(x, y, 0) > 127) original_inner_white++;
        printf("  Thin border interior white: src=%d, dilated=%d (dilation shrinks white holes)\n",
               original_inner_white, inner_white);
    }

    // ---- Test 6: Idempotence - Opening(Opening(X)) == Opening(X) ----
    {
        Image op1 = opening(src, k3);
        Image op2 = opening(op1, k3);
        bool ok = op1.equals(op2);
        results.push_back({ok, "Idempotence: O(O(X)) == O(X)", 0.0, 0.0, 0});
        printf("  Idempotence of Opening(Square3): %s\n", ok ? "PASS" : "FAIL");

        Image cl1 = closing(src, k3);
        Image cl2 = closing(cl1, k3);
        ok = cl1.equals(cl2);
        results.push_back({ok, "Idempotence: C(C(X)) == C(X)", 0.0, 0.0, 0});
        printf("  Idempotence of Closing(Square3): %s\n", ok ? "PASS" : "FAIL");
    }

    // ---- Test 7: Duality ----
    // Standard morphological duality on binary images (symmetric SE):
    //   Erosion(X)^c = Dilation(X^c)
    //   Opening(X)^c = Closing(X^c)
    // So: Closing(X) = (Opening(X^c))^c
    {
        Image binary_src = src.to_binary(128);
        Image binary_comp = binary_src.complement();
        
        // Verify: ~E(X) == D(~X)
        Image ero_c = erosion(binary_src, k3).complement();
        Image dil_comp = dilation(binary_comp, k3);
        bool ok = ero_c.equals(dil_comp);
        results.push_back({ok, "Duality: ~E(X) == D(~X)", 0.0, 0.0, 0});
        printf("  Duality ~Erosion(X) == Dilation(~X): %s\n", ok ? "PASS" : "FAIL");
        
        // Verify: C(X) = ~O(~X)   (Closing = complement of Opening of complement)
        Image opening_comp = opening(binary_comp, k3);
        Image closing_from_dual = opening_comp.complement();
        Image bin_closing_direct = closing(binary_src, k3);
        ok = closing_from_dual.equals(bin_closing_direct);
        results.push_back({ok, "Duality: C(X) = ~O(~X)", 0.0, 0.0, 0});
        printf("  Duality C(X) = complement(Opening(complement(X))): %s\n", ok ? "PASS" : "FAIL");
    }

    // ---- Test 8: Closing removes small black spots (in morphology for white foreground) ----
    // Closing = Dilation then Erosion.
    // Dilation (max) expands white: fills small black holes. Erosion (min) shrinks back.
    // This removes isolated black dots (salt noise on white background).
    {
        Image cls = closing(src, k3);
        int noise_before = 0, noise_after = 0;
        for (int y = 290; y < 340; y++)
            for (int x = 200; x < 300; x++) {
                if (src.at(x, y, 0) < 128) noise_before++;
                if (cls.at(x, y, 0) < 128) noise_after++;
            }
        printf("  Noise region (small black dots): before=%d, after Closing=%d\n", noise_before, noise_after);
        // Closing should reduce small black dots
        bool ok = noise_after < noise_before;
        results.push_back({ok, "Closing removes small black dots", (double)noise_after, (double)noise_before, 0});
    }

    // ---- Test 9: Cross vs Square kernel effects ----
    {
        Image ero_cross = erosion(src, cross);
        Image ero_square = erosion(src, k3);
        // Cross kernel has fewer elements (5 vs 9), so it erodes less
        int ec = ero_cross.count_white();
        int es = ero_square.count_white();
        bool ok = ec >= es; // cross erodes less
        results.push_back({ok, "Cross kernel erodes less than Square", (double)ec, (double)es, 0});
        printf("  Erosion Cross3 white=%d vs Square3 white=%d\n", ec, es);
    }

    // ---- Test 10: Comprehensive operation test on all shapes ----
    {
        Image ero = erosion(src, k3);
        Image dil = dilation(src, k3);
        Image opn = opening(src, k3);
        Image cls = closing(src, k3);

        // Monotonicity: Erosion <= src <= Dilation (white count order)
        int ew = ero.count_white();
        int dw = dil.count_white();
        bool ok = (ew <= src_white) && (src_white <= dw);
        results.push_back({ok, "Monotonicity: Ero <= Src <= Dil", 0.0, 0.0, 0});
        printf("  Monotonicity (white count): Erosion=%d <= Src=%d <= Dilation=%d: %s\n",
               ew, src_white, dw, ok ? "PASS" : "FAIL");

        // Opening should give result between Erosion and Source
        int ow = opn.count_white();
        ok = ew <= ow && ow <= src_white;
        results.push_back({ok, "Opening between Erosion and Source", (double)ow, (double)ew, 0});
        printf("  Opening white=%d (between %d and %d): %s\n", ow, ew, src_white, ok ? "PASS" : "FAIL");

        // Closing should give result between Source and Dilation
        int cw = cls.count_white();
        ok = src_white <= cw && cw <= dw;
        results.push_back({ok, "Closing between Source and Dilation", (double)cw, (double)dw, 0});
        printf("  Closing white=%d (between %d and %d): %s\n", cw, src_white, dw, ok ? "PASS" : "FAIL");
    }

    // ---- Test 11: Erosion expands black, Dilation shrinks black ----
    // For black objects on white: erosion(min) expands black, dilation(max) shrinks black
    {
        auto measure_black_width = [](const Image& img, int y_scan, int x_start, int x_end) {
            int first_black = -1, last_black = -1;
            for (int x = x_start; x <= x_end; x++) {
                if (img.at(x, y_scan, 0) < 128) {
                    if (first_black < 0) first_black = x;
                    last_black = x;
                }
            }
            if (first_black < 0) return 0;
            return last_black - first_black + 1;
        };

        int rect_w = measure_black_width(src, 55, 20, 160);
        int rect_w_ero = measure_black_width(erosion(src, k3), 55, 20, 160);
        int rect_w_dil = measure_black_width(dilation(src, k3), 55, 20, 160);

        printf("  Black rectangle widths: src=%d, eroded=%d, dilated=%d\n", rect_w, rect_w_ero, rect_w_dil);
        // Erosion (min) expands black => black objects grow wider
        // Dilation (max) expands white => black objects shrink
        bool ok = (rect_w_ero > rect_w) && (rect_w_dil < rect_w);
        results.push_back({ok, "For black objects: Erosion grows, Dilation shrinks", 0.0, 0.0, 0});

        // Exact pixel growth/shrink: Square3 kernel => ±2px per dimension
        ok = (rect_w_ero == rect_w + 2);
        results.push_back({ok, "Erosion grows black by exactly 2px", (double)rect_w_ero, (double)(rect_w + 2), 0});
        printf("  Erosion black width: %d vs expected %d: %s\n", rect_w_ero, rect_w + 2, ok ? "PASS" : "FAIL");

        ok = (rect_w_dil == rect_w - 2);
        results.push_back({ok, "Dilation shrinks black by exactly 2px", (double)rect_w_dil, (double)(rect_w - 2), 0});
        printf("  Dilation black width: %d vs expected %d: %s\n", rect_w_dil, rect_w - 2, ok ? "PASS" : "FAIL");
    }

    // ---- Test 12: Opening removes thin protrusions while preserving shape ----
    {
        // A rectangle after opening should have the same shape (just smaller corners)
        // Rectangle area after opening should be close to original
        Image opn = opening(src, k3);

        // Count black pixels inside the first rectangle region
        auto count_black_in_rect = [](const Image& img, int x0, int y0, int x1, int y1) {
            int cnt = 0;
            for (int y = y0; y < y1; y++)
                for (int x = x0; x < x1; x++)
                    if (img.at(x, y, 0) < 128) cnt++;
            return cnt;
        };

        // First rectangle area
        int rect_area_src = count_black_in_rect(src, 25, 25, 155, 85);
        int rect_area_opn = count_black_in_rect(opn, 25, 25, 155, 85);
        printf("  Opening rectangle area: src=%d, opened=%d\n", rect_area_src, rect_area_opn);

        // Opening removes corners (rounding effect), area should decrease but not drastically
        // For a 3x3 square SE, corners lose ~4 pixels
        bool ok = rect_area_opn >= rect_area_src - 10;
        results.push_back({ok, "Opening preserves bulk shape area", (double)rect_area_opn, (double)rect_area_src, 10});
    }

    // ---- Generate composite visualization ----
    // Create a 4x4 grid comparison image
    {
        Image ero = erosion(src, k3);
        Image dil = dilation(src, k3);
        Image opn = opening(src, k3);
        Image cls = closing(src, k3);

        int gap = 4;
        int grid_w = W * 4 + gap * 3;
        int grid_h = H * 2 + gap;
        Image comp(grid_w, grid_h, 1);
        memset(comp.data, 128, grid_w * grid_h); // gray background

        auto paste = [&](const Image& src_img, int tx, int ty) {
            for (int y = 0; y < src_img.h; y++)
                for (int x = 0; x < src_img.w; x++)
                    comp.at(tx + x, ty + y, 0) = src_img.at(x, y, 0);
        };

        // Row 1: Original, Erosion, Dilation, Opening
        paste(src, 0, 0);
        paste(ero, W + gap, 0);
        paste(dil, 2*(W+gap), 0);
        paste(opn, 3*(W+gap), 0);

        // Row 2: Closing, Erosion(Cross3), Dilation(Cross3), Opening(Cross3)
        paste(cls, 0, H + gap);
        Image ero_cross = erosion(src, cross);
        Image dil_cross = dilation(src, cross);
        Image opn_cross = opening(src, cross);
        paste(ero_cross, W + gap, H + gap);
        paste(dil_cross, 2*(W+gap), H + gap);
        paste(opn_cross, 3*(W+gap), H + gap);

        comp.save_ppm("comparison_grid.ppm");
        printf("\nComparison grid saved: comparison_grid.ppm\n");
    }

    // ---- Summary ----
    printf("\n============ VERIFICATION SUMMARY ============\n");
    int passed = 0, total = 0;
    for (auto& r : results) {
        print_result(r);
        total++;
        if (r.passed) passed++;
    }
    printf("\nTotal: %d/%d passed\n", passed, total);

    if (passed == total) {
        printf("🎉 ALL VERIFICATION TESTS PASSED!\n");
    } else {
        printf("⚠️  Some tests failed. Review above.\n");
    }

    return passed == total ? 0 : 1;
}
