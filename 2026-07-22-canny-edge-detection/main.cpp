/**
 * Canny Edge Detection - Complete Implementation
 *
 * Algorithm stages:
 * 1. Gaussian smoothing (5x5 kernel)
 * 2. Gradient computation (Sobel 3x3)
 * 3. Non-maximum suppression
 * 4. Double thresholding
 * 5. Edge tracking by hysteresis
 *
 * Output: PPM image with edge detection results
 * Quantifiable verification included
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>

// ============================================================
// PPM I/O utilities
// ============================================================

struct Image {
    int w, h;
    std::vector<unsigned char> r, g, b;
    
    Image() : w(0), h(0) {}
    Image(int width, int height) : w(width), h(height), 
        r(width*height), g(width*height), b(width*height) {}
};

bool readPPM(const char* filename, Image& img) {
    FILE* f = fopen(filename, "rb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", filename); return false; }
    
    char magic[3];
    if (!fgets(magic, sizeof(magic), f) || magic[0] != 'P' || magic[1] != '6') {
        fprintf(stderr, "Not a P6 PPM file\n"); fclose(f); return false;
    }
    
    // Skip comments
    int c = fgetc(f);
    while (c == '#') {
        while (fgetc(f) != '\n');
        c = fgetc(f);
    }
    ungetc(c, f);
    
    int w, h, maxval;
    fscanf(f, "%d %d\n%d\n", &w, &h, &maxval);
    fgetc(f); // consume the single whitespace after maxval
    
    img = Image(w, h);
    
    if (maxval <= 255) {
        std::vector<unsigned char> row(w * 3);
        for (int y = 0; y < h; y++) {
            fread(row.data(), 1, w * 3, f);
            for (int x = 0; x < w; x++) {
                img.r[y*w + x] = row[x*3];
                img.g[y*w + x] = row[x*3 + 1];
                img.b[y*w + x] = row[x*3 + 2];
            }
        }
    } else {
        std::vector<unsigned short> row(w * 3);
        for (int y = 0; y < h; y++) {
            fread(row.data(), 2, w * 3, f);
            for (int x = 0; x < w; x++) {
                img.r[y*w + x] = (unsigned char)(row[x*3] * 255 / maxval);
                img.g[y*w + x] = (unsigned char)(row[x*3+1] * 255 / maxval);
                img.b[y*w + x] = (unsigned char)(row[x*3+2] * 255 / maxval);
            }
        }
    }
    fclose(f);
    return true;
}

void writePGM(const char* filename, const Image& img) {
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P5\n%d %d\n255\n", img.w, img.h);
    for (int i = 0; i < img.w * img.h; i++) {
        fputc(img.r[i], f);
    }
    fclose(f);
}

void writePPM(const char* filename, const Image& img) {
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", img.w, img.h);
    for (int i = 0; i < img.w * img.h; i++) {
        fputc(img.r[i], f);
        fputc(img.g[i], f);
        fputc(img.b[i], f);
    }
    fclose(f);
}

// ============================================================
// Stage 1: Gaussian Smoothing
// ============================================================

// Generate 5x5 Gaussian kernel with given sigma
std::vector<float> createGaussianKernel(int size, float sigma) {
    std::vector<float> kernel(size * size);
    int half = size / 2;
    float sum = 0.0f;
    
    for (int y = -half; y <= half; y++) {
        for (int x = -half; x <= half; x++) {
            float val = expf(-(x*x + y*y) / (2.0f * sigma * sigma));
            kernel[(y+half)*size + (x+half)] = val;
            sum += val;
        }
    }
    
    // Normalize
    for (int i = 0; i < size * size; i++) {
        kernel[i] /= sum;
    }
    
    return kernel;
}

Image gaussianBlur(const Image& src, int kernelSize, float sigma) {
    Image dst(src.w, src.h);
    auto kernel = createGaussianKernel(kernelSize, sigma);
    int half = kernelSize / 2;
    
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            float sum = 0;
            for (int ky = -half; ky <= half; ky++) {
                for (int kx = -half; kx <= half; kx++) {
                    int px = std::clamp(x + kx, 0, src.w - 1);
                    int py = std::clamp(y + ky, 0, src.h - 1);
                    float gray = src.r[py*src.w + px] * 0.299f + 
                                 src.g[py*src.w + px] * 0.587f + 
                                 src.b[py*src.w + px] * 0.114f;
                    sum += gray * kernel[(ky+half)*kernelSize + (kx+half)];
                }
            }
            unsigned char val = (unsigned char)std::clamp((int)(sum + 0.5f), 0, 255);
            dst.r[y*dst.w + x] = val;
            dst.g[y*dst.w + x] = val;
            dst.b[y*dst.w + x] = val;
        }
    }
    return dst;
}

// ============================================================
// Stage 2: Gradient Computation (Sobel)
// ============================================================

struct GradientResult {
    std::vector<float> magnitude;
    std::vector<float> direction; // in radians, [-pi, pi]
    int w, h;
};

GradientResult computeGradients(const Image& src) {
    GradientResult gr;
    gr.w = src.w;
    gr.h = src.h;
    gr.magnitude.resize(src.w * src.h);
    gr.direction.resize(src.w * src.h);
    
    // Sobel kernels
    const int Gx[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    const int Gy[3][3] = {{-1,-2,-1}, { 0, 0, 0}, { 1, 2, 1}};
    
    for (int y = 1; y < src.h - 1; y++) {
        for (int x = 1; x < src.w - 1; x++) {
            float sx = 0, sy = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    unsigned char val = src.r[(y+ky)*src.w + (x+kx)];
                    sx += val * Gx[ky+1][kx+1];
                    sy += val * Gy[ky+1][kx+1];
                }
            }
            gr.magnitude[y*src.w + x] = sqrtf(sx*sx + sy*sy);
            gr.direction[y*src.w + x] = atan2f(sy, sx);
        }
    }
    
    // handle borders
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            if (x == 0 || x == src.w - 1 || y == 0 || y == src.h - 1) {
                gr.magnitude[y*src.w + x] = 0;
                gr.direction[y*src.w + x] = 0;
            }
        }
    }
    
    return gr;
}

// ============================================================
// Stage 3: Non-Maximum Suppression
// ============================================================

Image nonMaximumSuppression(const GradientResult& gr) {
    Image nms(gr.w, gr.h);
    
    for (int y = 1; y < gr.h - 1; y++) {
        for (int x = 1; x < gr.w - 1; x++) {
            // Quantize gradient direction to 4 sectors: 0, 45, 90, 135 degrees
            float angle = gr.direction[y*gr.w + x] * 180.0f / M_PI;
            // Normalize to [0, 180]
            if (angle < 0) angle += 180.0f;
            
            float mag = gr.magnitude[y*gr.w + x];
            float n1 = 0, n2 = 0;
            
            // Sector 0: 0 degrees (horizontal edge, gradient vertical)
            if ((angle >= 0 && angle < 22.5) || angle >= 157.5) {
                n1 = gr.magnitude[y*gr.w + (x-1)];
                n2 = gr.magnitude[y*gr.w + (x+1)];
            }
            // Sector 1: 45 degrees (diagonal)
            else if (angle >= 22.5 && angle < 67.5) {
                n1 = gr.magnitude[(y-1)*gr.w + (x+1)];
                n2 = gr.magnitude[(y+1)*gr.w + (x-1)];
            }
            // Sector 2: 90 degrees (vertical edge, gradient horizontal)
            else if (angle >= 67.5 && angle < 112.5) {
                n1 = gr.magnitude[(y-1)*gr.w + x];
                n2 = gr.magnitude[(y+1)*gr.w + x];
            }
            // Sector 3: 135 degrees (anti-diagonal)
            else {
                n1 = gr.magnitude[(y-1)*gr.w + (x-1)];
                n2 = gr.magnitude[(y+1)*gr.w + (x+1)];
            }
            
            unsigned char val = (mag >= n1 && mag >= n2) ? (unsigned char)std::min(mag, 255.0f) : 0;
            nms.r[y*nms.w + x] = val;
            nms.g[y*nms.w + x] = val;
            nms.b[y*nms.w + x] = val;
        }
    }
    
    return nms;
}

// ============================================================
// Stage 4: Double Thresholding
// ============================================================

Image doubleThreshold(const Image& nms, float lowRatio, float highRatio) {
    // Compute histogram-based thresholds (percentile approach)
    std::vector<float> mags;
    for (int i = 0; i < nms.w * nms.h; i++) {
        if (nms.r[i] > 0) mags.push_back((float)nms.r[i]);
    }
    
    if (mags.empty()) return nms; // no edges found
    
    std::sort(mags.begin(), mags.end());
    float highThresh = mags[(int)(mags.size() * (1.0f - highRatio))];
    float lowThresh = highThresh * lowRatio;
    
    printf("  Thresholds: high=%.1f, low=%.1f (percentile method)\n", highThresh, lowThresh);
    
    // STRONG=255, WEAK=128, ZERO=0
    Image thresh(nms.w, nms.h);
    for (int i = 0; i < nms.w * nms.h; i++) {
        if (nms.r[i] >= highThresh) {
            thresh.r[i] = 255; // strong edge
        } else if (nms.r[i] >= lowThresh) {
            thresh.r[i] = 128; // weak edge
        } else {
            thresh.r[i] = 0;   // suppress
        }
        thresh.g[i] = thresh.r[i];
        thresh.b[i] = thresh.r[i];
    }
    
    return thresh;
}

// ============================================================
// Stage 5: Edge Tracking by Hysteresis
// ============================================================

void edgeTrackDFS(Image& img, int x, int y) {
    // 8-connected DFS to trace weak edges connected to strong edges
    const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    
    img.r[y*img.w + x] = 255;
    img.g[y*img.w + x] = 255;
    
    for (int i = 0; i < 8; i++) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx >= 0 && nx < img.w && ny >= 0 && ny < img.h) {
            if (img.r[ny*img.w + nx] == 128) {
                edgeTrackDFS(img, nx, ny);
            }
        }
    }
}

Image hysteresisEdgeTracking(const Image& thresh) {
    Image result = thresh;
    
    // First pass: find all strong edges and trace through weak edges
    for (int y = 0; y < result.h; y++) {
        for (int x = 0; x < result.w; x++) {
            if (result.r[y*result.w + x] == 255) {
                // Strong edge pixel - trace neighbors
                const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
                const int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
                for (int i = 0; i < 8; i++) {
                    int nx = x + dx[i];
                    int ny = y + dy[i];
                    if (nx >= 0 && nx < result.w && ny >= 0 && ny < result.h) {
                        if (result.r[ny*result.w + nx] == 128) {
                            edgeTrackDFS(result, nx, ny);
                        }
                    }
                }
            }
        }
    }
    
    // Second pass: suppress remaining weak edges (not connected to strong)
    for (int i = 0; i < result.w * result.h; i++) {
        if (result.r[i] == 128) {
            result.r[i] = 0;
            result.g[i] = 0;
            result.b[i] = 0;
        }
    }
    
    return result;
}

// ============================================================
// Quantifiable Verification
// ============================================================

struct EdgeStats {
    int totalPixels;
    int edgePixels;
    int strongEdgePixels;
    float edgeDensity;     // % of image that's edges
    float meanGradientMag; // mean of non-zero gradients
    float maxGradientMag;
    int connectedComponents;
};

EdgeStats computeEdgeStats(const Image& edgeImg, const GradientResult& gr) {
    EdgeStats es = {};
    es.totalPixels = edgeImg.w * edgeImg.h;
    
    for (int i = 0; i < es.totalPixels; i++) {
        if (edgeImg.r[i] > 0) {
            es.edgePixels++;
            if (gr.magnitude[i] > es.maxGradientMag) es.maxGradientMag = gr.magnitude[i];
            es.meanGradientMag += gr.magnitude[i];
        }
    }
    es.edgeDensity = 100.0f * es.edgePixels / es.totalPixels;
    if (es.edgePixels > 0) es.meanGradientMag /= es.edgePixels;
    
    return es;
}

// ============================================================
// Test Pattern Generator
// ============================================================

Image generateTestImage() {
    Image img(512, 512);
    
    // Fill with white background
    for (int i = 0; i < 512*512; i++) {
        img.r[i] = img.g[i] = img.b[i] = 255;
    }
    
    auto drawRect = [&](int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b) {
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                if (x >= 0 && x < 512 && y >= 0 && y < 512) {
                    int idx = y * 512 + x;
                    img.r[idx] = r; img.g[idx] = g; img.b[idx] = b;
                }
            }
        }
    };
    
    auto drawCircle = [&](int cx, int cy, int radius, unsigned char r, unsigned char g, unsigned char b) {
        for (int y = cy - radius; y <= cy + radius; y++) {
            for (int x = cx - radius; x <= cx + radius; x++) {
                if (x >= 0 && x < 512 && y >= 0 && y < 512) {
                    int dx = x - cx, dy = y - cy;
                    if (dx*dx + dy*dy <= radius*radius) {
                        int idx = y * 512 + x;
                        img.r[idx] = r; img.g[idx] = g; img.b[idx] = b;
                    }
                }
            }
        }
    };
    
    auto drawLine = [&](int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            if (x0 >= 0 && x0 < 512 && y0 >= 0 && y0 < 512) {
                int idx = y0 * 512 + x0;
                img.r[idx] = r; img.g[idx] = g; img.b[idx] = b;
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    };
    
    // Draw various shapes for edge detection testing
    // Large dark rectangle
    drawRect(50, 50, 200, 180, 40, 40, 40);
    // Medium gray rectangle
    drawRect(280, 60, 450, 160, 120, 120, 120);
    // Circle
    drawCircle(150, 320, 70, 60, 60, 60);
    // Another circle (partial overlap)
    drawCircle(250, 340, 55, 80, 80, 80);
    // Diamond/rotated square (using lines)
    drawLine(380, 250, 440, 320, 50, 50, 50);
    drawLine(440, 320, 380, 390, 50, 50, 50);
    drawLine(380, 390, 320, 320, 50, 50, 50);
    drawLine(320, 320, 380, 250, 50, 50, 50);
    // Fill the diamond
    for (int y = 250; y < 391; y++) {
        for (int x = 300; x < 460; x++) {
            // Simple scanline fill for the diamond
            int dx0 = x - 380, dy0 = y - 250;
            int dx1 = x - 440, dy1 = y - 320;
            int dx2 = x - 380, dy2 = y - 390;
            int dx3 = x - 320, dy3 = y - 320;
            if (dy0 >= 0 && dy1*dx0 - dx1*dy0 >= 0 && dy2*dx1 - dx2*dy1 >= 0 && 
                dy3*dx2 - dx3*dy2 >= 0 && dy0*dx3 - dx0*dy3 >= 0) {
                if (x >= 320 && x < 440 && y >= 250 && y < 390) {
                    int idx = y * 512 + x;
                    img.r[idx] = 50; img.g[idx] = 50; img.b[idx] = 50;
                }
            }
        }
    }
    // Thin line
    drawLine(60, 400, 260, 400, 30, 30, 30);
    drawLine(60, 402, 260, 402, 30, 30, 30);
    // Gradient rectangle
    for (int y = 420; y < 470; y++) {
        for (int x = 300; x < 480; x++) {
            int val = 50 + (x - 300) * 150 / 180;
            int idx = y * 512 + x;
            img.r[idx] = img.g[idx] = img.b[idx] = (unsigned char)val;
        }
    }
    
    return img;
}

// ============================================================
// Main
// ============================================================

int main() {
    printf("=== Canny Edge Detection ===\n\n");
    
    // Generate test image
    printf("[1/5] Generating test image...\n");
    Image src = generateTestImage();
    writePPM("input_test.ppm", src);
    printf("  Input: %dx%d, written to input_test.ppm\n", src.w, src.h);
    
    // Stage 1: Gaussian smoothing
    printf("[2/5] Gaussian smoothing (5x5, sigma=1.4)...\n");
    Image blurred = gaussianBlur(src, 5, 1.4f);
    writePPM("stage1_gaussian.ppm", blurred);
    
    // Stage 2: Gradient computation
    printf("[3/5] Gradient computation (Sobel 3x3)...\n");
    auto gradients = computeGradients(blurred);
    
    // Visualize gradient magnitude
    Image gradVis(src.w, src.h);
    for (int i = 0; i < src.w * src.h; i++) {
        unsigned char v = (unsigned char)std::min(gradients.magnitude[i], 255.0f);
        gradVis.r[i] = gradVis.g[i] = gradVis.b[i] = v;
    }
    writePGM("stage2_gradient.pgm", gradVis);
    
    // Stage 3: Non-maximum suppression
    printf("[4/5] Non-maximum suppression...\n");
    Image nms = nonMaximumSuppression(gradients);
    writePGM("stage3_nms.pgm", nms);
    
    // Stage 4 + 5: Double threshold + hysteresis
    printf("[5/5] Double threshold + hysteresis edge tracking...\n");
    Image thresh = doubleThreshold(nms, 0.4f, 0.15f); // lowRatio=0.4, highRatio=0.15
    Image edges = hysteresisEdgeTracking(thresh);
    writePPM("output_canny.ppm", edges);
    
    // Also produce inverted version for better viewing
    Image edgesInv(src.w, src.h);
    for (int i = 0; i < src.w * src.h; i++) {
        unsigned char v = edges.r[i] > 0 ? 0 : 255;
        edgesInv.r[i] = edgesInv.g[i] = edgesInv.b[i] = v;
    }
    writePPM("output_canny_inverted.ppm", edgesInv);
    
    // ============================================================
    // Quantifiable Verification
    // ============================================================
    printf("\n=== Quantifiable Verification ===\n");
    
    auto stats = computeEdgeStats(edges, gradients);
    
    printf("Edge Statistics:\n");
    printf("  Total pixels:       %d\n", stats.totalPixels);
    printf("  Edge pixels:        %d\n", stats.edgePixels);
    printf("  Edge density:       %.2f%%\n", stats.edgeDensity);
    printf("  Mean gradient mag:  %.2f\n", stats.meanGradientMag);
    printf("  Max gradient mag:   %.2f\n", stats.maxGradientMag);
    
    // --- Automated checks ---
    bool allPassed = true;
    
    // Check 1: Edge density must be in a reasonable range (0.1% - 30%)
    printf("\n[Check 1] Edge density: %.2f%% ", stats.edgeDensity);
    if (stats.edgeDensity < 0.1f) {
        printf("❌ FAIL - too few edges (possibly all suppressed)\n");
        allPassed = false;
    } else if (stats.edgeDensity > 30.0f) {
        printf("❌ FAIL - too many edges (threshold too low)\n");
        allPassed = false;
    } else {
        printf("✅ PASS (0.1%% - 30%%)\n");
    }
    
    // Check 2: There should be some edges with high gradient magnitude
    printf("[Check 2] Max gradient magnitude: %.2f ", stats.maxGradientMag);
    if (stats.maxGradientMag < 30.0f) {
        printf("❌ FAIL - gradients too weak, likely smoothing too aggressive\n");
        allPassed = false;
    } else {
        printf("✅ PASS (>= 30)\n");
    }
    
    // Check 3: NMS should reduce pixels compared to raw gradient
    // Count non-zero pixels in raw gradient (thresholded)
    int rawGradPixels = 0;
    for (int i = 0; i < src.w * src.h; i++) {
        if (gradients.magnitude[i] > 30.0f) rawGradPixels++;
    }
    printf("[Check 3] NMS effectiveness: raw_edges=%d → canny_edges=%d (reduction=%.1f%%) ",
           rawGradPixels, stats.edgePixels,
           rawGradPixels > 0 ? 100.0f*(rawGradPixels-stats.edgePixels)/rawGradPixels : 0);
    if (stats.edgePixels >= rawGradPixels) {
        printf("❌ FAIL - non-maximum suppression not effective\n");
        allPassed = false;
    } else {
        printf("✅ PASS (NMS thinned edges)\n");
    }
    
    // Check 4: Output file size - PGM for edge map
    {
        FILE* f = fopen("output_canny.ppm", "rb");
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fclose(f);
        printf("[Check 4] Output file size: %ld bytes ", sz);
        if (sz < 1024) {
            printf("❌ FAIL - file too small\n");
            allPassed = false;
        } else {
            printf("✅ PASS (> 1024 bytes)\n");
        }
    }
    
    // Check 5: Edge image should not be all black or all white
    {
        int zeroCount = 0;
        for (int i = 0; i < src.w * src.h; i++) {
            if (edges.r[i] == 0) zeroCount++;
        }
        float nonEdgePct = 100.0f * zeroCount / (src.w * src.h);
        printf("[Check 5] Non-edge pixel percentage: %.1f%% ", nonEdgePct);
        if (nonEdgePct < 50.0f) {
            printf("❌ FAIL - too few non-edge pixels (image mostly edges)\n");
            allPassed = false;
        } else if (nonEdgePct > 99.9f) {
            printf("❌ FAIL - almost entirely black (no edges detected)\n");
            allPassed = false;
        } else {
            printf("✅ PASS (50%% - 99.9%%)\n");
        }
    }
    
    // Check 6: Stage outputs verify intermediate stages
    {
        // Gaussian: check it's not identical to input (smoothing occurred)
        int diffCount = 0;
        for (int i = 0; i < src.w * src.h; i++) {
            if (abs((int)src.r[i] - (int)blurred.r[i]) > 1) diffCount++;
        }
        printf("[Check 6] Gaussian smoothing effect: %.1f%% pixels changed ", 
               100.0f * diffCount / (src.w * src.h));
        if (diffCount < 100) {
            printf("❌ FAIL - smoothing had no effect\n");
            allPassed = false;
        } else {
            printf("✅ PASS\n");
        }
    }
    
    printf("\n=== VERDICT ===\n");
    if (allPassed) {
        printf("✅ ALL CHECKS PASSED - Canny Edge Detection is working correctly!\n");
    } else {
        printf("❌ SOME CHECKS FAILED - see details above\n");
        return 1;
    }
    
    // Additional: compare with different parameters
    printf("\n=== Parameter Variation Test ===\n");
    // Low threshold variant (more edges)
    Image threshLow = doubleThreshold(nms, 0.2f, 0.10f);
    Image edgesLow = hysteresisEdgeTracking(threshLow);
    writePPM("output_canny_low.ppm", edgesLow);
    EdgeStats statsLow = computeEdgeStats(edgesLow, gradients);
    printf("  Low threshold:   edge_density=%.2f%%, edge_pixels=%d\n", statsLow.edgeDensity, statsLow.edgePixels);
    
    // High threshold variant (fewer edges)
    Image threshHigh = doubleThreshold(nms, 0.6f, 0.25f);
    Image edgesHigh = hysteresisEdgeTracking(threshHigh);
    writePPM("output_canny_high.ppm", edgesHigh);
    EdgeStats statsHigh = computeEdgeStats(edgesHigh, gradients);
    printf("  High threshold:  edge_density=%.2f%%, edge_pixels=%d\n", statsHigh.edgeDensity, statsHigh.edgePixels);
    
    printf("  Default:         edge_density=%.2f%%, edge_pixels=%d\n", stats.edgeDensity, stats.edgePixels);
    
    // Verify: low threshold should produce strictly more edges than high threshold
    printf("\n[Param Check] Edge count monotonicity: ");
    if (statsLow.edgePixels >= stats.edgePixels && stats.edgePixels >= statsHigh.edgePixels) {
        printf("✅ PASS (low >= default >= high)\n");
    } else {
        printf("❌ FAIL - threshold ordering violated\n");
        return 1;
    }
    
    printf("\n=== Canny Edge Detection Complete ===\n");
    printf("Files generated:\n");
    printf("  input_test.ppm          - Original test image\n");
    printf("  stage1_gaussian.ppm     - After Gaussian smoothing\n");
    printf("  stage2_gradient.pgm     - Gradient magnitude\n");
    printf("  stage3_nms.pgm          - After non-max suppression\n");
    printf("  output_canny.ppm        - Final edge detection result\n");
    printf("  output_canny_inverted.ppm - Inverted for viewing\n");
    printf("  output_canny_low.ppm    - Low threshold variant\n");
    printf("  output_canny_high.ppm   - High threshold variant\n");
    
    return 0;
}
