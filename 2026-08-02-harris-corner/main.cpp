/*
 * Harris Corner Detection
 * =========================
 * 
 * Algorithm:
 * 1. Compute image gradients Ix, Iy using Sobel operators
 * 2. At each pixel, form the structure tensor M:
 *    M = [[ΣIx², ΣIxIy],
 *         [ΣIxIy, ΣIy²]]   (summed over a Gaussian window)
 * 3. Compute corner response R = det(M) - k * trace(M)²
 * 4. Apply non-maximum suppression over a local window
 * 5. Threshold to find strong corners
 *
 * Core concepts:
 * - Smooth region: both eigenvalues small    → R ≈ 0
 * - Edge: one eigenvalue large, one small    → R < 0
 * - Corner: both eigenvalues large           → R ≫ 0
 *
 * Validation:
 * - Quantitatively verify corners detected by computing local
 *   gradient statistics around each detected corner
 * - Verify that corner response is significantly higher than
 *   edge and flat regions via statistical comparison
 * - Measure corner localization accuracy on synthetic test patterns
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <cassert>

// ==================== Image I/O (PPM) ====================

struct Image {
    int w, h;
    std::vector<double> r, g, b;  // store as double for intermediate precision

    Image(int w_, int h_) : w(w_), h(h_), r(w*h, 0), g(w*h, 0), b(w*h, 0) {}

    int idx(int x, int y) const { return y * w + x; }

    void setPixel(int x, int y, double rv, double gv, double bv) {
        int i = idx(x, y);
        r[i] = rv; g[i] = gv; b[i] = bv;
    }

    bool savePPM(const std::string& filename) const {
        std::ofstream f(filename, std::ios::binary);
        if (!f) return false;
        f << "P6\n" << w << " " << h << "\n255\n";
        for (int i = 0; i < w * h; i++) {
            unsigned char cr = std::max(0, std::min(255, (int)(r[i] * 255)));
            unsigned char cg = std::max(0, std::min(255, (int)(g[i] * 255)));
            unsigned char cb = std::max(0, std::min(255, (int)(b[i] * 255)));
            f << cr << cg << cb;
        }
        f.close();
        return true;
    }

    static Image fromBinaryPPM(const std::string& filename) {
        std::ifstream f(filename, std::ios::binary);
        if (!f) { std::cerr << "Cannot open: " << filename << "\n"; exit(1); }
        std::string line;
        std::getline(f, line); // P6
        if (line != "P6") { std::cerr << "Only P6 PPM supported, got: " << line << "\n"; exit(1); }
        // Skip comments
        while (f.peek() == '#') { std::getline(f, line); }
        int w, h, maxval;
        f >> w >> h >> maxval;
        f.get(); // consume newline
        Image img(w, h);
        std::vector<unsigned char> data(w * h * 3);
        f.read(reinterpret_cast<char*>(data.data()), w * h * 3);
        for (int i = 0; i < w * h; i++) {
            img.r[i] = data[i*3] / 255.0;
            img.g[i] = data[i*3+1] / 255.0;
            img.b[i] = data[i*3+2] / 255.0;
        }
        return img;
    }
};

// ==================== Synthetic Test Pattern Generator ====================

Image generateTestPattern() {
    // Generate a synthetic image with known corners for validation
    // Pattern: white rectangles on black background with clear corners
    int w = 400, h = 300;
    Image img(w, h);

    // Fill with medium gray background
    for (int i = 0; i < w * h; i++) {
        img.r[i] = img.g[i] = img.b[i] = 0.3;
    }

    // Draw several white rectangles (known corner locations)
    auto fillRect = [&](int x1, int y1, int x2, int y2, double val) {
        for (int y = y1; y <= y2; y++)
            for (int x = x1; x <= x2; x++)
                if (x >= 0 && x < w && y >= 0 && y < h)
                    img.setPixel(x, y, val, val, val);
    };

    // Rectangle 1: top-left area
    fillRect(50, 50, 150, 120, 0.9);
    // Rectangle 2: bottom-right area
    fillRect(220, 160, 350, 260, 0.9);
    // Rectangle 3: thin bar (edge vs corner distinction)
    fillRect(50, 200, 180, 210, 0.85);
    // Dot (corner-like but small)
    fillRect(300, 40, 310, 50, 0.95);
    // Triangle approximation (staircase corners)
    for (int y = 60; y < 130; y++) {
        int x1 = 220;
        int x2 = 220 + (int)((y - 60) * 1.5);
        for (int x = x1; x <= x2 && x < w; x++)
            img.setPixel(x, y, 0.8, 0.8, 0.8);
    }

    return img;
}

// ==================== Gaussian Kernel ====================

std::vector<double> gaussianKernel1D(int radius, double sigma) {
    std::vector<double> kernel(2 * radius + 1);
    double sum = 0;
    for (int i = -radius; i <= radius; i++) {
        double val = std::exp(-(i * i) / (2 * sigma * sigma));
        kernel[i + radius] = val;
        sum += val;
    }
    for (auto& v : kernel) v /= sum;
    return kernel;
}

void gaussianBlur(const std::vector<double>& src, std::vector<double>& dst,
                  int w, int h, int radius, double sigma) {
    auto kernel = gaussianKernel1D(radius, sigma);
    std::vector<double> tmp(w * h, 0);

    // Horizontal pass
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sum = 0;
            for (int k = -radius; k <= radius; k++) {
                int sx = std::max(0, std::min(w - 1, x + k));
                sum += src[y * w + sx] * kernel[k + radius];
            }
            tmp[y * w + x] = sum;
        }
    }
    // Vertical pass
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sum = 0;
            for (int k = -radius; k <= radius; k++) {
                int sy = std::max(0, std::min(h - 1, y + k));
                sum += tmp[sy * w + x] * kernel[k + radius];
            }
            dst[y * w + x] = sum;
        }
    }
}

// ==================== Harris Corner Detection ====================

struct CornerResult {
    std::vector<double> response;  // corner response R for each pixel
    std::vector<std::pair<int,int>> corners;  // (x, y) of detected corners
    Image visualization;
    double threshold;
};

CornerResult detectHarrisCorners(const Image& img, double k = 0.04,
                                  int windowRadius = 3, double sigma = 1.5,
                                  double cornerThreshold = 0.01,
                                  int nmsRadius = 5) {
    int w = img.w, h = img.h;
    int N = w * h;

    // Convert to grayscale
    std::vector<double> gray(N);
    for (int i = 0; i < N; i++) {
        gray[i] = 0.299 * img.r[i] + 0.587 * img.g[i] + 0.114 * img.b[i];
    }

    // Step 1: Compute gradients using Sobel operators
    std::vector<double> Ix(N, 0), Iy(N, 0);

    // Sobel kernels
    const int sobel_x[3][3] = {{-1,0,1}, {-2,0,2}, {-1,0,1}};
    const int sobel_y[3][3] = {{-1,-2,-1}, {0,0,0}, {1,2,1}};

    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            double gx = 0, gy = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    double val = gray[(y+dy)*w + (x+dx)];
                    gx += val * sobel_x[dy+1][dx+1];
                    gy += val * sobel_y[dy+1][dx+1];
                }
            }
            Ix[y*w + x] = gx;
            Iy[y*w + x] = gy;
        }
    }

    // Step 2: Compute products Ix², IxIy, Iy² and smooth with Gaussian
    std::vector<double> Ixx(N), Ixy(N), Iyy(N);
    for (int i = 0; i < N; i++) {
        Ixx[i] = Ix[i] * Ix[i];
        Ixy[i] = Ix[i] * Iy[i];
        Iyy[i] = Iy[i] * Iy[i];
    }

    // Gaussian smooth the structure tensor components
    std::vector<double> Sxx(N), Sxy(N), Syy(N);
    int gaussRadius = windowRadius;
    gaussianBlur(Ixx, Sxx, w, h, gaussRadius, sigma);
    gaussianBlur(Ixy, Sxy, w, h, gaussRadius, sigma);
    gaussianBlur(Iyy, Syy, w, h, gaussRadius, sigma);

    // Step 3: Compute corner response R = det(M) - k * trace(M)²
    std::vector<double> R(N, 0);
    double maxR = 0;
    for (int i = 0; i < N; i++) {
        double det = Sxx[i] * Syy[i] - Sxy[i] * Sxy[i];
        double trace = Sxx[i] + Syy[i];
        R[i] = det - k * trace * trace;
        if (R[i] > maxR) maxR = R[i];
    }

    // Normalize R to [0, 1] for easier thresholding
    if (maxR > 0) {
        for (int i = 0; i < N; i++) R[i] /= maxR;
    }

    // Step 4: Non-maximum suppression
    std::vector<bool> isMax(N, true);
    for (int y = nmsRadius; y < h - nmsRadius; y++) {
        for (int x = nmsRadius; x < w - nmsRadius; x++) {
            int ci = y * w + x;
            double cr = R[ci];
            if (cr <= 0) { isMax[ci] = false; continue; }
            for (int dy = -nmsRadius; dy <= nmsRadius; dy++) {
                for (int dx = -nmsRadius; dx <= nmsRadius; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int ni = (y+dy)*w + (x+dx);
                    if (R[ni] > cr) { isMax[ci] = false; goto next_pixel; }
                }
            }
            next_pixel:;
        }
    }

    // Step 5: Threshold and collect corners
    std::vector<std::pair<int,int>> corners;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int i = y * w + x;
            if (isMax[i] && R[i] >= cornerThreshold) {
                corners.emplace_back(x, y);
            }
        }
    }

    // Create visualization
    Image vis(w, h);
    // Copy original
    for (int i = 0; i < N; i++) {
        vis.r[i] = img.r[i];
        vis.g[i] = img.g[i];
        vis.b[i] = img.b[i];
    }
    // Draw corners
    for (auto [cx, cy] : corners) {
        for (int dy = -2; dy <= 2; dy++) {
            for (int dx = -2; dx <= 2; dx++) {
                int nx = cx + dx, ny = cy + dy;
                if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
                    // Red crosshair
                    if (std::abs(dx) <= 1 && std::abs(dy) <= 1) {
                        vis.setPixel(nx, ny, 1.0, 0, 0);
                    } else {
                        vis.setPixel(nx, ny, 1.0, 0.2, 0.2);
                    }
                }
            }
        }
    }

    return {R, corners, vis, cornerThreshold};
}

// ==================== Quantitative Validation ====================

struct ValidationStats {
    int numCorners;
    double meanResponse;
    double maxResponse;
    double minResponse;
    double responseStdDev;
    // Corner vs non-corner statistics
    double cornerMeanGradientMag;
    double nonCornerMeanGradientMag;
    // Structure tensor trace statistics
    double cornerMeanTrace;
    double nonCornerMeanTrace;
};

ValidationStats validate(const Image& img, const std::vector<double>& response,
                          const std::vector<std::pair<int,int>>& corners,
                          double threshold) {
    int w = img.w, h = img.h, N = w * h;

    // Compute gradient magnitude for each pixel
    std::vector<double> gray(N);
    for (int i = 0; i < N; i++)
        gray[i] = 0.299 * img.r[i] + 0.587 * img.g[i] + 0.114 * img.b[i];

    std::vector<double> gradMag(N, 0);
    for (int y = 1; y < h - 1; y++) {
        for (int x = 1; x < w - 1; x++) {
            double gx = gray[(y+1)*w+x] - gray[(y-1)*w+x];
            double gy = gray[y*w+x+1] - gray[y*w+x-1];
            gradMag[y*w+x] = std::sqrt(gx*gx + gy*gy);
        }
    }

    ValidationStats s;
    s.numCorners = corners.size();

    // Corner statistics
    double sumR = 0, sumR2 = 0, maxR = 0, minR = 1e9;
    double sumGrad = 0;
    for (auto [x, y] : corners) {
        int i = y * w + x;
        double r = response[i];
        sumR += r;
        sumR2 += r * r;
        if (r > maxR) maxR = r;
        if (r < minR) minR = r;
        sumGrad += gradMag[i];
    }

    s.meanResponse = corners.empty() ? 0 : sumR / corners.size();
    s.maxResponse = maxR;
    s.minResponse = corners.empty() ? 0 : minR;
    s.responseStdDev = corners.empty() ? 0 :
        std::sqrt(sumR2 / corners.size() - s.meanResponse * s.meanResponse);
    s.cornerMeanGradientMag = corners.empty() ? 0 : sumGrad / corners.size();

    // Non-corner statistics (sample from regions far from corners)
    double sumNcGrad = 0;
    int ncCount = 0;
    for (int y = 10; y < h - 10 && ncCount < 1000; y += 8) {
        for (int x = 10; x < w - 10 && ncCount < 1000; x += 8) {
            int i = y * w + x;
            if (response[i] < threshold * 0.5) {
                sumNcGrad += gradMag[i];
                ncCount++;
            }
        }
    }
    s.nonCornerMeanGradientMag = ncCount > 0 ? sumNcGrad / ncCount : 0;

    return s;
}

void printValidation(const ValidationStats& s) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════╗\n";
    std::cout << "║     Harris Corner Detection - Validation      ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ Detected corners:    " << std::setw(6) << s.numCorners << "                   ║\n";
    std::cout << "║ Mean response:       " << std::fixed << std::setprecision(4) << std::setw(8) << s.meanResponse << "                ║\n";
    std::cout << "║ Max response:        " << std::setw(8) << s.maxResponse << "                ║\n";
    std::cout << "║ Min response:        " << std::setw(8) << s.minResponse << "                ║\n";
    std::cout << "║ Response std dev:    " << std::setw(8) << s.responseStdDev << "                ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ Corner grad mean:    " << std::setw(8) << std::setprecision(3) << s.cornerMeanGradientMag << "                ║\n";
    std::cout << "║ Non-corner grad mean:" << std::setw(8) << s.nonCornerMeanGradientMag << "                ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
}

// ==================== Synthetic precision test ====================

struct PrecisionTest {
    struct Region {
        int cx, cy;  // known corner center
        std::string label;
    };
    std::vector<Region> knownCorners;
    int detectedNearKnown;
    double localizationError;
};

PrecisionTest testPrecision(const std::vector<std::pair<int,int>>& corners) {
    PrecisionTest pt;

    // Known corners from our test pattern:
    // Rect 1: (50,50), (150,50), (50,120), (150,120)
    // Rect 2: (220,160), (350,160), (220,260), (350,260)
    // Rect 3: (50,200), (180,200), (50,210), (180,210)
    // Dot:    (300,40), (310,40), (300,50), (310,50)
    // Triangle: approximate
    std::vector<std::pair<int,int>> known = {
        {50,50}, {150,50}, {50,120}, {150,120},
        {220,160}, {350,160}, {220,260}, {350,260},
        {50,200}, {180,200}, {50,210}, {180,210},
        {300,40}, {310,40}, {300,50}, {310,50}
    };

    int detected = 0;
    double totalError = 0;
    int matchCount = 0;

    // For each known corner, find nearest detected corner
    for (auto [kx, ky] : known) {
        double minDist = 1e9;
        for (int ci = 0; ci < (int)corners.size(); ci++) {
            double dx = corners[ci].first - kx;
            double dy = corners[ci].second - ky;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < minDist) { minDist = dist; }
        }
        if (minDist <= 4.0) {  // within 4 pixels
            detected++;
            totalError += minDist;
            matchCount++;
        }
    }

    pt.detectedNearKnown = detected;
    pt.localizationError = matchCount > 0 ? totalError / matchCount : 0;
    return pt;
}

// ==================== Edge vs Corner discrimination test ====================

void testEdgeVsCorner(const Image& img, const CornerResult& result) {
    int w = img.w;

    // Sample responses from known flat, edge, and corner regions
    // Flat: center of empty area
    // Edge: along rectangle boundary
    // Corner: at rectangle corners

    struct Sample {
        double response;
        std::string type;
    };
    std::vector<Sample> samples;

    // Flat regions (sampled from background area)
    for (int y = 15; y < 35; y += 4)
        for (int x = 15; x < 35; x += 4)
            samples.push_back({result.response[y*w+x], "flat"});

    // Edge regions (sampled along rectangle sides)
    for (int x = 60; x <= 140; x += 4)
        samples.push_back({result.response[50*w + x], "edge_horizontal"});
    for (int x = 230; x <= 340; x += 4)
        samples.push_back({result.response[160*w + x], "edge_horizontal"});

    // Corner regions (near known corners)
    const int margin = 3;
    std::vector<std::pair<int,int>> cornerSamples = {
        {50,50}, {150,50}, {50,120}, {150,120},
        {220,160}, {350,160}, {220,260}, {350,260},
        {300,40}, {310,50}
    };
    for (auto [cx, cy] : cornerSamples) {
        for (int dy = -margin; dy <= margin; dy++)
            for (int dx = -margin; dx <= margin; dx++)
                samples.push_back({result.response[(cy+dy)*w + (cx+dx)], "corner"});
    }

    // Compute statistics per type
    double flatSum = 0, edgeSum = 0, cornerSum = 0;
    int flatN = 0, edgeN = 0, cornerN = 0;

    for (auto& s : samples) {
        if (s.type == "flat") { flatSum += s.response; flatN++; }
        else if (s.type == "edge_horizontal") { edgeSum += s.response; edgeN++; }
        else { cornerSum += s.response; cornerN++; }
    }

    double flatMean = flatN > 0 ? flatSum / flatN : 0;
    double edgeMean = edgeN > 0 ? edgeSum / edgeN : 0;
    double cornerMean = cornerN > 0 ? cornerSum / cornerN : 0;

    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║   Discrimination Test (Edge vs Corner)       ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ Flat region mean R:  " << std::setw(9) << std::fixed << std::setprecision(5) << flatMean << "            ║\n";
    std::cout << "║ Edge region mean R:  " << std::setw(9) << edgeMean << "            ║\n";
    std::cout << "║ Corner region mean R:" << std::setw(9) << cornerMean << "            ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";

    bool passesDiscrimination = (cornerMean > edgeMean * 1.2) &&
                                 (edgeMean > flatMean * 1.2);
    std::cout << "║ Corner > Edge:       " << (cornerMean > edgeMean * 1.2 ? "✅ PASS" : "❌ FAIL") << "             ║\n";
    std::cout << "║ Edge > Flat:         " << (edgeMean > flatMean * 1.2 ? "✅ PASS" : "❌ FAIL") << "             ║\n";
    std::cout << "║ Overall:             " << (passesDiscrimination ? "✅ PASS" : "❌ FAIL") << "             ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";
}

// ==================== Main ====================

int main() {
    std::cout << "=== Harris Corner Detection ===\n";
    std::cout << "Date: 2026-08-02\n\n";

    // Generate test pattern
    std::cout << "Generating test pattern...\n";
    Image img = generateTestPattern();
    img.savePPM("test_pattern.ppm");
    std::cout << "  → test_pattern.ppm saved\n";

    // Detect corners
    std::cout << "Detecting Harris corners...\n";
    auto result = detectHarrisCorners(img, 0.04, 3, 1.5, 0.01, 5);
    result.visualization.savePPM("harris_corners.ppm");
    std::cout << "  → harris_corners.ppm saved\n";

    // Also generate a response heatmap
    std::cout << "Generating response heatmap...\n";
    Image heatmap(img.w, img.h);
    for (int i = 0; i < img.w * img.h; i++) {
        double r = result.response[i];
        r = std::max(0.0, std::min(1.0, r));
        // Blue-white-red colormap
        heatmap.r[i] = r;
        heatmap.g[i] = r * (1 - std::abs(r - 0.5) * 2);
        heatmap.b[i] = 1 - r;
    }
    heatmap.savePPM("harris_response.ppm");
    std::cout << "  → harris_response.ppm saved\n";

    // Validate
    auto stats = validate(img, result.response, result.corners, result.threshold);
    printValidation(stats);

    // Precision test
    auto precision = testPrecision(result.corners);
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║     Corner Localization Precision Test       ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    std::cout << "║ Known corners:       " << std::setw(6) << "16" << "                 ║\n";
    std::cout << "║ Detected near known: " << std::setw(6) << precision.detectedNearKnown << "                 ║\n";
    std::cout << "║ Mean local. error:   " << std::setw(6) << std::fixed << std::setprecision(2) << precision.localizationError << " px            ║\n";
    double recall = precision.detectedNearKnown / 16.0;
    std::cout << "║ Recall:              " << std::setw(6) << std::fixed << std::setprecision(1) << recall * 100 << "%             ║\n";
    std::cout << "║ Recall check:        " << (recall >= 0.5 ? "✅ PASS" : "❌ FAIL") << "             ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";

    // Edge vs Corner discrimination
    testEdgeVsCorner(img, result);

    // Additional validation: check that corners concentrate at high-gradient regions
    std::cout << "\n╔══════════════════════════════════════════════╗\n";
    std::cout << "║  Gradient Concentration Validation            ║\n";
    std::cout << "╠══════════════════════════════════════════════╣\n";
    double ratio = stats.cornerMeanGradientMag / (stats.nonCornerMeanGradientMag + 1e-9);
    std::cout << "║ Corner/NonCorner     " << std::setw(8) << std::fixed << std::setprecision(2) << ratio << "x               ║\n";
    std::cout << "║ Ratio check (>2.0x): " << (ratio >= 2.0 ? "✅ PASS" : "❌ FAIL") << "             ║\n";
    std::cout << "╚══════════════════════════════════════════════╝\n";

    return 0;
}
