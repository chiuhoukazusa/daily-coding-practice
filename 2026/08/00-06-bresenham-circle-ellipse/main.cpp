/**
 * Bresenham Circle & Ellipse Rasterization
 *
 * Implements:
 *  1. Bresenham/Midpoint Circle Algorithm (integer arithmetic, 8-way symmetry)
 *  2. Midpoint Ellipse Algorithm (integer arithmetic, 4-way symmetry)
 *
 * Quantifiable verification:
 *  - Circle: 8-way pixel symmetry check
 *  - Circle: average radial error (distance from true radius)
 *  - Circle: pixel count verification
 *  - Ellipse: 4-way pixel symmetry check
 *  - Ellipse: average radial error against implicit equation
 *  - Ellipse: pixel count verification
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>
#include <set>
#include <string>
#include <cassert>

// PPM utilities
struct Color {
    uint8_t r, g, b;
};

struct Image {
    int w, h;
    std::vector<Color> pixels;

    Image(int width, int height) : w(width), h(height), pixels(width * height) {}

    Color& at(int x, int y) { return pixels[y * w + x]; }
    const Color& at(int x, int y) const { return pixels[y * w + x]; }

    void fill(Color c) {
        std::fill(pixels.begin(), pixels.end(), c);
    }

    bool inBounds(int x, int y) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    void setPixel(int x, int y, Color c) {
        if (inBounds(x, y)) at(x, y) = c;
    }

    void setPixelSafe(int x, int y, Color c) {
        setPixel(x, y, c);
    }

    bool savePPM(const char* path) {
        FILE* f = fopen(path, "wb");
        if (!f) return false;
        fprintf(f, "P6\n%d %d\n255\n", w, h);
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x) {
                const Color& c = at(x, y);
                fputc(c.r, f); fputc(c.g, f); fputc(c.b, f);
            }
        fclose(f);
        return true;
    }
};

// ===== Bresenham Midpoint Circle Algorithm =====
void bresenhamCircle(Image& img, int cx, int cy, int r, Color color) {
    int x = 0;
    int y = r;
    int d = 1 - r; // initial decision parameter

    while (x <= y) {
        // 8-way symmetry
        img.setPixel(cx + x, cy + y, color);
        img.setPixel(cx - x, cy + y, color);
        img.setPixel(cx + x, cy - y, color);
        img.setPixel(cx - x, cy - y, color);
        img.setPixel(cx + y, cy + x, color);
        img.setPixel(cx - y, cy + x, color);
        img.setPixel(cx + y, cy - x, color);
        img.setPixel(cx - y, cy - x, color);

        if (d < 0) {
            d += 2 * x + 3;
        } else {
            d += 2 * (x - y) + 5;
            y--;
        }
        x++;
    }
}

// ===== Midpoint Ellipse Algorithm =====
void bresenhamEllipse(Image& img, int cx, int cy, int rx, int ry, Color color) {
    // Region 1: x-dominant (slope > -1)
    {
        int x = 0;
        int y = ry;
        int rx2 = rx * rx;
        int ry2 = ry * ry;

        int d1 = ry2 - rx2 * ry + (rx2 >> 2); // initial decision param

        while (ry2 * x < rx2 * y) {
            // 4-way symmetry
            img.setPixel(cx + x, cy + y, color);
            img.setPixel(cx - x, cy + y, color);
            img.setPixel(cx + x, cy - y, color);
            img.setPixel(cx - x, cy - y, color);

            if (d1 < 0) {
                d1 += ry2 * (2 * x + 3);
            } else {
                d1 += ry2 * (2 * x + 3) + rx2 * (-2 * y + 2);
                y--;
            }
            x++;
        }
    }

    // Region 2: y-dominant (slope <= -1)
    {
        int x = rx;
        int y = 0;
        int rx2 = rx * rx;
        int ry2 = ry * ry;

        int d2 = rx2 - ry2 * rx + (ry2 >> 2);

        while (rx2 * y <= ry2 * x) {
            img.setPixel(cx + x, cy + y, color);
            img.setPixel(cx - x, cy + y, color);
            img.setPixel(cx + x, cy - y, color);
            img.setPixel(cx - x, cy - y, color);

            if (d2 < 0) {
                d2 += rx2 * (2 * y + 3);
            } else {
                d2 += rx2 * (2 * y + 3) + ry2 * (-2 * x + 2);
                x--;
            }
            y++;
        }
    }
}

// ===== Verification Utilities =====

// Collect drawn pixels for a circle relative to center
void collectCirclePixels(Image& img, int cx, int cy, int r, std::vector<std::pair<int,int>>& pixels) {
    Image tmp(img.w, img.h);
    tmp.fill({0,0,0});
    bresenhamCircle(tmp, cx, cy, r, {255,255,255});
    for (int y = 0; y < tmp.h; ++y)
        for (int x = 0; x < tmp.w; ++x)
            if (tmp.at(x,y).r > 128)
                pixels.push_back({x - cx, y - cy});
}

void collectEllipsePixels(Image& img, int cx, int cy, int rx, int ry, std::vector<std::pair<int,int>>& pixels) {
    Image tmp(img.w, img.h);
    tmp.fill({0,0,0});
    bresenhamEllipse(tmp, cx, cy, rx, ry, {255,255,255});
    for (int y = 0; y < tmp.h; ++y)
        for (int x = 0; x < tmp.w; ++x)
            if (tmp.at(x,y).r > 128)
                pixels.push_back({x - cx, y - cy});
}

double verifyCircleSymmetry(int r) {
    // Check 8-way symmetry: every drawn pixel should appear in all 8 octants
    Image img(4*r+4, 4*r+4);
    img.fill({0,0,0});
    int cx = 2*r+2, cy = 2*r+2;
    bresenhamCircle(img, cx, cy, r, {255,255,255});

    std::set<std::pair<int,int>> oct0; // octant 0: [45°, 90°): y >= x >= 0
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            if (img.at(x,y).r > 128) {
                int dx = x - cx, dy = y - cy;
                // Map to octant 0: take absolute, swap if needed
                int u = std::abs(dx), v = std::abs(dy);
                if (u < v) std::swap(u, v); // map to octant 0: x >= y >= 0
                oct0.insert({u, v});
            }
        }

    // For every octant-0 pixel, verify 8-way mapping
    int symmetryErrors = 0;
    for (const auto& p : oct0) {
        int u = p.first, v = p.second;
        // 8 octants: (±u,±v), (±v,±u)
        bool ok = true;
        ok &= img.at(cx+u, cy+v).r > 128;
        ok &= img.at(cx-u, cy+v).r > 128;
        ok &= img.at(cx+u, cy-v).r > 128;
        ok &= img.at(cx-u, cy-v).r > 128;
        ok &= img.at(cx+v, cy+u).r > 128;
        ok &= img.at(cx-v, cy+u).r > 128;
        ok &= img.at(cx+v, cy-u).r > 128;
        ok &= img.at(cx-v, cy-u).r > 128;
        if (!ok) symmetryErrors++;
    }

    return symmetryErrors > 0 ? static_cast<double>(symmetryErrors) / oct0.size() : 0.0;
}

double verifyCircleRadialError(int r) {
    // For each drawn pixel, compute radial error |sqrt(dx^2+dy^2) - r|
    std::vector<std::pair<int,int>> pixels;
    Image img(4*r+4, 4*r+4);
    img.fill({0,0,0});
    int cx = 2*r+2, cy = 2*r+2;
    collectCirclePixels(img, cx, cy, r, pixels);

    double totalError = 0;
    double maxError = 0;
    for (const auto& p : pixels) {
        double dist = std::sqrt(p.first * p.first + p.second * p.second);
        double err = std::abs(dist - r);
        totalError += err;
        maxError = std::max(maxError, err);
    }

    return totalError / pixels.size();
}

double verifyEllipseSymmetry(int rx, int ry) {
    // Check 4-way symmetry
    Image img(4*std::max(rx,ry)+4, 4*std::max(rx,ry)+4);
    img.fill({0,0,0});
    int cx = img.w/2, cy = img.h/2;
    bresenhamEllipse(img, cx, cy, rx, ry, {255,255,255});

    std::set<std::pair<int,int>> q1; // quadrant 1: x >= 0, y >= 0
    for (int y = 0; y < img.h; ++y)
        for (int x = 0; x < img.w; ++x) {
            if (img.at(x,y).r > 128) {
                int dx = x - cx, dy = y - cy;
                q1.insert({std::abs(dx), std::abs(dy)});
            }
        }

    int symmetryErrors = 0;
    for (const auto& p : q1) {
        int u = p.first, v = p.second;
        bool ok = true;
        ok &= img.at(cx+u, cy+v).r > 128;
        ok &= img.at(cx-u, cy+v).r > 128;
        ok &= img.at(cx+u, cy-v).r > 128;
        ok &= img.at(cx-u, cy-v).r > 128;
        if (!ok) symmetryErrors++;
    }
    return symmetryErrors > 0 ? static_cast<double>(symmetryErrors) / q1.size() : 0.0;
}

double verifyEllipseRadialError(int rx, int ry) {
    std::vector<std::pair<int,int>> pixels;
    Image img(4*std::max(rx,ry)+4, 4*std::max(rx,ry)+4);
    img.fill({0,0,0});
    int cx = img.w/2, cy = img.h/2;
    collectEllipsePixels(img, cx, cy, rx, ry, pixels);

    int rx2 = rx*rx, ry2 = ry*ry;
    double totalError = 0;
    for (const auto& p : pixels) {
        // Evaluate implicit equation: x^2/rx^2 + y^2/ry^2 - 1 should be ~0
        double val = (double)(p.first*p.first) / rx2 + (double)(p.second*p.second) / ry2 - 1.0;
        totalError += std::abs(val);
    }
    return totalError / pixels.size();
}

int countCirclePixels(int r) {
    std::vector<std::pair<int,int>> pixels;
    Image img(4*r+4, 4*r+4);
    img.fill({0,0,0});
    collectCirclePixels(img, 2*r+2, 2*r+2, r, pixels);
    return (int)pixels.size();
}

int countEllipsePixels(int rx, int ry) {
    std::vector<std::pair<int,int>> pixels;
    Image img(4*std::max(rx,ry)+4, 4*std::max(rx,ry)+4);
    img.fill({0,0,0});
    collectEllipsePixels(img, img.w/2, img.h/2, rx, ry, pixels);
    return (int)pixels.size();
}

int main() {
    printf("=== Bresenham Circle & Ellipse Rasterization ===\n\n");

    // ---- Circle Tests ----
    printf("--- Circle Tests ---\n");

    int circleRadii[] = {50, 100, 200};
    for (int r : circleRadii) {
        printf("\nCircle r=%d:\n", r);

        double symErr = verifyCircleSymmetry(r);
        double radErr = verifyCircleRadialError(r);
        int pixelCount = countCirclePixels(r);

        // Expected pixel count approx: circumference = 2*pi*r
        // Due to rasterization, actual count is usually ~ 4*sqrt(2)*r
        // But we just sanity check: should be > 0 and < circle area
        int minExpected = (int)(2 * M_PI * r * 0.7);
        int maxExpected = (int)(2 * M_PI * r * 1.1);

        printf("  8-way symmetry error rate: %.4f (%s)\n", symErr,
               symErr == 0.0 ? "PASS" : "FAIL");
        printf("  Avg radial error: %.4f pixels (%s)\n", radErr,
               radErr < 0.6 ? "PASS" : "FAIL");
        printf("  Pixel count: %d [%d, %d] (%s)\n", pixelCount, minExpected, maxExpected,
               pixelCount >= minExpected && pixelCount <= maxExpected ? "PASS" : "FAIL");

        assert(symErr == 0.0);
        assert(radErr < 0.6);
        assert(pixelCount >= minExpected && pixelCount <= maxExpected);
    }

    // ---- Ellipse Tests ----
    printf("\n--- Ellipse Tests ---\n");

    struct EllipseTest { int rx, ry; };
    EllipseTest ellipseTests[] = {{80, 40}, {60, 100}, {150, 80}, {100, 100}};
    for (const auto& t : ellipseTests) {
        int rx = t.rx, ry = t.ry;
        printf("\nEllipse rx=%d, ry=%d:\n", rx, ry);

        double symErr = verifyEllipseSymmetry(rx, ry);
        double radErr = verifyEllipseRadialError(rx, ry);
        int pixelCount = countEllipsePixels(rx, ry);

        // Ramanujan approximation for ellipse perimeter
        double a = rx, b = ry;
        double h = (a-b)*(a-b) / ((a+b)*(a+b));
        double perimeter = M_PI * (a+b) * (1 + 3*h/(10 + std::sqrt(4-3*h)));
        int minExpected = (int)(perimeter * 0.6);
        int maxExpected = (int)(perimeter * 1.2);

        printf("  4-way symmetry error rate: %.4f (%s)\n", symErr,
               symErr == 0.0 ? "PASS" : "FAIL");
        printf("  Avg implicit equation error: %.6f (%s)\n", radErr,
               radErr < 0.5 ? "PASS" : "FAIL");
        printf("  Pixel count: %d [%d, %d] (%s)\n", pixelCount, minExpected, maxExpected,
               pixelCount >= minExpected && pixelCount <= maxExpected ? "PASS" : "FAIL");

        assert(symErr == 0.0);
        assert(radErr < 0.5);
        assert(pixelCount >= minExpected && pixelCount <= maxExpected);
    }

    printf("\n=== All quantifiable verification tests PASSED ===\n\n");

    // ---- Generate visual output ----
    int imgW = 800, imgH = 600;
    Image output(imgW, imgH);
    output.fill({20, 20, 30}); // dark background

    // Draw concentric circles
    Color circleColors[] = {
        {255, 80, 80},   // red
        {80, 255, 80},   // green
        {80, 80, 255},   // blue
        {255, 255, 80},  // yellow
    };
    int radii[] = {30, 70, 110, 150};
    int cx = 250, cy = 300;
    for (int ci = 3; ci >= 0; ci--) {
        bresenhamCircle(output, cx, cy, radii[ci], circleColors[ci]);
    }

    // Draw ellipses
    Color elColors[] = {
        {255, 128, 0},
        {0, 255, 255},
        {255, 0, 255},
        {255, 255, 255},
    };
    int elParams[][2] = {{150, 60}, {80, 130}, {120, 90}, {100, 40}};
    int elCx = 600;
    for (int ei = 0; ei < 4; ei++) {
        bresenhamEllipse(output, elCx, 80 + ei*130, elParams[ei][0], elParams[ei][1], elColors[ei]);
    }

    // Separator line
    for (int y = 0; y < imgH; y++)
        output.setPixel(420, y, {60, 60, 60});

    // Label text: draw simple title
    // "Bresenham Circle (left) & Ellipse (right)"
    // We'll just output the image; labels can be added in post

    if (!output.savePPM("bresenham_output.ppm")) {
        fprintf(stderr, "Failed to save output\n");
        return 1;
    }
    printf("Visual output saved to bresenham_output.ppm (%d x %d)\n", imgW, imgH);

    // Pixel statistics for the output image
    int nonzeroCount = 0;
    for (int y = 0; y < imgH; ++y)
        for (int x = 0; x < imgW; ++x) {
            const Color& c = output.at(x, y);
            if (c.r != 20 || c.g != 20 || c.b != 30) nonzeroCount++;
        }
    printf("Drawn pixels (non-bg): %d\n", nonzeroCount);
    assert(nonzeroCount > 100); // sanity check

    return 0;
}
