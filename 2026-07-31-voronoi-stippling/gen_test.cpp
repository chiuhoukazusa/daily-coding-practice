/**
 * Generate test images for voronoi stippling
 */
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <vector>

struct Color { uint8_t r, g, b; };

void writePPM(const std::string& fn, int w, int h, const std::vector<Color>& p) {
    std::ofstream f(fn, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for (auto& c : p) f.write((char*)&c, 3);
}

int main() {
    int w = 400, h = 400;
    std::vector<Color> p(w * h);

    // Test 1: radial gradient (simulates a face-like distribution)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double dx = (x - w/2.0) / (w * 0.45);
            double dy = (y - h/2.0) / (h * 0.45);
            double r = std::sqrt(dx*dx + dy*dy);
            // Face-like: bright center, darker edges
            double val = std::exp(-r * r * 1.5) * 255;
            int v = (int)std::max(0.0, std::min(255.0, val));
            p[y*w+x] = {(uint8_t)v, (uint8_t)v, (uint8_t)v};
        }
    }
    writePPM("test_radial.ppm", w, h, p);
    std::cout << "test_radial.ppm\n";

    // Test 2: checkerboard pattern (clear boundaries)
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int cx = x / 40, cy = y / 40;
            int v = ((cx + cy) % 2) ? 220 : 60;
            p[y*w+x] = {(uint8_t)v, (uint8_t)v, (uint8_t)v};
        }
    }
    writePPM("test_checker.ppm", w, h, p);
    std::cout << "test_checker.ppm\n";

    // Test 3: simple circle + rectangle shapes
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double dx1 = x - 120, dy1 = y - 200;
            double dx2 = x - 280, dy2 = y - 200;
            bool inCircle1 = (dx1*dx1 + dy1*dy1) < 90*90;
            bool inCircle2 = (dx2*dx2 + dy2*dy2) < 60*60;
            bool inRect = (x > 150 && x < 250 && y > 50 && y < 150);
            int v = 250;
            if (inCircle1) v = 30;
            if (inCircle2) v = 60;
            if (inRect) v = 90;
            p[y*w+x] = {(uint8_t)v, (uint8_t)v, (uint8_t)v};
        }
    }
    writePPM("test_shapes.ppm", w, h, p);
    std::cout << "test_shapes.ppm\n";

    return 0;
}
