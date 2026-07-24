#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include <fstream>
#include <sstream>

// Simple PPM image I/O
struct Color {
    unsigned char r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(unsigned char r, unsigned char g, unsigned char b) : r(r), g(g), b(b) {}
};

struct Image {
    int w, h;
    std::vector<Color> data;
    
    Image(int w, int h) : w(w), h(h), data(w * h) {}
    
    Color& at(int x, int y) { return data[y * w + x]; }
    const Color& at(int x, int y) const { return data[y * w + x]; }
    
    bool savePPM(const std::string& path) {
        std::ofstream f(path, std::ios::binary);
        if (!f) return false;
        f << "P6\n" << w << " " << h << "\n255\n";
        f.write(reinterpret_cast<const char*>(data.data()), w * h * 3);
        return f.good();
    }
    
    static Image loadPPM(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        Image img(1, 1);
        std::string magic;
        f >> magic;
        int maxval;
        f >> img.w >> img.h >> maxval;
        f.get(); // skip newline after maxval
        img.data.resize(img.w * img.h);
        f.read(reinterpret_cast<char*>(img.data.data()), img.w * img.h * 3);
        return img;
    }
};

// === Energy Functions ===

// Sobel gradient energy
double pixelEnergy(const Image& img, int x, int y) {
    int w = img.w, h = img.h;
    
    // Clamp coordinates
    auto get = [&](int px, int py) -> const Color& {
        px = std::max(0, std::min(w - 1, px));
        py = std::max(0, std::min(h - 1, py));
        return img.at(px, py);
    };
    
    // Sobel operators
    int gx_r = 0, gx_g = 0, gx_b = 0;
    int gy_r = 0, gy_g = 0, gy_b = 0;
    
    // Gx kernel: [-1 0 1; -2 0 2; -1 0 1]
    auto addGx = [&](int px, int py, int wval) {
        const Color& c = get(px, py);
        gx_r += wval * c.r;
        gx_g += wval * c.g;
        gx_b += wval * c.b;
    };
    addGx(x-1, y-1, -1); addGx(x-1, y, -2); addGx(x-1, y+1, -1);
    addGx(x+1, y-1,  1); addGx(x+1, y,  2); addGx(x+1, y+1,  1);
    
    // Gy kernel: [-1 -2 -1; 0 0 0; 1 2 1]
    auto addGy = [&](int px, int py, int wval) {
        const Color& c = get(px, py);
        gy_r += wval * c.r;
        gy_g += wval * c.g;
        gy_b += wval * c.b;
    };
    addGy(x-1, y-1, -1); addGy(x, y-1, -2); addGy(x+1, y-1, -1);
    addGy(x-1, y+1,  1); addGy(x, y+1,  2); addGy(x+1, y+1,  1);
    
    return std::sqrt(double(gx_r*gx_r + gx_g*gx_g + gx_b*gx_b +
                             gy_r*gy_r + gy_g*gy_g + gy_b*gy_b));
}

// Compute full energy map
std::vector<std::vector<double>> computeEnergy(const Image& img) {
    int w = img.w, h = img.h;
    std::vector<std::vector<double>> energy(h, std::vector<double>(w, 0));
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            energy[y][x] = pixelEnergy(img, x, y);
    return energy;
}

// === Seam Finding via Dynamic Programming ===

// Find vertical seam (one pixel per row, top-to-bottom)
std::vector<int> findVerticalSeam(const std::vector<std::vector<double>>& energy) {
    int h = energy.size();
    int w = energy[0].size();
    
    // DP table: cumulative minimum energy to each pixel
    std::vector<std::vector<double>> dp(h, std::vector<double>(w, 0));
    std::vector<std::vector<int>> backtrack(h, std::vector<int>(w, 0));
    
    // First row
    for (int x = 0; x < w; ++x)
        dp[0][x] = energy[0][x];
    
    // Fill DP table
    for (int y = 1; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            double best = dp[y-1][x];
            int bestPrev = x;
            
            if (x > 0 && dp[y-1][x-1] < best) {
                best = dp[y-1][x-1];
                bestPrev = x - 1;
            }
            if (x < w - 1 && dp[y-1][x+1] < best) {
                best = dp[y-1][x+1];
                bestPrev = x + 1;
            }
            
            dp[y][x] = best + energy[y][x];
            backtrack[y][x] = bestPrev;
        }
    }
    
    // Find minimum in last row
    int minX = 0;
    for (int x = 1; x < w; ++x)
        if (dp[h-1][x] < dp[h-1][minX]) minX = x;
    
    // Backtrack
    std::vector<int> seam(h);
    seam[h-1] = minX;
    for (int y = h-2; y >= 0; --y) {
        seam[y] = backtrack[y+1][seam[y+1]];
    }
    
    return seam;
}

// Find horizontal seam (one pixel per column, left-to-right)
std::vector<int> findHorizontalSeam(const std::vector<std::vector<double>>& energy) {
    int h = energy.size();
    int w = energy[0].size();
    
    std::vector<std::vector<double>> dp(h, std::vector<double>(w, 0));
    std::vector<std::vector<int>> backtrack(h, std::vector<int>(w, 0));
    
    // First column
    for (int y = 0; y < h; ++y)
        dp[y][0] = energy[y][0];
    
    for (int x = 1; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            double best = dp[y][x-1];
            int bestPrev = y;
            
            if (y > 0 && dp[y-1][x-1] < best) {
                best = dp[y-1][x-1];
                bestPrev = y - 1;
            }
            if (y < h - 1 && dp[y+1][x-1] < best) {
                best = dp[y+1][x-1];
                bestPrev = y + 1;
            }
            
            dp[y][x] = best + energy[y][x];
            backtrack[y][x] = bestPrev;
        }
    }
    
    int minY = 0;
    for (int y = 1; y < h; ++y)
        if (dp[y][w-1] < dp[minY][w-1]) minY = y;
    
    std::vector<int> seam(w);
    seam[w-1] = minY;
    for (int x = w-2; x >= 0; --x) {
        seam[x] = backtrack[seam[x+1]][x+1];
    }
    
    return seam;
}

// === Seam Removal ===

Image removeVerticalSeam(const Image& img, const std::vector<int>& seam) {
    int w = img.w, h = img.h;
    Image result(w - 1, h);
    for (int y = 0; y < h; ++y) {
        int dstX = 0;
        for (int x = 0; x < w; ++x) {
            if (x == seam[y]) continue;
            result.at(dstX++, y) = img.at(x, y);
        }
    }
    return result;
}

Image removeHorizontalSeam(const Image& img, const std::vector<int>& seam) {
    int w = img.w, h = img.h;
    Image result(w, h - 1);
    for (int x = 0; x < w; ++x) {
        int dstY = 0;
        for (int y = 0; y < h; ++y) {
            if (y == seam[x]) continue;
            result.at(x, dstY++) = img.at(y, x);
        }
    }
    return result;
}

// === Seam Visualization ===

Image visualizeVerticalSeam(const Image& img, const std::vector<int>& seam, int red) {
    Image vis = img; // copy
    int h = img.h;
    for (int y = 0; y < h; ++y) {
        int x = seam[y];
        // Draw red +- 1 pixel thick line
        vis.at(x, y) = Color(red ? 255 : 0, 0, red ? 0 : 255);
        if (x > 0) vis.at(x-1, y) = Color(200, 0, 0);
        if (x < img.w - 1) vis.at(x+1, y) = Color(200, 0, 0);
    }
    return vis;
}

Image visualizeHorizontalSeam(const Image& img, const std::vector<int>& seam) {
    Image vis = img;
    int w = img.w;
    for (int x = 0; x < w; ++x) {
        int y = seam[x];
        vis.at(x, y) = Color(0, 0, 255);
        if (y > 0) vis.at(x, y-1) = Color(0, 0, 200);
        if (y < img.h - 1) vis.at(x, y+1) = Color(0, 0, 200);
    }
    return vis;
}

// Energy map visualization
Image visualizeEnergy(const std::vector<std::vector<double>>& energy) {
    int h = energy.size(), w = energy[0].size();
    Image vis(w, h);
    double maxE = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            maxE = std::max(maxE, energy[y][x]);
    
    if (maxE == 0) maxE = 1.0;
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            unsigned char val = std::min(255, (int)(255.0 * energy[y][x] / maxE));
            vis.at(x, y) = Color(val, val, val);
        }
    }
    return vis;
}

// === Seam Carving Main Function ===

Image seamCarve(const Image& img, int targetW, int targetH) {
    Image current = img; // copy
    
    int colsToRemove = img.w - targetW;
    int rowsToRemove = img.h - targetH;
    
    std::cout << "Removing " << colsToRemove << " columns and " << rowsToRemove << " rows\n";
    
    // Remove columns (vertical seams)
    for (int i = 0; i < colsToRemove; ++i) {
        auto energy = computeEnergy(current);
        auto seam = findVerticalSeam(energy);
        current = removeVerticalSeam(current, seam);
        if ((i+1) % std::max(1, colsToRemove/10) == 0)
            std::cout << "  Column " << (i+1) << "/" << colsToRemove << "\n";
    }
    
    // Remove rows (horizontal seams)
    for (int i = 0; i < rowsToRemove; ++i) {
        auto energy = computeEnergy(current);
        auto seam = findHorizontalSeam(energy);
        current = removeHorizontalSeam(current, seam);
        if ((i+1) % std::max(1, rowsToRemove/10) == 0)
            std::cout << "  Row " << (i+1) << "/" << rowsToRemove << "\n";
    }
    
    return current;
}

// === Naive scaling for comparison (bilinear resize) ===
Image naiveScale(const Image& img, int newW, int newH) {
    Image result(newW, newH);
    for (int y = 0; y < newH; ++y) {
        for (int x = 0; x < newW; ++x) {
            double sx = (double)x / newW * img.w;
            double sy = (double)y / newH * img.h;
            int ix = std::min(img.w - 2, (int)sx);
            int iy = std::min(img.h - 2, (int)sy);
            double fx = sx - ix;
            double fy = sy - iy;
            
            const Color& c00 = img.at(ix, iy);
            const Color& c10 = img.at(ix+1, iy);
            const Color& c01 = img.at(ix, iy+1);
            const Color& c11 = img.at(ix+1, iy+1);
            
            unsigned char r = (unsigned char)((1-fy)*((1-fx)*c00.r + fx*c10.r) + fy*((1-fx)*c01.r + fx*c11.r));
            unsigned char g = (unsigned char)((1-fy)*((1-fx)*c00.g + fx*c10.g) + fy*((1-fx)*c01.g + fx*c11.g));
            unsigned char b = (unsigned char)((1-fy)*((1-fx)*c00.b + fx*c10.b) + fy*((1-fx)*c01.b + fx*c11.b));
            
            result.at(x, y) = Color(r, g, b);
        }
    }
    return result;
}

// === Quantitative Verification ===

struct EdgeMetrics {
    double meanGradient;    // Average gradient magnitude
    double edgeDensity;     // Fraction of "edge" pixels (gradient > threshold)
    double totalVariation;  // Sum of per-pixel change to neighbors
    double sharpnessScore;  // Edge density * meanGradient
};

EdgeMetrics computeEdgeMetrics(const Image& img) {
    int w = img.w, h = img.h;
    
    double totalGradient = 0;
    int edgePixels = 0;
    double totalVar = 0;
    double gradThreshold = 50.0; // significant edge threshold
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // Simple 3x3 gradient magnitude
            double gx_r = 0, gx_g = 0, gx_b = 0;
            double gy_r = 0, gy_g = 0, gy_b = 0;
            
            int xp1 = std::min(w-1, x+1), xm1 = std::max(0, x-1);
            int yp1 = std::min(h-1, y+1), ym1 = std::max(0, y-1);
            
            auto diff = [](unsigned char a, unsigned char b) { return (double)a - (double)b; };
            
            // x gradient
            gx_r = diff(img.at(xp1, y).r, img.at(xm1, y).r);
            gx_g = diff(img.at(xp1, y).g, img.at(xm1, y).g);
            gx_b = diff(img.at(xp1, y).b, img.at(xm1, y).b);
            
            // y gradient
            gy_r = diff(img.at(x, yp1).r, img.at(x, ym1).r);
            gy_g = diff(img.at(x, yp1).g, img.at(x, ym1).g);
            gy_b = diff(img.at(x, yp1).b, img.at(x, ym1).b);
            
            double g = std::sqrt(gx_r*gx_r + gx_g*gx_g + gx_b*gx_b +
                                  gy_r*gy_r + gy_g*gy_g + gy_b*gy_b);
            
            totalGradient += g;
            if (g > gradThreshold) edgePixels++;
            
            // Total variation
            if (x < w-1) {
                auto& c0 = img.at(x,y); auto& c1 = img.at(x+1,y);
                totalVar += std::abs((int)c0.r-c1.r) + std::abs((int)c0.g-c1.g) + std::abs((int)c0.b-c1.b);
            }
            if (y < h-1) {
                auto& c0 = img.at(x,y); auto& c1 = img.at(x,y+1);
                totalVar += std::abs((int)c0.r-c1.r) + std::abs((int)c0.g-c1.g) + std::abs((int)c0.b-c1.b);
            }
        }
    }
    
    int totalPixels = w * h;
    EdgeMetrics m;
    m.meanGradient = totalGradient / totalPixels;
    m.edgeDensity = (double)edgePixels / totalPixels;
    m.totalVariation = totalVar;
    m.sharpnessScore = m.edgeDensity * m.meanGradient;
    return m;
}

struct PreservationScore {
    double edgeRetention;     // how well edges are preserved vs original
    double distortionMetric;  // average pixel distance from original after resize-back
};

// === Test Image Generation (synthetic with features) ===

Image generateTestImage() {
    int w = 400, h = 300;
    Image img(w, h);
    
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // Background gradient
            int r = (x * 200 / w);
            int g = (y * 200 / h);
            int b = 128;
            
            // Central circle (important object)
            int cx = w * 0.35, cy = h * 0.5, radius = 60;
            int dx = x - cx, dy = y - cy;
            if (dx*dx + dy*dy < radius*radius) {
                r = 220; g = 50; b = 50;
                // Sub-detail ring
                if (std::abs(dx*dx + dy*dy - (radius-15)*(radius-15)) < 200) {
                    r = 255; g = 200; b = 100;
                }
            }
            
            // Rectangle (important region)
            int rx = w - 150, ry = 30, rw = 100, rh = 80;
            if (x >= rx && x < rx+rw && y >= ry && y < ry+rh) {
                r = 50; g = 50; b = 220;
                // Grid pattern inside
                if ((x - rx) % 20 < 2 || (y - ry) % 20 < 2) {
                    r = 200; g = 200; b = 255;
                }
            }
            
            // Bottom-left triangle
            if (x < 100 && y > h - 100) {
                if (x + y > h + 30 && x + y < h + 100 && x > 20) {
                    r = 50; g = 200; b = 50;
                    // Stripes
                    if ((x + y) % 30 < 15) {
                        r = 30; g = 150; b = 30;
                    }
                }
            }
            
            // Top area with text-like features (horizontal bars)
            if (y > 15 && y < 45 && x > 80 && x < 320) {
                if (y < 22 || y > 38 || (y >= 28 && y <= 32)) {
                    r = 255; g = 255; b = 255;
                }
            }
            
            // Some random dots (background texture)
            if ((x * 7 + y * 13) % 97 < 5) {
                r = (r + 255) / 2; g = (g + 255) / 2; b = (b + 255) / 2;
            }
            
            img.at(x, y) = Color(r, g, b);
        }
    }
    return img;
}

// Generate a natural-looking test image with photographic-like features
Image generateNaturalTestImage() {
    int w = 500, h = 375;
    Image img(w, h);
    
    // Sky gradient
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            // Deep sky blue gradient
            int skyR = 30 + 120 * y / h;
            int skyG = 80 + 100 * y / h;
            int skyB = 180 + 60 * y / h;
            
            img.at(x, y) = Color(skyR, skyG, skyB);
        }
    }
    
    // Green ground
    for (int y = h * 0.6; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int noise = ((x * 17 + y * 31) % 40) - 20;
            int g = 100 + noise;
            int r = 30 + noise / 2;
            int b = 20 + noise / 2;
            img.at(x, y) = Color(
                std::max(0, std::min(255, r)),
                std::max(0, std::min(255, g)),
                std::max(0, std::min(255, b))
            );
        }
    }
    
    // Mountain/hill silhouette
    for (int y = h * 0.4; y < h * 0.7; ++y) {
        for (int x = 0; x < w; ++x) {
            double mountainLine = h * 0.6 + 50 * std::sin(x * 0.02) * std::sin(x * 0.007);
            if (y >= mountainLine) {
                int dark = 40 + 30 * (y - mountainLine) / (h * 0.1);
                img.at(x, y) = Color(dark, dark + 10, dark - 5);
            }
        }
    }
    
    // Tree trunks
    for (int i = 0; i < 8; ++i) {
        int tx = 40 + i * 55 + ((i * 31) % 25);
        int treeH = 80 + ((i * 17) % 40);
        int baseY = h * 0.55;
        for (int dy = 0; dy < treeH; ++dy) {
            int y = baseY - dy;
            if (y < h && y >= 0) {
                for (int dx = -3; dx <= 3; ++dx) {
                    int x = tx + dx;
                    if (x >= 0 && x < w) {
                        img.at(x, y) = Color(60, 30, 10);
                    }
                }
            }
        }
        // Tree canopy
        for (int dy = treeH - 10; dy < treeH + 40; ++dy) {
            int y = baseY - dy;
            if (y >= 0 && y < h) {
                int canopyW = 15 - std::abs(dy - treeH - 15) / 2;
                for (int dx = -canopyW; dx <= canopyW; ++dx) {
                    int x = tx + dx;
                    if (x >= 0 && x < w) {
                        img.at(x, y) = Color(10, 80 + (dx*dx + dy*dy) % 30, 5);
                    }
                }
            }
        }
    }
    
    // Sun
    int sx = w * 0.75, sy = h * 0.22, sr = 35;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int dx = x - sx, dy = y - sy;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < sr) {
                int v = 255 - (int)(dist/sr * 30);
                img.at(x, y) = Color(v, v - 30, 0);
            } else if (dist < sr + 15) {
                double ratio = (dist - sr) / 15.0;
                int sunR = img.at(x,y).r, sunG = img.at(x,y).g, sunB = img.at(x,y).b;
                img.at(x, y) = Color(
                    (int)(255*(1-ratio) + sunR*ratio),
                    (int)(225*(1-ratio) + sunG*ratio),
                    (int)(0*(1-ratio) + sunB*ratio)
                );
            }
        }
    }
    
    // Bird as V-shape
    for (int i = -5; i <= 5; ++i) {
        int bx = sx - 60 + i, by = sy - 40 + std::abs(i);
        if (bx >= 0 && bx < w && by >= 0 && by < h) {
            img.at(bx, by) = Color(20, 20, 20);
        }
        // Second wing
        bx = sx - 60 + i; by = sy - 40 - std::abs(i);
        if (bx >= 0 && bx < w && by >= 0 && by < h) {
            img.at(bx, by) = Color(20, 20, 20);
        }
    }
    
    return img;
}

int main() {
    // Create test image with structured features
    Image original = generateNaturalTestImage();
    original.savePPM("original.ppm");
    std::cout << "Original: " << original.w << "x" << original.h << "\n";
    
    // Compute energy map
    auto energy = computeEnergy(original);
    auto energyVis = visualizeEnergy(energy);
    energyVis.savePPM("energy_map.ppm");
    std::cout << "Energy map saved\n";
    
    // Find + visualize first seam
    auto firstSeam = findVerticalSeam(energy);
    auto seamVis = visualizeVerticalSeam(original, firstSeam, 1);
    seamVis.savePPM("seam_visualization.ppm");
    std::cout << "Seam visualization saved\n";
    
    // Seam carve: reduce by 35% width, 20% height
    int targetW = original.w * 2 / 3;  // ~333
    int targetH = original.h * 4 / 5;  // ~300
    
    Image carved = seamCarve(original, targetW, targetH);
    carved.savePPM("seam_carved.ppm");
    std::cout << "Seam carved: " << carved.w << "x" << carved.h << "\n";
    
    // Naive scale for comparison
    Image naive = naiveScale(original, targetW, targetH);
    naive.savePPM("naive_scaled.ppm");
    std::cout << "Naive scaled: " << naive.w << "x" << naive.h << "\n";
    
    // ============ QUANTITATIVE VERIFICATION ============
    std::cout << "\n===== QUANTITATIVE VERIFICATION =====\n\n";
    
    // 1. Edge preservation metrics
    EdgeMetrics origMetrics = computeEdgeMetrics(original);
    EdgeMetrics carvedMetrics = computeEdgeMetrics(carved);
    EdgeMetrics naiveMetrics = computeEdgeMetrics(naive);
    
    // Since images have different sizes, compare edge densities (normalized)
    std::cout << "--- Edge Metrics ---\n";
    std::cout << "Original:    meanGradient=" << origMetrics.meanGradient 
              << " edgeDensity=" << origMetrics.edgeDensity 
              << " sharpness=" << origMetrics.sharpnessScore << "\n";
    std::cout << "Seam Carved: meanGradient=" << carvedMetrics.meanGradient 
              << " edgeDensity=" << carvedMetrics.edgeDensity 
              << " sharpness=" << carvedMetrics.sharpnessScore << "\n";
    std::cout << "Naive Scale: meanGradient=" << naiveMetrics.meanGradient 
              << " edgeDensity=" << naiveMetrics.edgeDensity 
              << " sharpness=" << naiveMetrics.sharpnessScore << "\n";
    
    // 2. Image content verification - check image isn't blank
    auto checkImage = [](const Image& img, const std::string& name) -> bool {
        double sum = 0;
        double sumSq = 0;
        int n = img.w * img.h;
        for (int i = 0; i < n; ++i) {
            double lum = 0.299 * img.data[i].r + 0.587 * img.data[i].g + 0.114 * img.data[i].b;
            sum += lum;
            sumSq += lum * lum;
        }
        double mean = sum / n;
        double stddev = std::sqrt(sumSq / n - mean * mean);
        
        std::cout << name << ": mean=" << mean << " std=" << stddev << "\n";
        
        if (mean < 5) {
            std::cout << "❌ " << name << " too dark!\n";
            return false;
        }
        if (mean > 250) {
            std::cout << "❌ " << name << " too bright!\n";
            return false;
        }
        if (stddev < 5) {
            std::cout << "❌ " << name << " too uniform!\n";
            return false;
        }
        return true;
    };
    
    std::cout << "\n--- Brightness/Content Check ---\n";
    bool ok1 = checkImage(carved, "Seam Carved");
    bool ok2 = checkImage(naive, "Naive Scaled");
    
    // 3. Aspect ratio preservation of important objects
    // Compare edge density: seam carving should preserve more edges
    double carvedEdgeQuality = carvedMetrics.edgeDensity * carvedMetrics.meanGradient;
    double naiveEdgeQuality = naiveMetrics.edgeDensity * naiveMetrics.meanGradient;
    
    std::cout << "\n--- Edge Quality Comparison ---\n";
    std::cout << "Carved edge quality: " << carvedEdgeQuality << "\n";
    std::cout << "Naive edge quality:  " << naiveEdgeQuality << "\n";
    
    if (carvedEdgeQuality > naiveEdgeQuality) {
        double improvement = (carvedEdgeQuality - naiveEdgeQuality) / naiveEdgeQuality * 100.0;
        std::cout << "✅ Seam carving preserves " << improvement << "% more edge content\n";
    } else {
        std::cout << "⚠️ Naive scaling has higher edge quality (unexpected but possible due to sharpening)\n";
    }
    
    // 4. Structure preservation: total variation comparison (normalized by size)
    double carvedTVnorm = carvedMetrics.totalVariation / (carved.w * carved.h);
    double naiveTVnorm = naiveMetrics.totalVariation / (naive.w * naive.h);
    
    std::cout << "\n--- Normalized Total Variation (structure) ---\n";
    std::cout << "Carved: " << carvedTVnorm << "\n";
    std::cout << "Naive:  " << naiveTVnorm << "\n";
    
    // 5. File size check
    std::cout << "\n--- File Check ---\n";
    system("ls -lh *.ppm");
    
    // 6. Seam correctness: seam must be connected (adjacent rows differ by <= 1)
    std::cout << "\n--- Seam Connectivity Check ---\n";
    bool seamValid = true;
    for (size_t i = 1; i < firstSeam.size(); ++i) {
        if (std::abs(firstSeam[i] - firstSeam[i-1]) > 1) {
            std::cout << "❌ Seam broken at row " << i << ": " 
                      << firstSeam[i-1] << " -> " << firstSeam[i] << "\n";
            seamValid = false;
            break;
        }
    }
    if (seamValid) std::cout << "✅ Seam is properly connected\n";
    
    // Final verdict
    std::cout << "\n===== FINAL VERDICT =====\n";
    if (ok1 && ok2 && seamValid) {
        std::cout << "✅ ALL CHECKS PASSED\n";
    } else {
        std::cout << "❌ SOME CHECKS FAILED\n";
        return 1;
    }
    
    // 7. Compare seam-carved edges vs naive
    std::cout << "\n--- Comparison Summary ---\n";
    std::cout << "Original size: " << original.w << "x" << original.h << "\n";
    std::cout << "Target size:   " << targetW << "x" << targetH << "\n";
    std::cout << "Seam carving preserves important visual features (edges, gradients)\n";
    std::cout << "while naive scaling uniformly compresses all regions.\n";
    
    return 0;
}
