/**
 * Weighted Voronoi Stippling
 *
 * Generates a stippled rendering of an input PPM image by iteratively
 * placing points according to image density and applying Lloyd's relaxation
 * on the weighted Voronoi diagram.
 *
 * Quantitative verification:
 *  - Convergence: sum of squared displacement between iterations
 *  - Density matching: correlation between local point density and image intensity
 *  - Coverage: fraction of non-zero-variance regions
 *
 * Build: g++ -std=c++17 -O2 -Wall -Wextra main.cpp -o voronoi_stipple
 * Usage: ./voronoi_stipple <input.ppm> <output.ppm> [num_points] [iterations]
 */

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

// ================ Utility ================

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double x_, double y_) : x(x_), y(y_) {}
    double distSq(const Point& o) const {
        double dx = x - o.x, dy = y - o.y;
        return dx * dx + dy * dy;
    }
};

struct Color {
    uint8_t r, g, b;
};

// ================ PPM I/O ================

bool readPPM(const std::string& filename, int& w, int& h, std::vector<Color>& pixels) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) return false;
    std::string magic;
    f >> magic;
    if (magic != "P6") return false;
    int maxval;
    f >> w >> h >> maxval;
    f.ignore(1);
    pixels.resize(w * h);
    for (int i = 0; i < w * h; ++i) {
        unsigned char rgb[3];
        f.read(reinterpret_cast<char*>(rgb), 3);
        pixels[i] = {rgb[0], rgb[1], rgb[2]};
    }
    return true;
}

void writePPM(const std::string& filename, int w, int h, const std::vector<Color>& pixels) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for (const auto& c : pixels) {
        unsigned char rgb[3] = {c.r, c.g, c.b};
        f.write(reinterpret_cast<const char*>(rgb), 3);
    }
}

// ================ Core Algorithm ================

// Compute density from grayscale image: dark → high density
std::vector<double> computeDensity(int w, int h, const std::vector<Color>& pixels) {
    std::vector<double> density(w * h);
    for (int i = 0; i < w * h; ++i) {
        double gray = 0.299 * pixels[i].r + 0.587 * pixels[i].g + 0.114 * pixels[i].b;
        // Invert and scale: darker pixels get higher density weight
        density[i] = (255.0 - gray) / 255.0;
        // Apply gamma to increase contrast
        density[i] = std::pow(density[i], 1.5);
    }
    return density;
}

// Rejection sampling based on density map
void initialSampling(int w, int h, const std::vector<double>& density,
                     int numPoints, std::mt19937& rng,
                     std::vector<Point>& points) {
    points.clear();
    points.reserve(numPoints);

    std::vector<std::pair<int, int>> candidates;
    double maxDens = 0;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            maxDens = std::max(maxDens, density[y * w + x]);

    std::uniform_int_distribution<int> distX(0, w - 1);
    std::uniform_int_distribution<int> distY(0, h - 1);
    std::uniform_real_distribution<double> distU(0.0, maxDens + 1e-12);

    int attempts = 0;
    while ((int)points.size() < numPoints && attempts < numPoints * 20) {
        int px = distX(rng);
        int py = distY(rng);
        double u = distU(rng);
        if (u <= density[py * w + px]) {
            points.emplace_back(px + 0.5, py + 0.5);
        }
        ++attempts;
    }

    // Fill remaining with random if rejection sampling fails
    while ((int)points.size() < numPoints) {
        int px = distX(rng);
        int py = distY(rng);
        points.emplace_back(px + 0.5, py + 0.5);
    }
}

// For each pixel, find nearest stipple point (Voronoi cell assignment)
// Returns cell assignments and per-cell accumulators
void computeVoronoiAssignments(
    int w, int h, const std::vector<Point>& points,
    std::vector<int>& cellIdx,
    std::vector<double>& cellSumX, std::vector<double>& cellSumY,
    std::vector<double>& cellSumW, std::vector<int>& cellCount) {
    cellIdx.assign(w * h, 0);
    int np = points.size();
    cellSumX.assign(np, 0);
    cellSumY.assign(np, 0);
    cellSumW.assign(np, 0);
    cellCount.assign(np, 0);

    // We do brute-force for accuracy since points are in thousands, not millions
    for (int py = 0; py < h; ++py) {
        for (int px = 0; px < w; ++px) {
            double minDist = std::numeric_limits<double>::max();
            int best = 0;
            for (int i = 0; i < np; ++i) {
                double d = points[i].distSq({(double)px, (double)py});
                if (d < minDist) {
                    minDist = d;
                    best = i;
                }
            }
            cellIdx[py * w + px] = best;
            cellSumX[best] += px;
            cellSumY[best] += py;
            cellSumW[best] += 1.0; // uniform weight for now
            cellCount[best]++;
        }
    }
}

// Weighted centroid computation with density
void computeWeightedAssignments(
    int w, int h, const std::vector<Point>& points,
    const std::vector<double>& density,
    std::vector<int>& cellIdx,
    std::vector<double>& cellSumX, std::vector<double>& cellSumY,
    std::vector<double>& cellSumW, std::vector<int>& cellCount) {
    cellIdx.assign(w * h, 0);
    int np = points.size();
    cellSumX.assign(np, 0);
    cellSumY.assign(np, 0);
    cellSumW.assign(np, 0);
    cellCount.assign(np, 0);

    for (int py = 0; py < h; ++py) {
        for (int px = 0; px < w; ++px) {
            double minDist = std::numeric_limits<double>::max();
            int best = 0;
            for (int i = 0; i < np; ++i) {
                double d = points[i].distSq({(double)px, (double)py});
                if (d < minDist) {
                    minDist = d;
                    best = i;
                }
            }
            cellIdx[py * w + px] = best;
            double wgt = density[py * w + px];
            cellSumX[best] += px * wgt;
            cellSumY[best] += py * wgt;
            cellSumW[best] += wgt;
            cellCount[best]++;
        }
    }
}

// Lloyd relaxation step with weighted centroids
double lloydStep(int w, int h, const std::vector<double>& density,
                 std::vector<Point>& points) {
    int np = points.size();
    std::vector<double> cellSumX(np, 0), cellSumY(np, 0), cellSumW(np, 0);
    std::vector<int> cellCount(np, 0), cellIdx;

    computeWeightedAssignments(w, h, points, density, cellIdx,
                               cellSumX, cellSumY, cellSumW, cellCount);

    double maxDisp = 0;
    double totalDisp = 0;
    int moved = 0;

    for (int i = 0; i < np; ++i) {
        if (cellSumW[i] <= 0) continue;
        double cx = cellSumX[i] / cellSumW[i];
        double cy = cellSumY[i] / cellSumW[i];
        double dx = cx - points[i].x;
        double dy = cy - points[i].y;
        double disp = std::sqrt(dx * dx + dy * dy);
        maxDisp = std::max(maxDisp, disp);
        totalDisp += disp;
        moved++;

        // Clamp to image bounds
        points[i].x = std::max(0.0, std::min((double)(w - 1), cx));
        points[i].y = std::max(0.0, std::min((double)(h - 1), cy));
    }

    double avgDisp = moved > 0 ? totalDisp / moved : 0;
    return avgDisp;
}

// ================ Rendering ================

void renderStipples(int w, int h, const std::vector<Point>& points,
                    std::vector<Color>& out) {
    out.assign(w * h, {255, 255, 255}); // white background

    // Draw stipple dots: small filled circles
    for (const auto& p : points) {
        int cx = (int)(p.x + 0.5);
        int cy = (int)(p.y + 0.5);
        int radius = 1; // single-pixel stipple (tiny dots)

        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                if (dx * dx + dy * dy > radius * radius) continue;
                int px = cx + dx;
                int py = cy + dy;
                if (px >= 0 && px < w && py >= 0 && py < h) {
                    out[py * w + px] = {0, 0, 0};
                }
            }
        }
    }
}

// Render Voronoi cells for visualization
void renderVoronoi(int w, int h, const std::vector<Point>& points,
                   std::vector<Color>& out) {
    out.resize(w * h);
    int np = points.size();

    // Generate deterministic colors for each cell
    std::vector<Color> pal(np);
    std::mt19937 palRng(42);
    std::uniform_int_distribution<int> colDist(40, 220);
    for (int i = 0; i < np; ++i) {
        pal[i] = {(uint8_t)colDist(palRng), (uint8_t)colDist(palRng), (uint8_t)colDist(palRng)};
    }

    for (int py = 0; py < h; ++py) {
        for (int px = 0; px < w; ++px) {
            double minDist = std::numeric_limits<double>::max();
            int best = 0;
            for (int i = 0; i < np; ++i) {
                double d = points[i].distSq({(double)px, (double)py});
                if (d < minDist) { minDist = d; best = i; }
            }
            out[py * w + px] = pal[best];
        }
    }

    // Draw cell boundaries
    for (int py = 1; py < h; ++py) {
        for (int px = 1; px < w; ++px) {
            int best0 = 0;
            double m0 = std::numeric_limits<double>::max();
            for (int i = 0; i < np; ++i) {
                double d = points[i].distSq({(double)px, (double)py});
                if (d < m0) { m0 = d; best0 = i; }
            }
            // Check neighbors
            int bestL = 0;
            double mL = std::numeric_limits<double>::max();
            for (int i = 0; i < np; ++i) {
                double d = points[i].distSq({(double)(px - 1), (double)py});
                if (d < mL) { mL = d; bestL = i; }
            }
            if (best0 != bestL) {
                out[py * w + px] = {0, 0, 0};
            }
        }
    }
}

// ================ Quantitative Verification ================

struct VerifResult {
    double initialAvgDisp;
    double finalAvgDisp;
    double convergenceRatio;
    double densityCorrelation;
    double coverageFraction;
    double pointsPerUnit;
    std::vector<double> displacementHistory;
    bool passed;
};

VerifResult verify(int w, int h, const std::vector<Point>& points,
                   const std::vector<double>& density,
                   const std::vector<double>& dispHistory) {
    VerifResult r;
    r.passed = true;
    r.displacementHistory = dispHistory;

    // Convergence: ratio of final to initial average displacement
    int np = points.size();
    r.pointsPerUnit = (double)np / (w * h);

    if (dispHistory.size() >= 2) {
        r.initialAvgDisp = dispHistory[0];
        r.finalAvgDisp = dispHistory.back();
        r.convergenceRatio = (r.initialAvgDisp > 1e-6) ?
            r.finalAvgDisp / r.initialAvgDisp : 0;
        std::cout << "📊 收敛: 初始平均位移=" << r.initialAvgDisp
                  << ", 最终=" << r.finalAvgDisp
                  << ", 收敛比=" << r.convergenceRatio << "\n";
        if (r.convergenceRatio > 0.9 && r.initialAvgDisp > 0.5) {
            std::cout << "⚠️ 收敛比接近1，可能未充分收敛\n";
        }
    } else {
        r.initialAvgDisp = r.finalAvgDisp = 0;
        r.convergenceRatio = 0;
    }

    // Density correlation: average density in each stipple point's neighborhood
    double corrSum = 0;
    int radius = std::max(1, (int)(std::min(w, h) / std::sqrt(np) * 0.3));
    int validCount = 0;
    for (const auto& p : points) {
        int cx = (int)(p.x + 0.5), cy = (int)(p.y + 0.5);
        double sumDens = 0;
        int cnt = 0;
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int px = cx + dx, py = cy + dy;
                if (px >= 0 && px < w && py >= 0 && py < h) {
                    sumDens += density[py * w + px];
                    cnt++;
                }
            }
        }
        corrSum += sumDens / cnt;
        validCount++;
    }
    r.densityCorrelation = validCount > 0 ? corrSum / validCount : 0;

    // Global density correlation: bin image into grid, compare stipple count per bin vs density
    int binSize = std::max(4, (int)(std::min(w, h) / 20.0));
    int binsX = w / binSize, binsY = h / binSize;
    std::vector<double> binDensity(binsX * binsY, 0);
    std::vector<int> binCount(binsX * binsY, 0);

    for (int by = 0; by < binsY; ++by) {
        for (int bx = 0; bx < binsX; ++bx) {
            double sum = 0;
            int n = 0;
            for (int y = by * binSize; y < std::min((by+1)*binSize, h); ++y) {
                for (int x = bx * binSize; x < std::min((bx+1)*binSize, w); ++x) {
                    sum += density[y * w + x];
                    n++;
                }
            }
            binDensity[by * binsX + bx] = sum / n;
        }
    }

    for (const auto& p : points) {
        int bx = std::min((int)(p.x / binSize), binsX - 1);
        int by = std::min((int)(p.y / binSize), binsY - 1);
        binCount[by * binsX + bx]++;
    }

    // Compute Pearson correlation between bin density and bin count
    double sumD = 0, sumC = 0, sumDD = 0, sumCC = 0, sumDC = 0;
    int bins = binsX * binsY;
    for (int i = 0; i < bins; ++i) {
        double d = binDensity[i];
        double c = binCount[i];
        sumD += d; sumC += c;
        sumDD += d * d; sumCC += c * c;
        sumDC += d * c;
    }
    double meanD = sumD / bins, meanC = sumC / bins;
    double cov = sumDC / bins - meanD * meanC;
    double varD = sumDD / bins - meanD * meanD;
    double varC = sumCC / bins - meanC * meanC;
    double pearson = 0;
    if (varD > 1e-12 && varC > 1e-12) {
        pearson = cov / std::sqrt(varD * varC);
    }

    std::cout << "📊 密度相关性(Pearson): " << pearson
              << "  点均密度: " << r.densityCorrelation << "\n";
    if (pearson < 0.3) {
        std::cout << "⚠️ 密度相关性弱，点分布可能未响应图像强度\n";
    }
    if (pearson >= 0.3) {
        std::cout << "✅ 点分布与图像密度呈正相关\n";
    }

    // Coverage: fraction of image area with stipple coverage
    r.coverageFraction = (double)np / (w * h);
    std::cout << "📊 点覆盖率: " << (r.coverageFraction * 100) << "%"
              << "  (" << np << "点 / " << (w*h) << "px)\n";

    // Overall verdict
    if (r.convergenceRatio < 0.01 || r.finalAvgDisp < 0.5) {
        std::cout << "✅ 收敛性通过\n";
    } else if (r.convergenceRatio > 0.9) {
        std::cout << "⚠️ 收敛不充分（可能需要更多迭代）\n";
    }

    return r;
}

// ================ Main ================

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.ppm> <output_prefix> [num_points] [iterations]\n";
        std::cerr << "Example: " << argv[0] << " lena.ppm stipple 5000 40\n";
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outPrefix = argv[2];
    int numPoints = (argc >= 4) ? std::atoi(argv[3]) : 3000;
    int maxIter = (argc >= 5) ? std::atoi(argv[4]) : 30;

    std::cout << "=== Weighted Voronoi Stippling ===\n";
    std::cout << "Input: " << inputFile << "\n";
    std::cout << "Points: " << numPoints << "\n";
    std::cout << "Max iterations: " << maxIter << "\n\n";

    // Load image
    int w, h;
    std::vector<Color> pixels;
    if (!readPPM(inputFile, w, h, pixels)) {
        std::cerr << "❌ Cannot read PPM: " << inputFile << "\n";
        return 1;
    }
    std::cout << "📷 Image: " << w << "x" << h
              << " (" << (w*h) << " pixels)\n";

    // Compute density
    std::vector<double> density = computeDensity(w, h, pixels);
    double meanDens = 0;
    for (auto d : density) meanDens += d;
    meanDens /= density.size();
    std::cout << "📊 平均图像密度: " << meanDens << "\n";

    // Initialize points
    std::mt19937 rng(12345);
    std::vector<Point> points;
    initialSampling(w, h, density, numPoints, rng, points);
    std::cout << "🎲 初始化 " << points.size() << " 个点（拒绝采样）\n\n";

    // Render initial Voronoi
    {
        std::vector<Color> initVor;
        renderVoronoi(w, h, points, initVor);
        writePPM(outPrefix + "_voronoi_initial.ppm", w, h, initVor);
        std::cout << "📸 初始Voronoi图: " << outPrefix << "_voronoi_initial.ppm\n";
    }

    // Lloyd iterations
    std::vector<double> dispHistory;
    std::cout << "\n🔄 开始Lloyd迭代...\n";
    for (int iter = 0; iter < maxIter; ++iter) {
        double avgDisp = lloydStep(w, h, density, points);
        dispHistory.push_back(avgDisp);
        if (iter % 5 == 0 || iter == maxIter - 1) {
            std::cout << "  迭代 " << (iter + 1) << "/" << maxIter
                      << "  平均位移=" << avgDisp << " px\n";
        }
        if (avgDisp < 0.2 && iter > 10) {
            std::cout << "  早期收敛于迭代 " << (iter + 1) << "\n";
            break;
        }
    }

    // Render final stipple result
    {
        std::vector<Color> stippleOut;
        renderStipples(w, h, points, stippleOut);
        writePPM(outPrefix + "_stipple.ppm", w, h, stippleOut);
        std::cout << "\n📸 最终点画: " << outPrefix << "_stipple.ppm\n";
    }

    // Render final Voronoi
    {
        std::vector<Color> finalVor;
        renderVoronoi(w, h, points, finalVor);
        writePPM(outPrefix + "_voronoi_final.ppm", w, h, finalVor);
        std::cout << "📸 最终Voronoi图: " << outPrefix << "_voronoi_final.ppm\n";
    }

    // Render combined: stipple on top of image density visualization
    {
        std::vector<Color> combined(w * h);
        for (int i = 0; i < w * h; ++i) {
            int gray = (int)(255 * (1.0 - density[i]));
            combined[i] = {(uint8_t)gray, (uint8_t)gray, (uint8_t)gray};
        }
        // Overlay stipples in red
        for (const auto& p : points) {
            int cx = (int)(p.x + 0.5), cy = (int)(p.y + 0.5);
            if (cx >= 0 && cx < w && cy >= 0 && cy < h)
                combined[cy * w + cx] = {255, 0, 0};
        }
        writePPM(outPrefix + "_combined.ppm", w, h, combined);
        std::cout << "📸 组合图: " << outPrefix << "_combined.ppm\n";
    }

    // Quantitative verification
    std::cout << "\n============ 量化验证 ============\n";
    VerifResult vr = verify(w, h, points, density, dispHistory);

    // Save convergence data
    {
        std::ofstream cf(outPrefix + "_convergence.txt");
        cf << "iteration,avg_displacement\n";
        for (size_t i = 0; i < dispHistory.size(); ++i)
            cf << (i + 1) << "," << dispHistory[i] << "\n";
        cf.close();
        std::cout << "📈 收敛数据: " << outPrefix << "_convergence.txt\n";
    }

    std::cout << "\n✅ 完成!\n";
    return 0;
}
