// Histogram Equalization with Quantifiable Verification
// Implements global histogram equalization on grayscale PPM images
//
// Quantification metrics:
//   1. Cumulative Distribution Function (CDF) flatness after equalization
//   2. Histogram entropy comparison (before vs after)
//   3. Contrast improvement ratio (std dev before vs after)
//   4. KL divergence from uniform distribution

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <sstream>

// ========== Image I/O (PPM P6) ==========
struct Image {
    int width, height, maxval;
    std::vector<unsigned char> data; // RGB packed

    bool loadPPM(const std::string& path) {
        std::ifstream in(path, std::ios::binary);
        if (!in) return false;
        std::string magic;
        in >> magic;
        if (magic != "P6") { std::cerr << "Not P6 PPM\n"; return false; }
        in >> width >> height >> maxval;
        in.get(); // consume newline
        data.resize(width * height * 3);
        in.read(reinterpret_cast<char*>(data.data()), data.size());
        return in.good();
    }

    bool savePPM(const std::string& path) {
        std::ofstream out(path, std::ios::binary);
        out << "P6\n" << width << " " << height << "\n" << maxval << "\n";
        out.write(reinterpret_cast<const char*>(data.data()), data.size());
        return out.good();
    }

    // Convert to grayscale luminance
    unsigned char luminance(int x, int y) const {
        int idx = (y * width + x) * 3;
        return static_cast<unsigned char>(
            0.299 * data[idx] + 0.587 * data[idx + 1] + 0.114 * data[idx + 2]);
    }
};

// ========== Histogram ==========
struct Histogram {
    std::vector<int> bins;
    std::vector<double> pdf;
    std::vector<double> cdf;

    void compute(const Image& img, int numBins = 256) {
        bins.assign(numBins, 0);
        int total = img.width * img.height;
        for (int y = 0; y < img.height; y++)
            for (int x = 0; x < img.width; x++)
                bins[img.luminance(x, y)]++;

        pdf.resize(numBins);
        for (int i = 0; i < numBins; i++)
            pdf[i] = static_cast<double>(bins[i]) / total;

        cdf.resize(numBins);
        cdf[0] = pdf[0];
        for (int i = 1; i < numBins; i++)
            cdf[i] = cdf[i - 1] + pdf[i];
    }

    double entropy() const {
        double H = 0.0;
        for (double p : pdf)
            if (p > 0) H -= p * std::log2(p);
        return H; // max for 256 bins is 8.0
    }

    double klDivergenceFromUniform() const {
        double uniform = 1.0 / bins.size();
        double kl = 0.0;
        for (double p : pdf)
            if (p > 0) kl += p * std::log2(p / uniform);
        return kl;
    }

    // Kolmogorov-Smirnov statistic: max deviation from uniform CDF
    double ksStatistic() const {
        double maxDev = 0.0;
        int N = static_cast<int>(bins.size());
        for (int i = 0; i < N; i++) {
            double uniformCDF = static_cast<double>(i + 1) / N;
            double dev = std::abs(cdf[i] - uniformCDF);
            if (dev > maxDev) maxDev = dev;
        }
        return maxDev;
    }
};

// ========== Histogram Equalization ==========
Image equalize(const Image& src) {
    Image dst = src;
    int total = src.width * src.height;

    // Step 1: compute histogram
    std::vector<int> hist(256, 0);
    for (int y = 0; y < src.height; y++)
        for (int x = 0; x < src.width; x++)
            hist[src.luminance(x, y)]++;

    // Step 2: compute CDF
    std::vector<int> cdf(256, 0);
    cdf[0] = hist[0];
    for (int i = 1; i < 256; i++)
        cdf[i] = cdf[i - 1] + hist[i];

    // Step 3: find cdf_min (first non-zero)
    int cdf_min = 0;
    for (int i = 0; i < 256; i++) {
        if (cdf[i] > 0) { cdf_min = cdf[i]; break; }
    }

    // Step 4: build lookup table
    std::vector<unsigned char> lut(256);
    double scale = 255.0 / (total - cdf_min);
    for (int i = 0; i < 256; i++) {
        if (hist[i] == 0) { lut[i] = 0; continue; }
        int val = static_cast<int>(std::round((cdf[i] - cdf_min) * scale));
        lut[i] = static_cast<unsigned char>(std::max(0, std::min(255, val)));
    }

    // Step 5: apply LUT to all 3 channels pixel-by-pixel using luminance mapping
    // Preserve hue by scaling RGB proportionally
    for (int y = 0; y < dst.height; y++) {
        for (int x = 0; x < dst.width; x++) {
            int idx = (y * dst.width + x) * 3;
            unsigned char r = src.data[idx];
            unsigned char g = src.data[idx + 1];
            unsigned char b = src.data[idx + 2];
            double lum = 0.299 * r + 0.587 * g + 0.114 * b;
            if (lum < 1.0) {
                dst.data[idx] = lut[static_cast<int>(lum)];
                dst.data[idx + 1] = lut[static_cast<int>(lum)];
                dst.data[idx + 2] = lut[static_cast<int>(lum)];
            } else {
                double ratio = lut[static_cast<int>(lum)] / lum;
                dst.data[idx] = static_cast<unsigned char>(std::max(0.0, std::min(255.0, r * ratio)));
                dst.data[idx + 1] = static_cast<unsigned char>(std::max(0.0, std::min(255.0, g * ratio)));
                dst.data[idx + 2] = static_cast<unsigned char>(std::max(0.0, std::min(255.0, b * ratio)));
            }
        }
    }
    return dst;
}

// ========== Verification ==========

// Create a synthetic low-contrast test image for reproducible testing
Image createTestImage() {
    // Create a 400x300 image with a sine-wave gradient that covers only a
    // narrow range (e.g., 40-100 out of 0-255)
    Image img;
    img.width = 400;
    img.height = 300;
    img.maxval = 255;
    img.data.resize(img.width * img.height * 3);

    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            int idx = (y * img.width + x) * 3;
            // Narrow range: 40 + sin pattern scaled to 0..60
            double v = 40.0 + 30.0 * (1.0 + std::sin(x * 0.05) * std::cos(y * 0.03)) / 2.0;
            unsigned char c = static_cast<unsigned char>(std::max(0.0, std::min(255.0, v)));
            img.data[idx] = c;
            img.data[idx + 1] = c;
            img.data[idx + 2] = c;
        }
    }
    return img;
}

struct VerificationResult {
    double entropyBefore, entropyAfter;
    double entropyMax;
    double ksBefore, ksAfter;
    double klBefore, klAfter;
    double stdBefore, stdAfter;
    int rangeBefore[2], rangeAfter[2];
    bool passed;
    std::string details;
};

VerificationResult verify(const Image& before, const Image& after) {
    Histogram hBefore, hAfter;
    hBefore.compute(before);
    hAfter.compute(after);

    VerificationResult r;
    r.entropyBefore = hBefore.entropy();
    r.entropyAfter  = hAfter.entropy();
    r.entropyMax    = std::log2(256.0); // 8.0
    r.ksBefore = hBefore.ksStatistic();
    r.ksAfter  = hAfter.ksStatistic();
    r.klBefore = hBefore.klDivergenceFromUniform();
    r.klAfter  = hAfter.klDivergenceFromUniform();

    // Compute std dev of luminance
    {
        double sum = 0, sumSq = 0;
        int N = before.width * before.height;
        for (int y = 0; y < before.height; y++) {
            for (int x = 0; x < before.width; x++) {
                double v = before.luminance(x, y);
                sum += v; sumSq += v * v;
            }
        }
        double mean = sum / N;
        r.stdBefore = std::sqrt(sumSq / N - mean * mean);
    }
    {
        double sum = 0, sumSq = 0;
        int N = after.width * after.height;
        for (int y = 0; y < after.height; y++) {
            for (int x = 0; x < after.width; x++) {
                double v = after.luminance(x, y);
                sum += v; sumSq += v * v;
            }
        }
        double mean = sum / N;
        r.stdAfter = std::sqrt(sumSq / N - mean * mean);
    }

    // Compute range
    {
        unsigned char minV = 255, maxV = 0;
        for (int y = 0; y < before.height; y++)
            for (int x = 0; x < before.width; x++) {
                unsigned char v = before.luminance(x, y);
                if (v < minV) minV = v;
                if (v > maxV) maxV = v;
            }
        r.rangeBefore[0] = minV; r.rangeBefore[1] = maxV;
    }
    {
        unsigned char minV = 255, maxV = 0;
        for (int y = 0; y < after.height; y++)
            for (int x = 0; x < after.width; x++) {
                unsigned char v = after.luminance(x, y);
                if (v < minV) minV = v;
                if (v > maxV) maxV = v;
            }
        r.rangeAfter[0] = minV; r.rangeAfter[1] = maxV;
    }

    // Pass criteria (quantifiable!):
    // 1. Entropy must increase (better information distribution)
    // 2. KS statistic must decrease (closer to uniform CDF)
    // 3. KL divergence from uniform must decrease
    // 4. Std dev must increase (more contrast)
    // 5. Range must expand to cover most of [0,255]
    std::ostringstream oss;
    bool allPass = true;

    oss << std::fixed << std::setprecision(4);

    auto check = [&](bool cond, const std::string& name, const std::string& detail) {
        oss << (cond ? "  ✅ " : "  ❌ ") << name << ": " << detail << "\n";
        if (!cond) allPass = false;
    };

    check(r.entropyAfter > r.entropyBefore, "Entropy increase",
          std::to_string(r.entropyBefore) + " → " + std::to_string(r.entropyAfter) +
          " (max=" + std::to_string(r.entropyMax) + ")");

    check(r.ksAfter < r.ksBefore * 0.5, "KS statistic improvement",
          std::to_string(r.ksBefore) + " → " + std::to_string(r.ksAfter) +
          " (lower = closer to uniform)");

    check(r.klAfter < r.klBefore, "KL divergence decrease",
          std::to_string(r.klBefore) + " → " + std::to_string(r.klAfter) +
          " (lower = more uniform)");

    check(r.stdAfter > r.stdBefore * 1.5, "Contrast improvement",
          std::to_string(r.stdBefore) + " → " + std::to_string(r.stdAfter) +
          " (std dev, higher = more contrast)");

    check(r.rangeAfter[0] <= 5 && r.rangeAfter[1] >= 250, "Range expansion",
          "[" + std::to_string(r.rangeBefore[0]) + "," + std::to_string(r.rangeBefore[1]) + "] → [" +
          std::to_string(r.rangeAfter[0]) + "," + std::to_string(r.rangeAfter[1]) + "]");

    // Extra: image files must exist and be non-trivial
    oss << "\n--- Summary ---\n";
    oss << "Entropy:  " << r.entropyBefore << " → " << r.entropyAfter << " (+" 
        << (r.entropyAfter - r.entropyBefore) << ")\n";
    oss << "KS stat:  " << r.ksBefore << " → " << r.ksAfter << " (-" 
        << (r.ksBefore - r.ksAfter) << ")\n";
    oss << "KL div:   " << r.klBefore << " → " << r.klAfter << " (-" 
        << (r.klBefore - r.klAfter) << ")\n";
    oss << "Std dev:  " << r.stdBefore << " → " << r.stdAfter << " (+"
        << (r.stdAfter - r.stdBefore) << ")\n";
    oss << "Range:    [" << r.rangeBefore[0] << "," << r.rangeBefore[1] << "] → ["
        << r.rangeAfter[0] << "," << r.rangeAfter[1] << "]\n";

    if (allPass) oss << "\n🎉 ALL CHECKS PASSED\n";
    else oss << "\n⚠️  Some checks failed\n";

    r.passed = allPass;
    r.details = oss.str();
    return r;
}

// Save histogram as PPM for visualization
void saveHistogramImage(const Histogram& h, const std::string& path) {
    int w = 256, barH = 200, margin = 40;
    Image img;
    img.width = w + 2 * margin;
    img.height = barH + 2 * margin;
    img.maxval = 255;
    img.data.assign((img.width * img.height * 3), 255); // white bg

    int maxCount = *std::max_element(h.bins.begin(), h.bins.end());
    if (maxCount == 0) maxCount = 1;

    for (int i = 0; i < 256; i++) {
        int barHeight = static_cast<int>(static_cast<double>(h.bins[i]) / maxCount * barH);
        for (int row = 0; row < barH; row++) {
            int y = margin + barH - 1 - row;
            int idx = (y * img.width + margin + i) * 3;
            if (row < barHeight) {
                img.data[idx] = img.data[idx + 1] = img.data[idx + 2] = 0;
            }
        }
    }
    img.savePPM(path);
}

int main() {
    std::cout << "=== Histogram Equalization with Quantifiable Verification ===\n\n";

    // Step 1: Create synthetic low-contrast test image
    std::cout << "[1/5] Generating low-contrast test image...\n";
    Image src = createTestImage();
    src.savePPM("before.ppm");
    std::cout << "  Saved before.ppm (" << src.width << "x" << src.height << ")\n";

    // Step 2: Compute before-histogram
    std::cout << "[2/5] Computing original histogram...\n";
    Histogram hBefore;
    hBefore.compute(src);
    std::cout << "  Entropy: " << hBefore.entropy() << " bits (max 8.0)\n";
    std::cout << "  KS stat: " << hBefore.ksStatistic() << "\n";
    saveHistogramImage(hBefore, "hist_before.ppm");

    // Step 3: Apply histogram equalization
    std::cout << "[3/5] Applying histogram equalization...\n";
    Image dst = equalize(src);
    dst.savePPM("after.ppm");
    std::cout << "  Saved after.ppm\n";

    // Step 4: Compute after-histogram
    std::cout << "[4/5] Computing equalized histogram...\n";
    Histogram hAfter;
    hAfter.compute(dst);
    std::cout << "  Entropy: " << hAfter.entropy() << " bits\n";
    std::cout << "  KS stat: " << hAfter.ksStatistic() << "\n";
    saveHistogramImage(hAfter, "hist_after.ppm");

    // Step 5: Quantifiable verification
    std::cout << "\n[5/5] QUANTIFIABLE VERIFICATION\n";
    std::cout << "=====================================\n";
    auto result = verify(src, dst);
    std::cout << result.details;

    // Save verification report
    std::ofstream report("verification_report.txt");
    report << result.details;
    report.close();

    // Also check file sizes
    std::cout << "\n--- File Size Check ---\n";
    for (const auto& f : {"before.ppm", "after.ppm", "hist_before.ppm", "hist_after.ppm"}) {
        std::ifstream in(f, std::ios::binary | std::ios::ate);
        auto size = in.tellg();
        std::cout << "  " << f << ": " << size << " bytes "
                  << (size > 10240 ? "✅" : "❌ too small") << "\n";
    }

    return result.passed ? 0 : 1;
}
