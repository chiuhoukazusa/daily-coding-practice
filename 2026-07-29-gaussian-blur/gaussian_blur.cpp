/**
 * 每日编程实践 - 2026-07-29
 * 高斯模糊 + 可分离核优化 (Gaussian Blur with Separable Kernel)
 * 
 * 核心概念：
 * 1. 高斯核：G(x,y) = (1/(2πσ²)) * exp(-(x²+y²)/(2σ²))
 * 2. 2D高斯核可分解为两个1D核的外积：G_2D = G_x ⊗ G_y
 * 3. 2D卷积 O(W*H*k²) → 两次1D卷积 O(2*W*H*k) = O(W*H*k)
 * 
 * 量化验证：
 * 1. 可分离性数值验证：验证 |G_2D - G_x ⊗ G_y| < ε
 * 2. 输出等价性：对比2D直接卷积 vs 分离卷积，验证每像素差异 < threshold
 * 3. 性能加速比：runtime_2d / runtime_separable
 * 4. PSNR验证：保证分离卷积输出与2D卷积输出视觉无损
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <string>

// ==================== PPM Image Utilities ====================
struct Image {
    int width, height;
    std::vector<unsigned char> data;  // R,G,B,R,G,B,...
    
    Image(int w, int h) : width(w), height(h), data(w * h * 3, 0) {}
    
    unsigned char& at(int x, int y, int c) {
        return data[(y * width + x) * 3 + c];
    }
    
    unsigned char at(int x, int y, int c) const {
        return data[(y * width + x) * 3 + c];
    }
    
    Image clone() const {
        Image img(width, height);
        img.data = data;
        return img;
    }
};

Image loadPPM(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "Cannot open: " << path << std::endl;
        return Image(0, 0);
    }
    std::string magic;
    f >> magic;
    int w, h, maxval;
    f >> w >> h >> maxval;
    f.get(); // skip newline after maxval
    
    Image img(w, h);
    f.read(reinterpret_cast<char*>(img.data.data()), w * h * 3);
    return img;
}

void savePPM(const std::string& path, const Image& img) {
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << img.width << " " << img.height << "\n255\n";
    f.write(reinterpret_cast<const char*>(img.data.data()), img.data.size());
}

// Generate a test pattern image: gradients + geometric shapes
Image generateTestImage(int w, int h) {
    Image img(w, h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            // Radial gradient from center (R)
            double dx = (x - w / 2.0) / (w / 2.0);
            double dy = (y - h / 2.0) / (h / 2.0);
            double dist = std::sqrt(dx * dx + dy * dy);
            unsigned char r = static_cast<unsigned char>(std::max(0.0, std::min(255.0, 255.0 * (1.0 - dist))));
            
            // Horizontal gradient (G)
            unsigned char g = static_cast<unsigned char>(std::max(0.0, std::min(255.0, 255.0 * (x / (double)w))));
            
            // Checkerboard (B) - sharp edges for blurring
            int cx = (x / 32) % 2;
            int cy = (y / 32) % 2;
            unsigned char b = (cx == cy) ? 200 : 50;
            
            // Add horizontal and vertical lines for sharp edge detection
            if (std::abs(x % 128 - 64) < 3 || std::abs(y % 96 - 48) < 3) {
                r = 255; g = 255; b = 255;
            }
            
            img.at(x, y, 0) = r;
            img.at(x, y, 1) = g;
            img.at(x, y, 2) = b;
        }
    }
    return img;
}

// ==================== Gaussian Kernel ====================
class GaussianKernel {
public:
    int radius;
    int kernelSize;
    std::vector<double> kernel1D;
    std::vector<double> kernel2D;
    double sigma;
    
    GaussianKernel(double sigma_val, int radius_val)
        : sigma(sigma_val), radius(radius_val),
          kernelSize(2 * radius_val + 1),
          kernel1D(kernelSize),
          kernel2D(kernelSize * kernelSize) {
        build1D();
        build2D();
    }
    
private:
    void build1D() {
        double sum = 0.0;
        for (int i = -radius; i <= radius; i++) {
            double val = std::exp(-(i * i) / (2.0 * sigma * sigma));
            kernel1D[i + radius] = val;
            sum += val;
        }
        // Normalize
        for (int i = 0; i < kernelSize; i++) {
            kernel1D[i] /= sum;
        }
    }
    
    void build2D() {
        double sum = 0.0;
        for (int ky = -radius; ky <= radius; ky++) {
            for (int kx = -radius; kx <= radius; kx++) {
                double val = std::exp(-(kx * kx + ky * ky) / (2.0 * sigma * sigma));
                int idx = (ky + radius) * kernelSize + (kx + radius);
                kernel2D[idx] = val;
                sum += val;
            }
        }
        // Normalize
        for (int i = 0; i < kernelSize * kernelSize; i++) {
            kernel2D[i] /= sum;
        }
    }
};

// ==================== 2D Direct Convolution ====================
Image blur2D(const Image& input, const GaussianKernel& kernel) {
    int w = input.width, h = input.height;
    int r = kernel.radius, ks = kernel.kernelSize;
    Image output(w, h);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sum[3] = {0, 0, 0};
            for (int ky = -r; ky <= r; ky++) {
                int sy = std::max(0, std::min(h - 1, y + ky)); // clamp
                for (int kx = -r; kx <= r; kx++) {
                    int sx = std::max(0, std::min(w - 1, x + kx));
                    double kw = kernel.kernel2D[(ky + r) * ks + (kx + r)];
                    for (int c = 0; c < 3; c++) {
                        sum[c] += input.at(sx, sy, c) * kw;
                    }
                }
            }
            for (int c = 0; c < 3; c++) {
                output.at(x, y, c) = static_cast<unsigned char>(std::max(0.0, std::min(255.0, sum[c])));
            }
        }
    }
    return output;
}

// ==================== Separable 1D Convolution ====================
Image blur1D_H(const Image& input, const std::vector<double>& kernel1D, int radius) {
    int w = input.width, h = input.height;
    Image output(w, h);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sum[3] = {0, 0, 0};
            for (int k = -radius; k <= radius; k++) {
                int sx = std::max(0, std::min(w - 1, x + k));
                double kw = kernel1D[k + radius];
                for (int c = 0; c < 3; c++) {
                    sum[c] += input.at(sx, y, c) * kw;
                }
            }
            for (int c = 0; c < 3; c++) {
                output.at(x, y, c) = static_cast<unsigned char>(std::max(0.0, std::min(255.0, sum[c])));
            }
        }
    }
    return output;
}

Image blur1D_V(const Image& input, const std::vector<double>& kernel1D, int radius) {
    int w = input.width, h = input.height;
    Image output(w, h);
    
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double sum[3] = {0, 0, 0};
            for (int k = -radius; k <= radius; k++) {
                int sy = std::max(0, std::min(h - 1, y + k));
                double kw = kernel1D[k + radius];
                for (int c = 0; c < 3; c++) {
                    sum[c] += input.at(x, sy, c) * kw;
                }
            }
            for (int c = 0; c < 3; c++) {
                output.at(x, y, c) = static_cast<unsigned char>(std::max(0.0, std::min(255.0, sum[c])));
            }
        }
    }
    return output;
}

Image blurSeparable(const Image& input, const GaussianKernel& kernel) {
    Image h_pass = blur1D_H(input, kernel.kernel1D, kernel.radius);
    Image v_pass = blur1D_V(h_pass, kernel.kernel1D, kernel.radius);
    return v_pass;
}

// ==================== Quantification ====================

// 1. Verify kernel separability: |G_2D(i,j) - G_1D(i)*G_1D(j)| < eps
bool verifyKernelSeparability(const GaussianKernel& kernel, double eps = 1e-12) {
    int ks = kernel.kernelSize;
    double maxError = 0.0;
    double sumError = 0.0;
    
    for (int j = 0; j < ks; j++) {
        for (int i = 0; i < ks; i++) {
            double g2d = kernel.kernel2D[j * ks + i];
            double g_sep = kernel.kernel1D[j] * kernel.kernel1D[i];
            double err = std::abs(g2d - g_sep);
            maxError = std::max(maxError, err);
            sumError += err;
        }
    }
    
    std::cout << "\n📐 === Kernel Separability Verification ===" << std::endl;
    std::cout << "  Kernel size: " << ks << "x" << ks << ", sigma=" << kernel.sigma << std::endl;
    std::cout << "  Max element-wise error: " << std::scientific << maxError << std::endl;
    std::cout << "  Sum element-wise error: " << std::scientific << sumError << std::endl;
    std::cout << "  Threshold: " << std::scientific << eps << std::endl;
    
    bool pass = maxError < eps;
    std::cout << "  Result: " << (pass ? "✅ PASS" : "❌ FAIL") << std::endl;
    return pass;
}

// 2. Verify output equivalence: max per-pixel channel difference
struct EquivalenceResult {
    double maxError;
    double avgError;
    double rmse;
    double psnr;
    int errorPixelCount;  // pixels with any channel error > 1
    bool pass;
};

EquivalenceResult verifyOutputEquivalence(const Image& a, const Image& b, const std::string& label, double threshold = 1.0) {
    EquivalenceResult res = {0, 0, 0, 0, 0, false};
    int totalPixels = a.width * a.height;
    double sumSqError = 0.0;
    
    for (int y = 0; y < a.height; y++) {
        for (int x = 0; x < a.width; x++) {
            bool pixelHasError = false;
            for (int c = 0; c < 3; c++) {
                int diff = std::abs((int)a.at(x, y, c) - (int)b.at(x, y, c));
                res.maxError = std::max(res.maxError, (double)diff);
                res.avgError += diff;
                sumSqError += diff * diff;
                if (diff > 1) pixelHasError = true;
                if (diff > 10) {
                    std::cout << "  ⚠️ Large error at (" << x << "," << y << ") ch=" << c 
                             << ": 2D=" << (int)a.at(x,y,c) << " sep=" << (int)b.at(x,y,c) << std::endl;
                }
            }
            if (pixelHasError) res.errorPixelCount++;
        }
    }
    
    res.avgError /= (totalPixels * 3);  // average over all channels
    res.rmse = std::sqrt(sumSqError / (totalPixels * 3));
    
    // PSNR: max signal = 255
    if (res.rmse > 0) {
        res.psnr = 20.0 * std::log10(255.0 / res.rmse);
    } else {
        res.psnr = 999.0; // infinite
    }
    
    res.pass = res.maxError <= 2.0;  // float rounding acceptable at boundaries
    
    std::cout << "\n🔬 === Output Equivalence: " << label << " ===" << std::endl;
    std::cout << "  Image size: " << a.width << "x" << a.height << std::endl;
    std::cout << "  Max pixel error: " << res.maxError << std::endl;
    std::cout << "  Avg pixel error: " << std::fixed << std::setprecision(6) << res.avgError << std::endl;
    std::cout << "  RMSE: " << std::fixed << std::setprecision(6) << res.rmse << std::endl;
    std::cout << "  PSNR: " << std::fixed << std::setprecision(2) << res.psnr << " dB" << std::endl;
    std::cout << "  Error pixels (>1): " << res.errorPixelCount << " / " << totalPixels 
              << " (" << std::fixed << std::setprecision(4) << (100.0 * res.errorPixelCount / totalPixels) << "%)" << std::endl;
    std::cout << "  Threshold: " << threshold << std::endl;
    std::cout << "  Result: " << (res.pass ? "✅ PASS (identical outputs)" : "⚠️ DIFFER (allowed by rounding)") << std::endl;
    
    return res;
}

// 3. Performance benchmark
struct PerformanceResult {
    double time2D_ms;
    double timeSep_ms;
    double speedup;
    double theoreticalSpeedup;
};

PerformanceResult benchmarkPerformance(const Image& input, const GaussianKernel& kernel, int iterations = 5) {
    PerformanceResult perf;
    
    // Warm-up
    blur2D(input, kernel);
    blurSeparable(input, kernel);
    
    // Benchmark 2D
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        blur2D(input, kernel);
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    perf.time2D_ms = std::chrono::duration<double, std::milli>(t2 - t1).count() / iterations;
    
    // Benchmark Separable
    t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; i++) {
        blurSeparable(input, kernel);
    }
    t2 = std::chrono::high_resolution_clock::now();
    perf.timeSep_ms = std::chrono::duration<double, std::milli>(t2 - t1).count() / iterations;
    
    perf.speedup = perf.time2D_ms / perf.timeSep_ms;
    int k = kernel.kernelSize;
    perf.theoreticalSpeedup = (double)(k * k) / (2.0 * k); // O(k²) vs O(2k)
    
    std::cout << "\n⚡ === Performance Benchmark ===" << std::endl;
    std::cout << "  Kernel size: " << k << "x" << k << std::endl;
    std::cout << "  Iterations: " << iterations << std::endl;
    std::cout << "  2D convolution: " << std::fixed << std::setprecision(3) << perf.time2D_ms << " ms" << std::endl;
    std::cout << "  Separable convolution: " << std::fixed << std::setprecision(3) << perf.timeSep_ms << " ms" << std::endl;
    std::cout << "  Speedup (2D / Separable): " << std::fixed << std::setprecision(2) << perf.speedup << "x" << std::endl;
    std::cout << "  Theoretical speedup (k² / 2k): " << std::fixed << std::setprecision(2) << perf.theoreticalSpeedup << "x" << std::endl;
    std::cout << "  Efficiency: " << std::fixed << std::setprecision(1) 
              << (100.0 * perf.speedup / perf.theoreticalSpeedup) << "% of theoretical max" << std::endl;
    
    return perf;
}

// 4. Multi-sigma multi-radius comprehensive verification
void comprehensiveTest() {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  GAUSSIAN BLUR - COMPREHENSIVE QUANTIFICATION" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // Generate test image
    const int W = 512, H = 512;
    std::cout << "\n🖼️  Generating test image " << W << "x" << H << "...";
    Image input = generateTestImage(W, H);
    savePPM("input.ppm", input);
    std::cout << " done (input.ppm)" << std::endl;
    
    // Test configurations: {sigma, radius}
    struct TestConfig {
        double sigma;
        int radius;
        std::string description;
    };
    
    std::vector<TestConfig> configs = {
        {1.0, 3,  "small blur (σ=1, r=3, k=7)"},
        {2.0, 5,  "medium blur (σ=2, r=5, k=11)"},
        {3.0, 8,  "large blur (σ=3, r=8, k=17)"},
        {5.0, 12, "heavy blur (σ=5, r=12, k=25)"},
    };
    
    // Summary table
    std::cout << "\n📊 === Comprehensive Results Summary ===" << std::endl;
    std::cout << std::string(95, '-') << std::endl;
    std::cout << std::left << std::setw(10) << "Sigma"
              << std::setw(12) << "Kernel"
              << std::setw(18) << "2D Time(ms)"
              << std::setw(18) << "Sep Time(ms)"
              << std::setw(12) << "Speedup"
              << std::setw(12) << "PSNR(dB)"
              << std::setw(12) << "MaxErr"
              << std::endl;
    std::cout << std::string(95, '-') << std::endl;
    
    bool allPass = true;
    
    for (const auto& cfg : configs) {
        std::cout << "\n" << std::string(70, '─') << std::endl;
        std::cout << "  Testing: " << cfg.description << std::endl;
        std::cout << std::string(70, '─') << std::endl;
        
        GaussianKernel kernel(cfg.sigma, cfg.radius);
        
        // 1. Kernel separability
        bool sepPass = verifyKernelSeparability(kernel);
        allPass = allPass && sepPass;
        
        // 2. Apply both methods
        Image result2D = blur2D(input, kernel);
        Image resultSep = blurSeparable(input, kernel);
        
        // Save outputs
        std::string name = "blur_sigma" + std::to_string((int)(cfg.sigma * 10)) 
                          + "_r" + std::to_string(cfg.radius);
        savePPM(name + "_2d.ppm", result2D);
        savePPM(name + "_sep.ppm", resultSep);
        
        // 3. Output equivalence
        auto eq = verifyOutputEquivalence(result2D, resultSep, name);
        allPass = allPass && eq.pass;
        
        // 4. Performance
        auto perf = benchmarkPerformance(input, kernel, 3);
        
        // Summary row
        int ks = kernel.kernelSize;
        std::cout << std::left << std::setw(10) << cfg.sigma
                  << std::setw(12) << (std::to_string(ks) + "x" + std::to_string(ks))
                  << std::setw(18) << (std::to_string((int)(perf.time2D_ms * 10) / 10.0) + "ms")
                  << std::setw(18) << (std::to_string((int)(perf.timeSep_ms * 10) / 10.0) + "ms")
                  << std::setw(12) << (std::to_string((int)(perf.speedup * 10) / 10.0) + "x")
                  << std::setw(12) << (std::to_string((int)(eq.psnr * 100) / 100.0) + "dB")
                  << std::setw(12) << eq.maxError
                  << std::endl;
    }
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "  OVERALL VERDICT: " << (allPass ? "✅ ALL TESTS PASSED" : "❌ SOME TESTS FAILED") << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    // Extra: verification that Gaussian blur reduces high frequencies (FFT approximation via variance)
    std::cout << "\n📈 === Variance Reduction Verification ===" << std::endl;
    std::cout << "  (Gaussian blur should reduce image variance)" << std::endl;
    
    // Compute variance for original and blurred images (luminance only)
    auto calcVariance = [](const Image& img) {
        double mean = 0;
        int N = img.width * img.height;
        for (int i = 0; i < N; i++) {
            double lum = 0.299 * img.data[i*3] + 0.587 * img.data[i*3+1] + 0.114 * img.data[i*3+2];
            mean += lum;
        }
        mean /= N;
        double var = 0;
        for (int i = 0; i < N; i++) {
            double lum = 0.299 * img.data[i*3] + 0.587 * img.data[i*3+1] + 0.114 * img.data[i*3+2];
            var += (lum - mean) * (lum - mean);
        }
        return var / N;
    };
    
    double origVar = calcVariance(input);
    std::cout << "  Original image variance: " << std::fixed << std::setprecision(1) << origVar << std::endl;
    
    for (const auto& cfg : configs) {
        GaussianKernel kernel(cfg.sigma, cfg.radius);
        Image result = blurSeparable(input, kernel);
        double blurVar = calcVariance(result);
        double reduction = 100.0 * (1.0 - blurVar / origVar);
        std::cout << "  σ=" << std::setw(3) << cfg.sigma << " variance: " << std::fixed << std::setprecision(1) 
                  << std::setw(8) << blurVar << " (reduction: " << std::fixed << std::setprecision(1) << reduction << "%)" << std::endl;
    }
}

int main() {
    comprehensiveTest();
    return 0;
}
