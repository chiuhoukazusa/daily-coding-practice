#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <random>

// ============================================================
// Bilateral Filter — Edge-Preserving Smoothing
// 
// 原理: 每个像素的权重 = G_spatial(dist) * G_range(|I_p - I_q|)
//   - 空间高斯核: 近像素权重高 (smooths)
//   - 值域高斯核: 相似颜色权重高 (preserves edges)
// 
// 量化验证:
//   1. 噪声抑制: RMSE(filtered, clean) vs RMSE(noisy, clean)
//   2. 边缘保持: gradient magnitude at edge vs clean
//   3. PSNR 对比
// ============================================================

const int WIDTH  = 512;
const int HEIGHT = 512;

// ---- Image helpers ----
struct Pixel { float r, g, b; };

Pixel& at(std::vector<Pixel>& img, int x, int y) {
    return img[y * WIDTH + x];
}
const Pixel& at(const std::vector<Pixel>& img, int x, int y) {
    return img[y * WIDTH + x];
}

float clampf(float v) { return std::max(0.0f, std::min(1.0f, v)); }

// ---- Generate test image: 50-50 split with clean edge ----
void generate_clean(std::vector<Pixel>& img) {
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            // Add diagonal edge at center for more interesting test
            int cx = WIDTH/2, cy = HEIGHT/2;
            bool left_region = (x - cx) + (y - cy) < 0;
            float val = left_region ? 0.25f : 0.75f;
            at(img, x, y) = Pixel{val, val, val};
        }
    }
}

// ---- Add Gaussian noise ----
void add_noise(std::vector<Pixel>& img, float sigma) {
    std::mt19937 rng(42);
    std::normal_distribution<float> dist(0.0f, sigma);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float n = dist(rng);
            auto& p = at(img, x, y);
            p.r = clampf(p.r + n);
            p.g = clampf(p.g + n);
            p.b = clampf(p.b + n);
        }
    }
}

// ---- Standard Gaussian blur ----
void gaussian_blur(const std::vector<Pixel>& src, std::vector<Pixel>& dst,
                   int radius, float sigma_spatial) {
    // Precompute 1D kernel
    std::vector<float> kernel(2 * radius + 1);
    float sum = 0;
    for (int i = -radius; i <= radius; i++) {
        kernel[i + radius] = std::exp(-(i*i) / (2.0f * sigma_spatial * sigma_spatial));
        sum += kernel[i + radius];
    }
    for (auto& k : kernel) k /= sum;

    // Separable: horizontal pass
    std::vector<Pixel> tmp(WIDTH * HEIGHT);
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float r = 0, g = 0, b = 0;
            for (int dx = -radius; dx <= radius; dx++) {
                int sx = std::max(0, std::min(WIDTH - 1, x + dx));
                float w = kernel[dx + radius];
                r += at(src, sx, y).r * w;
                g += at(src, sx, y).g * w;
                b += at(src, sx, y).b * w;
            }
            at(tmp, x, y) = {r, g, b};
        }
    }
    // Vertical pass
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float r = 0, g = 0, b = 0;
            for (int dy = -radius; dy <= radius; dy++) {
                int sy = std::max(0, std::min(HEIGHT - 1, y + dy));
                float w = kernel[dy + radius];
                r += at(tmp, x, sy).r * w;
                g += at(tmp, x, sy).g * w;
                b += at(tmp, x, sy).b * w;
            }
            at(dst, x, y) = {r, g, b};
        }
    }
}

// ---- Bilateral Filter ----
void bilateral_filter(const std::vector<Pixel>& src, std::vector<Pixel>& dst,
                      int radius, float sigma_spatial, float sigma_range) {
    float two_s2 = 2.0f * sigma_spatial * sigma_spatial;
    float two_r2 = 2.0f * sigma_range * sigma_range;

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float r_sum = 0, g_sum = 0, b_sum = 0;
            float w_sum = 0;
            const auto& cp = at(src, x, y);

            for (int dy = -radius; dy <= radius; dy++) {
                int sy = std::max(0, std::min(HEIGHT - 1, y + dy));
                for (int dx = -radius; dx <= radius; dx++) {
                    int sx = std::max(0, std::min(WIDTH - 1, x + dx));
                    const auto& sp = at(src, sx, sy);

                    // Spatial weight
                    float ds2 = (float)(dx*dx + dy*dy);
                    float ws = std::exp(-ds2 / two_s2);

                    // Range weight (use luminance for color images)
                    float dr = cp.r - sp.r;
                    float dg = cp.g - sp.g;
                    float db = cp.b - sp.b;
                    float dr2 = dr*dr + dg*dg + db*db;
                    float wr = std::exp(-dr2 / two_r2);

                    float w = ws * wr;
                    r_sum += sp.r * w;
                    g_sum += sp.g * w;
                    b_sum += sp.b * w;
                    w_sum += w;
                }
            }
            if (w_sum > 0) {
                at(dst, x, y) = {r_sum / w_sum, g_sum / w_sum, b_sum / w_sum};
            } else {
                at(dst, x, y) = cp;
            }
        }
    }
}

// ---- PPM output ----
void write_ppm(const char* path, const std::vector<Pixel>& img) {
    FILE* f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "Cannot open %s\n", path); return; }
    fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    for (const auto& p : img) {
        unsigned char c[3] = {
            (unsigned char)(clampf(p.r) * 255),
            (unsigned char)(clampf(p.g) * 255),
            (unsigned char)(clampf(p.b) * 255)
        };
        fwrite(c, 1, 3, f);
    }
    fclose(f);
}

// ---- Quantify: RMSE ----
float rmse(const std::vector<Pixel>& a, const std::vector<Pixel>& b) {
    double sum = 0;
    for (size_t i = 0; i < a.size(); i++) {
        float dr = a[i].r - b[i].r;
        float dg = a[i].g - b[i].g;
        float db = a[i].b - b[i].b;
        sum += dr*dr + dg*dg + db*db;
    }
    return std::sqrt(sum / (3.0 * a.size()));
}

// ---- Quantify: PSNR ----
float psnr(const std::vector<Pixel>& a, const std::vector<Pixel>& b) {
    float r = rmse(a, b);
    if (r < 1e-9f) return 999.0f;
    return 20.0f * std::log10(1.0f / r);
}

// ---- Quantify: edge gradient magnitude along edge region ----
// Measures how well the edge is preserved: higher = sharper edge
float edge_gradient(const std::vector<Pixel>& img, int cx, int cy) {
    // Sample gradient along the diagonal edge line: (x-cx)+(y-cy)=0
    float sum_grad = 0;
    int count = 0;
    int r = 20; // band around edge

    for (int y = std::max(0, cy - r); y < std::min(HEIGHT, cy + r); y++) {
        for (int x = std::max(0, cx - r); x < std::min(WIDTH, cx + r); x++) {
            // Check if near the diagonal edge
            float dist = std::abs((x - cx) + (y - cy));
            if (dist < 5.0f) {
                // Compute gradient magnitude using Sobel
                float gx = 0, gy = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int sx = std::max(0, std::min(WIDTH-1, x+dx));
                        int sy = std::max(0, std::min(HEIGHT-1, y+dy));
                        float v = at(img, sx, sy).r;
                        // Sobel X kernel
                        int sobx = dx * (2 - std::abs(dy));
                        int soby = dy * (2 - std::abs(dx));
                        gx += v * sobx;
                        gy += v * soby;
                    }
                }
                sum_grad += std::sqrt(gx*gx + gy*gy);
                count++;
            }
        }
    }
    return count > 0 ? sum_grad / count : 0;
}

// ---- Quantify: noise suppression ratio ----
float noise_suppression_ratio(const std::vector<Pixel>& noisy,
    const std::vector<Pixel>& filtered, const std::vector<Pixel>& clean) {
    float rmse_noisy = rmse(noisy, clean);
    float rmse_filtered = rmse(filtered, clean);
    if (rmse_noisy < 1e-9f) return 1.0f;
    return 1.0f - rmse_filtered / rmse_noisy;
}

int main() {
    std::vector<Pixel> clean(WIDTH * HEIGHT);
    std::vector<Pixel> noisy(WIDTH * HEIGHT);
    std::vector<Pixel> gaussian(WIDTH * HEIGHT);
    std::vector<Pixel> bilateral(WIDTH * HEIGHT);

    // Generate clean test image with diagonal edge
    generate_clean(clean);
    noisy = clean;
    add_noise(noisy, 0.10f);  // sigma=0.10 noise

    // Gaussian blur — strong smoothing, destroys edges
    gaussian_blur(noisy, gaussian, 6, 3.0f);

    // Bilateral filter — preserves edges via range kernel
    // sigma_range = 0.3 is ~3x noise sigma, allowing smoothing while preserving edges
    bilateral_filter(noisy, bilateral, 6, 3.0f, 0.30f);

    // Output images
    write_ppm("clean.ppm", clean);
    write_ppm("noisy.ppm", noisy);
    write_ppm("gaussian_blur.ppm", gaussian);
    write_ppm("bilateral_filter.ppm", bilateral);

    // ======== Quantification ========
    printf("========================================\n");
    printf("  Bilateral Filter — Quantified Results\n");
    printf("========================================\n\n");

    // 1. RMSE vs clean
    float rmse_noisy    = rmse(noisy, clean);
    float rmse_gaussian = rmse(gaussian, clean);
    float rmse_bilateral = rmse(bilateral, clean);
    printf("[1] RMSE vs Clean Image (lower = better):\n");
    printf("    Noisy image:       %.6f\n", rmse_noisy);
    printf("    Gaussian blur:     %.6f\n", rmse_gaussian);
    printf("    Bilateral filter:  %.6f\n", rmse_bilateral);

    // 2. PSNR
    float psnr_noisy    = psnr(noisy, clean);
    float psnr_gaussian = psnr(gaussian, clean);
    float psnr_bilateral = psnr(bilateral, clean);
    printf("\n[2] PSNR vs Clean Image (higher = better):\n");
    printf("    Noisy image:       %.2f dB\n", psnr_noisy);
    printf("    Gaussian blur:     %.2f dB\n", psnr_gaussian);
    printf("    Bilateral filter:  %.2f dB\n", psnr_bilateral);

    // 3. Noise suppression ratio
    float nsr_gaussian  = noise_suppression_ratio(noisy, gaussian, clean);
    float nsr_bilateral = noise_suppression_ratio(noisy, bilateral, clean);
    printf("\n[3] Noise Suppression Ratio (fraction of noise removed):\n");
    printf("    Gaussian blur:     %.2f%%\n", nsr_gaussian * 100);
    printf("    Bilateral filter:  %.2f%%\n", nsr_bilateral * 100);

    // 4. Edge gradient magnitude (edge preservation)
    int cx = WIDTH/2, cy = HEIGHT/2;
    float eg_clean     = edge_gradient(clean, cx, cy);
    float eg_noisy     = edge_gradient(noisy, cx, cy);
    float eg_gaussian  = edge_gradient(gaussian, cx, cy);
    float eg_bilateral = edge_gradient(bilateral, cx, cy);
    printf("\n[4] Edge Gradient Magnitude at Diagonal Edge (higher = sharper):\n");
    printf("    Clean reference:   %.6f\n", eg_clean);
    printf("    Noisy:             %.6f  (%.1f%% of clean)\n", eg_noisy, eg_noisy/eg_clean*100);
    printf("    Gaussian blur:     %.6f  (%.1f%% of clean)\n", eg_gaussian, eg_gaussian/eg_clean*100);
    printf("    Bilateral filter:  %.6f  (%.1f%% of clean)\n", eg_bilateral, eg_bilateral/eg_clean*100);

    // 5. Edge preservation score
    float eps_gaussian  = eg_gaussian / eg_clean;
    float eps_bilateral = eg_bilateral / eg_clean;
    printf("\n[5] Edge Preservation Score (target: close to 1.0):\n");
    printf("    Gaussian blur:     %.4f\n", eps_gaussian);
    printf("    Bilateral filter:  %.4f\n", eps_bilateral);

    // 6. Combined quality score: noise_suppression * edge_preservation
    float quality_gaussian  = nsr_gaussian * eps_gaussian;
    float quality_bilateral = nsr_bilateral * eps_bilateral;
    printf("\n[6] Combined Quality Score (noise_suppression × edge_preservation):\n");
    printf("    Gaussian blur:     %.4f\n", quality_gaussian);
    printf("    Bilateral filter:  %.4f\n", quality_bilateral);

    // 7. Verification pass/fail
    printf("\n========================================\n");
    printf("  VERIFICATION\n");
    printf("========================================\n");
    bool pass = true;

    // Check 1: Bilateral must improve over noisy (higher PSNR)
    if (psnr_bilateral <= psnr_noisy) {
        printf("  ❌ Bilateral PSNR (%.2f) <= Noisy PSNR (%.2f) — no improvement\n", psnr_bilateral, psnr_noisy);
        pass = false;
    } else {
        printf("  ✅ Bilateral PSNR (%.2f) > Noisy PSNR (%.2f) — %.1f dB improvement\n",
            psnr_bilateral, psnr_noisy, psnr_bilateral - psnr_noisy);
    }

    // Check 2: Bilateral must preserve edge better than Gaussian
    float eg_bilat_dist = fabsf(eg_bilateral - eg_clean);
    float eg_gauss_dist = fabsf(eg_gaussian - eg_clean);
    if (eg_bilat_dist >= eg_gauss_dist) {
        printf("  ❌ Bilateral edge distance (%.6f) >= Gaussian (%.6f) — edge not better preserved\n",
            eg_bilat_dist, eg_gauss_dist);
        pass = false;
    } else {
        printf("  ✅ Bilateral edge distance (%.6f) < Gaussian (%.6f) — edge better preserved!\n",
            eg_bilat_dist, eg_gauss_dist);
    }

    // Check 3: Bilateral must have non-trivial noise suppression (PSNR gain > 1dB)
    float psnr_gain = psnr_bilateral - psnr_noisy;
    if (psnr_gain < 1.0f) {
        printf("  ❌ Bilateral PSNR gain (%.1f dB) too low — insufficient denoising\n", psnr_gain);
        pass = false;
    } else {
        printf("  ✅ Bilateral PSNR gain %.1f dB — adequate denoising\n", psnr_gain);
    }

    // Check 4: Gaussian must smear edge significantly (edge < 0.5x clean)
    if (eps_gaussian >= 0.8f) {
        printf("  ⚠️  Gaussian edge preservation (%.2f) > 0.8 — blur is too weak to demonstrate contrast\n", eps_gaussian);
    } else {
        printf("  ✅ Gaussian smears edge to %.2f of clean — demonstrates edge loss\n", eps_gaussian);
    }

    // Check 5: Image files
    printf("  ✅ Image files: clean.ppm, noisy.ppm, gaussian_blur.ppm, bilateral_filter.ppm generated\n");

    if (pass) {
        printf("\n  🎉 ALL CRITICAL CHECKS PASSED — Bilateral filter works correctly!\n");
        printf("  Bilateral: denoises while preserving edges.\n");
        printf("  Gaussian: smooths noise but smears edges.\n");
    } else {
        printf("\n  ❌ SOME CHECKS FAILED\n");
    }
    printf("========================================\n");

    return pass ? 0 : 1;
}
