/**
 * HDR Tone Mapping & Color Grading
 * 
 * 实现多种经典色调映射算法：
 * 1. Reinhard (全局/局部)
 * 2. ACES Filmic (Academy Color Encoding System)
 * 3. Uncharted 2 (Hable Filmic)
 * 4. Exposure + Gamma
 * 5. Lottes Filmic
 * 
 * 同时演示色彩分级（Color Grading）：
 * - 对比度调整
 * - 饱和度调整
 * - 色温调整
 * - 分色(Shadows/Midtones/Highlights)
 * 
 * 输出：对比图（横向拼接所有算法）
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <functional>
#include <cassert>
#include <random>

// ============================================================
// Vector Math
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3(float v = 0.f) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float t) const { return {x * t, y * t, z * t}; }
    Vec3 operator*(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
    Vec3 operator/(float t) const { return {x / t, y / t, z / t}; }
    Vec3 operator/(const Vec3& o) const { return {x / o.x, y / o.y, z / o.z}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator*=(float t) { x *= t; y *= t; z *= t; return *this; }
    
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const { float l = length(); return l > 0 ? *this / l : Vec3(0); }
    
    Vec3 clamp(float lo, float hi) const {
        return {std::max(lo, std::min(hi, x)),
                std::max(lo, std::min(hi, y)),
                std::max(lo, std::min(hi, z))};
    }
    float luminance() const { return 0.2126f * x + 0.7152f * y + 0.0722f * z; }
};

Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
    return a + (b - a) * t;
}

// ============================================================
// HDR Image (float buffer)
// ============================================================

struct Image {
    int width, height;
    std::vector<Vec3> pixels;

    Image(int w, int h) : width(w), height(h), pixels(w * h, Vec3(0.f)) {}

    Vec3& at(int x, int y) { return pixels[y * width + x]; }
    const Vec3& at(int x, int y) const { return pixels[y * width + x]; }

    float avgLuminance() const {
        float logSum = 0.f;
        int cnt = 0;
        for (const auto& p : pixels) {
            float lum = p.luminance();
            if (lum > 1e-5f) {
                logSum += std::log(lum);
                cnt++;
            }
        }
        return cnt > 0 ? std::exp(logSum / cnt) : 1.f;
    }

    float maxLuminance() const {
        float m = 0.f;
        for (const auto& p : pixels)
            m = std::max(m, p.luminance());
        return m;
    }
};

// ============================================================
// HDR Scene Generation
// ============================================================

// 生成一个包含多种亮度层次的HDR测试场景
// 模拟真实渲染输出：暗区(<0.1)、正常区(0.1-1.0)、高光区(1.0-20.0)
Image generateHDRScene(int width, int height) {
    Image img(width, height);
    
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float u = float(x) / width;
            float v = float(y) / height;

            Vec3 color(0.f);

            // 背景天空梯度（HDR: 0.5 ~ 3.0）
            float skyIntensity = 1.5f + 1.5f * (1.f - v);
            Vec3 skyColor = Vec3(0.4f, 0.6f, 1.0f) * skyIntensity;

            // 地面（暗区: 0.02 ~ 0.15）
            float groundIntensity = 0.02f + 0.13f * (v - 0.5f);
            Vec3 groundColor = Vec3(0.3f, 0.2f, 0.1f) * std::max(0.f, groundIntensity);

            // 混合天空/地面
            float horizon = 0.5f;
            if (v < horizon) {
                color = skyColor * (1.f - v / horizon) + Vec3(0.8f, 0.9f, 1.0f) * (v / horizon) * 2.0f;
            } else {
                float t = (v - horizon) / (1.f - horizon);
                color = lerp(Vec3(0.6f, 0.7f, 0.5f) * 0.3f, groundColor + Vec3(0.1f), t);
            }

            // 太阳 (超亮: 15~25)
            float sunU = 0.7f, sunV = 0.2f;
            float sunDist = std::sqrt((u - sunU)*(u - sunU) + (v - sunV)*(v - sunV));
            if (sunDist < 0.06f) {
                float sunGlow = std::exp(-sunDist * sunDist / 0.001f);
                float sunFull = std::exp(-sunDist * sunDist / 0.0003f);
                color += Vec3(20.0f, 18.0f, 12.0f) * sunFull;
                color += Vec3(6.0f, 5.0f, 3.0f) * sunGlow;
            }

            // 窗口/灯光区域 (高亮: 3~10)
            auto addLight = [&](float cx, float cy, float rx, float ry, Vec3 lc, float intensity) {
                if (u >= cx - rx && u <= cx + rx && v >= cy - ry && v <= cy + ry) {
                    float dx = std::min(std::abs(u - (cx - rx)), std::abs(u - (cx + rx)));
                    float dy = std::min(std::abs(v - (cy - ry)), std::abs(v - (cy + ry)));
                    float edge = std::min(dx, dy) / std::max(rx, ry) * 8.f;
                    edge = std::min(1.f, edge);
                    color += lc * intensity * edge;
                }
            };
            addLight(0.15f, 0.65f, 0.08f, 0.1f, Vec3(1.0f, 0.8f, 0.5f), 5.f);   // 暖光窗
            addLight(0.40f, 0.72f, 0.06f, 0.08f, Vec3(0.5f, 0.7f, 1.0f), 4.f);  // 冷光屏
            addLight(0.60f, 0.6f, 0.05f, 0.06f, Vec3(1.0f, 0.4f, 0.2f), 7.f);   // 橙色灯

            // 萤火虫/粒子高光
            std::mt19937 prng((uint32_t)(u * 1000) * 1000 + (uint32_t)(v * 1000));
            std::uniform_real_distribution<float> pd(0.f, 1.f);
            if (pd(prng) > 0.998f && v > 0.5f) {
                float brightness = 2.f + pd(prng) * 8.f;
                color += Vec3(pd(prng), pd(prng) * 0.5f + 0.5f, pd(prng)) * brightness;
            }

            img.at(x, y) = color;
        }
    }

    return img;
}

// ============================================================
// Tone Mapping Operators
// ============================================================

// 1. Reinhard (全局) — 最简单的HDR压缩
Vec3 tonemapReinhard(Vec3 color, float exposure = 1.f) {
    color = color * exposure;
    return color / (color + Vec3(1.f));
}

// 2. Reinhard Extended — 改进白点保护
Vec3 tonemapReinhardExtended(Vec3 color, float whitePoint = 4.f, float exposure = 1.f) {
    color = color * exposure;
    Vec3 numerator = color * (Vec3(1.f) + color / (whitePoint * whitePoint));
    return numerator / (color + Vec3(1.f));
}

// 3. ACES Filmic (Hill 近似) — 工业标准
// 参考: https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
Vec3 tonemapACES(Vec3 color, float exposure = 0.6f) {
    color = color * exposure;
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    Vec3 num = color * (color * a + Vec3(b));
    Vec3 den = color * (color * c + Vec3(d)) + Vec3(e);
    Vec3 result;
    result.x = num.x / den.x;
    result.y = num.y / den.y;
    result.z = num.z / den.z;
    return result.clamp(0.f, 1.f);
}

// 4. Uncharted 2 / Hable Filmic
// 参考: http://filmicgames.com/archives/75
Vec3 hableOp(Vec3 x) {
    const float A = 0.15f, B = 0.50f, C = 0.10f, D = 0.20f, E = 0.02f, F = 0.30f;
    return (x * (x * A + Vec3(C * B)) + Vec3(D * E)) / (x * (x * A + Vec3(B)) + Vec3(D * F)) - Vec3(E / F);
}

Vec3 tonemapUncharted2(Vec3 color, float exposure = 2.f) {
    color = color * exposure;
    Vec3 curr = hableOp(color);
    Vec3 whiteScale = Vec3(1.f) / hableOp(Vec3(11.2f));
    Vec3 result;
    result.x = curr.x * whiteScale.x;
    result.y = curr.y * whiteScale.y;
    result.z = curr.z * whiteScale.z;
    return result.clamp(0.f, 1.f);
}

// 5. Lottes Filmic
// 参考: https://gpuopen.com/learn/optimized-reversible-tonemapper-for-resolve/
Vec3 tonemapLottes(Vec3 color, float exposure = 1.f) {
    color = color * exposure;
    // Lottes 2016 curve
    const float a = 1.6f, d = 0.977f, hdrMax = 8.f, midIn = 0.18f, midOut = 0.267f;
    float b = (-std::pow(midIn, a) + std::pow(hdrMax, a) * midOut) /
              ((std::pow(hdrMax, a * d) - std::pow(midIn, a * d)) * midOut);
    float c2 = (std::pow(hdrMax, a * d) * std::pow(midIn, a) - std::pow(hdrMax, a) * std::pow(midIn, a * d) * midOut) /
               ((std::pow(hdrMax, a * d) - std::pow(midIn, a * d)) * midOut);
    auto lottesChannel = [&](float v) -> float {
        return std::pow(v, a) / (std::pow(v, a * d) * b + c2);
    };
    return Vec3(lottesChannel(color.x), lottesChannel(color.y), lottesChannel(color.z)).clamp(0.f, 1.f);
}

// 6. Gamma Correct Only (曝光 + 伽马，无压缩)
Vec3 tonemapGammaOnly(Vec3 color, float exposure = 1.f) {
    color = color * exposure;
    return color.clamp(0.f, 1.f);  // 直接裁剪
}

// ============================================================
// Color Grading
// ============================================================

// sRGB 伽马编码 (线性 → 显示器)
Vec3 gammaEncode(Vec3 color) {
    auto enc = [](float v) -> float {
        v = std::max(0.f, v);
        return v <= 0.0031308f ? 12.92f * v : 1.055f * std::pow(v, 1.f / 2.4f) - 0.055f;
    };
    return Vec3(enc(color.x), enc(color.y), enc(color.z));
}

// 对比度调整（S形曲线）
Vec3 adjustContrast(Vec3 color, float contrast = 1.2f) {
    auto curve = [&](float v) -> float {
        v = v - 0.5f;
        return v * contrast + 0.5f;
    };
    return Vec3(curve(color.x), curve(color.y), curve(color.z)).clamp(0.f, 1.f);
}

// 饱和度调整
Vec3 adjustSaturation(Vec3 color, float saturation = 1.3f) {
    float lum = color.luminance();
    return lerp(Vec3(lum), color, saturation).clamp(0.f, 1.f);
}

// 色温调整（暖/冷）
Vec3 adjustTemperature(Vec3 color, float temperature = 0.1f) {
    // 正值=暖（加红黄），负值=冷（加蓝）
    color.x = std::min(1.f, color.x + temperature * 0.2f);
    color.y = std::min(1.f, color.y + temperature * 0.05f);
    color.z = std::min(1.f, color.z - temperature * 0.15f);
    return color.clamp(0.f, 1.f);
}

// Shadows/Midtones/Highlights 分色
Vec3 splitToning(Vec3 color, 
                  Vec3 shadowColor = Vec3(0.05f, 0.05f, 0.1f),
                  Vec3 highlightColor = Vec3(0.1f, 0.07f, 0.0f)) {
    float lum = color.luminance();
    float shadowW = 1.f - std::min(1.f, lum * 2.f);   // 暗部权重
    float highlightW = std::max(0.f, lum * 2.f - 1.f); // 亮部权重
    color += shadowColor * shadowW;
    color += highlightColor * highlightW;
    return color.clamp(0.f, 1.f);
}

// 完整色彩分级管线
Vec3 colorGrade(Vec3 color) {
    color = adjustContrast(color, 1.15f);
    color = adjustSaturation(color, 1.2f);
    color = adjustTemperature(color, 0.08f);  // 略微暖色
    color = splitToning(color, 
                        Vec3(0.02f, 0.02f, 0.05f),   // 阴影略蓝
                        Vec3(0.08f, 0.04f, 0.0f));    // 高光略暖
    return color;
}

// ============================================================
// PNG Writer (简易无压缩 PNG)
// ============================================================

void writePNG(const std::string& filename, const std::vector<uint8_t>& data, int width, int height) {
    // 使用 PPM 格式（更简单，无需压缩库）
    // 但命名为 .png 便于后续 Python 检验
    // 实际用 ppm 先验证，再用 convert 转换
    std::string ppmName = filename.substr(0, filename.find_last_of('.')) + ".ppm";
    std::ofstream f(ppmName, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file: " + ppmName);
    f << "P6\n" << width << " " << height << "\n255\n";
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
    f.close();
    std::cout << "  PPM written: " << ppmName << " (" << width << "x" << height << ")\n";
}

void writeComparisonImage(const std::string& filename, 
                           const std::vector<std::pair<std::string, Image>>& panels) {
    if (panels.empty()) return;
    
    int panelW = panels[0].second.width;
    int panelH = panels[0].second.height;
    int cols = 3;  // 每行3列
    int rows = (int(panels.size()) + cols - 1) / cols;
    int labelH = 20;  // 标签高度
    int totalW = panelW * cols;
    int totalH = (panelH + labelH) * rows;

    std::vector<uint8_t> pixels(totalW * totalH * 3, 30);  // 深灰背景

    for (int pi = 0; pi < (int)panels.size(); pi++) {
        int col = pi % cols;
        int row = pi / cols;
        int ox = col * panelW;
        int oy = row * (panelH + labelH);

        const Image& img = panels[pi].second;
        for (int y = 0; y < panelH; y++) {
            for (int x = 0; x < panelW; x++) {
                Vec3 c = img.at(x, y).clamp(0.f, 1.f);
                int px = (oy + y) * totalW + (ox + x);
                pixels[px * 3 + 0] = (uint8_t)(c.x * 255.f + 0.5f);
                pixels[px * 3 + 1] = (uint8_t)(c.y * 255.f + 0.5f);
                pixels[px * 3 + 2] = (uint8_t)(c.z * 255.f + 0.5f);
            }
        }

        // 绘制标签背景（黑色条）
        for (int y = 0; y < labelH; y++) {
            for (int x = 0; x < panelW; x++) {
                int px = (oy + panelH + y) * totalW + (ox + x);
                pixels[px * 3 + 0] = 20;
                pixels[px * 3 + 1] = 20;
                pixels[px * 3 + 2] = 20;
            }
        }

        // 简单文字渲染（用像素点画字母，8x5 字体）
        // 为简洁起见，用 '#' 标记在PPM里是无法显示文字的
        // 实际标签通过控制台输出
    }

    // 写 PPM
    std::string ppmName = filename.substr(0, filename.find_last_of('.')) + "_compare.ppm";
    std::ofstream f(ppmName, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open: " + ppmName);
    f << "P6\n" << totalW << " " << totalH << "\n255\n";
    f.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
    f.close();
    std::cout << "  Comparison PPM: " << ppmName << " (" << totalW << "x" << totalH << ")\n";
}

// ============================================================
// Apply tone map to entire image
// ============================================================

using ToneMapper = std::function<Vec3(Vec3)>;

Image applyToneMap(const Image& hdr, ToneMapper tm, bool doColorGrade = false) {
    Image ldr(hdr.width, hdr.height);
    for (int y = 0; y < hdr.height; y++) {
        for (int x = 0; x < hdr.width; x++) {
            Vec3 c = hdr.at(x, y);
            c = tm(c);
            if (doColorGrade) c = colorGrade(c);
            c = gammaEncode(c);
            ldr.at(x, y) = c.clamp(0.f, 1.f);
        }
    }
    return ldr;
}

// ============================================================
// Write single image as PPM
// ============================================================

void writeImagePPM(const Image& img, const std::string& filename) {
    std::ofstream f(filename, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open: " + filename);
    f << "P6\n" << img.width << " " << img.height << "\n255\n";
    for (int y = 0; y < img.height; y++) {
        for (int x = 0; x < img.width; x++) {
            Vec3 c = img.at(x, y).clamp(0.f, 1.f);
            uint8_t r = (uint8_t)(c.x * 255.f + 0.5f);
            uint8_t g = (uint8_t)(c.y * 255.f + 0.5f);
            uint8_t b = (uint8_t)(c.z * 255.f + 0.5f);
            f.write(reinterpret_cast<char*>(&r), 1);
            f.write(reinterpret_cast<char*>(&g), 1);
            f.write(reinterpret_cast<char*>(&b), 1);
        }
    }
    std::cout << "  Wrote: " << filename << "\n";
}

// ============================================================
// Main
// ============================================================

int main() {
    std::cout << "=== HDR Tone Mapping & Color Grading ===\n\n";

    const int W = 320, H = 200;

    std::cout << "[1/4] Generating HDR scene (" << W << "x" << H << ")...\n";
    Image hdr = generateHDRScene(W, H);
    
    float avgLum = hdr.avgLuminance();
    float maxLum = hdr.maxLuminance();
    std::cout << "  HDR stats: avg_lum=" << avgLum << "  max_lum=" << maxLum << "\n";
    assert(maxLum > 5.f && "HDR scene should have bright highlights");
    assert(avgLum > 0.1f && "HDR scene should not be too dark");

    std::cout << "\n[2/4] Applying tone mapping operators...\n";

    // 定义所有操作符
    struct ToneOp {
        std::string name;
        std::string filename;
        ToneMapper fn;
        bool colorGrade;
    };

    std::vector<ToneOp> ops = {
        {"Gamma Only (clip)",  "tm_gamma.ppm",      [](Vec3 c){ return tonemapGammaOnly(c, 1.f); },       false},
        {"Reinhard",           "tm_reinhard.ppm",   [](Vec3 c){ return tonemapReinhard(c, 1.f); },        false},
        {"Reinhard Extended",  "tm_reinhard_ext.ppm",[](Vec3 c){ return tonemapReinhardExtended(c, 4.f, 1.f); }, false},
        {"ACES Filmic",        "tm_aces.ppm",       [](Vec3 c){ return tonemapACES(c, 0.6f); },           false},
        {"Uncharted 2",        "tm_uncharted2.ppm", [](Vec3 c){ return tonemapUncharted2(c, 2.f); },      false},
        {"Lottes Filmic",      "tm_lottes.ppm",     [](Vec3 c){ return tonemapLottes(c, 1.f); },          false},
        {"ACES + ColorGrade",  "tm_aces_graded.ppm",[](Vec3 c){ return tonemapACES(c, 0.6f); },           true},
    };

    std::vector<std::pair<std::string, Image>> panels;
    
    for (auto& op : ops) {
        std::cout << "  Processing: " << op.name << "...\n";
        Image ldr = applyToneMap(hdr, op.fn, op.colorGrade);
        writeImagePPM(ldr, op.filename);
        panels.push_back({op.name, ldr});
    }

    std::cout << "\n[3/4] Building comparison grid...\n";
    // 拼接比较图（4列 × 2行，7个变体最后一格留空但视觉更均衡）
    int cols = 4;
    int rows = ((int)panels.size() + cols - 1) / cols;
    int labelH = 4;  // 标签空间（像素分隔线）
    int totalW = W * cols;
    int totalH = (H + labelH) * rows;
    
    std::vector<uint8_t> grid(totalW * totalH * 3, 15);  // 深色背景

    for (int pi = 0; pi < (int)panels.size(); pi++) {
        int col = pi % cols;
        int row = pi / cols;
        int ox = col * W;
        int oy = row * (H + labelH);

        const Image& img = panels[pi].second;
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                Vec3 c = img.at(x, y).clamp(0.f, 1.f);
                int gx = ox + x;
                int gy = oy + y;
                int idx = (gy * totalW + gx) * 3;
                grid[idx + 0] = (uint8_t)(c.x * 255.f + 0.5f);
                grid[idx + 1] = (uint8_t)(c.y * 255.f + 0.5f);
                grid[idx + 2] = (uint8_t)(c.z * 255.f + 0.5f);
            }
        }

        // 分隔线（白色细线）
        for (int x = 0; x < W; x++) {
            int gx = ox + x;
            for (int ly = 0; ly < labelH; ly++) {
                int gy = oy + H + ly;
                int idx = (gy * totalW + gx) * 3;
                uint8_t val = (ly == 0 || ly == labelH-1) ? 200 : 30;
                grid[idx + 0] = val;
                grid[idx + 1] = val;
                grid[idx + 2] = val;
            }
        }

        // 左边竖线
        for (int y = 0; y < H + labelH; y++) {
            int gy = oy + y;
            int idx = (gy * totalW + ox) * 3;
            grid[idx + 0] = 80; grid[idx + 1] = 80; grid[idx + 2] = 80;
        }
    }

    // 写比较图
    std::ofstream gf("tonemap_comparison.ppm", std::ios::binary);
    if (!gf) { std::cerr << "Cannot write comparison file\n"; return 1; }
    gf << "P6\n" << totalW << " " << totalH << "\n255\n";
    gf.write(reinterpret_cast<char*>(grid.data()), grid.size());
    gf.close();
    std::cout << "  Comparison grid: tonemap_comparison.ppm (" << totalW << "x" << totalH << ")\n";

    std::cout << "\n[4/4] Validation...\n";

    // 验证每个输出图像的像素统计
    bool allValid = true;
    for (auto& [name, img] : panels) {
        float totalR = 0, totalG = 0, totalB = 0;
        int n = img.width * img.height;
        for (const auto& p : img.pixels) {
            totalR += p.x;
            totalG += p.y;
            totalB += p.z;
        }
        float meanR = totalR / n;
        float meanG = totalG / n;
        float meanB = totalB / n;
        float mean = (meanR + meanG + meanB) / 3.f;
        
        // 计算标准差
        float varSum = 0;
        for (const auto& p : img.pixels) {
            float lum = p.luminance();
            float d = lum - mean;
            varSum += d * d;
        }
        float stddev = std::sqrt(varSum / n);

        std::cout << "  [" << name << "] mean=" << mean 
                  << " std=" << stddev << "\n";

        if (mean < 0.02f || mean > 0.98f) {
            std::cerr << "  ❌ " << name << ": 均值超出范围 (" << mean << ")\n";
            allValid = false;
        }
        if (stddev < 0.02f) {
            std::cerr << "  ❌ " << name << ": 标准差过低，图像无内容 (" << stddev << ")\n";
            allValid = false;
        }
    }

    if (!allValid) {
        std::cerr << "\n❌ 验证失败！\n";
        return 1;
    }

    std::cout << "\n✅ All tone mapping operators validated!\n";
    std::cout << "\n=== Summary ===\n";
    std::cout << "HDR scene: avg_luminance=" << avgLum << ", max_luminance=" << maxLum << "\n";
    std::cout << "Outputs:\n";
    for (auto& op : ops) {
        std::cout << "  - " << op.filename << " [" << op.name << "]\n";
    }
    std::cout << "  - tonemap_comparison.ppm [Comparison Grid " << totalW << "x" << totalH << "]\n";
    std::cout << "\nDone!\n";
    return 0;
}
