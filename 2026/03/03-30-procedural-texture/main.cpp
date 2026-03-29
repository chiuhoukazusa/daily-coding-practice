/*
 * Procedural Texture Synthesis
 * 程序化纹理合成：Worley噪声 + Fractal Brownian Motion
 * 
 * 生成以下纹理：
 * 1. Worley F1 噪声（细胞/Voronoi纹理）
 * 2. Worley F2-F1 噪声（细胞边缘纹理）
 * 3. 大理石纹理（Perlin + 正弦扰动）
 * 4. 木纹纹理（同心圆 + 噪声扰动）
 * 5. 熔岩/气泡纹理（Worley F1反色）
 * 6. 有机细胞纹理（混合Worley）
 * 7. 对比图（所有纹理拼接）
 *
 * Date: 2026-03-30
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <iostream>
#include <sstream>
#include <array>

// ============================================================
// Vec2 / Vec3 helpers
// ============================================================

struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
    float dot(const Vec2& o) const { return x*o.x + y*o.y; }
    float length() const { return std::sqrt(x*x + y*y); }
};

struct Vec3 {
    float r, g, b;
    Vec3() : r(0), g(0), b(0) {}
    Vec3(float r, float g, float b) : r(r), g(g), b(b) {}
    Vec3 operator+(const Vec3& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Vec3 operator*(float t) const { return {r*t, g*t, b*t}; }
    Vec3 operator*(const Vec3& o) const { return {r*o.r, g*o.g, b*o.b}; }
    static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return a * (1-t) + b * t;
    }
};

// ============================================================
// Hash utilities
// ============================================================

// 快速整数哈希（确定性）
inline uint32_t hash2(int32_t x, int32_t y) {
    uint32_t h = (uint32_t)(x * 1664525u + 1013904223u) ^ (uint32_t)(y * 22695477u + 1u);
    h ^= h >> 16;
    h *= 0x45d9f3b;
    h ^= h >> 16;
    return h;
}

// 返回 [0, 1) 的伪随机浮点
inline float rand2f(int32_t x, int32_t y) {
    return (float)(hash2(x, y) & 0xFFFFFF) / (float)0x1000000;
}

// 返回网格点的随机偏移（[0,1)^2）
inline Vec2 cellPoint(int32_t cx, int32_t cy) {
    return {rand2f(cx, cy), rand2f(cy, cx + 7919)};
}

// ============================================================
// Worley Noise（细胞/Voronoi噪声）
// 返回 {F1, F2}：到最近点距离、第二近距离（均归一化到约 [0,1]）
// ============================================================

struct WorleyResult {
    float f1, f2;
};

WorleyResult worley(float px, float py, float scale = 4.0f) {
    float sx = px * scale;
    float sy = py * scale;
    
    int ix = (int)std::floor(sx);
    int iy = (int)std::floor(sy);

    float f1 = 1e9f, f2 = 1e9f;

    for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
            int cx = ix + dx;
            int cy = iy + dy;
            Vec2 cp = cellPoint(cx, cy);
            float ox = (float)cx + cp.x;
            float oy = (float)cy + cp.y;
            float ddx = ox - sx;
            float ddy = oy - sy;
            float dist = std::sqrt(ddx*ddx + ddy*ddy);
            if (dist < f1) { f2 = f1; f1 = dist; }
            else if (dist < f2) { f2 = dist; }
        }
    }
    // 归一化：在scale=4时，F1典型最大值约0.7，F2约1.0
    return { std::min(f1 / 0.7f, 1.0f), std::min(f2 / 1.0f, 1.0f) };
}

// ============================================================
// Perlin Noise 2D（用于大理石/木纹扰动）
// ============================================================

// 梯度表
static const Vec2 kGrad[8] = {
    {1,0},{-1,0},{0,1},{0,-1},
    {0.7071f,0.7071f},{-0.7071f,0.7071f},{0.7071f,-0.7071f},{-0.7071f,-0.7071f}
};

inline Vec2 gradient(int32_t x, int32_t y) {
    return kGrad[hash2(x, y) & 7];
}

inline float smoothstep(float t) {
    return t * t * t * (t * (t * 6 - 15) + 10); // 6t^5 - 15t^4 + 10t^3
}

float perlin(float px, float py) {
    int ix = (int)std::floor(px);
    int iy = (int)std::floor(py);
    float fx = px - ix;
    float fy = py - iy;
    float ux = smoothstep(fx);
    float uy = smoothstep(fy);

    auto dot_grad = [&](int cx, int cy, float dx, float dy) {
        Vec2 g = gradient(cx, cy);
        return g.x * dx + g.y * dy;
    };

    float v00 = dot_grad(ix,   iy,   fx,   fy  );
    float v10 = dot_grad(ix+1, iy,   fx-1, fy  );
    float v01 = dot_grad(ix,   iy+1, fx,   fy-1);
    float v11 = dot_grad(ix+1, iy+1, fx-1, fy-1);

    float a = v00 + ux * (v10 - v00);
    float b = v01 + ux * (v11 - v01);
    return a + uy * (b - a); // 约 [-0.7, 0.7]
}

// Fractal Brownian Motion
float fbm(float px, float py, int octaves = 6) {
    float val = 0.0f;
    float amp = 0.5f;
    float freq = 1.0f;
    for (int i = 0; i < octaves; ++i) {
        val += perlin(px * freq, py * freq) * amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return val; // 约 [-1, 1]
}

// ============================================================
// 纹理生成函数（返回 Vec3 颜色，输入 UV [0,1]^2）
// ============================================================

// 1. Worley F1 细胞纹理
Vec3 textureWorleyF1(float u, float v) {
    auto [f1, f2] = worley(u, v, 6.0f);
    // 深色边缘，亮色中心（反转+幂次）
    float t = 1.0f - std::pow(f1, 0.5f);
    // 着色：蓝紫色调
    Vec3 dark(0.05f, 0.02f, 0.15f);
    Vec3 bright(0.6f, 0.4f, 1.0f);
    return Vec3::lerp(dark, bright, t);
}

// 2. Worley F2-F1 边缘纹理
Vec3 textureWorleyEdge(float u, float v) {
    auto [f1, f2] = worley(u, v, 6.0f);
    float edge = f2 - f1; // 在细胞边界处较大
    float t = std::pow(std::min(edge / 0.5f, 1.0f), 0.5f);
    // 着色：深色细胞，亮色边缘（骨骼感）
    Vec3 cell(0.05f, 0.1f, 0.05f);
    Vec3 boundary(0.8f, 0.9f, 0.7f);
    return Vec3::lerp(cell, boundary, t);
}

// 3. 大理石纹理
Vec3 textureMarble(float u, float v) {
    float noise = fbm(u * 4.0f, v * 4.0f, 6);
    // 正弦条纹 + 噪声扰动
    float marble = std::sin((u * 6.0f + noise * 4.0f) * 3.14159f);
    marble = marble * 0.5f + 0.5f; // [0, 1]
    marble = std::pow(marble, 0.7f);

    Vec3 white(0.95f, 0.93f, 0.9f);
    Vec3 vein(0.3f, 0.25f, 0.2f);
    Vec3 darkVein(0.1f, 0.08f, 0.06f);

    // 双层混合
    float t1 = marble;
    float t2 = std::pow(marble, 3.0f);
    Vec3 col = Vec3::lerp(white, vein, t1 * 0.6f);
    col = Vec3::lerp(col, darkVein, t2 * 0.4f);
    return col;
}

// 4. 木纹纹理
Vec3 textureWood(float u, float v) {
    // 同心圆 + 噪声扰动
    float cx = u - 0.5f, cy = v - 0.5f;
    float dist = std::sqrt(cx*cx + cy*cy);
    float noise = fbm(u * 3.0f, v * 3.0f, 4) * 0.3f;
    float ring = (dist + noise) * 15.0f;
    float t = std::fmod(ring, 1.0f);
    t = std::sin(t * 3.14159f); // 0→1→0 的平滑波

    Vec3 light(0.82f, 0.62f, 0.35f);
    Vec3 dark(0.45f, 0.25f, 0.1f);
    return Vec3::lerp(dark, light, t);
}

// 5. 熔岩/气泡纹理（Worley反色 + 橙色着色）
Vec3 textureLava(float u, float v) {
    auto [f1, f2] = worley(u, v, 5.0f);
    // 反转F1：气泡中心亮，边缘暗
    float t = std::pow(f1, 1.5f);
    // 叠加FBM细节
    float detail = fbm(u * 8.0f, v * 8.0f, 4) * 0.15f + 0.15f;
    t = std::clamp(t + detail, 0.0f, 1.0f);

    Vec3 hot(1.0f, 0.9f, 0.2f);   // 亮黄色（热点）
    Vec3 warm(0.9f, 0.3f, 0.05f); // 橙红色
    Vec3 cool(0.1f, 0.02f, 0.0f); // 暗红色（冷却）
    
    Vec3 col;
    if (t < 0.5f) {
        col = Vec3::lerp(hot, warm, t * 2.0f);
    } else {
        col = Vec3::lerp(warm, cool, (t - 0.5f) * 2.0f);
    }
    return col;
}

// 6. 有机细胞纹理（混合F1+FBM，类皮肤/生物膜）
Vec3 textureOrganic(float u, float v) {
    auto [f1, f2] = worley(u, v, 7.0f);
    float noise = fbm(u * 5.0f, v * 5.0f, 5) * 0.2f;
    
    float t = f1 + noise;
    t = std::clamp(t, 0.0f, 1.0f);
    
    // 细胞内部绿色，边缘深色
    Vec3 inner(0.4f, 0.8f, 0.5f);
    Vec3 wall(0.05f, 0.15f, 0.1f);
    Vec3 nucleus(0.8f, 1.0f, 0.7f);
    
    Vec3 col;
    if (t < 0.3f) {
        col = Vec3::lerp(nucleus, inner, t / 0.3f);
    } else {
        col = Vec3::lerp(inner, wall, (t - 0.3f) / 0.7f);
    }
    return col;
}

// ============================================================
// 图像写入
// ============================================================

struct Image {
    int width, height;
    std::vector<uint8_t> data; // RGB
    
    Image(int w, int h) : width(w), height(h), data(w * h * 3, 0) {}
    
    void setPixel(int x, int y, Vec3 col) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        data[idx+0] = (uint8_t)std::clamp((int)(col.r * 255.0f), 0, 255);
        data[idx+1] = (uint8_t)std::clamp((int)(col.g * 255.0f), 0, 255);
        data[idx+2] = (uint8_t)std::clamp((int)(col.b * 255.0f), 0, 255);
    }
    
    bool save(const std::string& path) const {
        return stbi_write_png(path.c_str(), width, height, 3, data.data(), width * 3) != 0;
    }
};

// 渲染单张纹理
using TextureFunc = Vec3(*)(float, float);

Image renderTexture(TextureFunc fn, int w, int h) {
    Image img(w, h);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float u = (float)x / (float)w;
            float v = 1.0f - (float)y / (float)h; // 翻转Y轴，使V=0在下方
            Vec3 col = fn(u, v);
            img.setPixel(x, y, col);
        }
    }
    return img;
}

// ============================================================
// Main
// ============================================================

int main() {
    const int W = 512, H = 512;
    const std::string outDir = "./";
    
    std::cout << "=== Procedural Texture Synthesis ===" << std::endl;
    std::cout << "Date: 2026-03-30" << std::endl;
    std::cout << "Resolution: " << W << "x" << H << std::endl;
    std::cout << std::endl;

    // 定义所有纹理
    struct TextureInfo {
        std::string name;
        std::string filename;
        TextureFunc fn;
    };

    std::vector<TextureInfo> textures = {
        {"Worley F1 (Cell)", "worley_f1.png", textureWorleyF1},
        {"Worley Edge (F2-F1)", "worley_edge.png", textureWorleyEdge},
        {"Marble", "marble.png", textureMarble},
        {"Wood", "wood.png", textureWood},
        {"Lava/Bubble", "lava.png", textureLava},
        {"Organic Cell", "organic.png", textureOrganic},
    };

    // 渲染并保存每张纹理
    std::vector<Image> rendered;
    for (auto& tex : textures) {
        std::cout << "Rendering: " << tex.name << " ..." << std::flush;
        Image img = renderTexture(tex.fn, W, H);
        std::string path = outDir + tex.filename;
        if (!img.save(path)) {
            std::cerr << "\nERROR: Failed to save " << path << std::endl;
            return 1;
        }
        std::cout << " saved -> " << path << std::endl;
        rendered.push_back(std::move(img));
    }

    // 生成 3x2 对比图
    std::cout << "\nGenerating comparison grid..." << std::flush;
    const int COLS = 3, ROWS = 2;
    const int MARGIN = 8; // 间距
    Image grid(COLS * W + (COLS+1) * MARGIN, ROWS * H + (ROWS+1) * MARGIN);
    
    // 背景色（深灰）
    for (auto& px : grid.data) px = 32;
    
    for (int i = 0; i < (int)rendered.size(); ++i) {
        int col = i % COLS;
        int row = i / COLS;
        int ox = MARGIN + col * (W + MARGIN);
        int oy = MARGIN + row * (H + MARGIN);
        
        const Image& src = rendered[i];
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int si = (y * src.width + x) * 3;
                Vec3 c(
                    src.data[si+0] / 255.0f,
                    src.data[si+1] / 255.0f,
                    src.data[si+2] / 255.0f
                );
                grid.setPixel(ox + x, oy + y, c);
            }
        }
    }
    
    std::string gridPath = outDir + "procedural_texture_output.png";
    if (!grid.save(gridPath)) {
        std::cerr << "ERROR: Failed to save grid" << std::endl;
        return 1;
    }
    std::cout << " saved -> " << gridPath << std::endl;
    
    // 输出统计信息（供验证脚本读取）
    std::cout << "\n=== Output Summary ===" << std::endl;
    std::cout << "Total textures: " << textures.size() << std::endl;
    std::cout << "Grid size: " << grid.width << "x" << grid.height << std::endl;
    for (size_t i = 0; i < textures.size(); ++i) {
        std::cout << "  [" << i+1 << "] " << textures[i].name << " -> " << textures[i].filename << std::endl;
    }
    std::cout << "\n✅ All textures generated successfully!" << std::endl;
    
    return 0;
}
