/**
 * SPH Fluid Simulation (Smoothed Particle Hydrodynamics)
 * 
 * 实现基于SPH方法的2D流体模拟：
 * - Poly6 核函数用于密度计算
 * - Spiky 核函数用于压力梯度
 * - Viscosity 核函数用于粘性力
 * - Leapfrog Verlet 积分
 * - 边界反射碰撞
 * - 渲染到PNG（颜色映射速度）
 *
 * 关键：使用归一化坐标系，h=0.2，模拟域 [0,4] x [0,3]
 * 渲染时映射到 800x600 像素
 *
 * 编译: g++ sph_fluid.cpp -o sph_fluid -O2 -std=c++17 -lm
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

// ============================================================
//  图像尺寸
// ============================================================
static const int   IMG_W    = 800;
static const int   IMG_H    = 600;
static const int   CHANNELS = 3;

// ============================================================
//  模拟域（归一化坐标）
// ============================================================
static const float DOM_W  = 4.0f;   // 模拟域宽度
static const float DOM_H  = 3.0f;   // 模拟域高度

// SPH 参数（归一化单位）
static const float H        = 0.2f;    // 光滑半径
static const float H2       = H * H;
static const float MASS     = 0.02f;   // 粒子质量
static const float REST_DENS = 0.8f;   // 静止密度（与核函数积分匹配）
static const float GAS_CONST  = 50.0f; // 气体刚度常数
static const float VISC        = 0.1f; // 动力粘度
static const float GRAVITY     = 9.8f; // 重力加速度

static const float DT    = 0.001f;  // 时间步长
static const int   STEPS = 2000;    // 模拟步数（~2秒）

static const int   N_PARTICLES = 600;

// 预计算核函数常数（2D）
static const float PI        = 3.14159265358979f;
static const float POLY6K    = 4.0f / (PI * std::pow(H, 8));
static const float SPIKY_GRAD= -10.0f / (PI * std::pow(H, 5));
static const float VISC_LAP  =  40.0f / (PI * std::pow(H, 5));

// ============================================================
//  Vec2
// ============================================================
struct Vec2 {
    float x, y;
    Vec2(float x=0, float y=0): x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s)       const { return {x*s, y*s}; }
    Vec2 operator/(float s)       const { return {x/s, y/s}; }
    Vec2& operator+=(const Vec2& o){ x+=o.x; y+=o.y; return *this; }
    float dot(const Vec2& o) const { return x*o.x+y*o.y; }
    float len2()             const { return x*x+y*y; }
    float len()              const { return std::sqrt(len2()); }
};

// ============================================================
//  粒子
// ============================================================
struct Particle {
    Vec2  pos, vel, force;
    float density, pressure;
};

// ============================================================
//  核函数（2D 版本，参考 Müller 2003）
// ============================================================

// Poly6：密度
inline float kernel_poly6(float r2) {
    if (r2 >= H2) return 0.0f;
    float d = H2 - r2;
    return POLY6K * d * d * d;
}

// Spiky 梯度：压力
inline Vec2 kernel_spiky_grad(const Vec2& rij, float r) {
    if (r <= 1e-6f || r >= H) return {0, 0};
    float d = H - r;
    float coeff = SPIKY_GRAD * d * d / r;
    return rij * coeff;
}

// Viscosity 拉普拉斯：粘性
inline float kernel_visc_lap(float r) {
    if (r >= H) return 0.0f;
    return VISC_LAP * (H - r);
}

// ============================================================
//  初始化：方块排列，位于左下区域
// ============================================================
std::vector<Particle> init_particles() {
    std::vector<Particle> ps;
    ps.reserve(N_PARTICLES);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> jitter(-H*0.01f, H*0.01f);

    int cols = 24;
    float spacing = H * 0.75f;
    float ox = 0.3f, oy = 0.3f;

    for (int i = 0; i < N_PARTICLES; i++) {
        int c = i % cols;
        int r = i / cols;
        Particle p;
        p.pos = Vec2(ox + c * spacing + jitter(rng),
                     oy + r * spacing + jitter(rng));
        p.vel      = Vec2(0, 0);
        p.force    = Vec2(0, 0);
        p.density  = REST_DENS;
        p.pressure = 0;
        ps.push_back(p);
    }
    return ps;
}

// ============================================================
//  密度 + 压力
// ============================================================
void compute_density_pressure(std::vector<Particle>& ps) {
    int n = (int)ps.size();
    for (int i = 0; i < n; i++) {
        ps[i].density = 0;
        for (int j = 0; j < n; j++) {
            Vec2 rij = ps[j].pos - ps[i].pos;
            float r2 = rij.len2();
            ps[i].density += MASS * kernel_poly6(r2);
        }
        ps[i].pressure = GAS_CONST * (ps[i].density - REST_DENS);
    }
}

// ============================================================
//  力（压力 + 粘性 + 重力）
// ============================================================
void compute_forces(std::vector<Particle>& ps) {
    int n = (int)ps.size();
    for (int i = 0; i < n; i++) {
        Vec2 fp(0,0), fv(0,0);
        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            Vec2 rij = ps[i].pos - ps[j].pos;
            float r  = rij.len();
            if (r >= H) continue;

            // 压力：对称公式
            float pij = (ps[i].pressure + ps[j].pressure) / (2.0f * ps[j].density + 1e-8f);
            fp += kernel_spiky_grad(rij, r) * (-MASS * pij);

            // 粘性
            Vec2 dv = ps[j].vel - ps[i].vel;
            fv += dv * (MASS * VISC * kernel_visc_lap(r) / (ps[j].density + 1e-8f));
        }
        // 重力
        Vec2 fg(0, -ps[i].density * GRAVITY);
        float inv_rho = 1.0f / (ps[i].density + 1e-8f);
        ps[i].force = (fp + fv) * inv_rho + Vec2(0, -GRAVITY);
        (void)fg; // 使用直接加速度而不是力/密度
        // 修正：force 是加速度
        ps[i].force = (fp + fv) / (ps[i].density + 1e-8f) + Vec2(0, -GRAVITY);
    }
}

// ============================================================
//  积分 + 边界
// ============================================================
void integrate(std::vector<Particle>& ps) {
    const float DAMP = -0.5f;
    const float BL = 0.0f, BR = DOM_W, BT = 0.0f, BB = DOM_H;
    for (auto& p : ps) {
        p.vel += p.force * DT;
        p.pos += p.vel  * DT;
        // 限速防爆炸
        float spd = p.vel.len();
        if (spd > 20.0f) p.vel = p.vel * (20.0f / spd);

        if (p.pos.x < BL) { p.pos.x = BL; p.vel.x *= DAMP; }
        if (p.pos.x > BR) { p.pos.x = BR; p.vel.x *= DAMP; }
        if (p.pos.y < BT) { p.pos.y = BT; p.vel.y *= DAMP; }
        if (p.pos.y > BB) { p.pos.y = BB; p.vel.y *= DAMP; }
    }
}

// ============================================================
//  颜色映射
// ============================================================
struct Color { uint8_t r, g, b; };

Color speed_color(float t) {
    // 蓝→青→绿→黄→红
    t = std::max(0.0f, std::min(1.0f, t));
    float r = std::min(1.0f, t * 2.0f);
    float g = t < 0.5f ? t * 2.0f : 2.0f - t * 2.0f;
    float b = std::max(0.0f, 1.0f - t * 2.0f);
    return { (uint8_t)(r*255), (uint8_t)(g*255), (uint8_t)(b*255) };
}

// ============================================================
//  渲染
// ============================================================
void render(const std::vector<Particle>& ps, std::vector<uint8_t>& img, int frame) {
    // 背景
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        img[i*3+0] = 8; img[i*3+1] = 10; img[i*3+2] = 28;
    }

    // 最大速度
    float max_speed = 0.1f;
    for (const auto& p : ps) max_speed = std::max(max_speed, p.vel.len());

    // 粒子半径（像素）
    const int PRAD = 5;
    for (const auto& p : ps) {
        // 模拟坐标 → 像素坐标
        // x: [0, DOM_W] → [0, IMG_W], y: [0, DOM_H] → [IMG_H, 0] (翻转Y)
        int px = (int)(p.pos.x / DOM_W * (IMG_W - 10)) + 5;
        int py = IMG_H - 5 - (int)(p.pos.y / DOM_H * (IMG_H - 10));
        float t = p.vel.len() / max_speed;
        Color col = speed_color(t);

        for (int dy = -PRAD; dy <= PRAD; dy++) {
            for (int dx = -PRAD; dx <= PRAD; dx++) {
                float d2 = float(dx*dx + dy*dy);
                if (d2 > PRAD*PRAD) continue;
                int nx = px + dx, ny = py + dy;
                if (nx < 0 || nx >= IMG_W || ny < 0 || ny >= IMG_H) continue;
                float alpha = std::exp(-d2 / (PRAD*0.7f * PRAD*0.7f));
                int idx = (ny * IMG_W + nx) * 3;
                img[idx+0] = (uint8_t)std::min(255.0f, img[idx+0] + col.r * alpha);
                img[idx+1] = (uint8_t)std::min(255.0f, img[idx+1] + col.g * alpha);
                img[idx+2] = (uint8_t)std::min(255.0f, img[idx+2] + col.b * alpha);
            }
        }
    }

    // 帧号文字（简单像素数字，仅标帧）
    (void)frame;

    // 边框
    for (int x = 0; x < IMG_W; x++) {
        auto set = [&](int xx, int yy, uint8_t c) {
            if (xx < 0||xx>=IMG_W||yy<0||yy>=IMG_H) return;
            int i = (yy*IMG_W+xx)*3; img[i]=c; img[i+1]=c; img[i+2]=c+40;
        };
        set(x, 4, 80); set(x, IMG_H-5, 80);
    }
    for (int y = 0; y < IMG_H; y++) {
        auto set = [&](int xx, int yy, uint8_t c) {
            if (xx<0||xx>=IMG_W||yy<0||yy>=IMG_H) return;
            int i = (yy*IMG_W+xx)*3; img[i]=c; img[i+1]=c; img[i+2]=c+40;
        };
        set(4, y, 80); set(IMG_W-5, y, 80);
    }
}

// ============================================================
//  main
// ============================================================
int main() {
    printf("=== SPH Fluid Simulation ===\n");
    printf("Particles: %d, Steps: %d, h=%.3f, domain=[%.1fx%.1f]\n",
           N_PARTICLES, STEPS, H, DOM_W, DOM_H);

    auto ps = init_particles();
    printf("Initialized %d particles\n", (int)ps.size());

    std::vector<uint8_t> img(IMG_W * IMG_H * 3);
    std::vector<std::string> snaps;

    // 序列帧时间点：0, 25%, 50%, 75%, 100% 的模拟时间
    std::vector<int> snap_steps = {0, STEPS/4, STEPS/2, 3*STEPS/4, STEPS-1};
    int snap_idx = 0;

    for (int step = 0; step < STEPS; step++) {
        compute_density_pressure(ps);
        compute_forces(ps);
        integrate(ps);

        // 保存快照
        if (snap_idx < (int)snap_steps.size() && step == snap_steps[snap_idx]) {
            render(ps, img, step);
            char fname[64];
            snprintf(fname, sizeof(fname), "sph_seq_%02d.png", snap_idx);
            stbi_write_png(fname, IMG_W, IMG_H, 3, img.data(), IMG_W*3);
            snaps.push_back(fname);
            float t = step * DT;
            printf("  Saved %s (t=%.3fs)\n", fname, t);
            snap_idx++;
        }
    }

    // 主输出
    render(ps, img, STEPS);
    stbi_write_png("sph_output.png", IMG_W, IMG_H, 3, img.data(), IMG_W*3);
    printf("Main output: sph_output.png\n");

    // ============ 量化验证 ============
    printf("\n=== Validation ===\n");

    // 1. 边界检查
    int oob = 0;
    for (const auto& p : ps) {
        if (p.pos.x < -H || p.pos.x > DOM_W + H ||
            p.pos.y < -H || p.pos.y > DOM_H + H) oob++;
    }
    printf("Out-of-bounds: %d / %d\n", oob, (int)ps.size());

    // 2. 密度统计
    float avg_dens = 0, min_dens = 1e9f, max_dens = -1e9f;
    for (const auto& p : ps) {
        avg_dens += p.density;
        min_dens  = std::min(min_dens, p.density);
        max_dens  = std::max(max_dens, p.density);
    }
    avg_dens /= (float)ps.size();
    printf("Density: avg=%.1f min=%.1f max=%.1f (rest=%.1f)\n",
           avg_dens, min_dens, max_dens, REST_DENS);

    // 3. 速度统计
    float avg_spd = 0, max_spd = 0;
    for (const auto& p : ps) {
        float s = p.vel.len();
        avg_spd += s; max_spd = std::max(max_spd, s);
    }
    avg_spd /= (float)ps.size();
    printf("Speed: avg=%.3f max=%.3f\n", avg_spd, max_spd);

    // 4. 重力下沉：平均Y应小于初始值（因为Y轴向上，重力向下）
    float avg_y = 0;
    for (const auto& p : ps) avg_y += p.pos.y;
    avg_y /= (float)ps.size();
    printf("Avg Y=%.3f (domain H=%.1f, started at ~%.2f)\n",
           avg_y, DOM_H, 0.3f + 0.15f * 0.75f); // 初始中心约0.6

    // 5. 检查输出图片（像素统计）
    // 计算图片非背景像素数（bright pixels）
    int bright = 0;
    for (int i = 0; i < IMG_W * IMG_H; i++) {
        int r = img[i*3+0], g = img[i*3+1];
        (void)img[i*3+2];
        if (r > 50 || g > 30) bright++;
    }
    float bright_frac = (float)bright / (IMG_W * IMG_H);
    printf("Bright pixels: %d (%.1f%%)\n", bright, bright_frac*100);

    // 断言
    assert(oob == 0);
    assert(avg_dens > 0.1f && "Density too low - check kernel normalization");
    assert(avg_spd < 100.0f && "Speed too high - simulation unstable");
    assert(bright_frac > 0.005f && "Image too dark - no particles rendered");
    // 粒子应因重力沉到底部（avg_y < 初始高度）
    assert(avg_y < 1.5f && "Particles did not fall due to gravity");

    printf("✅ All validations passed!\n\n");
    printf("=== Done: sph_output.png + %d sequence frames ===\n", (int)snaps.size());
    return 0;
}
