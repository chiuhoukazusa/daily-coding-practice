/**
 * Atmospheric Scattering Sky Renderer
 * 
 * 实现基于物理的大气散射模型：
 * - Rayleigh 散射：波长相关，负责蓝天效果
 * - Mie 散射：波长无关，负责白色光晕和太阳盘
 * - 多次散射近似（单次散射 + 天空光照）
 * - 多时段渲染：正午/日落/日出/夜晚边缘
 * 
 * 参考：
 * - Nishita et al. 1993: "Display of the Earth Taking into Account Atmospheric Scattering"
 * - Preetham et al. 1999: "A Practical Analytic Model for Daylight"
 * - Scratchapixel atmospheric scattering tutorial
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <string>

// ============================================================
// 数学工具
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float t) { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l>1e-9f ? *this/l : Vec3(0,1,0); }
};

inline Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a*(1-t) + b*t;
}

inline float clamp01(float x) { return std::max(0.f, std::min(1.f, x)); }

// ============================================================
// 大气参数
// ============================================================

// 地球大气物理参数
const float Re  = 6360e3f;   // 地球半径 (m)
const float Ra  = 6420e3f;   // 大气层顶半径 (m)
const float Hr  = 7994.f;    // Rayleigh散射标高 (m)
const float Hm  = 1200.f;    // Mie散射标高 (m)

// Rayleigh 散射系数（RGB，m^-1）
// 对应可见光波长 680nm, 550nm, 440nm
const Vec3 BetaR(5.8e-6f, 13.5e-6f, 33.1e-6f);

// Mie 散射系数（m^-1，波长无关）
const float BetaM = 21e-6f;

// Mie 散射相位函数 g 参数（前向散射强度）
const float MieG = 0.76f;

// 太阳强度（近似辐照度）
const float SunIntensity = 20.0f;

// ============================================================
// 大气散射核心算法
// ============================================================

// Rayleigh 相位函数
float phaseRayleigh(float cosTheta) {
    return 3.f / (16.f * M_PI) * (1.f + cosTheta * cosTheta);
}

// Mie 相位函数（Henyey-Greenstein）
float phaseMie(float cosTheta, float g) {
    float g2 = g * g;
    float denom = 1.f + g2 - 2.f * g * cosTheta;
    return (1.f - g2) / (4.f * M_PI * denom * std::sqrt(denom) + 1e-10f);
}

// 射线与球体相交，返回 (t_near, t_far)
// origin: 射线起点, dir: 射线方向(单位向量), R: 球半径
bool raySphereIntersect(const Vec3& origin, const Vec3& dir, float R, float& t0, float& t1) {
    // ||origin + t*dir||^2 = R^2
    float a = dir.dot(dir);         // = 1 if dir is normalized
    float b = 2.f * origin.dot(dir);
    float c = origin.dot(origin) - R * R;
    float disc = b*b - 4*a*c;
    if (disc < 0) return false;
    float sqrtDisc = std::sqrt(disc);
    t0 = (-b - sqrtDisc) / (2*a);
    t1 = (-b + sqrtDisc) / (2*a);
    return true;
}

// 对一条视线计算大气散射颜色
// origin: 相机位置（地面上方1米）
// dir: 视线方向
// sunDir: 太阳方向（单位向量）
Vec3 computeSkyColor(const Vec3& origin, const Vec3& dir, const Vec3& sunDir) {
    // 与大气层顶的交点
    float t0, t1;
    if (!raySphereIntersect(origin, dir, Ra, t0, t1)) {
        return Vec3(0, 0, 0); // 不应发生
    }
    // 若从地面看，t0可能为负（相机在大气内），使用max(0, t0)
    float tStart = std::max(0.f, t0);
    float tEnd   = t1;

    // 光线可能撞地
    float tGround0, tGround1;
    if (raySphereIntersect(origin, dir, Re, tGround0, tGround1) && tGround1 > 0) {
        // 如果t0>0说明射线从地面内出来，tEnd取地面交点
        if (tGround0 > 0) tEnd = tGround0;
        // 否则(相机在大气内，向下看)不渲染
    }

    if (tEnd <= tStart) return Vec3(0,0,0);

    // 主射线积分步数
    const int numSamples = 16;
    float segLen = (tEnd - tStart) / numSamples;

    // 累积光学深度（透射率）
    float optDepthR = 0.f; // Rayleigh
    float optDepthM = 0.f; // Mie

    Vec3 sumR(0,0,0), sumM(0,0,0);

    for (int i = 0; i < numSamples; i++) {
        float tMid = tStart + segLen * (i + 0.5f);
        Vec3 pos = origin + dir * tMid;

        float height = pos.length() - Re;
        if (height < 0) height = 0;

        float hr = std::exp(-height / Hr);
        float hm = std::exp(-height / Hm);

        float dOptDepthR = hr * segLen;
        float dOptDepthM = hm * segLen;

        optDepthR += dOptDepthR;
        optDepthM += dOptDepthM;

        // 对当前样本点，向太阳方向积分（计算太阳光透射率）
        float t0s, t1s;
        if (!raySphereIntersect(pos, sunDir, Ra, t0s, t1s)) continue;
        float tSunEnd = t1s;

        // 检查是否撞地（太阳被地球遮挡）
        float tg0, tg1;
        bool sunBlocked = false;
        if (raySphereIntersect(pos, sunDir, Re, tg0, tg1) && tg1 > 0) {
            if (tg0 > 0) sunBlocked = true;
        }
        if (sunBlocked) continue; // 当前点被地球遮挡太阳

        const int numSamplesSun = 8;
        float segLenSun = tSunEnd / numSamplesSun;
        float optDepthSunR = 0.f, optDepthSunM = 0.f;

        for (int j = 0; j < numSamplesSun; j++) {
            float tSunMid = segLenSun * (j + 0.5f);
            Vec3 posSun = pos + sunDir * tSunMid;
            float hSun = posSun.length() - Re;
            if (hSun < 0) hSun = 0;
            optDepthSunR += std::exp(-hSun / Hr) * segLenSun;
            optDepthSunM += std::exp(-hSun / Hm) * segLenSun;
        }

        // 透射率 T = exp(-(BetaR*optDepth_R + BetaM*optDepth_M))
        Vec3 tau = (BetaR * (optDepthR + optDepthSunR) +
                    Vec3(BetaM, BetaM, BetaM) * 1.1f * (optDepthM + optDepthSunM));
        Vec3 attn(std::exp(-tau.x), std::exp(-tau.y), std::exp(-tau.z));

        sumR += attn * hr * dOptDepthR;
        sumM += attn * hm * dOptDepthM;
    }

    float cosTheta = dir.dot(sunDir);
    float phR = phaseRayleigh(cosTheta);
    float phM  = phaseMie(cosTheta, MieG);

    Vec3 scatter = (sumR * BetaR * phR + sumM * Vec3(BetaM,BetaM,BetaM) * phM) * SunIntensity;

    return scatter;
}

// ============================================================
// 色调映射
// ============================================================

// ACES Filmic tone mapping
Vec3 acesFilmic(Vec3 x) {
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    x.x = (x.x*(a*x.x+b)) / (x.x*(c*x.x+d)+e);
    x.y = (x.y*(a*x.y+b)) / (x.y*(c*x.y+d)+e);
    x.z = (x.z*(a*x.z+b)) / (x.z*(c*x.z+d)+e);
    x.x = clamp01(x.x);
    x.y = clamp01(x.y);
    x.z = clamp01(x.z);
    return x;
}

// Gamma correction
Vec3 gammaCorrect(Vec3 c, float gamma=2.2f) {
    float inv = 1.f / gamma;
    return Vec3(std::pow(c.x, inv), std::pow(c.y, inv), std::pow(c.z, inv));
}

// ============================================================
// 渲染参数 & 输出
// ============================================================

const int IMG_W = 800;
const int IMG_H = 400;

// 生成天空图像（equirectangular 投影）
// 上半球：天空，下半球：地面（简单填充地面颜色）
void renderSky(const Vec3& sunDir, const std::string& filename) {
    std::vector<unsigned char> img(IMG_W * IMG_H * 3);

    // 相机在地面上方 1m
    Vec3 origin(0, Re + 1.f, 0);

    for (int row = 0; row < IMG_H; row++) {
        for (int col = 0; col < IMG_W; col++) {
            // equirectangular: u in [0,1], v in [0,1]
            // v=0 -> 天顶(y=1), v=1 -> 地平线(y=0), v=0.5 -> 水平方向
            // 渲染上半球 + 紧贴地平线以下
            float u = (col + 0.5f) / IMG_W;       // azimuth [0, 2pi]
            float v = (row + 0.5f) / IMG_H;        // elevation [+90, -90]

            float phi   = u * 2.f * M_PI;          // azimuth
            float theta = (0.5f - v) * M_PI;       // elevation [-pi/2, pi/2]

            float cosEl = std::cos(theta);
            float sinEl = std::sin(theta);

            // 视线方向（右手坐标系，y向上）
            Vec3 dir(cosEl * std::cos(phi), sinEl, cosEl * std::sin(phi));
            dir = dir.normalized();

            Vec3 color;
            if (dir.y < -0.01f) {
                // 地面：简单地面色（棕绿色）
                float horizon = clamp01(1.f - std::abs(dir.y) * 10.f);
                // 混合大气颜色（地平线）
                Vec3 horizonColor(0.5f, 0.4f, 0.25f);
                Vec3 groundColor(0.1f, 0.12f, 0.05f);
                color = mix(groundColor, horizonColor, horizon * 0.5f);
                // 简单 gamma
                color = gammaCorrect(color);
            } else {
                // 天空：大气散射
                color = computeSkyColor(origin, dir, sunDir);
                color = acesFilmic(color);
                color = gammaCorrect(color);
            }

            int idx = (row * IMG_W + col) * 3;
            img[idx+0] = static_cast<unsigned char>(clamp01(color.x) * 255);
            img[idx+1] = static_cast<unsigned char>(clamp01(color.y) * 255);
            img[idx+2] = static_cast<unsigned char>(clamp01(color.z) * 255);
        }
    }

    stbi_write_png(filename.c_str(), IMG_W, IMG_H, 3, img.data(), IMG_W * 3);
    printf("  Saved: %s\n", filename.c_str());
}

// ============================================================
// 渲染多时段拼合最终图
// ============================================================

// 渲染4个时段，拼成 2x2 网格
void renderMultiTime() {
    const int OUT_W = IMG_W * 2;
    const int OUT_H = IMG_H * 2;
    std::vector<unsigned char> finalImg(OUT_W * OUT_H * 3, 0);

    // 临时 buffer
    std::vector<unsigned char> buf(IMG_W * IMG_H * 3);

    // 太阳位置（sunElevationDeg: 仰角，0=地平线，90=天顶）
    struct TimeSlot {
        float sunElevDeg;
        float sunAzimDeg;
        const char* label;
        int col; // 网格列 0或1
        int row; // 网格行 0或1
    };

    TimeSlot slots[] = {
        { 60.f,   0.f, "Noon",    0, 0 },   // 正午
        {  5.f,  45.f, "Sunset",  1, 0 },   // 日落
        {  2.f, 135.f, "Dawn",    0, 1 },   // 黎明
        { 30.f,  90.f, "Morning", 1, 1 },   // 早晨
    };

    Vec3 origin(0, Re + 1.f, 0);

    for (auto& slot : slots) {
        printf("Rendering: %s (elev=%.1f deg)...\n", slot.label, slot.sunElevDeg);

        float elevRad  = slot.sunElevDeg * M_PI / 180.f;
        float azimRad  = slot.sunAzimDeg * M_PI / 180.f;
        Vec3 sunDir(
            std::cos(elevRad) * std::cos(azimRad),
            std::sin(elevRad),
            std::cos(elevRad) * std::sin(azimRad)
        );
        sunDir = sunDir.normalized();

        // 填充 buf
        for (int row = 0; row < IMG_H; row++) {
            for (int col = 0; col < IMG_W; col++) {
                float u = (col + 0.5f) / IMG_W;
                float v = (row + 0.5f) / IMG_H;
                float phi   = u * 2.f * M_PI;
                float theta = (0.5f - v) * M_PI;

                float cosEl = std::cos(theta);
                float sinEl = std::sin(theta);
                Vec3 dir(cosEl * std::cos(phi), sinEl, cosEl * std::sin(phi));
                dir = dir.normalized();

                Vec3 color;
                if (dir.y < -0.01f) {
                    float horizon = clamp01(1.f - std::abs(dir.y) * 10.f);
                    Vec3 horizonColor(0.5f, 0.4f, 0.25f);
                    Vec3 groundColor(0.1f, 0.12f, 0.05f);
                    color = mix(groundColor, horizonColor, horizon * 0.5f);
                    color = gammaCorrect(color);
                } else {
                    color = computeSkyColor(origin, dir, sunDir);
                    color = acesFilmic(color);
                    color = gammaCorrect(color);
                }

                int idx = (row * IMG_W + col) * 3;
                buf[idx+0] = static_cast<unsigned char>(clamp01(color.x) * 255);
                buf[idx+1] = static_cast<unsigned char>(clamp01(color.y) * 255);
                buf[idx+2] = static_cast<unsigned char>(clamp01(color.z) * 255);
            }
        }

        // 拷贝到 finalImg 的对应网格
        int offX = slot.col * IMG_W;
        int offY = slot.row * IMG_H;
        for (int row = 0; row < IMG_H; row++) {
            int srcOff  = row * IMG_W * 3;
            int dstOff  = (offY + row) * OUT_W * 3 + offX * 3;
            std::memcpy(finalImg.data() + dstOff, buf.data() + srcOff, IMG_W * 3);
        }
    }

    // 写出最终图
    stbi_write_png("atmospheric_scattering_output.png", OUT_W, OUT_H, 3, finalImg.data(), OUT_W * 3);
    printf("Final output: atmospheric_scattering_output.png (%dx%d)\n", OUT_W, OUT_H);
}

// ============================================================
// main
// ============================================================

int main() {
    printf("=== Atmospheric Scattering Sky Renderer ===\n");
    printf("Resolution: %dx%d (2x2 multi-time grid)\n", IMG_W*2, IMG_H*2);
    printf("Model: Rayleigh + Mie (single scattering)\n\n");

    renderMultiTime();

    printf("\n✅ Done!\n");
    return 0;
}
