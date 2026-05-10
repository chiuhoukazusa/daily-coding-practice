/**
 * ACES Filmic Tone Mapping & HDR Rendering
 * 
 * 实现完整的HDR渲染管线，演示不同色调映射算法的效果对比：
 * - Linear（无映射，裁剪）
 * - Reinhard (全局/局部)
 * - Uncharted 2 Filmic
 * - ACES Filmic
 * 
 * 场景：多个强光源照射的材质球阵列，演示高动态范围光照
 * 输出：aces_hdr_output.png（4x2 对比图）
 * 
 * 编译：g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <array>
#include <random>
#include <cstdint>
#include <functional>

using namespace std;

// ============================================================
// Math Types
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float v) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator/(const Vec3& o) const { return {x/o.x, y/o.y, z/o.z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float t) { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float len() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 norm() const { float l = len(); return (l > 1e-8f) ? *this / l : Vec3(0,1,0); }
    Vec3 clamp01() const { return {max(0.f,min(1.f,x)), max(0.f,min(1.f,y)), max(0.f,min(1.f,z))}; }
    Vec3 reflect(const Vec3& n) const { return *this - n * (2.f * dot(n)); }
    static Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
};
Vec3 operator*(float t, const Vec3& v) { return v * t; }

struct Ray {
    Vec3 o, d;
    Ray(Vec3 o, Vec3 d) : o(o), d(d.norm()) {}
    Vec3 at(float t) const { return o + d * t; }
};

// ============================================================
// Tone Mapping Operators
// ============================================================
namespace ToneMap {

// 线性裁剪（无映射）
Vec3 linear(const Vec3& hdr) {
    return hdr.clamp01();
}

// Reinhard全局
Vec3 reinhard(const Vec3& hdr) {
    return Vec3(hdr.x/(1+hdr.x), hdr.y/(1+hdr.y), hdr.z/(1+hdr.z));
}

// Reinhard + 亮度保留版
Vec3 reinhardLuminance(const Vec3& hdr) {
    float lum = hdr.x*0.2126f + hdr.y*0.7152f + hdr.z*0.0722f;
    float lumOut = lum / (1.f + lum);
    float scale = (lum > 1e-6f) ? lumOut / lum : 1.f;
    return Vec3(hdr.x*scale, hdr.y*scale, hdr.z*scale).clamp01();
}

// Uncharted 2 Filmic (John Hable)
static Vec3 uncharted2Partial(Vec3 x) {
    const float A = 0.15f, B = 0.50f, C = 0.10f;
    const float D = 0.20f, E = 0.02f, F = 0.30f;
    Vec3 num = x*(x*A + C*B) + Vec3(D*E);
    Vec3 den = x*(x*A + B) + Vec3(D*F);
    return Vec3(num.x/den.x, num.y/den.y, num.z/den.z) - Vec3(E/F);
}
Vec3 uncharted2(const Vec3& hdr) {
    float exposure = 2.0f;
    Vec3 curr = uncharted2Partial(hdr * exposure);
    Vec3 W = Vec3(11.2f);
    Vec3 wp = uncharted2Partial(W);
    Vec3 whiteScale = Vec3(1.f/wp.x, 1.f/wp.y, 1.f/wp.z);
    Vec3 result = Vec3(curr.x*whiteScale.x, curr.y*whiteScale.y, curr.z*whiteScale.z);
    return result.clamp01();
}

// ACES Filmic (Narkowicz approximation)
Vec3 acesNarkowicz(const Vec3& hdr) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    auto f = [&](float x) -> float {
        return max(0.f, min(1.f, (x*(a*x+b)) / (x*(c*x+d)+e)));
    };
    return Vec3(f(hdr.x), f(hdr.y), f(hdr.z));
}

// ACES RRT+ODT (Hill/Langlands matrix version)
Vec3 acesHill(Vec3 v) {
    // Input transform (RRT)
    // 3x3 matrix multiply helper
    auto m33 = [](Vec3 r0, Vec3 r1, Vec3 r2, Vec3 v) -> Vec3 {
        return Vec3(r0.dot(v), r1.dot(v), r2.dot(v));
    };
    v = m33(
        Vec3(0.59719f, 0.35458f, 0.04823f),
        Vec3(0.07600f, 0.90834f, 0.01566f),
        Vec3(0.02840f, 0.13383f, 0.83777f),
        v
    );
    // RRT and ODT fit
    Vec3 a = v + Vec3(0.0245786f) - Vec3(0.000090537f) / (v + Vec3(0.0245786f)); // workaround
    // Actually use proper fit:
    auto rrt = [](Vec3 x) -> Vec3 {
        Vec3 num = x * (x + Vec3(0.0245786f)) - Vec3(0.000090537f);
        Vec3 den = x * (x * Vec3(0.983729f) + Vec3(0.4329510f)) + Vec3(0.238081f);
        return Vec3(num.x/den.x, num.y/den.y, num.z/den.z);
    };
    v = rrt(v);
    (void)a;
    // Output transform (ODT)
    v = m33(
        Vec3(1.60475f, -0.53108f, -0.07367f),
        Vec3(-0.10208f, 1.10813f, -0.00605f),
        Vec3(-0.00327f, -0.07276f, 1.07602f),
        v
    );
    return v.clamp01();
}

// Lottes 2016 filmic
Vec3 lottes(const Vec3& hdr) {
    const float a = 1.6f, d = 0.977f;
    const float hdrMax = 8.f, midIn = 0.18f, midOut = 0.267f;
    const float b = (-powf(midIn, a) + powf(hdrMax, a) * midOut) /
                    ((powf(hdrMax, a*d) - powf(midIn, a*d)) * midOut);
    const float c = (powf(hdrMax, a*d) * powf(midIn, a) - powf(hdrMax, a) * powf(midIn, a*d) * midOut) /
                    ((powf(hdrMax, a*d) - powf(midIn, a*d)) * midOut);
    auto f = [&](float x) -> float {
        return powf(x, a) / (powf(x, a*d) * b + c);
    };
    return Vec3(f(hdr.x), f(hdr.y), f(hdr.z)).clamp01();
}

// Gamma correction (sRGB approximate)
Vec3 gamma(const Vec3& linear) {
    auto srgb = [](float l) -> float {
        if (l <= 0.0031308f) return 12.92f * l;
        return 1.055f * powf(l, 1.f/2.4f) - 0.055f;
    };
    return Vec3(srgb(max(0.f,linear.x)), srgb(max(0.f,linear.y)), srgb(max(0.f,linear.z)));
}

} // namespace ToneMap

// ============================================================
// Scene: Spheres with PBR-like shading
// ============================================================
struct Sphere {
    Vec3 center;
    float radius;
    Vec3 albedo;
    float roughness;
    float metallic;
    float emissive;
};

struct Light {
    Vec3 pos;
    Vec3 color;  // HDR color, can be >> 1
    float radius;
};

struct HitInfo {
    float t;
    Vec3 pos, normal;
    int sphereIdx;
};

static const vector<Sphere> SPHERES = {
    // Central large sphere (metal)
    {{0, 0, 0}, 0.9f, {0.8f, 0.6f, 0.2f}, 0.1f, 1.0f, 0},
    // Left sphere (rough dielectric)
    {{-2.2f, 0, 0}, 0.9f, {0.9f, 0.2f, 0.1f}, 0.8f, 0.0f, 0},
    // Right sphere (smooth dielectric)
    {{2.2f, 0, 0}, 0.9f, {0.1f, 0.4f, 0.9f}, 0.05f, 0.0f, 0},
    // Front-left (rough metal)
    {{-1.1f, 0, 1.6f}, 0.9f, {0.5f, 0.5f, 0.5f}, 0.5f, 1.0f, 0},
    // Front-right (emissive ball)
    {{1.1f, 0, 1.6f}, 0.9f, {1.0f, 0.8f, 0.3f}, 0.3f, 0.0f, 4.0f},
    // Back-left
    {{-1.1f, 0, -1.6f}, 0.9f, {0.2f, 0.8f, 0.3f}, 0.3f, 0.0f, 0},
    // Back-right (mirror)
    {{1.1f, 0, -1.6f}, 0.9f, {0.95f, 0.95f, 0.95f}, 0.02f, 1.0f, 0},
    // Ground plane sphere (very large)
    {{0, -101.0f, 0}, 100.0f, {0.4f, 0.4f, 0.4f}, 0.9f, 0.0f, 0},
};

static const vector<Light> LIGHTS = {
    // Main key light (very bright, warm)
    {{3, 6, 4}, {30.0f, 25.0f, 18.0f}, 0.5f},
    // Fill light (cool)
    {{-4, 3, 2}, {8.0f, 10.0f, 15.0f}, 0.3f},
    // Rim light (back)
    {{0, 4, -5}, {15.0f, 12.0f, 20.0f}, 0.3f},
    // Small hot light (creates overexposure)
    {{1.5f, 2.5f, 1.5f}, {80.0f, 60.0f, 10.0f}, 0.2f},
    // Blue accent
    {{-2.5f, 1.5f, 2.0f}, {5.0f, 8.0f, 40.0f}, 0.2f},
};

bool intersectSphere(const Ray& ray, const Sphere& s, float tMin, float tMax, float& tOut) {
    Vec3 oc = ray.o - s.center;
    float a = ray.d.dot(ray.d);
    float hb = oc.dot(ray.d);
    float c = oc.dot(oc) - s.radius * s.radius;
    float disc = hb*hb - a*c;
    if (disc < 0) return false;
    float sqd = sqrtf(disc);
    float t = (-hb - sqd) / a;
    if (t < tMin || t > tMax) {
        t = (-hb + sqd) / a;
        if (t < tMin || t > tMax) return false;
    }
    tOut = t;
    return true;
}

bool traceScene(const Ray& ray, float tMin, float tMax, HitInfo& hit) {
    float closest = tMax;
    bool found = false;
    for (int i = 0; i < (int)SPHERES.size(); i++) {
        float t;
        if (intersectSphere(ray, SPHERES[i], tMin, closest, t)) {
            closest = t;
            hit.t = t;
            hit.pos = ray.at(t);
            hit.normal = (hit.pos - SPHERES[i].center).norm();
            hit.sphereIdx = i;
            found = true;
        }
    }
    return found;
}

bool shadowTest(const Vec3& pos, const Vec3& lightPos) {
    Vec3 dir = lightPos - pos;
    float dist = dir.len();
    Ray shadowRay(pos + dir.norm() * 1e-3f, dir);
    HitInfo hit;
    if (traceScene(shadowRay, 1e-3f, dist - 1e-2f, hit)) return true;
    return false;
}

// Simple GGX NDF for specular
float ggxNDF(float NoH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float denom = NoH * NoH * (a2 - 1.f) + 1.f;
    return a2 / (3.14159265f * denom * denom + 1e-7f);
}

float schlickFresnel(float cosTheta, float F0) {
    return F0 + (1.f - F0) * powf(1.f - max(0.f, cosTheta), 5.f);
}

// Shade a hit point (returns HDR color)
Vec3 shadePoint(const HitInfo& hit, const Vec3& viewDir) {
    const Sphere& sph = SPHERES[hit.sphereIdx];
    Vec3 N = hit.normal;
    Vec3 V = (viewDir * -1.f).norm();
    
    // Emissive contribution
    Vec3 radiance = sph.albedo * sph.emissive;
    
    // Ambient (very small)
    Vec3 ambient = sph.albedo * 0.03f;
    radiance += ambient;
    
    for (const auto& light : LIGHTS) {
        Vec3 L = (light.pos - hit.pos);
        float dist = L.len();
        L = L.norm();
        
        // Attenuation
        float attenuation = 1.f / (dist * dist + 0.1f);
        Vec3 lightColor = light.color * attenuation * 10.f;
        
        float NoL = max(0.f, N.dot(L));
        if (NoL < 1e-5f) continue;
        
        // Shadow
        if (shadowTest(hit.pos, light.pos)) continue;
        
        Vec3 H = (V + L).norm();
        float NoH = max(0.f, N.dot(H));
        float NoV = max(0.f, N.dot(V));
        float LoH = max(0.f, L.dot(H));
        
        // Diffuse (Lambertian)
        float kD = (1.f - sph.metallic);
        Vec3 diffuse = sph.albedo * kD * (1.f / 3.14159265f);
        
        // Specular (GGX)
        float D = ggxNDF(NoH, max(0.01f, sph.roughness));
        float F0val = 0.04f + sph.metallic * (1.f - 0.04f);
        float F = schlickFresnel(LoH, F0val);
        // Simplified geometry term
        float G = min(1.f, min(2.f*NoH*NoV/max(1e-7f, LoH+0.001f), 2.f*NoH*NoL/max(1e-7f, LoH+0.001f)));
        float denom = 4.f * NoV * NoL + 1e-7f;
        Vec3 specularColor = Vec3(sph.metallic) * sph.albedo + Vec3(1.f - sph.metallic) * Vec3(1.f);
        Vec3 specular = specularColor * (D * F * G / denom);
        
        radiance += (diffuse + specular) * lightColor * NoL;
    }
    
    return radiance;
}

// ============================================================
// Camera
// ============================================================
struct Camera {
    Vec3 origin, lowerLeft, horizontal, vertical;
    
    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, float fovY, float aspect) {
        float theta = fovY * 3.14159265f / 180.f;
        float h = tanf(theta * 0.5f);
        float vH = 2.f * h;
        float vW = aspect * vH;
        
        Vec3 w = (lookFrom - lookAt).norm();
        Vec3 u = up.dot(w) > 0.9999f ? Vec3(1,0,0) : (up - w * up.dot(w)).norm();
        Vec3 v = w.reflect(u).norm(); // Actually: v = cross(w, u) but we avoid cross
        // Proper cross product via component formula
        auto cross = [](Vec3 a, Vec3 b) -> Vec3 {
            return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
        };
        u = cross(up, w).norm();
        v = cross(w, u);
        
        origin = lookFrom;
        horizontal = u * vW;
        vertical = v * vH;
        lowerLeft = origin - horizontal*0.5f - vertical*0.5f - w;
    }
    
    Ray getRay(float s, float t) const {
        Vec3 dir = lowerLeft + horizontal*s + vertical*t - origin;
        return Ray(origin, dir);
    }
};

// ============================================================
// Render one tile (HDR)
// ============================================================
Vec3 skyColor(const Vec3& dir) {
    float t = 0.5f * (dir.y + 1.0f);
    Vec3 sky = Vec3(1,1,1) * (1-t) + Vec3(0.3f, 0.5f, 0.9f) * t;
    // Add sun glow
    Vec3 sunDir = Vec3(0.6f, 0.8f, 0.4f).norm();
    float sunDot = max(0.f, dir.dot(sunDir));
    sky += Vec3(5.0f, 3.0f, 0.5f) * powf(sunDot, 64.f);
    sky += Vec3(10.0f, 6.0f, 1.0f) * powf(sunDot, 256.f);
    return sky;
}

Vec3 renderPixel(float u, float v, const Camera& cam, int spp) {
    Vec3 accum;
    for (int s = 0; s < spp; s++) {
        float ju = u + (s % 2) * 0.5f / 800.f;
        float jv = v + (s / 2) * 0.5f / 600.f;
        Ray ray = cam.getRay(ju, jv);
        
        HitInfo hit;
        if (traceScene(ray, 1e-3f, 1e9f, hit)) {
            accum += shadePoint(hit, ray.d);
        } else {
            accum += skyColor(ray.d);
        }
    }
    return accum / (float)spp;
}

// ============================================================
// Write label text (simple bitmap font, 5x7)
// ============================================================
// Minimal 5x7 ASCII bitmaps for digits and letters we need
static const uint8_t FONT5X7[][7] = {
    // A-Z (0..25)
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x00}, // A
    {0x1E,0x11,0x11,0x1E,0x11,0x1E,0x00}, // B
    {0x0E,0x11,0x10,0x10,0x11,0x0E,0x00}, // C
    {0x1E,0x09,0x09,0x09,0x09,0x1E,0x00}, // D
    {0x1F,0x10,0x10,0x1E,0x10,0x1F,0x00}, // E
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x00}, // F
    {0x0E,0x11,0x10,0x17,0x11,0x0E,0x00}, // G
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x00}, // H
    {0x0E,0x04,0x04,0x04,0x04,0x0E,0x00}, // I
    {0x01,0x01,0x01,0x01,0x11,0x0E,0x00}, // J
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, // K - 10
    {0x10,0x10,0x10,0x10,0x10,0x1F,0x00}, // L
    {0x11,0x1B,0x15,0x11,0x11,0x11,0x00}, // M
    {0x11,0x19,0x15,0x13,0x11,0x11,0x00}, // N
    {0x0E,0x11,0x11,0x11,0x11,0x0E,0x00}, // O
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x00}, // P
    {0x0E,0x11,0x11,0x15,0x12,0x0D,0x00}, // Q
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, // R
    {0x0F,0x10,0x10,0x0E,0x01,0x1E,0x00}, // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x00}, // T
    {0x11,0x11,0x11,0x11,0x11,0x0E,0x00}, // U
    {0x11,0x11,0x11,0x11,0x0A,0x04,0x00}, // V
    {0x11,0x11,0x15,0x15,0x1B,0x11,0x00}, // W
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // X
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x00}, // Y
    {0x1F,0x01,0x02,0x04,0x08,0x1F,0x00}, // Z
    // 0-9 (26..35)
    {0x0E,0x11,0x13,0x15,0x19,0x0E,0x00}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x0E,0x00}, // 1
    {0x0E,0x11,0x01,0x0E,0x10,0x1F,0x00}, // 2
    {0x1F,0x01,0x06,0x01,0x11,0x0E,0x00}, // 3
    {0x02,0x06,0x0A,0x1F,0x02,0x02,0x00}, // 4
    {0x1F,0x10,0x1E,0x01,0x11,0x0E,0x00}, // 5
    {0x0E,0x10,0x1E,0x11,0x11,0x0E,0x00}, // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x00}, // 7
    {0x0E,0x11,0x0E,0x11,0x11,0x0E,0x00}, // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x0E,0x00}, // 9
    // space (36), hyphen(37), slash (38), dot (39), 2 (40)
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // -
    {0x01,0x02,0x04,0x08,0x10,0x00,0x00}, // /
    {0x00,0x00,0x00,0x00,0x04,0x04,0x00}, // .
};

int charIndex(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a'; // map to uppercase
    if (c >= '0' && c <= '9') return 26 + (c - '0');
    if (c == ' ') return 36;
    if (c == '-') return 37;
    if (c == '/') return 38;
    if (c == '.') return 39;
    return 36; // default space
}

void drawText(vector<uint8_t>& img, int W, int H, int x, int y, const string& text, Vec3 color) {
    int r = (int)(color.x * 255);
    int g = (int)(color.y * 255);
    int b = (int)(color.z * 255);
    int cx = x;
    for (char c : text) {
        int idx = charIndex(c);
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if (FONT5X7[idx][row] & (0x10 >> col)) {
                    int px = cx + col;
                    int py = y + row;
                    if (px >= 0 && px < W && py >= 0 && py < H) {
                        int i = (py * W + px) * 3;
                        img[i] = r; img[i+1] = g; img[i+2] = b;
                    }
                }
            }
        }
        cx += 6;
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    // Tile size per panel
    const int TW = 400, TH = 300;
    // 4x2 layout: 4 columns, 2 rows
    // Row 1: No tonemapping / Reinhard / Reinhard Lum / Uncharted2
    // Row 2: ACES Narkowicz / ACES Hill / Lottes / Comparison (center crop)
    const int COLS = 4, ROWS = 2;
    const int W = TW * COLS, H = TH * ROWS;
    
    vector<uint8_t> image(W * H * 3, 0);
    
    // Camera
    Camera cam(
        Vec3(0, 1.5f, 6.0f),
        Vec3(0, 0, 0),
        Vec3(0, 1, 0),
        45.f,
        (float)TW / TH
    );
    
    const int SPP = 4;
    
    // First render HDR buffer for all tiles
    printf("Rendering HDR scene (%dx%d per tile, %d SPP)...\n", TW, TH, SPP);
    
    vector<Vec3> hdrBuf(TW * TH);
    for (int y = 0; y < TH; y++) {
        for (int x = 0; x < TW; x++) {
            float u = (x + 0.5f) / TW;
            float v = (TH - 1 - y + 0.5f) / TH;
            hdrBuf[y * TW + x] = renderPixel(u, v, cam, SPP);
        }
        if (y % 50 == 0) {
            printf("  row %d/%d\n", y, TH);
            fflush(stdout);
        }
    }
    printf("HDR render done. Applying tone mapping...\n");
    
    // Tone map names
    struct TmEntry {
        string name;
        std::function<Vec3(const Vec3&)> fn;
    };
    
    vector<TmEntry> tms = {
        {"LINEAR", ToneMap::linear},
        {"REINHARD", ToneMap::reinhard},
        {"REINH-LUM", ToneMap::reinhardLuminance},
        {"UNCHARTED2", ToneMap::uncharted2},
        {"ACES-NARK", ToneMap::acesNarkowicz},
        {"ACES-HILL", ToneMap::acesHill},
        {"LOTTES", ToneMap::lottes},
        {"ACES-COMP", ToneMap::acesNarkowicz},  // duplicate for last slot
    };
    
    // For each tile, apply tone mapping and blit
    for (int tile = 0; tile < COLS * ROWS; tile++) {
        int tc = tile % COLS;
        int tr = tile / COLS;
        int ox = tc * TW;
        int oy = tr * TH;
        
        auto& tm = tms[tile];
        
        for (int y = 0; y < TH; y++) {
            for (int x = 0; x < TW; x++) {
                Vec3 hdr = hdrBuf[y * TW + x];
                // Apply exposure (1.0 for most, different for last)
                float exposure = 1.0f;
                if (tile == 7) exposure = 0.5f; // underexposed comparison
                hdr = hdr * exposure;
                
                Vec3 ldr = tm.fn(hdr);
                ldr = ToneMap::gamma(ldr);
                ldr = ldr.clamp01();
                
                int px = ox + x;
                int py = oy + y;
                int i = (py * W + px) * 3;
                image[i]   = (uint8_t)(ldr.x * 255.f + 0.5f);
                image[i+1] = (uint8_t)(ldr.y * 255.f + 0.5f);
                image[i+2] = (uint8_t)(ldr.z * 255.f + 0.5f);
            }
        }
        
        // Draw panel label
        drawText(image, W, H, ox + 5, oy + 5, tm.name, {1,1,0});
        
        // Draw separator lines
        // Right border
        if (tc < COLS - 1) {
            for (int y2 = oy; y2 < oy + TH; y2++) {
                int i = (y2 * W + ox + TW - 1) * 3;
                image[i] = image[i+1] = image[i+2] = 180;
            }
        }
        // Bottom border
        if (tr < ROWS - 1) {
            for (int x2 = ox; x2 < ox + TW; x2++) {
                int i = ((oy + TH - 1) * W + x2) * 3;
                image[i] = image[i+1] = image[i+2] = 180;
            }
        }
    }
    
    // Special: draw a bright comparison bar on the last tile showing luminance curve overlay
    // Just add a small note to last tile
    drawText(image, W, H, 3*TW + 5, 1*TH + 15, "EXP 0.5", {1.0f, 0.5f, 0.2f});
    
    // Title bar at very top
    drawText(image, W, H, W/2 - 120, 0, "TONE MAPPING COMPARISON - HDR RENDERING", {1,1,1});
    
    // Save
    const char* outFile = "aces_hdr_output.png";
    int res = stbi_write_png(outFile, W, H, 3, image.data(), W * 3);
    if (res == 0) {
        printf("❌ Failed to write PNG\n");
        return 1;
    }
    
    printf("✅ Saved: %s (%dx%d, %.1f KB)\n", outFile, W, H,
           (W*H*3) / 1024.f);
    return 0;
}
