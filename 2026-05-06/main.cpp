/**
 * IBL Split-Sum PBR Renderer
 * Date: 2026-05-06
 *
 * Implements Image-Based Lighting (IBL) using the Split-Sum approximation:
 *   L_o ≈ ∫L_i(l) dl  *  ∫(fr * cosθ) dl
 *       ≈ prefiltered_env(r, roughness) * BRDF_LUT(n·v, roughness)
 *
 * Features:
 * - Procedural HDR environment map (gradient sky + sun)
 * - Prefiltered environment map (6 roughness mip levels)
 * - BRDF integration lookup table (GGX + Smith masking)
 * - Cook-Torrance PBR BRDF with IBL
 * - 5x5 sphere grid: metallic (0→1) × roughness (0→1)
 * - Full software rasterizer with depth buffer
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <string>
#include <cstring>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>
#include <array>

// ─────────────────────────────────────────────
//  Math primitives
// ─────────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3(float v = 0) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)       const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)       const { return {x/t, y/t, z/t}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b)  const { return x*b.x + y*b.y + z*b.z; }
    Vec3  cross(const Vec3& b)const { return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x}; }
    float length2()           const { return x*x + y*y + z*z; }
    float length()            const { return sqrtf(length2()); }
    Vec3  normalized()        const { float l = length(); return l>1e-9f ? *this/l : Vec3(0); }
    Vec3  clamp01()           const { return {std::max(0.f,std::min(1.f,x)),
                                              std::max(0.f,std::min(1.f,y)),
                                              std::max(0.f,std::min(1.f,z))}; }
};
inline Vec3 operator*(float t, const Vec3& v){ return v*t; }
inline Vec3 mix(const Vec3& a, const Vec3& b, float t){ return a*(1-t) + b*t; }
inline float clamp(float v,float lo,float hi){ return std::max(lo,std::min(hi,v)); }
inline float saturate(float v){ return clamp(v,0.f,1.f); }

// ─────────────────────────────────────────────
//  Tone-mapping & gamma
// ─────────────────────────────────────────────
inline Vec3 aces(Vec3 c){
    // ACES filmic tonemapper (component-wise)
    float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
    auto f = [&](float v)->float{
        return (v*(a*v+b))/(v*(cc*v+d)+e);
    };
    return Vec3(f(c.x),f(c.y),f(c.z)).clamp01();
}
inline float gammaEncode(float v){ return powf(clamp(v,0.f,1.f), 1.f/2.2f); }
inline Vec3  gammaEncode(Vec3 v){ return {gammaEncode(v.x),gammaEncode(v.y),gammaEncode(v.z)}; }

// ─────────────────────────────────────────────
//  Procedural HDR environment (lat-lon)
// ─────────────────────────────────────────────
const int ENV_W = 512, ENV_H = 256;
const int BRDF_SIZE = 256;
const int MIP_LEVELS = 6; // roughness 0,0.2,0.4,0.6,0.8,1.0

Vec3 sampleSky(float phi, float theta) {
    // phi   ∈ [0, 2π],  theta ∈ [0, π]  (zenith → nadir)
    Vec3 dir = {
        sinf(theta) * cosf(phi),
        cosf(theta),
        sinf(theta) * sinf(phi)
    };

    // Horizon gradient
    float h = dir.y; // -1..1
    // float t = saturate(h * 0.5f + 0.5f); // reserved

    Vec3 sky_top    = {0.05f, 0.12f, 0.35f};
    Vec3 sky_mid    = {0.35f, 0.60f, 0.90f};
    Vec3 sky_bot    = {0.60f, 0.50f, 0.30f}; // ground/horizon

    Vec3 sky;
    if (h >= 0.0f)
        sky = mix(sky_mid, sky_top, h);
    else
        sky = mix(sky_mid, sky_bot, -h);

    // Sun disc
    Vec3 sunDir = Vec3(0.3f, 0.7f, 0.5f).normalized();
    float sunDot = saturate(dir.dot(sunDir));
    float sunDisc = powf(sunDot, 512.f);
    float sunGlow = powf(sunDot, 8.f) * 0.5f;
    Vec3  sunColor = {8.0f, 6.5f, 4.0f};
    Vec3  glowColor = {2.0f, 1.5f, 0.5f};

    sky += sunColor * sunDisc + glowColor * sunGlow;
    return sky;
}

struct EnvMap {
    int w, h;
    std::vector<Vec3> data;

    EnvMap(int w, int h) : w(w), h(h), data(w*h) {}

    void build() {
        const float PI = acosf(-1.f);
        for (int j = 0; j < h; ++j)
            for (int i = 0; i < w; ++i) {
                float phi   = (i + 0.5f) / w * 2*PI;
                float theta = (j + 0.5f) / h * PI;
                data[j*w+i] = sampleSky(phi, theta);
            }
    }

    // Bilinear sample by direction
    Vec3 sample(Vec3 dir) const {
        const float PI = acosf(-1.f);
        dir = dir.normalized();
        float phi   = atan2f(dir.z, dir.x);
        if (phi < 0) phi += 2*PI;
        float theta = acosf(clamp(dir.y, -1.f, 1.f));

        float u = phi   / (2*PI) * w - 0.5f;
        float v = theta / PI     * h - 0.5f;

        int x0 = (int)floorf(u), y0 = (int)floorf(v);
        float fu = u - x0, fv = v - y0;

        auto px = [&](int xi, int yi) -> Vec3 {
            xi = ((xi % w) + w) % w;
            yi = clamp(yi, 0, h-1);
            return data[yi*w + xi];
        };

        return px(x0,y0)*(1-fu)*(1-fv)
             + px(x0+1,y0)*fu*(1-fv)
             + px(x0,y0+1)*(1-fu)*fv
             + px(x0+1,y0+1)*fu*fv;
    }
};

// ─────────────────────────────────────────────
//  GGX / Smith helpers
// ─────────────────────────────────────────────
const float PI = acosf(-1.f);

float radicalInverseVdC(uint32_t bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10f; // 0x100000000
}

Vec3 importanceSampleGGX(Vec3 Xi, float roughness, Vec3 N) {
    float a = roughness * roughness;
    float phi      = 2 * PI * Xi.x;
    float cosTheta = sqrtf((1 - Xi.y) / (1 + (a*a - 1) * Xi.y + 1e-7f));
    float sinTheta = sqrtf(1 - cosTheta*cosTheta);

    Vec3 H = { sinTheta*cosf(phi), cosTheta, sinTheta*sinf(phi) };

    // TBN
    Vec3 up = fabsf(N.y) < 0.999f ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 T = up.cross(N).normalized();
    Vec3 B = N.cross(T);

    return (T*H.x + N*H.y + B*H.z).normalized();
}

float GSchlickGGX(float NdotV, float roughness) {
    float k = roughness * roughness / 2.f;
    return NdotV / (NdotV*(1-k) + k + 1e-7f);
}

float GSmith(float NdotV, float NdotL, float roughness) {
    return GSchlickGGX(NdotV, roughness) * GSchlickGGX(NdotL, roughness);
}

// ─────────────────────────────────────────────
//  Prefiltered environment map
// ─────────────────────────────────────────────
struct PrefilteredEnv {
    // Store 6 levels, each ENV_W/2 × ENV_H/2 (simple, no real mip chain)
    static const int MIPS = MIP_LEVELS;
    std::vector<EnvMap> levels;

    void build(const EnvMap& env) {
        const int SAMPLES = 1024;
        float roughnesses[MIPS] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};

        for (int m = 0; m < MIPS; ++m) {
            float roughness = roughnesses[m];
            int w = std::max(1, ENV_W >> m);
            int h = std::max(1, ENV_H >> m);
            EnvMap level(w, h);

            for (int j = 0; j < h; ++j) {
                for (int i = 0; i < w; ++i) {
                    float phi   = (i + 0.5f) / w * 2*PI;
                    float theta = (j + 0.5f) / h * PI;
                    Vec3 N = {
                        sinf(theta)*cosf(phi),
                        cosf(theta),
                        sinf(theta)*sinf(phi)
                    };
                    N = N.normalized();
                    Vec3 V = N; // assume V ≈ N (isotropic)

                    Vec3 prefilteredColor(0.f);
                    float totalWeight = 0.f;

                    for (int s = 0; s < SAMPLES; ++s) {
                        // Hammersley sequence
                        float xi1 = (float)s / SAMPLES;
                        float xi2 = radicalInverseVdC((uint32_t)s);
                        Vec3 Xi(xi1, xi2, 0.f);

                        Vec3 H = importanceSampleGGX(Xi, roughness, N);
                        Vec3 L = (2.f * V.dot(H) * H - V).normalized();

                        float NdotL = saturate(N.dot(L));
                        if (NdotL > 0.f) {
                            prefilteredColor += env.sample(L) * NdotL;
                            totalWeight += NdotL;
                        }
                    }
                    level.data[j*w+i] = prefilteredColor / (totalWeight + 1e-9f);
                }
            }
            levels.push_back(std::move(level));
        }
    }

    Vec3 sample(Vec3 dir, float roughness) const {
        // Choose mip level by roughness
        float mipF = roughness * (MIPS - 1);
        int   mip0 = (int)floorf(mipF);
        int   mip1 = std::min(mip0+1, MIPS-1);
        float t    = mipF - mip0;
        return mix(levels[mip0].sample(dir), levels[mip1].sample(dir), t);
    }
};

// ─────────────────────────────────────────────
//  BRDF integration LUT  (NdotV, roughness) → (F_scale, F_bias)
// ─────────────────────────────────────────────
struct BRDFLut {
    int size;
    std::vector<float> r; // channel r = F_scale
    std::vector<float> g; // channel g = F_bias

    BRDFLut(int sz) : size(sz), r(sz*sz), g(sz*sz) {}

    void build() {
        const int SAMPLES = 1024;
        for (int j = 0; j < size; ++j) {
            float roughness = (j + 0.5f) / size;
            for (int i = 0; i < size; ++i) {
                float NdotV = (i + 0.5f) / size;
                Vec3 V = { sqrtf(1 - NdotV*NdotV), NdotV, 0.f };
                Vec3 N = { 0, 1, 0 };

                float A = 0, B = 0;
                for (int s = 0; s < SAMPLES; ++s) {
                    float xi1 = (float)s / SAMPLES;
                    float xi2 = radicalInverseVdC((uint32_t)s);
                    Vec3  Xi(xi1, xi2, 0.f);

                    Vec3 H = importanceSampleGGX(Xi, roughness, N);
                    Vec3 L = (2.f * V.dot(H) * H - V).normalized();

                    float NdotL = saturate(N.dot(L));
                    float NdotH = saturate(N.dot(H));
                    float VdotH = saturate(V.dot(H));

                    if (NdotL > 0.f) {
                        float G   = GSmith(NdotV, NdotL, roughness);
                        float G_vis = (G * VdotH) / (NdotH * NdotV + 1e-7f);
                        float Fc  = powf(1 - VdotH, 5.f);
                        A += (1 - Fc) * G_vis;
                        B += Fc * G_vis;
                    }
                }
                r[j*size+i] = A / SAMPLES;
                g[j*size+i] = B / SAMPLES;
            }
        }
    }

    void sampleLUT(float NdotV, float roughness, float& F_scale, float& F_bias) const {
        float u = NdotV    * (size - 1);
        float v = roughness* (size - 1);
        int x0 = clamp((int)u, 0, size-2);
        int y0 = clamp((int)v, 0, size-2);
        float fu = u - x0, fv = v - y0;

        auto bilinear = [&](const std::vector<float>& ch) {
            return ch[y0*size+x0]*(1-fu)*(1-fv)
                 + ch[y0*size+x0+1]*fu*(1-fv)
                 + ch[(y0+1)*size+x0]*(1-fu)*fv
                 + ch[(y0+1)*size+x0+1]*fu*fv;
        };
        F_scale = bilinear(r);
        F_bias  = bilinear(g);
    }
};

// ─────────────────────────────────────────────
//  IBL shading
// ─────────────────────────────────────────────
Vec3 fresnelSchlick(float cosTheta, Vec3 F0) {
    float f = powf(clamp(1.f - cosTheta, 0.f, 1.f), 5.f);
    return F0 + (Vec3(1.f) - F0) * f;
}

Vec3 fresnelSchlickRoughness(float cosTheta, Vec3 F0, float roughness) {
    float f = powf(clamp(1.f - cosTheta, 0.f, 1.f), 5.f);
    Vec3 maxV = Vec3(std::max(1.f-roughness, F0.x),
                     std::max(1.f-roughness, F0.y),
                     std::max(1.f-roughness, F0.z));
    return F0 + (maxV - F0) * f;
}

Vec3 shadeIBL(Vec3 N, Vec3 V, Vec3 albedo, float metallic, float roughness,
              const PrefilteredEnv& prefEnv, const BRDFLut& lut, const EnvMap& /*env*/) {
    Vec3 R = (2.f * N.dot(V) * N - V).normalized();
    float NdotV = saturate(N.dot(V));

    // F0 = dielectric 0.04, metal = albedo
    Vec3 F0 = mix(Vec3(0.04f), albedo, metallic);

    // Fresnel
    Vec3 F = fresnelSchlickRoughness(NdotV, F0, roughness);

    // Specular term
    Vec3 prefilteredColor = prefEnv.sample(R, roughness);
    float F_scale, F_bias;
    lut.sampleLUT(NdotV, roughness, F_scale, F_bias);
    Vec3 specular = prefilteredColor * (F * F_scale + F_bias);

    // Diffuse irradiance  (simple average of low-roughness level)
    Vec3 irradiance = prefEnv.sample(N, 1.0f) * 0.5f;  // rough approx
    Vec3 kD = (Vec3(1.f) - F) * (1.f - metallic);
    Vec3 diffuse = kD * albedo * irradiance;

    // Small direct point light for catchlight
    Vec3 lightDir = Vec3(0.3f, 0.7f, 0.5f).normalized();
    Vec3 H = (V + lightDir).normalized();
    float NdotL = saturate(N.dot(lightDir));
    float NdotH = saturate(N.dot(H));
    float VdotH = saturate(V.dot(H));

    // Cook-Torrance for direct light
    float a  = roughness * roughness;
    float a2 = a * a;
    float denom = NdotH*NdotH*(a2-1) + 1;
    float D = a2 / (PI * denom*denom + 1e-7f);
    float G = GSmith(NdotV, NdotL, roughness);
    Vec3  Fd = fresnelSchlick(VdotH, F0);
    Vec3  brdf = (D * G * Fd) / (4.f * NdotV * NdotL + 1e-7f);

    Vec3 Lo = (brdf + kD * albedo / PI) * Vec3(3.0f, 2.8f, 2.2f) * NdotL;

    return diffuse + specular + Lo;
}

// ─────────────────────────────────────────────
//  Framebuffer & depth buffer
// ─────────────────────────────────────────────
const int WIDTH  = 960;
const int HEIGHT = 540;

struct Framebuffer {
    int w, h;
    std::vector<Vec3>  color;
    std::vector<float> depth;

    Framebuffer(int w, int h) : w(w), h(h), color(w*h, Vec3(0)), depth(w*h, 1e30f) {}

    void set(int x, int y, Vec3 c, float d) {
        if (x<0||x>=w||y<0||y>=h) return;
        int idx = y*w+x;
        if (d < depth[idx]) {
            depth[idx] = d;
            color[idx] = c;
        }
    }

    void clear(Vec3 bg) {
        std::fill(color.begin(), color.end(), bg);
        std::fill(depth.begin(), depth.end(), 1e30f);
    }
};

// ─────────────────────────────────────────────
//  Sphere rasterization
// ─────────────────────────────────────────────
struct Camera {
    Vec3 pos, forward, right, up;
    float aspect, fovY;

    Camera(Vec3 pos, Vec3 target, float fovDeg, float aspect)
        : pos(pos), aspect(aspect), fovY(fovDeg * PI / 180.f) {
        forward = (target - pos).normalized();
        right   = forward.cross(Vec3(0,1,0)).normalized();
        up      = right.cross(forward).normalized();
    }

    // Project 3D point to screen [0,1]x[0,1]
    bool project(Vec3 p, float& sx, float& sy, float& depth) const {
        Vec3 d = p - pos;
        float fwd = d.dot(forward);
        if (fwd <= 0.01f) return false;
        float rr  = d.dot(right);
        float uu  = d.dot(up);

        float halfH = tanf(fovY * 0.5f);
        float halfW = halfH * aspect;

        sx = 0.5f + rr / (fwd * halfW * 2);
        sy = 0.5f - uu / (fwd * halfH * 2);
        depth = fwd;
        return sx>=0 && sx<=1 && sy>=0 && sy<=1;
    }

    Vec3 getRayDir(float sx, float sy) const {
        float halfH = tanf(fovY * 0.5f);
        float halfW = halfH * aspect;
        float rr = (sx - 0.5f) * 2 * halfW;
        float uu = -(sy - 0.5f) * 2 * halfH;
        return (forward + right*rr + up*uu).normalized();
    }
};

void drawSphere(Framebuffer& fb, const Camera& cam,
                Vec3 center, float radius,
                Vec3 albedo, float metallic, float roughness,
                const PrefilteredEnv& prefEnv, const BRDFLut& lut, const EnvMap& env) {
    // Project bounding box to screen
    float sx0, sy0, depth0;
    if (!cam.project(center, sx0, sy0, depth0)) return;

    // Approximate pixel radius
    float halfH  = tanf(cam.fovY * 0.5f);
    float pixR   = (radius / depth0) / (2 * halfH) * fb.h;
    if (pixR < 1) return;

    int cx = (int)(sx0 * fb.w);
    int cy = (int)(sy0 * fb.h);
    int r  = (int)ceil(pixR) + 1;

    for (int py = cy-r; py <= cy+r; ++py)
        for (int px = cx-r; px <= cx+r; ++px) {
            if (px<0||px>=fb.w||py<0||py>=fb.h) continue;

            // Ray from camera pixel
            float su = (px + 0.5f) / fb.w;
            float sv = (py + 0.5f) / fb.h;
            Vec3 rayDir = cam.getRayDir(su, sv);
            Vec3 oc = cam.pos - center;

            float b = oc.dot(rayDir);
            float c = oc.dot(oc) - radius*radius;
            float disc = b*b - c;
            if (disc < 0) continue;

            float t = -b - sqrtf(disc);
            if (t < 0.01f) { t = -b + sqrtf(disc); if (t < 0.01f) continue; }

            Vec3 hitP = cam.pos + rayDir * t;
            Vec3 N    = (hitP - center).normalized();
            Vec3 V    = -rayDir;

            Vec3 color = shadeIBL(N, V, albedo, metallic, roughness, prefEnv, lut, env);
            fb.set(px, py, color, t);
        }
}

// ─────────────────────────────────────────────
//  PPM writer
// ─────────────────────────────────────────────
void savePPM(const std::string& path, const Framebuffer& fb) {
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << fb.w << " " << fb.h << "\n255\n";
    for (int i = 0; i < fb.w * fb.h; ++i) {
        Vec3 c = gammaEncode(aces(fb.color[i]));
        uint8_t r = (uint8_t)(c.x * 255.99f);
        uint8_t g = (uint8_t)(c.y * 255.99f);
        uint8_t b = (uint8_t)(c.z * 255.99f);
        f.write((char*)&r, 1);
        f.write((char*)&g, 1);
        f.write((char*)&b, 1);
    }
}

// ─────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────
int main() {
    std::cout << "[IBL Split-Sum PBR Renderer]\n";
    std::cout << "Resolution: " << WIDTH << "x" << HEIGHT << "\n";

    // 1. Build environment map
    std::cout << "Building environment map..." << std::flush;
    EnvMap env(ENV_W, ENV_H);
    env.build();
    std::cout << " done\n";

    // 2. Build prefiltered env
    std::cout << "Prefiltering environment (" << MIP_LEVELS << " levels)..." << std::flush;
    PrefilteredEnv prefEnv;
    prefEnv.build(env);
    std::cout << " done\n";

    // 3. Build BRDF LUT
    std::cout << "Building BRDF LUT (" << BRDF_SIZE << "x" << BRDF_SIZE << ")..." << std::flush;
    BRDFLut lut(BRDF_SIZE);
    lut.build();
    std::cout << " done\n";

    // 4. Render
    std::cout << "Rendering 5x5 sphere grid..." << std::flush;
    Framebuffer fb(WIDTH, HEIGHT);

    // Background: sample environment at each pixel
    Camera cam(Vec3(0, 1.2f, 8.f), Vec3(0, 0, 0), 45.f, (float)WIDTH/HEIGHT);

    // Background from env
    for (int py = 0; py < HEIGHT; ++py) {
        for (int px = 0; px < WIDTH; ++px) {
            float su = (px + 0.5f) / WIDTH;
            float sv = (py + 0.5f) / HEIGHT;
            Vec3 dir = cam.getRayDir(su, sv);
            fb.color[py*WIDTH+px] = env.sample(dir) * 0.8f;
            fb.depth[py*WIDTH+px] = 1e30f;
        }
    }

    // 5×5 grid: metallic (columns 0→1) × roughness (rows 0→1)
    const int GRID = 5;
    float spacing = 1.5f;
    float offsetX = -(GRID-1)*0.5f * spacing;
    float offsetY =  (GRID-1)*0.5f * spacing;

    // Base albedo options per row
    std::array<Vec3,5> albedos = {
        Vec3(1.0f, 0.85f, 0.57f), // gold
        Vec3(0.95f, 0.93f, 0.88f), // silver
        Vec3(0.56f, 0.57f, 0.58f), // iron
        Vec3(0.97f, 0.96f, 0.91f), // aluminum
        Vec3(0.30f, 0.55f, 0.90f), // blue plastic/metal
    };

    for (int row = 0; row < GRID; ++row) {
        float roughness = (float)row / (GRID-1) * 0.95f + 0.05f;
        for (int col = 0; col < GRID; ++col) {
            float metallic = (float)col / (GRID-1);
            Vec3  albedo   = albedos[col];

            Vec3 center = Vec3(offsetX + col*spacing, offsetY - row*spacing, 0.f);
            drawSphere(fb, cam, center, 0.6f, albedo, metallic, roughness, prefEnv, lut, env);
        }
    }

    // Labels: small indicator spheres on top row
    std::cout << " done\n";

    // Save
    std::cout << "Saving ibl_output.ppm..." << std::flush;
    savePPM("ibl_output.ppm", fb);
    std::cout << " done\n";

    // Convert PPM → PNG via ImageMagick
    std::cout << "Converting to PNG..." << std::flush;
    int ret = system("convert ibl_output.ppm ibl_output.png 2>/dev/null");
    if (ret == 0)
        std::cout << " done (ibl_output.png)\n";
    else
        std::cout << " skipped (ImageMagick not found, PPM available)\n";

    std::cout << "✅ Render complete!\n";
    return 0;
}
