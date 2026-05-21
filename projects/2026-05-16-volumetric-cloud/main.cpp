/**
 * Volumetric Cloud Ray Marching Renderer
 * 2026-05-16
 *
 * 技术点：
 * - 程序化 Perlin + FBM 噪声生成云体密度场
 * - Ray Marching 步进采样密度
 * - 大气散射（Henyey-Greenstein 相函数）
 * - Beer-Lambert 光学深度衰减
 * - 太阳光内散射（in-scattering）多步光线采样
 * - 天空渐变背景（Rayleigh 近似）
 * - 软光栅输出 1280×720 PNG
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <vector>
#include <random>

// ──────────────────────────────────────────
//  STB Image Write (header-only)
// ──────────────────────────────────────────
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ──────────────────────────────────────────
//  Math helpers
// ──────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b)  const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>0?(*this)/l:Vec3{}; }
    Vec3 clamp01() const { return { std::max(0.f,std::min(1.f,x)),
                                    std::max(0.f,std::min(1.f,y)),
                                    std::max(0.f,std::min(1.f,z)) }; }
};
inline Vec3 mix(const Vec3& a, const Vec3& b, float t){ return a*(1-t)+b*t; }
inline float clamp01(float v){ return std::max(0.f, std::min(1.f, v)); }
inline float lerp(float a, float b, float t){ return a*(1-t)+b*t; }

// ──────────────────────────────────────────
//  Perlin / FBM Noise
// ──────────────────────────────────────────
static uint32_t g_perm[512];

void initNoise(uint32_t seed=42) {
    std::mt19937 rng(seed);
    for(int i=0;i<256;i++) g_perm[i]=(uint32_t)i;
    for(int i=255;i>0;i--){
        int j = rng()%(i+1);
        std::swap(g_perm[i], g_perm[j]);
    }
    for(int i=0;i<256;i++) g_perm[256+i]=g_perm[i];
}

static float fade(float t){ return t*t*t*(t*(t*6-15)+10); }

static float grad3(int h, float x, float y, float z){
    int hh=h&15;
    float u = (hh<8)?x:y;
    float v = (hh<4)?y:(hh==12||hh==14)?x:z;
    return ((hh&1)?-u:u)+((hh&2)?-v:v);
}

float perlin3(float x, float y, float z){
    int xi=(int)floorf(x)&255, yi=(int)floorf(y)&255, zi=(int)floorf(z)&255;
    float xf=x-floorf(x), yf=y-floorf(y), zf=z-floorf(z);
    float u=fade(xf), v=fade(yf), w=fade(zf);
    int aaa=g_perm[g_perm[g_perm[xi  ]+yi  ]+zi  ];
    int aba=g_perm[g_perm[g_perm[xi  ]+yi+1]+zi  ];
    int aab=g_perm[g_perm[g_perm[xi  ]+yi  ]+zi+1];
    int abb=g_perm[g_perm[g_perm[xi  ]+yi+1]+zi+1];
    int baa=g_perm[g_perm[g_perm[xi+1]+yi  ]+zi  ];
    int bba=g_perm[g_perm[g_perm[xi+1]+yi+1]+zi  ];
    int bab=g_perm[g_perm[g_perm[xi+1]+yi  ]+zi+1];
    int bbb=g_perm[g_perm[g_perm[xi+1]+yi+1]+zi+1];
    float x1=lerp(grad3(aaa,xf,yf,zf),   grad3(baa,xf-1,yf,zf),   u);
    float x2=lerp(grad3(aba,xf,yf-1,zf), grad3(bba,xf-1,yf-1,zf), u);
    float y1=lerp(x1,x2,v);
    float x3=lerp(grad3(aab,xf,yf,zf-1),   grad3(bab,xf-1,yf,zf-1),   u);
    float x4=lerp(grad3(abb,xf,yf-1,zf-1), grad3(bbb,xf-1,yf-1,zf-1), u);
    float y2=lerp(x3,x4,v);
    return lerp(y1,y2,w);
}

// FBM (fractional Brownian motion)
float fbm(float x, float y, float z, int octaves=6){
    float val=0, amp=0.5f, freq=1.0f, sum=0;
    for(int i=0;i<octaves;i++){
        val += amp * perlin3(x*freq, y*freq, z*freq);
        sum += amp; amp*=0.5f; freq*=2.0f;
    }
    return val/sum;
}

// ──────────────────────────────────────────
//  Cloud density
// ──────────────────────────────────────────
// Cloud layer: y ∈ [cloudBase, cloudTop]
const float CLOUD_BASE  = 1200.f;   // metres
const float CLOUD_TOP   = 3500.f;
const float CLOUD_SCALE = 0.00035f; // noise scale
const float DENSITY_MULT = 1.4f;

// Rounded billowy coverage
float cloudDensity(const Vec3& p) {
    // Height gradient (denser in middle of layer)
    float fy = (p.y - CLOUD_BASE) / (CLOUD_TOP - CLOUD_BASE);
    if (fy < 0.f || fy > 1.f) return 0.f;
    float heightGrad = fy * (1.f - fy) * 4.f;   // bell curve 0→1→0

    // Base low-frequency shape
    float nx = p.x * CLOUD_SCALE;
    float ny = p.y * CLOUD_SCALE * 0.5f;
    float nz = p.z * CLOUD_SCALE;

    float base  = fbm(nx, ny, nz, 5);          // [-0.5, 0.5]ish
    float detail= fbm(nx*3.f+5.3f, ny*3.f, nz*3.f+1.7f, 4) * 0.35f;
    float d = (base + 0.05f + detail) * DENSITY_MULT * heightGrad;
    return std::max(0.f, d);
}

// ──────────────────────────────────────────
//  Phase function (Henyey-Greenstein)
// ──────────────────────────────────────────
float hg(float cosTheta, float g){
    float g2 = g*g;
    return (1.f - g2) / (4.f * 3.14159265f * powf(1.f + g2 - 2.f*g*cosTheta, 1.5f));
}
float phase(float cosTheta){
    // Two-lobe: forward scattering + isotropic
    return lerp(hg(cosTheta, 0.7f), hg(cosTheta, -0.2f), 0.5f);
}

// ──────────────────────────────────────────
//  Atmosphere background (sky + horizon)
// ──────────────────────────────────────────
Vec3 skyColor(const Vec3& dir, const Vec3& sunDir) {
    float cosTheta = std::max(0.f, dir.dot(sunDir));
    // Rayleigh sky
    float t = clamp01(dir.y * 0.5f + 0.5f);  // 0 at horizon, 1 at zenith
    Vec3 zenith  = {0.08f, 0.22f, 0.65f};
    Vec3 horizon = {0.72f, 0.82f, 0.90f};
    Vec3 sky = mix(horizon, zenith, t * t);
    // Sun disc + glow
    float sun = powf(cosTheta, 256.f) * 5.f;
    float glow= powf(cosTheta, 8.f)   * 0.3f;
    Vec3 sunColor = {1.0f, 0.95f, 0.7f};
    sky += sunColor * (sun + glow);
    return sky;
}

// ──────────────────────────────────────────
//  Light transmittance through cloud (beer)
// ──────────────────────────────────────────
float lightTransmittance(const Vec3& pos, const Vec3& sunDir){
    // March toward sun, accumulate optical depth
    const int LIGHT_STEPS = 6;
    const float LIGHT_STEP = 200.f;  // metres
    float optDepth = 0.f;
    Vec3 p = pos;
    for(int i=0;i<LIGHT_STEPS;i++){
        p = p + sunDir * LIGHT_STEP;
        optDepth += cloudDensity(p) * LIGHT_STEP;
    }
    float sigma_e = 0.008f;   // extinction coeff
    return expf(-optDepth * sigma_e);
}

// ──────────────────────────────────────────
//  Main ray march
// ──────────────────────────────────────────
Vec3 renderCloud(const Vec3& ro, const Vec3& rd, const Vec3& sunDir) {
    const int   MAX_STEPS = 64;
    const float STEP_SIZE = 80.f;    // metres per step
    const float sigma_e   = 0.008f;  // extinction (absorption+scattering)
    const float sigma_s   = 0.006f;  // scattering

    // Find entry/exit into cloud layer
    float tEntry = (CLOUD_BASE - ro.y) / rd.y;
    float tExit  = (CLOUD_TOP  - ro.y) / rd.y;
    if(rd.y == 0.f) return skyColor(rd, sunDir);
    if(tEntry > tExit) std::swap(tEntry, tExit);
    tEntry = std::max(0.f, tEntry);
    tExit  = std::min(tEntry + (float)MAX_STEPS * STEP_SIZE, tExit);
    if(tExit < 0.f) return skyColor(rd, sunDir);

    float cosTheta = rd.dot(sunDir);
    float phaseFn  = phase(cosTheta);
    Vec3  sunLight = {1.4f, 1.2f, 0.9f};   // warm directional
    Vec3  ambient  = {0.15f, 0.2f, 0.3f};  // soft blue fill

    float transmittance = 1.f;
    Vec3  inScatter = {};

    float t = tEntry;
    for(int i = 0; i < MAX_STEPS && t < tExit; i++, t += STEP_SIZE){
        Vec3 p = ro + rd * t;
        float density = cloudDensity(p);
        if(density <= 0.001f) continue;

        float optDepth = density * STEP_SIZE * sigma_e;
        float stepTrans = expf(-optDepth);

        // Light contribution at this sample
        float lt = lightTransmittance(p, sunDir);
        Vec3 scatter = (sunLight * lt * phaseFn + ambient) * sigma_s * density;

        // Integrate (energy-conserving)
        Vec3 scatterInteg = scatter * ((1.f - stepTrans) / (sigma_e + 1e-6f));
        inScatter += scatterInteg * transmittance;
        transmittance *= stepTrans;

        if(transmittance < 0.01f) break;
    }

    Vec3 bg = skyColor(rd, sunDir);
    return bg * transmittance + inScatter;
}

// ──────────────────────────────────────────
//  Tone mapping
// ──────────────────────────────────────────
Vec3 aces(Vec3 x){
    float a=2.51f, b=0.03f, c=2.43f, d=0.59f, e=0.14f;
    auto f=[&](float v){ return clamp01((v*(a*v+b))/(v*(c*v+d)+e)); };
    return {f(x.x), f(x.y), f(x.z)};
}

// ──────────────────────────────────────────
//  Main
// ──────────────────────────────────────────
int main(){
    initNoise(12345);

    const int W = 1280, H = 720;
    std::vector<uint8_t> pixels(W * H * 3);

    // Camera: standing on the ground looking up-ish
    Vec3 camPos = {0.f, 0.f, 0.f};    // ground level
    float fov   = 70.f * 3.14159265f / 180.f;
    float aspect= (float)W / H;
    float halfH = tanf(fov * 0.5f);

    // Sun direction (low angle, golden hour)
    Vec3 sunDir = Vec3{1.5f, 0.8f, -0.6f}.norm();

    printf("Rendering %dx%d volumetric cloud scene...\n", W, H);

    int progress = 0;
    for(int py = 0; py < H; py++){
        if(py % 72 == 0){
            printf("  Progress: %d%%\r", progress);
            fflush(stdout);
            progress += 10;
        }
        for(int px = 0; px < W; px++){
            float u = (px + 0.5f) / W  * 2.f - 1.f;
            float v = 1.f - (py + 0.5f) / H * 2.f;  // flip Y: top=sky

            // Ray direction (looking slightly upward)
            Vec3 rd = Vec3{ u * aspect * halfH,
                            v * halfH + 0.35f,   // slight upward tilt
                            -1.f }.norm();

            Vec3 color = renderCloud(camPos, rd, sunDir);
            color = aces(color);
            color = color.clamp01();

            int idx = (py * W + px) * 3;
            pixels[idx+0] = (uint8_t)(color.x * 255);
            pixels[idx+1] = (uint8_t)(color.y * 255);
            pixels[idx+2] = (uint8_t)(color.z * 255);
        }
    }

    printf("\n");

    const char* outFile = "volumetric_cloud_output.png";
    stbi_write_png(outFile, W, H, 3, pixels.data(), W*3);
    printf("✅ Saved: %s\n", outFile);
    return 0;
}
