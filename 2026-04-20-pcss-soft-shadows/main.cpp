/**
 * PCSS - Percentage Closer Soft Shadows
 * 
 * 实现原理：
 * 1. 生成 Shadow Map（从光源视角渲染深度图）
 * 2. 对每个着色点，搜索 Shadow Map 中遮挡物（Blocker Search）
 * 3. 根据平均遮挡深度估计半影宽度（Penumbra Estimation）
 * 4. 用可变大小的 PCF 滤波器采样阴影（Percentage Closer Filtering）
 * 
 * 对比：硬阴影（PCF size=1）vs 软阴影（PCSS动态 PCF size）
 * 
 * 场景：
 * - 地面平面（Y=0）
 * - 3个不同高度的球体
 * - 面光源（模拟为区域光源）
 * 
 * 输出：pcss_output.png（左：硬阴影，右：软阴影 PCSS）
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <string>
#include <cstring>
#include <cstdio>
#include <cassert>

// ============================================================
// Math
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    Vec3 operator*(const Vec3&o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3 operator-()const{return{-x,-y,-z};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-8f?(*this)/l:Vec3(0,1,0);}
    float& operator[](int i){return (&x)[i];}
    float  operator[](int i)const{return (&x)[i];}
};

struct Vec2 {
    float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
};

inline float clamp01(float v){return std::max(0.0f,std::min(1.0f,v));}
inline float clamp(float v,float lo,float hi){return std::max(lo,std::min(hi,v));}
inline Vec3 lerp(const Vec3&a,const Vec3&b,float t){return a*(1-t)+b*t;}

// ============================================================
// Scene Objects
// ============================================================
struct Sphere {
    Vec3 center;
    float radius;
    Vec3 albedo;
};

struct Plane {
    Vec3 normal;
    float d; // n·p = d
    Vec3 albedo;
    Vec3 albedo2; // checker pattern second color
    bool checker;
};

// Ray-sphere intersection
float intersectSphere(const Vec3& ro, const Vec3& rd, const Sphere& s) {
    Vec3 oc = ro - s.center;
    float a = rd.dot(rd);
    float b = 2.0f * oc.dot(rd);
    float c = oc.dot(oc) - s.radius * s.radius;
    float disc = b*b - 4*a*c;
    if (disc < 0) return -1.0f;
    float sq = std::sqrt(disc);
    float t0 = (-b - sq) / (2*a);
    float t1 = (-b + sq) / (2*a);
    if (t0 > 1e-4f) return t0;
    if (t1 > 1e-4f) return t1;
    return -1.0f;
}

// Ray-plane intersection
float intersectPlane(const Vec3& ro, const Vec3& rd, const Plane& p) {
    float denom = p.normal.dot(rd);
    if (std::abs(denom) < 1e-6f) return -1.0f;
    float t = (p.d - p.normal.dot(ro)) / denom;
    return t > 1e-4f ? t : -1.0f;
}

// ============================================================
// Shadow Map
// ============================================================
struct ShadowMap {
    int W, H;
    std::vector<float> depth; // shadow map depth buffer
    // Light's view-projection transform
    Vec3 lightPos;
    Vec3 lightDir; // direction FROM light to scene center
    Vec3 lightUp;
    Vec3 lightRight;
    float near_plane, far_plane;
    float half_size; // orthographic half size

    ShadowMap(int w, int h) : W(w), H(h), depth(w*h, 1.0f) {}

    // Project world point to shadow map UV (and depth)
    // Returns false if outside light frustum
    bool project(const Vec3& P, float& u, float& v, float& depthVal) const {
        Vec3 lp = P - lightPos;
        float z = lp.dot(lightDir);
        float x = lp.dot(lightRight);
        float y = lp.dot(lightUp);
        // Orthographic projection
        if (z < near_plane || z > far_plane) return false;
        u = (x / half_size) * 0.5f + 0.5f;
        v = (y / half_size) * 0.5f + 0.5f;
        if (u < 0 || u > 1 || v < 0 || v > 1) return false;
        depthVal = (z - near_plane) / (far_plane - near_plane);
        return true;
    }

    float sample(float u, float v) const {
        int px = clamp((int)(u * W), 0, W-1);
        int py = clamp((int)(v * H), 0, H-1);
        return depth[py * W + px];
    }

    // Bilinear sample
    float sampleBilinear(float u, float v) const {
        float fx = u * W - 0.5f;
        float fy = v * H - 0.5f;
        int ix = (int)std::floor(fx);
        int iy = (int)std::floor(fy);
        float tx = fx - ix;
        float ty = fy - iy;
        auto smp = [&](int px, int py) -> float {
            px = clamp(px, 0, W-1);
            py = clamp(py, 0, H-1);
            return depth[py * W + px];
        };
        return (smp(ix,iy)*(1-tx) + smp(ix+1,iy)*tx) * (1-ty)
             + (smp(ix,iy+1)*(1-tx) + smp(ix+1,iy+1)*tx) * ty;
    }
};

// ============================================================
// Scene
// ============================================================
const Vec3 LIGHT_COLOR = Vec3(1.0f, 0.95f, 0.85f);
const float LIGHT_RADIUS = 2.0f;   // area light radius (for PCSS penumbra)

struct Scene {
    std::vector<Sphere> spheres;
    Plane ground;
    Vec3 lightPos;

    Scene() {
        // Ground plane at y=0
        ground.normal = Vec3(0,1,0);
        ground.d = 0.0f;
        ground.albedo  = Vec3(0.7f, 0.7f, 0.7f);
        ground.albedo2 = Vec3(0.4f, 0.4f, 0.4f);
        ground.checker = true;

        // Light position (above and to the side)
        lightPos = Vec3(3.0f, 8.0f, 2.0f);

        // Spheres at different heights and positions
        spheres.push_back({Vec3(-2.5f, 1.0f, 0.0f), 1.0f, Vec3(0.9f, 0.3f, 0.3f)});  // Red sphere
        spheres.push_back({Vec3( 0.0f, 1.5f, 0.5f), 1.5f, Vec3(0.3f, 0.7f, 0.9f)});  // Blue sphere (taller)
        spheres.push_back({Vec3( 2.8f, 0.6f, -0.5f), 0.6f, Vec3(0.3f, 0.9f, 0.3f)}); // Green sphere (small)
    }
};

// Build shadow map
ShadowMap buildShadowMap(const Scene& scene, int smSize) {
    ShadowMap sm(smSize, smSize);
    sm.lightPos = scene.lightPos;

    // Light looks toward scene center
    Vec3 target(0, 0, 0);
    sm.lightDir = (target - scene.lightPos).norm();

    // Build orthonormal basis
    Vec3 worldUp(0,1,0);
    if (std::abs(sm.lightDir.dot(worldUp)) > 0.9f)
        worldUp = Vec3(0,0,1);
    sm.lightRight = worldUp.cross(sm.lightDir).norm();
    sm.lightUp = sm.lightDir.cross(sm.lightRight).norm();

    sm.near_plane = 1.0f;
    sm.far_plane  = 20.0f;
    sm.half_size  = 8.0f;

    // Rasterize scene into shadow map (depth only)
    // Use ray casting from each light texel
    for (int py = 0; py < smSize; py++) {
        for (int px = 0; px < smSize; px++) {
            float u = (px + 0.5f) / smSize;
            float v = (py + 0.5f) / smSize;
            float lx = (u * 2.0f - 1.0f) * sm.half_size;
            float ly = (v * 2.0f - 1.0f) * sm.half_size;
            Vec3 ro = sm.lightPos + sm.lightRight*lx + sm.lightUp*ly;
            Vec3 rd = sm.lightDir;

            float minDepth = 1.0f;

            // Test spheres
            for (auto& s : scene.spheres) {
                float t = intersectSphere(ro, rd, s);
                if (t > 0) {
                    Vec3 P = ro + rd*t;
                    Vec3 lp = P - sm.lightPos;
                    float z = lp.dot(sm.lightDir);
                    float d = (z - sm.near_plane) / (sm.far_plane - sm.near_plane);
                    d = clamp01(d);
                    if (d < minDepth) minDepth = d;
                }
            }

            // Test ground plane
            {
                float t = intersectPlane(ro, rd, scene.ground);
                if (t > 0) {
                    Vec3 P = ro + rd*t;
                    Vec3 lp = P - sm.lightPos;
                    float z = lp.dot(sm.lightDir);
                    float d = (z - sm.near_plane) / (sm.far_plane - sm.near_plane);
                    d = clamp01(d);
                    if (d < minDepth) minDepth = d;
                }
            }

            sm.depth[py * smSize + px] = minDepth;
        }
    }

    return sm;
}

// ============================================================
// PCF Shadow (fixed kernel size)
// ============================================================
float shadowPCF(const ShadowMap& sm, const Vec3& P, float bias, int kernelRadius) {
    float u, v, depth;
    if (!sm.project(P, u, v, depth)) return 1.0f; // outside frustum = lit

    depth -= bias;

    if (kernelRadius == 0) {
        // Hard shadow
        float smDepth = sm.sampleBilinear(u, v);
        return (depth <= smDepth) ? 1.0f : 0.0f;
    }

    int numSamples = (2*kernelRadius+1) * (2*kernelRadius+1);
    float shadow = 0.0f;

    for (int ky = -kernelRadius; ky <= kernelRadius; ky++) {
        for (int kx = -kernelRadius; kx <= kernelRadius; kx++) {
            float su = u + (float)kx / sm.W;
            float sv = v + (float)ky / sm.H;
            float smDepth = sm.sampleBilinear(su, sv);
            if (depth <= smDepth) shadow += 1.0f;
        }
    }
    return shadow / numSamples;
}

// ============================================================
// PCSS Shadow
// ============================================================
float shadowPCSS(const ShadowMap& sm, const Vec3& P, float bias,
                 int blockerSearchRadius, int maxPCFRadius)
{
    float u, v, recvDepth;
    if (!sm.project(P, u, v, recvDepth)) return 1.0f;
    recvDepth -= bias;

    // Step 1: Blocker Search
    float blockerSum = 0.0f;
    int   blockerCount = 0;
    int   bsr = blockerSearchRadius;

    for (int ky = -bsr; ky <= bsr; ky++) {
        for (int kx = -bsr; kx <= bsr; kx++) {
            float su = u + (float)kx / sm.W;
            float sv = v + (float)ky / sm.H;
            float smDepth = sm.sampleBilinear(su, sv);
            if (smDepth < recvDepth) {
                blockerSum += smDepth;
                blockerCount++;
            }
        }
    }

    // No blockers = fully lit
    if (blockerCount == 0) return 1.0f;

    float avgBlockerDepth = blockerSum / blockerCount;

    // Step 2: Penumbra estimation
    // penumbra = (d_receiver - d_blocker) / d_blocker * lightRadius
    // In normalized depth space:
    float dReceiver = recvDepth * (sm.far_plane - sm.near_plane) + sm.near_plane;
    float dBlocker  = avgBlockerDepth * (sm.far_plane - sm.near_plane) + sm.near_plane;

    float penumbraRatio = (dReceiver - dBlocker) / dBlocker * LIGHT_RADIUS;
    // Convert to texel kernel size
    float penumbraTexels = penumbraRatio * sm.W / (2.0f * sm.half_size);
    int pcfRadius = clamp((int)penumbraTexels, 1, maxPCFRadius);

    // Step 3: PCF with dynamic kernel
    float shadow = 0.0f;
    int numSamples = (2*pcfRadius+1) * (2*pcfRadius+1);
    for (int ky = -pcfRadius; ky <= pcfRadius; ky++) {
        for (int kx = -pcfRadius; kx <= pcfRadius; kx++) {
            float su = u + (float)kx / sm.W;
            float sv = v + (float)ky / sm.H;
            float smDepth = sm.sampleBilinear(su, sv);
            if (recvDepth <= smDepth) shadow += 1.0f;
        }
    }
    return shadow / numSamples;
}

// ============================================================
// Shading
// ============================================================
Vec3 shade(const Vec3& P, const Vec3& N, const Vec3& albedo,
           const Vec3& lightPos, const Vec3& viewDir, float shadowFactor)
{
    Vec3 L = (lightPos - P).norm();
    Vec3 H = (L + viewDir).norm();

    float NdotL = std::max(0.0f, N.dot(L));
    float NdotH = std::max(0.0f, N.dot(H));

    Vec3 ambient = albedo * 0.15f;
    Vec3 diffuse = albedo * LIGHT_COLOR * NdotL * shadowFactor;
    Vec3 specular = LIGHT_COLOR * std::pow(NdotH, 32.0f) * 0.3f * shadowFactor;

    return ambient + diffuse + specular;
}

// ============================================================
// Render
// ============================================================
struct Image {
    int W, H;
    std::vector<unsigned char> data;
    Image(int w, int h) : W(w), H(h), data(w*h*3, 0) {}
    void set(int x, int y, Vec3 c) {
        c.x = clamp01(c.x);
        c.y = clamp01(c.y);
        c.z = clamp01(c.z);
        // Gamma correction
        c.x = std::pow(c.x, 1.0f/2.2f);
        c.y = std::pow(c.y, 1.0f/2.2f);
        c.z = std::pow(c.z, 1.0f/2.2f);
        int idx = (y * W + x) * 3;
        data[idx]   = (unsigned char)(c.x * 255.0f);
        data[idx+1] = (unsigned char)(c.y * 255.0f);
        data[idx+2] = (unsigned char)(c.z * 255.0f);
    }
};

enum ShadowMode { HARD_SHADOW, PCSS_SHADOW };

Vec3 renderPixel(const Scene& scene, const ShadowMap& sm,
                 float px, float py, int W, int H,
                 ShadowMode mode)
{
    // Camera setup
    Vec3 camPos(0, 4, 10);
    Vec3 camTarget(0, 1, 0);
    Vec3 camUp(0, 1, 0);
    float fov = 45.0f * M_PI / 180.0f;
    float aspect = (float)W / H;

    Vec3 forward = (camTarget - camPos).norm();
    Vec3 right = forward.cross(camUp).norm();
    Vec3 up = right.cross(forward).norm();

    float tanHalfFov = std::tan(fov * 0.5f);
    float ndcX = (2.0f * px / W - 1.0f) * aspect * tanHalfFov;
    float ndcY = (1.0f - 2.0f * py / H) * tanHalfFov;

    Vec3 rd = (forward + right*ndcX + up*ndcY).norm();
    Vec3 ro = camPos;

    // Find closest intersection
    float tMin = 1e30f;
    Vec3 hitN(0,1,0), hitP, hitAlbedo;
    bool hit = false;

    // Test spheres
    for (auto& s : scene.spheres) {
        float t = intersectSphere(ro, rd, s);
        if (t > 0 && t < tMin) {
            tMin = t;
            hitP = ro + rd*t;
            hitN = (hitP - s.center).norm();
            hitAlbedo = s.albedo;
            hit = true;
        }
    }

    // Test ground
    {
        float t = intersectPlane(ro, rd, scene.ground);
        if (t > 0 && t < tMin) {
            tMin = t;
            hitP = ro + rd*t;
            hitN = scene.ground.normal;
            // Checker pattern
            float cx = std::floor(hitP.x) + std::floor(hitP.z);
            bool even = ((int)std::abs(cx) % 2) == 0;
            hitAlbedo = even ? scene.ground.albedo : scene.ground.albedo2;
            hit = true;
        }
    }

    if (!hit) {
        // Sky gradient
        float t = 0.5f * (rd.y + 1.0f);
        return lerp(Vec3(0.8f, 0.9f, 1.0f), Vec3(0.3f, 0.5f, 0.9f), t);
    }

    // Shadow computation
    float bias = 0.01f;
    float shadowFactor;
    if (mode == HARD_SHADOW) {
        shadowFactor = shadowPCF(sm, hitP, bias, 0);
    } else {
        shadowFactor = shadowPCSS(sm, hitP, bias, 8, 16);
    }

    Vec3 viewDir = -rd;
    return shade(hitP, hitN, hitAlbedo, scene.lightPos, viewDir, shadowFactor);
}

// ============================================================
// PPM Writer
// ============================================================
bool writePPM(const std::string& filename, const Image& img) {
    FILE* f = fopen(filename.c_str(), "wb");
    if (!f) return false;
    fprintf(f, "P6\n%d %d\n255\n", img.W, img.H);
    fwrite(img.data.data(), 1, img.data.size(), f);
    fclose(f);
    return true;
}

// ============================================================
// Main
// ============================================================
int main() {
    printf("PCSS Percentage Closer Soft Shadows\n");
    printf("Building scene...\n");

    Scene scene;
    const int SM_SIZE = 512;
    printf("Building shadow map (%dx%d)...\n", SM_SIZE, SM_SIZE);
    ShadowMap sm = buildShadowMap(scene, SM_SIZE);
    printf("Shadow map built.\n");

    // Render side-by-side comparison
    const int W = 800; // total width (400 per half)
    const int H = 400;
    const int HALF_W = W / 2;

    Image img(W, H);

    printf("Rendering hard shadows (left half)...\n");
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < HALF_W; x++) {
            Vec3 c = renderPixel(scene, sm, (float)x, (float)y, HALF_W, H, HARD_SHADOW);
            img.set(x, y, c);
        }
        if (y % 50 == 0) printf("  Hard shadow: row %d/%d\n", y, H);
    }

    printf("Rendering PCSS shadows (right half)...\n");
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < HALF_W; x++) {
            Vec3 c = renderPixel(scene, sm, (float)x, (float)y, HALF_W, H, PCSS_SHADOW);
            img.set(HALF_W + x, y, c);
        }
        if (y % 50 == 0) printf("  PCSS shadow: row %d/%d\n", y, H);
    }

    // Draw dividing line
    for (int y = 0; y < H; y++) {
        img.set(HALF_W-1, y, Vec3(1,1,0));
        img.set(HALF_W,   y, Vec3(1,1,0));
    }

    // Labels (top area - just leave as is, the division is clear)

    std::string outFile = "pcss_output.ppm";
    if (writePPM(outFile, img)) {
        printf("✅ Output written: %s (%dx%d)\n", outFile.c_str(), W, H);
    } else {
        printf("❌ Failed to write output\n");
        return 1;
    }

    printf("Done!\n");
    return 0;
}
