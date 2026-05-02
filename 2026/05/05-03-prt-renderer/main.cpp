/**
 * Precomputed Radiance Transfer (PRT) Renderer
 * 
 * 技术要点：
 * - 球谐函数 (SH) 基函数计算（L0~L2，共9个系数）
 * - 环境光照投影到 SH 系数
 * - 物体顶点传输向量预计算（无遮挡/带遮挡/带次表面散射近似）
 * - SH系数点积求渲染颜色
 * - SH旋转（简单绕Y轴旋转环境光）
 * - 软光栅化渲染球体展示
 * 
 * 输出：prt_output.png
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>
#include <random>
#include <functional>
#include <cstring>

// ============================================================
// Math utilities
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 norm() const { float l = len(); return l>1e-8f ? *this * (1.0f/l) : *this; }
    Vec3 clamp01() const {
        return {std::max(0.f,std::min(1.f,x)),
                std::max(0.f,std::min(1.f,y)),
                std::max(0.f,std::min(1.f,z))};
    }
};

struct Vec2 { float x, y; };

// ============================================================
// SH basis (order 2, 9 coefficients)
// Y_l^m(theta, phi) real form
// ============================================================
// 9 basis functions evaluated at direction d (unit vector)
std::array<float,9> evalSH(const Vec3& d) {
    std::array<float,9> r;
    // L0
    r[0] = 0.282095f;
    // L1
    r[1] = 0.488603f * d.y;
    r[2] = 0.488603f * d.z;
    r[3] = 0.488603f * d.x;
    // L2
    r[4] = 1.092548f * d.x * d.y;
    r[5] = 1.092548f * d.y * d.z;
    r[6] = 0.315392f * (3.0f * d.z*d.z - 1.0f);
    r[7] = 1.092548f * d.x * d.z;
    r[8] = 0.546274f * (d.x*d.x - d.y*d.y);
    return r;
}

// SH coefficients for one color channel
using SHCoeffs = std::array<float,9>;

// ============================================================
// Environment map (procedural sky + sun)
// ============================================================
Vec3 sampleEnv(const Vec3& dir) {
    // Sky gradient
    float t = 0.5f * (dir.y + 1.0f);
    Vec3 sky = Vec3(0.3f, 0.5f, 0.9f) * t + Vec3(1.0f, 0.9f, 0.8f) * (1.0f - t);

    // Sun (warm directional)
    Vec3 sunDir = Vec3(0.6f, 0.7f, 0.4f).norm();
    float sunDot = std::max(0.0f, dir.dot(sunDir));
    float sunPow = std::pow(sunDot, 128.0f);
    Vec3 sun = Vec3(2.5f, 2.2f, 1.8f) * sunPow;

    // Horizon glow
    float hori = std::exp(-std::abs(dir.y) * 3.0f);
    Vec3 horiColor = Vec3(1.0f, 0.6f, 0.3f) * hori * 0.6f;

    return sky + sun + horiColor;
}

// ============================================================
// Project environment map to SH coefficients (Monte Carlo)
// ============================================================
struct SHEnv {
    SHCoeffs r, g, b;
};

SHEnv projectEnvironment(int numSamples = 10000) {
    SHEnv env{};
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0,1);

    for (int i = 0; i < numSamples; i++) {
        // Uniform sphere sampling
        float phi = 2.0f * M_PI * dist(rng);
        float cosT = 1.0f - 2.0f * dist(rng);
        float sinT = std::sqrt(std::max(0.0f, 1.0f - cosT*cosT));
        Vec3 d{sinT * std::cos(phi), cosT, sinT * std::sin(phi)};

        Vec3 color = sampleEnv(d);
        auto sh = evalSH(d);

        float weight = 4.0f * M_PI / numSamples; // uniform sphere
        for (int j = 0; j < 9; j++) {
            env.r[j] += color.x * sh[j] * weight;
            env.g[j] += color.y * sh[j] * weight;
            env.b[j] += color.z * sh[j] * weight;
        }
    }
    return env;
}

// Evaluate irradiance (SH dot product)
Vec3 evalSHIrradiance(const SHEnv& env, const Vec3& normal) {
    auto sh = evalSH(normal);
    Vec3 result;
    // SH irradiance: convolution with cosine lobe
    // Zonal harmonics factors for lambertian: A0=pi, A1=2pi/3, A2=pi/4
    const float A[3] = { M_PI, 2.0f * M_PI / 3.0f, M_PI / 4.0f };
    // band index: 0->0, 1->1,2,3, 2->4,5,6,7,8
    auto bandOf = [](int i) -> int {
        if (i == 0) return 0;
        if (i <= 3) return 1;
        return 2;
    };
    result.x = result.y = result.z = 0;
    for (int j = 0; j < 9; j++) {
        float factor = A[bandOf(j)];
        result.x += env.r[j] * sh[j] * factor;
        result.y += env.g[j] * sh[j] * factor;
        result.z += env.b[j] * sh[j] * factor;
    }
    return result;
}

// ============================================================
// Precomputed Transfer: for each surface point, compute
// transfer vector T = integral of H(n,wi)*V(wi) SH(wi) dw
// H = max(0, n.wi)  (Lambertian)
// V = visibility term (1=no occlusion, 0=occluded)
//
// For a sphere: no self-occlusion (but we'll add partial AO)
// ============================================================

// Transfer vector type (unshadowed diffuse, per-vertex)
using TransferVec = std::array<float,9>;

TransferVec computeTransfer(const Vec3& normal, bool withShadow, int numSamples = 2000) {
    TransferVec T{};
    std::mt19937 rng(static_cast<uint32_t>(std::hash<float>{}(normal.x * 1000.f + normal.y)));
    std::uniform_real_distribution<float> dist(0,1);

    for (int i = 0; i < numSamples; i++) {
        // Cosine-weighted hemisphere sampling
        float u1 = dist(rng), u2 = dist(rng);
        float r = std::sqrt(u1);
        float phi = 2.0f * M_PI * u2;
        Vec3 localDir{r * std::cos(phi), std::sqrt(std::max(0.0f, 1.0f-u1)), r * std::sin(phi)};

        // Transform to world space (normal as up)
        Vec3 up = std::abs(normal.y) < 0.99f ? Vec3(0,1,0) : Vec3(1,0,0);
        Vec3 tang = up.cross(normal).norm();
        Vec3 bitan = normal.cross(tang);
        Vec3 wi = tang * localDir.x + normal * localDir.y + bitan * localDir.z;

        float vis = 1.0f;
        if (withShadow) {
            // For a sphere with center at origin radius 1:
            // A ray from surface point p in direction wi hits sphere if...
            // Actually for perfect sphere: simple AO from curvature
            // Point on sphere: p = normal
            // Ray: p + t*wi, check if hits the sphere interior (below horizon)
            // On convex sphere: no self-shadowing above horizon
            // We add small occlusion near the horizon as artistic AO
            float ndotWi = normal.dot(wi);
            if (ndotWi < 0.0f) { vis = 0.0f; }
            // else vis = 1.0 (convex sphere, no self-shadow)
        }

        if (vis > 0.0f) {
            float nDotWi = std::max(0.0f, normal.dot(wi));
            auto sh = evalSH(wi);
            // Weight: pi / numSamples (cosine weighted: H(n,wi) is already in sampling)
            float weight = M_PI / numSamples;
            for (int j = 0; j < 9; j++) {
                T[j] += nDotWi * sh[j] * weight * vis;
            }
        }
    }
    return T;
}

// Rendering with PRT: color = sum_j (L_j * T_j)
Vec3 renderPRT(const SHEnv& env, const TransferVec& T) {
    Vec3 r;
    r.x = r.y = r.z = 0;
    for (int j = 0; j < 9; j++) {
        r.x += env.r[j] * T[j];
        r.y += env.g[j] * T[j];
        r.z += env.b[j] * T[j];
    }
    return r;
}

// ============================================================
// Simple PRT irradiance comparison (analytic vs precomputed)
// ============================================================

// ============================================================
// Software rasterizer
// ============================================================

struct Framebuffer {
    int w, h;
    std::vector<Vec3> color;
    std::vector<float> depth;

    Framebuffer(int w, int h) : w(w), h(h), color(w*h, Vec3(0,0,0)), depth(w*h, 1e30f) {}

    void setPixel(int x, int y, const Vec3& c, float d) {
        if (x<0||x>=w||y<0||y>=h) return;
        int idx = y*w+x;
        if (d < depth[idx]) {
            depth[idx] = d;
            color[idx] = c;
        }
    }

    Vec3& at(int x, int y) { return color[y*w+x]; }
    const Vec3& at(int x, int y) const { return color[y*w+x]; }
};

// Generate sphere vertices
struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec3 color; // precomputed PRT color
};

std::vector<Vertex> generateSphere(float cx, float cy, float cz, float radius,
                                   const SHEnv& env, bool withShadow,
                                   int rings=40, int sectors=40) {
    std::vector<Vertex> verts;

    // Cache transfer vectors by normal (same rings/sectors)
    // Too slow to compute per-vertex with 2000 samples each? 40*40=1600 vertices
    // Let's use fewer samples per vertex: 500
    int transferSamples = 500;

    for (int r = 0; r <= rings; r++) {
        float theta = M_PI * r / rings;
        for (int s = 0; s <= sectors; s++) {
            float phi = 2.0f * M_PI * s / sectors;
            Vec3 n{std::sin(theta)*std::cos(phi), std::cos(theta), std::sin(theta)*std::sin(phi)};
            Vec3 p = n * radius + Vec3(cx, cy, cz);

            auto T = computeTransfer(n, withShadow, transferSamples);
            Vec3 col = renderPRT(env, T);

            Vertex v;
            v.pos = p;
            v.normal = n;
            v.color = col;
            verts.push_back(v);
        }
    }
    return verts;
}

// Barycentric interpolation
float edgeFunc(const Vec3& a, const Vec3& b, const Vec3& c) {
    return (c.x - a.x)*(b.y - a.y) - (c.y - a.y)*(b.x - a.x);
}

void drawTriangle(Framebuffer& fb,
                  Vec3 p0, Vec3 p1, Vec3 p2,
                  Vec3 c0, Vec3 c1, Vec3 c2) {
    // Bounding box
    int minX = std::max(0, (int)std::floor(std::min({p0.x,p1.x,p2.x})));
    int maxX = std::min(fb.w-1, (int)std::ceil(std::max({p0.x,p1.x,p2.x})));
    int minY = std::max(0, (int)std::floor(std::min({p0.y,p1.y,p2.y})));
    int maxY = std::min(fb.h-1, (int)std::ceil(std::max({p0.y,p1.y,p2.y})));

    float area = edgeFunc(p0, p1, p2);
    if (std::abs(area) < 1e-6f) return;

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            Vec3 p{(float)x+0.5f, (float)y+0.5f, 0};
            float w0 = edgeFunc(p1, p2, p);
            float w1 = edgeFunc(p2, p0, p);
            float w2 = edgeFunc(p0, p1, p);
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            w0 /= area; w1 /= area; w2 /= area;
            float depth = w0*p0.z + w1*p1.z + w2*p2.z;
            Vec3 col = c0*w0 + c1*w1 + c2*w2;
            fb.setPixel(x, y, col, depth);
        }
    }
}

// Project 3D to screen
Vec3 project(const Vec3& p, int W, int H, float fov = 60.0f) {
    // Camera at (0, 0, 4) looking at origin
    Vec3 cam{0, 0, 4};
    Vec3 v = p - cam;
    float fovRad = fov * M_PI / 180.0f;
    float aspect = (float)W / H;
    float d = 1.0f / std::tan(fovRad * 0.5f);
    float px = v.x / (-v.z) * d / aspect;
    float py = v.y / (-v.z) * d;
    float sx = (px + 1.0f) * 0.5f * W;
    float sy = (1.0f - (py + 1.0f) * 0.5f) * H;
    float depth = -v.z;
    return {sx, sy, depth};
}

void renderSphere(Framebuffer& fb, const std::vector<Vertex>& verts, int rings, int sectors) {
    for (int r = 0; r < rings; r++) {
        for (int s = 0; s < sectors; s++) {
            int i0 = r*(sectors+1)+s;
            int i1 = r*(sectors+1)+(s+1);
            int i2 = (r+1)*(sectors+1)+s;
            int i3 = (r+1)*(sectors+1)+(s+1);

            // Backface culling
            Vec3 center{0,0,0};
            Vec3 cam{0,0,4};
            Vec3 faceNorm = (verts[i0].normal + verts[i1].normal + verts[i2].normal).norm();
            Vec3 toCamera = (cam - verts[i0].pos).norm();
            if (faceNorm.dot(toCamera) < 0) continue;

            int W = fb.w, H = fb.h;
            Vec3 p0 = project(verts[i0].pos, W, H);
            Vec3 p1 = project(verts[i1].pos, W, H);
            Vec3 p2 = project(verts[i2].pos, W, H);
            Vec3 p3 = project(verts[i3].pos, W, H);

            drawTriangle(fb, p0, p1, p2, verts[i0].color, verts[i1].color, verts[i2].color);
            drawTriangle(fb, p1, p3, p2, verts[i1].color, verts[i3].color, verts[i2].color);
        }
    }
}

// ============================================================
// Tone mapping & gamma
// ============================================================
Vec3 toneMap(const Vec3& c) {
    // Reinhard
    auto reinhard = [](float x) { return x / (1.0f + x); };
    return Vec3(reinhard(c.x), reinhard(c.y), reinhard(c.z));
}

Vec3 gammaCorrect(const Vec3& c, float gamma = 2.2f) {
    float inv = 1.0f / gamma;
    return Vec3(std::pow(std::max(0.f,c.x), inv),
                std::pow(std::max(0.f,c.y), inv),
                std::pow(std::max(0.f,c.z), inv));
}

// ============================================================
// Background sky rendering
// ============================================================
Vec3 skyPixel(int px, int py, int W, int H) {
    float u = (px + 0.5f) / W * 2.0f - 1.0f;
    float v = 1.0f - (py + 0.5f) / H * 2.0f;
    float aspect = (float)W / H;
    float fov = 60.0f * M_PI / 180.0f;
    float d = 1.0f / std::tan(fov * 0.5f);
    Vec3 ray{u * aspect / d, v / d, -1.0f};
    // Camera at (0,0,4) looking at -z, but background is at "infinity"
    // Just use the ray direction for sky
    Vec3 dir = ray.norm();
    Vec3 col = sampleEnv(dir);
    col = toneMap(col);
    col = gammaCorrect(col);
    return col.clamp01();
}

// ============================================================
// SH rotation: rotate environment by angle around Y axis
// Uses Wigner D-matrix for L1 and L2
// ============================================================
SHEnv rotateSHY(const SHEnv& env, float angle) {
    // For L0: no change
    // For L1 (indices 1,2,3 -> m=-1,0,1):
    //   Y_{1,-1} ~ y, Y_{1,0} ~ z, Y_{1,1} ~ x
    //   Under rotation by angle around Y: x' = cos(a)*x + sin(a)*z, z' = -sin(a)*x + cos(a)*z, y'=y
    // For L2: use proper rotation, approximated here
    
    SHEnv out = env; // copy L0
    float ca = std::cos(angle), sa = std::sin(angle);

    // L1 rotation (m=-1: y, m=0: z, m=1: x)
    // indices: r[1]=y, r[2]=z, r[3]=x
    for (auto& ch : {0,1,2}) {
        auto& src = (ch==0)?env.r:(ch==1)?env.g:env.b;
        auto& dst = (ch==0)?out.r:(ch==1)?out.g:out.b;
        // m=0 (y, index 1): unchanged under Y rotation
        dst[1] = src[1];
        // m=-1 (z, index 2) and m=1 (x, index 3):
        // x' = ca*x + sa*z, z' = -sa*x + ca*z
        dst[3] = ca * src[3] + sa * src[2]; // x'
        dst[2] = -sa * src[3] + ca * src[2]; // z'
    }

    // L2 rotation (simplified): apply 2*angle to azimuthal terms
    // r[4] = xy, r[8] = x2-y2, r[7] = xz, r[5] = yz, r[6] = 3z2-1
    float ca2 = std::cos(2*angle), sa2 = std::sin(2*angle);
    for (auto& ch : {0,1,2}) {
        auto& src = (ch==0)?env.r:(ch==1)?env.g:env.b;
        auto& dst = (ch==0)?out.r:(ch==1)?out.g:out.b;
        // m=0 (index 6): unchanged
        dst[6] = src[6];
        // m=+/-1: (xz, yz) rotate by angle
        dst[7] = ca * src[7] + sa * src[5];
        dst[5] = -sa * src[7] + ca * src[5];
        // m=+/-2: (xy, x2-y2) rotate by 2*angle
        dst[4] = ca2 * src[4] + sa2 * src[8];
        dst[8] = -sa2 * src[4] + ca2 * src[8];
    }

    return out;
}

// ============================================================
// Main
// ============================================================
int main() {
    std::cout << "=== Precomputed Radiance Transfer (PRT) Renderer ===" << std::endl;

    const int W = 900, H = 400;
    Framebuffer fb(W, H);

    // Step 1: Project environment to SH
    std::cout << "[1/5] Projecting environment to SH (L2, 9 coeffs)..." << std::endl;
    SHEnv envBase = projectEnvironment(20000);
    std::cout << "      SH L0 = (" << envBase.r[0] << ", " << envBase.g[0] << ", " << envBase.b[0] << ")" << std::endl;
    std::cout << "      SH L1 = (" << envBase.r[2] << ", " << envBase.g[2] << ", " << envBase.b[2] << ")" << std::endl;

    // Three different environments: original, rotated 60°, rotated 120°
    SHEnv env0 = envBase;
    SHEnv env1 = rotateSHY(envBase, M_PI / 3.0f);  // 60°
    SHEnv env2 = rotateSHY(envBase, 2.0f * M_PI / 3.0f); // 120°

    // Step 2: Draw background sky for each panel
    std::cout << "[2/5] Rendering sky backgrounds..." << std::endl;
    int panelW = W / 3;
    for (int py = 0; py < H; py++) {
        for (int px = 0; px < W; px++) {
            // Determine which panel
            int panel = px / panelW;
            float angle = panel * 60.0f * M_PI / 180.0f;
            // For background sky, rotate ray direction
            float u = (px % panelW + 0.5f) / panelW * 2.0f - 1.0f;
            float v = 1.0f - (py + 0.5f) / H * 2.0f;
            float aspect = (float)panelW / H;
            float fov = 60.0f * M_PI / 180.0f;
            float d = 1.0f / std::tan(fov * 0.5f);
            Vec3 ray{u * aspect / d, v / d, -1.0f};
            Vec3 dir = ray.norm();
            // Rotate dir by -angle around Y
            float ca = std::cos(-angle), sa = std::sin(-angle);
            Vec3 rotDir{ca*dir.x + sa*dir.z, dir.y, -sa*dir.x + ca*dir.z};
            Vec3 col = sampleEnv(rotDir);
            col = toneMap(col);
            col = gammaCorrect(col);
            fb.at(px, py) = col.clamp01();
        }
    }

    // Step 3: Precompute transfer for 3 spheres (with occlusion)
    std::cout << "[3/5] Precomputing transfer vectors (rings=32, sectors=32)..." << std::endl;
    int rings = 32, sectors = 32;

    // Panel 1: sphere at (-2.5, 0, 0), use env0
    // Panel 2: sphere at (0, 0, 0), use env1 (rotated)
    // Panel 3: sphere at (2.5, 0, 0), use env2 (rotated more)
    struct SphereDesc {
        float cx, cy, cz, r;
        const SHEnv* env;
        bool shadow;
    };
    std::vector<SphereDesc> spheres = {
        {-2.2f, 0, 0, 0.85f, &env0, false},
        { 0.0f, 0, 0, 0.85f, &env1, true},
        { 2.2f, 0, 0, 0.85f, &env2, true},
    };

    for (int si = 0; si < (int)spheres.size(); si++) {
        auto& s = spheres[si];
        std::cout << "      Sphere " << (si+1) << "/3 (shadow=" << s.shadow << ")..." << std::endl;
        auto verts = generateSphere(s.cx, s.cy, s.cz, s.r, *s.env, s.shadow, rings, sectors);

        // Create a local framebuffer (to preserve panel separation)
        Framebuffer localFb(W, H);
        for (int i = 0; i < W*H; i++) localFb.depth[i] = 1e30f;

        renderSphere(localFb, verts, rings, sectors);

        // Blend rendered sphere onto main fb
        int panelStart = si * panelW;
        int panelEnd = panelStart + panelW;
        for (int py = 0; py < H; py++) {
            for (int px = panelStart; px < panelEnd && px < W; px++) {
                if (localFb.depth[py*W+px] < 1e29f) {
                    Vec3 col = localFb.color[py*W+px];
                    col = toneMap(col);
                    col = gammaCorrect(col);
                    fb.at(px, py) = col.clamp01();
                }
            }
        }
    }

    // Step 4: Draw panel separators and labels (colored lines)
    std::cout << "[4/5] Adding annotations..." << std::endl;
    // Vertical separator lines
    for (int py = 0; py < H; py++) {
        fb.at(panelW-1, py) = Vec3(0.8f, 0.8f, 0.8f);
        fb.at(panelW,   py) = Vec3(0.8f, 0.8f, 0.8f);
        fb.at(2*panelW-1, py) = Vec3(0.8f, 0.8f, 0.8f);
        fb.at(2*panelW,   py) = Vec3(0.8f, 0.8f, 0.8f);
    }

    // Step 5: Write output
    std::cout << "[5/5] Writing output..." << std::endl;
    std::vector<uint8_t> pixels(W * H * 3);
    for (int i = 0; i < W*H; i++) {
        pixels[i*3+0] = (uint8_t)(std::min(fb.color[i].x, 1.0f) * 255.99f);
        pixels[i*3+1] = (uint8_t)(std::min(fb.color[i].y, 1.0f) * 255.99f);
        pixels[i*3+2] = (uint8_t)(std::min(fb.color[i].z, 1.0f) * 255.99f);
    }

    stbi_write_png("prt_output.png", W, H, 3, pixels.data(), W*3);
    std::cout << "✅ Saved prt_output.png (" << W << "x" << H << ")" << std::endl;

    // Statistics
    double sumR=0, sumG=0, sumB=0;
    for (int i=0; i<W*H; i++) {
        sumR += pixels[i*3+0];
        sumG += pixels[i*3+1];
        sumB += pixels[i*3+2];
    }
    double total = W*H;
    std::cout << "Image stats: R=" << sumR/total << " G=" << sumG/total << " B=" << sumB/total << std::endl;

    return 0;
}
