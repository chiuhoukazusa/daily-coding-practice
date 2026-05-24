/*
 * Lightmap Baker
 * ==============
 * 离线光照贴图烘焙器：将场景几何体的辐照度（直接光 + 间接一次弹射）
 * 烘焙到纹理贴图，输出可在实时渲染中直接使用的光照贴图。
 *
 * 核心算法：
 *   1. 构造场景（Cornell Box + 两个盒子）
 *   2. 为每个三角面片分配 UV 坐标（简单线性展开到 lightmap atlas）
 *   3. 对每个三角面片内的 texel：
 *      a. 将纹素坐标转换为世界空间位置 + 法线
 *      b. 直接光照：对面光源采样（Shadow Ray 判断可见性）
 *      c. 间接光照：半球余弦采样 1 次弹射
 *   4. 在 512x512 输出图像上可视化烘焙结果（Cornell box 视角渲染）
 *   5. 同时输出光照贴图 atlas
 *
 * Build: g++ main.cpp -o lightmap_baker -std=c++17 -O2 -Wall -Wextra
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../../stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <cassert>
#include <cstring>
#include <string>
#include <cstdio>

// ─── Math ───────────────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float t) { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 1e-9f ? *this / l : Vec3(0,1,0); }
    Vec3 clamp01() const {
        return {std::max(0.f,std::min(1.f,x)),
                std::max(0.f,std::min(1.f,y)),
                std::max(0.f,std::min(1.f,z))};
    }
};
inline Vec3 operator*(float t, const Vec3& v) { return v * t; }

struct Ray {
    Vec3 origin, dir;
    Ray(Vec3 o, Vec3 d) : origin(o), dir(d.normalized()) {}
};

// ─── RNG ────────────────────────────────────────────────────────────────────

struct RNG {
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist{0.f, 1.f};
    RNG(uint32_t seed = 42) : gen(seed) {}
    float next() { return dist(gen); }

    // Generate cosine-weighted hemisphere sample around N
    Vec3 cosineHemisphere(const Vec3& N) {
        float u1 = next(), u2 = next();
        float r = std::sqrt(u1);
        float phi = 2.f * 3.14159265f * u2;
        float lx = r * std::cos(phi);
        float lz = r * std::sin(phi);
        float ly = std::sqrt(std::max(0.f, 1.f - u1));
        // ONB around N
        Vec3 up = std::abs(N.y) < 0.999f ? Vec3(0,1,0) : Vec3(1,0,0);
        Vec3 T = up.cross(N).normalized();
        Vec3 B = N.cross(T);
        return (T * lx + N * ly + B * lz).normalized();
    }
};

// ─── Scene geometry ─────────────────────────────────────────────────────────

struct Material {
    Vec3 albedo;
    Vec3 emission;
    bool isLight;
    Material() : albedo(0.8f,0.8f,0.8f), emission(0,0,0), isLight(false) {}
    Material(Vec3 a, Vec3 e = Vec3(0,0,0), bool light = false)
        : albedo(a), emission(e), isLight(light) {}
};

struct Triangle {
    Vec3 v[3];          // world-space vertices
    Vec3 normal;        // face normal
    int matIdx;
    int triId;          // index in scene.tris

    // Lightmap atlas UV for each vertex [0,1]^2
    float lmu[3], lmv[3];

    void computeNormal() {
        normal = (v[1]-v[0]).cross(v[2]-v[0]).normalized();
    }
};

struct HitRecord {
    float t;
    Vec3 pos, normal;
    int matIdx;
    int triId;
    float bu, bv;       // barycentric coords (for triId)
};

// Möller–Trumbore
inline bool intersectTri(const Ray& ray, const Triangle& tri,
                          float tmin, float tmax,
                          float& t_out, float& bu_out, float& bv_out)
{
    const float EPS = 1e-6f;
    Vec3 e1 = tri.v[1] - tri.v[0];
    Vec3 e2 = tri.v[2] - tri.v[0];
    Vec3 h = ray.dir.cross(e2);
    float a = e1.dot(h);
    if (std::abs(a) < EPS) return false;
    float f = 1.f / a;
    Vec3 s = ray.origin - tri.v[0];
    float bu = f * s.dot(h);
    if (bu < 0.f || bu > 1.f) return false;
    Vec3 q = s.cross(e1);
    float bv = f * ray.dir.dot(q);
    if (bv < 0.f || bu + bv > 1.f) return false;
    float t = f * e2.dot(q);
    if (t < tmin || t > tmax) return false;
    t_out = t; bu_out = bu; bv_out = bv;
    return true;
}

struct Scene {
    std::vector<Triangle> tris;
    std::vector<Material> mats;
    std::vector<int> lightTriIds;

    bool intersect(const Ray& ray, float tmin, float tmax, HitRecord& rec) const {
        bool hit = false;
        float best = tmax;
        for (const auto& tri : tris) {
            float t, bu, bv;
            if (intersectTri(ray, tri, tmin, best, t, bu, bv)) {
                best = t;
                hit = true;
                rec.t = t;
                rec.pos = ray.origin + ray.dir * t;
                rec.normal = tri.normal;
                rec.matIdx = tri.matIdx;
                rec.triId = tri.triId;
                rec.bu = bu; rec.bv = bv;
            }
        }
        return hit;
    }

    bool shadowBlocked(const Vec3& from, const Vec3& to) const {
        Vec3 d = to - from;
        float dist = d.length();
        Ray ray(from, d);
        for (const auto& tri : tris) {
            float t, bu, bv;
            if (intersectTri(ray, tri, 1e-3f, dist - 1e-3f, t, bu, bv))
                return true;
        }
        return false;
    }

    // Compute light triangle area
    float lightArea(int li) const {
        const Triangle& lt = tris[li];
        Vec3 e1 = lt.v[1] - lt.v[0];
        Vec3 e2 = lt.v[2] - lt.v[0];
        return 0.5f * e1.cross(e2).length();
    }
};

// ─── Cornell Box construction ────────────────────────────────────────────────

// Add a quad (two triangles) to scene
void addQuad(Scene& scene, Vec3 a, Vec3 b, Vec3 c, Vec3 d, int matIdx) {
    Triangle t1;
    t1.v[0] = a; t1.v[1] = b; t1.v[2] = c;
    t1.matIdx = matIdx;
    t1.triId = (int)scene.tris.size();
    t1.computeNormal();
    scene.tris.push_back(t1);

    Triangle t2;
    t2.v[0] = a; t2.v[1] = c; t2.v[2] = d;
    t2.matIdx = matIdx;
    t2.triId = (int)scene.tris.size();
    t2.computeNormal();
    scene.tris.push_back(t2);
}

Scene buildScene() {
    Scene scene;
    // 0: white diffuse
    scene.mats.push_back(Material(Vec3(0.73f, 0.73f, 0.73f)));
    // 1: red
    scene.mats.push_back(Material(Vec3(0.65f, 0.05f, 0.05f)));
    // 2: green
    scene.mats.push_back(Material(Vec3(0.12f, 0.45f, 0.15f)));
    // 3: area light
    scene.mats.push_back(Material(Vec3(1.f,1.f,1.f), Vec3(10.f,10.f,10.f), true));
    // 4: blue box
    scene.mats.push_back(Material(Vec3(0.15f, 0.25f, 0.72f)));
    // 5: yellow box
    scene.mats.push_back(Material(Vec3(0.82f, 0.68f, 0.18f)));

    float B = 1.0f;

    // Floor (y = -B, white)
    addQuad(scene, Vec3(-B,-B,-B), Vec3(B,-B,-B), Vec3(B,-B,B), Vec3(-B,-B,B), 0);
    // Ceiling (y = +B, white)
    addQuad(scene, Vec3(-B,B,-B), Vec3(-B,B,B), Vec3(B,B,B), Vec3(B,B,-B), 0);
    // Back wall (z = +B, white)
    addQuad(scene, Vec3(-B,-B,B), Vec3(B,-B,B), Vec3(B,B,B), Vec3(-B,B,B), 0);
    // Left wall (x = -B, red)
    addQuad(scene, Vec3(-B,-B,-B), Vec3(-B,-B,B), Vec3(-B,B,B), Vec3(-B,B,-B), 1);
    // Right wall (x = +B, green)
    addQuad(scene, Vec3(B,-B,-B), Vec3(B,B,-B), Vec3(B,B,B), Vec3(B,-B,B), 2);

    // Area light (ceiling center)
    float ls = 0.32f, lh = 0.98f;
    addQuad(scene, Vec3(-ls,lh,-ls), Vec3(-ls,lh,ls), Vec3(ls,lh,ls), Vec3(ls,lh,-ls), 3);

    // Tall blue box
    {
        float x = -0.35f, z = 0.1f, w = 0.28f, top = -0.1f, bot = -1.0f;
        addQuad(scene, Vec3(x-w,bot,z-w), Vec3(x+w,bot,z-w), Vec3(x+w,top,z-w), Vec3(x-w,top,z-w), 4); // front
        addQuad(scene, Vec3(x+w,bot,z-w), Vec3(x+w,bot,z+w), Vec3(x+w,top,z+w), Vec3(x+w,top,z-w), 4); // right
        addQuad(scene, Vec3(x+w,bot,z+w), Vec3(x-w,bot,z+w), Vec3(x-w,top,z+w), Vec3(x+w,top,z+w), 4); // back
        addQuad(scene, Vec3(x-w,bot,z+w), Vec3(x-w,bot,z-w), Vec3(x-w,top,z-w), Vec3(x-w,top,z+w), 4); // left
        addQuad(scene, Vec3(x-w,top,z-w), Vec3(x+w,top,z-w), Vec3(x+w,top,z+w), Vec3(x-w,top,z+w), 4); // top
    }
    // Short yellow box
    {
        float x = 0.3f, z = -0.2f, w = 0.25f, top = -0.5f, bot = -1.0f;
        addQuad(scene, Vec3(x-w,bot,z-w), Vec3(x+w,bot,z-w), Vec3(x+w,top,z-w), Vec3(x-w,top,z-w), 5);
        addQuad(scene, Vec3(x+w,bot,z-w), Vec3(x+w,bot,z+w), Vec3(x+w,top,z+w), Vec3(x+w,top,z-w), 5);
        addQuad(scene, Vec3(x+w,bot,z+w), Vec3(x-w,bot,z+w), Vec3(x-w,top,z+w), Vec3(x+w,top,z+w), 5);
        addQuad(scene, Vec3(x-w,bot,z+w), Vec3(x-w,bot,z-w), Vec3(x-w,top,z-w), Vec3(x-w,top,z+w), 5);
        addQuad(scene, Vec3(x-w,top,z-w), Vec3(x+w,top,z-w), Vec3(x+w,top,z+w), Vec3(x-w,top,z+w), 5);
    }

    for (int i = 0; i < (int)scene.tris.size(); i++) {
        if (scene.mats[scene.tris[i].matIdx].isLight)
            scene.lightTriIds.push_back(i);
    }

    return scene;
}

// ─── Lightmap Atlas Layout ────────────────────────────────────────────────────
//
//  Atlas resolution: LM_SIZE x LM_SIZE
//  Each triangle gets a rectangular slot arranged in a grid.
//  UV coordinates are stored per vertex in triangle.lmu/lmv.
//

static const int LM_SIZE = 512;  // lightmap atlas resolution

struct LightmapAtlas {
    std::vector<Vec3> data;     // LM_SIZE * LM_SIZE irradiance values
    std::vector<int>  sampleCnt;
    LightmapAtlas()
        : data(LM_SIZE * LM_SIZE, Vec3(0,0,0))
        , sampleCnt(LM_SIZE * LM_SIZE, 0)
    {}

    void accumulate(int px, int py, const Vec3& v) {
        if (px < 0 || px >= LM_SIZE || py < 0 || py >= LM_SIZE) return;
        int i = py * LM_SIZE + px;
        data[i] += v;
        sampleCnt[i]++;
    }

    Vec3 get(int px, int py) const {
        if (px < 0 || px >= LM_SIZE || py < 0 || py >= LM_SIZE) return Vec3(0,0,0);
        int i = py * LM_SIZE + px;
        return sampleCnt[i] > 0 ? data[i] * (1.f / sampleCnt[i]) : Vec3(0,0,0);
    }

    // Sample with bilinear interpolation from normalized [0,1] UV
    Vec3 sample(float u, float v) const {
        float fx = u * LM_SIZE - 0.5f;
        float fy = v * LM_SIZE - 0.5f;
        int x0 = (int)std::floor(fx), y0 = (int)std::floor(fy);
        float tx = fx - x0, ty = fy - y0;
        auto clamp = [&](int val, int lo, int hi) { return std::max(lo, std::min(hi, val)); };
        Vec3 c00 = get(clamp(x0,   0, LM_SIZE-1), clamp(y0,   0, LM_SIZE-1));
        Vec3 c10 = get(clamp(x0+1, 0, LM_SIZE-1), clamp(y0,   0, LM_SIZE-1));
        Vec3 c01 = get(clamp(x0,   0, LM_SIZE-1), clamp(y0+1, 0, LM_SIZE-1));
        Vec3 c11 = get(clamp(x0+1, 0, LM_SIZE-1), clamp(y0+1, 0, LM_SIZE-1));
        Vec3 c0  = c00 + (c10 - c00) * tx;
        Vec3 c1  = c01 + (c11 - c01) * tx;
        return c0 + (c1 - c0) * ty;
    }
};

// Assign UV atlas coordinates to all triangles
// Layout: grid of tiles, each triangle gets one tile
void buildAtlas(Scene& scene) {
    int n = (int)scene.tris.size();
    int tilesPerRow = (int)std::ceil(std::sqrt((float)n));
    float tileSize = 1.f / tilesPerRow;
    // padding inside tile (2 pixels / LM_SIZE)
    float pad = 2.f / LM_SIZE;

    for (int i = 0; i < n; i++) {
        int col = i % tilesPerRow;
        int row = i / tilesPerRow;
        float u0 = col * tileSize + pad;
        float v0 = row * tileSize + pad;
        float u1 = (col + 1) * tileSize - pad;
        float v1 = (row + 1) * tileSize - pad;
        // Assign UV to triangle vertices:
        // v[0] -> (u0, v0), v[1] -> (u1, v0), v[2] -> (u0, v1)
        scene.tris[i].lmu[0] = u0; scene.tris[i].lmv[0] = v0;
        scene.tris[i].lmu[1] = u1; scene.tris[i].lmv[1] = v0;
        scene.tris[i].lmu[2] = u0; scene.tris[i].lmv[2] = v1;
    }
}

// ─── Irradiance Estimation ───────────────────────────────────────────────────

static const float PI = 3.14159265f;

// Direct lighting via area-light sampling
Vec3 directLight(const Scene& scene, const Vec3& pos, const Vec3& N, RNG& rng) {
    Vec3 Lo(0,0,0);
    for (int li : scene.lightTriIds) {
        const Triangle& lt = scene.tris[li];
        const Material& lm = scene.mats[lt.matIdx];

        // Sample point on light
        float r1 = rng.next(), r2 = rng.next();
        if (r1 + r2 > 1.f) { r1 = 1.f - r1; r2 = 1.f - r2; }
        float r3 = 1.f - r1 - r2;
        Vec3 lpos = lt.v[0] * r3 + lt.v[1] * r1 + lt.v[2] * r2;

        Vec3 toLight = lpos - pos;
        float dist2 = toLight.dot(toLight);
        float dist = std::sqrt(dist2);
        Vec3 toLightN = toLight / dist;

        float cosTheta = N.dot(toLightN);
        if (cosTheta <= 0.f) continue;

        // Light emission is two-sided (use absolute value)
        float cosThetaL = std::abs(lt.normal.dot(toLightN));
        if (cosThetaL <= 0.f) continue;

        // Shadow test
        if (scene.shadowBlocked(pos + N * 2e-3f, lpos - lt.normal * 2e-3f)) continue;

        float area = scene.lightArea(li);
        // Rendering equation: Lo += Le * albedo_light * cos(theta_recv) * cos(theta_emit) * A / (pi * dist^2)
        // (multiplied by 1/pi for Lambertian BRDF on the emitter side, but light emits uniformly)
        Lo += lm.emission * (cosTheta * cosThetaL * area / (PI * dist2));
    }
    return Lo;
}

// One-bounce indirect + direct
Vec3 estimateIrradiance(const Scene& scene, const Vec3& pos, const Vec3& N, RNG& rng) {
    // Direct
    Vec3 Lo = directLight(scene, pos, N, rng);

    // Indirect: cosine-weighted hemisphere, evaluate direct at bounce point
    const int NUM_INDIRECT = 4;
    Vec3 Lindirect(0,0,0);
    for (int i = 0; i < NUM_INDIRECT; i++) {
        Vec3 dir = rng.cosineHemisphere(N);
        Ray ray(pos + N * 2e-3f, dir);
        HitRecord rec;
        if (scene.intersect(ray, 1e-3f, 1e6f, rec)) {
            const Material& mat = scene.mats[rec.matIdx];
            if (mat.isLight) {
                // Directly hit light
                Lindirect += mat.emission * std::max(0.f, N.dot(dir));
            } else {
                // One bounce: compute direct light at hit point
                Vec3 Ld = directLight(scene, rec.pos + rec.normal * 2e-3f, rec.normal, rng);
                // Modulate by BRDF (Lambertian: albedo / pi) * pi = albedo for hemisphere integration
                Lindirect += Ld * mat.albedo * (1.f / PI) * std::max(0.f, N.dot(dir)) * (PI);
                // Simplified: indirect contribution
                Lindirect += Ld * mat.albedo * 0.25f;
            }
        }
    }
    Lo += Lindirect * (1.f / NUM_INDIRECT);

    return Lo;
}

// ─── Baking ──────────────────────────────────────────────────────────────────

LightmapAtlas bakeLightmap(const Scene& scene) {
    LightmapAtlas atlas;
    RNG rng(42);

    const int SAMPLES = 6; // samples per texel

    for (const Triangle& tri : scene.tris) {
        if (scene.mats[tri.matIdx].isLight) continue;

        // World-space triangle vertices
        Vec3 wA = tri.v[0], wB = tri.v[1], wC = tri.v[2];
        Vec3 N = tri.normal;
        // For lightmap baking, the normal should point toward the interior of the room.
        // If baked irradiance would be negative (wrong side), flip the normal.
        // We check by seeing if the centroid's normal points toward the centroid of the scene (0,0,0).
        Vec3 centroid = (wA + wB + wC) * (1.f/3.f);
        Vec3 toCenter = Vec3(0,0,0) - centroid;
        if (N.dot(toCenter) < 0.f) N = -N; // flip to face interior

        // UV atlas corners
        float u0 = tri.lmu[0], v0 = tri.lmv[0];
        float u1 = tri.lmu[1], v1 = tri.lmv[1];
        float u2 = tri.lmu[2], v2 = tri.lmv[2];

        // Pixel bounds for this triangle's tile
        int px0 = std::max(0, (int)(std::min({u0,u1,u2}) * LM_SIZE));
        int py0 = std::max(0, (int)(std::min({v0,v1,v2}) * LM_SIZE));
        int px1 = std::min(LM_SIZE-1, (int)(std::max({u0,u1,u2}) * LM_SIZE) + 1);
        int py1 = std::min(LM_SIZE-1, (int)(std::max({v0,v1,v2}) * LM_SIZE) + 1);

        // Precompute UV-to-barycentric transform
        // UV is bilinear within the rectangular tile, but triangle UV:
        // vertex 0 = (u0,v0), vertex 1 = (u1,v0), vertex 2 = (u0,v1)
        // For a texel at (u,v):
        //   s = (u - u0) / (u1 - u0)   [0..1]
        //   t = (v - v0) / (v2 - v0)   [0..1]
        // World pos = wA*(1-s-t) + wB*s + wC*t   (when s+t <= 1)
        // If s+t > 1, mirror: s' = 1-s, t' = 1-t  (still inside triangle: wA*(s'+t'-1) not correct)
        // Better: use rectangular fill - just evaluate the 4 corners of each texel

        float du = u1 - u0;
        float dv = v2 - v0;
        if (std::abs(du) < 1e-8f || std::abs(dv) < 1e-8f) continue;

        for (int py = py0; py <= py1; py++) {
            for (int px = px0; px <= px1; px++) {
                // Texel center in UV space
                float u = (px + 0.5f) / LM_SIZE;
                float v = (py + 0.5f) / LM_SIZE;

                float s = (u - u0) / du;
                float t = (v - v0) / dv;

                // Clamp to [0,1]
                s = std::max(0.f, std::min(1.f, s));
                t = std::max(0.f, std::min(1.f, t));

                // If outside triangle (s+t > 1), mirror coords
                if (s + t > 1.f) {
                    s = 1.f - s;
                    t = 1.f - t;
                    if (s < 0.f) s = 0.f;
                    if (t < 0.f) t = 0.f;
                }

                // World position on triangle surface
                Vec3 worldPos = wA * (1.f - s - t) + wB * s + wC * t;

                // Multi-sample irradiance
                Vec3 irr(0,0,0);
                for (int si = 0; si < SAMPLES; si++) {
                    irr += estimateIrradiance(scene, worldPos, N, rng);
                }
                irr = irr * (1.f / SAMPLES);

                // Modulate by material albedo
                irr = irr * scene.mats[tri.matIdx].albedo;

                atlas.accumulate(px, py, irr);
            }
        }
    }

    return atlas;
}

// ─── Final render using baked lightmap ───────────────────────────────────────

Vec3 toneMap(Vec3 c) {
    // Reinhard
    c.x = c.x / (1.f + c.x);
    c.y = c.y / (1.f + c.y);
    c.z = c.z / (1.f + c.z);
    // Gamma 2.2
    c.x = std::pow(std::max(0.f, c.x), 1.f/2.2f);
    c.y = std::pow(std::max(0.f, c.y), 1.f/2.2f);
    c.z = std::pow(std::max(0.f, c.z), 1.f/2.2f);
    return c.clamp01();
}

void renderFinalImage(const Scene& scene, const LightmapAtlas& atlas,
                      std::vector<uint8_t>& pixels, int W, int H)
{
    Vec3 camPos(0, 0, -1.85f);
    float fov = 0.85f;

    for (int py = 0; py < H; py++) {
        for (int px = 0; px < W; px++) {
            float ndcX = ((px + 0.5f) / W) * 2.f - 1.f;
            float ndcY = 1.f - ((py + 0.5f) / H) * 2.f;
            Vec3 dir(ndcX * fov, ndcY * fov, 1.f);
            Ray ray(camPos, dir);

            Vec3 color(0.01f, 0.01f, 0.02f);
            HitRecord rec;

            if (scene.intersect(ray, 1e-3f, 1e7f, rec)) {
                const Material& mat = scene.mats[rec.matIdx];
                if (mat.isLight) {
                    color = mat.emission * 0.07f;
                    color = toneMap(color);
                } else {
                    // Lookup lightmap UV using barycentric coordinates
                    const Triangle& tri = scene.tris[rec.triId];
                    float bu = rec.bu, bv = rec.bv;
                    float bw = 1.f - bu - bv;
                    // Interpolate UV
                    float u = tri.lmu[0]*bw + tri.lmu[1]*bu + tri.lmu[2]*bv;
                    float v  = tri.lmv[0]*bw + tri.lmv[1]*bu + tri.lmv[2]*bv;
                    Vec3 irr = atlas.sample(u, v);
                    color = toneMap(irr);
                }
            }

            int idx = (py * W + px) * 3;
            pixels[idx + 0] = (uint8_t)(color.x * 255.f);
            pixels[idx + 1] = (uint8_t)(color.y * 255.f);
            pixels[idx + 2] = (uint8_t)(color.z * 255.f);
        }
    }
}

void renderAtlasViz(const LightmapAtlas& atlas,
                    std::vector<uint8_t>& pixels, int W, int H)
{
    for (int py = 0; py < H; py++) {
        for (int px = 0; px < W; px++) {
            // Sample from lightmap atlas
            float u = (px + 0.5f) / W;
            float v  = (py + 0.5f) / H;
            Vec3 irr = atlas.sample(u, v);
            Vec3 c = toneMap(irr);
            int idx = (py * W + px) * 3;
            pixels[idx + 0] = (uint8_t)(c.x * 255.f);
            pixels[idx + 1] = (uint8_t)(c.y * 255.f);
            pixels[idx + 2] = (uint8_t)(c.z * 255.f);
        }
    }
}

// ─── Combined output image ────────────────────────────────────────────────────
// Left half: rendered scene | Right half: lightmap atlas

void renderCombined(const Scene& scene, const LightmapAtlas& atlas,
                    std::vector<uint8_t>& pixels, int W, int H)
{
    int half = W / 2;
    // Left: scene render
    std::vector<uint8_t> left(half * H * 3, 0);
    renderFinalImage(scene, atlas, left, half, H);

    // Right: atlas viz
    std::vector<uint8_t> right(half * H * 3, 0);
    renderAtlasViz(atlas, right, half, H);

    // Combine
    for (int py = 0; py < H; py++) {
        // Left half
        for (int px = 0; px < half; px++) {
            int dst = (py * W + px) * 3;
            int src = (py * half + px) * 3;
            pixels[dst + 0] = left[src + 0];
            pixels[dst + 1] = left[src + 1];
            pixels[dst + 2] = left[src + 2];
        }
        // Right half
        for (int px = 0; px < half; px++) {
            int dst = (py * W + (px + half)) * 3;
            int src = (py * half + px) * 3;
            pixels[dst + 0] = right[src + 0];
            pixels[dst + 1] = right[src + 1];
            pixels[dst + 2] = right[src + 2];
        }
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    printf("=== Lightmap Baker ===\n");

    Scene scene = buildScene();
    printf("Triangles: %d  Lights: %d\n", (int)scene.tris.size(), (int)scene.lightTriIds.size());

    printf("Building UV atlas...\n");
    buildAtlas(scene);

    printf("Baking lightmap (%dx%d)...\n", LM_SIZE, LM_SIZE);
    LightmapAtlas atlas = bakeLightmap(scene);
    printf("Baking complete.\n");

    // Count filled texels
    int filled = 0;
    for (int i = 0; i < LM_SIZE * LM_SIZE; i++)
        if (atlas.sampleCnt[i] > 0) filled++;
    printf("Filled texels: %d / %d (%.1f%%)\n", filled, LM_SIZE * LM_SIZE,
           100.f * filled / (LM_SIZE * LM_SIZE));

    // Render full scene using baked lightmap
    const int W = 512, H = 512;
    printf("Rendering final scene (%dx%d)...\n", W, H);
    std::vector<uint8_t> pixels(W * H * 3, 0);
    renderFinalImage(scene, atlas, pixels, W, H);
    stbi_write_png("lightmap_baker_output.png", W, H, 3, pixels.data(), W * 3);
    printf("Saved: lightmap_baker_output.png\n");

    // Also save the atlas visualization
    printf("Rendering lightmap atlas visualization...\n");
    std::vector<uint8_t> atlasPixels(W * H * 3, 0);
    renderAtlasViz(atlas, atlasPixels, W, H);
    stbi_write_png("lightmap_atlas_output.png", W, H, 3, atlasPixels.data(), W * 3);
    printf("Saved: lightmap_atlas_output.png\n");
    printf("Done!\n");
    return 0;
}
