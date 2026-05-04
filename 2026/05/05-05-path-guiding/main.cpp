/**
 * Path Guiding Renderer with Spatial SD-Tree Caching
 * 
 * 技术要点:
 * 1. SD-Tree (Spatial-Directional Tree) 用于学习场景中光的分布
 * 2. 每个空间体素维护一个 Q-Table 记录方向采样权重
 * 3. 自适应重要性采样: 初始随机采样 → 学习分布 → 引导采样
 * 4. 多Pass迭代: 奇数Pass学习，偶数Pass渲染
 * 5. 场景: Cornell Box + 复杂光路（焦散、间接光）
 * 
 * 参考: 
 * - "Practical Path Guiding" (Müller et al. 2017)
 * - "Variance-Aware Path Guiding" (Rath et al. 2020)
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <random>
#include <algorithm>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <numeric>
#include <functional>

// ─────────────────────────────────────────────────────────────────────────────
// Math
// ─────────────────────────────────────────────────────────────────────────────

static const float PI = 3.14159265358979f;
static const float INV_PI = 1.0f / PI;
static const float TWO_PI = 2.0f * PI;
static const float EPS = 1e-4f;

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b)  const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator*=(float t) { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3  cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float lengthSq() const { return x*x + y*y + z*z; }
    float length()   const { return std::sqrt(lengthSq()); }
    Vec3  normalize() const {
        float l = length();
        return l > 1e-8f ? *this / l : Vec3(0,0,0);
    }
    bool  isNan() const { return std::isnan(x)||std::isnan(y)||std::isnan(z); }
    float maxComp() const { return std::max({x,y,z}); }
};

Vec3 operator*(float t, const Vec3& v) { return v*t; }

// ─────────────────────────────────────────────────────────────────────────────
// RNG
// ─────────────────────────────────────────────────────────────────────────────

struct RNG {
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist{0.0f, 1.0f};
    RNG(uint32_t seed=42) : gen(seed) {}
    float next() { return dist(gen); }
    Vec3 uniformSphere() {
        float u = next(), v = next();
        float theta = std::acos(1.0f - 2.0f*u);
        float phi   = TWO_PI * v;
        float st = std::sin(theta);
        return { st*std::cos(phi), std::cos(theta), st*std::sin(phi) };
    }
    Vec3 cosineHemisphere(const Vec3& n) {
        float u = next(), v = next();
        float r = std::sqrt(u);
        float phi = TWO_PI * v;
        float x = r * std::cos(phi);
        float z = r * std::sin(phi);
        float y = std::sqrt(std::max(0.0f, 1.0f - u));
        // Build TBN
        Vec3 t, b;
        if (std::abs(n.x) > 0.9f) t = Vec3(0,1,0).cross(n).normalize();
        else                       t = Vec3(1,0,0).cross(n).normalize();
        b = n.cross(t);
        return (t*x + n*y + b*z).normalize();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Ray & Hit
// ─────────────────────────────────────────────────────────────────────────────

struct Ray {
    Vec3 o, d;
    Ray(const Vec3& o, const Vec3& d) : o(o), d(d.normalize()) {}
    Vec3 at(float t) const { return o + d*t; }
};

struct Material {
    enum Type { DIFF, SPEC, REFL_REFR } type;
    Vec3 albedo;    // diffuse color
    Vec3 emission;  // light emission
    float ior;      // index of refraction (for glass)
    Material() : type(DIFF), albedo(0.8f,0.8f,0.8f), emission(0,0,0), ior(1.5f) {}
    Material(Type t, Vec3 a, Vec3 e={}, float ior=1.5f)
        : type(t), albedo(a), emission(e), ior(ior) {}
};

struct HitInfo {
    float t;
    Vec3  pos;
    Vec3  normal;
    bool  hit;
    int   matId;
    HitInfo() : t(1e30f), hit(false), matId(-1) {}
};

// ─────────────────────────────────────────────────────────────────────────────
// Scene Primitives
// ─────────────────────────────────────────────────────────────────────────────

struct Sphere {
    Vec3  center;
    float radius;
    int   matId;
    bool intersect(const Ray& r, float tmin, float tmax, HitInfo& hit) const {
        Vec3  oc = r.o - center;
        float a = r.d.dot(r.d);
        float b = 2.0f * oc.dot(r.d);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if (disc < 0) return false;
        float sq = std::sqrt(disc);
        float t = (-b - sq) / (2*a);
        if (t < tmin || t > tmax) {
            t = (-b + sq) / (2*a);
            if (t < tmin || t > tmax) return false;
        }
        hit.t     = t;
        hit.pos   = r.at(t);
        hit.normal= (hit.pos - center) / radius;
        hit.matId = matId;
        hit.hit   = true;
        return true;
    }
};

struct Plane {
    Vec3  normal;
    float d;        // n·x = d
    int   matId;
    bool intersect(const Ray& r, float tmin, float tmax, HitInfo& hit) const {
        float denom = normal.dot(r.d);
        if (std::abs(denom) < 1e-6f) return false;
        float t = (d - normal.dot(r.o)) / denom;
        if (t < tmin || t > tmax) return false;
        hit.t     = t;
        hit.pos   = r.at(t);
        hit.normal= (denom < 0) ? normal : -normal;
        hit.matId = matId;
        hit.hit   = true;
        return true;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Scene
// ─────────────────────────────────────────────────────────────────────────────

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane>  planes;
    std::vector<Material> mats;

    int addMat(const Material& m) {
        mats.push_back(m);
        return (int)mats.size()-1;
    }

    void buildCornellBox() {
        // Materials
        int mWhite  = addMat({Material::DIFF, {0.73f,0.73f,0.73f}});
        int mRed    = addMat({Material::DIFF, {0.65f,0.05f,0.05f}});
        int mGreen  = addMat({Material::DIFF, {0.12f,0.45f,0.15f}});
        int mLight  = addMat({Material::DIFF, {0.78f,0.78f,0.78f}, {12,12,12}});
        int mMirror = addMat({Material::SPEC, {0.95f,0.95f,0.95f}});
        int mGlass  = addMat({Material::REFL_REFR, {0.99f,0.99f,0.99f},{},1.5f});
        int mGold   = addMat({Material::SPEC, {0.8f,0.6f,0.2f}});

        // Cornell Box walls (box 5x5x5, centered at origin, z: -5..0)
        planes.push_back({{0,1,0}, -2.5f, mWhite});  // floor
        planes.push_back({{0,-1,0}, -2.5f, mWhite}); // ceiling
        planes.push_back({{0,0,1},  -5.0f, mWhite});  // back wall
        planes.push_back({{1,0,0}, -2.5f, mRed});    // left wall
        planes.push_back({{-1,0,0}, -2.5f, mGreen});  // right wall

        // Area light on ceiling (sphere approximation)
        spheres.push_back({{0, 2.1f, -2.5f}, 0.5f, mLight});

        // Glass ball (complex refraction paths)
        spheres.push_back({{-0.7f, -1.7f, -3.2f}, 0.8f, mGlass});

        // Mirror ball
        spheres.push_back({{1.0f, -1.8f, -2.8f}, 0.7f, mMirror});

        // Gold sphere for indirect lighting
        spheres.push_back({{0.0f, -1.9f, -4.2f}, 0.6f, mGold});
    }

    bool intersect(const Ray& r, HitInfo& hit) const {
        hit = HitInfo();
        for (const auto& s : spheres) {
            HitInfo tmp;
            if (s.intersect(r, EPS, hit.t, tmp)) {
                hit = tmp;
            }
        }
        for (const auto& p : planes) {
            HitInfo tmp;
            if (p.intersect(r, EPS, hit.t, tmp)) {
                hit = tmp;
            }
        }
        return hit.hit;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// SD-Tree: Spatial-Directional Caching for Path Guiding
// ─────────────────────────────────────────────────────────────────────────────

// Direction quantization: map unit direction to integer index
// Using a simple 2D grid on (theta, phi)
static const int D_THETA = 16;
static const int D_PHI   = 32;
static const int D_SIZE  = D_THETA * D_PHI;

struct DirCache {
    std::array<float, D_SIZE> weights;
    float totalWeight;
    int   sampleCount;

    DirCache() : weights{}, totalWeight(0.0f), sampleCount(0) {
        weights.fill(1.0f / D_SIZE); // uniform init
        totalWeight = 1.0f;
    }

    // Convert direction to bin index
    static int dirToBin(const Vec3& d) {
        // Clamp to valid range
        float cosTheta = std::max(-1.0f, std::min(1.0f, d.y));
        float theta = std::acos(cosTheta);
        float phi   = std::atan2(d.z, d.x);
        if (phi < 0) phi += TWO_PI;

        int ti = (int)(theta / PI * D_THETA);
        int pi = (int)(phi / TWO_PI * D_PHI);
        ti = std::max(0, std::min(D_THETA-1, ti));
        pi = std::max(0, std::min(D_PHI-1, pi));
        return ti * D_PHI + pi;
    }

    // Sample a direction proportional to weights (CDF sampling)
    Vec3 sample(RNG& rng, float& pdf) const {
        // Build CDF
        float r = rng.next() * totalWeight;
        float cum = 0.0f;
        int idx = 0;
        for (int i = 0; i < D_SIZE; i++) {
            cum += weights[i];
            if (cum >= r) { idx = i; break; }
        }
        if (idx >= D_SIZE) idx = D_SIZE - 1;

        int ti = idx / D_PHI;
        int pi = idx % D_PHI;

        // Sample uniform within bin
        float thetaMin = (float)ti / D_THETA * PI;
        float thetaMax = (float)(ti+1) / D_THETA * PI;
        float phiMin   = (float)pi / D_PHI * TWO_PI;
        float phiMax   = (float)(pi+1) / D_PHI * TWO_PI;

        float theta = thetaMin + rng.next() * (thetaMax - thetaMin);
        float phi   = phiMin   + rng.next() * (phiMax   - phiMin);

        float sinTheta = std::sin(theta);
        Vec3 dir(sinTheta * std::cos(phi), std::cos(theta), sinTheta * std::sin(phi));

        // PDF = weight / (totalWeight * solid_angle_of_bin)
        float solidAngle = (std::cos(thetaMin) - std::cos(thetaMax)) * (phiMax - phiMin);
        solidAngle = std::max(1e-6f, solidAngle);
        pdf = (weights[idx] / totalWeight) / solidAngle;
        pdf = std::max(1e-6f, pdf);

        return dir.normalize();
    }

    // Evaluate PDF for a given direction
    float evalPdf(const Vec3& d) const {
        int idx = dirToBin(d);
        int ti  = idx / D_PHI;
        float thetaMin = (float)ti / D_THETA * PI;
        float thetaMax = (float)(ti+1) / D_THETA * PI;
        int pi = idx % D_PHI;
        float phiMin = (float)pi / D_PHI * TWO_PI;
        float phiMax = (float)(pi+1) / D_PHI * TWO_PI;

        float solidAngle = (std::cos(thetaMin) - std::cos(thetaMax)) * (phiMax - phiMin);
        solidAngle = std::max(1e-6f, solidAngle);
        float w = weights[idx] / totalWeight;
        return std::max(1e-6f, w / solidAngle);
    }

    // Update cache with a radiance sample arriving from direction d
    void update(const Vec3& d, float radiance) {
        if (radiance <= 0.0f) return;
        int idx = dirToBin(d);
        // Exponential moving average
        float alpha = std::max(0.01f, 1.0f / (1.0f + sampleCount * 0.01f));
        weights[idx] = (1.0f - alpha) * weights[idx] + alpha * radiance;
        // Recompute total
        totalWeight = 0.0f;
        for (float w : weights) totalWeight += w;
        if (totalWeight < 1e-8f) {
            weights.fill(1.0f / D_SIZE);
            totalWeight = 1.0f;
        }
        sampleCount++;
    }
};

// Spatial grid for SD-Tree
static const int GRID_RES = 8; // 8x8x8 voxels over scene AABB
static const int GRID_TOTAL = GRID_RES * GRID_RES * GRID_RES;

struct SDTree {
    Vec3 aabbMin, aabbMax;
    std::vector<DirCache> caches;

    SDTree() 
        : aabbMin(-2.5f, -2.5f, -5.0f)
        , aabbMax( 2.5f,  2.5f,  0.0f)
        , caches(GRID_TOTAL)
    {}

    int posToIdx(const Vec3& p) const {
        Vec3 size = aabbMax - aabbMin;
        float gx = (p.x - aabbMin.x) / size.x * GRID_RES;
        float gy = (p.y - aabbMin.y) / size.y * GRID_RES;
        float gz = (p.z - aabbMin.z) / size.z * GRID_RES;
        int ix = std::max(0, std::min(GRID_RES-1, (int)gx));
        int iy = std::max(0, std::min(GRID_RES-1, (int)gy));
        int iz = std::max(0, std::min(GRID_RES-1, (int)gz));
        return ix * GRID_RES * GRID_RES + iy * GRID_RES + iz;
    }

    DirCache& get(const Vec3& p) {
        return caches[posToIdx(p)];
    }

    const DirCache& get(const Vec3& p) const {
        return caches[posToIdx(p)];
    }

    void record(const Vec3& pos, const Vec3& dir, float radiance) {
        get(pos).update(dir, radiance);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Path Tracer
// ─────────────────────────────────────────────────────────────────────────────

struct PathTracer {
    const Scene& scene;
    SDTree&      sdTree;
    int          maxDepth;
    float        guidingFraction; // fraction of bounces guided by SD-Tree

    PathTracer(const Scene& s, SDTree& t, int depth=8, float gf=0.5f)
        : scene(s), sdTree(t), maxDepth(depth), guidingFraction(gf) {}

    // Fresnel for dielectric (Schlick)
    float fresnelDielectric(float cosThetaI, float eta) const {
        float sinThetaTSq = eta*eta * (1.0f - cosThetaI*cosThetaI);
        if (sinThetaTSq >= 1.0f) return 1.0f; // TIR
        float cosThetaT = std::sqrt(std::max(0.0f, 1.0f - sinThetaTSq));
        float rs = (cosThetaI - eta*cosThetaT) / (cosThetaI + eta*cosThetaT);
        float rp = (eta*cosThetaI - cosThetaT) / (eta*cosThetaI + cosThetaT);
        return 0.5f * (rs*rs + rp*rp);
    }

    // Trace one path, accumulate radiance
    // If learningPass=true: update SD-Tree along path
    Vec3 trace(Ray ray, RNG& rng, bool learningPass) {
        Vec3 L(0,0,0);
        Vec3 throughput(1,1,1);

        for (int depth = 0; depth < maxDepth; depth++) {
            HitInfo hit;
            if (!scene.intersect(ray, hit)) {
                // Environment (none in Cornell box)
                break;
            }

            const Material& mat = scene.mats[hit.matId];

            // Add emission
            if (mat.emission.maxComp() > 0) {
                L += throughput * mat.emission;
                break;
            }

            Vec3 pos = hit.pos;
            Vec3 n   = hit.normal.normalize();

            // --- Handle material ---
            Vec3 newDir;
            float pdf = 1.0f;
            Vec3  brdf;

            if (mat.type == Material::SPEC) {
                // Perfect mirror
                newDir = (ray.d - n * 2.0f * ray.d.dot(n)).normalize();
                pdf  = 1.0f;
                brdf = mat.albedo;

            } else if (mat.type == Material::REFL_REFR) {
                // Glass: Fresnel blend
                bool  inside    = ray.d.dot(n) > 0;
                Vec3  nl        = inside ? -n : n;
                float eta       = inside ? mat.ior : 1.0f/mat.ior;
                float cosThetaI = std::abs(ray.d.dot(nl));
                float F = fresnelDielectric(cosThetaI, 1.0f/eta);

                if (rng.next() < F) {
                    // Reflect
                    newDir = (ray.d - nl * 2.0f * ray.d.dot(nl)).normalize();
                } else {
                    // Refract
                    float etaI = inside ? mat.ior : 1.0f;
                    float etaT = inside ? 1.0f : mat.ior;
                    float etaR = etaI / etaT;
                    float cosI = cosThetaI;
                    float sinT2 = etaR*etaR*(1.0f - cosI*cosI);
                    if (sinT2 >= 1.0f) {
                        newDir = (ray.d - nl * 2.0f * ray.d.dot(nl)).normalize(); // TIR
                    } else {
                        float cosT = std::sqrt(1.0f - sinT2);
                        newDir = (ray.d * etaR + nl * (etaR*cosI - cosT)).normalize();
                    }
                }
                pdf  = 1.0f;
                brdf = mat.albedo;

            } else {
                // Diffuse: path guiding or cosine hemisphere
                Vec3 guidedDir;
                float guidedPdf = 0.0f;
                bool  useGuiding = (depth >= 1) && 
                                   (rng.next() < guidingFraction) &&
                                   (sdTree.get(pos).sampleCount > 20);

                if (useGuiding) {
                    guidedDir = sdTree.get(pos).sample(rng, guidedPdf);
                    // Must be in upper hemisphere relative to normal
                    if (guidedDir.dot(n) <= 0) {
                        useGuiding = false;
                    }
                }

                if (useGuiding) {
                    // MIS: combine guiding and cosine sampling
                    float cosinePdf = std::max(0.0f, guidedDir.dot(n)) * INV_PI;
                    float misPdf = 0.5f * guidedPdf + 0.5f * cosinePdf;
                    newDir = guidedDir;
                    pdf  = std::max(1e-6f, misPdf);
                    brdf = mat.albedo * INV_PI;
                } else {
                    // Standard cosine-weighted hemisphere sampling
                    newDir  = rng.cosineHemisphere(n);
                    float cosTheta = std::max(0.0f, newDir.dot(n));
                    pdf  = std::max(1e-6f, cosTheta * INV_PI);
                    brdf = mat.albedo * INV_PI;
                }

                float cosTheta = std::max(0.0f, newDir.dot(n));
                throughput = throughput * brdf * cosTheta / pdf;
            }

            // Russian roulette
            if (depth >= 3) {
                float q = std::max(0.05f, 1.0f - throughput.maxComp());
                if (rng.next() < q) break;
                throughput = throughput * (1.0f / (1.0f - q));
            }

            // Learning pass: record partial path contribution estimate
            if (learningPass && mat.type == Material::DIFF) {
                // We'll estimate radiance by tracing a short sub-path
                // Simple heuristic: use current throughput magnitude as proxy
                float estimate = throughput.maxComp() * 0.3f;
                sdTree.record(pos, newDir, estimate);
            }

            ray = Ray(pos, newDir);
        }

        return L;
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Camera & Film
// ─────────────────────────────────────────────────────────────────────────────

struct Camera {
    Vec3  eye, lower_left, horizontal, vertical;

    Camera(const Vec3& lookFrom, const Vec3& lookAt, const Vec3& up,
           float vfov, float aspect) {
        float theta = vfov * PI / 180.0f;
        float h     = std::tan(theta * 0.5f);
        float vp_h  = 2.0f * h;
        float vp_w  = aspect * vp_h;

        Vec3 w = (lookFrom - lookAt).normalize();
        Vec3 u = up.cross(w).normalize();
        Vec3 v = w.cross(u);

        eye         = lookFrom;
        horizontal  = u * vp_w;
        vertical    = v * vp_h;
        lower_left  = eye - horizontal*0.5f - vertical*0.5f - w;
    }

    Ray getRay(float s, float t, RNG& rng) const {
        // Slightly jittered
        float jx = (rng.next() - 0.5f) * 0.001f;
        float jy = (rng.next() - 0.5f) * 0.001f;
        Vec3 dir = lower_left + horizontal*(s+jx) + vertical*(t+jy) - eye;
        return Ray(eye, dir);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Tone Mapping & Gamma
// ─────────────────────────────────────────────────────────────────────────────

float aces(float x) {
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    return std::max(0.0f, std::min(1.0f, (x*(a*x+b))/(x*(c*x+d)+e)));
}

uint8_t toU8(float f) {
    return (uint8_t)(aces(f) * 255.0f + 0.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Overlay: draw progress bar / label on image
// ─────────────────────────────────────────────────────────────────────────────

// Tiny bitmap font for digits and letters (5x7)
static const uint8_t FONT5x7[128][7] = {
    // Only define printable ASCII we need
};

void drawPixel(std::vector<uint8_t>& img, int W, int H,
               int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    int idx = (y*W + x) * 3;
    img[idx+0] = r; img[idx+1] = g; img[idx+2] = b;
}

void drawRect(std::vector<uint8_t>& img, int W, int H,
              int x0, int y0, int w, int h,
              uint8_t r, uint8_t g, uint8_t b) {
    for (int y = y0; y < y0+h; y++)
        for (int x = x0; x < x0+w; x++)
            drawPixel(img, W, H, x, y, r, g, b);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    // Image
    const int W = 600, H = 600;
    const int SPP_LEARN  = 16;   // samples per pixel for learning pass
    const int SPP_RENDER = 64;   // samples per pixel for rendering pass
    const int NUM_PASSES = 4;    // alternating learn/render iterations

    std::vector<Vec3> accumA(W * H, Vec3(0,0,0)); // accumulator for guided rendering
    std::vector<Vec3> accumB(W * H, Vec3(0,0,0)); // accumulator for reference (unguided)

    // Build scene
    Scene scene;
    scene.buildCornellBox();

    // SD-Tree
    SDTree sdTree;

    // Camera: look into Cornell box
    Camera cam(
        Vec3(0, 0, 3.5f),    // eye
        Vec3(0, 0, -2.5f),   // look at center of box
        Vec3(0, 1, 0),       // up
        50.0f,               // FOV
        (float)W/H
    );

    std::cout << "Path Guiding Renderer\n";
    std::cout << "Resolution: " << W << "x" << H << "\n";
    std::cout << "Learning SPP: " << SPP_LEARN << ", Rendering SPP: " << SPP_RENDER << "\n";
    std::cout << "Passes: " << NUM_PASSES << "\n\n";

    int totalLearningPasses  = 0;
    int totalRenderingSamples = 0;

    for (int pass = 0; pass < NUM_PASSES; pass++) {
        bool isLearning = (pass % 2 == 0);
        int  spp        = isLearning ? SPP_LEARN : SPP_RENDER;

        std::cout << "Pass " << (pass+1) << "/" << NUM_PASSES
                  << " (" << (isLearning ? "LEARNING" : "RENDERING") 
                  << ", spp=" << spp << ")... " << std::flush;

        if (isLearning) {
            // Learning pass: trace paths and update SD-Tree
            PathTracer pt(scene, sdTree, 8, 0.0f); // no guiding during learning
            for (int j = 0; j < H; j++) {
                for (int i = 0; i < W; i++) {
                    RNG rng((pass*H+j)*W + i + 137);
                    for (int s = 0; s < spp; s++) {
                        float u = (i + rng.next()) / W;
                        float v = (j + rng.next()) / H;
                        Ray ray = cam.getRay(u, 1.0f-v, rng);
                        pt.trace(ray, rng, /*learningPass=*/true);
                    }
                }
            }
            totalLearningPasses++;
        } else {
            // Rendering pass: use guided sampling
            float guidingFrac = std::min(0.9f, 0.3f * pass);
            PathTracer pt(scene, sdTree, 10, guidingFrac);
            for (int j = 0; j < H; j++) {
                for (int i = 0; i < W; i++) {
                    RNG rng((pass*H+j)*W + i + 42);
                    for (int s = 0; s < SPP_RENDER; s++) {
                        float u = (i + rng.next()) / W;
                        float v = (j + rng.next()) / H;
                        Ray ray = cam.getRay(u, 1.0f-v, rng);
                        Vec3 L = pt.trace(ray, rng, /*learningPass=*/false);
                        // Clamp fireflies
                        float maxL = 20.0f;
                        if (L.maxComp() > maxL) L = L * (maxL / L.maxComp());
                        accumA[j*W+i] += L;
                    }
                    totalRenderingSamples++;
                }
            }
        }
        std::cout << "done\n";
    }

    // Also render reference (pure cosine, no guiding)
    std::cout << "Rendering reference (unguided)... " << std::flush;
    {
        PathTracer pt(scene, sdTree, 10, 0.0f);
        for (int j = 0; j < H; j++) {
            for (int i = 0; i < W; i++) {
                RNG rng(99999 + j*W + i);
                for (int s = 0; s < SPP_RENDER; s++) {
                    float u = (i + rng.next()) / W;
                    float v = (j + rng.next()) / H;
                    Ray ray = cam.getRay(u, 1.0f-v, rng);
                    Vec3 L = pt.trace(ray, rng, false);
                    float maxL = 20.0f;
                    if (L.maxComp() > maxL) L = L * (maxL / L.maxComp());
                    accumB[j*W+i] += L;
                }
            }
        }
        std::cout << "done\n";
    }

    // ─── Build comparison image ───────────────────────────────────────────
    // Left half: guided | Right half: reference | Center: thin divider
    const int OUT_W = W * 2 + 4;
    const int OUT_H = H + 60; // extra row for legend
    std::vector<uint8_t> outImg(OUT_W * OUT_H * 3, 20);

    int renderSPP_denom = (NUM_PASSES / 2) * SPP_RENDER;
    if (renderSPP_denom == 0) renderSPP_denom = 1;

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            // Guided
            Vec3 cA = accumA[j*W+i] / (float)renderSPP_denom;
            int idxA = (j * OUT_W + i) * 3;
            outImg[idxA+0] = toU8(cA.x);
            outImg[idxA+1] = toU8(cA.y);
            outImg[idxA+2] = toU8(cA.z);

            // Reference
            Vec3 cB = accumB[j*W+i] / (float)SPP_RENDER;
            int idxB = (j * OUT_W + (W + 4 + i)) * 3;
            outImg[idxB+0] = toU8(cB.x);
            outImg[idxB+1] = toU8(cB.y);
            outImg[idxB+2] = toU8(cB.z);
        }
        // Divider
        for (int d = 0; d < 4; d++) {
            int idx = (j * OUT_W + W + d) * 3;
            outImg[idx+0] = outImg[idx+1] = outImg[idx+2] = 200;
        }
    }

    // Legend bar
    // Left label: "Path Guided" (blue-ish bg)
    drawRect(outImg, OUT_W, OUT_H, 0, H, W, 60, 30, 60, 100);
    // Right label: "Unguided" (dark bg)
    drawRect(outImg, OUT_W, OUT_H, W+4, H, W, 60, 50, 30, 30);

    // Draw colored indicator bars as labels
    // "Path Guided" indicator - cyan bar
    drawRect(outImg, OUT_W, OUT_H, 20, H+20, 40, 8, 0, 200, 220);
    // "Unguided" indicator - orange bar
    drawRect(outImg, OUT_W, OUT_H, W+24, H+20, 40, 8, 220, 140, 0);

    // Draw small progress pixels representing SD-Tree usage
    // Visualize guiding distribution in a corner strip
    for (int gy = 0; gy < 32; gy++) {
        for (int gx = 0; gx < 32; gx++) {
            // Sample the SD-Tree at center of scene
            Vec3 samplePos(0, 0, -2.5f);
            const DirCache& dc = sdTree.get(samplePos);
            int bin = gy * (D_PHI * D_THETA / 32) / 1 + gx % D_SIZE;
            bin = bin % D_SIZE;
            float w = dc.weights[bin] / (dc.totalWeight / D_SIZE + 1e-8f);
            w = std::min(1.0f, w * 2.0f);
            uint8_t val = (uint8_t)(w * 255);
            // Draw in bottom-right legend
            int px = W + 4 + W - 50 + gx;
            int py = H + 10 + gy;
            drawPixel(outImg, OUT_W, OUT_H, px, py, val, val/2, 0);
        }
    }

    // ─── Save output ──────────────────────────────────────────────────────
    const char* filename = "path_guiding_output.png";
    if (!stbi_write_png(filename, OUT_W, OUT_H, 3, outImg.data(), OUT_W*3)) {
        std::cerr << "Failed to write " << filename << "\n";
        return 1;
    }

    // Stats
    long long totalCacheSamples = 0;
    for (const auto& c : sdTree.caches) totalCacheSamples += c.sampleCount;
    int activeVoxels = 0;
    for (const auto& c : sdTree.caches) if (c.sampleCount > 0) activeVoxels++;

    std::cout << "\n=== Results ===\n";
    std::cout << "Output: " << filename << " (" << OUT_W << "x" << OUT_H << ")\n";
    std::cout << "SD-Tree voxels: " << GRID_TOTAL << " total, " 
              << activeVoxels << " active\n";
    std::cout << "Total cache updates: " << totalCacheSamples << "\n";
    std::cout << "Learning passes: " << totalLearningPasses << "\n";
    std::cout << "Rendering samples/pixel: " << renderSPP_denom << " (guided)\n";

    return 0;
}
