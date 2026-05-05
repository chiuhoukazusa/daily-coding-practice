// Caustics Renderer
// 使用光子映射模拟折射焦散效果
// 场景：玻璃球放在漫反射地板上，点光源照射产生焦散光斑
//
// 算法流程：
//   Pass 1 - 光子追踪：从光源发射光子，追踪折射/反射路径，存储落地的光子
//   Pass 2 - 渲染：直接光照 + 焦散光子密度估计
//
// 编译：g++ main.cpp -o output -std=c++17 -O3 -Wall -Wextra
// 运行：./output
// 输出：caustics_output.png (600x450)

#include "stb_wrapper.h"

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <iostream>
#include <cassert>
#include <numeric>

// ─────────────────────────────────────────────────────────────────────────────
// 基础数学
// ─────────────────────────────────────────────────────────────────────────────

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t)       const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t)       const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    double dot(const Vec3& o)      const { return x*o.x + y*o.y + z*o.z; }
    Vec3   cross(const Vec3& o)    const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    double len2() const { return x*x + y*y + z*z; }
    double len()  const { return std::sqrt(len2()); }
    Vec3   normalize() const { double l = len(); return l > 1e-15 ? (*this)/l : Vec3(0,1,0); }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
};
inline Vec3 operator*(double t, const Vec3& v) { return v*t; }

// ─────────────────────────────────────────────────────────────────────────────
// 随机数
// ─────────────────────────────────────────────────────────────────────────────
struct RNG {
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> dist{0.0, 1.0};
    explicit RNG(uint64_t seed = 42) : gen(seed) {}
    double next() { return dist(gen); }

    Vec3 cosineHemisphere(const Vec3& normal) {
        double u = next(), v = next();
        double r = std::sqrt(u);
        double theta = 2.0 * M_PI * v;
        double lx = r * std::cos(theta);
        double lz = r * std::sin(theta);
        double ly = std::sqrt(std::max(0.0, 1.0 - u));
        Vec3 up = (std::abs(normal.y) < 0.9) ? Vec3(0,1,0) : Vec3(1,0,0);
        Vec3 tangent = normal.cross(up).normalize();
        Vec3 bitangent = normal.cross(tangent);
        return (tangent*lx + normal*ly + bitangent*lz).normalize();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 光线和求交
// ─────────────────────────────────────────────────────────────────────────────
struct Ray {
    Vec3 origin, dir;
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalize()) {}
    Vec3 at(double t) const { return origin + dir*t; }
};

struct HitInfo {
    double t = 1e18;
    Vec3 point, normal;
    bool inside = false;
    int matId = -1;
};

const int MAT_FLOOR = 0;
const int MAT_GLASS = 1;
const int MAT_WALL  = 2;

const Vec3 GLASS_CENTER(0, 0.75, -1.5);
const double GLASS_RADIUS = 0.75;
const double IOR = 1.5;

const Vec3 LIGHT_POS(0, 4.5, -1.5);
const Vec3 LIGHT_COLOR(1.5, 1.3, 0.9);

bool hitSphere(const Ray& ray, const Vec3& center, double radius,
               double tMin, double tMax, HitInfo& hit) {
    Vec3 oc = ray.origin - center;
    double a = ray.dir.dot(ray.dir);
    double b = 2.0 * oc.dot(ray.dir);
    double c = oc.dot(oc) - radius*radius;
    double disc = b*b - 4*a*c;
    if (disc < 0) return false;
    double sqrtDisc = std::sqrt(disc);
    double t = (-b - sqrtDisc) / (2.0*a);
    if (t < tMin || t > tMax) {
        t = (-b + sqrtDisc) / (2.0*a);
        if (t < tMin || t > tMax) return false;
    }
    hit.t = t;
    hit.point = ray.at(t);
    Vec3 outN = (hit.point - center) / radius;
    hit.inside = ray.dir.dot(outN) > 0;
    hit.normal = hit.inside ? -outN : outN;
    return true;
}

bool hitFloor(const Ray& ray, double tMin, double tMax, HitInfo& hit) {
    if (std::abs(ray.dir.y) < 1e-10) return false;
    double t = -ray.origin.y / ray.dir.y;
    if (t < tMin || t > tMax) return false;
    Vec3 p = ray.at(t);
    if (std::abs(p.x) > 4.5 || std::abs(p.z) > 4.5) return false;
    hit.t = t; hit.point = p; hit.normal = Vec3(0,1,0); hit.inside = false;
    return true;
}

bool hitBackWall(const Ray& ray, double tMin, double tMax, HitInfo& hit) {
    if (std::abs(ray.dir.z) < 1e-10) return false;
    double t = (-3.0 - ray.origin.z) / ray.dir.z;
    if (t < tMin || t > tMax) return false;
    Vec3 p = ray.at(t);
    if (std::abs(p.x) > 4.5 || p.y < 0 || p.y > 4.5) return false;
    hit.t = t; hit.point = p; hit.normal = Vec3(0,0,1); hit.inside = false;
    return true;
}

bool sceneHit(const Ray& ray, double tMin, HitInfo& hit) {
    hit.t = 1e18; hit.matId = -1;
    HitInfo tmp;
    if (hitSphere(ray, GLASS_CENTER, GLASS_RADIUS, tMin, hit.t, tmp)) { hit=tmp; hit.matId=MAT_GLASS; }
    if (hitFloor(ray, tMin, hit.t, tmp))    { hit=tmp; hit.matId=MAT_FLOOR; }
    if (hitBackWall(ray, tMin, hit.t, tmp)) { hit=tmp; hit.matId=MAT_WALL; }
    return hit.matId >= 0;
}

// ─────────────────────────────────────────────────────────────────────────────
// Fresnel / Snell
// ─────────────────────────────────────────────────────────────────────────────
bool refract(const Vec3& d, const Vec3& n, double eta, Vec3& refracted) {
    Vec3 unitD = d.normalize();
    double cosI = -unitD.dot(n);
    double sin2T = eta*eta*(1.0 - cosI*cosI);
    if (sin2T > 1.0) return false;
    refracted = (eta*unitD + (eta*cosI - std::sqrt(1.0-sin2T))*n).normalize();
    return true;
}

double schlick(double cosine, double ref_idx) {
    double r0 = (1.0-ref_idx)/(1.0+ref_idx); r0 *= r0;
    return r0 + (1.0-r0)*std::pow(1.0-cosine, 5.0);
}

// ─────────────────────────────────────────────────────────────────────────────
// 光子存储
// ─────────────────────────────────────────────────────────────────────────────
struct Photon { Vec3 pos, power, dir; };

// ─────────────────────────────────────────────────────────────────────────────
// Pass 1：光子追踪
// ─────────────────────────────────────────────────────────────────────────────
void tracePhoton(const Ray& startRay, Vec3 power, int maxBounce,
                 RNG& rng, std::vector<Photon>& photons) {
    Ray ray = startRay;
    for (int b = 0; b < maxBounce; ++b) {
        HitInfo hit;
        if (!sceneHit(ray, 1e-4, hit)) break;

        if (hit.matId == MAT_FLOOR || hit.matId == MAT_WALL) {
            photons.push_back({hit.point, power, ray.dir});
            // 俄罗斯轮盘赌漫反射
            if (rng.next() > 0.5) break;
            power = power * (0.6 / 0.5);
            ray = Ray(hit.point, rng.cosineHemisphere(hit.normal));
        } else if (hit.matId == MAT_GLASS) {
            double nRatio = hit.inside ? IOR : (1.0/IOR);
            Vec3 refDir;
            double cosine = std::abs(ray.dir.dot(hit.normal));
            if (refract(ray.dir, hit.normal, nRatio, refDir)) {
                double fr = schlick(cosine, nRatio);
                if (rng.next() < fr) {
                    Vec3 reflDir = (ray.dir - 2.0*ray.dir.dot(hit.normal)*hit.normal).normalize();
                    ray = Ray(hit.point, reflDir);
                    power = power * fr;
                } else {
                    ray = Ray(hit.point, refDir);
                    power = power * (1.0-fr);
                }
            } else {
                Vec3 reflDir = (ray.dir - 2.0*ray.dir.dot(hit.normal)*hit.normal).normalize();
                ray = Ray(hit.point, reflDir);
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// KD-Tree（原地构建）
// ─────────────────────────────────────────────────────────────────────────────
struct KDTree {
    std::vector<Photon> photons;
    std::vector<int> idx;
    std::vector<int> splitAxis;  // per-node
    int rootNode = -1;

    // 构建（返回节点编号）
    // 使用 left/right 子树 [start, mid) [mid+1, end)
    void buildRecur(int* arr, int start, int end, int depth) {
        if (end <= start) return;
        int axis = depth % 3;
        // 按轴排序
        std::sort(arr + start, arr + end, [&](int a, int b){
            const double* pa = &photons[a].pos.x;
            const double* pb = &photons[b].pos.x;
            return pa[axis] < pb[axis];
        });
        int mid = (start + end) / 2;
        buildRecur(arr, start, mid, depth+1);
        buildRecur(arr, mid+1, end, depth+1);
    }

    void build() {
        idx.resize(photons.size());
        std::iota(idx.begin(), idx.end(), 0);
        buildRecur(idx.data(), 0, (int)idx.size(), 0);
    }

    // 在 [start, end) 范围内搜索半径内光子（kd数组已排好序，中值分割）
    void searchRec(int start, int end, int depth,
                   const Vec3& pos, double r2,
                   std::vector<std::pair<double,int>>& res) const {
        if (end <= start) return;
        int axis = depth % 3;
        int mid = (start + end) / 2;

        const Photon& ph = photons[idx[mid]];
        double dx = ph.pos.x-pos.x, dy = ph.pos.y-pos.y, dz = ph.pos.z-pos.z;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < r2) res.push_back({d2, idx[mid]});

        const double* pp = &pos.x;
        const double* pm = &ph.pos.x;
        double planeDist = pp[axis] - pm[axis];

        if (planeDist <= 0) {
            searchRec(start, mid, depth+1, pos, r2, res);
            if (planeDist*planeDist < r2)
                searchRec(mid+1, end, depth+1, pos, r2, res);
        } else {
            searchRec(mid+1, end, depth+1, pos, r2, res);
            if (planeDist*planeDist < r2)
                searchRec(start, mid, depth+1, pos, r2, res);
        }
    }

    Vec3 estimate(const Vec3& pos, double radius, int kMax) const {
        if (photons.empty()) return {0,0,0};
        std::vector<std::pair<double,int>> nearby;
        nearby.reserve(kMax * 2);
        searchRec(0, (int)idx.size(), 0, pos, radius*radius, nearby);
        if (nearby.empty()) return {0,0,0};

        int k = (int)std::min((int)nearby.size(), kMax);
        std::partial_sort(nearby.begin(), nearby.begin()+k, nearby.end());

        Vec3 power(0,0,0);
        double r2 = nearby[k-1].first;
        for (int i = 0; i < k; ++i) power += photons[nearby[i].second].power;
        double area = M_PI * r2;
        return (area > 1e-15) ? power/area : Vec3(0,0,0);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// 直接光照（点光源，软阴影：玻璃部分透光）
// ─────────────────────────────────────────────────────────────────────────────
Vec3 directLight(const Vec3& pos, const Vec3& normal, int matId) {
    Vec3 toLight = LIGHT_POS - pos;
    double distToLight = toLight.len();
    toLight = toLight * (1.0 / distToLight);

    Ray shadowRay(pos, toLight);
    HitInfo shadowHit;
    bool blocked = sceneHit(shadowRay, 1e-3, shadowHit) && shadowHit.t < distToLight - 1e-3;
    if (blocked && shadowHit.matId != MAT_GLASS) return {0,0,0};

    double ndotl = std::max(0.0, normal.dot(toLight));
    double atten = 1.0 / (distToLight * distToLight + 2.0);
    // 直接光照系数：0.8（减弱，让焦散显眼）
    Vec3 albedo = (matId == MAT_FLOOR) ? Vec3(0.8, 0.72, 0.60) : Vec3(0.65, 0.70, 0.80);
    return albedo * (LIGHT_COLOR * (ndotl * atten * 0.8));
}

// ─────────────────────────────────────────────────────────────────────────────
// 渲染像素（直接光照 + 焦散查询）
// ─────────────────────────────────────────────────────────────────────────────
Vec3 renderPixel(const Ray& camRay, const KDTree& causticMap, RNG& rng) {
    Ray ray = camRay;
    Vec3 throughput(1,1,1);
    Vec3 color(0,0,0);

    for (int bounce = 0; bounce < 4; ++bounce) {
        HitInfo hit;
        if (!sceneHit(ray, 1e-4, hit)) {
            double t = 0.5*(ray.dir.y + 1.0);
            Vec3 sky = (1.0-t)*Vec3(0.8,0.85,0.9) + t*Vec3(0.3,0.5,0.85);
            color += throughput * sky * 0.08;
            break;
        }

        if (hit.matId == MAT_FLOOR || hit.matId == MAT_WALL) {
            Vec3 albedo = (hit.matId == MAT_FLOOR) ? Vec3(0.8,0.72,0.60) : Vec3(0.65,0.70,0.80);

            // 直接光照
            Vec3 direct = directLight(hit.point, hit.normal, hit.matId);
            color += throughput * direct;

            // 焦散（光子密度估计）
            Vec3 caustic = causticMap.estimate(hit.point, 0.12, 60);
            color += throughput * albedo * caustic * 2.0;

            // 仅第一次弹射做漫反射（避免多次弹射过亮）
            if (bounce == 0) {
                Vec3 newDir = rng.cosineHemisphere(hit.normal);
                ray = Ray(hit.point, newDir);
                throughput = throughput * albedo * 0.25;  // 大幅衰减
            } else {
                break;
            }
        } else if (hit.matId == MAT_GLASS) {
            double nRatio = hit.inside ? IOR : (1.0/IOR);
            Vec3 refDir;
            double cosine = std::abs(ray.dir.dot(hit.normal));
            if (refract(ray.dir, hit.normal, nRatio, refDir)) {
                double fr = schlick(cosine, nRatio);
                if (rng.next() < fr) {
                    Vec3 reflDir = (ray.dir - 2.0*ray.dir.dot(hit.normal)*hit.normal).normalize();
                    ray = Ray(hit.point, reflDir);
                    throughput = throughput * fr;
                } else {
                    ray = Ray(hit.point, refDir);
                    throughput = throughput * (1.0-fr);
                }
            } else {
                Vec3 reflDir = (ray.dir - 2.0*ray.dir.dot(hit.normal)*hit.normal).normalize();
                ray = Ray(hit.point, reflDir);
            }
        }
    }
    return color;
}

// ─────────────────────────────────────────────────────────────────────────────
// 主程序
// ─────────────────────────────────────────────────────────────────────────────
int main() {
    const int W = 640, H = 480;
    const int SPP = 8;
    const int NUM_PHOTONS = 300000;
    const int MAX_PHOTON_BOUNCE = 6;

    std::cout << "=== Caustics Renderer ===" << std::endl;
    std::cout << "分辨率: " << W << "x" << H << "  SPP=" << SPP << "  光子=" << NUM_PHOTONS << std::endl;

    RNG rng(2026);

    // ── Pass 1：光子追踪 ──────────────────────────────────────────────────────
    std::cout << "\n[Pass 1] 发射光子..." << std::flush;
    KDTree causticMap;
    causticMap.photons.reserve(NUM_PHOTONS / 3);

    // 单位：光子能量 = 总能量 / 光子数 × 立体角权重
    double cosMax = std::cos(M_PI / 4.0);
    double solidAngle = 2.0 * M_PI * (1.0 - cosMax);
    Vec3 photonPower = LIGHT_COLOR * (solidAngle / NUM_PHOTONS * 8.0);

    Vec3 toSphere = (GLASS_CENTER - LIGHT_POS).normalize();
    Vec3 up2 = (std::abs(toSphere.x) < 0.9) ? Vec3(1,0,0) : Vec3(0,1,0);
    Vec3 t1 = toSphere.cross(up2).normalize();
    Vec3 t2 = toSphere.cross(t1);

    for (int i = 0; i < NUM_PHOTONS; ++i) {
        double u = rng.next(), v = rng.next();
        double cosTheta = 1.0 - u*(1.0-cosMax);
        double sinTheta = std::sqrt(std::max(0.0, 1.0-cosTheta*cosTheta));
        double phi = 2.0*M_PI*v;
        Vec3 dir = (t1*(sinTheta*std::cos(phi)) + t2*(sinTheta*std::sin(phi)) + toSphere*cosTheta).normalize();
        Ray photonRay(LIGHT_POS, dir);
        tracePhoton(photonRay, photonPower, MAX_PHOTON_BOUNCE, rng, causticMap.photons);
        if (i % 100000 == 0 && i > 0) std::cout << " " << i/1000 << "k" << std::flush;
    }
    std::cout << "\n  存储光子数: " << causticMap.photons.size() << std::endl;

    std::cout << "[Pass 1] 构建 KD-Tree..." << std::flush;
    causticMap.build();
    std::cout << " 完成" << std::endl;

    // ── Pass 2：渲染 ──────────────────────────────────────────────────────────
    std::cout << "\n[Pass 2] 渲染..." << std::flush;

    Vec3 camPos(0, 2.2, 3.0);
    Vec3 camTarget(0, 0.2, -1.5);
    Vec3 camUp(0, 1, 0);
    Vec3 camDir = (camTarget - camPos).normalize();
    Vec3 camRight = camDir.cross(camUp).normalize();
    Vec3 camUpActual = camRight.cross(camDir);

    double aspect = (double)W / H;
    double fovHalf = std::tan(0.5 * 48.0 * M_PI / 180.0);

    std::vector<Vec3> fb(W*H, Vec3(0,0,0));
    for (int y = 0; y < H; ++y) {
        if (y % 48 == 0) std::cout << " " << y << "/" << H << std::flush;
        for (int x = 0; x < W; ++x) {
            Vec3 pixel(0,0,0);
            for (int s = 0; s < SPP; ++s) {
                double u = (x + rng.next()) / W;
                double v = 1.0 - (y + rng.next()) / H;
                double ndcX = (2.0*u - 1.0)*aspect*fovHalf;
                double ndcY = (2.0*v - 1.0)*fovHalf;
                Vec3 rayDir = (camDir + camRight*ndcX + camUpActual*ndcY).normalize();
                pixel += renderPixel(Ray(camPos, rayDir), causticMap, rng);
            }
            fb[y*W+x] = pixel / (double)SPP;
        }
    }
    std::cout << "\n渲染完成" << std::endl;

    // ── 色调映射 + 伽马 ───────────────────────────────────────────────────────
    std::vector<uint8_t> pixels(W*H*3);
    for (int i = 0; i < W*H; ++i) {
        Vec3 c = fb[i];
        // ACES tone mapping
        c.x = c.x*(2.51*c.x+0.03)/(c.x*(2.43*c.x+0.59)+0.14);
        c.y = c.y*(2.51*c.y+0.03)/(c.y*(2.43*c.y+0.59)+0.14);
        c.z = c.z*(2.51*c.z+0.03)/(c.z*(2.43*c.z+0.59)+0.14);
        c.x = std::pow(std::max(0.0, std::min(1.0, c.x)), 1.0/2.2);
        c.y = std::pow(std::max(0.0, std::min(1.0, c.y)), 1.0/2.2);
        c.z = std::pow(std::max(0.0, std::min(1.0, c.z)), 1.0/2.2);
        pixels[i*3+0] = (uint8_t)(c.x*255.0);
        pixels[i*3+1] = (uint8_t)(c.y*255.0);
        pixels[i*3+2] = (uint8_t)(c.z*255.0);
    }

    const char* outFile = "caustics_output.png";
    stbi_write_png(outFile, W, H, 3, pixels.data(), W*3);
    std::cout << "\n✅ 输出: " << outFile << std::endl;

    double sumR=0,sumG=0,sumB=0;
    for (int i = 0; i < W*H; ++i) { sumR+=pixels[i*3]; sumG+=pixels[i*3+1]; sumB+=pixels[i*3+2]; }
    double n = W*H;
    std::cout << "像素均值 R=" << sumR/n << " G=" << sumG/n << " B=" << sumB/n << std::endl;
    std::cout << "总均值=" << (sumR+sumG+sumB)/(3.0*n) << std::endl;
    return 0;
}
