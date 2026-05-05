/*
 * SPPM - Stochastic Progressive Photon Mapping
 * 随机渐进光子映射 - 全局光照渲染器
 *
 * 正确的 SPPM 单位和公式：
 * - 相机 Pass：记录 hp.weight = 从相机到 HP 的路径吞吐量（不含表面 albedo）
 *             同时记录 hp.albedo = HP 表面的漫反射率
 * - 光子 Pass：ph.flux = photon_power（代表功率 W，不含表面属性）
 * - 密度估计：dL = Σ ph.flux * albedo / (π * r²)
 *            L += hp.weight * dL
 * - 最终：L = accFlux / (π * r² * N_e * ITERS)，其中 accFlux 累积了 hp.weight * hp.albedo * Σ ph.flux
 *
 * 场景：Cornell Box
 * 编译: g++ main.cpp -o sppm -std=c++17 -O2 -Wall -Wextra
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <numeric>
#include <cstring>
#include <cstdio>

static const double PI = 3.14159265358979323846;

// ======================================================================
// Vec3
// ======================================================================
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t)      const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t)      const { return {x/t, y/t, z/t}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    double dot(const Vec3& o)   const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o)   const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double len2() const { return x*x + y*y + z*z; }
    double len()  const { return std::sqrt(len2()); }
    Vec3 unit()   const { double l = len(); return l > 1e-12 ? *this/l : Vec3(0,1,0); }
    bool zero()   const { return x==0 && y==0 && z==0; }
};
Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ======================================================================
// RNG
// ======================================================================
struct RNG {
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> d01;
    explicit RNG(uint64_t seed) : gen(seed), d01(0.0, 1.0) {}
    double next()  { return d01(gen); }

    Vec3 cosHemi(const Vec3& n) {
        double r1 = next(), r2 = next();
        double phi = 2.0 * PI * r1;
        double sr2 = std::sqrt(r2);
        double lx = std::cos(phi) * sr2;
        double ly = std::sin(phi) * sr2;
        double lz = std::sqrt(std::max(0.0, 1.0 - r2));
        Vec3 up = (std::abs(n.x) < 0.9) ? Vec3(1,0,0) : Vec3(0,1,0);
        Vec3 s = n.cross(up).unit();
        Vec3 t = n.cross(s);
        return (s * lx + t * ly + n * lz).unit();
    }
};

// ======================================================================
// Ray
// ======================================================================
struct Ray {
    Vec3 o, d;
    Ray(Vec3 o, Vec3 d) : o(o), d(d.unit()) {}
    Vec3 at(double t) const { return o + d * t; }
};

// ======================================================================
// Materials
// ======================================================================
enum class Mat { DIFFUSE, MIRROR, GLASS };

struct Material {
    Mat   type    = Mat::DIFFUSE;
    Vec3  albedo  = {0.73, 0.73, 0.73};
    Vec3  emit    = {0, 0, 0};
    double ior    = 1.5;
    bool isLight() const { return !emit.zero(); }
};

// ======================================================================
// Hit record
// ======================================================================
struct Hit {
    double t   = 1e18;
    Vec3 p, n;
    bool front = true;
    int matId  = -1;
    void setN(const Ray& r, Vec3 outN) {
        front = r.d.dot(outN) < 0;
        n = front ? outN : -outN;
    }
};

// ======================================================================
// Sphere
// ======================================================================
struct Sphere {
    Vec3 c; double r; int mat;
    bool hit(const Ray& ray, double tMin, double tMax, Hit& h) const {
        Vec3 oc = ray.o - c;
        double a = ray.d.dot(ray.d);
        double hb = oc.dot(ray.d);
        double cc = oc.dot(oc) - r*r;
        double disc = hb*hb - a*cc;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double root = (-hb - sq) / a;
        if (root < tMin || root > tMax) {
            root = (-hb + sq) / a;
            if (root < tMin || root > tMax) return false;
        }
        h.t = root; h.p = ray.at(root);
        h.setN(ray, (h.p - c) / r); h.matId = mat;
        return true;
    }
};

// ======================================================================
// AxisPlane
// ======================================================================
struct AxisPlane {
    int axis;
    double pos;
    double ua, ub, va, vb;
    int mat;
    bool negNorm;

    bool hit(const Ray& ray, double tMin, double tMax, Hit& h) const {
        double rd, ro;
        if (axis == 0) { rd = ray.d.x; ro = ray.o.x; }
        else if (axis == 1) { rd = ray.d.y; ro = ray.o.y; }
        else { rd = ray.d.z; ro = ray.o.z; }

        if (std::abs(rd) < 1e-10) return false;
        double t = (pos - ro) / rd;
        if (t < tMin || t > tMax) return false;

        Vec3 p = ray.at(t);
        double pu, pv;
        if (axis == 0) { pu = p.y; pv = p.z; }
        else if (axis == 1) { pu = p.x; pv = p.z; }
        else { pu = p.x; pv = p.y; }

        if (pu < ua || pu > ub || pv < va || pv > vb) return false;

        h.t = t; h.p = p;
        Vec3 outN;
        if (axis == 0)      outN = Vec3(1, 0, 0);
        else if (axis == 1) outN = Vec3(0, 1, 0);
        else                outN = Vec3(0, 0, 1);
        if (negNorm) outN = -outN;
        h.setN(ray, outN);
        h.matId = mat;
        return true;
    }
};

// ======================================================================
// Scene
// ======================================================================
struct Scene {
    std::vector<Material>  mats;
    std::vector<Sphere>    spheres;
    std::vector<AxisPlane> planes;

    Vec3   lightCenter;
    double lightHalfU, lightHalfV;
    Vec3   lightEmit;

    int addMat(Material m) { mats.push_back(m); return (int)mats.size()-1; }

    bool intersect(const Ray& ray, Hit& best) const {
        best.t = 1e18;
        Hit tmp; bool hit = false;
        for (auto& s : spheres)
            if (s.hit(ray, 1e-4, best.t, tmp)) { hit = true; best = tmp; }
        for (auto& p : planes)
            if (p.hit(ray, 1e-4, best.t, tmp)) { hit = true; best = tmp; }
        return hit;
    }

    Vec3 sampleLight(RNG& rng) const {
        double u = (rng.next() * 2 - 1) * lightHalfU;
        double v = (rng.next() * 2 - 1) * lightHalfV;
        return lightCenter + Vec3(u, 0, v);
    }
    double lightArea() const { return 4.0 * lightHalfU * lightHalfV; }
};

// Cornell Box: x[-1,1] y[-1,1] z[0,2]
Scene makeCornellBox() {
    Scene sc;
    int white  = sc.addMat({ Mat::DIFFUSE, {0.73, 0.73, 0.73} });
    int red    = sc.addMat({ Mat::DIFFUSE, {0.65, 0.05, 0.05} });
    int green  = sc.addMat({ Mat::DIFFUSE, {0.12, 0.45, 0.15} });
    int mirror = sc.addMat({ Mat::MIRROR,  {0.95, 0.95, 0.95} });
    int glass  = sc.addMat({ Mat::GLASS,   {1.0,  1.0,  1.0 }, {}, 1.5 });
    int light  = sc.addMat({ Mat::DIFFUSE, {0,0,0}, {15, 15, 12} });

    // 地板 y=-1，法线 +y
    sc.planes.push_back({1, -1.0, -1.0, 1.0, 0.0, 2.0, white, false});
    // 天花板 y=+1，法线 -y
    sc.planes.push_back({1, +1.0, -1.0, 1.0, 0.0, 2.0, white, true});
    // 背墙 z=+2，法线 -z
    sc.planes.push_back({2, +2.0, -1.0, 1.0, -1.0, 1.0, white, true});
    // 左墙 x=-1，法线 +x（红色）
    sc.planes.push_back({0, -1.0, -1.0, 1.0, 0.0, 2.0, red,   false});
    // 右墙 x=+1，法线 -x（绿色）
    sc.planes.push_back({0, +1.0, -1.0, 1.0, 0.0, 2.0, green, true});
    // 天花板光源（y=0.999）
    sc.planes.push_back({1, 0.999, -0.3, 0.3, 0.7, 1.3, light, true});

    sc.lightCenter = {0.0, 0.999, 1.0};
    sc.lightHalfU  = 0.3;
    sc.lightHalfV  = 0.3;
    sc.lightEmit   = {15, 15, 12};

    // 左球：镜面
    sc.spheres.push_back({{-0.43, -0.57, 0.85}, 0.43, mirror});
    // 右球：玻璃
    sc.spheres.push_back({{ 0.43, -0.57, 1.35}, 0.43, glass});

    return sc;
}

// ======================================================================
// Fresnel (Schlick)
// ======================================================================
double fresnel(double cosI, double ior1, double ior2) {
    double r0 = (ior1 - ior2) / (ior1 + ior2);
    r0 *= r0;
    double c = 1.0 - cosI;
    return r0 + (1 - r0) * c*c*c*c*c;
}

// ======================================================================
// HitPoint
// ======================================================================
struct HitPoint {
    Vec3   pos;
    Vec3   normal;
    Vec3   weight;     // cam_throughput（不含表面 albedo）
    Vec3   albedo;     // 表面漫反射率
    Vec3   accFlux;    // 累积 (weight * albedo * Σ ph.flux)
    double radius2;
    int    N;          // 累积修正光子数
    int    pixIdx;
    bool   valid;      // 有效漫反射 HP
    Vec3   directL;    // 直接打到光源
};

// ======================================================================
// Camera Pass
// ======================================================================
void cameraPass(const Scene& sc, RNG& rng, int W, int H,
                std::vector<HitPoint>& hps, double initR2) {
    hps.resize(W * H);
    for (int i = 0; i < W*H; i++) {
        hps[i] = {};
        hps[i].radius2 = initR2;
        hps[i].pixIdx  = i;
    }

    // 相机参数
    Vec3 eye(0, 0, -1.0);
    Vec3 fwd(0, 0,  1.0);
    // fwd.cross(worldUp) = (0,0,1)x(0,1,0) = (-1,0,0)，取反得 +x
    Vec3 rgt(1, 0, 0);
    Vec3 up (0, 1, 0);

    double fovY  = 50.0 * PI / 180.0;
    double halfH = std::tan(fovY / 2.0);
    double halfW = halfH * (double)W / H;

    for (int py = 0; py < H; py++) {
        for (int px = 0; px < W; px++) {
            int idx = py * W + px;
            double u = ((px + rng.next()) / W * 2.0 - 1.0) * halfW;
            double v = (1.0 - (py + rng.next()) / H * 2.0) * halfH;
            Vec3 dir = (fwd + rgt * u + up * v).unit();
            Ray ray(eye, dir);

            Vec3 throughput(1, 1, 1);

            for (int depth = 0; depth < 10; depth++) {
                Hit h;
                if (!sc.intersect(ray, h)) break;
                const Material& m = sc.mats[h.matId];

                if (m.isLight()) {
                    hps[idx].directL = throughput * m.emit;
                    break;
                }

                if (m.type == Mat::DIFFUSE) {
                    // weight = throughput（不含 albedo）
                    hps[idx].pos    = h.p;
                    hps[idx].normal = h.n;
                    hps[idx].weight = throughput;        // ← 不含 albedo
                    hps[idx].albedo = m.albedo;          // ← 单独记录
                    hps[idx].valid  = true;
                    break;
                }

                if (m.type == Mat::MIRROR) {
                    Vec3 refl = ray.d - h.n * (2.0 * ray.d.dot(h.n));
                    throughput = throughput * m.albedo;
                    ray = Ray(h.p, refl);
                    continue;
                }

                if (m.type == Mat::GLASS) {
                    double ior1 = h.front ? 1.0 : m.ior;
                    double ior2 = h.front ? m.ior : 1.0;
                    double cosI = std::abs(ray.d.dot(h.n));
                    double eta  = ior1 / ior2;
                    double sinT2 = eta * eta * (1.0 - cosI * cosI);
                    double fr = (sinT2 >= 1.0) ? 1.0 : fresnel(cosI, ior1, ior2);
                    if (rng.next() < fr) {
                        Vec3 refl = ray.d - h.n * (2.0 * ray.d.dot(h.n));
                        ray = Ray(h.p, refl);
                    } else {
                        double cosT = std::sqrt(std::max(0.0, 1.0 - sinT2));
                        Vec3 refr = ray.d * eta + h.n * (eta * cosI - cosT);
                        ray = Ray(h.p, refr.unit());
                    }
                    throughput = throughput * m.albedo;
                    continue;
                }
            }
        }
    }
}

// ======================================================================
// Photon
// ======================================================================
struct Photon {
    Vec3 pos;
    Vec3 power;   // 光子功率（不含表面反射率，仅路径吞吐量）
    Vec3 dir;     // 入射方向
};

// ======================================================================
// Photon Pass
//
// 每个光子代表 totalLightFlux / N 的功率
// totalLightFlux = lightEmit * π * lightArea（朗伯面光源）
// 光子在漫反射面时，存储 ph.power = path_throughput（不含表面 albedo）
// 漫反射后继续传播时，乘以 albedo（因为漫反射概率 = albedo）
// ======================================================================
void photonPass(const Scene& sc, RNG& rng, int N,
                std::vector<Photon>& photons) {
    photons.clear();
    photons.reserve((size_t)N * 3);

    double area       = sc.lightArea();
    // 朗伯面光源总功率 = L_emit * π * A（半球 cos 积分 = π）
    // 余弦加权采样 PDF = cos/π，每个光子代表 Φ_total / N
    Vec3 photonPower  = sc.lightEmit * (PI * area / (double)N);

    Vec3 downNorm(0, -1, 0);

    for (int i = 0; i < N; i++) {
        Vec3 origin = sc.sampleLight(rng);
        Vec3 dir    = rng.cosHemi(downNorm);

        Vec3 power = photonPower;  // 每个光子代表的功率
        Ray ray(origin, dir);

        for (int depth = 0; depth < 8; depth++) {
            Hit h;
            if (!sc.intersect(ray, h)) break;
            const Material& m = sc.mats[h.matId];
            if (m.isLight()) break;

            if (m.type == Mat::DIFFUSE) {
                // 存储光子（power = 路径功率，不含当前表面 albedo）
                Photon ph;
                ph.pos   = h.p;
                ph.power = power;     // ← 不含当前表面 albedo
                ph.dir   = ray.d;
                photons.push_back(ph);

                // 继续传播（乘以 albedo 因为漫反射反射率）
                double q = std::max({m.albedo.x, m.albedo.y, m.albedo.z});
                if (depth >= 2 && rng.next() > q) break;
                double quse = (depth >= 2) ? q : 1.0;
                power = power * m.albedo / quse;
                ray = Ray(h.p, rng.cosHemi(h.n));
                continue;
            }

            if (m.type == Mat::MIRROR) {
                Vec3 refl = ray.d - h.n * (2.0 * ray.d.dot(h.n));
                power = power * m.albedo;
                ray   = Ray(h.p, refl);
                continue;
            }

            if (m.type == Mat::GLASS) {
                double ior1 = h.front ? 1.0 : m.ior;
                double ior2 = h.front ? m.ior : 1.0;
                double cosI = std::abs(ray.d.dot(h.n));
                double eta  = ior1 / ior2;
                double sinT2 = eta * eta * (1.0 - cosI * cosI);
                double fr = (sinT2 >= 1.0) ? 1.0 : fresnel(cosI, ior1, ior2);
                if (rng.next() < fr) {
                    Vec3 refl = ray.d - h.n * (2.0 * ray.d.dot(h.n));
                    ray = Ray(h.p, refl);
                } else {
                    double cosT = std::sqrt(std::max(0.0, 1.0 - sinT2));
                    Vec3 refr = ray.d * eta + h.n * (eta * cosI - cosT);
                    ray = Ray(h.p, refr.unit());
                }
                power = power * m.albedo;
                continue;
            }

            break;
        }
    }
}

// ======================================================================
// kd-tree
// ======================================================================
struct KdTree {
    struct Node { int idx, left, right, axis; };
    std::vector<Node> nodes;
    const std::vector<Photon>* phs = nullptr;
    int root = -1, nc = 0;

    void build(const std::vector<Photon>& p) {
        phs = &p;
        if (p.empty()) { root = -1; nc = 0; return; }
        nodes.resize(p.size());
        nc = 0;
        std::vector<int> idx(p.size());
        std::iota(idx.begin(), idx.end(), 0);
        root = rec(idx, 0, (int)idx.size(), 0);
    }

    int rec(std::vector<int>& idx, int lo, int hi, int d) {
        if (lo >= hi) return -1;
        int axis = d % 3, mid = (lo + hi) / 2;
        std::nth_element(idx.begin()+lo, idx.begin()+mid, idx.begin()+hi,
            [&](int a, int b) {
                auto& pa = (*phs)[a].pos;
                auto& pb = (*phs)[b].pos;
                return (axis==0?pa.x:(axis==1?pa.y:pa.z)) <
                       (axis==0?pb.x:(axis==1?pb.y:pb.z));
            });
        int ni = nc++;
        nodes[ni] = { idx[mid], -1, -1, axis };
        nodes[ni].left  = rec(idx, lo, mid, d+1);
        nodes[ni].right = rec(idx, mid+1, hi, d+1);
        return ni;
    }

    void query(int ni, const Vec3& pos, double r2,
               std::vector<int>& result) const {
        if (ni < 0 || !phs) return;
        const Node& n = nodes[ni];
        Vec3 d = pos - (*phs)[n.idx].pos;
        if (d.len2() <= r2) result.push_back(n.idx);
        double diff = (n.axis==0 ? d.x : (n.axis==1 ? d.y : d.z));
        if (diff * diff <= r2) {
            query(n.left,  pos, r2, result);
            query(n.right, pos, r2, result);
        } else if (diff < 0) {
            query(n.left,  pos, r2, result);
        } else {
            query(n.right, pos, r2, result);
        }
    }
};

// ======================================================================
// SPPM Update
//
// 正确公式（per-iteration）：
//   dFlux = Σ ph.power（在半径内的光子功率之和）
//   L_iter = weight * albedo * dFlux / (π * r²)
//   （因为 ph.power 不含 albedo，BSDF = albedo/π，PDF = cos/π，BSDF/PDF = albedo）
//
// 累积（Hachisuka 2009）：
//   N_new = N_old + α * M
//   r²_new = r²_old * (N_old + α * M) / (N_old + M)
//   Φ_new = (Φ_old + weight * albedo * dFlux) * r²_new / r²_old
// ======================================================================
void sppmUpdate(std::vector<HitPoint>& hps,
                const KdTree& kdt,
                const std::vector<Photon>& photons,
                double alpha = 0.7) {
    std::vector<int> nb;
    for (auto& hp : hps) {
        if (!hp.valid) continue;
        nb.clear();
        kdt.query(kdt.root, hp.pos, hp.radius2, nb);
        if (nb.empty()) continue;

        int M = (int)nb.size();
        int N = hp.N;

        // 收集本轮光子功率（注意：ph.power 不含表面 albedo）
        Vec3 dFlux;
        for (int j : nb) {
            // 只接受正面入射光子
            if (hp.normal.dot(-photons[j].dir) > 0)
                dFlux += photons[j].power;
        }

        // SPPM 更新
        double ratio = (double)(N + alpha * M) / (double)(N + M);
        hp.radius2 *= ratio;
        // accFlux += weight × albedo/π × Σph.power
        // （正确的密度估计：BSDF/pdf = albedo，再额外除以 π 用于密度-辐射转换）
        hp.accFlux = (hp.accFlux + hp.weight * hp.albedo * dFlux * (1.0 / PI)) * ratio;
        hp.N += (int)(alpha * M);
    }
}

// ======================================================================
// Tone mapping + gamma
// ======================================================================
uint8_t toU8(double v) {
    double a = 2.51, b = 0.03, c = 2.43, d_ = 0.59, e = 0.14;
    v = std::clamp((v*(a*v+b)) / (v*(c*v+d_)+e), 0.0, 1.0);
    return (uint8_t)(std::pow(v, 1.0/2.2) * 255.0 + 0.5);
}

// ======================================================================
// Main
// ======================================================================
int main() {
    const int W = 512, H = 512;
    const int ITERS = 40;
    const int N_PHOTONS = 200000;

    printf("SPPM Cornell Box: %dx%d, %d iters × %d photons/iter\n",
           W, H, ITERS, N_PHOTONS);
    fflush(stdout);

    Scene sc = makeCornellBox();
    RNG rng(20260321ULL);

    // ------ Camera Pass ------
    printf("[1/3] Camera pass...\n"); fflush(stdout);
    std::vector<HitPoint> hps;
    double initR2 = 0.08 * 0.08;
    cameraPass(sc, rng, W, H, hps, initR2);

    int nValid = 0;
    for (auto& hp : hps) if (hp.valid) nValid++;
    printf("  Valid HPs: %d/%d\n", nValid, W*H); fflush(stdout);

    // ------ Photon iterations ------
    long long totalPhotons = 0;
    for (int iter = 0; iter < ITERS; iter++) {
        std::vector<Photon> photons;
        photonPass(sc, rng, N_PHOTONS, photons);
        totalPhotons += N_PHOTONS;

        KdTree kdt;
        kdt.build(photons);

        sppmUpdate(hps, kdt, photons, 0.7);

        if ((iter+1) % 10 == 0 || iter == 0) {
            printf("  iter %2d/%d: stored=%d photons\n",
                   iter+1, ITERS, (int)photons.size());
            fflush(stdout);
        }
    }

    // ------ Reconstruct ------
    printf("[3/3] Reconstructing image...\n"); fflush(stdout);

    std::vector<uint8_t> img(W * H * 3, 0);

    // 计算亮度统计（用于自动曝光调整）
    double maxL = 0;
    for (int i = 0; i < W*H; i++) {
        const HitPoint& hp = hps[i];
        if (hp.valid && hp.N > 0 && hp.radius2 > 1e-15) {
            double scale = 1.0 / (PI * hp.radius2 * (double)ITERS);
            Vec3 L = hp.accFlux * scale;
            maxL = std::max(maxL, std::max({L.x, L.y, L.z}));
        }
    }
    printf("  Max radiance: %.4f\n", maxL); fflush(stdout);

    for (int i = 0; i < W*H; i++) {
        const HitPoint& hp = hps[i];
        Vec3 color;

        if (!hp.directL.zero()) {
            color = hp.directL;
        } else if (hp.valid && hp.N > 0 && hp.radius2 > 1e-15) {
            double scale = 1.0 / (PI * hp.radius2 * (double)ITERS);
            color = hp.accFlux * scale;
        }

        int row = i / W, col = i % W;
        int outIdx = ((H - 1 - row) * W + col) * 3;
        img[outIdx+0] = toU8(color.x);
        img[outIdx+1] = toU8(color.y);
        img[outIdx+2] = toU8(color.z);
    }

    stbi_write_png("sppm_output.png", W, H, 3, img.data(), W*3);
    printf("✅ 已保存 sppm_output.png\n");
    return 0;
}
