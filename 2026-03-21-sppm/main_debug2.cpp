/*
 * SPPM - Stochastic Progressive Photon Mapping
 * 随机渐进光子映射 - 全局光照渲染
 *
 * 实现特性：
 * - 渐进光子映射（Progressive Photon Mapping, PPM）
 * - 随机采样改进（Stochastic，解决内存问题）
 * - Cornell Box 场景（左红右绿，白色天花板/地板/背墙）
 * - 漫反射 + 镜面反射 + 折射（玻璃球）
 * - kd树加速光子查找
 * - 多轮迭代累积，噪声随迭代减少
 *
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
#include <array>
#include <memory>
#include <cassert>
#include <cstring>

static const double PI = 3.14159265358979323846;

// ============================================================
// 基础数学
// ============================================================
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    double length2() const { return x*x + y*y + z*z; }
    Vec3 normalized() const { double l = length(); return (l > 1e-12) ? *this / l : Vec3(0,1,0); }
    bool isZero() const { return x==0 && y==0 && z==0; }
};
Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
// 随机数
// ============================================================
struct RNG {
    std::mt19937_64 gen;
    std::uniform_real_distribution<double> dist;
    RNG(uint64_t seed = 42) : gen(seed), dist(0.0, 1.0) {}
    double next() { return dist(gen); }
    Vec3 cosineHemisphere(const Vec3& n) {
        double u1 = next(), u2 = next();
        double r = std::sqrt(u1);
        double theta = 2.0 * PI * u2;
        double x = r * std::cos(theta);
        double y = r * std::sin(theta);
        double z = std::sqrt(std::max(0.0, 1.0 - u1));
        Vec3 up = (std::abs(n.x) < 0.9) ? Vec3(1,0,0) : Vec3(0,1,0);
        Vec3 tangent = n.cross(up).normalized();
        Vec3 bitangent = n.cross(tangent);
        return (tangent * x + bitangent * y + n * z).normalized();
    }
};

// ============================================================
// 光线
// ============================================================
struct Ray {
    Vec3 origin, dir;
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalized()) {}
    Vec3 at(double t) const { return origin + dir * t; }
};

// ============================================================
// 材质类型
// ============================================================
enum class MatType { DIFFUSE, MIRROR, GLASS };

struct Material {
    MatType type;
    Vec3 albedo;
    Vec3 emission;
    double ior;

    static Material diffuse(Vec3 col) {
        Material m; m.type = MatType::DIFFUSE; m.albedo = col;
        m.emission = {0,0,0}; m.ior = 1.0; return m;
    }
    static Material mirror(Vec3 col) {
        Material m; m.type = MatType::MIRROR; m.albedo = col;
        m.emission = {0,0,0}; m.ior = 1.0; return m;
    }
    static Material glass(double ior = 1.5) {
        Material m; m.type = MatType::GLASS; m.albedo = {1,1,1};
        m.emission = {0,0,0}; m.ior = ior; return m;
    }
    static Material emissive(Vec3 emit) {
        Material m; m.type = MatType::DIFFUSE; m.albedo = {0,0,0};
        m.emission = emit; m.ior = 1.0; return m;
    }
};

// ============================================================
// 几何体
// ============================================================
struct HitRecord {
    double t;
    Vec3 point, normal;
    bool frontFace;
    int matId;

    void setFaceNormal(const Ray& ray, const Vec3& outNormal) {
        frontFace = ray.dir.dot(outNormal) < 0;
        normal = frontFace ? outNormal : -outNormal;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    int matId;

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = ray.origin - center;
        double a = ray.dir.dot(ray.dir);
        double hb = oc.dot(ray.dir);
        double c = oc.dot(oc) - radius * radius;
        double disc = hb*hb - a*c;
        if (disc < 0) return false;
        double sqrtD = std::sqrt(disc);
        double root = (-hb - sqrtD) / a;
        if (root < tMin || root > tMax) {
            root = (-hb + sqrtD) / a;
            if (root < tMin || root > tMax) return false;
        }
        rec.t = root;
        rec.point = ray.at(root);
        rec.setFaceNormal(ray, (rec.point - center) / radius);
        rec.matId = matId;
        return true;
    }
};

// 轴对齐平面（Cornell Box 用）
struct Plane {
    int axis;     // 0=固定x, 1=固定y, 2=固定z
    double pos;
    double u0, u1, v0, v1;  // 面内边界
    int matId;
    bool flipNormal;  // 法线朝负方向

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        double rayD, rayO;
        if (axis == 0) { rayD = ray.dir.x; rayO = ray.origin.x; }
        else if (axis == 1) { rayD = ray.dir.y; rayO = ray.origin.y; }
        else { rayD = ray.dir.z; rayO = ray.origin.z; }

        if (std::abs(rayD) < 1e-10) return false;
        double t = (pos - rayO) / rayD;
        if (t < tMin || t > tMax) return false;

        Vec3 p = ray.at(t);
        double pu, pv;
        if (axis == 0) { pu = p.y; pv = p.z; }
        else if (axis == 1) { pu = p.x; pv = p.z; }
        else { pu = p.x; pv = p.y; }

        if (pu < u0 || pu > u1 || pv < v0 || pv > v1) return false;

        rec.t = t;
        rec.point = p;
        Vec3 outNorm;
        if (axis == 0) outNorm = flipNormal ? Vec3(-1,0,0) : Vec3(1,0,0);
        else if (axis == 1) outNorm = flipNormal ? Vec3(0,-1,0) : Vec3(0,1,0);
        else outNorm = flipNormal ? Vec3(0,0,-1) : Vec3(0,0,1);
        rec.setFaceNormal(ray, outNorm);
        rec.matId = matId;
        return true;
    }
};

// ============================================================
// 场景
// ============================================================
struct Scene {
    std::vector<Material> materials;
    std::vector<Sphere>   spheres;
    std::vector<Plane>    planes;

    Vec3 lightPos;
    double lightHW, lightHH;
    Vec3 lightEmit;

    int addMat(Material m) {
        materials.push_back(m);
        return (int)materials.size() - 1;
    }

    bool intersect(const Ray& ray, HitRecord& rec) const {
        bool hit = false;
        double tMax = 1e18;
        HitRecord tmp;

        for (auto& s : spheres) {
            if (s.intersect(ray, 1e-4, tMax, tmp)) {
                hit = true; tMax = tmp.t; rec = tmp;
            }
        }
        for (auto& p : planes) {
            if (p.intersect(ray, 1e-4, tMax, tmp)) {
                hit = true; tMax = tmp.t; rec = tmp;
            }
        }
        return hit;
    }

    Vec3 sampleLight(RNG& rng, Vec3& lightNormal) const {
        double u = (rng.next() * 2 - 1) * lightHW;
        double v = (rng.next() * 2 - 1) * lightHH;
        lightNormal = Vec3(0, -1, 0);
        return lightPos + Vec3(u, 0, v);
    }
    double lightArea() const { return 4.0 * lightHW * lightHH; }
};

// Cornell Box: 盒子 x:[-1,1] y:[-1,1] z:[0,2]
// 相机在 z=-1（盒子外），看向 z=1（盒子中心）
Scene buildCornellBox() {
    Scene sc;

    int white  = sc.addMat(Material::diffuse({0.73, 0.73, 0.73}));
    int red    = sc.addMat(Material::diffuse({0.65, 0.05, 0.05}));
    int green  = sc.addMat(Material::diffuse({0.12, 0.45, 0.15}));
    int mirror = sc.addMat(Material::mirror({0.95, 0.95, 0.95}));
    int glass  = sc.addMat(Material::glass(1.5));
    int light  = sc.addMat(Material::emissive({15, 15, 12}));

    // 地板 y=-1，法线朝上(+y)
    sc.planes.push_back({1, -1.0, -1.0, 1.0, 0.0, 2.0, white, false});
    // 天花板 y=1，法线朝下(-y)
    sc.planes.push_back({1,  1.0, -1.0, 1.0, 0.0, 2.0, white, true});
    // 背墙 z=2，法线朝 -z
    sc.planes.push_back({2,  2.0, -1.0, 1.0, -1.0, 1.0, white, true});
    // 左墙 x=-1，法线朝 +x
    sc.planes.push_back({0, -1.0, -1.0, 1.0, 0.0, 2.0, red,   false});
    // 右墙 x=1，法线朝 -x
    sc.planes.push_back({0,  1.0, -1.0, 1.0, 0.0, 2.0, green, true});

    // 天花板光源（y=0.999，法线朝下）
    sc.planes.push_back({1, 0.999, -0.35, 0.35, 0.65, 1.35, light, true});
    sc.lightPos   = {0.0, 0.999, 1.0};
    sc.lightHW    = 0.35;
    sc.lightHH    = 0.35;
    sc.lightEmit  = {15, 15, 12};

    // 左侧镜面球（前方偏左）
    sc.spheres.push_back({{-0.42, -0.58, 0.8}, 0.42, mirror});
    // 右侧玻璃球（后方偏右）
    sc.spheres.push_back({{ 0.42, -0.58, 1.4}, 0.42, glass});

    return sc;
}

// ============================================================
// SPPM 数据结构
// ============================================================
struct HitPoint {
    Vec3 pos;
    Vec3 normal;
    Vec3 weight;
    Vec3 flux;
    double radius2;
    int photonCount;
    int pixelIdx;
    bool valid;  // 是否有效（打到漫反射表面）
    bool isLight; // 直接打到光源
    Vec3 lightColor; // 光源颜色

    HitPoint() : radius2(0.04), photonCount(0), pixelIdx(-1), valid(false), isLight(false) {
        flux = {0,0,0};
        weight = {0,0,0};
    }
};

// ============================================================
// kd树（光子查找加速）
// ============================================================
struct Photon {
    Vec3 pos;
    Vec3 power;
    Vec3 dir;
};

struct KdNode {
    int photonIdx;
    int left, right;
    int splitAxis;
};

struct KdTree {
    std::vector<KdNode> nodes;
    std::vector<Photon>* photonsPtr = nullptr;
    std::vector<int> buildIdx;
    int root = -1;
    int nodeCount = 0;

    void build(std::vector<Photon>& photons) {
        if (photons.empty()) { root = -1; nodeCount = 0; return; }
        photonsPtr = &photons;
        buildIdx.resize(photons.size());
        for (int i = 0; i < (int)photons.size(); i++) buildIdx[i] = i;
        nodes.resize(photons.size());
        nodeCount = 0;
        root = buildRec(0, (int)buildIdx.size(), 0);
    }

    int buildRec(int lo, int hi, int depth) {
        if (lo >= hi) return -1;
        int axis = depth % 3;
        int mid = (lo + hi) / 2;
        std::nth_element(buildIdx.begin() + lo,
                         buildIdx.begin() + mid,
                         buildIdx.begin() + hi,
            [&](int a, int b) {
                auto& pa = (*photonsPtr)[a].pos;
                auto& pb = (*photonsPtr)[b].pos;
                if (axis == 0) return pa.x < pb.x;
                if (axis == 1) return pa.y < pb.y;
                return pa.z < pb.z;
            });
        int nodeIdx = nodeCount++;
        int photonIdx = buildIdx[mid];
        nodes[nodeIdx].photonIdx = photonIdx;
        nodes[nodeIdx].splitAxis = axis;
        nodes[nodeIdx].left  = buildRec(lo, mid, depth+1);
        nodes[nodeIdx].right = buildRec(mid+1, hi, depth+1);
        return nodeIdx;
    }

    void query(int nodeIdx, const Vec3& pos, double r2,
               std::vector<int>& result) const {
        if (nodeIdx < 0 || !photonsPtr) return;
        const KdNode& n = nodes[nodeIdx];
        const Photon& ph = (*photonsPtr)[n.photonIdx];
        Vec3 d = pos - ph.pos;
        if (d.length2() <= r2) result.push_back(n.photonIdx);
        int axis = n.splitAxis;
        double diff;
        if (axis == 0) diff = d.x;
        else if (axis == 1) diff = d.y;
        else diff = d.z;

        if (diff * diff <= r2) {
            query(n.left, pos, r2, result);
            query(n.right, pos, r2, result);
        } else if (diff < 0) {
            query(n.left, pos, r2, result);
        } else {
            query(n.right, pos, r2, result);
        }
    }
};

// ============================================================
// Fresnel（Schlick 近似）
// ============================================================
double fresnelSchlick(double cosI, double n1, double n2) {
    double r0 = (n1 - n2) / (n1 + n2);
    r0 = r0 * r0;
    double c = 1.0 - cosI;
    return r0 + (1.0 - r0) * c*c*c*c*c;
}

// ============================================================
// 路径追踪（相机 → 首个漫反射点 = Hit Point）
// ============================================================
void cameraPass(const Scene& sc, RNG& rng,
                int width, int height,
                std::vector<HitPoint>& hitPoints,
                double initRadius2) {

    int npx = width * height;
    hitPoints.resize(npx);
    for (int i = 0; i < npx; i++) {
        hitPoints[i] = HitPoint();
        hitPoints[i].radius2 = initRadius2;
        hitPoints[i].pixelIdx = i;
    }

    // 相机：位于 z=-1，看向 z=1（盒子中心）
    // 盒子: x[-1,1] y[-1,1] z[0,2]，中心 (0,0,1)
    Vec3 camPos(0, 0, -1.0);
    Vec3 camTarget(0, 0, 1.0);
    Vec3 worldUp(0, 1, 0);
    Vec3 camDir = (camTarget - camPos).normalized();
    Vec3 camRight = camDir.cross(worldUp).normalized();
    // 注意：camDir=(0,0,1), worldUp=(0,1,0)
    // camDir.cross(worldUp) = (0,0,1)x(0,1,0) = (0*0-1*1, 1*0-0*0, 0*1-0*0) = (-1,0,0)
    // 这是右手系，x 轴朝右应该是 +x，所以取反
    camRight = -camRight; // 修正：让 camRight 指向 +x
    Vec3 camUp = camRight.cross(camDir).normalized();

    // FOV 角度：看到盒子宽度 2（x:-1到1），距离 2，halfFOV = atan(1/2) ≈ 26.6°
    double fov = 55.0 * PI / 180.0;
    double halfH = std::tan(fov / 2.0);
    double halfW = halfH * (double)width / height;

    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            int idx = py * width + px;
            // 抖动采样（一个像素一个样本）
            double u = ((px + rng.next()) / width * 2.0 - 1.0) * halfW;
            double v = (1.0 - (py + rng.next()) / height * 2.0) * halfH;
            Vec3 rayDir = (camDir + camRight * u + camUp * v).normalized();
            Ray ray(camPos, rayDir);

            Vec3 throughput(1, 1, 1);
            int maxDepth = 8;

            for (int depth = 0; depth < maxDepth; depth++) {
                HitRecord rec;
                if (!sc.intersect(ray, rec)) break;

                const Material& mat = sc.materials[rec.matId];

                if (!mat.emission.isZero()) {
                    // 直接打到光源
                    hitPoints[idx].isLight = true;
                    hitPoints[idx].lightColor = throughput * mat.emission;
                    hitPoints[idx].valid = false;
                    break;
                }

                if (mat.type == MatType::DIFFUSE) {
                    hitPoints[idx].pos    = rec.point;
                    hitPoints[idx].normal = rec.normal;
                    hitPoints[idx].weight = throughput * mat.albedo;
                    hitPoints[idx].valid  = true;
                    break;
                } else if (mat.type == MatType::MIRROR) {
                    Vec3 refl = ray.dir - rec.normal * 2.0 * ray.dir.dot(rec.normal);
                    throughput = throughput * mat.albedo;
                    ray = Ray(rec.point, refl);
                } else if (mat.type == MatType::GLASS) {
                    double n1 = rec.frontFace ? 1.0 : mat.ior;
                    double n2 = rec.frontFace ? mat.ior : 1.0;
                    double cosI = std::abs(ray.dir.dot(rec.normal));
                    double fr = fresnelSchlick(cosI, n1, n2);
                    double eta = n1 / n2;
                    double sinT2 = eta * eta * (1.0 - cosI * cosI);
                    bool tir = sinT2 >= 1.0;
                    if (tir || rng.next() < fr) {
                        Vec3 refl = ray.dir - rec.normal * 2.0 * ray.dir.dot(rec.normal);
                        ray = Ray(rec.point, refl);
                    } else {
                        Vec3 n = rec.normal;
                        double cosT = std::sqrt(1.0 - sinT2);
                        Vec3 refr = ray.dir * eta + n * (eta * cosI - cosT);
                        ray = Ray(rec.point, refr.normalized());
                    }
                    throughput = throughput * mat.albedo;
                }
            }
        }
    }
}

// ============================================================
// 光子发射
// ============================================================
void emitPhotons(const Scene& sc, RNG& rng, int numPhotons,
                 std::vector<Photon>& photons) {
    photons.clear();
    photons.reserve(numPhotons * 2);

    double lightArea = sc.lightArea();

    for (int i = 0; i < numPhotons; i++) {
        Vec3 lightNorm;
        Vec3 lightPt = sc.sampleLight(rng, lightNorm);
        Vec3 lightDownNorm = Vec3(0, -1, 0); // 朝下

        // 功率（除以采样 PDF：1/area × 1/PI × cos 采样消掉 cos/PI）
        Vec3 power = sc.lightEmit * lightArea;

        // 余弦加权向下半球发射
        Vec3 dir = rng.cosineHemisphere(lightDownNorm);
        Vec3 throughput = power / (double)numPhotons;

        Ray ray(lightPt, dir);

        int maxDepth = 8;
        for (int depth = 0; depth < maxDepth; depth++) {
            HitRecord rec;
            if (!sc.intersect(ray, rec)) break;

            const Material& mat = sc.materials[rec.matId];
            if (!mat.emission.isZero()) break;

            if (mat.type == MatType::DIFFUSE) {
                // 存储光子（到达漫反射表面）
                Photon ph;
                ph.pos   = rec.point;
                ph.power = throughput * mat.albedo;
                ph.dir   = ray.dir;
                photons.push_back(ph);

                // Russian Roulette
                double maxComp = std::max({mat.albedo.x, mat.albedo.y, mat.albedo.z});
                if (depth >= 3) {
                    if (rng.next() > maxComp) break;
                    throughput = throughput * mat.albedo / maxComp;
                } else {
                    throughput = throughput * mat.albedo;
                }
                ray = Ray(rec.point, rng.cosineHemisphere(rec.normal));

            } else if (mat.type == MatType::MIRROR) {
                Vec3 refl = ray.dir - rec.normal * 2.0 * ray.dir.dot(rec.normal);
                throughput = throughput * mat.albedo;
                ray = Ray(rec.point, refl);
            } else if (mat.type == MatType::GLASS) {
                double n1 = rec.frontFace ? 1.0 : mat.ior;
                double n2 = rec.frontFace ? mat.ior : 1.0;
                double cosI = std::abs(ray.dir.dot(rec.normal));
                double fr = fresnelSchlick(cosI, n1, n2);
                double eta = n1 / n2;
                double sinT2 = eta * eta * (1.0 - cosI * cosI);
                bool tir = sinT2 >= 1.0;
                if (tir || rng.next() < fr) {
                    Vec3 refl = ray.dir - rec.normal * 2.0 * ray.dir.dot(rec.normal);
                    ray = Ray(rec.point, refl);
                } else {
                    Vec3 n = rec.normal;
                    double cosT = std::sqrt(std::max(0.0, 1.0 - sinT2));
                    Vec3 refr = ray.dir * eta + n * (eta * cosI - cosT);
                    ray = Ray(rec.point, refr.normalized());
                }
                throughput = throughput * mat.albedo;
            }
        }
    }
}

// ============================================================
// SPPM 密度估计 + 半径收缩
// ============================================================
void updateHitPoints(std::vector<HitPoint>& hitPoints,
                     const KdTree& kdTree,
                     const std::vector<Photon>& photons,
                     double alpha = 0.7) {
    std::vector<int> nearby;
    for (auto& hp : hitPoints) {
        if (!hp.valid) continue;

        nearby.clear();
        kdTree.query(kdTree.root, hp.pos, hp.radius2, nearby);

        if (nearby.empty()) continue;

        int m = (int)nearby.size();
        int n = hp.photonCount;

        Vec3 newFlux(0,0,0);
        for (int ni : nearby) {
            const Photon& ph = photons[ni];
            double cosD = std::max(0.0, hp.normal.dot(-ph.dir));
            newFlux += ph.power * (cosD / PI);
        }

        // SPPM 更新
        double ratio = (double)(n + alpha * m) / (double)(n + m);
        hp.radius2 *= ratio;
        hp.flux = (hp.flux + hp.weight * newFlux) * ratio;
        hp.photonCount += (int)(alpha * m);
    }
}

// ============================================================
// 色调映射 + Gamma
// ============================================================
double aces(double x) {
    double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return std::clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.0, 1.0);
}
uint8_t toU8(double v) {
    v = aces(v);
    return (uint8_t)(std::pow(std::clamp(v, 0.0, 1.0), 1.0/2.2) * 255.0 + 0.5);
}

// ============================================================
// 主函数
// ============================================================
int main() {
    const int WIDTH  = 512;
    const int HEIGHT = 512;
    const int ITERS  = 40;
    const int PHOTONS_PER_ITER = 150000;

    Scene sc = buildCornellBox();
    RNG rng(99887766);

    printf("SPPM: %dx%d, %d iters, %d photons/iter\n",
           WIDTH, HEIGHT, ITERS, PHOTONS_PER_ITER);
    fflush(stdout);

    // Step 1: 相机 Pass
    printf("[1/3] Camera pass...\n"); fflush(stdout);
    std::vector<HitPoint> hitPoints;
    double initRadius2 = 0.1 * 0.1; // 初始搜索半径 0.1
    cameraPass(sc, rng, WIDTH, HEIGHT, hitPoints, initRadius2);

    // 统计有效 HP
    int validHPs = 0;
    for (auto& hp : hitPoints) if (hp.valid) validHPs++;
    printf("  Valid hit points: %d/%d\n", validHPs, WIDTH*HEIGHT);
    fflush(stdout);

    // Step 2~3: 迭代
    long long totalPhotonsEmitted = 0;
    for (int iter = 0; iter < ITERS; iter++) {
        printf("[2/3] Iter %2d/%d: ", iter+1, ITERS); fflush(stdout);

        std::vector<Photon> photons;
        emitPhotons(sc, rng, PHOTONS_PER_ITER, photons);
        totalPhotonsEmitted += PHOTONS_PER_ITER;
        printf("emitted %d photons, stored %d\n", PHOTONS_PER_ITER, (int)photons.size());
        fflush(stdout);

        if (photons.empty()) continue;

        // 建 kd树
        KdTree kdt;
        kdt.build(photons);

        // 更新 hit points
        updateHitPoints(hitPoints, kdt, photons, 0.7);

        // === DEBUG iter 0 ===
        if (iter == 0) {
            int hpWithPhotons = 0;
            int totalPhotonsInHP = 0;
            for (auto& hp : hitPoints) {
                if (hp.photonCount > 0) {
                    hpWithPhotons++;
                    totalPhotonsInHP += hp.photonCount;
                }
            }
            printf("  After iter 1: HPs with photons=%d, avg photons per valid HP=%.2f\n",
                   hpWithPhotons, (validHPs>0)?(double)totalPhotonsInHP/validHPs:0.0);
            
            // 找一个有效 HP，检查其附近是否有光子
            Vec3 testPos; bool found=false;
            for (auto& hp : hitPoints) {
                if (hp.valid) { testPos=hp.pos; found=true; break; }
            }
            if (found) {
                printf("  First valid HP pos: (%.3f,%.3f,%.3f) r=%.4f\n",
                       testPos.x,testPos.y,testPos.z,std::sqrt(hitPoints[0].radius2));
                // 暴力查找附近光子（对 photons）
                int nearby=0;
                for (auto& ph : photons) {
                    Vec3 d = testPos - ph.pos;
                    if (d.length2() <= hitPoints[0].radius2) nearby++;
                }
                printf("  Brute force photons near first HP: %d\n", nearby);
            }
            fflush(stdout);
        }

    }
    printf("[3/3] Rendering image...\n"); fflush(stdout);

    // 输出图像
    std::vector<uint8_t> img(WIDTH * HEIGHT * 3, 0);

    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        const HitPoint& hp = hitPoints[i];
        Vec3 color(0,0,0);

        if (hp.isLight) {
            color = hp.lightColor;
        } else if (hp.valid && hp.photonCount > 0 && hp.radius2 > 1e-15) {
            // 辐射亮度 = flux / (π * r² * N_total)
            double scale = 1.0 / (PI * hp.radius2 * (double)totalPhotonsEmitted);
            color = hp.flux * scale;
        }

        int row = i / WIDTH;
        int col = i % WIDTH;
        int outIdx = (HEIGHT - 1 - row) * WIDTH + col;
        img[outIdx * 3 + 0] = toU8(color.x);
        img[outIdx * 3 + 1] = toU8(color.y);
        img[outIdx * 3 + 2] = toU8(color.z);
    }

    stbi_write_png("sppm_output.png", WIDTH, HEIGHT, 3, img.data(), WIDTH * 3);
    printf("✅ 图像已保存: sppm_output.png\n");
    fflush(stdout);
    return 0;
}
