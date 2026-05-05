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
    Vec3 uniformSphere() {
        double z = 1.0 - 2.0 * next();
        double r = std::sqrt(std::max(0.0, 1.0 - z*z));
        double phi = 2.0 * M_PI * next();
        return {r * std::cos(phi), r * std::sin(phi), z};
    }
    Vec3 cosineHemisphere(const Vec3& n) {
        // cosine weighted hemisphere sampling
        double u1 = next(), u2 = next();
        double r = std::sqrt(u1);
        double theta = 2.0 * M_PI * u2;
        double x = r * std::cos(theta);
        double y = r * std::sin(theta);
        double z = std::sqrt(std::max(0.0, 1.0 - u1));
        // build ONB
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
    Vec3 albedo;      // 漫反射/镜面颜色
    Vec3 emission;    // 发光
    double ior;       // 折射率（玻璃用）

    static Material diffuse(Vec3 col) {
        Material m; m.type = MatType::DIFFUSE; m.albedo = col; m.ior = 1.0; return m;
    }
    static Material mirror(Vec3 col) {
        Material m; m.type = MatType::MIRROR; m.albedo = col; m.ior = 1.0; return m;
    }
    static Material glass(double ior = 1.5) {
        Material m; m.type = MatType::GLASS; m.albedo = {1,1,1}; m.ior = ior; return m;
    }
    static Material emissive(Vec3 emit) {
        Material m; m.type = MatType::DIFFUSE; m.albedo = {0,0,0}; m.emission = emit; m.ior = 1.0; return m;
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
    int axis;     // 0=YZ, 1=XZ, 2=XY
    double pos;
    double u0, u1, v0, v1;
    int matId;
    bool flipNormal;

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        // axis 0: 固定x; axis 1: 固定y; axis 2: 固定z
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

// 辅助：让 Vec3 支持下标
double& getComp(Vec3& v, int i) {
    if (i==0) return v.x;
    if (i==1) return v.y;
    return v.z;
}

// ============================================================
// 场景
// ============================================================
struct Scene {
    std::vector<Material> materials;
    std::vector<Sphere>   spheres;
    std::vector<Plane>    planes;

    // 光源：矩形面光（Cornell Box 天花板光源）
    Vec3 lightPos;    // 光源中心
    double lightHW;  // 半宽
    double lightHH;  // 半高
    Vec3 lightEmit;
    int lightMatId;

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

    // 从光源采样一个点
    Vec3 sampleLight(RNG& rng, Vec3& lightNormal) const {
        double u = (rng.next() * 2 - 1) * lightHW;
        double v = (rng.next() * 2 - 1) * lightHH;
        lightNormal = Vec3(0, -1, 0);
        return lightPos + Vec3(u, 0, v);
    }
    double lightArea() const { return 4.0 * lightHW * lightHH; }
};

Scene buildCornellBox() {
    Scene sc;

    // 材质
    int white  = sc.addMat(Material::diffuse({0.73, 0.73, 0.73}));
    int red    = sc.addMat(Material::diffuse({0.65, 0.05, 0.05}));
    int green  = sc.addMat(Material::diffuse({0.12, 0.45, 0.15}));
    int mirror = sc.addMat(Material::mirror({0.95, 0.95, 0.95}));
    int glass  = sc.addMat(Material::glass(1.5));
    int light  = sc.addMat(Material::emissive({15, 15, 12}));

    // Cornell Box 尺寸：[-1,1]^3，相机在 z=-3 看向 z=0
    // 地板 y=-1（法线朝上）
    sc.planes.push_back({1, -1.0, -1.0, 1.0, -1.0, 1.0, white, false});
    // 天花板 y=1（法线朝下）
    sc.planes.push_back({1,  1.0, -1.0, 1.0, -1.0, 1.0, white, true});
    // 背墙 z=1（法线朝 -z）
    sc.planes.push_back({2,  1.0, -1.0, 1.0, -1.0, 1.0, white, true});
    // 左墙 x=-1（法线朝 +x）
    sc.planes.push_back({0, -1.0, -1.0, 1.0, -1.0, 1.0, red,   false});
    // 右墙 x=1（法线朝 -x）
    sc.planes.push_back({0,  1.0, -1.0, 1.0, -1.0, 1.0, green, true});

    // 天花板光源（小矩形平面）
    sc.planes.push_back({1, 0.999, -0.35, 0.35, -0.35, 0.35, light, true});
    sc.lightPos   = {0.0, 0.999, 0.0};
    sc.lightHW    = 0.35;
    sc.lightHH    = 0.35;
    sc.lightEmit  = {15, 15, 12};
    sc.lightMatId = light;

    // 左侧镜面球
    sc.spheres.push_back({{-0.45, -0.65, 0.2}, 0.35, mirror});
    // 右侧玻璃球
    sc.spheres.push_back({{ 0.45, -0.65, -0.2}, 0.35, glass});

    return sc;
}

// ============================================================
// SPPM 数据结构
// ============================================================
struct HitPoint {
    Vec3 pos;       // 位置
    Vec3 normal;    // 法线
    Vec3 weight;    // 路径吞吐量（从相机出发）
    Vec3 flux;      // 累积光子通量
    double radius2; // 当前搜索半径²
    int photonCount;// 累积光子数
    int pixelIdx;   // 对应像素

    HitPoint() : radius2(0.04), photonCount(0), pixelIdx(-1) {
        flux = {0,0,0};
    }
};

// ============================================================
// kd树（光子查找加速）
// ============================================================
struct Photon {
    Vec3 pos;
    Vec3 power;
    Vec3 dir;       // 入射方向
};

struct KdNode {
    Photon photon;
    int left, right; // 子节点下标，-1 表示空
    int splitAxis;
};

struct KdTree {
    std::vector<KdNode> nodes;
    std::vector<int> indices;
    int root = -1;

    void build(std::vector<Photon>& photons) {
        if (photons.empty()) return;
        indices.resize(photons.size());
        for (int i = 0; i < (int)photons.size(); i++) indices[i] = i;
        nodes.resize(photons.size());
        for (int i = 0; i < (int)photons.size(); i++) {
            nodes[i].photon = photons[i];
            nodes[i].left = nodes[i].right = -1;
        }
        root = buildRec(photons, indices, 0, (int)indices.size(), 0);
    }

    int buildRec(std::vector<Photon>& photons, std::vector<int>& idx,
                 int lo, int hi, int depth) {
        if (lo >= hi) return -1;
        int axis = depth % 3;
        int mid = (lo + hi) / 2;
        std::nth_element(idx.begin() + lo, idx.begin() + mid, idx.begin() + hi,
            [&](int a, int b) {
                auto& pa = photons[a].pos;
                auto& pb = photons[b].pos;
                if (axis == 0) return pa.x < pb.x;
                if (axis == 1) return pa.y < pb.y;
                return pa.z < pb.z;
            });
        int nodeIdx = idx[mid];
        nodes[nodeIdx].splitAxis = axis;
        nodes[nodeIdx].left  = buildRec(photons, idx, lo, mid, depth+1);
        nodes[nodeIdx].right = buildRec(photons, idx, mid+1, hi, depth+1);
        return nodeIdx;
    }

    // 查找 pos 附近半径 r 内所有光子
    void query(int nodeIdx, const Vec3& pos, double r2,
               std::vector<int>& result) const {
        if (nodeIdx < 0) return;
        const KdNode& n = nodes[nodeIdx];
        Vec3 d = pos - n.photon.pos;
        if (d.length2() <= r2) result.push_back(nodeIdx);
        int axis = n.splitAxis;
        double diff;
        if (axis == 0) diff = d.x;
        else if (axis == 1) diff = d.y;
        else diff = d.z;

        if (diff*diff <= r2) {
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

    // 相机参数
    Vec3 camPos(0, 0, -2.8);
    Vec3 camDir(0, 0, 1);
    Vec3 camUp(0, 1, 0);
    Vec3 camRight = camDir.cross(camUp).normalized();
    camUp = camRight.cross(camDir).normalized();
    double fov = 40.0 * M_PI / 180.0;
    double halfH = std::tan(fov / 2.0);
    double halfW = halfH * (double)width / height;

    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            int idx = py * width + px;
            // NDC
            double u = ((px + 0.5) / width * 2.0 - 1.0) * halfW;
            double v = (1.0 - (py + 0.5) / height * 2.0) * halfH;
            Vec3 rayDir = (camDir + camRight * u + camUp * v).normalized();
            Ray ray(camPos, rayDir);

            Vec3 throughput(1, 1, 1);
            int maxDepth = 8;

            for (int depth = 0; depth < maxDepth; depth++) {
                HitRecord rec;
                if (!sc.intersect(ray, rec)) break;

                const Material& mat = sc.materials[rec.matId];

                // 如果打到光源，hitpoint 直接记录发光
                if (!mat.emission.isZero()) {
                    hitPoints[idx].weight = throughput * mat.emission;
                    hitPoints[idx].pos = rec.point;
                    hitPoints[idx].normal = rec.normal;
                    break;
                }

                if (mat.type == MatType::DIFFUSE) {
                    // 找到漫反射点，设为 Hit Point
                    hitPoints[idx].pos    = rec.point;
                    hitPoints[idx].normal = rec.normal;
                    hitPoints[idx].weight = throughput * mat.albedo;
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
                        // 全内反射 or Fresnel 反射
                        Vec3 refl = ray.dir - rec.normal * 2.0 * ray.dir.dot(rec.normal);
                        ray = Ray(rec.point, refl);
                    } else {
                        // 折射
                        Vec3 n = rec.normal;
                        Vec3 refr = ray.dir * eta + n * (eta * cosI - std::sqrt(1.0 - sinT2));
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
    photons.reserve(numPhotons);

    double lightArea = sc.lightArea();

    for (int i = 0; i < numPhotons; i++) {
        // 从光源采样发射位置
        Vec3 lightNorm;
        Vec3 lightPt = sc.sampleLight(rng, lightNorm);

        // 光子初始功率（面光源：power = emit * area）
        Vec3 power = sc.lightEmit * lightArea / (double)numPhotons;

        // 余弦加权向下发射
        Vec3 dir = rng.cosineHemisphere(-lightNorm); // 向下半球

        Ray ray(lightPt, dir);
        Vec3 throughput = power * M_PI; // cos 加权采样消掉了 cos/pi

        int maxDepth = 6;
        for (int depth = 0; depth < maxDepth; depth++) {
            HitRecord rec;
            if (!sc.intersect(ray, rec)) break;

            const Material& mat = sc.materials[rec.matId];
            if (!mat.emission.isZero()) break; // 打到光源停止

            if (mat.type == MatType::DIFFUSE) {
                // 存储光子
                Photon ph;
                ph.pos   = rec.point;
                ph.power = throughput * mat.albedo;
                ph.dir   = ray.dir;
                photons.push_back(ph);

                // Russian Roulette
                double maxComp = std::max({mat.albedo.x, mat.albedo.y, mat.albedo.z});
                if (rng.next() > maxComp || depth >= 4) break;
                throughput = throughput * mat.albedo / maxComp;

                // 余弦加权漫反射
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
                    Vec3 refr = ray.dir * eta + n * (eta * cosI - std::sqrt(1.0 - sinT2));
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
                     double alpha = 0.7) {
    std::vector<int> nearby;
    for (auto& hp : hitPoints) {
        if (hp.pixelIdx < 0) continue;
        if (hp.weight.isZero()) continue;

        nearby.clear();
        kdTree.query(kdTree.root, hp.pos, hp.radius2, nearby);

        if (nearby.empty()) continue;

        int m = (int)nearby.size();
        int n = hp.photonCount;

        // 新光子通量贡献
        Vec3 newFlux(0,0,0);
        for (int ni : nearby) {
            const Photon& ph = kdTree.nodes[ni].photon;
            double cosD = std::max(0.0, hp.normal.dot(-ph.dir));
            newFlux += ph.power * cosD;
        }

        // SPPM 更新（Hachisuka et al. 2009）
        if (n + m > 0) {
            double ratio = (n + alpha * m) / (double)(n + m);
            hp.radius2 *= ratio;
            hp.flux = (hp.flux + hp.weight * newFlux) * ratio;
            hp.photonCount += (int)(alpha * m);
        }
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
    const int ITERS  = 32;        // 迭代轮数（减少以加速，可增加提升质量）
    const int PHOTONS_PER_ITER = 100000;  // 每轮光子数

    Scene sc = buildCornellBox();
    RNG rng(12345);

    printf("SPPM: %dx%d, %d iters, %d photons/iter\n",
           WIDTH, HEIGHT, ITERS, PHOTONS_PER_ITER);
    fflush(stdout);

    // Step 1: 相机 Pass（只做一次）
    printf("[1/3] Camera pass...\n"); fflush(stdout);
    std::vector<HitPoint> hitPoints;
    double initRadius2 = 0.02 * 0.02 * 4.0; // 初始搜索半径
    cameraPass(sc, rng, WIDTH, HEIGHT, hitPoints, initRadius2);


    // === DEBUG: 统计有效 hit points ===
    {
        int validHPs = 0, lightHPs = 0;
        for (auto& hp : hitPoints) {
            if (!hp.weight.isZero()) {
                if (hp.weight.x > 5.0) lightHPs++;
                else validHPs++;
            }
        }
        printf("DEBUG: valid diffuse HPs=%d, light HPs=%d, total=%d\n",
               validHPs, lightHPs, (int)hitPoints.size());
        if (validHPs > 0) {
            // 打印前几个 HP 的位置
            int cnt=0;
            for (auto& hp : hitPoints) {
                if (!hp.weight.isZero() && hp.weight.x <= 5.0 && cnt < 5) {
                    printf("  HP pos=(%.3f,%.3f,%.3f) w=(%.3f,%.3f,%.3f) r=%.4f\n",
                           hp.pos.x,hp.pos.y,hp.pos.z,
                           hp.weight.x,hp.weight.y,hp.weight.z,
                           std::sqrt(hp.radius2));
                    cnt++;
                }
            }
        }
    }
    fflush(stdout);

    // Step 2~3: 迭代光子发射 + 密度估计
    for (int iter = 0; iter < ITERS; iter++) {
        printf("[2/3] Iter %d/%d: emitting photons...\r", iter+1, ITERS);
        fflush(stdout);

        std::vector<Photon> photons;
        emitPhotons(sc, rng, PHOTONS_PER_ITER, photons);

        // 建 kd树
        KdTree kdt;
        kdt.build(photons);

        // 更新 hit points
        updateHitPoints(hitPoints, kdt, 0.7);
    }
    printf("\n[3/3] Rendering to image...\n"); fflush(stdout);

    // 输出图像
    std::vector<uint8_t> img(WIDTH * HEIGHT * 3);
    int totalPhotons = ITERS * PHOTONS_PER_ITER;

    for (int i = 0; i < WIDTH * HEIGHT; i++) {
        const HitPoint& hp = hitPoints[i];
        Vec3 color(0,0,0);

        if (hp.photonCount > 0 && hp.radius2 > 1e-12) {
            // 辐射度 = 通量 / (π * r² * N_total)
            double scale = 1.0 / (M_PI * hp.radius2 * totalPhotons);
            color = hp.flux * scale;
        }
        // 直接发光
        if (!hp.weight.isZero() && hp.weight.x > 5.0) {
            // 打到光源
            color = hp.weight;
        }

        int row = i / WIDTH;
        int col = i % WIDTH;
        // 翻转 y（图像坐标 y 朝下）
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
