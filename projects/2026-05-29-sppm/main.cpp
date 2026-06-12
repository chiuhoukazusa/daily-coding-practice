/*
 * Stochastic Progressive Photon Mapping (SPPM) Renderer
 * ======================================================
 * 完整实现：直接光 + 焦散/GI
 *
 * 渲染方程分解：
 *   L = L_direct + L_caustic
 *   L_direct: 从相机到漫射面的直接光（Next Event Estimation / shadow ray）
 *   L_caustic: 光子映射 —— 捕捉透过玻璃的折射焦散
 *
 * 场景：Cornell Box + 玻璃球（产生焦散）+ 镜面小球
 * 输出：sppm_final.png（合成图）+ sppm_caustic.png（焦散通道）
 *
 * 坐标系：右手系，Y向上，Z朝向相机（Cornell Box 0~555）
 * 编译：g++ main.cpp -o sppm_renderer -std=c++17 -O2 -Wall -Wextra
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <random>
#include <limits>
#include <unordered_map>
#include <cstdio>

// ─────────────────────── 数学基础 ───────────────────────

static const double PI = 3.14159265358979323846;
static const double INF = std::numeric_limits<double>::infinity();
static const double EPS = 1e-6;

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    explicit Vec3(double v) : x(v), y(v), z(v) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return Vec3(x+b.x, y+b.y, z+b.z); }
    Vec3 operator-(const Vec3& b) const { return Vec3(x-b.x, y-b.y, z-b.z); }
    Vec3 operator-() const { return Vec3(-x, -y, -z); }
    Vec3 operator*(double t) const { return Vec3(x*t, y*t, z*t); }
    Vec3 operator*(const Vec3& b) const { return Vec3(x*b.x, y*b.y, z*b.z); }
    Vec3 operator/(double t) const { return Vec3(x/t, y/t, z/t); }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator*=(double t) { x*=t; y*=t; z*=t; return *this; }

    double dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const { return Vec3(y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x); }
    double len2() const { return x*x + y*y + z*z; }
    double len() const { return std::sqrt(len2()); }
    Vec3 normalized() const { double l = len(); return l > EPS ? *this / l : Vec3(); }
    bool isZero() const { return len2() < EPS*EPS; }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ─────────────────────── 随机数 ───────────────────────

static std::mt19937_64 rng(123456789ULL);
static std::uniform_real_distribution<double> dist01(0.0, 1.0);
inline double rand01() { return dist01(rng); }

// 余弦加权半球采样（重要性采样漫射BRDF）
Vec3 cosineHemisphere(const Vec3& normal) {
    double r1 = rand01(), r2 = rand01();
    double phi = 2.0 * PI * r1;
    double sr2 = std::sqrt(r2);
    double lx = std::cos(phi) * sr2;
    double lz = std::sin(phi) * sr2;
    double ly = std::sqrt(std::max(0.0, 1.0 - r2));
    // 构建正交基：避免 normal 与 up 平行
    Vec3 up;
    if (std::abs(normal.y) < 0.999) {
        up = Vec3(0, 1, 0);
    } else {
        up = Vec3(1, 0, 0); // normal 近似 (0,±1,0) 时用 X 轴
    }
    Vec3 tang = normal.cross(up).normalized();
    Vec3 bita = normal.cross(tang);
    return (tang * lx + normal * ly + bita * lz).normalized();
}

// 均匀球面采样
Vec3 uniformSphere() {
    double u = rand01(), v = rand01();
    double theta = 2.0 * PI * u;
    double phi = std::acos(std::max(-1.0, std::min(1.0, 2.0 * v - 1.0)));
    return Vec3(std::sin(phi)*std::cos(theta), std::cos(phi), std::sin(phi)*std::sin(theta));
}

// ─────────────────────── Ray ───────────────────────

struct Ray {
    Vec3 origin, dir;
    Ray(Vec3 o, Vec3 d) : origin(o), dir(d.normalized()) {}
    Vec3 at(double t) const { return origin + dir * t; }
};

// ─────────────────────── 材质 ───────────────────────

enum class MatType { DIFFUSE, MIRROR, GLASS };

struct Material {
    Vec3 albedo;
    Vec3 emission;
    MatType type;
    double ior;

    Material() : albedo(Vec3(0.5)), emission(), type(MatType::DIFFUSE), ior(1.5) {}
    Material(Vec3 a, Vec3 e, MatType t = MatType::DIFFUSE, double i = 1.5)
        : albedo(a), emission(e), type(t), ior(i) {}
};

// ─────────────────────── 几何体 ───────────────────────

struct HitRecord {
    double t = INF;
    Vec3 pos, normal;
    const Material* mat = nullptr;
    bool frontFace = true;

    void setNormal(const Ray& r, Vec3 outN) {
        frontFace = r.dir.dot(outN) < 0;
        normal = frontFace ? outN : -outN;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    Material mat;

    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.dir.len2();
        double hb = oc.dot(r.dir);
        double c = oc.len2() - radius*radius;
        double disc = hb*hb - a*c;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double t = (-hb - sq) / a;
        if (t < tMin || t > tMax) {
            t = (-hb + sq) / a;
            if (t < tMin || t > tMax) return false;
        }
        rec.t = t;
        rec.pos = r.at(t);
        rec.setNormal(r, (rec.pos - center) * (1.0 / radius));
        rec.mat = &mat;
        return true;
    }
};

// XZ 平面（水平面，法线 ±Y）
struct AARectXZ {
    double x0, x1, z0, z1, k;
    Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.y) < EPS) return false;
        double t = (k - r.origin.y) / r.dir.y;
        if (t < tMin || t > tMax) return false;
        double x = r.origin.x + t*r.dir.x;
        double z = r.origin.z + t*r.dir.z;
        if (x < x0 || x > x1 || z < z0 || z > z1) return false;
        rec.t = t; rec.pos = r.at(t);
        rec.setNormal(r, Vec3(0,1,0));
        rec.mat = &mat; return true;
    }
};

// XY 平面（垂直，法线 ±Z）
struct AARectXY {
    double x0, x1, y0, y1, k;
    Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.z) < EPS) return false;
        double t = (k - r.origin.z) / r.dir.z;
        if (t < tMin || t > tMax) return false;
        double x = r.origin.x + t*r.dir.x;
        double y = r.origin.y + t*r.dir.y;
        if (x < x0 || x > x1 || y < y0 || y > y1) return false;
        rec.t = t; rec.pos = r.at(t);
        rec.setNormal(r, Vec3(0,0,-1)); // 朝向 -Z（朝向相机）
        rec.mat = &mat; return true;
    }
};

// YZ 平面（垂直，法线 ±X）
struct AARectYZ {
    double y0, y1, z0, z1, k;
    Material mat;
    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        if (std::abs(r.dir.x) < EPS) return false;
        double t = (k - r.origin.x) / r.dir.x;
        if (t < tMin || t > tMax) return false;
        double y = r.origin.y + t*r.dir.y;
        double z = r.origin.z + t*r.dir.z;
        if (y < y0 || y > y1 || z < z0 || z > z1) return false;
        rec.t = t; rec.pos = r.at(t);
        // x=0 面：法线指向 +X（朝盒内）
        // x=555 面：法线指向 -X（朝盒内）
        // setNormal 会根据 frontFace 自动处理
        rec.setNormal(r, Vec3(1,0,0));
        rec.mat = &mat; return true;
    }
};

// ─────────────────────── 场景 ───────────────────────

struct LightRect {
    double x0, x1, z0, z1, y; // 水平光源矩形
    Vec3 emission;
    double area() const { return (x1-x0) * (z1-z0); }
    Vec3 sample(Vec3& normal) const {
        Vec3 p(x0 + rand01()*(x1-x0), y, z0 + rand01()*(z1-z0));
        normal = Vec3(0,-1,0); // 向下照射
        return p;
    }
};

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<AARectXZ> xzRects;
    std::vector<AARectXY> xyRects;
    std::vector<AARectYZ> yzRects;
    LightRect light;

    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tmp; bool any = false; double closest = tMax;
        for (auto& s : spheres)  if (s.hit(r, tMin, closest, tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : xzRects)  if (s.hit(r, tMin, closest, tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : xyRects)  if (s.hit(r, tMin, closest, tmp)) { any=true; closest=tmp.t; rec=tmp; }
        for (auto& s : yzRects)  if (s.hit(r, tMin, closest, tmp)) { any=true; closest=tmp.t; rec=tmp; }
        return any;
    }

    // 阴影测试：pos 到 lightPos 是否被遮挡
    bool isOccluded(Vec3 pos, Vec3 lightPos) const {
        Vec3 dir = lightPos - pos;
        double dist = dir.len();
        Ray shadowRay(pos, dir);
        HitRecord rec;
        if (hit(shadowRay, EPS * 2, dist - EPS * 2, rec)) {
            // 如果命中的是玻璃球，允许穿透（简化）
            return rec.mat->type != MatType::GLASS;
        }
        return false;
    }
};

Scene buildScene() {
    Scene sc;
    Material white(Vec3(0.73, 0.73, 0.73), Vec3());
    Material red(Vec3(0.65, 0.05, 0.05), Vec3());
    Material green(Vec3(0.12, 0.45, 0.15), Vec3());
    Material lightMat(Vec3(1.0, 1.0, 0.9), Vec3(12, 12, 10));
    Material glass(Vec3(1,1,1), Vec3(), MatType::GLASS, 1.5);
    Material mirror(Vec3(0.9, 0.85, 0.8), Vec3(), MatType::MIRROR);

    // Cornell Box 墙面
    sc.xzRects.push_back({0, 555, 0, 555, 0,   white});    // 地面
    sc.xzRects.push_back({0, 555, 0, 555, 555, white});    // 天花板
    sc.xyRects.push_back({0, 555, 0, 555, 555, white});    // 背墙
    sc.yzRects.push_back({0, 555, 0, 555, 0,   green});    // 左墙（x=0）
    sc.yzRects.push_back({0, 555, 0, 555, 555, red});      // 右墙（x=555）

    // 光源（仅渲染到场景，同时作为直接光采样）
    sc.xzRects.push_back({213, 343, 227, 332, 554, lightMat}); // 天花板光源

    sc.light = {213, 343, 227, 332, 554.0, Vec3(12, 12, 10)};

    // 玻璃球（产生焦散）- 球心在 (278,97,250) r=80, IOR=1.5
    // 第一面证明：薄透镜公式 f=r/(2(n-1))=80, do=554-97=457, di=97 → 焦散点在 y=0（地面）
    sc.spheres.push_back({Vec3(278, 97, 250), 80, glass});

    // 镜面小球
    sc.spheres.push_back({Vec3(100, 70, 120), 70, mirror});

    return sc;
}

// ─────────────────────── 折射/反射 ───────────────────────

Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - n * (2.0 * v.dot(n));
}

bool refract(const Vec3& v, const Vec3& n, double niOverNt, Vec3& out) {
    Vec3 uv = v.normalized();
    double dt = uv.dot(n);
    double disc = 1.0 - niOverNt*niOverNt*(1.0-dt*dt);
    if (disc <= 0) return false;
    out = (uv - n*dt) * niOverNt - n * std::sqrt(disc);
    return true;
}

double schlick(double cosine, double ior) {
    double r0 = (1.0-ior)/(1.0+ior); r0 *= r0;
    return r0 + (1.0-r0)*std::pow(1.0-cosine, 5.0);
}

// ─────────────────────── 直接光估计（Next Event Estimation）───────────────────────

// 从 pos（法线 normal，albedo）向光源发射 shadow ray 估计直接照明
Vec3 directLighting(const Vec3& pos, const Vec3& normal, const Vec3& albedo, const Scene& sc) {
    Vec3 lnormal;
    Vec3 lpos = sc.light.sample(lnormal);

    Vec3 dir = lpos - pos;
    double dist2 = dir.len2();
    double dist = std::sqrt(dist2);
    Vec3 dirN = dir * (1.0 / dist);

    double cosTheta = std::max(0.0, dirN.dot(normal));
    if (cosTheta < EPS) return Vec3();

    // shadow ray
    if (sc.isOccluded(pos + normal * EPS, lpos)) return Vec3();

    // 几何因子（面光源到着色点）
    double cosLight = std::max(0.0, (-dirN).dot(lnormal));
    double lightArea = sc.light.area();
    double geom = cosTheta * cosLight * lightArea / dist2;

    // Lambert BRDF: albedo / PI
    return albedo * (1.0 / PI) * sc.light.emission * geom;
}

// ─────────────────────── 路径追踪（直接光 + 镜面/玻璃路径）───────────────────────

// 返回直接光贡献（漫射面用 NEE，镜面/玻璃继续追踪）
Vec3 traceDirectPath(const Ray& ray, const Scene& sc, int maxDepth = 6) {
    Vec3 result;
    Vec3 throughput(1.0);
    Ray r = ray;

    for (int depth = 0; depth < maxDepth; ++depth) {
        HitRecord rec;
        if (!sc.hit(r, EPS, INF, rec)) break;
        const Material* mat = rec.mat;

        // 命中光源
        if (!mat->emission.isZero()) {
            if (depth == 0) result += throughput * mat->emission; // 只在第一跳显示光源
            break;
        }

        if (mat->type == MatType::DIFFUSE) {
            // 漫射面：直接光估计 + 停止（不做间接光，间接光由光子映射负责）
            result += throughput * directLighting(rec.pos, rec.normal, mat->albedo, sc);
            break;
        } else if (mat->type == MatType::MIRROR) {
            throughput = throughput * mat->albedo;
            Vec3 refl = reflect(r.dir, rec.normal).normalized();
            r = Ray(rec.pos + rec.normal * EPS, refl);
        } else { // GLASS
            double niOverNt = rec.frontFace ? (1.0/mat->ior) : mat->ior;
            double cosT = std::min(-r.dir.normalized().dot(rec.normal), 1.0);
            double sinT = std::sqrt(std::max(0.0, 1.0-cosT*cosT));
            bool totalR = (niOverNt * sinT > 1.0);
            double rProb = totalR ? 1.0 : schlick(cosT, niOverNt);
            Vec3 side = rec.frontFace ? rec.normal : -rec.normal;
            if (rand01() < rProb) {
                r = Ray(rec.pos + side * EPS, reflect(r.dir, rec.normal).normalized());
            } else {
                Vec3 refr;
                refract(r.dir, rec.normal, niOverNt, refr);
                r = Ray(rec.pos - side * EPS, refr.normalized());
            }
            // 玻璃不衰减 throughput
        }
    }
    return result;
}

// ─────────────────────── 光子追踪 ───────────────────────

struct Photon {
    Vec3 pos, dir, power;
};

std::vector<Photon> tracePhotons(const Scene& sc, int N) {
    std::vector<Photon> photons;
    photons.reserve((size_t)N * 4);

    double lightArea = sc.light.area();
    Vec3 totalPower = sc.light.emission * lightArea * PI;

    for (int i = 0; i < N; ++i) {
        // 光源上均匀采样
        Vec3 lpos(sc.light.x0 + rand01()*(sc.light.x1-sc.light.x0),
                  sc.light.y,
                  sc.light.z0 + rand01()*(sc.light.z1-sc.light.z0));
        // 向下半球发射（余弦加权）
        Vec3 lnormal(0, -1, 0);
        Vec3 ldir = cosineHemisphere(lnormal); // 注意：这会给出法线朝上的余弦分布
        // 由于 lnormal = (0,-1,0)，cosineHemisphere 应该给出 Y<0 的方向
        // 检查：cosineHemisphere(n) 中 ly = sqrt(1-r2) 是沿 n 方向的分量
        // 所以 ldir 的主分量在 lnormal=(0,-1,0) 方向，即 y<0 ✓

        Ray r(lpos, ldir);
        Vec3 power = totalPower * (1.0 / N);

        for (int d = 0; d < 10; ++d) {
            HitRecord rec;
            if (!sc.hit(r, EPS, INF, rec)) break;
            const Material* mat = rec.mat;
            if (!mat->emission.isZero()) break;

            if (mat->type == MatType::DIFFUSE) {
                // 在漫射面沉积光子
                photons.push_back({rec.pos, r.dir, power});
                // 俄罗斯轮盘赌
                double maxAlbedo = std::max({mat->albedo.x, mat->albedo.y, mat->albedo.z});
                maxAlbedo = std::min(maxAlbedo, 0.95);
                if (rand01() > maxAlbedo) break;
                power = power * mat->albedo * (1.0 / maxAlbedo);
                r = Ray(rec.pos + rec.normal * EPS, cosineHemisphere(rec.normal));
            } else if (mat->type == MatType::MIRROR) {
                power = power * mat->albedo;
                r = Ray(rec.pos + rec.normal * EPS, reflect(r.dir, rec.normal).normalized());
            } else { // GLASS
                double niOverNt = rec.frontFace ? (1.0/mat->ior) : mat->ior;
                double cosT = std::min(-r.dir.normalized().dot(rec.normal), 1.0);
                double sinT = std::sqrt(std::max(0.0, 1.0-cosT*cosT));
                bool totalR = (niOverNt * sinT > 1.0);
                double rProb = totalR ? 1.0 : schlick(cosT, niOverNt);
                Vec3 side = rec.frontFace ? rec.normal : -rec.normal;
                if (rand01() < rProb) {
                    r = Ray(rec.pos + side * EPS, reflect(r.dir, rec.normal).normalized());
                } else {
                    Vec3 refr;
                    refract(r.dir, rec.normal, niOverNt, refr);
                    r = Ray(rec.pos - side * EPS, refr.normalized());
                }
            }
        }
    }
    return photons;
}

// ─────────────────────── 空间哈希 ───────────────────────

struct SpatialHash {
    double cellSize = 1.0;
    std::unordered_map<int64_t, std::vector<int>> grid;

    static int64_t hashCell(int ix, int iy, int iz) {
        return (int64_t)ix * 1000003LL ^ (int64_t)iy * 999983LL ^ (int64_t)iz * 1000033LL;
    }
    int cell(double v) const { return (int)std::floor(v / cellSize); }

    void build(const std::vector<Photon>& photons, double radius) {
        cellSize = radius * 2.0;
        grid.clear();
        for (int i = 0; i < (int)photons.size(); ++i) {
            auto& p = photons[i];
            int ix = cell(p.pos.x), iy = cell(p.pos.y), iz = cell(p.pos.z);
            grid[hashCell(ix,iy,iz)].push_back(i);
        }
    }

    void query(const Vec3& pos, double radius, const std::vector<Photon>& photons,
               std::vector<int>& result) const {
        result.clear();
        double r2 = radius * radius;
        int ix0 = cell(pos.x-radius), ix1 = cell(pos.x+radius);
        int iy0 = cell(pos.y-radius), iy1 = cell(pos.y+radius);
        int iz0 = cell(pos.z-radius), iz1 = cell(pos.z+radius);
        for (int ix = ix0; ix <= ix1; ++ix)
        for (int iy = iy0; iy <= iy1; ++iy)
        for (int iz = iz0; iz <= iz1; ++iz) {
            auto it = grid.find(hashCell(ix,iy,iz));
            if (it == grid.end()) continue;
            for (int idx : it->second) {
                Vec3 d = photons[idx].pos - pos;
                if (d.len2() <= r2) result.push_back(idx);
            }
        }
    }
};

// ─────────────────────── SPPM 可见点 ───────────────────────

struct VisiblePoint {
    Vec3 pos, normal, weight; // 路径权重
    double radius, N;
    Vec3 causticFlux; // 焦散/间接通量（光子映射）
    bool valid = false;
};

// 追踪相机光线，找到漫射面上的可见点
// 同时通过 NEE 计算直接光贡献
Vec3 traceCameraWithDirect(const Ray& ray, const Scene& sc, double savedR, double savedN,
                            Vec3 savedFlux, VisiblePoint& vp) {
    vp.valid = false;
    vp.radius = savedR;
    vp.N = savedN;
    vp.causticFlux = savedFlux;

    // 直接光颜色（通过路径追踪）
    Vec3 directColor = traceDirectPath(ray, sc, 6);

    // 同时找可见点（第一个漫射面）
    Ray r = ray;
    Vec3 weight(1.0);

    for (int depth = 0; depth < 8; ++depth) {
        HitRecord rec;
        if (!sc.hit(r, EPS, INF, rec)) break;
        const Material* mat = rec.mat;
        if (!mat->emission.isZero()) break;

        if (mat->type == MatType::DIFFUSE) {
            vp.valid = true;
            vp.pos = rec.pos;
            vp.normal = rec.normal;
            vp.weight = weight * mat->albedo;
            break;
        } else if (mat->type == MatType::MIRROR) {
            weight = weight * mat->albedo;
            r = Ray(rec.pos + rec.normal * EPS, reflect(r.dir, rec.normal).normalized());
        } else { // GLASS
            double niOverNt = rec.frontFace ? (1.0/mat->ior) : mat->ior;
            double cosT = std::min(-r.dir.normalized().dot(rec.normal), 1.0);
            double sinT = std::sqrt(std::max(0.0, 1.0-cosT*cosT));
            bool totalR = (niOverNt * sinT > 1.0);
            double rProb = totalR ? 1.0 : schlick(cosT, niOverNt);
            Vec3 side = rec.frontFace ? rec.normal : -rec.normal;
            if (rand01() < rProb) {
                r = Ray(rec.pos + side * EPS, reflect(r.dir, rec.normal).normalized());
            } else {
                Vec3 refr;
                refract(r.dir, rec.normal, niOverNt, refr);
                r = Ray(rec.pos - side * EPS, refr.normalized());
            }
        }
    }
    return directColor;
}

// 光子收集（更新可见点的焦散通量）
void collectPhotons(VisiblePoint& vp, const std::vector<Photon>& photons,
                    const SpatialHash& hash, double alpha) {
    if (!vp.valid) return;

    static std::vector<int> nearby;
    hash.query(vp.pos, vp.radius, photons, nearby);

    if (nearby.empty()) return;

    int count = 0;
    Vec3 newFlux;
    for (int idx : nearby) {
        const Photon& p = photons[idx];
        if (-p.dir.dot(vp.normal) <= 0) continue;
        newFlux += p.power * (1.0 / PI);
        count++;
    }

    if (count > 0) {
        double Nnew = vp.N + alpha * count;
        double shrink = Nnew / (vp.N + count);
        vp.causticFlux = (vp.causticFlux + vp.weight * newFlux) * shrink;
        vp.radius *= std::sqrt(shrink);
        vp.N = Nnew;
    }
}

// ─────────────────────── 相机 ───────────────────────

struct Camera {
    Vec3 origin, lower_left, horiz, vert;

    Camera(Vec3 eye, Vec3 target, Vec3 up, double vfov, double aspect) {
        double theta = vfov * PI / 180.0;
        double halfH = std::tan(theta / 2.0);
        double halfW = aspect * halfH;
        Vec3 w = (eye - target).normalized();
        Vec3 u = up.cross(w).normalized();
        Vec3 v = w.cross(u);
        origin = eye;
        lower_left = eye - u * halfW - v * halfH - w;
        horiz = u * (2 * halfW);
        vert  = v * (2 * halfH);
    }

    Ray getRay(double s, double t) const {
        return Ray(origin, lower_left + horiz * s + vert * t - origin);
    }
};

// ─────────────────────── 图像 ───────────────────────

struct Image {
    int W, H;
    std::vector<Vec3> pix;
    Image(int w, int h) : W(w), H(h), pix((size_t)w*h) {}
    Vec3& at(int x, int y) { return pix[(size_t)y*W+x]; }
    const Vec3& at(int x, int y) const { return pix[(size_t)y*W+x]; }
};

uint8_t tonemap(double v) {
    v = v / (v + 1.0);   // Reinhard
    v = std::pow(std::max(0.0, v), 1.0/2.2); // gamma
    return (uint8_t)(std::min(1.0, v) * 255.999);
}

void saveImage(const Image& img, const char* fn) {
    std::vector<uint8_t> buf((size_t)img.W * img.H * 3);
    for (int i = 0; i < img.W * img.H; ++i) {
        buf[i*3+0] = tonemap(img.pix[i].x);
        buf[i*3+1] = tonemap(img.pix[i].y);
        buf[i*3+2] = tonemap(img.pix[i].z);
    }
    stbi_write_png(fn, img.W, img.H, 3, buf.data(), img.W * 3);
    printf("  Saved: %s (%dx%d)\n", fn, img.W, img.H);
}

// ─────────────────────── MAIN ───────────────────────

int main() {
    const int W = 400, H = 400;
    const int PASSES = 20;
    const int PHOTONS_PER_PASS = 60000;
    const double ALPHA = 0.7;
    const double INIT_R = 20.0;

    printf("=== SPPM Renderer ===\n");
    printf("Resolution: %dx%d | %d passes | %d photons/pass\n", W, H, PASSES, PHOTONS_PER_PASS);
    printf("Alpha: %.1f | Init radius: %.1f\n\n", ALPHA, INIT_R);

    Scene scene = buildScene();
    Camera cam(Vec3(278,278,-800), Vec3(278,278,0), Vec3(0,1,0), 40.0, (double)W/H);

    // 持久状态（跨 pass）
    std::vector<double> savedRadius(W*H, INIT_R);
    std::vector<double> savedN(W*H, 0.0);
    std::vector<Vec3>   savedFlux((size_t)W*H);
    std::vector<Vec3>   directAccum((size_t)W*H);

    SpatialHash spatialHash;

    for (int pass = 0; pass < PASSES; ++pass) {
        printf("Pass %2d/%d | ", pass+1, PASSES); fflush(stdout);

        // Step 1: 相机光线追踪（直接光 + 建立可见点）
        std::vector<VisiblePoint> vps((size_t)W*H);
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                int idx = y*W+x;
                double u = (x + rand01()) / W;
                double v = (y + rand01()) / H;
                Ray r = cam.getRay(u, v);
                Vec3 dc = traceCameraWithDirect(r, scene, savedRadius[idx], savedN[idx],
                                                savedFlux[idx], vps[idx]);
                directAccum[idx] += dc;
            }
        }

        // Step 2: 光子追踪
        auto photons = tracePhotons(scene, PHOTONS_PER_PASS);
        printf("photons=%zu | ", photons.size()); fflush(stdout);

        // Step 3: 构建空间哈希
        double maxR = *std::max_element(savedRadius.begin(), savedRadius.end());
        spatialHash.build(photons, maxR);

        // Step 4: 收集光子
        int vpCount = 0;
        for (int i = 0; i < W*H; ++i) {
            collectPhotons(vps[i], photons, spatialHash, ALPHA);
            if (vps[i].valid) {
                savedRadius[i] = vps[i].radius;
                savedN[i]      = vps[i].N;
                savedFlux[i]   = vps[i].causticFlux;
                vpCount++;
            }
        }

        double sumR = 0;
        for (int i = 0; i < W*H; ++i) sumR += savedRadius[i];
        printf("vp=%d avgR=%.3f\n", vpCount, sumR/(W*H)); fflush(stdout);
    }

    // 合成最终图像
    Image finalImg(W, H);
    Image causticImg(W, H);
    Image directImg(W, H);

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            int idx = y*W+x;
            int flipY = H-1-y;

            // 直接光：平均
            Vec3 direct = directAccum[idx] * (1.0 / PASSES);
            directImg.at(x, flipY) = direct;

            // 焦散：光子映射
            Vec3 caustic;
            if (savedN[idx] > 0.5) {
                double r2 = savedRadius[idx] * savedRadius[idx];
                // SPPM 辐射率公式： L = flux / (PI * r² * passes)
                // flux 是 PASSES 轮累积的结果，除以 passes 进行时域平均
                double factor = 1.0 / (PI * r2 * (double)PASSES);
                caustic = savedFlux[idx] * factor;
            }
            causticImg.at(x, flipY) = caustic;

            finalImg.at(x, flipY) = direct + caustic;
        }
    }

    printf("\nSaving images...\n");
    saveImage(finalImg,   "sppm_final.png");
    saveImage(causticImg, "sppm_caustic.png");
    saveImage(directImg,  "sppm_direct.png");

    // 输出统计
    double sumR=0, sumG=0, sumB=0;
    int nonBlack = 0;
    for (auto& c : finalImg.pix) {
        sumR += c.x; sumG += c.y; sumB += c.z;
        if (c.len2() > 1e-10) nonBlack++;
    }
    int total = W*H;
    printf("\n=== Image Stats ===\n");
    printf("Non-black pixels: %d/%d (%.1f%%)\n", nonBlack, total, 100.0*nonBlack/total);
    printf("Mean HDR: R=%.4f G=%.4f B=%.4f\n", sumR/total, sumG/total, sumB/total);

    return 0;
}
