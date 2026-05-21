/**
 * Photon Mapping Renderer
 * 
 * 实现经典两阶段全局光照算法：
 * Phase 1: 光子发射 - 从光源发射光子，跟踪并存入 kd-tree
 * Phase 2: 渲染    - 从摄像机出发，查询周围光子估计辐射度
 * 
 * 特性：
 * - kd-tree 加速光子查询（最近邻搜索）
 * - Cornell Box 场景（漫反射+镜面反射）
 * - 直接光照 + 间接光照（通过光子图）
 * - Caustics 焦散效果
 * - 软光栅化输出 PNG（stb_image_write）
 */

#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <limits>
#include <cassert>
#include <memory>
#include <queue>

// ─── stb_image_write (header-only) ───────────────────────────────────────────
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ─── 基础数学 ─────────────────────────────────────────────────────────────────
static const float PI    = 3.14159265358979f;
static const float INF   = std::numeric_limits<float>::max();
static const float EPS   = 1e-4f;

struct Vec3 {
    float x, y, z;
    Vec3(float a=0,float b=0,float c=0): x(a),y(b),z(c){}
    Vec3 operator+(const Vec3& o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)    const{return{x*t,y*t,z*t};}
    Vec3 operator*(const Vec3& o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3 operator/(float t)    const{return{x/t,y/t,z/t};}
    Vec3& operator+=(const Vec3& o){x+=o.x;y+=o.y;z+=o.z;return*this;}
    Vec3  operator-()const{return{-x,-y,-z};}
    float dot(const Vec3& o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3& o)const{
        return{y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len2()const{return dot(*this);}
    float len() const{return std::sqrt(len2());}
    Vec3 norm()const{float l=len();return l>0?(*this)*(1.f/l):Vec3(0);}
};
Vec3 operator*(float t, const Vec3& v){return v*t;}

// ─── 随机数 ───────────────────────────────────────────────────────────────────
static thread_local std::mt19937 rng(42);
static float randf(){return std::uniform_real_distribution<float>(0,1)(rng);}

Vec3 randomHemisphere(const Vec3& n){
    // Cosine-weighted hemisphere sampling
    float r1 = randf(), r2 = randf();
    float sinTheta = std::sqrt(r1);
    float cosTheta = std::sqrt(1.f - r1);
    float phi = 2.f * PI * r2;
    Vec3 local(sinTheta*std::cos(phi), sinTheta*std::sin(phi), cosTheta);
    // Build orthonormal basis
    Vec3 up = std::fabs(n.x) < 0.9f ? Vec3(1,0,0) : Vec3(0,1,0);
    Vec3 t = n.cross(up).norm();
    Vec3 b = n.cross(t);
    return (t*local.x + b*local.y + n*local.z).norm();
}

Vec3 randomSphere(){
    // Uniform sphere sampling
    float r1 = randf(), r2 = randf();
    float z   = 1.f - 2.f*r1;
    float r   = std::sqrt(std::max(0.f, 1.f-z*z));
    float phi = 2.f * PI * r2;
    return {r*std::cos(phi), r*std::sin(phi), z};
}

// ─── 光线 ─────────────────────────────────────────────────────────────────────
struct Ray {
    Vec3 o, d;
    Ray(Vec3 o, Vec3 d): o(o), d(d.norm()){}
    Vec3 at(float t)const{return o + d*t;}
};

// ─── 材质 ─────────────────────────────────────────────────────────────────────
enum MatType { DIFFUSE, MIRROR, GLASS };

struct Material {
    Vec3    color;      // 漫反射颜色 / 玻璃颜色
    Vec3    emission;   // 自发光
    MatType type;
    float   ior;        // 折射率（GLASS 用）
    Material(Vec3 c={1,1,1}, Vec3 e={0,0,0}, MatType t=DIFFUSE, float ior=1.5f)
        : color(c), emission(e), type(t), ior(ior){}
};

// ─── 场景几何 ─────────────────────────────────────────────────────────────────
struct HitInfo {
    float   t = INF;
    Vec3    pos, normal;
    int     matId = -1;
};

struct Sphere {
    Vec3  c; float r; int matId;
    bool intersect(const Ray& ray, float tmin, float tmax, HitInfo& h) const {
        Vec3 oc = ray.o - c;
        float a = ray.d.dot(ray.d);
        float b = 2.f * oc.dot(ray.d);
        float cc = oc.dot(oc) - r*r;
        float disc = b*b - 4.f*a*cc;
        if (disc < 0) return false;
        float sq = std::sqrt(disc);
        float t = (-b - sq)/(2.f*a);
        if (t < tmin || t > tmax) {
            t = (-b + sq)/(2.f*a);
            if (t < tmin || t > tmax) return false;
        }
        h.t = t; h.pos = ray.at(t);
        h.normal = (h.pos - c).norm();
        h.matId = matId;
        return true;
    }
};

struct Plane {
    Vec3 n; float d; int matId; // n·x = d
    bool intersect(const Ray& ray, float tmin, float tmax, HitInfo& h) const {
        float denom = n.dot(ray.d);
        if (std::fabs(denom) < 1e-6f) return false;
        float t = (d - n.dot(ray.o)) / denom;
        if (t < tmin || t > tmax) return false;
        h.t = t; h.pos = ray.at(t);
        h.normal = (denom < 0) ? n : (n*-1.f);
        h.matId = matId;
        return true;
    }
};

// Cornell Box 场景
std::vector<Material> materials;
std::vector<Sphere>   spheres;
std::vector<Plane>    planes;

// 光源中心和面积（面光源用球体近似）
Vec3  lightCenter;
float lightRadius;
int   lightMatId;

void buildScene() {
    // 材质
    materials.push_back(Material({0.8f,0.8f,0.8f}));         // 0: 白墙
    materials.push_back(Material({0.8f,0.2f,0.2f}));         // 1: 红墙
    materials.push_back(Material({0.2f,0.8f,0.2f}));         // 2: 绿墙
    materials.push_back(Material({0.9f,0.9f,0.9f},{0,0,0},MIRROR)); // 3: 镜面球
    materials.push_back(Material({0.9f,0.95f,1.0f},{0,0,0},GLASS,1.5f)); // 4: 玻璃球
    materials.push_back(Material({1,1,1},{15,15,15}));        // 5: 光源
    materials.push_back(Material({0.8f,0.8f,0.2f}));         // 6: 黄色漫反射球

    // 场景尺寸：[-1,1]^3
    // 地板 y=-1
    planes.push_back({{0,1,0},  -1.f, 0});
    // 天花板 y=1
    planes.push_back({{0,-1,0}, -1.f, 0});
    // 后墙 z=-2
    planes.push_back({{0,0,1},  -2.f, 0});
    // 左墙 x=-1 (红)
    planes.push_back({{1,0,0},  -1.f, 1});
    // 右墙 x=1 (绿)
    planes.push_back({{-1,0,0}, -1.f, 2});
    // 前墙（相机后面，不需要）

    // 球体
    // 镜面球
    spheres.push_back({{-0.45f,-0.55f,-1.3f}, 0.45f, 3});
    // 玻璃球 (产生焦散)
    spheres.push_back({{ 0.45f,-0.55f,-1.6f}, 0.45f, 4});
    // 黄色漫射球
    spheres.push_back({{  0.0f,-0.75f,-0.85f}, 0.25f, 6});

    // 面光源（小球）
    lightCenter  = Vec3(0.f, 0.85f, -1.2f);
    lightRadius  = 0.18f;
    lightMatId   = 5;
    spheres.push_back({lightCenter, lightRadius, lightMatId});
}

bool sceneIntersect(const Ray& ray, HitInfo& h, bool skipLight=false) {
    h.t = INF;
    bool hit = false;
    HitInfo tmp;
    for (auto& s : spheres) {
        if (skipLight && s.matId == lightMatId) continue;
        if (s.intersect(ray, EPS, h.t, tmp)) { h = tmp; hit = true; }
    }
    for (auto& p : planes) {
        if (p.intersect(ray, EPS, h.t, tmp)) { h = tmp; hit = true; }
    }
    return hit;
}

// ─── 光子 ─────────────────────────────────────────────────────────────────────
struct Photon {
    Vec3  pos;
    Vec3  power;    // RGB 功率
    Vec3  dir;      // 入射方向
};

// ─── kd-tree ──────────────────────────────────────────────────────────────────
struct KdNode {
    Photon photon;
    int    left = -1, right = -1;
    int    axis = 0;   // 分割轴
};

class PhotonMap {
public:
    std::vector<KdNode> nodes;

    void build(std::vector<Photon>& photons) {
        if (photons.empty()) return;
        nodes.reserve(photons.size());
        buildRecursive(photons, 0, (int)photons.size());
    }

    // 最近邻查询：返回 r 半径内的光子
    void query(const Vec3& pos, float r, std::vector<const Photon*>& result) const {
        if (nodes.empty()) return;
        queryRecursive(0, pos, r*r, result);
    }

private:
    int buildRecursive(std::vector<Photon>& ph, int l, int r) {
        if (l >= r) return -1;
        // 选择分割轴（最大范围轴）
        Vec3 mn = ph[l].pos, mx = ph[l].pos;
        for (int i = l+1; i < r; i++) {
            mn.x = std::min(mn.x, ph[i].pos.x); mx.x = std::max(mx.x, ph[i].pos.x);
            mn.y = std::min(mn.y, ph[i].pos.y); mx.y = std::max(mx.y, ph[i].pos.y);
            mn.z = std::min(mn.z, ph[i].pos.z); mx.z = std::max(mx.z, ph[i].pos.z);
        }
        Vec3 span = mx - mn;
        int axis = 0;
        if (span.y > span.x) axis = 1;
        if (span.z > (axis==0?span.x:span.y)) axis = 2;

        int mid = (l + r) / 2;
        std::nth_element(ph.begin()+l, ph.begin()+mid, ph.begin()+r,
            [axis](const Photon& a, const Photon& b){
                return (&a.pos.x)[axis] < (&b.pos.x)[axis];
            });

        int idx = (int)nodes.size();
        nodes.push_back({});
        nodes[idx].photon = ph[mid];
        nodes[idx].axis   = axis;

        int leftIdx  = buildRecursive(ph, l, mid);
        // 重新引用（nodes 可能 reallocate）
        int rightIdx = buildRecursive(ph, mid+1, r);
        nodes[idx].left  = leftIdx;
        nodes[idx].right = rightIdx;
        return idx;
    }

    void queryRecursive(int idx, const Vec3& pos, float r2,
                        std::vector<const Photon*>& result) const {
        if (idx < 0) return;
        const KdNode& node = nodes[idx];
        float d2 = (node.photon.pos - pos).len2();
        if (d2 <= r2) result.push_back(&node.photon);

        int axis = node.axis;
        float diff = (&pos.x)[axis] - (&node.photon.pos.x)[axis];
        int first  = diff < 0 ? node.left  : node.right;
        int second = diff < 0 ? node.right : node.left;

        queryRecursive(first, pos, r2, result);
        if (diff*diff <= r2)
            queryRecursive(second, pos, r2, result);
    }
};

// ─── 光子追踪 ─────────────────────────────────────────────────────────────────
PhotonMap globalMap;   // 全局光子图（间接光照）
PhotonMap causticMap;  // 焦散光子图

void emitPhotons(int numPhotons) {
    // 光源功率
    Vec3 totalPower = materials[lightMatId].emission * (4.f * PI * lightRadius * lightRadius * PI);
    Vec3 photonPower = totalPower * (1.f / numPhotons);

    std::vector<Photon> globalPhotons, causticPhotons;
    globalPhotons.reserve(numPhotons);
    causticPhotons.reserve(numPhotons / 4);

    for (int i = 0; i < numPhotons; i++) {
        // 从球形光源表面采样发射方向
        Vec3 lightNorm = randomSphere();
        Vec3 origin    = lightCenter + lightNorm * (lightRadius + EPS);
        Vec3 dir       = randomHemisphere(lightNorm);
        Ray  ray(origin, dir);
        Vec3 power = photonPower;

        bool causticPath = false;
        bool prevSpecular = true; // 来自光源

        for (int depth = 0; depth < 8; depth++) {
            HitInfo h;
            if (!sceneIntersect(ray, h, true)) break;

            const Material& mat = materials[h.matId];

            // 自发光体不存储
            if (mat.emission.len2() > 0) break;

            if (mat.type == DIFFUSE) {
                // 存储到全局光子图（直接光来自漫射后）
                if (!prevSpecular) {
                    Photon ph;
                    ph.pos   = h.pos;
                    ph.power = power;
                    ph.dir   = ray.d;
                    globalPhotons.push_back(ph);
                }
                // 如果是焦散路径（经过镜面/折射后首次打到漫反射）
                if (causticPath) {
                    Photon ph;
                    ph.pos   = h.pos;
                    ph.power = power;
                    ph.dir   = ray.d;
                    causticPhotons.push_back(ph);
                    causticPath = false;
                }

                // 俄罗斯轮盘决定是否继续
                float prob = std::max({mat.color.x, mat.color.y, mat.color.z});
                prob = std::min(prob, 0.95f);
                if (randf() > prob) break;
                power = power * mat.color * (1.f / prob);
                ray = Ray(h.pos + h.normal*EPS, randomHemisphere(h.normal));
                prevSpecular = false;

            } else if (mat.type == MIRROR) {
                Vec3 refl = ray.d - h.normal * 2.f * ray.d.dot(h.normal);
                power = power * mat.color;
                ray = Ray(h.pos + h.normal*EPS, refl);
                causticPath = true;
                prevSpecular = true;

            } else { // GLASS
                float ni = 1.0f, nt = mat.ior;
                Vec3 n = h.normal;
                float cosi = -ray.d.dot(n);
                if (cosi < 0) { // 从内部出射
                    std::swap(ni, nt);
                    n = n * -1.f;
                    cosi = -cosi;
                }
                float eta = ni / nt;
                float k   = 1.f - eta*eta*(1.f - cosi*cosi);
                if (k < 0) {
                    // 全内反射
                    Vec3 refl = ray.d + n * 2.f*cosi;
                    ray = Ray(h.pos + n*EPS, refl);
                } else {
                    // 折射
                    Vec3 refr = ray.d*eta + n*(eta*cosi - std::sqrt(k));
                    // Schlick 近似
                    float r0 = (ni-nt)/(ni+nt); r0 *= r0;
                    float fr = r0 + (1-r0)*std::pow(1.f-cosi, 5.f);
                    if (randf() < fr) {
                        Vec3 refl = ray.d + n * 2.f*cosi;
                        ray = Ray(h.pos + n*EPS, refl);
                    } else {
                        ray = Ray(h.pos - n*EPS, refr.norm());
                    }
                }
                power = power * mat.color;
                causticPath = true;
                prevSpecular = true;
            }
        }
    }

    // 构建 kd-tree
    globalMap.build(globalPhotons);
    causticMap.build(causticPhotons);

    printf("  全局光子: %zu  焦散光子: %zu\n", globalPhotons.size(), causticPhotons.size());
}

// ─── 渲染：直接光照（next event estimation） ──────────────────────────────────
Vec3 directLight(const Vec3& pos, const Vec3& normal, const Vec3& color) {
    // 从面光源采样
    Vec3 lightNorm = randomSphere();
    Vec3 lightPoint = lightCenter + lightNorm * lightRadius;
    Vec3 toLight = lightPoint - pos;
    float dist2 = toLight.len2();
    float dist  = std::sqrt(dist2);
    Vec3 toL = toLight * (1.f / dist);

    float cosLight  = std::max(0.f, (-lightNorm).dot(toL));
    float cosSurface = std::max(0.f, normal.dot(toL));
    if (cosLight < EPS || cosSurface < EPS) return Vec3(0);

    // 阴影测试
    Ray shadowRay(pos + normal*EPS, toL);
    HitInfo sh;
    if (sceneIntersect(shadowRay, sh, false)) {
        if (sh.t < dist - EPS && sh.matId != lightMatId) return Vec3(0);
    }

    // 几何项
    float lightArea = 2.f * PI * lightRadius * lightRadius; // 半球投影
    Vec3  Le = materials[lightMatId].emission;
    float G  = cosSurface * cosLight / dist2;
    return Le * color * (G * lightArea / PI);
}

// ─── 渲染：从光子图估计辐射 ────────────────────────────────────────────────────
Vec3 estimateGlobal(const Vec3& pos, const Vec3& normal, const Vec3& color, float r) {
    std::vector<const Photon*> nearby;
    globalMap.query(pos, r, nearby);
    if (nearby.empty()) return Vec3(0);

    Vec3 result(0);
    for (const Photon* ph : nearby) {
        if (ph->dir.dot(normal) >= 0) continue; // 背面光子剔除
        result += ph->power * color;
    }
    return result * (1.f / (PI * r * r));
}

Vec3 estimateCaustic(const Vec3& pos, const Vec3& normal, const Vec3& color, float r) {
    std::vector<const Photon*> nearby;
    causticMap.query(pos, r, nearby);
    if (nearby.empty()) return Vec3(0);

    Vec3 result(0);
    for (const Photon* ph : nearby) {
        if (ph->dir.dot(normal) >= 0) continue;
        result += ph->power * color;
    }
    return result * (1.f / (PI * r * r));
}

// ─── 路径追踪（用于直接光照和镜面反射链） ─────────────────────────────────────
Vec3 trace(const Ray& ray, int depth=0) {
    if (depth > 6) return Vec3(0);

    HitInfo h;
    if (!sceneIntersect(ray, h, false)) return Vec3(0.01f, 0.02f, 0.05f); // 背景

    const Material& mat = materials[h.matId];

    // 自发光
    if (mat.emission.len2() > 0) {
        return (depth == 0) ? mat.emission * 0.5f : mat.emission;
    }

    Vec3 pos    = h.pos;
    Vec3 normal = h.normal;

    if (mat.type == DIFFUSE) {
        // 直接光照
        Vec3 direct  = directLight(pos, normal, mat.color);
        // 间接光照（全局光子图）
        Vec3 indirect = estimateGlobal(pos, normal, mat.color, 0.12f);
        // 焦散
        Vec3 caustic  = estimateCaustic(pos, normal, mat.color, 0.06f);
        return direct + indirect + caustic;

    } else if (mat.type == MIRROR) {
        Vec3 refl = ray.d - normal * 2.f * ray.d.dot(normal);
        return mat.color * trace(Ray(pos + normal*EPS, refl), depth+1);

    } else { // GLASS
        float ni = 1.0f, nt = mat.ior;
        Vec3 n = normal;
        float cosi = -ray.d.dot(n);
        if (cosi < 0) {
            std::swap(ni, nt);
            n = n * -1.f;
            cosi = -cosi;
        }
        float eta = ni / nt;
        float k   = 1.f - eta*eta*(1.f - cosi*cosi);
        // Schlick
        float r0 = (ni-nt)/(ni+nt); r0 *= r0;
        float fr  = r0 + (1-r0)*std::pow(1.f-cosi, 5.f);

        Vec3 refl = (ray.d + n*2.f*cosi).norm();
        Vec3 reflColor = mat.color * trace(Ray(pos + n*EPS, refl), depth+1);

        if (k < 0) return reflColor; // 全反射

        Vec3 refr = (ray.d*eta + n*(eta*cosi - std::sqrt(k))).norm();
        Vec3 refrColor = mat.color * trace(Ray(pos - n*EPS, refr), depth+1);

        return reflColor*fr + refrColor*(1.f-fr);
    }
}

// ─── 色调映射 ─────────────────────────────────────────────────────────────────
inline float aces(float x){
    const float a=2.51f, b=0.03f, c=2.43f, d=0.59f, e=0.14f;
    return std::clamp((x*(a*x+b))/(x*(c*x+d)+e), 0.f, 1.f);
}
inline uint8_t toSRGB(float v){
    float g = v <= 0.0031308f ? 12.92f*v : 1.055f*std::pow(v,1.f/2.4f)-0.055f;
    return (uint8_t)std::clamp(g*255.f+0.5f, 0.f, 255.f);
}

// ─── 主程序 ───────────────────────────────────────────────────────────────────
int main() {
    printf("=== Photon Mapping Renderer ===\n");

    buildScene();

    const int W = 512, H = 512;
    const int SAMPLES  = 32;      // 每像素采样数
    const int NUM_PHOTONS = 200000;

    printf("[Phase 1] 发射光子: %d ...\n", NUM_PHOTONS);
    emitPhotons(NUM_PHOTONS);
    printf("[Phase 1] 光子图构建完成\n");

    printf("[Phase 2] 渲染 %dx%d, %d spp ...\n", W, H, SAMPLES);

    std::vector<uint8_t> img(W * H * 3);

    // 相机参数
    Vec3  eye(0, 0, 2.0f);
    float fov = 45.f * PI / 180.f;
    float aspect = (float)W / H;
    float tanHalfFov = std::tan(fov * 0.5f);

    // 并行渲染（简单版，不用 OpenMP，保证可移植）
    for (int y = 0; y < H; y++) {
        if (y % 64 == 0) printf("  行 %d / %d\n", y, H);
        for (int x = 0; x < W; x++) {
            Vec3 accum(0);
            for (int s = 0; s < SAMPLES; s++) {
                float dx = randf() - 0.5f;
                float dy = randf() - 0.5f;
                float u = ((x + dx + 0.5f) / W  * 2.f - 1.f) * aspect * tanHalfFov;
                float v = ((H - y + dy - 0.5f) / H * 2.f - 1.f) * tanHalfFov;
                Ray ray(eye, Vec3(u, v, -1.f));
                accum += trace(ray);
            }
            accum = accum * (1.f / SAMPLES);

            uint8_t r = toSRGB(aces(accum.x));
            uint8_t g = toSRGB(aces(accum.y));
            uint8_t b = toSRGB(aces(accum.z));

            int idx = (y * W + x) * 3;
            img[idx+0] = r; img[idx+1] = g; img[idx+2] = b;
        }
    }

    const char* outFile = "photon_map_output.png";
    stbi_write_png(outFile, W, H, 3, img.data(), W*3);
    printf("✅ 输出: %s\n", outFile);

    return 0;
}
