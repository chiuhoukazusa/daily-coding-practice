/*
 * Depth of Field Renderer
 * 
 * 使用薄透镜模型实现物理正确的景深效果（散景/Bokeh）
 * 
 * 技术要点:
 * - 薄透镜模型 (Thin Lens Model)
 * - 光圈模拟 (Aperture Simulation)
 * - 焦平面控制 (Focus Distance)
 * - 圆形散景 (Circular Bokeh)
 * - 路径追踪积分 (Path Tracing Integration)
 * - Monte Carlo 采样
 * 
 * 场景: Cornell Box 变体，前景/中景/背景球体，展示不同焦距效果
 * 
 * 输出: dof_output.png (800x600)
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <random>
#include <string>
#include <limits>
#include <memory>

// ============================================================
//  数学工具
// ============================================================

struct Vec3 {
    double x, y, z;
    Vec3(double a = 0, double b = 0, double c = 0) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t)      const { return {x*t,   y*t,   z*t};   }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t)      const { return {x/t,   y/t,   z/t};   }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    double dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3   cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 normalize() const { double l = length(); return l > 1e-12 ? *this / l : Vec3(0,1,0); }
    Vec3 clamp(double lo, double hi) const {
        return {std::max(lo, std::min(hi, x)),
                std::max(lo, std::min(hi, y)),
                std::max(lo, std::min(hi, z))};
    }
};

inline Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
//  随机数
// ============================================================

thread_local std::mt19937 rng(42);

double rand01() {
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(rng);
}

// 单位圆内均匀采样（用于光圈）
Vec3 randomInUnitDisk() {
    while (true) {
        double x = rand01() * 2 - 1;
        double y = rand01() * 2 - 1;
        if (x*x + y*y < 1.0) return {x, y, 0};
    }
}

// 半球余弦加权采样
Vec3 cosineSampleHemisphere() {
    double r1 = rand01();
    double r2 = rand01();
    double phi = 2.0 * M_PI * r1;
    double sqr2 = std::sqrt(r2);
    return {
        std::cos(phi) * sqr2,
        std::sin(phi) * sqr2,
        std::sqrt(1.0 - r2)
    };
}

Vec3 toWorld(const Vec3& local, const Vec3& normal) {
    Vec3 up = (std::abs(normal.x) > 0.9) ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 tangent   = normal.cross(up).normalize();
    Vec3 bitangent = normal.cross(tangent);
    return tangent   * local.x
         + bitangent * local.y
         + normal    * local.z;
}

// ============================================================
//  光线
// ============================================================

struct Ray {
    Vec3 origin, dir;
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalize()) {}
    Vec3 at(double t) const { return origin + dir * t; }
};

// ============================================================
//  材质
// ============================================================

enum class MatType { DIFFUSE, METAL, GLASS, EMISSIVE };

struct Material {
    MatType type   = MatType::DIFFUSE;
    Vec3    color  = {0.8, 0.8, 0.8};
    Vec3    emit   = {0, 0, 0};
    double  rough  = 0.0;   // METAL: roughness
    double  ior    = 1.5;   // GLASS: index of refraction
};

// ============================================================
//  场景物体
// ============================================================

struct HitInfo {
    double   t     = std::numeric_limits<double>::infinity();
    Vec3     pos;
    Vec3     normal;
    Material mat;
    bool     front = true;
};

struct Sphere {
    Vec3     center;
    double   radius;
    Material mat;

    bool hit(const Ray& ray, double tMin, double tMax, HitInfo& h) const {
        Vec3 oc = ray.origin - center;
        double a = ray.dir.dot(ray.dir);
        double b = oc.dot(ray.dir);
        double c = oc.dot(oc) - radius * radius;
        double disc = b*b - a*c;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double t  = (-b - sq) / a;
        if (t < tMin || t > tMax) {
            t = (-b + sq) / a;
            if (t < tMin || t > tMax) return false;
        }
        h.t   = t;
        h.pos = ray.at(t);
        Vec3 outN = (h.pos - center) / radius;
        h.front  = ray.dir.dot(outN) < 0;
        h.normal = h.front ? outN : outN * -1.0;
        h.mat    = mat;
        return true;
    }
};

struct Plane {
    Vec3     point, normal;
    Material mat;

    bool hit(const Ray& ray, double tMin, double tMax, HitInfo& h) const {
        double denom = ray.dir.dot(normal);
        if (std::abs(denom) < 1e-8) return false;
        double t = (point - ray.origin).dot(normal) / denom;
        if (t < tMin || t > tMax) return false;
        h.t      = t;
        h.pos    = ray.at(t);
        h.front  = denom < 0;
        h.normal = h.front ? normal : normal * -1.0;
        h.mat    = mat;
        return true;
    }
};

// ============================================================
//  场景
// ============================================================

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane>  planes;

    bool hit(const Ray& ray, double tMin, double tMax, HitInfo& h) const {
        HitInfo tmp;
        bool any = false;
        for (auto& s : spheres) {
            if (s.hit(ray, tMin, tMax, tmp) && tmp.t < h.t) {
                h = tmp; any = true;
            }
        }
        for (auto& p : planes) {
            if (p.hit(ray, tMin, tMax, tmp) && tmp.t < h.t) {
                h = tmp; any = true;
            }
        }
        return any;
    }
};

// ============================================================
//  Fresnel (Schlick 近似)
// ============================================================

double fresnelSchlick(double cosTheta, double r0) {
    double c = 1.0 - std::max(0.0, cosTheta);
    return r0 + (1.0 - r0) * c * c * c * c * c;
}

// ============================================================
//  路径追踪
// ============================================================

Vec3 trace(const Ray& ray, const Scene& scene, int depth) {
    if (depth <= 0) return {0, 0, 0};

    HitInfo h;
    if (!scene.hit(ray, 1e-4, 1e10, h)) {
        // 天空梯度
        double t = 0.5 * (ray.dir.y + 1.0);
        return Vec3(1,1,1) * (1-t) + Vec3(0.5, 0.7, 1.0) * t;
    }

    auto& mat = h.mat;

    // 自发光
    if (mat.type == MatType::EMISSIVE) {
        return mat.emit;
    }

    if (mat.type == MatType::DIFFUSE) {
        // Lambert 漫反射
        Vec3 scattered = toWorld(cosineSampleHemisphere(), h.normal);
        Vec3 incoming  = trace(Ray(h.pos, scattered), scene, depth - 1);
        return mat.color * incoming;
    }

    if (mat.type == MatType::METAL) {
        // 镜面反射 + 粗糙扰动
        Vec3 refl = ray.dir - h.normal * 2.0 * ray.dir.dot(h.normal);
        if (mat.rough > 0) {
            // 半球扰动
            Vec3 fuzz = toWorld(cosineSampleHemisphere(), refl.normalize()) * mat.rough;
            refl = refl + fuzz;
        }
        if (refl.dot(h.normal) <= 0) return {0,0,0};
        Vec3 incoming = trace(Ray(h.pos, refl.normalize()), scene, depth - 1);
        return mat.color * incoming;
    }

    if (mat.type == MatType::GLASS) {
        double etaRatio = h.front ? (1.0 / mat.ior) : mat.ior;
        double cosI  = -ray.dir.dot(h.normal);
        double sinT2 = etaRatio * etaRatio * (1.0 - cosI * cosI);

        double r0 = (1 - mat.ior) / (1 + mat.ior); r0 *= r0;
        double fresnelP = fresnelSchlick(cosI, r0);

        if (sinT2 > 1.0 || rand01() < fresnelP) {
            // 全反射 or 菲涅尔反射
            Vec3 refl = ray.dir + h.normal * 2.0 * cosI;
            return mat.color * trace(Ray(h.pos, refl.normalize()), scene, depth - 1);
        } else {
            // 折射
            double cosT = std::sqrt(1.0 - sinT2);
            Vec3 refr = ray.dir * etaRatio + h.normal * (etaRatio * cosI - cosT);
            return mat.color * trace(Ray(h.pos, refr.normalize()), scene, depth - 1);
        }
    }

    return {0, 0, 0};
}

// ============================================================
//  薄透镜相机  (Thin Lens Camera)
// ============================================================

struct Camera {
    Vec3   pos;
    Vec3   lower_left, horizontal, vertical;
    Vec3   u, v, w;            // 相机坐标系
    double lensRadius;         // 光圈半径
    double focusDist;          // 焦距（到焦平面距离）

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 vup,
           double vfov,         // 垂直视角，度
           double aspect,       // 宽高比
           double aperture,     // 光圈直径
           double focusDistance)
    {
        lensRadius = aperture / 2.0;
        focusDist  = focusDistance;

        double theta     = vfov * M_PI / 180.0;
        double halfHeight = std::tan(theta / 2.0);
        double halfWidth  = aspect * halfHeight;

        w = (lookFrom - lookAt).normalize();
        u = vup.cross(w).normalize();
        v = w.cross(u);

        pos = lookFrom;

        lower_left = pos
                   - u * halfWidth  * focusDist
                   - v * halfHeight * focusDist
                   - w * focusDist;

        horizontal = u * 2.0 * halfWidth  * focusDist;
        vertical   = v * 2.0 * halfHeight * focusDist;
    }

    // 生成穿过薄透镜的光线
    // s,t: 像素坐标 [0,1]
    Ray getRay(double s, double t) const {
        Vec3 rd     = randomInUnitDisk() * lensRadius;
        Vec3 offset = u * rd.x + v * rd.y;

        Vec3 target = lower_left
                    + horizontal * s
                    + vertical   * t;

        Vec3 origin = pos + offset;
        Vec3 dir    = (target - origin).normalize();
        return Ray(origin, dir);
    }
};

// ============================================================
//  构建场景
// ============================================================

Scene buildScene() {
    Scene scene;

    // ---- 地面 ----
    Plane ground;
    ground.point  = {0, -0.5, 0};
    ground.normal = {0, 1, 0};
    ground.mat.type  = MatType::DIFFUSE;
    ground.mat.color = {0.55, 0.50, 0.45};
    scene.planes.push_back(ground);

    // ---- 背景墙 ----
    Plane back;
    back.point  = {0, 0, -8};
    back.normal = {0, 0, 1};
    back.mat.type  = MatType::DIFFUSE;
    back.mat.color = {0.7, 0.7, 0.75};
    scene.planes.push_back(back);

    // ---- 前景球 (z=1.5) — 焦点附近 ----
    {
        Sphere s;
        s.center = {-1.2, 0.0, 1.5};
        s.radius = 0.5;
        s.mat.type  = MatType::DIFFUSE;
        s.mat.color = {0.9, 0.2, 0.2};   // 红色
        scene.spheres.push_back(s);
    }
    {
        Sphere s;
        s.center = {0.0, 0.0, 1.5};
        s.radius = 0.5;
        s.mat.type  = MatType::GLASS;
        s.mat.color = {1.0, 1.0, 1.0};
        s.mat.ior   = 1.5;
        scene.spheres.push_back(s);
    }
    {
        Sphere s;
        s.center = {1.2, 0.0, 1.5};
        s.radius = 0.5;
        s.mat.type  = MatType::METAL;
        s.mat.color = {0.8, 0.7, 0.2};   // 金色
        s.mat.rough = 0.05;
        scene.spheres.push_back(s);
    }

    // ---- 中景球 (z=-0.5) ----
    {
        Sphere s;
        s.center = {-0.7, -0.1, -0.5};
        s.radius = 0.4;
        s.mat.type  = MatType::DIFFUSE;
        s.mat.color = {0.2, 0.6, 0.9};   // 蓝色
        scene.spheres.push_back(s);
    }
    {
        Sphere s;
        s.center = {0.7, -0.1, -0.5};
        s.radius = 0.4;
        s.mat.type  = MatType::METAL;
        s.mat.color = {0.7, 0.85, 0.7};  // 银绿
        s.mat.rough = 0.3;
        scene.spheres.push_back(s);
    }

    // ---- 背景球 (z=-4) ----
    {
        Sphere s;
        s.center = {-1.5, 0.1, -4.0};
        s.radius = 0.6;
        s.mat.type  = MatType::DIFFUSE;
        s.mat.color = {0.8, 0.5, 0.1};   // 橙色
        scene.spheres.push_back(s);
    }
    {
        Sphere s;
        s.center = {1.5, 0.1, -4.0};
        s.radius = 0.6;
        s.mat.type  = MatType::DIFFUSE;
        s.mat.color = {0.3, 0.8, 0.4};   // 绿色
        scene.spheres.push_back(s);
    }

    // ---- 光源球（天空光已够，加一盏暖光灯）----
    {
        Sphere s;
        s.center = {0, 4.0, -1};
        s.radius = 1.5;
        s.mat.type = MatType::EMISSIVE;
        s.mat.emit = {2.5, 2.2, 1.8};
        scene.spheres.push_back(s);
    }

    return scene;
}

// ============================================================
//  色调映射 + Gamma 校正
// ============================================================

uint8_t toU8(double v) {
    // ACES Filmic Tonemapping
    double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    v = (v * (a * v + b)) / (v * (c * v + d) + e);
    v = std::max(0.0, std::min(1.0, v));
    // Gamma 2.2
    v = std::pow(v, 1.0 / 2.2);
    return static_cast<uint8_t>(v * 255.999);
}

// ============================================================
//  主函数
// ============================================================

int main() {
    // 图像参数
    const int W       = 800;
    const int H       = 600;
    const int SAMPLES = 128;   // 每像素采样数
    const int DEPTH   = 6;     // 最大反射深度

    // 薄透镜相机参数
    // 焦点对准前景球群 (z=1.5)，使用大光圈产生明显散景
    Vec3 lookFrom = {0, 0.6, 4.5};
    Vec3 lookAt   = {0, 0.0, 1.5};  // 焦平面在前景球
    Vec3 vup      = {0, 1, 0};
    double aperture     = 0.25;     // 光圈：较大，产生明显散景
    double focusDist    = (lookFrom - lookAt).length();
    double vfov         = 40.0;     // 较窄视角，突出景深效果
    double aspect       = (double)W / H;

    Camera cam(lookFrom, lookAt, vup, vfov, aspect, aperture, focusDist);
    Scene  scene = buildScene();

    // 渲染缓冲
    std::vector<uint8_t> img(W * H * 3);

    // 进度报告
    for (int row = 0; row < H; ++row) {
        if (row % 60 == 0) {
            fprintf(stderr, "Rendering row %d / %d (%.0f%%)\n",
                    row, H, 100.0 * row / H);
        }

        for (int col = 0; col < W; ++col) {
            Vec3 color = {0, 0, 0};

            for (int s = 0; s < SAMPLES; ++s) {
                // 像素内抖动采样（反走样）
                double su = (col + rand01()) / W;
                double sv = (H - 1 - row + rand01()) / H;  // 翻转 Y：天空朝上
                Ray ray = cam.getRay(su, sv);
                color  += trace(ray, scene, DEPTH);
            }

            color = color / (double)SAMPLES;

            int idx = (row * W + col) * 3;
            img[idx+0] = toU8(color.x);
            img[idx+1] = toU8(color.y);
            img[idx+2] = toU8(color.z);
        }
    }

    stbi_write_png("dof_output.png", W, H, 3, img.data(), W * 3);
    fprintf(stderr, "Done! Written dof_output.png\n");

    return 0;
}
