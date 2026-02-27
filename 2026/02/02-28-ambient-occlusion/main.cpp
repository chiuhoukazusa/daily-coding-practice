/**
 * Ambient Occlusion (AO) - 环境光遮蔽渲染器
 * 
 * 实现原理：
 * - 在场景中的每个着色点，朝半球方向发射多条随机光线
 * - 统计有多少光线被场景几何体遮挡
 * - 遮挡越多，该点越暗（环境光被遮蔽）
 * 
 * 场景：Cornell Box 风格，包含球体和平面
 * 输出：ao_output.png（800x600）
 * 
 * 技术要点：
 * - 蒙特卡洛积分估算遮蔽率
 * - 半球采样（余弦加权）
 * - 切线空间变换
 * - Gram-Schmidt 正交化构造TBN
 * 
 * 编译：g++ main.cpp -o ao_renderer -O2 -std=c++17
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <cassert>

// ============================================================
// 基础数学库
// ============================================================

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double t)      const { return {x*t, y*t, z*t}; }
    Vec3 operator/(double t)      const { return {x/t, y/t, z/t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }

    double dot(const Vec3& b)   const { return x*b.x + y*b.y + z*b.z; }
    Vec3   cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    double length()  const { return std::sqrt(x*x + y*y + z*z); }
    Vec3   normalized() const {
        double l = length();
        if (l < 1e-12) return {0,0,1};
        return *this / l;
    }
    Vec3 neg() const { return {-x, -y, -z}; }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
// 光线
// ============================================================

struct Ray {
    Vec3 origin;
    Vec3 direction; // normalized
};

// ============================================================
// 命中记录
// ============================================================

struct HitRecord {
    double t;
    Vec3 point;
    Vec3 normal; // 始终朝向光线来源方向
    bool hit;
};

// ============================================================
// 场景对象
// ============================================================

// 球体
struct Sphere {
    Vec3 center;
    double radius;

    HitRecord intersect(const Ray& ray, double tmin, double tmax) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double disc = b*b - 4*a*c;

        if (disc < 0) return {0, {}, {}, false};

        double sqrtDisc = std::sqrt(disc);
        double t = (-b - sqrtDisc) / (2*a);
        if (t < tmin || t > tmax) {
            t = (-b + sqrtDisc) / (2*a);
            if (t < tmin || t > tmax) return {0, {}, {}, false};
        }

        Vec3 p = ray.origin + ray.direction * t;
        Vec3 n = (p - center).normalized();

        // 法线朝向光线来源
        if (n.dot(ray.direction) > 0) n = n.neg();

        return {t, p, n, true};
    }
};

// 无限平面
struct Plane {
    Vec3 point;   // 平面上一点
    Vec3 normal;  // 平面法线（已归一化）

    HitRecord intersect(const Ray& ray, double tmin, double tmax) const {
        double denom = normal.dot(ray.direction);
        if (std::abs(denom) < 1e-10) return {0, {}, {}, false};

        double t = (point - ray.origin).dot(normal) / denom;
        if (t < tmin || t > tmax) return {0, {}, {}, false};

        Vec3 p = ray.origin + ray.direction * t;
        Vec3 n = normal;
        if (n.dot(ray.direction) > 0) n = n.neg();

        return {t, p, n, true};
    }
};

// ============================================================
// 场景
// ============================================================

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane>  planes;

    // 返回最近的命中
    HitRecord intersect(const Ray& ray, double tmin, double tmax) const {
        HitRecord closest;
        closest.hit = false;
        closest.t   = tmax;

        for (const auto& s : spheres) {
            HitRecord h = s.intersect(ray, tmin, closest.t);
            if (h.hit) closest = h;
        }
        for (const auto& p : planes) {
            HitRecord h = p.intersect(ray, tmin, closest.t);
            if (h.hit) closest = h;
        }
        return closest;
    }

    // 只检查是否命中（遮蔽测试用）
    bool occluded(const Ray& ray, double tmin, double tmax) const {
        for (const auto& s : spheres) {
            if (s.intersect(ray, tmin, tmax).hit) return true;
        }
        for (const auto& p : planes) {
            if (p.intersect(ray, tmin, tmax).hit) return true;
        }
        return false;
    }
};

// ============================================================
// 采样工具
// ============================================================

// 全局随机数生成器（线程安全性不做要求）
std::mt19937 rng(42);
std::uniform_real_distribution<double> dist01(0.0, 1.0);

double rand01() { return dist01(rng); }

// 半球余弦加权采样
// 返回局部坐标系下的方向（Z轴为半球极轴，即法线方向）
Vec3 cosineSampleHemisphere() {
    double r1 = rand01();
    double r2 = rand01();

    double phi = 2.0 * M_PI * r1;
    double sinTheta = std::sqrt(r2);
    double cosTheta = std::sqrt(1.0 - r2);

    return {
        std::cos(phi) * sinTheta,
        std::sin(phi) * sinTheta,
        cosTheta
    };
}

// 构造切线空间（TBN），使 Z 轴对齐法线
// 使用 Gram-Schmidt 正交化
void buildTBN(const Vec3& N, Vec3& T, Vec3& B) {
    // 选一个不与 N 平行的辅助向量
    Vec3 up = (std::abs(N.x) < 0.9) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);

    T = up.cross(N).normalized();
    B = N.cross(T);
}

// 将局部坐标的方向变换到世界空间
Vec3 localToWorld(const Vec3& local, const Vec3& N, const Vec3& T, const Vec3& B) {
    return T * local.x + B * local.y + N * local.z;
}

// ============================================================
// 环境光遮蔽计算
// ============================================================

/**
 * 计算点 p 处（法线为 N）的 AO 值
 * @param numSamples  采样数（越多越准确但越慢）
 * @param maxDist     最大遮蔽距离（超过此距离的物体不计入遮蔽）
 */
double computeAO(const Vec3& p, const Vec3& N, const Scene& scene,
                  int numSamples, double maxDist) {
    Vec3 T, B;
    buildTBN(N, T, B);

    int occluded = 0;
    for (int i = 0; i < numSamples; ++i) {
        // 采样半球方向（余弦加权）
        Vec3 localDir = cosineSampleHemisphere();
        Vec3 worldDir = localToWorld(localDir, N, T, B);

        Ray aoRay;
        aoRay.origin    = p + N * 1e-4; // 偏移避免自交
        aoRay.direction = worldDir.normalized();

        if (scene.occluded(aoRay, 1e-4, maxDist)) {
            ++occluded;
        }
    }

    // AO 值：0=完全遮蔽（暗），1=完全不遮蔽（亮）
    return 1.0 - (double)occluded / numSamples;
}

// ============================================================
// 摄像机
// ============================================================

struct Camera {
    Vec3 origin;
    Vec3 lowerLeft;
    Vec3 horizontal;
    Vec3 vertical;

    Camera(Vec3 lookFrom, Vec3 lookAt, Vec3 up, double fovDeg, double aspect) {
        double theta = fovDeg * M_PI / 180.0;
        double halfH = std::tan(theta / 2.0);
        double halfW = aspect * halfH;

        Vec3 w = (lookFrom - lookAt).normalized();
        Vec3 u = up.cross(w).normalized();
        Vec3 v = w.cross(u);

        origin     = lookFrom;
        lowerLeft  = lookFrom - u*halfW - v*halfH - w;
        horizontal = u * (2 * halfW);
        vertical   = v * (2 * halfH);
    }

    Ray getRay(double s, double t) const {
        Vec3 dir = lowerLeft + horizontal*s + vertical*t - origin;
        return {origin, dir.normalized()};
    }
};

// ============================================================
// 色调映射 & 颜色工具
// ============================================================

// 将 [0,1] 的 AO 值转为灰色（带 gamma 校正）
Vec3 aoToColor(double ao) {
    // gamma 2.2 校正
    double c = std::pow(ao, 1.0/2.2);
    c = std::max(0.0, std::min(1.0, c));
    return {c, c, c};
}

// ============================================================
// 主渲染函数
// ============================================================

int main() {
    // 分辨率
    const int W = 800;
    const int H = 600;

    // AO 参数
    const int   AO_SAMPLES = 64;  // 每像素采样数
    const double AO_DIST   = 2.0; // 遮蔽最大距离

    // ---- 场景构建 ----
    Scene scene;

    // 地板平面（Y=-1）
    scene.planes.push_back({ {0,-1,0}, {0,1,0} });

    // 左墙（X=-3）
    scene.planes.push_back({ {-3,0,0}, {1,0,0} });

    // 右墙（X=3）
    scene.planes.push_back({ {3,0,0}, {-1,0,0} });

    // 后墙（Z=-4）
    scene.planes.push_back({ {0,0,-4}, {0,0,1} });

    // 顶面（Y=3）
    scene.planes.push_back({ {0,3,0}, {0,-1,0} });

    // 中央大球（明显 AO 阴影）
    scene.spheres.push_back({ {0, 0, -2}, 1.0 });

    // 左小球（紧贴地面）
    scene.spheres.push_back({ {-1.5, -0.5, -2.5}, 0.5 });

    // 右小球（紧贴地面）
    scene.spheres.push_back({ {1.5, -0.5, -2.5}, 0.5 });

    // 角落小球（展示角落遮蔽）
    scene.spheres.push_back({ {-2.2, -0.7, -3.5}, 0.3 });
    scene.spheres.push_back({ {2.2, -0.7, -3.5}, 0.3 });

    // ---- 摄像机 ----
    Camera cam(
        {0, 1, 2},   // lookFrom
        {0, 0, -2},  // lookAt
        {0, 1, 0},   // up
        60.0,        // fov
        (double)W / H
    );

    // ---- 渲染 ----
    std::vector<uint8_t> pixels(W * H * 3);

    std::cout << "Rendering " << W << "x" << H << " with " << AO_SAMPLES << " AO samples..." << std::endl;

    for (int j = 0; j < H; ++j) {
        if (j % 60 == 0) {
            std::cout << "Progress: " << (j * 100 / H) << "%" << std::endl;
        }
        for (int i = 0; i < W; ++i) {
            // 子像素抗锯齿（2x2 超采样）
            Vec3 color = {0,0,0};
            const int AA = 2;
            for (int aj = 0; aj < AA; ++aj) {
                for (int ai = 0; ai < AA; ++ai) {
                    double u = (i + (ai + 0.5) / AA) / W;
                    double v = ((H - 1 - j) + (aj + 0.5) / AA) / H;

                    Ray ray = cam.getRay(u, v);
                    HitRecord hit = scene.intersect(ray, 1e-4, 1e6);

                    double ao = 1.0;
                    if (hit.hit) {
                        ao = computeAO(hit.point, hit.normal, scene, AO_SAMPLES, AO_DIST);
                    }

                    // 背景（天空）渐变
                    double bgAO = 0.5 * (1.0 + ray.direction.y); // 天空渐变
                    if (!hit.hit) {
                        ao = 0.5 + 0.5 * bgAO; // 淡蓝灰色背景
                    }

                    Vec3 c = aoToColor(ao);
                    color = color + c;
                }
            }
            color = color / (AA * AA);

            int idx = (j * W + i) * 3;
            pixels[idx + 0] = (uint8_t)(std::min(color.x, 1.0) * 255 + 0.5);
            pixels[idx + 1] = (uint8_t)(std::min(color.y, 1.0) * 255 + 0.5);
            pixels[idx + 2] = (uint8_t)(std::min(color.z, 1.0) * 255 + 0.5);
        }
    }

    // ---- 保存输出 ----
    int result = stbi_write_png("ao_output.png", W, H, 3, pixels.data(), W * 3);
    if (result == 0) {
        std::cerr << "❌ Failed to write ao_output.png" << std::endl;
        return 1;
    }

    std::cout << "✅ ao_output.png saved (" << W << "x" << H << ")" << std::endl;

    // ---- 量化验证 ----
    // 验证：地板球接触点应该更暗（AO 值低），球顶部应该更亮
    // 我们在代码内部通过打印几个关键点来验证
    std::cout << "\n=== 量化验证 ===" << std::endl;

    // 测试点1：中央球顶部（应该较亮，AO≈0.7~1.0）
    Vec3 topPoint  = {0, 1.01, -2};  // 球顶部稍上方的法线方向点
    Vec3 topNormal = {0, 1, 0};
    double aoTop = computeAO(topPoint, topNormal, scene, 128, AO_DIST);
    std::cout << "球顶部 AO = " << aoTop << " (预期: > 0.5)" << std::endl;

    // 测试点2：球底部靠近地面（应该更暗，被地面遮蔽，AO≈0.3~0.6）
    Vec3 bottomPoint  = {0, -0.9, -2}; // 球底部附近
    Vec3 bottomNormal = {0, -1, 0};
    double aoBottom = computeAO(bottomPoint, bottomNormal, scene, 128, AO_DIST);
    std::cout << "球底部 AO = " << aoBottom << " (预期: < 0.7，被遮蔽)" << std::endl;

    // 测试点3：地板中央（开阔，应该较亮）
    Vec3 floorCenter  = {0, -1.01, -2};
    Vec3 floorNormal  = {0, 1, 0};
    double aoFloor = computeAO(floorCenter, floorNormal, scene, 128, AO_DIST);
    std::cout << "地板中央 AO = " << aoFloor << " (预期: 0.4~0.8，球投影遮蔽)" << std::endl;

    // 测试点4：角落（应该较暗，被两面墙遮蔽）
    Vec3 cornerPoint  = {-2.9, -0.9, -3.9};
    Vec3 cornerNormal = {0, 1, 0};  // 地板法线
    double aoCorner = computeAO(cornerPoint, cornerNormal, scene, 128, AO_DIST);
    std::cout << "角落 AO = " << aoCorner << " (预期: < 0.5，角落遮蔽)" << std::endl;

    // 验证 AO 差异正确（球顶 > 球底，角落较暗）
    bool valid = true;
    if (aoTop <= 0.5) {
        std::cerr << "❌ 球顶部过暗！AO = " << aoTop << std::endl;
        valid = false;
    }
    if (aoTop <= aoBottom) {
        std::cerr << "❌ 顶部 AO 应该高于底部！top=" << aoTop << " bottom=" << aoBottom << std::endl;
        valid = false;
    }
    if (aoCorner >= 0.8) {
        std::cerr << "❌ 角落应该有明显遮蔽！AO = " << aoCorner << std::endl;
        valid = false;
    }

    if (valid) {
        std::cout << "\n✅ 量化验证通过！AO 梯度正确（角落暗，开阔区域亮）" << std::endl;
    } else {
        std::cerr << "\n❌ 量化验证失败！" << std::endl;
        return 1;
    }

    return 0;
}
