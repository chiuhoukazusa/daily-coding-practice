/**
 * Environment Mapping with Cube Map Skybox
 * 日期: 2026-03-03
 * 技术: 立方体贴图, 天空盒渲染, 环境反射, 程序化天空生成
 *
 * 功能:
 * 1. 程序化生成 Cube Map (6 个面) - 渐变天空 + 星空
 * 2. 天空盒渲染 (通过光线方向采样 Cube Map)
 * 3. 环境反射 (金属球, 玻璃球)
 * 4. 最终合成渲染图
 *
 * 输出:
 * - skybox_output.png  : 天空盒场景 + 反射球
 * - cubemap_faces.png  : 展开的 Cube Map 6 个面预览
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <array>
#include <string>
#include <cstdio>

// ============================================================
// 基础数据结构
// ============================================================

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const {
        double len = length();
        if (len < 1e-10) return {0,0,0};
        return *this / len;
    }
};

inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return a * (1.0 - t) + b * t;
}

// ============================================================
// Cube Map 数据结构 (6面, 每面 256x256)
// ============================================================

static const int FACE_SIZE = 256;

enum CubeFace { POS_X=0, NEG_X=1, POS_Y=2, NEG_Y=3, POS_Z=4, NEG_Z=5 };

struct CubeMap {
    // 6 faces, each FACE_SIZE x FACE_SIZE, RGB
    std::vector<Vec3> faces[6];

    CubeMap() {
        for (int i = 0; i < 6; i++)
            faces[i].resize(FACE_SIZE * FACE_SIZE);
    }

    Vec3& at(int face, int x, int y) {
        return faces[face][y * FACE_SIZE + x];
    }

    const Vec3& at(int face, int x, int y) const {
        return faces[face][y * FACE_SIZE + x];
    }

    // 根据方向向量采样 Cube Map
    Vec3 sample(const Vec3& dir) const {
        Vec3 d = dir.normalize();
        double ax = std::abs(d.x), ay = std::abs(d.y), az = std::abs(d.z);

        int face;
        double u, v;

        if (ax >= ay && ax >= az) {
            if (d.x > 0) {
                face = POS_X;
                u = -d.z / d.x;
                v = -d.y / d.x;
            } else {
                face = NEG_X;
                u = d.z / (-d.x);
                v = -d.y / (-d.x);
            }
        } else if (ay >= ax && ay >= az) {
            if (d.y > 0) {
                face = POS_Y;
                u = d.x / d.y;
                v = d.z / d.y;
            } else {
                face = NEG_Y;
                u = d.x / (-d.y);
                v = -d.z / (-d.y);
            }
        } else {
            if (d.z > 0) {
                face = POS_Z;
                u = d.x / d.z;
                v = -d.y / d.z;
            } else {
                face = NEG_Z;
                u = -d.x / (-d.z);
                v = -d.y / (-d.z);
            }
        }

        // [-1,1] -> [0, FACE_SIZE-1]
        int px = static_cast<int>((u + 1.0) * 0.5 * (FACE_SIZE - 1) + 0.5);
        int py = static_cast<int>((v + 1.0) * 0.5 * (FACE_SIZE - 1) + 0.5);
        px = std::max(0, std::min(FACE_SIZE - 1, px));
        py = std::max(0, std::min(FACE_SIZE - 1, py));

        return at(face, px, py);
    }
};

// ============================================================
// 程序化 Cube Map 生成 - 美丽的渐变天空 + 太阳 + 星空
// ============================================================

// 随机星空哈希函数
inline double hash(double n) {
    n = std::sin(n) * 43758.5453123;
    return n - std::floor(n);
}

inline double hash2(double x, double y) {
    return hash(x * 127.1 + y * 311.7);
}

// 生成一个星星亮度
double starField(double u, double v) {
    double cx = std::floor(u * 40.0);
    double cy = std::floor(v * 40.0);
    double fx = u * 40.0 - cx;
    double fy = v * 40.0 - cy;

    double h = hash2(cx, cy);
    if (h > 0.97) {
        double dist = std::sqrt((fx-0.5)*(fx-0.5) + (fy-0.5)*(fy-0.5));
        double brightness = std::max(0.0, 1.0 - dist * 12.0);
        return brightness * (0.7 + 0.3 * hash2(cx+10.0, cy+20.0));
    }
    return 0.0;
}

// 太阳方向
const Vec3 SUN_DIR = Vec3(0.3, 0.8, 0.5).normalize();

// 根据方向计算天空颜色（程序化）
Vec3 skyColor(const Vec3& dir) {
    Vec3 d = dir.normalize();

    // 日落/黎明时天空渐变
    double t = std::max(0.0, d.y); // 垂直高度因子 [0,1]

    // 天顶颜色 (深蓝) -> 地平线颜色 (橙粉)
    Vec3 zenithColor(0.1, 0.2, 0.8);
    Vec3 horizonColor(0.95, 0.55, 0.2);
    Vec3 groundColor(0.25, 0.18, 0.12);

    Vec3 baseColor;
    if (d.y >= 0.0) {
        double blendT = std::pow(t, 0.4);
        baseColor = lerp(horizonColor, zenithColor, blendT);
    } else {
        // 地面颜色
        baseColor = lerp(groundColor, horizonColor, std::max(0.0, 1.0 + d.y * 5.0));
    }

    // 太阳光晕
    double sunDot = std::max(0.0, d.dot(SUN_DIR));
    // 太阳核心
    if (sunDot > 0.9998) {
        baseColor = Vec3(1.0, 0.98, 0.9);
    } else if (sunDot > 0.995) {
        double f = (sunDot - 0.995) / (0.9998 - 0.995);
        Vec3 glowColor(1.0, 0.9, 0.6);
        baseColor = lerp(baseColor, glowColor, f * f);
    } else {
        // 大光晕
        double glowF = std::pow(std::max(0.0, sunDot - 0.3) / 0.7, 4.0);
        Vec3 sunGlow(1.0, 0.7, 0.3);
        baseColor = baseColor + sunGlow * glowF * 0.5;
    }

    // 星空（只在背对太阳的上半球）
    if (d.y > 0.05) {
        // 计算 UV（简单等距投影）
        double starU = 0.5 + std::atan2(d.z, d.x) / (2.0 * M_PI);
        double starV = std::asin(std::max(-1.0, std::min(1.0, d.y))) / M_PI + 0.5;
        double star = starField(starU, starV);

        // 星空可见度（远离太阳时才看得见）
        double starVis = (1.0 - std::pow(sunDot, 0.1)) * std::min(1.0, (d.y - 0.05) * 10.0);
        Vec3 starColor(0.9, 0.95, 1.0);
        baseColor = baseColor + starColor * star * starVis * 0.8;
    }

    // HDR clamp
    baseColor.x = std::min(1.5, baseColor.x);
    baseColor.y = std::min(1.5, baseColor.y);
    baseColor.z = std::min(1.5, baseColor.z);

    return baseColor;
}

void buildCubeMap(CubeMap& cm) {
    // POS_X face: 右面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(1.0, -v, -u).normalize();
            cm.at(POS_X, px, py) = skyColor(dir);
        }
    }
    // NEG_X face: 左面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(-1.0, -v, u).normalize();
            cm.at(NEG_X, px, py) = skyColor(dir);
        }
    }
    // POS_Y face: 上面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(u, 1.0, v).normalize();
            cm.at(POS_Y, px, py) = skyColor(dir);
        }
    }
    // NEG_Y face: 下面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(u, -1.0, -v).normalize();
            cm.at(NEG_Y, px, py) = skyColor(dir);
        }
    }
    // POS_Z face: 前面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(u, -v, 1.0).normalize();
            cm.at(POS_Z, px, py) = skyColor(dir);
        }
    }
    // NEG_Z face: 后面
    for (int py = 0; py < FACE_SIZE; py++) {
        for (int px = 0; px < FACE_SIZE; px++) {
            double u = (px + 0.5) / FACE_SIZE * 2.0 - 1.0;
            double v = (py + 0.5) / FACE_SIZE * 2.0 - 1.0;
            Vec3 dir = Vec3(-u, -v, -1.0).normalize();
            cm.at(NEG_Z, px, py) = skyColor(dir);
        }
    }
}

// ============================================================
// 光线追踪
// ============================================================

struct Ray {
    Vec3 origin, direction;
};

struct HitRecord {
    double t;
    Vec3 point, normal;
    bool frontFace;
    int materialId; // 0=metal, 1=glass, 2=diffuse
    Vec3 albedo;
    double roughness;
    double ior;
};

bool hitSphere(const Ray& r, const Vec3& center, double radius,
               double tMin, double tMax, HitRecord& rec) {
    Vec3 oc = r.origin - center;
    double a = r.direction.dot(r.direction);
    double halfB = oc.dot(r.direction);
    double c = oc.dot(oc) - radius * radius;
    double discriminant = halfB*halfB - a*c;
    if (discriminant < 0) return false;

    double sqrtD = std::sqrt(discriminant);
    double t = (-halfB - sqrtD) / a;
    if (t < tMin || t > tMax) {
        t = (-halfB + sqrtD) / a;
        if (t < tMin || t > tMax) return false;
    }
    rec.t = t;
    rec.point = r.origin + r.direction * t;
    Vec3 outNormal = (rec.point - center) / radius;
    rec.frontFace = r.direction.dot(outNormal) < 0;
    rec.normal = rec.frontFace ? outNormal : -outNormal;
    return true;
}

// 菲涅耳近似 (Schlick)
double fresnelSchlick(double cosTheta, double n1, double n2) {
    double r0 = (n1 - n2) / (n1 + n2);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - std::max(0.0, cosTheta), 5.0);
}

// 追踪射线（带递归深度）
Vec3 traceRay(const Ray& ray, const CubeMap& cm, int depth) {
    if (depth <= 0) return Vec3(0, 0, 0);

    // 场景: 3个球体
    struct Sphere {
        Vec3 center;
        double radius;
        int matId;      // 0=metal, 1=glass, 2=diffuse
        Vec3 albedo;
        double roughness;
        double ior;
    };

    Sphere spheres[] = {
        // 大金属球（左）
        { Vec3(-1.5, 0.0, -4.0), 1.0, 0, Vec3(0.9, 0.7, 0.3), 0.05, 1.5 },
        // 大玻璃球（中）
        { Vec3(0.0, 0.0, -4.0),  1.0, 1, Vec3(0.95, 0.95, 1.0), 0.0, 1.52 },
        // 大铬金属球（右）
        { Vec3(1.5, 0.0, -4.0),  1.0, 0, Vec3(0.95, 0.95, 0.95), 0.02, 1.5 },
        // 地面大球（模拟地板）
        { Vec3(0.0, -101.0, -4.0), 100.0, 2, Vec3(0.4, 0.35, 0.3), 0.8, 1.0 },
    };

    HitRecord closestRec;
    closestRec.t = 1e18;
    bool hit = false;

    for (const auto& sp : spheres) {
        HitRecord rec;
        if (hitSphere(ray, sp.center, sp.radius, 0.001, closestRec.t, rec)) {
            rec.materialId = sp.matId;
            rec.albedo = sp.albedo;
            rec.roughness = sp.roughness;
            rec.ior = sp.ior;
            closestRec = rec;
            hit = true;
        }
    }

    if (!hit) {
        // 采样天空
        return cm.sample(ray.direction);
    }

    Vec3 N = closestRec.normal;
    Vec3 V = (-ray.direction).normalize();

    if (closestRec.materialId == 0) {
        // 金属反射
        Vec3 R = ray.direction - N * 2.0 * ray.direction.dot(N);
        R = R.normalize();
        // 略微随机化方向模拟粗糙度（简化：固定偏移）
        Ray reflRay { closestRec.point + N * 0.001, R };
        Vec3 envColor = traceRay(reflRay, cm, depth - 1);
        return closestRec.albedo * envColor;
    } else if (closestRec.materialId == 1) {
        // 玻璃折射 + 反射
        double etaI = closestRec.frontFace ? 1.0 : closestRec.ior;
        double etaT = closestRec.frontFace ? closestRec.ior : 1.0;
        double cosI = std::max(0.0, std::min(1.0, V.dot(N)));
        double sinI = std::sqrt(1.0 - cosI*cosI);
        double ratio = etaI / etaT;

        bool totalInternalReflection = (ratio * sinI > 1.0);
        double fr = fresnelSchlick(cosI, etaI, etaT);

        Vec3 R = ray.direction - N * 2.0 * ray.direction.dot(N);
        R = R.normalize();
        Ray reflRay { closestRec.point + N * 0.001, R };

        if (totalInternalReflection || fr > 0.9) {
            // 全反射
            return closestRec.albedo * traceRay(reflRay, cm, depth - 1);
        }

        // 折射
        double cosT = std::sqrt(1.0 - ratio*ratio*sinI*sinI);
        Vec3 refractDir = ray.direction * ratio + N * (ratio * cosI - cosT);
        refractDir = refractDir.normalize();
        Vec3 refractOrigin = closestRec.point - N * 0.001;
        Ray refractRay { refractOrigin, refractDir };

        Vec3 refractColor = closestRec.albedo * traceRay(refractRay, cm, depth - 1);
        Vec3 reflectColor = closestRec.albedo * traceRay(reflRay, cm, depth - 1);

        return lerp(refractColor, reflectColor, fr);
    } else {
        // 漫反射 (简化: 只考虑直射光 + 环境光)
        // 直射光
        Vec3 lightDir = SUN_DIR;
        double diff = std::max(0.0, N.dot(lightDir));
        Vec3 diffuse = closestRec.albedo * Vec3(1.3, 1.1, 0.9) * diff;

        // 环境光（采样天球平均色近似）
        Vec3 ambient = closestRec.albedo * Vec3(0.3, 0.35, 0.5);

        return diffuse + ambient;
    }
}

// ============================================================
// 主渲染函数
// ============================================================

int main() {
    printf("=== Environment Mapping with Cube Map Skybox ===\n");
    printf("Building procedural cube map...\n");

    // 1. 构建 Cube Map
    CubeMap cm;
    buildCubeMap(cm);
    printf("  Cube map built (6 faces, %dx%d each)\n", FACE_SIZE, FACE_SIZE);

    // --------------------------------------------------------
    // 输出 1: Cube Map 展开图 (十字形)
    // 布局:
    //         [POS_Y]
    // [NEG_X] [NEG_Z] [POS_X] [POS_Z]
    //         [NEG_Y]
    // 宽: 4*FACE_SIZE, 高: 3*FACE_SIZE
    // --------------------------------------------------------
    {
        int W = 4 * FACE_SIZE, H = 3 * FACE_SIZE;
        std::vector<uint8_t> img(W * H * 3, 30);

        auto gamma = [](double x) -> uint8_t {
            x = std::max(0.0, std::min(1.0, x));
            return static_cast<uint8_t>(std::pow(x, 1.0/2.2) * 255 + 0.5);
        };

        auto blitFace = [&](int face, int offX, int offY) {
            for (int py = 0; py < FACE_SIZE; py++) {
                for (int px = 0; px < FACE_SIZE; px++) {
                    int imgX = offX + px;
                    int imgY = offY + py;
                    if (imgX < 0 || imgX >= W || imgY < 0 || imgY >= H) continue;
                    Vec3 c = cm.at(face, px, py);
                    int idx = (imgY * W + imgX) * 3;
                    img[idx+0] = gamma(c.x);
                    img[idx+1] = gamma(c.y);
                    img[idx+2] = gamma(c.z);
                }
            }
        };

        blitFace(POS_Y, FACE_SIZE,       0);
        blitFace(NEG_X, 0,               FACE_SIZE);
        blitFace(NEG_Z, FACE_SIZE,       FACE_SIZE);
        blitFace(POS_X, 2 * FACE_SIZE,   FACE_SIZE);
        blitFace(POS_Z, 3 * FACE_SIZE,   FACE_SIZE);
        blitFace(NEG_Y, FACE_SIZE,       2 * FACE_SIZE);

        // 绘制面标签分隔线（简单黑线）
        for (int i = 0; i <= 4; i++) {
            for (int py = 0; py < H; py++) {
                int px = i * FACE_SIZE;
                if (px >= 0 && px < W) {
                    int idx = (py * W + px) * 3;
                    img[idx] = img[idx+1] = img[idx+2] = 60;
                }
            }
        }
        for (int j = 0; j <= 3; j++) {
            for (int px = 0; px < W; px++) {
                int py = j * FACE_SIZE;
                if (py >= 0 && py < H) {
                    int idx = (py * W + px) * 3;
                    img[idx] = img[idx+1] = img[idx+2] = 60;
                }
            }
        }

        stbi_write_png("cubemap_faces.png", W, H, 3, img.data(), W * 3);
        printf("Output: cubemap_faces.png (%dx%d)\n", W, H);
    }

    // --------------------------------------------------------
    // 输出 2: 主渲染图 (天空盒 + 反射球)
    // --------------------------------------------------------
    {
        const int W = 800, H = 500;
        std::vector<uint8_t> img(W * H * 3);

        // 相机
        Vec3 camPos(0.0, 0.5, 0.0);
        double fov = 70.0 * M_PI / 180.0;
        double aspect = static_cast<double>(W) / H;
        double halfH = std::tan(fov * 0.5);
        double halfW = aspect * halfH;

        Vec3 camRight(1, 0, 0);
        Vec3 camUp(0, 1, 0);
        Vec3 camFwd(0, 0, -1);

        auto gamma = [](double x) -> uint8_t {
            x = std::max(0.0, std::min(1.0, x));
            // ACES tone mapping + gamma
            double a = 2.51, b = 0.03, c2 = 2.43, d = 0.59, e = 0.14;
            x = (x * (a * x + b)) / (x * (c2 * x + d) + e);
            x = std::max(0.0, std::min(1.0, x));
            return static_cast<uint8_t>(std::pow(x, 1.0/2.2) * 255 + 0.5);
        };

        printf("Rendering main scene (%dx%d)...\n", W, H);

        for (int py = 0; py < H; py++) {
            if (py % 50 == 0) printf("  Row %d/%d\n", py, H);
            for (int px = 0; px < W; px++) {
                // 简单 2x2 SSAA
                Vec3 color(0, 0, 0);
                for (int sy = 0; sy < 2; sy++) {
                    for (int sx = 0; sx < 2; sx++) {
                        double ndcX = ((px + 0.25 + sx * 0.5) / W * 2.0 - 1.0) * halfW;
                        double ndcY = (1.0 - (py + 0.25 + sy * 0.5) / H * 2.0) * halfH;
                        Vec3 dir = (camFwd + camRight * ndcX + camUp * ndcY).normalize();
                        Ray r { camPos, dir };
                        color = color + traceRay(r, cm, 6);
                    }
                }
                color = color / 4.0;

                int idx = (py * W + px) * 3;
                img[idx+0] = gamma(color.x);
                img[idx+1] = gamma(color.y);
                img[idx+2] = gamma(color.z);
            }
        }

        stbi_write_png("skybox_output.png", W, H, 3, img.data(), W * 3);
        printf("Output: skybox_output.png (%dx%d)\n", W, H);
    }

    printf("\n=== Rendering complete! ===\n");
    printf("Files generated:\n");
    printf("  - skybox_output.png : Main scene (skybox + reflection balls)\n");
    printf("  - cubemap_faces.png : Cube map face layout\n");

    return 0;
}
