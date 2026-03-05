/*
 * Subsurface Scattering (SSS) - 次表面散射渲染器
 *
 * 日期: 2026-03-06
 * 技术要点:
 *   - 次表面散射：光线穿透物体内部多次散射后从表面逸出
 *   - 简化的 Dipole 模型（Donner & Jensen, 2005）
 *   - 多层散射（蜡烛、皮肤、玉石等半透明材质）
 *   - 与 Phong 光照模型结合：SSS + 漫反射 + 高光
 *   - 程序化球体场景：多材质对比（蜡烛、皮肤、玉石、金属）
 *
 * 编译: g++ -O2 -std=c++17 -o sss main.cpp -lm
 * 运行: ./sss
 * 输出: sss_output.png (800x600)
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>
#include <cassert>
#include <sstream>

// ─────────────────────────────────────────────
//  数学工具
// ─────────────────────────────────────────────
struct Vec3 {
    double x, y, z;
    Vec3(double v = 0) : x(v), y(v), z(v) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(double t)       const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b)  const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(double t)       const { return {x/t, y/t, z/t}; }
    Vec3 operator/(const Vec3& b)  const { return {x/b.x, y/b.y, z/b.z}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator*=(double t)     { x*=t;   y*=t;   z*=t;   return *this; }

    double dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { double l = length(); return l > 1e-9 ? *this/l : Vec3(0); }
    double lengthSq() const { return x*x + y*y + z*z; }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }
Vec3 clamp(const Vec3& v, double lo, double hi) {
    return { std::max(lo, std::min(hi, v.x)),
             std::max(lo, std::min(hi, v.y)),
             std::max(lo, std::min(hi, v.z)) };
}
Vec3 lerp(const Vec3& a, const Vec3& b, double t) {
    return a * (1.0 - t) + b * t;
}

// ─────────────────────────────────────────────
//  随机数
// ─────────────────────────────────────────────
static std::mt19937 rng(42);
static std::uniform_real_distribution<double> uni01(0.0, 1.0);
double rand01() { return uni01(rng); }

Vec3 randomInUnitSphere() {
    while (true) {
        Vec3 p(rand01()*2-1, rand01()*2-1, rand01()*2-1);
        if (p.lengthSq() < 1.0) return p;
    }
}
Vec3 randomOnHemisphere(const Vec3& normal) {
    Vec3 v = randomInUnitSphere().normalized();
    return v.dot(normal) > 0 ? v : -v;
}

// ─────────────────────────────────────────────
//  光线
// ─────────────────────────────────────────────
struct Ray {
    Vec3 origin, dir;
    Vec3 at(double t) const { return origin + dir * t; }
};

// ─────────────────────────────────────────────
//  材质类型
// ─────────────────────────────────────────────
enum MaterialType {
    MAT_DIFFUSE,         // 普通漫反射（不透明参考）
    MAT_SSS_WAX,         // 蜡烛（强散射，暖橙色）
    MAT_SSS_SKIN,        // 皮肤（中等散射，肉色）
    MAT_SSS_JADE,        // 玉石（弱散射，翠绿）
    MAT_METAL            // 金属（反射，对比用）
};

struct Material {
    MaterialType type;
    Vec3  albedo;          // 表面颜色
    Vec3  sssColor;        // 散射颜色（内部光）
    double scatterCoeff;   // 散射系数 σ_s
    double absorptionCoeff;// 吸收系数 σ_a
    double g;              // 各向异性因子 (Henyey-Greenstein)
    double roughness;      // 高光粗糙度
    double metallic;       // 金属度
};

// 预定义材质
Material makeDiffuse(Vec3 color) {
    return { MAT_DIFFUSE, color, Vec3(0), 0, 0, 0, 0.8, 0.0 };
}
Material makeWax() {
    // 蜡烛：高散射，暖橙色透射
    return { MAT_SSS_WAX,
             Vec3(0.95, 0.85, 0.70),  // 表面：奶白色
             Vec3(1.0, 0.6, 0.2),     // SSS：橙色
             8.0, 0.5, 0.3, 0.6, 0.0 };
}
Material makeSkin() {
    // 皮肤：中等散射，肉色
    return { MAT_SSS_SKIN,
             Vec3(0.9, 0.7, 0.6),     // 表面
             Vec3(0.9, 0.4, 0.3),     // SSS：红色（血液散射）
             5.0, 1.0, 0.1, 0.7, 0.0 };
}
Material makeJade() {
    // 玉石：低散射，翠绿
    return { MAT_SSS_JADE,
             Vec3(0.3, 0.7, 0.4),     // 表面
             Vec3(0.1, 0.8, 0.3),     // SSS：绿色
             3.0, 2.0, -0.1, 0.5, 0.0 };
}
Material makeMetal(Vec3 color) {
    return { MAT_METAL, color, Vec3(0), 0, 0, 0, 0.1, 1.0 };
}

// ─────────────────────────────────────────────
//  球体
// ─────────────────────────────────────────────
struct Sphere {
    Vec3 center;
    double radius;
    Material mat;
};

struct HitRecord {
    double t;
    Vec3 point;
    Vec3 normal;
    bool frontFace;
    const Material* mat;

    void setFaceNormal(const Ray& r, const Vec3& outNormal) {
        frontFace = r.dir.dot(outNormal) < 0;
        normal = frontFace ? outNormal : -outNormal;
    }
};

bool hitSphere(const Sphere& s, const Ray& r, double tMin, double tMax, HitRecord& rec) {
    Vec3 oc = r.origin - s.center;
    double a = r.dir.dot(r.dir);
    double half_b = oc.dot(r.dir);
    double c = oc.dot(oc) - s.radius * s.radius;
    double disc = half_b * half_b - a * c;
    if (disc < 0) return false;
    double sqrtD = std::sqrt(disc);
    double root = (-half_b - sqrtD) / a;
    if (root < tMin || root > tMax) {
        root = (-half_b + sqrtD) / a;
        if (root < tMin || root > tMax) return false;
    }
    rec.t = root;
    rec.point = r.at(root);
    Vec3 outNormal = (rec.point - s.center) / s.radius;
    rec.setFaceNormal(r, outNormal);
    rec.mat = &s.mat;
    return true;
}

// ─────────────────────────────────────────────
//  场景
// ─────────────────────────────────────────────
struct Scene {
    std::vector<Sphere> spheres;

    bool hit(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        HitRecord tmp;
        bool hitAny = false;
        double closest = tMax;
        for (auto& s : spheres) {
            if (hitSphere(s, r, tMin, closest, tmp)) {
                hitAny = true;
                closest = tmp.t;
                rec = tmp;
            }
        }
        return hitAny;
    }
};

// ─────────────────────────────────────────────
//  光源
// ─────────────────────────────────────────────
struct Light {
    Vec3 pos;
    Vec3 color;
    double intensity;
};

// ─────────────────────────────────────────────
//  Dipole 次表面散射模型（简化版）
//
//  基于 Donner & Jensen (2005) 的 Dipole 近似：
//  在半径 r 处的散射贡献 R(r) ∝ exp(-σ_eff * r) / r
//  其中 σ_eff = sqrt(3 * σ_a * (σ_a + σ_s))
//
//  实现思路：
//  1. 对于每个光源，计算光线穿透球体后在内部散射的贡献
//  2. 使用 dipole 函数近似：SSS 强度 ∝ exp(-σ_eff * thickness)
//  3. thickness = 光线从入射点到出射点的路径长度
// ─────────────────────────────────────────────

// 计算光线穿过球体的距离（thickness）
double computeThickness(const Vec3& hitPoint, const Vec3& lightDir,
                        const Sphere& sphere) {
    // 从当前 hitPoint 出发，沿 lightDir 的反方向找到对面的交点
    Ray backRay;
    backRay.origin = hitPoint - lightDir * 1e-4;
    backRay.dir = -lightDir;
    Vec3 oc = backRay.origin - sphere.center;
    double a = backRay.dir.dot(backRay.dir);
    double half_b = oc.dot(backRay.dir);
    double c = oc.dot(oc) - sphere.radius * sphere.radius;
    double disc = half_b * half_b - a * c;
    if (disc < 0) return 2.0 * sphere.radius; // 默认厚度
    double sqrtD = std::sqrt(disc);
    double t = (-half_b + sqrtD) / a;
    return std::max(0.0, t);
}

// Dipole 近似函数
// 返回次表面散射的颜色贡献
Vec3 computeSSS(const Vec3& hitPoint, const Vec3& normal,
                const Vec3& lightDir, const Vec3& lightColor,
                const Sphere& sphere, const Material& mat,
                double lightDistance) {
    // 计算有效散射系数
    double sigmaS = mat.scatterCoeff;
    double sigmaA = mat.absorptionCoeff;
    // σ_t = σ_s + σ_a（消光系数）
    double sigmaT = sigmaS + sigmaA;
    // σ_eff = sqrt(3 * σ_a * σ_t)（有效消光系数）
    double sigmaEff = std::sqrt(3.0 * sigmaA * sigmaT);

    // 计算穿透厚度（球体直径方向）
    double thickness = computeThickness(hitPoint, lightDir, sphere);

    // 光源到物体的角度贡献（背光时更强的 SSS）
    // 对于次表面散射，背面光源贡献更明显
    double wrap = 1.0; // wrap lighting factor
    double dotBack = std::max(0.0, -normal.dot(lightDir)); // 背光分量
    double dotFront = std::max(0.0, normal.dot(lightDir));  // 正面分量
    // Wrapped diffuse for translucency
    double wrappedDiffuse = (dotFront + wrap) / (1.0 + wrap);

    // Dipole 散射项：exp(-σ_eff * d)
    double scatter = std::exp(-sigmaEff * thickness);

    // 反向透射（背面光穿透）
    double backTransmit = std::exp(-sigmaT * thickness * 0.5);

    // 距离衰减
    double atten = 1.0 / (1.0 + lightDistance * lightDistance * 0.05);

    // 组合 SSS 颜色
    Vec3 sssContrib = mat.sssColor * lightColor *
                      (scatter * dotBack + wrappedDiffuse * backTransmit * 0.5) *
                      atten;

    return clamp(sssContrib, 0, 2.0);
}

// ─────────────────────────────────────────────
//  Phong 光照
// ─────────────────────────────────────────────
Vec3 phongShading(const HitRecord& rec, const Ray& viewRay,
                  const Light& light, const Scene& scene,
                  const Sphere& /*hitSphere*/) {
    const Material& mat = *rec.mat;
    Vec3 N = rec.normal.normalized();
    Vec3 L = (light.pos - rec.point).normalized();
    Vec3 V = (-viewRay.dir).normalized();
    Vec3 R = (2.0 * N.dot(L) * N - L).normalized();

    // 阴影检测
    Ray shadowRay;
    shadowRay.origin = rec.point + N * 1e-4;
    shadowRay.dir = L;
    HitRecord shadowRec;
    double lightDist = (light.pos - rec.point).length();
    bool inShadow = scene.hit(shadowRay, 1e-4, lightDist, shadowRec);

    // 漫反射
    double diff = std::max(0.0, N.dot(L));
    Vec3 diffuse = mat.albedo * light.color * diff * light.intensity;

    // 高光
    double spec = 0.0;
    if (!inShadow) {
        spec = std::pow(std::max(0.0, V.dot(R)), 1.0 / (mat.roughness + 1e-6) * 16.0);
    }
    Vec3 specular = light.color * spec * light.intensity * (1.0 - mat.roughness) * 0.5;

    // 如果在阴影中，漫反射减弱
    if (inShadow) diffuse *= 0.0;

    return diffuse + specular;
}

// ─────────────────────────────────────────────
//  环境光
// ─────────────────────────────────────────────
Vec3 ambientLight(const Vec3& normal, const Vec3& albedo) {
    // 简单的半球环境光
    double skyFactor = 0.5 * (1.0 + normal.y);
    Vec3 skyColor(0.5, 0.7, 1.0);
    Vec3 groundColor(0.2, 0.15, 0.1);
    Vec3 ambient = lerp(groundColor, skyColor, skyFactor);
    return albedo * ambient * 0.3;
}

// ─────────────────────────────────────────────
//  金属反射（简化 Blinn-Phong 高光）
// ─────────────────────────────────────────────
Vec3 metalShading(const HitRecord& rec, const Ray& viewRay,
                  const std::vector<Light>& lights, const Scene& scene) {
    const Material& mat = *rec.mat;
    Vec3 N = rec.normal.normalized();
    Vec3 V = (-viewRay.dir).normalized();

    Vec3 result(0);
    for (const auto& light : lights) {
        Vec3 L = (light.pos - rec.point).normalized();
        Vec3 H = (V + L).normalized();

        double lightDist = (light.pos - rec.point).length();
        Ray shadowRay;
        shadowRay.origin = rec.point + N * 1e-4;
        shadowRay.dir = L;
        HitRecord shadowRec;
        bool inShadow = scene.hit(shadowRay, 1e-4, lightDist, shadowRec);

        if (!inShadow) {
            double ndotl = std::max(0.0, N.dot(L));
            double ndoth = std::max(0.0, N.dot(H));
            double spec = std::pow(ndoth, 64.0 / (mat.roughness + 0.01));
            double atten = 1.0 / (1.0 + lightDist * lightDist * 0.02);
            result += mat.albedo * light.color * ndotl * atten * light.intensity * 0.5;
            result += light.color * spec * atten * light.intensity;
        }
    }
    return result;
}

// ─────────────────────────────────────────────
//  主着色函数
// ─────────────────────────────────────────────
Vec3 shade(const Ray& ray, const Scene& scene,
           const std::vector<Sphere>& spheres,
           const std::vector<Light>& lights,
           int depth = 0) {
    if (depth > 3) return Vec3(0);

    HitRecord rec;
    if (!scene.hit(ray, 1e-4, 1e9, rec)) {
        // 背景：渐变天空
        Vec3 dir = ray.dir.normalized();
        double t = 0.5 * (dir.y + 1.0);
        return lerp(Vec3(0.2, 0.2, 0.3), Vec3(0.4, 0.6, 0.9), t);
    }

    const Material& mat = *rec.mat;
    Vec3 color(0);

    // 找到对应的 sphere（用于计算厚度）
    const Sphere* hitSph = nullptr;
    {
        HitRecord tmp;
        double closest = 1e9;
        for (auto& s : spheres) {
            if (hitSphere(s, ray, 1e-4, closest, tmp)) {
                closest = tmp.t;
                hitSph = &s;
            }
        }
    }

    if (mat.type == MAT_METAL) {
        color = metalShading(rec, ray, lights, scene);
        // 环境反射
        Vec3 N = rec.normal.normalized();
        Vec3 reflDir = ray.dir - 2.0 * N.dot(ray.dir) * N;
        Ray reflRay;
        reflRay.origin = rec.point + N * 1e-4;
        reflRay.dir = (reflDir + randomInUnitSphere() * mat.roughness).normalized();
        Vec3 reflColor = shade(reflRay, scene, spheres, lights, depth + 1);
        color = lerp(color, reflColor * mat.albedo, 0.6);
    }
    else if (mat.type == MAT_DIFFUSE) {
        color = ambientLight(rec.normal, mat.albedo);
        if (hitSph) {
            for (const auto& light : lights) {
                color += phongShading(rec, ray, light, scene, *hitSph);
            }
        }
    }
    else {
        // SSS 材质（Wax/Skin/Jade）
        color = ambientLight(rec.normal, mat.albedo);

        if (hitSph) {
            for (const auto& light : lights) {
                // 普通 Phong 漫反射（表面直接光）
                Vec3 N = rec.normal.normalized();
                Vec3 L = (light.pos - rec.point).normalized();
                double lightDist = (light.pos - rec.point).length();

                // 阴影检测（半透明物体只做软阴影）
                Ray shadowRay;
                shadowRay.origin = rec.point + N * 1e-4;
                shadowRay.dir = L;
                HitRecord shadowRec;
                bool inShadow = scene.hit(shadowRay, 1e-4, lightDist, shadowRec);

                double diff = std::max(0.0, N.dot(L));
                Vec3 V = (-ray.dir).normalized();
                Vec3 R = (2.0 * N.dot(L) * N - L).normalized();
                double spec = std::pow(std::max(0.0, V.dot(R)), 16.0);

                double atten = 1.0 / (1.0 + lightDist * lightDist * 0.02);
                double shadowFactor = inShadow ? 0.0 : 1.0;

                Vec3 directDiff = mat.albedo * light.color * diff *
                                  atten * light.intensity * shadowFactor;
                Vec3 directSpec = light.color * spec * 0.4 *
                                  atten * light.intensity * shadowFactor;

                // 次表面散射贡献
                Vec3 sss = computeSSS(rec.point, N, L, light.color,
                                      *hitSph, mat, lightDist);
                sss *= light.intensity * atten;

                color += directDiff + directSpec + sss;
            }
        }

        // 微弱的表面反射（菲涅尔效果）
        Vec3 N = rec.normal.normalized();
        Vec3 V = (-ray.dir).normalized();
        double cosTheta = std::max(0.0, V.dot(N));
        double fresnel = 0.04 + 0.96 * std::pow(1.0 - cosTheta, 5.0);
        if (fresnel > 0.05 && depth < 2) {
            Vec3 reflDir = ray.dir - 2.0 * N.dot(ray.dir) * N;
            Ray reflRay;
            reflRay.origin = rec.point + N * 1e-4;
            reflRay.dir = reflDir.normalized();
            Vec3 reflColor = shade(reflRay, scene, spheres, lights, depth + 1);
            color = lerp(color, reflColor, fresnel * 0.3);
        }
    }

    return color;
}

// ─────────────────────────────────────────────
//  色调映射
// ─────────────────────────────────────────────
Vec3 acesTonemap(Vec3 x) {
    const double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

Vec3 gammaCorrect(Vec3 color) {
    color = acesTonemap(color);
    return { std::pow(color.x, 1.0/2.2),
             std::pow(color.y, 1.0/2.2),
             std::pow(color.z, 1.0/2.2) };
}

// ─────────────────────────────────────────────
//  PPM 图像保存
// ─────────────────────────────────────────────
struct Image {
    int width, height;
    std::vector<uint8_t> data; // RGB

    Image(int w, int h) : width(w), height(h), data(w * h * 3, 0) {}

    void setPixel(int x, int y, Vec3 color) {
        int idx = (y * width + x) * 3;
        data[idx+0] = static_cast<uint8_t>(std::min(255.0, color.x * 255.0 + 0.5));
        data[idx+1] = static_cast<uint8_t>(std::min(255.0, color.y * 255.0 + 0.5));
        data[idx+2] = static_cast<uint8_t>(std::min(255.0, color.z * 255.0 + 0.5));
    }

    Vec3 getPixel(int x, int y) const {
        int idx = (y * width + x) * 3;
        return Vec3(data[idx]/255.0, data[idx+1]/255.0, data[idx+2]/255.0);
    }

    bool savePPM(const std::string& filename) const {
        std::ofstream f(filename, std::ios::binary);
        if (!f) return false;
        f << "P6\n" << width << " " << height << "\n255\n";
        f.write(reinterpret_cast<const char*>(data.data()), data.size());
        return true;
    }

    // 内嵌 BMP 保存（无需外部库）
    bool saveBMP(const std::string& filename) const {
        std::ofstream f(filename, std::ios::binary);
        if (!f) return false;

        int rowSize = (width * 3 + 3) & ~3;
        int pixelData = 54;
        int fileSize = pixelData + rowSize * height;

        auto write2 = [&](uint16_t v) { f.write(reinterpret_cast<char*>(&v), 2); };
        auto write4 = [&](uint32_t v) { f.write(reinterpret_cast<char*>(&v), 4); };

        // File Header
        f << "BM";
        write4(fileSize); write4(0); write4(pixelData);
        // Info Header
        write4(40); write4(width); write4(-static_cast<int32_t>(height));
        write2(1); write2(24); write4(0); write4(rowSize * height);
        write4(2835); write4(2835); write4(0); write4(0);

        std::vector<uint8_t> row(rowSize, 0);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int src = (y * width + x) * 3;
                int dst = x * 3;
                row[dst+0] = data[src+2]; // B
                row[dst+1] = data[src+1]; // G
                row[dst+2] = data[src+0]; // R
            }
            f.write(reinterpret_cast<char*>(row.data()), rowSize);
        }
        return true;
    }
};

// ─────────────────────────────────────────────
//  BMP → PNG 转换（通过 convert 工具）
// ─────────────────────────────────────────────
bool convertToPNG(const std::string& bmpFile, const std::string& pngFile) {
    std::string cmd = "convert " + bmpFile + " " + pngFile + " 2>&1";
    int ret = std::system(cmd.c_str());
    return (ret == 0);
}

// ─────────────────────────────────────────────
//  场景构建
// ─────────────────────────────────────────────
void buildScene(std::vector<Sphere>& spheres, std::vector<Light>& lights) {
    // ── 材质 ──
    // 中心：蜡烛球（最强 SSS）
    Sphere waxSphere;
    waxSphere.center = Vec3(-1.2, 0.0, -3.5);
    waxSphere.radius = 0.9;
    waxSphere.mat = makeWax();
    spheres.push_back(waxSphere);

    // 左：皮肤球
    Sphere skinSphere;
    skinSphere.center = Vec3(1.2, 0.0, -3.5);
    skinSphere.radius = 0.9;
    skinSphere.mat = makeSkin();
    spheres.push_back(skinSphere);

    // 右：玉石球
    Sphere jadeSphere;
    jadeSphere.center = Vec3(0.0, 0.0, -2.5);
    jadeSphere.radius = 0.7;
    jadeSphere.mat = makeJade();
    spheres.push_back(jadeSphere);

    // 右上：金属球（对比）
    Sphere metalSphere;
    metalSphere.center = Vec3(2.5, 0.5, -4.0);
    metalSphere.radius = 0.8;
    metalSphere.mat = makeMetal(Vec3(0.8, 0.7, 0.6));
    spheres.push_back(metalSphere);

    // 左上角：小蜡烛球（灯源附近）
    Sphere smallWax;
    smallWax.center = Vec3(-2.5, 1.0, -4.0);
    smallWax.radius = 0.5;
    smallWax.mat = makeWax();
    spheres.push_back(smallWax);

    // 地面（大球模拟）
    Sphere ground;
    ground.center = Vec3(0, -101, -3.5);
    ground.radius = 100.0;
    ground.mat = makeDiffuse(Vec3(0.4, 0.4, 0.45));
    spheres.push_back(ground);

    // ── 光源 ──
    // 主光源（从右上照射，暖白光）
    Light mainLight;
    mainLight.pos = Vec3(4.0, 5.0, 0.0);
    mainLight.color = Vec3(1.0, 0.95, 0.85);
    mainLight.intensity = 2.0;
    lights.push_back(mainLight);

    // 背光（蓝色冷光，从左后照射，增强 SSS 透射效果）
    Light backLight;
    backLight.pos = Vec3(-4.0, 2.0, -8.0);
    backLight.color = Vec3(0.3, 0.5, 1.0);
    backLight.intensity = 1.5;
    lights.push_back(backLight);

    // 底部补光（模拟地面反弹）
    Light fillLight;
    fillLight.pos = Vec3(0.0, -2.0, 0.0);
    fillLight.color = Vec3(0.8, 0.7, 0.6);
    fillLight.intensity = 0.8;
    lights.push_back(fillLight);
}

// ─────────────────────────────────────────────
//  主函数
// ─────────────────────────────────────────────
int main() {
    const int WIDTH  = 800;
    const int HEIGHT = 600;
    const int SAMPLES = 4; // 每像素采样数（抗锯齿）
    const double INV_SAMPLES = 1.0 / SAMPLES;

    std::cout << "[SSS Renderer] 次表面散射渲染器" << std::endl;
    std::cout << "  分辨率: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "  每像素采样: " << SAMPLES << std::endl;

    // 构建场景
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    buildScene(spheres, lights);
    std::cout << "  球体数量: " << spheres.size() << std::endl;
    std::cout << "  光源数量: " << lights.size() << std::endl;

    // 构建 BVH-like scene（直接线性搜索，球少不需要 BVH）
    Scene scene;
    scene.spheres = spheres;

    // 相机设置
    Vec3 camPos(0.0, 1.0, 2.0);
    Vec3 camTarget(0.0, 0.0, -3.0);
    Vec3 camUp(0.0, 1.0, 0.0);
    Vec3 camDir = (camTarget - camPos).normalized();
    Vec3 camRight = camDir.cross(camUp).normalized();
    Vec3 camUpReal = camRight.cross(camDir).normalized();

    double fovY = 45.0 * M_PI / 180.0;
    double aspect = static_cast<double>(WIDTH) / HEIGHT;
    double halfH = std::tan(fovY / 2.0);
    double halfW = halfH * aspect;

    Image img(WIDTH, HEIGHT);

    int lastPct = -1;
    std::cout << "  渲染进度: " << std::flush;

    for (int y = 0; y < HEIGHT; y++) {
        int pct = (y * 100) / HEIGHT;
        if (pct != lastPct && pct % 10 == 0) {
            std::cout << pct << "%" << std::flush;
            lastPct = pct;
        }

        for (int x = 0; x < WIDTH; x++) {
            Vec3 accumulated(0);

            for (int s = 0; s < SAMPLES; s++) {
                double u = (x + rand01() - 0.5) / (WIDTH  - 1);
                double v = (y + rand01() - 0.5) / (HEIGHT - 1);

                // 构建光线
                double px =  (2.0 * u - 1.0) * halfW;
                double py = -(2.0 * v - 1.0) * halfH;

                Vec3 rayDir = (camDir + camRight * px + camUpReal * py).normalized();
                Ray ray;
                ray.origin = camPos;
                ray.dir = rayDir;

                Vec3 color = shade(ray, scene, spheres, lights);
                accumulated += color;
            }

            Vec3 final = gammaCorrect(accumulated * INV_SAMPLES);
            img.setPixel(x, y, final);
        }
    }
    std::cout << " 100%" << std::endl;

    // 保存文件
    std::string bmpFile = "sss_output.bmp";
    std::string pngFile = "sss_output.png";

    if (!img.saveBMP(bmpFile)) {
        std::cerr << "[ERROR] 无法保存 BMP 文件" << std::endl;
        return 1;
    }
    std::cout << "  ✅ 保存 BMP: " << bmpFile << std::endl;

    // 转换为 PNG
    if (convertToPNG(bmpFile, pngFile)) {
        std::cout << "  ✅ 转换 PNG: " << pngFile << std::endl;
    } else {
        std::cout << "  ⚠️ convert 命令失败，尝试直接使用 BMP" << std::endl;
        // 使用 ppm 作为备选
        img.savePPM("sss_output.ppm");
        std::cout << "  ✅ 保存 PPM: sss_output.ppm (备选)" << std::endl;
    }

    // 简单的输出验证（在代码中打印统计数据）
    // 统计图像亮度
    double totalBrightness = 0;
    int validPixels = 0;
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            Vec3 p = img.getPixel(x, y);
            double brightness = 0.299*p.x + 0.587*p.y + 0.114*p.z;
            totalBrightness += brightness;
            validPixels++;
        }
    }
    double avgBrightness = totalBrightness / validPixels * 255.0;
    std::cout << "\n  📊 渲染统计:" << std::endl;
    std::cout << "     平均亮度: " << avgBrightness << " /255" << std::endl;

    // 检查中心球（蜡烛球，在 -1.2, 0, -3.5）
    // 投影到图像空间检查颜色
    // 大致位置：约 x=230, y=290（基于相机参数估算）
    Vec3 centerPixel = img.getPixel(240, 295);
    std::cout << "     蜡烛球中心 RGB: (" 
              << static_cast<int>(centerPixel.x * 255) << ", "
              << static_cast<int>(centerPixel.y * 255) << ", "
              << static_cast<int>(centerPixel.z * 255) << ")" << std::endl;

    if (avgBrightness < 5.0) {
        std::cerr << "[VALIDATION ERROR] 图像太暗（全黑）！" << std::endl;
        return 1;
    }
    if (avgBrightness > 250.0) {
        std::cerr << "[VALIDATION ERROR] 图像太亮（全白）！" << std::endl;
        return 1;
    }
    std::cout << "  ✅ 亮度验证通过" << std::endl;

    std::cout << "\n[完成] 次表面散射渲染成功！" << std::endl;
    return 0;
}
