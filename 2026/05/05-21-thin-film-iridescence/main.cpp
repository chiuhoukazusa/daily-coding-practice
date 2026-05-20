/*
 * Thin Film Iridescence Renderer
 * 薄膜干涉彩虹色渲染器
 *
 * 技术要点:
 * - 薄膜干涉 (Thin Film Interference): 光在薄膜两个界面的反射光相互叠加
 * - 波动光学: 利用波长依赖的相位差计算干涉强度
 * - 菲涅尔方程 (Fresnel Equations): 计算每个界面的反射/透射系数
 * - 色彩映射: 将干涉光谱转换为 RGB 颜色 (CIE XYZ 色匹配函数)
 * - 场景: 肥皂泡球体 + 油膜地面 + 甲虫翅膀平面
 *
 * 数学原理:
 * 薄膜干涉公式: δ = 4π·n₂·d·cos(θ₂) / λ
 * 其中 n₂ = 薄膜折射率, d = 薄膜厚度, θ₂ = 折射角, λ = 波长
 * 反射强度: I = |r₁² + t₁²·r₂²·t₂² + 2·r₁·t₁²·r₂·t₂·cos(δ)|² / (1 - r₁²·r₂²)²
 *
 * 输出: thin_film_output.png (800x600)
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <cstdint>

// ===== 数学工具 =====

struct Vec2 {
    double x, y;
    Vec2(double x=0, double y=0) : x(x), y(y) {}
};

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    double lengthSq() const { return x*x + y*y + z*z; }
    Vec3 normalized() const {
        double len = length();
        if (len < 1e-12) return {0,0,1};
        return *this / len;
    }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    Vec3& operator*=(double t) { x*=t; y*=t; z*=t; return *this; }
};

inline Vec3 operator*(double t, const Vec3& v) { return v * t; }
inline Vec3 reflect(const Vec3& I, const Vec3& N) {
    return I - 2.0 * I.dot(N) * N;
}

// ===== 光线 =====
struct Ray {
    Vec3 origin, dir;
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalized()) {}
    Vec3 at(double t) const { return origin + dir * t; }
};

// ===== 常量 =====
const double PI = 3.14159265358979323846;
const int WIDTH  = 800;
const int HEIGHT = 600;
const double INF = 1e18;

// ===== 薄膜干涉物理计算 =====

// CIE 1931 色匹配函数 (简化版, 380-780nm, 步长10nm)
// 数据来源: CIE colorimetry standard
static const double CIE_X[41] = {
    0.0014, 0.0042, 0.0143, 0.0435, 0.1344, 0.2839, 0.3483, 0.3362, 0.2908, 0.1954,
    0.0956, 0.0320, 0.0049, 0.0093, 0.0633, 0.1655, 0.2904, 0.4334, 0.5945, 0.7621,
    0.9163, 1.0263, 1.0622, 1.0026, 0.8544, 0.6424, 0.4479, 0.2835, 0.1649, 0.0874,
    0.0468, 0.0227, 0.0114, 0.0058, 0.0029, 0.0014, 0.0007, 0.0003, 0.0002, 0.0001,
    0.0000
};
static const double CIE_Y[41] = {
    0.0000, 0.0001, 0.0004, 0.0012, 0.0040, 0.0116, 0.0230, 0.0380, 0.0600, 0.0910,
    0.1390, 0.2080, 0.3230, 0.5030, 0.7100, 0.8620, 0.9540, 0.9950, 0.9950, 0.9520,
    0.8700, 0.7570, 0.6310, 0.5030, 0.3810, 0.2650, 0.1750, 0.1070, 0.0610, 0.0320,
    0.0170, 0.0082, 0.0041, 0.0021, 0.0010, 0.0005, 0.0003, 0.0001, 0.0001, 0.0000,
    0.0000
};
static const double CIE_Z[41] = {
    0.0065, 0.0201, 0.0679, 0.2074, 0.6456, 1.3856, 1.7471, 1.7721, 1.6692, 1.2876,
    0.8130, 0.4652, 0.2720, 0.1582, 0.0782, 0.0422, 0.0203, 0.0087, 0.0039, 0.0021,
    0.0017, 0.0011, 0.0008, 0.0003, 0.0002, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
    0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
    0.0000
};

// 菲涅尔方程 - 计算振幅反射系数 (s偏振 和 p偏振)
// n1: 入射介质折射率, n2: 折射介质折射率, cosTheta1: 入射角余弦
// 返回 {rs, rp, ts, tp}
struct FresnelCoeffs {
    double rs, rp, ts, tp;
};

FresnelCoeffs fresnelAmplitude(double n1, double n2, double cosTheta1) {
    // 使用 Snell 定律计算折射角
    double sinTheta1 = std::sqrt(std::max(0.0, 1.0 - cosTheta1*cosTheta1));
    double sinTheta2 = n1 * sinTheta1 / n2;

    FresnelCoeffs result;
    // 全内反射
    if (sinTheta2 >= 1.0) {
        result.rs = -1.0; result.rp = 1.0;
        result.ts = 0.0;  result.tp = 0.0;
        return result;
    }

    double cosTheta2 = std::sqrt(std::max(0.0, 1.0 - sinTheta2*sinTheta2));

    // 振幅反射/透射系数 (Fresnel 方程)
    result.rs = (n1*cosTheta1 - n2*cosTheta2) / (n1*cosTheta1 + n2*cosTheta2);
    result.rp = (n2*cosTheta1 - n1*cosTheta2) / (n2*cosTheta1 + n1*cosTheta2);
    result.ts = 2.0*n1*cosTheta1 / (n1*cosTheta1 + n2*cosTheta2);
    result.tp = 2.0*n1*cosTheta1 / (n2*cosTheta1 + n1*cosTheta2);

    return result;
}

// 计算薄膜干涉在特定波长下的反射率
// n0: 外部介质折射率 (通常=1.0 空气)
// n1: 薄膜折射率
// n2: 基底折射率
// d:  薄膜厚度 (nm)
// lambda: 波长 (nm)
// cosTheta0: 入射角余弦
double thinFilmReflectance(double n0, double n1, double n2,
                            double d, double lambda, double cosTheta0) {
    // 在薄膜中的折射角
    double sinTheta0 = std::sqrt(std::max(0.0, 1.0 - cosTheta0*cosTheta0));
    double sinTheta1 = n0 * sinTheta0 / n1;
    if (sinTheta1 >= 1.0) return 1.0; // 全内反射
    double cosTheta1 = std::sqrt(std::max(0.0, 1.0 - sinTheta1*sinTheta1));

    // 在基底的折射角
    double sinTheta2 = n1 * sinTheta1 / n2;
    double cosTheta2 = (sinTheta2 < 1.0) ?
        std::sqrt(std::max(0.0, 1.0 - sinTheta2*sinTheta2)) : 0.0;

    // 界面0-1 的菲涅尔系数
    FresnelCoeffs f01;
    f01.rs = (n0*cosTheta0 - n1*cosTheta1) / (n0*cosTheta0 + n1*cosTheta1 + 1e-10);
    f01.rp = (n1*cosTheta0 - n0*cosTheta1) / (n1*cosTheta0 + n0*cosTheta1 + 1e-10);
    f01.ts = 2.0*n0*cosTheta0 / (n0*cosTheta0 + n1*cosTheta1 + 1e-10);
    f01.tp = 2.0*n0*cosTheta0 / (n1*cosTheta0 + n0*cosTheta1 + 1e-10);

    // 界面1-2 的菲涅尔系数
    FresnelCoeffs f12;
    f12.rs = (sinTheta2 < 1.0) ?
        (n1*cosTheta1 - n2*cosTheta2) / (n1*cosTheta1 + n2*cosTheta2 + 1e-10) : -1.0;
    f12.rp = (sinTheta2 < 1.0) ?
        (n2*cosTheta1 - n1*cosTheta2) / (n2*cosTheta1 + n1*cosTheta2 + 1e-10) : 1.0;
    (void)f12.ts; (void)f12.tp; // 不需要透射系数

    // 相位差: δ = 4π·n₁·d·cosθ₁ / λ
    double delta = 4.0 * PI * n1 * d * cosTheta1 / lambda;

    // s偏振的总反射振幅 (薄膜干涉公式)
    // r_total = (r01_s + r12_s * e^{iδ}) / (1 + r01_s * r12_s * e^{iδ})
    // 实部: (r01 + r12*cos(δ)) / denom
    // 虚部: r12*sin(δ) / denom
    double cosD = std::cos(delta);
    double sinD = std::sin(delta);

    double denomS = 1.0 + f01.rs*f01.rs*f12.rs*f12.rs
                    + 2.0*f01.rs*f12.rs*cosD;
    double Rs = 0.0;
    if (denomS > 1e-10) {
        double numS_re = f01.rs + f12.rs*cosD;
        double numS_im = f12.rs*sinD;
        Rs = (numS_re*numS_re + numS_im*numS_im) / denomS;
    }

    double denomP = 1.0 + f01.rp*f01.rp*f12.rp*f12.rp
                    + 2.0*f01.rp*f12.rp*cosD;
    double Rp = 0.0;
    if (denomP > 1e-10) {
        double numP_re = f01.rp + f12.rp*cosD;
        double numP_im = f12.rp*sinD;
        Rp = (numP_re*numP_re + numP_im*numP_im) / denomP;
    }

    // 非偏振光: 平均 s 和 p 分量
    return 0.5 * (Rs + Rp);
}

// 将薄膜反射光谱转换为 XYZ 颜色
// 对 380-780nm 范围内的波长积分
Vec3 thinFilmToXYZ(double n0, double n1, double n2, double d, double cosTheta0) {
    double X = 0, Y = 0, Z = 0;
    double dLambda = 10.0; // 步长 10nm

    for (int i = 0; i < 41; i++) {
        double lambda = 380.0 + i * dLambda;
        double R = thinFilmReflectance(n0, n1, n2, d, lambda, cosTheta0);

        // 假设白色入射光 (等能量光谱)
        X += R * CIE_X[i] * dLambda;
        Y += R * CIE_Y[i] * dLambda;
        Z += R * CIE_Z[i] * dLambda;
    }

    return {X, Y, Z};
}

// XYZ 转 sRGB (D65 白点)
Vec3 xyzToRGB(const Vec3& xyz) {
    // CIE XYZ to sRGB (D65) 变换矩阵
    double r =  3.2404542*xyz.x - 1.5371385*xyz.y - 0.4985314*xyz.z;
    double g = -0.9692660*xyz.x + 1.8760108*xyz.y + 0.0415560*xyz.z;
    double b =  0.0556434*xyz.x - 0.2040259*xyz.y + 1.0572252*xyz.z;

    // Gamma校正 (sRGB)
    auto gammaEncode = [](double c) -> double {
        if (c <= 0.0031308) return 12.92 * c;
        return 1.055 * std::pow(std::max(0.0, c), 1.0/2.4) - 0.055;
    };

    return { gammaEncode(r), gammaEncode(g), gammaEncode(b) };
}

// ===== 场景几何 =====

struct HitRecord {
    double t;
    Vec3 point, normal;
    int materialID;
    Vec2 uv;
    bool frontFace;

    void setFaceNormal(const Ray& r, const Vec3& outNorm) {
        frontFace = r.dir.dot(outNorm) < 0;
        normal = frontFace ? outNorm : -outNorm;
    }
};

// 球体
struct Sphere {
    Vec3 center;
    double radius;
    int materialID;

    bool intersect(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.dir.dot(r.dir);
        double b = oc.dot(r.dir);
        double c = oc.dot(oc) - radius*radius;
        double disc = b*b - a*c;
        if (disc < 0) return false;

        double sqrtD = std::sqrt(disc);
        double t = (-b - sqrtD) / a;
        if (t < tMin || t > tMax) {
            t = (-b + sqrtD) / a;
            if (t < tMin || t > tMax) return false;
        }

        rec.t = t;
        rec.point = r.at(t);
        Vec3 outNorm = (rec.point - center) / radius;
        rec.setFaceNormal(r, outNorm);
        rec.materialID = materialID;

        // UV 坐标 (球面映射)
        Vec3 n = outNorm;
        double phi   = std::atan2(-n.z, n.x) + PI;
        double theta = std::acos(-n.y);
        rec.uv = {phi / (2*PI), theta / PI};

        return true;
    }
};

// 无限平面
struct Plane {
    Vec3 point, normal;
    int materialID;

    bool intersect(const Ray& r, double tMin, double tMax, HitRecord& rec) const {
        double denom = r.dir.dot(normal);
        if (std::abs(denom) < 1e-6) return false;
        double t = (point - r.origin).dot(normal) / denom;
        if (t < tMin || t > tMax) return false;

        rec.t = t;
        rec.point = r.at(t);
        rec.setFaceNormal(r, normal);
        rec.materialID = materialID;

        // UV: xz 坐标
        rec.uv = {rec.point.x, rec.point.z};

        return true;
    }
};

// ===== 材质定义 =====
// 材质类型
enum class MatType {
    SOAP_BUBBLE,      // 肥皂泡 - 很薄的薄膜, n=1.33 (水/肥皂)
    OIL_FILM,         // 油膜   - 中等厚度, n=1.45 (油脂)
    BEETLE_WING,      // 甲虫翅膀 - 较厚薄膜, n=1.5 (角质层)
    DIFFUSE,          // 漫反射
    SKY               // 天空背景
};

struct Material {
    MatType type;
    Vec3 baseColor;
    double filmN;      // 薄膜折射率
    double filmThickness; // 薄膜厚度 (nm, 基础值)
    double thicknessVariation; // 厚度变化幅度 (nm) - 用于厚度渐变效果
    double substN;     // 基底折射率
};

const Material MATERIALS[] = {
    // SOAP_BUBBLE: n1=1.33, 厚度 300-600nm 渐变
    {MatType::SOAP_BUBBLE, {0.9,0.9,0.9}, 1.33, 450.0, 200.0, 1.0},
    // OIL_FILM: n1=1.47, 厚度 200-800nm
    {MatType::OIL_FILM,    {0.8,0.75,0.7}, 1.47, 400.0, 400.0, 1.5},
    // BEETLE_WING: n1=1.5, 固定厚度约 250nm
    {MatType::BEETLE_WING, {0.2,0.6,0.2}, 1.50, 250.0, 100.0, 1.7},
    // DIFFUSE (地面)
    {MatType::DIFFUSE,     {0.15, 0.14, 0.13}, 1.0, 0.0, 0.0, 1.0},
    // SKY
    {MatType::SKY,         {0.5, 0.7, 1.0}, 1.0, 0.0, 0.0, 1.0},
};

// ===== 光源 =====
struct Light {
    Vec3 pos, color;
    double intensity;
};

// ===== 渲染器 =====

// 环境光/天空颜色
Vec3 skyColor(const Vec3& dir) {
    Vec3 d = dir.normalized();
    double t = 0.5 * (d.y + 1.0);
    Vec3 zenith  = {0.1, 0.2, 0.6};  // 天顶深蓝色
    Vec3 horizon = {0.6, 0.7, 0.9};  // 地平线淡蓝
    Vec3 ground  = {0.3, 0.25, 0.2}; // 地面暖棕
    if (t < 0.5) {
        // 地平线以下 -> 地面色
        double u = t * 2.0;
        return ground * (1.0-u) + horizon * u;
    } else {
        // 地平线以上 -> 天空蓝
        double u = (t - 0.5) * 2.0;
        return horizon * (1.0-u) + zenith * u;
    }
}

// 场景对象
std::vector<Sphere> spheres;
std::vector<Plane>  planes;
std::vector<Light>  lights;

// 场景求交
bool intersectScene(const Ray& r, double tMin, double tMax, HitRecord& rec) {
    bool hit = false;
    double closestT = tMax;
    HitRecord tmp;

    for (const auto& s : spheres) {
        if (s.intersect(r, tMin, closestT, tmp)) {
            hit = true;
            closestT = tmp.t;
            rec = tmp;
        }
    }
    for (const auto& p : planes) {
        if (p.intersect(r, tMin, closestT, tmp)) {
            hit = true;
            closestT = tmp.t;
            rec = tmp;
        }
    }
    return hit;
}

// 薄膜干涉颜色计算
// cosTheta: 观察方向与法线的夹角余弦
// mat: 材质
// uv: 表面 UV 坐标 (用于厚度变化)
Vec3 computeIridescentColor(double cosTheta, const Material& mat, const Vec2& uv) {
    double n0 = 1.0;         // 空气
    double n1 = mat.filmN;   // 薄膜
    double n2 = mat.substN;  // 基底

    // 厚度随位置变化 (模拟真实薄膜的厚度梯度)
    // 使用 UV 坐标生成厚度变化
    double thicknessNoise = 0.0;
    if (mat.type == MatType::SOAP_BUBBLE) {
        // 球形肥皂泡: 重力导致底部薄膜更厚
        // 使用 UV.y (球面纬度) 模拟
        double v = uv.y; // 0=顶部, 1=底部
        thicknessNoise = -v * mat.thicknessVariation;
    } else if (mat.type == MatType::OIL_FILM) {
        // 油膜: 不规则厚度变化, 用 UV 的 sin/cos 组合
        double u = uv.x, v = uv.y;
        thicknessNoise = mat.thicknessVariation * (
            0.3 * std::sin(u * 8.0) * std::cos(v * 6.0) +
            0.2 * std::sin(u * 15.0 + v * 10.0) +
            0.1 * std::cos(u * 20.0 - v * 18.0)
        );
    } else { // BEETLE_WING
        // 甲虫翅膀: 比较均匀的薄膜, 轻微变化
        double u = uv.x, v = uv.y;
        thicknessNoise = mat.thicknessVariation * 0.3 * std::sin(u*5.0)*std::sin(v*7.0);
    }

    double d = mat.filmThickness + thicknessNoise;
    d = std::max(50.0, std::min(1200.0, d)); // 限制在合理范围

    // 计算 XYZ 颜色
    Vec3 xyz = thinFilmToXYZ(n0, n1, n2, d, cosTheta);

    // 归一化 (Y通道白点)
    double ynorm = 0.0;
    for (int i = 0; i < 41; i++) ynorm += CIE_Y[i] * 10.0;
    if (ynorm > 0) {
        xyz.x /= ynorm;
        xyz.y /= ynorm;
        xyz.z /= ynorm;
    }

    // 转换为 RGB
    Vec3 rgb = xyzToRGB(xyz);
    rgb.x = std::max(0.0, std::min(1.0, rgb.x));
    rgb.y = std::max(0.0, std::min(1.0, rgb.y));
    rgb.z = std::max(0.0, std::min(1.0, rgb.z));

    return rgb;
}

// 阴影测试
bool inShadow(const Vec3& point, const Vec3& lightPos) {
    Vec3 toLight = lightPos - point;
    double dist = toLight.length();
    if (dist < 1e-4) return false;
    Vec3 toLightNorm = toLight / dist;
    Ray shadowRay(point + toLightNorm * 1e-3, toLightNorm);
    HitRecord tmp;
    return intersectScene(shadowRay, 1e-3, dist - 1e-3, tmp);
}

// 主着色函数
Vec3 shade(const Ray& ray, int depth) {
    if (depth <= 0) return {0,0,0};

    HitRecord rec;
    if (!intersectScene(ray, 1e-4, INF, rec)) {
        return skyColor(ray.dir);
    }

    const Material& mat = MATERIALS[rec.materialID];

    if (mat.type == MatType::DIFFUSE) {
        // 简单漫反射 + 环境光
        Vec3 color = {0,0,0};
        Vec3 ambient = mat.baseColor * 0.08;
        color += ambient;

        for (const auto& light : lights) {
            if (!inShadow(rec.point, light.pos)) {
                Vec3 L = (light.pos - rec.point).normalized();
                double diff = std::max(0.0, rec.normal.dot(L));
                Vec3 H = (L - ray.dir).normalized();
                double spec = std::pow(std::max(0.0, rec.normal.dot(H)), 64.0);
                double dist = (light.pos - rec.point).length();
                double atten = light.intensity / (1.0 + 0.001*dist*dist);

                color += mat.baseColor * diff * atten * light.color;
                color += Vec3{1.0,1.0,1.0} * spec * atten * 0.3 * light.color;
            }
        }
        return color;
    }

    // 薄膜材质 (SOAP_BUBBLE, OIL_FILM, BEETLE_WING)
    Vec3 N = rec.normal;
    Vec3 V = -ray.dir.normalized();
    double cosTheta = std::abs(V.dot(N));

    // 计算薄膜干涉颜色
    Vec3 iridescentColor = computeIridescentColor(cosTheta, mat, rec.uv);

    // 菲涅尔项 (用于混合透明度)
    // 基础菲涅尔 (Schlick 近似)
    double f0 = ((1.0 - mat.filmN) / (1.0 + mat.filmN));
    f0 = f0*f0;
    double fresnel = f0 + (1.0 - f0) * std::pow(1.0 - cosTheta, 5.0);

    Vec3 finalColor = {0,0,0};

    // 直接光照贡献 (强调彩虹色)
    for (const auto& light : lights) {
        if (!inShadow(rec.point, light.pos)) {
            Vec3 L = (light.pos - rec.point).normalized();
            double NdotL = std::max(0.0, N.dot(L));
            double dist  = (light.pos - rec.point).length();
            double atten = light.intensity / (1.0 + 0.0005*dist*dist);

            // 薄膜干涉颜色在灯光方向下的贡献
            Vec3 H = (V + L).normalized();
            double cosH = std::abs(V.dot(N)); // 使用V-N角
            Vec3 iridLight = computeIridescentColor(cosH, mat, rec.uv);

            finalColor += iridLight * NdotL * atten * light.color * 1.2;

            // 高光镜面反射 (白色)
            double specPow = std::pow(std::max(0.0, N.dot(H)), 128.0);
            finalColor += Vec3{1,1,1} * specPow * fresnel * atten * 0.5;
        }
    }

    // 环境/天空反射
    if (depth > 1 && mat.type == MatType::SOAP_BUBBLE) {
        Vec3 reflDir = reflect(ray.dir, N);
        Vec3 reflColor = shade(Ray(rec.point + N*1e-4, reflDir), depth - 1);

        // 肥皂泡: 反射和透射的混合
        double reflWeight = fresnel * 0.6;
        finalColor += reflColor * reflWeight;

        // 透射 (折射穿过)
        double transWeight = (1.0 - fresnel) * 0.25;
        Vec3 transDir = ray.dir; // 简化: 不计算折射角
        Vec3 transColor = shade(Ray(rec.point - N*1e-4, transDir), depth - 1);
        finalColor += transColor * transWeight;

        // 薄膜干涉颜色叠加在反射上
        finalColor += iridescentColor * 0.6;
    } else if (mat.type == MatType::OIL_FILM) {
        // 油膜: 主要是反射 + 薄膜色彩
        Vec3 reflDir = reflect(ray.dir, N);
        Vec3 reflColor = shade(Ray(rec.point + N*1e-4, reflDir), depth - 1);
        finalColor += reflColor * fresnel * 0.5;
        finalColor += iridescentColor * 1.2;
    } else { // BEETLE_WING
        // 甲虫翅膀: 主要是薄膜干涉颜色 + 少量漫反射
        finalColor += iridescentColor * 1.3;
        finalColor += mat.baseColor * 0.15;
    }

    // 环境光
    Vec3 skyAmb = skyColor(N) * 0.05;
    finalColor += skyAmb * iridescentColor;

    // HDR clamp
    finalColor.x = std::max(0.0, finalColor.x);
    finalColor.y = std::max(0.0, finalColor.y);
    finalColor.z = std::max(0.0, finalColor.z);

    return finalColor;
}

// Reinhard 色调映射
Vec3 tonemap(const Vec3& c) {
    double exposure = 0.9;
    Vec3 hdr = c * exposure;
    // Reinhard
    Vec3 mapped = {
        hdr.x / (1.0 + hdr.x),
        hdr.y / (1.0 + hdr.y),
        hdr.z / (1.0 + hdr.z)
    };
    return mapped;
}

// 设置场景
void setupScene() {
    // --- 球体 ---
    // 大肥皂泡 (中央)
    spheres.push_back({{0, 0.8, -4.0}, 1.2, 0}); // SOAP_BUBBLE
    // 中等肥皂泡 (左)
    spheres.push_back({{-2.5, 0.5, -5.0}, 0.8, 0}); // SOAP_BUBBLE
    // 小肥皂泡 (右上)
    spheres.push_back({{2.2, 1.2, -4.5}, 0.6, 0}); // SOAP_BUBBLE
    // 甲虫翅膀球 (右下)
    spheres.push_back({{1.8, -0.2, -3.5}, 0.5, 2}); // BEETLE_WING
    // 背景小球 (群)
    spheres.push_back({{-3.5, -0.3, -6.5}, 0.4, 0}); // SOAP_BUBBLE
    spheres.push_back({{ 3.2, -0.1, -7.0}, 0.5, 2}); // BEETLE_WING
    spheres.push_back({{-1.0,  2.5, -6.0}, 0.35, 0}); // SOAP_BUBBLE
    spheres.push_back({{ 0.5,  0.2, -3.0}, 0.3, 0}); // SOAP_BUBBLE (修改Y避免与地面交叉)

    // --- 平面 ---
    // 地面 - 油膜 (水面上的油)
    planes.push_back({{0, -1.0, 0}, {0, 1, 0}, 1}); // OIL_FILM

    // --- 光源 ---
    lights.push_back({{5, 8, 2},  {1.0, 0.95, 0.9}, 200.0});  // 主光
    lights.push_back({{-8, 5, -1}, {0.8, 0.85, 1.0}, 100.0}); // 补光
    lights.push_back({{0, 10, -10}, {1.0, 1.0, 0.95}, 80.0}); // 顶光
}

int main() {
    setupScene();

    const int SPP = 2; // samples per pixel (小, 保证速度)

    // 帧缓冲
    std::vector<uint8_t> image(WIDTH * HEIGHT * 3);

    // 相机设置
    double aspectRatio = (double)WIDTH / HEIGHT;
    Vec3 cameraPos  = {0, 1.0, 2.0};
    Vec3 lookAt     = {0, 0.2, -4.0};
    Vec3 up         = {0, 1, 0};
    double fov      = 50.0; // degrees
    double fovRad   = fov * PI / 180.0;
    double halfH    = std::tan(fovRad / 2.0);
    double halfW    = aspectRatio * halfH;

    Vec3 forward = (lookAt - cameraPos).normalized();
    Vec3 right   = forward.cross(up).normalized();
    Vec3 upDir   = right.cross(forward);

    // 子像素偏移 (简单超采样)
    const double jitter[2][2] = {{0.25, 0.25}, {0.75, 0.75}};

    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            Vec3 color = {0,0,0};
            for (int s = 0; s < SPP; s++) {
                double u = (x + jitter[s][0]) / (double)WIDTH;
                double v = (y + jitter[s][1]) / (double)HEIGHT;
                // NDC: 屏幕空间 [-1, 1]
                double nx = (2.0*u - 1.0) * halfW;
                double ny = (1.0 - 2.0*v) * halfH; // 翻转Y: 上为正

                Vec3 dir = (forward + right*nx + upDir*ny).normalized();
                Ray ray(cameraPos, dir);

                Vec3 c = shade(ray, 4);
                color += c;
            }
            color = color * (1.0 / SPP);

            // 色调映射
            color = tonemap(color);

            // Gamma 校正已在 xyzToRGB 里做了, 但非薄膜材质需要
            auto gammaSimple = [](double c) {
                return std::pow(std::max(0.0, std::min(1.0, c)), 1.0/2.2);
            };

            int idx = (y * WIDTH + x) * 3;
            image[idx+0] = (uint8_t)(std::min(1.0, color.x) * 255.0);
            image[idx+1] = (uint8_t)(std::min(1.0, color.y) * 255.0);
            image[idx+2] = (uint8_t)(std::min(1.0, color.z) * 255.0);

            (void)gammaSimple;
        }
    }

    // 保存图像
    stbi_write_png("thin_film_output.png", WIDTH, HEIGHT, 3,
                   image.data(), WIDTH * 3);

    printf("✅ 渲染完成: thin_film_output.png (%dx%d)\n", WIDTH, HEIGHT);
    printf("   场景: %zu 球体, %zu 平面, %zu 光源\n",
           spheres.size(), planes.size(), lights.size());

    return 0;
}
