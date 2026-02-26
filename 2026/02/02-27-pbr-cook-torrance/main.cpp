/**
 * PBR Cook-Torrance BRDF Renderer
 * 
 * 实现基于物理的渲染（Physically Based Rendering）
 * 使用 Cook-Torrance BRDF 模型
 * 
 * 主要组件：
 * - D: 法线分布函数（GGX/Trowbridge-Reitz）
 * - G: 几何遮蔽函数（Smith's method + Schlick-GGX）
 * - F: Fresnel 方程（Schlick 近似）
 * 
 * 渲染一个球体阵列，展示不同金属度和粗糙度的 PBR 材质
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <algorithm>
#include <vector>
#include <iostream>
#include <string>
#include <limits>

// 常量
const double PI = 3.14159265358979323846;
const double EPSILON = 1e-6;

// ============================================================
// 向量数学
// ============================================================
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(double t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const {
        double len = length();
        if (len < EPSILON) return {0,0,0};
        return *this / len;
    }
    double maxComponent() const { return std::max({x, y, z}); }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ============================================================
// 光线与场景
// ============================================================
struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
};

struct Hit {
    double t;
    Vec3 position;
    Vec3 normal;
    bool valid;
    Hit() : t(1e18), valid(false) {}
};

// ============================================================
// PBR 材质
// ============================================================
struct PBRMaterial {
    Vec3 albedo;       // 基础颜色
    double metallic;   // 金属度 [0,1]
    double roughness;  // 粗糙度 [0,1]
    
    PBRMaterial(Vec3 a, double m, double r) : albedo(a), metallic(m), roughness(r) {}
};

// ============================================================
// Cook-Torrance BRDF 组件
// ============================================================

// GGX 法线分布函数 (NDF)
// D(h) = alpha^2 / (pi * (n·h)^2 * (alpha^2 - 1) + 1)^2
double distributionGGX(double NdotH, double roughness) {
    double a = roughness * roughness;
    double a2 = a * a;
    double NdotH2 = NdotH * NdotH;
    double denom = NdotH2 * (a2 - 1.0) + 1.0;
    denom = PI * denom * denom;
    return a2 / std::max(denom, EPSILON);
}

// Schlick-GGX 几何函数（单边）
double geometrySchlickGGX(double NdotV, double roughness) {
    double r = roughness + 1.0;
    double k = (r * r) / 8.0;
    double denom = NdotV * (1.0 - k) + k;
    return NdotV / std::max(denom, EPSILON);
}

// Smith's 方法（双向几何遮蔽）
double geometrySmith(double NdotV, double NdotL, double roughness) {
    double ggx1 = geometrySchlickGGX(NdotV, roughness);
    double ggx2 = geometrySchlickGGX(NdotL, roughness);
    return ggx1 * ggx2;
}

// Fresnel-Schlick 近似
Vec3 fresnelSchlick(double cosTheta, const Vec3& F0) {
    double f = std::pow(std::max(1.0 - cosTheta, 0.0), 5.0);
    return F0 + (Vec3(1,1,1) - F0) * f;
}

// ============================================================
// Cook-Torrance BRDF 主函数
// ============================================================
Vec3 cookTorranceBRDF(
    const Vec3& N,       // 法线
    const Vec3& V,       // 视线方向（指向相机）
    const Vec3& L,       // 光源方向
    const PBRMaterial& mat
) {
    Vec3 H = (V + L).normalize();  // 半程向量
    
    double NdotV = std::max(N.dot(V), 0.0);
    double NdotL = std::max(N.dot(L), 0.0);
    double NdotH = std::max(N.dot(H), 0.0);
    double HdotV = std::max(H.dot(V), 0.0);
    
    // F0: 基础反射率
    // 非金属默认 0.04，金属使用 albedo 颜色
    Vec3 F0(0.04, 0.04, 0.04);
    F0 = F0 * (1.0 - mat.metallic) + mat.albedo * mat.metallic;
    
    // 计算 Cook-Torrance 镜面项
    double D = distributionGGX(NdotH, mat.roughness);
    double G = geometrySmith(NdotV, NdotL, mat.roughness);
    Vec3 F = fresnelSchlick(HdotV, F0);
    
    Vec3 numerator = F * D * G;
    double denominator = 4.0 * NdotV * NdotL + EPSILON;
    Vec3 specular = numerator / denominator;
    
    // kS = Fresnel（镜面反射比例）
    // kD = 1 - kS（漫反射比例），金属没有漫反射
    Vec3 kS = F;
    Vec3 kD = Vec3(1,1,1) - kS;
    kD = kD * (1.0 - mat.metallic);
    
    // Lambert 漫反射
    Vec3 diffuse = kD * mat.albedo / PI;
    
    return (diffuse + specular) * NdotL;
}

// ============================================================
// 场景：球体
// ============================================================
struct Sphere {
    Vec3 center;
    double radius;
    PBRMaterial material;
    
    Sphere(Vec3 c, double r, PBRMaterial m) : center(c), radius(r), material(m) {}
    
    Hit intersect(const Ray& ray) const {
        Hit hit;
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double disc = b * b - 4 * a * c;
        
        if (disc < 0) return hit;
        
        double t = (-b - std::sqrt(disc)) / (2.0 * a);
        if (t < 0.001) {
            t = (-b + std::sqrt(disc)) / (2.0 * a);
        }
        if (t < 0.001) return hit;
        
        hit.t = t;
        hit.valid = true;
        hit.position = ray.origin + ray.direction * t;
        hit.normal = (hit.position - center).normalize();
        return hit;
    }
};

// ============================================================
// 光源
// ============================================================
struct PointLight {
    Vec3 position;
    Vec3 color;
    double intensity;
};

// ============================================================
// 色调映射（HDR -> LDR）
// ============================================================
Vec3 ACESFilm(Vec3 x) {
    // ACES 近似色调映射曲线
    double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    Vec3 result;
    result.x = (x.x * (a*x.x + b)) / (x.x * (c*x.x + d) + e);
    result.y = (x.y * (a*x.y + b)) / (x.y * (c*x.y + d) + e);
    result.z = (x.z * (a*x.z + b)) / (x.z * (c*x.z + d) + e);
    result.x = std::clamp(result.x, 0.0, 1.0);
    result.y = std::clamp(result.y, 0.0, 1.0);
    result.z = std::clamp(result.z, 0.0, 1.0);
    return result;
}

// Gamma 校正（sRGB）
Vec3 gammaCorrect(Vec3 color) {
    return {
        std::pow(color.x, 1.0/2.2),
        std::pow(color.y, 1.0/2.2),
        std::pow(color.z, 1.0/2.2)
    };
}

// ============================================================
// 主渲染函数
// ============================================================
int main() {
    // 图像尺寸
    const int WIDTH = 800;
    const int HEIGHT = 600;
    std::vector<uint8_t> image(WIDTH * HEIGHT * 3);
    
    // 相机设置
    Vec3 cameraPos(0, 0, 8);
    Vec3 cameraTarget(0, 0, 0);
    Vec3 cameraUp(0, 1, 0);
    
    double fov = 45.0 * PI / 180.0;
    double aspect = (double)WIDTH / HEIGHT;
    double halfH = std::tan(fov / 2.0);
    double halfW = aspect * halfH;
    
    // 相机坐标系
    Vec3 camZ = (cameraPos - cameraTarget).normalize();
    Vec3 camX = cameraUp.cross(camZ).normalize();
    Vec3 camY = camZ.cross(camX);
    
    // 构建 5x4 球体阵列（金属度 x 粗糙度）
    std::vector<Sphere> spheres;
    
    int gridCols = 5;  // 金属度变化（0.0 -> 1.0）
    int gridRows = 4;  // 粗糙度变化（0.0 -> 1.0）
    
    double spacing = 1.8;
    double startX = -(gridCols - 1) * spacing / 2.0;
    double startY = -(gridRows - 1) * spacing / 2.0;
    
    // 主球体颜色：金黄色
    Vec3 goldAlbedo(1.0, 0.71, 0.29);
    
    for (int row = 0; row < gridRows; row++) {
        double roughness = std::clamp((double)row / (gridRows - 1), 0.05, 1.0);
        for (int col = 0; col < gridCols; col++) {
            double metallic = (double)col / (gridCols - 1);
            
            Vec3 pos(
                startX + col * spacing,
                startY + (gridRows - 1 - row) * spacing,  // 行从下到上粗糙度增加
                0
            );
            
            spheres.emplace_back(pos, 0.7, PBRMaterial(goldAlbedo, metallic, roughness));
        }
    }
    
    // 光源设置（4个点光源）
    std::vector<PointLight> lights = {
        { Vec3(-4,  4, 5), Vec3(1.0, 0.95, 0.9), 30.0 },
        { Vec3( 4,  4, 5), Vec3(0.9, 0.95, 1.0), 30.0 },
        { Vec3(-4, -4, 5), Vec3(1.0, 0.9,  0.9), 20.0 },
        { Vec3( 4, -4, 5), Vec3(0.9, 1.0,  0.9), 20.0 }
    };
    
    // 环境光
    Vec3 ambient(0.03, 0.03, 0.04);
    
    std::cout << "渲染 PBR 材质球阵列..." << std::endl;
    std::cout << "尺寸: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "球体数量: " << spheres.size() << std::endl;
    
    // 渲染循环
    for (int py = 0; py < HEIGHT; py++) {
        if (py % 100 == 0) {
            std::cout << "进度: " << (py * 100 / HEIGHT) << "%" << std::endl;
        }
        
        for (int px = 0; px < WIDTH; px++) {
            // NDC 坐标
            double u = (2.0 * px / WIDTH - 1.0);
            double v = (1.0 - 2.0 * py / HEIGHT);
            
            // 光线方向
            Vec3 rayDir = (camX * (u * halfW) + camY * (v * halfH) - camZ).normalize();
            Ray ray(cameraPos, rayDir);
            
            // 找最近交叉点
            Hit closest;
            const Sphere* hitSphere = nullptr;
            
            for (const auto& sphere : spheres) {
                Hit h = sphere.intersect(ray);
                if (h.valid && h.t < closest.t) {
                    closest = h;
                    hitSphere = &sphere;
                }
            }
            
            Vec3 finalColor(0, 0, 0);
            
            if (hitSphere) {
                Vec3 N = closest.normal;
                Vec3 V = (cameraPos - closest.position).normalize();
                const PBRMaterial& mat = hitSphere->material;
                
                // 环境光贡献
                finalColor = ambient * mat.albedo;
                
                // 每个光源的 PBR 贡献
                for (const auto& light : lights) {
                    Vec3 L = (light.position - closest.position).normalize();
                    double dist2 = (light.position - closest.position).dot(
                                   light.position - closest.position);
                    
                    // 简单阴影检测
                    Ray shadowRay(closest.position + N * 0.001, L);
                    bool inShadow = false;
                    for (const auto& sphere : spheres) {
                        Hit sh = sphere.intersect(shadowRay);
                        if (sh.valid && sh.t < std::sqrt(dist2)) {
                            inShadow = true;
                            break;
                        }
                    }
                    if (inShadow) continue;
                    
                    // 辐射度（距离衰减）
                    Vec3 radiance = light.color * (light.intensity / dist2);
                    
                    // Cook-Torrance BRDF
                    Vec3 brdf = cookTorranceBRDF(N, V, L, mat);
                    finalColor += brdf * radiance;
                }
            } else {
                // 背景：深灰色渐变
                double t = 0.5 * (rayDir.y + 1.0);
                finalColor = Vec3(0.08, 0.08, 0.12) * (1.0 - t) + Vec3(0.05, 0.05, 0.08) * t;
            }
            
            // HDR 色调映射
            finalColor = ACESFilm(finalColor);
            
            // Gamma 校正
            finalColor = gammaCorrect(finalColor);
            
            // 写入像素
            int idx = (py * WIDTH + px) * 3;
            image[idx + 0] = (uint8_t)(std::clamp(finalColor.x * 255.0, 0.0, 255.0));
            image[idx + 1] = (uint8_t)(std::clamp(finalColor.y * 255.0, 0.0, 255.0));
            image[idx + 2] = (uint8_t)(std::clamp(finalColor.z * 255.0, 0.0, 255.0));
        }
    }
    
    // 保存图片
    int result = stbi_write_png("pbr_output.png", WIDTH, HEIGHT, 3, image.data(), WIDTH * 3);
    if (result) {
        std::cout << "✅ 图片已保存: pbr_output.png" << std::endl;
    } else {
        std::cerr << "❌ 图片保存失败" << std::endl;
        return 1;
    }
    
    std::cout << "渲染完成！" << std::endl;
    return 0;
}
