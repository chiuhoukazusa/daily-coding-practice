// Shadow Ray Tracing - 带阴影的光线追踪器
// 日期: 2026-02-17
// 技术: 光线追踪 + Shadow Ray + Phong光照模型

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <limits>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

const double EPSILON = 1e-6;
const double INF = std::numeric_limits<double>::infinity();

// 3D 向量类
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { double len = length(); return len > 0 ? *this / len : *this; }
};

// 光线
struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
};

// 材质
struct Material {
    Vec3 color;           // 基础颜色
    double ambient;       // 环境光系数
    double diffuse;       // 漫反射系数
    double specular;      // 镜面反射系数
    double shininess;     // 光泽度
    
    Material(const Vec3& c, double ka = 0.1, double kd = 0.6, double ks = 0.3, double sh = 32)
        : color(c), ambient(ka), diffuse(kd), specular(ks), shininess(sh) {}
};

// 球体
struct Sphere {
    Vec3 center;
    double radius;
    Material material;
    
    Sphere(const Vec3& c, double r, const Material& m) 
        : center(c), radius(r), material(m) {}
    
    // 光线与球体求交
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        double sqrt_d = std::sqrt(discriminant);
        double t1 = (-b - sqrt_d) / (2.0 * a);
        double t2 = (-b + sqrt_d) / (2.0 * a);
        
        // 取最近的正交点
        if (t1 > EPSILON) {
            t = t1;
            return true;
        } else if (t2 > EPSILON) {
            t = t2;
            return true;
        }
        return false;
    }
    
    // 获取球面法线
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
};

// 点光源
struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;
    
    Light(const Vec3& pos, const Vec3& col, double intens = 1.0)
        : position(pos), color(col), intensity(intens) {}
};

// 场景
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Vec3 backgroundColor;
    
    Scene() : backgroundColor(0.1, 0.1, 0.2) {}  // 深蓝色背景
    
    // 查找最近的交点
    bool findNearestIntersection(const Ray& ray, double& nearestT, const Sphere*& hitSphere) const {
        nearestT = INF;
        hitSphere = nullptr;
        
        for (const auto& sphere : spheres) {
            double t;
            if (sphere.intersect(ray, t) && t < nearestT) {
                nearestT = t;
                hitSphere = &sphere;
            }
        }
        
        return hitSphere != nullptr;
    }
    
    // 检查阴影：从点到光源的路径上是否有遮挡物
    bool isInShadow(const Vec3& point, const Vec3& lightPos) const {
        Vec3 toLight = lightPos - point;
        double distanceToLight = toLight.length();
        Ray shadowRay(point, toLight);
        
        // 检查是否有物体遮挡
        for (const auto& sphere : spheres) {
            double t;
            if (sphere.intersect(shadowRay, t)) {
                // 如果交点在光源之前，说明被遮挡
                if (t > EPSILON && t < distanceToLight) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    // Phong 光照模型 + 阴影
    Vec3 computePhongLighting(const Vec3& point, const Vec3& normal, const Vec3& viewDir, const Material& material) const {
        Vec3 color(0, 0, 0);
        
        // 环境光（不受阴影影响）
        Vec3 ambient = material.color * material.ambient;
        color = color + ambient;
        
        // 遍历所有光源
        for (const auto& light : lights) {
            // 检查阴影
            if (isInShadow(point, light.position)) {
                continue;  // 在阴影中，跳过这个光源的漫反射和镜面反射
            }
            
            Vec3 lightDir = (light.position - point).normalize();
            
            // 漫反射 (Diffuse)
            double diffuseIntensity = std::max(0.0, normal.dot(lightDir));
            Vec3 diffuse = material.color * (material.diffuse * diffuseIntensity * light.intensity);
            
            // 镜面反射 (Specular) - Phong模型
            Vec3 reflectDir = (normal * (2.0 * normal.dot(lightDir)) - lightDir).normalize();
            double specularIntensity = std::pow(std::max(0.0, viewDir.dot(reflectDir)), material.shininess);
            Vec3 specular = light.color * (material.specular * specularIntensity * light.intensity);
            
            color = color + diffuse + specular;
        }
        
        return color;
    }
    
    // 追踪光线
    Vec3 traceRay(const Ray& ray, int depth = 0) const {
        if (depth > 3) return backgroundColor;  // 递归深度限制
        
        double t;
        const Sphere* hitSphere;
        
        if (!findNearestIntersection(ray, t, hitSphere)) {
            return backgroundColor;  // 未击中任何物体
        }
        
        // 计算交点和法线
        Vec3 hitPoint = ray.origin + ray.direction * t;
        Vec3 normal = hitSphere->getNormal(hitPoint);
        Vec3 viewDir = (ray.origin - hitPoint).normalize();
        
        // 计算 Phong 光照 + 阴影
        Vec3 color = computePhongLighting(hitPoint, normal, viewDir, hitSphere->material);
        
        return color;
    }
};

// 渲染器
class Renderer {
private:
    int width, height;
    std::vector<unsigned char> pixels;
    
public:
    Renderer(int w, int h) : width(w), height(h) {
        pixels.resize(width * height * 3);
    }
    
    void render(const Scene& scene, const Vec3& cameraPos, double fov) {
        double aspectRatio = static_cast<double>(width) / height;
        double scale = std::tan(fov * 0.5 * M_PI / 180.0);
        
        std::cout << "开始渲染 " << width << "x" << height << " 图像..." << std::endl;
        
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                // 计算射线方向
                double px = (2.0 * (x + 0.5) / width - 1.0) * aspectRatio * scale;
                double py = (1.0 - 2.0 * (y + 0.5) / height) * scale;
                
                Vec3 rayDir(px, py, -1);
                Ray ray(cameraPos, rayDir);
                
                // 追踪光线
                Vec3 color = scene.traceRay(ray);
                
                // Gamma校正和裁剪
                color.x = std::pow(std::min(1.0, color.x), 1.0 / 2.2);
                color.y = std::pow(std::min(1.0, color.y), 1.0 / 2.2);
                color.z = std::pow(std::min(1.0, color.z), 1.0 / 2.2);
                
                // 写入像素
                int index = (y * width + x) * 3;
                pixels[index + 0] = static_cast<unsigned char>(color.x * 255);
                pixels[index + 1] = static_cast<unsigned char>(color.y * 255);
                pixels[index + 2] = static_cast<unsigned char>(color.z * 255);
            }
            
            if (y % 50 == 0) {
                std::cout << "进度: " << (100 * y / height) << "%" << std::endl;
            }
        }
        
        std::cout << "渲染完成！" << std::endl;
    }
    
    void savePNG(const char* filename) {
        stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
        std::cout << "已保存: " << filename << std::endl;
    }
};

int main() {
    // 创建场景
    Scene scene;
    
    // 添加球体
    // 中心大球（红色，高光泽）
    scene.spheres.push_back(Sphere(Vec3(0, 0, -5), 1.0, 
        Material(Vec3(1.0, 0.2, 0.2), 0.1, 0.7, 0.5, 64)));
    
    // 左侧小球（绿色，低光泽）
    scene.spheres.push_back(Sphere(Vec3(-2.5, -0.5, -4), 0.6, 
        Material(Vec3(0.2, 1.0, 0.2), 0.1, 0.8, 0.2, 16)));
    
    // 右侧小球（蓝色，中等光泽）
    scene.spheres.push_back(Sphere(Vec3(2.0, 0, -4.5), 0.7, 
        Material(Vec3(0.2, 0.5, 1.0), 0.1, 0.7, 0.4, 32)));
    
    // 地面球（灰白色，大球模拟平面）
    scene.spheres.push_back(Sphere(Vec3(0, -101, -5), 100.0, 
        Material(Vec3(0.8, 0.8, 0.8), 0.1, 0.6, 0.1, 8)));
    
    // 添加光源
    // 主光源（右上方，白色强光）
    scene.lights.push_back(Light(Vec3(5, 5, -2), Vec3(1, 1, 1), 1.5));
    
    // 辅助光源（左侧，橙色弱光）
    scene.lights.push_back(Light(Vec3(-3, 3, 0), Vec3(1.0, 0.7, 0.3), 0.5));
    
    // 创建渲染器
    int width = 800;
    int height = 600;
    Renderer renderer(width, height);
    
    // 相机设置
    Vec3 cameraPos(0, 1, 2);
    double fov = 60.0;
    
    // 渲染场景
    renderer.render(scene, cameraPos, fov);
    
    // 保存图像
    renderer.savePNG("shadow_output.png");
    
    return 0;
}
