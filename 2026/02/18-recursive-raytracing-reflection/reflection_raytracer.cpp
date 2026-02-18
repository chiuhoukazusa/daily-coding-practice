#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <limits>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// 三维向量
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); } // 逐分量乘法
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    
    Vec3 normalize() const {
        double len = length();
        return len > 1e-8 ? *this / len : Vec3(0, 0, 0);
    }
    
    // 反射向量：v - 2 * (v·n) * n
    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * (2.0 * this->dot(normal));
    }
};

// 光线
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    
    Vec3 at(double t) const { return origin + direction * t; }
};

// 材质
struct Material {
    Vec3 color;
    double diffuse;      // 漫反射系数 [0,1]
    double specular;     // 镜面反射系数 [0,1]
    double reflectivity; // 反射率 [0,1]
    
    Material(const Vec3& c = Vec3(1, 1, 1), double d = 0.8, double s = 0.2, double r = 0.0)
        : color(c), diffuse(d), specular(s), reflectivity(r) {}
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
        
        double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
        double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
        
        if (t1 > 1e-4) {
            t = t1;
            return true;
        }
        if (t2 > 1e-4) {
            t = t2;
            return true;
        }
        return false;
    }
};

// 点光源
struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;
    
    Light(const Vec3& p, const Vec3& c, double i)
        : position(p), color(c), intensity(i) {}
};

// 场景
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Vec3 ambient;
    
    Scene() : ambient(0.2, 0.2, 0.2) {}  // 增强环境光
    
    void addSphere(const Sphere& sphere) { spheres.push_back(sphere); }
    void addLight(const Light& light) { lights.push_back(light); }
};

// 递归光线追踪（支持反射）
Vec3 trace(const Ray& ray, const Scene& scene, int depth) {
    if (depth <= 0) {
        return Vec3(0, 0, 0); // 达到最大递归深度，返回黑色
    }
    
    // 找到最近的交点
    double closest_t = std::numeric_limits<double>::infinity();
    const Sphere* hit_sphere = nullptr;
    
    for (const auto& sphere : scene.spheres) {
        double t;
        if (sphere.intersect(ray, t) && t < closest_t) {
            closest_t = t;
            hit_sphere = &sphere;
        }
    }
    
    // 没有击中任何物体，返回背景色（天空渐变）
    if (!hit_sphere) {
        double t = 0.5 * (ray.direction.y + 1.0);
        Vec3 white(1.0, 1.0, 1.0);
        Vec3 blue(0.5, 0.7, 1.0);
        return white * (1.0 - t) + blue * t;
    }
    
    // 计算交点信息
    Vec3 hit_point = ray.at(closest_t);
    Vec3 normal = (hit_point - hit_sphere->center).normalize();
    Vec3 view_dir = (ray.origin - hit_point).normalize();
    
    // 环境光
    Vec3 color = scene.ambient * hit_sphere->material.color;
    
    // 遍历所有光源
    for (const auto& light : scene.lights) {
        Vec3 light_dir = (light.position - hit_point).normalize();
        double light_distance = (light.position - hit_point).length();
        
        // 阴影检测
        Ray shadow_ray(hit_point + normal * 1e-4, light_dir);
        bool in_shadow = false;
        
        for (const auto& sphere : scene.spheres) {
            double t;
            if (sphere.intersect(shadow_ray, t) && t < light_distance) {
                in_shadow = true;
                break;
            }
        }
        
        if (!in_shadow) {
            // 漫反射 (Lambert)
            double diffuse_intensity = std::max(0.0, normal.dot(light_dir));
            Vec3 diffuse = hit_sphere->material.color * light.color * diffuse_intensity 
                         * hit_sphere->material.diffuse * light.intensity;
            
            // 镜面反射 (Phong)
            Vec3 reflect_dir = (light_dir * -1.0).reflect(normal);
            double spec_intensity = std::pow(std::max(0.0, reflect_dir.dot(view_dir)), 32);
            Vec3 specular = light.color * spec_intensity 
                          * hit_sphere->material.specular * light.intensity;
            
            color = color + diffuse + specular;
        }
    }
    
    // 递归反射
    if (hit_sphere->material.reflectivity > 0.0) {
        Vec3 reflect_dir = (ray.direction * -1.0).reflect(normal);
        Ray reflect_ray(hit_point + normal * 1e-4, reflect_dir);
        Vec3 reflect_color = trace(reflect_ray, scene, depth - 1);
        color = color * (1.0 - hit_sphere->material.reflectivity) 
              + reflect_color * hit_sphere->material.reflectivity;
    }
    
    // Clamp 颜色值
    color.x = std::min(1.0, color.x);
    color.y = std::min(1.0, color.y);
    color.z = std::min(1.0, color.z);
    
    return color;
}

int main() {
    // 图像参数
    const int width = 800;
    const int height = 600;
    const int max_depth = 5; // 最大反射深度
    
    // 创建场景
    Scene scene;
    
    // 添加球体（不同反射率）
    // 中心大镜面球（纯镜面反射，完全镜子效果）
    scene.addSphere(Sphere(Vec3(0, 0, -5), 1.0, 
                           Material(Vec3(1.0, 1.0, 1.0), 0.0, 1.0, 1.0)));
    
    // 左侧红色球（中等反射率）
    scene.addSphere(Sphere(Vec3(-2.5, 0, -4), 0.8, 
                           Material(Vec3(1.0, 0.2, 0.2), 0.6, 0.4, 0.4)));
    
    // 右侧蓝色球（低反射率）
    scene.addSphere(Sphere(Vec3(2.5, 0, -4), 0.8, 
                           Material(Vec3(0.2, 0.2, 1.0), 0.8, 0.2, 0.2)));
    
    // 地面（绿色，无反射）
    scene.addSphere(Sphere(Vec3(0, -1001, -5), 1000, 
                           Material(Vec3(0.3, 0.8, 0.3), 0.9, 0.1, 0.0)));
    
    // 顶部小球（金色，高反射）
    scene.addSphere(Sphere(Vec3(0, 1.5, -4), 0.5, 
                           Material(Vec3(1.0, 0.84, 0.0), 0.3, 0.7, 0.6)));
    
    // 添加光源（增强亮度）
    scene.addLight(Light(Vec3(5, 5, -2), Vec3(1, 1, 1), 1.5));      // 主光源增强
    scene.addLight(Light(Vec3(-5, 3, -3), Vec3(0.9, 0.9, 1.0), 1.0)); // 副光源增强
    
    // 渲染
    std::vector<unsigned char> image(width * height * 3);
    
    std::cout << "Rendering " << width << "x" << height << " image..." << std::endl;
    std::cout << "Max reflection depth: " << max_depth << std::endl;
    
    for (int y = 0; y < height; y++) {
        if (y % 50 == 0) {
            std::cout << "Progress: " << (y * 100 / height) << "%" << std::endl;
        }
        
        for (int x = 0; x < width; x++) {
            // 将像素坐标映射到 [-1, 1] 范围
            double u = (2.0 * x / width - 1.0) * (double)width / height;
            double v = 1.0 - 2.0 * y / height;
            
            // 创建光线
            Vec3 ray_origin(0, 0, 0);
            Vec3 ray_direction(u, v, -1);
            Ray ray(ray_origin, ray_direction);
            
            // 追踪光线
            Vec3 color = trace(ray, scene, max_depth);
            
            // 写入像素（RGB）
            int idx = (y * width + x) * 3;
            image[idx + 0] = static_cast<unsigned char>(color.x * 255);
            image[idx + 1] = static_cast<unsigned char>(color.y * 255);
            image[idx + 2] = static_cast<unsigned char>(color.z * 255);
        }
    }
    
    // 保存图像
    std::string filename = "reflection_output.png";
    if (stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3)) {
        std::cout << "✅ Image saved: " << filename << std::endl;
    } else {
        std::cerr << "❌ Failed to save image" << std::endl;
        return 1;
    }
    
    std::cout << "\n🎉 Ray tracing completed!" << std::endl;
    std::cout << "Scene: 5 spheres (reflective materials)" << std::endl;
    std::cout << "Lights: 2 point lights" << std::endl;
    std::cout << "Features: Shadows + Phong lighting + Recursive reflections" << std::endl;
    
    return 0;
}
