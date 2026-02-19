// 递归光线追踪 - 折射效果（玻璃球）
// 支持反射、折射、菲涅尔效应
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

// ========== 向量类 ==========
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    
    Vec3 normalize() const {
        double len = length();
        return len > 0 ? Vec3(x / len, y / len, z / len) : Vec3(0, 0, 0);
    }
    
    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * 2.0 * this->dot(normal);
    }
    
    // 折射计算（Snell定律）
    // n1/n2: 折射率比
    // 返回折射方向，如果发生全反射则返回 (0,0,0)
    Vec3 refract(const Vec3& normal, double eta) const {
        double cos_i = -this->dot(normal);
        double sin2_t = eta * eta * (1.0 - cos_i * cos_i);
        
        // 全反射
        if (sin2_t > 1.0) {
            return Vec3(0, 0, 0);
        }
        
        double cos_t = std::sqrt(1.0 - sin2_t);
        return (*this * eta) + normal * (eta * cos_i - cos_t);
    }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ========== 光线类 ==========
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    
    Vec3 at(double t) const { return origin + direction * t; }
};

// ========== 材质类型 ==========
enum MaterialType {
    DIFFUSE,
    METAL,
    GLASS
};

// ========== 球体类 ==========
struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    MaterialType material;
    double roughness;  // 金属：粗糙度；玻璃：折射率
    
    Sphere(const Vec3& c, double r, const Vec3& col, MaterialType mat = DIFFUSE, double rough = 0.0)
        : center(c), radius(r), color(col), material(mat), roughness(rough) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        double sqrt_disc = std::sqrt(discriminant);
        double t1 = (-b - sqrt_disc) / (2.0 * a);
        double t2 = (-b + sqrt_disc) / (2.0 * a);
        
        // 选择最近的正交点
        if (t1 > 0.001) {
            t = t1;
            return true;
        } else if (t2 > 0.001) {
            t = t2;
            return true;
        }
        return false;
    }
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
};

// ========== 光源类 ==========
struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;
    
    Light(const Vec3& pos, const Vec3& col, double inten = 1.0)
        : position(pos), color(col), intensity(inten) {}
};

// ========== 场景类 ==========
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Light> lights;
    Vec3 backgroundColor;
    
    Scene() : backgroundColor(0.1, 0.1, 0.15) {}  // 深蓝色背景
    
    void addSphere(const Sphere& sphere) { spheres.push_back(sphere); }
    void addLight(const Light& light) { lights.push_back(light); }
    
    bool intersect(const Ray& ray, Sphere*& hitSphere, double& t) const {
        double closest = std::numeric_limits<double>::max();
        hitSphere = nullptr;
        
        for (size_t i = 0; i < spheres.size(); ++i) {
            double temp_t;
            if (spheres[i].intersect(ray, temp_t) && temp_t < closest) {
                closest = temp_t;
                hitSphere = const_cast<Sphere*>(&spheres[i]);
                t = temp_t;
            }
        }
        
        return hitSphere != nullptr;
    }
    
    bool isInShadow(const Vec3& point, const Vec3& lightPos) const {
        Vec3 dir = (lightPos - point).normalize();
        double dist = (lightPos - point).length();
        Ray shadowRay(point, dir);
        
        Sphere* hitSphere;
        double t;
        if (intersect(shadowRay, hitSphere, t)) {
            return t < dist;
        }
        return false;
    }
};

// ========== Fresnel 计算（Schlick近似）==========
double fresnel(double cos_theta, double ior) {
    double r0 = (1.0 - ior) / (1.0 + ior);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - cos_theta, 5.0);
}

// ========== 递归光线追踪 ==========
Vec3 trace(const Ray& ray, const Scene& scene, int depth) {
    // 递归深度限制
    if (depth <= 0) {
        return scene.backgroundColor;
    }
    
    Sphere* hitSphere;
    double t;
    if (!scene.intersect(ray, hitSphere, t)) {
        return scene.backgroundColor;
    }
    
    Vec3 hitPoint = ray.at(t);
    Vec3 normal = hitSphere->getNormal(hitPoint);
    
    // 根据材质类型处理
    if (hitSphere->material == DIFFUSE) {
        // 漫反射：Phong 光照模型
        Vec3 color(0, 0, 0);
        Vec3 ambient = hitSphere->color * 0.1;
        
        for (const Light& light : scene.lights) {
            if (!scene.isInShadow(hitPoint, light.position)) {
                Vec3 lightDir = (light.position - hitPoint).normalize();
                double diff = std::max(0.0, normal.dot(lightDir));
                
                Vec3 viewDir = (ray.origin - hitPoint).normalize();
                Vec3 reflectDir = (lightDir * -1.0).reflect(normal);
                double spec = std::pow(std::max(0.0, viewDir.dot(reflectDir)), 32);
                
                Vec3 diffuse = hitSphere->color * diff * light.intensity;
                Vec3 specular = light.color * spec * 0.5 * light.intensity;
                
                color = color + diffuse + specular;
            }
        }
        
        return ambient + color;
    }
    else if (hitSphere->material == METAL) {
        // 镜面反射（金属）
        Vec3 reflectDir = ray.direction.reflect(normal);
        Ray reflectRay(hitPoint, reflectDir);
        Vec3 reflectColor = trace(reflectRay, scene, depth - 1);
        
        // ✅ 修复：金属反射必须乘上金属本身的颜色（金黄色）
        // 否则就是无色镜子，看起来像透明材质
        Vec3 metalColor = hitSphere->color;
        return Vec3(
            reflectColor.x * metalColor.x,
            reflectColor.y * metalColor.y,
            reflectColor.z * metalColor.z
        ) * 0.9;  // 90% 能量保留
    }
    else if (hitSphere->material == GLASS) {
        // 玻璃材质：反射 + 折射
        double ior = hitSphere->roughness;  // 折射率（例如玻璃1.5）
        
        // 判断光线是从外部进入还是内部射出
        bool entering = ray.direction.dot(normal) < 0;
        Vec3 n = entering ? normal : normal * -1.0;
        double eta = entering ? (1.0 / ior) : ior;
        
        // 计算 Fresnel 系数
        double cos_theta = std::abs(ray.direction.dot(n));
        double F = fresnel(cos_theta, ior);
        
        // 折射
        Vec3 refractDir = ray.direction.refract(n, eta);
        
        // 如果发生全反射，只计算反射
        if (refractDir.length() < 0.001) {
            Vec3 reflectDir = ray.direction.reflect(n);
            Ray reflectRay(hitPoint, reflectDir);
            return trace(reflectRay, scene, depth - 1);
        }
        
        // 反射
        Vec3 reflectDir = ray.direction.reflect(n);
        Ray reflectRay(hitPoint, reflectDir);
        Vec3 reflectColor = trace(reflectRay, scene, depth - 1);
        
        // 折射
        Ray refractRay(hitPoint, refractDir);
        Vec3 refractColor = trace(refractRay, scene, depth - 1);
        
        // 根据 Fresnel 混合
        return reflectColor * F + refractColor * (1.0 - F);
    }
    
    return scene.backgroundColor;
}

// ========== 主函数 ==========
int main() {
    const int width = 800;
    const int height = 600;
    const int channels = 3;
    
    std::vector<unsigned char> image(width * height * channels);
    
    // 创建场景
    Scene scene;
    
    // 添加球体：左（漫反射绿球）、中（玻璃球）、右（金属球）
    // 球心间距改为 4.0，避免重叠（半径1.5，间距至少要 > 3.0）
    scene.addSphere(Sphere(Vec3(-4.0, 0, -10), 1.5, Vec3(0.2, 0.8, 0.2), DIFFUSE));
    scene.addSphere(Sphere(Vec3(0, 0, -10), 1.5, Vec3(1.0, 1.0, 1.0), GLASS, 1.5));  // 折射率1.5
    scene.addSphere(Sphere(Vec3(4.0, 0, -10), 1.5, Vec3(0.8, 0.6, 0.2), METAL));
    
    // 地板（大球）
    scene.addSphere(Sphere(Vec3(0, -101.5, -10), 100, Vec3(0.5, 0.5, 0.5), DIFFUSE));
    
    // 添加光源
    scene.addLight(Light(Vec3(5, 5, -5), Vec3(1.0, 1.0, 1.0), 1.2));
    scene.addLight(Light(Vec3(-5, 3, -3), Vec3(0.8, 0.8, 1.0), 0.8));
    
    // 相机参数
    Vec3 cameraPos(0, 1, 0);
    double fov = 60.0 * M_PI / 180.0;
    double aspectRatio = double(width) / double(height);
    
    // 渲染
    std::cout << "开始渲染 " << width << "x" << height << " ..." << std::endl;
    
    for (int y = 0; y < height; ++y) {
        if (y % 50 == 0) {
            std::cout << "进度: " << (100 * y / height) << "%" << std::endl;
        }
        
        for (int x = 0; x < width; ++x) {
            // NDC 坐标
            double px = (2.0 * (x + 0.5) / width - 1.0) * aspectRatio * std::tan(fov / 2.0);
            double py = (1.0 - 2.0 * (y + 0.5) / height) * std::tan(fov / 2.0);
            
            Vec3 rayDir(px, py, -1.0);
            Ray ray(cameraPos, rayDir);
            
            Vec3 color = trace(ray, scene, 5);  // 最大递归深度5
            
            // Gamma 校正
            color.x = std::pow(std::clamp(color.x, 0.0, 1.0), 1.0 / 2.2);
            color.y = std::pow(std::clamp(color.y, 0.0, 1.0), 1.0 / 2.2);
            color.z = std::pow(std::clamp(color.z, 0.0, 1.0), 1.0 / 2.2);
            
            int idx = (y * width + x) * channels;
            image[idx + 0] = static_cast<unsigned char>(color.x * 255);
            image[idx + 1] = static_cast<unsigned char>(color.y * 255);
            image[idx + 2] = static_cast<unsigned char>(color.z * 255);
        }
    }
    
    // 保存图片
    stbi_write_png("refraction_output.png", width, height, channels, image.data(), width * channels);
    std::cout << "✅ 渲染完成！输出: refraction_output.png" << std::endl;
    
    return 0;
}
