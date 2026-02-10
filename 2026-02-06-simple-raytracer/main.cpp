// Simple Ray Tracer - 2026-02-06
// 每日编程实践 #1: 基础光线追踪器

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

// 简单向量类
struct Vec3 {
    float x, y, z;
    
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
    
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { 
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); 
    }
    
    float length() const { return sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { return *this / length(); }
};

// 光线类
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    
    Vec3 pointAt(float t) const { return origin + direction * t; }
};

// 球体类
struct Sphere {
    Vec3 center;
    float radius;
    Vec3 color;
    
    Sphere(const Vec3& c, float r, const Vec3& col) : center(c), radius(r), color(col) {}
    
    bool intersect(const Ray& ray, float& t) const {
        Vec3 oc = ray.origin - center;
        float a = ray.direction.dot(ray.direction);
        float b = 2.0f * oc.dot(ray.direction);
        float c = oc.dot(oc) - radius * radius;
        float discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        float sqrtD = sqrt(discriminant);
        float t0 = (-b - sqrtD) / (2.0f * a);
        float t1 = (-b + sqrtD) / (2.0f * a);
        
        // 选择最小的正根
        if (t0 > 0.001f) {
            t = t0;
            return true;
        } else if (t1 > 0.001f) {
            t = t1;
            return true;
        }
        return false;
    }
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
};

// 简单相机类
struct Camera {
    Vec3 position;
    Vec3 lookAt;
    Vec3 up;
    float fov;
    
    Camera(const Vec3& pos, const Vec3& target, const Vec3& upVec, float fovDegrees)
        : position(pos), lookAt(target), up(upVec), fov(fovDegrees * M_PI / 180.0f) {}
    
    Ray getRay(float u, float v, int width, int height) const {
        Vec3 forward = (lookAt - position).normalize();
        Vec3 right = forward.cross(up).normalize();
        Vec3 upVec = right.cross(forward);
        
        float aspect = (float)width / height;
        float scale = tan(fov / 2.0f);
        
        Vec3 dir = forward + right * (u * aspect * scale) + upVec * (v * scale);
        return Ray(position, dir.normalize());
    }
};

// 光源类
struct Light {
    Vec3 position;
    Vec3 color;
    float intensity;
    
    Light(const Vec3& pos, const Vec3& col, float i) : position(pos), color(col), intensity(i) {}
};

// PPM图像写入函数
void writePPM(const std::string& filename, const std::vector<Vec3>& image, int width, int height) {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            Vec3 pixel = image[y * width + x];
            int r = std::min(255, std::max(0, (int)(pixel.x * 255)));
            int g = std::min(255, std::max(0, (int)(pixel.y * 255)));
            int b = std::min(255, std::max(0, (int)(pixel.z * 255)));
            file << r << " " << g << " " << b << "\n";
        }
    }
    
    file.close();
}

// 主渲染函数
Vec3 traceRay(const Ray& ray, const std::vector<Sphere>& spheres, const Light& light, int depth = 0) {
    if (depth > 5) return Vec3(0, 0, 0); // 递归深度限制
    
    float closestT = std::numeric_limits<float>::max();
    const Sphere* closestSphere = nullptr;
    
    // 寻找最近的交点
    for (const auto& sphere : spheres) {
        float t;
        if (sphere.intersect(ray, t) && t < closestT) {
            closestT = t;
            closestSphere = &sphere;
            std::cout << "找到交点! t=" << t << " 球体位置: (" << sphere.center.x << ", " << sphere.center.y << ", " << sphere.center.z << ")" << std::endl;
        }
    }
    
    if (!closestSphere) return Vec3(0.2f, 0.2f, 0.2f); // 背景色
    
    Vec3 hitPoint = ray.pointAt(closestT);
    Vec3 normal = closestSphere->getNormal(hitPoint);
    Vec3 color = closestSphere->color;
    
    // 漫反射光照
    Vec3 lightDir = (light.position - hitPoint).normalize();
    float diffuse = std::max(0.0f, normal.dot(lightDir));
    
    // 环境光
    float ambient = 0.1f;
    
    // 最终颜色
    Vec3 finalColor = color * (ambient + diffuse * light.intensity);
    
    return finalColor;
}

int main() {
    std::cout << "🚀 开始渲染简单光线追踪器...\n";
    
    // 图像参数
    const int WIDTH = 400;
    const int HEIGHT = 300;
    std::vector<Vec3> image(WIDTH * HEIGHT);
    
    // 场景设置
    Camera camera(Vec3(0, 0, 3), Vec3(0, 0, 0), Vec3(0, 1, 0), 60.0f);
    Light light(Vec3(2, 3, 2), Vec3(1, 1, 1), 1.0f);
    
    std::vector<Sphere> spheres = {
        Sphere(Vec3(0, 0, 0), 1.0f, Vec3(1.0f, 0.2f, 0.2f)), // 红色球
        Sphere(Vec3(-1.5f, -0.5f, -1.0f), 0.5f, Vec3(0.2f, 0.8f, 0.2f)), // 绿色球
        Sphere(Vec3(1.5f, 0.0f, -0.5f), 0.7f, Vec3(0.2f, 0.2f, 1.0f)) // 蓝色球
    };
    
    std::cout << "📊 场景: 3个球体 + 1个光源\n";
    std::cout << "📐 分辨率: " << WIDTH << "x" << HEIGHT << "\n";
    
    // 渲染循环
    for (int y = 0; y < HEIGHT; y++) {
        for (int x = 0; x < WIDTH; x++) {
            float u = (x + 0.5f) / WIDTH * 2.0f - 1.0f;
            float v = (y + 0.5f) / HEIGHT * 2.0f - 1.0f;
            
            Ray ray = camera.getRay(u, v, WIDTH, HEIGHT);
            Vec3 color = traceRay(ray, spheres, light);
            
            image[y * WIDTH + x] = color;
        }
        
        if (y % 30 == 0) {
            std::cout << "⏳ 进度: " << (y * 100 / HEIGHT) << "%\n";
        }
    }
    
    // 保存图像
    writePPM("output.ppm", image, WIDTH, HEIGHT);
    
    std::cout << "✅ 渲染完成！保存为 output.ppm\n";
    std::cout << "🎯 预期结果: 3个彩色球体，有基本光照效果\n";
    
    return 0;
}