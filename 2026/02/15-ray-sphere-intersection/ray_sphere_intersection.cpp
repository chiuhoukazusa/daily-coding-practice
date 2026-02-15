#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>

// 3D向量类
struct Vec3 {
    float x, y, z;
    
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
    
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    float length() const { return sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { return (*this) / length(); }
};

// 球体类
struct Sphere {
    Vec3 center;
    float radius;
    
    Sphere(const Vec3& center, float radius) : center(center), radius(radius) {}
};

// 光线类
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {}
};

// 光线与球体相交检测
bool raySphereIntersect(const Ray& ray, const Sphere& sphere, float& t) {
    Vec3 oc = ray.origin - sphere.center;
    float a = ray.direction.dot(ray.direction);
    float b = 2.0f * oc.dot(ray.direction);
    float c = oc.dot(oc) - sphere.radius * sphere.radius;
    
    float discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return false; // 没有交点
    }
    
    float sqrtDisc = sqrt(discriminant);
    float t1 = (-b - sqrtDisc) / (2.0f * a);
    float t2 = (-b + sqrtDisc) / (2.0f * a);
    
    if (t1 > 0 && t2 > 0) {
        t = (t1 < t2) ? t1 : t2; // 取较近的交点
        return true;
    } else if (t1 > 0) {
        t = t1;
        return true;
    } else if (t2 > 0) {
        t = t2;
        return true;
    }
    
    return false;
}

// 写入PPM图像文件
void writePPM(const std::string& filename, int width, int height, const std::vector<std::vector<Vec3>>& pixels) {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Vec3 color = pixels[y][x];
            // 限制颜色范围并转换为整数 (0-255)
            int r = std::min(255, std::max(0, static_cast<int>(color.x * 255)));
            int g = std::min(255, std::max(0, static_cast<int>(color.y * 255)));
            int b = std::min(255, std::max(0, static_cast<int>(color.z * 255)));
            file << r << " " << g << " " << b << "\n";
        }
    }
    
    file.close();
}

int main() {
    std::cout << "Ray-Sphere Intersection Visualization" << std::endl;
    
    // 图像尺寸
    const int width = 400;
    const int height = 300;
    
    // 创建像素缓冲区
    std::vector<std::vector<Vec3>> pixels(height, std::vector<Vec3>(width, Vec3(0, 0, 0)));
    
    // 创建球体
    Sphere sphere(Vec3(200, 150, 100), 80.0f);
    
    // 光线原点（相机位置）
    Vec3 cameraPos(200, 150, -200);
    
    // 渲染图像
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 创建从相机到像素的光线
            Vec3 pixelPos(x, y, 0);
            Vec3 rayDir = (pixelPos - cameraPos).normalize();
            Ray ray(cameraPos, rayDir);
            
            float t;
            if (raySphereIntersect(ray, sphere, t)) {
                // 计算交点位置
                Vec3 hitPoint = ray.origin + ray.direction * t;
                
                // 计算法线
                Vec3 normal = (hitPoint - sphere.center).normalize();
                
                // 使用法线作为颜色（简单的法线映射）
                Vec3 color = (normal + Vec3(1, 1, 1)) * 0.5f;
                pixels[y][x] = color;
            } else {
                // 背景色 - 渐变色
                float gradient = y / static_cast<float>(height);
                pixels[y][x] = Vec3(0.1f, 0.1f, 0.3f + gradient * 0.2f);
            }
        }
    }
    
    // 保存图像
    writePPM("ray_sphere_intersection.ppm", width, height, pixels);
    
    std::cout << "图像生成完成: ray_sphere_intersection.ppm" << std::endl;
    std::cout << "球体中心: (200, 150, 100), 半径: 80" << std::endl;
    std::cout << "相机位置: (200, 150, -200)" << std::endl;
    
    return 0;
}