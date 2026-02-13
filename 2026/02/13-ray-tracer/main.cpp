#include <cmath>
#include <vector>
#include <fstream>
#include <cfloat>
#include <algorithm>
#include <iostream>

struct Vec3 {
    float x, y, z;
    
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }
    
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { 
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    float length() const { return sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { float l = length(); return l > 0 ? Vec3(x/l, y/l, z/l) : *this; }
};

struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction.normalize()) {}
    
    Vec3 at(float t) const { return origin + direction * t; }
};

struct Sphere {
    Vec3 center;
    float radius;
    Vec3 color;
    
    Sphere(const Vec3& center, float radius, const Vec3& color) 
        : center(center), radius(radius), color(color) {}
    
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
        
        t = t0;
        if (t0 < 0) t = t1;
        if (t < 0) return false;
        
        return true;
    }
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
};

Vec3 simpleTrace(const Ray& ray, const std::vector<Sphere>& spheres) {
    float minT = FLT_MAX;
    int hitSphere = -1;
    float t;
    
    for (size_t i = 0; i < spheres.size(); i++) {
        if (spheres[i].intersect(ray, t) && t < minT) {
            minT = t;
            hitSphere = i;
        }
    }
    
    if (hitSphere == -1) {
        // 背景色 - 简单的渐变天空
        float t = 0.5f * (ray.direction.y + 1.0f);
        return Vec3(1.0f, 1.0f, 1.0f) * (1.0f - t) + Vec3(0.5f, 0.7f, 1.0f) * t;
    }
    
    Vec3 hitPoint = ray.at(minT);
    Vec3 normal = spheres[hitSphere].getNormal(hitPoint);
    
    // 简单的光照：假设光源在相机上方
    Vec3 lightDir = Vec3(0.0f, 1.0f, 0.5f).normalize();
    float diffuse = std::max(0.0f, normal.dot(lightDir));
    
    // 返回颜色，添加一些环境光
    Vec3 color = spheres[hitSphere].color;
    return color * (0.3f + 0.7f * diffuse);
}

void savePPM(const std::vector<Vec3>& pixels, int width, int height, const std::string& filename) {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            Vec3 pixel = pixels[idx];
            
            // 将浮点颜色值转换为0-255范围
            int r = static_cast<int>(std::min(255.0f, std::max(0.0f, pixel.x * 255.0f)));
            int g = static_cast<int>(std::min(255.0f, std::max(0.0f, pixel.y * 255.0f)));
            int b = static_cast<int>(std::min(255.0f, std::max(0.0f, pixel.z * 255.0f)));
            
            file << r << " " << g << " " << b << " ";
        }
        file << "\n";
    }
    
    file.close();
}

int main() {
    // 设置图像参数
    const int width = 600;
    const int height = 400;
    std::vector<Vec3> pixels(width * height);
    
    // 创建场景中的球体
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3(0.0f, 0.0f, -2.0f), 0.5f, Vec3(1.0f, 0.2f, 0.2f))); // 红色球
    spheres.push_back(Sphere(Vec3(1.0f, 0.0f, -2.0f), 0.3f, Vec3(0.2f, 1.0f, 0.2f))); // 绿色球
    spheres.push_back(Sphere(Vec3(-0.7f, -0.1f, -1.0f), 0.2f, Vec3(0.2f, 0.2f, 1.0f))); // 蓝色球
    spheres.push_back(Sphere(Vec3(0.0f, -100.5f, -1.0f), 100.0f, Vec3(0.8f, 0.8f, 0.8f))); // 地面
    
    // 相机设置
    Vec3 cameraPos(0.0f, 0.0f, 0.0f);
    float aspectRatio = float(width) / float(height);
    float viewportHeight = 2.0f;
    float viewportWidth = aspectRatio * viewportHeight;
    
    // 渲染循环
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 计算像素坐标到虚拟平面的映射
            float u = float(x) / float(width - 1);
            float v = float(height - y - 1) / float(height - 1);
            
            // 计算光线方向
            Vec3 pixelPos = Vec3(
                cameraPos.x + (u - 0.5f) * viewportWidth,
                cameraPos.y + (v - 0.5f) * viewportHeight,
                cameraPos.z - 1.0f
            );
            
            Ray ray(cameraPos, pixelPos - cameraPos);
            
            // 追踪光线，获取颜色
            Vec3 color = simpleTrace(ray, spheres);
            
            // 存储像素颜色
            pixels[y * width + x] = color;
        }
    }
    
    // 保存图像
    savePPM(pixels, width, height, "output.ppm");
    
    // 创建PNG格式的可查看图像
    std::cout << "渲染完成！创建了 " << width << "x" << height << " 像素的图像" << std::endl;
    std::cout << "输出文件: output.ppm" << std::endl;
    
    return 0;
}