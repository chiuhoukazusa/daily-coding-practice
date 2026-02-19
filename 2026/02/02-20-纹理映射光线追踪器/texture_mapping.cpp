// 纹理映射光线追踪器
// 功能: 球面UV映射 + 棋盘格纹理采样
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 normalize() const {
        double len = sqrt(x*x + y*y + z*z);
        return (len > 0) ? Vec3(x/len, y/len, z/len) : Vec3(0,0,0);
    }
};

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
};

// UV坐标结构
struct UV {
    double u, v;
    UV(double u=0, double v=0) : u(u), v(v) {}
};

// 计算球面UV坐标
UV sphereUV(const Vec3& point) {
    // point是单位球面上的点
    Vec3 p = point.normalize();
    double u = 0.5 + atan2(p.z, p.x) / (2 * M_PI);
    double v = 0.5 - asin(p.y) / M_PI;
    return UV(u, v);
}

// 棋盘格纹理采样
Vec3 checkerboardTexture(const UV& uv, int scale = 10) {
    int ui = static_cast<int>(floor(uv.u * scale));
    int vi = static_cast<int>(floor(uv.v * scale));
    bool isEven = ((ui + vi) % 2) == 0;
    return isEven ? Vec3(0.9, 0.9, 0.9) : Vec3(0.2, 0.2, 0.8);
}

struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    bool hasTexture;
    
    Sphere(const Vec3& c, double r, const Vec3& col, bool tex = false)
        : center(c), radius(r), color(col), hasTexture(tex) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b*b - 4*a*c;
        
        if (discriminant < 0) return false;
        
        double t1 = (-b - sqrt(discriminant)) / (2*a);
        double t2 = (-b + sqrt(discriminant)) / (2*a);
        
        if (t1 > 0.001) { t = t1; return true; }
        if (t2 > 0.001) { t = t2; return true; }
        return false;
    }
    
    Vec3 normal(const Vec3& point) const {
        return (point - center).normalize();
    }
    
    Vec3 getColor(const Vec3& point) const {
        if (!hasTexture) return color;
        
        // 将交点转换为球心为原点的坐标
        Vec3 localPoint = (point - center) * (1.0 / radius);
        UV uv = sphereUV(localPoint);
        return checkerboardTexture(uv, 10);
    }
};

struct Scene {
    std::vector<Sphere> spheres;
    Vec3 lightPos;
    
    Scene() {
        lightPos = Vec3(5, 5, -5);
        
        // 中心大球 - 带纹理
        spheres.push_back(Sphere(Vec3(0, 0, 0), 1.0, Vec3(0,0,0), true));
        
        // 左侧小球 - 纯色
        spheres.push_back(Sphere(Vec3(-2.5, 0, -1), 0.8, Vec3(1.0, 0.3, 0.3), false));
        
        // 右侧小球 - 带纹理
        spheres.push_back(Sphere(Vec3(2.5, 0, -1), 0.8, Vec3(0,0,0), true));
        
        // 地面大球 - 带纹理（模拟平面）
        spheres.push_back(Sphere(Vec3(0, -1001, 0), 1000, Vec3(0,0,0), true));
    }
    
    bool trace(const Ray& ray, int& hitIdx, double& hitT) const {
        hitIdx = -1;
        hitT = 1e20;
        
        for (size_t i = 0; i < spheres.size(); i++) {
            double t;
            if (spheres[i].intersect(ray, t) && t < hitT) {
                hitT = t;
                hitIdx = i;
            }
        }
        return hitIdx >= 0;
    }
    
    Vec3 shade(const Vec3& point, const Vec3& normal, const Vec3& color) const {
        Vec3 lightDir = (lightPos - point).normalize();
        
        // 检查阴影
        Ray shadowRay(point, lightDir);
        int shadowIdx;
        double shadowT;
        if (trace(shadowRay, shadowIdx, shadowT)) {
            double lightDist = sqrt((lightPos - point).dot(lightPos - point));
            if (shadowT < lightDist) {
                // 在阴影中，只有环境光
                return color * 0.2;
            }
        }
        
        // Phong光照
        double diffuse = std::max(0.0, normal.dot(lightDir));
        Vec3 ambient = color * 0.2;
        Vec3 diff = color * diffuse * 0.8;
        
        return ambient + diff;
    }
};

Vec3 render(const Ray& ray, const Scene& scene, int depth = 0) {
    if (depth > 3) return Vec3(0, 0, 0);
    
    int hitIdx;
    double hitT;
    
    if (!scene.trace(ray, hitIdx, hitT)) {
        // 背景渐变
        double t = 0.5 * (ray.direction.y + 1.0);
        Vec3 white(1.0, 1.0, 1.0);
        Vec3 blue(0.5, 0.7, 1.0);
        return white * (1.0 - t) + blue * t;
    }
    
    const Sphere& sphere = scene.spheres[hitIdx];
    Vec3 hitPoint = ray.origin + ray.direction * hitT;
    Vec3 normal = sphere.normal(hitPoint);
    Vec3 color = sphere.getColor(hitPoint);
    
    return scene.shade(hitPoint, normal, color);
}

int main() {
    const int width = 800;
    const int height = 600;
    std::vector<unsigned char> image(width * height * 3);
    
    Scene scene;
    Vec3 cameraPos(0, 2, -8);
    Vec3 cameraTarget(0, 0, 0);
    Vec3 cameraUp(0, 1, 0);
    
    Vec3 forward = (cameraTarget - cameraPos).normalize();
    Vec3 right = Vec3(forward.z, 0, -forward.x).normalize();
    Vec3 up = Vec3(
        right.y*forward.z - right.z*forward.y,
        right.z*forward.x - right.x*forward.z,
        right.x*forward.y - right.y*forward.x
    ).normalize();
    
    double fov = M_PI / 3.0;
    double aspectRatio = static_cast<double>(width) / height;
    
    std::cout << "开始渲染 " << width << "x" << height << " ..." << std::endl;
    
    for (int y = 0; y < height; y++) {
        if (y % 100 == 0) {
            std::cout << "进度: " << (100*y/height) << "%" << std::endl;
        }
        
        for (int x = 0; x < width; x++) {
            double px = (2.0 * (x + 0.5) / width - 1.0) * tan(fov/2) * aspectRatio;
            double py = (1.0 - 2.0 * (y + 0.5) / height) * tan(fov/2);
            
            // ✅ 修复：up向量指向下方（-Y），需要翻转py
            py = -py;
            
            Vec3 rayDir = (forward + right * px + up * py).normalize();
            Ray ray(cameraPos, rayDir);
            
            Vec3 color = render(ray, scene);
            
            int idx = (y * width + x) * 3;
            image[idx + 0] = static_cast<unsigned char>(std::min(255.0, color.x * 255));
            image[idx + 1] = static_cast<unsigned char>(std::min(255.0, color.y * 255));
            image[idx + 2] = static_cast<unsigned char>(std::min(255.0, color.z * 255));
        }
    }
    
    std::cout << "渲染完成，保存图片..." << std::endl;
    stbi_write_png("texture_output.png", width, height, 3, image.data(), width * 3);
    std::cout << "✅ 图片已保存: texture_output.png" << std::endl;
    
    return 0;
}
