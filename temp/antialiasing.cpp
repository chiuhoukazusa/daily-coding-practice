// 抗锯齿光线追踪器 - MSAA/SSAA 对比
// Anti-Aliased Ray Tracer with SSAA and MSAA
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

using namespace std;

// 向量类
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
    
    double length() const { return sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { return *this / length(); }
    
    Vec3 reflect(const Vec3& n) const {
        return *this - n * (2.0 * dot(n));
    }
};

// 光线
struct Ray {
    Vec3 origin, direction;
    Ray(Vec3 o, Vec3 d) : origin(o), direction(d.normalize()) {}
};

// 材质
struct Material {
    Vec3 color;
    double ambient, diffuse, specular, shininess, reflectivity;
    
    Material(Vec3 c = Vec3(1,1,1), double amb = 0.1, double diff = 0.7, 
             double spec = 0.2, double shin = 32, double refl = 0.0)
        : color(c), ambient(amb), diffuse(diff), specular(spec), 
          shininess(shin), reflectivity(refl) {}
};

// 球体
struct Sphere {
    Vec3 center;
    double radius;
    Material material;
    
    Sphere(Vec3 c, double r, Material m) : center(c), radius(r), material(m) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
        double t2 = (-b + sqrt(discriminant)) / (2.0 * a);
        
        if (t1 > 0.001) { t = t1; return true; }
        if (t2 > 0.001) { t = t2; return true; }
        return false;
    }
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
};

// 场景
struct Scene {
    vector<Sphere> spheres;
    Vec3 lightPos;
    Vec3 lightColor;
    Vec3 bgColor;
    
    Scene() : lightPos(10, 10, 10), lightColor(1, 1, 1), bgColor(0.2, 0.3, 0.5) {}
    
    void addSphere(const Sphere& s) { spheres.push_back(s); }
    
    bool intersect(const Ray& ray, int& hitIdx, double& t) const {
        hitIdx = -1;
        t = 1e30;
        
        for (size_t i = 0; i < spheres.size(); i++) {
            double tHit;
            if (spheres[i].intersect(ray, tHit) && tHit < t) {
                t = tHit;
                hitIdx = i;
            }
        }
        return hitIdx >= 0;
    }
    
    Vec3 trace(const Ray& ray, int depth = 0) const {
        if (depth > 3) return bgColor;
        
        int hitIdx;
        double t;
        if (!intersect(ray, hitIdx, t)) return bgColor;
        
        const Sphere& sphere = spheres[hitIdx];
        Vec3 hitPoint = ray.origin + ray.direction * t;
        Vec3 normal = sphere.getNormal(hitPoint);
        Vec3 viewDir = (ray.origin - hitPoint).normalize();
        
        // Ambient
        Vec3 color = sphere.material.color * sphere.material.ambient;
        
        // Shadow ray
        Vec3 lightDir = (lightPos - hitPoint).normalize();
        Ray shadowRay(hitPoint, lightDir);
        int shadowIdx;
        double shadowT;
        bool inShadow = intersect(shadowRay, shadowIdx, shadowT) && 
                        shadowT < (lightPos - hitPoint).length();
        
        if (!inShadow) {
            // Diffuse
            double diffuseFactor = max(0.0, normal.dot(lightDir));
            color = color + sphere.material.color * sphere.material.diffuse * diffuseFactor;
            
            // Specular
            Vec3 reflectDir = (lightDir * -1.0).reflect(normal);
            double specFactor = pow(max(0.0, reflectDir.dot(viewDir)), sphere.material.shininess);
            color = color + lightColor * sphere.material.specular * specFactor;
        }
        
        // Reflection
        if (sphere.material.reflectivity > 0.0) {
            Vec3 reflectDir = (viewDir * -1.0).reflect(normal);
            Ray reflectRay(hitPoint, reflectDir);
            Vec3 reflectColor = trace(reflectRay, depth + 1);
            color = color * (1.0 - sphere.material.reflectivity) + 
                    reflectColor * sphere.material.reflectivity;
        }
        
        return color;
    }
};

// 随机数生成器
random_device rd;
mt19937 gen(rd());
uniform_real_distribution<> dis(0.0, 1.0);

// SSAA: 超级采样抗锯齿（每像素多条光线）
Vec3 renderPixelSSAA(const Scene& scene, int x, int y, int width, int height, int samples) {
    Vec3 color(0, 0, 0);
    
    for (int s = 0; s < samples; s++) {
        // 抖动采样
        double jitterX = dis(gen);
        double jitterY = dis(gen);
        
        double u = (x + jitterX) / width * 2.0 - 1.0;
        double v = 1.0 - (y + jitterY) / height * 2.0;
        
        Vec3 rayDir(u, v, -1);
        Ray ray(Vec3(0, 0, 5), rayDir);
        
        color = color + scene.trace(ray);
    }
    
    return color / samples;
}

// MSAA: 多重采样抗锯齿（边缘检测+超采样）
Vec3 renderPixelMSAA(const Scene& scene, int x, int y, int width, int height,
                      const vector<Vec3>& buffer, int samples) {
    // 检测边缘（与相邻像素颜色差异）
    bool isEdge = false;
    
    if (x > 0 && x < width - 1 && y > 0 && y < height - 1) {
        Vec3 center = buffer[y * width + x];
        Vec3 left = buffer[y * width + (x-1)];
        Vec3 right = buffer[y * width + (x+1)];
        Vec3 top = buffer[(y-1) * width + x];
        Vec3 bottom = buffer[(y+1) * width + x];
        
        double threshold = 0.1;
        isEdge = ((center - left).length() > threshold ||
                  (center - right).length() > threshold ||
                  (center - top).length() > threshold ||
                  (center - bottom).length() > threshold);
    }
    
    // 如果是边缘，使用SSAA；否则直接返回原始颜色
    if (isEdge) {
        return renderPixelSSAA(scene, x, y, width, height, samples);
    } else {
        return buffer[y * width + x];
    }
}

// 渲染无抗锯齿图像（用于对比）
void renderNoAA(const Scene& scene, int width, int height, const string& filename) {
    vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double u = x / (double)width * 2.0 - 1.0;
            double v = 1.0 - y / (double)height * 2.0;
            
            Vec3 rayDir(u, v, -1);
            Ray ray(Vec3(0, 0, 5), rayDir);
            
            Vec3 color = scene.trace(ray);
            
            int idx = (y * width + x) * 3;
            image[idx] = min(255, (int)(color.x * 255));
            image[idx + 1] = min(255, (int)(color.y * 255));
            image[idx + 2] = min(255, (int)(color.z * 255));
        }
    }
    
    stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    cout << "✅ 无抗锯齿图像已保存: " << filename << endl;
}

// 渲染SSAA图像
void renderSSAA(const Scene& scene, int width, int height, int samples, const string& filename) {
    vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 color = renderPixelSSAA(scene, x, y, width, height, samples);
            
            int idx = (y * width + x) * 3;
            image[idx] = min(255, (int)(color.x * 255));
            image[idx + 1] = min(255, (int)(color.y * 255));
            image[idx + 2] = min(255, (int)(color.z * 255));
        }
    }
    
    stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    cout << "✅ SSAA图像已保存: " << filename << endl;
}

// 渲染MSAA图像
void renderMSAA(const Scene& scene, int width, int height, int samples, const string& filename) {
    // 第一遍：渲染无抗锯齿版本
    vector<Vec3> buffer(width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double u = x / (double)width * 2.0 - 1.0;
            double v = 1.0 - y / (double)height * 2.0;
            
            Vec3 rayDir(u, v, -1);
            Ray ray(Vec3(0, 0, 5), rayDir);
            
            buffer[y * width + x] = scene.trace(ray);
        }
    }
    
    // 第二遍：边缘检测+超采样
    vector<unsigned char> image(width * height * 3);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            Vec3 color = renderPixelMSAA(scene, x, y, width, height, buffer, samples);
            
            int idx = (y * width + x) * 3;
            image[idx] = min(255, (int)(color.x * 255));
            image[idx + 1] = min(255, (int)(color.y * 255));
            image[idx + 2] = min(255, (int)(color.z * 255));
        }
    }
    
    stbi_write_png(filename.c_str(), width, height, 3, image.data(), width * 3);
    cout << "✅ MSAA图像已保存: " << filename << endl;
}

int main() {
    // 创建场景
    Scene scene;
    
    // 添加球体（有明显边缘的场景）
    scene.addSphere(Sphere(Vec3(0, 0, 0), 1.0, 
                           Material(Vec3(1.0, 0.3, 0.3), 0.1, 0.7, 0.5, 64, 0.3)));
    scene.addSphere(Sphere(Vec3(-2.5, 0, -1), 0.8, 
                           Material(Vec3(0.3, 1.0, 0.3), 0.1, 0.7, 0.3, 32, 0.0)));
    scene.addSphere(Sphere(Vec3(2.5, 0, -1), 0.8, 
                           Material(Vec3(0.3, 0.3, 1.0), 0.1, 0.7, 0.3, 32, 0.0)));
    scene.addSphere(Sphere(Vec3(0, -101, 0), 100, 
                           Material(Vec3(0.8, 0.8, 0.8), 0.2, 0.6, 0.1, 16, 0.0)));
    
    int width = 800;
    int height = 600;
    
    cout << "🎨 开始渲染抗锯齿对比图像..." << endl;
    
    // 1. 无抗锯齿
    cout << "1️⃣ 渲染无抗锯齿版本..." << endl;
    renderNoAA(scene, width, height, "no_aa.png");
    
    // 2. SSAA 4x
    cout << "2️⃣ 渲染SSAA 4x版本..." << endl;
    renderSSAA(scene, width, height, 4, "ssaa_4x.png");
    
    // 3. MSAA 4x
    cout << "3️⃣ 渲染MSAA 4x版本..." << endl;
    renderMSAA(scene, width, height, 4, "msaa_4x.png");
    
    cout << "\n✅ 所有图像渲染完成！" << endl;
    cout << "📊 对比结果：" << endl;
    cout << "   - no_aa.png:     无抗锯齿（锯齿明显）" << endl;
    cout << "   - ssaa_4x.png:   SSAA 4x（质量最高，最慢）" << endl;
    cout << "   - msaa_4x.png:   MSAA 4x（边缘优化，性能最佳）" << endl;
    
    return 0;
}
