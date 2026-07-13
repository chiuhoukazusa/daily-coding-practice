#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

// 向量结构
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
    Vec3 normalize() const { return *this / length(); }
    
    Vec3 reflect(const Vec3& n) const {
        return *this - n * (2 * this->dot(n));
    }
};

// 光线结构
struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
};

// 球体结构
struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    double reflection;  // 反射系数
    
    Sphere(const Vec3& c, double r, const Vec3& col, double refl = 0.0)
        : center(c), radius(r), color(col), reflection(refl) {}
    
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
        
        if (t1 > 0.001) {
            t = t1;
            return true;
        }
        if (t2 > 0.001) {
            t = t2;
            return true;
        }
        return false;
    }
};

// 光源结构
struct Light {
    Vec3 position;
    Vec3 color;
    double intensity;
    
    Light(const Vec3& pos, const Vec3& col, double intens = 1.0)
        : position(pos), color(col), intensity(intens) {}
};

// 场景
std::vector<Sphere> scene;
std::vector<Light> lights;

// 递归光线追踪（支持反射）
Vec3 trace(const Ray& ray, int depth = 0) {
    if (depth > 3) return Vec3(0.1, 0.1, 0.15);  // 背景色
    
    double closest_t = 1e10;
    const Sphere* hit_sphere = nullptr;
    
    // 找到最近的交点
    for (const auto& sphere : scene) {
        double t;
        if (sphere.intersect(ray, t) && t < closest_t) {
            closest_t = t;
            hit_sphere = &sphere;
        }
    }
    
    // 没有击中物体，返回背景色
    if (!hit_sphere) {
        return Vec3(0.1, 0.1, 0.15);
    }
    
    // 计算交点和法线
    Vec3 hit_point = ray.origin + ray.direction * closest_t;
    Vec3 normal = (hit_point - hit_sphere->center).normalize();
    
    // 基础颜色
    Vec3 color = hit_sphere->color * 0.1;  // 环境光
    
    // 遍历所有光源
    for (const auto& light : lights) {
        Vec3 light_dir = (light.position - hit_point).normalize();
        
        // 阴影检测
        Ray shadow_ray(hit_point, light_dir);
        bool in_shadow = false;
        for (const auto& sphere : scene) {
            double t;
            if (sphere.intersect(shadow_ray, t) && t < (light.position - hit_point).length()) {
                in_shadow = true;
                break;
            }
        }
        
        if (!in_shadow) {
            // 漫反射 (Lambert)
            double diffuse = std::max(0.0, normal.dot(light_dir));
            
            // 镜面反射 (Phong)
            Vec3 reflect_dir = (light_dir * -1.0).reflect(normal);
            double specular = std::pow(std::max(0.0, reflect_dir.dot(ray.direction * -1.0)), 32);
            
            color = color + hit_sphere->color * diffuse * light.intensity * 0.7 
                          + Vec3(1, 1, 1) * specular * light.intensity * 0.5;
        }
    }
    
    // 反射
    if (hit_sphere->reflection > 0.0) {
        Vec3 reflect_dir = ray.direction.reflect(normal);
        Ray reflect_ray(hit_point, reflect_dir);
        Vec3 reflect_color = trace(reflect_ray, depth + 1);
        color = color * (1.0 - hit_sphere->reflection) + reflect_color * hit_sphere->reflection;
    }
    
    return color;
}

// 颜色转换为整数
int to_int(double x) {
    return std::min(255, std::max(0, int(x * 255)));
}

int main() {
    // 图像设置
    const int render_width = 1600;   // 渲染分辨率（4倍）
    const int render_height = 1200;
    const int output_width = 800;    // 输出分辨率
    const int output_height = 600;
    const int samples_per_pixel = 4; // SSAA 2x2 (每个像素采样4次)
    
    // 场景设置
    scene.push_back(Sphere(Vec3(0, 0, -5), 1.0, Vec3(0.8, 0.3, 0.3), 0.5));     // 红色镜面球
    scene.push_back(Sphere(Vec3(-2, 0, -6), 1.0, Vec3(0.3, 0.8, 0.3), 0.3));    // 绿色半镜面球
    scene.push_back(Sphere(Vec3(2, 0, -4), 0.8, Vec3(0.3, 0.3, 0.8), 0.7));     // 蓝色高反射球
    scene.push_back(Sphere(Vec3(0, -1001, 0), 1000, Vec3(0.6, 0.6, 0.6), 0.1)); // 地面
    
    // 光源
    lights.push_back(Light(Vec3(5, 5, -2), Vec3(1, 1, 1), 1.0));
    lights.push_back(Light(Vec3(-5, 3, -3), Vec3(0.7, 0.7, 1.0), 0.6));
    
    // 相机设置
    Vec3 camera_pos(0, 0, 0);
    double aspect_ratio = double(output_width) / output_height;
    double viewport_height = 2.0;
    double viewport_width = viewport_height * aspect_ratio;
    double focal_length = 1.0;
    
    // 渲染到高分辨率缓冲区
    std::vector<Vec3> render_buffer(render_width * render_height);
    
    std::cout << "渲染中 (SSAA 2x2, " << render_width << "x" << render_height 
              << " -> " << output_width << "x" << output_height << ")..." << std::endl;
    
    for (int j = 0; j < render_height; j++) {
        if (j % 100 == 0) {
            std::cout << "进度: " << (100 * j / render_height) << "%" << std::endl;
        }
        for (int i = 0; i < render_width; i++) {
            double u = (i + 0.5) / render_width;
            double v = (j + 0.5) / render_height;
            
            double x = (u - 0.5) * viewport_width;
            double y = (0.5 - v) * viewport_height;
            
            Vec3 direction = Vec3(x, y, -focal_length);
            Ray ray(camera_pos, direction);
            
            render_buffer[j * render_width + i] = trace(ray);
        }
    }
    
    std::cout << "下采样中..." << std::endl;
    
    // 下采样到输出分辨率（2x2 box filter）
    std::vector<Vec3> output_buffer(output_width * output_height);
    
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            Vec3 sum(0, 0, 0);
            
            // 采样2x2区域
            for (int dy = 0; dy < 2; dy++) {
                for (int dx = 0; dx < 2; dx++) {
                    int x = i * 2 + dx;
                    int y = j * 2 + dy;
                    sum = sum + render_buffer[y * render_width + x];
                }
            }
            
            output_buffer[j * output_width + i] = sum / 4.0;
        }
    }
    
    // 写入PNG文件
    std::cout << "保存图片..." << std::endl;
    
    std::ofstream ofs("ssaa_output.ppm", std::ios::binary);
    ofs << "P6\n" << output_width << " " << output_height << "\n255\n";
    
    for (int j = 0; j < output_height; j++) {
        for (int i = 0; i < output_width; i++) {
            Vec3 color = output_buffer[j * output_width + i];
            ofs << (unsigned char)to_int(color.x)
                << (unsigned char)to_int(color.y)
                << (unsigned char)to_int(color.z);
        }
    }
    
    ofs.close();
    std::cout << "渲染完成！输出文件: ssaa_output.ppm" << std::endl;
    
    // 转换为PNG
    system("convert ssaa_output.ppm ssaa_output.png");
    std::cout << "已转换为PNG格式: ssaa_output.png" << std::endl;
    
    return 0;
}
