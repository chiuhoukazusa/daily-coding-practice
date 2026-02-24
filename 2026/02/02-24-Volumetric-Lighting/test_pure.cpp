#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

const int WIDTH = 1200;
const int HEIGHT = 800;
const double PI = 3.14159265358979323846;

struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { return *this / length(); }
    
    Vec3 clamp(double min = 0.0, double max = 1.0) const {
        return Vec3(std::max(min, std::min(max, x)),
                    std::max(min, std::min(max, y)),
                    std::max(min, std::min(max, z)));
    }
};

struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(double t) const { return origin + direction * t; }
};

// 简单的体积光（无遮挡物）
Vec3 pure_volumetric(const Ray& ray, const Vec3& light_pos, const Vec3& light_color, double max_distance) {
    const int NUM_STEPS = 64;
    const double SCATTERING = 0.15;  // 雾的密度
    
    double step_size = max_distance / NUM_STEPS;
    Vec3 accumulated(0, 0, 0);
    
    for (int i = 0; i < NUM_STEPS; i++) {
        double t = (i + 0.5) * step_size;
        Vec3 sample_pos = ray.at(t);
        
        Vec3 to_light = light_pos - sample_pos;
        double distance = to_light.length();
        
        // 距离平方衰减
        double attenuation = 1.0 / (1.0 + 0.02 * distance * distance);
        
        // 累积散射光
        double scatter = SCATTERING * step_size * attenuation;
        accumulated = accumulated + light_color * scatter;
    }
    
    return accumulated.clamp();
}

void save_ppm(const std::string& filename, const std::vector<Vec3>& pixels, int width, int height) {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int j = height - 1; j >= 0; j--) {
        for (int i = 0; i < width; i++) {
            Vec3 color = pixels[j * width + i];
            int r = static_cast<int>(255.99 * color.x);
            int g = static_cast<int>(255.99 * color.y);
            int b = static_cast<int>(255.99 * color.z);
            file << r << " " << g << " " << b << "\n";
        }
    }
}

int main() {
    std::cout << "渲染纯体积光（无遮挡）" << std::endl;
    
    Vec3 camera_pos(0, 0, 3);
    Vec3 light_pos(3, 2, -2);  // 右上方的光源
    Vec3 light_color(1.0, 0.95, 0.8);  // 温暖的阳光
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 100 == 0) {
            std::cout << "  进度: " << (100.0 * j / HEIGHT) << "%" << std::endl;
        }
        
        for (int i = 0; i < WIDTH; i++) {
            double u = (2.0 * (i + 0.5) / WIDTH - 1.0) * (double(WIDTH) / HEIGHT);
            double v = 2.0 * (j + 0.5) / HEIGHT - 1.0;
            
            Vec3 ray_dir = Vec3(u, v, -1.5).normalize();
            Ray ray(camera_pos, ray_dir);
            
            Vec3 color = pure_volumetric(ray, light_pos, light_color, 15.0);
            pixels[j * WIDTH + i] = color;
        }
    }
    
    save_ppm("pure_volumetric.ppm", pixels, WIDTH, HEIGHT);
    std::cout << "✅ 已保存: pure_volumetric.ppm" << std::endl;
    
    return 0;
}
