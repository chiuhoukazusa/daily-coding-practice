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

// 球体
struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;
    
    Sphere(const Vec3& c, double r, const Vec3& col) 
        : center(c), radius(r), color(col) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        t = (-b - std::sqrt(discriminant)) / (2.0 * a);
        return t > 0.001;
    }
    
    Vec3 get_normal(const Vec3& p) const {
        return (p - center).normalize();
    }
};

// 场景
class Scene {
public:
    std::vector<Sphere> spheres;
    Vec3 light_pos;
    Vec3 light_color;
    
    Scene() {
        light_pos = Vec3(5, 4, 2);  // 右上前方
        light_color = Vec3(1.0, 0.95, 0.85) * 1.2;  // 降低光强（原3.0太强）
        
        // 三个球体作为遮挡物
        spheres.push_back(Sphere(Vec3(-2, 0, -3), 1.2, Vec3(0.8, 0.3, 0.3)));
        spheres.push_back(Sphere(Vec3(1, -0.5, -2), 0.8, Vec3(0.3, 0.6, 0.9)));
        spheres.push_back(Sphere(Vec3(0, 1.5, -4), 1.0, Vec3(0.4, 0.8, 0.4)));
    }
    
    bool intersect(const Ray& ray, double& t, Vec3& color, Vec3& normal) const {
        double closest_t = 1e10;
        bool hit = false;
        
        for (const auto& sphere : spheres) {
            double temp_t;
            if (sphere.intersect(ray, temp_t) && temp_t < closest_t) {
                closest_t = temp_t;
                color = sphere.color;
                Vec3 hit_point = ray.at(temp_t);
                normal = sphere.get_normal(hit_point);
                hit = true;
            }
        }
        
        if (hit) t = closest_t;
        return hit;
    }
    
    // 检查点到光源是否被遮挡
    bool is_occluded(const Vec3& point, const Vec3& light_dir, double light_dist) const {
        Ray shadow_ray(point, light_dir);
        double t;
        Vec3 dummy_color, dummy_normal;
        
        if (intersect(shadow_ray, t, dummy_color, dummy_normal)) {
            return t < light_dist - 0.01;
        }
        return false;
    }
};

// 简单光照
Vec3 simple_shading(const Vec3& point, const Vec3& normal, const Vec3& base_color,
                    const Vec3& light_pos, const Vec3& light_color) {
    Vec3 light_dir = (light_pos - point).normalize();
    double dist = (light_pos - point).length();
    double atten = 1.0 / (1.0 + 0.05 * dist * dist);
    
    double diff = std::max(0.0, normal.dot(light_dir));
    Vec3 ambient = base_color * 0.3;
    Vec3 diffuse = base_color * light_color * diff * atten;
    
    return (ambient + diffuse).clamp();
}

// 体积光（Ray Marching with occlusion）
Vec3 volumetric_light(const Ray& ray, const Scene& scene, double max_dist) {
    const int NUM_STEPS = 60;
    const double SCATTERING = 0.03;  // 再次降低散射系数
    
    double step_size = max_dist / NUM_STEPS;
    Vec3 accumulated(0, 0, 0);
    
    for (int i = 0; i < NUM_STEPS; i++) {
        double t = (i + 0.5) * step_size;
        Vec3 sample_pos = ray.at(t);
        
        Vec3 to_light = scene.light_pos - sample_pos;
        double light_dist = to_light.length();
        Vec3 light_dir = to_light.normalize();
        
        // 检查是否被遮挡
        if (!scene.is_occluded(sample_pos, light_dir, light_dist)) {
            double atten = 1.0 / (1.0 + 0.02 * light_dist * light_dist);
            double scatter = SCATTERING * step_size * atten;
            accumulated = accumulated + scene.light_color * scatter;
        }
    }
    
    return accumulated.clamp();
}

void save_ppm(const std::string& filename, const std::vector<Vec3>& pixels, int w, int h) {
    std::ofstream file(filename);
    file << "P3\n" << w << " " << h << "\n255\n";
    
    for (int j = h - 1; j >= 0; j--) {
        for (int i = 0; i < w; i++) {
            Vec3 c = pixels[j * w + i];
            file << int(255.99*c.x) << " " << int(255.99*c.y) << " " << int(255.99*c.z) << "\n";
        }
    }
}

void render(const std::string& filename, bool use_volumetric, const std::string& desc) {
    std::cout << "\n📸 " << desc << std::endl;
    
    Scene scene;
    Vec3 camera_pos(0, 0, 5);
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 100 == 0) std::cout << "  进度: " << (100.0*j/HEIGHT) << "%" << std::endl;
        
        for (int i = 0; i < WIDTH; i++) {
            double u = (2.0*(i+0.5)/WIDTH - 1.0) * (double(WIDTH)/HEIGHT);
            double v = 2.0*(j+0.5)/HEIGHT - 1.0;
            
            Vec3 ray_dir = Vec3(u, v, -1.5).normalize();
            Ray ray(camera_pos, ray_dir);
            
            Vec3 final_color(0, 0, 0);
            double t;
            Vec3 surface_color, normal;
            
            if (scene.intersect(ray, t, surface_color, normal)) {
                // 击中物体
                Vec3 hit_point = ray.at(t);
                final_color = simple_shading(hit_point, normal, surface_color, 
                                            scene.light_pos, scene.light_color);
                
                // 添加体积光（相机到表面）
                if (use_volumetric) {
                    Vec3 vol = volumetric_light(ray, scene, t);
                    final_color = final_color + vol;
                }
            } else {
                // 未击中，背景
                if (use_volumetric) {
                    final_color = volumetric_light(ray, scene, 15.0);
                } else {
                    // 深色背景
                    final_color = Vec3(0.05, 0.05, 0.08);
                }
            }
            
            pixels[j*WIDTH + i] = final_color.clamp();
        }
    }
    
    save_ppm(filename, pixels, WIDTH, HEIGHT);
    std::cout << "✅ 已保存: " << filename << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  体积光渲染 - God Rays" << std::endl;
    std::cout << "========================================" << std::endl;
    
    render("scene_no_vol.ppm", false, "普通渲染（无体积光）");
    render("scene_with_vol.ppm", true, "体积光渲染（God Rays）");
    
    std::cout << "\n🎉 渲染完成！" << std::endl;
    
    return 0;
}
