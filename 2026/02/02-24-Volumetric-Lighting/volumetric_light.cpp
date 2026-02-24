#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

const int WIDTH = 1200;
const int HEIGHT = 800;
const double PI = 3.14159265358979323846;

// 向量类
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }  // 逐元素乘法
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalize() const { return *this / length(); }
    
    Vec3 clamp(double min = 0.0, double max = 1.0) const {
        return Vec3(std::max(min, std::min(max, x)),
                    std::max(min, std::min(max, y)),
                    std::max(min, std::min(max, z)));
    }
};

// 射线
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    
    Vec3 at(double t) const {
        return origin + direction * t;
    }
};

// 场景物体：平面
struct Plane {
    Vec3 point;
    Vec3 normal;
    Vec3 color;
    
    Plane(const Vec3& p, const Vec3& n, const Vec3& c) 
        : point(p), normal(n.normalize()), color(c) {}
    
    bool intersect(const Ray& ray, double& t) const {
        double denom = normal.dot(ray.direction);
        if (std::abs(denom) < 1e-6) return false;
        
        t = (point - ray.origin).dot(normal) / denom;
        return t > 0.001;
    }
};

// 场景物体：球体
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
};

// 场景：简单的房间
class Scene {
public:
    std::vector<Plane> planes;
    std::vector<Sphere> spheres;
    Vec3 light_pos;
    Vec3 light_color;
    
    Scene() {
        // 光源位置（窗外，右上方）
        light_pos = Vec3(8.0, 6.0, -2.0);
        light_color = Vec3(1.0, 0.95, 0.8) * 2.0;  // 温暖的阳光
        
        // 地板
        planes.push_back(Plane(Vec3(0, -2, 0), Vec3(0, 1, 0), Vec3(0.3, 0.3, 0.35)));
        
        // 天花板
        planes.push_back(Plane(Vec3(0, 4, 0), Vec3(0, -1, 0), Vec3(0.4, 0.4, 0.45)));
        
        // 后墙
        planes.push_back(Plane(Vec3(0, 0, -5), Vec3(0, 0, 1), Vec3(0.5, 0.45, 0.4)));
        
        // 左墙
        planes.push_back(Plane(Vec3(-5, 0, 0), Vec3(1, 0, 0), Vec3(0.6, 0.3, 0.3)));
        
        // 右墙（有"窗户"，不渲染实体）
        // planes.push_back(Plane(Vec3(5, 0, 0), Vec3(-1, 0, 0), Vec3(0.3, 0.4, 0.6)));
        
        // 球体装饰
        spheres.push_back(Sphere(Vec3(-2, -1, -2), 1.0, Vec3(0.8, 0.6, 0.3)));
        spheres.push_back(Sphere(Vec3(1.5, -1.3, -1.5), 0.7, Vec3(0.4, 0.7, 0.9)));
    }
    
    // 场景求交（找最近的交点）
    bool intersect(const Ray& ray, double& t, Vec3& color, Vec3& normal) const {
        double closest_t = 1e10;
        bool hit = false;
        
        // 检查平面
        for (const auto& plane : planes) {
            double temp_t;
            if (plane.intersect(ray, temp_t) && temp_t < closest_t) {
                closest_t = temp_t;
                color = plane.color;
                normal = plane.normal;
                hit = true;
            }
        }
        
        // 检查球体
        for (const auto& sphere : spheres) {
            double temp_t;
            if (sphere.intersect(ray, temp_t) && temp_t < closest_t) {
                closest_t = temp_t;
                color = sphere.color;
                Vec3 hit_point = ray.at(temp_t);
                normal = (hit_point - sphere.center).normalize();
                hit = true;
            }
        }
        
        if (hit) {
            t = closest_t;
        }
        
        return hit;
    }
    
    // 检查点到光源是否被遮挡（阴影测试）
    bool is_in_shadow(const Vec3& point, const Vec3& light_direction, double light_distance) const {
        Ray shadow_ray(point, light_direction);
        double t;
        Vec3 dummy_color, dummy_normal;
        
        if (intersect(shadow_ray, t, dummy_color, dummy_normal)) {
            return t < light_distance - 0.001;
        }
        return false;
    }
};

// 简单的Phong光照
Vec3 phong_shading(const Vec3& point, const Vec3& normal, const Vec3& view_dir,
                   const Vec3& light_pos, const Vec3& light_color, const Vec3& base_color) {
    Vec3 light_dir = (light_pos - point).normalize();
    double distance = (light_pos - point).length();
    double attenuation = 1.0 / (1.0 + 0.05 * distance + 0.01 * distance * distance);
    
    // 环境光
    Vec3 ambient = base_color * 0.2;
    
    // 漫反射
    double diff = std::max(0.0, normal.dot(light_dir));
    Vec3 diffuse = base_color * light_color * diff * attenuation;
    
    // 镜面反射
    Vec3 reflect_dir = (normal * 2.0 * normal.dot(light_dir) - light_dir).normalize();
    double spec = std::pow(std::max(0.0, view_dir.dot(reflect_dir)), 32.0);
    Vec3 specular = light_color * spec * 0.3 * attenuation;
    
    return (ambient + diffuse + specular).clamp();
}

// 体积光计算（Ray Marching）
Vec3 volumetric_lighting(const Ray& ray, const Scene& scene, double max_distance) {
    const int NUM_STEPS = 80;  // 步进次数
    const double SCATTERING = 0.25;  // 散射系数（雾的密度）- 增大到0.25
    
    double step_size = max_distance / NUM_STEPS;
    Vec3 accumulated_light(0, 0, 0);
    double accumulated_transmission = 1.0;  // 透射率（光线能穿透的比例）
    
    // 沿着射线步进
    for (int i = 0; i < NUM_STEPS; i++) {
        double t = (i + 0.5) * step_size;
        Vec3 sample_pos = ray.at(t);
        
        // 计算该点到光源的方向和距离
        Vec3 to_light = scene.light_pos - sample_pos;
        double light_distance = to_light.length();
        Vec3 light_dir = to_light.normalize();
        
        // 检查该点是否能看到光源（阴影测试）
        bool in_shadow = scene.is_in_shadow(sample_pos, light_dir, light_distance);
        
        if (!in_shadow) {
            // 计算光照衰减（距离平方衰减）
            double attenuation = 1.0 / (1.0 + 0.05 * light_distance + 0.01 * light_distance * light_distance);
            
            // 累积散射光（考虑透射率）
            double scatter_amount = SCATTERING * step_size * attenuation;
            accumulated_light = accumulated_light + scene.light_color * scatter_amount * accumulated_transmission;
            
            // 更新透射率（光被散射后，后续步进的贡献会减少）
            accumulated_transmission *= std::exp(-SCATTERING * step_size);
        } else {
            // 在阴影中，透射率也会降低（被物体遮挡）
            accumulated_transmission *= 0.95;
        }
        
        // 如果透射率太低，提前退出
        if (accumulated_transmission < 0.01) {
            break;
        }
    }
    
    return accumulated_light.clamp();
}

// 保存PPM图片
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
    file.close();
}

// 主渲染函数
void render_scene(const std::string& filename, bool use_volumetric) {
    std::cout << "\n📸 渲染" << (use_volumetric ? "【体积光】" : "【普通光照】") << std::endl;
    
    Scene scene;
    Vec3 camera_pos(-1.0, 1.0, 3.0);  // 相机位置
    Vec3 look_at(0.0, 0.5, -2.0);     // 看向场景中央
    Vec3 up(0, 1, 0);
    
    // 构建相机坐标系
    Vec3 forward = (look_at - camera_pos).normalize();
    Vec3 right = forward.cross(up).normalize();
    Vec3 camera_up = right.cross(forward).normalize();
    
    double fov = 60.0 * PI / 180.0;
    double aspect = double(WIDTH) / double(HEIGHT);
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 100 == 0) {
            std::cout << "  进度: " << (100.0 * j / HEIGHT) << "%" << std::endl;
        }
        
        for (int i = 0; i < WIDTH; i++) {
            double u = (2.0 * (i + 0.5) / WIDTH - 1.0) * aspect * std::tan(fov / 2.0);
            double v = (2.0 * (j + 0.5) / HEIGHT - 1.0) * std::tan(fov / 2.0);
            
            Vec3 ray_dir = (forward + right * u + camera_up * v).normalize();
            Ray ray(camera_pos, ray_dir);
            
            // 场景求交
            double t;
            Vec3 surface_color, normal;
            Vec3 final_color(0, 0, 0);
            
            if (scene.intersect(ray, t, surface_color, normal)) {
                // 击中物体，计算表面光照
                Vec3 hit_point = ray.at(t);
                Vec3 view_dir = (camera_pos - hit_point).normalize();
                final_color = phong_shading(hit_point, normal, view_dir, 
                                           scene.light_pos, scene.light_color, surface_color);
                
                // 添加体积光（从相机到表面的路径）
                if (use_volumetric) {
                    Vec3 volumetric = volumetric_lighting(ray, scene, t);
                    final_color = final_color + volumetric;
                }
            } else {
                // 未击中物体，只渲染体积光
                if (use_volumetric) {
                    final_color = volumetric_lighting(ray, scene, 20.0);
                } else {
                    // 背景色（渐变天空）
                    double gradient = 0.5 * (ray_dir.y + 1.0);
                    final_color = Vec3(0.3, 0.4, 0.6) * gradient + Vec3(0.1, 0.1, 0.15) * (1.0 - gradient);
                }
            }
            
            pixels[j * WIDTH + i] = final_color.clamp();
        }
    }
    
    save_ppm(filename, pixels, WIDTH, HEIGHT);
    std::cout << "✅ 已保存: " << filename << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  体积光渲染（Volumetric Lighting）" << std::endl;
    std::cout << "========================================" << std::endl;
    
    render_scene("no_volumetric.ppm", false);
    render_scene("with_volumetric.ppm", true);
    
    std::cout << "\n🎉 渲染完成！" << std::endl;
    std::cout << "  no_volumetric.ppm   - 普通光照" << std::endl;
    std::cout << "  with_volumetric.ppm - 体积光效果" << std::endl;
    
    return 0;
}
