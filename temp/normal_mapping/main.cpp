#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

// ===== Vector3 Class =====
struct Vec3 {
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    
    // 逐元素乘法（Hadamard product）
    Vec3 mul(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    
    double length() const { return std::sqrt(x * x + y * y + z * z); }
    
    Vec3 normalize() const {
        double len = length();
        return len > 0 ? (*this / len) : Vec3(0, 0, 0);
    }
};

// ===== Ray Class =====
struct Ray {
    Vec3 origin, direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    
    Vec3 at(double t) const { return origin + direction * t; }
};

// ===== Material Class =====
struct Material {
    Vec3 albedo;
    double ka, kd, ks;
    int specular_exp;
    bool use_normal_map;
    
    Material(Vec3 alb = Vec3(0.8, 0.8, 0.8), double ambient = 0.1, double diffuse = 0.7, 
             double specular = 0.5, int exp = 32, bool use_nm = false)
        : albedo(alb), ka(ambient), kd(diffuse), ks(specular), specular_exp(exp), use_normal_map(use_nm) {}
};

// ===== Sphere Class =====
struct Sphere {
    Vec3 center;
    double radius;
    Material material;
    
    Sphere(const Vec3& c, double r, const Material& m) : center(c), radius(r), material(m) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) return false;
        
        double t1 = (-b - std::sqrt(discriminant)) / (2 * a);
        double t2 = (-b + std::sqrt(discriminant)) / (2 * a);
        
        t = (t1 > 0.001) ? t1 : t2;
        return t > 0.001;
    }
    
    Vec3 get_normal(const Vec3& point) const {
        return (point - center).normalize();
    }
    
    // 获取球面 UV 坐标
    void get_uv(const Vec3& point, double& u, double& v) const {
        Vec3 local = (point - center).normalize();
        u = 0.5 + atan2(local.z, local.x) / (2 * M_PI);
        v = 0.5 - asin(local.y) / M_PI;
    }
};

// ===== 程序化法线贴图生成 =====
Vec3 procedural_normal_map(double u, double v) {
    // 生成砖块图案的法线贴图
    const int brick_rows = 6;
    const int brick_cols = 12;
    
    double brick_u = u * brick_cols;
    double brick_v = v * brick_rows;
    
    int row = static_cast<int>(brick_v);
    
    // 交错砖块
    if (row % 2 == 1) {
        brick_u += 0.5;
    }
    
    double local_u = brick_u - std::floor(brick_u);
    double local_v = brick_v - std::floor(brick_v);
    
    // 砖块边界
    const double mortar_width = 0.05;
    bool is_mortar = (local_u < mortar_width || local_u > 1.0 - mortar_width ||
                      local_v < mortar_width || local_v > 1.0 - mortar_width);
    
    Vec3 normal;
    if (is_mortar) {
        // 灰缝区域：向内凹陷
        normal = Vec3(0, 0, -0.3).normalize();
    } else {
        // 砖块区域：添加微小的随机凹凸
        double noise = sin(local_u * 20) * cos(local_v * 20) * 0.1;
        normal = Vec3(noise, noise, 1.0).normalize();
    }
    
    return normal;
}

// ===== 切线空间到世界空间的转换 =====
Vec3 tangent_to_world(const Vec3& tangent_normal, const Vec3& world_normal) {
    // 构建 TBN 矩阵（Tangent, Bitangent, Normal）
    Vec3 N = world_normal;
    
    // 构造切线向量（沿着 x 轴方向）
    Vec3 up = (std::abs(N.y) > 0.999) ? Vec3(1, 0, 0) : Vec3(0, 1, 0);
    Vec3 T = up.cross(N).normalize();
    Vec3 B = N.cross(T);
    
    // 从切线空间转换到世界空间
    // 注意：法线贴图通常存储的是 [0,1] 范围，这里已经是 [-1,1]
    Vec3 world = T * tangent_normal.x + B * tangent_normal.y + N * tangent_normal.z;
    return world.normalize();
}

// ===== Phong 光照模型 =====
Vec3 phong_lighting(const Vec3& point, const Vec3& normal, const Vec3& view_dir, 
                    const Material& mat, const Vec3& light_pos, const Vec3& light_color) {
    // Ambient
    Vec3 ambient = mat.albedo.mul(light_color) * mat.ka;
    
    // Diffuse
    Vec3 light_dir = (light_pos - point).normalize();
    double diff = std::max(0.0, normal.dot(light_dir));
    Vec3 diffuse = mat.albedo.mul(light_color) * (mat.kd * diff);
    
    // Specular
    Vec3 reflect_dir = (normal * (2.0 * normal.dot(light_dir)) - light_dir).normalize();
    double spec = std::pow(std::max(0.0, reflect_dir.dot(view_dir)), mat.specular_exp);
    Vec3 specular = light_color * (mat.ks * spec);
    
    return ambient + diffuse + specular;
}

// ===== 场景渲染 =====
Vec3 trace(const Ray& ray, const std::vector<Sphere>& spheres, const Vec3& light_pos, const Vec3& light_color) {
    double closest_t = std::numeric_limits<double>::max();
    const Sphere* hit_sphere = nullptr;
    
    for (const auto& sphere : spheres) {
        double t;
        if (sphere.intersect(ray, t) && t < closest_t) {
            closest_t = t;
            hit_sphere = &sphere;
        }
    }
    
    if (hit_sphere) {
        Vec3 hit_point = ray.at(closest_t);
        Vec3 geometric_normal = hit_sphere->get_normal(hit_point);
        Vec3 shading_normal = geometric_normal;
        
        // 如果使用法线贴图
        if (hit_sphere->material.use_normal_map) {
            double u, v;
            hit_sphere->get_uv(hit_point, u, v);
            Vec3 tangent_normal = procedural_normal_map(u, v);
            shading_normal = tangent_to_world(tangent_normal, geometric_normal);
        }
        
        Vec3 view_dir = (ray.origin - hit_point).normalize();
        return phong_lighting(hit_point, shading_normal, view_dir, hit_sphere->material, light_pos, light_color);
    }
    
    // 背景渐变
    double t = 0.5 * (ray.direction.y + 1.0);
    return Vec3(1.0, 1.0, 1.0) * (1.0 - t) + Vec3(0.5, 0.7, 1.0) * t;
}

// ===== 输出 PPM 图像 =====
void write_ppm(const std::string& filename, const std::vector<Vec3>& pixels, int width, int height) {
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int j = height - 1; j >= 0; --j) {
        for (int i = 0; i < width; ++i) {
            Vec3 color = pixels[j * width + i];
            int r = static_cast<int>(std::clamp(color.x, 0.0, 1.0) * 255.99);
            int g = static_cast<int>(std::clamp(color.y, 0.0, 1.0) * 255.99);
            int b = static_cast<int>(std::clamp(color.z, 0.0, 1.0) * 255.99);
            file << r << " " << g << " " << b << "\n";
        }
    }
    
    file.close();
    std::cout << "✅ 图像已保存: " << filename << std::endl;
}

// ===== 主程序 =====
int main() {
    const int width = 800;
    const int height = 600;
    const double aspect_ratio = static_cast<double>(width) / height;
    
    // 相机设置
    Vec3 camera_pos(0, 0, 5);
    double viewport_height = 2.0;
    double viewport_width = viewport_height * aspect_ratio;
    double focal_length = 1.0;
    
    Vec3 horizontal(viewport_width, 0, 0);
    Vec3 vertical(0, viewport_height, 0);
    Vec3 lower_left = camera_pos - horizontal / 2 - vertical / 2 - Vec3(0, 0, focal_length);
    
    // 场景设置
    Vec3 light_pos(5, 5, 5);
    Vec3 light_color(1.0, 1.0, 1.0);
    
    std::vector<Sphere> spheres;
    
    // 左侧球体：不使用法线贴图（平滑）
    Material smooth_mat(Vec3(0.8, 0.3, 0.3), 0.1, 0.7, 0.5, 32, false);
    spheres.push_back(Sphere(Vec3(-1.5, 0, 0), 1.0, smooth_mat));
    
    // 右侧球体：使用法线贴图（砖块纹理）
    Material normal_mapped_mat(Vec3(0.8, 0.3, 0.3), 0.1, 0.7, 0.5, 32, true);
    spheres.push_back(Sphere(Vec3(1.5, 0, 0), 1.0, normal_mapped_mat));
    
    // 渲染
    std::vector<Vec3> pixels(width * height);
    
    std::cout << "🎨 开始渲染..." << std::endl;
    
    for (int j = 0; j < height; ++j) {
        if (j % 50 == 0) {
            std::cout << "  进度: " << (100 * j / height) << "%" << std::endl;
        }
        
        for (int i = 0; i < width; ++i) {
            double u = static_cast<double>(i) / (width - 1);
            double v = static_cast<double>(j) / (height - 1);
            
            Vec3 direction = lower_left + horizontal * u + vertical * v - camera_pos;
            Ray ray(camera_pos, direction);
            
            pixels[j * width + i] = trace(ray, spheres, light_pos, light_color);
        }
    }
    
    std::cout << "  进度: 100%" << std::endl;
    
    write_ppm("normal_mapping_output.ppm", pixels, width, height);
    
    std::cout << "\n✅ 渲染完成！" << std::endl;
    std::cout << "📊 对比说明：" << std::endl;
    std::cout << "   - 左侧球体：平滑表面（无法线贴图）" << std::endl;
    std::cout << "   - 右侧球体：砖块纹理（使用法线贴图）" << std::endl;
    std::cout << "   - 法线贴图在不增加几何复杂度的情况下，增加了表面细节" << std::endl;
    
    return 0;
}
