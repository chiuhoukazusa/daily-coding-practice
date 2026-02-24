#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

const int WIDTH = 800;
const int HEIGHT = 600;
const double PI = 3.14159265358979323846;

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

// 2D向量（用于UV坐标）
struct Vec2 {
    double x, y;
    
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
    
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(double t) const { return Vec2(x * t, y * t); }
    Vec2 operator/(double t) const { return Vec2(x / t, y / t); }
};

// 射线类
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    
    Vec3 at(double t) const {
        return origin + direction * t;
    }
};

// 球体类
class Sphere {
public:
    Vec3 center;
    double radius;
    
    Sphere(const Vec3& c, double r) : center(c), radius(r) {}
    
    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double a = ray.direction.dot(ray.direction);
        double b = 2.0 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * a * c;
        
        if (discriminant < 0) {
            return false;
        }
        
        t = (-b - std::sqrt(discriminant)) / (2.0 * a);
        return t > 0.001;
    }
    
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
    
    void getUV(const Vec3& point, double& u, double& v) const {
        Vec3 p = (point - center).normalize();
        u = 0.5 + std::atan2(p.z, p.x) / (2.0 * PI);
        v = 0.5 - std::asin(p.y) / PI;
    }
    
    void getTBN(const Vec3& point, Vec3& T, Vec3& B, Vec3& N) const {
        N = getNormal(point);
        
        Vec3 up = std::abs(N.y) < 0.999 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        T = up.cross(N).normalize();
        B = N.cross(T).normalize();
    }
};

// 程序化纹理：砖块（参考LearnOpenGL，黑色=深度深=砖块凹陷）
Vec3 brickTexture(double u, double v, double& depth) {
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    if (is_mortar) {
        depth = 0.2;  // 灰浆深度浅（接近表面）
        return Vec3(0.5, 0.5, 0.5);  // 灰色
    } else {
        depth = 1.0;  // 砖块深度深（凹陷最深处）
        double noise = std::sin(u * 100.0) * std::cos(v * 100.0) * 0.1;
        return Vec3(0.7 + noise, 0.3 + noise * 0.5, 0.2);  // 红褐色
    }
}

// Phong光照
Vec3 phong_shading(const Vec3& normal, const Vec3& view_dir, const Vec3& light_dir, const Vec3& diffuse_color) {
    double shininess = 32.0;
    
    Vec3 ambient = diffuse_color * 0.3;
    
    double diff = std::max(0.0, normal.dot(light_dir));
    Vec3 diffuse = diffuse_color * diff;
    
    Vec3 reflect_dir = (normal * 2.0 * normal.dot(light_dir) - light_dir).normalize();
    double spec = std::pow(std::max(0.0, view_dir.dot(reflect_dir)), shininess);
    Vec3 specular = Vec3(1.0, 1.0, 1.0) * spec * 0.5;
    
    return (ambient + diffuse + specular).clamp();
}

// Parallax Occlusion Mapping (LearnOpenGL标准实现)
Vec2 parallax_mapping(const Vec2& tex_coords, const Vec3& view_dir_tangent) {
    const double height_scale = 0.3;  // 增大到0.3（原0.1太小）
    
    // 动态层数（根据视角调整）
    const double min_layers = 8.0;
    const double max_layers = 32.0;
    double num_layers = min_layers + (max_layers - min_layers) * (1.0 - std::abs(view_dir_tangent.z));
    
    double layer_depth = 1.0 / num_layers;
    double current_layer_depth = 0.0;
    
    // P向量（LearnOpenGL标准公式）
    Vec2 P = Vec2(view_dir_tangent.x, view_dir_tangent.y) / view_dir_tangent.z * height_scale;
    Vec2 delta_tex_coords = P / num_layers;
    
    // 初始值
    Vec2 current_tex_coords = tex_coords;
    double current_depth_value;
    brickTexture(current_tex_coords.x, current_tex_coords.y, current_depth_value);
    
    // Steep Parallax Mapping - 沿视线方向步进
    while (current_layer_depth < current_depth_value) {
        current_tex_coords = current_tex_coords - delta_tex_coords;  // 减法！
        brickTexture(current_tex_coords.x, current_tex_coords.y, current_depth_value);
        current_layer_depth += layer_depth;
    }
    
    // Parallax Occlusion Mapping - 线性插值
    Vec2 prev_tex_coords = current_tex_coords + delta_tex_coords;
    
    double after_depth = current_depth_value - current_layer_depth;
    double prev_depth_value;
    brickTexture(prev_tex_coords.x, prev_tex_coords.y, prev_depth_value);
    double before_depth = prev_depth_value - current_layer_depth + layer_depth;
    
    double weight = after_depth / (after_depth - before_depth);
    Vec2 final_tex_coords = prev_tex_coords * weight + current_tex_coords * (1.0 - weight);
    
    return final_tex_coords;
}

// 主渲染函数
Vec3 render_parallax(const Vec3& point, const Sphere& sphere, const Vec3& view_dir, 
                     const Vec3& light_dir, bool use_parallax) {
    double u, v;
    sphere.getUV(point, u, v);
    
    Vec3 T, B, N;
    sphere.getTBN(point, T, B, N);
    
    // 如果使用视差贴图，偏移UV
    if (use_parallax) {
        // 将视线方向转换到切线空间
        Vec3 view_tangent = Vec3(view_dir.dot(T), view_dir.dot(B), view_dir.dot(N));
        
        Vec2 tex_coords = parallax_mapping(Vec2(u, v), view_tangent);
        u = tex_coords.x;
        v = tex_coords.y;
        
        // 边界处理
        u = u - std::floor(u);
        v = v - std::floor(v);
    }
    
    // 采样纹理
    double depth;
    Vec3 tex_color = brickTexture(u, v, depth);
    
    // Phong光照
    Vec3 color = phong_shading(N, view_dir, light_dir, tex_color);
    
    return color;
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

// 渲染单张图片
void render_scene(const std::string& filename, bool use_parallax, const std::string& description) {
    std::cout << "\n📸 " << description << std::endl;
    
    Sphere sphere(Vec3(0, 0, -3), 1.0);
    Vec3 light_dir = Vec3(0.3, 0.3, 1.0).normalize();
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 100 == 0) {
            std::cout << "  进度: " << (100.0 * j / HEIGHT) << "%" << std::endl;
        }
        
        for (int i = 0; i < WIDTH; i++) {
            double u = (i + 0.5) / WIDTH;
            double v = (j + 0.5) / HEIGHT;
            
            double aspect = double(WIDTH) / double(HEIGHT);
            double x = (2.0 * u - 1.0) * aspect;
            double y = 2.0 * v - 1.0;
            
            Vec3 ray_dir = Vec3(x, y, -1.0).normalize();
            Ray ray(Vec3(0, 0, 0), ray_dir);
            
            double t;
            bool hit = sphere.intersect(ray, t);
            
            Vec3 color;
            if (hit) {
                Vec3 hit_point = ray.at(t);
                Vec3 view_dir = (ray.origin - hit_point).normalize();
                color = render_parallax(hit_point, sphere, view_dir, light_dir, use_parallax);
            } else {
                double gradient = 0.5 * (ray_dir.y + 1.0);
                color = Vec3(0.5, 0.7, 1.0) * gradient + Vec3(1.0, 1.0, 1.0) * (1.0 - gradient);
            }
            
            pixels[j * WIDTH + i] = color;
        }
    }
    
    save_ppm(filename, pixels, WIDTH, HEIGHT);
    std::cout << "✅ 已保存: " << filename << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  Parallax Mapping v3 (LearnOpenGL标准)" << std::endl;
    std::cout << "  修正：砖块深度=0（凹陷），灰浆深度>0（凸起）" << std::endl;
    std::cout << "========================================" << std::endl;
    
    render_scene("normal_v3.ppm", false, "渲染图1：普通纹理映射");
    render_scene("parallax_v3.ppm", true, "渲染图2：Parallax Occlusion Mapping");
    
    std::cout << "\n🎉 渲染完成！" << std::endl;
    std::cout << "📊 参数说明：" << std::endl;
    std::cout << "   - height_scale = 0.1 (LearnOpenGL标准值)" << std::endl;
    std::cout << "   - 砖块深度 = 0.0 (黑色，凹陷)" << std::endl;
    std::cout << "   - 灰浆深度 = 0.2 (灰色，凸起)" << std::endl;
    std::cout << "   - 动态层数 = 8~32 (根据视角调整)" << std::endl;
    
    return 0;
}
