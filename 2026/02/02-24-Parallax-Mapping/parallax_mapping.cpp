// Parallax Mapping (视差贴图) - 基于高度图的纹理坐标偏移
// 实现简单视差贴图，对比普通纹理映射效果

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>

// 定义常量
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
    
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(double t) const { return Vec2(x * t, y * t); }
};

// 光线类
struct Ray {
    Vec3 origin;
    Vec3 direction;
    
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d.normalize()) {}
    
    Vec3 at(double t) const { return origin + direction * t; }
};

// 球体类
struct Sphere {
    Vec3 center;
    double radius;
    
    Sphere(const Vec3& c, double r) : center(c), radius(r) {}
    
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
    
    // 获取球体表面法线
    Vec3 getNormal(const Vec3& point) const {
        return (point - center).normalize();
    }
    
    // 获取球体UV坐标
    void getUV(const Vec3& point, double& u, double& v) const {
        Vec3 p = (point - center).normalize();
        u = 0.5 + std::atan2(p.z, p.x) / (2.0 * PI);
        v = 0.5 - std::asin(p.y) / PI;
    }
    
    // 获取TBN矩阵（切线空间）
    void getTBN(const Vec3& point, Vec3& T, Vec3& B, Vec3& N) const {
        N = getNormal(point);
        
        // 计算切线
        Vec3 up = std::abs(N.y) < 0.999 ? Vec3(0, 1, 0) : Vec3(1, 0, 0);
        T = up.cross(N).normalize();
        B = N.cross(T).normalize();
    }
};

// 程序化纹理：砖块纹理（带高度信息）
Vec3 brickTexture(double u, double v, double& height) {
    // 砖块尺寸
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    // 交错排列
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    // 判断是砖块还是灰浆
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    if (is_mortar) {
        height = 0.0; // 灰浆深度为0
        return Vec3(0.5, 0.5, 0.5); // 灰色灰浆
    } else {
        height = 0.3; // 增大砖块凸起高度（原来0.1）
        // 砖块颜色（红褐色，带一些变化）
        double noise = std::sin(u * 100.0) * std::cos(v * 100.0) * 0.1;
        return Vec3(0.7 + noise, 0.3 + noise * 0.5, 0.2);
    }
}

// Phong光照模型
Vec3 phong_shading(const Vec3& normal, const Vec3& view_dir, const Vec3& light_dir, 
                   const Vec3& diffuse_color, double shininess = 32.0) {
    // 环境光
    Vec3 ambient = diffuse_color * 0.2;
    
    // 漫反射
    double diff = std::max(0.0, normal.dot(light_dir));
    Vec3 diffuse = diffuse_color * diff;
    
    // 镜面反射
    Vec3 reflect_dir = (normal * 2.0 * normal.dot(light_dir) - light_dir).normalize();
    double spec = std::pow(std::max(0.0, view_dir.dot(reflect_dir)), shininess);
    Vec3 specular = Vec3(1.0, 1.0, 1.0) * spec * 0.5;
    
    return (ambient + diffuse + specular).clamp();
}

// 简单视差贴图
Vec3 parallax_mapping(const Vec3& point, const Sphere& sphere, const Vec3& view_dir, const Vec3& light_dir, bool use_parallax) {
    // 获取UV坐标
    double u, v;
    sphere.getUV(point, u, v);
    
    // 获取TBN矩阵
    Vec3 T, B, N;
    sphere.getTBN(point, T, B, N);
    
    // 如果使用视差贴图，偏移UV坐标
    if (use_parallax) {
        // 将视线方向转换到切线空间
        Vec3 view_tangent = Vec3(view_dir.dot(T), view_dir.dot(B), view_dir.dot(N));
        
        // Steep Parallax Mapping - 沿视线方向分层采样
        const int num_layers = 32;  // 增加采样层数
        double layer_depth = 1.0 / num_layers;
        double current_depth = 0.0;
        
        double parallax_scale = 0.3;  // 进一步增大视差强度
        Vec2 delta_uv = Vec2(view_tangent.x / view_tangent.z * parallax_scale,
                             view_tangent.y / view_tangent.z * parallax_scale);
        Vec2 current_uv = Vec2(u, v);
        
        // 沿着视线方向步进，直到找到高度匹配的点
        double current_height;
        brickTexture(current_uv.x, current_uv.y, current_height);
        
        while (current_depth < current_height && current_depth < 1.0) {
            current_uv = current_uv - delta_uv * layer_depth;
            brickTexture(current_uv.x, current_uv.y, current_height);
            current_depth += layer_depth;
        }
        
        u = current_uv.x;
        v = current_uv.y;
        
        // 确保UV在[0,1]范围内（可以重复）
        u = u - std::floor(u);
        v = v - std::floor(v);
    }
    
    // 采样纹理颜色
    double height;
    Vec3 tex_color = brickTexture(u, v, height);
    
    // Phong光照（使用表面法线）
    Vec3 color = phong_shading(N, view_dir, light_dir, tex_color);
    
    return color;
}

// 保存为PPM图像
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

int main() {
    std::cout << "开始渲染 Parallax Mapping 效果对比..." << std::endl;
    
    // 场景设置：同一个球体，左右半边对比
    Sphere sphere(Vec3(0, 0, -3), 1.0);  // 单个球体，居中
    Vec3 light_dir = Vec3(0.0, 0.0, 1.0).normalize();  // 正面光照
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 50 == 0) {
            std::cout << "渲染进度: " << (100.0 * j / HEIGHT) << "%" << std::endl;
        }
        
        for (int i = 0; i < WIDTH; i++) {
            // 归一化屏幕坐标
            double u = (i + 0.5) / WIDTH;
            double v = (j + 0.5) / HEIGHT;
            
            // 计算射线方向
            double aspect = double(WIDTH) / double(HEIGHT);
            double x = (2.0 * u - 1.0) * aspect;
            double y = 2.0 * v - 1.0;
            
            Vec3 ray_dir = Vec3(x, y, -1.0).normalize();
            Ray ray(Vec3(0, 0, 0), ray_dir);
            
            // 检查球体交点
            double t;
            bool hit = sphere.intersect(ray, t);
            
            Vec3 color;
            if (hit) {
                Vec3 hit_point = ray.at(t);
                Vec3 view_dir = (ray.origin - hit_point).normalize();
                
                // 左半边不使用视差，右半边使用视差
                bool use_parallax = (i >= WIDTH / 2);
                color = parallax_mapping(hit_point, sphere, view_dir, light_dir, use_parallax);
            } else {
                // 背景色
                double gradient = 0.5 * (ray_dir.y + 1.0);
                color = Vec3(0.5, 0.7, 1.0) * gradient + Vec3(1.0, 1.0, 1.0) * (1.0 - gradient);
            }
            
            pixels[j * WIDTH + i] = color;
        }
    }
    
    std::cout << "渲染完成，保存图片..." << std::endl;
    save_ppm("parallax_output.ppm", pixels, WIDTH, HEIGHT);
    
    std::cout << "✅ 图片已保存: parallax_output.ppm" << std::endl;
    std::cout << "左半边：普通纹理映射" << std::endl;
    std::cout << "右半边：Parallax Mapping（视差贴图）" << std::endl;
    
    return 0;
}
