#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

const int WIDTH = 800;
const int HEIGHT = 600;

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double t) const { return Vec3(x*t, y*t, z*t); }
    Vec3 operator/(double t) const { return Vec3(x/t, y/t, z/t); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 normalize() const { double l = std::sqrt(x*x+y*y+z*z); return *this / l; }
    Vec3 clamp(double min=0.0, double max=1.0) const {
        return Vec3(std::max(min, std::min(max, x)),
                    std::max(min, std::min(max, y)),
                    std::max(min, std::min(max, z)));
    }
};

struct Vec2 {
    double x, y;
    Vec2(double x=0, double y=0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
    Vec2 operator*(double t) const { return Vec2(x*t, y*t); }
    Vec2 operator/(double t) const { return Vec2(x/t, y/t); }
};

// 砖块深度图（LearnOpenGL标准：灰浆深度小，砖块深度大）
double brick_depth(double u, double v) {
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    u = u - std::floor(u);  // 循环纹理
    v = v - std::floor(v);
    
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    return is_mortar ? 0.2 : 1.0;  // 灰浆0.2，砖块1.0
}

Vec3 brick_color(double u, double v) {
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    u = u - std::floor(u);
    v = v - std::floor(v);
    
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    if (is_mortar) {
        return Vec3(0.5, 0.5, 0.5);  // 灰色灰浆
    } else {
        double noise = std::sin(u * 100.0) * std::cos(v * 100.0) * 0.1;
        return Vec3(0.7 + noise, 0.3 + noise*0.5, 0.2);  // 红褐色砖块
    }
}

// Parallax Occlusion Mapping (完全按LearnOpenGL实现)
Vec2 parallax_mapping(const Vec2& tex_coords, const Vec3& view_dir_tangent) {
    const double height_scale = 0.1;
    
    // 动态层数
    const double min_layers = 8.0;
    const double max_layers = 32.0;
    double num_layers = min_layers + (max_layers - min_layers) * (1.0 - std::abs(view_dir_tangent.z));
    
    double layer_depth = 1.0 / num_layers;
    double current_layer_depth = 0.0;
    
    // P向量
    Vec2 P = Vec2(view_dir_tangent.x, view_dir_tangent.y) / view_dir_tangent.z * height_scale;
    Vec2 delta_tex_coords = P / num_layers;
    
    Vec2 current_tex_coords = tex_coords;
    double current_depth_map_value = brick_depth(current_tex_coords.x, current_tex_coords.y);
    
    // Steep Parallax Mapping
    while (current_layer_depth < current_depth_map_value) {
        current_tex_coords = current_tex_coords - delta_tex_coords;
        current_depth_map_value = brick_depth(current_tex_coords.x, current_tex_coords.y);
        current_layer_depth += layer_depth;
    }
    
    // Parallax Occlusion Mapping - 插值
    Vec2 prev_tex_coords = current_tex_coords + delta_tex_coords;
    
    double after_depth = current_depth_map_value - current_layer_depth;
    double before_depth = brick_depth(prev_tex_coords.x, prev_tex_coords.y) - current_layer_depth + layer_depth;
    
    double weight = after_depth / (after_depth - before_depth);
    Vec2 final_tex_coords = prev_tex_coords * weight + current_tex_coords * (1.0 - weight);
    
    return final_tex_coords;
}

void save_ppm(const std::string& filename, const std::vector<Vec3>& pixels, int w, int h) {
    std::ofstream file(filename);
    file << "P3\n" << w << " " << h << "\n255\n";
    for (int j = h-1; j >= 0; j--) {
        for (int i = 0; i < w; i++) {
            Vec3 c = pixels[j*w + i];
            file << int(255.99*c.x) << " " << int(255.99*c.y) << " " << int(255.99*c.z) << "\n";
        }
    }
}

int main() {
    std::cout << "渲染平面视差贴图（LearnOpenGL标准实现）" << std::endl;
    
    // 定义一个平面，法线指向Z正方向，覆盖屏幕中央
    Vec3 light_pos(1.0, 1.0, 2.0);
    Vec3 view_pos(0.0, 0.0, 5.0);
    
    // 切线空间基向量（平面朝向+Z，T=+X，B=+Y，N=+Z）
    Vec3 T(1, 0, 0);
    Vec3 B(0, 1, 0);
    Vec3 N(0, 0, 1);
    
    std::vector<Vec3> normal_img(WIDTH * HEIGHT);
    std::vector<Vec3> parallax_img(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        if (j % 100 == 0) std::cout << "  进度: " << (100*j/HEIGHT) << "%" << std::endl;
        
        for (int i = 0; i < WIDTH; i++) {
            // 归一化屏幕坐标 [-1, 1]
            double x = (i / double(WIDTH) - 0.5) * 2.0;
            double y = (j / double(HEIGHT) - 0.5) * 2.0;
            
            // 限制在平面范围内
            if (std::abs(x) > 1.0 || std::abs(y) > 1.0) {
                normal_img[j*WIDTH + i] = Vec3(0.2, 0.2, 0.2);
                parallax_img[j*WIDTH + i] = Vec3(0.2, 0.2, 0.2);
                continue;
            }
            
            // 平面上的点
            Vec3 frag_pos(x, y, 0);
            
            // UV坐标（平面映射）
            double u = (x + 1.0) * 0.5 * 2.0;  // 重复2次
            double v = (y + 1.0) * 0.5 * 2.0;
            
            // 视线方向（世界空间）
            Vec3 view_dir = (view_pos - frag_pos).normalize();
            
            // 转换到切线空间
            Vec3 view_tangent(view_dir.dot(T), view_dir.dot(B), view_dir.dot(N));
            
            // 渲染普通纹理
            normal_img[j*WIDTH + i] = brick_color(u, v);
            
            // 渲染视差纹理
            Vec2 parallax_uv = parallax_mapping(Vec2(u, v), view_tangent);
            parallax_img[j*WIDTH + i] = brick_color(parallax_uv.x, parallax_uv.y);
        }
    }
    
    save_ppm("plane_normal.ppm", normal_img, WIDTH, HEIGHT);
    save_ppm("plane_parallax.ppm", parallax_img, WIDTH, HEIGHT);
    
    std::cout << "✅ 渲染完成！" << std::endl;
    std::cout << "  plane_normal.ppm - 普通纹理" << std::endl;
    std::cout << "  plane_parallax.ppm - 视差贴图" << std::endl;
    
    return 0;
}
