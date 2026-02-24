// 测试：打印一个像素的视差计算过程
#include <iostream>
#include <cmath>

struct Vec2 {
    double x, y;
    Vec2(double x=0, double y=0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x+v.x, y+v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x-v.x, y-v.y); }
    Vec2 operator*(double t) const { return Vec2(x*t, y*t); }
    Vec2 operator/(double t) const { return Vec2(x/t, y/t); }
};

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
};

double brick_depth(double u, double v) {
    const double brick_width = 0.3;
    const double brick_height = 0.15;
    const double mortar_width = 0.02;
    
    double row = std::floor(v / brick_height);
    double offset = (int(row) % 2) * brick_width * 0.5;
    
    double x = std::fmod(u + offset, brick_width);
    double y = std::fmod(v, brick_height);
    
    bool is_mortar = (x < mortar_width || x > brick_width - mortar_width ||
                      y < mortar_width || y > brick_height - mortar_width);
    
    return is_mortar ? 0.8 : 0.0;  // 灰浆0.8, 砖块0.0
}

int main() {
    // 模拟一个球面上的点，看视差是否计算
    Vec2 tex_coords(0.5, 0.5);  // 球心位置
    Vec3 view_dir_tangent(0.3, 0.2, 0.8);  // 斜向视角（切线空间）
    
    const double height_scale = 0.3;
    const int num_layers = 16;
    
    std::cout << "==========  视差计算调试 ==========" << std::endl;
    std::cout << "初始UV: (" << tex_coords.x << ", " << tex_coords.y << ")" << std::endl;
    std::cout << "视线方向（切线空间）: (" << view_dir_tangent.x << ", " 
              << view_dir_tangent.y << ", " << view_dir_tangent.z << ")" << std::endl;
    
    // 计算P向量
    Vec2 P = Vec2(view_dir_tangent.x, view_dir_tangent.y) / view_dir_tangent.z * height_scale;
    std::cout << "P向量: (" << P.x << ", " << P.y << ")" << std::endl;
    std::cout << "P长度: " << std::sqrt(P.x*P.x + P.y*P.y) << std::endl;
    
    Vec2 delta = P / num_layers;
    std::cout << "每层偏移: (" << delta.x << ", " << delta.y << ")" << std::endl;
    
    double layer_depth = 1.0 / num_layers;
    double current_layer_depth = 0.0;
    Vec2 current_coords = tex_coords;
    
    std::cout << "\n步进过程:" << std::endl;
    for (int i = 0; i < num_layers; i++) {
        double depth_value = brick_depth(current_coords.x, current_coords.y);
        
        if (i < 5 || i == num_layers-1) {  // 只打印前5层和最后一层
            std::cout << "  Layer " << i << ": UV(" << current_coords.x << ", " << current_coords.y 
                      << ") depth=" << depth_value << " layer_depth=" << current_layer_depth << std::endl;
        }
        
        if (current_layer_depth >= depth_value) {
            std::cout << "\n碰撞发生在第 " << i << " 层!" << std::endl;
            std::cout << "最终UV: (" << current_coords.x << ", " << current_coords.y << ")" << std::endl;
            std::cout << "UV偏移量: (" << (current_coords.x - tex_coords.x) << ", " 
                      << (current_coords.y - tex_coords.y) << ")" << std::endl;
            return 0;
        }
        
        current_coords = current_coords - delta;
        current_layer_depth += layer_depth;
    }
    
    std::cout << "\n未碰撞（视差太弱或深度图全0）" << std::endl;
    std::cout << "最终UV: (" << current_coords.x << ", " << current_coords.y << ")" << std::endl;
    
    return 0;
}
