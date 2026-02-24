// 简化测试：只渲染右球，展示极端视差效果
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>

const int WIDTH = 400;
const int HEIGHT = 400;
const double PI = 3.14159265358979323846;

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
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { return *this / length(); }
    Vec3 clamp(double mn = 0.0, double mx = 1.0) const {
        return Vec3(std::max(mn, std::min(mx, x)),
                    std::max(mn, std::min(mx, y)),
                    std::max(mn, std::min(mx, z)));
    }
};

// 棋盘格纹理（简单对比）
Vec3 checkerboard(double u, double v, double& height) {
    int iu = int(u * 8.0);
    int iv = int(v * 8.0);
    bool is_white = ((iu + iv) % 2) == 0;
    
    if (is_white) {
        height = 0.5;  // 白色方格凸起
        return Vec3(0.9, 0.9, 0.9);
    } else {
        height = 0.0;  // 黑色方格凹陷
        return Vec3(0.1, 0.1, 0.1);
    }
}

int main() {
    std::cout << "测试极端视差贴图效果..." << std::endl;
    
    std::vector<Vec3> pixels(WIDTH * HEIGHT);
    
    for (int j = 0; j < HEIGHT; j++) {
        for (int i = 0; i < WIDTH; i++) {
            double u = double(i) / WIDTH;
            double v = double(j) / HEIGHT;
            
            // 模拟视线方向（简化：直接使用UV）
            double view_angle_x = (u - 0.5) * 2.0;
            double view_angle_y = (v - 0.5) * 2.0;
            
            // 视差偏移（极端）
            double parallax_scale = 0.5;  // 超大偏移
            double sample_u = u;
            double sample_v = v;
            
            if (i < WIDTH / 2) {
                // 左半边：普通纹理
                double height;
                pixels[j * WIDTH + i] = checkerboard(sample_u, sample_v, height);
            } else {
                // 右半边：视差贴图 - 根据角度偏移UV
                sample_u -= view_angle_x * parallax_scale;
                sample_v -= view_angle_y * parallax_scale;
                
                double height;
                pixels[j * WIDTH + i] = checkerboard(sample_u, sample_v, height);
            }
        }
    }
    
    // 保存PPM
    std::ofstream file("test_output.ppm");
    file << "P3\n" << WIDTH << " " << HEIGHT << "\n255\n";
    for (int j = HEIGHT - 1; j >= 0; j--) {
        for (int i = 0; i < WIDTH; i++) {
            Vec3 c = pixels[j * WIDTH + i];
            file << int(255 * c.x) << " " << int(255 * c.y) << " " << int(255 * c.z) << "\n";
        }
    }
    file.close();
    
    std::cout << "✅ 测试图片已保存" << std::endl;
    return 0;
}
