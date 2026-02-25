#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

// 简单的 Vec2/Vec3 类
struct Vec2 {
    float x, y;
    Vec2(float x = 0, float y = 0) : x(x), y(y) {}
};

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator*(float t) const { return Vec3(x*t, y*t, z*t); }
};

// 颜色类
struct Color {
    unsigned char r, g, b;
    Color(unsigned char r = 0, unsigned char g = 0, unsigned char b = 0) 
        : r(r), g(g), b(b) {}
};

// 三角形顶点
struct Vertex {
    Vec2 pos;  // 屏幕坐标
    float z;   // 深度值
    Color color;
    
    Vertex(Vec2 p, float z, Color c) : pos(p), z(z), color(c) {}
};

// 计算重心坐标
Vec3 barycentric(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c) {
    Vec2 v0(c.x - a.x, c.y - a.y);
    Vec2 v1(b.x - a.x, b.y - a.y);
    Vec2 v2(p.x - a.x, p.y - a.y);
    
    float den = v0.x * v1.y - v1.x * v0.y;
    if (std::abs(den) < 1e-6) return Vec3(-1, 1, 1); // 退化三角形
    
    float v = (v2.x * v1.y - v1.x * v2.y) / den;
    float w = (v0.x * v2.y - v2.x * v0.y) / den;
    float u = 1.0f - v - w;
    
    return Vec3(u, v, w);
}

// 光栅化三角形
void rasterizeTriangle(
    const Vertex& v0, const Vertex& v1, const Vertex& v2,
    std::vector<Color>& framebuffer,
    std::vector<float>& zbuffer,
    int width, int height
) {
    // 计算边界框
    float minX = std::min({v0.pos.x, v1.pos.x, v2.pos.x});
    float maxX = std::max({v0.pos.x, v1.pos.x, v2.pos.x});
    float minY = std::min({v0.pos.y, v1.pos.y, v2.pos.y});
    float maxY = std::max({v0.pos.y, v1.pos.y, v2.pos.y});
    
    // 裁剪到屏幕范围
    int x0 = std::max(0, (int)minX);
    int x1 = std::min(width - 1, (int)maxX);
    int y0 = std::max(0, (int)minY);
    int y1 = std::min(height - 1, (int)maxY);
    
    // 遍历边界框内的所有像素
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            Vec2 p(x + 0.5f, y + 0.5f); // 像素中心
            Vec3 bc = barycentric(p, v0.pos, v1.pos, v2.pos);
            
            // 检查点是否在三角形内
            if (bc.x < 0 || bc.y < 0 || bc.z < 0) continue;
            
            // 插值深度
            float z = bc.x * v0.z + bc.y * v1.z + bc.z * v2.z;
            
            // 深度测试
            int idx = y * width + x;
            if (z < zbuffer[idx]) {
                zbuffer[idx] = z;
                
                // 插值颜色
                float r = bc.x * v0.color.r + bc.y * v1.color.r + bc.z * v2.color.r;
                float g = bc.x * v0.color.g + bc.y * v1.color.g + bc.z * v2.color.g;
                float b = bc.x * v0.color.b + bc.y * v1.color.b + bc.z * v2.color.b;
                
                framebuffer[idx] = Color(
                    (unsigned char)r,
                    (unsigned char)g,
                    (unsigned char)b
                );
            }
        }
    }
}

// 保存为 PPM 图片
void savePPM(const std::string& filename, const std::vector<Color>& framebuffer, int width, int height) {
    std::ofstream file(filename, std::ios::binary);
    file << "P6\n" << width << " " << height << "\n255\n";
    for (const auto& color : framebuffer) {
        file << color.r << color.g << color.b;
    }
}

int main() {
    const int width = 800;
    const int height = 600;
    
    // 初始化帧缓冲和深度缓冲
    std::vector<Color> framebuffer(width * height, Color(30, 30, 40));
    std::vector<float> zbuffer(width * height, std::numeric_limits<float>::max());
    
    // 定义三个三角形（不同颜色渐变）
    
    // 三角形1: 红色渐变
    Vertex tri1_v0(Vec2(100, 100), 0.5f, Color(255, 0, 0));
    Vertex tri1_v1(Vec2(300, 150), 0.5f, Color(255, 100, 100));
    Vertex tri1_v2(Vec2(200, 300), 0.5f, Color(200, 50, 50));
    
    // 三角形2: 绿色渐变 (稍微靠后)
    Vertex tri2_v0(Vec2(400, 100), 0.6f, Color(0, 255, 0));
    Vertex tri2_v1(Vec2(600, 150), 0.6f, Color(100, 255, 100));
    Vertex tri2_v2(Vec2(500, 300), 0.6f, Color(50, 200, 50));
    
    // 三角形3: 蓝色渐变 (最前面，与前两个重叠)
    Vertex tri3_v0(Vec2(250, 200), 0.3f, Color(0, 0, 255));
    Vertex tri3_v1(Vec2(550, 200), 0.3f, Color(100, 100, 255));
    Vertex tri3_v2(Vec2(400, 500), 0.3f, Color(50, 50, 200));
    
    // 光栅化三个三角形
    std::cout << "Rasterizing triangles...\n";
    rasterizeTriangle(tri1_v0, tri1_v1, tri1_v2, framebuffer, zbuffer, width, height);
    rasterizeTriangle(tri2_v0, tri2_v1, tri2_v2, framebuffer, zbuffer, width, height);
    rasterizeTriangle(tri3_v0, tri3_v1, tri3_v2, framebuffer, zbuffer, width, height);
    
    // 保存图片
    savePPM("rasterization_output.ppm", framebuffer, width, height);
    std::cout << "✅ 渲染完成: rasterization_output.ppm\n";
    
    // 转换为 PNG
    std::cout << "Converting to PNG...\n";
    int result = system("convert rasterization_output.ppm rasterization_output.png");
    if (result == 0) {
        std::cout << "✅ PNG 输出: rasterization_output.png\n";
        system("rm rasterization_output.ppm"); // 删除临时 PPM
    } else {
        std::cout << "⚠️ ImageMagick not found, keeping PPM format\n";
    }
    
    return 0;
}
