#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// 3D向量结构
struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(float t) const { return Vec3(x * t, y * t, z * t); }
    float dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    Vec3 normalize() const {
        float len = std::sqrt(x * x + y * y + z * z);
        return len > 0 ? Vec3(x / len, y / len, z / len) : Vec3(0, 0, 0);
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
};

// 三角形面结构
struct Triangle {
    int v0, v1, v2;  // 顶点索引
    Triangle(int a, int b, int c) : v0(a), v1(b), v2(c) {}
};

// OBJ模型加载器
class OBJLoader {
public:
    std::vector<Vec3> vertices;
    std::vector<Triangle> faces;
    
    bool load(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;
            
            if (prefix == "v") {
                // 顶点坐标
                float x, y, z;
                iss >> x >> y >> z;
                vertices.push_back(Vec3(x, y, z));
            }
            else if (prefix == "f") {
                // 三角形面（支持格式：f v1 v2 v3 或 f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3）
                std::string v1, v2, v3;
                iss >> v1 >> v2 >> v3;
                
                int idx1 = parseVertexIndex(v1);
                int idx2 = parseVertexIndex(v2);
                int idx3 = parseVertexIndex(v3);
                
                if (idx1 >= 0 && idx2 >= 0 && idx3 >= 0) {
                    faces.push_back(Triangle(idx1, idx2, idx3));
                }
            }
        }
        
        file.close();
        std::cout << "模型加载完成: " << vertices.size() << " 顶点, " 
                  << faces.size() << " 三角形" << std::endl;
        return true;
    }
    
private:
    int parseVertexIndex(const std::string& token) {
        // 解析格式：v 或 v/vt 或 v/vt/vn 或 v//vn
        size_t pos = token.find('/');
        std::string indexStr = (pos == std::string::npos) ? token : token.substr(0, pos);
        int index = std::stoi(indexStr);
        // OBJ索引从1开始，转换为从0开始
        return index - 1;
    }
};

// 简单的线框渲染器
class WireframeRenderer {
public:
    WireframeRenderer(int width, int height) 
        : width(width), height(height), buffer(width * height * 3, 255) {}
    
    void render(const OBJLoader& model) {
        // 计算模型边界
        Vec3 minBound(1e10, 1e10, 1e10);
        Vec3 maxBound(-1e10, -1e10, -1e10);
        for (const auto& v : model.vertices) {
            minBound.x = std::min(minBound.x, v.x);
            minBound.y = std::min(minBound.y, v.y);
            minBound.z = std::min(minBound.z, v.z);
            maxBound.x = std::max(maxBound.x, v.x);
            maxBound.y = std::max(maxBound.y, v.y);
            maxBound.z = std::max(maxBound.z, v.z);
        }
        
        // 计算缩放和平移
        Vec3 center = (minBound + maxBound) * 0.5f;
        Vec3 size = maxBound - minBound;
        float scale = std::min(width, height) * 0.4f / std::max(std::max(size.x, size.y), size.z);
        
        // 渲染所有三角形边缘
        for (const auto& tri : model.faces) {
            Vec3 v0 = model.vertices[tri.v0];
            Vec3 v1 = model.vertices[tri.v1];
            Vec3 v2 = model.vertices[tri.v2];
            
            // 投影到2D（简单正交投影）
            auto project = [&](const Vec3& v) -> std::pair<int, int> {
                float x = (v.x - center.x) * scale + width / 2;
                float y = (v.y - center.y) * scale + height / 2;
                return {(int)x, (int)y};
            };
            
            auto p0 = project(v0);
            auto p1 = project(v1);
            auto p2 = project(v2);
            
            // 绘制三条边
            drawLine(p0.first, p0.second, p1.first, p1.second);
            drawLine(p1.first, p1.second, p2.first, p2.second);
            drawLine(p2.first, p2.second, p0.first, p0.second);
        }
    }
    
    void save(const std::string& filename) {
        stbi_write_png(filename.c_str(), width, height, 3, buffer.data(), width * 3);
        std::cout << "图像已保存: " << filename << std::endl;
    }
    
private:
    int width, height;
    std::vector<unsigned char> buffer;
    
    void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            int idx = (y * width + x) * 3;
            buffer[idx] = r;
            buffer[idx + 1] = g;
            buffer[idx + 2] = b;
        }
    }
    
    // Bresenham直线算法
    void drawLine(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            setPixel(x0, y0, 0, 0, 0);  // 黑色线条
            
            if (x0 == x1 && y0 == y1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
};

// 生成简单的测试OBJ模型（立方体）
void generateCubeOBJ(const std::string& filename) {
    std::ofstream file(filename);
    file << "# Simple Cube OBJ\n";
    file << "# 8 vertices\n";
    file << "v -1.0 -1.0 -1.0\n";
    file << "v  1.0 -1.0 -1.0\n";
    file << "v  1.0  1.0 -1.0\n";
    file << "v -1.0  1.0 -1.0\n";
    file << "v -1.0 -1.0  1.0\n";
    file << "v  1.0 -1.0  1.0\n";
    file << "v  1.0  1.0  1.0\n";
    file << "v -1.0  1.0  1.0\n";
    file << "# 12 triangles (6 faces * 2 triangles)\n";
    file << "f 1 2 3\n";
    file << "f 1 3 4\n";
    file << "f 5 7 6\n";
    file << "f 5 8 7\n";
    file << "f 1 5 6\n";
    file << "f 1 6 2\n";
    file << "f 2 6 7\n";
    file << "f 2 7 3\n";
    file << "f 3 7 8\n";
    file << "f 3 8 4\n";
    file << "f 4 8 5\n";
    file << "f 4 5 1\n";
    file.close();
    std::cout << "测试立方体OBJ已生成: " << filename << std::endl;
}

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;
    
    // 生成测试模型
    std::string objFile = "cube.obj";
    generateCubeOBJ(objFile);
    
    // 加载模型
    OBJLoader loader;
    if (!loader.load(objFile)) {
        std::cerr << "模型加载失败" << std::endl;
        return 1;
    }
    
    // 渲染线框
    WireframeRenderer renderer(WIDTH, HEIGHT);
    renderer.render(loader);
    renderer.save("obj_loader_output.png");
    
    std::cout << "OBJ模型加载器测试完成！" << std::endl;
    return 0;
}
