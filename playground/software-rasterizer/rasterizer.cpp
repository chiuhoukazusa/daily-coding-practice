#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

struct Vec3 {
    double x, y, z;
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const {
        return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    double length() const { return sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const { double l = length(); return Vec3(x/l, y/l, z/l); }
};

struct Vec4 {
    double x, y, z, w;
    Vec4(double x = 0, double y = 0, double z = 0, double w = 1) : x(x), y(y), z(z), w(w) {}
    Vec4(Vec3 v, double w = 1) : x(v.x), y(v.y), z(v.z), w(w) {}
};

struct Mat4 {
    double m[16];
    
    Mat4() { for (int i = 0; i < 16; i++) m[i] = 0; }
    
    static Mat4 identity() {
        Mat4 mat;
        mat.m[0] = mat.m[5] = mat.m[10] = mat.m[15] = 1;
        return mat;
    }
    
    static Mat4 perspective(double fov, double aspect, double near, double far) {
        Mat4 mat;
        double f = 1.0 / tan(fov / 2.0);
        mat.m[0] = f / aspect;
        mat.m[5] = f;
        mat.m[10] = (far + near) / (near - far);
        mat.m[11] = -1;
        mat.m[14] = (2 * far * near) / (near - far);
        return mat;
    }
    
    static Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
        Vec3 f = (center - eye).normalized();
        Vec3 s = f.cross(up).normalized();
        Vec3 u = s.cross(f);
        
        Mat4 mat = identity();
        mat.m[0] = s.x; mat.m[4] = s.y; mat.m[8] = s.z;
        mat.m[1] = u.x; mat.m[5] = u.y; mat.m[9] = u.z;
        mat.m[2] = -f.x; mat.m[6] = -f.y; mat.m[10] = -f.z;
        mat.m[12] = -s.dot(eye);
        mat.m[13] = -u.dot(eye);
        mat.m[14] = f.dot(eye);
        return mat;
    }
    
    Vec4 operator*(const Vec4& v) const {
        return Vec4(
            m[0]*v.x + m[4]*v.y + m[8]*v.z + m[12]*v.w,
            m[1]*v.x + m[5]*v.y + m[9]*v.z + m[13]*v.w,
            m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14]*v.w,
            m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15]*v.w
        );
    }
    
    Mat4 operator*(const Mat4& other) const {
        Mat4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i*4+j] = 0;
                for (int k = 0; k < 4; k++) {
                    result.m[i*4+j] += m[i*4+k] * other.m[k*4+j];
                }
            }
        }
        return result;
    }
};

struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec3 color;
    
    Vertex(Vec3 p, Vec3 n, Vec3 c) : pos(p), normal(n), color(c) {}
};

struct Triangle {
    Vertex v0, v1, v2;
    Triangle(Vertex a, Vertex b, Vertex c) : v0(a), v1(b), v2(c) {}
};

class Rasterizer {
private:
    int width, height;
    std::vector<unsigned char> framebuffer;
    std::vector<double> depthbuffer;
    
    void setPixel(int x, int y, Vec3 color) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        framebuffer[idx] = (unsigned char)(std::clamp(color.x, 0.0, 1.0) * 255);
        framebuffer[idx+1] = (unsigned char)(std::clamp(color.y, 0.0, 1.0) * 255);
        framebuffer[idx+2] = (unsigned char)(std::clamp(color.z, 0.0, 1.0) * 255);
    }
    
    bool depthTest(int x, int y, double depth) {
        if (x < 0 || x >= width || y < 0 || y >= height) return false;
        int idx = y * width + x;
        if (depth < depthbuffer[idx]) {
            depthbuffer[idx] = depth;
            return true;
        }
        return false;
    }
    
    Vec3 barycentric(Vec3 p, Vec3 a, Vec3 b, Vec3 c) {
        Vec3 v0 = b - a, v1 = c - a, v2 = p - a;
        double d00 = v0.dot(v0);
        double d01 = v0.dot(v1);
        double d11 = v1.dot(v1);
        double d20 = v2.dot(v0);
        double d21 = v2.dot(v1);
        double denom = d00 * d11 - d01 * d01;
        double v = (d11 * d20 - d01 * d21) / denom;
        double w = (d00 * d21 - d01 * d20) / denom;
        double u = 1.0 - v - w;
        return Vec3(u, v, w);
    }
    
public:
    Rasterizer(int w, int h) : width(w), height(h) {
        framebuffer.resize(w * h * 3, 255);
        depthbuffer.resize(w * h, std::numeric_limits<double>::infinity());
    }
    
    void clear(Vec3 color = Vec3(0.1, 0.1, 0.1)) {
        for (int i = 0; i < width * height * 3; i += 3) {
            framebuffer[i] = (unsigned char)(color.x * 255);
            framebuffer[i+1] = (unsigned char)(color.y * 255);
            framebuffer[i+2] = (unsigned char)(color.z * 255);
        }
        std::fill(depthbuffer.begin(), depthbuffer.end(), std::numeric_limits<double>::infinity());
    }
    
    void drawTriangle(const Triangle& tri, const Mat4& mvp, Vec3 lightDir) {
        // 变换到裁剪空间
        Vec4 v0_clip = mvp * Vec4(tri.v0.pos, 1);
        Vec4 v1_clip = mvp * Vec4(tri.v1.pos, 1);
        Vec4 v2_clip = mvp * Vec4(tri.v2.pos, 1);
        
        // 透视除法
        Vec3 v0_ndc(v0_clip.x / v0_clip.w, v0_clip.y / v0_clip.w, v0_clip.z / v0_clip.w);
        Vec3 v1_ndc(v1_clip.x / v1_clip.w, v1_clip.y / v1_clip.w, v1_clip.z / v1_clip.w);
        Vec3 v2_ndc(v2_clip.x / v2_clip.w, v2_clip.y / v2_clip.w, v2_clip.z / v2_clip.w);
        
        // 视口变换
        Vec3 v0_screen((v0_ndc.x + 1) * width / 2, (1 - v0_ndc.y) * height / 2, v0_ndc.z);
        Vec3 v1_screen((v1_ndc.x + 1) * width / 2, (1 - v1_ndc.y) * height / 2, v1_ndc.z);
        Vec3 v2_screen((v2_ndc.x + 1) * width / 2, (1 - v2_ndc.y) * height / 2, v2_ndc.z);
        
        // 包围盒
        int minX = std::max(0, (int)std::min({v0_screen.x, v1_screen.x, v2_screen.x}));
        int maxX = std::min(width-1, (int)std::max({v0_screen.x, v1_screen.x, v2_screen.x}));
        int minY = std::max(0, (int)std::min({v0_screen.y, v1_screen.y, v2_screen.y}));
        int maxY = std::min(height-1, (int)std::max({v0_screen.y, v1_screen.y, v2_screen.y}));
        
        // 光栅化
        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                Vec3 p(x + 0.5, y + 0.5, 0);
                Vec3 bc = barycentric(p, v0_screen, v1_screen, v2_screen);
                
                if (bc.x >= 0 && bc.y >= 0 && bc.z >= 0) {
                    // 深度插值
                    double depth = bc.x * v0_screen.z + bc.y * v1_screen.z + bc.z * v2_screen.z;
                    
                    if (depthTest(x, y, depth)) {
                        // 法线插值
                        Vec3 normal = (tri.v0.normal * bc.x + tri.v1.normal * bc.y + tri.v2.normal * bc.z).normalized();
                        
                        // 颜色插值
                        Vec3 color = tri.v0.color * bc.x + tri.v1.color * bc.y + tri.v2.color * bc.z;
                        
                        // 简单光照
                        double diffuse = std::max(0.0, normal.dot(lightDir));
                        Vec3 finalColor = color * (0.2 + 0.8 * diffuse);
                        
                        setPixel(x, y, finalColor);
                    }
                }
            }
        }
    }
    
    void save(const char* filename) {
        stbi_write_png(filename, width, height, 3, framebuffer.data(), width * 3);
    }
};

// 创建立方体
std::vector<Triangle> createCube(Vec3 center, double size, Vec3 color) {
    std::vector<Triangle> triangles;
    double h = size / 2;
    
    Vec3 vertices[8] = {
        center + Vec3(-h, -h, -h), center + Vec3(h, -h, -h),
        center + Vec3(h, h, -h), center + Vec3(-h, h, -h),
        center + Vec3(-h, -h, h), center + Vec3(h, -h, h),
        center + Vec3(h, h, h), center + Vec3(-h, h, h)
    };
    
    int faces[6][4] = {
        {0,1,2,3}, {4,5,6,7}, {0,1,5,4},
        {2,3,7,6}, {0,3,7,4}, {1,2,6,5}
    };
    
    Vec3 normals[6] = {
        Vec3(0,0,-1), Vec3(0,0,1), Vec3(0,-1,0),
        Vec3(0,1,0), Vec3(-1,0,0), Vec3(1,0,0)
    };
    
    for (int f = 0; f < 6; f++) {
        Vertex v0(vertices[faces[f][0]], normals[f], color);
        Vertex v1(vertices[faces[f][1]], normals[f], color);
        Vertex v2(vertices[faces[f][2]], normals[f], color);
        Vertex v3(vertices[faces[f][3]], normals[f], color);
        
        triangles.push_back(Triangle(v0, v1, v2));
        triangles.push_back(Triangle(v0, v2, v3));
    }
    
    return triangles;
}

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;
    
    Rasterizer rasterizer(WIDTH, HEIGHT);
    
    // 相机设置
    Vec3 eye(3, 3, 5);
    Vec3 center(0, 0, 0);
    Vec3 up(0, 1, 0);
    
    Mat4 view = Mat4::lookAt(eye, center, up);
    Mat4 projection = Mat4::perspective(45.0 * 3.14159 / 180.0, (double)WIDTH / HEIGHT, 0.1, 100.0);
    Mat4 mvp = projection * view;
    
    Vec3 lightDir = Vec3(1, 1, 1).normalized();
    
    // 创建场景
    std::vector<Triangle> scene;
    
    // 几个彩色立方体
    auto cube1 = createCube(Vec3(0, 0, 0), 1.5, Vec3(1, 0.3, 0.3));
    auto cube2 = createCube(Vec3(-2, 0, -1), 1.0, Vec3(0.3, 1, 0.3));
    auto cube3 = createCube(Vec3(2, 1, 1), 0.8, Vec3(0.3, 0.3, 1));
    auto cube4 = createCube(Vec3(0, -2, 0), 3.0, Vec3(0.7, 0.7, 0.7));  // 地板
    
    scene.insert(scene.end(), cube1.begin(), cube1.end());
    scene.insert(scene.end(), cube2.begin(), cube2.end());
    scene.insert(scene.end(), cube3.begin(), cube3.end());
    scene.insert(scene.end(), cube4.begin(), cube4.end());
    
    // 渲染
    rasterizer.clear(Vec3(0.2, 0.3, 0.4));
    for (const auto& tri : scene) {
        rasterizer.drawTriangle(tri, mvp, lightDir);
    }
    rasterizer.save("rasterizer_output.png");
    
    printf("Rendered 3D scene with software rasterizer!\n");
    
    return 0;
}
