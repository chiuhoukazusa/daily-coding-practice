#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <algorithm>

struct Vec2 {
    double x, y;
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    double length() const { return sqrt(x * x + y * y); }
    Vec2 normalized() const { double l = length(); return l > 0 ? Vec2(x/l, y/l) : Vec2(0,0); }
};

struct Particle {
    Vec2 pos, oldPos;
    Vec2 acc;
    bool pinned;
    
    Particle(Vec2 p, bool pin = false) : pos(p), oldPos(p), acc(0, 0), pinned(pin) {}
    
    void update(double dt) {
        if (pinned) return;
        
        Vec2 vel = pos - oldPos;
        oldPos = pos;
        pos = pos + vel * 0.99 + acc * dt * dt;  // Verlet 积分
        acc = Vec2(0, 0);
    }
    
    void applyForce(Vec2 force) {
        if (!pinned) acc = acc + force;
    }
};

struct Constraint {
    Particle* p1;
    Particle* p2;
    double restLength;
    
    Constraint(Particle* a, Particle* b) : p1(a), p2(b) {
        restLength = (a->pos - b->pos).length();
    }
    
    void satisfy() {
        Vec2 delta = p2->pos - p1->pos;
        double currentLength = delta.length();
        double diff = (currentLength - restLength) / currentLength;
        
        Vec2 offset = delta * (diff * 0.5);
        
        if (!p1->pinned) p1->pos = p1->pos + offset;
        if (!p2->pinned) p2->pos = p2->pos - offset;
    }
};

class ClothSimulation {
public:
    std::vector<Particle> particles;
    std::vector<Constraint> constraints;
    int width, height;
    
    ClothSimulation(int w, int h, double spacing) : width(w), height(h) {
        // 创建粒子网格
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                bool pinned = (y == 0 && (x == 0 || x == w - 1));  // 固定顶部两个角
                particles.push_back(Particle(Vec2(x * spacing, y * spacing), pinned));
            }
        }
        
        // 创建约束
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int idx = y * w + x;
                
                // 结构约束（上下左右）
                if (x < w - 1) constraints.push_back(Constraint(&particles[idx], &particles[idx + 1]));
                if (y < h - 1) constraints.push_back(Constraint(&particles[idx], &particles[idx + w]));
                
                // 剪切约束（对角线）
                if (x < w - 1 && y < h - 1) {
                    constraints.push_back(Constraint(&particles[idx], &particles[idx + w + 1]));
                    constraints.push_back(Constraint(&particles[idx + 1], &particles[idx + w]));
                }
                
                // 弯曲约束（隔一个）
                if (x < w - 2) constraints.push_back(Constraint(&particles[idx], &particles[idx + 2]));
                if (y < h - 2) constraints.push_back(Constraint(&particles[idx], &particles[idx + w * 2]));
            }
        }
    }
    
    void update(Vec2 gravity, int iterations, double dt) {
        // 应用力
        for (auto& p : particles) {
            p.applyForce(gravity);
        }
        
        // 更新粒子
        for (auto& p : particles) {
            p.update(dt);
        }
        
        // 约束求解（多次迭代以稳定）
        for (int iter = 0; iter < iterations; iter++) {
            for (auto& c : constraints) {
                c.satisfy();
            }
        }
    }
    
    void render(std::vector<unsigned char>& pixels, int imgWidth, int imgHeight) {
        // 清空画布
        std::fill(pixels.begin(), pixels.end(), 255);
        
        // 绘制约束（布料的线条）
        for (const auto& c : constraints) {
            drawLine(pixels, imgWidth, imgHeight,
                    (int)c.p1->pos.x, (int)c.p1->pos.y,
                    (int)c.p2->pos.x, (int)c.p2->pos.y,
                    100, 100, 200);
        }
        
        // 绘制粒子
        for (const auto& p : particles) {
            int px = (int)p.pos.x;
            int py = (int)p.pos.y;
            if (px >= 0 && px < imgWidth && py >= 0 && py < imgHeight) {
                int idx = (py * imgWidth + px) * 3;
                if (p.pinned) {
                    pixels[idx] = 255;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 0;
                } else {
                    pixels[idx] = 0;
                    pixels[idx + 1] = 0;
                    pixels[idx + 2] = 255;
                }
            }
        }
    }
    
private:
    void drawLine(std::vector<unsigned char>& pixels, int w, int h,
                  int x0, int y0, int x1, int y1, int r, int g, int b) {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
                int idx = (y0 * w + x0) * 3;
                pixels[idx] = r;
                pixels[idx + 1] = g;
                pixels[idx + 2] = b;
            }
            
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
};

int main() {
    const int imgWidth = 800;
    const int imgHeight = 800;
    const int clothWidth = 20;
    const int clothHeight = 20;
    const double spacing = 20.0;
    
    ClothSimulation cloth(clothWidth, clothHeight, spacing);
    std::vector<unsigned char> pixels(imgWidth * imgHeight * 3);
    
    Vec2 gravity(0, 100);
    
    // 模拟多帧并保存关键帧
    int frames = 200;
    int savedFrames = 0;
    
    for (int frame = 0; frame < frames; frame++) {
        cloth.update(gravity, 3, 0.016);  // ~60 FPS
        
        // 保存关键帧
        if (frame % 40 == 0 || frame == frames - 1) {
            cloth.render(pixels, imgWidth, imgHeight);
            char filename[100];
            sprintf(filename, "cloth_frame_%02d.png", savedFrames++);
            stbi_write_png(filename, imgWidth, imgHeight, 3, pixels.data(), imgWidth * 3);
            printf("Saved: %s\n", filename);
        }
    }
    
    return 0;
}
