#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

struct Vec2 {
    double x, y;
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
    double length() const { return sqrt(x * x + y * y); }
    Vec2 normalized() const { double l = length(); return l > 0 ? Vec2(x/l, y/l) : Vec2(0, 0); }
};

struct Particle {
    Vec2 pos, vel;
    double mass;
    unsigned char r, g, b;
    
    Particle(Vec2 p, Vec2 v, double m, int r, int g, int b) 
        : pos(p), vel(v), mass(m), r(r), g(g), b(b) {}
};

class ParticleSystem {
public:
    std::vector<Particle> particles;
    int width, height;
    double gravity;
    double damping;
    
    ParticleSystem(int w, int h) : width(w), height(h), gravity(0.5), damping(0.99) {}
    
    void addParticle(Particle p) {
        particles.push_back(p);
    }
    
    void update(double dt) {
        // 更新速度和位置
        for (auto& p : particles) {
            // 简单重力
            p.vel.y += gravity * dt;
            
            // 阻尼
            p.vel = p.vel * damping;
            
            // 更新位置
            p.pos = p.pos + p.vel * dt;
            
            // 边界碰撞（简单反弹）
            if (p.pos.x < 0 || p.pos.x >= width) {
                p.vel.x *= -0.8;
                p.pos.x = std::max(0.0, std::min((double)width - 1, p.pos.x));
            }
            if (p.pos.y < 0 || p.pos.y >= height) {
                p.vel.y *= -0.8;
                p.pos.y = std::max(0.0, std::min((double)height - 1, p.pos.y));
            }
        }
    }
    
    void render(std::vector<unsigned char>& pixels, bool trails = false) {
        if (!trails) {
            // 清空画布（黑色背景）
            std::fill(pixels.begin(), pixels.end(), 0);
        } else {
            // 渐隐效果
            for (size_t i = 0; i < pixels.size(); i++) {
                pixels[i] = pixels[i] * 0.95;
            }
        }
        
        // 绘制粒子
        for (const auto& p : particles) {
            int px = (int)p.pos.x;
            int py = (int)p.pos.y;
            
            // 画一个小圆
            for (int dy = -2; dy <= 2; dy++) {
                for (int dx = -2; dx <= 2; dx++) {
                    if (dx*dx + dy*dy <= 4) {
                        int x = px + dx;
                        int y = py + dy;
                        if (x >= 0 && x < width && y >= 0 && y < height) {
                            int idx = (y * width + x) * 3;
                            pixels[idx] = std::min(255, pixels[idx] + p.r);
                            pixels[idx + 1] = std::min(255, pixels[idx + 1] + p.g);
                            pixels[idx + 2] = std::min(255, pixels[idx + 2] + p.b);
                        }
                    }
                }
            }
        }
    }
};

// 爆炸效果
void generateExplosion(const char* filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 3, 0);
    ParticleSystem ps(width, height);
    ps.gravity = 0.2;
    ps.damping = 0.98;
    
    // 在中心生成粒子
    std::mt19937 rng(42);
    std::uniform_real_distribution<> angleDist(0, 2 * M_PI);
    std::uniform_real_distribution<> speedDist(2, 10);
    
    for (int i = 0; i < 300; i++) {
        double angle = angleDist(rng);
        double speed = speedDist(rng);
        Vec2 vel(cos(angle) * speed, sin(angle) * speed);
        
        // 彩色粒子
        int r = 200 + rng() % 56;
        int g = 100 + rng() % 100;
        int b = 50;
        
        ps.addParticle(Particle(Vec2(width/2, height/2), vel, 1.0, r, g, b));
    }
    
    // 模拟60帧
    for (int frame = 0; frame < 60; frame++) {
        ps.update(1.0);
        ps.render(pixels, true);  // 拖尾效果
    }
    
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

// 喷泉效果
void generateFountain(const char* filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 3, 0);
    ParticleSystem ps(width, height);
    ps.gravity = 0.3;
    ps.damping = 0.99;
    
    std::mt19937 rng(123);
    std::uniform_real_distribution<> angleDist(-M_PI/4, M_PI/4);
    std::uniform_real_distribution<> speedDist(8, 12);
    
    // 模拟80帧，每帧添加新粒子
    for (int frame = 0; frame < 80; frame++) {
        // 每帧添加粒子
        if (frame % 2 == 0) {
            for (int i = 0; i < 5; i++) {
                double angle = angleDist(rng) - M_PI/2;  // 向上喷射
                double speed = speedDist(rng);
                Vec2 vel(cos(angle) * speed, sin(angle) * speed);
                
                // 蓝色水滴
                int b = 200 + rng() % 56;
                ps.addParticle(Particle(Vec2(width/2, height - 50), vel, 1.0, 100, 150, b));
            }
        }
        
        ps.update(1.0);
        ps.render(pixels, true);
    }
    
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

// 螺旋星系
void generateSpiral(const char* filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 3, 0);
    ParticleSystem ps(width, height);
    ps.gravity = 0;
    ps.damping = 1.0;
    
    std::mt19937 rng(456);
    double centerX = width / 2;
    double centerY = height / 2;
    
    // 生成螺旋臂
    for (int arm = 0; arm < 3; arm++) {
        for (int i = 0; i < 200; i++) {
            double t = i / 20.0;
            double angle = arm * 2 * M_PI / 3 + t;
            double radius = 20 + t * 25;
            
            double x = centerX + radius * cos(angle);
            double y = centerY + radius * sin(angle);
            
            // 切向速度
            double vx = -sin(angle) * 2;
            double vy = cos(angle) * 2;
            
            // 星星颜色
            int brightness = 150 + rng() % 106;
            ps.addParticle(Particle(Vec2(x, y), Vec2(vx, vy), 1.0, 
                                   brightness, brightness, 255));
        }
    }
    
    // 渲染静态场景
    ps.render(pixels, false);
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

int main() {
    const int W = 800, H = 600;
    
    generateExplosion("particles_explosion.png", W, H);
    generateFountain("particles_fountain.png", W, H);
    generateSpiral("particles_spiral.png", W, H);
    
    return 0;
}
