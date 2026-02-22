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
    double dot(const Vec2& v) const { return x * v.x + y * v.y; }
    double length() const { return sqrt(x * x + y * y); }
    Vec2 normalized() const { double l = length(); return l > 0 ? Vec2(x/l, y/l) : Vec2(0,0); }
    double cross(const Vec2& v) const { return x * v.y - y * v.x; }
};

// 刚体
struct RigidBody {
    Vec2 pos, vel, force;
    double angle, angularVel, torque;
    double mass, invMass, inertia, invInertia;
    double restitution;  // 弹性系数
    double radius;       // 圆形刚体
    
    RigidBody(Vec2 p, double r, double m, double e = 0.8) 
        : pos(p), vel(0,0), force(0,0), angle(0), angularVel(0), torque(0), 
          radius(r), mass(m), restitution(e) {
        invMass = (m > 0) ? 1.0 / m : 0;
        inertia = 0.5 * m * r * r;  // 圆盘转动惯量
        invInertia = (inertia > 0) ? 1.0 / inertia : 0;
    }
    
    void applyForce(Vec2 f) {
        force = force + f;
    }
    
    void applyImpulse(Vec2 impulse, Vec2 contactVector) {
        vel = vel + impulse * invMass;
        angularVel += contactVector.cross(impulse) * invInertia;
    }
    
    void update(double dt) {
        if (invMass == 0) return;  // 静止物体
        
        vel = vel + force * invMass * dt;
        pos = pos + vel * dt;
        
        angularVel += torque * invInertia * dt;
        angle += angularVel * dt;
        
        force = Vec2(0, 0);
        torque = 0;
    }
};

// 碰撞检测与响应
void resolveCollision(RigidBody& a, RigidBody& b) {
    Vec2 delta = b.pos - a.pos;
    double distance = delta.length();
    double sumRadius = a.radius + b.radius;
    
    if (distance < sumRadius) {
        // 碰撞法线
        Vec2 normal = delta.normalized();
        
        // 穿透深度
        double penetration = sumRadius - distance;
        
        // 分离物体
        Vec2 correction = normal * (penetration / (a.invMass + b.invMass));
        a.pos = a.pos - correction * a.invMass;
        b.pos = b.pos + correction * b.invMass;
        
        // 相对速度
        Vec2 relativeVel = b.vel - a.vel;
        double velAlongNormal = relativeVel.dot(normal);
        
        if (velAlongNormal < 0) return;  // 已经分离
        
        // 冲量计算
        double e = std::min(a.restitution, b.restitution);
        double j = -(1 + e) * velAlongNormal;
        j /= a.invMass + b.invMass;
        
        Vec2 impulse = normal * j;
        a.applyImpulse(impulse * -1, Vec2(0,0));
        b.applyImpulse(impulse, Vec2(0,0));
    }
}

// 边界碰撞
void applyBoundaryConstraints(RigidBody& body, double width, double height) {
    if (body.pos.x - body.radius < 0) {
        body.pos.x = body.radius;
        body.vel.x = -body.vel.x * body.restitution;
    }
    if (body.pos.x + body.radius > width) {
        body.pos.x = width - body.radius;
        body.vel.x = -body.vel.x * body.restitution;
    }
    if (body.pos.y - body.radius < 0) {
        body.pos.y = body.radius;
        body.vel.y = -body.vel.y * body.restitution;
    }
    if (body.pos.y + body.radius > height) {
        body.pos.y = height - body.radius;
        body.vel.y = -body.vel.y * body.restitution;
    }
}

// 渲染
void render(const std::vector<RigidBody>& bodies, std::vector<unsigned char>& pixels, 
            int width, int height) {
    
    std::fill(pixels.begin(), pixels.end(), 255);
    
    for (const auto& body : bodies) {
        int cx = (int)body.pos.x;
        int cy = (int)body.pos.y;
        int r = (int)body.radius;
        
        // 画圆
        for (int y = -r; y <= r; y++) {
            for (int x = -r; x <= r; x++) {
                if (x*x + y*y <= r*r) {
                    int px = cx + x;
                    int py = cy + y;
                    if (px >= 0 && px < width && py >= 0 && py < height) {
                        int idx = (py * width + px) * 3;
                        
                        // 根据速度着色
                        double speed = body.vel.length();
                        int red = std::min(255, (int)(speed * 10));
                        int blue = 255 - red;
                        
                        pixels[idx] = red;
                        pixels[idx + 1] = 100;
                        pixels[idx + 2] = blue;
                    }
                }
            }
        }
        
        // 画方向指示线
        int dirX = cx + (int)(cos(body.angle) * r);
        int dirY = cy + (int)(sin(body.angle) * r);
        
        // Bresenham 画线
        int dx = abs(dirX - cx);
        int dy = abs(dirY - cy);
        int sx = (cx < dirX) ? 1 : -1;
        int sy = (cy < dirY) ? 1 : -1;
        int err = dx - dy;
        
        int x0 = cx, y0 = cy;
        while (true) {
            if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) {
                int idx = (y0 * width + x0) * 3;
                pixels[idx] = 0;
                pixels[idx + 1] = 0;
                pixels[idx + 2] = 0;
            }
            
            if (x0 == dirX && y0 == dirY) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
}

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;
    
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> xDist(50, WIDTH - 50);
    std::uniform_real_distribution<double> yDist(50, HEIGHT - 300);
    std::uniform_real_distribution<double> velDist(-50, 50);
    
    // 创建刚体
    std::vector<RigidBody> bodies;
    
    // 添加随机球体
    for (int i = 0; i < 20; i++) {
        Vec2 pos(xDist(rng), yDist(rng));
        Vec2 vel(velDist(rng), velDist(rng));
        double radius = 15 + rng() % 20;
        double mass = radius * radius * 0.1;
        
        RigidBody body(pos, radius, mass, 0.7);
        body.vel = vel;
        bodies.push_back(body);
    }
    
    // 添加几个大球
    bodies.push_back(RigidBody(Vec2(WIDTH/2, 100), 40, 50, 0.9));
    bodies[bodies.size()-1].vel = Vec2(0, 20);
    
    bodies.push_back(RigidBody(Vec2(200, 200), 35, 40, 0.85));
    bodies[bodies.size()-1].vel = Vec2(30, -10);
    
    bodies.push_back(RigidBody(Vec2(600, 200), 35, 40, 0.85));
    bodies[bodies.size()-1].vel = Vec2(-30, -10);
    
    // 添加静止的地面球
    bodies.push_back(RigidBody(Vec2(WIDTH/2, HEIGHT - 50), 45, 0, 1.0));
    
    std::vector<unsigned char> pixels(WIDTH * HEIGHT * 3);
    
    Vec2 gravity(0, 200);
    double dt = 0.016;  // ~60 FPS
    
    // 模拟并保存关键帧
    int savedFrames = 0;
    for (int frame = 0; frame < 300; frame++) {
        // 应用重力
        for (auto& body : bodies) {
            body.applyForce(gravity * body.mass);
        }
        
        // 更新物体
        for (auto& body : bodies) {
            body.update(dt);
        }
        
        // 碰撞检测与响应
        for (size_t i = 0; i < bodies.size(); i++) {
            for (size_t j = i + 1; j < bodies.size(); j++) {
                resolveCollision(bodies[i], bodies[j]);
            }
        }
        
        // 边界约束
        for (auto& body : bodies) {
            applyBoundaryConstraints(body, WIDTH, HEIGHT);
        }
        
        // 保存关键帧
        if (frame % 30 == 0 || frame == 299) {
            render(bodies, pixels, WIDTH, HEIGHT);
            char filename[100];
            sprintf(filename, "physics_frame_%02d.png", savedFrames++);
            stbi_write_png(filename, WIDTH, HEIGHT, 3, pixels.data(), WIDTH * 3);
            printf("Saved: %s\n", filename);
        }
    }
    
    return 0;
}
