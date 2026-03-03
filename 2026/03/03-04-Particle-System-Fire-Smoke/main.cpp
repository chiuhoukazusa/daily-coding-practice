/**
 * Particle System - Fire & Smoke Simulation
 * 
 * 功能：
 * - CPU粒子系统，模拟火焰和烟雾效果
 * - 粒子具有：位置、速度、加速度、生命周期、颜色、大小
 * - 火焰：黑色→红色→橙色→黄色→白色的颜色渐变
 * - 烟雾：白色→灰色→透明的渐变
 * - 随机初始化、力场模拟（重力、浮力、湍流）
 * - 多帧时序渲染为合成图（时间序列）
 * 
 * 输出：
 * - fire_output.png: 800x600 火焰粒子系统最终帧
 * - fire_sequence.png: 800x300 四帧时序对比图
 * 
 * 编译：g++ main.cpp -o particle_system -std=c++17 -O2 -Wno-missing-field-initializers
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <random>
#include <iostream>
#include <functional>

// =================== 基础数学 ===================

struct Vec2 {
    float x, y;
    Vec2(float x=0, float y=0): x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float s) const { return {x*s, y*s}; }
    Vec2& operator+=(const Vec2& o) { x+=o.x; y+=o.y; return *this; }
};

struct Color {
    float r, g, b, a; // [0,1]
    Color(float r=0, float g=0, float b=0, float a=1): r(r), g(g), b(b), a(a) {}
    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b, a+o.a}; }
    Color operator*(float s) const { return {r*s, g*s, b*s, a*s}; }
};

// 颜色线性插值
Color lerpColor(const Color& a, const Color& b, float t) {
    t = std::max(0.0f, std::min(1.0f, t));
    return {
        a.r + (b.r - a.r) * t,
        a.g + (b.g - a.g) * t,
        a.b + (b.b - a.b) * t,
        a.a + (b.a - a.a) * t
    };
}

// =================== 噪声工具 ===================

// 简单哈希噪声 用于湍流
float hash1(float x) {
    x = std::sin(x * 127.1f) * 43758.5453f;
    return x - std::floor(x);
}

float hash2(float x, float y) {
    float n = std::sin(x * 127.1f + y * 311.7f) * 43758.5453f;
    return n - std::floor(n);
}

// 简单噪声 [-1,1]
float noise2(float x, float y) {
    return hash2(x, y) * 2.0f - 1.0f;
}

// =================== 粒子定义 ===================

enum class ParticleType {
    FIRE,
    SMOKE,
    EMBER
};

struct Particle {
    Vec2 pos;           // 当前位置
    Vec2 vel;           // 速度
    float life;         // 当前生命值 [0,1] (1=刚出生, 0=死亡)
    float maxLife;      // 生命持续时间（秒）
    float size;         // 粒子大小（像素）
    float maxSize;      // 最大尺寸
    float rotation;     // 旋转角度
    float rotSpeed;     // 旋转速度
    float turbSeed;     // 湍流种子
    ParticleType type;
    bool alive;
    float age;          // 当前年龄（秒）
};

// =================== 颜色梯度 ===================

// 火焰颜色梯度（生命周期 0→1 对应 老→新）
Color getFireColor(float life) {
    // life=1: 刚出生（蓝白热核心）
    // life=0.8: 黄白 
    // life=0.6: 橙黄
    // life=0.4: 橙红
    // life=0.2: 暗红
    // life=0: 黑/透明
    if (life > 0.85f) {
        return lerpColor(Color(1.0f, 0.95f, 0.6f, 0.9f), Color(1.0f, 1.0f, 1.0f, 1.0f), (life - 0.85f) / 0.15f);
    } else if (life > 0.65f) {
        return lerpColor(Color(1.0f, 0.55f, 0.1f, 0.95f), Color(1.0f, 0.95f, 0.6f, 0.9f), (life - 0.65f) / 0.20f);
    } else if (life > 0.40f) {
        return lerpColor(Color(0.8f, 0.15f, 0.02f, 0.85f), Color(1.0f, 0.55f, 0.1f, 0.95f), (life - 0.40f) / 0.25f);
    } else if (life > 0.15f) {
        return lerpColor(Color(0.3f, 0.0f, 0.0f, 0.4f), Color(0.8f, 0.15f, 0.02f, 0.85f), (life - 0.15f) / 0.25f);
    } else {
        return lerpColor(Color(0.0f, 0.0f, 0.0f, 0.0f), Color(0.3f, 0.0f, 0.0f, 0.4f), life / 0.15f);
    }
}

// 烟雾颜色梯度
Color getSmokeColor(float life) {
    // life=1: 深灰（刚出生）
    // life=0.5: 灰色
    // life=0: 透明白
    float gray = 0.3f + (1.0f - life) * 0.5f;
    float alpha = life * 0.5f;
    return Color(gray, gray, gray, alpha);
}

// 火星颜色
Color getEmberColor(float life) {
    if (life > 0.5f) {
        return lerpColor(Color(1.0f, 0.6f, 0.0f, 0.9f), Color(1.0f, 1.0f, 0.5f, 1.0f), (life - 0.5f) * 2.0f);
    } else {
        return lerpColor(Color(0.4f, 0.0f, 0.0f, 0.0f), Color(1.0f, 0.6f, 0.0f, 0.9f), life * 2.0f);
    }
}

// =================== 粒子系统 ===================

class ParticleSystem {
public:
    std::vector<Particle> particles;
    int maxParticles;
    Vec2 emitterPos;    // 发射器位置
    float emitRate;     // 每秒发射粒子数
    float emitTimer;    // 发射计时器
    float time;         // 总时间
    
    std::mt19937 rng;
    std::uniform_real_distribution<float> dist01;
    std::uniform_real_distribution<float> distNeg11;
    
    ParticleSystem(Vec2 pos, int maxP, float rate)
        : emitterPos(pos), maxParticles(maxP), emitRate(rate),
          emitTimer(0), time(0),
          rng(42), dist01(0.0f, 1.0f), distNeg11(-1.0f, 1.0f)
    {
        particles.reserve(maxP);
    }
    
    float rand01() { return dist01(rng); }
    float randN() { return distNeg11(rng); }
    
    // 发射新粒子
    void emitFire(float dt) {
        int count = (int)(emitRate * dt + emitTimer);
        emitTimer += emitRate * dt - count;
        if (emitTimer < 0) emitTimer = 0;
        
        for (int i = 0; i < count; i++) {
            if ((int)particles.size() >= maxParticles) break;
            
            Particle p;
            // 在发射器附近随机位置
            float spread = 20.0f;
            p.pos = Vec2(
                emitterPos.x + randN() * spread,
                emitterPos.y + randN() * 5.0f
            );
            
            // 初始速度：向上，带随机分量
            p.vel = Vec2(
                randN() * 15.0f,
                -(60.0f + rand01() * 80.0f)  // 向上（y轴向下为正，所以用负值）
            );
            
            p.maxLife = 1.5f + rand01() * 1.5f;  // 1.5-3秒
            p.age = 0.0f;
            p.life = 1.0f;
            p.size = 4.0f + rand01() * 8.0f;
            p.maxSize = p.size * (2.0f + rand01() * 2.0f);
            p.rotation = rand01() * 360.0f;
            p.rotSpeed = randN() * 120.0f;
            p.turbSeed = rand01() * 1000.0f;
            p.type = ParticleType::FIRE;
            p.alive = true;
            particles.push_back(p);
        }
    }
    
    // 发射烟雾
    void emitSmoke(float dt) {
        static float smokeTimer = 0;
        smokeTimer += dt;
        float smokeRate = emitRate * 0.3f;
        int count = (int)(smokeRate * smokeTimer);
        smokeTimer -= count / smokeRate;
        if (count <= 0) return;
        
        for (int i = 0; i < count; i++) {
            if ((int)particles.size() >= maxParticles) break;
            
            Particle p;
            p.pos = Vec2(
                emitterPos.x + randN() * 15.0f,
                emitterPos.y - 80.0f - rand01() * 30.0f  // 从火焰上方出现
            );
            p.vel = Vec2(
                randN() * 8.0f,
                -(15.0f + rand01() * 20.0f)
            );
            p.maxLife = 3.0f + rand01() * 2.0f;  // 3-5秒
            p.age = 0.0f;
            p.life = 1.0f;
            p.size = 10.0f + rand01() * 20.0f;
            p.maxSize = p.size * (3.0f + rand01() * 3.0f);
            p.rotation = rand01() * 360.0f;
            p.rotSpeed = randN() * 30.0f;
            p.turbSeed = rand01() * 1000.0f + 500.0f;
            p.type = ParticleType::SMOKE;
            p.alive = true;
            particles.push_back(p);
        }
    }
    
    // 发射火星
    void emitEmbers(float dt) {
        static float emberTimer = 0;
        emberTimer += dt;
        float emberRate = emitRate * 0.1f;
        int count = (int)(emberRate * emberTimer);
        emberTimer -= (count > 0) ? count / emberRate : 0;
        
        for (int i = 0; i < count; i++) {
            if ((int)particles.size() >= maxParticles) break;
            
            Particle p;
            p.pos = Vec2(
                emitterPos.x + randN() * 25.0f,
                emitterPos.y - 30.0f - rand01() * 40.0f
            );
            // 火星速度更快，向上且带大随机分量
            p.vel = Vec2(
                randN() * 40.0f,
                -(80.0f + rand01() * 120.0f)
            );
            p.maxLife = 1.0f + rand01() * 2.0f;
            p.age = 0.0f;
            p.life = 1.0f;
            p.size = 2.0f + rand01() * 3.0f;
            p.maxSize = p.size;
            p.rotation = 0;
            p.rotSpeed = 0;
            p.turbSeed = rand01() * 1000.0f + 200.0f;
            p.type = ParticleType::EMBER;
            p.alive = true;
            particles.push_back(p);
        }
    }
    
    // 更新单个粒子
    void updateParticle(Particle& p, float dt) {
        p.age += dt;
        p.life = 1.0f - (p.age / p.maxLife);
        
        if (p.life <= 0.0f) {
            p.alive = false;
            return;
        }
        
        // 湍流力
        float tx = noise2(p.pos.x * 0.01f + p.turbSeed, p.pos.y * 0.01f + time * 0.5f);
        float ty = noise2(p.pos.x * 0.01f + p.turbSeed + 100.0f, p.pos.y * 0.01f + time * 0.7f);
        
        Vec2 turbForce(tx * 30.0f, ty * 15.0f);
        
        switch (p.type) {
            case ParticleType::FIRE: {
                // 浮力（向上）
                Vec2 buoyancy(0, -40.0f * p.life);
                // 风力扰动
                Vec2 wind(std::sin(time * 1.5f + p.pos.y * 0.02f) * 10.0f, 0);
                
                p.vel += (buoyancy + turbForce + wind) * dt;
                // 阻尼
                p.vel = p.vel * 0.97f;
                
                // 大小：先增大后减小
                float lifeFactor = std::sin(p.life * 3.14159f);
                p.size = 3.0f + lifeFactor * p.maxSize;
                break;
            }
            case ParticleType::SMOKE: {
                // 烟雾：轻微浮力，强湍流
                Vec2 buoyancy(0, -20.0f);
                Vec2 wind(std::sin(time * 0.8f + p.pos.y * 0.01f) * 15.0f, 0);
                
                p.vel += (buoyancy + turbForce * 1.5f + wind) * dt;
                p.vel = p.vel * 0.98f;
                
                // 烟雾不断扩散变大
                p.size = p.maxSize * (1.0f - p.life * 0.5f);
                break;
            }
            case ParticleType::EMBER: {
                // 火星：重力
                Vec2 gravity(0, 80.0f);
                p.vel += (gravity + turbForce * 2.0f) * dt;
                p.vel = p.vel * 0.99f;
                break;
            }
        }
        
        p.pos += p.vel * dt;
        p.rotation += p.rotSpeed * dt;
    }
    
    // 更新整个系统
    void update(float dt) {
        time += dt;
        
        emitFire(dt);
        emitSmoke(dt);
        emitEmbers(dt);
        
        // 更新粒子
        for (auto& p : particles) {
            if (p.alive) updateParticle(p, dt);
        }
        
        // 移除死亡粒子
        particles.erase(
            std::remove_if(particles.begin(), particles.end(),
                [](const Particle& p) { return !p.alive; }),
            particles.end()
        );
    }
};

// =================== 渲染 ===================

struct Image {
    int width, height;
    std::vector<uint8_t> data; // RGBA
    
    Image(int w, int h) : width(w), height(h), data(w * h * 4, 0) {}
    
    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 4;
        data[idx+0] = r;
        data[idx+1] = g;
        data[idx+2] = b;
        data[idx+3] = a;
    }
    
    void getPixel(int x, int y, uint8_t& r, uint8_t& g, uint8_t& b, uint8_t& a) const {
        if (x < 0 || x >= width || y < 0 || y >= height) { r=g=b=a=0; return; }
        int idx = (y * width + x) * 4;
        r = data[idx+0];
        g = data[idx+1];
        b = data[idx+2];
        a = data[idx+3];
    }
    
    // 清空为黑色背景
    void clear(uint8_t r=0, uint8_t g=0, uint8_t b=0) {
        for (int i = 0; i < width * height; i++) {
            data[i*4+0] = r;
            data[i*4+1] = g;
            data[i*4+2] = b;
            data[i*4+3] = 255;
        }
    }
    
    // Additive/Alpha 混合绘制粒子（软圆形）
    void blendParticle(float cx, float cy, float radius, Color color, bool additive) {
        int x0 = (int)(cx - radius - 1);
        int x1 = (int)(cx + radius + 1);
        int y0 = (int)(cy - radius - 1);
        int y1 = (int)(cy + radius + 1);
        
        x0 = std::max(0, x0);
        x1 = std::min(width - 1, x1);
        y0 = std::max(0, y0);
        y1 = std::min(height - 1, y1);
        
        for (int py = y0; py <= y1; py++) {
            for (int px = x0; px <= x1; px++) {
                float dx = px - cx;
                float dy = py - cy;
                float dist = std::sqrt(dx*dx + dy*dy);
                if (dist > radius) continue;
                
                // 软边缘：高斯衰减
                float t = dist / radius;
                float falloff = std::exp(-t * t * 3.0f);
                float alpha = color.a * falloff;
                
                if (alpha < 0.001f) continue;
                
                int idx = (py * width + px) * 4;
                float sr = data[idx+0] / 255.0f;
                float sg = data[idx+1] / 255.0f;
                float sb = data[idx+2] / 255.0f;
                
                float nr, ng, nb;
                if (additive) {
                    // 加法混合（火焰效果）
                    nr = std::min(1.0f, sr + color.r * alpha);
                    ng = std::min(1.0f, sg + color.g * alpha);
                    nb = std::min(1.0f, sb + color.b * alpha);
                } else {
                    // Alpha混合（烟雾效果）
                    nr = sr * (1.0f - alpha) + color.r * alpha;
                    ng = sg * (1.0f - alpha) + color.g * alpha;
                    nb = sb * (1.0f - alpha) + color.b * alpha;
                }
                
                data[idx+0] = (uint8_t)(std::min(1.0f, nr) * 255);
                data[idx+1] = (uint8_t)(std::min(1.0f, ng) * 255);
                data[idx+2] = (uint8_t)(std::min(1.0f, nb) * 255);
                data[idx+3] = 255;
            }
        }
    }
    
    bool save(const char* filename) const {
        return stbi_write_png(filename, width, height, 4, data.data(), width * 4) != 0;
    }
};

// 绘制粒子系统到图像
void renderParticles(Image& img, const ParticleSystem& ps) {
    // 先排序：烟雾在下，火焰在上，火星在最上
    std::vector<const Particle*> sorted;
    sorted.reserve(ps.particles.size());
    for (const auto& p : ps.particles) {
        if (p.alive) sorted.push_back(&p);
    }
    
    // 按类型排序：SMOKE → FIRE → EMBER
    std::stable_sort(sorted.begin(), sorted.end(),
        [](const Particle* a, const Particle* b) {
            return (int)a->type < (int)b->type;
        });
    
    for (const Particle* p : sorted) {
        Color color;
        bool additive;
        
        switch (p->type) {
            case ParticleType::SMOKE:
                color = getSmokeColor(p->life);
                additive = false;
                break;
            case ParticleType::FIRE:
                color = getFireColor(p->life);
                additive = true;
                break;
            case ParticleType::EMBER:
                color = getEmberColor(p->life);
                additive = true;
                break;
        }
        
        img.blendParticle(p->pos.x, p->pos.y, p->size, color, additive);
    }
}

// =================== 主程序 ===================

int main() {
    const int WIDTH  = 800;
    const int HEIGHT = 600;
    
    std::cout << "=== Particle System - Fire & Smoke Simulation ===" << std::endl;
    std::cout << "输出尺寸: " << WIDTH << "x" << HEIGHT << std::endl;
    
    // 创建粒子系统，发射器在图像中央底部
    Vec2 emitterPos(WIDTH / 2.0f, HEIGHT - 80.0f);
    ParticleSystem ps(emitterPos, 1000, 60.0f);  // 最多1000粒子，每秒60个
    
    // ---- 渲染主图（模拟4秒的稳定状态）----
    
    // 先预热3秒让粒子系统达到稳定
    float dt = 1.0f / 60.0f;  // 60fps模拟
    int warmupFrames = (int)(3.0f / dt);
    
    std::cout << "预热粒子系统 (" << warmupFrames << " 帧)..." << std::endl;
    for (int i = 0; i < warmupFrames; i++) {
        ps.update(dt);
    }
    
    std::cout << "预热完成，当前粒子数: " << ps.particles.size() << std::endl;
    
    // 渲染稳定帧
    Image mainImg(WIDTH, HEIGHT);
    mainImg.clear(5, 5, 10);  // 深色背景
    
    // 绘制地面参考线
    for (int x = 0; x < WIDTH; x++) {
        int groundY = HEIGHT - 80;
        for (int dy = 0; dy < 2; dy++) {
            int idx = ((groundY + dy) * WIDTH + x) * 4;
            if ((groundY + dy) < HEIGHT) {
                mainImg.data[idx+0] = 40;
                mainImg.data[idx+1] = 30;
                mainImg.data[idx+2] = 20;
            }
        }
    }
    
    renderParticles(mainImg, ps);
    
    // 简单的辉光效果：对亮像素进行模糊扩散
    // （只做一次简单的水平竖直扩散）
    {
        Image blurImg = mainImg;
        for (int y = 1; y < HEIGHT-1; y++) {
            for (int x = 1; x < WIDTH-1; x++) {
                // 找周围高亮像素
                int cx = mainImg.data[(y*WIDTH+x)*4+0];
                int cy = mainImg.data[(y*WIDTH+x)*4+1];
                int cz = mainImg.data[(y*WIDTH+x)*4+2];
                
                int sum = cx + cy + cz;
                if (sum > 600) {  // 高亮像素
                    // 向周围扩散
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            if (dx == 0 && dy == 0) continue;
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
                                int ni = (ny*WIDTH+nx)*4;
                                float spread = 0.15f;
                                blurImg.data[ni+0] = (uint8_t)std::min(255, (int)blurImg.data[ni+0] + (int)(cx * spread));
                                blurImg.data[ni+1] = (uint8_t)std::min(255, (int)blurImg.data[ni+1] + (int)(cy * spread));
                                blurImg.data[ni+2] = (uint8_t)std::min(255, (int)blurImg.data[ni+2] + (int)(cz * spread));
                            }
                        }
                    }
                }
            }
        }
        mainImg = blurImg;
    }
    
    // ---- 渲染时序图（4帧：0.5s, 1.5s, 2.5s, 3.5s 对应稳定之后） ----
    
    const int SEQ_WIDTH  = 800;
    const int SEQ_HEIGHT = 300;
    const int FRAME_W = SEQ_WIDTH / 4;
    const int FRAME_H = SEQ_HEIGHT;
    
    Image seqImg(SEQ_WIDTH, SEQ_HEIGHT);
    seqImg.clear(5, 5, 10);
    
    // 四帧序列
    float frameTimes[] = {0.0f, 0.5f, 1.0f, 1.5f};
    
    // 重新创建粒子系统（使用不同种子确保多样性）
    for (int fi = 0; fi < 4; fi++) {
        ParticleSystem fps2(Vec2(FRAME_W/2.0f, FRAME_H - 30.0f), 500, 40.0f);
        fps2.rng.seed(fi * 137 + 42);
        
        // 预热1秒
        int preFrames = (int)(1.0f / dt);
        for (int f = 0; f < preFrames; f++) fps2.update(dt);
        
        // 额外运行frameTimes[fi]秒
        int extraFrames = (int)(frameTimes[fi] / dt);
        for (int f = 0; f < extraFrames; f++) fps2.update(dt);
        
        // 渲染到子图像
        Image subImg(FRAME_W, FRAME_H);
        subImg.clear(5, 5, 10);
        
        // 复制粒子到子图（偏移量为0）
        renderParticles(subImg, fps2);
        
        // 将子图复制到序列图
        int offsetX = fi * FRAME_W;
        for (int y = 0; y < FRAME_H; y++) {
            for (int x = 0; x < FRAME_W; x++) {
                if (offsetX + x >= SEQ_WIDTH) continue;
                int si = (y * FRAME_W + x) * 4;
                int di = (y * SEQ_WIDTH + offsetX + x) * 4;
                seqImg.data[di+0] = subImg.data[si+0];
                seqImg.data[di+1] = subImg.data[si+1];
                seqImg.data[di+2] = subImg.data[si+2];
                seqImg.data[di+3] = subImg.data[si+3];
            }
        }
        
        // 绘制分隔线
        if (fi > 0) {
            for (int y = 0; y < FRAME_H; y++) {
                int di = (y * SEQ_WIDTH + offsetX) * 4;
                seqImg.data[di+0] = 60;
                seqImg.data[di+1] = 60;
                seqImg.data[di+2] = 60;
                seqImg.data[di+3] = 255;
            }
        }
        
        std::cout << "  帧 " << fi+1 << "/4: " << fps2.particles.size() << " 粒子" << std::endl;
    }
    
    // ---- 保存图像 ----
    
    if (mainImg.save("fire_output.png")) {
        std::cout << "✅ fire_output.png 保存成功 (" << WIDTH << "x" << HEIGHT << ")" << std::endl;
    } else {
        std::cerr << "❌ fire_output.png 保存失败" << std::endl;
        return 1;
    }
    
    if (seqImg.save("fire_sequence.png")) {
        std::cout << "✅ fire_sequence.png 保存成功 (" << SEQ_WIDTH << "x" << SEQ_HEIGHT << ")" << std::endl;
    } else {
        std::cerr << "❌ fire_sequence.png 保存失败" << std::endl;
        return 1;
    }
    
    // ---- 统计输出 ----
    
    int fireCount = 0, smokeCount = 0, emberCount = 0;
    for (const auto& p : ps.particles) {
        if (p.type == ParticleType::FIRE) fireCount++;
        else if (p.type == ParticleType::SMOKE) smokeCount++;
        else emberCount++;
    }
    
    std::cout << "\n=== 粒子统计 ===" << std::endl;
    std::cout << "总粒子数: " << ps.particles.size() << std::endl;
    std::cout << "  火焰粒子: " << fireCount << std::endl;
    std::cout << "  烟雾粒子: " << smokeCount << std::endl;
    std::cout << "  火星粒子: " << emberCount << std::endl;
    std::cout << "模拟时间: " << ps.time << " 秒" << std::endl;
    
    std::cout << "\n✅ 粒子系统模拟完成！" << std::endl;
    
    return 0;
}
