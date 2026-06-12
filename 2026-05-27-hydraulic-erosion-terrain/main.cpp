/*
 * Procedural Terrain with Hydraulic Erosion
 * 程序化地形生成 + 水力侵蚀模拟 + 软光栅化渲染
 *
 * 特性：
 * - Perlin噪声多倍频叠加生成基础地形
 * - 粒子化水力侵蚀模拟（数千个水滴迭代）
 * - 高度图着色（水/沙/草/岩/雪分层）
 * - 法线计算与Lambert漫射光照
 * - 透视投影软光栅化
 */

#include <cmath>
#include <cstring>
#include <vector>
#include <random>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <string>
#include <array>

// ============================================================
// 图像输出
// ============================================================
struct Color {
    uint8_t r, g, b;
};

struct Image {
    int width, height;
    std::vector<Color> pixels;

    Image(int w, int h) : width(w), height(h), pixels(w * h, {0, 0, 0}) {}

    void set(int x, int y, Color c) {
        if (x >= 0 && x < width && y >= 0 && y < height)
            pixels[y * width + x] = c;
    }

    Color get(int x, int y) const {
        if (x < 0) x = 0;
        if (x >= width) x = width - 1;
        if (y < 0) y = 0;
        if (y >= height) y = height - 1;
        return pixels[y * width + x];
    }

    void savePPM(const std::string& filename) const {
        std::ofstream f(filename, std::ios::binary);
        f << "P6\n" << width << " " << height << "\n255\n";
        for (const auto& c : pixels)
            f.write(reinterpret_cast<const char*>(&c), 3);
    }
};

// ============================================================
// Perlin 噪声
// ============================================================
class PerlinNoise {
    std::array<int, 512> p;
public:
    explicit PerlinNoise(uint32_t seed = 42) {
        std::mt19937 rng(seed);
        std::array<int, 256> perm;
        for (int i = 0; i < 256; ++i) perm[i] = i;
        std::shuffle(perm.begin(), perm.end(), rng);
        for (int i = 0; i < 512; ++i) p[i] = perm[i & 255];
    }

    static double fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    static double lerp(double a, double b, double t) { return a + t * (b - a); }
    static double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
    }

    double noise(double x, double y, double z = 0.0) const {
        int X = (int)std::floor(x) & 255;
        int Y = (int)std::floor(y) & 255;
        int Z = (int)std::floor(z) & 255;
        x -= std::floor(x); y -= std::floor(y); z -= std::floor(z);
        double u = fade(x), v = fade(y), w = fade(z);
        int A  = p[X] + Y,   AA = p[A] + Z, AB = p[A + 1] + Z;
        int B  = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
        return lerp(
            lerp(lerp(grad(p[AA], x, y, z),     grad(p[BA], x-1, y,   z),   u),
                 lerp(grad(p[AB], x, y-1, z),   grad(p[BB], x-1, y-1, z),   u), v),
            lerp(lerp(grad(p[AA+1], x, y, z-1), grad(p[BA+1], x-1, y,   z-1), u),
                 lerp(grad(p[AB+1], x, y-1, z-1), grad(p[BB+1], x-1, y-1, z-1), u), v), w);
    }

    // 分形布朗运动 (fBm)
    double fbm(double x, double y, int octaves, double lacunarity = 2.0, double gain = 0.5) const {
        double val = 0, amp = 0.5, freq = 1.0;
        for (int i = 0; i < octaves; ++i) {
            val += amp * noise(x * freq, y * freq);
            freq *= lacunarity;
            amp  *= gain;
        }
        return val;
    }
};

// ============================================================
// 地形高度图
// ============================================================
struct Terrain {
    int size;           // 地形分辨率
    std::vector<float> height;
    std::vector<float> water;   // 侵蚀后的水沉积量

    Terrain(int sz) : size(sz), height(sz * sz, 0.0f), water(sz * sz, 0.0f) {}

    float& h(int x, int y) { return height[y * size + x]; }
    float  h(int x, int y) const {
        if (x < 0 || x >= size || y < 0 || y >= size) return 0.0f;
        return height[y * size + x];
    }
    float& w(int x, int y) { return water[y * size + x]; }
};

// ============================================================
// 生成基础地形 (fBm噪声)
// ============================================================
void generateBaseTerrain(Terrain& terrain, uint32_t seed) {
    PerlinNoise pn(seed);
    int sz = terrain.size;
    for (int y = 0; y < sz; ++y) {
        for (int x = 0; x < sz; ++x) {
            double nx = x / (double)sz * 4.0;
            double ny = y / (double)sz * 4.0;
            // 6倍频fBm
            double h = pn.fbm(nx, ny, 6, 2.0, 0.5);
            // 山峰增强 (幂函数使山峰更尖锐)
            h = (h + 1.0) * 0.5; // 归一化到 [0,1]
            h = std::pow(h, 1.5);
            terrain.h(x, y) = (float)h;
        }
    }
}

// ============================================================
// 水力侵蚀模拟（Particle-based Hydraulic Erosion）
// ============================================================
struct Droplet {
    float x, y;
    float vx, vy;
    float water;
    float sediment;
};

void simulateHydraulicErosion(Terrain& terrain, int numDroplets, uint32_t seed) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> distPos(0.0f, (float)(terrain.size - 2));

    // 侵蚀参数
    const float inertia        = 0.05f;  // 方向惯性
    const float capacity       = 8.0f;   // 沉积物容量系数
    const float erosion        = 0.3f;   // 侵蚀速率
    const float deposition     = 0.3f;   // 沉积速率
    const float evaporation    = 0.01f;  // 蒸发速率
    const float gravity        = 4.0f;   // 重力
    const float minSlope       = 0.01f;  // 最小坡度
    const int   maxLifetime    = 30;     // 最大生命周期

    int sz = terrain.size;

    for (int d = 0; d < numDroplets; ++d) {
        Droplet drop;
        drop.x = distPos(rng);
        drop.y = distPos(rng);
        drop.vx = 0; drop.vy = 0;
        drop.water = 1.0f;
        drop.sediment = 0.0f;

        for (int life = 0; life < maxLifetime; ++life) {
            // 确保坐标在有效范围
            if (drop.x < 1 || drop.x >= sz-2 || drop.y < 1 || drop.y >= sz-2) break;

            int cellX = (int)drop.x;
            int cellY = (int)drop.y;
            float fracX = drop.x - cellX;
            float fracY = drop.y - cellY;

            if (cellX < 1 || cellX >= sz-2 || cellY < 1 || cellY >= sz-2) break;

            // 双线性插值高度和梯度
            float h00 = terrain.h(cellX,   cellY);
            float h10 = terrain.h(cellX+1, cellY);
            float h01 = terrain.h(cellX,   cellY+1);
            float h11 = terrain.h(cellX+1, cellY+1);

            float gradX = (h10 - h00) * (1 - fracY) + (h11 - h01) * fracY;
            float gradY = (h01 - h00) * (1 - fracX) + (h11 - h10) * fracX;
            float curH  = h00*(1-fracX)*(1-fracY) + h10*fracX*(1-fracY)
                        + h01*(1-fracX)*fracY      + h11*fracX*fracY;

            // 更新速度（梯度驱动 + 惯性）
            drop.vx = drop.vx * inertia - gradX * (1 - inertia);
            drop.vy = drop.vy * inertia - gradY * (1 - inertia);

            float speed = std::sqrt(drop.vx * drop.vx + drop.vy * drop.vy);
            if (speed < 1e-6f) break;

            // 归一化速度
            drop.vx /= speed;
            drop.vy /= speed;

            // 移动
            drop.x += drop.vx;
            drop.y += drop.vy;

            // 检查新位置是否在安全范围内（需要能访问 nX+1, nY+1）
            if (drop.x < 1 || drop.x >= sz-2 || drop.y < 1 || drop.y >= sz-2) {
                // 到达边界，沉积剩余沉积物
                float w00 = (1-fracX)*(1-fracY);
                float w10 = fracX*(1-fracY);
                float w01 = (1-fracX)*fracY;
                float w11 = fracX*fracY;
                terrain.h(cellX,   cellY)   += drop.sediment * w00;
                terrain.h(cellX+1, cellY)   += drop.sediment * w10;
                terrain.h(cellX,   cellY+1) += drop.sediment * w01;
                terrain.h(cellX+1, cellY+1) += drop.sediment * w11;
                break;
            }

            // 新位置高度（此时已确保 nX+1, nY+1 在范围内）
            int nX = (int)drop.x;
            int nY = (int)drop.y;
            float nfX = drop.x - nX;
            float nfY = drop.y - nY;
            // 额外安全检查
            if (nX < 0 || nX+1 >= sz || nY < 0 || nY+1 >= sz) break;
            float newH = terrain.h(nX,   nY)   * (1-nfX)*(1-nfY)
                       + terrain.h(nX+1, nY)   * nfX*(1-nfY)
                       + terrain.h(nX,   nY+1) * (1-nfX)*nfY
                       + terrain.h(nX+1, nY+1) * nfX*nfY;

            float deltaH = newH - curH;

            // 沉积物容量
            float sedCap = std::max(-deltaH * speed * drop.water * capacity, minSlope);

            if (drop.sediment > sedCap || deltaH > 0) {
                // 沉积
                float deposit = (deltaH > 0)
                    ? std::min(drop.sediment, deltaH)
                    : (drop.sediment - sedCap) * deposition;
                drop.sediment -= deposit;

                float w00 = (1-fracX)*(1-fracY);
                float w10 = fracX*(1-fracY);
                float w01 = (1-fracX)*fracY;
                float w11 = fracX*fracY;
                terrain.h(cellX,   cellY)   += deposit * w00;
                terrain.h(cellX+1, cellY)   += deposit * w10;
                terrain.h(cellX,   cellY+1) += deposit * w01;
                terrain.h(cellX+1, cellY+1) += deposit * w11;
                // 记录水沉积
                terrain.w(cellX, cellY) += 0.1f;
            } else {
                // 侵蚀
                float erode = std::min((sedCap - drop.sediment) * erosion, -deltaH);
                if (erode < 0) erode = 0;
                drop.sediment += erode;

                // 从周围4个顶点侵蚀
                float w00 = (1-fracX)*(1-fracY);
                float w10 = fracX*(1-fracY);
                float w01 = (1-fracX)*fracY;
                float w11 = fracX*fracY;
                terrain.h(cellX,   cellY)   -= erode * w00;
                terrain.h(cellX+1, cellY)   -= erode * w10;
                terrain.h(cellX,   cellY+1) -= erode * w01;
                terrain.h(cellX+1, cellY+1) -= erode * w11;
            }

            // 更新水量（蒸发）
            drop.water *= (1 - evaporation);
            if (drop.water < 0.01f) break;

            // 速度更新（加入重力）
            float velLen = std::sqrt(drop.vx*drop.vx + drop.vy*drop.vy);
            if (velLen > 1e-6f) {
                drop.vx = drop.vx / velLen * std::sqrt(speed*speed + deltaH * gravity);
                drop.vy = drop.vy / velLen * std::sqrt(speed*speed + deltaH * gravity);
            }
        }
    }

    // 高度归一化到 [0,1]
    float minH = *std::min_element(terrain.height.begin(), terrain.height.end());
    float maxH = *std::max_element(terrain.height.begin(), terrain.height.end());
    float range = maxH - minH;
    if (range > 1e-6f) {
        for (auto& h : terrain.height)
            h = (h - minH) / range;
    }
}

// ============================================================
// 计算法线
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    Vec3 normalized() const {
        float len = std::sqrt(x*x + y*y + z*z);
        if (len < 1e-6f) return {0, 1, 0};
        return {x/len, y/len, z/len};
    }
};

Vec3 terrainNormal(const Terrain& t, int x, int y) {
    float hL = t.h(x-1, y);
    float hR = t.h(x+1, y);
    float hD = t.h(x, y-1);
    float hU = t.h(x, y+1);
    // 地形尺度：x/z方向步长为1/size，高度方向有缩放
    float scale = (float)t.size * 0.5f;
    return Vec3(hL - hR, 2.0f / scale, hD - hU).normalized();
}

// ============================================================
// 地形着色（基于高度分层）
// ============================================================
Color terrainColor(float h, float slope, bool /*hasRiver*/) {
    // 水（低洼区域）
    if (h < 0.12f) {
        float t = h / 0.12f;
        return {(uint8_t)(20 + t*30), (uint8_t)(60 + t*50), (uint8_t)(120 + t*80)};
    }
    // 浅水/湿地
    if (h < 0.16f) {
        return {(uint8_t)80, (uint8_t)(120), (uint8_t)(60)};
    }
    // 沙滩/沙地
    if (h < 0.22f) {
        float t = (h - 0.16f) / 0.06f;
        return {(uint8_t)(194 + t*10), (uint8_t)(178 + t*5), (uint8_t)(128)};
    }
    // 草地
    if (h < 0.45f && slope < 0.7f) {
        float t = (h - 0.22f) / 0.23f;
        uint8_t g = (uint8_t)(100 + t*40);
        return {(uint8_t)(50 + t*20), g, (uint8_t)(40 + t*10)};
    }
    // 灌木/低矮植被
    if (h < 0.55f && slope < 0.6f) {
        float t = (h - 0.45f) / 0.10f;
        return {(uint8_t)(80 + t*20), (uint8_t)(90 + t*10), (uint8_t)(50)};
    }
    // 岩石
    if (h < 0.72f || slope >= 0.6f) {
        float t = std::min((h - 0.45f) / 0.27f, 1.0f);
        uint8_t v = (uint8_t)(100 + t*60);
        return {v, (uint8_t)(v*0.9f), (uint8_t)(v*0.85f)};
    }
    // 高山岩石
    if (h < 0.82f) {
        uint8_t v = (uint8_t)(120 + (h-0.72f)/0.10f * 50);
        return {v, (uint8_t)(v*0.95f), (uint8_t)(v*0.9f)};
    }
    // 雪
    float t = std::min((h - 0.82f) / 0.18f, 1.0f);
    uint8_t v = (uint8_t)(200 + t*55);
    return {v, v, (uint8_t)(v*1.0f)};
}

// ============================================================
// 俯视图渲染（高度图着色 + Lambert光照）
// ============================================================
void renderTopView(const Terrain& terrain, Image& img) {
    int sz = terrain.size;
    int W = img.width, H = img.height;

    // 光方向（斜照）
    Vec3 sunDir = Vec3(1, 3, 1).normalized();

    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            // 映射到地形坐标
            int tx = (int)(px * (sz - 1.0f) / (W - 1));
            int ty = (int)(py * (sz - 1.0f) / (H - 1));
            tx = std::clamp(tx, 0, sz-1);
            ty = std::clamp(ty, 0, sz-1);

            float h = terrain.h(tx, ty);
            Vec3 normal = terrainNormal(terrain, tx, ty);

            // 坡度（法线y分量偏离竖直的程度）
            float slope = 1.0f - normal.y;

            // Lambert漫射 + 环境光
            float diffuse = std::max(0.0f, normal.dot(sunDir));
            float ambient = 0.35f;
            float light = ambient + (1.0f - ambient) * diffuse;

            Color baseCol = terrainColor(h, slope, false);

            // 阴影（简单基于高度变化的近似AO）
            float ao = 1.0f;
            {
                // 采样附近高度做近似AO
                float hCenter = h;
                float hMax = hCenter;
                for (int si = 1; si <= 3; ++si) {
                    int sx = tx + si, sy = ty - si;
                    if (sx < sz && sy >= 0)
                        hMax = std::max(hMax, terrain.h(sx, sy));
                }
                float occlusion = std::max(0.0f, (hMax - hCenter) * 5.0f);
                ao = 1.0f / (1.0f + occlusion * 0.5f);
            }

            float finalLight = light * ao;
            finalLight = std::clamp(finalLight, 0.0f, 1.0f);

            img.set(px, py, {
                (uint8_t)(baseCol.r * finalLight),
                (uint8_t)(baseCol.g * finalLight),
                (uint8_t)(baseCol.b * finalLight)
            });
        }
    }
}

// ============================================================
// 透视 3D 渲染（等角斜视）
// ============================================================
void render3DPerspective(const Terrain& terrain, Image& img) {
    int sz = terrain.size;
    int W = img.width, H = img.height;

    // 深度缓冲
    std::vector<float> zbuf(W * H, 1e30f);
    // 清空为天空色
    for (int py = 0; py < H; ++py)
        for (int px = 0; px < W; ++px)
            img.set(px, py, {100, 150, 200}); // 天空蓝

    Vec3 sunDir = Vec3(1.5f, 4.0f, 1.0f).normalized();

    // 等角投影参数（从右上俯视）
    // 地形中心在(0,0,0)，x/z范围[-1,1]，高度范围[0, 0.5]
    float heightScale = 0.5f;

    // 投影：斜45度俯视
    // world -> screen 变换
    auto project = [&](float wx, float wy, float wz) -> std::pair<int,int> {
        // 等角投影
        float sx = (wx - wz) * 0.866f;           // cos30 ≈ 0.866
        float sy = (wx + wz) * 0.5f - wy * 1.2f; // 高度方向拉伸
        // 归一化到屏幕
        int px = (int)((sx + 1.5f) / 3.0f * W);
        int py = (int)((sy + 1.2f) / 2.4f * H);
        return {px, py};
    };

    // 从后到前绘制地形条带（Painter's算法）
    // 按 z 从大到小（从后向前）
    for (int y = sz-1; y >= 0; --y) {
        for (int x = sz-1; x >= 0; --x) {
            float wx = (x / (float)(sz-1) * 2.0f - 1.0f);
            float wz = (y / (float)(sz-1) * 2.0f - 1.0f);
            float wy = terrain.h(x, y) * heightScale;

            // 计算法线用于光照
            Vec3 normal = terrainNormal(terrain, x, y);
            float diffuse = std::max(0.0f, normal.dot(sunDir));
            float ambient = 0.35f;
            float light = ambient + (1.0f - ambient) * diffuse;

            float h = terrain.h(x, y);
            float slope = 1.0f - normal.y;
            Color baseCol = terrainColor(h, slope, false);

            Color finalCol = {
                (uint8_t)(baseCol.r * light),
                (uint8_t)(baseCol.g * light),
                (uint8_t)(baseCol.b * light)
            };

            // 投影到屏幕，绘制小方块
            auto [px, py_] = project(wx, wy, wz);

            // 绘制 2x2 的块
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    int spx = px + dx, spy = py_ + dy;
                    if (spx < 0 || spx >= W || spy < 0 || spy >= H) continue;
                    float depth = wz - wy; // 用于深度排序
                    int idx = spy * W + spx;
                    if (depth < zbuf[idx]) {
                        zbuf[idx] = depth;
                        img.set(spx, spy, finalCol);
                    }
                }
            }
        }
    }
}

// ============================================================
// 保存 PNG（通过 PPM 中间格式 + ImageMagick 转换）
// ============================================================
bool savePNG(const Image& img, const std::string& filename) {
    std::string ppmFile = filename + ".ppm";
    img.savePPM(ppmFile);
    // 使用 Python PIL 转换 PPM -> PNG
    std::string cmd = "python3 -c \"from PIL import Image; img=Image.open('" 
                    + ppmFile + "'); img.save('" + filename + "')\" 2>/dev/null";
    int ret = std::system(cmd.c_str());
    if (ret == 0) {
        std::remove(ppmFile.c_str());
    }
    return (ret == 0);
}

// ============================================================
// 主程序
// ============================================================
int main() {
    std::cout << "=== Procedural Terrain with Hydraulic Erosion ===" << std::endl;

    const int TERRAIN_SIZE = 256;   // 地形分辨率
    const int NUM_DROPLETS = 80000; // 侵蚀水滴数
    const int IMG_W = 800;
    const int IMG_H = 600;

    // 1. 生成基础地形
    std::cout << "[1/5] 生成 Perlin fBm 地形 (" << TERRAIN_SIZE << "x" << TERRAIN_SIZE << ")..." << std::endl;
    Terrain terrain(TERRAIN_SIZE);
    generateBaseTerrain(terrain, 12345);

    // 2. 保存侵蚀前高度图（用于对比）
    std::cout << "[2/5] 渲染侵蚀前地形..." << std::endl;
    {
        Image imgBefore(IMG_W, IMG_H);
        renderTopView(terrain, imgBefore);
        savePNG(imgBefore, "terrain_before_erosion.png");
    }

    // 3. 水力侵蚀模拟
    std::cout << "[3/5] 模拟水力侵蚀 (" << NUM_DROPLETS << " 水滴)..." << std::endl;
    simulateHydraulicErosion(terrain, NUM_DROPLETS, 54321);

    // 4. 渲染侵蚀后俯视图
    std::cout << "[4/5] 渲染侵蚀后地形（俯视）..." << std::endl;
    {
        Image imgTop(IMG_W, IMG_H);
        renderTopView(terrain, imgTop);
        savePNG(imgTop, "terrain_top_view.png");
    }

    // 5. 3D 等角透视渲染
    std::cout << "[5/5] 渲染 3D 等角视角..." << std::endl;
    {
        Image img3D(IMG_W, IMG_H);
        render3DPerspective(terrain, img3D);
        savePNG(img3D, "terrain_3d_view.png");
    }

    // 输出统计信息
    float minH = *std::min_element(terrain.height.begin(), terrain.height.end());
    float maxH = *std::max_element(terrain.height.begin(), terrain.height.end());
    float sumH = 0;
    for (float h : terrain.height) sumH += h;
    float avgH = sumH / terrain.height.size();

    std::cout << "\n=== 地形统计 ===" << std::endl;
    std::cout << "最低高度: " << minH << std::endl;
    std::cout << "最高高度: " << maxH << std::endl;
    std::cout << "平均高度: " << avgH << std::endl;
    std::cout << "\n输出文件：" << std::endl;
    std::cout << "  terrain_before_erosion.png  (侵蚀前地形)" << std::endl;
    std::cout << "  terrain_top_view.png        (侵蚀后俯视)" << std::endl;
    std::cout << "  terrain_3d_view.png         (3D等角视角)" << std::endl;
    std::cout << "\n✅ 所有阶段完成！" << std::endl;

    return 0;
}
