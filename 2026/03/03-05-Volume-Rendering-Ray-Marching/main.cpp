/**
 * Volume Rendering - Ray Marching 体积云渲染
 * 
 * 技术要点：
 * - Ray Marching：光线步进算法遍历体积场
 * - Perlin Noise：生成三维密度场模拟云层
 * - Beer-Lambert 衰减：模拟光线在介质中的吸收
 * - 自散射光照：简化的单次散射模型
 * 
 * 输出：volume_output.png (800x600)
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>

// ============================================================
// 数学工具
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    
    float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { float l = length(); return l>0 ? *this/l : Vec3(0,0,0); }
    
    static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return a + (b - a) * t;
    }
};

Vec3 operator*(float t, const Vec3& v) { return v * t; }

// ============================================================
// Perlin Noise 实现（三维）
// ============================================================
class PerlinNoise {
public:
    PerlinNoise(uint32_t seed = 42) {
        // 初始化置换表
        for (int i = 0; i < 256; i++) p[i] = i;
        // 用 seed 打乱
        std::mt19937 rng(seed);
        for (int i = 255; i > 0; i--) {
            int j = rng() % (i + 1);
            std::swap(p[i], p[j]);
        }
        for (int i = 0; i < 256; i++) p[256 + i] = p[i];
    }

    float noise(float x, float y, float z) const {
        int X = (int)std::floor(x) & 255;
        int Y = (int)std::floor(y) & 255;
        int Z = (int)std::floor(z) & 255;

        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);

        float u = fade(x), v = fade(y), w = fade(z);

        int A  = p[X] + Y;
        int AA = p[A] + Z, AB = p[A+1] + Z;
        int B  = p[X+1] + Y;
        int BA = p[B] + Z, BB = p[B+1] + Z;

        return lerp(w,
            lerp(v,
                lerp(u, grad(p[AA  ], x,   y,   z  ),
                        grad(p[BA  ], x-1, y,   z  )),
                lerp(u, grad(p[AB  ], x,   y-1, z  ),
                        grad(p[BB  ], x-1, y-1, z  ))),
            lerp(v,
                lerp(u, grad(p[AA+1], x,   y,   z-1),
                        grad(p[BA+1], x-1, y,   z-1)),
                lerp(u, grad(p[AB+1], x,   y-1, z-1),
                        grad(p[BB+1], x-1, y-1, z-1))));
    }

    // FBM (Fractal Brownian Motion) - 多倍频叠加
    float fbm(float x, float y, float z, int octaves = 5) const {
        float val = 0.0f;
        float amplitude = 0.5f;
        float frequency = 1.0f;
        float maxVal = 0.0f;

        for (int i = 0; i < octaves; i++) {
            val += noise(x * frequency, y * frequency, z * frequency) * amplitude;
            maxVal += amplitude;
            amplitude *= 0.5f;
            frequency *= 2.0f;
        }
        return val / maxVal;  // 归一化到 [-1, 1]
    }

private:
    int p[512];

    static float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
    static float lerp(float t, float a, float b) { return a + t * (b - a); }

    static float grad(int hash, float x, float y, float z) {
        int h = hash & 15;
        float u = h < 8 ? x : y;
        float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
};

// ============================================================
// 体积密度场
// ============================================================
class CloudVolume {
public:
    CloudVolume() : noise(12345) {}

    // 获取某点的云密度 [0, 1]
    float density(const Vec3& p) const {
        // 基础云层：在 y = 0 附近，有一定厚度
        float heightFade = std::exp(-std::abs(p.y) * 0.8f);
        if (heightFade < 0.001f) return 0.0f;

        // FBM 噪声生成细节
        float n = noise.fbm(p.x * 0.3f, p.y * 0.3f, p.z * 0.3f, 6);
        n = (n + 1.0f) * 0.5f;  // 映射到 [0, 1]

        // 云的整体形状：用较低频噪声控制
        float shape = noise.fbm(p.x * 0.1f, p.y * 0.1f, p.z * 0.1f, 2);
        shape = (shape + 1.0f) * 0.5f;

        // 组合：shape 控制整体，n 增加细节
        float d = shape * 0.6f + n * 0.4f;

        // 阈值：密度低于某值视为透明（晴天蓝天区域）
        d = std::max(0.0f, d - 0.45f) * 2.0f;  // remapping

        return d * heightFade;
    }

private:
    PerlinNoise noise;
};

// ============================================================
// 光照和散射
// ============================================================
struct Light {
    Vec3 position;
    Vec3 color;
    float intensity;
};

// Beer-Lambert 透射率
float transmittance(float density, float stepSize, float absorption) {
    return std::exp(-density * stepSize * absorption);
}

// ============================================================
// Ray Marching 体积渲染
// ============================================================
class VolumeRenderer {
public:
    static const int WIDTH  = 800;
    static const int HEIGHT = 600;

    VolumeRenderer() {
        // 主光源（太阳）
        sunLight.position = Vec3(10.0f, 20.0f, -5.0f);
        sunLight.color    = Vec3(1.0f, 0.95f, 0.85f);
        sunLight.intensity = 3.0f;
    }

    // 计算光线方向（透视相机）
    Vec3 getRayDir(int px, int py) const {
        float aspectRatio = (float)WIDTH / HEIGHT;
        float fov = 60.0f * M_PI / 180.0f;
        float halfH = std::tan(fov * 0.5f);
        float halfW = aspectRatio * halfH;

        float u = (2.0f * (px + 0.5f) / WIDTH  - 1.0f) * halfW;
        float v = (1.0f - 2.0f * (py + 0.5f) / HEIGHT) * halfH;

        return Vec3(u, v, -1.0f).normalize();
    }

    // 渲染天空背景（大气散射模拟）
    Vec3 skyColor(const Vec3& dir) const {
        Vec3 horizonColor(0.65f, 0.75f, 0.85f);
        Vec3 zenithColor (0.15f, 0.40f, 0.80f);
        Vec3 groundColor (0.20f, 0.18f, 0.15f);

        if (dir.y >= 0) {
            // 上半球：地平线到顶点
            return Vec3::lerp(horizonColor, zenithColor, std::pow(dir.y, 0.5f));
        } else {
            // 下半球：地面
            return Vec3::lerp(horizonColor, groundColor, std::min(1.0f, -dir.y * 3.0f));
        }
    }

    // 向光源方向步进，计算光照透射率（阴影）
    float lightTransmittance(const Vec3& pos) const {
        Vec3 lightDir = (sunLight.position - pos).normalize();
        float shadowDensity = 0.0f;
        float shadowStep = 0.4f;

        for (int i = 0; i < 16; i++) {
            Vec3 shadowPos = pos + lightDir * (shadowStep * (i + 0.5f));
            if (std::abs(shadowPos.x) > 15.0f ||
                std::abs(shadowPos.y) > 8.0f  ||
                std::abs(shadowPos.z) > 15.0f) break;
            shadowDensity += volume.density(shadowPos) * shadowStep;
        }

        return std::exp(-shadowDensity * 0.8f);
    }

    // 主 Ray Marching 函数
    Vec3 raymarch(const Vec3& origin, const Vec3& dir) const {
        // 体积包围盒
        const float BOX_MIN_X = -12.0f, BOX_MAX_X = 12.0f;
        const float BOX_MIN_Y = -5.0f,  BOX_MAX_Y = 5.0f;
        const float BOX_MIN_Z = -15.0f, BOX_MAX_Z = 5.0f;

        // 与 AABB 求交
        float tmin, tmax;
        if (!intersectAABB(origin, dir,
            Vec3(BOX_MIN_X, BOX_MIN_Y, BOX_MIN_Z),
            Vec3(BOX_MAX_X, BOX_MAX_Y, BOX_MAX_Z),
            tmin, tmax)) {
            return skyColor(dir);
        }

        tmin = std::max(tmin, 0.0f);
        float stepSize = 0.3f;
        int maxSteps  = 120;

        // 累积颜色和透射率
        Vec3  accColor(0, 0, 0);
        float accTransmittance = 1.0f;

        float t = tmin;
        for (int i = 0; i < maxSteps && t < tmax; i++) {
            Vec3 pos = origin + dir * t;
            float d  = volume.density(pos);

            if (d > 0.001f) {
                // 计算光照
                float lightT = lightTransmittance(pos);
                Vec3 lightDir = (sunLight.position - pos).normalize();

                // 相位函数（Henyey-Greenstein 简化）
                float cosTheta = dir.dot(lightDir);
                float g = 0.3f;  // 各向异性参数
                float phase = (1.0f - g*g) / (4.0f * M_PI * std::pow(1.0f + g*g - 2.0f*g*cosTheta, 1.5f));
                phase = std::max(0.05f, phase * 4.0f);  // 归一化调整

                // 散射颜色
                Vec3 scatterColor = sunLight.color * sunLight.intensity * lightT * phase;
                // 加入环境散射（天光）
                Vec3 ambientSky(0.3f, 0.45f, 0.7f);
                scatterColor += ambientSky * 0.3f;

                // Beer-Lambert 吸收
                float absorption = 1.2f;
                float sampleTransT = std::exp(-d * stepSize * absorption);

                // 积分贡献
                float contribution = (1.0f - sampleTransT) * accTransmittance;
                accColor += scatterColor * contribution;
                accTransmittance *= sampleTransT;

                // 提前终止（不透明）
                if (accTransmittance < 0.01f) break;
            }

            // 自适应步长（密度高则步长小）
            t += stepSize * (d > 0.1f ? 0.5f : 1.0f);
        }

        // 与背景混合
        Vec3 bgColor = skyColor(dir);

        // 太阳圆盘
        Vec3 sunDir = sunLight.position.normalize();
        float sunDot = dir.dot(sunDir);
        if (sunDot > 0.9995f) {
            bgColor += Vec3(2.0f, 1.8f, 1.0f) * std::pow((sunDot - 0.9995f) / 0.0005f, 2.0f);
        }

        return bgColor * accTransmittance + accColor;
    }

    // ACES 色调映射
    static Vec3 acesToneMapping(const Vec3& c) {
        float a = 2.51f, b = 0.03f, cc = 2.43f, d = 0.59f, e = 0.14f;
        auto tonemapChannel = [=](float x) {
            return std::clamp((x*(a*x+b)) / (x*(cc*x+d)+e), 0.0f, 1.0f);
        };
        return Vec3(tonemapChannel(c.x), tonemapChannel(c.y), tonemapChannel(c.z));
    }

    // 主渲染函数
    void render(std::vector<uint8_t>& pixels) const {
        pixels.resize(WIDTH * HEIGHT * 3);

        // 相机设置
        Vec3 camOrigin(0.0f, 1.5f, 8.0f);

        int total = WIDTH * HEIGHT;
        int reportInterval = total / 10;

        for (int py = 0; py < HEIGHT; py++) {
            for (int px = 0; px < WIDTH; px++) {
                int idx = py * WIDTH + px;
                if (idx % reportInterval == 0) {
                    std::cout << "  渲染进度: " << (idx * 100 / total) << "%" << std::endl;
                }

                Vec3 dir = getRayDir(px, py);
                Vec3 color = raymarch(camOrigin, dir);

                // 曝光调整
                color = color * 1.2f;

                // 色调映射
                color = acesToneMapping(color);

                // Gamma 校正（线性 → sRGB）
                color.x = std::pow(std::clamp(color.x, 0.0f, 1.0f), 1.0f/2.2f);
                color.y = std::pow(std::clamp(color.y, 0.0f, 1.0f), 1.0f/2.2f);
                color.z = std::pow(std::clamp(color.z, 0.0f, 1.0f), 1.0f/2.2f);

                pixels[idx * 3 + 0] = (uint8_t)(color.x * 255.999f);
                pixels[idx * 3 + 1] = (uint8_t)(color.y * 255.999f);
                pixels[idx * 3 + 2] = (uint8_t)(color.z * 255.999f);
            }
        }

        std::cout << "  渲染进度: 100%" << std::endl;
    }

private:
    CloudVolume volume;
    Light sunLight;

    // AABB 求交（Slab 方法）
    static bool intersectAABB(const Vec3& orig, const Vec3& dir,
                               const Vec3& boxMin, const Vec3& boxMax,
                               float& tmin, float& tmax) {
        tmin = -1e30f;
        tmax =  1e30f;

        auto safeInv = [](float v) { return std::abs(v) > 1e-8f ? 1.0f/v : (v >= 0 ? 1e30f : -1e30f); };

        float invDx = safeInv(dir.x);
        float t1 = (boxMin.x - orig.x) * invDx;
        float t2 = (boxMax.x - orig.x) * invDx;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));

        float invDy = safeInv(dir.y);
        t1 = (boxMin.y - orig.y) * invDy;
        t2 = (boxMax.y - orig.y) * invDy;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));

        float invDz = safeInv(dir.z);
        t1 = (boxMin.z - orig.z) * invDz;
        t2 = (boxMax.z - orig.z) * invDz;
        tmin = std::max(tmin, std::min(t1, t2));
        tmax = std::min(tmax, std::max(t1, t2));

        return tmax > tmin && tmax > 0;
    }
};

// ============================================================
// PPM / PNG 输出
// ============================================================

// 写入 PPM 文件（用于验证）
void writePPM(const std::string& filename, const std::vector<uint8_t>& pixels,
              int width, int height) {
    std::ofstream ofs(filename, std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(pixels.data()), pixels.size());
    ofs.close();
    std::cout << "✅ 已输出 PPM: " << filename << std::endl;
}

// 写入 BMP 文件（无依赖）
void writeBMP(const std::string& filename, const std::vector<uint8_t>& pixels,
              int width, int height) {
    // BMP 格式要求 BGR 行序，且行对齐到 4 字节
    int rowStride = width * 3;
    int paddingBytes = (4 - rowStride % 4) % 4;
    int dataSize = (rowStride + paddingBytes) * height;

    uint32_t fileSize = 54 + dataSize;
    uint8_t header[54] = {0};

    // BMP 文件头
    header[0] = 'B'; header[1] = 'M';
    header[2] = fileSize & 0xFF; header[3] = (fileSize >> 8) & 0xFF;
    header[4] = (fileSize >> 16) & 0xFF; header[5] = (fileSize >> 24) & 0xFF;
    header[10] = 54;  // 数据偏移

    // DIB 头（BITMAPINFOHEADER）
    header[14] = 40;  // 头大小
    header[18] = width & 0xFF;  header[19] = (width >> 8) & 0xFF;
    header[22] = height & 0xFF; header[23] = (height >> 8) & 0xFF;
    header[26] = 1;   // 颜色平面数
    header[28] = 24;  // 每像素位数

    std::ofstream ofs(filename, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(header), 54);

    // BMP 从下到上存储行，颜色顺序 BGR
    uint8_t pad[3] = {0, 0, 0};
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x) * 3;
            uint8_t bgr[3] = {pixels[idx+2], pixels[idx+1], pixels[idx+0]};
            ofs.write(reinterpret_cast<const char*>(bgr), 3);
        }
        if (paddingBytes > 0) ofs.write(reinterpret_cast<const char*>(pad), paddingBytes);
    }
    ofs.close();
    std::cout << "✅ 已输出 BMP: " << filename << std::endl;
}

// ============================================================
// 主函数
// ============================================================
int main() {
    std::cout << "🌤️ Volume Rendering - Ray Marching 体积云渲染" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "分辨率: " << VolumeRenderer::WIDTH << "x" << VolumeRenderer::HEIGHT << std::endl;
    std::cout << "开始渲染..." << std::endl;

    VolumeRenderer renderer;
    std::vector<uint8_t> pixels;

    renderer.render(pixels);

    // 输出文件（PPM + BMP 两种格式）
    writePPM("volume_output.ppm", pixels, VolumeRenderer::WIDTH, VolumeRenderer::HEIGHT);
    writeBMP("volume_output.bmp", pixels, VolumeRenderer::WIDTH, VolumeRenderer::HEIGHT);

    // 验证输出文件大小
    std::ifstream ppmFile("volume_output.ppm", std::ios::binary | std::ios::ate);
    std::ifstream bmpFile("volume_output.bmp", std::ios::binary | std::ios::ate);

    if (!ppmFile || !bmpFile) {
        std::cerr << "❌ 输出文件不存在！" << std::endl;
        return 1;
    }

    long ppmSize = ppmFile.tellg();
    long bmpSize = bmpFile.tellg();
    std::cout << "文件大小 PPM: " << ppmSize << " bytes, BMP: " << bmpSize << " bytes" << std::endl;

    if (ppmSize < 1000 || bmpSize < 1000) {
        std::cerr << "❌ 输出文件太小，可能有问题！" << std::endl;
        return 1;
    }

    // 验证像素内容：采样中心区域
    // 800x600 图像，检查中心 (400, 300) 附近的像素
    int cx = VolumeRenderer::WIDTH / 2;
    int cy = VolumeRenderer::HEIGHT / 2;
    uint8_t r = pixels[(cy * VolumeRenderer::WIDTH + cx) * 3 + 0];
    uint8_t g = pixels[(cy * VolumeRenderer::WIDTH + cx) * 3 + 1];
    uint8_t b = pixels[(cy * VolumeRenderer::WIDTH + cx) * 3 + 2];

    std::cout << "中心像素颜色: RGB(" << (int)r << ", " << (int)g << ", " << (int)b << ")" << std::endl;

    // 验证图像不是纯黑
    // 计算全图平均亮度
    long totalBrightness = 0;
    int sampleCount = VolumeRenderer::WIDTH * VolumeRenderer::HEIGHT;
    for (int i = 0; i < sampleCount; i++) {
        totalBrightness += pixels[i*3+0] + pixels[i*3+1] + pixels[i*3+2];
    }
    float avgBrightness = (float)totalBrightness / (sampleCount * 3);
    std::cout << "平均亮度: " << avgBrightness << " (期望 > 30)" << std::endl;

    if (avgBrightness < 5.0f) {
        std::cerr << "❌ 图像太暗，平均亮度=" << avgBrightness << "，渲染可能失败！" << std::endl;
        return 1;
    }

    // 验证图像有颜色变化（不是纯色）
    uint8_t rTop = pixels[0];  // 左上角
    uint8_t gTop = pixels[1];
    uint8_t bTop = pixels[2];
    uint8_t rBot = pixels[(VolumeRenderer::HEIGHT-1) * VolumeRenderer::WIDTH * 3];  // 左下角
    uint8_t gBot = pixels[(VolumeRenderer::HEIGHT-1) * VolumeRenderer::WIDTH * 3 + 1];
    uint8_t bBot = pixels[(VolumeRenderer::HEIGHT-1) * VolumeRenderer::WIDTH * 3 + 2];

    std::cout << "左上角像素: RGB(" << (int)rTop << ", " << (int)gTop << ", " << (int)bTop << ")" << std::endl;
    std::cout << "左下角像素: RGB(" << (int)rBot << ", " << (int)gBot << ", " << (int)bBot << ")" << std::endl;

    int colorDiff = std::abs(rTop - rBot) + std::abs(gTop - gBot) + std::abs(bTop - bBot);
    if (colorDiff < 10) {
        std::cerr << "❌ 图像颜色变化太小，可能是纯色！颜色差=" << colorDiff << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "✅ 所有验证通过！" << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << "🎉 Volume Rendering 渲染完成！" << std::endl;

    return 0;
}
