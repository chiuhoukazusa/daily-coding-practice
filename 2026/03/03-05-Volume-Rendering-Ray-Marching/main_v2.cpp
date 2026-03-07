/**
 * Volume Rendering - Ray Marching 体积云渲染 v2
 * 
 * 改进：
 * - 拉远视角，展示更大规模云层
 * - 增强云层形状对比（厚薄分明，明暗分明）
 * - 更强的自遮蔽阴影（云底明显暗）
 * - 大气散射（地平线颜色变化）
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
    Vec3 normalize() const { float l = length(); return l > 1e-8f ? *this/l : Vec3(0,0,1); }

    static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return a + (b - a) * t;
    }
};

Vec3 operator*(float t, const Vec3& v) { return v * t; }

// clamp helper
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// ============================================================
// Perlin Noise（标准实现）
// ============================================================
class PerlinNoise {
public:
    PerlinNoise(uint32_t seed = 42) {
        for (int i = 0; i < 256; i++) p[i] = i;
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
        x -= std::floor(x); y -= std::floor(y); z -= std::floor(z);
        float u = fade(x), v = fade(y), w = fade(z);
        int A  = p[X]+Y,  AA = p[A]+Z,  AB = p[A+1]+Z;
        int B  = p[X+1]+Y, BA = p[B]+Z, BB = p[B+1]+Z;
        return lerp(w,
            lerp(v, lerp(u, grad(p[AA  ],x,  y,  z  ), grad(p[BA  ],x-1,y,  z  )),
                    lerp(u, grad(p[AB  ],x,  y-1,z  ), grad(p[BB  ],x-1,y-1,z  ))),
            lerp(v, lerp(u, grad(p[AA+1],x,  y,  z-1), grad(p[BA+1],x-1,y,  z-1)),
                    lerp(u, grad(p[AB+1],x,  y-1,z-1), grad(p[BB+1],x-1,y-1,z-1))));
    }

    // FBM：多倍频叠加，产生分形细节
    float fbm(float x, float y, float z, int octaves = 6) const {
        float val = 0.0f, amplitude = 0.5f, frequency = 1.0f, maxVal = 0.0f;
        for (int i = 0; i < octaves; i++) {
            val      += noise(x*frequency, y*frequency, z*frequency) * amplitude;
            maxVal   += amplitude;
            amplitude *= 0.5f;
            frequency *= 2.0f;
        }
        return val / maxVal;
    }

private:
    int p[512];
    static float fade(float t) { return t*t*t*(t*(t*6-15)+10); }
    static float lerp(float t, float a, float b) { return a + t*(b-a); }
    static float grad(int hash, float x, float y, float z) {
        int h = hash & 15;
        float u = h<8 ? x : y;
        float v = h<4 ? y : (h==12||h==14 ? x : z);
        return ((h&1)==0 ? u : -u) + ((h&2)==0 ? v : -v);
    }
};

// ============================================================
// 云层密度场
// ============================================================
class CloudVolume {
public:
    CloudVolume() : noise(9527) {}

    float density(const Vec3& p) const {
        // 1. 高度衰减：云层集中在 y ∈ [-2, 4]
        //    用 smoothstep 做软边界，使云底/云顶边缘自然
        float yBottom = smoothstep(-2.0f, 0.5f, p.y);    // 云底渐入（更窄）
        float yTop    = 1.0f - smoothstep(1.5f, 4.5f, p.y); // 云顶渐出
        float heightMask = yBottom * yTop;
        if (heightMask < 0.001f) return 0.0f;

        // 2. 大尺度形状（低频）：决定云团位置
        float shape = noise.fbm(p.x*0.05f, p.y*0.08f, p.z*0.05f, 3);
        shape = (shape + 1.0f) * 0.5f;  // 映射到 [0,1]

        // 3. 中频细节：云的卷曲边缘
        float detail = noise.fbm(p.x*0.18f, p.y*0.25f, p.z*0.18f, 4);
        detail = (detail + 1.0f) * 0.5f;

        // 4. 高频细节：表面絮状纹理
        float fine = noise.fbm(p.x*0.55f, p.y*0.70f, p.z*0.55f, 3);
        fine = (fine + 1.0f) * 0.5f;

        // 5. 组合：大形状主导，细节增加变化感
        float combined = shape * 0.60f + detail * 0.28f + fine * 0.12f;

        // 6. 密度截断 + 幂次增强对比
        //    threshold 越高 → 云越稀少但越扎实
        float d = std::max(0.0f, combined - 0.46f) / 0.54f;
        d = std::pow(d, 1.6f);  // 幂次压缩：让云心浓、边缘薄

        return d * heightMask * 3.5f;  // 高密度上限
    }

private:
    PerlinNoise noise;

    static float smoothstep(float edge0, float edge1, float x) {
        float t = clampf((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    }
};

// ============================================================
// 光线追踪
// ============================================================
class VolumeRenderer {
public:
    static const int WIDTH  = 1200;
    static const int HEIGHT = 675;  // 16:9

    VolumeRenderer() {
        // 太阳位置：右上方 45°，侧光让云有明暗
        sunDir = Vec3(0.6f, 0.75f, -0.2f).normalize();
        sunColor = Vec3(1.0f, 0.90f, 0.72f);
        sunIntensity = 3.5f;
    }

    // 相机光线方向（透视投影）
    Vec3 getRayDir(int px, int py) const {
        float aspect = (float)WIDTH / HEIGHT;
        float fovY   = 45.0f * (float)M_PI / 180.0f;  // 更小视角 = 感觉更近/更宏大
        float halfH  = std::tan(fovY * 0.5f);
        float halfW  = aspect * halfH;

        float u = (2.0f*(px+0.5f)/WIDTH  - 1.0f) * halfW;
        float v = (1.0f - 2.0f*(py+0.5f)/HEIGHT) * halfH;
        return Vec3(u, v, -1.0f).normalize();
    }

    // 大气天空颜色
    Vec3 skyColor(const Vec3& dir) const {
        // 地平线暖色 → 天顶蓝色
        Vec3 horizon (0.72f, 0.78f, 0.88f);
        Vec3 zenith  (0.18f, 0.38f, 0.75f);
        Vec3 ground  (0.22f, 0.20f, 0.16f);

        float t = clampf(dir.y, -1.0f, 1.0f);
        Vec3 sky;
        if (t >= 0.0f) {
            sky = Vec3::lerp(horizon, zenith, std::pow(t, 0.4f));
        } else {
            sky = Vec3::lerp(horizon, ground, clampf(-t * 4.0f, 0.0f, 1.0f));
        }

        // 太阳圆盘 + 光晕
        float sunDot = dir.dot(sunDir);
        if (sunDot > 0.9998f) {
            sky += Vec3(3.0f, 2.6f, 1.8f) * 5.0f;
        } else if (sunDot > 0.996f) {
            float t2 = (sunDot - 0.996f) / (0.9998f - 0.996f);
            sky += Vec3(1.5f, 1.2f, 0.7f) * std::pow(t2, 3.0f);
        }

        return sky;
    }

    // 向太阳方向采样：计算某点的阴影透射率
    // （光线被周围云层遮挡多少）
    float lightTransmittance(const Vec3& pos) const {
        const int SHADOW_STEPS = 8;
        const float SHADOW_STEP_SIZE = 0.6f;
        float shadowAbs = 0.0f;

        for (int i = 0; i < SHADOW_STEPS; i++) {
            Vec3 shadowPos = pos + sunDir * (SHADOW_STEP_SIZE * (i + 0.5f));
            shadowAbs += volume.density(shadowPos) * SHADOW_STEP_SIZE;
        }
        // Beer-Lambert：透射率 = e^(-吸收量)
        return std::exp(-shadowAbs * 3.0f);
    }

    // 主 Ray Marching 循环
    Vec3 raymarch(const Vec3& origin, const Vec3& dir) const {
        // 渲染云层的 AABB 包围盒（大场景）
        Vec3 bMin(-30.0f, -4.0f, -60.0f);
        Vec3 bMax( 30.0f,  6.0f,  10.0f);

        float tmin, tmax;
        if (!intersectAABB(origin, dir, bMin, bMax, tmin, tmax)) {
            return skyColor(dir);
        }
        tmin = std::max(tmin, 0.1f);

        // Ray Marching 参数
        const float STEP_SIZE  = 0.35f;
        const int   MAX_STEPS  = 180;
        const float ABSORPTION = 1.8f;  // 云的吸光系数

        Vec3  accColor(0, 0, 0);
        float transmittance = 1.0f;  // 当前透射率（初始1.0=完全透明）

        float t = tmin;
        for (int step = 0; step < MAX_STEPS && t < tmax; step++) {
            Vec3  pos = origin + dir * t;
            float d   = volume.density(pos);

            if (d > 0.002f) {
                // ---- 光照计算 ----
                // 1. 来自太阳的直射光（穿过上方云层的透射率）
                float lightT = lightTransmittance(pos);

                // 2. Henyey-Greenstein 相位函数
                //    描述散射方向：g>0 前向散射（云边缘亮），g<0 后向散射
                float cosTheta = dir.dot(sunDir);
                float g = 0.35f;
                float denom = 1.0f + g*g - 2.0f*g*cosTheta;
                float phase = (1.0f - g*g) / (4.0f*(float)M_PI * std::pow(denom, 1.5f));
                phase = clampf(phase * 4.5f, 0.08f, 5.0f);

                // 3. 直射散射光
                Vec3 scatterColor = sunColor * (sunIntensity * lightT * phase);

                // 4. 天空环境光（各方向来的散射天光）
                Vec3 skyAmbient(0.28f, 0.40f, 0.65f);
                Vec3 groundAmbient(0.18f, 0.16f, 0.10f);
                // 简单近似：用密度场估算向上/向下的不透明度
                float heightFactor = clampf((pos.y + 2.0f) / 5.0f, 0.0f, 1.0f);
                Vec3 ambient = Vec3::lerp(groundAmbient, skyAmbient, heightFactor) * 0.55f;
                scatterColor += ambient;

                // ---- 透射率积分 ----
                // Beer-Lambert：这一步的透射率
                float sampleT = std::exp(-d * STEP_SIZE * ABSORPTION);

                // 贡献量 = 散射颜色 × 当前透射率 × (1 - 本步透射率)
                // 物理含义：这段云贡献的光 = 它散射的光 × 它之前有多透明
                float contribution = (1.0f - sampleT) * transmittance;
                accColor      += scatterColor * contribution;
                transmittance *= sampleT;

                // 早期退出：当前透射率 < 1%，后面全是黑云，没意义继续
                if (transmittance < 0.005f) break;
            }

            // 自适应步长：密度高的地方步长减半，增加精度
            t += STEP_SIZE * (d > 0.05f ? 0.6f : 1.0f);
        }

        // 背景天空（根据剩余透射率混合）
        Vec3 bg = skyColor(dir);
        return bg * transmittance + accColor;
    }

    // ACES 色调映射
    static Vec3 aces(const Vec3& c) {
        float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
        auto f = [=](float x) {
            return clampf((x*(a*x+b)) / (x*(cc*x+d)+e), 0.0f, 1.0f);
        };
        return Vec3(f(c.x), f(c.y), f(c.z));
    }

    void render(std::vector<uint8_t>& pixels) const {
        pixels.resize(WIDTH * HEIGHT * 3);

        // 相机：从远处低角度看云层（增强宏大感）
        Vec3 camOrigin(0.0f, -1.5f, 20.0f);

        int total = WIDTH * HEIGHT;
        for (int py = 0; py < HEIGHT; py++) {
            if (py % 50 == 0) {
                std::cout << "  进度: " << (py*100/HEIGHT) << "%" << std::endl;
            }
            for (int px = 0; px < WIDTH; px++) {
                Vec3 dir   = getRayDir(px, py);
                Vec3 color = raymarch(camOrigin, dir);

                // 曝光调整
                color = color * 1.1f;
                // ACES 色调映射
                color = aces(color);
                // sRGB Gamma 校正
                color.x = std::pow(clampf(color.x,0,1), 1.0f/2.2f);
                color.y = std::pow(clampf(color.y,0,1), 1.0f/2.2f);
                color.z = std::pow(clampf(color.z,0,1), 1.0f/2.2f);

                int idx = py * WIDTH + px;
                pixels[idx*3+0] = (uint8_t)(color.x * 255.999f);
                pixels[idx*3+1] = (uint8_t)(color.y * 255.999f);
                pixels[idx*3+2] = (uint8_t)(color.z * 255.999f);
            }
        }
        std::cout << "  进度: 100%" << std::endl;
    }

private:
    CloudVolume volume;
    Vec3 sunDir, sunColor;
    float sunIntensity;

    static bool intersectAABB(const Vec3& o, const Vec3& d,
                               const Vec3& bMin, const Vec3& bMax,
                               float& tmin, float& tmax) {
        tmin = -1e30f; tmax = 1e30f;
        auto inv = [](float v) {
            return std::abs(v) > 1e-8f ? 1.0f/v : (v>=0?1e30f:-1e30f);
        };
        for (int i = 0; i < 3; i++) {
            float di = (&d.x)[i];
            float oi = (&o.x)[i];
            float lo = (&bMin.x)[i];
            float hi = (&bMax.x)[i];
            float t1 = (lo - oi) * inv(di);
            float t2 = (hi - oi) * inv(di);
            tmin = std::max(tmin, std::min(t1, t2));
            tmax = std::min(tmax, std::max(t1, t2));
        }
        return tmax > tmin && tmax > 0;
    }
};

// ============================================================
// BMP 输出（无外部依赖）
// ============================================================
void writeBMP(const std::string& fn, const std::vector<uint8_t>& px, int w, int h) {
    int rowStride = w * 3;
    int pad = (4 - rowStride%4) % 4;
    int dataSize = (rowStride + pad) * h;
    uint32_t fileSize = 54 + dataSize;

    uint8_t hdr[54] = {};
    hdr[0]='B'; hdr[1]='M';
    for (int i=0;i<4;i++) hdr[2+i]=(fileSize>>(i*8))&0xFF;
    hdr[10]=54; hdr[14]=40;
    for (int i=0;i<4;i++) hdr[18+i]=(w>>(i*8))&0xFF;
    for (int i=0;i<4;i++) hdr[22+i]=(h>>(i*8))&0xFF;
    hdr[26]=1; hdr[28]=24;

    std::ofstream ofs(fn, std::ios::binary);
    ofs.write((char*)hdr, 54);
    uint8_t zeros[3]={};
    for (int y = h-1; y >= 0; y--) {
        for (int x = 0; x < w; x++) {
            int idx = (y*w+x)*3;
            uint8_t bgr[3] = {px[idx+2], px[idx+1], px[idx]};
            ofs.write((char*)bgr, 3);
        }
        if (pad) ofs.write((char*)zeros, pad);
    }
    std::cout << "✅ " << fn << std::endl;
}

void writePPM(const std::string& fn, const std::vector<uint8_t>& px, int w, int h) {
    std::ofstream ofs(fn, std::ios::binary);
    ofs << "P6\n" << w << " " << h << "\n255\n";
    ofs.write((char*)px.data(), px.size());
    std::cout << "✅ " << fn << std::endl;
}

// ============================================================
// 主函数
// ============================================================
int main() {
    std::cout << "🌤️  体积云渲染 v2 - Ray Marching" << std::endl;
    std::cout << "分辨率: " << VolumeRenderer::WIDTH << "x" << VolumeRenderer::HEIGHT << std::endl;

    VolumeRenderer renderer;
    std::vector<uint8_t> pixels;
    renderer.render(pixels);

    writeBMP("volume_v2.bmp", pixels, VolumeRenderer::WIDTH, VolumeRenderer::HEIGHT);
    writePPM("volume_v2.ppm", pixels, VolumeRenderer::WIDTH, VolumeRenderer::HEIGHT);

    // 简单验证：平均亮度
    long sum = 0;
    for (auto v : pixels) sum += v;
    float avg = (float)sum / pixels.size();
    std::cout << "平均亮度: " << avg << " / 255" << std::endl;

    // 采样验证：天空区域（顶部）和云层区域（中部）应有明显差异
    int topPx   = (20 * VolumeRenderer::WIDTH + VolumeRenderer::WIDTH/2) * 3;
    int midPx   = (VolumeRenderer::HEIGHT/2 * VolumeRenderer::WIDTH + VolumeRenderer::WIDTH/2) * 3;
    std::cout << "天空像素 RGB(" << (int)pixels[topPx] << "," << (int)pixels[topPx+1] << "," << (int)pixels[topPx+2] << ")" << std::endl;
    std::cout << "云层像素 RGB(" << (int)pixels[midPx] << "," << (int)pixels[midPx+1] << "," << (int)pixels[midPx+2] << ")" << std::endl;

    return 0;
}
