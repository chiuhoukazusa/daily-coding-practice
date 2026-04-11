/**
 * Volumetric Light Scattering (God Rays) Renderer
 * 体积光散射 / 丁达尔效应渲染器
 *
 * 正确的 God Rays 场景布局：
 * - 太阳在相机后方上面（光朝向相机打）
 * - 相机往前看，看向被光照亮的场景
 * - 遮挡物（球体）遮住部分光线
 * - 从相机往前看，光束从遮挡物边缘散射到视野中
 *
 * 或者改用"屏幕空间 God Rays"方法：
 * - 对每个像素做射线步进，沿光源方向叠加可见性
 * - 这是 Crepuscular Rays 的标准实现
 *
 * 技术要点:
 * - Mie + Rayleigh 散射相位函数（前向散射）
 * - Beer-Lambert 透射衰减
 * - 遮挡采样（阴影测试）
 * - 场景：多球体 + 棋盘格地面 + 强光源
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <iostream>
#include <limits>

// ─── Math ─────────────────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len2() const { return x*x + y*y + z*z; }
    float len() const { return sqrtf(len2()); }
    Vec3 norm() const { float l = len(); return l > 1e-8f ? (*this)/l : Vec3(0,1,0); }
    Vec3 clamp(float lo, float hi) const {
        return { std::min(hi, std::max(lo, x)),
                 std::min(hi, std::max(lo, y)),
                 std::min(hi, std::max(lo, z)) };
    }
};
inline Vec3 operator*(float t, const Vec3& v) { return v*t; }

// ─── ACES 色调映射 ────────────────────────────────────────────
Vec3 aces(Vec3 c) {
    float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
    return Vec3(
        (c.x*(a*c.x+b))/(c.x*(cc*c.x+d)+e),
        (c.y*(a*c.y+b))/(c.y*(cc*c.y+d)+e),
        (c.z*(a*c.z+b))/(c.z*(cc*c.z+d)+e)
    ).clamp(0,1);
}

// ─── 场景 ─────────────────────────────────────────────────────
// 关键：太阳在场景正前方（z正方向），相机也往 z+ 方向看
// 光源在相机前方！这样视线方向 ≈ 光源方向 → 前向散射最强

static const Vec3 SUN_POS   = {1.5f, 6.0f, 18.0f}; // 场景正前方
static const Vec3 SUN_COLOR = {5.0f, 4.0f, 2.2f};  // 极强暖黄光
static const Vec3 AMBIENT   = {0.01f, 0.015f, 0.04f};
static const float GROUND_Y = -1.8f;

struct Sphere {
    Vec3  center;
    float radius;
    Vec3  albedo;
};

static const Sphere SPHERES[] = {
    // 主遮挡球：靠近光源，遮住部分光线，在球边缘产生光晕
    {{ 0.5f,  1.0f,  8.0f}, 2.2f, {0.2f, 0.2f, 0.25f}},
    // 小球群
    {{-3.5f, -0.2f,  5.0f}, 0.9f, {0.75f, 0.2f, 0.1f}},
    {{ 3.2f, -0.4f,  6.0f}, 0.8f, {0.1f, 0.4f, 0.75f}},
    {{-1.5f, -1.0f,  3.5f}, 0.5f, {0.6f, 0.55f, 0.1f}},
    {{ 1.8f, -0.8f,  3.0f}, 0.6f, {0.35f, 0.6f, 0.2f}},
};
static const int NUM_SPHERES = 5;

// 光线与球相交
bool sphereHit(const Vec3& ro, const Vec3& rd, const Sphere& s, float& t) {
    Vec3 oc = ro - s.center;
    float a = rd.dot(rd);
    float b = 2.f * oc.dot(rd);
    float c = oc.dot(oc) - s.radius * s.radius;
    float d = b*b - 4*a*c;
    if (d < 0) return false;
    float sq = sqrtf(d);
    float t0 = (-b - sq) / (2*a);
    float t1 = (-b + sq) / (2*a);
    if (t1 < 1e-4f) return false;
    t = (t0 > 1e-4f) ? t0 : t1;
    return true;
}

// 光线与地面相交
bool planeHit(const Vec3& ro, const Vec3& rd, float& t) {
    if (fabsf(rd.y) < 1e-6f) return false;
    t = (GROUND_Y - ro.y) / rd.y;
    return t > 1e-4f;
}

// 阴影：pos 是否被球体遮挡（相对于太阳）
bool isShadowed(const Vec3& pos) {
    Vec3 toSun = SUN_POS - pos;
    float dist = toSun.len();
    Vec3 dir   = toSun / dist;
    for (int i = 0; i < NUM_SPHERES; ++i) {
        float t;
        if (sphereHit(pos + dir * 0.02f, dir, SPHERES[i], t) && t < dist)
            return true;
    }
    return false;
}

// ─── Mie 相位函数（Henyey-Greenstein） ────────────────────────
float miePhase(float cosT, float g = 0.76f) {
    float g2 = g*g;
    float d  = 1.f + g2 - 2.f*g*cosT;
    return (1.f - g2) / (4.f * 3.14159265f * d * sqrtf(d));
}

// ─── 着色 ─────────────────────────────────────────────────────
Vec3 shade(const Vec3& pos, const Vec3& N, const Vec3& albedo) {
    Vec3 toSun = (SUN_POS - pos).norm();
    float diff = std::max(0.f, N.dot(toSun));
    Vec3 H     = (toSun + Vec3(0,0,-1)).norm();
    float spec = powf(std::max(0.f, N.dot(H)), 64.f);
    bool  shad = isShadowed(pos + N * 0.02f);
    float vis  = shad ? 0.f : 1.f;
    float d    = (SUN_POS - pos).len();
    float att  = 1.f / (1.f + 0.02f * d);
    return AMBIENT * albedo + SUN_COLOR * albedo * (diff * att * vis)
         + SUN_COLOR * Vec3(1,1,1) * (spec * 0.3f * att * vis);
}

// ─── 场景追踪 ─────────────────────────────────────────────────
struct RayRes { Vec3 color; float tHit; };

RayRes trace(const Vec3& ro, const Vec3& rd) {
    float tMin = std::numeric_limits<float>::max();
    Vec3  N, alb;
    bool  hit = false;

    for (int i = 0; i < NUM_SPHERES; ++i) {
        float t;
        if (sphereHit(ro, rd, SPHERES[i], t) && t < tMin) {
            tMin = t; N = (ro + rd*t - SPHERES[i].center).norm();
            alb = SPHERES[i].albedo; hit = true;
        }
    }
    {
        float t;
        if (planeHit(ro, rd, t) && t < tMin) {
            tMin = t; N = {0,1,0};
            Vec3 p = ro + rd*t;
            int ix = (int)floorf(p.x*0.65f);
            int iz = (int)floorf(p.z*0.65f);
            alb = ((ix+iz)&1) ? Vec3{0.12f,0.1f,0.08f} : Vec3{0.7f,0.65f,0.55f};
            hit = true;
        }
    }

    if (!hit) {
        // 深夜天空 + 太阳圆盘
        float yy = rd.y * 0.5f + 0.5f;
        Vec3 sky = Vec3{0.005f,0.008f,0.02f} * (1-yy) + Vec3{0.01f,0.015f,0.04f} * yy;
        Vec3 toSun = (SUN_POS - ro).norm();
        float s = rd.dot(toSun);
        if (s > 0.998f) sky += SUN_COLOR * powf((s-0.998f)/0.002f, 2.f);
        else if (s > 0.96f) sky += SUN_COLOR * powf((s-0.96f)/0.038f, 4.f) * 0.15f;
        return {sky, -1.f};
    }

    Vec3 hp = ro + rd * tMin;
    return {shade(hp, N, alb), tMin};
}

// ─── 体积光散射 ───────────────────────────────────────────────
Vec3 godRays(const Vec3& ro, const Vec3& rd, float tScene) {
    const int STEPS  = 80;
    float marchDist  = (tScene > 0) ? std::min(tScene, 24.f) : 24.f;
    float step       = marchDist / float(STEPS);

    // 视线方向与光源方向夹角
    Vec3  toSun  = (SUN_POS - ro).norm();
    float cosT   = rd.dot(toSun);
    float phase  = miePhase(cosT, 0.76f);

    // 散射参数
    const float sigma_s = 0.14f;
    const float sigma_t = 0.16f;

    Vec3  acc    = {0,0,0};
    float transm = 1.f;

    for (int i = 0; i < STEPS; ++i) {
        float t   = (i + 0.5f) * step;
        if (t >= marchDist) break;
        Vec3 p    = ro + rd * t;
        if (p.y < GROUND_Y) break;

        if (!isShadowed(p)) {
            float d      = (SUN_POS - p).len();
            // 减小距离衰减，让更远处也有贡献
            float atten  = 120.f / (d * d + 8.f);
            acc += SUN_COLOR * (atten * phase * sigma_s * transm * step);
        }

        transm *= expf(-sigma_t * step);
        if (transm < 0.002f) break;
    }
    return acc;
}

// ─── 渲染 ────────────────────────────────────────────────────
int main() {
    const int W = 800, H = 600;

    Vec3 camPos = {0.f, 1.0f, -2.5f};
    float fov   = 58.f * 3.14159265f / 180.f;
    float asp   = float(W) / float(H);
    float hH    = tanf(fov * 0.5f);
    float hW    = asp * hH;

    // 相机往 z+ 方向看（朝向光源所在方向）
    Vec3 fwd  = Vec3(0.05f, -0.05f, 1.f).norm();
    Vec3 rgt  = fwd.cross(Vec3(0,1,0)).norm();
    Vec3 up   = rgt.cross(fwd).norm();

    std::cout << "渲染 " << W << "x" << H << " God Rays..." << std::endl;
    Vec3 toSunDir = (SUN_POS - camPos).norm();
    float camSunDot = fwd.dot(toSunDir);
    std::cout << "相机朝向与光源方向夹角余弦: " << camSunDot
              << "  (>0.8表示相机基本面向光源，前向散射强)" << std::endl;

    std::vector<uint8_t> img(W * H * 3);

    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            float u  = (2.f*(px+0.5f)/W - 1.f) * hW;
            float v  = (1.f - 2.f*(py+0.5f)/H) * hH;
            Vec3 rd  = (fwd + rgt*u + up*v).norm();

            RayRes res = trace(camPos, rd);
            Vec3   gr  = godRays(camPos, rd, res.tHit);

            Vec3 final = aces(res.color + gr * 6.f);
            final.x = powf(std::max(0.f, final.x), 1.f/2.2f);
            final.y = powf(std::max(0.f, final.y), 1.f/2.2f);
            final.z = powf(std::max(0.f, final.z), 1.f/2.2f);

            int idx = (py*W + px) * 3;
            img[idx+0] = uint8_t(std::min(255.f, final.x*255));
            img[idx+1] = uint8_t(std::min(255.f, final.y*255));
            img[idx+2] = uint8_t(std::min(255.f, final.z*255));
        }
        if (py % 100 == 0)
            std::cout << "  " << py << "/" << H << " (" << int(100.f*py/H) << "%)\n";
    }

    stbi_write_png("god_rays_output.png", W, H, 3, img.data(), W*3);
    std::cout << "✅ 保存: god_rays_output.png\n";

    // 对比图
    std::vector<uint8_t> cmp(W*2 * H * 3);
    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            float u = (2.f*(px+0.5f)/W-1.f)*hW;
            float v = (1.f-2.f*(py+0.5f)/H)*hH;
            Vec3 rd = (fwd+rgt*u+up*v).norm();
            RayRes res = trace(camPos, rd);

            // 左：无 god rays
            {
                Vec3 c = aces(res.color);
                c.x=powf(std::max(0.f,c.x),1.f/2.2f);
                c.y=powf(std::max(0.f,c.y),1.f/2.2f);
                c.z=powf(std::max(0.f,c.z),1.f/2.2f);
                int i=(py*W*2+px)*3;
                cmp[i+0]=uint8_t(std::min(255.f,c.x*255));
                cmp[i+1]=uint8_t(std::min(255.f,c.y*255));
                cmp[i+2]=uint8_t(std::min(255.f,c.z*255));
            }
            // 右：有 god rays
            {
                Vec3 gr = godRays(camPos, rd, res.tHit);
                Vec3 c  = aces(res.color + gr*6.f);
                c.x=powf(std::max(0.f,c.x),1.f/2.2f);
                c.y=powf(std::max(0.f,c.y),1.f/2.2f);
                c.z=powf(std::max(0.f,c.z),1.f/2.2f);
                int i=(py*W*2+px+W)*3;
                cmp[i+0]=uint8_t(std::min(255.f,c.x*255));
                cmp[i+1]=uint8_t(std::min(255.f,c.y*255));
                cmp[i+2]=uint8_t(std::min(255.f,c.z*255));
            }
        }
    }
    stbi_write_png("god_rays_comparison.png", W*2, H, 3, cmp.data(), W*2*3);
    std::cout << "✅ 保存: god_rays_comparison.png\n";

    // 仅体积光
    std::vector<uint8_t> vol(W*H*3);
    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            float u = (2.f*(px+0.5f)/W-1.f)*hW;
            float v = (1.f-2.f*(py+0.5f)/H)*hH;
            Vec3 rd = (fwd+rgt*u+up*v).norm();
            RayRes res = trace(camPos, rd);
            Vec3 gr = godRays(camPos, rd, res.tHit) * 6.f;
            Vec3 c = aces(gr);
            c.x=powf(std::max(0.f,c.x),1.f/2.2f);
            c.y=powf(std::max(0.f,c.y),1.f/2.2f);
            c.z=powf(std::max(0.f,c.z),1.f/2.2f);
            int i=(py*W+px)*3;
            vol[i+0]=uint8_t(std::min(255.f,c.x*255));
            vol[i+1]=uint8_t(std::min(255.f,c.y*255));
            vol[i+2]=uint8_t(std::min(255.f,c.z*255));
        }
    }
    stbi_write_png("god_rays_volume_only.png", W, H, 3, vol.data(), W*3);
    std::cout << "✅ 保存: god_rays_volume_only.png\n";

    std::cout << "\n🎉 完成！\n";
    return 0;
}
