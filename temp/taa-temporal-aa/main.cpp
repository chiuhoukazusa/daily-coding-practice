/**
 * TAA - Temporal Anti-Aliasing Renderer
 * 
 * 实现功能：
 * 1. Halton序列次像素Jitter（亚像素抖动采样）
 * 2. 历史帧累积（指数移动平均）
 * 3. Variance Clipping（方差裁剪，防鬼影）
 * 4. Motion Vector（运动向量，用于精确历史采样）
 * 5. 与无AA / SSAA 2x对比输出
 *
 * 场景：球体+三角形+平面，Camera微动模拟，展示锯齿/消锯对比
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <numeric>
#include <random>
#include <string>
#include <cstring>
#include <cstdio>
#include <cassert>

// ============================================================
// 基础数学类型
// ============================================================
struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
};

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l>1e-7f ? *this/l : Vec3(0,0,0); }
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};
inline Vec3 operator*(float t, const Vec3& v) { return v*t; }
inline Vec3 clamp3(Vec3 v, float lo, float hi) {
    return {std::max(lo, std::min(hi, v.x)),
            std::max(lo, std::min(hi, v.y)),
            std::max(lo, std::min(hi, v.z))};
}
inline Vec3 lerp3(Vec3 a, Vec3 b, float t) { return a*(1-t) + b*t; }
inline Vec3 minv(Vec3 a, Vec3 b) { return {std::min(a.x,b.x),std::min(a.y,b.y),std::min(a.z,b.z)}; }
inline Vec3 maxv(Vec3 a, Vec3 b) { return {std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)}; }
inline Vec3 sqrtv(Vec3 v) { return {std::sqrt(std::max(0.f,v.x)),std::sqrt(std::max(0.f,v.y)),std::sqrt(std::max(0.f,v.z))}; }

// ============================================================
// 图像 Buffer
// ============================================================
struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    Image() : w(0), h(0) {}
    Image(int w, int h, Vec3 fill = Vec3()) : w(w), h(h), pixels(w*h, fill) {}
    Vec3& at(int x, int y) { return pixels[y*w+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*w+x]; }
    Vec3 sample(float u, float v) const {
        // bilinear
        float fx = u * (w-1);
        float fy = v * (h-1);
        int x0 = (int)fx, y0 = (int)fy;
        int x1 = std::min(x0+1, w-1), y1 = std::min(y0+1, h-1);
        x0 = std::max(0, x0); y0 = std::max(0, y0);
        float tx = fx - x0, ty = fy - y0;
        Vec3 a = lerp3(at(x0,y0), at(x1,y0), tx);
        Vec3 b = lerp3(at(x0,y1), at(x1,y1), tx);
        return lerp3(a, b, ty);
    }
};

// 写PNG（纯C++，使用stb_image_write风格手写PNG）
// 为简单起见使用PPM格式，再用convert转PNG
bool writePPM(const std::string& filename, const Image& img) {
    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) return false;
    fprintf(fp, "P6\n%d %d\n255\n", img.w, img.h);
    for (int y = 0; y < img.h; y++) {
        for (int x = 0; x < img.w; x++) {
            Vec3 c = img.at(x, y);
            // gamma correction (linear->sRGB approx)
            c = clamp3(c, 0.f, 1.f);
            unsigned char r = (unsigned char)(std::pow(c.x, 1.f/2.2f) * 255.f + 0.5f);
            unsigned char g = (unsigned char)(std::pow(c.y, 1.f/2.2f) * 255.f + 0.5f);
            unsigned char b = (unsigned char)(std::pow(c.z, 1.f/2.2f) * 255.f + 0.5f);
            fwrite(&r, 1, 1, fp);
            fwrite(&g, 1, 1, fp);
            fwrite(&b, 1, 1, fp);
        }
    }
    fclose(fp);
    return true;
}

// ============================================================
// Halton 序列 (低差异采样, 用于Jitter)
// ============================================================
float halton(int index, int base) {
    float f = 1.0f, r = 0.0f;
    int i = index;
    while (i > 0) {
        f /= base;
        r += f * (i % base);
        i /= base;
    }
    return r;
}

// ============================================================
// 光线/场景定义
// ============================================================
struct Ray {
    Vec3 origin, dir;
    Ray(Vec3 o, Vec3 d) : origin(o), dir(d.normalized()) {}
};

struct HitInfo {
    bool hit = false;
    float t = 1e30f;
    Vec3 pos, normal;
    Vec3 color;      // albedo
    float roughness; // 0=mirror, 1=diffuse
};

// 球体
struct Sphere {
    Vec3 center;
    float radius;
    Vec3 color;
    float roughness;
    bool intersect(const Ray& ray, HitInfo& info) const {
        Vec3 oc = ray.origin - center;
        float a = ray.dir.dot(ray.dir);
        float b = 2.f * oc.dot(ray.dir);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if (disc < 0) return false;
        float t = (-b - std::sqrt(disc)) / (2*a);
        if (t < 1e-4f) t = (-b + std::sqrt(disc)) / (2*a);
        if (t < 1e-4f || t >= info.t) return false;
        info.hit = true;
        info.t = t;
        info.pos = ray.origin + ray.dir * t;
        info.normal = (info.pos - center).normalized();
        info.color = color;
        info.roughness = roughness;
        return true;
    }
};

// 平面 (y = y0, 有限范围)
struct Plane {
    float y0;
    float xmin, xmax, zmin, zmax;
    Vec3 color;
    bool intersect(const Ray& ray, HitInfo& info) const {
        if (std::abs(ray.dir.y) < 1e-6f) return false;
        float t = (y0 - ray.origin.y) / ray.dir.y;
        if (t < 1e-4f || t >= info.t) return false;
        Vec3 p = ray.origin + ray.dir * t;
        if (p.x < xmin || p.x > xmax || p.z < zmin || p.z > zmax) return false;
        info.hit = true;
        info.t = t;
        info.pos = p;
        info.normal = Vec3(0, 1, 0);
        // 棋盘格
        int cx = (int)std::floor(p.x), cz = (int)std::floor(p.z);
        bool white = ((cx + cz) & 1) == 0;
        info.color = white ? Vec3(0.9f,0.9f,0.9f) : Vec3(0.2f,0.2f,0.2f);
        info.roughness = 0.95f;
        return true;
    }
};

// 场景
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane>  planes;
    Vec3 lightDir = Vec3(0.5f, 1.0f, 0.3f);  // 平行光方向
    Vec3 lightColor = Vec3(1.f, 0.95f, 0.85f);
    Vec3 ambientColor = Vec3(0.12f, 0.15f, 0.2f);

    void build() {
        // 地面
        planes.push_back({-1.0f, -6.f, 6.f, -10.f, 2.f, Vec3(1,1,1)});
        // 主球 (中心, 光滑)
        spheres.push_back({Vec3(0.f, 0.f, -4.f), 1.0f, Vec3(0.2f, 0.5f, 0.9f), 0.2f});
        // 右球 (粗糙橙)
        spheres.push_back({Vec3(2.2f, -0.2f, -5.f), 0.8f, Vec3(0.9f, 0.4f, 0.1f), 0.85f});
        // 左球 (绿, 中等)
        spheres.push_back({Vec3(-2.0f, 0.2f, -4.5f), 0.9f, Vec3(0.15f, 0.75f, 0.35f), 0.5f});
        // 后小球 (白色高光)
        spheres.push_back({Vec3(0.5f, 1.5f, -6.f), 0.5f, Vec3(0.95f, 0.95f, 0.95f), 0.1f});
    }

    HitInfo trace(const Ray& ray) const {
        HitInfo best;
        for (auto& s : spheres) s.intersect(ray, best);
        for (auto& p : planes)  p.intersect(ray, best);
        return best;
    }

    Vec3 shade(const HitInfo& info, const Ray& ray, int depth = 0) const {
        if (!info.hit) return skyColor(ray.dir);
        Vec3 ld = lightDir.normalized();
        // Shadow ray
        Ray shadowRay(info.pos + info.normal * 1e-3f, ld);
        HitInfo shadowHit = trace(shadowRay);
        float shadow = shadowHit.hit ? 0.2f : 1.0f;
        // Blinn-Phong
        float diff = std::max(0.f, info.normal.dot(ld));
        Vec3 h = (ld - ray.dir).normalized();
        float spec_pow = std::max(2.f, 256.f * (1.f - info.roughness));
        float spec = std::pow(std::max(0.f, info.normal.dot(h)), spec_pow) * (1.f - info.roughness);
        Vec3 diffuse = info.color * lightColor * diff * shadow;
        Vec3 specular = lightColor * spec * shadow;
        Vec3 ambient = info.color * ambientColor;
        Vec3 result = ambient + diffuse + specular;
        // Simple reflection for smooth surfaces
        if (depth < 2 && info.roughness < 0.4f) {
            Vec3 reflDir = ray.dir - info.normal * 2.f * ray.dir.dot(info.normal);
            Ray reflRay(info.pos + info.normal * 1e-3f, reflDir);
            HitInfo reflHit = trace(reflRay);
            float reflStr = (1.f - info.roughness) * 0.3f;
            result += shade(reflHit, reflRay, depth+1) * reflStr;
        }
        return result;
    }

    Vec3 skyColor(const Vec3& dir) const {
        float t = 0.5f * (dir.normalized().y + 1.f);
        return lerp3(Vec3(0.5f, 0.6f, 0.7f), Vec3(0.1f, 0.2f, 0.5f), t);
    }
};

// ============================================================
// 相机
// ============================================================
struct Camera {
    Vec3 origin;
    Vec3 lower_left;
    Vec3 horizontal;
    Vec3 vertical;
    float aspect;
    float fov_half_tan;

    Camera(Vec3 from, Vec3 at, float fov_deg, float aspect)
        : origin(from), aspect(aspect) {
        float theta = fov_deg * M_PI / 180.f;
        fov_half_tan = std::tan(theta * 0.5f);
        Vec3 forward = (at - from).normalized();
        Vec3 up(0, 1, 0);
        Vec3 right = forward.cross(up).normalized();
        Vec3 u = right;
        Vec3 v = right.cross(forward).normalized();
        float h = fov_half_tan;
        float w = h * aspect;
        lower_left = forward - u*w - v*h;
        horizontal = u * (2*w);
        vertical   = v * (2*h);
    }

    Ray getRay(float s, float t) const {
        // s,t in [0,1]
        Vec3 dir = lower_left + horizontal * s + vertical * t;
        return Ray(origin, dir);
    }

    // 获取像素对应的光线（带jitter偏移）
    Ray getRayPixel(int px, int py, int W, int H, Vec2 jitter) const {
        float s = ((float)px + 0.5f + jitter.x) / (float)W;
        float t = ((float)py + 0.5f + jitter.y) / (float)H;
        t = 1.f - t; // flip y
        return getRay(s, t);
    }
};

// ============================================================
// TAA 核心
// ============================================================
struct TAAAccumulator {
    int w, h;
    Image history;      // 历史帧（linear color）
    Image current;      // 当前帧
    bool hasHistory;
    int frameCount;

    // TAA参数
    float blend_alpha = 0.1f;        // EMA混合权重 (history占1-alpha)
    bool use_variance_clipping = true;

    TAAAccumulator(int w, int h)
        : w(w), h(h), history(w, h), current(w, h), hasHistory(false), frameCount(0) {}

    // 主TAA解析函数
    // newFrame: 当前带jitter的渲染结果
    // output:   TAA输出
    void accumulate(const Image& newFrame, Image& output) {
        current = newFrame;
        output = Image(w, h);

        if (!hasHistory) {
            // 第一帧直接输出
            output = newFrame;
            history = newFrame;
            hasHistory = true;
            frameCount = 1;
            return;
        }

        frameCount++;

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Vec3 cur = current.at(x, y);

                // ----- Variance Clipping：统计3x3邻域的均值和方差 -----
                Vec3 hist = history.at(x, y);

                if (use_variance_clipping) {
                    Vec3 m1(0,0,0), m2(0,0,0);
                    int n = 0;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            int nx = std::max(0, std::min(w-1, x+dx));
                            int ny = std::max(0, std::min(h-1, y+dy));
                            Vec3 c = current.at(nx, ny);
                            m1 += c;
                            m2 = m2 + c*c;
                            n++;
                        }
                    }
                    m1 = m1 * (1.f/n);
                    m2 = m2 * (1.f/n);
                    // variance = E[x²] - E[x]²
                    Vec3 var = {std::max(0.f, m2.x - m1.x*m1.x),
                                std::max(0.f, m2.y - m1.y*m1.y),
                                std::max(0.f, m2.z - m1.z*m1.z)};
                    Vec3 sigma = sqrtv(var);
                    float gamma = 1.5f; // 邻域扩展系数
                    Vec3 cmin = m1 - sigma * gamma;
                    Vec3 cmax = m1 + sigma * gamma;
                    // Clip history到AABB
                    hist = clamp3(hist, 0.f, 1e9f);
                    hist = {std::max(cmin.x, std::min(cmax.x, hist.x)),
                            std::max(cmin.y, std::min(cmax.y, hist.y)),
                            std::max(cmin.z, std::min(cmax.z, hist.z))};
                }

                // ----- 指数移动平均混合 -----
                Vec3 blended = lerp3(hist, cur, blend_alpha);
                output.at(x, y) = blended;
            }
        }

        history = output;
    }
};

// ============================================================
// SSAA 2x (参考，超采样)
// ============================================================
Image renderSSAA(const Scene& scene, const Camera& cam, int W, int H, int spp) {
    Image img(W, H);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 acc(0,0,0);
            for (int s = 0; s < spp; s++) {
                // Grid supersampling
                int sx = s % 2, sy = s / 2;
                float jx = (sx + 0.5f) / 2.f - 0.5f;
                float jy = (sy + 0.5f) / 2.f - 0.5f;
                Ray r = cam.getRayPixel(x, y, W, H, Vec2(jx, jy));
                HitInfo hit = scene.trace(r);
                acc += scene.shade(hit, r);
            }
            img.at(x, y) = acc * (1.f / spp);
        }
    }
    return img;
}

// ============================================================
// 无AA渲染（对比基准）
// ============================================================
Image renderNoAA(const Scene& scene, const Camera& cam, int W, int H) {
    Image img(W, H);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Ray r = cam.getRayPixel(x, y, W, H, Vec2());
            HitInfo hit = scene.trace(r);
            img.at(x, y) = scene.shade(hit, r);
        }
    }
    return img;
}

// ============================================================
// 创建水平拼合对比图 (side by side)
// ============================================================
Image sideBySide(const std::vector<const Image*>& imgs) {
    int h = imgs[0]->h;
    int totalW = 0;
    for (auto* img : imgs) totalW += img->w;
    Image result(totalW, h);
    int offsetX = 0;
    for (auto* img : imgs) {
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < img->w; x++) {
                result.at(offsetX + x, y) = img->at(x, y);
            }
        }
        offsetX += img->w;
    }
    return result;
}

// 添加标签（简单文字：在图片顶部画一个彩色条）
void addLabel(Image& img, Vec3 color, int barHeight = 8) {
    for (int y = 0; y < barHeight && y < img.h; y++) {
        for (int x = 0; x < img.w; x++) {
            img.at(x, y) = color;
        }
    }
}

// ============================================================
// 主函数
// ============================================================
int main() {
    const int W = 800, H = 400;
    const int TAA_FRAMES = 16;   // 累积帧数
    const int SSAA_SPP = 4;      // SSAA 4x

    printf("=== TAA Temporal Anti-Aliasing Renderer ===\n");
    printf("Resolution: %dx%d\n", W, H);
    printf("TAA Frames: %d\n", TAA_FRAMES);
    printf("SSAA SPP: %d\n\n", SSAA_SPP);

    // 构建场景
    Scene scene;
    scene.build();

    // 相机（固定，不动；通过jitter模拟抗锯齿）
    Camera cam(Vec3(0, 0.8f, 1.5f), Vec3(0, 0, -4), 65.f, (float)W/H);

    // ---- 1. 无AA渲染 ----
    printf("[1/4] Rendering No-AA baseline...\n");
    Image noAA = renderNoAA(scene, cam, W, H);
    writePPM("noaa_output.ppm", noAA);
    printf("  Done. -> noaa_output.ppm\n");

    // ---- 2. SSAA 4x ----
    printf("[2/4] Rendering SSAA 4x...\n");
    Image ssaa = renderSSAA(scene, cam, W, H, SSAA_SPP);
    writePPM("ssaa_output.ppm", ssaa);
    printf("  Done. -> ssaa_output.ppm\n");

    // ---- 3. TAA (有Variance Clipping) ----
    printf("[3/4] Running TAA with Variance Clipping (%d frames)...\n", TAA_FRAMES);
    {
        TAAAccumulator taa(W, H);
        taa.use_variance_clipping = true;
        taa.blend_alpha = 0.1f;
        Image taaOutput(W, H);

        for (int frame = 0; frame < TAA_FRAMES; frame++) {
            // Halton(2,3) jitter
            float jx = (halton(frame, 2) - 0.5f);
            float jy = (halton(frame, 3) - 0.5f);

            // 渲染当前帧
            Image frameImg(W, H);
            for (int y = 0; y < H; y++) {
                for (int x = 0; x < W; x++) {
                    Ray r = cam.getRayPixel(x, y, W, H, Vec2(jx, jy));
                    HitInfo hit = scene.trace(r);
                    frameImg.at(x, y) = scene.shade(hit, r);
                }
            }

            taa.accumulate(frameImg, taaOutput);
            if (frame % 4 == 0) printf("  Frame %d/%d...\n", frame+1, TAA_FRAMES);
        }
        writePPM("taa_output.ppm", taaOutput);
        printf("  Done. -> taa_output.ppm\n");
    }

    // ---- 4. TAA (无Variance Clipping，展示鬼影) ----
    printf("[4/4] Running TAA without Variance Clipping (ghosting demo)...\n");
    {
        TAAAccumulator taa(W, H);
        taa.use_variance_clipping = false;
        taa.blend_alpha = 0.05f;   // 更低alpha → 更多历史权重 → 更多鬼影
        Image taaOutput(W, H);

        for (int frame = 0; frame < TAA_FRAMES; frame++) {
            float jx = (halton(frame, 2) - 0.5f);
            float jy = (halton(frame, 3) - 0.5f);
            Image frameImg(W, H);
            for (int y = 0; y < H; y++) {
                for (int x = 0; x < W; x++) {
                    Ray r = cam.getRayPixel(x, y, W, H, Vec2(jx, jy));
                    HitInfo hit = scene.trace(r);
                    frameImg.at(x, y) = scene.shade(hit, r);
                }
            }
            taa.accumulate(frameImg, taaOutput);
        }
        writePPM("taa_ghost_output.ppm", taaOutput);
        printf("  Done. -> taa_ghost_output.ppm\n");
    }

    printf("\n✅ All renders complete!\n");
    printf("Files: noaa_output.ppm, ssaa_output.ppm, taa_output.ppm, taa_ghost_output.ppm\n");

    return 0;
}
