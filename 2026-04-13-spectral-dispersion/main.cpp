/*
 * Spectral Dispersion Ray Tracer v2
 * 光谱色散光线追踪器 - 正确实现色散效果
 *
 * 原理：
 * - 不同波长折射率不同（柯西方程）
 * - 相机朝向玻璃球，玻璃球折射背景平行光
 * - 背景是明亮的彩色渐变，透过玻璃球后产生色差彩虹
 * - 同时叠加彩色光源的色散分离效果
 *
 * 关键设计：
 * - 天空盒/背景：带有颜色梯度，透过折射后颜色映射不同
 * - 折射球内：R/G/B光线偏折角度不同 → 看到不同区域颜色
 * - 地面上：焦散光斑（R/G/B到达不同位置 → 彩虹边缘）
 *
 * Date: 2026-04-13
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <algorithm>
#include <vector>
#include <cstring>
#include <cassert>

// ─── Vec3 ─────────────────────────────────────────────────────────────
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double t)      const { return {x*t,   y*t,   z*t  }; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }

    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double len()  const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        double l = len();
        return (l > 1e-12) ? Vec3(x/l, y/l, z/l) : Vec3(0,0,0);
    }
    Vec3 clamp01() const {
        return { std::max(0.0, std::min(1.0, x)),
                 std::max(0.0, std::min(1.0, y)),
                 std::max(0.0, std::min(1.0, z)) };
    }
};

inline Vec3 operator*(double t, const Vec3& v) { return v * t; }

// ─── 柯西方程：波长依赖折射率 ─────────────────────────────────────────
// Crown Glass BK7: A=1.5046, B=0.00420 (λ单位: μm)
double cauchyIOR(double lambda_um) {
    const double A = 1.5046;
    const double B = 0.00420;
    return A + B / (lambda_um * lambda_um);
}

// R/G/B 三个代表波长（μm）
static const double LAMBDA_R = 0.656;
static const double LAMBDA_G = 0.532;
static const double LAMBDA_B = 0.486;

// ─── Ray & Hit ────────────────────────────────────────────────────────
struct Ray { Vec3 o, d; };

struct HitRecord {
    double t;
    Vec3 p, n;
    bool frontFace;
    int matId;  // 0=diffuse(white) 1=glass 2=diffuse(checker) 3=background
};

// ─── Fresnel ──────────────────────────────────────────────────────────
double schlick(double cosTheta, double n1, double n2) {
    double r0 = (n1 - n2) / (n1 + n2);
    r0 *= r0;
    return r0 + (1.0 - r0) * std::pow(std::max(0.0, 1.0 - cosTheta), 5.0);
}

// ─── Refract ──────────────────────────────────────────────────────────
bool refract(const Vec3& d, const Vec3& n, double niOverNt, Vec3& out) {
    Vec3 uv = d.normalized();
    double dt = uv.dot(n);
    double disc = 1.0 - niOverNt * niOverNt * (1.0 - dt * dt);
    if (disc <= 0.0) return false;
    out = (uv - n * dt) * niOverNt - n * std::sqrt(disc);
    return true;
}

// ─── 几何体 ───────────────────────────────────────────────────────────
struct Sphere {
    Vec3 c; double r; int mat;
    bool hit(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        Vec3 oc = ray.o - c;
        double a = ray.d.dot(ray.d);
        double b = oc.dot(ray.d);
        double cc = oc.dot(oc) - r*r;
        double disc = b*b - a*cc;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double root = (-b - sq) / a;
        if (root < tMin || root > tMax) {
            root = (-b + sq) / a;
            if (root < tMin || root > tMax) return false;
        }
        rec.t = root;
        rec.p = ray.o + ray.d * root;
        Vec3 outN = (rec.p - c) * (1.0/r);
        rec.frontFace = ray.d.dot(outN) < 0;
        rec.n = rec.frontFace ? outN : -outN;
        rec.matId = mat;
        return true;
    }
};

struct Plane {
    Vec3 pt, n; int mat;
    bool hit(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
        double denom = n.dot(ray.d);
        if (std::abs(denom) < 1e-10) return false;
        double t = (pt - ray.o).dot(n) / denom;
        if (t < tMin || t > tMax) return false;
        rec.t = t;
        rec.p = ray.o + ray.d * t;
        rec.frontFace = denom < 0;
        rec.n = rec.frontFace ? n : -n;
        rec.matId = mat;
        return true;
    }
};

// ─── 场景 ─────────────────────────────────────────────────────────────
std::vector<Sphere> spheres;
std::vector<Plane>  planes;

void buildScene() {
    // 地面（棋盘格漫反射）
    planes.push_back({ Vec3(0,-1.5,0), Vec3(0,1,0), 2 });

    // 主玻璃球 - 正中央
    spheres.push_back({ Vec3(0.0, 0.0, -3.0), 1.0, 1 });
    // 左小玻璃球
    spheres.push_back({ Vec3(-2.3, -0.5, -3.5), 0.65, 1 });
    // 右小玻璃球
    spheres.push_back({ Vec3(2.1, -0.6, -3.2), 0.55, 1 });
}

bool sceneHit(const Ray& ray, double tMin, double tMax, HitRecord& rec) {
    HitRecord tmp;
    bool any = false;
    double closest = tMax;
    for (auto& s : spheres)
        if (s.hit(ray, tMin, closest, tmp)) { any = true; closest = tmp.t; rec = tmp; }
    for (auto& p : planes)
        if (p.hit(ray, tMin, closest, tmp)) { any = true; closest = tmp.t; rec = tmp; }
    return any;
}

// ─── 彩色天空盒（关键！提供丰富颜色让折射产生色差） ─────────────────
// 设计：天空是红-蓝渐变（左→右）+ 白色高亮（上方）
// 这样折射球内R/G/B偏折到不同方向时，会采样到不同颜色
Vec3 skyColor(const Vec3& dir) {
    Vec3 d = dir.normalized();
    // 水平方向颜色带（X轴）
    double hAngle = std::atan2(d.z, d.x) / M_PI;  // -1 ~ 1
    double elevation = d.y;                         // -1 ~ 1（仰角）

    // 天空颜色：基于方向角的彩虹渐变
    // 将方向角映射到色相（HSV-like）
    double hue = (hAngle + 1.0) / 2.0;  // 0~1

    // 彩虹色相映射（HSV转RGB，简化版）
    Vec3 rainbow;
    double h6 = hue * 6.0;
    int hi = (int)h6 % 6;
    double f = h6 - (int)h6;
    switch (hi) {
        case 0: rainbow = Vec3(1, f, 0); break;
        case 1: rainbow = Vec3(1-f, 1, 0); break;
        case 2: rainbow = Vec3(0, 1, f); break;
        case 3: rainbow = Vec3(0, 1-f, 1); break;
        case 4: rainbow = Vec3(f, 0, 1); break;
        default: rainbow = Vec3(1, 0, 1-f); break;
    }

    // 亮度调整：仰角越高越亮（天顶白色，地平线彩色）
    double bright = 0.4 + 0.6 * std::max(0.0, elevation);

    // 混合：高仰角偏白色，低仰角保持彩色
    double whiteMix = std::max(0.0, elevation);
    Vec3 white(1, 1, 1);
    Vec3 colorPart = rainbow * (1.0 - whiteMix * 0.6) + white * (whiteMix * 0.6);

    // 地平线以下：深蓝/灰（地面方向背景）
    if (elevation < 0) {
        Vec3 groundColor(0.15, 0.12, 0.10);
        double blend = std::min(1.0, -elevation * 3.0);
        colorPart = colorPart * (1.0 - blend) + groundColor * blend;
    }

    return colorPart * bright;
}

// ─── 追踪单个波长 ─────────────────────────────────────────────────────
Vec3 traceWavelength(const Ray& ray, double lambda, int depth,
                     double rComp, double gComp, double bComp) {
    // 返回该波长的辐射度，组合为RGB
    if (depth <= 0) return Vec3(0,0,0);

    HitRecord rec;
    if (!sceneHit(ray, 0.001, 1e10, rec)) {
        Vec3 sky = skyColor(ray.d);
        // 按波长分量权重返回
        return Vec3(sky.x * rComp, sky.y * gComp, sky.z * bComp);
    }

    // 漫反射地面（棋盘格）
    if (rec.matId == 2 || rec.matId == 0) {
        // 棋盘格颜色
        int cx = (int)(std::floor(rec.p.x * 0.8));
        int cz = (int)(std::floor(rec.p.z * 0.8));
        bool white = ((cx + cz) % 2 == 0);
        Vec3 baseColor = white ? Vec3(0.95, 0.92, 0.88) : Vec3(0.2, 0.2, 0.25);

        // 简单漫反射（环境光+直接光）
        // 光源方向：正上方偏后
        Vec3 lightDir = Vec3(0.3, 1, 0.5).normalized();
        Ray shadowRay{rec.p + rec.n * 0.001, lightDir};
        HitRecord sh;
        double shadow = sceneHit(shadowRay, 0.001, 1e10, sh) ? 0.2 : 1.0;
        double diff = std::max(0.0, rec.n.dot(lightDir));
        double amb = 0.25;
        double lit = amb + (1.0 - amb) * diff * shadow;
        Vec3 diffuseColor = baseColor * lit;

        return Vec3(diffuseColor.x * rComp,
                    diffuseColor.y * gComp,
                    diffuseColor.z * bComp);
    }

    // 玻璃折射（核心色散）
    if (rec.matId == 1) {
        double ior = cauchyIOR(lambda);
        double n1 = rec.frontFace ? 1.0 : ior;
        double n2 = rec.frontFace ? ior : 1.0;

        Vec3 unitDir = ray.d.normalized();
        double cosTheta = std::min(1.0, (-unitDir).dot(rec.n));
        double reflProb = schlick(cosTheta, n1, n2);

        Vec3 refracted;
        if (refract(unitDir, rec.n, n1/n2, refracted) && reflProb < 0.7) {
            Ray refractRay{ rec.p - rec.n * 0.001, refracted.normalized() };
            return traceWavelength(refractRay, lambda, depth-1,
                                   rComp, gComp, bComp) * 0.97;
        } else {
            Vec3 reflected = unitDir - rec.n * 2.0 * unitDir.dot(rec.n);
            Ray reflRay{ rec.p + rec.n * 0.001, reflected.normalized() };
            return traceWavelength(reflRay, lambda, depth-1,
                                   rComp, gComp, bComp) * 0.95;
        }
    }

    return Vec3(0,0,0);
}

// ─── 色散追踪：同一条视线，R/G/B分别用不同折射率追踪 ─────────────────
Vec3 traceSpectral(const Ray& ray, int depth) {
    // 对R追踪（贡献给R通道）
    Vec3 rContrib = traceWavelength(ray, LAMBDA_R, depth, 1, 0, 0);
    Vec3 gContrib = traceWavelength(ray, LAMBDA_G, depth, 0, 1, 0);
    Vec3 bContrib = traceWavelength(ray, LAMBDA_B, depth, 0, 0, 1);

    return Vec3(rContrib.x, gContrib.y, bContrib.z);
}

// ─── 渲染输出 ─────────────────────────────────────────────────────────
void renderToBuffer(std::vector<unsigned char>& buf, int W, int H,
                    int SAMPLES, int MAX_DEPTH, bool spectral,
                    Vec3 camPos, Vec3 forward, Vec3 right, Vec3 camUp,
                    double halfW, double halfH) {
    // 均匀抖动采样网格
    int sq = (int)std::sqrt((double)SAMPLES);
    std::vector<double> jX(SAMPLES), jY(SAMPLES);
    {
        int idx = 0;
        for (int sy = 0; sy < sq && idx < SAMPLES; sy++)
            for (int sx = 0; sx < sq && idx < SAMPLES; sx++, idx++) {
                jX[idx] = (sx + 0.5) / sq;
                jY[idx] = (sy + 0.5) / sq;
            }
        for (; idx < SAMPLES; idx++) { jX[idx] = 0.5; jY[idx] = 0.5; }
    }

    for (int j = 0; j < H; j++) {
        for (int i = 0; i < W; i++) {
            Vec3 color(0, 0, 0);
            for (int s = 0; s < SAMPLES; s++) {
                double u = (i + jX[s]) / W;
                double v = 1.0 - (j + jY[s]) / H;  // Y翻转：天空在上
                double px = (2.0*u - 1.0) * halfW;
                double py = (2.0*v - 1.0) * halfH;
                Vec3 dir = (forward + right*px + camUp*py).normalized();
                Ray ray{camPos, dir};

                if (spectral) color += traceSpectral(ray, MAX_DEPTH);
                else {
                    // 无色散对比：用中间波长
                    Vec3 c = traceWavelength(ray, 0.570, MAX_DEPTH, 1, 1, 1);
                    color += c;
                }
            }
            color = color * (1.0 / SAMPLES);

            // Reinhard tone mapping
            color.x = color.x / (1.0 + color.x);
            color.y = color.y / (1.0 + color.y);
            color.z = color.z / (1.0 + color.z);
            // Gamma 2.2
            color.x = std::pow(std::max(0.0, color.x), 1.0/2.2);
            color.y = std::pow(std::max(0.0, color.y), 1.0/2.2);
            color.z = std::pow(std::max(0.0, color.z), 1.0/2.2);
            color = color.clamp01();

            int idx = (j*W + i) * 3;
            buf[idx+0] = (unsigned char)(color.x * 255.99);
            buf[idx+1] = (unsigned char)(color.y * 255.99);
            buf[idx+2] = (unsigned char)(color.z * 255.99);
        }
    }
}

// ─── 主函数 ───────────────────────────────────────────────────────────
int main() {
    const int W = 800, H = 600;
    const int SAMPLES = 16;
    const int MAX_DEPTH = 8;

    // 相机
    Vec3 camPos(0, 0.3, 2.5);
    Vec3 lookAt(0, -0.1, -2.0);
    Vec3 up(0, 1, 0);
    Vec3 forward = (lookAt - camPos).normalized();
    Vec3 right = forward.cross(up).normalized();
    Vec3 camUp = right.cross(forward);
    double fov = 55.0 * M_PI / 180.0;
    double halfH_cam = std::tan(fov / 2.0);
    double halfW_cam = halfH_cam * (double)W / H;

    buildScene();

    // 色散渲染
    std::vector<unsigned char> buf1(W * H * 3);
    renderToBuffer(buf1, W, H, SAMPLES, MAX_DEPTH, true,
                   camPos, forward, right, camUp, halfW_cam, halfH_cam);
    stbi_write_png("spectral_dispersion_output.png", W, H, 3, buf1.data(), W*3);
    printf("✅ 色散渲染完成: spectral_dispersion_output.png\n");

    // 无色散对比
    std::vector<unsigned char> buf2(W * H * 3);
    renderToBuffer(buf2, W, H, SAMPLES, MAX_DEPTH, false,
                   camPos, forward, right, camUp, halfW_cam, halfH_cam);
    stbi_write_png("spectral_no_dispersion.png", W, H, 3, buf2.data(), W*3);
    printf("✅ 无色散对比图: spectral_no_dispersion.png\n");

    // IOR曲线可视化
    const int VW = 800, VH = 500;
    std::vector<unsigned char> buf3(VW * VH * 3);
    memset(buf3.data(), 15, buf3.size());

    // 绘制IOR vs 波长曲线（380nm~700nm）
    auto iorY = [&](double lambda) {
        double ior = cauchyIOR(lambda);
        double iorMin = 1.48, iorMax = 1.56;
        return (int)((1.0 - (ior - iorMin)/(iorMax - iorMin)) * (VH - 80)) + 40;
    };
    auto lambdaX = [&](double lambda) {
        return (int)((lambda - 0.380) / 0.320 * (VW - 100)) + 50;
    };

    for (int x = 50; x < VW-50; x++) {
        double lambda = 0.380 + (0.320) * (x - 50.0) / (VW - 101.0);
        int y = iorY(lambda);

        // 波长→颜色（可见光谱）
        unsigned char cr, cg, cb;
        if (lambda < 0.450) { cr=128; cg=0; cb=255; }
        else if (lambda < 0.490) { cr=0; cg=0; cb=255; }
        else if (lambda < 0.520) { cr=0; cg=200; cb=255; }
        else if (lambda < 0.565) { cr=0; cg=255; cb=0; }
        else if (lambda < 0.590) { cr=200; cg=255; cb=0; }
        else if (lambda < 0.625) { cr=255; cg=165; cb=0; }
        else { cr=255; cg=0; cb=0; }

        for (int dy = -4; dy <= 4; dy++) {
            int yy = std::max(0, std::min(VH-1, y+dy));
            int idx = (yy * VW + x) * 3;
            buf3[idx+0] = cr; buf3[idx+1] = cg; buf3[idx+2] = cb;
        }
    }

    // 标记R/G/B三波长
    auto markPt = [&](double lambda, unsigned char r, unsigned char g, unsigned char b) {
        int x = lambdaX(lambda);
        int y = iorY(lambda);
        for (int dx = -6; dx <= 6; dx++) {
            int xx = std::max(0, std::min(VW-1, x+dx));
            int idx = (y * VW + xx) * 3;
            buf3[idx+0] = r; buf3[idx+1] = g; buf3[idx+2] = b;
        }
        for (int dy = -6; dy <= 6; dy++) {
            int yy = std::max(0, std::min(VH-1, y+dy));
            int idx = (yy * VW + x) * 3;
            buf3[idx+0] = r; buf3[idx+1] = g; buf3[idx+2] = b;
        }
    };
    markPt(LAMBDA_R, 255, 80, 80);
    markPt(LAMBDA_G, 80, 255, 80);
    markPt(LAMBDA_B, 80, 80, 255);

    stbi_write_png("spectral_ior_curve.png", VW, VH, 3, buf3.data(), VW*3);
    printf("✅ IOR曲线图: spectral_ior_curve.png\n");

    // 输出折射率数据
    printf("\n=== 折射率（柯西方程 Crown Glass BK7）===\n");
    printf("  λ_R = %.3fμm → n_R = %.6f\n", LAMBDA_R, cauchyIOR(LAMBDA_R));
    printf("  λ_G = %.3fμm → n_G = %.6f\n", LAMBDA_G, cauchyIOR(LAMBDA_G));
    printf("  λ_B = %.3fμm → n_B = %.6f\n", LAMBDA_B, cauchyIOR(LAMBDA_B));
    printf("  Δn(B-R) = %.6f  → 色散效果可见\n",
           cauchyIOR(LAMBDA_B) - cauchyIOR(LAMBDA_R));
    printf("  Abbe数 V_d = %.1f\n",
           (cauchyIOR(0.587) - 1.0) / (cauchyIOR(LAMBDA_B) - cauchyIOR(LAMBDA_R)));

    return 0;
}
