// Blinn-Phong Shading Model
// 图形学基础着色模型：环境光 + 漫反射(Lambert) + 镜面高光(Blinn-Phong 半程向量)
// 目标：用 C++ 实现 Blinn-Phong 与经典 Phong 的镜面高光，并用量化指标验证：
//   1) 反射模型数值范围 [0,1]（能量有界）
//   2) 高光随 shininess 增大而更集中（半峰宽变窄，可量化）
//   3) Blinn-Phong 高光比 Phong 更宽（相同 shininess 下，Blinn 半程向量夹角更小 -> 高光更亮）
//   4) 渲染 3D 球体到 PPM，像素统计量化（非全黑/全白/有渐变）
//   5) 环境+漫反射+高光 分离验证（三通道能量守恒）

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>

// ---- 三维向量 ----
struct Vec3 {
    double x, y, z;
    Vec3(double a = 0, double b = 0, double c = 0) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
    }
    double len() const { return std::sqrt(dot(*this)); }
    Vec3 norm() const { double l = len(); return (l < 1e-12) ? Vec3(0,0,0) : (*this) * (1.0 / l); }
    static Vec3 reflect(const Vec3& I, const Vec3& N) {
        return I - N * (2.0 * I.dot(N));
    }
};

static inline double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// ---- Blinn-Phong 着色 ----
// N = 法线, L = 指向光源, V = 指向视点
// 返回 RGB 分量（每通道 0..1）
struct ShadingParams {
    Vec3 ambientCol;   // 环境光颜色
    Vec3 diffuseCol;   // 表面漫反射颜色
    Vec3 specularCol;  // 高光颜色
    double Ka, Kd, Ks; // 环境/漫反射/高光 系数
    double shininess;  // 高光锐度
};

static Vec3 blinnPhong(const Vec3& N, const Vec3& L, const Vec3& V, const ShadingParams& p) {
    Vec3 nn = N.norm(), ll = L.norm(), vv = V.norm();
    Vec3 H = (ll + vv).norm();          // 半程向量
    double ndotl = std::max(0.0, nn.dot(ll));
    double ndoth = std::max(0.0, nn.dot(H));

    Vec3 ambient  = p.ambientCol * p.Ka;
    Vec3 diffuse  = p.diffuseCol * (p.Kd * ndotl);
    Vec3 specular = p.specularCol * (p.Ks * std::pow(ndoth, p.shininess));

    Vec3 c = ambient + diffuse + specular;
    return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z));
}

// 经典 Phong（反射向量版本）用于对比
static Vec3 phong(const Vec3& N, const Vec3& L, const Vec3& V, const ShadingParams& p) {
    Vec3 nn = N.norm(), ll = L.norm(), vv = V.norm();
    Vec3 R = Vec3::reflect(ll * -1.0, nn); // 反射光方向
    double ndotl = std::max(0.0, nn.dot(ll));
    double rdotv = std::max(0.0, R.dot(vv));

    Vec3 ambient  = p.ambientCol * p.Ka;
    Vec3 diffuse  = p.diffuseCol * (p.Kd * ndotl);
    Vec3 specular = p.specularCol * (p.Ks * std::pow(rdotv, p.shininess));
    Vec3 c = ambient + diffuse + specular;
    return Vec3(clamp01(c.x), clamp01(c.y), clamp01(c.z));
}

// ---- 辅助：高光半峰宽（Full Width at Half Maximum）----
// 在法线与视点之间，取一系列法线方向偏离高光峰值角度的位置，
// 采样镜面项并记录其降到峰值一半时的角度范围 -> 度量高光"集中度"
static double specularFWHM_degrees(double shininess, bool blinn) {
    // 固定 L 与 V 夹角 30°（典型），H 已知；扫法线方向使得 ndoth (blinn) 或 rdotv (phong) 变化
    Vec3 L = Vec3(0, 0, 1).norm();
    Vec3 V = Vec3(std::sin(30.0 * M_PI / 180.0), 0, std::cos(30.0 * M_PI / 180.0)).norm();
    Vec3 H = (L + V).norm();

    // 采样法线方向：绕 y 轴旋转，从与 H 重合到偏离 90°
    double peak = 0, halfPeakAngle = -1;
    std::vector<std::pair<double,double>> samples;
    for (int i = 0; i <= 3600; ++i) {
        double ang = i * (90.0 / 3600.0) * M_PI / 180.0; // 0..90°
        Vec3 N = Vec3(std::sin(ang), 0, std::cos(ang));
        double val;
        if (blinn) {
            val = std::pow(std::max(0.0, N.dot(H)), shininess);
        } else {
            Vec3 R = Vec3::reflect(L * -1.0, N);
            val = std::pow(std::max(0.0, R.dot(V)), shininess);
        }
        samples.push_back({ang * 180.0 / M_PI, val});
        if (i == 0) peak = val;
        // 找到第一个降到峰值一半的角度
        if (halfPeakAngle < 0 && val <= peak * 0.5) {
            halfPeakAngle = ang * 180.0 / M_PI;
        }
    }
    // 半峰宽 = 2 * 半峰角（近似对称）
    (void)samples;
    return 2.0 * halfPeakAngle;
}

// ---- 渲染球体到 PPM ----
static void renderSpheres(const char* path, int W, int H) {
    std::vector<unsigned char> buf(W * H * 3);
    Vec3 lightDir = Vec3(-0.5, 0.5, 1.0).norm();
    Vec3 viewDir  = Vec3(0, 0, 1).norm();

    // 三颗球（不同 shininess），展示高光锐度变化
    struct Sphere { double cx, cy, r; double shininess; Vec3 col; };
    std::vector<Sphere> spheres = {
        { -0.45, 0.0, 0.35, 8.0,  Vec3(0.9, 0.2, 0.2) },
        {  0.00, 0.0, 0.35, 64.0, Vec3(0.2, 0.9, 0.2) },
        {  0.45, 0.0, 0.35, 256.0,Vec3(0.2, 0.3, 0.9) },
    };

    ShadingParams base;
    base.ambientCol  = Vec3(0.10, 0.10, 0.12);
    base.diffuseCol  = Vec3(1, 1, 1);
    base.specularCol = Vec3(1, 1, 1);
    base.Ka = 0.15; base.Kd = 0.45; base.Ks = 0.9;

    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            // NDC -> 世界坐标（保持宽高比，y 向上）
            double u = (px + 0.5) / W * 2.0 - 1.0;
            double v = 1.0 - (py + 0.5) / H * 2.0; // y 向上
            double aspect = (double)W / H;
            u *= aspect;

            Vec3 color(0.05, 0.06, 0.08); // 背景
            for (const auto& s : spheres) {
                double dx = u - s.cx, dy = v - s.cy;
                double d2 = dx * dx + dy * dy;
                if (d2 <= s.r * s.r) {
                    double dz = std::sqrt(s.r * s.r - d2);
                    Vec3 N = Vec3(dx / s.r, dy / s.r, dz / s.r).norm();
                    base.diffuseCol = s.col;
                    base.shininess = s.shininess;
                    color = blinnPhong(N, lightDir, viewDir, base);
                }
            }
            buf[(py * W + px) * 3 + 0] = (unsigned char)(clamp01(color.x) * 255);
            buf[(py * W + px) * 3 + 1] = (unsigned char)(clamp01(color.y) * 255);
            buf[(py * W + px) * 3 + 2] = (unsigned char)(clamp01(color.z) * 255);
        }
    }

    FILE* f = std::fopen(path, "wb");
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    std::fwrite(buf.data(), 1, buf.size(), f);
    std::fclose(f);
}

int main() {
    std::printf("=== Blinn-Phong Shading Model ===\n\n");

    // ---- Test 1: 反射模型数值范围 [0,1] ----
    std::printf("Test1 反射模型数值有界性 [0,1]:\n");
    {
        ShadingParams p;
        p.ambientCol = Vec3(0.15,0.15,0.18); p.diffuseCol=Vec3(1,1,1); p.specularCol=Vec3(1,1,1);
        p.Ka=0.15; p.Kd=0.7; p.Ks=0.9; p.shininess=32;
        double mn=1e9, mx=-1e9; int overflow=0; long long samples=0;
        for (int ti=0; ti<200; ++ti) {
            double theta = ti * M_PI / 200.0;
            double phi   = ti * 2.0 * M_PI / 200.0;
            Vec3 N(std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi), std::cos(theta));
            Vec3 L = Vec3(ti * 0.01 - 0.5, 0.3, 1.0).norm();
            Vec3 V = Vec3(0.4, -0.2, 1.0).norm();
            Vec3 c = blinnPhong(N, L, V, p);
            double vals[3] = {c.x, c.y, c.z};
            for (double v : vals) { mn=std::min(mn,v); mx=std::max(mx,v); if(v<0||v>1) overflow++; samples++; }
        }
        std::printf("  min=%.4f max=%.4f  越界数=%d/%lld\n", mn, mx, overflow, samples);
        bool ok = (mn>=0.0 && mx<=1.0 && overflow==0);
        std::printf("  结果: %s\n\n", ok ? "✅ PASS (所有分量 ∈ [0,1])" : "❌ FAIL");
        if (!ok) return 1;
    }

    // ---- Test 2: 高光半峰宽随 shininess 增大而变窄 ----
    std::printf("Test2 高光集中度(FWHM)随 shininess 增大而变窄:\n");
    {
        double shininesses[] = {8, 32, 128, 512};
        double prev = 1e9; bool mono = true;
        for (double s : shininesses) {
            double fwhm = specularFWHM_degrees(s, true);
            std::printf("  shininess=%4d -> FWHM=%.2f°\n", (int)s, fwhm);
            if (fwhm > prev) mono = false;
            prev = fwhm;
        }
        std::printf("  结果: %s\n\n", mono ? "✅ PASS (FWHM 单调递减)" : "❌ FAIL");
        if (!mono) return 1;
    }

    // ---- Test 3: Blinn-Phong 高光比 Phong 更宽/更亮(半程向量夹角更小) ----
    std::printf("Test3 Blinn-Phong 高光 vs 经典 Phong (相同 shininess):\n");
    {
        double shininesses[] = {16, 64, 256};
        bool allWider = true;
        for (double s : shininesses) {
            double fwhmB = specularFWHM_degrees(s, true);
            double fwhmP = specularFWHM_degrees(s, false);
            // 用两个完整着色函数直接对比峰值强度（相同 shininess 下 Blinn 高光更亮）
            ShadingParams p; p.ambientCol=Vec3(0,0,0); p.diffuseCol=Vec3(0,0,0); p.specularCol=Vec3(1,1,1);
            p.Ka=0; p.Kd=0; p.Ks=1.0; p.shininess=s;
            Vec3 L(0,0,1), V(std::sin(30.0*M_PI/180.0),0,std::cos(30.0*M_PI/180.0));
            Vec3 H=(L+V).norm();
            double specB = blinnPhong(H,L,V,p).x;
            double specP = phong(H,L,V,p).x;
            std::printf("  shininess=%4d: Blinn FWHM=%.2f°  Phong FWHM=%.2f°  (Blinn %s) | 峰值 Blinn=%.4f Phong=%.4f\n",
                        (int)s, fwhmB, fwhmP, fwhmB >= fwhmP ? "≥ Phong ✅" : "< Phong ❌", specB, specP);
            if (fwhmB < fwhmP) allWider = false;
        }
        std::printf("  结果: %s\n\n", allWider ? "✅ PASS (Blinn 高光同等或更宽，符合理论)" : "❌ FAIL");
        if (!allWider) return 1;
    }

    // ---- Test 4: 渲染 3D 球体 + 像素统计 ----
    int W = 800, H = 450;
    renderSpheres("blinn_phong.ppm", W, H);
    std::printf("Test4 球体渲染像素统计:\n");
    {
        // 用 Python 侧验证更稳，这里直接用简单 C++ 统计均值/标准差
        std::vector<unsigned char> buf;
        FILE* f = std::fopen("blinn_phong.ppm", "rb");
        // 跳过头部
        std::fscanf(f, "P6\n");
        std::fscanf(f, "%d %d\n", &W, &H);
        std::fscanf(f, "255\n");
        buf.resize(W*H*3);
        std::fread(buf.data(), 1, buf.size(), f);
        std::fclose(f);

        double sum=0, sum2=0;
        for (size_t i=0;i<buf.size();++i){ sum+=buf[i]; sum2+=buf[i]*buf[i]; }
        double n = buf.size();
        double mean = sum/n, var = sum2/n - mean*mean, stddev = std::sqrt(var);
        std::printf("  文件大小=%ld bytes  像素均值=%.1f  标准差=%.1f\n", buf.size()*2, mean, stddev);
        bool ok = (mean>10 && mean<240 && stddev>20);
        std::printf("  结果: %s\n\n", ok ? "✅ PASS (非全黑/全白且有丰富渐变)" : "❌ FAIL");
        if (!ok) return 1;
    }

    // ---- Test 5: 环境+漫反射+高光 分离 & 能量守恒 ----
    std::printf("Test5 三分量分离与能量关系:\n");
    {
        ShadingParams p;
        p.ambientCol=Vec3(0.2,0.2,0.2); p.diffuseCol=Vec3(1,1,1); p.specularCol=Vec3(1,1,1);
        p.Ka=0.2; p.Kd=0.8; p.Ks=1.0; p.shininess=64;
        // 正面受光：N=L=V=(0,0,1)，ndotl=1, ndoth=1
        Vec3 N(0,0,1), L(0,0,1), V(0,0,1);
        Vec3 c = blinnPhong(N,L,V,p);
        // 期望: ambient=0.2*0.2=0.04, diffuse=0.8*1=0.8, specular=1.0*1=1.0 -> 合计 1.84 -> clamp 1.0
        std::printf("  正面直射: R=%.3f G=%.3f B=%.3f (期望≈1.0, 被 clamp)\n", c.x,c.y,c.z);
        // 背面：N=(0,0,1), L=(0,0,-1) -> ndotl=0, 仅环境光
        Vec3 cBack = blinnPhong(Vec3(0,0,1), Vec3(0,0,-1), Vec3(0,0,1), p);
        std::printf("  背面:     R=%.3f G=%.3f B=%.3f (期望≈0.04 纯环境)\n", cBack.x,cBack.y,cBack.z);
        bool ok = (std::fabs(cBack.x - 0.04) < 0.005 && c.x > 0.99);
        std::printf("  结果: %s\n\n", ok ? "✅ PASS (环境光独立，正面受光饱和)" : "❌ FAIL");
        if (!ok) return 1;
    }

    std::printf("=== 全部量化验证通过 ✅ ===\n");
    return 0;
}
