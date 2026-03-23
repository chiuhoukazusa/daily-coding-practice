/**
 * Subsurface Scattering Renderer
 * 日期: 2026-03-24
 *
 * 实现技术:
 *   - Jensen 偶极子模型 (Dipole Approximation)
 *   - BSSRDF (Bidirectional Subsurface Scattering Reflectance Distribution Function)
 *   - 漫射剖面 R_d(r) 积分
 *   - 多材质: 皮肤 / 大理石 / 蜡烛
 *   - 简单光线追踪框架 (直接光照 + SSS)
 *   - PPM 图像输出
 *
 * 编译: g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra -lm
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <random>
#include <string>
#include <array>

// ─────────────────────────────────────────────
// 基础向量
// ─────────────────────────────────────────────
struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t)      const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t)      const { return {x/t, y/t, z/t}; }
    Vec3 operator-()              const { return {-x, -y, -z}; }
    double dot(const Vec3& o)     const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o)     const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length2() const { return x*x + y*y + z*z; }
    double length()  const { return std::sqrt(length2()); }
    Vec3 norm()      const { double l = length(); return l > 1e-10 ? (*this)/l : Vec3(); }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
};

// ─────────────────────────────────────────────
// 图像缓冲
// ─────────────────────────────────────────────
struct Image {
    int W, H;
    std::vector<Vec3> pixels;
    Image(int w, int h): W(w), H(h), pixels(w*h) {}
    Vec3& at(int x, int y) { return pixels[y*W + x]; }
    const Vec3& at(int x, int y) const { return pixels[y*W + x]; }
    void savePPM(const std::string& name) const {
        FILE* f = fopen(name.c_str(), "wb");
        fprintf(f, "P6\n%d %d\n255\n", W, H);
        for (const auto& p : pixels) {
            // ACES Tone mapping
            auto tonemapACES = [](double v) {
                double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
                return (v*(a*v+b)) / (v*(c*v+d)+e);
            };
            auto gamma = [](double v) {
                return std::pow(std::max(0.0, std::min(1.0, v)), 1.0/2.2);
            };
            unsigned char r = (unsigned char)(255 * gamma(tonemapACES(p.x)));
            unsigned char g = (unsigned char)(255 * gamma(tonemapACES(p.y)));
            unsigned char b = (unsigned char)(255 * gamma(tonemapACES(p.z)));
            fwrite(&r,1,1,f); fwrite(&g,1,1,f); fwrite(&b,1,1,f);
        }
        fclose(f);
    }
};

// ─────────────────────────────────────────────
// 随机数
// ─────────────────────────────────────────────
thread_local std::mt19937 rng(42);
double randF() { return std::uniform_real_distribution<double>(0,1)(rng); }

// 余弦重要性采样半球
Vec3 cosineSampleHemisphere() {
    double r1 = randF(), r2 = randF();
    double r = std::sqrt(r1);
    double theta = 2 * M_PI * r2;
    double x = r * std::cos(theta);
    double y = r * std::sin(theta);
    double z = std::sqrt(std::max(0.0, 1.0 - r1));
    return {x, y, z};
}

// 把向量从局部坐标系变换到世界坐标系
Vec3 toWorld(const Vec3& n, const Vec3& localV) {
    Vec3 up = (std::abs(n.y) < 0.99) ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 tangent  = up.cross(n).norm();
    Vec3 bitangent = n.cross(tangent);
    return tangent * localV.x + bitangent * localV.y + n * localV.z;
}

// ─────────────────────────────────────────────
// Jensen 偶极子 SSS 材质参数
// ─────────────────────────────────────────────
struct SSSMaterial {
    Vec3 sigma_a;    // 吸收系数 (per wavelength)
    Vec3 sigma_sp;   // 折减散射系数
    double eta;      // 折射率
    Vec3 albedo;     // 表面颜色（直接光照）

    // 导出参数
    Vec3 sigma_t()   const { return sigma_a + sigma_sp; }
    Vec3 sigma_tr()  const {
        Vec3 st = sigma_t();
        return Vec3(
            std::sqrt(3.0 * sigma_a.x * st.x),
            std::sqrt(3.0 * sigma_a.y * st.y),
            std::sqrt(3.0 * sigma_a.z * st.z)
        );
    }

    // Fresnel 内反射系数近似 (Jensen 2001)
    double Fdr() const {
        // Parameterization by eta
        if (eta >= 1.0) {
            return -1.440 / (eta*eta) + 0.710 / eta + 0.668 + 0.0636 * eta;
        } else {
            return -0.4399 + 0.7099/eta - 0.3319/(eta*eta) + 0.0636/(eta*eta*eta);
        }
    }

    double A() const {
        double fdr = Fdr();
        return (1.0 + fdr) / (1.0 - fdr);
    }

    // 漫射剖面 R_d(r): 偶极子模型核心公式
    // R_d(r) = alpha_prime/(4*pi) * [z_r*(sigma_tr_r+1/d_r)*exp(-sigma_tr*d_r)/d_r^2 +
    //                                  z_v*(sigma_tr_v+1/d_v)*exp(-sigma_tr*d_v)/d_v^2]
    Vec3 Rd(double r) const {
        Vec3 st = sigma_t();
        Vec3 sp = sigma_sp;

        // alpha' = sigma_sp / sigma_t
        Vec3 alphap = Vec3(sp.x/st.x, sp.y/st.y, sp.z/st.z);

        // 有效传输系数
        Vec3 sigtr = sigma_tr();
        double a = A();

        // 偶极子深度
        // z_r = 1 / sigma_t
        Vec3 zr = Vec3(1.0/st.x, 1.0/st.y, 1.0/st.z);
        // z_v = z_r + 4*A/3 / sigma_t... = z_r * (1 + 4A/3)
        // Actually: z_v = z_r + 4*A*D where D = 1/(3*sigma_t)
        Vec3 D = Vec3(1.0/(3.0*st.x), 1.0/(3.0*st.y), 1.0/(3.0*st.z));
        Vec3 zv = Vec3(zr.x + 4.0*a*D.x, zr.y + 4.0*a*D.y, zr.z + 4.0*a*D.z);

        Vec3 result;
        for (int c = 0; c < 3; c++) {
            double sc = (c==0)?sigtr.x:(c==1)?sigtr.y:sigtr.z;
            double zcr = (c==0)?zr.x:(c==1)?zr.y:zr.z;
            double zcv = (c==0)?zv.x:(c==1)?zv.y:zv.z;
            double alc = (c==0)?alphap.x:(c==1)?alphap.y:alphap.z;

            double d_r = std::sqrt(r*r + zcr*zcr);
            double d_v = std::sqrt(r*r + zcv*zcv);

            double term_r = zcr * (sc + 1.0/d_r) * std::exp(-sc * d_r) / (d_r * d_r);
            double term_v = zcv * (sc + 1.0/d_v) * std::exp(-sc * d_v) / (d_v * d_v);

            double val = (alc / (4.0 * M_PI)) * (term_r + term_v);
            if (c==0) result.x = val;
            else if (c==1) result.y = val;
            else result.z = val;
        }
        return result;
    }
};

// ─────────────────────────────────────────────
// 预定义材质
// ─────────────────────────────────────────────
namespace Materials {
    // 皮肤 (Jensen 2001 Table 2 近似值，单位 mm^-1)
    SSSMaterial skin() {
        return {
            {0.032, 0.17, 0.48},    // sigma_a
            {0.74, 0.88, 1.01},     // sigma_sp
            1.4,                     // eta
            {0.9, 0.7, 0.6}         // albedo
        };
    }
    // 大理石
    SSSMaterial marble() {
        return {
            {0.0021, 0.0041, 0.0071},
            {2.19, 2.62, 3.00},
            1.5,
            {0.9, 0.9, 0.85}
        };
    }
    // 蜡烛/牛奶
    SSSMaterial wax() {
        return {
            {0.004, 0.009, 0.016},
            {1.0, 0.8, 0.5},
            1.45,
            {1.0, 0.9, 0.7}
        };
    }
    // 玉石
    SSSMaterial jade() {
        return {
            {0.005, 0.012, 0.02},
            {0.6, 1.2, 0.9},
            1.55,
            {0.6, 0.85, 0.65}
        };
    }
}

// ─────────────────────────────────────────────
// 场景：球体
// ─────────────────────────────────────────────
struct Sphere {
    Vec3 center;
    double radius;
    int matIdx;  // 0=skin 1=marble 2=wax 3=jade
    bool isLight;
    Vec3 emission;
};

struct Hit {
    bool valid = false;
    double t = 1e18;
    Vec3 pos, normal;
    int matIdx;
};

Hit intersectSphere(const Vec3& ro, const Vec3& rd, const Sphere& s) {
    Vec3 oc = ro - s.center;
    double a = rd.dot(rd);
    double b = 2.0 * oc.dot(rd);
    double c = oc.dot(oc) - s.radius * s.radius;
    double disc = b*b - 4*a*c;
    if (disc < 0) return {};
    double sq = std::sqrt(disc);
    double t = (-b - sq) / (2*a);
    if (t < 1e-4) t = (-b + sq) / (2*a);
    if (t < 1e-4) return {};
    Hit h;
    h.valid = true;
    h.t = t;
    h.pos = ro + rd * t;
    h.normal = (h.pos - s.center).norm();
    h.matIdx = s.matIdx;
    return h;
}

// ─────────────────────────────────────────────
// SSS 直方采样 (Texture-space diffusion 近似)
// 对球体表面做均匀随机采样，用 R_d(|xi - xo|) 权重积分
// ─────────────────────────────────────────────

// 球面均匀采样
Vec3 uniformSampleSphere() {
    double u = randF(), v = randF();
    double theta = 2.0*M_PI * u;
    double phi   = std::acos(1.0 - 2.0*v);
    return {
        std::sin(phi)*std::cos(theta),
        std::sin(phi)*std::sin(theta),
        std::cos(phi)
    };
}

// ─────────────────────────────────────────────
// 光源 & 直接光照
// ─────────────────────────────────────────────
struct PointLight {
    Vec3 pos;
    Vec3 color;
    double intensity;
};

// ─────────────────────────────────────────────
// 场景定义
// ─────────────────────────────────────────────
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<PointLight> lights;
    std::array<SSSMaterial, 4> mats;

    Scene() {
        mats[0] = Materials::skin();
        mats[1] = Materials::marble();
        mats[2] = Materials::wax();
        mats[3] = Materials::jade();

        // 四个 SSS 球体，排成一排
        spheres.push_back({{-4.5, 0, 0}, 1.4, 0, false, {}});  // 皮肤
        spheres.push_back({{-1.5, 0, 0}, 1.4, 1, false, {}});  // 大理石
        spheres.push_back({{ 1.5, 0, 0}, 1.4, 2, false, {}});  // 蜡烛
        spheres.push_back({{ 4.5, 0, 0}, 1.4, 3, false, {}});  // 玉石

        // 地面大球
        spheres.push_back({{0, -1002, 0}, 1000.0, 1, false, {}});

        // 光源
        lights.push_back({{0, 8, 4}, {1.0, 1.0, 0.9}, 80.0});
        lights.push_back({{-6, 5, 3}, {0.8, 0.6, 0.5}, 40.0});
    }

    Hit trace(const Vec3& ro, const Vec3& rd) const {
        Hit best;
        for (const auto& s : spheres) {
            auto h = intersectSphere(ro, rd, s);
            if (h.valid && h.t < best.t) {
                best = h;
            }
        }
        return best;
    }

    // 检查阴影
    bool inShadow(const Vec3& pos, const Vec3& lightPos) const {
        Vec3 dir = (lightPos - pos).norm();
        double dist = (lightPos - pos).length();
        Vec3 ro = pos + dir * 1e-3;
        Hit h = trace(ro, dir);
        return h.valid && h.t < dist - 1e-3;
    }
};

// ─────────────────────────────────────────────
// SSS 贡献计算 (对球体表面积分)
// ─────────────────────────────────────────────
Vec3 computeSSS(
    const Scene& scene,
    const Sphere& sphere,
    const Vec3& xo,         // 出射点
    const Vec3& no,         // 出射点法线
    const SSSMaterial& mat,
    int sampleCount
) {
    Vec3 result;

    double r = sphere.radius;
    double area = 4.0 * M_PI * r * r;

    for (int i = 0; i < sampleCount; i++) {
        // 在球面上均匀采样入射点 xi
        Vec3 localDir = uniformSampleSphere();
        Vec3 xi = sphere.center + localDir * r;
        Vec3 ni = localDir;  // 球面法线

        // 计算入射点直接光照 (Lambert)
        Vec3 Li;
        for (const auto& light : scene.lights) {
            if (!scene.inShadow(xi + ni * 1e-3, light.pos)) {
                Vec3 ld = (light.pos - xi).norm();
                double ndotl = std::max(0.0, ni.dot(ld));
                double dist2 = (light.pos - xi).length2();
                Li += light.color * (light.intensity * ndotl / dist2);
            }
        }

        // r_dist = 入射点与出射点间距离
        double dist = (xi - xo).length();

        // R_d 漫射剖面
        Vec3 Rd = mat.Rd(dist);

        // r_dist = 入射点与出射点间距离
        // 采样权重 = area (因为均匀采样)
        Vec3 contribution = Rd * Li * (area / sampleCount);
        // 乘以出射方向透射因子
        result += contribution * std::max(0.0, no.dot(ni) * 0.5 + 0.5);
    }

    return result * (1.0 / M_PI);
}

// ─────────────────────────────────────────────
// 直接 Phong 光照
// ─────────────────────────────────────────────
Vec3 directLighting(const Scene& scene, const Vec3& pos, const Vec3& normal,
                     const Vec3& viewDir, const Vec3& albedo) {
    Vec3 result;
    for (const auto& light : scene.lights) {
        if (scene.inShadow(pos + normal * 1e-3, light.pos)) continue;
        Vec3 ld = (light.pos - pos).norm();
        double dist2 = (light.pos - pos).length2();
        double ndotl = std::max(0.0, normal.dot(ld));

        // Diffuse
        Vec3 diff = albedo * (light.color * light.intensity * ndotl / dist2);

        // Specular (Blinn-Phong)
        Vec3 h = (ld + viewDir).norm();
        double spec = std::pow(std::max(0.0, normal.dot(h)), 64.0);
        Vec3 specCol = light.color * (light.intensity * spec * 0.3 / dist2);

        result += diff + specCol;
    }
    return result;
}

// ─────────────────────────────────────────────
// 渲染单像素
// ─────────────────────────────────────────────
Vec3 renderPixel(const Scene& scene, const Vec3& ro, const Vec3& rd, int sssSamples) {
    Hit hit = scene.trace(ro, rd);
    if (!hit.valid) {
        // 背景渐变
        double t = 0.5 * (rd.y + 1.0);
        return Vec3(0.1, 0.15, 0.3) * (1-t) + Vec3(0.3, 0.4, 0.6) * t;
    }

    const SSSMaterial& mat = scene.mats[hit.matIdx];
    Vec3 viewDir = (-rd).norm();

    // 判断这个交点属于哪个球（找最近的 SSS 球）
    int sphereIdx = -1;
    {
        double best = 1e18;
        for (int i = 0; i < (int)scene.spheres.size()-1; i++) {
            // 仅四个 SSS 球
            if (i >= 4) break;
            double d = (hit.pos - scene.spheres[i].center).length();
            if (d < best) { best = d; sphereIdx = i; }
        }
    }

    // 直接光照 (表面反射)
    Vec3 direct = directLighting(scene, hit.pos, hit.normal, viewDir, mat.albedo);

    // SSS 贡献 (只对4个 SSS 球做)
    Vec3 sss;
    if (sphereIdx >= 0 && sphereIdx < 4) {
        sss = computeSSS(scene, scene.spheres[sphereIdx], hit.pos, hit.normal, mat, sssSamples);
        // SSS 乘以材质颜色
        sss = sss * mat.albedo;
    }

    // 混合: 直接光照 + SSS 透射（SSS 比例根据材质调整）
    double sssWeight = 0.6;
    return direct * (1.0 - sssWeight) + sss * sssWeight + Vec3(0.01, 0.01, 0.01); // 环境光
}

// ─────────────────────────────────────────────
// 主程序
// ─────────────────────────────────────────────
int main() {
    const int W = 1024, H = 512;
    const int AA_SAMPLES = 4;     // 抗锯齿采样数
    const int SSS_SAMPLES = 64;   // SSS 表面积分采样数

    Image img(W, H);

    // 相机
    Vec3 camPos(0, 1.5, 9);
    Vec3 camTarget(0, 0, 0);
    Vec3 camUp(0, 1, 0);
    Vec3 camDir = (camTarget - camPos).norm();
    Vec3 camRight = camDir.cross(camUp).norm();
    Vec3 camUpOrtho = camRight.cross(camDir);
    double fovY = M_PI / 4.0;
    double aspectRatio = (double)W / H;
    double halfH = std::tan(fovY / 2.0);
    double halfW = halfH * aspectRatio;

    Scene scene;

    printf("渲染中... %d x %d, AA=%d, SSS samples=%d\n", W, H, AA_SAMPLES, SSS_SAMPLES);
    printf("场景: 皮肤 | 大理石 | 蜡烛 | 玉石\n");

    for (int y = 0; y < H; y++) {
        if (y % 64 == 0) {
            printf("进度: %d/%d (%.0f%%)\n", y, H, 100.0*y/H);
            fflush(stdout);
        }
        for (int x = 0; x < W; x++) {
            Vec3 color;
            for (int s = 0; s < AA_SAMPLES; s++) {
                double u = (x + randF()) / W * 2.0 - 1.0;
                double v = 1.0 - (y + randF()) / H * 2.0;
                Vec3 rd = (camDir + camRight * (u * halfW) + camUpOrtho * (v * halfH)).norm();
                color += renderPixel(scene, camPos, rd, SSS_SAMPLES);
            }
            img.at(x, y) = color / (double)AA_SAMPLES;
        }
    }

    // 保存主图
    img.savePPM("sss_output.ppm");
    printf("✅ 保存: sss_output.ppm\n");

    // ─── 生成漫射剖面曲线图 ───
    // 绘制 R_d(r) 随距离 r 的变化（可视化不同材质的散射半径）
    const int CW = 800, CH = 400;
    Image curve(CW, CH);

    // 填充背景
    for (int cy = 0; cy < CH; cy++)
        for (int cx = 0; cx < CW; cx++)
            curve.at(cx, cy) = Vec3(0.05, 0.05, 0.08);

    // 对每种材质画曲线
    // Material names: Skin, Marble, Wax, Jade
    SSSMaterial mats[4] = {Materials::skin(), Materials::marble(), Materials::wax(), Materials::jade()};
    Vec3 lineColors[4] = {
        {1.0, 0.6, 0.4},  // 皮肤 - 橙红
        {0.7, 0.7, 1.0},  // 大理石 - 蓝紫
        {1.0, 0.95, 0.5}, // 蜡烛 - 黄
        {0.4, 0.9, 0.5}   // 玉石 - 绿
    };

    double rMax = 15.0; // 最大 r 距离 (mm)
    double RdMax = 0.0;

    // 先找最大值用于归一化
    for (int m = 0; m < 4; m++) {
        for (int i = 0; i < CW; i++) {
            double r = (i + 0.5) / CW * rMax;
            Vec3 rd = mats[m].Rd(r);
            double val = (rd.x + rd.y + rd.z) / 3.0;
            RdMax = std::max(RdMax, val);
        }
    }

    // 绘制曲线
    for (int m = 0; m < 4; m++) {
        for (int cx = 1; cx < CW; cx++) {
            double r0 = ((cx-1) + 0.5) / CW * rMax;
            double r1 = (cx + 0.5) / CW * rMax;
            Vec3 rd0 = mats[m].Rd(r0);
            Vec3 rd1 = mats[m].Rd(r1);
            double v0 = (rd0.x + rd0.y + rd0.z) / 3.0 / RdMax;
            double v1 = (rd1.x + rd1.y + rd1.z) / 3.0 / RdMax;
            int y0 = (int)((1.0 - v0) * (CH - 40)) + 20;
            int y1 = (int)((1.0 - v1) * (CH - 40)) + 20;
            y0 = std::max(0, std::min(CH-1, y0));
            y1 = std::max(0, std::min(CH-1, y1));
            // 绘制线段（简单竖向填充）
            int ya = std::min(y0, y1), yb = std::max(y0, y1);
            for (int cy = ya; cy <= yb; cy++) {
                curve.at(cx, cy) = lineColors[m];
            }
        }
    }

    // 绘制坐标轴
    for (int cx = 0; cx < CW; cx++) curve.at(cx, CH-20) = Vec3(0.4, 0.4, 0.4);
    for (int cy = 0; cy < CH; cy++) curve.at(20, cy) = Vec3(0.4, 0.4, 0.4);

    curve.savePPM("sss_diffusion_profile.ppm");
    printf("✅ 保存: sss_diffusion_profile.ppm\n");

    // ─── 局部放大：皮肤球特写 ───
    const int PW = 512, PH = 512;
    Image portrait(PW, PH);
    Vec3 pCamPos(0-4.5, 0.5, 5.5);
    Vec3 pCamTarget(-4.5, 0, 0);
    Vec3 pCamDir = (pCamTarget - pCamPos).norm();
    Vec3 pCamRight = pCamDir.cross(Vec3(0,1,0)).norm();
    Vec3 pCamUpOrtho = pCamRight.cross(pCamDir);
    double pHalfH = std::tan(M_PI / 6.0);
    double pHalfW = pHalfH;

    printf("渲染皮肤特写...\n");
    for (int py = 0; py < PH; py++) {
        for (int px = 0; px < PW; px++) {
            Vec3 color;
            for (int s = 0; s < AA_SAMPLES; s++) {
                double u = (px + randF()) / PW * 2.0 - 1.0;
                double v = 1.0 - (py + randF()) / PH * 2.0;
                Vec3 rd = (pCamDir + pCamRight*(u*pHalfW) + pCamUpOrtho*(v*pHalfH)).norm();
                color += renderPixel(scene, pCamPos, rd, SSS_SAMPLES);
            }
            portrait.at(px, py) = color / (double)AA_SAMPLES;
        }
    }
    portrait.savePPM("sss_skin_closeup.ppm");
    printf("✅ 保存: sss_skin_closeup.ppm\n");

    printf("\n🎉 所有图像渲染完成！\n");
    printf("输出文件:\n");
    printf("  sss_output.ppm            - 四材质主渲染图\n");
    printf("  sss_diffusion_profile.ppm - 漫射剖面曲线\n");
    printf("  sss_skin_closeup.ppm      - 皮肤球特写\n");
    return 0;
}
