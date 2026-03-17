/**
 * LTC Area Light Rendering
 * 
 * 使用线性变换余弦（Linearly Transformed Cosines）方法
 * 实现矩形面光源的实时PBR渲染。
 * 
 * 核心技术：
 * - LTC (Linearly Transformed Cosines) by Heitz et al. 2016
 * - GGX BRDF 近似（通过预计算LTC矩阵）
 * - 矩形光源的解析积分（通过投影到球面多边形）
 * - PBR材质：metallic/roughness workflow
 * - 软光栅化渲染
 * 
 * 参考：
 * "Real-Time Polygonal-Light Shading with Linearly Transformed Cosines"
 * Heitz, Dupuy, Hill, Neubelt, SIGGRAPH 2016
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ============================================================
// Math utilities
// ============================================================
static const double PI = 3.14159265358979323846;
static const double TWO_PI = 2.0 * PI;
static const double INV_PI = 1.0 / PI;

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        double len = length();
        if (len < 1e-12) return {0, 0, 0};
        return *this / len;
    }
    double operator[](int i) const { return i==0 ? x : (i==1 ? y : z); }
    double& operator[](int i) { return i==0 ? x : (i==1 ? y : z); }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline Vec3 clamp(const Vec3& v, double lo, double hi) {
    return { std::max(lo, std::min(hi, v.x)),
             std::max(lo, std::min(hi, v.y)),
             std::max(lo, std::min(hi, v.z)) };
}
inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
inline double max3(double a, double b, double c) { return std::max(a, std::max(b,c)); }
inline Vec3 mix(const Vec3& a, const Vec3& b, double t) { return a*(1-t) + b*t; }
inline double mix(double a, double b, double t) { return a*(1-t) + b*t; }

// 3x3 矩阵
struct Mat3 {
    double m[3][3];
    Mat3() { memset(m, 0, sizeof(m)); }
    // 单位矩阵
    static Mat3 identity() {
        Mat3 r;
        r.m[0][0] = r.m[1][1] = r.m[2][2] = 1.0;
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }
    Mat3 inverse() const;
};

// 3x3 矩阵求逆
Mat3 Mat3::inverse() const {
    Mat3 inv;
    double det = m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])
               - m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])
               + m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]);
    if (std::abs(det) < 1e-12) return Mat3::identity();
    double invDet = 1.0 / det;
    inv.m[0][0] = (m[1][1]*m[2][2] - m[1][2]*m[2][1]) * invDet;
    inv.m[0][1] = (m[0][2]*m[2][1] - m[0][1]*m[2][2]) * invDet;
    inv.m[0][2] = (m[0][1]*m[1][2] - m[0][2]*m[1][1]) * invDet;
    inv.m[1][0] = (m[1][2]*m[2][0] - m[1][0]*m[2][2]) * invDet;
    inv.m[1][1] = (m[0][0]*m[2][2] - m[0][2]*m[2][0]) * invDet;
    inv.m[1][2] = (m[0][2]*m[1][0] - m[0][0]*m[1][2]) * invDet;
    inv.m[2][0] = (m[1][0]*m[2][1] - m[1][1]*m[2][0]) * invDet;
    inv.m[2][1] = (m[0][1]*m[2][0] - m[0][0]*m[2][1]) * invDet;
    inv.m[2][2] = (m[0][0]*m[1][1] - m[0][1]*m[1][0]) * invDet;
    return inv;
}

// ============================================================
// LTC (Linearly Transformed Cosines) Core
// ============================================================

/**
 * LTC 表查找参数
 * 真实实现使用 64x64 预计算表（roughness x cos(theta)）
 * 这里我们用解析近似：
 * - 对于不同 roughness 和视角，计算合适的 LTC 矩阵 M
 * 
 * LTC 将 GGX BRDF 映射到余弦分布：
 *   L_o = integral_Omega D_o(l) L(l) dl
 * 其中 D_o(l) = D_cos(M^-1 l) / (|M^-1 l|^3 * |det(M)|)
 */

// 近似GGX分布用LTC的M矩阵参数
// 基于 Heitz 2016 的解析近似
struct LTCParams {
    double m11, m22, m13; // 对称矩阵的主要参数
    double amplitude;      // BRDF振幅
};

// 根据 roughness 和 NoV 计算 LTC 参数
// 使用 Heitz 2016 论文中的拟合公式
LTCParams computeLTCParams(double roughness, double NoV) {
    // alpha = roughness^2 (GGX 粗糙度参数)
    double alpha = roughness * roughness;
    
    LTCParams p;
    double a = alpha;
    
    // m11, m22 控制椭圆形状（宽度）：
    //   roughness→0: m11→1 (lobe 极窄，但拟合退化到余弦，只靠 amplitude 放大)
    //   roughness→1: m11→1 (退化为 Lambertian，单位矩阵)
    // 注：LTC 的精确做法是查 64×64 LUT，这里用多项式近似
    // 中间 roughness（0.3~0.7）拟合最准，极端值有误差
    p.m11 = 1.0 + a * (0.5 + a * (-1.0 + a * 1.5));
    p.m22 = 1.0 + a * (0.5 + a * (-0.5));
    
    // m13 是偏斜项（只在 [0][2] 位置，不对称）：
    //   表示视角倾斜时 BRDF lobe 前倾（掠射角前向散射）
    //   roughness 越高、NoV 越小时偏斜越明显
    p.m13 = -a * (1.0 - NoV) * 0.5;
    
    // amplitude 控制 BRDF 的总能量缩放
    // 低 roughness 时 GGX 形成极窄的镜面峰，amplitude 补偿拟合误差
    p.amplitude = 1.0 - a * (0.5 - a * 0.3);
    
    // 低粗糙度特殊处理：GGX 趋向 delta 函数，
    // 此时 LTC 拟合误差大，用更高 amplitude + 更小矩阵补偿
    if (alpha < 0.25) {
        double t = alpha / 0.25;  // [0,1]
        // amplitude 从高到低平滑过渡（roughness=0时约4.0，roughness=0.5时约1.0）
        p.amplitude = mix(4.0, 1.0, t * t);
        // 同时压缩矩阵让 lobe 更窄
        p.m11 = mix(0.5, p.m11, t);
        p.m22 = mix(0.5, p.m22, t);
    }
    
    return p;
}

// 从 LTCParams 构造 M^-1 矩阵（逆LTC变换矩阵）
// LTC 变换：将余弦分布变换到 GGX 分布
// 我们直接存储 M^-1 用于积分计算
Mat3 buildLTCMatrix(const LTCParams& p) {
    // LTC 标准矩阵（Heitz 2016）：
    // M = [[m11, 0, m13],
    //      [0, m22,   0],
    //      [0,   0,   1]]
    // m13 只在 [0][2] 位置（非对称），表示前倾偏斜
    // 注：不是对称矩阵，[2][0] 应为 0
    Mat3 M;
    M.m[0][0] = p.m11;
    M.m[1][1] = p.m22;
    M.m[2][2] = 1.0;
    M.m[0][2] = p.m13;   // 正确：只设 [0][2]
    // M.m[2][0] = 0.0;  // 这里保持 0，不设对称项
    return M;
}

// ============================================================
// LTC 矩形光源积分（球面多边形方法）
// ============================================================

/**
 * 计算单位球上多边形的球面积分
 * 使用 solid angle 方法（Lambert's formula）
 * 
 * 输入：变换到 LTC 空间后的矩形光源顶点
 * 输出：LTC 域中的积分值（= 余弦加权立体角）
 */

// 将向量 clip 到半球（z > 0）
// 返回 clip 后的多边形顶点数
int clipPolygonToHorizon(Vec3* poly, int n) {
    Vec3 clipped[8];
    int nc = 0;
    for (int i = 0; i < n; i++) {
        Vec3 A = poly[i];
        Vec3 B = poly[(i + 1) % n];
        
        bool A_above = A.z > 0;
        bool B_above = B.z > 0;
        
        if (A_above) {
            clipped[nc++] = A;
        }
        if (A_above != B_above) {
            // 边与 z=0 相交
            double t = A.z / (A.z - B.z);
            clipped[nc++] = A + (B - A) * t;
        }
    }
    for (int i = 0; i < nc; i++) poly[i] = clipped[i];
    return nc;
}

// 计算球面多边形积分（solid angle）
// 使用 Van Oosterom & Strackee 1983 公式
double integrateEdge(const Vec3& v1, const Vec3& v2) {
    double cosTheta = clamp(v1.dot(v2), -1.0, 1.0);
    double theta = std::acos(cosTheta);
    // cross 的 z 分量
    double sinTheta = v1.x * v2.y - v1.y * v2.x;
    if (std::abs(sinTheta) < 1e-12) return 0.0;
    return theta * sinTheta / (std::sin(theta) + 1e-12);
}

/**
 * 计算矩形光源的 LTC 积分
 * 
 * @param Minv  M^-1 矩阵（LTC 逆变换）
 * @param points  矩形光源的4个顶点（世界空间，相对于着色点）
 * @param N  着色点法线
 * @param V  视向量
 * @return  LTC 积分值（辐照度）
 */
double ltcEvaluate(const Mat3& Minv, const Vec3 points[4], const Vec3& N, const Vec3& V) {
    // 构建 shading 坐标系
    // Z = N, X = normalize(V - (V.N)N), Y = Z x X
    Vec3 Z = N;
    Vec3 X = (V - N * N.dot(V)).normalized();
    if (X.length() < 1e-6) {
        // V 平行于 N，使用任意切向量
        Vec3 tmp = std::abs(N.x) < 0.9 ? Vec3(1,0,0) : Vec3(0,1,0);
        X = N.cross(tmp).normalized();
    }
    Vec3 Y = Z.cross(X);
    
    // 将光源顶点变换到 shading 坐标系，然后应用 M^-1
    Vec3 L[4];
    for (int i = 0; i < 4; i++) {
        // 变换到 shading 坐标系
        Vec3 p = points[i];
        Vec3 pLocal(p.dot(X), p.dot(Y), p.dot(Z));
        // 应用 LTC 逆变换
        L[i] = (Minv * pLocal).normalized();
    }
    
    // Clip 到上半球
    int n = 4;
    n = clipPolygonToHorizon(L, n);
    if (n < 3) return 0.0;
    
    // 计算球面多边形积分
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += integrateEdge(L[i].normalized(), L[(i+1)%n].normalized());
    }
    
    return std::abs(sum) * 0.5 * INV_PI;
}

// ============================================================
// 面光源直接照明（含 Diffuse + Specular LTC）
// ============================================================

struct Material {
    Vec3 albedo;        // 基础颜色
    double metallic;    // 金属度
    double roughness;   // 粗糙度
};

struct AreaLight {
    Vec3 position;      // 中心位置
    Vec3 right;         // 右方向（归一化）
    Vec3 up;            // 上方向（归一化）
    double halfWidth;   // 半宽
    double halfHeight;  // 半高
    Vec3 color;         // 光源颜色
    double intensity;   // 光源强度
    
    // 获取4个顶点（逆时针）
    void getCorners(Vec3 corners[4]) const {
        Vec3 r = right * halfWidth;
        Vec3 u = up * halfHeight;
        corners[0] = position - r - u;
        corners[1] = position + r - u;
        corners[2] = position + r + u;
        corners[3] = position - r + u;
    }
};

// 计算面光源对某点的辐照度
Vec3 evaluateAreaLight(
    const AreaLight& light,
    const Vec3& pos,      // 着色点
    const Vec3& normal,   // 着色点法线
    const Vec3& viewDir,  // 视向量（指向相机）
    const Material& mat
) {
    // 获取光源顶点（相对于着色点）
    Vec3 corners[4];
    light.getCorners(corners);
    Vec3 relCorners[4];
    for (int i = 0; i < 4; i++) relCorners[i] = corners[i] - pos;
    
    // 检查是否在光源正面
    // Vec3 lightNormal = light.right.cross(light.up).normalized(); // for back-face culling
    // Vec3 toLight = (light.position - pos).normalized(); // for back-face culling
    // if (toLight.dot(lightNormal) >= 0) return Vec3(0,0,0); // 背面
    
    double NoV = clamp(normal.dot(viewDir), 0.0, 1.0);
    
    // Fresnel（先算，后面两路都要用）
    Vec3 F0 = mix(Vec3(0.04, 0.04, 0.04), mat.albedo, mat.metallic);
    Vec3 fresnel = F0 + (Vec3(1,1,1) - F0) * std::pow(1.0 - NoV, 5.0);
    Vec3 kS = fresnel;
    Vec3 kD = (Vec3(1,1,1) - kS) * (1.0 - mat.metallic);

    // ---- Diffuse (Lambertian LTC = 单位矩阵) ----
    Mat3 Minv_diff = Mat3::identity();
    double diffuseIntegral = ltcEvaluate(Minv_diff, relCorners, normal, viewDir);
    Vec3 diffuse = kD * mat.albedo * INV_PI * diffuseIntegral;

    // ---- Specular ----
    Vec3 specular(0, 0, 0);
    const double MIRROR_THRESH = 0.08; // roughness < 此值退化为镜面点光源近似
    
    if (mat.roughness < MIRROR_THRESH) {
        // 低 roughness：LTC 拟合失真，改用最近点（Representative Point）近似
        // 找面光源上离反射方向最近的点，当作一个"最亮的点光"
        Vec3 rv = (-viewDir);
        Vec3 reflDir = rv - normal * (2.0 * rv.dot(normal)); // 手写 reflect
        // 在光源平面上找反射射线最近交点
        Vec3 corners[4]; light.getCorners(corners);
        Vec3 lightNorm = light.right.cross(light.up).normalized();
        double denom = reflDir.dot(lightNorm);
        Vec3 repPoint = light.position; // 默认用光源中心
        if (std::abs(denom) > 1e-6) {
            double t = (light.position - pos).dot(lightNorm) / denom;
            if (t > 0) {
                Vec3 hit = pos + reflDir * t;
                // clip 到光源范围内
                Vec3 local = hit - light.position;
                double u = clamp(local.dot(light.right), -light.halfWidth, light.halfWidth);
                double v = clamp(local.dot(light.up),    -light.halfHeight, light.halfHeight);
                repPoint = light.position + light.right * u + light.up * v;
            }
        }
        Vec3 toRep = repPoint - pos;
        double dist2 = toRep.dot(toRep);
        Vec3 L = toRep / std::sqrt(dist2 + 1e-8);
        double NoL = clamp(normal.dot(L), 0.0, 1.0);
        Vec3 H = (viewDir + L).normalized();
        double NoH = clamp(normal.dot(H), 0.0, 1.0);
        double alpha = mat.roughness * mat.roughness;
        // GGX NDF（简化，无分母归一化，只取形状）
        double denom2 = NoH * NoH * (alpha * alpha - 1.0) + 1.0;
        double D = alpha * alpha / (PI * denom2 * denom2 + 1e-8);
        // 混合因子：低roughness全用此路，向LTC平滑过渡
        double blend = 1.0 - mat.roughness / MIRROR_THRESH;
        blend = blend * blend; // 二次曲线，收敛更自然
        specular = kS * D * NoL * blend * (1.0 / (dist2 + 1.0));
    } else {
        // 普通 roughness：LTC 积分有效
        LTCParams params = computeLTCParams(mat.roughness, NoV);
        Mat3 M_spec = buildLTCMatrix(params);
        Mat3 Minv_spec = M_spec.inverse();
        double specularIntegral = ltcEvaluate(Minv_spec, relCorners, normal, viewDir) * params.amplitude;
        // 在 MIRROR_THRESH 处做平滑过渡，避免硬跳变
        double blendLTC = clamp((mat.roughness - MIRROR_THRESH) / (MIRROR_THRESH * 0.5), 0.0, 1.0);
        specular = kS * specularIntegral * blendLTC;
    }

    Vec3 Lo = (diffuse + specular) * light.color * light.intensity;
    return Lo;
}

// ============================================================
// 软光栅化渲染器
// ============================================================

struct Camera {
    Vec3 origin;
    Vec3 forward;   // 观察方向（normalized）
    Vec3 right;
    Vec3 up;
    double fov;     // 水平 FOV（弧度）
    int width, height;
    
    // 生成像素射线方向
    Vec3 getRayDir(int px, int py) const {
        double aspect = (double)width / height;
        double halfW = std::tan(fov * 0.5);
        double halfH = halfW / aspect;
        double u = (2.0 * (px + 0.5) / width - 1.0) * halfW;
        double v = (1.0 - 2.0 * (py + 0.5) / height) * halfH;
        return (forward + right * u + up * v).normalized();
    }
};

// G-Buffer
struct GBuffer {
    Vec3 position;
    Vec3 normal;
    Material material;
    bool valid;
    double depth;
};

// 场景几何：简单平面 + 几个球体
struct Sphere {
    Vec3 center;
    double radius;
    Material mat;
};

struct Plane {
    Vec3 normal;
    double d; // 平面方程 N.p = d
    Material mat;
    Plane() {}
    Plane(const Vec3& n, double dd, const Material& m) : normal(n), d(dd), mat(m) {}
};

// 射线-球相交
double intersectSphere(const Vec3& orig, const Vec3& dir, const Sphere& sphere) {
    Vec3 oc = orig - sphere.center;
    double a = dir.dot(dir);
    double b = 2.0 * oc.dot(dir);
    double c = oc.dot(oc) - sphere.radius * sphere.radius;
    double disc = b*b - 4*a*c;
    if (disc < 0) return -1.0;
    double t = (-b - std::sqrt(disc)) / (2*a);
    if (t < 0.001) t = (-b + std::sqrt(disc)) / (2*a);
    return t < 0.001 ? -1.0 : t;
}

// 射线-平面相交
double intersectPlane(const Vec3& orig, const Vec3& dir, const Plane& plane) {
    double denom = plane.normal.dot(dir);
    if (std::abs(denom) < 1e-6) return -1.0;
    double t = (plane.d - plane.normal.dot(orig)) / denom;
    return t < 0.001 ? -1.0 : t;
}

// 场景定义
struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;
    std::vector<AreaLight> lights;
    Vec3 ambientColor;
    double ambientIntensity;
};

// 射线与场景求交，填充 G-Buffer
GBuffer traceRay(const Vec3& orig, const Vec3& dir, const Scene& scene) {
    GBuffer gb;
    gb.valid = false;
    gb.depth = 1e30;
    
    // 检查球体
    for (const auto& sphere : scene.spheres) {
        double t = intersectSphere(orig, dir, sphere);
        if (t > 0 && t < gb.depth) {
            gb.depth = t;
            gb.position = orig + dir * t;
            gb.normal = (gb.position - sphere.center).normalized();
            gb.material = sphere.mat;
            gb.valid = true;
        }
    }
    
    // 检查平面
    for (const auto& plane : scene.planes) {
        double t = intersectPlane(orig, dir, plane);
        if (t > 0 && t < gb.depth) {
            gb.depth = t;
            gb.position = orig + dir * t;
            gb.normal = plane.normal;
            gb.material = plane.mat;
            gb.valid = true;
        }
    }
    
    return gb;
}

// ACES 色调映射
Vec3 acesTonemap(const Vec3& x) {
    const double a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    Vec3 num = x * (x * a + Vec3(b,b,b));
    Vec3 denom = x * (x * c + Vec3(d,d,d)) + Vec3(e,e,e);
    Vec3 result(num.x/denom.x, num.y/denom.y, num.z/denom.z);
    return clamp(result, 0.0, 1.0);
}

// 线性转 sRGB gamma
Vec3 linearToSRGB(const Vec3& c) {
    auto encode = [](double v) -> double {
        v = clamp(v, 0.0, 1.0);
        return v <= 0.0031308 ? 12.92 * v : 1.055 * std::pow(v, 1.0/2.2) - 0.055;
    };
    return { encode(c.x), encode(c.y), encode(c.z) };
}

// 渲染完整图像
void renderImage(
    const Scene& scene,
    const Camera& cam,
    std::vector<unsigned char>& pixels,
    int width, int height
) {
    pixels.resize(width * height * 3);
    
    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            Vec3 dir = cam.getRayDir(px, py);
            GBuffer gb = traceRay(cam.origin, dir, scene);
            
            Vec3 color(0, 0, 0);
            if (gb.valid) {
                Vec3 viewDir = -dir; // 指向相机
                
                // 环境光
                color = scene.ambientColor * scene.ambientIntensity * gb.material.albedo;
                
                // 面光源贡献
                for (const auto& light : scene.lights) {
                    Vec3 Lo = evaluateAreaLight(light, gb.position, gb.normal, viewDir, gb.material);
                    color += Lo;
                }
                
                // 色调映射
                color = acesTonemap(color);
            } else {
                // 背景渐变天空
                double t = 0.5 * (dir.y + 1.0);
                color = mix(Vec3(0.1, 0.1, 0.12), Vec3(0.05, 0.07, 0.15), t);
            }
            
            // Gamma correction
            Vec3 gamma = linearToSRGB(color);
            
            int idx = (py * width + px) * 3;
            pixels[idx+0] = (unsigned char)(gamma.x * 255.999);
            pixels[idx+1] = (unsigned char)(gamma.y * 255.999);
            pixels[idx+2] = (unsigned char)(gamma.z * 255.999);
        }
        if (py % 50 == 0) {
            printf("\rRendering... %d%%", (py * 100) / height);
            fflush(stdout);
        }
    }
    printf("\rRendering... 100%%\n");
}

// ============================================================
// 主程序：多场景演示
// ============================================================

int main() {
    const int W = 800, H = 600;
    
    printf("=== LTC Area Light Rendering ===\n");
    printf("Heitz et al. SIGGRAPH 2016\n");
    printf("Resolution: %dx%d\n\n", W, H);
    
    // ---- 场景设置 ----
    // 相机
    Camera cam;
    cam.origin = Vec3(0, 2.5, 6.0);
    cam.forward = Vec3(0, -0.3, -1.0).normalized();
    cam.right = Vec3(1, 0, 0);
    cam.up = cam.right.cross(cam.forward).normalized();
    cam.fov = 60.0 * PI / 180.0;
    cam.width = W;
    cam.height = H;
    
    // 面光源（矩形）
    AreaLight mainLight;
    mainLight.position = Vec3(0, 7.0, 0);    // 拉高到 y=7，增大距离减弱立体角差异
    mainLight.right = Vec3(1, 0, 0);
    mainLight.up = Vec3(0, 0, 1);
    mainLight.halfWidth = 1.2;               // 稍微缩小
    mainLight.halfHeight = 0.8;
    mainLight.color = Vec3(1.0, 0.95, 0.8);
    mainLight.intensity = 18.0;              // 拉高补偿距离平方

    // 辅助光源（蓝色，侧面）
    AreaLight fillLight;
    fillLight.position = Vec3(-7, 4.0, 1.0); // 同样拉远
    fillLight.right = Vec3(0, 0, 1);
    fillLight.up = Vec3(0, 1, 0);
    fillLight.halfWidth = 0.8;
    fillLight.halfHeight = 1.0;
    fillLight.color = Vec3(0.4, 0.6, 1.0);
    fillLight.intensity = 8.0;
    
    // 地面平面
    Plane ground;
    ground.normal = Vec3(0, 1, 0);
    ground.d = 0.0;
    ground.mat.albedo = Vec3(0.5, 0.5, 0.5);
    ground.mat.metallic = 0.0;
    ground.mat.roughness = 0.8;
    
    // 背景平面
    Plane backWall;
    backWall.normal = Vec3(0, 0, 1);
    backWall.d = -3.0;
    backWall.mat.albedo = Vec3(0.6, 0.6, 0.65);
    backWall.mat.metallic = 0.0;
    backWall.mat.roughness = 0.9;
    
    // ============================================================
    // Scene 1: Roughness 对比（从光滑到粗糙）
    // ============================================================
    {
        Scene scene;
        scene.lights.push_back(mainLight);
        scene.lights.push_back(fillLight);
        scene.planes.push_back(ground);
        scene.planes.push_back(backWall);
        scene.ambientColor = Vec3(0.2, 0.2, 0.25);
        scene.ambientIntensity = 0.15;
        
        // 5个球体，roughness 从 0.1 到 0.9
        double roughnesses[] = {0.1, 0.3, 0.5, 0.7, 0.9};
        double posX[] = {-4.0, -2.0, 0.0, 2.0, 4.0};
        for (int i = 0; i < 5; i++) {
            Sphere s;
            s.center = Vec3(posX[i], 1.0, 0.0);
            s.radius = 0.85;
            s.mat.albedo = Vec3(0.8, 0.6, 0.3); // 金色
            s.mat.metallic = 1.0;
            s.mat.roughness = roughnesses[i];
            scene.spheres.push_back(s);
        }
        
        std::vector<unsigned char> pixels;
        printf("Rendering Scene 1: Roughness Comparison (metallic gold)...\n");
        renderImage(scene, cam, pixels, W, H);
        stbi_write_png("ltc_roughness.png", W, H, 3, pixels.data(), W*3);
        printf("Saved: ltc_roughness.png\n\n");
    }
    
    // ============================================================
    // Scene 2: Metallic 对比（金属 vs 非金属）
    // ============================================================
    {
        Scene scene;
        scene.lights.push_back(mainLight);
        scene.lights.push_back(fillLight);
        scene.planes.push_back(ground);
        scene.planes.push_back(backWall);
        scene.ambientColor = Vec3(0.2, 0.2, 0.25);
        scene.ambientIntensity = 0.15;
        
        // 5个球体：metallic 从 0.0 到 1.0
        double metallics[] = {0.0, 0.25, 0.5, 0.75, 1.0};
        double posX[] = {-4.0, -2.0, 0.0, 2.0, 4.0};
        for (int i = 0; i < 5; i++) {
            Sphere s;
            s.center = Vec3(posX[i], 1.0, 0.0);
            s.radius = 0.85;
            s.mat.albedo = Vec3(0.8, 0.2, 0.2); // 红色
            s.mat.metallic = metallics[i];
            s.mat.roughness = 0.3;
            scene.spheres.push_back(s);
        }
        
        std::vector<unsigned char> pixels;
        printf("Rendering Scene 2: Metallic Comparison...\n");
        renderImage(scene, cam, pixels, W, H);
        stbi_write_png("ltc_metallic.png", W, H, 3, pixels.data(), W*3);
        printf("Saved: ltc_metallic.png\n\n");
    }
    
    // ============================================================
    // Scene 3: 多材质混合场景（主输出图）
    // ============================================================
    {
        Scene scene;
        scene.lights.push_back(mainLight);
        scene.lights.push_back(fillLight);
        scene.planes.push_back(ground);
        scene.planes.push_back(backWall);
        scene.ambientColor = Vec3(0.1, 0.12, 0.15);
        scene.ambientIntensity = 0.2;
        
        // 中心铬金属球
        {
            Sphere s;
            s.center = Vec3(0.0, 1.0, 0.0);
            s.radius = 1.0;
            s.mat.albedo = Vec3(0.9, 0.9, 0.9);
            s.mat.metallic = 1.0;
            s.mat.roughness = 0.05;
            scene.spheres.push_back(s);
        }
        // 左边金球
        {
            Sphere s;
            s.center = Vec3(-2.5, 0.85, 0.5);
            s.radius = 0.8;
            s.mat.albedo = Vec3(1.0, 0.76, 0.33);
            s.mat.metallic = 1.0;
            s.mat.roughness = 0.2;
            scene.spheres.push_back(s);
        }
        // 右边铜球
        {
            Sphere s;
            s.center = Vec3(2.5, 0.85, 0.5);
            s.radius = 0.8;
            s.mat.albedo = Vec3(0.95, 0.64, 0.54);
            s.mat.metallic = 1.0;
            s.mat.roughness = 0.4;
            scene.spheres.push_back(s);
        }
        // 左前蓝色电介质
        {
            Sphere s;
            s.center = Vec3(-1.5, 0.6, 2.0);
            s.radius = 0.6;
            s.mat.albedo = Vec3(0.2, 0.4, 0.9);
            s.mat.metallic = 0.0;
            s.mat.roughness = 0.6;
            scene.spheres.push_back(s);
        }
        // 右前白色电介质
        {
            Sphere s;
            s.center = Vec3(1.5, 0.6, 2.0);
            s.radius = 0.6;
            s.mat.albedo = Vec3(0.9, 0.9, 0.9);
            s.mat.metallic = 0.0;
            s.mat.roughness = 0.8;
            scene.spheres.push_back(s);
        }
        
        std::vector<unsigned char> pixels;
        printf("Rendering Scene 3: Multi-Material (main output)...\n");
        renderImage(scene, cam, pixels, W, H);
        stbi_write_png("ltc_output.png", W, H, 3, pixels.data(), W*3);
        printf("Saved: ltc_output.png\n\n");
    }
    
    // ============================================================
    // Scene 4: 对比图（无面光 vs 有面光 LTC）
    // ============================================================
    {
        // 无面光版本（只有环境光）
        Scene noLight;
        { Material gm; gm.albedo=Vec3(0.5,0.5,0.5); gm.metallic=0.0; gm.roughness=0.8; noLight.planes.push_back(Plane(Vec3(0,1,0), 0.0, gm)); }
        { Material bm; bm.albedo=Vec3(0.6,0.6,0.65); bm.metallic=0.0; bm.roughness=0.9; noLight.planes.push_back(Plane(Vec3(0,0,1), -3.0, bm)); }
        noLight.ambientColor = Vec3(0.4, 0.42, 0.5);
        noLight.ambientIntensity = 0.5;
        for (int i = 0; i < 5; i++) {
            Sphere s;
            s.center = Vec3(-4.0 + i * 2.0, 1.0, 0.0);
            s.radius = 0.85;
            s.mat.albedo = Vec3(0.8, 0.6, 0.3);
            s.mat.metallic = 1.0;
            s.mat.roughness = 0.1 + i * 0.2;
            noLight.spheres.push_back(s);
        }
        
        std::vector<unsigned char> pixNoLight;
        printf("Rendering Scene 4a: Ambient Only...\n");
        renderImage(noLight, cam, pixNoLight, W, H);
        
        // 有面光版本
        Scene withLight;
        withLight.lights.push_back(mainLight);
        withLight.lights.push_back(fillLight);
        { Material gm; gm.albedo=Vec3(0.5,0.5,0.5); gm.metallic=0.0; gm.roughness=0.8; withLight.planes.push_back(Plane(Vec3(0,1,0), 0.0, gm)); }
        { Material bm; bm.albedo=Vec3(0.6,0.6,0.65); bm.metallic=0.0; bm.roughness=0.9; withLight.planes.push_back(Plane(Vec3(0,0,1), -3.0, bm)); }
        withLight.ambientColor = Vec3(0.2, 0.2, 0.25);
        withLight.ambientIntensity = 0.1;
        for (const auto& s : noLight.spheres) withLight.spheres.push_back(s);
        
        std::vector<unsigned char> pixWithLight;
        printf("Rendering Scene 4b: LTC Area Light...\n");
        renderImage(withLight, cam, pixWithLight, W, H);
        
        // 生成对比图（左右拼接）
        std::vector<unsigned char> comparison(W * 2 * H * 3);
        for (int py = 0; py < H; py++) {
            for (int px = 0; px < W; px++) {
                int srcIdx = (py * W + px) * 3;
                // 左半部分
                int dstIdx = (py * W * 2 + px) * 3;
                comparison[dstIdx+0] = pixNoLight[srcIdx+0];
                comparison[dstIdx+1] = pixNoLight[srcIdx+1];
                comparison[dstIdx+2] = pixNoLight[srcIdx+2];
                // 右半部分
                dstIdx = (py * W * 2 + W + px) * 3;
                comparison[dstIdx+0] = pixWithLight[srcIdx+0];
                comparison[dstIdx+1] = pixWithLight[srcIdx+1];
                comparison[dstIdx+2] = pixWithLight[srcIdx+2];
            }
        }
        // 分隔线（白色）
        for (int py = 0; py < H; py++) {
            int dstIdx = (py * W * 2 + W - 1) * 3;
            comparison[dstIdx+0] = comparison[dstIdx+1] = comparison[dstIdx+2] = 255;
            dstIdx = (py * W * 2 + W) * 3;
            comparison[dstIdx+0] = comparison[dstIdx+1] = comparison[dstIdx+2] = 255;
        }
        stbi_write_png("ltc_comparison.png", W * 2, H, 3, comparison.data(), W * 2 * 3);
        printf("Saved: ltc_comparison.png\n\n");
    }
    
    printf("=== All renders complete! ===\n");
    printf("Output files:\n");
    printf("  ltc_roughness.png   - Roughness comparison (metallic gold spheres)\n");
    printf("  ltc_metallic.png    - Metallic comparison (dielectric to metal)\n");
    printf("  ltc_output.png      - Multi-material scene (main output)\n");
    printf("  ltc_comparison.png  - Comparison: ambient only vs LTC area light\n");
    
    return 0;
}
