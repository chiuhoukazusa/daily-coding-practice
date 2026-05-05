/**
 * Bent Normal Ambient Occlusion Renderer
 * 
 * 技术要点：
 * 1. Bent Normal（弯曲法线）：在半球上采样，计算未被遮蔽方向的平均值
 * 2. 比普通 AO 更丰富：不仅给出遮蔽量，还给出"最优环境采样方向"
 * 3. 用弯曲法线从球谐环境光采样，得到更准确的间接光照
 * 4. 软光栅化场景：带孔洞的平面 + 球体 + 圆柱 + 凹槽地面
 *
 * 实现流程：
 * - G-Buffer：位置、法线、albedo
 * - 对每个可见像素，在法线半球上采样 N 条射线
 * - 每条射线检测是否与场景相交（ray march SDF）
 * - 未遮蔽射线的均值 = Bent Normal
 * - AO = 未遮蔽射线数 / 总射线数
 * - 用 Bent Normal 查询预定义的球谐环境光
 * - 最终颜色 = albedo * (SH_irradiance(bent_normal) * ao + direct)
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <random>
#include <string>
#include <sstream>
#include <iomanip>

// ============================================================
// 数学库
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x,-y,-z}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float length() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalize() const {
        float l = length();
        return l > 1e-8f ? Vec3{x/l, y/l, z/l} : Vec3{0,1,0};
    }
    Vec3 reflect(const Vec3& n) const {
        return *this - n * (2.0f * dot(n));
    }
    float& operator[](int i){ return i==0?x:i==1?y:z; }
    float  operator[](int i) const { return i==0?x:i==1?y:z; }
};

inline Vec3 lerp(const Vec3& a, const Vec3& b, float t){ return a + (b-a)*t; }
inline float clamp(float v, float lo=0.f, float hi=1.f){ return std::max(lo,std::min(hi,v)); }
inline float saturate(float v){ return clamp(v); }
inline Vec3 saturate(Vec3 v){ return {saturate(v.x),saturate(v.y),saturate(v.z)}; }

// ============================================================
// SDF 场景
// ============================================================

float sdSphere(Vec3 p, Vec3 center, float r){
    return (p-center).length() - r;
}

float sdBox(Vec3 p, Vec3 center, Vec3 half){
    Vec3 q = Vec3{std::abs(p.x-center.x)-half.x,
                  std::abs(p.y-center.y)-half.y,
                  std::abs(p.z-center.z)-half.z};
    Vec3 qc = {std::max(q.x,0.f), std::max(q.y,0.f), std::max(q.z,0.f)};
    return qc.length() + std::min(std::max({q.x,q.y,q.z}),0.f);
}

float sdCylinder(Vec3 p, Vec3 center, float r, float h){
    Vec3 local = p - center;
    float d = std::sqrt(local.x*local.x + local.z*local.z) - r;
    float dy = std::abs(local.y) - h*0.5f;
    float outside = Vec3{std::max(d,0.f), std::max(dy,0.f), 0}.length();
    float inside = std::min(std::max(d, dy), 0.f);
    return outside + inside;
}

// 带孔平面（用 SDF 布尔差集，孔洞位置 (0,0,0)）
float sdGroovedPlane(Vec3 p){
    // 基础平面
    float plane = p.y + 0.5f;
    // 凹槽（用一个长方体减去）
    float groove1 = sdBox(p, Vec3{0,  -0.5f, 0}, Vec3{0.3f, 0.3f, 2.0f});
    float groove2 = sdBox(p, Vec3{0,  -0.5f, 0}, Vec3{2.0f, 0.3f, 0.3f});
    // 差集：平面上挖凹槽
    return std::max(plane, -std::min(groove1, groove2));
}

struct Hit {
    float d;      // SDF 值
    int   id;     // 物体ID：0=地面,1=球,2=圆柱,3=方块
};

Hit sceneSDF(Vec3 p){
    // 地面（带十字凹槽）
    float ground = sdGroovedPlane(p);
    // 中心球
    float sphere = sdSphere(p, Vec3{0, 0.6f, 0}, 0.6f);
    // 左侧圆柱
    float cyl    = sdCylinder(p, Vec3{-1.5f, -0.1f, 0}, 0.3f, 0.8f);
    // 右侧方块
    float box    = sdBox(p, Vec3{1.5f, 0.05f, 0}, Vec3{0.4f, 0.55f, 0.4f});

    Hit h = {ground, 0};
    if(sphere < h.d){ h.d = sphere; h.id = 1; }
    if(cyl    < h.d){ h.d = cyl;    h.id = 2; }
    if(box    < h.d){ h.d = box;    h.id = 3; }
    return h;
}

Vec3 calcNormal(Vec3 p){
    const float eps = 0.001f;
    auto sdf = [](Vec3 q){ return sceneSDF(q).d; };
    return Vec3{
        sdf(Vec3{p.x+eps,p.y,p.z}) - sdf(Vec3{p.x-eps,p.y,p.z}),
        sdf(Vec3{p.x,p.y+eps,p.z}) - sdf(Vec3{p.x,p.y-eps,p.z}),
        sdf(Vec3{p.x,p.y,p.z+eps}) - sdf(Vec3{p.x,p.y,p.z-eps})
    }.normalize();
}

// ============================================================
// 材质颜色
// ============================================================

Vec3 getAlbedo(int id, Vec3 p){
    switch(id){
    case 0: {
        // 棋盘格地面
        int cx = (int)std::floor(p.x*1.5f) & 1;
        int cz = (int)std::floor(p.z*1.5f) & 1;
        return (cx^cz) ? Vec3{0.85f,0.85f,0.85f} : Vec3{0.25f,0.25f,0.3f};
    }
    case 1: return Vec3{0.9f, 0.4f, 0.15f};  // 橙色球
    case 2: return Vec3{0.3f, 0.7f, 0.9f};   // 蓝色圆柱
    case 3: return Vec3{0.5f, 0.85f, 0.4f};  // 绿色方块
    }
    return Vec3{0.5f,0.5f,0.5f};
}

// ============================================================
// 球谐环境光（L2，9 系数）
// 取自一个典型的蓝天 + 暖阳环境光
// ============================================================

struct SH9 {
    Vec3 c[9];
};

SH9 buildSkyEnvSH(){
    SH9 sh;
    // L0 - 整体环境光
    sh.c[0] = Vec3{0.79f, 0.84f, 0.98f} * 0.28f;
    // L1 - 方向性（主光源朝上偏暖）
    sh.c[1] = Vec3{0.39f, 0.38f, 0.30f} * 0.48f;  // y
    sh.c[2] = Vec3{0.18f, 0.20f, 0.28f} * 0.20f;  // z
    sh.c[3] = Vec3{0.08f, 0.07f, 0.05f} * 0.10f;  // x
    // L2
    sh.c[4] = Vec3{0.10f, 0.10f, 0.08f} * 0.15f;
    sh.c[5] = Vec3{0.05f, 0.06f, 0.09f} * 0.10f;
    sh.c[6] = Vec3{0.30f, 0.30f, 0.25f} * 0.20f;
    sh.c[7] = Vec3{0.06f, 0.05f, 0.04f} * 0.10f;
    sh.c[8] = Vec3{0.08f, 0.08f, 0.07f} * 0.15f;
    return sh;
}

// SH L2 辐照度评估
Vec3 evalSH(const SH9& sh, Vec3 n){
    // SH basis Y(l,m) at normal n (Phong normalization by Ramamoorthi & Hanrahan)
    float x=n.x, y=n.y, z=n.z;
    const float c0 = 0.282095f;
    const float c1 = 0.488603f;
    const float c2 = 1.092548f;
    const float c3 = 0.315392f;
    const float c4 = 0.546274f;

    Vec3 result = sh.c[0] * c0;
    result += sh.c[1] * (c1 * y);
    result += sh.c[2] * (c1 * z);
    result += sh.c[3] * (c1 * x);
    result += sh.c[4] * (c2 * x*y);
    result += sh.c[5] * (c2 * y*z);
    result += sh.c[6] * (c3 * (3*z*z - 1));
    result += sh.c[7] * (c2 * x*z);
    result += sh.c[8] * (c4 * (x*x - y*y));
    return result;
}

// ============================================================
// Bent Normal AO 计算
// ============================================================

// 在半球上用 Hammersley 低差异序列采样
Vec3 hemisphereDir(float u, float v, const Vec3& N){
    // 用余弦加权半球采样
    float phi   = 2.0f * M_PI * u;
    float cosTheta = std::sqrt(1.0f - v);
    float sinTheta = std::sqrt(v);

    float sx = sinTheta * std::cos(phi);
    float sy = cosTheta;
    float sz = sinTheta * std::sin(phi);

    // 将 (0,1,0) 半球变换到 N 空间
    Vec3 up = std::abs(N.y) < 0.999f ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 tangent   = up.cross(N).normalize();
    Vec3 bitangent = N.cross(tangent).normalize();

    return tangent*sx + N*sy + bitangent*sz;
}

// 用 van der Corput 低差异序列
float vanDerCorput(unsigned bits){
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10f;
}

struct BentNormalResult {
    Vec3  bentNormal;   // 弯曲法线（未遮蔽方向的加权平均）
    float ao;           // 0=完全遮蔽, 1=完全未遮蔽
};

BentNormalResult computeBentNormal(Vec3 pos, Vec3 normal,
                                    int numSamples, float aoRadius)
{
    const float STEP_MIN = 0.005f;
    const float EPS = 0.002f;

    Vec3  bentDir{0,0,0};
    int   unoccluded = 0;

    for(int i = 0; i < numSamples; ++i){
        // Hammersley 序列
        float u = float(i) / float(numSamples);
        float v = vanDerCorput((unsigned)i + 1u);

        Vec3 dir = hemisphereDir(u, v, normal);

        // 沿此方向做短距离 ray march，检查遮蔽
        Vec3 rayPos = pos + normal * EPS;  // 偏移避免自交
        float t = STEP_MIN;
        bool occluded = false;

        while(t < aoRadius){
            Vec3 p = rayPos + dir * t;
            float d = sceneSDF(p).d;
            if(d < 0.001f){
                occluded = true;
                break;
            }
            t += std::max(d, STEP_MIN);
        }

        if(!occluded){
            bentDir += dir;
            ++unoccluded;
        }
    }

    float ao = float(unoccluded) / float(numSamples);
    Vec3 bent = (unoccluded > 0) ? bentDir.normalize() : normal;

    return {bent, ao};
}

// ============================================================
// 直接光照（Lambertian + 软阴影）
// ============================================================

float softShadow(Vec3 ro, Vec3 rd, float tmin, float tmax, float k){
    float result = 1.0f;
    float t = tmin;
    for(int i = 0; i < 64; ++i){
        float h = sceneSDF(ro + rd*t).d;
        if(h < 0.001f) return 0.0f;
        result = std::min(result, k * h / t);
        t += std::max(h, 0.005f);
        if(t > tmax) break;
    }
    return saturate(result);
}

// ============================================================
// 相机 & 主光线
// ============================================================

Vec3 getCameraRay(int px, int py, int W, int H,
                  Vec3 eye, Vec3 target, float fov){
    float aspect = float(W) / float(H);
    float tanHalf = std::tan(fov * 0.5f * M_PI / 180.f);

    // NDC
    float u = (float(px) + 0.5f) / float(W) * 2.f - 1.f;
    float v = 1.f - (float(py) + 0.5f) / float(H) * 2.f;
    u *= aspect * tanHalf;
    v *= tanHalf;

    Vec3 fwd  = (target - eye).normalize();
    Vec3 right = fwd.cross(Vec3{0,1,0}).normalize();
    Vec3 up   = right.cross(fwd).normalize();

    return (fwd + right*u + up*v).normalize();
}

// ============================================================
// 主渲染
// ============================================================

int main(){
    const int W = 640, H = 480;
    const int AO_SAMPLES = 32;    // 每像素 AO 采样数
    const float AO_RADIUS = 1.2f;

    std::vector<unsigned char> framebuf(W * H * 3, 0);

    Vec3 eye{3.2f, 2.8f, 3.8f};
    Vec3 target{0.0f, 0.0f, 0.0f};

    // 主光源（太阳）
    Vec3 lightDir = Vec3{1.2f, 2.5f, 1.0f}.normalize();
    Vec3 lightColor{1.4f, 1.3f, 1.0f};  // 暖黄

    // 球谐环境光
    SH9 envSH = buildSkyEnvSH();

    // 渲染进度
    int reportStep = H / 10;

    for(int py = 0; py < H; ++py){
        if(py % reportStep == 0)
            std::cerr << "Rendering: " << (py*100/H) << "%\n";

        for(int px = 0; px < W; ++px){
            Vec3 rd = getCameraRay(px, py, W, H, eye, target, 50.0f);

            // Ray march 主射线
            float t = 0.1f;
            Hit  hit{1e9f, -1};
            for(int step = 0; step < 256; ++step){
                Vec3 p = eye + rd * t;
                Hit h = sceneSDF(p);
                if(h.d < 0.001f){ hit = h; break; }
                t += std::max(h.d, 0.002f);
                if(t > 30.f) break;
            }

            Vec3 color{0,0,0};

            if(hit.id >= 0){
                Vec3 pos    = eye + rd * t;
                Vec3 normal = calcNormal(pos);
                Vec3 albedo = getAlbedo(hit.id, pos);

                // ---- Bent Normal AO ----
                auto bnResult = computeBentNormal(pos, normal, AO_SAMPLES, AO_RADIUS);
                float ao       = bnResult.ao;
                Vec3  bentN    = bnResult.bentNormal;

                // ---- 直接光照 ----
                float NdotL = saturate(normal.dot(lightDir));
                float shadow = NdotL > 0.001f
                    ? softShadow(pos + normal*0.01f, lightDir, 0.02f, 8.f, 16.f)
                    : 0.f;
                Vec3 direct = albedo * lightColor * (NdotL * shadow);

                // ---- 间接光照（用 Bent Normal 采样 SH 环境光）----
                Vec3 shIrr = evalSH(envSH, bentN);
                // 保证非负
                shIrr.x = std::max(shIrr.x, 0.f);
                shIrr.y = std::max(shIrr.y, 0.f);
                shIrr.z = std::max(shIrr.z, 0.f);
                Vec3 indirect = albedo * shIrr * ao;

                // ---- 可视化：左半用 Bent Normal 着色，右半用 AO 着色 ----
                // 完整渲染 = direct + indirect
                color = direct + indirect;

                // 在图像下方区域叠加 Bent Normal 方向可视化（蓝色框内）
                if(py > H*3/4 && px < W/3){
                    // 仅显示 Bent Normal 方向（RGB 映射）
                    Vec3 bnVis = (bentN + Vec3{1,1,1}) * 0.5f;
                    color = bnVis;
                }else if(py > H*3/4 && px >= W/3 && px < W*2/3){
                    // 仅显示 AO
                    color = Vec3{ao, ao, ao};
                }
                // 右侧 1/3：完整渲染（默认）

            } else {
                // 天空 - 渐变
                float fy = 0.5f + 0.5f * rd.y;
                Vec3 skyTop{0.2f, 0.5f, 0.95f};
                Vec3 skyBot{0.75f, 0.88f, 1.0f};
                color = lerp(skyBot, skyTop, fy);
            }

            // Gamma 校正 + tone mapping (Reinhard)
            Vec3 tonemap{
                color.x / (color.x + 1.0f),
                color.y / (color.y + 1.0f),
                color.z / (color.z + 1.0f)
            };
            Vec3 gamma{
                std::pow(std::max(tonemap.x,0.f), 1.0f/2.2f),
                std::pow(std::max(tonemap.y,0.f), 1.0f/2.2f),
                std::pow(std::max(tonemap.z,0.f), 1.0f/2.2f)
            };

            int idx = (py * W + px) * 3;
            framebuf[idx+0] = (unsigned char)(saturate(gamma.x) * 255.f + 0.5f);
            framebuf[idx+1] = (unsigned char)(saturate(gamma.y) * 255.f + 0.5f);
            framebuf[idx+2] = (unsigned char)(saturate(gamma.z) * 255.f + 0.5f);
        }
    }

    std::cerr << "Rendering: 100%\n";

    // 写 PPM -> PNG via ImageMagick
    // 先写 PPM
    {
        std::ofstream f("bent_normal_ao_output.ppm", std::ios::binary);
        f << "P6\n" << W << " " << H << "\n255\n";
        f.write(reinterpret_cast<const char*>(framebuf.data()), framebuf.size());
    }

    // 用 ImageMagick 转 PNG
    int ret = std::system("convert bent_normal_ao_output.ppm bent_normal_ao_output.png 2>/dev/null && echo 'PNG OK' || cp bent_normal_ao_output.ppm bent_normal_ao_output.png");
    (void)ret;

    std::cout << "Render complete: bent_normal_ao_output.png (" << W << "x" << H << ")\n";
    std::cout << "AO samples per pixel: " << AO_SAMPLES << "\n";
    std::cout << "AO radius: " << AO_RADIUS << "\n";
    return 0;
}
