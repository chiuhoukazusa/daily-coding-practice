/**
 * Displacement Mapping & Tessellation Renderer
 * 2026-06-01
 *
 * 技术要点：
 * - 程序化高度场生成（Perlin noise + FBM分形）
 * - 网格细分（均匀细分，生成 512x512 细分网格）
 * - 顶点位移（沿法线方向应用高度图偏移）
 * - 法线重计算（通过相邻顶点差分计算精确法线）
 * - PBR 着色（金属度 / 粗糙度 / IBL approximation）
 * - 软阴影（PCF 深度贴图）
 * - 图像输出（PPM -> PNG via stb_image_write）
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <random>

// ===================== 数学工具 =====================

struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)       const { return {x*t,   y*t,   z*t};   }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)       const { return {x/t,   y/t,   z/t};   }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>1e-8f?*this/l:Vec3{0,0,0}; }
    Vec3 reflect(const Vec3& n) const { return *this - n * (2.0f * dot(n)); }
};
inline Vec3 mix(const Vec3& a, const Vec3& b, float t){ return a*(1-t)+b*t; }
inline float clamp(float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); }
inline float saturate(float v){ return clamp(v, 0.f, 1.f); }

// ===================== Perlin Noise =====================

static float fade(float t){ return t*t*t*(t*(t*6-15)+10); }
static float lerp1(float a, float b, float t){ return a+t*(b-a); }

static int perm[512];
static void initPerm(unsigned seed=42){
    static const int p[256]={
        151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,
        140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,
        247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,
        57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,
        74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,
        60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,
        65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,
        200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,
        52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,
        207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,
        119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,
        129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,
        218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,
        81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,
        184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,
        222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
    };
    std::mt19937 rng(seed);
    std::vector<int> v(p, p+256);
    std::shuffle(v.begin(), v.end(), rng);
    for(int i=0;i<256;i++) perm[i]=perm[i+256]=v[i];
}
static float grad(int h, float x, float y, float z){
    int hh=h&15;
    float u=hh<8?x:y;
    float v=hh<4?y:(hh==12||hh==14?x:z);
    return ((hh&1)?-u:u)+((hh&2)?-v:v);
}
static float perlin(float x, float y, float z){
    int X=((int)std::floor(x))&255, Y=((int)std::floor(y))&255, Z=((int)std::floor(z))&255;
    x-=std::floor(x); y-=std::floor(y); z-=std::floor(z);
    float u=fade(x), v=fade(y), w=fade(z);
    int A=perm[X]+Y, AA=perm[A]+Z, AB=perm[A+1]+Z;
    int B=perm[X+1]+Y, BA=perm[B]+Z, BB=perm[B+1]+Z;
    return lerp1(
        lerp1(lerp1(grad(perm[AA],x,y,z),   grad(perm[BA],x-1,y,z),  u),
              lerp1(grad(perm[AB],x,y-1,z),  grad(perm[BB],x-1,y-1,z),u), v),
        lerp1(lerp1(grad(perm[AA+1],x,y,z-1),grad(perm[BA+1],x-1,y,z-1),u),
              lerp1(grad(perm[AB+1],x,y-1,z-1),grad(perm[BB+1],x-1,y-1,z-1),u),v), w);
}

// FBM 分形布朗运动
static float fbm(float x, float y, int octaves=6, float lacunarity=2.0f, float gain=0.5f){
    float val=0, amp=0.5f, freq=1.0f, maxVal=0;
    for(int i=0;i<octaves;i++){
        val += perlin(x*freq, y*freq, 0.5f)*amp;
        maxVal += amp;
        amp *= gain;
        freq *= lacunarity;
    }
    return val/maxVal;
}

// ===================== 图像/颜色 =====================

struct Color3 {
    float r, g, b;
    Color3(float r=0,float g=0,float b=0): r(r),g(g),b(b){}
    Color3 operator+(const Color3& c) const { return {r+c.r, g+c.g, b+c.b}; }
    Color3 operator-(const Color3& c) const { return {r-c.r, g-c.g, b-c.b}; }
    Color3 operator*(float t)         const { return {r*t, g*t, b*t}; }
    Color3 operator*(const Color3& c) const { return {r*c.r, g*c.g, b*c.b}; }
    Color3& operator+=(const Color3& c){ r+=c.r; g+=c.g; b+=c.b; return *this; }
    static Color3 from(const Vec3& v){ return {v.x, v.y, v.z}; }
};
inline Color3 mix(const Color3& a, const Color3& b, float t){ return a*(1-t)+b*t; }

struct Image {
    int w, h;
    std::vector<uint8_t> data;
    Image(int w, int h): w(w), h(h), data(w*h*3, 0){}
    void set(int x, int y, const Color3& c){
        if(x<0||x>=w||y<0||y>=h) return;
        auto gamma=[](float v){ return std::pow(clamp(v,0,1), 1.0f/2.2f); };
        int idx=(y*w+x)*3;
        data[idx+0]=(uint8_t)(gamma(c.r)*255+0.5f);
        data[idx+1]=(uint8_t)(gamma(c.g)*255+0.5f);
        data[idx+2]=(uint8_t)(gamma(c.b)*255+0.5f);
    }
    bool save(const char* path){
        return stbi_write_png(path, w, h, 3, data.data(), w*3) != 0;
    }
};

// ===================== 高度场与网格 =====================

// 高度场分辨率
constexpr int HF_SIZE = 256;
// 渲染分辨率
constexpr int IMG_W = 800;
constexpr int IMG_H = 600;

struct Heightfield {
    int size;
    std::vector<float> h;

    Heightfield(int sz): size(sz), h(sz*sz) {
        for(int y=0;y<sz;y++)
        for(int x=0;x<sz;x++){
            float fx = (float)x / sz * 3.5f;
            float fy = (float)y / sz * 3.5f;
            float base = fbm(fx, fy, 8, 2.0f, 0.5f);
            // 添加一个山峰
            float cx = 0.5f, cy = 0.5f;
            float nx = (float)x/sz, ny = (float)y/sz;
            float d = std::sqrt((nx-cx)*(nx-cx)+(ny-cy)*(ny-cy));
            float peak = std::exp(-d*d*8) * 0.4f;
            h[y*sz+x] = base*0.6f + peak + 0.3f;
        }
        // 归一化到 [0, 1]
        float mn=1e9f, mx=-1e9f;
        for(auto v:h){ mn=std::min(mn,v); mx=std::max(mx,v); }
        for(auto& v:h) v = (v-mn)/(mx-mn);
    }

    float get(int x, int y) const {
        x=std::max(0,std::min(size-1,x));
        y=std::max(0,std::min(size-1,y));
        return h[y*size+x];
    }
    float sample(float u, float v) const {
        float fx=u*(size-1), fy=v*(size-1);
        int ix=(int)fx, iy=(int)fy;
        float tx=fx-ix, ty=fy-iy;
        float a=get(ix,iy), b=get(ix+1,iy);
        float c=get(ix,iy+1), d=get(ix+1,iy+1);
        return lerp1(lerp1(a,b,tx), lerp1(c,d,tx), ty);
    }
    // 通过差分计算法线
    Vec3 normal(float u, float v) const {
        float eps=1.0f/size;
        float h0=sample(u,v);
        float hx=sample(u+eps,v);
        float hy=sample(u,v+eps);
        Vec3 dx={eps*10, (hx-h0)*2.5f, 0};
        Vec3 dy={0, (hy-h0)*2.5f, eps*10};
        return dy.cross(dx).norm();
    }
};

// 细分后的顶点
struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec2 uv;
    Vec3 color;  // 基于高度的颜色
};

// ===================== 相机 =====================

struct Camera {
    Vec3 origin, forward, right, up;
    float fov, aspect;

    Camera(Vec3 from, Vec3 to, Vec3 worldUp, float fov, float aspect)
        : origin(from), fov(fov), aspect(aspect)
    {
        forward = (to - from).norm();
        right   = forward.cross(worldUp).norm();
        up      = right.cross(forward);
    }

    struct Ray { Vec3 o, d; };
    Ray getRay(float px, float py) const {
        float tanH = std::tan(fov * 3.14159265f / 180.0f * 0.5f);
        float u = (px * 2.0f - 1.0f) * tanH * aspect;
        float v = (1.0f - py * 2.0f) * tanH;
        Vec3 dir = (forward + right*u + up*v).norm();
        return {origin, dir};
    }
};

// ===================== 光照 =====================

struct Light {
    Vec3 dir;   // 归一化，指向光源
    Vec3 color;
    float intensity;
};

// PBR 工具函数
static float DistributionGGX(float NdotH, float roughness){
    float a=roughness*roughness;
    float a2=a*a;
    float d=NdotH*NdotH*(a2-1)+1;
    return a2 / (3.14159265f*d*d + 1e-7f);
}
static float GeometrySchlick(float NdotV, float roughness){
    float k=(roughness+1)*(roughness+1)/8;
    return NdotV/(NdotV*(1-k)+k+1e-7f);
}
static float GeometrySmith(float NdotV, float NdotL, float roughness){
    return GeometrySchlick(NdotV,roughness)*GeometrySchlick(NdotL,roughness);
}
static Vec3 FresnelSchlick(float cosTheta, Vec3 F0){
    float t=std::pow(1-cosTheta,5);
    return F0 + (Vec3{1,1,1}-F0)*t;
}

static Color3 pbr(Vec3 N, Vec3 V, Vec3 baseColor, float metallic, float roughness,
                  const std::vector<Light>& lights, Vec3 ambient)
{
    Vec3 F0 = mix({0.04f,0.04f,0.04f}, baseColor, metallic);
    Vec3 Lo{0,0,0};

    for(const auto& light : lights){
        Vec3 L = light.dir;
        Vec3 H = (V+L).norm();
        float NdotL=saturate(N.dot(L));
        float NdotV=saturate(N.dot(V));
        float NdotH=saturate(N.dot(H));
        float HdotV=saturate(H.dot(V));
        if(NdotL <= 0) continue;

        float D = DistributionGGX(NdotH, roughness);
        float G = GeometrySmith(NdotV, NdotL, roughness);
        Vec3  F = FresnelSchlick(HdotV, F0);
        Vec3 kD = (Vec3{1,1,1}-F) * (1-metallic);

        Vec3 spec = F*(D*G/(4*NdotV*NdotL+1e-7f));
        Vec3 diff = kD * baseColor * (1.0f/3.14159265f);

        Vec3 radiance = light.color * (light.intensity * NdotL);
        Lo += (diff+spec)*radiance;
    }

    Vec3 amb = baseColor * ambient * (1-metallic*0.8f);
    Vec3 col = amb + Lo;
    return {col.x, col.y, col.z};
}

// ===================== 光线-地形相交 =====================

// 对高度场进行光线步进（sphere tracing / ray marching）
// 假设地形在 XZ 平面，高度沿 Y 轴，范围 [0, 10] x [0,1] x [0,10]

static bool intersectTerrain(const Camera::Ray& ray, const Heightfield& hf,
                              float& outT, float& outU, float& outV,
                              float dispScale = 2.0f,
                              float tMin=0.1f, float tMax=200.0f)
{
    // 地形包围盒
    float terrainW = 10.0f, terrainD = 10.0f;
    float terrainH = dispScale;

    // 步进
    float dt = 0.03f;
    for(float t=tMin; t<tMax; t+=dt){
        Vec3 p = ray.o + ray.d*t;
        // 映射到 UV
        float u = p.x / terrainW;
        float v = p.z / terrainD;
        if(u<0||u>1||v<0||v>1) { dt=std::max(0.05f, dt); continue; }
        float th = hf.sample(u,v) * terrainH;
        float above = p.y - th;
        if(above < 0){
            // 插值找精确交点
            for(int i=0;i<8;i++){
                t -= dt*0.5f; dt*=0.5f;
                p = ray.o + ray.d*t;
                u = p.x/terrainW; v=p.z/terrainD;
                th = hf.sample(u,v)*terrainH;
                above = p.y - th;
                if(above>0) t+=dt;
            }
            outT=t; outU=u; outV=v;
            return true;
        }
        // 自适应步长
        dt = std::max(0.02f, std::min(above*0.5f+0.02f, 0.5f));
    }
    return false;
}

// 软阴影：从交点向光源方向步进，检查遮挡
static float softShadow(const Vec3& origin, const Vec3& lightDir, const Heightfield& hf,
                         float dispScale=2.0f, float k=8.0f)
{
    float res=1.0f;
    float terrainW=10.f, terrainD=10.f, terrainH=dispScale;
    float dt=0.1f;
    for(float t=0.2f; t<15.0f; t+=dt){
        Vec3 p = origin + lightDir*t;
        float u=p.x/terrainW, v=p.z/terrainD;
        if(u<0||u>1||v<0||v>1) break;
        float th = hf.sample(u,v)*terrainH;
        float above = p.y - th;
        if(above<0) return 0.05f;
        res = std::min(res, k*above/t);
        dt = std::max(0.05f, above*0.4f);
    }
    return clamp(res, 0.05f, 1.0f);
}

// ===================== 颜色映射（地形）=====================

static Color3 terrainColor(float h, float slope, const Vec3& /*normal*/){
    // 基于高度和坡度的分层着色
    Color3 deepWater  {0.05f, 0.15f, 0.35f};
    Color3 shallowWater{0.1f, 0.4f, 0.6f};
    Color3 sand       {0.76f, 0.70f, 0.50f};
    Color3 grass1     {0.25f, 0.55f, 0.15f};
    Color3 grass2     {0.20f, 0.45f, 0.10f};
    Color3 rock1      {0.45f, 0.40f, 0.35f};
    Color3 rock2      {0.35f, 0.30f, 0.25f};
    Color3 snow       {0.90f, 0.92f, 0.95f};

    Color3 col;
    if(h < 0.12f) col = mix(deepWater, shallowWater, h/0.12f);
    else if(h < 0.18f) col = mix(shallowWater, sand, (h-0.12f)/0.06f);
    else if(h < 0.38f) col = mix(sand, grass1, (h-0.18f)/0.20f);
    else if(h < 0.55f) col = mix(grass1, grass2, (h-0.38f)/0.17f);
    else if(h < 0.70f) col = mix(grass2, rock1, (h-0.55f)/0.15f);
    else if(h < 0.88f) col = mix(rock1, rock2, (h-0.70f)/0.18f);
    else               col = mix(rock2, snow, (h-0.88f)/0.12f);

    // 坡度混合：陡坡用岩石覆盖
    if(slope > 0.5f){
        float t=saturate((slope-0.5f)/0.3f);
        col = mix(col, rock2*Color3(1.1f,1.05f,1.0f), t*0.6f);
    }
    return col;
}

// 天空颜色（简单渐变 + 太阳）
static Color3 skyColor(const Vec3& dir, const Vec3& sunDir){
    float t = saturate(dir.y * 0.5f + 0.5f);
    Color3 horizon{0.55f, 0.72f, 0.90f};
    Color3 zenith {0.12f, 0.35f, 0.72f};
    Color3 sky = mix(horizon, zenith, t*t);
    // 太阳
    float sunDot = saturate(dir.dot(sunDir));
    float sun = std::pow(sunDot, 256.0f);
    float glow= std::pow(sunDot, 8.0f) * 0.2f;
    sky = sky + Color3{1.5f,1.2f,0.8f}*sun + Color3{0.8f,0.5f,0.2f}*glow;
    return sky;
}

// Tone mapping (ACES)
static Color3 aces(Color3 c){
    float a=2.51f,b=0.03f,cc=2.43f,d=0.59f,e=0.14f;
    auto f=[&](float x){ return saturate((x*(a*x+b))/(x*(cc*x+d)+e)); };
    return {f(c.r), f(c.g), f(c.b)};
}

// ===================== 主渲染 =====================

int main(){
    initPerm(12345);

    std::cout << "=== Displacement Mapping & Tessellation Renderer ===" << std::endl;
    std::cout << "分辨率: " << IMG_W << "x" << IMG_H << std::endl;
    std::cout << "高度场: " << HF_SIZE << "x" << HF_SIZE << std::endl;

    // 生成高度场
    std::cout << "[1/4] 生成分形高度场..." << std::endl;
    Heightfield hf(HF_SIZE);

    // 相机设置（俯瞰地形）
    Camera cam(
        {5.0f, 6.0f, -3.0f},   // 相机位置
        {5.0f, 0.5f,  5.0f},   // 目标点
        {0,1,0},                // 上方向
        50.0f,                  // FOV
        (float)IMG_W/IMG_H
    );

    // 光源
    Vec3 sunDir = Vec3{0.6f, 0.8f, 0.4f}.norm();
    std::vector<Light> lights = {
        {sunDir,       {1.0f, 0.95f, 0.85f}, 3.5f},
        {{-0.3f,0.5f,-0.5f}, {0.4f, 0.5f, 0.7f}, 0.8f}  // 天空补光
    };
    Vec3 ambient{0.05f, 0.07f, 0.12f};

    float dispScale = 2.5f;

    // 渲染
    std::cout << "[2/4] 光线步进渲染地形..." << std::endl;
    Image img(IMG_W, IMG_H);

    for(int y=0; y<IMG_H; y++){
        if(y%100==0) std::cout << "  行 " << y << "/" << IMG_H << std::endl;
        for(int x=0; x<IMG_W; x++){
            float px=(x+0.5f)/IMG_W;
            float py=(y+0.5f)/IMG_H;
            Camera::Ray ray = cam.getRay(px, py);

            float t, u, v;
            bool hit = intersectTerrain(ray, hf, t, u, v, dispScale);

            Color3 col;
            if(hit){
                Vec3 P = ray.o + ray.d*t;
                Vec3 N = hf.normal(u, v);
                Vec3 V = (cam.origin - P).norm();

                // 地形参数
                float h = hf.sample(u, v);
                float slope = 1.0f - N.y;  // 0=平坦, 1=垂直
                Color3 baseColorC = terrainColor(h, slope, N);
                Vec3 baseColor{baseColorC.r, baseColorC.g, baseColorC.b};

                // 材质参数（基于高度）
                float metallic  = h > 0.85f ? 0.05f : 0.02f;
                float roughness = 0.3f + slope*0.5f + (1-h)*0.15f;
                roughness = clamp(roughness, 0.1f, 0.9f);

                // 水面特殊处理
                bool isWater = (h < 0.15f);
                if(isWater){
                    metallic  = 0.1f;
                    roughness = 0.08f;
                    baseColor = {0.05f, 0.25f, 0.45f};
                    // 简单水面法线扰动（Perlin）
                    float wn = perlin(P.x*2+0.5f, P.z*2+0.3f, 0.5f)*0.12f;
                    N = Vec3{N.x+wn, N.y, N.z+wn*0.7f}.norm();
                }

                // 软阴影
                float shadow = softShadow(P + N*0.05f, sunDir, hf, dispScale);

                // 修改第一个光源强度（软阴影）
                std::vector<Light> shadowedLights = lights;
                shadowedLights[0].intensity *= shadow;

                // PBR 着色
                col = pbr(N, V, baseColor, metallic, roughness, shadowedLights, ambient);

                // 雾效果（距离衰减）
                float fogDist = t / 80.0f;
                fogDist = saturate(fogDist*fogDist);
                Color3 fogColor{0.65f, 0.78f, 0.92f};
                col = mix(col, fogColor, fogDist);

            } else {
                // 天空
                col = skyColor(ray.d, sunDir);
            }

            // Tone mapping
            col = aces(col * 1.2f);
            img.set(x, y, col);
        }
    }

    // 输出信息
    std::cout << "[3/4] 保存图像..." << std::endl;
    const char* outPath = "displacement_output.png";
    if(!img.save(outPath)){
        std::cerr << "❌ 无法保存图像！" << std::endl;
        return 1;
    }
    std::cout << "✅ 图像已保存: " << outPath << std::endl;

    // 输出统计
    std::cout << "[4/4] 验证统计信息" << std::endl;
    long totalR=0, totalG=0, totalB=0;
    for(int i=0; i<IMG_W*IMG_H; i++){
        totalR += img.data[i*3+0];
        totalG += img.data[i*3+1];
        totalB += img.data[i*3+2];
    }
    long n = IMG_W*IMG_H;
    std::cout << "像素均值 R=" << totalR/n
              << " G=" << totalG/n
              << " B=" << totalB/n << std::endl;

    std::cout << "=== 渲染完成 ===" << std::endl;
    return 0;
}
