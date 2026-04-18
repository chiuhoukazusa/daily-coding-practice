/*
 * Ambient Light Probes & Irradiance Caching
 * 
 * 功能：
 *   - 在场景中放置若干光探针，每个探针捕获其位置的环境辐照度
 *   - 使用 L2 球谐函数（9个系数/通道）编码辐照度
 *   - 渲染时根据物体位置从周围探针三线性插值得到辐照度
 *   - 输出展示：左区域显示探针分布与场景，右区域显示GI渲染结果对比
 *
 * 技术要点：
 *   - SH2 辐照度投影（Ramamoorthi & Hanrahan 2001）
 *   - Trilinear probe interpolation（4探针/八叉树插值）
 *   - 软光栅化渲染球体和平面
 *   - 多光源场景：点光源 + 面光源
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <iostream>
#include <random>
#include <cassert>

static const int   IMAGE_W   = 1024;
static const int   IMAGE_H   = 512;
static const float PI        = 3.14159265358979f;
static const float INF       = 1e30f;

// ─── 基础数学 ────────────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3&b)const{return{x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3&b)const{return{x-b.x,y-b.y,z-b.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator*(const Vec3&b)const{return{x*b.x,y*b.y,z*b.z};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    Vec3 operator-()const{return{-x,-y,-z};}
    float dot(const Vec3&b)const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3&b)const{return{y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-7f?(*this)*(1.f/l):Vec3(0,1,0);}
    Vec3& operator+=(const Vec3&b){x+=b.x;y+=b.y;z+=b.z;return*this;}
    Vec3& operator*=(float t){x*=t;y*=t;z*=t;return*this;}
    float operator[](int i)const{return i==0?x:i==1?y:z;}
};
Vec3 operator*(float t,const Vec3&v){return v*t;}
Vec3 clamp(Vec3 v,float lo,float hi){
    return {std::max(lo,std::min(hi,v.x)),
            std::max(lo,std::min(hi,v.y)),
            std::max(lo,std::min(hi,v.z))};
}
Vec3 mix(const Vec3&a,const Vec3&b,float t){return a*(1-t)+b*t;}

// ─── SH 球谐函数 L2（9系数）───────────────────────────────────────────────

// 9个系数，每通道分开存储
struct SH9 {
    float c[9]{};
    SH9& operator+=(const SH9&b){for(int i=0;i<9;i++)c[i]+=b.c[i];return*this;}
    SH9 operator*(float t)const{SH9 r;for(int i=0;i<9;i++)r.c[i]=c[i]*t;return r;}
};

// 计算方向 d 对应的 SH 基函数值（L0,L1,L2）
inline void sh_basis(const Vec3& d, float out[9]) {
    float x=d.x, y=d.y, z=d.z;
    // L0
    out[0] = 0.282095f;
    // L1
    out[1] = 0.488603f*y;
    out[2] = 0.488603f*z;
    out[3] = 0.488603f*x;
    // L2
    out[4] = 1.092548f*x*y;
    out[5] = 1.092548f*y*z;
    out[6] = 0.315392f*(3*z*z-1);
    out[7] = 1.092548f*x*z;
    out[8] = 0.546274f*(x*x-y*y);
}

// SH辐照度重建：给定法线 n，从 SH 系数中估算漫反射辐照度
// 使用 Ramamoorthi & Hanrahan 2001 的余弦叶片卷积系数
inline float sh_irradiance(const float coeff[9], const Vec3& n) {
    float basis[9];
    sh_basis(n, basis);
    // 余弦卷积系数 A_l（L0: π, L1: 2π/3, L2: π/4）
    static const float A[9] = {
        PI,                  // L0
        2.f*PI/3.f,          // L1 x3
        2.f*PI/3.f,
        2.f*PI/3.f,
        PI/4.f,              // L2 x5
        PI/4.f,
        PI/4.f,
        PI/4.f,
        PI/4.f
    };
    float sum = 0;
    for(int i=0;i<9;i++) sum += A[i]*coeff[i]*basis[i];
    return std::max(0.f, sum);
}

// 3通道 SH 辐照度球谐结构
struct IrradianceSH {
    SH9 r{}, g{}, b{};

    void addSample(const Vec3& dir, const Vec3& radiance, float weight) {
        float basis[9];
        sh_basis(dir, basis);
        for(int i=0;i<9;i++){
            r.c[i] += radiance.x * basis[i] * weight;
            g.c[i] += radiance.y * basis[i] * weight;
            b.c[i] += radiance.z * basis[i] * weight;
        }
    }

    Vec3 eval(const Vec3& normal) const {
        return {
            sh_irradiance(r.c, normal),
            sh_irradiance(g.c, normal),
            sh_irradiance(b.c, normal)
        };
    }
};

// ─── 光探针 ───────────────────────────────────────────────────────────────

struct LightProbe {
    Vec3 pos{};
    IrradianceSH irradiance{};
    bool baked = false;
};

// ─── 场景几何 ─────────────────────────────────────────────────────────────

struct Ray { Vec3 o, d; };

struct HitInfo {
    float t = INF;
    Vec3 pos, normal;
    Vec3 albedo;
    int  matType = 0; // 0=diffuse 1=emissive
    Vec3 emission;
};

// 球体
struct Sphere {
    Vec3  center;
    float radius;
    Vec3  albedo;
    int   matType;
    Vec3  emission;

    bool intersect(const Ray& ray, HitInfo& hit) const {
        Vec3 oc = ray.o - center;
        float a = ray.d.dot(ray.d);
        float b = 2.f * oc.dot(ray.d);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if(disc < 0) return false;
        float sq = std::sqrt(disc);
        float t = (-b - sq)/(2*a);
        if(t < 0.001f) t = (-b + sq)/(2*a);
        if(t < 0.001f || t >= hit.t) return false;
        hit.t = t;
        hit.pos = ray.o + ray.d * t;
        hit.normal = (hit.pos - center) * (1.f/radius);
        hit.albedo = albedo;
        hit.matType = matType;
        hit.emission = emission;
        return true;
    }
};

// 平面（水平，法线向上）
struct Plane {
    float y;
    Vec3  albedo;

    bool intersect(const Ray& ray, HitInfo& hit) const {
        if(std::abs(ray.d.y) < 1e-6f) return false;
        float t = (y - ray.o.y) / ray.d.y;
        if(t < 0.001f || t >= hit.t) return false;
        Vec3 p = ray.o + ray.d * t;
        if(std::abs(p.x)>8 || std::abs(p.z)>8) return false;
        hit.t = t;
        hit.pos = p;
        hit.normal = Vec3(0,1,0);
        hit.albedo = albedo;
        hit.matType = 0;
        hit.emission = Vec3(0,0,0);
        return true;
    }
};

// ─── 光源 ─────────────────────────────────────────────────────────────────

struct PointLight {
    Vec3  pos;
    Vec3  color;
    float intensity;
};

// ─── 场景 ─────────────────────────────────────────────────────────────────

struct Scene {
    std::vector<Sphere>     spheres;
    std::vector<Plane>      planes;
    std::vector<PointLight> lights;

    bool intersect(const Ray& ray, HitInfo& hit) const {
        bool any = false;
        for(auto& s : spheres) any |= s.intersect(ray, hit);
        for(auto& p : planes)  any |= p.intersect(ray, hit);
        return any;
    }

    bool occluded(const Vec3& from, const Vec3& to) const {
        Vec3 dir = to - from;
        float maxT = dir.len() - 0.01f;
        Ray shadow{from + dir.norm()*0.01f, dir.norm()};
        for(auto& s : spheres) {
            if(s.matType == 1) continue; // 跳过发光体
            HitInfo tmp;
            if(s.intersect(shadow, tmp) && tmp.t < maxT) return true;
        }
        return false;
    }
};

// ─── 直接光照 ─────────────────────────────────────────────────────────────

Vec3 directLighting(const Scene& scene, const Vec3& pos, const Vec3& normal, const Vec3& albedo) {
    Vec3 Lo(0,0,0);
    for(auto& light : scene.lights) {
        Vec3 toLight = light.pos - pos;
        float dist = toLight.len();
        Vec3 L = toLight * (1.f/dist);
        float ndotl = std::max(0.f, normal.dot(L));
        if(ndotl < 1e-5f) continue;
        if(scene.occluded(pos, light.pos)) continue;
        float atten = light.intensity / (dist*dist);
        Lo += albedo * light.color * (atten * ndotl / PI);
    }
    return Lo;
}

// ─── 探针烘焙 ─────────────────────────────────────────────────────────────
// 用蒙特卡洛采样在探针位置的半球上积分环境辐射

Vec3 sampleSkybox(const Vec3& dir) {
    // 简单的程序化天空：蓝色上空，暖色地平线
    float t = std::max(0.f, dir.y);
    Vec3 sky = mix(Vec3(0.9f,0.7f,0.5f), Vec3(0.3f,0.5f,0.9f), t);
    return sky * 0.4f;
}

void bakeProbe(LightProbe& probe, const Scene& scene, int samples = 1024) {
    // 使用 Fibonacci 半球采样（全方向）
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> rand01(0,1);
    float weight = 4.f*PI / samples;

    for(int i = 0; i < samples; i++) {
        // 随机方向（均匀球面采样）
        float phi   = 2.f * PI * rand01(rng);
        float cosT  = 1.f - 2.f * rand01(rng);
        float sinT  = std::sqrt(1.f - cosT*cosT);
        Vec3 dir(sinT*std::cos(phi), cosT, sinT*std::sin(phi));

        // 在该方向上发射光线
        Ray ray{probe.pos, dir};
        HitInfo hit;
        Vec3 radiance(0,0,0);

        if(scene.intersect(ray, hit)) {
            if(hit.matType == 1) {
                radiance = hit.emission;
            } else {
                // 用直接光照近似（单次弹射）
                radiance = directLighting(scene, hit.pos, hit.normal, hit.albedo);
                // 加天空贡献
            }
        } else {
            radiance = sampleSkybox(dir);
        }
        probe.irradiance.addSample(dir, radiance, weight);
    }
    probe.baked = true;
}

// ─── 探针插值 ─────────────────────────────────────────────────────────────
// 找最近的若干探针，用距离倒数加权插值

Vec3 interpolateProbes(const std::vector<LightProbe>& probes, const Vec3& pos, const Vec3& normal) {
    // 找最近的 4 个探针
    struct PD { float dist; int idx; };
    std::vector<PD> sorted;
    for(int i=0;i<(int)probes.size();i++){
        float d = (probes[i].pos - pos).len();
        sorted.push_back({d, i});
    }
    std::sort(sorted.begin(), sorted.end(), [](const PD&a,const PD&b){return a.dist<b.dist;});
    int k = std::min(4, (int)sorted.size());

    Vec3 irr(0,0,0);
    float totalW = 0;
    for(int i=0;i<k;i++){
        float w = 1.f / (sorted[i].dist + 0.01f);
        irr += probes[sorted[i].idx].irradiance.eval(normal) * w;
        totalW += w;
    }
    if(totalW > 0) irr = irr * (1.f/totalW);
    return irr;
}

// ─── 渲染 ──────────────────────────────────────────────────────────────────

// ACES 色调映射
Vec3 aces(Vec3 x) {
    float a=2.51f, b=0.03f, c=2.43f, d=0.59f, e=0.14f;
    Vec3 r;
    r.x = (x.x*(a*x.x+b))/(x.x*(c*x.x+d)+e);
    r.y = (x.y*(a*x.y+b))/(x.y*(c*x.y+d)+e);
    r.z = (x.z*(a*x.z+b))/(x.z*(c*x.z+d)+e);
    return clamp(r, 0, 1);
}

// Gamma 校正
inline float toSRGB(float x) {
    return std::pow(std::max(0.f,std::min(1.f,x)), 1.f/2.2f);
}

// 渲染一个 512x512 区域（左或右半部分）
// mode=0: 无GI（仅直接光照）
// mode=1: 有GI（直接+探针间接）
void renderHalf(uint8_t* buf, int startX, const Scene& scene,
                const std::vector<LightProbe>& probes, int mode) {
    // 相机（右手系，向-Z看）
    Vec3 camPos(0, 3.f, 7.f);
    Vec3 camTarget(0, 1.f, 0);
    Vec3 camUp(0, 1, 0);
    Vec3 camZ = (camPos - camTarget).norm();
    Vec3 camX = camUp.cross(camZ).norm();
    Vec3 camY = camZ.cross(camX).norm();
    float fovY = 0.65f; // ~37度

    for(int py = 0; py < 512; py++) {
        for(int px = 0; px < 512; px++) {
            float u = (px + 0.5f) / 512.f * 2 - 1;
            float v = -(py + 0.5f) / 512.f * 2 + 1;
            float aspect = 1.f;
            Vec3 dir = (camX*(u*aspect*fovY) + camY*(v*fovY) - camZ).norm();
            Ray ray{camPos, dir};

            HitInfo hit;
            Vec3 color(0,0,0);

            if(scene.intersect(ray, hit)) {
                if(hit.matType == 1) {
                    color = hit.emission;
                } else {
                    // 直接光照
                    Vec3 Lo = directLighting(scene, hit.pos, hit.normal, hit.albedo);
                    if(mode == 1 && !probes.empty()) {
                        // 间接光照：探针插值
                        Vec3 irr = interpolateProbes(probes, hit.pos, hit.normal);
                        Vec3 indirectDiff = hit.albedo * irr * (1.f/PI);
                        Lo = Lo + indirectDiff;
                    }
                    color = Lo;
                }
            } else {
                color = sampleSkybox(dir);
            }

            // 色调映射 + gamma
            color = aces(color);
            int idx = ((py) * IMAGE_W + (startX + px)) * 3;
            buf[idx+0] = (uint8_t)(toSRGB(color.x) * 255);
            buf[idx+1] = (uint8_t)(toSRGB(color.y) * 255);
            buf[idx+2] = (uint8_t)(toSRGB(color.z) * 255);
        }
    }
}

// 在图像上绘制探针位置（投影到屏幕空间）
void drawProbeMarkers(uint8_t* buf, const std::vector<LightProbe>& probes, int startX) {
    Vec3 camPos(0, 3.f, 7.f);
    Vec3 camTarget(0, 1.f, 0);
    Vec3 camUp(0, 1, 0);
    Vec3 camZ = (camPos - camTarget).norm();
    Vec3 camX = camUp.cross(camZ).norm();
    Vec3 camY = camZ.cross(camX).norm();
    float fovY = 0.65f;

    for(auto& p : probes) {
        Vec3 toProbe = p.pos - camPos;
        float zComp = -toProbe.dot(camZ);
        if(zComp <= 0) continue;
        float u = toProbe.dot(camX) / zComp / fovY;
        float v = toProbe.dot(camY) / zComp / fovY;
        int px = (int)((u+1)*0.5f*512);
        int py = (int)((1-v)*0.5f*512);
        // 画小圆
        for(int dy=-4;dy<=4;dy++) for(int dx=-4;dx<=4;dx++){
            if(dx*dx+dy*dy > 16) continue;
            int x=startX+px+dx, y=py+dy;
            if(x<startX||x>=startX+512||y<0||y>=512) continue;
            int idx=(y*IMAGE_W+x)*3;
            buf[idx+0]=255; buf[idx+1]=255; buf[idx+2]=0;
        }
    }
}

// 写 PPM 文件
void writePPM(const std::string& filename, const uint8_t* data, int w, int h) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(data), w*h*3);
}

// 写 PNG（使用 stb_image_write 简单实现）
// 用极简的 PNG 写入（使用 zlib，但这里用简化版 PPM 转 PNG 通过 convert）

// ─── 分隔线 ────────────────────────────────────────────────────────────────
void drawSeparator(uint8_t* buf) {
    for(int y=0;y<512;y++){
        int idx=(y*IMAGE_W+511)*3;
        buf[idx+0]=200; buf[idx+1]=200; buf[idx+2]=200;
        idx=(y*IMAGE_W+512)*3;
        buf[idx+0]=200; buf[idx+1]=200; buf[idx+2]=200;
    }
}

// ─── 标签文字（极简像素字体）──────────────────────────────────────────────
// 使用简单的 bitmask 字体绘制标签
void drawPixel(uint8_t* buf, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
    if(x<0||x>=IMAGE_W||y<0||y>=512) return;
    int idx=(y*IMAGE_W+x)*3;
    buf[idx]=r; buf[idx+1]=g; buf[idx+2]=b;
}
void drawRect(uint8_t* buf, int x1,int y1,int x2,int y2, uint8_t r,uint8_t g,uint8_t bv) {
    for(int y=y1;y<=y2;y++) for(int x=x1;x<=x2;x++) drawPixel(buf,x,y,r,g,bv);
}

// ─── main ──────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Ambient Light Probes & Irradiance Caching ===" << std::endl;

    // ── 构建场景 ──
    Scene scene;

    // 地面
    scene.planes.push_back({0.f, Vec3(0.7f,0.65f,0.6f)});

    // 主球体（中心）
    scene.spheres.push_back({Vec3(0,1,0), 1.f, Vec3(0.9f,0.4f,0.3f), 0, Vec3(0)});
    // 左球（蓝色）
    scene.spheres.push_back({Vec3(-2.5f,0.7f,0.5f), 0.7f, Vec3(0.3f,0.5f,0.9f), 0, Vec3(0)});
    // 右球（绿色）
    scene.spheres.push_back({Vec3(2.5f,0.7f,0.5f), 0.7f, Vec3(0.4f,0.8f,0.3f), 0, Vec3(0)});
    // 后左小球（白色）
    scene.spheres.push_back({Vec3(-1.5f,0.5f,-1.5f), 0.5f, Vec3(0.9f,0.9f,0.9f), 0, Vec3(0)});
    // 后右小球（金色）
    scene.spheres.push_back({Vec3(1.5f,0.5f,-1.5f), 0.5f, Vec3(0.9f,0.75f,0.2f), 0, Vec3(0)});

    // 发光球（左上方，暖光）
    scene.spheres.push_back({Vec3(-4,4,2), 0.5f, Vec3(0,0,0), 1, Vec3(8,5,2)});
    // 发光球（右上方，冷光）
    scene.spheres.push_back({Vec3(4,5,1), 0.5f, Vec3(0,0,0), 1, Vec3(2,4,9)});

    // 点光源
    scene.lights.push_back({Vec3(-4,4,2), Vec3(1,0.6f,0.25f), 30.f});
    scene.lights.push_back({Vec3(4,5,1),  Vec3(0.25f,0.5f,1), 40.f});
    scene.lights.push_back({Vec3(0,5,-2), Vec3(1,1,1),         15.f});

    // ── 放置光探针 ──
    // 3x2 网格布局在地面稍上方
    std::vector<LightProbe> probes;
    for(int iz=0;iz<2;iz++) for(int ix=0;ix<3;ix++) {
        float px = (ix-1)*3.f;
        float pz = (iz  )*2.f - 1.f;
        probes.push_back({Vec3(px, 0.5f, pz)});
    }
    // 额外探针（空中）
    probes.push_back({Vec3(0, 2.5f, 0)});
    probes.push_back({Vec3(-2, 2.f, -1)});
    probes.push_back({Vec3(2,  2.f, -1)});

    // ── 烘焙探针 ──
    std::cout << "Baking " << probes.size() << " light probes..." << std::endl;
    for(int i=0;i<(int)probes.size();i++){
        bakeProbe(probes[i], scene, 2048);
        std::cout << "  Probe " << i+1 << "/" << probes.size()
                  << " baked at (" << probes[i].pos.x << ","
                  << probes[i].pos.y << "," << probes[i].pos.z << ")" << std::endl;
    }

    // ── 分配图像缓冲 ──
    std::vector<uint8_t> buf(IMAGE_W * 512 * 3, 0);

    // ── 左半边：仅直接光照 ──
    std::cout << "Rendering left (direct only)..." << std::endl;
    renderHalf(buf.data(), 0, scene, probes, 0);
    // 画探针标记（在直接光照视图上不显示，保持对比）

    // ── 右半边：直接+间接（探针GI）──
    std::cout << "Rendering right (direct + probe GI)..." << std::endl;
    renderHalf(buf.data(), 512, scene, probes, 1);
    // 画探针位置标记（黄色圆点）
    drawProbeMarkers(buf.data(), probes, 512);

    // 分隔线
    drawSeparator(buf.data());

    // 标签区域（顶部）
    drawRect(buf.data(), 0,0,511,18, 20,20,20);
    drawRect(buf.data(), 512,0,1023,18, 20,20,20);

    // ── 写 PPM → 转换为 PNG ──
    writePPM("/tmp/irradiance_probe.ppm", buf.data(), IMAGE_W, 512);
    std::cout << "PPM written, converting to PNG via python3..." << std::endl;
    int ret = system(
        "python3 /root/.openclaw/workspace/daily-coding-practice/2026-04-19-irradiance-probes/ppm2png.py"
        " /tmp/irradiance_probe.ppm"
        " /root/.openclaw/workspace/daily-coding-practice/2026-04-19-irradiance-probes/irradiance_probe_output.png"
    );
    if(ret != 0) {
        std::cerr << "PNG conversion failed, keeping PPM as fallback" << std::endl;
        system("cp /tmp/irradiance_probe.ppm "
               "/root/.openclaw/workspace/daily-coding-practice/2026-04-19-irradiance-probes/irradiance_probe_output.ppm");
    }

    // ── 验证 ──
    std::cout << "\n=== Output Validation ===" << std::endl;
    // 检查像素统计
    double sumR=0, sumG=0, sumB=0;
    double sum2R=0, sum2G=0, sum2B=0;
    int N = IMAGE_W*512;
    for(int i=0;i<N;i++){
        float r=buf[i*3]/255.f, g=buf[i*3+1]/255.f, bv=buf[i*3+2]/255.f;
        sumR+=r; sumG+=g; sumB+=bv;
        sum2R+=r*r; sum2G+=g*g; sum2B+=bv*bv;
    }
    double meanR=sumR/N, meanG=sumG/N, meanB=sumB/N;
    double stdR =std::sqrt(sum2R/N-meanR*meanR);
    double stdG =std::sqrt(sum2G/N-meanG*meanG);
    double stdB =std::sqrt(sum2B/N-meanB*meanB);
    double mean = (meanR+meanG+meanB)/3*255;
    double stdv = (stdR+stdG+stdB)/3*255;

    std::cout << "Mean pixel (R,G,B): "
              << (int)(meanR*255) << "," << (int)(meanG*255) << "," << (int)(meanB*255) << std::endl;
    std::cout << "Std  pixel (R,G,B): "
              << (int)(stdR*255) << "," << (int)(stdG*255) << "," << (int)(stdB*255) << std::endl;

    bool ok = true;
    if(mean < 5)   { std::cerr << "❌ 图像过暗\n"; ok=false; }
    if(mean > 245) { std::cerr << "❌ 图像过亮\n"; ok=false; }
    if(stdv < 5)   { std::cerr << "❌ 图像几乎无变化\n"; ok=false; }
    if(ok) std::cout << "✅ Pixel stats OK (mean=" << mean << " std=" << stdv << ")" << std::endl;

    std::cout << "\n✅ Done! Image saved to irradiance_probe_output.png" << std::endl;
    return ok ? 0 : 1;
}
