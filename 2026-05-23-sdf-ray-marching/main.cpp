// SDF Ray Marching Scene Renderer
// 使用 Signed Distance Functions 构建程序化场景
// 特性：软阴影、AO环境遮蔽、Phong着色、多材质、多SDF几何体组合

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <vector>
#include <string>
#include <cstdio>

// ==================== 数学工具 ====================
struct Vec2 {
    float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
    Vec2 operator+(const Vec2& b) const { return {x+b.x, y+b.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
    float dot(const Vec2& b) const { return x*b.x + y*b.y; }
    float length() const { return sqrtf(x*x+y*y); }
};

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l>1e-8f ? *this/l : Vec3(0,1,0); }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
};

inline Vec3 clamp3(Vec3 v, float lo, float hi) {
    return { std::max(lo,std::min(hi,v.x)),
             std::max(lo,std::min(hi,v.y)),
             std::max(lo,std::min(hi,v.z)) };
}
inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
inline float mixf(float a, float b, float t) { return a + (b-a)*t; }
inline Vec3 mix3(Vec3 a, Vec3 b, float t) { return a + (b-a)*t; }

// ==================== SDF 基元 ====================

// 球体 SDF
float sdSphere(Vec3 p, float r) {
    return p.length() - r;
}

// 盒子 SDF
float sdBox(Vec3 p, Vec3 b) {
    Vec3 q = { fabsf(p.x)-b.x, fabsf(p.y)-b.y, fabsf(p.z)-b.z };
    Vec3 qp = { std::max(q.x,0.0f), std::max(q.y,0.0f), std::max(q.z,0.0f) };
    return qp.length() + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0f);
}

// 平面 SDF（y=c平面）
float sdPlane(Vec3 p, float height) {
    return p.y - height;
}

// 胶囊体 SDF
float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
    Vec3 pa = p - a, ba = b - a;
    float h = clampf(pa.dot(ba) / ba.dot(ba), 0.0f, 1.0f);
    return (pa - ba*h).length() - r;
}

// 环形面SDF（torus）
float sdTorus(Vec3 p, float R, float r) {
    float qx = sqrtf(p.x*p.x + p.z*p.z) - R;
    return sqrtf(qx*qx + p.y*p.y) - r;
}

// 圆柱体SDF（无限高，需配合box截断）
float sdCylinder(Vec3 p, float r, float h) {
    Vec2 d = {sqrtf(p.x*p.x+p.z*p.z) - r, fabsf(p.y) - h};
    float dx = std::max(d.x, 0.0f);
    float dy = std::max(d.y, 0.0f);
    return std::min(std::max(d.x, d.y), 0.0f) + sqrtf(dx*dx+dy*dy);
}

// SDF 平滑融合（smooth union）
float sdfSmoothUnion(float d1, float d2, float k) {
    float h = clampf(0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f);
    return mixf(d2, d1, h) - k*h*(1.0f-h);
}

// SDF 差（subtraction）
float sdfSubtract(float d1, float d2) {
    return std::max(-d2, d1);
}

// ==================== 材质系统 ====================
struct Material {
    Vec3 albedo;
    float roughness;
    float metallic;
    float emissive;
    
    Material(Vec3 a={1,1,1}, float r=0.5f, float m=0.0f, float e=0.0f)
        : albedo(a), roughness(r), metallic(m), emissive(e) {}
};

// ==================== 场景 ====================
// 场景SDF，返回最近距离 + 材质ID
struct SDFResult {
    float dist;
    int matID;  // 0=地面, 1=大球, 2=盒子, 3=胶囊, 4=torus, 5=小球群, 6=圆柱
};

SDFResult sceneMap(Vec3 p) {
    // 地面平面
    float dPlane = sdPlane(p, -1.0f);
    SDFResult res = { dPlane, 0 };
    
    // 中心大球（金属球）
    float dSphere1 = sdSphere(p - Vec3(0, 0.5f, 0), 1.5f);
    if (dSphere1 < res.dist) { res.dist = dSphere1; res.matID = 1; }
    
    // 左侧盒子
    Vec3 pb = p - Vec3(-3.5f, 0.0f, 0.5f);
    float dBox = sdBox(pb, Vec3(0.7f, 1.0f, 0.7f));
    if (dBox < res.dist) { res.dist = dBox; res.matID = 2; }
    
    // 右侧圆环（torus）
    Vec3 pt = p - Vec3(3.5f, 0.3f, 0.0f);
    float dTorus = sdTorus(pt, 1.0f, 0.35f);
    if (dTorus < res.dist) { res.dist = dTorus; res.matID = 4; }
    
    // 后方胶囊体
    Vec3 capA = Vec3(-1.5f, -0.5f, -3.5f);
    Vec3 capB = Vec3(1.5f, 0.5f, -3.5f);
    float dCapsule = sdCapsule(p, capA, capB, 0.4f);
    if (dCapsule < res.dist) { res.dist = dCapsule; res.matID = 3; }
    
    // 圆柱体（左后）
    Vec3 pcyl = p - Vec3(-3.0f, -0.5f, -2.5f);
    float dCyl = sdCylinder(pcyl, 0.5f, 0.5f);
    if (dCyl < res.dist) { res.dist = dCyl; res.matID = 6; }
    
    // 前方小球群（平滑融合）
    float dSmall = 1e10f;
    float smallPositions[4][3] = {
        { 2.0f, -0.7f, 2.5f},
        { 2.8f, -0.5f, 2.0f},
        { 1.5f, -0.6f, 1.8f},
        { 2.3f, -0.3f, 1.5f}
    };
    float smallRadii[4] = { 0.3f, 0.25f, 0.2f, 0.22f };
    for(int i = 0; i < 4; i++) {
        Vec3 pp = p - Vec3(smallPositions[i][0], smallPositions[i][1], smallPositions[i][2]);
        float d = sdSphere(pp, smallRadii[i]);
        dSmall = sdfSmoothUnion(dSmall, d, 0.3f);
    }
    if (dSmall < res.dist) { res.dist = dSmall; res.matID = 5; }
    
    return res;
}

// 材质查询
Material getMaterial(int matID) {
    switch(matID) {
        case 0: return Material(Vec3(0.8f, 0.8f, 0.75f), 0.9f, 0.0f);     // 地面：灰白，高粗糙度
        case 1: return Material(Vec3(0.95f, 0.9f, 0.85f), 0.1f, 1.0f);    // 金属球：高光泽金属
        case 2: return Material(Vec3(0.3f, 0.5f, 0.9f), 0.6f, 0.0f);      // 盒子：蓝色漫反射
        case 3: return Material(Vec3(0.9f, 0.4f, 0.3f), 0.4f, 0.0f);      // 胶囊：红色
        case 4: return Material(Vec3(0.9f, 0.8f, 0.2f), 0.3f, 0.5f);      // torus：金黄色半金属
        case 5: return Material(Vec3(0.4f, 0.9f, 0.5f), 0.5f, 0.0f);      // 小球群：绿色
        case 6: return Material(Vec3(0.7f, 0.3f, 0.8f), 0.4f, 0.0f);      // 圆柱：紫色
        default: return Material(Vec3(0.8f,0.8f,0.8f), 0.5f, 0.0f);
    }
}

// ==================== Ray Marching ====================
const int MAX_STEPS = 128;
const float MAX_DIST = 50.0f;
const float SURF_DIST = 0.001f;

SDFResult rayMarch(Vec3 ro, Vec3 rd) {
    float t = 0.02f;
    SDFResult hit = { -1.0f, -1 };
    for(int i = 0; i < MAX_STEPS; i++) {
        Vec3 p = ro + rd * t;
        SDFResult r = sceneMap(p);
        if(r.dist < SURF_DIST) {
            hit.dist = t;
            hit.matID = r.matID;
            return hit;
        }
        t += r.dist;
        if(t > MAX_DIST) break;
    }
    return hit; // miss
}

// ==================== 法线计算 ====================
Vec3 calcNormal(Vec3 p) {
    const float eps = 0.001f;
    float dx = sceneMap(p + Vec3(eps,0,0)).dist - sceneMap(p - Vec3(eps,0,0)).dist;
    float dy = sceneMap(p + Vec3(0,eps,0)).dist - sceneMap(p - Vec3(0,eps,0)).dist;
    float dz = sceneMap(p + Vec3(0,0,eps)).dist - sceneMap(p - Vec3(0,0,eps)).dist;
    return Vec3(dx, dy, dz).normalized();
}

// ==================== 软阴影 ====================
float softShadow(Vec3 ro, Vec3 rd, float mint, float maxt, float k) {
    float res = 1.0f;
    float t = mint;
    for(int i = 0; i < 64; i++) {
        Vec3 p = ro + rd * t;
        float h = sceneMap(p).dist;
        if(h < 0.001f) return 0.0f;
        res = std::min(res, k * h / t);
        t += h;
        if(t > maxt) break;
    }
    return clampf(res, 0.0f, 1.0f);
}

// ==================== 环境光遮蔽（AO）====================
float calcAO(Vec3 pos, Vec3 nor) {
    float occ = 0.0f;
    float sca = 1.0f;
    for(int i = 0; i < 5; i++) {
        float hr = 0.01f + 0.12f * float(i) / 4.0f;
        Vec3 aopos = nor * hr + pos;
        float dd = sceneMap(aopos).dist;
        occ += (hr - dd) * sca;
        sca *= 0.95f;
    }
    return clampf(1.0f - 3.0f * occ, 0.0f, 1.0f);
}

// ==================== 着色 ====================
Vec3 shade(Vec3 pos, Vec3 nor, Vec3 rd, Material mat, Vec3 lightPos, Vec3 lightCol) {
    Vec3 lightDir = (lightPos - pos).normalized();
    float dist2Light = (lightPos - pos).length();
    
    // 漫反射
    float diff = clampf(nor.dot(lightDir), 0.0f, 1.0f);
    
    // 高光（Blinn-Phong）
    Vec3 viewDir = (-rd).normalized();
    Vec3 halfDir = (lightDir + viewDir).normalized();
    float spec = 0.0f;
    if(diff > 0.0f) {
        float shininess = 2.0f / (mat.roughness * mat.roughness + 1e-4f) - 2.0f;
        shininess = std::max(shininess, 1.0f);
        spec = powf(clampf(nor.dot(halfDir), 0.0f, 1.0f), shininess);
    }
    
    // 软阴影
    float shadow = softShadow(pos + nor * 0.01f, lightDir, 0.02f, dist2Light, 12.0f);
    
    // AO
    float ao = calcAO(pos, nor);
    
    // 衰减
    float attenuation = 1.0f / (1.0f + 0.05f * dist2Light * dist2Light);
    
    // 混合金属/非金属
    Vec3 specCol = mix3(Vec3(0.04f, 0.04f, 0.04f), mat.albedo, mat.metallic);
    Vec3 diffCol = mat.albedo * (1.0f - mat.metallic);
    
    Vec3 ambient = diffCol * Vec3(0.15f, 0.18f, 0.22f) * ao;
    Vec3 diffuse = diffCol * lightCol * diff * shadow * attenuation;
    Vec3 specular = specCol * lightCol * spec * shadow * attenuation;
    
    Vec3 color = ambient + diffuse + specular;
    
    // 自发光
    if(mat.emissive > 0.0f) {
        color += mat.albedo * mat.emissive * 0.5f;
    }
    
    return color;
}

// ==================== 天空/背景 ====================
Vec3 skyColor(Vec3 rd) {
    // 简单渐变天空：上方蓝色 → 地平线偏白
    float t = clampf(rd.y * 0.5f + 0.5f, 0.0f, 1.0f);
    Vec3 top    = Vec3(0.2f, 0.4f, 0.8f);
    Vec3 bottom = Vec3(0.8f, 0.85f, 0.9f);
    Vec3 sky = mix3(bottom, top, t);
    
    // 太阳光晕
    Vec3 sunDir = Vec3(3.0f, 5.0f, -4.0f).normalized();
    float sun = clampf(rd.dot(sunDir), 0.0f, 1.0f);
    sky += Vec3(1.0f, 0.9f, 0.7f) * powf(sun, 64.0f) * 0.8f;
    
    return sky;
}

// ==================== 色调映射 ====================
Vec3 tonemapACES(Vec3 x) {
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    Vec3 num = x*(x*a + Vec3(b,b,b));
    Vec3 den = x*(x*c + Vec3(d,d,d)) + Vec3(e,e,e);
    return clamp3(Vec3(num.x/den.x, num.y/den.y, num.z/den.z), 0.0f, 1.0f);
}

// gamma correction
Vec3 gammaCorrect(Vec3 c) {
    return { powf(c.x, 1.0f/2.2f), powf(c.y, 1.0f/2.2f), powf(c.z, 1.0f/2.2f) };
}

// ==================== 主渲染 ====================
int main() {
    const int W = 800, H = 600;
    std::vector<uint8_t> image(W * H * 3, 0);
    
    // 相机参数
    Vec3 camPos = Vec3(0.0f, 2.0f, 8.0f);
    Vec3 camTarget = Vec3(0.0f, 0.0f, 0.0f);
    Vec3 camUp = Vec3(0.0f, 1.0f, 0.0f);
    float fov = 50.0f;  // degrees
    
    Vec3 camFwd = (camTarget - camPos).normalized();
    Vec3 camRight = camFwd.cross(camUp).normalized();
    Vec3 camUpOrtho = camRight.cross(camFwd).normalized();
    float aspect = float(W) / float(H);
    float tanHalfFov = tanf(fov * 0.5f * 3.14159265f / 180.0f);
    
    // 光源
    Vec3 lightPos = Vec3(4.0f, 8.0f, 5.0f);
    Vec3 lightCol = Vec3(1.2f, 1.1f, 0.95f);
    
    // 副光源（填补阴影）
    Vec3 light2Pos = Vec3(-5.0f, 4.0f, 3.0f);
    Vec3 light2Col = Vec3(0.2f, 0.25f, 0.4f);
    
    int totalPixels = W * H;
    int reportInterval = totalPixels / 10;
    
    for(int y = 0; y < H; y++) {
        for(int x = 0; x < W; x++) {
            // 计算ray方向（NDC → camera space → world space）
            float nx = (2.0f * (x + 0.5f) / W - 1.0f) * aspect * tanHalfFov;
            float ny = (1.0f - 2.0f * (y + 0.5f) / H) * tanHalfFov;
            
            Vec3 rd = (camFwd + camRight * nx + camUpOrtho * ny).normalized();
            Vec3 ro = camPos;
            
            Vec3 color;
            
            // Ray march
            SDFResult hit = rayMarch(ro, rd);
            
            if(hit.dist > 0) {
                Vec3 pos = ro + rd * hit.dist;
                Vec3 nor = calcNormal(pos);
                Material mat = getMaterial(hit.matID);
                
                // 主光源着色
                color = shade(pos, nor, rd, mat, lightPos, lightCol);
                
                // 副光源（不带阴影，填补暗部）
                Vec3 l2Dir = (light2Pos - pos).normalized();
                float diff2 = clampf(nor.dot(l2Dir), 0.0f, 1.0f);
                float ao = calcAO(pos, nor);
                color += mat.albedo * (1.0f - mat.metallic) * light2Col * diff2 * 0.4f * ao;
                
                // 地面棋盘格纹理
                if(hit.matID == 0) {
                    float cx = floorf(pos.x) + floorf(pos.z);
                    float checker = fmodf(fabsf(cx), 2.0f) < 1.0f ? 1.0f : 0.85f;
                    color = color * checker;
                }
                
                // 雾效
                float fogFactor = expf(-hit.dist * 0.025f);
                Vec3 fogColor = Vec3(0.7f, 0.75f, 0.85f);
                color = mix3(fogColor, color, fogFactor);
                
            } else {
                // 天空
                color = skyColor(rd);
            }
            
            // 色调映射 + gamma
            color = tonemapACES(color);
            color = gammaCorrect(color);
            color = clamp3(color, 0.0f, 1.0f);
            
            int idx = (y * W + x) * 3;
            image[idx+0] = uint8_t(color.x * 255.0f);
            image[idx+1] = uint8_t(color.y * 255.0f);
            image[idx+2] = uint8_t(color.z * 255.0f);
        }
        
        if((y * W) % reportInterval == 0) {
            printf("Rendering: %.1f%%\n", 100.0f * y / H);
            fflush(stdout);
        }
    }
    
    printf("Rendering: 100.0%%\n");
    
    // 保存图片
    if(!stbi_write_png("sdf_scene_output.png", W, H, 3, image.data(), W*3)) {
        fprintf(stderr, "Error: failed to write output PNG\n");
        return 1;
    }
    
    printf("Output saved: sdf_scene_output.png (%dx%d)\n", W, H);
    return 0;
}
