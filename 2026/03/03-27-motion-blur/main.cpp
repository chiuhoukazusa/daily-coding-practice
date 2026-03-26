/**
 * Motion Blur Renderer
 * 
 * 技术要点：
 * - 软光栅渲染管线（顶点变换 → 光栅化 → 着色）
 * - 速度缓冲（Velocity Buffer）：存储每像素屏幕空间运动矢量
 * - 多物体运动：旋转的球/立方体 + 相机轨道运动
 * - 后处理运动模糊：沿运动矢量方向采样累积
 * - 输出对比：无模糊 / 运动模糊 / 速度可视化 / 合成对比图
 * 
 * 作者：Daily Coding Practice 2026-03-27
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <string>
#include <sstream>
#include <iostream>
#include <random>
#include <cassert>
#include <limits>
#include <functional>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ============================================================
// 数学基础
// ============================================================

struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return {x+v.x, y+v.y}; }
    Vec2 operator-(const Vec2& v) const { return {x-v.x, y-v.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
    float length() const { return std::sqrt(x*x + y*y); }
};

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 1e-6f ? *this / l : Vec3(0,1,0); }
};

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(1) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    Vec3 xyz() const { return {x, y, z}; }
};

// 4x4矩阵（列主序）
struct Mat4 {
    float m[4][4];
    Mat4() {
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) m[i][j]=(i==j)?1.f:0.f;
    }
    static Mat4 identity() { return Mat4(); }
    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        r.x = m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w;
        r.y = m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w;
        r.z = m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w;
        r.w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w;
        return r;
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++){
                r.m[i][j]=0;
                for(int k=0;k<4;k++) r.m[i][j]+=m[i][k]*o.m[k][j];
            }
        return r;
    }
};

Mat4 makePerspective(float fovY, float aspect, float near, float far) {
    Mat4 m;
    float tanHalf = std::tan(fovY * 0.5f);
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) m.m[i][j]=0;
    m.m[0][0] = 1.f / (aspect * tanHalf);
    m.m[1][1] = 1.f / tanHalf;
    m.m[2][2] = -(far + near) / (far - near);
    m.m[2][3] = -(2.f * far * near) / (far - near);
    m.m[3][2] = -1.f;
    return m;
}

Mat4 makeLookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = (center - eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);
    Mat4 m;
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
    m.m[2][0]=-f.x; m.m[2][1]=-f.y; m.m[2][2]=-f.z; m.m[2][3]=f.dot(eye);
    m.m[3][0]=0; m.m[3][1]=0; m.m[3][2]=0; m.m[3][3]=1;
    return m;
}

Mat4 makeRotationY(float angle) {
    Mat4 m;
    m.m[0][0]=std::cos(angle); m.m[0][2]=std::sin(angle);
    m.m[2][0]=-std::sin(angle); m.m[2][2]=std::cos(angle);
    return m;
}

Mat4 makeRotationX(float angle) {
    Mat4 m;
    m.m[1][1]=std::cos(angle); m.m[1][2]=-std::sin(angle);
    m.m[2][1]=std::sin(angle); m.m[2][2]=std::cos(angle);
    return m;
}

Mat4 makeRotationZ(float angle) {
    Mat4 m;
    m.m[0][0]=std::cos(angle); m.m[0][1]=-std::sin(angle);
    m.m[1][0]=std::sin(angle); m.m[1][1]=std::cos(angle);
    return m;
}

Mat4 makeTranslation(float tx, float ty, float tz) {
    Mat4 m;
    m.m[0][3]=tx; m.m[1][3]=ty; m.m[2][3]=tz;
    return m;
}

Mat4 makeScale(float sx, float sy, float sz) {
    Mat4 m;
    m.m[0][0]=sx; m.m[1][1]=sy; m.m[2][2]=sz;
    return m;
}

// ============================================================
// 图像与缓冲区
// ============================================================

struct Color {
    float r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(float r, float g, float b) : r(r), g(g), b(b) {}
    Color operator+(const Color& c) const { return {r+c.r, g+c.g, b+c.b}; }
    Color operator*(float t) const { return {r*t, g*t, b*t}; }
    Color clamp() const {
        return { std::max(0.f, std::min(1.f, r)),
                 std::max(0.f, std::min(1.f, g)),
                 std::max(0.f, std::min(1.f, b)) };
    }
};

struct Image {
    int width, height;
    std::vector<Color> pixels;
    Image(int w, int h) : width(w), height(h), pixels(w*h) {}
    Color& at(int x, int y) { return pixels[y*width + x]; }
    const Color& at(int x, int y) const { return pixels[y*width + x]; }
    void clear(Color c = Color()) { std::fill(pixels.begin(), pixels.end(), c); }
    bool save(const std::string& filename) const {
        std::vector<uint8_t> data(width*height*3);
        for(int i=0; i<width*height; i++){
            Color c = pixels[i].clamp();
            data[i*3+0] = static_cast<uint8_t>(c.r * 255.f);
            data[i*3+1] = static_cast<uint8_t>(c.g * 255.f);
            data[i*3+2] = static_cast<uint8_t>(c.b * 255.f);
        }
        return stbi_write_png(filename.c_str(), width, height, 3, data.data(), width*3) != 0;
    }
};

// 深度缓冲
struct DepthBuffer {
    int width, height;
    std::vector<float> data;
    DepthBuffer(int w, int h) : width(w), height(h), data(w*h, std::numeric_limits<float>::infinity()) {}
    float& at(int x, int y) { return data[y*width + x]; }
    void clear() { std::fill(data.begin(), data.end(), std::numeric_limits<float>::infinity()); }
};

// 速度缓冲（Motion Vector Buffer）
struct VelocityBuffer {
    int width, height;
    std::vector<Vec2> data;
    VelocityBuffer(int w, int h) : width(w), height(h), data(w*h) {}
    Vec2& at(int x, int y) { return data[y*width + x]; }
    const Vec2& at(int x, int y) const { return data[y*width + x]; }
    void clear() { std::fill(data.begin(), data.end(), Vec2()); }
};

// ============================================================
// 几何体生成
// ============================================================

struct Vertex {
    Vec3 pos;    // 模型空间
    Vec3 normal;
    Color color;
};

struct Triangle {
    Vertex v[3];
};

// 生成球体
std::vector<Triangle> makeSphere(int latSegs, int lonSegs, Color col) {
    std::vector<Triangle> tris;
    auto vtx = [&](int lat, int lon) -> Vertex {
        float theta = (float)lat / latSegs * M_PI;
        float phi   = (float)lon / lonSegs * 2.f * M_PI;
        Vec3 p = {std::sin(theta)*std::cos(phi),
                  std::cos(theta),
                  std::sin(theta)*std::sin(phi)};
        return Vertex{p, p, col};
    };
    for(int i=0;i<latSegs;i++){
        for(int j=0;j<lonSegs;j++){
            Vertex v00=vtx(i,j), v01=vtx(i,j+1), v10=vtx(i+1,j), v11=vtx(i+1,j+1);
            Triangle t1, t2;
            t1.v[0]=v00; t1.v[1]=v10; t1.v[2]=v11;
            t2.v[0]=v00; t2.v[1]=v11; t2.v[2]=v01;
            tris.push_back(t1);
            tris.push_back(t2);
        }
    }
    return tris;
}

// 生成立方体
std::vector<Triangle> makeCube(Color col) {
    std::vector<Triangle> tris;
    // 6 面，每面 2 个三角形
    struct Face { Vec3 n; std::array<Vec3,4> verts; };
    std::vector<Face> faces = {
        { Vec3( 0, 0, 1), {Vec3(-1,-1, 1),Vec3( 1,-1, 1),Vec3( 1, 1, 1),Vec3(-1, 1, 1)} },
        { Vec3( 0, 0,-1), {Vec3( 1,-1,-1),Vec3(-1,-1,-1),Vec3(-1, 1,-1),Vec3( 1, 1,-1)} },
        { Vec3( 1, 0, 0), {Vec3( 1,-1, 1),Vec3( 1,-1,-1),Vec3( 1, 1,-1),Vec3( 1, 1, 1)} },
        { Vec3(-1, 0, 0), {Vec3(-1,-1,-1),Vec3(-1,-1, 1),Vec3(-1, 1, 1),Vec3(-1, 1,-1)} },
        { Vec3( 0, 1, 0), {Vec3(-1, 1, 1),Vec3( 1, 1, 1),Vec3( 1, 1,-1),Vec3(-1, 1,-1)} },
        { Vec3( 0,-1, 0), {Vec3(-1,-1,-1),Vec3( 1,-1,-1),Vec3( 1,-1, 1),Vec3(-1,-1, 1)} },
    };
    for(auto& f : faces){
        Triangle t1, t2;
        t1.v[0]={f.verts[0],f.n,col}; t1.v[1]={f.verts[1],f.n,col}; t1.v[2]={f.verts[2],f.n,col};
        t2.v[0]={f.verts[0],f.n,col}; t2.v[1]={f.verts[2],f.n,col}; t2.v[2]={f.verts[3],f.n,col};
        tris.push_back(t1);
        tris.push_back(t2);
    }
    return tris;
}

// 生成地面平面
std::vector<Triangle> makeGround(Color col) {
    std::vector<Triangle> tris;
    float size = 6.f;
    Vec3 n{0,1,0};
    Triangle t1, t2;
    t1.v[0]={Vec3(-size,-1,-size),n,col};
    t1.v[1]={Vec3( size,-1,-size),n,col};
    t1.v[2]={Vec3( size,-1, size),n,col};
    t2.v[0]={Vec3(-size,-1,-size),n,col};
    t2.v[1]={Vec3( size,-1, size),n,col};
    t2.v[2]={Vec3(-size,-1, size),n,col};
    tris.push_back(t1);
    tris.push_back(t2);
    return tris;
}

// ============================================================
// 光照
// ============================================================

Color phongLighting(const Vec3& normal, const Vec3& viewDir, 
                    const Color& diffuseCol, const Vec3& fragPos) {
    Vec3 lightDir = Vec3(1.f, 2.f, 1.5f).normalized();
    Vec3 lightDir2 = Vec3(-1.5f, 1.f, 0.5f).normalized();
    
    // 环境光
    Color ambient = diffuseCol * 0.15f;
    
    // 漫反射 (两个光源)
    float diff1 = std::max(0.f, normal.dot(lightDir));
    float diff2 = std::max(0.f, normal.dot(lightDir2)) * 0.4f;
    Color diffuse = diffuseCol * (diff1 + diff2);
    
    // 镜面反射
    Vec3 halfVec = (lightDir + viewDir).normalized();
    float spec = std::pow(std::max(0.f, normal.dot(halfVec)), 32.f);
    Color specular = Color(1,1,1) * spec * 0.5f;
    
    // 简单阴影衰减（高度越低越暗）
    float shadowFactor = std::min(1.f, (fragPos.y + 1.f) * 0.5f + 0.5f);
    
    Color result = ambient + (diffuse + specular) * shadowFactor;
    return result.clamp();
}

// ============================================================
// 渲染器
// ============================================================

struct RenderContext {
    int width, height;
    Mat4 proj;
    Mat4 view;
    Mat4 prevProj;
    Mat4 prevView;
    Vec3 cameraPos;
    
    // 输出
    Image* colorBuffer;
    DepthBuffer* depthBuffer;
    VelocityBuffer* velocityBuffer;
};

// NDC -> 屏幕坐标
Vec2 ndcToScreen(float ndcX, float ndcY, int width, int height) {
    return { (ndcX * 0.5f + 0.5f) * width,
             (1.f - (ndcY * 0.5f + 0.5f)) * height };
}

// 重心坐标
bool barycentric(float px, float py,
                 float ax, float ay,
                 float bx, float by,
                 float cx, float cy,
                 float& u, float& v, float& w) {
    float denom = (by-cy)*(ax-cx) + (cx-bx)*(ay-cy);
    if(std::abs(denom) < 1e-8f) return false;
    u = ((by-cy)*(px-cx) + (cx-bx)*(py-cy)) / denom;
    v = ((cy-ay)*(px-cx) + (ax-cx)*(py-cy)) / denom;
    w = 1.f - u - v;
    return (u >= -1e-5f && v >= -1e-5f && w >= -1e-5f);
}

void renderMesh(RenderContext& ctx,
                const std::vector<Triangle>& mesh,
                const Mat4& model,
                const Mat4& prevModel) {
    Mat4 mvp     = ctx.proj * ctx.view * model;
    Mat4 prevMVP = ctx.prevProj * ctx.prevView * prevModel;
    // 法线矩阵（简化：只取旋转部分）
    Mat4 normalMat = ctx.view * model;
    
    for(const auto& tri : mesh){
        // 变换顶点到裁剪空间
        Vec4 clip[3], prevClip[3];
        Vec3 worldPos[3];
        for(int i=0;i<3;i++){
            clip[i]     = mvp * Vec4(tri.v[i].pos, 1.f);
            prevClip[i] = prevMVP * Vec4(tri.v[i].pos, 1.f);
            // 当前帧世界坐标（用于光照）
            Vec4 wp = model * Vec4(tri.v[i].pos, 1.f);
            worldPos[i] = wp.xyz();
        }
        
        // 视锥裁剪（简单近平面裁剪）
        bool visible = true;
        for(int i=0;i<3;i++){
            if(clip[i].w < 0.01f){ visible=false; break; }
        }
        if(!visible) continue;
        
        // NDC
        Vec2 ndc[3], prevNDC[3];
        float invW[3];
        for(int i=0;i<3;i++){
            invW[i] = 1.f / clip[i].w;
            ndc[i] = { clip[i].x * invW[i], clip[i].y * invW[i] };
            float pInvW = 1.f / (prevClip[i].w > 0.001f ? prevClip[i].w : 0.001f);
            prevNDC[i] = { prevClip[i].x * pInvW, prevClip[i].y * pInvW };
        }
        
        // 深度 (z/w, 用于插值)
        float ndcZ[3];
        for(int i=0;i<3;i++) ndcZ[i] = clip[i].z * invW[i];
        
        // 屏幕坐标
        Vec2 scr[3], prevScr[3];
        for(int i=0;i<3;i++){
            scr[i]     = ndcToScreen(ndc[i].x, ndc[i].y, ctx.width, ctx.height);
            prevScr[i] = ndcToScreen(prevNDC[i].x, prevNDC[i].y, ctx.width, ctx.height);
        }
        
        // 法线变换到视空间
        Vec3 viewNormals[3];
        for(int i=0;i<3;i++){
            Vec4 vn = normalMat * Vec4(tri.v[i].normal, 0.f);
            viewNormals[i] = vn.xyz().normalized();
        }
        
        // 背面剔除：用屏幕空间叉积
        Vec2 d1 = scr[1] - scr[0];
        Vec2 d2 = scr[2] - scr[0];
        float cross2d = d1.x * d2.y - d1.y * d2.x;
        if(cross2d >= 0.f) continue; // 背面（右手系屏幕）
        
        // 包围盒
        int minX = std::max(0, (int)std::floor(std::min({scr[0].x, scr[1].x, scr[2].x})));
        int maxX = std::min(ctx.width-1, (int)std::ceil(std::max({scr[0].x, scr[1].x, scr[2].x})));
        int minY = std::max(0, (int)std::floor(std::min({scr[0].y, scr[1].y, scr[2].y})));
        int maxY = std::min(ctx.height-1, (int)std::ceil(std::max({scr[0].y, scr[1].y, scr[2].y})));
        
        for(int py = minY; py <= maxY; py++){
            for(int px = minX; px <= maxX; px++){
                float u, v, w;
                if(!barycentric((float)px+0.5f, (float)py+0.5f,
                                scr[0].x, scr[0].y,
                                scr[1].x, scr[1].y,
                                scr[2].x, scr[2].y,
                                u, v, w)) continue;
                
                // 透视校正深度
                float depth = u*ndcZ[0] + v*ndcZ[1] + w*ndcZ[2];
                if(depth >= ctx.depthBuffer->at(px, py)) continue;
                ctx.depthBuffer->at(px, py) = depth;
                
                // 透视校正插值权重
                float wu = u*invW[0], wv = v*invW[1], ww = w*invW[2];
                float wSum = wu + wv + ww;
                if(wSum < 1e-8f) continue;
                float nu = wu/wSum, nv = wv/wSum, nw = ww/wSum;
                
                // 插值法线
                Vec3 norm = viewNormals[0]*nu + viewNormals[1]*nv + viewNormals[2]*nw;
                norm = norm.normalized();
                
                // 插值颜色
                Color col = tri.v[0].color*nu + tri.v[1].color*nv + tri.v[2].color*nw;
                
                // 插值世界坐标
                Vec3 wp = worldPos[0]*nu + worldPos[1]*nv + worldPos[2]*nw;
                
                // 视方向
                Vec3 viewDir = (ctx.cameraPos - wp).normalized();
                
                // 光照
                Color litColor = phongLighting(norm, viewDir, col, wp);
                ctx.colorBuffer->at(px, py) = litColor;
                
                // 计算运动矢量：当前帧屏幕坐标 - 上一帧屏幕坐标
                Vec2 curScr  = { scr[0].x*nu + scr[1].x*nv + scr[2].x*nw,
                                  scr[0].y*nu + scr[1].y*nv + scr[2].y*nw };
                Vec2 prevScrP = { prevScr[0].x*nu + prevScr[1].x*nv + prevScr[2].x*nw,
                                   prevScr[0].y*nu + prevScr[1].y*nv + prevScr[2].y*nw };
                Vec2 velocity = curScr - prevScrP;
                ctx.velocityBuffer->at(px, py) = velocity;
            }
        }
    }
}

// 后处理运动模糊
Image applyMotionBlur(const Image& src, const VelocityBuffer& vel, int numSamples = 16) {
    int w = src.width, h = src.height;
    Image result(w, h);
    
    for(int y=0; y<h; y++){
        for(int x=0; x<w; x++){
            Vec2 v = vel.at(x, y);
            float speed = v.length();
            
            // 速度太小不模糊
            if(speed < 0.5f){
                result.at(x,y) = src.at(x,y);
                continue;
            }
            
            // 沿速度方向采样
            Color sum;
            int validSamples = 0;
            for(int i=0; i<numSamples; i++){
                float t = ((float)i / (numSamples-1)) - 0.5f; // [-0.5, 0.5]
                int sx = (int)(x + v.x * t + 0.5f);
                int sy = (int)(y + v.y * t + 0.5f);
                if(sx >= 0 && sx < w && sy >= 0 && sy < h){
                    sum = sum + src.at(sx, sy);
                    validSamples++;
                }
            }
            if(validSamples > 0){
                result.at(x,y) = sum * (1.f / validSamples);
            } else {
                result.at(x,y) = src.at(x,y);
            }
        }
    }
    return result;
}

// 速度缓冲可视化
Image visualizeVelocity(const VelocityBuffer& vel, int width, int height) {
    Image img(width, height);
    // 找最大速度用于归一化
    float maxSpeed = 0.f;
    for(int y=0;y<height;y++)
        for(int x=0;x<width;x++)
            maxSpeed = std::max(maxSpeed, vel.at(x,y).length());
    if(maxSpeed < 0.001f) maxSpeed = 1.f;
    
    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            Vec2 v = vel.at(x,y);
            // 将速度向量映射到颜色：R=+X, G=+Y, B=speed
            float r = std::max(0.f, v.x) / maxSpeed;
            float g = std::max(0.f, v.y) / maxSpeed;
            float b = v.length() / maxSpeed;
            img.at(x,y) = Color(r, g, b);
        }
    }
    return img;
}

// ============================================================
// 场景设置
// ============================================================

struct SceneObject {
    std::vector<Triangle> mesh;
    std::function<Mat4(float)> modelFn;  // 给定时间返回模型矩阵
};

// ============================================================
// 主程序
// ============================================================

int main() {
    const int WIDTH  = 640;
    const int HEIGHT = 480;
    const float PI   = 3.14159265f;
    
    // 相机参数（当前帧时间 t=1.0，上一帧 t=0.0）
    float tCurr = 1.0f;  // 当前帧（模拟第1帧，快速运动）
    float tPrev = 0.0f;  // 上一帧
    
    // 摄像机轨道运动（缓慢旋转，模拟相机运动模糊）
    auto getCameraPos = [&](float t) -> Vec3 {
        float angle = t * 0.3f;  // 轨道速度
        return Vec3(5.f * std::cos(angle), 2.5f, 5.f * std::sin(angle));
    };
    Vec3 camPosCurr = getCameraPos(tCurr);
    Vec3 camPosPrev = getCameraPos(tPrev);
    Vec3 camTarget = Vec3(0, 0, 0);
    Vec3 camUp = Vec3(0, 1, 0);
    
    // 投影矩阵
    float aspect = (float)WIDTH / HEIGHT;
    Mat4 proj = makePerspective(60.f * PI / 180.f, aspect, 0.1f, 50.f);
    
    // 观察矩阵（当前帧和上一帧）
    Mat4 viewCurr = makeLookAt(camPosCurr, camTarget, camUp);
    Mat4 viewPrev = makeLookAt(camPosPrev, camTarget, camUp);
    
    // 场景物体
    // 1. 快速旋转的球 (橙色)
    auto sphereMesh = makeSphere(24, 32, Color(1.0f, 0.55f, 0.1f));
    auto sphereModelFn = [&](float t) -> Mat4 {
        float angle = t * 3.f;  // 快速旋转
        return makeTranslation(-1.5f, 0.2f, 0.f) * makeRotationY(angle) * makeScale(0.7f,0.7f,0.7f);
    };
    
    // 2. 旋转的立方体 (蓝色)
    auto cubeMesh = makeCube(Color(0.2f, 0.5f, 1.0f));
    auto cubeModelFn = [&](float t) -> Mat4 {
        float angle = t * 2.f;
        float angle2 = t * 1.5f;
        return makeTranslation(1.5f, 0.0f, 0.f) * makeRotationY(angle) * makeRotationX(angle2) * makeScale(0.6f,0.6f,0.6f);
    };
    
    // 3. 绕轨道运动的小球 (绿色)
    auto orbitMesh = makeSphere(16, 24, Color(0.2f, 0.9f, 0.3f));
    auto orbitModelFn = [&](float t) -> Mat4 {
        float angle = t * 4.f;  // 非常快的轨道运动
        float ox = 2.8f * std::cos(angle);
        float oz = 2.8f * std::sin(angle);
        return makeTranslation(ox, -0.3f, oz) * makeScale(0.35f, 0.35f, 0.35f);
    };
    
    // 4. 地面 (灰色)
    auto groundMesh = makeGround(Color(0.35f, 0.35f, 0.38f));
    auto groundModelFn = [](float) -> Mat4 { return Mat4::identity(); };
    
    // 背景颜色（天空渐变）
    auto skyColor = [](int y, int height) -> Color {
        float t = 1.f - (float)y / height;
        t = std::max(0.f, std::min(1.f, t));
        // 天空蓝到地平线
        Color sky(0.2f + 0.4f*t, 0.4f + 0.3f*t, 0.7f + 0.2f*t);
        return sky;
    };
    
    // ---- 渲染当前帧（无模糊）----
    std::cout << "渲染当前帧 (无模糊)..." << std::endl;
    Image colorBuf(WIDTH, HEIGHT);
    DepthBuffer depthBuf(WIDTH, HEIGHT);
    VelocityBuffer velBuf(WIDTH, HEIGHT);
    
    // 填充天空背景
    for(int y=0;y<HEIGHT;y++)
        for(int x=0;x<WIDTH;x++)
            colorBuf.at(x,y) = skyColor(y, HEIGHT);
    
    RenderContext ctx;
    ctx.width = WIDTH; ctx.height = HEIGHT;
    ctx.proj = proj; ctx.view = viewCurr;
    ctx.prevProj = proj; ctx.prevView = viewPrev;
    ctx.cameraPos = camPosCurr;
    ctx.colorBuffer = &colorBuf;
    ctx.depthBuffer = &depthBuf;
    ctx.velocityBuffer = &velBuf;
    
    // 渲染所有物体
    renderMesh(ctx, groundMesh,  groundModelFn(tCurr), groundModelFn(tPrev));
    renderMesh(ctx, sphereMesh,  sphereModelFn(tCurr), sphereModelFn(tPrev));
    renderMesh(ctx, cubeMesh,    cubeModelFn(tCurr),   cubeModelFn(tPrev));
    renderMesh(ctx, orbitMesh,   orbitModelFn(tCurr),  orbitModelFn(tPrev));
    
    // 保存无模糊图
    colorBuf.save("motion_blur_noamb.png");
    std::cout << "  → 保存: motion_blur_noamb.png" << std::endl;
    
    // ---- 速度缓冲可视化 ----
    Image velViz = visualizeVelocity(velBuf, WIDTH, HEIGHT);
    velViz.save("motion_blur_velocity.png");
    std::cout << "  → 保存: motion_blur_velocity.png" << std::endl;
    
    // ---- 运动模糊后处理 ----
    std::cout << "应用运动模糊..." << std::endl;
    Image blurred = applyMotionBlur(colorBuf, velBuf, 24);
    blurred.save("motion_blur_output.png");
    std::cout << "  → 保存: motion_blur_output.png" << std::endl;
    
    // ---- 合成对比图 (左：无模糊 | 右：有模糊) ----
    std::cout << "生成对比图..." << std::endl;
    Image compare(WIDTH*2, HEIGHT);
    for(int y=0;y<HEIGHT;y++){
        for(int x=0;x<WIDTH;x++){
            compare.at(x, y)       = colorBuf.at(x, y);  // 左：原图
            compare.at(x+WIDTH, y) = blurred.at(x, y);   // 右：模糊
        }
        // 分隔线
        compare.at(WIDTH-1, y) = Color(1,1,0);
        compare.at(WIDTH,   y) = Color(1,1,0);
    }
    compare.save("motion_blur_compare.png");
    std::cout << "  → 保存: motion_blur_compare.png" << std::endl;
    
    // ---- 验证输出 ----
    std::cout << "\n=== 输出验证 ===" << std::endl;
    
    // 统计主输出图像像素
    {
        double sumR=0, sumG=0, sumB=0;
        int count = 0;
        for(int y=0;y<HEIGHT;y++){
            for(int x=0;x<WIDTH;x++){
                Color c = blurred.at(x,y).clamp();
                sumR += c.r; sumG += c.g; sumB += c.b;
                count++;
            }
        }
        float meanR = sumR/count, meanG = sumG/count, meanB = sumB/count;
        float mean = (meanR + meanG + meanB) / 3.f * 255.f;
        
        double varSum=0;
        for(int y=0;y<HEIGHT;y++){
            for(int x=0;x<WIDTH;x++){
                Color c = blurred.at(x,y).clamp();
                float lum = (c.r+c.g+c.b)/3.f*255.f;
                varSum += (lum-mean)*(lum-mean);
            }
        }
        float stddev = std::sqrt(varSum / count);
        
        std::cout << "像素均值: " << mean << "  标准差: " << stddev << std::endl;
        
        bool ok = true;
        if(mean < 10 || mean > 240){
            std::cout << "❌ 均值异常: " << mean << std::endl; ok=false;
        }
        if(stddev < 5){
            std::cout << "❌ 标准差过低: " << stddev << std::endl; ok=false;
        }
        if(ok) std::cout << "✅ 像素统计正常" << std::endl;
    }
    
    // 验证速度缓冲有非零值
    {
        int nonZeroVel = 0;
        float maxVelMag = 0.f;
        for(int y=0;y<HEIGHT;y++){
            for(int x=0;x<WIDTH;x++){
                float mag = velBuf.at(x,y).length();
                if(mag > 0.5f) nonZeroVel++;
                maxVelMag = std::max(maxVelMag, mag);
            }
        }
        std::cout << "速度缓冲非零像素: " << nonZeroVel 
                  << " / " << (WIDTH*HEIGHT) 
                  << " 最大速度幅度: " << maxVelMag << " px" << std::endl;
        if(nonZeroVel < 1000){
            std::cout << "❌ 速度缓冲几乎为空，运动模糊效果无效" << std::endl;
        } else {
            std::cout << "✅ 速度缓冲有效 (" << nonZeroVel << " 个运动像素)" << std::endl;
        }
    }
    
    // 验证模糊效果（与原图差异）
    {
        double diffSum = 0;
        int count = 0;
        for(int y=0;y<HEIGHT;y++){
            for(int x=0;x<WIDTH;x++){
                Color o = colorBuf.at(x,y).clamp();
                Color b = blurred.at(x,y).clamp();
                float d = std::abs(o.r-b.r) + std::abs(o.g-b.g) + std::abs(o.b-b.b);
                diffSum += d;
                if(d > 0.01f) count++;
            }
        }
        float avgDiff = diffSum / (WIDTH*HEIGHT) * 255.f;
        std::cout << "模糊差异: 均值=" << avgDiff << " 影响像素=" << count << std::endl;
        if(avgDiff < 0.5f){
            std::cout << "❌ 模糊效果太弱" << std::endl;
        } else {
            std::cout << "✅ 模糊效果明显 (均值差异=" << avgDiff << ")" << std::endl;
        }
    }
    
    std::cout << "\n✅ 所有输出文件已保存！" << std::endl;
    std::cout << "  - motion_blur_noamb.png   (无运动模糊)" << std::endl;
    std::cout << "  - motion_blur_velocity.png (速度缓冲可视化)" << std::endl;
    std::cout << "  - motion_blur_output.png   (运动模糊效果)" << std::endl;
    std::cout << "  - motion_blur_compare.png  (左右对比)" << std::endl;
    
    return 0;
}
