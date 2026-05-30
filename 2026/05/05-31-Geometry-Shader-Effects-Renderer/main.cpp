/**
 * Geometry Shader Effects Renderer
 * 
 * 软光栅化实现几何着色器效果：
 * - 线框叠加（Wireframe Overlay）：使用重心坐标检测三角形边缘
 * - 法线可视化（Normal Visualization）：在每个顶点/面绘制法线方向
 * - 切线空间可视化（Tangent Space）：显示 TBN 三元组
 * - 多 Pass 渲染：先实体，再叠加线框和法线
 *
 * 输出：geometry_shader_output.png（800x600）
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <limits>
#include <string>
#include <sstream>
#include <iostream>

// ── 数学工具 ────────────────────────────────────────────────────

struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        float l = length();
        return l > 1e-8f ? (*this / l) : Vec3{0,0,0};
    }
};

struct Vec4 {
    float x, y, z, w;
};

inline Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

struct Mat4 {
    float m[4][4]{};
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1.f; return r;
    }
    Vec4 transform(Vec4 v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++)
            for(int k=0;k<4;k++) r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 perspective(float fovY, float aspect, float near, float far) {
    Mat4 r;
    float t = std::tan(fovY * 0.5f);
    r.m[0][0] = 1.f/(aspect*t);
    r.m[1][1] = 1.f/t;
    r.m[2][2] = -(far+near)/(far-near);
    r.m[2][3] = -2.f*far*near/(far-near);
    r.m[3][2] = -1.f;
    return r;
}

Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = (center - eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);
    Mat4 m;
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-(r.dot(eye));
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-(u.dot(eye));
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]= f.dot(eye);
    m.m[3][3]=1.f;
    return m;
}

Mat4 rotateY(float angle) {
    Mat4 r = Mat4::identity();
    float c=std::cos(angle), s=std::sin(angle);
    r.m[0][0]=c; r.m[0][2]=s; r.m[2][0]=-s; r.m[2][2]=c;
    return r;
}

// ── 几何体定义 ───────────────────────────────────────────────────

struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec2 uv;
};

struct Triangle {
    Vertex v[3];
};

// 生成一个细分球体（UV Sphere）
std::vector<Triangle> generateSphere(int stacks, int slices, float radius) {
    std::vector<Vertex> verts;
    const float PI = 3.14159265f;
    for(int i=0; i<=stacks; i++) {
        float phi = PI * i / stacks;  // 0..PI
        for(int j=0; j<=slices; j++) {
            float theta = 2*PI * j / slices;  // 0..2PI
            Vertex v;
            v.normal = {std::sin(phi)*std::cos(theta),
                        std::cos(phi),
                        std::sin(phi)*std::sin(theta)};
            v.pos    = v.normal * radius;
            v.uv     = {(float)j/slices, (float)i/stacks};
            verts.push_back(v);
        }
    }
    std::vector<Triangle> tris;
    for(int i=0; i<stacks; i++) {
        for(int j=0; j<slices; j++) {
            int a = i*(slices+1)+j;
            int b = a+1;
            int c = a+(slices+1);
            int d = c+1;
            Triangle t1, t2;
            t1.v[0]=verts[a]; t1.v[1]=verts[b]; t1.v[2]=verts[c];
            t2.v[0]=verts[b]; t2.v[1]=verts[d]; t2.v[2]=verts[c];
            tris.push_back(t1);
            tris.push_back(t2);
        }
    }
    return tris;
}

// 生成立方体（带法线，每面独立）
std::vector<Triangle> generateCube(float size) {
    float h = size * 0.5f;
    struct Face { Vec3 n; Vec3 pts[4]; };
    std::vector<Face> faces = {
        { {0,0,1}, { {-h,-h,h},{h,-h,h},{h,h,h},{-h,h,h} } },
        { {0,0,-1},{ {h,-h,-h},{-h,-h,-h},{-h,h,-h},{h,h,-h} } },
        { {0,1,0}, { {-h,h,-h},{h,h,-h},{h,h,h},{-h,h,h} } },
        { {0,-1,0},{ {-h,-h,h},{h,-h,h},{h,-h,-h},{-h,-h,-h} } },
        { {1,0,0}, { {h,-h,h},{h,-h,-h},{h,h,-h},{h,h,h} } },
        { {-1,0,0},{ {-h,-h,-h},{-h,-h,h},{-h,h,h},{-h,h,-h} } },
    };
    std::vector<Triangle> tris;
    for(auto& f : faces) {
        Triangle t1, t2;
        // quad -> 2 tri 划分 (0,1,2) + (0,2,3)
        t1.v[0].pos=f.pts[0]; t1.v[0].normal=f.n; t1.v[0].uv={0,0};
        t1.v[1].pos=f.pts[1]; t1.v[1].normal=f.n; t1.v[1].uv={1,0};
        t1.v[2].pos=f.pts[2]; t1.v[2].normal=f.n; t1.v[2].uv={1,1};
        t2.v[0].pos=f.pts[0]; t2.v[0].normal=f.n; t2.v[0].uv={0,0};
        t2.v[1].pos=f.pts[2]; t2.v[1].normal=f.n; t2.v[1].uv={1,1};
        t2.v[2].pos=f.pts[3]; t2.v[2].normal=f.n; t2.v[2].uv={0,1};
        tris.push_back(t1); tris.push_back(t2);
    }
    return tris;
}

// ── 帧缓冲 ─────────────────────────────────────────────────────

const int W = 800, H = 600;

struct Color { uint8_t r, g, b; };

struct Framebuffer {
    std::vector<Color> color;
    std::vector<float> depth;
    Framebuffer() : color(W*H, {0,0,0}), depth(W*H, std::numeric_limits<float>::infinity()) {}
    void clear(Color bg = {20, 20, 35}) {
        std::fill(color.begin(), color.end(), bg);
        std::fill(depth.begin(), depth.end(), std::numeric_limits<float>::infinity());
    }
    void setPixel(int x, int y, Color c, float d = -std::numeric_limits<float>::infinity()) {
        if(x<0||x>=W||y<0||y>=H) return;
        int idx = y*W+x;
        if(d <= depth[idx]) { color[idx]=c; depth[idx]=d; }
    }
    // 叠加混合（不做深度测试，用于线框/法线）
    void blendPixel(int x, int y, Color c, float alpha=1.f) {
        if(x<0||x>=W||y<0||y>=H) return;
        int idx = y*W+x;
        color[idx].r = uint8_t(color[idx].r*(1-alpha) + c.r*alpha);
        color[idx].g = uint8_t(color[idx].g*(1-alpha) + c.g*alpha);
        color[idx].b = uint8_t(color[idx].b*(1-alpha) + c.b*alpha);
    }
};

// ── 投影工具 ───────────────────────────────────────────────────

struct ScreenVert {
    Vec3 world;      // 世界坐标
    Vec3 normal;     // 世界法线
    Vec2 screen;     // 像素坐标
    float depth;
};

ScreenVert projectVertex(Vec3 wp, Vec3 wn, const Mat4& MVP, const Mat4& modelMat) {
    Vec4 v = MVP.transform({wp.x, wp.y, wp.z, 1.f});
    float invW = 1.f / v.w;
    float ndcX = v.x * invW, ndcY = v.y * invW, ndcZ = v.z * invW;
    float sx = (ndcX * 0.5f + 0.5f) * W;
    float sy = (1.f - (ndcY * 0.5f + 0.5f)) * H;
    // 变换法线（简化：只旋转，不考虑非均匀缩放）
    Vec4 wn4 = modelMat.transform({wn.x, wn.y, wn.z, 0.f});
    Vec3 worldN = Vec3{wn4.x, wn4.y, wn4.z}.normalized();
    // 计算世界坐标
    Vec4 wp4 = modelMat.transform({wp.x, wp.y, wp.z, 1.f});
    return { {wp4.x, wp4.y, wp4.z}, worldN, {sx, sy}, ndcZ };
}

// ── 光栅化三角形（Phong 着色）─────────────────────────────────

void drawLine(Framebuffer& fb, int x0, int y0, int x1, int y1, Color c, float alpha=0.9f) {
    int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx-dy;
    while(true) {
        fb.blendPixel(x0, y0, c, alpha);
        if(x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if(e2>-dy) { err-=dy; x0+=sx; }
        if(e2< dx) { err+=dx; y0+=sy; }
    }
}

Color phongShading(Vec3 worldPos, Vec3 normal, Vec3 baseColor) {
    Vec3 lightPos  = {3.f, 5.f, 4.f};
    Vec3 lightColor = {1.f, 0.95f, 0.85f};
    Vec3 ambient   = {0.1f, 0.1f, 0.15f};
    Vec3 eyePos    = {0.f, 2.f, 6.f};

    Vec3 L = (lightPos - worldPos).normalized();
    Vec3 V = (eyePos - worldPos).normalized();
    Vec3 R = (normal * (2.f * normal.dot(L)) - L).normalized();

    float diff = std::max(0.f, normal.dot(L));
    float spec = std::pow(std::max(0.f, R.dot(V)), 32.f);

    Vec3 col = {
        baseColor.x * (ambient.x + lightColor.x * diff) + lightColor.x * spec * 0.5f,
        baseColor.y * (ambient.y + lightColor.y * diff) + lightColor.y * spec * 0.5f,
        baseColor.z * (ambient.z + lightColor.z * diff) + lightColor.z * spec * 0.5f
    };
    auto clamp01 = [](float v){ return std::max(0.f, std::min(1.f, v)); };
    return {
        uint8_t(clamp01(col.x)*255),
        uint8_t(clamp01(col.y)*255),
        uint8_t(clamp01(col.z)*255)
    };
}

// 重心坐标
Vec3 barycentricCoords(Vec2 p, Vec2 a, Vec2 b, Vec2 c) {
    float denom = (b.y-c.y)*(a.x-c.x) + (c.x-b.x)*(a.y-c.y);
    if(std::abs(denom) < 1e-8f) return {-1,0,0};
    float u = ((b.y-c.y)*(p.x-c.x) + (c.x-b.x)*(p.y-c.y)) / denom;
    float v = ((c.y-a.y)*(p.x-c.x) + (a.x-c.x)*(p.y-c.y)) / denom;
    float w = 1.f - u - v;
    return {u, v, w};
}

// 计算三角形边缘距离（用于线框效果）
// 返回到最近边的距离（像素）
float edgeDistance(Vec3 bary, Vec2 va, Vec2 vb, Vec2 vc) {
    // 三个边的距离（用三角形面积公式）
    auto edgeDist = [](Vec2 p, Vec2 a, Vec2 b) -> float {
        Vec2 ab = {b.x-a.x, b.y-a.y};
        Vec2 ap = {p.x-a.x, p.y-a.y};
        float len = std::sqrt(ab.x*ab.x + ab.y*ab.y);
        if(len < 1e-6f) return std::sqrt(ap.x*ap.x + ap.y*ap.y);
        return std::abs(ab.x*ap.y - ab.y*ap.x) / len;
    };
    // 像素坐标
    Vec2 p = {va.x*bary.x + vb.x*bary.y + vc.x*bary.z,
              va.y*bary.x + vb.y*bary.y + vc.y*bary.z};
    float d0 = edgeDist(p, vb, vc);
    float d1 = edgeDist(p, va, vc);
    float d2 = edgeDist(p, va, vb);
    return std::min({d0, d1, d2});
}

struct RenderOptions {
    bool wireframe   = true;
    bool showNormals = true;
    bool showTangent = false;
    float wireWidth  = 1.2f;
    float normalLen  = 0.25f;  // 世界空间法线可视化长度
    Vec3 baseColor   = {0.7f, 0.5f, 0.3f};
};

// 渲染三角形（实体 + 线框叠加）
void rasterizeTriangle(Framebuffer& fb,
                       ScreenVert sv[3],
                       const RenderOptions& opts) {
    // 包围盒
    float minX = std::min({sv[0].screen.x, sv[1].screen.x, sv[2].screen.x});
    float maxX = std::max({sv[0].screen.x, sv[1].screen.x, sv[2].screen.x});
    float minY = std::min({sv[0].screen.y, sv[1].screen.y, sv[2].screen.y});
    float maxY = std::max({sv[0].screen.y, sv[1].screen.y, sv[2].screen.y});

    int ixMin = std::max(0, (int)std::floor(minX));
    int ixMax = std::min(W-1, (int)std::ceil(maxX));
    int iyMin = std::max(0, (int)std::floor(minY));
    int iyMax = std::min(H-1, (int)std::ceil(maxY));

    for(int py = iyMin; py <= iyMax; py++) {
        for(int px = ixMin; px <= ixMax; px++) {
            Vec2 p = {px + 0.5f, py + 0.5f};
            Vec3 bc = barycentricCoords(p, sv[0].screen, sv[1].screen, sv[2].screen);
            if(bc.x < 0 || bc.y < 0 || bc.z < 0) continue;

            float depth = sv[0].depth*bc.x + sv[1].depth*bc.y + sv[2].depth*bc.z;

            // 插值世界坐标和法线
            Vec3 wpos = sv[0].world*bc.x + sv[1].world*bc.y + sv[2].world*bc.z;
            Vec3 wnorm = (sv[0].normal*bc.x + sv[1].normal*bc.y + sv[2].normal*bc.z).normalized();

            // 基础 Phong 着色
            Color shade = phongShading(wpos, wnorm, opts.baseColor);

            // 线框叠加：计算到最近边的距离
            if(opts.wireframe) {
                float ed = edgeDistance(bc, sv[0].screen, sv[1].screen, sv[2].screen);
                float wireAlpha = 1.f - std::min(1.f, ed / opts.wireWidth);
                wireAlpha = wireAlpha * wireAlpha;  // 加锐利化
                if(wireAlpha > 0.05f) {
                    Color wireColor = {230, 230, 255};
                    shade.r = uint8_t(shade.r * (1-wireAlpha) + wireColor.r * wireAlpha);
                    shade.g = uint8_t(shade.g * (1-wireAlpha) + wireColor.g * wireAlpha);
                    shade.b = uint8_t(shade.b * (1-wireAlpha) + wireColor.b * wireAlpha);
                }
            }

            fb.setPixel(px, py, shade, depth);
        }
    }
}

// 在世界坐标绘制法线箭头（投影到屏幕）
void drawNormalArrow(Framebuffer& fb,
                     Vec3 worldPos, Vec3 worldNormal,
                     float len, Color col,
                     const Mat4& VP) {
    Vec3 endPos = worldPos + worldNormal * len;

    auto toScreen = [&](Vec3 wp) -> Vec2 {
        Vec4 v = VP.transform({wp.x, wp.y, wp.z, 1.f});
        if(std::abs(v.w) < 1e-8f) return {-1,-1};
        float invW = 1.f / v.w;
        return {
            (v.x*invW * 0.5f + 0.5f) * W,
            (1.f - (v.y*invW * 0.5f + 0.5f)) * H
        };
    };

    Vec2 s0 = toScreen(worldPos);
    Vec2 s1 = toScreen(endPos);
    drawLine(fb, (int)s0.x, (int)s0.y, (int)s1.x, (int)s1.y, col, 0.95f);
}

// ── 主函数 ─────────────────────────────────────────────────────

int main() {
    Framebuffer fb;
    fb.clear({18, 18, 30});

    // ── 相机/变换矩阵 ─────────────────────────────────────────
    Vec3 eyePos = {0.f, 2.f, 6.f};
    Mat4 view   = lookAt(eyePos, {0,0,0}, {0,1,0});
    Mat4 proj   = perspective(45.f * 3.14159f / 180.f, float(W)/H, 0.1f, 100.f);
    Mat4 VP     = proj * view;

    const float PI = 3.14159265f;

    // ── 对象 1：中央球体（Phong + 线框）─────────────────────────
    {
        Mat4 model = rotateY(PI * 0.25f);
        Mat4 MVP   = VP * model;

        auto sphere = generateSphere(16, 24, 1.2f);

        RenderOptions opts;
        opts.wireframe  = true;
        opts.wireWidth  = 1.5f;
        opts.baseColor  = {0.6f, 0.7f, 0.9f};  // 蓝紫色球体

        for(auto& tri : sphere) {
            ScreenVert sv[3];
            for(int k=0;k<3;k++) {
                sv[k] = projectVertex(tri.v[k].pos, tri.v[k].normal, MVP, model);
            }
            // 背面剔除
            Vec2 ab = {sv[1].screen.x - sv[0].screen.x, sv[1].screen.y - sv[0].screen.y};
            Vec2 ac = {sv[2].screen.x - sv[0].screen.x, sv[2].screen.y - sv[0].screen.y};
            if(ab.x*ac.y - ab.y*ac.x < 0) continue;  // 顺时针 → 背面
            rasterizeTriangle(fb, sv, opts);
        }

        // 法线可视化
        for(auto& tri : sphere) {
            // 取三角形重心
            Vec3 centroid = (tri.v[0].pos + tri.v[1].pos + tri.v[2].pos) / 3.f;
            Vec4 cn = model.transform({tri.v[0].normal.x, tri.v[0].normal.y, tri.v[0].normal.z, 0.f});
            Vec3 worldNorm = Vec3{cn.x, cn.y, cn.z}.normalized();
            Vec4 cp = model.transform({centroid.x, centroid.y, centroid.z, 1.f});
            Vec3 worldCentroid = {cp.x, cp.y, cp.z};
            // 只绘制面朝摄像机的法线
            Vec3 toEye = (eyePos - worldCentroid).normalized();
            if(worldNorm.dot(toEye) > 0.1f)
                drawNormalArrow(fb, worldCentroid, worldNorm, 0.18f,
                                {100, 255, 120}, VP);
        }
    }

    // ── 对象 2：左侧立方体（线框 + 切线空间）────────────────────
    {
        // 平移矩阵
        Mat4 model = Mat4::identity();
        model.m[0][3] = -2.8f;
        model.m[1][3] =  0.0f;
        model.m[2][3] = -0.5f;
        Mat4 rot = rotateY(-PI * 0.2f);
        model = model * rot;
        Mat4 MVP = VP * model;

        auto cube = generateCube(1.6f);

        RenderOptions opts;
        opts.wireframe = true;
        opts.wireWidth = 1.8f;
        opts.baseColor = {0.85f, 0.5f, 0.3f};  // 橙色立方体

        for(auto& tri : cube) {
            ScreenVert sv[3];
            for(int k=0;k<3;k++) {
                sv[k] = projectVertex(tri.v[k].pos, tri.v[k].normal, MVP, model);
            }
            Vec2 ab = {sv[1].screen.x - sv[0].screen.x, sv[1].screen.y - sv[0].screen.y};
            Vec2 ac = {sv[2].screen.x - sv[0].screen.x, sv[2].screen.y - sv[0].screen.y};
            if(ab.x*ac.y - ab.y*ac.x < 0) continue;
            rasterizeTriangle(fb, sv, opts);
        }

        // 面法线（每面一条）—— 每相邻两个 tri 合并为一面
        for(size_t i = 0; i+1 < cube.size(); i += 2) {
            // 取面中心
            Vec3 centroid = {0,0,0};
            for(int k=0;k<3;k++) centroid = centroid + cube[i].v[k].pos;
            for(int k=0;k<3;k++) centroid = centroid + cube[i+1].v[k].pos;
            centroid = centroid / 6.f;

            Vec3 faceNorm = cube[i].v[0].normal;
            Vec4 cn4 = model.transform({faceNorm.x, faceNorm.y, faceNorm.z, 0.f});
            Vec3 worldNorm = Vec3{cn4.x, cn4.y, cn4.z}.normalized();
            Vec4 cp4 = model.transform({centroid.x, centroid.y, centroid.z, 1.f});
            Vec3 worldCentroid = {cp4.x, cp4.y, cp4.z};

            Vec3 toEye = (eyePos - worldCentroid).normalized();
            if(worldNorm.dot(toEye) > 0.05f) {
                // 法线（蓝色）
                drawNormalArrow(fb, worldCentroid, worldNorm, 0.35f, {80, 160, 255}, VP);

                // 切线（红色）—— 简单地取与法线垂直的世界 Up 方向投影
                Vec3 up = {0,1,0};
                Vec3 tangent = (up - worldNorm * worldNorm.dot(up)).normalized();
                if(tangent.length() > 0.1f)
                    drawNormalArrow(fb, worldCentroid, tangent, 0.28f, {255, 80, 80}, VP);

                // 副切线（绿色）
                Vec3 bitangent = worldNorm.cross(tangent).normalized();
                drawNormalArrow(fb, worldCentroid, bitangent, 0.28f, {80, 220, 80}, VP);
            }
        }
    }

    // ── 对象 3：右侧球体（纯法线可视化，颜色 = 法线方向）──────────
    {
        Mat4 model = Mat4::identity();
        model.m[0][3] = 2.8f;
        model.m[1][3] = 0.0f;
        model.m[2][3] = -0.5f;
        Mat4 rot = rotateY(PI * 0.15f);
        model = model * rot;
        Mat4 MVP = VP * model;

        auto sphere = generateSphere(20, 30, 1.0f);

        for(auto& tri : sphere) {
            ScreenVert sv[3];
            for(int k=0;k<3;k++) {
                sv[k] = projectVertex(tri.v[k].pos, tri.v[k].normal, MVP, model);
            }
            Vec2 ab = {sv[1].screen.x - sv[0].screen.x, sv[1].screen.y - sv[0].screen.y};
            Vec2 ac = {sv[2].screen.x - sv[0].screen.x, sv[2].screen.y - sv[0].screen.y};
            if(ab.x*ac.y - ab.y*ac.x < 0) continue;

            // 法线着色（直接将法线 [-1,1] 映射到颜色 [0,255]）
            int ixMin = std::max(0, (int)std::min({sv[0].screen.x, sv[1].screen.x, sv[2].screen.x}));
            int ixMax = std::min(W-1, (int)std::max({sv[0].screen.x, sv[1].screen.x, sv[2].screen.x})+1);
            int iyMin = std::max(0, (int)std::min({sv[0].screen.y, sv[1].screen.y, sv[2].screen.y}));
            int iyMax = std::min(H-1, (int)std::max({sv[0].screen.y, sv[1].screen.y, sv[2].screen.y})+1);

            for(int py = iyMin; py <= iyMax; py++) {
                for(int px = ixMin; px <= ixMax; px++) {
                    Vec2 p = {px + 0.5f, py + 0.5f};
                    Vec3 bc = barycentricCoords(p, sv[0].screen, sv[1].screen, sv[2].screen);
                    if(bc.x < 0 || bc.y < 0 || bc.z < 0) continue;
                    float depth = sv[0].depth*bc.x + sv[1].depth*bc.y + sv[2].depth*bc.z;
                    Vec3 wnorm = (sv[0].normal*bc.x + sv[1].normal*bc.y + sv[2].normal*bc.z).normalized();

                    // 线框
                    float ed = edgeDistance(bc, sv[0].screen, sv[1].screen, sv[2].screen);
                    float wa = 1.f - std::min(1.f, ed / 1.2f);
                    wa = wa * wa;

                    Color normColor = {
                        uint8_t((wnorm.x * 0.5f + 0.5f) * 255),
                        uint8_t((wnorm.y * 0.5f + 0.5f) * 255),
                        uint8_t((wnorm.z * 0.5f + 0.5f) * 255)
                    };

                    if(wa > 0.05f) {
                        normColor.r = uint8_t(normColor.r * (1-wa) + 230 * wa);
                        normColor.g = uint8_t(normColor.g * (1-wa) + 230 * wa);
                        normColor.b = uint8_t(normColor.b * (1-wa) + 255 * wa);
                    }

                    fb.setPixel(px, py, normColor, depth);
                }
            }
        }
    }

    // ── 地面网格 ────────────────────────────────────────────────
    {
        // 用折线绘制地面格子
        int gridSize = 8;
        float step = 0.8f;
        float yGround = -1.2f;

        auto toScreen = [&](Vec3 wp) -> Vec2 {
            Vec4 v = VP.transform({wp.x, wp.y, wp.z, 1.f});
            if(std::abs(v.w) < 1e-8f) return {-1,-1};
            float iw = 1.f / v.w;
            return {
                (v.x*iw * 0.5f + 0.5f) * W,
                (1.f - (v.y*iw * 0.5f + 0.5f)) * H
            };
        };

        Color gridColor = {50, 55, 80};
        float origin = -gridSize * step * 0.5f;
        for(int i = 0; i <= gridSize; i++) {
            float x = origin + i * step;
            float z0 = origin, z1 = origin + gridSize * step;
            Vec2 s0 = toScreen({x, yGround, z0});
            Vec2 s1 = toScreen({x, yGround, z1});
            drawLine(fb, (int)s0.x, (int)s0.y, (int)s1.x, (int)s1.y, gridColor, 0.5f);
        }
        for(int j = 0; j <= gridSize; j++) {
            float z = origin + j * step;
            float x0 = origin, x1 = origin + gridSize * step;
            Vec2 s0 = toScreen({x0, yGround, z});
            Vec2 s1 = toScreen({x1, yGround, z});
            drawLine(fb, (int)s0.x, (int)s0.y, (int)s1.x, (int)s1.y, gridColor, 0.5f);
        }
    }

    // ── 图例标注（简单像素文字 → 色块+色柱）────────────────────
    // 在图像右下角绘制简单图例色条
    {
        // 线框颜色示例
        int x = W - 150, y = H - 90;
        for(int i=0;i<18;i++) for(int j=0;j<8;j++)
            fb.blendPixel(x+i, y+j, {230,230,255}, 0.9f);
        // 法线颜色示例
        y += 14;
        for(int i=0;i<18;i++) for(int j=0;j<8;j++)
            fb.blendPixel(x+i, y+j, {100,255,120}, 0.9f);
        // 切线 T
        y += 14;
        for(int i=0;i<18;i++) for(int j=0;j<8;j++)
            fb.blendPixel(x+i, y+j, {255,80,80}, 0.9f);
        // 副切线 B
        y += 14;
        for(int i=0;i<18;i++) for(int j=0;j<8;j++)
            fb.blendPixel(x+i, y+j, {80,220,80}, 0.9f);
    }

    // ── 保存图像 ────────────────────────────────────────────────
    const std::string outPath = "geometry_shader_output.png";
    std::vector<uint8_t> pixels(W * H * 3);
    for(int i = 0; i < W*H; i++) {
        pixels[i*3+0] = fb.color[i].r;
        pixels[i*3+1] = fb.color[i].g;
        pixels[i*3+2] = fb.color[i].b;
    }
    int ok = stbi_write_png(outPath.c_str(), W, H, 3, pixels.data(), W*3);
    if(!ok) {
        std::cerr << "❌ Failed to write PNG\n";
        return 1;
    }
    std::cout << "✅ Rendered: " << outPath << " (" << W << "x" << H << ")\n";
    return 0;
}
