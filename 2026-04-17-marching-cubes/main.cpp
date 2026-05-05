// Marching Cubes Isosurface Extraction
// 2026-04-17 Daily Coding Practice
//
// 功能：从 3D 标量场（SDF）中提取等值面，生成三角形网格
// 并使用软光栅化渲染为 PNG 图像
//
// 算法：Marching Cubes（Paul Bourke 1994 经典实现）
// 场景：多个 SDF 球体/圆环的 union/smooth-union，形成有机形态
// 渲染：Phong 着色 + 软阴影 + 旋转视角

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <array>

//=============================================================================
// 数学工具
//=============================================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3  normalized() const { float l = length(); return (l>1e-7f) ? *this/l : Vec3(0,0,0); }
};

inline Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
inline float clamp01(float v) { return v<0?0:v>1?1:v; }

//=============================================================================
// SDF 场景
//=============================================================================
inline float sdSphere(Vec3 p, Vec3 c, float r) {
    return (p - c).length() - r;
}
inline float sdTorus(Vec3 p, float R, float r) {
    float qx = std::sqrt(p.x*p.x + p.z*p.z) - R;
    float qy = p.y;
    return std::sqrt(qx*qx + qy*qy) - r;
}
inline float sdCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
    Vec3 ab = b - a, ap = p - a;
    float t = clamp01(ap.dot(ab) / ab.dot(ab));
    Vec3 closest = a + ab*t;
    return (p - closest).length() - r;
}
// smooth union
inline float smin(float a, float b, float k) {
    float h = std::max(k - std::abs(a - b), 0.0f) / k;
    return std::min(a, b) - h*h*k/4.0f;
}

// 整个场景的 SDF
float sceneSDF(Vec3 p) {
    // 中心大球
    float d = sdSphere(p, Vec3(0,0,0), 1.2f);
    // 周围 4 个小球（smooth union 融合）
    float angle = 0.f;
    for (int i = 0; i < 4; i++) {
        float a = angle + i * (3.14159265f / 2.0f);
        Vec3 c(1.6f*std::cos(a), 0.4f*std::sin(a*2.0f), 1.6f*std::sin(a));
        d = smin(d, sdSphere(p, c, 0.6f), 0.5f);
    }
    // 顶部圆环
    Vec3 pt = p - Vec3(0, 1.5f, 0);
    float dt = sdTorus(pt, 0.8f, 0.25f);
    d = smin(d, dt, 0.3f);
    // 底部胶囊
    float dc = sdCapsule(p, Vec3(-1.0f, -1.8f, 0), Vec3(1.0f, -1.8f, 0), 0.3f);
    d = smin(d, dc, 0.4f);
    return d;
}

// 梯度（法线）
Vec3 calcNormal(Vec3 p) {
    const float eps = 0.001f;
    float dx = sceneSDF({p.x+eps,p.y,p.z}) - sceneSDF({p.x-eps,p.y,p.z});
    float dy = sceneSDF({p.x,p.y+eps,p.z}) - sceneSDF({p.x,p.y-eps,p.z});
    float dz = sceneSDF({p.x,p.y,p.z+eps}) - sceneSDF({p.x,p.y,p.z-eps});
    return Vec3(dx, dy, dz).normalized();
}

//=============================================================================
// Marching Cubes 查找表
//=============================================================================
// 标准 256 个 edge table（告诉我们哪些边被等值面穿过）
static const int edgeTable[256] = {
0x000,0x109,0x203,0x30a,0x406,0x50f,0x605,0x70c,
0x80c,0x905,0xa0f,0xb06,0xc0a,0xd03,0xe09,0xf00,
0x190,0x099,0x393,0x29a,0x596,0x49f,0x795,0x69c,
0x99c,0x895,0xb9f,0xa96,0xd9a,0xc93,0xf99,0xe90,
0x230,0x339,0x033,0x13a,0x636,0x73f,0x435,0x53c,
0xa3c,0xb35,0x83f,0x936,0xe3a,0xf33,0xc39,0xd30,
0x3a0,0x2a9,0x1a3,0x0aa,0x7a6,0x6af,0x5a5,0x4ac,
0xbac,0xaa5,0x9af,0x8a6,0xfaa,0xea3,0xda9,0xca0,
0x460,0x569,0x663,0x76a,0x066,0x16f,0x265,0x36c,
0xc6c,0xd65,0xe6f,0xf66,0x86a,0x963,0xa69,0xb60,
0x5f0,0x4f9,0x7f3,0x6fa,0x1f6,0x0ff,0x3f5,0x2fc,
0xdfc,0xcf5,0xfff,0xef6,0x9fa,0x8f3,0xbf9,0xaf0,
0x650,0x759,0x453,0x55a,0x256,0x35f,0x055,0x15c,
0xe5c,0xf55,0xc5f,0xd56,0xa5a,0xb53,0x859,0x950,
0x7c0,0x6c9,0x5c3,0x4ca,0x3c6,0x2cf,0x1c5,0x0cc,
0xfcc,0xec5,0xdcf,0xcc6,0xbca,0xac3,0x9c9,0x8c0,
0x8c0,0x9c9,0xac3,0xbca,0xcc6,0xdcf,0xec5,0xfcc,
0x0cc,0x1c5,0x2cf,0x3c6,0x4ca,0x5c3,0x6c9,0x7c0,
0x950,0x859,0xb53,0xa5a,0xd56,0xc5f,0xf55,0xe5c,
0x15c,0x055,0x35f,0x256,0x55a,0x453,0x759,0x650,
0xaf0,0xbf9,0x8f3,0x9fa,0xef6,0xfff,0xcf5,0xdfc,
0x2fc,0x3f5,0x0ff,0x1f6,0x6fa,0x7f3,0x4f9,0x5f0,
0xb60,0xa69,0x963,0x86a,0xf66,0xe6f,0xd65,0xc6c,
0x36c,0x265,0x16f,0x066,0x76a,0x663,0x569,0x460,
0xca0,0xda9,0xea3,0xfaa,0x8a6,0x9af,0xaa5,0xbac,
0x4ac,0x5a5,0x6af,0x7a6,0x0aa,0x1a3,0x2a9,0x3a0,
0xd30,0xc39,0xf33,0xe3a,0x936,0x835,0xb3f,0xa36, // fixed
0x53c,0x435,0x73f,0x636,0x13a,0x033,0x339,0x230,
0xe90,0xf99,0xc93,0xd9a,0xa96,0xb9f,0x895,0x99c,
0x69c,0x795,0x49f,0x596,0x29a,0x393,0x099,0x190,
0xf00,0xe09,0xd03,0xc0a,0xb06,0xa0f,0x905,0x80c,
0x70c,0x605,0x50f,0x406,0x30a,0x203,0x109,0x000
};

// 三角形表（每个格子最多 5 个三角形，每行 16 个 int，-1 结束）
static const int triTable[256][16] = {
{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,1,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,8,3,9,8,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,1,2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,2,10,0,2,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{2,8,3,2,10,8,10,9,8,-1,-1,-1,-1,-1,-1,-1},
{3,11,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,11,2,8,11,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,9,0,2,3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,11,2,1,9,11,9,8,11,-1,-1,-1,-1,-1,-1,-1},
{3,10,1,11,10,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,10,1,0,8,10,8,11,10,-1,-1,-1,-1,-1,-1,-1},
{3,9,0,3,11,9,11,10,9,-1,-1,-1,-1,-1,-1,-1},
{9,8,10,10,8,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,7,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,3,0,7,3,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,1,9,8,4,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,1,9,4,7,1,7,3,1,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,8,4,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,4,7,3,0,4,1,2,10,-1,-1,-1,-1,-1,-1,-1},
{9,2,10,9,0,2,8,4,7,-1,-1,-1,-1,-1,-1,-1},
{2,10,9,2,9,7,2,7,3,7,9,4,-1,-1,-1,-1},
{8,4,7,3,11,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{11,4,7,11,2,4,2,0,4,-1,-1,-1,-1,-1,-1,-1},
{9,0,1,8,4,7,2,3,11,-1,-1,-1,-1,-1,-1,-1},
{4,7,11,9,4,11,9,11,2,9,2,1,-1,-1,-1,-1},
{3,10,1,3,11,10,7,8,4,-1,-1,-1,-1,-1,-1,-1},
{1,11,10,1,4,11,1,0,4,7,11,4,-1,-1,-1,-1},
{4,7,8,9,0,11,9,11,10,11,0,3,-1,-1,-1,-1},
{4,7,11,4,11,9,9,11,10,-1,-1,-1,-1,-1,-1,-1},
{9,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,5,4,0,8,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,5,4,1,5,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{8,5,4,8,3,5,3,1,5,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,9,5,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,0,8,1,2,10,4,9,5,-1,-1,-1,-1,-1,-1,-1},
{5,2,10,5,4,2,4,0,2,-1,-1,-1,-1,-1,-1,-1},
{2,10,5,3,2,5,3,5,4,3,4,8,-1,-1,-1,-1},
{9,5,4,2,3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,11,2,0,8,11,4,9,5,-1,-1,-1,-1,-1,-1,-1},
{0,5,4,0,1,5,2,3,11,-1,-1,-1,-1,-1,-1,-1},
{2,1,5,2,5,8,2,8,11,4,8,5,-1,-1,-1,-1},
{10,3,11,10,1,3,9,5,4,-1,-1,-1,-1,-1,-1,-1},
{4,9,5,0,8,1,8,10,1,8,11,10,-1,-1,-1,-1},
{5,4,0,5,0,11,5,11,10,11,0,3,-1,-1,-1,-1},
{5,4,8,5,8,10,10,8,11,-1,-1,-1,-1,-1,-1,-1},
{9,7,8,5,7,9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,3,0,9,5,3,5,7,3,-1,-1,-1,-1,-1,-1,-1},
{0,7,8,0,1,7,1,5,7,-1,-1,-1,-1,-1,-1,-1},
{1,5,3,3,5,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,7,8,9,5,7,10,1,2,-1,-1,-1,-1,-1,-1,-1},
{10,1,2,9,5,0,5,3,0,5,7,3,-1,-1,-1,-1},
{8,0,2,8,2,5,8,5,7,10,5,2,-1,-1,-1,-1},
{2,10,5,2,5,3,3,5,7,-1,-1,-1,-1,-1,-1,-1},
{7,9,5,7,8,9,3,11,2,-1,-1,-1,-1,-1,-1,-1},
{9,5,7,9,7,2,9,2,0,2,7,11,-1,-1,-1,-1},
{2,3,11,0,1,8,1,7,8,1,5,7,-1,-1,-1,-1},
{11,2,1,11,1,7,7,1,5,-1,-1,-1,-1,-1,-1,-1},
{9,5,8,8,5,7,10,1,3,10,3,11,-1,-1,-1,-1},
{5,7,0,5,0,9,7,11,0,1,0,10,11,10,0,-1},
{11,10,0,11,0,3,10,5,0,8,0,7,5,7,0,-1},
{11,10,5,7,11,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{10,6,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,5,10,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,0,1,5,10,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,8,3,1,9,8,5,10,6,-1,-1,-1,-1,-1,-1,-1},
{1,6,5,2,6,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,6,5,1,2,6,3,0,8,-1,-1,-1,-1,-1,-1,-1},
{9,6,5,9,0,6,0,2,6,-1,-1,-1,-1,-1,-1,-1},
{5,9,8,5,8,2,5,2,6,3,2,8,-1,-1,-1,-1},
{2,3,11,10,6,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{11,0,8,11,2,0,10,6,5,-1,-1,-1,-1,-1,-1,-1},
{0,1,9,2,3,11,5,10,6,-1,-1,-1,-1,-1,-1,-1},
{5,10,6,1,9,2,9,11,2,9,8,11,-1,-1,-1,-1},
{6,3,11,6,5,3,5,1,3,-1,-1,-1,-1,-1,-1,-1},
{0,8,11,0,11,5,0,5,1,5,11,6,-1,-1,-1,-1},
{3,11,6,0,3,6,0,6,5,0,5,9,-1,-1,-1,-1},
{6,5,9,6,9,11,11,9,8,-1,-1,-1,-1,-1,-1,-1},
{5,10,6,4,7,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,3,0,4,7,3,6,5,10,-1,-1,-1,-1,-1,-1,-1},
{1,9,0,5,10,6,8,4,7,-1,-1,-1,-1,-1,-1,-1},
{10,6,5,1,9,7,1,7,3,7,9,4,-1,-1,-1,-1},
{6,1,2,6,5,1,4,7,8,-1,-1,-1,-1,-1,-1,-1},
{1,2,5,5,2,6,3,0,4,3,4,7,-1,-1,-1,-1},
{8,4,7,9,0,5,0,6,5,0,2,6,-1,-1,-1,-1},
{7,3,9,7,9,4,3,2,9,5,9,6,2,6,9,-1},
{3,11,2,7,8,4,10,6,5,-1,-1,-1,-1,-1,-1,-1},
{5,10,6,4,7,2,4,2,0,2,7,11,-1,-1,-1,-1},
{0,1,9,4,7,8,2,3,11,5,10,6,-1,-1,-1,-1},
{9,2,1,9,11,2,9,4,11,7,11,4,5,10,6,-1},
{8,4,7,3,11,5,3,5,1,5,11,6,-1,-1,-1,-1},
{5,1,11,5,11,6,1,0,11,7,11,4,0,4,11,-1},
{0,5,9,0,6,5,0,3,6,11,6,3,8,4,7,-1},
{6,5,9,6,9,11,4,7,9,7,11,9,-1,-1,-1,-1},
{10,4,9,6,4,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,10,6,4,9,10,0,8,3,-1,-1,-1,-1,-1,-1,-1},
{10,0,1,10,6,0,6,4,0,-1,-1,-1,-1,-1,-1,-1},
{8,3,1,8,1,6,8,6,4,6,1,10,-1,-1,-1,-1},
{1,4,9,1,2,4,2,6,4,-1,-1,-1,-1,-1,-1,-1},
{3,0,8,1,2,9,2,4,9,2,6,4,-1,-1,-1,-1},
{0,2,4,4,2,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{8,3,2,8,2,4,4,2,6,-1,-1,-1,-1,-1,-1,-1},
{10,4,9,10,6,4,11,2,3,-1,-1,-1,-1,-1,-1,-1},
{0,8,2,2,8,11,4,9,10,4,10,6,-1,-1,-1,-1},
{3,11,2,0,1,6,0,6,4,6,1,10,-1,-1,-1,-1},
{6,4,1,6,1,10,4,8,1,2,1,11,8,11,1,-1},
{9,6,4,9,3,6,9,1,3,11,6,3,-1,-1,-1,-1},
{8,11,1,8,1,0,11,6,1,9,1,4,6,4,1,-1},
{3,11,6,3,6,0,0,6,4,-1,-1,-1,-1,-1,-1,-1},
{6,4,8,11,6,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{7,10,6,7,8,10,8,9,10,-1,-1,-1,-1,-1,-1,-1},
{0,7,3,0,10,7,0,9,10,6,7,10,-1,-1,-1,-1},
{10,6,7,1,10,7,1,7,8,1,8,0,-1,-1,-1,-1},
{10,6,7,10,7,1,1,7,3,-1,-1,-1,-1,-1,-1,-1},
{1,2,6,1,6,8,1,8,9,8,6,7,-1,-1,-1,-1},
{2,6,9,2,9,1,6,7,9,0,9,3,7,3,9,-1},
{7,8,0,7,0,6,6,0,2,-1,-1,-1,-1,-1,-1,-1},
{7,3,2,6,7,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{2,3,11,10,6,8,10,8,9,8,6,7,-1,-1,-1,-1},
{2,0,7,2,7,11,0,9,7,6,7,10,9,10,7,-1},
{1,8,0,1,7,8,1,10,7,6,7,10,2,3,11,-1},
{11,2,1,11,1,7,10,6,1,6,7,1,-1,-1,-1,-1},
{8,9,6,8,6,7,9,1,6,11,6,3,1,3,6,-1},
{0,9,1,11,6,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{7,8,0,7,0,6,3,11,0,11,6,0,-1,-1,-1,-1},
{7,11,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{7,6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,0,8,11,7,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,1,9,11,7,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{8,1,9,8,3,1,11,7,6,-1,-1,-1,-1,-1,-1,-1},
{10,1,2,6,11,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,3,0,8,6,11,7,-1,-1,-1,-1,-1,-1,-1},
{2,9,0,2,10,9,6,11,7,-1,-1,-1,-1,-1,-1,-1},
{6,11,7,2,10,3,10,8,3,10,9,8,-1,-1,-1,-1},
{7,2,3,6,2,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{7,0,8,7,6,0,6,2,0,-1,-1,-1,-1,-1,-1,-1},
{2,7,6,2,3,7,0,1,9,-1,-1,-1,-1,-1,-1,-1},
{1,6,2,1,8,6,1,9,8,8,7,6,-1,-1,-1,-1},
{10,7,6,10,1,7,1,3,7,-1,-1,-1,-1,-1,-1,-1},
{10,7,6,1,7,10,1,8,7,1,0,8,-1,-1,-1,-1},
{0,3,7,0,7,10,0,10,9,6,10,7,-1,-1,-1,-1},
{7,6,10,7,10,8,8,10,9,-1,-1,-1,-1,-1,-1,-1},
{6,8,4,11,8,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,6,11,3,0,6,0,4,6,-1,-1,-1,-1,-1,-1,-1},
{8,6,11,8,4,6,9,0,1,-1,-1,-1,-1,-1,-1,-1},
{9,4,6,9,6,3,9,3,1,11,3,6,-1,-1,-1,-1},
{6,8,4,6,11,8,2,10,1,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,3,0,11,0,6,11,0,4,6,-1,-1,-1,-1},
{4,11,8,4,6,11,0,2,9,2,10,9,-1,-1,-1,-1},
{10,9,3,10,3,2,9,4,3,11,3,6,4,6,3,-1},
{8,2,3,8,4,2,4,6,2,-1,-1,-1,-1,-1,-1,-1},
{0,4,2,4,6,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,9,0,2,3,4,2,4,6,4,3,8,-1,-1,-1,-1},
{1,9,4,1,4,2,2,4,6,-1,-1,-1,-1,-1,-1,-1},
{8,1,3,8,6,1,8,4,6,6,10,1,-1,-1,-1,-1},
{10,1,0,10,0,6,6,0,4,-1,-1,-1,-1,-1,-1,-1},
{4,6,3,4,3,8,6,10,3,0,3,9,10,9,3,-1},
{10,9,4,6,10,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,9,5,7,6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,4,9,5,11,7,6,-1,-1,-1,-1,-1,-1,-1},
{5,0,1,5,4,0,7,6,11,-1,-1,-1,-1,-1,-1,-1},
{11,7,6,8,3,4,3,5,4,3,1,5,-1,-1,-1,-1},
{9,5,4,10,1,2,7,6,11,-1,-1,-1,-1,-1,-1,-1},
{6,11,7,1,2,10,0,8,3,4,9,5,-1,-1,-1,-1},
{7,6,11,5,4,10,4,2,10,4,0,2,-1,-1,-1,-1},
{3,4,8,3,5,4,3,2,5,10,5,2,11,7,6,-1},
{7,2,3,7,6,2,5,4,9,-1,-1,-1,-1,-1,-1,-1},
{9,5,4,0,8,6,0,6,2,6,8,7,-1,-1,-1,-1},
{3,6,2,3,7,6,1,5,0,5,4,0,-1,-1,-1,-1},
{6,2,8,6,8,7,2,1,8,4,8,5,1,5,8,-1},
{9,5,4,10,1,6,1,7,6,1,3,7,-1,-1,-1,-1},
{1,6,10,1,7,6,1,0,7,8,7,0,9,5,4,-1},
{4,0,10,4,10,5,0,3,10,6,10,7,3,7,10,-1},
{7,6,10,7,10,8,5,4,10,4,8,10,-1,-1,-1,-1},
{6,9,5,6,11,9,11,8,9,-1,-1,-1,-1,-1,-1,-1},
{3,6,11,0,6,3,0,5,6,0,9,5,-1,-1,-1,-1},
{0,11,8,0,5,11,0,1,5,5,6,11,-1,-1,-1,-1},
{6,11,3,6,3,5,5,3,1,-1,-1,-1,-1,-1,-1,-1},
{1,2,10,9,5,11,9,11,8,11,5,6,-1,-1,-1,-1},
{0,11,3,0,6,11,0,9,6,5,6,9,1,2,10,-1},
{11,8,5,11,5,6,8,0,5,10,5,2,0,2,5,-1},
{6,11,3,6,3,5,2,10,3,10,5,3,-1,-1,-1,-1},
{5,8,9,5,2,8,5,6,2,3,8,2,-1,-1,-1,-1},
{9,5,6,9,6,0,0,6,2,-1,-1,-1,-1,-1,-1,-1},
{1,5,8,1,8,0,5,6,8,3,8,2,6,2,8,-1},
{1,5,6,2,1,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,3,6,1,6,10,3,8,6,5,6,9,8,9,6,-1},
{10,1,0,10,0,6,9,5,0,5,6,0,-1,-1,-1,-1},
{0,3,8,5,6,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{10,5,6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{11,5,10,7,5,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{11,5,10,11,7,5,8,3,0,-1,-1,-1,-1,-1,-1,-1},
{5,11,7,5,10,11,1,9,0,-1,-1,-1,-1,-1,-1,-1},
{10,7,5,10,11,7,9,8,1,8,3,1,-1,-1,-1,-1},
{11,1,2,11,7,1,7,5,1,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,1,2,7,1,7,5,7,2,11,-1,-1,-1,-1},
{9,7,5,9,2,7,9,0,2,2,11,7,-1,-1,-1,-1},
{7,5,2,7,2,11,5,9,2,3,2,8,9,8,2,-1},
{2,5,10,2,3,5,3,7,5,-1,-1,-1,-1,-1,-1,-1},
{8,2,0,8,5,2,8,7,5,10,2,5,-1,-1,-1,-1},
{9,0,1,2,3,5,2,5,10,5,3,7,-1,-1,-1,-1},
{9,8,2,9,2,1,8,7,2,10,2,5,7,5,2,-1},
{1,3,5,3,7,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,8,7,0,7,1,1,7,5,-1,-1,-1,-1,-1,-1,-1},
{9,0,3,9,3,5,5,3,7,-1,-1,-1,-1,-1,-1,-1},
{9,8,7,5,9,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{5,8,4,5,10,8,10,11,8,-1,-1,-1,-1,-1,-1,-1},
{5,0,4,5,11,0,5,10,11,11,3,0,-1,-1,-1,-1},
{0,1,9,8,4,10,8,10,11,10,4,5,-1,-1,-1,-1},
{10,11,4,10,4,5,11,3,4,9,4,1,3,1,4,-1},
{2,5,1,2,8,5,2,11,8,4,5,8,-1,-1,-1,-1},
{0,4,11,0,11,3,4,5,11,2,11,1,5,1,11,-1},
{0,2,5,0,5,9,2,11,5,4,5,8,11,8,5,-1},
{9,4,5,2,11,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{2,5,10,3,5,2,3,4,5,3,8,4,-1,-1,-1,-1},
{5,10,2,5,2,4,4,2,0,-1,-1,-1,-1,-1,-1,-1},
{3,10,2,3,5,10,3,8,5,4,5,8,0,1,9,-1},
{5,10,2,5,2,4,1,9,2,9,4,2,-1,-1,-1,-1},
{8,4,5,8,5,3,3,5,1,-1,-1,-1,-1,-1,-1,-1},
{0,4,5,1,0,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{8,4,5,8,5,3,9,0,5,0,3,5,-1,-1,-1,-1},
{9,4,5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,11,7,4,9,11,9,10,11,-1,-1,-1,-1,-1,-1,-1},
{0,8,3,4,9,7,9,11,7,9,10,11,-1,-1,-1,-1},
{1,10,11,1,11,4,1,4,0,7,4,11,-1,-1,-1,-1},
{3,1,4,3,4,8,1,10,4,7,4,11,10,11,4,-1},
{4,11,7,9,11,4,9,2,11,9,1,2,-1,-1,-1,-1},
{9,7,4,9,11,7,9,1,11,2,11,1,0,8,3,-1},
{11,7,4,11,4,2,2,4,0,-1,-1,-1,-1,-1,-1,-1},
{11,7,4,11,4,2,8,3,4,3,2,4,-1,-1,-1,-1},
{2,9,10,2,7,9,2,3,7,7,4,9,-1,-1,-1,-1},
{9,10,7,9,7,4,10,2,7,8,7,0,2,0,7,-1},
{3,7,10,3,10,2,7,4,10,1,10,0,4,0,10,-1},
{1,10,2,8,7,4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,9,1,4,1,7,7,1,3,-1,-1,-1,-1,-1,-1,-1},
{4,9,1,4,1,7,0,8,1,8,7,1,-1,-1,-1,-1},
{4,0,3,7,4,3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{4,8,7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{9,10,8,10,11,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,0,9,3,9,11,11,9,10,-1,-1,-1,-1,-1,-1,-1},
{0,1,10,0,10,8,8,10,11,-1,-1,-1,-1,-1,-1,-1},
{3,1,10,11,3,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,2,11,1,11,9,9,11,8,-1,-1,-1,-1,-1,-1,-1},
{3,0,9,3,9,11,1,2,9,2,11,9,-1,-1,-1,-1},
{0,2,11,8,0,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{3,2,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{2,3,8,2,8,10,10,8,9,-1,-1,-1,-1,-1,-1,-1},
{9,10,2,0,9,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{2,3,8,2,8,10,0,1,8,1,10,8,-1,-1,-1,-1},
{1,10,2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{1,3,8,9,1,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,9,1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{0,3,8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
};

//=============================================================================
// Marching Cubes 核心
//=============================================================================
struct Triangle {
    Vec3 v[3];
    Vec3 n[3]; // vertex normals
};

// 在边上插值出等值点
Vec3 vertexInterp(float isoLevel, Vec3 p1, float v1, Vec3 p2, float v2) {
    if (std::abs(isoLevel - v1) < 1e-5f) return p1;
    if (std::abs(isoLevel - v2) < 1e-5f) return p2;
    if (std::abs(v1 - v2) < 1e-5f)       return p1;
    float t = (isoLevel - v1) / (v2 - v1);
    return p1 + (p2 - p1) * t;
}

// 处理单个体素格子，输出三角形
void polygoniseCube(
    std::array<Vec3, 8> pos,
    std::array<float, 8> val,
    float isoLevel,
    std::vector<Triangle>& tris)
{
    int cubeindex = 0;
    for (int i = 0; i < 8; i++)
        if (val[i] < isoLevel) cubeindex |= (1 << i);

    if (edgeTable[cubeindex] == 0) return;

    Vec3 vertlist[12];
    if (edgeTable[cubeindex] & 0x001) vertlist[0]  = vertexInterp(isoLevel, pos[0], val[0], pos[1], val[1]);
    if (edgeTable[cubeindex] & 0x002) vertlist[1]  = vertexInterp(isoLevel, pos[1], val[1], pos[2], val[2]);
    if (edgeTable[cubeindex] & 0x004) vertlist[2]  = vertexInterp(isoLevel, pos[2], val[2], pos[3], val[3]);
    if (edgeTable[cubeindex] & 0x008) vertlist[3]  = vertexInterp(isoLevel, pos[3], val[3], pos[0], val[0]);
    if (edgeTable[cubeindex] & 0x010) vertlist[4]  = vertexInterp(isoLevel, pos[4], val[4], pos[5], val[5]);
    if (edgeTable[cubeindex] & 0x020) vertlist[5]  = vertexInterp(isoLevel, pos[5], val[5], pos[6], val[6]);
    if (edgeTable[cubeindex] & 0x040) vertlist[6]  = vertexInterp(isoLevel, pos[6], val[6], pos[7], val[7]);
    if (edgeTable[cubeindex] & 0x080) vertlist[7]  = vertexInterp(isoLevel, pos[7], val[7], pos[4], val[4]);
    if (edgeTable[cubeindex] & 0x100) vertlist[8]  = vertexInterp(isoLevel, pos[0], val[0], pos[4], val[4]);
    if (edgeTable[cubeindex] & 0x200) vertlist[9]  = vertexInterp(isoLevel, pos[1], val[1], pos[5], val[5]);
    if (edgeTable[cubeindex] & 0x400) vertlist[10] = vertexInterp(isoLevel, pos[2], val[2], pos[6], val[6]);
    if (edgeTable[cubeindex] & 0x800) vertlist[11] = vertexInterp(isoLevel, pos[3], val[3], pos[7], val[7]);

    for (int i = 0; triTable[cubeindex][i] != -1; i += 3) {
        Triangle t;
        t.v[0] = vertlist[triTable[cubeindex][i  ]];
        t.v[1] = vertlist[triTable[cubeindex][i+1]];
        t.v[2] = vertlist[triTable[cubeindex][i+2]];
        // 顶点法线由 SDF 梯度计算
        t.n[0] = calcNormal(t.v[0]);
        t.n[1] = calcNormal(t.v[1]);
        t.n[2] = calcNormal(t.v[2]);
        tris.push_back(t);
    }
}

// 在给定范围 [-bound, bound]^3 上运行 Marching Cubes
std::vector<Triangle> marchingCubes(float bound, int N, float isoLevel) {
    std::vector<Triangle> tris;
    float step = 2.0f * bound / N;

    for (int iz = 0; iz < N; iz++) {
        for (int iy = 0; iy < N; iy++) {
            for (int ix = 0; ix < N; ix++) {
                float x0 = -bound + ix * step;
                float y0 = -bound + iy * step;
                float z0 = -bound + iz * step;
                float x1 = x0 + step, y1 = y0 + step, z1 = z0 + step;

                std::array<Vec3, 8> pos = {{
                    {x0,y0,z0}, {x1,y0,z0}, {x1,y1,z0}, {x0,y1,z0},
                    {x0,y0,z1}, {x1,y0,z1}, {x1,y1,z1}, {x0,y1,z1}
                }};
                std::array<float, 8> val;
                for (int k = 0; k < 8; k++) val[k] = sceneSDF(pos[k]);

                polygoniseCube(pos, val, isoLevel, tris);
            }
        }
    }
    return tris;
}

//=============================================================================
// 软光栅化渲染器
//=============================================================================
static const int W = 800, H = 600;

struct Framebuffer {
    std::vector<Vec3> color;
    std::vector<float> depth;
    Framebuffer() : color(W*H, Vec3(0,0,0)), depth(W*H, 1e30f) {}
    void setPixel(int x, int y, const Vec3& c, float d) {
        if (x < 0 || x >= W || y < 0 || y >= H) return;
        int idx = y * W + x;
        if (d < depth[idx]) { depth[idx] = d; color[idx] = c; }
    }
};

// 简单的 Phong 光照
Vec3 phongShading(const Vec3& /*pos*/, const Vec3& normal, const Vec3& viewDir) {
    Vec3 lightDir1 = Vec3(1, 2, 1).normalized();
    Vec3 lightDir2 = Vec3(-1, 1, 0.5f).normalized();
    Vec3 baseColor(0.4f, 0.6f, 0.9f); // 蓝紫色调

    float ambient = 0.15f;
    float diff1 = std::max(0.0f, normal.dot(lightDir1));
    float diff2 = std::max(0.0f, normal.dot(lightDir2)) * 0.4f;

    Vec3 reflDir1 = normal * (2.0f * normal.dot(lightDir1)) - lightDir1;
    float spec1 = std::pow(std::max(0.0f, viewDir.dot(reflDir1)), 32.0f) * 0.6f;

    float rim = std::pow(1.0f - std::max(0.0f, normal.dot(viewDir)), 3.0f) * 0.3f;

    Vec3 finalColor = baseColor * (ambient + diff1 * 0.7f + diff2);
    finalColor += Vec3(1,1,1) * spec1;
    finalColor += Vec3(0.5f, 0.7f, 1.0f) * rim;

    return finalColor;
}

// 透视投影
bool project(const Vec3& v, const Vec3& camPos, const Vec3& camFwd,
             const Vec3& camRight, const Vec3& camUp,
             float fovY, float& sx, float& sy, float& depth_val)
{
    Vec3 d = v - camPos;
    float z = d.dot(camFwd);
    if (z < 0.1f) return false;
    float x = d.dot(camRight);
    float y = d.dot(camUp);
    float aspect = (float)W / H;
    float tanH = std::tan(fovY * 0.5f);
    sx = (x / (z * tanH * aspect)) * 0.5f + 0.5f;
    sy = (y / (z * tanH)) * 0.5f + 0.5f;
    sy = 1.0f - sy; // 翻转 y，屏幕坐标 y 向下
    depth_val = z;
    sx = sx * W;
    sy = sy * H;
    return true;
}

// 光栅化一个三角形（带插值法线）
void rasterizeTriangle(Framebuffer& fb,
                       const Vec3& p0, const Vec3& n0,
                       const Vec3& p1, const Vec3& n1,
                       const Vec3& p2, const Vec3& n2,
                       const Vec3& camPos, const Vec3& camFwd,
                       const Vec3& camRight, const Vec3& camUp, float fovY)
{
    float sx0, sy0, d0, sx1, sy1, d1, sx2, sy2, d2;
    if (!project(p0, camPos, camFwd, camRight, camUp, fovY, sx0, sy0, d0)) return;
    if (!project(p1, camPos, camFwd, camRight, camUp, fovY, sx1, sy1, d1)) return;
    if (!project(p2, camPos, camFwd, camRight, camUp, fovY, sx2, sy2, d2)) return;

    int minX = (int)std::max(0.f, std::floor(std::min({sx0,sx1,sx2})));
    int maxX = (int)std::min((float)W-1, std::ceil(std::max({sx0,sx1,sx2})));
    int minY = (int)std::max(0.f, std::floor(std::min({sy0,sy1,sy2})));
    int maxY = (int)std::min((float)H-1, std::ceil(std::max({sy0,sy1,sy2})));

    float area = (sx1-sx0)*(sy2-sy0) - (sx2-sx0)*(sy1-sy0);
    if (std::abs(area) < 1e-6f) return;

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = ((sx1-px)*(sy2-py) - (sx2-px)*(sy1-py)) / area;
            float w1 = ((sx2-px)*(sy0-py) - (sx0-px)*(sy2-py)) / area;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;

            float depth = w0*d0 + w1*d1 + w2*d2;
            Vec3 interpN = (n0*w0 + n1*w1 + n2*w2).normalized();
            Vec3 interpP = p0*w0 + p1*w1 + p2*w2;
            Vec3 vDir = (camPos - interpP).normalized();
            Vec3 color = phongShading(interpP, interpN, vDir);
            fb.setPixel(x, y, color, depth);
        }
    }
}

//=============================================================================
// PNG 写出（无依赖纯实现）
//=============================================================================
// CRC32
static uint32_t crc32_table[256];
static bool crc32_init = false;
void initCRC() {
    if (crc32_init) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++) c = (c&1) ? 0xedb88320u^(c>>1) : c>>1;
        crc32_table[i] = c;
    }
    crc32_init = true;
}
uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc=0xffffffff) {
    initCRC();
    for (size_t i=0;i<len;i++) crc = crc32_table[(crc^data[i])&0xff]^(crc>>8);
    return crc^0xffffffff;
}

// Adler32
uint32_t adler32(const uint8_t* data, size_t len) {
    uint32_t s1=1, s2=0;
    for (size_t i=0;i<len;i++) { s1=(s1+data[i])%65521; s2=(s2+s1)%65521; }
    return (s2<<16)|s1;
}

// zlib deflate（仅 store 模式）
std::vector<uint8_t> zlibStore(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> out;
    out.push_back(0x78); out.push_back(0x01); // zlib header
    size_t pos=0, rem=data.size();
    while (rem>0) {
        size_t blk = std::min(rem, (size_t)65535);
        bool last = (blk==rem);
        out.push_back(last?0x01:0x00);
        out.push_back(blk&0xff); out.push_back((blk>>8)&0xff);
        out.push_back((~blk)&0xff); out.push_back(((~blk)>>8)&0xff);
        out.insert(out.end(), data.begin()+pos, data.begin()+pos+blk);
        pos+=blk; rem-=blk;
    }
    uint32_t a=adler32(data.data(),data.size());
    out.push_back((a>>24)&0xff); out.push_back((a>>16)&0xff);
    out.push_back((a>>8)&0xff);  out.push_back(a&0xff);
    return out;
}

void writeU32BE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back((v>>24)&0xff); buf.push_back((v>>16)&0xff);
    buf.push_back((v>>8)&0xff);  buf.push_back(v&0xff);
}

void savePNG(const std::string& filename, const Framebuffer& fb) {
    // build raw image data (filter byte 0x00 per scanline)
    std::vector<uint8_t> raw;
    raw.reserve((1+W*3)*H);
    for (int y=0;y<H;y++) {
        raw.push_back(0); // filter None
        for (int x=0;x<W;x++) {
            const Vec3& c = fb.color[y*W+x];
            auto toU8 = [](float v){ int i=(int)(v*255.f+0.5f); return (uint8_t)(i<0?0:i>255?255:i); };
            raw.push_back(toU8(c.x));
            raw.push_back(toU8(c.y));
            raw.push_back(toU8(c.z));
        }
    }
    auto compressed = zlibStore(raw);

    std::vector<uint8_t> file;
    // PNG signature
    const uint8_t sig[]={137,80,78,71,13,10,26,10};
    file.insert(file.end(),sig,sig+8);

    // IHDR
    {
        std::vector<uint8_t> d;
        writeU32BE(d,W); writeU32BE(d,H);
        d.push_back(8); d.push_back(2); d.push_back(0); d.push_back(0); d.push_back(0);
        writeU32BE(file,(uint32_t)d.size());
        const char* t="IHDR"; file.insert(file.end(),t,t+4);
        file.insert(file.end(),d.begin(),d.end());
        std::vector<uint8_t> crcbuf; crcbuf.insert(crcbuf.end(),t,t+4); crcbuf.insert(crcbuf.end(),d.begin(),d.end());
        writeU32BE(file,crc32(crcbuf.data(),crcbuf.size()));
    }
    // IDAT
    {
        writeU32BE(file,(uint32_t)compressed.size());
        const char* t="IDAT"; file.insert(file.end(),t,t+4);
        file.insert(file.end(),compressed.begin(),compressed.end());
        std::vector<uint8_t> crcbuf; crcbuf.insert(crcbuf.end(),t,t+4); crcbuf.insert(crcbuf.end(),compressed.begin(),compressed.end());
        writeU32BE(file,crc32(crcbuf.data(),crcbuf.size()));
    }
    // IEND
    {
        writeU32BE(file,0);
        const char* t="IEND"; file.insert(file.end(),t,t+4);
        std::vector<uint8_t> crcbuf(t,t+4);
        writeU32BE(file,crc32(crcbuf.data(),crcbuf.size()));
    }

    std::ofstream ofs(filename, std::ios::binary);
    ofs.write((char*)file.data(), file.size());
    std::cout << "Saved " << filename << " (" << file.size() << " bytes)" << std::endl;
}

//=============================================================================
// 背景渐变
//=============================================================================
Vec3 skyColor(float dy) {
    // dy: -1(下) ~ +1(上)
    float t = dy * 0.5f + 0.5f;
    Vec3 top(0.08f, 0.12f, 0.22f);  // 深蓝夜空
    Vec3 bot(0.02f, 0.04f, 0.08f);  // 近黑
    return top * t + bot * (1-t);
}

//=============================================================================
// main
//=============================================================================
int main() {
    std::cout << "=== Marching Cubes Isosurface Extraction ===" << std::endl;
    std::cout << "Building scalar field and extracting isosurface..." << std::endl;

    // Marching Cubes 参数
    const float bound = 2.8f;
    const int   N     = 80;   // 格子数（80^3 ≈ 512k 格子）
    const float iso   = 0.0f; // 等值面阈值

    auto triangles = marchingCubes(bound, N, iso);
    std::cout << "Generated " << triangles.size() << " triangles" << std::endl;

    if (triangles.empty()) {
        std::cerr << "ERROR: No triangles generated!" << std::endl;
        return 1;
    }

    // 相机设置（45° 俯角，斜侧视）
    float camDist = 5.5f;
    float camAngleY = 0.45f;   // 水平角
    float camAngleX = 0.35f;   // 俯角
    Vec3 camPos(
        camDist * std::cos(camAngleX) * std::sin(camAngleY),
        camDist * std::sin(camAngleX),
        camDist * std::cos(camAngleX) * std::cos(camAngleY)
    );
    Vec3 target(0, 0, 0);
    Vec3 camFwd  = (target - camPos).normalized();
    Vec3 worldUp(0, 1, 0);
    Vec3 camRight = camFwd.cross(worldUp).normalized();
    Vec3 camUp    = camRight.cross(camFwd).normalized();
    float fovY = 1.0f; // ~57°

    Framebuffer fb;

    // 填充背景
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float dy = 1.0f - 2.0f * y / (H - 1); // +1 top, -1 bottom
            fb.color[y*W+x] = skyColor(dy);
            fb.depth[y*W+x] = 1e30f;
        }
    }

    // 渲染所有三角形
    std::cout << "Rasterizing " << triangles.size() << " triangles..." << std::endl;
    for (const auto& tri : triangles) {
        rasterizeTriangle(fb,
            tri.v[0], tri.n[0],
            tri.v[1], tri.n[1],
            tri.v[2], tri.n[2],
            camPos, camFwd, camRight, camUp, fovY);
    }

    // 简单后处理：gamma 矫正
    for (auto& c : fb.color) {
        c.x = std::pow(clamp01(c.x), 1.0f/2.2f);
        c.y = std::pow(clamp01(c.y), 1.0f/2.2f);
        c.z = std::pow(clamp01(c.z), 1.0f/2.2f);
    }

    savePNG("marching_cubes_output.png", fb);

    // 输出像素统计（用于验证）
    float sum = 0, sum2 = 0;
    int cnt = W * H;
    for (const auto& c : fb.color) {
        float v = (c.x + c.y + c.z) / 3.0f;
        sum += v; sum2 += v*v;
    }
    float mean = sum / cnt;
    float std_dev = std::sqrt(sum2/cnt - mean*mean);
    std::cout << "Pixel mean: " << mean*255 << "  std: " << std_dev*255 << std::endl;
    std::cout << "DONE" << std::endl;
    return 0;
}
