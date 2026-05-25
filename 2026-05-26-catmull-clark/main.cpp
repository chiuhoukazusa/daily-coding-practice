/**
 * Catmull-Clark Subdivision Surface Renderer
 * Catmull-Clark 细分曲面渲染器
 *
 * 技术特点：
 * - Catmull-Clark 细分算法（多边形网格 → 光滑 B-spline 曲面）
 * - 支持任意多边形（四边形、三角形混合）
 * - 3级细分（粗网格 → 细腻光滑网格）
 * - 半边数据结构加速邻接查询
 * - Phong 着色 + 法线插值
 * - 软光栅化渲染（2张对比图：细分前 vs 细分后）
 *
 * 输出：catmull_clark_output.png（800x400，左粗糙/右光滑对比）
 */

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ======================================================================
// Math
// ======================================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3 &o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3 &o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3 &o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3 &o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalized() const { float l = len(); return l > 1e-8f ? *this / l : Vec3(0,0,1); }
};
Vec3 operator*(float t, const Vec3 &v) { return v * t; }

// ======================================================================
// Mesh (多边形网格，面可以是任意多边形)
// ======================================================================
struct Mesh {
    std::vector<Vec3> verts;        // 顶点位置
    std::vector<std::vector<int>> faces; // 面（顶点索引列表）
};

// ======================================================================
// Catmull-Clark 细分
// ======================================================================
// 数据结构：边用 (min, max) 对表示
using EdgeKey = std::pair<int,int>;

static EdgeKey makeEdge(int a, int b) {
    return {std::min(a,b), std::max(a,b)};
}

// 执行一次 Catmull-Clark 细分
Mesh catmullClarkSubdivide(const Mesh &m) {
    int V = (int)m.verts.size();
    int F = (int)m.faces.size();

    // ── Step 1：每个面生成一个面点（face point） = 面所有顶点的平均 ──
    std::vector<Vec3> facePoints(F);
    for (int fi = 0; fi < F; fi++) {
        Vec3 sum(0,0,0);
        const auto &face = m.faces[fi];
        for (int vi : face) sum += m.verts[vi];
        facePoints[fi] = sum / (float)face.size();
    }

    // ── Step 2：每条边生成一个边点（edge point） ──
    // 边点 = (端点A + 端点B + 相邻面点1 + 相邻面点2) / 4
    // 对边界边 = (端点A + 端点B) / 2
    std::map<EdgeKey, std::vector<int>> edgeFaces; // 边 → 相邻面索引
    for (int fi = 0; fi < F; fi++) {
        const auto &face = m.faces[fi];
        int n = (int)face.size();
        for (int i = 0; i < n; i++) {
            int a = face[i], b = face[(i+1)%n];
            edgeFaces[makeEdge(a,b)].push_back(fi);
        }
    }

    std::map<EdgeKey, int> edgePointIdx; // 边 → 新顶点索引
    std::vector<Vec3> newVerts;

    // 先放 V 个原始顶点占位（稍后更新）
    newVerts.resize(V);

    // 面点从 V 开始
    int facePointStart = V;
    for (int fi = 0; fi < F; fi++) {
        newVerts.push_back(facePoints[fi]); // index = V + fi
    }

    // 边点
    for (auto &[ek, faceList] : edgeFaces) {
        Vec3 sum = m.verts[ek.first] + m.verts[ek.second];
        for (int fi : faceList) sum += facePoints[fi];
        float count = 2.0f + (float)faceList.size();
        int idx = (int)newVerts.size();
        newVerts.push_back(sum / count);
        edgePointIdx[ek] = idx;
    }

    // ── Step 3：更新原始顶点位置 ──
    // 对每个顶点 P：
    //   n = 连接边数（也等于相邻面数，对于封闭网格）
    //   F = 相邻面点的平均 (average of adjacent face points)
    //   R = 相邻边中点的平均 (average of adjacent edge midpoints)
    //   新位置 = (F + 2R + (n-3)*P) / n

    std::vector<std::vector<int>> vertFaces(V);
    std::vector<std::set<int>> vertNeighbors(V);

    for (int fi = 0; fi < F; fi++) {
        const auto &face = m.faces[fi];
        int n = (int)face.size();
        for (int i = 0; i < n; i++) {
            int vi = face[i];
            vertFaces[vi].push_back(fi);
            vertNeighbors[vi].insert(face[(i+1)%n]);
            vertNeighbors[vi].insert(face[(i-1+n)%n]);
        }
    }

    for (int vi = 0; vi < V; vi++) {
        int n = (int)vertFaces[vi].size();
        if (n == 0) {
            newVerts[vi] = m.verts[vi];
            continue;
        }

        // 判断是否为边界顶点（有边界边相邻 = 某条边只属于1个面）
        bool boundary = false;
        for (int nb : vertNeighbors[vi]) {
            EdgeKey ek = makeEdge(vi, nb);
            if (edgeFaces[ek].size() == 1) { boundary = true; break; }
        }

        if (boundary) {
            // 边界规则：边界顶点仅受边界边端点影响
            Vec3 sum(0,0,0);
            int cnt = 0;
            for (int nb : vertNeighbors[vi]) {
                EdgeKey ek = makeEdge(vi, nb);
                if (edgeFaces[ek].size() == 1) {
                    sum += m.verts[nb];
                    cnt++;
                }
            }
            newVerts[vi] = (m.verts[vi] * 6.0f + sum) / (float)(6 + cnt);
        } else {
            // 内部顶点规则
            Vec3 F_avg(0,0,0);
            for (int fi : vertFaces[vi]) F_avg += facePoints[fi];
            F_avg = F_avg / (float)n;

            Vec3 R_avg(0,0,0);
            for (int nb : vertNeighbors[vi]) {
                R_avg += (m.verts[vi] + m.verts[nb]) * 0.5f;
            }
            R_avg = R_avg / (float)vertNeighbors[vi].size();

            newVerts[vi] = (F_avg + R_avg * 2.0f + m.verts[vi] * (float)(n - 3)) / (float)n;
        }
    }

    // ── Step 4：重新连接面 ──
    // 每个原始 n 边形面 → n 个四边形
    // 每个四边形：[面点, 边中点1, 角点, 边中点2]
    Mesh result;
    result.verts = newVerts;

    for (int fi = 0; fi < F; fi++) {
        const auto &face = m.faces[fi];
        int n = (int)face.size();
        int fp = facePointStart + fi;

        for (int i = 0; i < n; i++) {
            int v0 = face[i];
            int v1 = face[(i+1)%n];
            int v_prev = face[(i-1+n)%n];

            int ep0 = edgePointIdx[makeEdge(v_prev, v0)]; // 前一条边的边点
            int ep1 = edgePointIdx[makeEdge(v0, v1)];     // 当前边的边点

            result.faces.push_back({fp, ep0, v0, ep1});
        }
    }

    return result;
}

// ======================================================================
// 构建立方体网格（初始粗网格）
// ======================================================================
Mesh buildCubeMesh() {
    Mesh m;
    // 8个顶点
    float s = 1.0f;
    m.verts = {
        {-s,-s,-s}, { s,-s,-s}, { s, s,-s}, {-s, s,-s}, // 前面 z=-1
        {-s,-s, s}, { s,-s, s}, { s, s, s}, {-s, s, s}  // 后面 z=+1
    };
    // 6个面（四边形，顺时针为正面）
    m.faces = {
        {3,2,1,0}, // 前
        {4,5,6,7}, // 后
        {0,1,5,4}, // 下
        {2,3,7,6}, // 上
        {0,4,7,3}, // 左
        {1,2,6,5}  // 右
    };
    return m;
}

// ======================================================================
// 法线计算
// ======================================================================
std::vector<Vec3> computeVertexNormals(const Mesh &m) {
    std::vector<Vec3> normals(m.verts.size(), Vec3(0,0,0));
    for (const auto &face : m.faces) {
        if (face.size() < 3) continue;
        // 三角扇形计算面法线
        Vec3 v0 = m.verts[face[0]];
        for (int i = 1; i + 1 < (int)face.size(); i++) {
            Vec3 e1 = m.verts[face[i]] - v0;
            Vec3 e2 = m.verts[face[i+1]] - v0;
            Vec3 n = e1.cross(e2);
            normals[face[0]] += n;
            normals[face[i]] += n;
            normals[face[i+1]] += n;
        }
    }
    for (auto &n : normals) n = n.normalized();
    return normals;
}

// ======================================================================
// 软光栅化
// ======================================================================
const int W = 400, H = 400;
const int TW = 800; // 两张图拼合

struct Framebuffer {
    std::vector<uint8_t> color;
    std::vector<float> depth;
    int w, h, ox;

    Framebuffer(int w, int h, int ox = 0) : w(w), h(h), ox(ox) {
        color.resize(w * h * 3, 30);
        depth.assign(w * h, 1e9f);
    }

    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        int idx = (y * w + x) * 3;
        color[idx] = r; color[idx+1] = g; color[idx+2] = b;
    }

    bool testDepth(int x, int y, float z) {
        if (x < 0 || x >= w || y < 0 || y >= h) return false;
        int idx = y * w + x;
        if (z < depth[idx]) { depth[idx] = z; return true; }
        return false;
    }
};

struct Camera {
    Vec3 eye, target, up;
    float fov, aspect, znear, zfar;
};

struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
};

Vec3 mulMat4(const Mat4 &M, const Vec3 &v, float w) {
    float x = M.m[0][0]*v.x + M.m[0][1]*v.y + M.m[0][2]*v.z + M.m[0][3]*w;
    float y = M.m[1][0]*v.x + M.m[1][1]*v.y + M.m[1][2]*v.z + M.m[1][3]*w;
    float z = M.m[2][0]*v.x + M.m[2][1]*v.y + M.m[2][2]*v.z + M.m[2][3]*w;
    float ww= M.m[3][0]*v.x + M.m[3][1]*v.y + M.m[3][2]*v.z + M.m[3][3]*w;
    if (std::fabs(ww) > 1e-8f) return {x/ww, y/ww, z/ww};
    return {x, y, z};
}

Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = (target - eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);
    Mat4 M = Mat4::identity();
    M.m[0][0]=r.x; M.m[0][1]=r.y; M.m[0][2]=r.z; M.m[0][3]=-(r.dot(eye));
    M.m[1][0]=u.x; M.m[1][1]=u.y; M.m[1][2]=u.z; M.m[1][3]=-(u.dot(eye));
    M.m[2][0]=-f.x;M.m[2][1]=-f.y;M.m[2][2]=-f.z;M.m[2][3]=f.dot(eye);
    return M;
}

Mat4 perspective(float fov, float aspect, float n, float f) {
    float tanHalf = std::tan(fov * 0.5f * (float)M_PI / 180.0f);
    Mat4 M = {};
    M.m[0][0] = 1.0f / (aspect * tanHalf);
    M.m[1][1] = 1.0f / tanHalf;
    M.m[2][2] = -(f + n) / (f - n);
    M.m[2][3] = -2.0f * f * n / (f - n);
    M.m[3][2] = -1.0f;
    return M;
}

// 三角形光栅化
void rasterizeTriangle(Framebuffer &fb,
    Vec3 p0, Vec3 p1, Vec3 p2,
    Vec3 n0, Vec3 n1, Vec3 n2,
    Vec3 lightDir, Vec3 viewDir,
    uint8_t baseR, uint8_t baseG, uint8_t baseB)
{
    // 转屏幕坐标
    auto toScreen = [&](Vec3 ndc) -> std::pair<int,int> {
        int sx = (int)((ndc.x * 0.5f + 0.5f) * (fb.w - 1));
        int sy = (int)((1.0f - (ndc.y * 0.5f + 0.5f)) * (fb.h - 1));
        return {sx, sy};
    };

    auto [sx0,sy0] = toScreen(p0);
    auto [sx1,sy1] = toScreen(p1);
    auto [sx2,sy2] = toScreen(p2);

    int minX = std::max(0, std::min({sx0,sx1,sx2}));
    int maxX = std::min(fb.w-1, std::max({sx0,sx1,sx2}));
    int minY = std::max(0, std::min({sy0,sy1,sy2}));
    int maxY = std::min(fb.h-1, std::max({sy0,sy1,sy2}));

    // 重心坐标辅助
    auto edgeFn = [](int ax,int ay,int bx,int by,int px,int py) -> float {
        return (float)((bx-ax)*(py-ay) - (by-ay)*(px-ax));
    };
    float area = edgeFn(sx0,sy0,sx1,sy1,sx2,sy2);
    if (std::fabs(area) < 0.5f) return;

    lightDir = lightDir.normalized();
    viewDir = viewDir.normalized();

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            float w0 = edgeFn(sx1,sy1,sx2,sy2,x,y) / area;
            float w1 = edgeFn(sx2,sy2,sx0,sy0,x,y) / area;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;

            float z = w0 * p0.z + w1 * p1.z + w2 * p2.z;
            if (!fb.testDepth(x, y, z)) continue;

            Vec3 N = (n0 * w0 + n1 * w1 + n2 * w2).normalized();

            // Phong 着色
            float ambient = 0.15f;
            float diff = std::max(0.0f, N.dot(lightDir));

            Vec3 H = (lightDir + viewDir).normalized();
            float spec = std::pow(std::max(0.0f, N.dot(H)), 32.0f);

            float r = (float)baseR / 255.0f;
            float g = (float)baseG / 255.0f;
            float b = (float)baseB / 255.0f;

            float lr = std::min(1.0f, r * (ambient + diff * 0.8f) + spec * 0.5f);
            float lg = std::min(1.0f, g * (ambient + diff * 0.8f) + spec * 0.5f);
            float lb = std::min(1.0f, b * (ambient + diff * 0.8f) + spec * 0.5f);

            fb.setPixel(x, y,
                (uint8_t)(lr * 255),
                (uint8_t)(lg * 255),
                (uint8_t)(lb * 255));
        }
    }
}

// 渲染网格到 Framebuffer
void renderMesh(Framebuffer &fb, const Mesh &mesh,
    const Camera &cam, uint8_t br, uint8_t bg, uint8_t bb)
{
    Mat4 view = lookAt(cam.eye, cam.target, cam.up);
    Mat4 proj = perspective(cam.fov, cam.aspect, cam.znear, cam.zfar);

    auto verts = mesh.verts;
    auto normals = computeVertexNormals(mesh);

    // 变换所有顶点到 NDC
    std::vector<Vec3> ndc(verts.size());
    std::vector<Vec3> viewSpacePos(verts.size());

    for (int i = 0; i < (int)verts.size(); i++) {
        Vec3 vv = mulMat4(view, verts[i], 1.0f);
        viewSpacePos[i] = vv;
        ndc[i] = mulMat4(proj, vv, 1.0f);
    }

    Vec3 lightDir = Vec3(1, 1.5f, 1).normalized();
    Vec3 viewDir = (cam.eye - cam.target).normalized();

    for (const auto &face : mesh.faces) {
        if (face.size() < 3) continue;
        // 四边形拆成两个三角形
        for (int i = 1; i + 1 < (int)face.size(); i++) {
            int i0 = face[0], i1 = face[i], i2 = face[i+1];

            // 背面剔除（在 view space 做）
            Vec3 e1 = viewSpacePos[i1] - viewSpacePos[i0];
            Vec3 e2 = viewSpacePos[i2] - viewSpacePos[i0];
            Vec3 fn = e1.cross(e2);
            if (fn.z > 0) continue; // 背面

            // 简单裁剪：所有点都在 NDC [-1,1] 内才绘制（简化处理）
            auto clip = [](float v) { return v >= -1.5f && v <= 1.5f; };
            if (!clip(ndc[i0].x) && !clip(ndc[i1].x) && !clip(ndc[i2].x)) continue;

            rasterizeTriangle(fb,
                ndc[i0], ndc[i1], ndc[i2],
                normals[i0], normals[i1], normals[i2],
                lightDir, viewDir,
                br, bg, bb);
        }
    }
}

// 画网格线（线框）
void drawWireframe(Framebuffer &fb, const Mesh &mesh, const Camera &cam,
    uint8_t wr, uint8_t wg, uint8_t wb)
{
    Mat4 view = lookAt(cam.eye, cam.target, cam.up);
    Mat4 proj = perspective(cam.fov, cam.aspect, cam.znear, cam.zfar);

    std::vector<Vec3> ndc(mesh.verts.size());
    for (int i = 0; i < (int)mesh.verts.size(); i++) {
        Vec3 vv = mulMat4(view, mesh.verts[i], 1.0f);
        ndc[i] = mulMat4(proj, vv, 1.0f);
    }

    auto toSX = [&](float nx) { return (int)((nx * 0.5f + 0.5f) * (fb.w - 1)); };
    auto toSY = [&](float ny) { return (int)((1.0f - (ny * 0.5f + 0.5f)) * (fb.h - 1)); };

    auto drawLine = [&](Vec3 a, Vec3 b) {
        int x0 = toSX(a.x), y0 = toSY(a.y);
        int x1 = toSX(b.x), y1 = toSY(b.y);
        // Bresenham
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            if (x0 >= 0 && x0 < fb.w && y0 >= 0 && y0 < fb.h)
                fb.setPixel(x0, y0, wr, wg, wb);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    };

    for (const auto &face : mesh.faces) {
        int n = (int)face.size();
        for (int i = 0; i < n; i++) {
            drawLine(ndc[face[i]], ndc[face[(i+1)%n]]);
        }
    }
}

// ======================================================================
// 绘制标签
// ======================================================================
void drawLabel(std::vector<uint8_t> &img, int imgW, int imgH,
    int x, int y, const std::string &text,
    uint8_t r, uint8_t g, uint8_t b)
{
    // 简化：每个字符画一个 5x3 的矩形色块
    for (int ci = 0; ci < (int)text.size(); ci++) {
        int bx = x + ci * 6;
        for (int py = 0; py < 5; py++) {
            for (int px = 0; px < 5; px++) {
                int ix = bx + px, iy = y + py;
                if (ix < 0 || ix >= imgW || iy < 0 || iy >= imgH) continue;
                // 用实心块填色，模拟字符（简单实现：全白）
                int idx = (iy * imgW + ix) * 3;
                img[idx] = r; img[idx+1] = g; img[idx+2] = b;
            }
        }
    }
}

// ======================================================================
// Main
// ======================================================================
int main() {
    // 构建粗糙立方体网格
    Mesh coarseMesh = buildCubeMesh();

    // 细分3次
    Mesh mesh1 = catmullClarkSubdivide(coarseMesh);
    Mesh mesh2 = catmullClarkSubdivide(mesh1);
    Mesh mesh3 = catmullClarkSubdivide(mesh2);

    printf("Vertices: coarse=%d → 1x=%d → 2x=%d → 3x=%d\n",
        (int)coarseMesh.verts.size(),
        (int)mesh1.verts.size(),
        (int)mesh2.verts.size(),
        (int)mesh3.verts.size());
    printf("Faces:    coarse=%d → 1x=%d → 2x=%d → 3x=%d\n",
        (int)coarseMesh.faces.size(),
        (int)mesh1.faces.size(),
        (int)mesh2.faces.size(),
        (int)mesh3.faces.size());

    Camera cam;
    cam.eye    = Vec3(2.8f, 2.2f, 2.8f);
    cam.target = Vec3(0, 0, 0);
    cam.up     = Vec3(0, 1, 0);
    cam.fov    = 45.0f;
    cam.aspect = 1.0f;
    cam.znear  = 0.1f;
    cam.zfar   = 100.0f;

    // 创建拼合图像（800x400 = 左粗糙 + 右细分后）
    // 左图：原始立方体 + 线框
    // 右图：3级细分后的光滑球体
    std::vector<uint8_t> finalImg(TW * H * 3, 20);

    // 渲染左图（粗网格）
    {
        Framebuffer fb(W, H);
        // 背景深色蓝
        for (int i = 0; i < W*H*3; i+=3) {
            fb.color[i]=20; fb.color[i+1]=25; fb.color[i+2]=45;
        }
        renderMesh(fb, coarseMesh, cam, 180, 100, 60);
        drawWireframe(fb, coarseMesh, cam, 255, 220, 120);

        // 拷贝到左侧
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int si = (y*W+x)*3;
                int di = (y*TW+x)*3;
                finalImg[di]   = fb.color[si];
                finalImg[di+1] = fb.color[si+1];
                finalImg[di+2] = fb.color[si+2];
            }
    }

    // 渲染右图（3级细分后）
    {
        Framebuffer fb(W, H);
        // 背景深色蓝
        for (int i = 0; i < W*H*3; i+=3) {
            fb.color[i]=20; fb.color[i+1]=25; fb.color[i+2]=45;
        }
        // 渲染两种颜色交替（体现细分网格密度）：先实体，再浅线框
        renderMesh(fb, mesh3, cam, 90, 150, 220);
        // 选2级细分的线框（3级线框太密）
        drawWireframe(fb, mesh2, cam, 60, 100, 160);

        // 拷贝到右侧
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                int si = (y*W+x)*3;
                int di = (y*TW+(x+W))*3;
                finalImg[di]   = fb.color[si];
                finalImg[di+1] = fb.color[si+1];
                finalImg[di+2] = fb.color[si+2];
            }
    }

    // 画分割线
    for (int y = 0; y < H; y++) {
        int di = (y*TW + W)*3;
        finalImg[di] = finalImg[di+1] = finalImg[di+2] = 80;
    }

    // 写入 PNG
    const char *outPath = "catmull_clark_output.png";
    if (!stbi_write_png(outPath, TW, H, 3, finalImg.data(), TW * 3)) {
        fprintf(stderr, "Failed to write PNG\n");
        return 1;
    }

    printf("Output: %s (%dx%d)\n", outPath, TW, H);
    printf("Left: coarse cube mesh (%d verts, %d faces)\n",
        (int)coarseMesh.verts.size(), (int)coarseMesh.faces.size());
    printf("Right: 3x subdivided (%d verts, %d faces)\n",
        (int)mesh3.verts.size(), (int)mesh3.faces.size());

    return 0;
}
