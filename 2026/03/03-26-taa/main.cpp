/**
 * Temporal Anti-Aliasing (TAA) Renderer
 * 
 * 实现完整的TAA流程：
 * 1. 抖动采样（Halton序列）：每帧使用不同的子像素偏移
 * 2. 几何场景光栅化（软光栅，包含深度测试）
 * 3. 运动向量计算（基于物体运动的重投影）
 * 4. 历史缓冲区混合（alpha=0.1混合因子）
 * 5. 邻域颜色钳制（防止鬼影）
 * 6. 输出对比：无抗锯齿 vs TAA结果
 * 
 * 场景：旋转的多边形场景（含锯齿感强的斜线/细节）
 * 输出：side-by-side对比图 taa_output.png
 */

#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <algorithm>
#include <random>
#include <functional>

// ============================================================
// STB Image Write (inline minimal version)
// ============================================================
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ============================================================
// Math Types
// ============================================================
struct Vec2 {
    float x, y;
    Vec2(float x=0, float y=0): x(x), y(y){}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
};

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>1e-9f?Vec3{x/l,y/l,z/l}:Vec3{0,0,1}; }
    Vec3 clamp(float lo, float hi) const {
        return {std::max(lo,std::min(hi,x)),
                std::max(lo,std::min(hi,y)),
                std::max(lo,std::min(hi,z))};
    }
};

// ============================================================
// Image Buffer
// ============================================================
struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    Image(int w, int h): w(w), h(h), pixels(w*h, Vec3{0,0,0}) {}
    Vec3& at(int x, int y) { return pixels[y*w+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*w+x]; }
    Vec3 sample(float u, float v) const {
        // Bilinear sampling
        float fx = u * (w-1), fy = v * (h-1);
        int x0 = (int)fx, y0 = (int)fy;
        int x1 = std::min(x0+1, w-1), y1 = std::min(y0+1, h-1);
        float tx = fx - x0, ty = fy - y0;
        x0 = std::max(0, std::min(w-1, x0));
        y0 = std::max(0, std::min(h-1, y0));
        Vec3 c00 = at(x0,y0), c10 = at(x1,y0);
        Vec3 c01 = at(x0,y1), c11 = at(x1,y1);
        Vec3 top = c00*(1-tx) + c10*tx;
        Vec3 bot = c01*(1-tx) + c11*tx;
        return top*(1-ty) + bot*ty;
    }
    Vec3 operator+(const Vec3& v) const { (void)v; return {}; }
};

// ============================================================
// Depth Buffer
// ============================================================
struct DepthBuffer {
    int w, h;
    std::vector<float> data;
    DepthBuffer(int w, int h): w(w), h(h), data(w*h, 1e9f) {}
    float& at(int x, int y) { return data[y*w+x]; }
    void clear() { std::fill(data.begin(), data.end(), 1e9f); }
};

// ============================================================
// Motion Vector Buffer
// ============================================================
struct MotionBuffer {
    int w, h;
    std::vector<Vec2> data;
    MotionBuffer(int w, int h): w(w), h(h), data(w*h, Vec2{0,0}) {}
    Vec2& at(int x, int y) { return data[y*w+x]; }
    const Vec2& at(int x, int y) const { return data[y*w+x]; }
    void clear() { std::fill(data.begin(), data.end(), Vec2{0,0}); }
};

// ============================================================
// Halton Sequence for Jitter
// ============================================================
float halton(int index, int base) {
    float result = 0.0f;
    float f = 1.0f;
    int i = index;
    while (i > 0) {
        f /= base;
        result += f * (i % base);
        i /= base;
    }
    return result;
}

Vec2 getJitter(int frameIndex, int width, int height) {
    // Halton(2,3) sequence, centered at 0
    float jx = halton(frameIndex+1, 2) - 0.5f;
    float jy = halton(frameIndex+1, 3) - 0.5f;
    // Convert to pixel space (we'll add this in NDC)
    return Vec2{jx / width, jy / height};
}

// ============================================================
// Simple Rasterizer
// ============================================================
struct Vertex {
    Vec3 pos;   // world space
    Vec3 color;
};

struct Triangle {
    Vertex v[3];
    Triangle() = default;
    Triangle(Vertex a, Vertex b, Vertex c) { v[0]=a; v[1]=b; v[2]=c; }
};

// Project world pos to NDC (simple orthographic with jitter)
Vec3 project(Vec3 worldPos, float angleRad, Vec2 jitter, float scale) {
    // Rotate around Z axis
    float c = cosf(angleRad), s = sinf(angleRad);
    float rx = worldPos.x * c - worldPos.y * s;
    float ry = worldPos.x * s + worldPos.y * c;
    float rz = worldPos.z;
    
    // Scale + translate to NDC [-1, 1]
    float ndcX = rx * scale + jitter.x * 2.0f;
    float ndcY = ry * scale + jitter.y * 2.0f;
    return Vec3{ndcX, ndcY, rz};
}

Vec3 ndcToScreen(Vec3 ndc, int w, int h) {
    return Vec3{(ndc.x + 1.0f) * 0.5f * (w-1),
                (1.0f - ndc.y) * 0.5f * (h-1),
                ndc.z};
}

// Edge function for rasterization
float edgeFunc(Vec2 a, Vec2 b, Vec2 p) {
    return (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
}

void rasterizeTriangle(
    const Triangle& tri,
    float angle, Vec2 jitter, float scale,
    float prevAngle, // for motion vectors
    Image& colorBuf,
    DepthBuffer& depthBuf,
    MotionBuffer& motionBuf)
{
    int W = colorBuf.w, H = colorBuf.h;
    
    // Project current frame
    Vec3 p0 = ndcToScreen(project(tri.v[0].pos, angle, jitter, scale), W, H);
    Vec3 p1 = ndcToScreen(project(tri.v[1].pos, angle, jitter, scale), W, H);
    Vec3 p2 = ndcToScreen(project(tri.v[2].pos, angle, jitter, scale), W, H);
    
    // Project previous frame (no jitter for motion vectors)
    Vec3 pp0 = ndcToScreen(project(tri.v[0].pos, prevAngle, Vec2{0,0}, scale), W, H);
    Vec3 pp1 = ndcToScreen(project(tri.v[1].pos, prevAngle, Vec2{0,0}, scale), W, H);
    Vec3 pp2 = ndcToScreen(project(tri.v[2].pos, prevAngle, Vec2{0,0}, scale), W, H);
    
    // AABB
    float minX = std::min({p0.x, p1.x, p2.x});
    float maxX = std::max({p0.x, p1.x, p2.x});
    float minY = std::min({p0.y, p1.y, p2.y});
    float maxY = std::max({p0.y, p1.y, p2.y});
    
    int x0 = std::max(0, (int)floorf(minX));
    int x1 = std::min(W-1, (int)ceilf(maxX));
    int y0 = std::max(0, (int)floorf(minY));
    int y1 = std::min(H-1, (int)ceilf(maxY));
    
    Vec2 s0{p0.x, p0.y}, s1{p1.x, p1.y}, s2{p2.x, p2.y};
    Vec2 ps0{pp0.x, pp0.y}, ps1{pp1.x, pp1.y}, ps2{pp2.x, pp2.y};
    
    float area = edgeFunc(s0, s1, s2);
    if (fabsf(area) < 1e-6f) return;
    
    for (int y = y0; y <= y1; y++) {
        for (int x = x0; x <= x1; x++) {
            Vec2 p{(float)x + 0.5f, (float)y + 0.5f};
            float w0 = edgeFunc(s1, s2, p);
            float w1 = edgeFunc(s2, s0, p);
            float w2 = edgeFunc(s0, s1, p);
            
            if (area > 0 && (w0 < 0 || w1 < 0 || w2 < 0)) continue;
            if (area < 0 && (w0 > 0 || w1 > 0 || w2 > 0)) continue;
            
            w0 /= area; w1 /= area; w2 /= area;
            
            float depth = p0.z * w0 + p1.z * w1 + p2.z * w2;
            if (depth >= depthBuf.at(x,y)) continue;
            depthBuf.at(x,y) = depth;
            
            // Interpolate color
            Vec3 col = tri.v[0].color * w0 + tri.v[1].color * w1 + tri.v[2].color * w2;
            colorBuf.at(x,y) = col;
            
            // Compute motion vector (current pos -> previous pos)
            Vec2 prevP{ps0.x*w0 + ps1.x*w1 + ps2.x*w2,
                       ps0.y*w0 + ps1.y*w1 + ps2.y*w2};
            motionBuf.at(x,y) = Vec2{prevP.x - p.x, prevP.y - p.y};
        }
    }
}

// ============================================================
// TAA Resolve
// ============================================================
Vec3 clipToAABB(Vec3 history, Vec3 minC, Vec3 maxC) {
    // Clamp history color to the AABB of current frame neighbors
    return Vec3{
        std::max(minC.x, std::min(maxC.x, history.x)),
        std::max(minC.y, std::min(maxC.y, history.y)),
        std::max(minC.z, std::min(maxC.z, history.z))
    };
}

void taaResolve(
    const Image& current,
    const Image& history,
    const MotionBuffer& motion,
    Image& output,
    float blendFactor) // typically 0.1 (10% current, 90% history)
{
    int W = current.w, H = current.h;
    
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 currColor = current.at(x, y);
            
            // Compute neighborhood AABB (3x3)
            Vec3 minC{1e9f,1e9f,1e9f}, maxC{-1e9f,-1e9f,-1e9f};
            Vec3 m1{0,0,0}, m2{0,0,0};
            int count = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int nx = std::max(0, std::min(W-1, x+dx));
                    int ny = std::max(0, std::min(H-1, y+dy));
                    Vec3 c = current.at(nx, ny);
                    minC.x = std::min(minC.x, c.x);
                    minC.y = std::min(minC.y, c.y);
                    minC.z = std::min(minC.z, c.z);
                    maxC.x = std::max(maxC.x, c.x);
                    maxC.y = std::max(maxC.y, c.y);
                    maxC.z = std::max(maxC.z, c.z);
                    m1 = m1 + c;
                    m2 = m2 + Vec3{c.x*c.x, c.y*c.y, c.z*c.z};
                    count++;
                }
            }
            
            // Variance clip (tighter than simple AABB)
            Vec3 mu = m1 * (1.0f/count);
            Vec3 var{m2.x/count - mu.x*mu.x,
                     m2.y/count - mu.y*mu.y,
                     m2.z/count - mu.z*mu.z};
            Vec3 sigma{sqrtf(std::max(0.0f, var.x)),
                       sqrtf(std::max(0.0f, var.y)),
                       sqrtf(std::max(0.0f, var.z))};
            float gamma = 1.25f;
            Vec3 varMinC = mu - sigma*gamma;
            Vec3 varMaxC = mu + sigma*gamma;
            // Use tighter of the two
            Vec3 clipMin{std::max(minC.x, varMinC.x),
                         std::max(minC.y, varMinC.y),
                         std::max(minC.z, varMinC.z)};
            Vec3 clipMax{std::min(maxC.x, varMaxC.x),
                         std::min(maxC.y, varMaxC.y),
                         std::min(maxC.z, varMaxC.z)};
            
            // Motion vector reprojection
            Vec2 mv = motion.at(x, y);
            float prevX = x + mv.x;
            float prevY = y + mv.y;
            
            Vec3 histColor;
            if (prevX >= 0 && prevX < W-1 && prevY >= 0 && prevY < H-1) {
                // Bilinear sample history
                float u = prevX / (W-1), v = prevY / (H-1);
                histColor = history.sample(u, v);
            } else {
                // Out of bounds: no history, use current
                histColor = currColor;
            }
            
            // Clamp history to neighborhood
            histColor = clipToAABB(histColor, clipMin, clipMax);
            
            // Blend: output = blend * current + (1-blend) * history
            Vec3 result = currColor * blendFactor + histColor * (1.0f - blendFactor);
            output.at(x, y) = result.clamp(0.0f, 1.0f);
        }
    }
}

// ============================================================
// Build Scene Triangles
// ============================================================
std::vector<Triangle> buildScene() {
    std::vector<Triangle> tris;
    
    // Large background quad (two triangles)
    // Floor/background
    tris.push_back(Triangle(
        Vertex{{-3.0f, -3.0f, 0.5f}, {0.12f, 0.12f, 0.18f}},
        Vertex{{ 3.0f, -3.0f, 0.5f}, {0.15f, 0.12f, 0.20f}},
        Vertex{{ 3.0f,  3.0f, 0.5f}, {0.10f, 0.15f, 0.18f}}
    ));;
    tris.push_back(Triangle(
        Vertex{{-3.0f, -3.0f, 0.5f}, {0.12f, 0.12f, 0.18f}},
        Vertex{{ 3.0f,  3.0f, 0.5f}, {0.10f, 0.15f, 0.18f}},
        Vertex{{-3.0f,  3.0f, 0.5f}, {0.13f, 0.14f, 0.19f}}
    ));;
    
    // Center large hexagon (6 triangles) - main aliasing source
    auto addHexTri = [&](float a0, float a1, Vec3 col) {
        float r = 1.0f;
        tris.push_back(Triangle(
            Vertex{{0.0f, 0.0f, 0.0f}, col},
            Vertex{{r*cosf(a0), r*sinf(a0), 0.0f}, col * 0.9f},
            Vertex{{r*cosf(a1), r*sinf(a1), 0.0f}, col * 0.85f}
        ));;
    };
    
    Vec3 hexColors[] = {
        {0.9f, 0.2f, 0.2f}, {0.9f, 0.6f, 0.1f},
        {0.2f, 0.8f, 0.2f}, {0.1f, 0.6f, 0.9f},
        {0.6f, 0.1f, 0.9f}, {0.9f, 0.4f, 0.6f}
    };
    for (int i = 0; i < 6; i++) {
        float a0 = i * 3.14159f/3.0f;
        float a1 = (i+1) * 3.14159f/3.0f;
        addHexTri(a0, a1, hexColors[i]);
    }
    
    // Thin star shape (high-frequency, alias-prone)
    auto addStarTip = [&](float angle, float innerR, float outerR, Vec3 col) {
        float a0 = angle - 3.14159f/8.0f;
        float a1 = angle + 3.14159f/8.0f;
        tris.push_back(Triangle(
            Vertex{{innerR*cosf(a0), innerR*sinf(a0), -0.1f}, col * 0.7f},
            Vertex{{outerR*cosf(angle), outerR*sinf(angle), -0.1f}, col},
            Vertex{{innerR*cosf(a1), innerR*sinf(a1), -0.1f}, col * 0.7f}
        ));;
    };
    
    Vec3 starCol{1.0f, 0.95f, 0.4f};
    for (int i = 0; i < 8; i++) {
        float a = i * 2.0f * 3.14159f / 8.0f;
        addStarTip(a, 1.1f, 1.7f, starCol);
    }
    
    // Small thin triangles at corners (aliasing-prone diagonal edges)
    auto addCornerTri = [&](float cx, float cy, float size, Vec3 col) {
        tris.push_back(Triangle(
            Vertex{{cx, cy + size, -0.2f}, col},
            Vertex{{cx - size*0.5f, cy - size*0.5f, -0.2f}, col * 0.8f},
            Vertex{{cx + size*0.5f, cy - size*0.5f, -0.2f}, col * 0.6f}
        ));;
    };
    
    addCornerTri(-1.8f, -1.8f, 0.35f, Vec3{0.3f, 0.9f, 0.8f});
    addCornerTri( 1.8f, -1.8f, 0.35f, Vec3{0.9f, 0.5f, 0.2f});
    addCornerTri(-1.8f,  1.8f, 0.35f, Vec3{0.5f, 0.3f, 0.9f});
    addCornerTri( 1.8f,  1.8f, 0.35f, Vec3{0.2f, 0.8f, 0.4f});
    
    // Fine grid lines (very alias-prone)
    auto addThinRect = [&](float x0, float y0, float x1, float y1, float thick, Vec3 col) {
        Vec3 dir{x1-x0, y1-y0, 0};
        float len2 = sqrtf(dir.x*dir.x + dir.y*dir.y);
        if (len2 < 1e-6f) return;
        Vec3 n{-dir.y/len2, dir.x/len2, 0};
        Vec3 A{x0 + n.x*thick, y0 + n.y*thick, -0.05f};
        Vec3 B{x0 - n.x*thick, y0 - n.y*thick, -0.05f};
        Vec3 C{x1 + n.x*thick, y1 + n.y*thick, -0.05f};
        Vec3 D{x1 - n.x*thick, y1 - n.y*thick, -0.05f};
        tris.push_back(Triangle(Vertex{A,col}, Vertex{B,col}, Vertex{C,col}));;
        tris.push_back(Triangle(Vertex{B,col}, Vertex{D,col}, Vertex{C,col}));;
    };
    
    // Diagonal lines (maximum alias)
    addThinRect(-2.2f, -2.2f, 2.2f, 2.2f, 0.025f, Vec3{1.0f, 1.0f, 0.5f});
    addThinRect(-2.2f,  2.2f, 2.2f, -2.2f, 0.025f, Vec3{0.5f, 1.0f, 1.0f});
    addThinRect(-2.5f, 0.0f, 2.5f, 0.0f, 0.018f, Vec3{0.8f, 0.8f, 0.8f});
    addThinRect( 0.0f, -2.5f, 0.0f, 2.5f, 0.018f, Vec3{0.8f, 0.8f, 0.8f});
    
    return tris;
}

// ============================================================
// Render one frame
// ============================================================
void renderFrame(
    const std::vector<Triangle>& scene,
    float angle, float prevAngle,
    Vec2 jitter, float scale,
    Image& colorOut,
    DepthBuffer& depthBuf,
    MotionBuffer& motionBuf)
{
    // Clear
    for (int y = 0; y < colorOut.h; y++)
        for (int x = 0; x < colorOut.w; x++)
            colorOut.at(x,y) = Vec3{0.05f, 0.05f, 0.08f};
    depthBuf.clear();
    motionBuf.clear();
    
    for (const auto& tri : scene) {
        rasterizeTriangle(tri, angle, jitter, scale, prevAngle, colorOut, depthBuf, motionBuf);
    }
}

// ============================================================
// Gamma correction
// ============================================================
Vec3 gammaCorrect(Vec3 c) {
    return Vec3{
        powf(std::max(0.0f, std::min(1.0f, c.x)), 1.0f/2.2f),
        powf(std::max(0.0f, std::min(1.0f, c.y)), 1.0f/2.2f),
        powf(std::max(0.0f, std::min(1.0f, c.z)), 1.0f/2.2f)
    };
}

// ============================================================
// SSAA reference (for ground truth)
// ============================================================
void renderSSAA(
    const std::vector<Triangle>& scene,
    float angle,
    int W, int H, int supersample,
    Image& output)
{
    int SW = W * supersample, SH = H * supersample;
    Image hiRes(SW, SH);
    DepthBuffer hiDepth(SW, SH);
    MotionBuffer hiMotion(SW, SH);
    
    for (int y = 0; y < SH; y++)
        for (int x = 0; x < SW; x++)
            hiRes.at(x,y) = Vec3{0.05f, 0.05f, 0.08f};
    hiDepth.clear();
    hiMotion.clear();
    
    float scale = 0.35f;
    for (const auto& tri : scene) {
        rasterizeTriangle(tri, angle, Vec2{0,0}, scale, angle, hiRes, hiDepth, hiMotion);
    }
    
    // Downsample
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 sum{0,0,0};
            for (int sy = 0; sy < supersample; sy++)
                for (int sx = 0; sx < supersample; sx++)
                    sum = sum + hiRes.at(x*supersample+sx, y*supersample+sy);
            float inv = 1.0f/(supersample*supersample);
            output.at(x,y) = Vec3{sum.x*inv, sum.y*inv, sum.z*inv};
        }
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    const int W = 512, H = 512;
    const float SCALE = 0.35f;
    const int TAA_FRAMES = 16;  // Accumulate 16 frames
    const float TAA_BLEND = 0.1f; // 10% new, 90% history
    
    auto scene = buildScene();
    
    printf("TAA Renderer\n");
    printf("Resolution: %dx%d\n", W, H);
    printf("TAA frames: %d, blend factor: %.1f%%\n", TAA_FRAMES, TAA_BLEND*100);
    
    // ---- Phase 1: Render without AA (single frame, no jitter) ----
    printf("\n[1/3] Rendering no-AA frame...\n");
    Image noAA(W, H);
    DepthBuffer noAADepth(W, H);
    MotionBuffer noAAMotion(W, H);
    float baseAngle = 0.15f; // slight rotation to show diagonal aliasing
    renderFrame(scene, baseAngle, baseAngle, Vec2{0,0}, SCALE, noAA, noAADepth, noAAMotion);
    
    // ---- Phase 2: TAA accumulation ----
    printf("[2/3] Accumulating TAA (%d frames)...\n", TAA_FRAMES);
    Image taaHistory(W, H);
    Image taaCurrent(W, H);
    Image taaOutput(W, H);
    DepthBuffer taaDepth(W, H);
    MotionBuffer taaMotion(W, H);
    
    // Initialize history with first frame
    float firstAngle = baseAngle;
    renderFrame(scene, firstAngle, firstAngle, Vec2{0,0}, SCALE, taaHistory, taaDepth, taaMotion);
    
    // Accumulate frames with jitter
    float angularVelocity = 0.003f; // slow rotation per frame
    float currentAngle = baseAngle;
    
    for (int frame = 1; frame <= TAA_FRAMES; frame++) {
        float prevAngle = currentAngle;
        currentAngle = baseAngle + frame * angularVelocity;
        
        Vec2 jitter = getJitter(frame, W, H);
        
        // Render current frame with jitter
        renderFrame(scene, currentAngle, prevAngle, jitter, SCALE, taaCurrent, taaDepth, taaMotion);
        
        // TAA resolve
        taaResolve(taaCurrent, taaHistory, taaMotion, taaOutput, TAA_BLEND);
        
        // Output becomes new history
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++)
                taaHistory.at(x,y) = taaOutput.at(x,y);
        
        if (frame % 4 == 0)
            printf("  Frame %d/%d done\n", frame, TAA_FRAMES);
    }
    
    // ---- Phase 3: SSAA reference (4x) ----
    printf("[3/3] Rendering 4x SSAA reference...\n");
    Image ssaaRef(W, H);
    renderSSAA(scene, baseAngle, W, H, 4, ssaaRef);
    
    // ---- Compose output: 3-panel comparison ----
    // Output: [No AA | TAA | SSAA Reference]
    // Total width: W*3 + 2*gap
    const int GAP = 8;
    int outW = W*3 + GAP*2, outH = H + 60;
    std::vector<uint8_t> outPixels(outW * outH * 3, 30);
    
    auto writePixel = [&](int x, int y, Vec3 c) {
        if (x < 0 || x >= outW || y < 0 || y >= outH) return;
        int idx = (y * outW + x) * 3;
        Vec3 gc = gammaCorrect(c);
        outPixels[idx+0] = (uint8_t)(gc.x * 255.0f);
        outPixels[idx+1] = (uint8_t)(gc.y * 255.0f);
        outPixels[idx+2] = (uint8_t)(gc.z * 255.0f);
    };
    
    // Panel offsets
    int offsets[3] = {0, W+GAP, W*2+GAP*2};
    Image* panels[3] = {&noAA, &taaOutput, &ssaaRef};
    
    for (int p = 0; p < 3; p++) {
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                writePixel(offsets[p] + x, y, panels[p]->at(x,y));
            }
        }
    }
    
    // Draw labels area (dark background)
    // Already dark from initialization
    
    // Simple pixel-font labels (using a minimal 5x7 bitmap font)
    // Since we can't embed a full font, we'll draw simple marker lines
    // Label bars at bottom
    auto drawRect = [&](int x, int y, int w, int h, Vec3 col) {
        for (int dy = 0; dy < h; dy++)
            for (int dx = 0; dx < w; dx++)
                writePixel(x+dx, y+dy, col);
    };
    
    // Color-coded label bars
    drawRect(offsets[0] + 10, H + 10, W-20, 6, Vec3{0.9f, 0.3f, 0.3f}); // No AA = red
    drawRect(offsets[1] + 10, H + 10, W-20, 6, Vec3{0.3f, 0.9f, 0.3f}); // TAA = green
    drawRect(offsets[2] + 10, H + 10, W-20, 6, Vec3{0.3f, 0.5f, 0.9f}); // SSAA = blue
    
    // Additional sub-labels (thin lines)
    // No AA indicator: jagged line to suggest aliasing
    for (int x = 0; x < W-20; x += 4) {
        int yy = H + 25 + (x/4) % 4;
        drawRect(offsets[0] + 10 + x, yy, 2, 2, Vec3{0.7f, 0.2f, 0.2f});
    }
    // TAA indicator: smooth line
    for (int x = 0; x < W-20; x++) {
        drawRect(offsets[1] + 10 + x, H + 27, 1, 1, Vec3{0.2f, 0.7f, 0.2f});
    }
    // SSAA indicator: double smooth line
    for (int x = 0; x < W-20; x++) {
        drawRect(offsets[2] + 10 + x, H + 25, 1, 1, Vec3{0.2f, 0.4f, 0.8f});
        drawRect(offsets[2] + 10 + x, H + 29, 1, 1, Vec3{0.2f, 0.4f, 0.8f});
    }
    
    // Save output
    printf("\nSaving taa_output.png...\n");
    int result = stbi_write_png("taa_output.png", outW, outH, 3, outPixels.data(), outW*3);
    if (result == 0) {
        fprintf(stderr, "ERROR: Failed to write output image!\n");
        return 1;
    }
    
    // Also save individual panels
    std::vector<uint8_t> panelPx(W*H*3);
    auto savePanel = [&](Image& img, const char* filename) {
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                int idx = (y*W+x)*3;
                Vec3 gc = gammaCorrect(img.at(x,y));
                panelPx[idx+0] = (uint8_t)(gc.x*255.0f);
                panelPx[idx+1] = (uint8_t)(gc.y*255.0f);
                panelPx[idx+2] = (uint8_t)(gc.z*255.0f);
            }
        }
        stbi_write_png(filename, W, H, 3, panelPx.data(), W*3);
    };
    
    savePanel(noAA, "taa_noaa.png");
    savePanel(taaOutput, "taa_result.png");
    savePanel(ssaaRef, "taa_ssaa.png");
    
    printf("\n✅ Done!\n");
    printf("Output files:\n");
    printf("  taa_output.png  - side-by-side comparison (%dx%d)\n", outW, outH);
    printf("  taa_noaa.png    - no anti-aliasing\n");
    printf("  taa_result.png  - TAA result (%d frames accumulated)\n", TAA_FRAMES);
    printf("  taa_ssaa.png    - 4x SSAA reference\n");
    
    // Print stats
    // Compute average absolute difference (measure of aliasing reduction)
    double diffNoAA = 0, diffTAA = 0;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 ref = ssaaRef.at(x,y);
            Vec3 na = noAA.at(x,y);
            Vec3 ta = taaOutput.at(x,y);
            diffNoAA += fabsf(na.x-ref.x) + fabsf(na.y-ref.y) + fabsf(na.z-ref.z);
            diffTAA  += fabsf(ta.x-ref.x) + fabsf(ta.y-ref.y) + fabsf(ta.z-ref.z);
        }
    }
    double norm = 3.0 * W * H;
    printf("\nQuality vs SSAA reference:\n");
    printf("  No-AA  MAE: %.4f\n", diffNoAA/norm);
    printf("  TAA    MAE: %.4f\n", diffTAA/norm);
    printf("  Improvement: %.1f%%\n", 100.0*(diffNoAA - diffTAA)/diffNoAA);
    
    return 0;
}
