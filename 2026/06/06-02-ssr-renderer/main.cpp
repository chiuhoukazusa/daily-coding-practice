/*
 * Screen-Space Reflections (SSR) Renderer
 * ========================================
 * 技术要点：
 *   - 软光栅化渲染管线（无 OpenGL）
 *   - G-Buffer：颜色、法线、深度、粗糙度
 *   - 屏幕空间线性步进反射（Ray Marching in Screen Space）
 *   - Fresnel 菲涅尔混合（Schlick 近似）
 *   - 粗糙度遮罩（高粗糙度减弱反射）
 *   - 反射衰减（边缘淡出）
 *   - 场景：地面镜面 + 多个彩色球体
 *   - 输出：ssr_output.png（1024×576）
 *
 * 编译：g++ main.cpp -o ssr_renderer -std=c++17 -O2 -Wall -Wextra
 * 运行：./ssr_renderer
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// ─────────────────────────── Math Primitives ────────────────────────────────

struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o)const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float length()            const { return std::sqrt(x*x+y*y+z*z); }
    Vec3  normalize()         const { float l=length(); return l>1e-9f ? *this/l : Vec3(0,0,0); }
    float operator[](int i)   const { return (&x)[i]; }
    float& operator[](int i)        { return (&x)[i]; }
};
inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline Vec3 clamp3(const Vec3& v, float lo, float hi) {
    return { std::clamp(v.x,lo,hi), std::clamp(v.y,lo,hi), std::clamp(v.z,lo,hi) };
}
inline Vec3 lerp3(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
inline float saturate(float v) { return std::clamp(v, 0.0f, 1.0f); }

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(1) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    Vec3 xyz() const { return {x,y,z}; }
};

struct Mat4 {
    float m[4][4];
    Mat4() { std::memset(m, 0, sizeof(m)); }
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
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
    static Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
        Mat4 r;
        float f = 1.0f / std::tan(fovY * 0.5f);
        r.m[0][0] = f / aspect;
        r.m[1][1] = f;
        r.m[2][2] = (zFar + zNear) / (zNear - zFar);
        r.m[2][3] = 2.0f * zFar * zNear / (zNear - zFar);
        r.m[3][2] = -1.0f;
        return r;
    }
    static Mat4 lookAt(const Vec3& eye, const Vec3& at, const Vec3& up) {
        Vec3 f = (at - eye).normalize();
        Vec3 r = f.cross(up).normalize();
        Vec3 u = r.cross(f);
        Mat4 m = identity();
        m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
        m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
        m.m[2][0]=-f.x; m.m[2][1]=-f.y; m.m[2][2]=-f.z; m.m[2][3]=f.dot(eye);
        m.m[3][3]=1;
        return m;
    }
    Mat4 transpose() const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) r.m[i][j]=m[j][i];
        return r;
    }
};

// ─────────────────────────── G-Buffer ───────────────────────────────────────

static const int W = 1024, H = 576;

struct GBuffer {
    std::vector<Vec3>  color;      // albedo/base color
    std::vector<Vec3>  normal;     // view-space normal
    std::vector<float> depth;      // NDC depth
    std::vector<float> roughness;
    std::vector<float> metallic;
    std::vector<Vec3>  posVS;      // view-space position

    GBuffer() : color(W*H), normal(W*H), depth(W*H, 1.0f),
                roughness(W*H, 1.0f), metallic(W*H, 0.0f), posVS(W*H) {}
    int idx(int x, int y) const { return y*W + x; }
};

struct Framebuffer {
    std::vector<Vec3>  color;
    Framebuffer() : color(W*H, Vec3(0,0,0)) {}
    int idx(int x, int y) const { return y*W + x; }
};

// ─────────────────────────── Scene Objects ──────────────────────────────────

struct Sphere {
    Vec3  center;
    float radius;
    Vec3  albedo;
    float roughness;
    float metallic;
    bool  emissive;
    Vec3  emissionColor;
    float emissionStrength;
};

// Transform a Vec3 by Mat4 (point, w=1)
inline Vec3 transformPoint(const Mat4& m, const Vec3& p) {
    Vec4 v = m * Vec4(p, 1.0f);
    return v.xyz() / v.w;
}

// Transform direction (w=0)
inline Vec3 transformDir(const Mat4& m, const Vec3& d) {
    Vec4 v = m * Vec4(d, 0.0f);
    return v.xyz();
}

// ─────────────────────────── Rasterizer ─────────────────────────────────────

struct Vertex {
    Vec3 posVS;   // view-space position
    Vec3 posNDC;  // after perspective
    Vec2 screen;  // pixel coords
    Vec3 normal;  // view-space normal
};

Vertex projectVertex(const Vec3& posVS, const Vec3& normalVS, const Mat4& proj) {
    Vertex v;
    v.posVS  = posVS;
    v.normal = normalVS;
    Vec4 clip = proj * Vec4(posVS, 1.0f);
    if(std::abs(clip.w) < 1e-9f) { v.posNDC = {0,0,2}; return v; }
    Vec3 ndc = clip.xyz() / clip.w;
    v.posNDC = ndc;
    v.screen.x = (ndc.x * 0.5f + 0.5f) * W;
    v.screen.y = (1.0f - (ndc.y * 0.5f + 0.5f)) * H;
    return v;
}

// Rasterize a triangle into the G-Buffer (with depth test)
void rasterizeTriangle(
    const Vertex& v0, const Vertex& v1, const Vertex& v2,
    const Vec3& albedo, float roughness, float metallic,
    GBuffer& gbuf)
{
    // Bounding box
    int x0 = std::clamp((int)std::floor(std::min({v0.screen.x, v1.screen.x, v2.screen.x})), 0, W-1);
    int x1 = std::clamp((int)std::ceil (std::max({v0.screen.x, v1.screen.x, v2.screen.x})), 0, W-1);
    int y0 = std::clamp((int)std::floor(std::min({v0.screen.y, v1.screen.y, v2.screen.y})), 0, H-1);
    int y1 = std::clamp((int)std::ceil (std::max({v0.screen.y, v1.screen.y, v2.screen.y})), 0, H-1);

    // Triangle area (screen space)
    float ax = v1.screen.x - v0.screen.x, ay = v1.screen.y - v0.screen.y;
    float bx = v2.screen.x - v0.screen.x, by = v2.screen.y - v0.screen.y;
    float area = ax * by - ay * bx;
    if(std::abs(area) < 0.5f) return;

    for(int py = y0; py <= y1; py++) {
        for(int px = x0; px <= x1; px++) {
            float cx = (float)px + 0.5f - v0.screen.x;
            float cy = (float)py + 0.5f - v0.screen.y;
            float u = (cx * by - cy * bx) / area;
            float v = (ax * cy - ay * cx) / area;
            float w = 1.0f - u - v;
            if(u < 0 || v < 0 || w < 0) continue;

            // Depth
            float d = w * v0.posNDC.z + u * v1.posNDC.z + v * v2.posNDC.z;
            if(d < -1.0f || d > 1.0f) continue;
            int i = gbuf.idx(px, py);
            if(d >= gbuf.depth[i]) continue;

            gbuf.depth[i]     = d;
            gbuf.color[i]     = albedo;
            gbuf.roughness[i] = roughness;
            gbuf.metallic[i]  = metallic;

            // Interpolate view-space normal + position
            Vec3 n = (v0.normal * w + v1.normal * u + v2.normal * v).normalize();
            gbuf.normal[i] = n;
            Vec3 p = v0.posVS * w + v1.posVS * u + v2.posVS * v;
            gbuf.posVS[i] = p;
        }
    }
}

// Generate sphere mesh (view-space)
void renderSphere(const Sphere& sph, const Mat4& view, const Mat4& proj, GBuffer& gbuf) {
    const int stacks = 28, slices = 36;
    // Vertices: (stack+1) x (slices+1)
    std::vector<Vec3> vsPos, vsNorm;
    for(int si = 0; si <= stacks; si++) {
        float phi = (float)si / stacks * (float)M_PI;
        for(int sl = 0; sl <= slices; sl++) {
            float theta = (float)sl / slices * 2.0f * (float)M_PI;
            Vec3 localN = {
                std::sin(phi) * std::cos(theta),
                std::cos(phi),
                std::sin(phi) * std::sin(theta)
            };
            Vec3 worldP = sph.center + localN * sph.radius;
            Vec4 viewP4 = view * Vec4(worldP, 1.0f);
            Vec3 viewP  = viewP4.xyz();
            // Normal in view space (assumes uniform scale)
            Vec4 viewN4 = view * Vec4(localN, 0.0f);
            Vec3 viewN  = viewN4.xyz().normalize();
            vsPos.push_back(viewP);
            vsNorm.push_back(viewN);
        }
    }

    Vec3 albedo    = sph.emissive ? sph.emissionColor : sph.albedo;
    float roughness = sph.roughness;
    float metallic  = sph.metallic;

    for(int si = 0; si < stacks; si++) {
        for(int sl = 0; sl < slices; sl++) {
            int i00 = si*(slices+1)+sl;
            int i10 = i00+1;
            int i01 = i00+(slices+1);
            int i11 = i01+1;
            Vertex v00 = projectVertex(vsPos[i00], vsNorm[i00], proj);
            Vertex v10 = projectVertex(vsPos[i10], vsNorm[i10], proj);
            Vertex v01 = projectVertex(vsPos[i01], vsNorm[i01], proj);
            Vertex v11 = projectVertex(vsPos[i11], vsNorm[i11], proj);
            rasterizeTriangle(v00, v10, v01, albedo, roughness, metallic, gbuf);
            rasterizeTriangle(v10, v11, v01, albedo, roughness, metallic, gbuf);
        }
    }
}

// Render a horizontal quad (the floor)
void renderFloor(float y, float minXZ, float maxXZ,
                 const Vec3& albedo, float roughness, float metallic,
                 const Mat4& view, const Mat4& proj, GBuffer& gbuf)
{
    // 4 corners, subdivide into grid to reduce perspective artifact
    const int N = 20;
    float step = (maxXZ - minXZ) / N;
    Vec3 up = {0, 1, 0};
    for(int iz = 0; iz < N; iz++) {
        for(int ix = 0; ix < N; ix++) {
            float x0 = minXZ + ix*step, x1 = x0 + step;
            float z0 = minXZ + iz*step, z1 = z0 + step;
            Vec3 p00(x0,y,z0), p10(x1,y,z0), p01(x0,y,z1), p11(x1,y,z1);
            auto toVS = [&](const Vec3& p, const Vec3& n) -> Vertex {
                Vec4 vp = view * Vec4(p, 1.0f);
                Vec4 vn = view * Vec4(n, 0.0f);
                return projectVertex(vp.xyz(), vn.xyz().normalize(), proj);
            };
            Vertex v00 = toVS(p00, up);
            Vertex v10 = toVS(p10, up);
            Vertex v01 = toVS(p01, up);
            Vertex v11 = toVS(p11, up);
            rasterizeTriangle(v00, v10, v11, albedo, roughness, metallic, gbuf);
            rasterizeTriangle(v00, v11, v01, albedo, roughness, metallic, gbuf);
        }
    }
}

// ─────────────────────────── Lighting (Blinn-Phong for base color) ──────────

Vec3 shadeDirect(const Vec3& posVS, const Vec3& normalVS, const Vec3& albedo,
                 float roughness, float metallic,
                 const std::vector<Vec3>& lightPosVS,
                 const std::vector<Vec3>& lightColors)
{
    Vec3 viewDir = (-posVS).normalize();
    Vec3 result(0,0,0);

    // Ambient
    Vec3 ambient = albedo * Vec3(0.08f, 0.08f, 0.12f);
    result += ambient;

    for(size_t li = 0; li < lightPosVS.size(); li++) {
        Vec3 L = (lightPosVS[li] - posVS);
        float dist2 = L.dot(L);
        float dist  = std::sqrt(dist2);
        L = L / dist;
        float attenuation = 1.0f / (1.0f + 0.05f * dist + 0.005f * dist2);

        float NdotL = std::max(0.0f, normalVS.dot(L));
        Vec3 H = (L + viewDir).normalize();
        float NdotH = std::max(0.0f, normalVS.dot(H));

        // Diffuse
        float kD = (1.0f - metallic);
        Vec3 diffuse = albedo * (lightColors[li] * (NdotL * kD * attenuation));

        // Specular (Blinn-Phong adapted for roughness)
        float shininess = std::max(1.0f, (1.0f - roughness) * (1.0f - roughness) * 512.0f);
        float spec = std::pow(NdotH, shininess) * NdotL;
        Vec3 F0 = lerp3(Vec3(0.04f, 0.04f, 0.04f), albedo, metallic);
        Vec3 specular = F0 * (lightColors[li] * (spec * attenuation));

        result += diffuse + specular;
    }
    return result;
}

// ─────────────────────────── SSR ────────────────────────────────────────────

// Convert NDC depth to view-space Z
// Proj: standard perspective, zNear=0.1, zFar=100
static const float Z_NEAR = 0.1f, Z_FAR = 100.0f;
float ndcDepthToViewZ(float ndcZ) {
    // For GL-style projection: z_view = (2*zn*zf) / (zf+zn - ndcZ*(zf-zn))
    // But our proj stores z = (zf+zn)/(zn-zf) + 2*zf*zn/((zn-zf)*w)
    // w=-z_view → viewZ = -clip.w
    // Simplify: ndcZ in [-1,1], z_ndc = (a*z_view + b) / (-z_view)
    float a = (Z_FAR + Z_NEAR) / (Z_NEAR - Z_FAR);
    float b = 2.0f * Z_FAR * Z_NEAR / (Z_NEAR - Z_FAR);
    // ndcZ = a + b/(-viewZ)  → viewZ = -b / (ndcZ - a)
    float viewZ = -b / (ndcZ - a);
    return viewZ; // negative (camera looks along -Z)
}

// Project view-space point to screen pixel (returns false if out of screen)
bool projectToScreen(const Vec3& posVS, const Mat4& proj, float& outPx, float& outPy, float& outNdcZ) {
    Vec4 clip = proj * Vec4(posVS, 1.0f);
    if(clip.w <= 0.0f) return false;
    Vec3 ndc = clip.xyz() / clip.w;
    if(ndc.x < -1 || ndc.x > 1 || ndc.y < -1 || ndc.y > 1) return false;
    outNdcZ = ndc.z;
    outPx = (ndc.x * 0.5f + 0.5f) * W;
    outPy = (1.0f - (ndc.y * 0.5f + 0.5f)) * H;
    return true;
}

// Fresnel Schlick
float fresnelSchlick(float cosTheta, float F0) {
    return F0 + (1.0f - F0) * std::pow(1.0f - saturate(cosTheta), 5.0f);
}

// SSR: trace a reflection ray in screen space, return hit color or miss
Vec3 ssrTrace(
    int px, int py,
    const GBuffer& gbuf,
    const Framebuffer& shaded,
    const Mat4& proj)
{
    int i = gbuf.idx(px, py);
    float ndcDepth = gbuf.depth[i];
    if(ndcDepth >= 1.0f) return Vec3(0,0,0); // sky

    Vec3 posVS   = gbuf.posVS[i];
    Vec3 normalVS = gbuf.normal[i];
    float rough  = gbuf.roughness[i];

    if(rough > 0.85f) return Vec3(0,0,0); // too rough, skip

    Vec3 viewDir = (-posVS).normalize();
    Vec3 reflDir = ((-viewDir) - normalVS * 2.0f * (-viewDir).dot(normalVS)).normalize();
    // reflDir = 2*(N·V)*N - V  (where V = -view direction toward surface)
    Vec3 V = viewDir;
    reflDir = (normalVS * 2.0f * normalVS.dot(V) - V).normalize();

    if(reflDir.z > -0.05f) return Vec3(0,0,0); // reflect away from camera

    // March the reflection ray in view space
    const int MAX_STEPS = 64;
    const float STEP_SIZE_INITIAL = 0.1f;
    float stepSize = STEP_SIZE_INITIAL;

    Vec3 rayPos = posVS + normalVS * 0.02f; // tiny offset to avoid self-hit

    for(int step = 0; step < MAX_STEPS; step++) {
        rayPos = rayPos + reflDir * stepSize;
        stepSize *= 1.05f; // exponential step

        float spx, spy, spNdcZ;
        if(!projectToScreen(rayPos, proj, spx, spy, spNdcZ)) break;

        int sx = (int)spx, sy = (int)spy;
        if(sx < 0 || sx >= W || sy < 0 || sy >= H) break;

        float gbufDepth = gbuf.depth[gbuf.idx(sx, sy)];
        if(gbufDepth >= 1.0f) continue; // sky, no intersection

        // Compare NDC Z
        float rayNdcZ = spNdcZ;
        if(rayNdcZ > gbufDepth && (rayNdcZ - gbufDepth) < 0.05f) {
            // Hit!
            // Compute screen-edge fade
            float edgeX = std::min(spx / W, 1.0f - spx / W);
            float edgeY = std::min(spy / H, 1.0f - spy / H);
            float edgeFade = saturate(edgeX * 8.0f) * saturate(edgeY * 8.0f);

            // Distance fade
            float distFade = saturate(1.0f - (float)step / MAX_STEPS);

            Vec3 hitColor = shaded.color[gbuf.idx(sx, sy)];
            return hitColor * (edgeFade * distFade);
        }
    }
    return Vec3(0,0,0); // no hit
}

// ─────────────────────────── Sky ────────────────────────────────────────────

Vec3 skyColor(float /*px*/, float py) {
    // Simple gradient sky
    float t = saturate(1.0f - py / H);
    Vec3 horizonColor(0.7f, 0.82f, 0.98f);
    Vec3 zenithColor(0.25f, 0.45f, 0.78f);
    Vec3 groundColor(0.55f, 0.50f, 0.45f);
    if(t > 0.5f) {
        return lerp3(horizonColor, zenithColor, (t - 0.5f) * 2.0f);
    } else {
        return lerp3(groundColor, horizonColor, t * 2.0f);
    }
}

// ─────────────────────────── Main ───────────────────────────────────────────

int main() {
    std::cout << "[SSR] Starting Screen-Space Reflections Renderer..." << std::endl;

    // Camera setup
    Vec3 eye(0.0f, 2.5f, 8.0f);
    Vec3 at (0.0f, 0.5f, 0.0f);
    Vec3 upV(0.0f, 1.0f, 0.0f);

    Mat4 view = Mat4::lookAt(eye, at, upV);
    Mat4 proj = Mat4::perspective(
        45.0f * (float)M_PI / 180.0f,
        (float)W / H,
        Z_NEAR, Z_FAR);

    // Lights (defined in view space later)
    std::vector<Vec3> lightWorldPos = {
        {  4.0f, 6.0f,  3.0f },
        { -3.0f, 5.0f,  4.0f },
        {  0.0f, 8.0f, -2.0f }
    };
    std::vector<Vec3> lightColors = {
        { 1.2f, 1.0f, 0.85f },
        { 0.6f, 0.7f, 1.2f  },
        { 0.9f, 0.9f, 0.9f  }
    };

    // Transform lights to view space
    std::vector<Vec3> lightVS;
    for(auto& lp : lightWorldPos) {
        Vec4 lv = view * Vec4(lp, 1.0f);
        lightVS.push_back(lv.xyz());
    }

    // Scene objects
    std::vector<Sphere> spheres = {
        // Center mirror sphere
        { {0.0f, 1.0f, 0.0f}, 1.0f, {0.95f,0.95f,0.97f}, 0.02f, 1.0f, false, {}, 0.0f },
        // Colored spheres
        { {-2.5f, 0.7f, 0.5f}, 0.7f, {0.9f,0.2f,0.15f}, 0.3f, 0.0f, false, {}, 0.0f },
        { { 2.5f, 0.7f, 0.5f}, 0.7f, {0.15f,0.6f,0.9f}, 0.2f, 0.0f, false, {}, 0.0f },
        { {-1.5f, 0.5f, 2.0f}, 0.5f, {0.95f,0.85f,0.1f}, 0.15f, 0.8f, false, {}, 0.0f },
        { { 1.5f, 0.5f, 2.0f}, 0.5f, {0.1f,0.9f,0.4f},  0.25f, 0.0f, false, {}, 0.0f },
        { { 0.0f, 0.5f,-2.0f}, 0.5f, {0.9f,0.5f,0.9f},  0.1f,  0.5f, false, {}, 0.0f },
        // Small emissive spheres (area lights)
        { { 3.5f, 3.0f, -1.0f}, 0.3f, {0,0,0}, 0.0f, 0.0f, true, {1.2f,0.8f,0.4f}, 3.0f },
        { {-3.5f, 3.0f, -1.0f}, 0.3f, {0,0,0}, 0.0f, 0.0f, true, {0.4f,0.8f,1.2f}, 3.0f },
    };

    // ── Step 1: Render scene into G-Buffer ──────────────────────────────────
    std::cout << "[SSR] Rendering G-Buffer..." << std::endl;
    GBuffer gbuf;

    // Floor (highly reflective)
    renderFloor(-0.01f, -8.0f, 8.0f,
                Vec3(0.85f, 0.85f, 0.85f), 0.05f, 0.0f,
                view, proj, gbuf);

    // Spheres
    for(auto& sph : spheres) {
        renderSphere(sph, view, proj, gbuf);
    }

    std::cout << "[SSR] G-Buffer complete. Computing direct shading..." << std::endl;

    // ── Step 2: Direct shading into framebuffer ─────────────────────────────
    Framebuffer shaded;
    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            int i = gbuf.idx(px, py);
            if(gbuf.depth[i] >= 1.0f) {
                // Sky
                shaded.color[i] = skyColor((float)px, (float)py);
            } else {
                shaded.color[i] = shadeDirect(
                    gbuf.posVS[i], gbuf.normal[i],
                    gbuf.color[i], gbuf.roughness[i], gbuf.metallic[i],
                    lightVS, lightColors);
            }
        }
    }

    std::cout << "[SSR] Direct shading complete. Computing SSR..." << std::endl;

    // ── Step 3: SSR pass ────────────────────────────────────────────────────
    Framebuffer final_buf;
    int ssrPixels = 0;
    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            int i = gbuf.idx(px, py);
            Vec3 baseColor = shaded.color[i];

            if(gbuf.depth[i] < 1.0f && gbuf.roughness[i] < 0.85f) {
                Vec3 posVS   = gbuf.posVS[i];
                Vec3 normalVS = gbuf.normal[i];
                Vec3 viewDir = (-posVS).normalize();
                float NdotV  = saturate(normalVS.dot(viewDir));
                float F0     = lerp3(Vec3(0.04f,0.04f,0.04f), gbuf.color[i], gbuf.metallic[i]).x;
                float fresnel = fresnelSchlick(NdotV, F0 + (1.0f - gbuf.roughness[i]) * 0.5f);
                fresnel = std::clamp(fresnel, 0.0f, 0.9f);

                Vec3 reflColor = ssrTrace(px, py, gbuf, shaded, proj);
                bool hit = reflColor.length() > 1e-6f;
                if(hit) ssrPixels++;

                // Blend: base + fresnel * reflection
                baseColor = baseColor + reflColor * fresnel * (1.0f - gbuf.roughness[i]);
            }

            final_buf.color[i] = clamp3(baseColor, 0.0f, 1.0f);
        }
        if(py % 100 == 0) {
            std::cout << "[SSR] Progress: " << py * 100 / H << "%" << std::endl;
        }
    }
    std::cout << "[SSR] SSR complete. Reflection pixels: " << ssrPixels << std::endl;

    // ── Step 4: Tone map + gamma correct ───────────────────────────────────
    // Reinhard tone mapping
    std::vector<uint8_t> pixels(W * H * 3);
    for(int i = 0; i < W*H; i++) {
        Vec3 c = final_buf.color[i];
        // Reinhard
        c.x = c.x / (c.x + 1.0f);
        c.y = c.y / (c.y + 1.0f);
        c.z = c.z / (c.z + 1.0f);
        // Gamma 2.2
        c.x = std::pow(c.x, 1.0f/2.2f);
        c.y = std::pow(c.y, 1.0f/2.2f);
        c.z = std::pow(c.z, 1.0f/2.2f);
        pixels[i*3+0] = (uint8_t)(std::clamp(c.x, 0.0f, 1.0f) * 255.0f);
        pixels[i*3+1] = (uint8_t)(std::clamp(c.y, 0.0f, 1.0f) * 255.0f);
        pixels[i*3+2] = (uint8_t)(std::clamp(c.z, 0.0f, 1.0f) * 255.0f);
    }

    // ── Step 5: Write PNG ───────────────────────────────────────────────────
    const char* outFile = "ssr_output.png";
    int written = stbi_write_png(outFile, W, H, 3, pixels.data(), W*3);
    if(!written) {
        std::cerr << "[SSR] ERROR: Failed to write PNG!" << std::endl;
        return 1;
    }

    std::cout << "[SSR] Written: " << outFile << std::endl;
    std::cout << "[SSR] Resolution: " << W << "x" << H << std::endl;
    std::cout << "[SSR] Done!" << std::endl;
    return 0;
}
