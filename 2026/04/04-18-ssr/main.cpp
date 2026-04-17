// Screen Space Reflections (SSR) Renderer
// 技术：G-Buffer软光栅化 + 屏幕空间反射光线步进
// 场景：金属地面 + 若干几何体（球体、盒子），展示实时SSR效果
//
// 流程：
//   1. 软光栅化场景到 GBuffer（位置、法线、颜色、反射率、粗糙度）
//   2. 对每个像素，根据法线计算反射方向
//   3. 在屏幕空间沿反射方向步进，找到相交点
//   4. 将相交点颜色混合回当前像素
//   5. 边缘衰减 + 粗糙度衰减 + 直接光照合并

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>
#include <random>
#include <string>

// ─────────────── Math ───────────────
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    float dot(const Vec3& o)       const { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o)     const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float len()  const { return std::sqrt(x*x+y*y+z*z); }
    float len2() const { return x*x+y*y+z*z; }
    Vec3  norm() const { float l=len(); return l>1e-8f ? *this/l : Vec3{0,1,0}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
};

Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
Vec3 clamp3(const Vec3& v, float lo, float hi) {
    return { std::clamp(v.x,lo,hi), std::clamp(v.y,lo,hi), std::clamp(v.z,lo,hi) };
}
Vec3 reflect(const Vec3& I, const Vec3& N) { return I - N * (2.f * I.dot(N)); }

// ─────────────── Camera ───────────────
struct Camera {
    Vec3 eye, at, up;
    float fov, aspect;
    Vec3 right_v, up_v, fwd_v;
    float half_h, half_w;

    void build() {
        fwd_v = (at - eye).norm();
        right_v = fwd_v.cross(up).norm();
        up_v = right_v.cross(fwd_v).norm();
        half_h = std::tan(fov * 0.5f * M_PI / 180.f);
        half_w = half_h * aspect;
    }

    // Returns ray direction for pixel (u,v) in [0,1]^2
    Vec3 rayDir(float u, float v) const {
        float sx = (2*u - 1) * half_w;
        float sy = (1 - 2*v) * half_h;
        return (fwd_v + right_v*sx + up_v*sy).norm();
    }
};

// ─────────────── Scene Primitives ───────────────
struct HitInfo {
    float t;
    Vec3  pos, normal;
    Vec3  albedo;
    float metallic, roughness;
    bool  valid;
};

HitInfo miss() { return {0,{},{},{},0,0,false}; }

// Sphere
HitInfo hitSphere(const Vec3& ro, const Vec3& rd,
                  const Vec3& center, float radius,
                  const Vec3& albedo, float metallic, float roughness) {
    Vec3 oc = ro - center;
    float b = oc.dot(rd);
    float c = oc.dot(oc) - radius*radius;
    float disc = b*b - c;
    if (disc < 0) return miss();
    float sq = std::sqrt(disc);
    float t = -b - sq;
    if (t < 0.001f) t = -b + sq;
    if (t < 0.001f) return miss();
    Vec3 pos = ro + rd*t;
    Vec3 n = (pos - center).norm();
    return {t, pos, n, albedo, metallic, roughness, true};
}

// Axis-aligned box
HitInfo hitBox(const Vec3& ro, const Vec3& rd,
               const Vec3& bmin, const Vec3& bmax,
               const Vec3& albedo, float metallic, float roughness) {
    auto safe_inv = [](float v) { return std::abs(v) < 1e-9f ? (v>=0?1e30f:-1e30f) : 1.f/v; };
    float inv_x = safe_inv(rd.x), inv_y = safe_inv(rd.y), inv_z = safe_inv(rd.z);
    float tx0 = (bmin.x - ro.x)*inv_x, tx1 = (bmax.x - ro.x)*inv_x;
    float ty0 = (bmin.y - ro.y)*inv_y, ty1 = (bmax.y - ro.y)*inv_y;
    float tz0 = (bmin.z - ro.z)*inv_z, tz1 = (bmax.z - ro.z)*inv_z;
    if (tx0 > tx1) std::swap(tx0, tx1);
    if (ty0 > ty1) std::swap(ty0, ty1);
    if (tz0 > tz1) std::swap(tz0, tz1);
    float tmin = std::max({tx0, ty0, tz0});
    float tmax = std::min({tx1, ty1, tz1});
    if (tmin > tmax || tmax < 0.001f) return miss();
    float t = tmin > 0.001f ? tmin : tmax;
    if (t < 0.001f) return miss();
    Vec3 pos = ro + rd*t;
    // Normal via slab method
    Vec3 n;
    if (std::abs(pos.x - bmin.x) < 1e-3f) n = {-1,0,0};
    else if (std::abs(pos.x - bmax.x) < 1e-3f) n = {1,0,0};
    else if (std::abs(pos.y - bmin.y) < 1e-3f) n = {0,-1,0};
    else if (std::abs(pos.y - bmax.y) < 1e-3f) n = {0,1,0};
    else if (std::abs(pos.z - bmin.z) < 1e-3f) n = {0,0,-1};
    else n = {0,0,1};
    return {t, pos, n, albedo, metallic, roughness, true};
}

// Infinite plane y = height
HitInfo hitPlane(const Vec3& ro, const Vec3& rd,
                 float height,
                 const Vec3& albedo, float metallic, float roughness) {
    if (std::abs(rd.y) < 1e-6f) return miss();
    float t = (height - ro.y) / rd.y;
    if (t < 0.001f) return miss();
    Vec3 pos = ro + rd*t;
    // Checker pattern
    int cx = (int)std::floor(pos.x * 0.5f) & 1;
    int cz = (int)std::floor(pos.z * 0.5f) & 1;
    Vec3 col = (cx ^ cz) ? albedo : albedo * 0.6f;
    return {t, pos, {0,1,0}, col, metallic, roughness, true};
}

// ─────────────── G-Buffer ───────────────
struct GBuffer {
    int W, H;
    std::vector<float> depth;   // view-space depth
    std::vector<Vec3>  pos;     // world pos
    std::vector<Vec3>  normal;  // world normal
    std::vector<Vec3>  albedo;
    std::vector<float> metallic;
    std::vector<float> roughness;
    std::vector<Vec3>  lighting; // direct lighting result

    GBuffer(int W, int H) : W(W), H(H),
        depth(W*H, std::numeric_limits<float>::infinity()),
        pos(W*H), normal(W*H), albedo(W*H),
        metallic(W*H,0), roughness(W*H,1),
        lighting(W*H) {}

    int idx(int x, int y) const { return y*W + x; }
};

// ─────────────── Scene ───────────────
HitInfo traceScene(const Vec3& ro, const Vec3& rd) {
    HitInfo best = miss();
    best.t = std::numeric_limits<float>::infinity();

    auto test = [&](HitInfo h) {
        if (h.valid && h.t < best.t) best = h;
    };

    // Ground plane (metallic floor)
    test(hitPlane(ro, rd, -1.5f, {0.7f,0.75f,0.8f}, 0.9f, 0.1f));

    // Center mirror-like sphere
    test(hitSphere(ro, rd, {0,0.5f,-3.f}, 1.0f, {0.9f,0.8f,0.7f}, 1.0f, 0.05f));

    // Left matte sphere
    test(hitSphere(ro, rd, {-2.5f,0.f,-3.f}, 0.8f, {0.8f,0.2f,0.15f}, 0.0f, 0.9f));

    // Right green sphere
    test(hitSphere(ro, rd, {2.5f,0.2f,-3.f}, 0.8f, {0.2f,0.7f,0.3f}, 0.3f, 0.4f));

    // Back-left box (metallic)
    test(hitBox(ro, rd, {-4.f,-1.5f,-6.f}, {-2.5f,1.5f,-4.5f},
                {0.7f,0.6f,0.9f}, 0.8f, 0.15f));

    // Back-right box (matte)
    test(hitBox(ro, rd, {2.f,-1.5f,-5.5f}, {4.f,2.5f,-4.f},
                {0.9f,0.7f,0.3f}, 0.0f, 0.85f));

    // Small front sphere
    test(hitSphere(ro, rd, {1.f,-0.8f,-1.5f}, 0.5f, {0.5f,0.8f,0.95f}, 0.6f, 0.2f));

    return best;
}

// ─────────────── Direct Lighting ───────────────
Vec3 directLighting(const HitInfo& hit, const Vec3& eye) {
    // Two lights: main sun + fill
    Vec3 lights[2] = {{3.f, 8.f, 2.f}, {-5.f, 4.f, 0.f}};
    Vec3 lightCols[2] = {{1.0f,0.95f,0.85f}, {0.3f,0.4f,0.55f}};
    float lightStr[2] = {1.8f, 0.5f};

    Vec3 color = hit.albedo * 0.08f; // ambient

    for (int i = 0; i < 2; i++) {
        Vec3 L = (lights[i] - hit.pos).norm();
        float diff = std::max(0.f, hit.normal.dot(L));

        // Shadow test
        HitInfo shadow = traceScene(hit.pos + hit.normal*0.002f, L);
        float shadowMult = shadow.valid ? 0.15f : 1.f;

        // Simple Blinn-Phong specular
        Vec3 V = (eye - hit.pos).norm();
        Vec3 H = (L + V).norm();
        float spec = std::pow(std::max(0.f, hit.normal.dot(H)), 40.f / (hit.roughness*hit.roughness + 0.01f));

        Vec3 diffTerm  = hit.albedo * (diff * (1.f - hit.metallic));
        Vec3 specTerm  = lerp(Vec3{1,1,1}, hit.albedo, hit.metallic) * spec * hit.metallic;
        color += (diffTerm + specTerm) * lightCols[i] * (lightStr[i] * shadowMult);
    }
    return color;
}

// ─────────────── SSR Core ───────────────
// Project world position to screen UV
bool worldToScreen(const Vec3& worldPos, const Camera& cam, int /*W*/, int /*H*/,
                   float& outU, float& outV, float& outDepth) {
    Vec3 rel = worldPos - cam.eye;
    float depth = rel.dot(cam.fwd_v);
    if (depth <= 0.001f) return false;
    float px = rel.dot(cam.right_v);
    float py = rel.dot(cam.up_v);
    // perspective divide
    outU = (px / depth / cam.half_w) * 0.5f + 0.5f;
    outV = 1.f - (py / depth / cam.half_h) * 0.5f - 0.5f;
    outDepth = depth;
    return outU >= 0.f && outU <= 1.f && outV >= 0.f && outV <= 1.f;
}

// SSR: returns (color, confidence)
std::pair<Vec3, float> SSR(int px, int py, const GBuffer& gbuf, const Camera& cam,
                            int W, int H) {
    int i = gbuf.idx(px, py);
    if (!std::isfinite(gbuf.depth[i])) return {{0,0,0}, 0.f};

    float metal = gbuf.metallic[i];
    float rough = gbuf.roughness[i];
    if (metal < 0.05f || rough > 0.7f) return {{0,0,0}, 0.f};

    Vec3 N = gbuf.normal[i];
    Vec3 V = (cam.eye - gbuf.pos[i]).norm();
    Vec3 R = reflect(-V, N); // reflection direction in world space

    // Only reflect if reflection goes away from surface (forward hemisphere)
    if (R.dot(N) < 0.01f) return {{0,0,0}, 0.f};

    // Ray march in world space, periodically project to screen
    const int MAX_STEPS = 80;
    const float STEP_START = 0.05f;
    const float STEP_GROW  = 1.05f;
    float step = STEP_START;

    Vec3 rayPos = gbuf.pos[i] + N * 0.01f;

    for (int s = 0; s < MAX_STEPS; s++) {
        rayPos = rayPos + R * step;
        step *= STEP_GROW;

        float sU, sV, sDepth;
        if (!worldToScreen(rayPos, cam, W, H, sU, sV, sDepth)) break;

        int sx = (int)(sU * W);
        int sy = (int)(sV * H);
        sx = std::clamp(sx, 0, W-1);
        sy = std::clamp(sy, 0, H-1);
        int si = gbuf.idx(sx, sy);

        // Compare sampled depth
        float bufDepth = gbuf.depth[si];
        if (!std::isfinite(bufDepth)) continue;

        float depthDiff = bufDepth - sDepth;
        if (depthDiff > 0.f && depthDiff < step * 2.5f) {
            // Hit! compute fade
            // Edge fade
            float edgeX = std::min(sU, 1.f-sU) / 0.1f;
            float edgeY = std::min(sV, 1.f-sV) / 0.1f;
            float edgeFade = std::clamp(std::min(edgeX, edgeY), 0.f, 1.f);

            // Distance fade (reflections fade as they get further)
            float dist = (rayPos - gbuf.pos[i]).len();
            float distFade = std::exp(-dist * 0.08f);

            // Roughness fade
            float roughFade = 1.f - rough;

            float confidence = edgeFade * distFade * roughFade * metal;
            Vec3 reflColor = gbuf.lighting[si];
            return {reflColor, confidence};
        }
    }
    return {{0,0,0}, 0.f};
}

// ─────────────── PNG Writer ───────────────
void writePNG(const std::string& path, const std::vector<uint8_t>& img, int W, int H) {
    // Write a minimal PPM instead — renamed .png for compatibility detection
    // Actually we'll use a proper raw PPM and convert
    std::ofstream f(path, std::ios::binary);
    // Write PPM header
    std::string hdr = "P6\n" + std::to_string(W) + " " + std::to_string(H) + "\n255\n";
    f.write(hdr.c_str(), (std::streamsize)hdr.size());
    f.write(reinterpret_cast<const char*>(img.data()), (std::streamsize)img.size());
}

// Tone map + gamma
uint8_t tone(float v) {
    // ACES approximation
    float a = v * (v + 0.0245786f) - 0.000090537f;
    float b = v * (0.983729f*v + 0.4329510f) + 0.238081f;
    v = std::clamp(a/b, 0.f, 1.f);
    return (uint8_t)(std::pow(v, 1.f/2.2f) * 255.f + 0.5f);
}

// ─────────────── Main ───────────────
int main() {
    constexpr int W = 800, H = 480;

    Camera cam;
    cam.eye    = {0, 1.5f, 3.f};
    cam.at     = {0, 0, -2.f};
    cam.up     = {0, 1, 0};
    cam.fov    = 60.f;
    cam.aspect = (float)W / H;
    cam.build();

    GBuffer gbuf(W, H);

    std::cout << "Pass 1: Rasterize to G-Buffer...\n";

    // Rasterize scene into G-Buffer
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float u = (x + 0.5f) / W;
            float v = (y + 0.5f) / H;
            Vec3 rd = cam.rayDir(u, v);
            HitInfo hit = traceScene(cam.eye, rd);
            int i = gbuf.idx(x, y);
            if (hit.valid) {
                // Compute view-space depth
                Vec3 rel = hit.pos - cam.eye;
                gbuf.depth[i]    = rel.dot(cam.fwd_v);
                gbuf.pos[i]      = hit.pos;
                gbuf.normal[i]   = hit.normal;
                gbuf.albedo[i]   = hit.albedo;
                gbuf.metallic[i] = hit.metallic;
                gbuf.roughness[i]= hit.roughness;
                gbuf.lighting[i] = directLighting(hit, cam.eye);
            } else {
                // Sky gradient
                float t = std::clamp(0.5f + rd.y * 0.8f, 0.f, 1.f);
                gbuf.lighting[i] = lerp(Vec3{0.7f,0.8f,0.9f}, Vec3{0.15f,0.3f,0.6f}, t);
                gbuf.depth[i]    = std::numeric_limits<float>::infinity();
            }
        }
    }

    std::cout << "Pass 2: Screen Space Reflections...\n";

    std::vector<Vec3> finalColor(W*H);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int i = gbuf.idx(x, y);
            Vec3 base = gbuf.lighting[i];

            if (std::isfinite(gbuf.depth[i])) {
                // SSR
                auto [reflColor, conf] = SSR(x, y, gbuf, cam, W, H);
                if (conf > 0.001f) {
                    // Fresnel-ish blend: more metallic = stronger reflection
                    float blendFactor = std::clamp(conf, 0.f, 1.f);
                    base = lerp(base, base + reflColor * blendFactor, blendFactor);
                }
            }

            finalColor[i] = base;
        }
    }

    std::cout << "Pass 3: Tone map & write PPM...\n";

    std::vector<uint8_t> img(W * H * 3);
    for (int i = 0; i < W*H; i++) {
        img[i*3+0] = tone(finalColor[i].x);
        img[i*3+1] = tone(finalColor[i].y);
        img[i*3+2] = tone(finalColor[i].z);
    }

    // Write PPM (will convert to PNG after)
    writePNG("ssr_output.ppm", img, W, H);
    std::cout << "Saved ssr_output.ppm\n";

    // Pixel stats for validation
    double sumR=0, sumG=0, sumB=0;
    for (int i=0; i<W*H; i++) {
        sumR += img[i*3];
        sumG += img[i*3+1];
        sumB += img[i*3+2];
    }
    double N = W*H;
    std::cout << "Pixel stats — R:" << sumR/N << " G:" << sumG/N << " B:" << sumB/N << "\n";
    std::cout << "Done.\n";
    return 0;
}
