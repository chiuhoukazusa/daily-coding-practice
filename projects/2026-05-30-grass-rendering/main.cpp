/*
 * Procedural Grass Rendering with Wind Simulation
 * ================================================
 * Techniques:
 *   - Procedural grass blade geometry (quadratic bezier curves)
 *   - Wind field simulation (sinusoidal displacement)
 *   - LOD based on distance (fewer blades further away)
 *   - Phong shading with ambient + diffuse
 *   - Ground plane with color variation
 *   - Sky gradient with sun
 *   - Depth sorting for transparency-like layering
 *   - Soft shadow approximation via AO
 *
 * Output: grass_output.png (800x600)
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

// ─────────────────────── Math ───────────────────────
struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float t) const { return {x * t, y * t, z * t}; }
    Vec3 operator*(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }
    Vec3 operator/(float t) const { return {x / t, y / t, z / t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }
    float length() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3 normalized() const {
        float l = length();
        if (l < 1e-8f) return {0, 1, 0};
        return *this / l;
    }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};

inline Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a * (1 - t) + b * t;
}
inline float clamp01(float v) { return std::max(0.f, std::min(1.f, v)); }
inline float clamp(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

// ─────────────────────── Framebuffer ───────────────────────
const int W = 800, H = 600;
Vec3 framebuf[H][W];
float depthbuf[H][W];

void clearBuffers() {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            framebuf[y][x] = {0, 0, 0};
            depthbuf[y][x] = 1e30f;
        }
}

// ─────────────────────── Camera / Projection ───────────────────────
struct Camera {
    Vec3 pos, forward, right, up;
    float fovY, aspect, nearZ;

    Camera() {
        pos = {0, 2.5f, 12.f};
        Vec3 target = {0, 0.5f, 0};
        forward = (target - pos).normalized();
        right = forward.cross({0, 1, 0}).normalized();
        up = right.cross(forward).normalized();
        fovY = 55.f * 3.14159f / 180.f;
        aspect = (float)W / H;
        nearZ = 0.1f;
    }

    // Returns depth (positive = visible), (ndcX, ndcY) in [-1,1]
    bool project(const Vec3& world, float& sx, float& sy, float& depth) const {
        Vec3 d = world - pos;
        float z = d.dot(forward);
        if (z < nearZ) return false;
        float px = d.dot(right);
        float py = d.dot(up);
        float halfH = std::tan(fovY * 0.5f) * z;
        float halfW = halfH * aspect;
        float ndcX = px / halfW;
        float ndcY = py / halfH;
        sx = (ndcX + 1.f) * 0.5f * W;
        sy = (1.f - (ndcY + 1.f) * 0.5f) * H;
        depth = z;
        return true;
    }
} cam;

// ─────────────────────── Rasterize Triangle ───────────────────────
void drawTriangle(const Vec3& p0, const Vec3& p1, const Vec3& p2,
                  const Vec3& c0, const Vec3& c1, const Vec3& c2,
                  float z0, float z1, float z2) {
    float ax = p0.x, ay = p0.y;
    float bx = p1.x, by = p1.y;
    float cx = p2.x, cy = p2.y;

    int minX = (int)std::max(0.f, std::min({ax, bx, cx}));
    int maxX = (int)std::min((float)(W - 1), std::max({ax, bx, cx}));
    int minY = (int)std::max(0.f, std::min({ay, by, cy}));
    int maxY = (int)std::min((float)(H - 1), std::max({ay, by, cy}));

    float denom = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
    if (std::abs(denom) < 1e-6f) return;
    float invDenom = 1.f / denom;

    for (int py = minY; py <= maxY; py++) {
        for (int px = minX; px <= maxX; px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            float w0 = ((by - cy) * (fx - cx) + (cx - bx) * (fy - cy)) * invDenom;
            float w1 = ((cy - ay) * (fx - cx) + (ax - cx) * (fy - cy)) * invDenom;
            float w2 = 1.f - w0 - w1;
            if (w0 < -0.01f || w1 < -0.01f || w2 < -0.01f) continue;
            float depth = w0 * z0 + w1 * z1 + w2 * z2;
            if (depth >= depthbuf[py][px]) continue;
            depthbuf[py][px] = depth;
            framebuf[py][px] = c0 * w0 + c1 * w1 + c2 * w2;
        }
    }
}

// ─────────────────────── Sky ───────────────────────
Vec3 skyColor(float ndcY) {
    // ndcY in [0,1] (0=top,1=bottom of image)
    // Sky gradient: top=deep blue, horizon=light blue/orange
    float t = ndcY; // 0 = top, 1 = bottom
    Vec3 zenith = {0.1f, 0.3f, 0.7f};
    Vec3 horizon = {0.7f, 0.85f, 0.95f};
    return mix(zenith, horizon, std::pow(t, 0.5f));
}

void renderSky() {
    Vec3 sunDir = Vec3(0.6f, 0.8f, 0.4f).normalized();
    for (int y = 0; y < H; y++) {
        float fy = (float)y / H;
        Vec3 sky = skyColor(fy);
        for (int x = 0; x < W; x++) {
            float fx = (float)x / W;
            // Sun disk
            float ndcX = fx * 2 - 1;
            float ndcY2 = (1 - fy) * 2 - 1;
            Vec3 rayDir = (cam.right * ndcX * std::tan(cam.fovY * 0.5f) * cam.aspect
                         + cam.up * ndcY2 * std::tan(cam.fovY * 0.5f)
                         + cam.forward).normalized();
            float sunDot = rayDir.dot(sunDir);
            if (sunDot > 0.998f) {
                sky = {1.f, 0.95f, 0.8f};
            } else if (sunDot > 0.99f) {
                float t = (sunDot - 0.99f) / 0.008f;
                sky = mix(sky, Vec3(1.f, 0.9f, 0.6f), t * t);
            }
            framebuf[y][x] = sky;
            depthbuf[y][x] = 1e30f;
        }
    }
}

// ─────────────────────── Wind ───────────────────────
// Wind field: sin-based displacement at world (x,z), time=0 (static frame)
float windStrength(float worldX, float worldZ) {
    // Multiple frequency wind
    float w = std::sin(worldX * 0.5f + worldZ * 0.3f) * 0.5f
            + std::sin(worldX * 1.1f - worldZ * 0.7f) * 0.3f
            + std::sin(worldX * 0.2f + worldZ * 1.4f) * 0.2f;
    return w; // [-1, 1]
}

Vec3 windDisplace(float worldX, float worldZ, float height, float bladeTilt) {
    float ws = windStrength(worldX, worldZ);
    // Wind bends blade tip more than base
    float t2 = height * height;
    Vec3 windDir = {0.8f, 0, 0.6f}; // normalized wind direction (x,z plane)
    return windDir * (ws * bladeTilt * t2 * 0.8f);
}

// ─────────────────────── Grass Blade ───────────────────────
// Each blade: quadratic bezier with 3 control points
// We tessellate into N quads (2N triangles)
struct GrassBlade {
    float x, z;      // world position
    float height;    // blade height
    float width;     // blade width at base
    float tilt;      // natural tilt angle
    float facing;    // blade facing angle (rotation around Y)
    Vec3  baseColor;
    Vec3  tipColor;
};

void renderBlade(const GrassBlade& blade, int segments = 5) {
    Vec3 sunDir = Vec3(0.6f, 0.8f, 0.4f).normalized();
    Vec3 sunColor = {1.0f, 0.95f, 0.8f};
    Vec3 ambColor = {0.15f, 0.25f, 0.15f};

    float cosF = std::cos(blade.facing);
    float sinF = std::sin(blade.facing);

    // Blade-local right direction (perpendicular to facing, in XZ plane)
    Vec3 bladeRight = {cosF, 0, sinF};
    Vec3 bladeFwd = {-sinF, 0, cosF};

    // Wind displacement at this blade position
    float ws = windStrength(blade.x, blade.z);

    // Control points in world space (bezier: base P0, mid P1, tip P2)
    Vec3 P0 = {blade.x, 0.0f, blade.z};
    // Natural tilt direction + wind
    float tiltX = std::cos(blade.facing) * blade.tilt;
    float tiltZ = std::sin(blade.facing) * blade.tilt;
    Vec3 P2 = {
        blade.x + tiltX * blade.height + ws * 0.8f * blade.height * bladeFwd.x,
        blade.height,
        blade.z + tiltZ * blade.height + ws * 0.8f * blade.height * bladeFwd.z
    };
    Vec3 P1 = {
        (P0.x + P2.x) * 0.5f,
        blade.height * 0.6f,
        (P0.z + P2.z) * 0.5f
    };

    // Tessellate bezier into vertices
    std::vector<Vec3> leftVerts(segments + 1), rightVerts(segments + 1);
    std::vector<Vec3> colors(segments + 1);

    for (int i = 0; i <= segments; i++) {
        float t = (float)i / segments;
        float tm = 1.f - t;
        // Quadratic bezier
        Vec3 center = P0 * (tm * tm) + P1 * (2 * tm * t) + P2 * (t * t);
        // Width tapers toward tip
        float w = blade.width * (1.f - t * 0.85f);
        Vec3 left = center - bladeRight * (w * 0.5f);
        Vec3 right2 = center + bladeRight * (w * 0.5f);
        leftVerts[i] = left;
        rightVerts[i] = right2;
        // Color gradient base→tip
        float g = t;
        colors[i] = mix(blade.baseColor, blade.tipColor, g);
    }

    // Compute blade tangent for shading
    // Use forward direction of bezier at midpoint for normal approximation
    float t_mid = 0.5f;
    float tm_mid = 1.f - t_mid;
    Vec3 tangent = (P1 - P0) * (2 * tm_mid) + (P2 - P1) * (2 * t_mid);
    tangent = tangent.normalized();
    // Face normal computed per-segment below
    (void)tangent; // suppress unused warning

    // Shade each segment pair
    for (int i = 0; i < segments; i++) {
        float t0 = (float)i / segments;
        float t1 = (float)(i + 1) / segments;

        // Bezier tangents for local normals
        float tm0 = 1.f - t0, tm1 = 1.f - t1;
        Vec3 tang0 = ((P1 - P0) * (2 * tm0) + (P2 - P1) * (2 * t0)).normalized();
        Vec3 tang1 = ((P1 - P0) * (2 * tm1) + (P2 - P1) * (2 * t1)).normalized();
        Vec3 n0 = bladeRight.cross(tang0).normalized();
        Vec3 n1 = bladeRight.cross(tang1).normalized();
        if (n0.y < 0) n0 = -n0;
        if (n1.y < 0) n1 = -n1;

        // Shading
        auto shade = [&](const Vec3& n, const Vec3& col) -> Vec3 {
            float diff = std::max(0.f, n.dot(sunDir));
            // Translucency: some light from behind
            float back = std::max(0.f, (-n).dot(sunDir)) * 0.3f;
            Vec3 lit = col * (ambColor + sunColor * (diff + back));
            return lit;
        };

        Vec3 c0 = shade(n0, colors[i]);
        Vec3 c1 = shade(n0, colors[i]);
        Vec3 c2 = shade(n1, colors[i + 1]);
        Vec3 c3 = shade(n1, colors[i + 1]);

        // Project vertices
        float sx[4], sy[4], sz[4];
        Vec3 wv[4] = {leftVerts[i], rightVerts[i], leftVerts[i+1], rightVerts[i+1]};
        bool ok[4];
        for (int k = 0; k < 4; k++) {
            ok[k] = cam.project(wv[k], sx[k], sy[k], sz[k]);
        }
        if (!ok[0] || !ok[1] || !ok[2] || !ok[3]) continue;

        Vec3 s0 = {sx[0], sy[0], 0};
        Vec3 s1 = {sx[1], sy[1], 0};
        Vec3 s2 = {sx[2], sy[2], 0};
        Vec3 s3 = {sx[3], sy[3], 0};

        drawTriangle(s0, s1, s2, c0, c1, c2, sz[0], sz[1], sz[2]);
        drawTriangle(s1, s3, s2, c1, c3, c2, sz[1], sz[3], sz[2]);
    }
}

// ─────────────────────── Ground Plane ───────────────────────
void renderGround() {
    Vec3 sunDir = Vec3(0.6f, 0.8f, 0.4f).normalized();
    Vec3 sunColor = {1.0f, 0.95f, 0.8f};
    Vec3 ambGround = {0.1f, 0.2f, 0.08f};

    // Tile the ground with colored quads
    float xMin = -20.f, xMax = 20.f;
    float zMin = -5.f, zMax = 18.f;
    int nx = 40, nz = 40;
    float dx = (xMax - xMin) / nx;
    float dz = (zMax - zMin) / nz;

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.f, 1.f);

    for (int iz = 0; iz < nz; iz++) {
        for (int ix = 0; ix < nx; ix++) {
            float wx0 = xMin + ix * dx;
            float wz0 = zMin + iz * dz;
            float wx1 = wx0 + dx;
            float wz1 = wz0 + dz;

            // Ground color variation
            float var = dist(rng);
            Vec3 groundCol = {0.12f + var * 0.05f, 0.22f + var * 0.08f, 0.08f + var * 0.03f};

            Vec3 groundNorm = {0, 1, 0};
            float diff = std::max(0.f, groundNorm.dot(sunDir));
            Vec3 litGround = groundCol * (ambGround + sunColor * diff * 0.5f);

            Vec3 corners[4] = {
                {wx0, 0, wz0}, {wx1, 0, wz0},
                {wx0, 0, wz1}, {wx1, 0, wz1}
            };
            float sx[4], sy[4], sz[4];
            bool ok[4];
            for (int k = 0; k < 4; k++)
                ok[k] = cam.project(corners[k], sx[k], sy[k], sz[k]);
            if (!ok[0] || !ok[1] || !ok[2] || !ok[3]) continue;

            Vec3 c = litGround;
            Vec3 s0 = {sx[0], sy[0], 0};
            Vec3 s1 = {sx[1], sy[1], 0};
            Vec3 s2 = {sx[2], sy[2], 0};
            Vec3 s3 = {sx[3], sy[3], 0};

            drawTriangle(s0, s1, s2, c, c, c, sz[0], sz[1], sz[2]);
            drawTriangle(s1, s3, s2, c, c, c, sz[1], sz[3], sz[2]);
        }
    }
}

// ─────────────────────── Grass Field ───────────────────────
std::vector<GrassBlade> generateGrassField(int count, float xMin, float xMax,
                                            float zMin, float zMax,
                                            unsigned seed = 12345) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> dx(xMin, xMax);
    std::uniform_real_distribution<float> dz(zMin, zMax);
    std::uniform_real_distribution<float> dh(0.4f, 0.9f);
    std::uniform_real_distribution<float> dw(0.04f, 0.09f);
    std::uniform_real_distribution<float> dtilt(-0.15f, 0.15f);
    std::uniform_real_distribution<float> dfacing(0.f, 6.2832f);

    std::vector<GrassBlade> blades;
    blades.reserve(count);

    for (int i = 0; i < count; i++) {
        GrassBlade b;
        b.x = dx(rng);
        b.z = dz(rng);
        b.height = dh(rng);
        b.width = dw(rng);
        b.tilt = dtilt(rng) + 0.1f;
        b.facing = dfacing(rng);

        // Color variation: yellowish-green to dark green
        std::uniform_real_distribution<float> dc(0.f, 1.f);
        float cv = dc(rng);
        b.baseColor = mix({0.1f, 0.28f, 0.05f}, {0.15f, 0.35f, 0.07f}, cv);
        b.tipColor  = mix({0.35f, 0.5f, 0.1f},  {0.5f, 0.65f, 0.15f}, cv);

        blades.push_back(b);
    }
    return blades;
}

// Sort blades back-to-front for correct overdraw
void sortBlades(std::vector<GrassBlade>& blades) {
    std::sort(blades.begin(), blades.end(), [](const GrassBlade& a, const GrassBlade& b) {
        // Distance from camera z (larger = further = render first)
        return a.z < b.z; // simple z-sort (cam looks toward -z)
    });
}

// ─────────────────────── Tone Mapping ───────────────────────
Vec3 reinhardTonemap(Vec3 c) {
    return {c.x / (1 + c.x), c.y / (1 + c.y), c.z / (1 + c.z)};
}

Vec3 gammaCorrect(Vec3 c, float gamma = 2.2f) {
    float inv = 1.f / gamma;
    return {std::pow(clamp01(c.x), inv), std::pow(clamp01(c.y), inv), std::pow(clamp01(c.z), inv)};
}

// ─────────────────────── Main ───────────────────────
int main() {
    clearBuffers();

    // Step 1: Sky
    renderSky();

    // Step 2: Ground
    renderGround();

    // Step 3: Generate grass field
    // Far patch (z=2..12): fewer blades, LOD
    // Mid patch (z=0..5): medium density
    // Near patch (z=-2..2): full density

    std::vector<GrassBlade> allBlades;

    // Far zone (z=6..14): low density
    {
        auto b = generateGrassField(2500, -15.f, 15.f, 6.f, 14.f, 111);
        for (auto& blade : b) blade.height *= 0.9f;
        allBlades.insert(allBlades.end(), b.begin(), b.end());
    }
    // Mid zone (z=1..8): medium density
    {
        auto b = generateGrassField(3500, -12.f, 12.f, 1.f, 8.f, 222);
        allBlades.insert(allBlades.end(), b.begin(), b.end());
    }
    // Near zone (z=-3..3): full density
    {
        auto b = generateGrassField(2000, -8.f, 8.f, -3.f, 3.f, 333);
        for (auto& blade : b) blade.height *= 1.1f;
        allBlades.insert(allBlades.end(), b.begin(), b.end());
    }

    // Sort back to front
    std::sort(allBlades.begin(), allBlades.end(), [](const GrassBlade& a, const GrassBlade& b) {
        float da = (a.x - cam.pos.x) * (a.x - cam.pos.x) + (a.z - cam.pos.z) * (a.z - cam.pos.z);
        float db = (b.x - cam.pos.x) * (b.x - cam.pos.x) + (b.z - cam.pos.z) * (b.z - cam.pos.z);
        return da > db; // furthest first
    });

    // Step 4: Render grass (use fewer segments for far blades for speed)
    for (const auto& blade : allBlades) {
        float dist2 = (blade.x - cam.pos.x) * (blade.x - cam.pos.x)
                    + (blade.z - cam.pos.z) * (blade.z - cam.pos.z);
        int segs = (dist2 > 100.f) ? 3 : (dist2 > 36.f) ? 4 : 5;
        renderBlade(blade, segs);
    }

    // Step 5: Tone map and write output
    std::vector<uint8_t> pixels(W * H * 3);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 c = framebuf[y][x];
            c = reinhardTonemap(c);
            c = gammaCorrect(c);
            pixels[(y * W + x) * 3 + 0] = (uint8_t)(c.x * 255.f);
            pixels[(y * W + x) * 3 + 1] = (uint8_t)(c.y * 255.f);
            pixels[(y * W + x) * 3 + 2] = (uint8_t)(c.z * 255.f);
        }
    }

    stbi_write_png("grass_output.png", W, H, 3, pixels.data(), W * 3);
    printf("Rendered grass_output.png (%dx%d)\n", W, H);
    return 0;
}
