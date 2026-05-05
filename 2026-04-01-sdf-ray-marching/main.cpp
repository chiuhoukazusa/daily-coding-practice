/*
 * SDF Ray Marching Renderer
 * Date: 2026-04-01
 * 
 * Techniques:
 * - Signed Distance Field (SDF) primitives: sphere, box, torus, capsule, plane
 * - Ray Marching (Sphere Tracing) algorithm
 * - SDF boolean operations: union, subtraction, intersection, smooth union
 * - Normal estimation via central differences
 * - Blinn-Phong shading with multiple lights
 * - Soft shadows via shadow ray marching
 * - Ambient Occlusion (AO) estimation
 * - Fog effect
 * - Multi-material support
 */

#include <cmath>
#include <cstdio>
#include <algorithm>
#include <cstring>

// ─────────────────────────────────────────────
// STB image writer (single header)
// ─────────────────────────────────────────────
#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wcast-qual"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

// ─────────────────────────────────────────────
// Math types
// ─────────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float v) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const {
        float l = length();
        return l > 1e-8f ? (*this) / l : Vec3(0,1,0);
    }
    float maxComp() const { return std::max({x, y, z}); }
    float minComp() const { return std::min({x, y, z}); }
};

inline Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a * (1.0f - t) + b * t;
}
inline float clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}
inline float saturate(float v) { return clamp(v, 0.0f, 1.0f); }
inline Vec3 saturate(const Vec3& v) {
    return {saturate(v.x), saturate(v.y), saturate(v.z)};
}

// ─────────────────────────────────────────────
// SDF Math helpers
// ─────────────────────────────────────────────
inline float sdSphere(const Vec3& p, float r) {
    return p.length() - r;
}

inline float sdBox(const Vec3& p, const Vec3& b) {
    Vec3 q = {std::abs(p.x) - b.x,
              std::abs(p.y) - b.y,
              std::abs(p.z) - b.z};
    Vec3 qMax = {std::max(q.x, 0.0f), std::max(q.y, 0.0f), std::max(q.z, 0.0f)};
    return qMax.length() + std::min(q.maxComp(), 0.0f);
}

inline float sdTorus(const Vec3& p, float R, float r) {
    float qx = std::sqrt(p.x*p.x + p.z*p.z) - R;
    float qy = p.y;
    return std::sqrt(qx*qx + qy*qy) - r;
}

inline float sdCapsule(const Vec3& p, const Vec3& a, const Vec3& b, float r) {
    Vec3 pa = p - a;
    Vec3 ba = b - a;
    float h = clamp(pa.dot(ba) / ba.dot(ba), 0.0f, 1.0f);
    return (pa - ba * h).length() - r;
}

inline float sdPlane(const Vec3& p, const Vec3& n, float d) {
    return p.dot(n) + d;
}

// SDF operations
inline float opUnion(float d1, float d2) { return std::min(d1, d2); }
inline float opSubtraction(float d1, float d2) { return std::max(-d1, d2); }
inline float opIntersection(float d1, float d2) { return std::max(d1, d2); }
inline float opSmoothUnion(float d1, float d2, float k) {
    float h = clamp(0.5f + 0.5f*(d2-d1)/k, 0.0f, 1.0f);
    return d1*(1.0f-h) + d2*h - k*h*(1.0f-h);
}

// Domain operations
inline Vec3 opRepeat(Vec3 p, const Vec3& c) {
    return {std::fmod(p.x + c.x*0.5f, c.x) - c.x*0.5f,
            std::fmod(p.y + c.y*0.5f, c.y) - c.y*0.5f,
            std::fmod(p.z + c.z*0.5f, c.z) - c.z*0.5f};
}

// ─────────────────────────────────────────────
// Material IDs
// ─────────────────────────────────────────────
enum MaterialID {
    MAT_NONE   = -1,
    MAT_GROUND = 0,
    MAT_SPHERE1 = 1,
    MAT_SPHERE2 = 2,
    MAT_BOX    = 3,
    MAT_TORUS  = 4,
    MAT_CAPSULE = 5,
    MAT_SMOOTH = 6,
};

struct SDFResult {
    float dist;
    int   matID;
};

// ─────────────────────────────────────────────
// Scene SDF
// ─────────────────────────────────────────────
SDFResult sceneMap(const Vec3& p) {
    // Ground plane
    float dGround = sdPlane(p, Vec3(0.0f, 1.0f, 0.0f), 1.0f);
    SDFResult res = {dGround, MAT_GROUND};

    // Large sphere (center-left)
    float dSphere1 = sdSphere(p - Vec3(-2.5f, 0.0f, 0.0f), 0.9f);
    if (dSphere1 < res.dist) res = {dSphere1, MAT_SPHERE1};

    // Small sphere (center)
    float dSphere2 = sdSphere(p - Vec3(0.0f, 0.3f, 0.0f), 0.6f);
    if (dSphere2 < res.dist) res = {dSphere2, MAT_SPHERE2};

    // Box (right)
    float dBox = sdBox(p - Vec3(2.5f, -0.3f, 0.0f), Vec3(0.6f, 0.7f, 0.6f));
    if (dBox < res.dist) res = {dBox, MAT_BOX};

    // Torus (back-center, tilted)
    {
        Vec3 tp = p - Vec3(0.0f, 0.5f, -2.5f);
        // Rotate around X by 60 degrees
        float cosA = 0.5f, sinA = 0.866f;
        Vec3 tpr = {tp.x,
                    tp.y * cosA - tp.z * sinA,
                    tp.y * sinA + tp.z * cosA};
        float dTorus = sdTorus(tpr, 0.7f, 0.25f);
        if (dTorus < res.dist) res = {dTorus, MAT_TORUS};
    }

    // Capsule (leaning, front-left)
    float dCapsule = sdCapsule(p,
        Vec3(-1.5f, -1.0f, 1.5f),
        Vec3(-0.8f,  0.4f, 1.5f),
        0.22f);
    if (dCapsule < res.dist) res = {dCapsule, MAT_CAPSULE};

    // Smooth union demo: two small spheres melting together
    {
        float dA = sdSphere(p - Vec3(1.5f, -0.5f, 2.0f), 0.45f);
        float dB = sdSphere(p - Vec3(2.2f, -0.5f, 2.0f), 0.45f);
        float dMelt = opSmoothUnion(dA, dB, 0.4f);
        if (dMelt < res.dist) res = {dMelt, MAT_SMOOTH};
    }

    return res;
}

// ─────────────────────────────────────────────
// Normal estimation (central differences)
// ─────────────────────────────────────────────
Vec3 calcNormal(const Vec3& p) {
    const float eps = 0.001f;
    float dx = sceneMap(p + Vec3(eps,0,0)).dist - sceneMap(p - Vec3(eps,0,0)).dist;
    float dy = sceneMap(p + Vec3(0,eps,0)).dist - sceneMap(p - Vec3(0,eps,0)).dist;
    float dz = sceneMap(p + Vec3(0,0,eps)).dist - sceneMap(p - Vec3(0,0,eps)).dist;
    return Vec3(dx, dy, dz).normalize();
}

// ─────────────────────────────────────────────
// Ray Marching
// ─────────────────────────────────────────────
struct MarchResult {
    float t;
    int   matID;
    bool  hit;
};

MarchResult rayMarch(const Vec3& ro, const Vec3& rd,
                     float tMin = 0.01f, float tMax = 100.0f) {
    float t = tMin;
    for (int i = 0; i < 256; i++) {
        Vec3 p = ro + rd * t;
        SDFResult r = sceneMap(p);
        if (r.dist < 0.0005f) {
            return {t, r.matID, true};
        }
        t += r.dist;
        if (t > tMax) break;
    }
    return {tMax, MAT_NONE, false};
}

// ─────────────────────────────────────────────
// Soft shadows
// ─────────────────────────────────────────────
float softShadow(const Vec3& ro, const Vec3& rd,
                 float tMin, float tMax, float k) {
    float res = 1.0f;
    float t = tMin;
    for (int i = 0; i < 64; i++) {
        float d = sceneMap(ro + rd * t).dist;
        if (d < 0.001f) return 0.0f;
        res = std::min(res, k * d / t);
        t += d;
        if (t > tMax) break;
    }
    return saturate(res);
}

// ─────────────────────────────────────────────
// Ambient Occlusion
// ─────────────────────────────────────────────
float calcAO(const Vec3& pos, const Vec3& nor) {
    float occ = 0.0f;
    float sca = 1.0f;
    for (int i = 0; i < 5; i++) {
        float h = 0.01f + 0.12f * float(i) / 4.0f;
        float d = sceneMap(pos + nor * h).dist;
        occ += (h - d) * sca;
        sca *= 0.95f;
    }
    return saturate(1.0f - 3.0f * occ);
}

// ─────────────────────────────────────────────
// Materials
// ─────────────────────────────────────────────
struct Material {
    Vec3  albedo;
    float specular;   // specular intensity
    float shininess;  // Blinn-Phong shininess
    float roughness;
};

Material getMaterial(int matID, const Vec3& pos) {
    switch (matID) {
        case MAT_GROUND: {
            // Checkerboard
            float cx = std::floor(pos.x);
            float cz = std::floor(pos.z);
            bool checker = std::fmod(std::abs(cx + cz), 2.0f) < 0.5f;
            Vec3 col = checker ? Vec3(0.85f, 0.85f, 0.85f) : Vec3(0.35f, 0.35f, 0.38f);
            return {col, 0.4f, 32.0f, 0.7f};
        }
        case MAT_SPHERE1:
            return {Vec3(0.8f, 0.3f, 0.2f), 0.8f, 64.0f, 0.3f};   // red metallic
        case MAT_SPHERE2:
            return {Vec3(0.3f, 0.6f, 0.9f), 0.6f, 128.0f, 0.2f};  // blue glossy
        case MAT_BOX:
            return {Vec3(0.85f, 0.75f, 0.3f), 0.5f, 32.0f, 0.5f}; // gold
        case MAT_TORUS:
            return {Vec3(0.3f, 0.8f, 0.4f), 0.7f, 96.0f, 0.25f};  // green metallic
        case MAT_CAPSULE:
            return {Vec3(0.7f, 0.4f, 0.7f), 0.5f, 48.0f, 0.4f};   // purple
        case MAT_SMOOTH:
            return {Vec3(1.0f, 0.6f, 0.2f), 0.6f, 80.0f, 0.3f};   // orange smooth-union
        default:
            return {Vec3(0.5f), 0.3f, 16.0f, 0.8f};
    }
}

// ─────────────────────────────────────────────
// Sky / background
// ─────────────────────────────────────────────
Vec3 skyColor(const Vec3& rd) {
    // Simple gradient sky
    float t = 0.5f * (rd.y + 1.0f);
    Vec3 skyTop    = {0.2f, 0.4f, 0.8f};
    Vec3 skyHorizon = {0.7f, 0.85f, 1.0f};
    Vec3 sky = mix(skyHorizon, skyTop, t);

    // Sun disc
    Vec3 sunDir = Vec3(0.6f, 0.5f, 0.5f).normalize();
    float sunDot = rd.dot(sunDir);
    if (sunDot > 0.998f) sky += Vec3(2.0f, 1.8f, 1.2f);
    else if (sunDot > 0.99f)
        sky += Vec3(1.0f, 0.9f, 0.6f) * std::pow(std::max(sunDot - 0.99f, 0.0f) / 0.008f, 2.0f);

    return sky;
}

// ─────────────────────────────────────────────
// Shading
// ─────────────────────────────────────────────
Vec3 shade(const Vec3& pos, const Vec3& nor, const Vec3& rd, int matID) {
    Material mat = getMaterial(matID, pos);

    // Lights
    struct Light {
        Vec3  dir;
        Vec3  color;
        float intensity;
    };
    Light lights[3] = {
        {Vec3(0.6f, 0.5f, 0.5f).normalize(), {1.2f, 1.1f, 0.9f}, 1.0f},   // key light (sun)
        {Vec3(-0.8f, 0.3f, -0.2f).normalize(), {0.3f, 0.4f, 0.6f}, 0.5f}, // fill (sky)
        {Vec3(0.0f, 0.8f, -1.0f).normalize(), {0.2f, 0.2f, 0.25f}, 0.3f}, // rim
    };

    Vec3 viewDir = (-rd).normalize();
    Vec3 color = {0,0,0};

    for (int li = 0; li < 3; li++) {
        Vec3 ldir = lights[li].dir;
        float ndl = saturate(nor.dot(ldir));
        if (ndl <= 0.0f) continue;

        // Shadow (only key light)
        float shadow = 1.0f;
        if (li == 0) {
            shadow = softShadow(pos + nor * 0.002f, ldir, 0.02f, 20.0f, 16.0f);
        }

        // Diffuse
        Vec3 diff = mat.albedo * ndl;

        // Specular (Blinn-Phong)
        Vec3 halfVec = (ldir + viewDir).normalize();
        float ndh = saturate(nor.dot(halfVec));
        float spec = mat.specular * std::pow(ndh, mat.shininess);

        color += (diff + Vec3(spec)) * lights[li].color * (lights[li].intensity * shadow);
    }

    // Ambient + AO
    float ao = calcAO(pos, nor);
    Vec3 ambient = mat.albedo * Vec3(0.15f, 0.2f, 0.25f) * ao;
    color += ambient;

    return color;
}

// ─────────────────────────────────────────────
// Camera
// ─────────────────────────────────────────────
struct Camera {
    Vec3 pos, target, up;
    float fov;

    void getRay(float u, float v, int W, int H, Vec3& ro, Vec3& rd) const {
        Vec3 fwd = (target - pos).normalize();
        Vec3 right = fwd.cross(up).normalize();
        Vec3 camUp = right.cross(fwd).normalize();

        float aspect = float(W) / float(H);
        float halfH = std::tan(fov * 0.5f * 3.14159265f / 180.0f);
        float halfW = halfH * aspect;

        float px = (u / float(W) * 2.0f - 1.0f) * halfW;
        float py = (1.0f - v / float(H) * 2.0f) * halfH;

        ro = pos;
        rd = (fwd + right * px + camUp * py).normalize();
    }
};

// ─────────────────────────────────────────────
// Tone mapping
// ─────────────────────────────────────────────
Vec3 acesFilmic(const Vec3& x) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    Vec3 num = x * (x * a + Vec3(b));
    Vec3 den = x * (x * c + Vec3(d)) + Vec3(e);
    return {saturate(num.x/den.x), saturate(num.y/den.y), saturate(num.z/den.z)};
}

Vec3 gammaCorrect(const Vec3& c, float gamma = 2.2f) {
    float ig = 1.0f / gamma;
    return {std::pow(std::max(c.x, 0.0f), ig),
            std::pow(std::max(c.y, 0.0f), ig),
            std::pow(std::max(c.z, 0.0f), ig)};
}

// ─────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────
int main() {
    const int W = 800, H = 450;
    auto* img = new unsigned char[W * H * 3];

    Camera cam;
    cam.pos    = Vec3(4.5f, 2.5f, 7.0f);
    cam.target = Vec3(0.0f, -0.2f, 0.0f);
    cam.up     = Vec3(0.0f, 1.0f, 0.0f);
    cam.fov    = 40.0f;

    // Fog parameters
    const float fogStart = 12.0f;
    const float fogEnd   = 40.0f;
    const Vec3  fogColor = {0.7f, 0.85f, 1.0f};

    printf("Rendering %dx%d SDF Ray Marching scene...\n", W, H);
    printf("Primitives: sphere x2, box, torus, capsule, smooth-union spheres x2, ground\n");

    for (int j = 0; j < H; j++) {
        if (j % 50 == 0) printf("  Row %d / %d\n", j, H);

        for (int i = 0; i < W; i++) {
            Vec3 ro, rd;
            cam.getRay(float(i) + 0.5f, float(j) + 0.5f, W, H, ro, rd);

            Vec3 color;
            MarchResult mr = rayMarch(ro, rd);

            if (mr.hit) {
                Vec3 pos = ro + rd * mr.t;
                Vec3 nor = calcNormal(pos);
                color = shade(pos, nor, rd, mr.matID);

                // Fog
                float fogFactor = saturate((mr.t - fogStart) / (fogEnd - fogStart));
                color = mix(color, fogColor, fogFactor * fogFactor);
            } else {
                color = skyColor(rd);
            }

            // Tone map + gamma
            color = acesFilmic(color);
            color = gammaCorrect(color);

            int idx = (j * W + i) * 3;
            img[idx+0] = static_cast<unsigned char>(clamp(color.x * 255.0f, 0, 255));
            img[idx+1] = static_cast<unsigned char>(clamp(color.y * 255.0f, 0, 255));
            img[idx+2] = static_cast<unsigned char>(clamp(color.z * 255.0f, 0, 255));
        }
    }

    const char* outPath = "sdf_output.png";
    stbi_write_png(outPath, W, H, 3, img, W * 3);
    printf("Saved: %s\n", outPath);
    delete[] img;
    return 0;
}
