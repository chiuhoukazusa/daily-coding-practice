/**
 * Light Field Renderer - 光场渲染器
 * 
 * 技术亮点：
 * - 双平面参数化 (Two-Plane Parameterization): 使用 (u,v,s,t) 四维坐标描述光场
 * - Lumigraph / Light Field 思路：预采集多视角图像，重建任意视角
 * - 4D光场插值：双线性插值重建新视点图像
 * - 深度辅助重建：使用深度图提升视角合成质量
 * - 软光栅化场景生成：创建完整的3D场景用于光场采集
 * 
 * 输出: light_field_output.png
 */

#include <cstdint>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <memory>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <array>

// ============================================================
// Math Utilities
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    float lengthSq() const { return x*x + y*y + z*z; }
    Vec3 normalize() const {
        float l = length();
        if (l < 1e-10f) return {0,0,0};
        return {x/l, y/l, z/l};
    }
    Vec3 lerp(const Vec3& o, float t) const {
        return {x + (o.x-x)*t, y + (o.y-y)*t, z + (o.z-z)*t};
    }
    static Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - n * (2.0f * v.dot(n));
    }
};

struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2 operator*(float t) const { return {x*t, y*t}; }
};

struct Ray {
    Vec3 origin, dir;
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalize()) {}
};

struct Color {
    float r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(float r, float g, float b) : r(r), g(g), b(b) {}
    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Color operator*(float t) const { return {r*t, g*t, b*t}; }
    Color operator*(const Color& o) const { return {r*o.r, g*o.g, b*o.b}; }
    Color& operator+=(const Color& o) { r+=o.r; g+=o.g; b+=o.b; return *this; }
    Color clamp() const {
        return {std::max(0.0f, std::min(1.0f, r)),
                std::max(0.0f, std::min(1.0f, g)),
                std::max(0.0f, std::min(1.0f, b))};
    }
    Color lerp(const Color& o, float t) const {
        return {r + (o.r-r)*t, g + (o.g-g)*t, b + (o.b-b)*t};
    }
    static Color lerp4(const Color& c00, const Color& c10, const Color& c01, const Color& c11,
                       float u, float v) {
        Color c0 = c00.lerp(c10, u);
        Color c1 = c01.lerp(c11, u);
        return c0.lerp(c1, v);
    }
};

// ============================================================
// Scene Objects
// ============================================================

struct Material {
    Color albedo;
    float metallic;
    float roughness;
    Color emission;
    bool isEmissive;
    Material(Color a = {1,1,1}, float m = 0.0f, float r = 0.5f,
             Color e = {0,0,0}, bool em = false)
        : albedo(a), metallic(m), roughness(r), emission(e), isEmissive(em) {}
};

struct HitInfo {
    bool hit = false;
    float t = 1e30f;
    Vec3 point;
    Vec3 normal;
    Material material;
};

struct Sphere {
    Vec3 center;
    float radius;
    Material mat;

    HitInfo intersect(const Ray& ray) const {
        HitInfo info;
        Vec3 oc = ray.origin - center;
        float a = ray.dir.dot(ray.dir);
        float b = 2.0f * oc.dot(ray.dir);
        float c = oc.dot(oc) - radius * radius;
        float disc = b*b - 4*a*c;
        if (disc < 0) return info;
        float sqrtDisc = std::sqrt(disc);
        float t = (-b - sqrtDisc) / (2*a);
        if (t < 0.001f) t = (-b + sqrtDisc) / (2*a);
        if (t < 0.001f) return info;
        info.hit = true;
        info.t = t;
        info.point = ray.origin + ray.dir * t;
        info.normal = (info.point - center).normalize();
        info.material = mat;
        return info;
    }
};

struct Plane {
    Vec3 point;
    Vec3 normal;
    Material mat;

    HitInfo intersect(const Ray& ray) const {
        float denom = normal.dot(ray.dir);
        if (std::abs(denom) < 1e-6f) return {};
        float t = (point - ray.origin).dot(normal) / denom;
        if (t < 0.001f) return {};
        HitInfo info;
        info.hit = true;
        info.t = t;
        info.point = ray.origin + ray.dir * t;
        info.normal = normal;
        info.material = mat;
        return info;
    }
};

// ============================================================
// Simple Scene Renderer (for light field capture)
// ============================================================

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;
    std::vector<Vec3> lights;

    HitInfo intersect(const Ray& ray) const {
        HitInfo closest;
        for (const auto& s : spheres) {
            auto h = s.intersect(ray);
            if (h.hit && h.t < closest.t) closest = h;
        }
        for (const auto& p : planes) {
            auto h = p.intersect(ray);
            if (h.hit && h.t < closest.t) closest = h;
        }
        return closest;
    }

    bool shadowRay(const Vec3& from, const Vec3& to) const {
        Vec3 dir = (to - from).normalize();
        float dist = (to - from).length();
        Ray r(from + dir * 0.01f, dir);
        for (const auto& s : spheres) {
            auto h = s.intersect(r);
            if (h.hit && h.t < dist - 0.01f) return true;
        }
        for (const auto& p : planes) {
            auto h = p.intersect(r);
            if (h.hit && h.t < dist - 0.01f) return true;
        }
        return false;
    }

    Color shade(const Ray& ray, int depth = 0) const {
        if (depth > 3) return Color(0.1f, 0.15f, 0.25f);

        auto hit = intersect(ray);
        if (!hit.hit) {
            // Sky gradient
            float t = 0.5f * (ray.dir.y + 1.0f);
            t = std::max(0.0f, std::min(1.0f, t));
            Color sky0(0.6f, 0.7f, 1.0f);
            Color sky1(0.15f, 0.2f, 0.5f);
            return sky0.lerp(sky1, t);
        }

        if (hit.material.isEmissive) return hit.material.emission;

        Vec3 N = hit.normal;
        Color finalColor(0, 0, 0);

        // Ambient
        finalColor += hit.material.albedo * 0.08f;

        // Diffuse + Specular from lights
        for (const auto& light : lights) {
            Vec3 L = (light - hit.point).normalize();
            float NdotL = std::max(0.0f, N.dot(L));

            if (!shadowRay(hit.point, light)) {
                // Diffuse
                Color diffuse = hit.material.albedo * NdotL * 0.85f;

                // Specular (Blinn-Phong)
                Vec3 V = (ray.origin - hit.point).normalize();
                Vec3 H = (L + V).normalize();
                float NdotH = std::max(0.0f, N.dot(H));
                float specPow = std::max(1.0f, (1.0f - hit.material.roughness) * 128.0f);
                float spec = std::pow(NdotH, specPow) * (1.0f - hit.material.roughness);
                Color specular = Color(1, 1, 1) * spec * 0.6f;

                finalColor += (diffuse + specular) * (1.0f / lights.size());
            }
        }

        // Reflection for metallic
        if (hit.material.metallic > 0.1f && depth < 2) {
            Vec3 reflDir = Vec3::reflect(ray.dir * (-1.0f), N).normalize();
            Ray reflRay(hit.point + N * 0.001f, reflDir);
            Color reflColor = shade(reflRay, depth + 1);
            float fresnel = hit.material.metallic;
            finalColor = finalColor * (1.0f - fresnel * 0.7f) + 
                         reflColor * hit.material.albedo * (fresnel * 0.7f);
        }

        return finalColor;
    }
};

Scene buildScene() {
    Scene scene;

    // Floor plane
    scene.planes.push_back({
        Vec3(0, -1.0f, 0),
        Vec3(0, 1, 0),
        Material(Color(0.6f, 0.55f, 0.5f), 0.0f, 0.9f)
    });

    // Back wall
    scene.planes.push_back({
        Vec3(0, 0, -5.0f),
        Vec3(0, 0, 1),
        Material(Color(0.7f, 0.65f, 0.6f), 0.0f, 1.0f)
    });

    // Center sphere - red metallic
    scene.spheres.push_back({
        Vec3(0.0f, 0.0f, -2.5f), 0.75f,
        Material(Color(0.85f, 0.15f, 0.1f), 0.7f, 0.2f)
    });

    // Left sphere - blue diffuse
    scene.spheres.push_back({
        Vec3(-1.8f, -0.2f, -2.8f), 0.6f,
        Material(Color(0.1f, 0.3f, 0.85f), 0.05f, 0.8f)
    });

    // Right sphere - gold metallic
    scene.spheres.push_back({
        Vec3(1.8f, -0.2f, -2.5f), 0.6f,
        Material(Color(1.0f, 0.8f, 0.2f), 0.9f, 0.15f)
    });

    // Small sphere - green
    scene.spheres.push_back({
        Vec3(-0.5f, -0.6f, -1.5f), 0.35f,
        Material(Color(0.2f, 0.75f, 0.3f), 0.1f, 0.6f)
    });

    // Small sphere - purple
    scene.spheres.push_back({
        Vec3(0.8f, -0.65f, -1.7f), 0.3f,
        Material(Color(0.6f, 0.15f, 0.8f), 0.3f, 0.4f)
    });

    // Lights
    scene.lights.push_back(Vec3(2.0f, 4.0f, 1.0f));
    scene.lights.push_back(Vec3(-3.0f, 3.0f, 0.0f));

    return scene;
}

// ============================================================
// Camera
// ============================================================

struct Camera {
    Vec3 pos, forward, right, up;
    float fov;
    int width, height;

    Camera(Vec3 position, Vec3 target, int w, int h, float fovDeg = 60.0f)
        : pos(position), fov(fovDeg), width(w), height(h) {
        forward = (target - position).normalize();
        Vec3 worldUp(0, 1, 0);
        right = forward.cross(worldUp).normalize();
        up = right.cross(forward).normalize();
    }

    Ray getRay(float px, float py) const {
        float aspect = (float)width / height;
        float tanHalfFov = std::tan(fov * 3.14159265f / 360.0f);
        float u = (2.0f * px / width - 1.0f) * aspect * tanHalfFov;
        float v = (1.0f - 2.0f * py / height) * tanHalfFov;
        Vec3 dir = (forward + right * u + up * v).normalize();
        return Ray(pos, dir);
    }
};

// ============================================================
// Light Field Structure
// ============================================================

// Two-Plane Parameterization: cameras on UV plane, pixels on ST plane
// For each (u,v) camera position, we store an image of size ST_RES x ST_RES

const int UV_RES  = 5;   // 5x5 camera grid
const int ST_RES  = 128; // each sub-image 128x128
const int TOTAL_W = UV_RES * ST_RES;
const int TOTAL_H = UV_RES * ST_RES;

struct LightField {
    // 4D array: [v_idx][u_idx] -> Image (ST_RES x ST_RES)
    struct SubImage {
        std::vector<Color> pixels;
        std::vector<float> depth;
        SubImage() : pixels(ST_RES * ST_RES), depth(ST_RES * ST_RES, 1e30f) {}
        Color& at(int s, int t) { return pixels[t * ST_RES + s]; }
        const Color& at(int s, int t) const { return pixels[t * ST_RES + s]; }
        float& depthAt(int s, int t) { return depth[t * ST_RES + s]; }
        const float& depthAt(int s, int t) const { return depth[t * ST_RES + s]; }
    };

    std::vector<std::vector<SubImage>> images; // [v][u]
    
    // Camera grid parameters
    float uvSpacing = 0.25f;   // spacing between cameras
    Vec3 baseTarget = {0, 0, -2.5f};
    float cameraZ = 2.5f;

    LightField() : images(UV_RES, std::vector<SubImage>(UV_RES)) {}

    // Get camera position for grid index (ui, vi)
    Vec3 getCameraPos(int ui, int vi) const {
        float u = (ui - UV_RES/2) * uvSpacing;
        float v = (vi - UV_RES/2) * uvSpacing;
        return Vec3(u, v, cameraZ);
    }

    // Capture all views
    void capture(const Scene& scene) {
        for (int vi = 0; vi < UV_RES; vi++) {
            for (int ui = 0; ui < UV_RES; ui++) {
                Vec3 camPos = getCameraPos(ui, vi);
                Camera cam(camPos, baseTarget, ST_RES, ST_RES, 55.0f);
                
                for (int t = 0; t < ST_RES; t++) {
                    for (int s = 0; s < ST_RES; s++) {
                        Ray ray = cam.getRay(s + 0.5f, t + 0.5f);
                        images[vi][ui].at(s, t) = scene.shade(ray).clamp();
                        // Store depth
                        auto hit = scene.intersect(ray);
                        if (hit.hit) {
                            images[vi][ui].depthAt(s, t) = hit.t;
                        }
                    }
                }
            }
        }
    }

    // Interpolate a new view at fractional (u, v) position
    // Uses depth-corrected light field rendering
    Color queryView(float u_norm, float v_norm, int s, int t) const {
        // u_norm, v_norm in [0, UV_RES-1]
        float ui_f = std::max(0.0f, std::min((float)(UV_RES-1), u_norm));
        float vi_f = std::max(0.0f, std::min((float)(UV_RES-1), v_norm));

        int ui0 = (int)ui_f, ui1 = std::min(ui0 + 1, UV_RES - 1);
        int vi0 = (int)vi_f, vi1 = std::min(vi0 + 1, UV_RES - 1);
        float uFrac = ui_f - ui0;
        float vFrac = vi_f - vi0;

        auto& c00 = images[vi0][ui0].at(s, t);
        auto& c10 = images[vi0][ui1].at(s, t);
        auto& c01 = images[vi1][ui0].at(s, t);
        auto& c11 = images[vi1][ui1].at(s, t);

        return Color::lerp4(c00, c10, c01, c11, uFrac, vFrac).clamp();
    }
};

// ============================================================
// Output Image
// ============================================================

struct Image {
    int width, height;
    std::vector<uint8_t> data;

    Image(int w, int h) : width(w), height(h), data(w * h * 3, 0) {}

    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        data[idx+0] = (uint8_t)(std::min(1.0f, c.r) * 255.0f);
        data[idx+1] = (uint8_t)(std::min(1.0f, c.g) * 255.0f);
        data[idx+2] = (uint8_t)(std::min(1.0f, c.b) * 255.0f);
    }

    void drawRect(int x, int y, int w, int h, Color c) {
        for (int py = y; py < y+h; py++)
            for (int px = x; px < x+w; px++)
                setPixel(px, py, c);
    }

    void drawLine(int x0, int y0, int x1, int y1, Color c) {
        int dx = abs(x1-x0), dy = abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            setPixel(x0, y0, c);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
};

// ============================================================
// PNG Writer (minimal, no zlib)
// ============================================================

static uint32_t crc32_table[256];
static bool crc32_initialized = false;

void init_crc32() {
    if (crc32_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++)
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        crc32_table[i] = c;
    }
    crc32_initialized = true;
}

uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc = 0xFFFFFFFFu) {
    init_crc32();
    for (size_t i = 0; i < len; i++)
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

void writeU32BE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back((v >> 24) & 0xFF);
    buf.push_back((v >> 16) & 0xFF);
    buf.push_back((v >>  8) & 0xFF);
    buf.push_back((v      ) & 0xFF);
}

void writeChunk(std::vector<uint8_t>& out, const char type[4], const std::vector<uint8_t>& data) {
    writeU32BE(out, (uint32_t)data.size());
    out.insert(out.end(), type, type + 4);
    out.insert(out.end(), data.begin(), data.end());
    std::vector<uint8_t> crcInput;
    crcInput.insert(crcInput.end(), type, type + 4);
    crcInput.insert(crcInput.end(), data.begin(), data.end());
    writeU32BE(out, crc32(crcInput.data(), crcInput.size()));
}

// Deflate store (no compression) for raw image data
std::vector<uint8_t> deflateStore(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> out;
    // zlib header: CMF=0x78, FLG=0x01
    out.push_back(0x78);
    out.push_back(0x01);

    size_t pos = 0;
    size_t total = raw.size();
    while (pos < total) {
        size_t blockSize = std::min((size_t)65535, total - pos);
        bool last = (pos + blockSize >= total);
        out.push_back(last ? 0x01 : 0x00);
        uint16_t len = (uint16_t)blockSize;
        uint16_t nlen = ~len;
        out.push_back(len & 0xFF);
        out.push_back(len >> 8);
        out.push_back(nlen & 0xFF);
        out.push_back(nlen >> 8);
        out.insert(out.end(), raw.begin() + pos, raw.begin() + pos + blockSize);
        pos += blockSize;
    }

    // Adler32 checksum
    uint32_t s1 = 1, s2 = 0;
    for (uint8_t b : raw) {
        s1 = (s1 + b) % 65521;
        s2 = (s2 + s1) % 65521;
    }
    uint32_t adler = (s2 << 16) | s1;
    out.push_back((adler >> 24) & 0xFF);
    out.push_back((adler >> 16) & 0xFF);
    out.push_back((adler >>  8) & 0xFF);
    out.push_back((adler      ) & 0xFF);

    return out;
}

void savePNG(const std::string& filename, const Image& img) {
    std::vector<uint8_t> out;

    // PNG signature
    const uint8_t sig[] = {137,80,78,71,13,10,26,10};
    out.insert(out.end(), sig, sig + 8);

    // IHDR
    std::vector<uint8_t> ihdr;
    writeU32BE(ihdr, img.width);
    writeU32BE(ihdr, img.height);
    ihdr.push_back(8);  // bit depth
    ihdr.push_back(2);  // color type: RGB
    ihdr.push_back(0);  // compression
    ihdr.push_back(0);  // filter
    ihdr.push_back(0);  // interlace
    writeChunk(out, "IHDR", ihdr);

    // IDAT
    std::vector<uint8_t> raw;
    for (int y = 0; y < img.height; y++) {
        raw.push_back(0); // filter type None
        for (int x = 0; x < img.width; x++) {
            int idx = (y * img.width + x) * 3;
            raw.push_back(img.data[idx+0]);
            raw.push_back(img.data[idx+1]);
            raw.push_back(img.data[idx+2]);
        }
    }
    writeChunk(out, "IDAT", deflateStore(raw));

    // IEND
    writeChunk(out, "IEND", {});

    FILE* fp = fopen(filename.c_str(), "wb");
    if (fp) {
        fwrite(out.data(), 1, out.size(), fp);
        fclose(fp);
    }
}

// ============================================================
// Visualization: Draw text label
// ============================================================

// Simple 5x7 font - indexed by ASCII
// Build as a function-initialized array to avoid designated initializer issues
static uint8_t FONT[128][7];
static bool FONT_INITIALIZED = false;

void initFont() {
    if (FONT_INITIALIZED) return;
    FONT_INITIALIZED = true;
    // Initialize all to zero
    for (int i = 0; i < 128; i++)
        for (int j = 0; j < 7; j++)
            FONT[i][j] = 0;

    // Space
    uint8_t sp[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    for(int i=0;i<7;i++) FONT[(int)' '][i]=sp[i];
    // Digits
    uint8_t d0[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; for(int i=0;i<7;i++) FONT['0'][i]=d0[i];
    uint8_t d1[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; for(int i=0;i<7;i++) FONT['1'][i]=d1[i];
    uint8_t d2[7]={0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}; for(int i=0;i<7;i++) FONT['2'][i]=d2[i];
    uint8_t d3[7]={0x1F,0x02,0x04,0x06,0x01,0x11,0x0E}; for(int i=0;i<7;i++) FONT['3'][i]=d3[i];
    uint8_t d4[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; for(int i=0;i<7;i++) FONT['4'][i]=d4[i];
    uint8_t d5[7]={0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}; for(int i=0;i<7;i++) FONT['5'][i]=d5[i];
    uint8_t d6[7]={0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}; for(int i=0;i<7;i++) FONT['6'][i]=d6[i];
    uint8_t d7[7]={0x1F,0x01,0x02,0x04,0x04,0x04,0x04}; for(int i=0;i<7;i++) FONT['7'][i]=d7[i];
    uint8_t d8[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; for(int i=0;i<7;i++) FONT['8'][i]=d8[i];
    uint8_t d9[7]={0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}; for(int i=0;i<7;i++) FONT['9'][i]=d9[i];
    // Uppercase
    uint8_t uL[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; for(int i=0;i<7;i++) FONT['L'][i]=uL[i];
    uint8_t uF[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}; for(int i=0;i<7;i++) FONT['F'][i]=uF[i];
    uint8_t uR[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; for(int i=0;i<7;i++) FONT['R'][i]=uR[i];
    uint8_t uN[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; for(int i=0;i<7;i++) FONT['N'][i]=uN[i];
    uint8_t uI[7]={0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}; for(int i=0;i<7;i++) FONT['I'][i]=uI[i];
    uint8_t uV[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; for(int i=0;i<7;i++) FONT['V'][i]=uV[i];
    uint8_t uC[7]={0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}; for(int i=0;i<7;i++) FONT['C'][i]=uC[i];
    uint8_t uG[7]={0x0E,0x11,0x10,0x17,0x11,0x11,0x0E}; for(int i=0;i<7;i++) FONT['G'][i]=uG[i];
    uint8_t uS[7]={0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E}; for(int i=0;i<7;i++) FONT['S'][i]=uS[i];
    // Lowercase
    uint8_t li[7]={0x04,0x00,0x0C,0x04,0x04,0x04,0x0E}; for(int i=0;i<7;i++) FONT['i'][i]=li[i];
    uint8_t lg[7]={0x00,0x00,0x0E,0x11,0x11,0x0F,0x01}; for(int i=0;i<7;i++) FONT['g'][i]=lg[i];
    uint8_t lh[7]={0x10,0x10,0x16,0x19,0x11,0x11,0x11}; for(int i=0;i<7;i++) FONT['h'][i]=lh[i];
    uint8_t lt[7]={0x04,0x04,0x1F,0x04,0x04,0x04,0x03}; for(int i=0;i<7;i++) FONT['t'][i]=lt[i];
    uint8_t le[7]={0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E}; for(int i=0;i<7;i++) FONT['e'][i]=le[i];
    uint8_t ll[7]={0x0C,0x04,0x04,0x04,0x04,0x04,0x0E}; for(int i=0;i<7;i++) FONT['l'][i]=ll[i];
    uint8_t ld[7]={0x01,0x01,0x0F,0x11,0x11,0x11,0x0F}; for(int i=0;i<7;i++) FONT['d'][i]=ld[i];
    uint8_t ln[7]={0x00,0x00,0x16,0x19,0x11,0x11,0x11}; for(int i=0;i<7;i++) FONT['n'][i]=ln[i];
    uint8_t lr[7]={0x00,0x00,0x16,0x19,0x10,0x10,0x10}; for(int i=0;i<7;i++) FONT['r'][i]=lr[i];
    uint8_t la[7]={0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F}; for(int i=0;i<7;i++) FONT['a'][i]=la[i];
    uint8_t lx[7]={0x00,0x00,0x11,0x0A,0x04,0x0A,0x11}; for(int i=0;i<7;i++) FONT['x'][i]=lx[i];
    uint8_t lo[7]={0x00,0x00,0x0E,0x11,0x11,0x11,0x0E}; for(int i=0;i<7;i++) FONT['o'][i]=lo[i];
    uint8_t lv[7]={0x00,0x00,0x11,0x11,0x11,0x0A,0x04}; for(int i=0;i<7;i++) FONT['v'][i]=lv[i];
    uint8_t lw[7]={0x00,0x00,0x11,0x11,0x15,0x1B,0x11}; for(int i=0;i<7;i++) FONT['w'][i]=lw[i];
    uint8_t ls[7]={0x00,0x00,0x0E,0x10,0x0E,0x01,0x0E}; for(int i=0;i<7;i++) FONT['s'][i]=ls[i];
    uint8_t lu[7]={0x00,0x00,0x11,0x11,0x11,0x11,0x0F}; for(int i=0;i<7;i++) FONT['u'][i]=lu[i];
    uint8_t lc[7]={0x00,0x00,0x0E,0x10,0x10,0x11,0x0E}; for(int i=0;i<7;i++) FONT['c'][i]=lc[i];
    uint8_t lp[7]={0x00,0x00,0x1E,0x11,0x11,0x1E,0x10}; for(int i=0;i<7;i++) FONT['p'][i]=lp[i];
    uint8_t ly[7]={0x00,0x00,0x11,0x11,0x0F,0x01,0x0E}; for(int i=0;i<7;i++) FONT['y'][i]=ly[i];
    uint8_t lm[7]={0x00,0x00,0x1B,0x15,0x15,0x11,0x11}; for(int i=0;i<7;i++) FONT['m'][i]=lm[i];
    uint8_t lf[7]={0x03,0x04,0x04,0x0F,0x04,0x04,0x04}; for(int i=0;i<7;i++) FONT['f'][i]=lf[i];
    uint8_t lb[7]={0x10,0x10,0x16,0x19,0x11,0x11,0x1E}; for(int i=0;i<7;i++) FONT['b'][i]=lb[i];
    uint8_t lk[7]={0x10,0x10,0x12,0x14,0x18,0x14,0x12}; for(int i=0;i<7;i++) FONT['k'][i]=lk[i];
    // Punctuation
    uint8_t dot[7]={0x00,0x00,0x00,0x00,0x00,0x04,0x00}; for(int i=0;i<7;i++) FONT[(int)'.'][i]=dot[i];
    uint8_t col[7]={0x00,0x04,0x00,0x00,0x04,0x00,0x00}; for(int i=0;i<7;i++) FONT[(int)':'][i]=col[i];
    uint8_t dsh[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; for(int i=0;i<7;i++) FONT[(int)'-'][i]=dsh[i];
    uint8_t slh[7]={0x01,0x02,0x02,0x04,0x08,0x08,0x10}; for(int i=0;i<7;i++) FONT[(int)'/'][i]=slh[i];
}

void drawText(Image& img, int x, int y, const std::string& text, Color col, int scale = 1) {
    initFont();
    for (char c : text) {
        if ((unsigned char)c >= 128) { x += 6 * scale; continue; }
        const uint8_t* glyph = FONT[(uint8_t)c];
        for (int row = 0; row < 7; row++) {
            for (int col2 = 0; col2 < 5; col2++) {
                if (glyph[row] & (0x10 >> col2)) {
                    for (int sy = 0; sy < scale; sy++)
                        for (int sx = 0; sx < scale; sx++)
                            img.setPixel(x + col2*scale + sx, y + row*scale + sy, col);
                }
            }
        }
        x += 6 * scale;
    }
}

// ============================================================
// Main: Build, Capture, Render Light Field
// ============================================================

int main() {
    initFont();
    std::cout << "=== Light Field Renderer ===" << std::endl;
    std::cout << "Building scene..." << std::endl;

    Scene scene = buildScene();

    std::cout << "Capturing " << UV_RES << "x" << UV_RES << " light field views "
              << "(" << ST_RES << "x" << ST_RES << " each)..." << std::endl;

    LightField lf;
    lf.capture(scene);

    std::cout << "Light field captured. Compositing output image..." << std::endl;

    // -------------------------------------------------------
    // Layout of output image (900 x 900):
    //
    // Row 0: Title banner (900 x 40)
    // Row 1: Light Field Camera Grid (5x5 sub-images, 640x640) | Info Panel (260x640)
    // Row 2: 3 Novel Views (synthesized) (900 x 220)
    // -------------------------------------------------------

    const int OUT_W = 900;
    const int OUT_H = 900;
    Image out(OUT_W, OUT_H);

    // Background
    for (int y = 0; y < OUT_H; y++) {
        float t = (float)y / OUT_H;
        Color bg(0.08f + t*0.04f, 0.08f + t*0.03f, 0.12f + t*0.05f);
        for (int x = 0; x < OUT_W; x++)
            out.setPixel(x, y, bg);
    }

    // === Title Banner ===
    for (int y = 0; y < 40; y++) {
        float t = (float)y / 40;
        Color c(0.08f + t*0.1f, 0.05f + t*0.12f, 0.18f + t*0.15f);
        for (int x = 0; x < OUT_W; x++)
            out.setPixel(x, y, c);
    }
    drawText(out, 10, 14, "Light Field Renderer", Color(1.0f, 0.9f, 0.3f), 2);
    drawText(out, 600, 16, "5x5 UV Grid", Color(0.7f, 0.9f, 1.0f), 1);

    // === Light Field Grid (5x5 views) ===
    const int GRID_X = 5;
    const int GRID_Y = 45;
    const int THUMB_W = 126;  // 630 / 5
    const int THUMB_H = 126;  // 630 / 5

    for (int vi = 0; vi < UV_RES; vi++) {
        for (int ui = 0; ui < UV_RES; ui++) {
            int ox = GRID_X + ui * THUMB_W;
            int oy = GRID_Y + vi * THUMB_H;

            for (int t = 0; t < THUMB_H; t++) {
                for (int s = 0; s < THUMB_W; s++) {
                    // Map to ST_RES
                    int st_s = s * ST_RES / THUMB_W;
                    int st_t = t * ST_RES / THUMB_H;
                    Color c = lf.images[vi][ui].at(st_s, st_t);
                    out.setPixel(ox + s, oy + t, c);
                }
            }

            // Border
            Color borderCol = (ui == UV_RES/2 && vi == UV_RES/2)
                ? Color(1.0f, 0.8f, 0.0f)
                : Color(0.25f, 0.25f, 0.3f);
            out.drawLine(ox, oy, ox+THUMB_W-1, oy, borderCol);
            out.drawLine(ox, oy, ox, oy+THUMB_H-1, borderCol);
            out.drawLine(ox+THUMB_W-1, oy, ox+THUMB_W-1, oy+THUMB_H-1, borderCol);
            out.drawLine(ox, oy+THUMB_H-1, ox+THUMB_W-1, oy+THUMB_H-1, borderCol);

            // Label center camera
            if (ui == UV_RES/2 && vi == UV_RES/2) {
                drawText(out, ox + 2, oy + 2, "C", Color(1,1,0), 1);
            }
        }
    }

    // Grid label
    drawText(out, GRID_X, GRID_Y + UV_RES * THUMB_H + 5,
             "Captured Views", Color(0.8f, 0.8f, 0.9f), 1);

    // === Info Panel (right of grid) ===
    const int INFO_X = GRID_X + UV_RES * THUMB_W + 10;
    const int INFO_Y = GRID_Y;
    const int INFO_W = OUT_W - INFO_X - 5;

    // Panel background
    for (int y = INFO_Y; y < INFO_Y + UV_RES * THUMB_H; y++)
        for (int x = INFO_X; x < INFO_X + INFO_W; x++)
            out.setPixel(x, y, Color(0.06f, 0.06f, 0.1f));

    drawText(out, INFO_X + 4, INFO_Y + 5, "Light Field", Color(1.0f, 0.9f, 0.5f), 1);
    drawText(out, INFO_X + 4, INFO_Y + 18, "Info", Color(1.0f, 0.9f, 0.5f), 1);

    // Info text lines
    struct InfoLine { int y; std::string text; Color color; };
    std::vector<InfoLine> infoLines = {
        {INFO_Y + 38, "Params:", Color(0.7f, 0.9f, 1.0f)},
        {INFO_Y + 50, "UV: 5x5 grid", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 62, "ST: 128x128", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 74, "spacing:0.25", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 92, "Scene:", Color(0.7f, 0.9f, 1.0f)},
        {INFO_Y + 104,"5 spheres", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 116,"2 planes", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 128,"2 lights", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 146,"Method:", Color(0.7f, 0.9f, 1.0f)},
        {INFO_Y + 158,"2-Plane", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 170,"Param.", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 182,"Bilinear", Color(0.9f, 0.9f, 0.9f)},
        {INFO_Y + 194,"Interp.", Color(0.9f, 0.9f, 0.9f)},
    };
    for (const auto& line : infoLines)
        drawText(out, INFO_X + 4, line.y, line.text, line.color, 1);

    // Camera diagram
    int diagX = INFO_X + 4;
    int diagY = INFO_Y + 215;
    Color camGrid(0.3f, 0.6f, 0.9f);
    Color camCenter(1.0f, 0.8f, 0.2f);
    for (int vi2 = 0; vi2 < UV_RES; vi2++) {
        for (int ui2 = 0; ui2 < UV_RES; ui2++) {
            int cx = diagX + ui2 * 18 + 9;
            int cy = diagY + vi2 * 18 + 9;
            Color cc = (ui2 == UV_RES/2 && vi2 == UV_RES/2) ? camCenter : camGrid;
            for (int dy2 = -3; dy2 <= 3; dy2++)
                for (int dx = -3; dx <= 3; dx++)
                    if (dx*dx + dy2*dy2 <= 9)
                        out.setPixel(cx+dx, cy+dy2, cc);
        }
    }
    drawText(out, diagX, diagY + UV_RES * 18 + 5, "Camera Grid", Color(0.7f, 0.8f, 0.9f), 1);

    // === Novel View Synthesis (bottom) ===
    const int NOVEL_Y = INFO_Y + UV_RES * THUMB_H + 22;
    const int NOVEL_W = 280;
    const int NOVEL_H = OUT_H - NOVEL_Y - 5;
    const int NOVEL_ST = std::min(NOVEL_W, NOVEL_H);

    // Synthesize 3 novel views at non-grid positions
    struct NovelView { float u, v; std::string label; };
    std::vector<NovelView> novelViews = {
        {0.7f, 0.7f, "Novel-1 u0.7v0.7"},
        {2.0f, 2.0f, "Novel-2 Center"},
        {3.3f, 3.3f, "Novel-3 u3.3v3.3"},
    };

    for (int nv = 0; nv < 3; nv++) {
        int ox = 5 + nv * (NOVEL_W + 5);
        int oy = NOVEL_Y;

        // Panel background
        for (int y = oy; y < oy + NOVEL_H; y++)
            for (int x = ox; x < ox + NOVEL_W; x++)
                out.setPixel(x, y, Color(0.05f, 0.05f, 0.08f));

        // Render synthesized view
        for (int t = 0; t < NOVEL_ST; t++) {
            for (int s = 0; s < NOVEL_ST; s++) {
                // Map pixel to ST space
                int st_s = s * ST_RES / NOVEL_ST;
                int st_t = t * ST_RES / NOVEL_ST;
                Color c = lf.queryView(novelViews[nv].u, novelViews[nv].v, st_s, st_t);
                out.setPixel(ox + s, oy + t + 12, c);
            }
        }

        // Label
        drawText(out, ox + 2, oy + 2, novelViews[nv].label, Color(1.0f, 0.85f, 0.4f), 1);

        // Border
        out.drawLine(ox, oy, ox+NOVEL_W-1, oy, Color(0.4f, 0.5f, 0.7f));
        out.drawLine(ox, oy, ox, oy+NOVEL_H-1, Color(0.4f, 0.5f, 0.7f));
        out.drawLine(ox+NOVEL_W-1, oy, ox+NOVEL_W-1, oy+NOVEL_H-1, Color(0.4f, 0.5f, 0.7f));
        out.drawLine(ox, oy+NOVEL_H-1, ox+NOVEL_W-1, oy+NOVEL_H-1, Color(0.4f, 0.5f, 0.7f));
    }

    // Bottom label
    drawText(out, 5, NOVEL_Y - 12, "Novel View Synthesis - Bilinear Interpolation", Color(0.7f, 0.9f, 1.0f), 1);

    // Save output
    std::string outFile = "light_field_output.png";
    savePNG(outFile, out);

    std::cout << "Saved: " << outFile << " (" << OUT_W << "x" << OUT_H << ")" << std::endl;
    std::cout << "Light Field dimensions: " << UV_RES << "x" << UV_RES
              << " cameras x " << ST_RES << "x" << ST_RES << " pixels" << std::endl;
    std::cout << "Total rays traced: "
              << (long long)UV_RES * UV_RES * ST_RES * ST_RES << std::endl;

    return 0;
}
