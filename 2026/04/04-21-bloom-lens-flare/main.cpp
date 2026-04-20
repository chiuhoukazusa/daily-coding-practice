/*
 * Physically Based Bloom & Lens Flare
 * ====================================
 * Demonstrates post-processing effects used in modern PBR pipelines:
 *
 * 1. Scene rendering: HDR scene with bright light sources
 * 2. Brightness threshold extraction (bright-pass filter)
 * 3. Multi-level Gaussian blur (downscale + blur + upscale = bloom)
 * 4. Lens flare: ghosts (reflections along lens axis), halo, starburst
 * 5. Tone mapping (ACES filmic) + gamma correction → LDR output
 *
 * Output: bloom_output.png (512x512)
 *
 * Compile: g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <random>
#include <cstdint>

// ─── Math primitives ────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = len(); return l>1e-9f ? Vec3(x/l,y/l,z/l) : Vec3(0,0,0); }
    Vec3 clamp01() const { return {std::max(0.f,std::min(1.f,x)), std::max(0.f,std::min(1.f,y)), std::max(0.f,std::min(1.f,z))}; }
};

inline Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a * (1-t) + b * t;
}

inline float clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

// ─── HDR Framebuffer ─────────────────────────────────────────────────────────

struct HDRBuffer {
    int w, h;
    std::vector<Vec3> pixels;

    HDRBuffer(int w, int h) : w(w), h(h), pixels(w*h, Vec3(0,0,0)) {}

    Vec3& at(int x, int y) { return pixels[y*w + x]; }
    const Vec3& at(int x, int y) const { return pixels[y*w + x]; }

    // Bilinear sample (UV 0..1)
    Vec3 sample(float u, float v) const {
        float px = u * (w-1);
        float py = v * (h-1);
        int x0 = clamp((int)px, 0, w-2);
        int y0 = clamp((int)py, 0, h-2);
        float fx = px - x0;
        float fy = py - y0;
        Vec3 c00 = at(x0,   y0);
        Vec3 c10 = at(x0+1, y0);
        Vec3 c01 = at(x0,   y0+1);
        Vec3 c11 = at(x0+1, y0+1);
        return mix(mix(c00,c10,fx), mix(c01,c11,fx), fy);
    }

    void add(const HDRBuffer& other) {
        for (int i = 0; i < w*h; i++) pixels[i] += other.pixels[i];
    }

    void addScaled(const HDRBuffer& other, float scale) {
        for (int i = 0; i < w*h; i++) pixels[i] += other.pixels[i] * scale;
    }
};

// ─── Gaussian blur ────────────────────────────────────────────────────────────

std::vector<float> makeGaussianKernel(int radius, float sigma) {
    int size = 2*radius + 1;
    std::vector<float> k(size);
    float sum = 0;
    for (int i = 0; i < size; i++) {
        float x = (float)(i - radius);
        k[i] = std::exp(-x*x / (2*sigma*sigma));
        sum += k[i];
    }
    for (auto& v : k) v /= sum;
    return k;
}

HDRBuffer gaussianBlurH(const HDRBuffer& src, int radius, float sigma) {
    auto kernel = makeGaussianKernel(radius, sigma);
    HDRBuffer dst(src.w, src.h);
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            Vec3 sum(0,0,0);
            for (int k = -radius; k <= radius; k++) {
                int sx = clamp(x+k, 0, src.w-1);
                sum += src.at(sx, y) * kernel[k+radius];
            }
            dst.at(x,y) = sum;
        }
    }
    return dst;
}

HDRBuffer gaussianBlurV(const HDRBuffer& src, int radius, float sigma) {
    auto kernel = makeGaussianKernel(radius, sigma);
    HDRBuffer dst(src.w, src.h);
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            Vec3 sum(0,0,0);
            for (int k = -radius; k <= radius; k++) {
                int sy = clamp(y+k, 0, src.h-1);
                sum += src.at(x, sy) * kernel[k+radius];
            }
            dst.at(x,y) = sum;
        }
    }
    return dst;
}

HDRBuffer gaussianBlur(const HDRBuffer& src, int radius, float sigma) {
    return gaussianBlurV(gaussianBlurH(src, radius, sigma), radius, sigma);
}

// Downsample 2x
HDRBuffer downsample2x(const HDRBuffer& src) {
    int nw = src.w / 2;
    int nh = src.h / 2;
    HDRBuffer dst(nw, nh);
    for (int y = 0; y < nh; y++) {
        for (int x = 0; x < nw; x++) {
            dst.at(x,y) = (src.at(2*x,2*y) + src.at(2*x+1,2*y) +
                           src.at(2*x,2*y+1) + src.at(2*x+1,2*y+1)) * 0.25f;
        }
    }
    return dst;
}

// Upsample to target size using bilinear
HDRBuffer upsampleTo(const HDRBuffer& src, int tw, int th) {
    HDRBuffer dst(tw, th);
    for (int y = 0; y < th; y++) {
        for (int x = 0; x < tw; x++) {
            float u = (float)x / (tw-1);
            float v = (float)y / (th-1);
            dst.at(x,y) = src.sample(u, v);
        }
    }
    return dst;
}

// ─── Scene rendering ──────────────────────────────────────────────────────────

struct Light {
    float cx, cy;  // center (pixel coords)
    float radius;  // soft disc radius
    Vec3 color;
    float intensity;
};

// Soft circular gradient for a light source
float lightDisc(float dx, float dy, float radius) {
    float d = std::sqrt(dx*dx + dy*dy);
    float t = clamp(1.f - d/radius, 0.f, 1.f);
    return t * t * t;  // cubic falloff for soft appearance
}

HDRBuffer renderScene(int W, int H) {
    HDRBuffer hdr(W, H);

    // Background: deep night sky gradient
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float fy = (float)y / H;
            // Horizon glow
            float horizonGlow = std::exp(-std::pow(fy - 0.82f, 2) * 60.f) * 0.18f;
            Vec3 sky = Vec3(0.02f, 0.03f, 0.07f) + Vec3(0.3f, 0.15f, 0.05f) * horizonGlow;
            hdr.at(x,y) = sky;
        }
    }

    // Add stars (tiny bright points)
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> distX(0.f, (float)W);
    std::uniform_real_distribution<float> distY(0.f, (float)H * 0.8f);
    std::uniform_real_distribution<float> distB(0.3f, 1.8f);
    for (int i = 0; i < 200; i++) {
        int sx = (int)distX(rng);
        int sy = (int)distY(rng);
        if (sx < 0 || sx >= W || sy < 0 || sy >= H) continue;
        float b = distB(rng);
        hdr.at(sx, sy) = Vec3(b, b, b * 1.1f);
    }

    // Ground plane (reflective)
    for (int y = (int)(H * 0.82f); y < H; y++) {
        for (int x = 0; x < W; x++) {
            float t = (float)(y - (int)(H*0.82f)) / (H * 0.18f);
            Vec3 ground = Vec3(0.02f, 0.03f, 0.04f) * (1.f - t*0.8f);
            hdr.at(x,y) = ground;
        }
    }

    // Define bright light sources
    std::vector<Light> lights = {
        // Main sun/moon near horizon
        {(float)(W * 0.38f), (float)(H * 0.78f), 22.f, Vec3(1.0f, 0.92f, 0.7f),  28.0f},
        // Secondary warm light left
        {(float)(W * 0.15f), (float)(H * 0.74f), 12.f, Vec3(1.0f, 0.5f, 0.1f),   12.0f},
        // Cool blue light right
        {(float)(W * 0.72f), (float)(H * 0.71f), 10.f, Vec3(0.3f, 0.6f, 1.0f),   10.0f},
        // Small intense white spot
        {(float)(W * 0.58f), (float)(H * 0.66f),  6.f, Vec3(1.0f, 1.0f, 1.0f),   20.0f},
        // Distant tiny light
        {(float)(W * 0.82f), (float)(H * 0.76f),  4.f, Vec3(1.0f, 0.8f, 0.4f),   15.0f},
    };

    // Add light source discs to scene
    for (const auto& lt : lights) {
        int rx = (int)(lt.radius * 2 + 2);
        int x0 = clamp((int)(lt.cx - rx), 0, W-1);
        int x1 = clamp((int)(lt.cx + rx), 0, W-1);
        int y0 = clamp((int)(lt.cy - rx), 0, H-1);
        int y1 = clamp((int)(lt.cy + rx), 0, H-1);
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                float dx = (float)x - lt.cx;
                float dy = (float)y - lt.cy;
                float v = lightDisc(dx, dy, lt.radius) * lt.intensity;
                hdr.at(x,y) += lt.color * v;
            }
        }
        // Ground reflections (mirror below horizon)
        float horizonY = H * 0.82f;
        float reflY = 2.f * horizonY - lt.cy;
        if (reflY > 0 && reflY < H) {
            int rx2 = (int)(lt.radius * 1.5f + 2);
            int x0r = clamp((int)(lt.cx - rx2), 0, W-1);
            int x1r = clamp((int)(lt.cx + rx2), 0, W-1);
            int y0r = clamp((int)(reflY - rx2), 0, H-1);
            int y1r = clamp((int)(reflY + rx2), 0, H-1);
            for (int y = y0r; y <= y1r; y++) {
                for (int x = x0r; x <= x1r; x++) {
                    float dx = (float)x - lt.cx;
                    float dy = (float)y - reflY;
                    float v = lightDisc(dx, dy, lt.radius * 0.6f) * lt.intensity * 0.25f;
                    hdr.at(x,y) += lt.color * v;
                }
            }
        }
    }

    return hdr;
}

// ─── Bloom post-processing ────────────────────────────────────────────────────

// Brightness threshold extraction
HDRBuffer brightPass(const HDRBuffer& src, float threshold) {
    HDRBuffer dst(src.w, src.h);
    for (int i = 0; i < src.w * src.h; i++) {
        const Vec3& p = src.pixels[i];
        float lum = 0.2126f*p.x + 0.7152f*p.y + 0.0722f*p.z;
        float factor = clamp((lum - threshold) / (threshold + 0.5f), 0.f, 1.f);
        dst.pixels[i] = src.pixels[i] * factor;
    }
    return dst;
}

// Multi-level bloom: 4 mip levels
HDRBuffer computeBloom(const HDRBuffer& scene, float threshold) {
    HDRBuffer bright = brightPass(scene, threshold);

    // Level 0: full res blur
    HDRBuffer b0 = gaussianBlur(bright, 8, 4.0f);

    // Level 1: half res
    HDRBuffer d1 = downsample2x(bright);
    HDRBuffer b1 = gaussianBlur(d1, 8, 4.0f);
    b1 = upsampleTo(b1, scene.w, scene.h);

    // Level 2: quarter res
    HDRBuffer d2 = downsample2x(d1);
    HDRBuffer b2 = gaussianBlur(d2, 8, 4.0f);
    b2 = upsampleTo(b2, scene.w, scene.h);

    // Level 3: eighth res
    HDRBuffer d3 = downsample2x(d2);
    HDRBuffer b3 = gaussianBlur(d3, 8, 4.0f);
    b3 = upsampleTo(b3, scene.w, scene.h);

    // Combine with decreasing weight (wider = softer, lower weight)
    HDRBuffer bloom(scene.w, scene.h);
    bloom.addScaled(b0, 0.40f);
    bloom.addScaled(b1, 0.30f);
    bloom.addScaled(b2, 0.20f);
    bloom.addScaled(b3, 0.10f);

    return bloom;
}

// ─── Lens Flare ───────────────────────────────────────────────────────────────

struct LensFlare {
    float cx, cy;   // flare center source
    Vec3 color;
    float intensity;
};

// Hexagonal aperture shape (starburst)
float hexAperture(float dx, float dy, float r) {
    if (r < 1e-6f) return 0.f;
    float a1 = std::abs(dx);
    float a2 = std::abs(dy);
    float a3 = std::abs(dx * 0.5f + dy * 0.866f);
    float a4 = std::abs(dx * 0.5f - dy * 0.866f);
    float d = std::max(a1, std::max(a2, std::max(a3, a4)));
    // Use max of two values (simplified hex)
    float d2 = std::max(std::abs(dx), std::abs(dy * 1.1547f));
    float dist = std::min(d, d2);
    return clamp(1.f - dist/r, 0.f, 1.f);
}

// Starburst: 6-ray airy disk diffraction pattern
float starburst(float dx, float dy, float r, int rays) {
    float angle = std::atan2(dy, dx);
    float dist  = std::sqrt(dx*dx + dy*dy);
    float base  = std::exp(-dist / r);
    float spike = 0.f;
    float pi    = 3.14159265f;
    for (int i = 0; i < rays; i++) {
        float a = (float)i * pi / (float)rays;
        float c = std::abs(std::cos(angle - a));
        spike += std::pow(c, 40.f);
    }
    spike /= (float)rays;
    return base * (0.3f + 0.7f * spike);
}

HDRBuffer computeLensFlare(const HDRBuffer& scene, const std::vector<LensFlare>& flares) {
    int W = scene.w, H = scene.h;
    float cx = W * 0.5f, cy = H * 0.5f;
    HDRBuffer dst(W, H);

    for (const auto& fl : flares) {
        float dx = fl.cx - cx;
        float dy = fl.cy - cy;

        // Starburst at the flare source
        {
            int rx = std::min(80, W/4);
            int x0 = clamp((int)(fl.cx - rx), 0, W-1);
            int x1 = clamp((int)(fl.cx + rx), 0, W-1);
            int y0 = clamp((int)(fl.cy - rx), 0, H-1);
            int y1 = clamp((int)(fl.cy + rx), 0, H-1);
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    float px = (float)x - fl.cx;
                    float py = (float)y - fl.cy;
                    float v = starburst(px, py, 14.f, 6) * fl.intensity * 0.6f;
                    dst.at(x,y) += fl.color * v;
                }
            }
        }

        // Ghosts: reflections along the lens axis (source -> screen center -> far side)
        // Each ghost is a scaled, colored disc placed at position = center - t*(source-center)
        struct Ghost { float t; float r; Vec3 tint; float strength; };
        std::vector<Ghost> ghosts = {
            {0.4f,  18.f, Vec3(1.0f, 0.7f, 0.3f), 0.35f},
            {0.65f, 12.f, Vec3(0.5f, 0.8f, 1.0f), 0.28f},
            {0.85f,  8.f, Vec3(0.9f, 0.4f, 0.8f), 0.20f},
            {1.1f,  22.f, Vec3(0.3f, 1.0f, 0.6f), 0.15f},
            {1.35f,  6.f, Vec3(1.0f, 0.9f, 0.5f), 0.22f},
            {1.6f,  30.f, Vec3(0.6f, 0.6f, 1.0f), 0.10f},
            {-0.25f, 10.f,Vec3(1.0f, 0.5f, 0.2f), 0.18f},
        };

        for (const auto& g : ghosts) {
            float gx = cx - g.t * dx;
            float gy = cy - g.t * dy;
            int rx = (int)(g.r * 2 + 2);
            int x0 = clamp((int)(gx - rx), 0, W-1);
            int x1 = clamp((int)(gx + rx), 0, W-1);
            int y0 = clamp((int)(gy - rx), 0, H-1);
            int y1 = clamp((int)(gy + rx), 0, H-1);
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    float px = (float)x - gx;
                    float py = (float)y - gy;
                    float d  = std::sqrt(px*px + py*py);
                    float v  = clamp(1.f - d/g.r, 0.f, 1.f);
                    v = v * v * g.strength * fl.intensity;
                    dst.at(x,y) += (fl.color * g.tint) * v;
                }
            }
        }

        // Halo: large ring around the flare source
        {
            float haloR = 60.f;
            float haloW = 12.f;
            int rx = (int)(haloR + haloW + 2);
            int x0 = clamp((int)(fl.cx - rx), 0, W-1);
            int x1 = clamp((int)(fl.cx + rx), 0, W-1);
            int y0 = clamp((int)(fl.cy - rx), 0, H-1);
            int y1 = clamp((int)(fl.cy + rx), 0, H-1);
            for (int y = y0; y <= y1; y++) {
                for (int x = x0; x <= x1; x++) {
                    float px = (float)x - fl.cx;
                    float py = (float)y - fl.cy;
                    float d  = std::sqrt(px*px + py*py);
                    float ring = std::exp(-std::pow(d - haloR, 2) / (2*haloW*haloW));
                    float v = ring * fl.intensity * 0.15f;
                    dst.at(x,y) += Vec3(0.8f, 0.9f, 1.0f) * fl.color * v;
                }
            }
        }
    }

    return dst;
}

// ─── Tone mapping ─────────────────────────────────────────────────────────────

// ACES filmic approximation
Vec3 acesFilmic(Vec3 x) {
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    auto apply = [&](float v) {
        return clamp((v*(a*v+b)) / (v*(c*v+d)+e), 0.f, 1.f);
    };
    return {apply(x.x), apply(x.y), apply(x.z)};
}

// Gamma correction
Vec3 gammaCorrect(Vec3 c, float gamma = 2.2f) {
    float inv = 1.f / gamma;
    return {std::pow(std::max(0.f,c.x), inv),
            std::pow(std::max(0.f,c.y), inv),
            std::pow(std::max(0.f,c.z), inv)};
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    const int W = 512, H = 512;

    // 1. Render HDR scene
    HDRBuffer scene = renderScene(W, H);

    // 2. Bloom
    HDRBuffer bloom = computeBloom(scene, 1.2f);

    // 3. Lens flare — attach to main bright sources
    std::vector<LensFlare> flares = {
        {(float)(W * 0.38f), (float)(H * 0.78f), Vec3(1.0f, 0.92f, 0.7f), 1.0f},
        {(float)(W * 0.15f), (float)(H * 0.74f), Vec3(1.0f, 0.5f,  0.1f), 0.6f},
        {(float)(W * 0.58f), (float)(H * 0.66f), Vec3(1.0f, 1.0f,  1.0f), 0.8f},
    };
    HDRBuffer lensFlare = computeLensFlare(scene, flares);

    // 4. Composite: scene + bloom * strength + lens flare * strength
    const float bloomStrength  = 0.85f;
    const float flareStrength  = 0.55f;
    HDRBuffer composite(W, H);
    for (int i = 0; i < W * H; i++) {
        composite.pixels[i] = scene.pixels[i]
                            + bloom.pixels[i]     * bloomStrength
                            + lensFlare.pixels[i] * flareStrength;
    }

    // 5. Tone map + gamma
    std::vector<uint8_t> ldr(W * H * 3);
    for (int i = 0; i < W * H; i++) {
        Vec3 mapped = gammaCorrect(acesFilmic(composite.pixels[i]));
        ldr[i*3+0] = (uint8_t)(clamp(mapped.x, 0.f, 1.f) * 255.f);
        ldr[i*3+1] = (uint8_t)(clamp(mapped.y, 0.f, 1.f) * 255.f);
        ldr[i*3+2] = (uint8_t)(clamp(mapped.z, 0.f, 1.f) * 255.f);
    }

    // 6. Save PNG
    stbi_write_png("bloom_output.png", W, H, 3, ldr.data(), W * 3);

    printf("Saved bloom_output.png (%dx%d)\n", W, H);
    return 0;
}
