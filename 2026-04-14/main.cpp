/**
 * Volumetric Cloud Renderer
 * 
 * 技术特性：
 * - FBM (Fractional Brownian Motion) 分形噪声生成3D云密度场
 * - 光线步进 (Ray Marching) 体积渲染
 * - Beer-Lambert 衰减定律
 * - Henyey-Greenstein 相位函数（前向散射）
 * - 多次散射近似（环境光照贡献）
 * - 程序化天空背景（大气渐变）
 * - 软光栅化输出 PPM -> PNG
 */

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iomanip>

// ===================== Math =====================
struct Vec3 {
    float x, y, z;
    Vec3(float v = 0) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { float l = length(); return l > 1e-8f ? *this / l : Vec3(0); }
    Vec3 clamp(float lo, float hi) const {
        return {std::max(lo, std::min(hi, x)),
                std::max(lo, std::min(hi, y)),
                std::max(lo, std::min(hi, z))};
    }
};

Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a * (1-t) + b * t; }

// ===================== Hash / Noise =====================
// Permutation hash
static uint32_t hash3(int ix, int iy, int iz) {
    uint32_t h = (uint32_t)(ix * 1234567 + iy * 7654321 + iz * 9999991);
    h ^= h >> 16;
    h *= 0x45d9f3b;
    h ^= h >> 16;
    h *= 0x45d9f3b;
    h ^= h >> 16;
    return h;
}

static float gradNoise3(float x, float y, float z) {
    int ix = (int)std::floor(x), iy = (int)std::floor(y), iz = (int)std::floor(z);
    float fx = x - ix, fy = y - iy, fz = z - iz;
    // Smoothstep
    auto s = [](float t){ return t*t*t*(t*(t*6-15)+10); };
    float ux = s(fx), uy = s(fy), uz = s(fz);

    // Gradient directions (unit cube corners)
    auto grad = [](uint32_t h, float dx, float dy, float dz) -> float {
        int h4 = h & 15;
        float u = (h4 < 8) ? dx : dy;
        float v = (h4 < 4) ? dy : ((h4==12||h4==14) ? dx : dz);
        return ((h4 & 1) ? -u : u) + ((h4 & 2) ? -v : v);
    };

    float n000 = grad(hash3(ix,   iy,   iz),   fx,   fy,   fz);
    float n100 = grad(hash3(ix+1, iy,   iz),   fx-1, fy,   fz);
    float n010 = grad(hash3(ix,   iy+1, iz),   fx,   fy-1, fz);
    float n110 = grad(hash3(ix+1, iy+1, iz),   fx-1, fy-1, fz);
    float n001 = grad(hash3(ix,   iy,   iz+1), fx,   fy,   fz-1);
    float n101 = grad(hash3(ix+1, iy,   iz+1), fx-1, fy,   fz-1);
    float n011 = grad(hash3(ix,   iy+1, iz+1), fx,   fy-1, fz-1);
    float n111 = grad(hash3(ix+1, iy+1, iz+1), fx-1, fy-1, fz-1);

    float x0 = n000*(1-ux) + n100*ux;
    float x1 = n010*(1-ux) + n110*ux;
    float x2 = n001*(1-ux) + n101*ux;
    float x3 = n011*(1-ux) + n111*ux;
    float y0 = x0*(1-uy) + x1*uy;
    float y1 = x2*(1-uy) + x3*uy;
    return y0*(1-uz) + y1*uz;
}

// FBM - Fractional Brownian Motion
static float fbm(float x, float y, float z, int octaves = 6) {
    float value = 0.0f;
    float amplitude = 0.5f;
    float frequency = 1.0f;
    float total_amplitude = 0.0f;
    for (int i = 0; i < octaves; i++) {
        value += amplitude * gradNoise3(x * frequency, y * frequency, z * frequency);
        total_amplitude += amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }
    return value / total_amplitude;
}

// ===================== Cloud Density =====================
// Cloud layer: altitude range [cloudBottom, cloudTop]
static const float CLOUD_BOTTOM = 0.4f;
static const float CLOUD_TOP    = 0.85f;
static const float CLOUD_SCALE  = 2.5f;

float cloudDensity(const Vec3& p) {
    float altitude = p.y; // 0=ground, 1=top of scene

    // Altitude gradient falloff
    if (altitude < CLOUD_BOTTOM || altitude > CLOUD_TOP) return 0.0f;
    float altFade = std::sin(3.14159f * (altitude - CLOUD_BOTTOM) / (CLOUD_TOP - CLOUD_BOTTOM));
    altFade = altFade * altFade;

    // Domain warp for more natural shapes
    float warpX = fbm(p.x * 1.3f + 0.5f, p.y * 0.7f, p.z * 1.3f - 0.5f, 3) * 0.4f;
    float warpZ = fbm(p.x * 1.3f - 0.5f, p.y * 0.7f, p.z * 1.3f + 0.5f, 3) * 0.4f;

    float wx = p.x * CLOUD_SCALE + warpX;
    float wy = p.y * CLOUD_SCALE * 0.4f;
    float wz = p.z * CLOUD_SCALE + warpZ;

    float noise = fbm(wx, wy, wz, 6);
    noise = noise * 0.5f + 0.5f; // remap [-1,1] -> [0,1]

    // Threshold: density only where noise > 0.42
    float density = std::max(0.0f, noise - 0.42f) * 3.5f;
    density *= altFade;

    return density;
}

// ===================== Henyey-Greenstein Phase Function =====================
float hgPhase(float cosTheta, float g) {
    float g2 = g * g;
    return (1.0f - g2) / (4.0f * 3.14159f * std::pow(1.0f + g2 - 2.0f * g * cosTheta, 1.5f));
}

// ===================== Sky Background =====================
Vec3 skyColor(const Vec3& dir) {
    float t = std::max(0.0f, dir.y) * 0.5f + 0.5f;
    Vec3 zenith(0.08f, 0.28f, 0.65f);
    Vec3 horizon(0.55f, 0.73f, 0.92f);
    Vec3 ground(0.45f, 0.55f, 0.52f);
    if (dir.y < 0) {
        float tb = std::max(0.0f, -dir.y);
        return mix(horizon, ground, tb * 2.0f);
    }
    return mix(horizon, zenith, t);
}

// ===================== Sun =====================
static const Vec3 SUN_DIR = Vec3(0.6f, 0.8f, 0.4f).normalize();
static const Vec3 SUN_COLOR(1.2f, 1.0f, 0.7f);

// ===================== Volumetric March =====================
struct MarchResult {
    Vec3 color;
    float transmittance; // 1 = fully transparent
};

// Light march toward sun to compute in-scattering
float lightMarch(const Vec3& pos) {
    const int LIGHT_STEPS = 6;
    const float LIGHT_STEP = 0.08f;
    float density_accum = 0.0f;
    Vec3 p = pos;
    for (int i = 0; i < LIGHT_STEPS; i++) {
        p += SUN_DIR * LIGHT_STEP;
        density_accum += cloudDensity(p);
    }
    // Beer-Lambert transmittance toward sun
    return std::exp(-density_accum * LIGHT_STEP * 8.0f);
}

MarchResult volumetricMarch(const Vec3& rayOrigin, const Vec3& rayDir) {
    const int MAX_STEPS = 64;
    const float STEP_SIZE = 0.025f;

    Vec3 transmittanceColor(1.0f);
    Vec3 scattered(0.0f);
    float T = 1.0f; // transmittance

    float cosTheta = rayDir.dot(SUN_DIR);
    float phase = hgPhase(cosTheta, 0.65f) * 0.7f + hgPhase(cosTheta, -0.2f) * 0.3f;

    Vec3 p = rayOrigin;
    for (int i = 0; i < MAX_STEPS && T > 0.01f; i++) {
        float d = cloudDensity(p);
        if (d > 0.001f) {
            float sigma_a = d * 6.0f;  // absorption
            float sigma_s = d * 10.0f; // scattering
            float sigma_t = sigma_a + sigma_s;

            float stepT = std::exp(-sigma_t * STEP_SIZE);
            float dT = T * (1.0f - stepT);

            // In-scattering: sun direct
            float sunT = lightMarch(p);
            Vec3 sunLight = SUN_COLOR * sunT * phase;

            // Ambient (multiple scattering approximation)
            Vec3 ambLight = Vec3(0.35f, 0.45f, 0.65f) * 0.3f;

            // Combine
            Vec3 inScatter = (sunLight + ambLight) * dT;
            scattered += inScatter;
            T *= stepT;
        }
        p += rayDir * STEP_SIZE;
        // Exit cloud layer
        if (p.y > 1.1f || p.y < 0.0f) break;
    }

    return {scattered, T};
}

// ===================== Camera =====================
Vec3 rayDirection(int px, int py, int W, int H, float fovDeg) {
    float aspect = (float)W / H;
    float tanHalfFov = std::tan(fovDeg * 0.5f * 3.14159f / 180.0f);
    float u = (2.0f * (px + 0.5f) / W - 1.0f) * aspect * tanHalfFov;
    float v = (1.0f - 2.0f * (py + 0.5f) / H) * tanHalfFov;
    return Vec3(u, v, -1.0f).normalize();
}

// ===================== Tone Mapping =====================
Vec3 acesToneMap(const Vec3& c) {
    float a = 2.51f, b = 0.03f, cc2 = 2.43f, d = 0.59f, e = 0.14f;
    auto tm = [&](float x) {
        return std::max(0.0f, std::min(1.0f, (x*(a*x+b)) / (x*(cc2*x+d)+e)));
    };
    return {tm(c.x), tm(c.y), tm(c.z)};
}

// ===================== PPM Write =====================
void writePPM(const std::string& filename, const std::vector<Vec3>& pixels, int W, int H) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << W << " " << H << "\n255\n";
    for (const auto& p : pixels) {
        uint8_t r = (uint8_t)(std::max(0.0f, std::min(1.0f, p.x)) * 255.0f);
        uint8_t g = (uint8_t)(std::max(0.0f, std::min(1.0f, p.y)) * 255.0f);
        uint8_t b = (uint8_t)(std::max(0.0f, std::min(1.0f, p.z)) * 255.0f);
        f.write((char*)&r, 1);
        f.write((char*)&g, 1);
        f.write((char*)&b, 1);
    }
}

// ===================== Main =====================
int main() {
    const int W = 800, H = 450;
    std::vector<Vec3> pixels(W * H);

    // Camera: looking forward (slightly upward to see clouds)
    Vec3 camPos(0.5f, 0.05f, 0.5f);

    // Camera orientation: looking toward horizon + slight tilt
    // We'll use a simple look-at matrix
    // Actually we directly map pixel->ray with tilt
    int done = 0;

    std::cout << "Rendering " << W << "x" << H << " volumetric cloud image..." << std::endl;

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            // Map pixel to ray direction
            float aspect = (float)W / H;
            float fovY = 60.0f * 3.14159f / 180.0f;
            float tanHalfFov = std::tan(fovY * 0.5f);

            float u = (2.0f * (x + 0.5f) / W - 1.0f) * aspect * tanHalfFov;
            float v = (1.0f - 2.0f * (y + 0.5f) / H) * tanHalfFov;

            // Camera tilted slightly upward (pitch angle)
            float pitchRad = 15.0f * 3.14159f / 180.0f;
            float cosPitch = std::cos(pitchRad), sinPitch = std::sin(pitchRad);
            // Forward: (0, sinPitch, -cosPitch), Up: (0, cosPitch, sinPitch)
            Vec3 forward(0, sinPitch, -cosPitch);
            Vec3 up(0, cosPitch, sinPitch);
            Vec3 right(1, 0, 0);

            Vec3 rayDir = (forward + right * u + up * v).normalize();

            // Sky color as background
            Vec3 bgColor = skyColor(rayDir);

            // Sun disk
            float sunCos = rayDir.dot(SUN_DIR);
            if (sunCos > 0.9998f) {
                bgColor = Vec3(2.0f, 1.8f, 1.4f);
            } else if (sunCos > 0.998f) {
                float t = (sunCos - 0.998f) / 0.0018f;
                bgColor = mix(bgColor, Vec3(1.8f, 1.5f, 1.0f), t * 0.8f);
            }

            // Ray-march clouds
            // Only march if ray is going somewhat upward (into cloud layer)
            Vec3 finalColor = bgColor;
            if (rayDir.y > -0.1f) {
                // Intersect ray with cloud bounding box (y: CLOUD_BOTTOM..CLOUD_TOP in world)
                // World: camPos.y=0.05, cloud is in [0.4, 0.85] of unit cube
                float tStart = 0.0f, tEnd = 3.5f;
                if (rayDir.y > 0.001f) {
                    tStart = std::max(0.0f, (CLOUD_BOTTOM - camPos.y) / rayDir.y);
                    tEnd   = (CLOUD_TOP   - camPos.y) / rayDir.y;
                } else if (rayDir.y < -0.001f) {
                    tEnd = std::min(tEnd, (CLOUD_BOTTOM - camPos.y) / rayDir.y);
                }

                if (tEnd > tStart) {
                    Vec3 startPos = camPos + rayDir * tStart;
                    MarchResult result = volumetricMarch(startPos, rayDir);
                    // Composite: cloud in front of background
                    finalColor = bgColor * result.transmittance + result.color;
                }
            }

            // Tone mapping
            finalColor = acesToneMap(finalColor);
            // Gamma correct
            finalColor.x = std::pow(std::max(0.0f, finalColor.x), 1.0f/2.2f);
            finalColor.y = std::pow(std::max(0.0f, finalColor.y), 1.0f/2.2f);
            finalColor.z = std::pow(std::max(0.0f, finalColor.z), 1.0f/2.2f);

            pixels[y * W + x] = finalColor;
        }
        done++;
        if (done % 50 == 0) {
            std::cout << "  Row " << done << "/" << H << " (" << (done*100/H) << "%)" << std::endl;
        }
    }

    writePPM("volumetric_cloud_output.ppm", pixels, W, H);
    std::cout << "Saved volumetric_cloud_output.ppm" << std::endl;

    // Convert to PNG using Python/Pillow (no ImageMagick dependency)
    int ret = system("python3 -c \""
                     "from PIL import Image; "
                     "img = Image.open('volumetric_cloud_output.ppm'); "
                     "img.save('volumetric_cloud_output.png'); "
                     "print('PNG saved:', img.size)"
                     "\" 2>&1");
    if (ret == 0) {
        std::cout << "Converted to PNG: volumetric_cloud_output.png" << std::endl;
    } else {
        std::cerr << "PNG conversion failed (code " << ret << ")" << std::endl;
        return 1;
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
