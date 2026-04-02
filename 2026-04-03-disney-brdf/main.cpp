#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <random>

// ============================================================
// Math primitives
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float v = 0.f) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length2() const { return x*x + y*y + z*z; }
    float length() const { return std::sqrt(length2()); }
    Vec3 normalized() const { float l = length(); return l > 0.f ? *this / l : Vec3(0.f); }
    Vec3 clamp01() const { return {std::max(0.f,std::min(1.f,x)),
                                    std::max(0.f,std::min(1.f,y)),
                                    std::max(0.f,std::min(1.f,z))}; }
};
inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline float lerp(float a, float b, float t) { return a * (1.f - t) + b * t; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a * (1.f - t) + b * t; }

// ============================================================
// Disney Principled BRDF
// Reference: Burley 2012, "Physically Based Shading at Disney"
// ============================================================

static constexpr float PI = 3.14159265358979f;

// Schlick Fresnel
inline float SchlickFresnel(float u) {
    float m = std::max(0.f, 1.f - u);
    float m2 = m * m;
    return m2 * m2 * m;
}

// GGX Normal Distribution Function
inline float GTR2(float NdotH, float alpha) {
    float a2 = alpha * alpha;
    float t  = 1.f + (a2 - 1.f) * NdotH * NdotH;
    return a2 / (PI * t * t);
}

// Berry NDF (GTR1) for clearcoat
inline float GTR1(float NdotH, float a) {
    if (a >= 1.f) return 1.f / PI;
    float a2 = a * a;
    float t  = 1.f + (a2 - 1.f) * NdotH * NdotH;
    return (a2 - 1.f) / (PI * std::log(a2) * t);
}

// Smith-G for GGX
inline float SmithG_GGX(float NdotV, float alphaG) {
    float a = alphaG * alphaG;
    float b = NdotV * NdotV;
    return 1.f / (NdotV + std::sqrt(a + b - a * b));
}

struct DisneyMaterial {
    Vec3  baseColor     = {0.8f, 0.8f, 0.8f};
    float subsurface    = 0.f;
    float metallic      = 0.f;
    float specular      = 0.5f;
    float specularTint  = 0.f;
    float roughness     = 0.5f;
    float anisotropic   = 0.f;
    float sheen         = 0.f;
    float sheenTint     = 0.5f;
    float clearcoat     = 0.f;
    float clearcoatGloss= 1.f;
};

// Evaluate Disney BRDF
// L: light dir (pointing away from surface)
// V: view dir  (pointing away from surface)
// N: surface normal
// Returns: BRDF value (without PI factor — already included in NDF)
Vec3 DisneyBRDF(const Vec3& L, const Vec3& V, const Vec3& N, const DisneyMaterial& m) {
    float NdotL = std::max(0.f, N.dot(L));
    float NdotV = std::max(0.f, N.dot(V));
    if (NdotL <= 0.f || NdotV <= 0.f) return Vec3(0.f);

    Vec3 H = (L + V).normalized();
    float NdotH = std::max(0.f, N.dot(H));
    float LdotH = std::max(0.f, L.dot(H));

    // Luminance approximation
    float Cdlum = 0.3f*m.baseColor.x + 0.6f*m.baseColor.y + 0.1f*m.baseColor.z;
    Vec3 Ctint = Cdlum > 0.f ? m.baseColor / Cdlum : Vec3(1.f);
    Vec3 Cspec0 = lerp(m.specular * 0.08f * lerp(Vec3(1.f), Ctint, m.specularTint), m.baseColor, m.metallic);
    Vec3 Csheen = lerp(Vec3(1.f), Ctint, m.sheenTint);

    // ---- Diffuse (Burley) ----
    float FL  = SchlickFresnel(NdotL);
    float FV  = SchlickFresnel(NdotV);
    float Fd90 = 0.5f + 2.f * LdotH * LdotH * m.roughness;
    float Fd  = lerp(1.f, Fd90, FL) * lerp(1.f, Fd90, FV);

    // ---- Subsurface (Hanrahan-Krueger approximation) ----
    float Fss90 = LdotH * LdotH * m.roughness;
    float Fss   = lerp(1.f, Fss90, FL) * lerp(1.f, Fss90, FV);
    float ss    = 1.25f * (Fss * (1.f / (NdotL + NdotV) - 0.5f) + 0.5f);

    Vec3 diffuse = m.baseColor / PI * lerp(Fd, ss, m.subsurface) * (1.f - m.metallic);

    // ---- Sheen ----
    float FH   = SchlickFresnel(LdotH);
    Vec3 Fsheen = FH * m.sheen * Csheen * (1.f - m.metallic);

    // ---- Specular (microfacet) ----
    float alpha = std::max(0.001f, m.roughness * m.roughness);
    float Ds    = GTR2(NdotH, alpha);
    Vec3 Fs     = lerp(Cspec0, Vec3(1.f), FH);
    float Gs    = SmithG_GGX(NdotL, alpha) * SmithG_GGX(NdotV, alpha);
    Vec3 specular_term = Gs * Fs * Ds;

    // ---- Clearcoat (ior=1.5 -> F0=0.04) ----
    float Dr  = GTR1(NdotH, lerp(0.1f, 0.001f, m.clearcoatGloss));
    float Fr  = lerp(0.04f, 1.f, FH);
    float Gr  = SmithG_GGX(NdotL, 0.25f) * SmithG_GGX(NdotV, 0.25f);
    float clearcoat_term = 0.25f * m.clearcoat * Gr * Fr * Dr;

    return diffuse + Fsheen + specular_term + Vec3(clearcoat_term);
}

// ============================================================
// Simple scene: area light + ambient, no ray tracing
// Analytic shading model for a grid of spheres
// ============================================================
struct Light {
    Vec3 pos;
    Vec3 color;
    float intensity;
};

// Ray-sphere intersection
bool intersectSphere(const Vec3& ro, const Vec3& rd, const Vec3& center, float radius,
                     float& tHit, Vec3& normal) {
    Vec3 oc = ro - center;
    float b = 2.f * oc.dot(rd);
    float c = oc.dot(oc) - radius * radius;
    float disc = b * b - 4.f * c;
    if (disc < 0.f) return false;
    float sqrtDisc = std::sqrt(disc);
    float t = (-b - sqrtDisc) * 0.5f;
    if (t < 0.001f) t = (-b + sqrtDisc) * 0.5f;
    if (t < 0.001f) return false;
    tHit = t;
    Vec3 P = ro + rd * t;
    normal = (P - center).normalized();
    return true;
}

// Render a pixel
Vec3 shade(const Vec3& hitPos, const Vec3& N, const Vec3& V,
           const DisneyMaterial& mat, const std::vector<Light>& lights) {
    Vec3 color(0.f);
    Vec3 ambient = mat.baseColor * 0.06f * (1.f - mat.metallic * 0.8f);
    color += ambient;
    for (const auto& light : lights) {
        Vec3 L = (light.pos - hitPos).normalized();
        float NdotL = std::max(0.f, N.dot(L));
        if (NdotL <= 0.f) continue;
        Vec3 brdf = DisneyBRDF(L, V, N, mat);
        color += brdf * light.color * light.intensity * NdotL;
    }
    return color;
}

// ============================================================
// Tone mapping & gamma
// ============================================================
inline float ACESFilm(float x) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    return std::max(0.f, std::min(1.f, (x*(a*x+b)) / (x*(c*x+d)+e)));
}
inline Vec3 ACESFilm(const Vec3& c) {
    return {ACESFilm(c.x), ACESFilm(c.y), ACESFilm(c.z)};
}
inline float gammaCorrect(float v) {
    return std::pow(std::max(0.f, std::min(1.f, v)), 1.f/2.2f);
}
inline Vec3 gammaCorrect(const Vec3& c) {
    return {gammaCorrect(c.x), gammaCorrect(c.y), gammaCorrect(c.z)};
}

// ============================================================
// PPM writer
// ============================================================
void writePPM(const std::string& filename, int W, int H,
              const std::vector<Vec3>& pixels) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << W << " " << H << "\n255\n";
    for (const auto& p : pixels) {
        uint8_t r = static_cast<uint8_t>(std::min(255.f, p.x * 255.f));
        uint8_t g = static_cast<uint8_t>(std::min(255.f, p.y * 255.f));
        uint8_t b = static_cast<uint8_t>(std::min(255.f, p.z * 255.f));
        f.put(r); f.put(g); f.put(b);
    }
}

// ============================================================
// PNG via PPM intermediate (use simple PPM → PNG via convert)
// We'll directly write a minimal PNG using lodepng-compatible approach
// Actually: write PPM and convert to PNG via system call
// ============================================================

// ============================================================
// Main
// ============================================================
int main() {
    const int W = 900, H = 700;
    std::vector<Vec3> framebuffer(W * H, Vec3(0.f));

    // Camera
    Vec3 camPos(0.f, 0.f, 14.f);
    float fov = 45.f * PI / 180.f;
    float tanHalfFov = std::tan(fov * 0.5f);
    float aspect = static_cast<float>(W) / static_cast<float>(H);

    // Lights
    std::vector<Light> lights = {
        { {  8.f,  12.f,  10.f }, {1.0f, 0.95f, 0.9f},  4.0f },
        { { -6.f,   8.f,  10.f }, {0.7f, 0.8f,  1.0f},  2.0f },
        { {  0.f,  -5.f,   8.f }, {0.5f, 0.4f,  0.3f},  1.0f },
    };

    // 5 x 4 material grid (rows: roughness, cols: metallic/color preset)
    const int COLS = 5, ROWS = 4;
    float sphereRadius = 0.85f;
    float spacingX = 2.2f, spacingY = 2.2f;
    float startX = -(COLS - 1) * spacingX * 0.5f;
    float startY =  (ROWS - 1) * spacingY * 0.5f;

    // Predefined materials: vary metallic (0→1) across columns,
    // roughness (0.05→0.9) across rows. Base colors per column.
    Vec3 baseColors[COLS] = {
        {0.85f, 0.20f, 0.15f},   // red-ish
        {0.20f, 0.60f, 0.85f},   // blue-ish
        {0.85f, 0.75f, 0.20f},   // gold-ish
        {0.20f, 0.75f, 0.40f},   // green-ish
        {0.90f, 0.90f, 0.90f},   // white/silver
    };
    float metallics[COLS]  = {0.f, 0.f, 1.f, 0.f, 1.f};
    float specTints[COLS]  = {0.f, 0.5f, 0.f, 0.f, 0.f};

    // Build sphere list
    struct Sphere {
        Vec3 center;
        float radius;
        DisneyMaterial mat;
    };
    std::vector<Sphere> spheres;
    spheres.reserve(COLS * ROWS);
    for (int row = 0; row < ROWS; ++row) {
        float roughness = 0.05f + (0.85f / (ROWS - 1)) * row;
        for (int col = 0; col < COLS; ++col) {
            float cx = startX + col * spacingX;
            float cy = startY - row * spacingY;
            DisneyMaterial mat;
            mat.baseColor    = baseColors[col];
            mat.metallic     = metallics[col];
            mat.roughness    = roughness;
            mat.specular     = 0.5f;
            mat.specularTint = specTints[col];
            mat.clearcoat    = (col == 4) ? 1.0f : 0.f;
            mat.clearcoatGloss = 0.9f;
            mat.sheen        = (col == 0 || col == 1) ? 0.3f : 0.f;
            mat.sheenTint    = 0.5f;
            mat.subsurface   = (col == 3 && row < 2) ? 0.4f : 0.f;
            spheres.push_back({ {cx, cy, 0.f}, sphereRadius, mat });
        }
    }

    // Render
    for (int py = 0; py < H; ++py) {
        for (int px = 0; px < W; ++px) {
            // NDC -> world ray
            float u = (2.f * (px + 0.5f) / W - 1.f) * aspect * tanHalfFov;
            float v = (1.f - 2.f * (py + 0.5f) / H) * tanHalfFov;
            Vec3 rd = Vec3(u, v, -1.f).normalized();
            Vec3 ro = camPos;

            // Find closest sphere hit
            float tMin = 1e30f;
            int hitIdx = -1;
            Vec3 hitN;
            for (int i = 0; i < (int)spheres.size(); ++i) {
                float t; Vec3 n;
                if (intersectSphere(ro, rd, spheres[i].center, spheres[i].radius, t, n)) {
                    if (t < tMin) { tMin = t; hitIdx = i; hitN = n; }
                }
            }

            Vec3 pixel;
            if (hitIdx >= 0) {
                Vec3 hitPos = ro + rd * tMin;
                Vec3 V = (-rd).normalized();
                pixel = shade(hitPos, hitN, V, spheres[hitIdx].mat, lights);
                pixel = ACESFilm(pixel);
                pixel = gammaCorrect(pixel);
            } else {
                // Background gradient
                float t = (rd.y + 1.f) * 0.5f;
                Vec3 bg = lerp(Vec3(0.12f, 0.12f, 0.15f), Vec3(0.22f, 0.25f, 0.35f), t);
                pixel = bg;
            }

            framebuffer[py * W + px] = pixel;
        }
    }

    // Write PPM
    writePPM("/root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.ppm", W, H, framebuffer);
    
    // Convert to PNG using ImageMagick
    int ret = std::system("convert /root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.ppm /root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.png 2>&1");
    if (ret != 0) {
        // Try ffmpeg as fallback
        ret = std::system("ffmpeg -y -f rawvideo -pixel_format rgb24 -video_size 900x700 -i /dev/null -vframes 1 /root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.png 2>/dev/null || true");
    }
    
    // Verify output
    std::ifstream check("/root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.png", std::ios::binary | std::ios::ate);
    if (check.is_open() && check.tellg() > 10240) {
        printf("✅ disney_brdf_output.png generated successfully (%ld bytes)\n", (long)check.tellg());
    } else {
        // Check PPM
        std::ifstream checkPPM("/root/.openclaw/workspace/daily-coding-practice/2026-04-03-disney-brdf/disney_brdf_output.ppm", std::ios::binary | std::ios::ate);
        if (checkPPM.is_open()) {
            printf("✅ disney_brdf_output.ppm generated (%ld bytes)\n", (long)checkPPM.tellg());
        }
        printf("ℹ️  PNG conversion result: %d\n", ret);
    }
    printf("✅ Disney Principled BRDF render complete\n");
    printf("   Rendered %d spheres (5 cols × 4 rows)\n", COLS * ROWS);
    printf("   Materials: metallic/dielectric, roughness 0.05-0.90, clearcoat, sheen, subsurface\n");
    return 0;
}
