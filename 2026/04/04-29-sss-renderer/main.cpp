/*
 * Subsurface Scattering (SSS) Renderer
 * 次表面散射渲染器 - 软光栅化实现
 *
 * 技术要点：
 * - BSSRDF (双向次表面散射反射分布函数) 近似
 * - Jensen et al. 2001 Dipole Diffusion Model
 * - 皮肤多层散射剖面 (Scatter Profile)
 * - 高斯混合散射近似 (Sum of Gaussians)
 * - 分离轴 Blinn-Phong 高光 + SSS 漫反射
 * - 皮肤材质球阵列（不同散射半径 / 颜色）
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <string>
#include <fstream>
#include <sstream>
#include <random>

// ============================================================
// Math
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float len2() const { return dot(*this); }
    float len()  const { return sqrtf(len2()); }
    Vec3 norm()  const { float l=len(); return l>1e-7f?(*this)*(1/l):Vec3(0,0,1); }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    Vec3 clamp(float lo=0, float hi=1) const {
        return {std::max(lo, std::min(hi, x)),
                std::max(lo, std::min(hi, y)),
                std::max(lo, std::min(hi, z))};
    }
};
inline Vec3 operator*(float t, const Vec3& v) { return v*t; }

// ============================================================
// Framebuffer
// ============================================================
struct Framebuffer {
    int W, H;
    std::vector<Vec3> buf;
    explicit Framebuffer(int w, int h) : W(w), H(h), buf(w*h, Vec3(0,0,0)) {}
    void set(int x, int y, const Vec3& c) {
        if(x<0||x>=W||y<0||y>=H) return;
        buf[y*W+x] = c;
    }
    Vec3 get(int x, int y) const {
        if(x<0||x>=W||y<0||y>=H) return {};
        return buf[y*W+x];
    }
    // Tone-map (ACES Filmic) + gamma
    Vec3 tonemap(const Vec3& c) const {
        auto aces = [](float x) {
            const float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
            return std::max(0.f, std::min(1.f, (x*(a*x+b))/(x*(cc*x+d)+e)));
        };
        return Vec3(aces(c.x), aces(c.y), aces(c.z));
    }
    Vec3 gamma(const Vec3& c) const {
        return {powf(c.x,1/2.2f), powf(c.y,1/2.2f), powf(c.z,1/2.2f)};
    }
    bool save_ppm(const std::string& path) const {
        std::ofstream f(path, std::ios::binary);
        if(!f) return false;
        f << "P6\n" << W << " " << H << "\n255\n";
        for(const auto& p : buf) {
            Vec3 t = gamma(tonemap(p));
            f.put((unsigned char)(t.x*255));
            f.put((unsigned char)(t.y*255));
            f.put((unsigned char)(t.z*255));
        }
        return true;
    }
};

// ============================================================
// PPM → PNG converter (via P3 intermediate, using raw PPM)
// We'll write PPM and rename; the validation script handles PNG check.
// Actually we output .png directly by embedding raw PPM bytes in a
// proper PNG using a minimal PNG writer.
// ============================================================
// --- Minimal PNG writer (DEFLATE store-only, no compression) ---
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
    static uint32_t table[256];
    static bool init = false;
    if(!init) {
        for(uint32_t i=0;i<256;i++){
            uint32_t c=i;
            for(int k=0;k<8;k++) c = (c&1)?(0xEDB88320u^(c>>1)):(c>>1);
            table[i]=c;
        }
        init=true;
    }
    crc ^= 0xFFFFFFFFu;
    for(size_t i=0;i<len;i++) crc = table[(crc^data[i])&0xFF]^(crc>>8);
    return crc ^ 0xFFFFFFFFu;
}
static void write_u32be(std::ofstream& f, uint32_t v) {
    uint8_t b[4]={(uint8_t)(v>>24),(uint8_t)(v>>16),(uint8_t)(v>>8),(uint8_t)v};
    f.write((char*)b,4);
}
static void write_chunk(std::ofstream& f, const char* type, const std::vector<uint8_t>& data) {
    write_u32be(f, (uint32_t)data.size());
    f.write(type,4);
    if(!data.empty()) f.write((char*)data.data(), data.size());
    uint32_t crc = crc32_update(0, (const uint8_t*)type, 4);
    crc = crc32_update(crc, data.data(), data.size());
    write_u32be(f, crc);
}

bool save_png(const std::string& path, int W, int H, const std::vector<Vec3>& buf) {
    // prepare raw image data: filter byte 0 + RGB per row
    auto clamp01 = [](float v) -> uint8_t {
        return (uint8_t)(std::max(0.f,std::min(1.f,v))*255+0.5f);
    };
    std::vector<uint8_t> raw;
    raw.reserve((3*W+1)*H);
    for(int y=0;y<H;y++){
        raw.push_back(0); // None filter
        for(int x=0;x<W;x++){
            const Vec3& p = buf[y*W+x];
            // ACES + gamma inline
            auto aces=[](float x){
                const float a=2.51f,b=0.03f,cc=2.43f,d=0.59f,e=0.14f;
                return std::max(0.f,std::min(1.f,(x*(a*x+b))/(x*(cc*x+d)+e)));
            };
            auto gam=[](float v){ return powf(std::max(0.f,std::min(1.f,v)),1/2.2f); };
            raw.push_back(clamp01(gam(aces(p.x))));
            raw.push_back(clamp01(gam(aces(p.y))));
            raw.push_back(clamp01(gam(aces(p.z))));
        }
    }
    // DEFLATE store (non-compressed blocks, max 65535 bytes each)
    std::vector<uint8_t> zlib;
    zlib.push_back(0x78); zlib.push_back(0x01); // zlib header
    size_t pos = 0, total = raw.size();
    uint32_t adler_s1=1, adler_s2=0;
    for(size_t i=0;i<total;i++){
        adler_s1 = (adler_s1 + raw[i]) % 65521;
        adler_s2 = (adler_s2 + adler_s1) % 65521;
    }
    while(pos < total) {
        size_t block = std::min((size_t)65535, total - pos);
        bool last = (pos + block >= total);
        zlib.push_back(last ? 0x01 : 0x00);
        uint16_t len16 = (uint16_t)block;
        uint16_t nlen  = ~len16;
        zlib.push_back(len16 & 0xFF); zlib.push_back(len16 >> 8);
        zlib.push_back(nlen & 0xFF);  zlib.push_back(nlen >> 8);
        for(size_t i=0;i<block;i++) zlib.push_back(raw[pos+i]);
        pos += block;
    }
    // Adler-32
    zlib.push_back((adler_s2>>8)&0xFF); zlib.push_back(adler_s2&0xFF);
    zlib.push_back((adler_s1>>8)&0xFF); zlib.push_back(adler_s1&0xFF);

    std::ofstream f(path, std::ios::binary);
    if(!f) return false;
    // PNG signature
    const uint8_t sig[]={0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A};
    f.write((char*)sig,8);
    // IHDR
    std::vector<uint8_t> ihdr(13);
    auto pu=[&](int off,uint32_t v){
        ihdr[off]=(v>>24)&0xFF; ihdr[off+1]=(v>>16)&0xFF;
        ihdr[off+2]=(v>>8)&0xFF; ihdr[off+3]=v&0xFF;
    };
    pu(0,W); pu(4,H);
    ihdr[8]=8; ihdr[9]=2; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0;
    write_chunk(f,"IHDR",ihdr);
    write_chunk(f,"IDAT",zlib);
    write_chunk(f,"IEND",{});
    return true;
}

// ============================================================
// SSS - Dipole Diffusion Profile
// Jensen et al. 2001: A Practical Model for Subsurface Light Transport
// Approximated as Sum of Gaussians (Gaussian Scatter Profile)
// ============================================================

// Single Gaussian scatter kernel: G(r, sigma) = exp(-r^2 / (2*sigma^2)) / (2*pi*sigma^2)
inline float gaussian_kernel(float r, float sigma) {
    if(sigma < 1e-6f) return (r < 1e-4f) ? 1.0f : 0.0f;
    return expf(-(r*r) / (2.0f*sigma*sigma)) / (2.0f*3.14159265f*sigma*sigma);
}

// Sum-of-Gaussians scatter profile per color channel
// weights[i], sigmas[i] describe the mix
struct ScatterProfile {
    // 3 Gaussians per channel (R, G, B)
    float weightsR[3], sigmasR[3];
    float weightsG[3], sigmasG[3];
    float weightsB[3], sigmasB[3];

    // Evaluate diffusion profile at radius r
    Vec3 eval(float r) const {
        auto sog = [&](const float* w, const float* s) {
            float v = 0;
            for(int i=0;i<3;i++) v += w[i] * gaussian_kernel(r, s[i]);
            return v;
        };
        return Vec3(sog(weightsR, sigmasR), sog(weightsG, sigmasG), sog(weightsB, sigmasB));
    }
};

// Presets from skin rendering literature
ScatterProfile make_skin_profile() {
    ScatterProfile p;
    // R channel: wide spread (blood, deep scattering)
    p.weightsR[0]=0.233f; p.sigmasR[0]=0.0064f;
    p.weightsR[1]=0.100f; p.sigmasR[1]=0.0484f;
    p.weightsR[2]=0.118f; p.sigmasR[2]=0.187f;
    // G channel: medium
    p.weightsG[0]=0.113f; p.sigmasG[0]=0.0064f;
    p.weightsG[1]=0.358f; p.sigmasG[1]=0.0484f;
    p.weightsG[2]=0.078f; p.sigmasG[2]=0.187f;
    // B channel: narrow (surface)
    p.weightsB[0]=0.007f; p.sigmasB[0]=0.0064f;
    p.weightsB[1]=0.004f; p.sigmasB[1]=0.0484f;
    p.weightsB[2]=0.005f; p.sigmasB[2]=0.187f;
    return p;
}

ScatterProfile make_wax_profile() {
    ScatterProfile p;
    // Wax: more uniform, yellowish
    p.weightsR[0]=0.40f; p.sigmasR[0]=0.05f;
    p.weightsR[1]=0.30f; p.sigmasR[1]=0.15f;
    p.weightsR[2]=0.10f; p.sigmasR[2]=0.40f;

    p.weightsG[0]=0.35f; p.sigmasG[0]=0.05f;
    p.weightsG[1]=0.28f; p.sigmasG[1]=0.15f;
    p.weightsG[2]=0.08f; p.sigmasG[2]=0.40f;

    p.weightsB[0]=0.10f; p.sigmasB[0]=0.04f;
    p.weightsB[1]=0.08f; p.sigmasB[1]=0.12f;
    p.weightsB[2]=0.03f; p.sigmasB[2]=0.30f;
    return p;
}

ScatterProfile make_marble_profile() {
    ScatterProfile p;
    // Marble: blue-tinted uniform scattering
    p.weightsR[0]=0.25f; p.sigmasR[0]=0.03f;
    p.weightsR[1]=0.20f; p.sigmasR[1]=0.10f;
    p.weightsR[2]=0.05f; p.sigmasR[2]=0.30f;

    p.weightsG[0]=0.27f; p.sigmasG[0]=0.03f;
    p.weightsG[1]=0.22f; p.sigmasG[1]=0.10f;
    p.weightsG[2]=0.06f; p.sigmasG[2]=0.30f;

    p.weightsB[0]=0.38f; p.sigmasB[0]=0.03f;
    p.weightsB[1]=0.32f; p.sigmasB[1]=0.10f;
    p.weightsB[2]=0.09f; p.sigmasB[2]=0.30f;
    return p;
}

ScatterProfile make_milk_profile() {
    ScatterProfile p;
    // Milk: very white, wide scatter
    p.weightsR[0]=0.50f; p.sigmasR[0]=0.08f;
    p.weightsR[1]=0.30f; p.sigmasR[1]=0.25f;
    p.weightsR[2]=0.10f; p.sigmasR[2]=0.60f;

    p.weightsG[0]=0.52f; p.sigmasG[0]=0.07f;
    p.weightsG[1]=0.32f; p.sigmasG[1]=0.22f;
    p.weightsG[2]=0.11f; p.sigmasG[2]=0.55f;

    p.weightsB[0]=0.55f; p.sigmasB[0]=0.07f;
    p.weightsB[1]=0.35f; p.sigmasB[1]=0.20f;
    p.weightsB[2]=0.12f; p.sigmasB[2]=0.50f;
    return p;
}

// ============================================================
// Scene: sphere rendering with SSS
// ============================================================
struct Ray {
    Vec3 origin, dir;
};

struct Hit {
    float t;
    Vec3 pos, normal;
    bool valid = false;
};

struct Sphere {
    Vec3 center;
    float radius;
    Hit intersect(const Ray& ray) const {
        Vec3 oc = ray.origin - center;
        float a = ray.dir.dot(ray.dir);
        float b = 2.0f * oc.dot(ray.dir);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        Hit h;
        if(disc < 0) return h;
        float sqrtD = sqrtf(disc);
        float t = (-b - sqrtD) / (2*a);
        if(t < 0.001f) t = (-b + sqrtD) / (2*a);
        if(t < 0.001f) return h;
        h.valid = true;
        h.t = t;
        h.pos = ray.origin + ray.dir * t;
        h.normal = (h.pos - center).norm();
        return h;
    }
};

// ============================================================
// SSS rendering via texture-space diffusion approximation
// We use "wrapped lighting" + Gaussian blurred irradiance simulation
// (the full texture-space SSS is complex; we approximate with
//  analytical scatter profile convolution over local geometry)
// ============================================================

// Simplified SSS: for each surface point, sample nearby surface points
// within a disk, weight by scatter profile, accumulate irradiance.
// We simulate this analytically on a sphere using spherical geometry.

// Compute SSS color at hit point by integrating over hemisphere
// using the scatter profile as a function of arc-distance
Vec3 compute_sss(const Vec3& pos, const Vec3& /*normal*/, const Vec3& light_dir,
                 const Vec3& light_color, const ScatterProfile& profile,
                 const Sphere& sphere, int N_samples = 64) {
    // Sample points on the sphere surface, compute their distance to pos,
    // compute direct irradiance at those points, weight by scatter profile
    Vec3 accum(0,0,0);
    float weight_sum = 0;

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist01(0, 1);

    for(int i=0; i<N_samples; i++) {
        // Fibonacci sphere sampling for uniform distribution
        float phi = acosf(1 - 2*(i+0.5f)/N_samples);
        float theta = 2*3.14159265f * i * 0.6180339887f; // golden ratio

        Vec3 sp(sinf(phi)*cosf(theta), sinf(phi)*sinf(theta), cosf(phi));
        // Compute arc distance (geodesic) on sphere surface
        float dot_centers = (pos - sphere.center).norm().dot(sp);
        dot_centers = std::max(-1.f, std::min(1.f, dot_centers));
        float arc_angle = acosf(dot_centers);
        float r = arc_angle * sphere.radius; // arc length

        // Direct irradiance at sample point
        float ndotl = std::max(0.f, sp.dot(light_dir));
        Vec3 irr = light_color * ndotl;

        // Weight by scatter profile
        Vec3 w = profile.eval(r);
        accum += irr * w;
        weight_sum += 1.0f;
    }
    // Normalize: multiply by 4*pi*r^2 / N (hemisphere area element)
    float norm = (4.0f * 3.14159265f * sphere.radius * sphere.radius) / N_samples;
    return accum * norm;
}

// Standard Blinn-Phong specular
Vec3 blinn_phong_specular(const Vec3& normal, const Vec3& light_dir, const Vec3& view_dir,
                           const Vec3& spec_color, float shininess) {
    Vec3 half_vec = (light_dir + view_dir).norm();
    float ndoth = std::max(0.f, normal.dot(half_vec));
    return spec_color * powf(ndoth, shininess);
}

// ============================================================
// Render one sphere with SSS material
// ============================================================
struct Material {
    ScatterProfile profile;
    Vec3 specular_color;
    float shininess;
    Vec3 surface_albedo; // Base color tint
    std::string name;
};

void render_sphere(Framebuffer& fb, int ox, int oy, int sphere_px,
                   const Material& mat, const Vec3& light_dir, const Vec3& light_color,
                   const Vec3& ambient) {
    float R = sphere_px * 0.45f; // sphere radius in pixels
    Vec3 sphere_center_world(0, 0, 0);
    float world_radius = 1.0f;
    Sphere sphere{sphere_center_world, world_radius};

    Vec3 cam_pos(0, 0, 3.5f);

    for(int py = oy; py < oy + sphere_px; py++) {
        for(int px = ox; px < ox + sphere_px; px++) {
            // Map pixel to NDC
            float u = (px - ox - sphere_px*0.5f) / R;
            float v = (py - oy - sphere_px*0.5f) / R;

            // Ray from camera through pixel
            Vec3 ray_dir = Vec3(u * 0.4f, -v * 0.4f, -1.0f).norm();
            Ray ray{cam_pos, ray_dir};

            Hit hit = sphere.intersect(ray);
            if(!hit.valid) {
                // Background gradient
                float t = 0.5f * (ray_dir.y + 1.0f);
                Vec3 bg = Vec3(0.12f, 0.14f, 0.18f) * (1-t) + Vec3(0.18f, 0.20f, 0.28f) * t;
                fb.set(px, py, bg);
                continue;
            }

            Vec3 N = hit.normal;
            Vec3 V = (cam_pos - hit.pos).norm();

            // Direct diffuse (standard Lambertian for comparison baseline)
            float ndotl = std::max(0.f, N.dot(light_dir));
            Vec3 direct_diffuse = light_color * ndotl * mat.surface_albedo;

            // SSS contribution (replaces most of diffuse, simulates subsurface)
            Vec3 sss = compute_sss(hit.pos, N, light_dir, light_color,
                                    mat.profile, sphere, 128);
            sss = sss * mat.surface_albedo;

            // Mix: SSS replaces diffuse, keep a tiny bit of direct diffuse for surface detail
            Vec3 diffuse = direct_diffuse * 0.15f + sss * 0.85f;

            // Specular
            Vec3 spec = blinn_phong_specular(N, light_dir, V, mat.specular_color, mat.shininess);

            // Ambient
            Vec3 amb = ambient * mat.surface_albedo;

            Vec3 color = amb + diffuse + spec;
            fb.set(px, py, color);
        }
    }
}

// ============================================================
// Render label text (simple pixel font)
// ============================================================
// 5x7 bitmap font for uppercase + digits + space
static const uint8_t font5x7[][7] = {
    // A-Z (0-25), 0-9 (26-35), space (36), '-' (37), '.' (38), '(' (39), ')' (40)
    {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}, // A
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // B
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, // C
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // D
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, // E
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, // F
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, // G
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // H
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // I
    {0x01,0x01,0x01,0x01,0x01,0x11,0x0E}, // J
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, // K
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, // L
    {0x11,0x1B,0x15,0x11,0x11,0x11,0x11}, // M
    {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, // N
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // O
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, // P
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, // Q
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, // R
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, // S
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // T
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // U
    {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}, // V
    {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}, // W
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // X
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // Y
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, // Z
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // 0
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
    {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}, // 2
    {0x1F,0x02,0x04,0x06,0x01,0x11,0x0E}, // 3
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // 4
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // 5
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // 6
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // 7
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // 8
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, // 9
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x06,0x06}, // .
};

int char_index(char c) {
    if(c>='A'&&c<='Z') return c-'A';
    if(c>='a'&&c<='z') return c-'a';
    if(c>='0'&&c<='9') return 26 + c-'0';
    if(c==' ') return 36;
    if(c=='-') return 37;
    if(c=='.') return 38;
    return 36;
}

void draw_text(Framebuffer& fb, int x, int y, const std::string& text, const Vec3& color, int scale=1) {
    int cx = x;
    for(char c : text) {
        int idx = char_index(c);
        for(int row=0; row<7; row++) {
            uint8_t bits = font5x7[idx][row];
            for(int col=0; col<5; col++) {
                if(bits & (0x10 >> col)) {
                    for(int sy=0;sy<scale;sy++)
                        for(int sx=0;sx<scale;sx++)
                            fb.set(cx + col*scale + sx, y + row*scale + sy, color);
                }
            }
        }
        cx += (5+1)*scale;
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    const int W = 900, H = 500;
    Framebuffer fb(W, H);

    // Background: dark gradient
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
        float t = y / float(H);
        fb.set(x, y, Vec3(0.05f, 0.06f, 0.08f) * (1-t) + Vec3(0.10f, 0.11f, 0.15f) * t);
    }

    // Light setup: warm key light
    Vec3 light_dir = Vec3(0.6f, 0.8f, 0.5f).norm();
    Vec3 light_color(2.5f, 2.2f, 1.8f); // Warm light
    Vec3 ambient(0.04f, 0.04f, 0.06f);

    // Four materials: skin, wax, marble, milk
    std::vector<Material> materials = {
        {make_skin_profile(), Vec3(0.8f,0.6f,0.5f), 32.0f,
         Vec3(0.95f, 0.68f, 0.52f), "SKIN"},
        {make_wax_profile(),  Vec3(0.9f,0.9f,0.7f), 64.0f,
         Vec3(1.0f, 0.92f, 0.65f),  "WAX"},
        {make_marble_profile(),Vec3(0.9f,0.9f,1.0f), 96.0f,
         Vec3(0.85f, 0.88f, 0.95f), "MARBLE"},
        {make_milk_profile(),  Vec3(1.0f,1.0f,1.0f), 16.0f,
         Vec3(0.98f, 0.98f, 1.0f),  "MILK"},
    };

    // Layout: 4 spheres in a row
    int sphere_size = 200;
    int pad = 25;
    int total_w = 4 * sphere_size + 3 * pad;
    int start_x = (W - total_w) / 2;
    int start_y = 40;

    printf("Rendering SSS spheres...\n");
    for(int i=0; i<4; i++) {
        int ox = start_x + i * (sphere_size + pad);
        int oy = start_y;
        printf("  Sphere %d: %s\n", i+1, materials[i].name.c_str());
        render_sphere(fb, ox, oy, sphere_size, materials[i],
                      light_dir, light_color, ambient);
    }

    // Title
    draw_text(fb, W/2-250, H-55, "SUBSURFACE SCATTERING - SSS RENDERER", Vec3(0.9f,0.85f,0.7f), 1);
    draw_text(fb, W/2-200, H-40, "DIPOLE DIFFUSION + SUM OF GAUSSIANS", Vec3(0.6f,0.65f,0.7f), 1);

    // Labels under each sphere
    for(int i=0;i<4;i++) {
        int ox = start_x + i*(sphere_size+pad);
        int lx = ox + sphere_size/2 - 18;
        draw_text(fb, lx, start_y + sphere_size + 10, materials[i].name, Vec3(0.85f,0.85f,0.85f), 1);
    }

    printf("Saving output...\n");
    bool ok = save_png("sss_output.png", W, H, fb.buf);
    printf(ok ? "Saved sss_output.png\n" : "Failed to save!\n");
    return ok ? 0 : 1;
}
