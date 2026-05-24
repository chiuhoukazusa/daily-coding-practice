/**
 * Subsurface Scattering (SSS) Renderer
 * 
 * 技术要点：
 * - Jensen SSS 偶极子模型 (Dipole Approximation)
 * - 漫射轮廓函数 Rd(r)
 * - 半透明阴影 (Translucent Shadows)
 * - 皮肤/大理石/蜡/牛奶等材质演示
 * - 软光栅化管线（CPU 实现）
 * 
 * 场景：人脸球（皮肤 SSS）、大理石球、蜡烛柱体
 * 
 * 参考：
 * - H. W. Jensen et al. "A Practical Model for Subsurface Light Transport" (SIGGRAPH 2001)
 * - Real-Time Skin Rendering (d'Eon & Luebke, GPU Gems 3)
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <string>
#include <sstream>
#include <iomanip>

// ==================== Math ====================

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o)const{return {x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o)const{return {x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)const{return {x*t,y*t,z*t};}
    Vec3 operator*(const Vec3& o)const{return {x*o.x,y*o.y,z*o.z};}
    Vec3 operator/(float t)const{return {x/t,y/t,z/t};}
    Vec3 operator-()const{return {-x,-y,-z};}
    Vec3& operator+=(const Vec3& o){x+=o.x;y+=o.y;z+=o.z;return *this;}
    float dot(const Vec3& o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3& o)const{return {y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-8f?Vec3{x/l,y/l,z/l}:Vec3{0,0,0};}
    Vec3 clamp(float lo,float hi)const{return {std::max(lo,std::min(hi,x)),
                                                std::max(lo,std::min(hi,y)),
                                                std::max(lo,std::min(hi,z))};}
};

inline Vec3 operator*(float t,const Vec3& v){return v*t;}
inline float clamp01(float v){return std::max(0.f,std::min(1.f,v));}
inline float lerp(float a,float b,float t){return a+(b-a)*t;}
inline Vec3 lerp3(Vec3 a,Vec3 b,float t){return a+(b-a)*t;}

struct Ray { Vec3 o,d; };

// ==================== SSS Materials ====================

struct SSSMaterial {
    std::string name;
    // Dipole parameters (per-channel: RGB)
    Vec3 sigma_a;     // absorption coefficient
    Vec3 sigma_s;     // scattering coefficient
    Vec3 sigma_sp;    // reduced scattering = sigma_s * (1 - g)
    float eta;        // index of refraction
    float g;          // scattering anisotropy
    Vec3 albedo;      // surface albedo (for direct lighting blend)
    float sssStrength;// SSS blend weight
    Vec3 emissive;    // for candle
};

// Jensen SSS dipole model
// Rd(r) = alpha'/(4*pi) * (z_r*(sigma_tr_r+1/d_r)*exp(-sigma_tr*d_r)/d_r^3
//                        + z_v*(sigma_tr_v+1/d_v)*exp(-sigma_tr*d_v)/d_v^3)
struct DipoleProfile {
    Vec3 sigma_a, sigma_sp;
    float eta;

    void init(Vec3 sa, Vec3 ss_prime, float n) {
        sigma_a = sa; sigma_sp = ss_prime; eta = n;
    }

    Vec3 computeRd(float r) const {
        // sigma_t' = sigma_a + sigma_sp
        // alpha' = sigma_sp / sigma_t'
        Vec3 sigma_t_prime = sigma_a + sigma_sp;
        Vec3 alpha_prime;
        alpha_prime.x = sigma_sp.x / (sigma_t_prime.x + 1e-6f);
        alpha_prime.y = sigma_sp.y / (sigma_t_prime.y + 1e-6f);
        alpha_prime.z = sigma_sp.z / (sigma_t_prime.z + 1e-6f);

        // sigma_tr = sqrt(3 * sigma_a * sigma_t')
        Vec3 sigma_tr;
        sigma_tr.x = std::sqrt(3.f * sigma_a.x * sigma_t_prime.x);
        sigma_tr.y = std::sqrt(3.f * sigma_a.y * sigma_t_prime.y);
        sigma_tr.z = std::sqrt(3.f * sigma_a.z * sigma_t_prime.z);

        // z_r = 1 / sigma_t'
        Vec3 z_r;
        z_r.x = 1.f / (sigma_t_prime.x + 1e-6f);
        z_r.y = 1.f / (sigma_t_prime.y + 1e-6f);
        z_r.z = 1.f / (sigma_t_prime.z + 1e-6f);

        // Fresnel term approx: F_dr = -1.440/eta^2 + 0.710/eta + 0.668 + 0.0636*eta
        float Fdr = -1.44f/(eta*eta) + 0.71f/eta + 0.668f + 0.0636f*eta;
        float A = (1.f + Fdr) / (1.f - Fdr);

        // z_v = z_r + 4*A*D  where D = 1/(3*sigma_t')
        Vec3 D;
        D.x = 1.f/(3.f*sigma_t_prime.x + 1e-6f);
        D.y = 1.f/(3.f*sigma_t_prime.y + 1e-6f);
        D.z = 1.f/(3.f*sigma_t_prime.z + 1e-6f);

        Vec3 z_v;
        z_v.x = z_r.x + 4.f*A*D.x;
        z_v.y = z_r.y + 4.f*A*D.y;
        z_v.z = z_r.z + 4.f*A*D.z;

        float r2 = r*r;

        // d_r = sqrt(r^2 + z_r^2),  d_v = sqrt(r^2 + z_v^2)
        auto evalChannel = [&](float zr, float zv, float str, float alp) -> float {
            float dr = std::sqrt(r2 + zr*zr);
            float dv = std::sqrt(r2 + zv*zv);
            float term_r = zr * (str * dr + 1.f) * std::exp(-str * dr) / (dr*dr*dr + 1e-12f);
            float term_v = zv * (str * dv + 1.f) * std::exp(-str * dv) / (dv*dv*dv + 1e-12f);
            float Rd = alp / (4.f * 3.14159265f) * (term_r + term_v);
            return std::max(0.f, Rd);
        };

        return Vec3{
            evalChannel(z_r.x, z_v.x, sigma_tr.x, alpha_prime.x),
            evalChannel(z_r.y, z_v.y, sigma_tr.y, alpha_prime.y),
            evalChannel(z_r.z, z_v.z, sigma_tr.z, alpha_prime.z)
        };
    }
};

// Pre-bake diffusion profile into 1D LUT [0..1] -> radius [0..maxR]
struct DiffusionLUT {
    static constexpr int N = 256;
    static constexpr float MAX_R = 8.0f;
    Vec3 lut[N];
    DipoleProfile profile;

    void build(const DipoleProfile& p) {
        profile = p;
        for(int i=0;i<N;i++){
            float r = (float)i/(N-1) * MAX_R + 0.001f;
            lut[i] = p.computeRd(r);
        }
    }

    Vec3 sample(float r) const {
        float t = clamp01(r / MAX_R) * (N-1);
        int lo = (int)t, hi = std::min(lo+1, N-1);
        float f = t - lo;
        return lerp3(lut[lo], lut[hi], f);
    }
};

// ==================== Material Presets ====================

SSSMaterial makeSkinMaterial() {
    SSSMaterial m;
    m.name = "Skin";
    // Jensen 2001 human skin coefficients (scaled for our scene units)
    m.sigma_a  = Vec3{0.0032f, 0.017f, 0.048f};   // RGB absorption (skin)
    m.sigma_sp = Vec3{0.74f,   0.88f,  1.01f};     // reduced scattering
    m.eta = 1.3f;
    m.g = 0.0f; // already reduced
    m.albedo = Vec3{0.9f, 0.7f, 0.6f};             // warm skin tone
    m.sssStrength = 0.85f;
    m.emissive = Vec3{0,0,0};
    return m;
}

SSSMaterial makeMarbleMaterial() {
    SSSMaterial m;
    m.name = "Marble";
    m.sigma_a  = Vec3{0.002f, 0.002f, 0.002f};    // very low absorption
    m.sigma_sp = Vec3{2.19f,  2.62f,  3.00f};     // strong scattering
    m.eta = 1.5f;
    m.g = 0.0f;
    m.albedo = Vec3{0.95f, 0.93f, 0.90f};          // off-white
    m.sssStrength = 0.75f;
    m.emissive = Vec3{0,0,0};
    return m;
}

SSSMaterial makeWaxMaterial() {
    SSSMaterial m;
    m.name = "Wax";
    m.sigma_a  = Vec3{0.008f, 0.012f, 0.020f};
    m.sigma_sp = Vec3{0.5f,   0.45f,  0.4f};
    m.eta = 1.4f;
    m.g = 0.0f;
    m.albedo = Vec3{0.95f, 0.88f, 0.70f};          // yellowish wax
    m.sssStrength = 0.9f;
    m.emissive = Vec3{0.8f, 0.4f, 0.05f};          // candle glow
    return m;
}

SSSMaterial makeMilkMaterial() {
    SSSMaterial m;
    m.name = "Milk";
    m.sigma_a  = Vec3{0.0015f, 0.0015f, 0.0015f};
    m.sigma_sp = Vec3{4.5f,    5.5f,    7.0f};     // highly scattering
    m.eta = 1.35f;
    m.g = 0.0f;
    m.albedo = Vec3{0.98f, 0.97f, 0.96f};
    m.sssStrength = 0.95f;
    m.emissive = Vec3{0,0,0};
    return m;
}

// ==================== Geometry ====================

struct HitRecord {
    float t;
    Vec3 pos, normal;
    int matId;
    bool hit;
};

struct Sphere {
    Vec3 center;
    float radius;
    int matId;

    HitRecord intersect(const Ray& ray) const {
        Vec3 oc = ray.o - center;
        float a = ray.d.dot(ray.d);
        float b = 2.f * oc.dot(ray.d);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if(disc < 0) return {0,{},{},0,false};
        float sq = std::sqrt(disc);
        float t = (-b - sq)/(2*a);
        if(t < 0.001f) t = (-b + sq)/(2*a);
        if(t < 0.001f) return {0,{},{},0,false};
        Vec3 pos = ray.o + ray.d * t;
        Vec3 nrm = (pos - center).norm();
        return {t, pos, nrm, matId, true};
    }
};

struct Cylinder {
    Vec3 base;
    float radius, height;
    int matId;

    HitRecord intersect(const Ray& ray) const {
        // Infinite cylinder along Y axis
        float ax = ray.d.x, ay = ray.d.z;
        float bx = ray.o.x - base.x, bz = ray.o.z - base.z;
        float A = ax*ax + ay*ay;
        float B = 2.f*(ax*bx + ay*bz);
        float C = bx*bx + bz*bz - radius*radius;
        float disc = B*B - 4*A*C;
        if(disc < 0 || A < 1e-6f) return {0,{},{},0,false};
        float sq = std::sqrt(disc);
        float t = (-B - sq)/(2*A);
        if(t < 0.001f) t = (-B + sq)/(2*A);
        if(t < 0.001f) return {0,{},{},0,false};
        Vec3 pos = ray.o + ray.d * t;
        float y = pos.y;
        if(y < base.y || y > base.y + height) return {0,{},{},0,false};
        Vec3 nrm = Vec3{pos.x-base.x, 0, pos.z-base.z}.norm();
        return {t, pos, nrm, matId, true};
    }
};

struct Plane {
    Vec3 point, normal;
    int matId;

    HitRecord intersect(const Ray& ray) const {
        float denom = normal.dot(ray.d);
        if(std::abs(denom) < 1e-6f) return {0,{},{},0,false};
        float t = (point - ray.o).dot(normal) / denom;
        if(t < 0.001f) return {0,{},{},0,false};
        Vec3 pos = ray.o + ray.d * t;
        return {t, pos, normal, matId, true};
    }
};

// ==================== Light ====================

struct PointLight {
    Vec3 pos, color;
    float intensity;
};

// ==================== Scene ====================

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Cylinder> cylinders;
    std::vector<Plane> planes;
    std::vector<SSSMaterial> materials;
    std::vector<DiffusionLUT> sssLUTs;
    std::vector<PointLight> lights;

    void buildLUTs() {
        sssLUTs.resize(materials.size());
        for(int i=0;i<(int)materials.size();i++){
            const auto& m = materials[i];
            DipoleProfile dp;
            dp.init(m.sigma_a, m.sigma_sp, m.eta);
            sssLUTs[i].build(dp);
        }
    }

    HitRecord intersectAll(const Ray& ray) const {
        HitRecord best; best.hit=false; best.t=1e9f;
        for(auto& s:spheres){auto h=s.intersect(ray);if(h.hit&&h.t<best.t)best=h;}
        for(auto& c:cylinders){auto h=c.intersect(ray);if(h.hit&&h.t<best.t)best=h;}
        for(auto& p:planes){auto h=p.intersect(ray);if(h.hit&&h.t<best.t)best=h;}
        return best;
    }

    bool shadow(const Vec3& pos, const Vec3& lpos) const {
        Vec3 dir = (lpos - pos);
        float dist = dir.len();
        Ray r{pos + dir.norm()*0.005f, dir.norm()};
        auto h = intersectAll(r);
        return h.hit && h.t < dist - 0.01f;
    }
};

// ==================== SSS Shading ====================

// Wrap lighting: approximate translucent shadow / back-scatter
// Using the "wrapped diffuse" approximation for back-lit SSS contribution
Vec3 computeSSS(const Scene& scene, const HitRecord& hit,
                const Vec3& /*viewDir*/, int /*numLightSamples*/) {
    const auto& mat = scene.materials[hit.matId];
    const auto& lut = scene.sssLUTs[hit.matId];
    Vec3 totalSSS{0,0,0};

    for(const auto& light : scene.lights) {
        Vec3 toLight = light.pos - hit.pos;
        float dist = toLight.len();
        Vec3 L = toLight.norm();

        // Attenuation
        float atten = light.intensity / (dist*dist + 1.f);

        // --- Direct (surface) diffuse ---
        float NdotL = std::max(0.f, hit.normal.dot(L));
        Vec3 directDiff = mat.albedo * NdotL * light.color * atten;

        // --- SSS approximation ---
        // 1. Wrap term: back-lit scatter (light wraps around object)
        float wrapW = 0.3f; // wrap width
        float wrapTerm = clamp01((hit.normal.dot(L) + wrapW) / (1.f + wrapW));
        wrapTerm = wrapTerm * wrapTerm; // soften

        // 2. Dipole Rd at a few virtual distances to simulate thick SSS
        // Simplified: use average "exit radius" based on mean free path
        float mfp = 1.f / (mat.sigma_a.x + mat.sigma_sp.x + 1e-4f);
        float r_sss = mfp * 0.5f;

        Vec3 Rd = lut.sample(r_sss);

        // 3. Translucent shadow: light seen through material
        // Approximate with negative NdotL (back-facing irradiance)
        float backIrr = clamp01(-hit.normal.dot(L) * 0.5f + 0.5f);
        Vec3 transColor = mat.albedo * Rd * backIrr * light.color * atten * 3.0f;

        // 4. Shadow softening for SSS materials (penumbra effect)
        bool inShadow = scene.shadow(hit.pos, light.pos);
        float shadowFactor = inShadow ? 0.15f : 1.f; // not fully dark due to SSS

        Vec3 directContrib = directDiff * shadowFactor;
        Vec3 sssContrib = transColor * mat.sssStrength;

        // Blend direct + SSS
        totalSSS += directContrib + sssContrib;
    }

    return totalSSS;
}

Vec3 computeSpecular(const HitRecord& hit, const Vec3& viewDir,
                     const PointLight& light, float roughness) {
    Vec3 L = (light.pos - hit.pos).norm();
    Vec3 H = (L + (-viewDir).norm()).norm();
    float NdotH = std::max(0.f, hit.normal.dot(H));
    float spec = std::pow(NdotH, 1.f/(roughness*roughness + 0.01f));
    float dist = (light.pos - hit.pos).len();
    float atten = light.intensity / (dist*dist + 1.f);
    return light.color * (spec * atten * 0.3f);
}

// ==================== Floor plane special shading ====================

Vec3 shadeFloor(const Vec3& pos) {
    // Checkerboard
    float cx = std::floor(pos.x * 0.5f);
    float cz = std::floor(pos.z * 0.5f);
    float check = std::fmod(std::abs(cx + cz), 2.f);
    return check < 1.f ? Vec3{0.85f,0.85f,0.85f} : Vec3{0.35f,0.35f,0.35f};
}

// ==================== Renderer ====================

struct Camera {
    Vec3 pos, fwd, up, right;
    float fov, aspect;

    Camera(Vec3 p, Vec3 target, float fovDeg, float asp) {
        pos = p;
        fwd = (target - p).norm();
        right = fwd.cross(Vec3{0,1,0}).norm();
        up = right.cross(fwd).norm();
        fov = fovDeg * 3.14159265f / 180.f;
        aspect = asp;
    }

    Ray getRay(float u, float v) const {
        float tanHalfFov = std::tan(fov * 0.5f);
        float px = (2*u - 1) * aspect * tanHalfFov;
        float py = (1 - 2*v) * tanHalfFov;
        Vec3 dir = (fwd + right*px + up*py).norm();
        return {pos, dir};
    }
};

Vec3 trace(const Scene& scene, const Ray& ray, int depth) {
    if(depth <= 0) return Vec3{0,0,0};

    auto hit = scene.intersectAll(ray);
    if(!hit.hit) {
        // Sky gradient
        float t = clamp01(ray.d.y * 0.5f + 0.5f);
        return lerp3(Vec3{0.6f,0.75f,0.9f}, Vec3{0.15f,0.2f,0.35f}, t);
    }

    const auto& mat = scene.materials[hit.matId];

    // Floor plane
    if(hit.matId < 0) {
        return shadeFloor(hit.pos);
    }

    Vec3 viewDir = ray.d;

    // Ambient
    Vec3 ambient = mat.albedo * 0.04f;

    // SSS + diffuse
    Vec3 sssColor = computeSSS(scene, hit, viewDir, 1);

    // Specular highlights for all lights
    Vec3 specColor{0,0,0};
    for(const auto& l : scene.lights) {
        specColor += computeSpecular(hit, viewDir, l, 0.4f);
    }

    // Emissive (candle glow)
    Vec3 emissive = mat.emissive;

    Vec3 result = ambient + sssColor + specColor + emissive;

    // Fresnel edge glow for SSS materials (chromatic rim)
    float fresnel = 1.f - std::max(0.f, (-viewDir).dot(hit.normal));
    fresnel = std::pow(fresnel, 3.f);
    Vec3 rimColor = mat.albedo * 0.4f * fresnel;
    result += rimColor;

    return result.clamp(0.f, 1.f);
}

// ==================== PPM Writer ====================

struct Image {
    int W, H;
    std::vector<Vec3> pixels;
    Image(int w,int h):W(w),H(h),pixels(w*h,{0,0,0}){}
    void set(int x,int y,Vec3 c){pixels[y*W+x]=c;}
    Vec3 get(int x,int y)const{return pixels[y*W+x];}
};

void writePPM(const Image& img, const std::string& filename) {
    FILE* f = fopen(filename.c_str(), "wb");
    fprintf(f, "P6\n%d %d\n255\n", img.W, img.H);
    for(int y=0;y<img.H;y++)
        for(int x=0;x<img.W;x++){
            Vec3 c = img.get(x,y);
            uint8_t r=(uint8_t)(c.x*255.99f);
            uint8_t g=(uint8_t)(c.y*255.99f);
            uint8_t b=(uint8_t)(c.z*255.99f);
            fwrite(&r,1,1,f); fwrite(&g,1,1,f); fwrite(&b,1,1,f);
        }
    fclose(f);
}

// ==================== Gamma / Tone Map ====================

Vec3 toneMapACES(Vec3 x) {
    // Narkowicz ACES
    const float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;
    Vec3 r;
    r.x = (x.x*(a*x.x+b))/(x.x*(c*x.x+d)+e);
    r.y = (x.y*(a*x.y+b))/(x.y*(c*x.y+d)+e);
    r.z = (x.z*(a*x.z+b))/(x.z*(c*x.z+d)+e);
    return r.clamp(0.f,1.f);
}

Vec3 gammaCorrect(Vec3 c, float gamma=2.2f) {
    float ig = 1.f/gamma;
    return Vec3{std::pow(c.x,ig), std::pow(c.y,ig), std::pow(c.z,ig)};
}

// ==================== Main ====================

int main() {
    const int W = 800, H = 600;
    const int AA = 3; // 3x3 MSAA
    Image img(W, H);

    // ---- Materials ----
    Scene scene;
    scene.materials.push_back(makeSkinMaterial());   // 0: skin
    scene.materials.push_back(makeMarbleMaterial()); // 1: marble
    scene.materials.push_back(makeWaxMaterial());    // 2: wax/candle
    scene.materials.push_back(makeMilkMaterial());   // 3: milk

    // Floor (special handling - use matId -1 flag via a large sphere trick)
    SSSMaterial floorMat; floorMat.name="Floor";
    floorMat.sigma_a={}; floorMat.sigma_sp={};
    floorMat.albedo=Vec3{1,1,1}; floorMat.sssStrength=0; floorMat.eta=1.f;
    floorMat.emissive={}; floorMat.g=0;
    scene.materials.push_back(floorMat);             // 4: floor (checker)

    scene.buildLUTs();

    // ---- Geometry ----
    // Skin sphere (left)
    scene.spheres.push_back({Vec3{-2.5f, 0.8f, 0.f}, 0.9f, 0}); // skin
    // Marble sphere (center)
    scene.spheres.push_back({Vec3{0.f, 0.8f, 0.f}, 0.9f, 1});   // marble
    // Milk sphere (right)
    scene.spheres.push_back({Vec3{2.5f, 0.8f, 0.f}, 0.9f, 3});  // milk
    // Candle (wax cylinder, back-center)
    scene.cylinders.push_back({Vec3{0.f, 0.f, -2.f}, 0.3f, 2.2f, 2}); // wax cylinder

    // Small wax sphere as candle top
    scene.spheres.push_back({Vec3{0.f, 2.2f, -2.f}, 0.32f, 2}); // wax cap

    // Floor plane
    scene.planes.push_back({Vec3{0,-0.1f,0}, Vec3{0,1,0}, 4});

    // ---- Lights ----
    // Main warm key light (slightly above-left)
    scene.lights.push_back({Vec3{-4.f, 6.f, 3.f}, Vec3{1.0f,0.92f,0.78f}, 20.f});
    // Cool fill light (right)
    scene.lights.push_back({Vec3{4.f, 3.f, 4.f}, Vec3{0.5f,0.65f,1.0f}, 8.f});
    // Candle glow point light (warm orange)
    scene.lights.push_back({Vec3{0.f, 2.6f, -2.f}, Vec3{1.0f,0.55f,0.1f}, 3.f});
    // Back light (rim)
    scene.lights.push_back({Vec3{0.f, 4.f, -8.f}, Vec3{0.7f,0.8f,1.0f}, 10.f});

    // ---- Camera ----
    Camera cam(Vec3{0.f, 2.5f, 7.5f}, Vec3{0.f, 0.5f, 0.f}, 50.f, (float)W/H);

    // ---- Render ----
    float invAA = 1.f / (AA * AA);
    for(int y=0;y<H;y++) {
        for(int x=0;x<W;x++) {
            Vec3 acc{0,0,0};
            for(int sy=0;sy<AA;sy++) {
                for(int sx=0;sx<AA;sx++) {
                    float u = (x + (sx + 0.5f)/AA) / W;
                    float v = (y + (sy + 0.5f)/AA) / H;
                    Ray r = cam.getRay(u, v);
                    Vec3 col = trace(scene, r, 2);
                    // Tone mapping + gamma
                    col = toneMapACES(col * 0.7f);
                    col = gammaCorrect(col);
                    acc += col;
                }
            }
            img.set(x, y, (acc * invAA).clamp(0.f, 1.f));
        }
    }

    // ---- Save ----
    writePPM(img, "sss_output.ppm");
    printf("Saved sss_output.ppm (%dx%d)\n", W, H);

    // Convert to PNG using ImageMagick
    int ret = system("convert sss_output.ppm sss_renderer_output.png 2>/dev/null");
    if(ret == 0) {
        printf("Converted to sss_renderer_output.png\n");
        // Pixel validation
        ret = system("python3 -c \"\n"
            "from PIL import Image\n"
            "import numpy as np, sys\n"
            "img = Image.open('sss_renderer_output.png')\n"
            "pixels = np.array(img).astype(float)\n"
            "mean = pixels.mean()\n"
            "std = pixels.std()\n"
            "print(f'Pixel mean: {mean:.1f}  std: {std:.1f}')\n"
            "if mean < 5: print('FAIL: too dark'); sys.exit(1)\n"
            "if mean > 250: print('FAIL: too bright'); sys.exit(1)\n"
            "if std < 5: print('FAIL: no variation'); sys.exit(1)\n"
            "print('VALIDATION OK')\n"
            "\" 2>&1");
    }

    return 0;
}
