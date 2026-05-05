/*
 * Voxel Cone Tracing (VCT) - Approximate Global Illumination
 * 
 * 技术要点：
 * 1. 场景体素化 (Scene Voxelization) - 将几何体素化到 3D 网格
 * 2. Mipmap 层级体素 - 存储不同分辨率的体素颜色/不透明度
 * 3. 锥形追踪 (Cone Tracing) - 从表面点发射锥形采样间接光照
 * 4. 漫反射 + 镜面反射 GI
 * 5. 直接光照 + 间接光照合成
 */

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <array>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>
#include <functional>

// ============================================================
// Math Types
// ============================================================

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float v) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float t) { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 1e-10f ? *this / l : Vec3(0); }
    Vec3 clamp(float lo, float hi) const {
        return {std::clamp(x,lo,hi), std::clamp(y,lo,hi), std::clamp(z,lo,hi)};
    }
    float maxComp() const { return std::max({x,y,z}); }
    Vec3 abs() const { return {std::abs(x), std::abs(y), std::abs(z)}; }
};

inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a * (1.0f - t) + b * t; }
inline Vec3 reflect(const Vec3& v, const Vec3& n) { return v - n * (2.0f * n.dot(v)); }

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(0) {}
    Vec4(float v) : x(v), y(v), z(v), w(v) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(const Vec3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    Vec3 xyz() const { return {x, y, z}; }
    Vec4 operator+(const Vec4& o) const { return {x+o.x, y+o.y, z+o.z, w+o.w}; }
    Vec4& operator+=(const Vec4& o) { x+=o.x; y+=o.y; z+=o.z; w+=o.w; return *this; }
    Vec4 operator*(float t) const { return {x*t, y*t, z*t, w*t}; }
};

// ============================================================
// 3D Voxel Grid with Mipmaps
// ============================================================

static const int VOXEL_DIM = 64;    // 64^3 voxels
static const int MIP_LEVELS = 6;    // log2(64) = 6

struct VoxelGrid {
    // Each level: rgba (color + opacity)
    // Level 0 = 64^3, Level 1 = 32^3, ...Level 5 = 2^3
    std::vector<std::vector<Vec4>> levels;  
    std::vector<int> dims;
    
    VoxelGrid() {
        int dim = VOXEL_DIM;
        for (int i = 0; i < MIP_LEVELS; i++) {
            dims.push_back(dim);
            levels.push_back(std::vector<Vec4>(dim * dim * dim, Vec4(0)));
            dim /= 2;
        }
    }
    
    int idx(int level, int x, int y, int z) const {
        int d = dims[level];
        x = std::clamp(x, 0, d-1);
        y = std::clamp(y, 0, d-1);
        z = std::clamp(z, 0, d-1);
        return z * d * d + y * d + x;
    }
    
    Vec4 get(int level, int x, int y, int z) const {
        return levels[level][idx(level, x, y, z)];
    }
    
    void set(int level, int x, int y, int z, const Vec4& v) {
        levels[level][idx(level, x, y, z)] = v;
    }
    
    void accumulate(int level, int x, int y, int z, const Vec4& v) {
        levels[level][idx(level, x, y, z)] += v;
    }
    
    // Trilinear interpolation at float coordinates in [0,1]^3
    Vec4 sample(int level, float u, float v, float w) const {
        int d = dims[level];
        float fx = u * d - 0.5f;
        float fy = v * d - 0.5f;
        float fz = w * d - 0.5f;
        int ix = (int)std::floor(fx), iy = (int)std::floor(fy), iz = (int)std::floor(fz);
        float tx = fx - ix, ty = fy - iy, tz = fz - iz;
        
        auto g = [&](int dx, int dy, int dz) -> Vec4 {
            return get(level, ix+dx, iy+dy, iz+dz);
        };
        
        // trilinear
        auto lerp4 = [](const Vec4& a, const Vec4& b, float t) {
            return a * (1-t) + b * t;
        };
        Vec4 c00 = lerp4(g(0,0,0), g(1,0,0), tx);
        Vec4 c10 = lerp4(g(0,1,0), g(1,1,0), tx);
        Vec4 c01 = lerp4(g(0,0,1), g(1,0,1), tx);
        Vec4 c11 = lerp4(g(0,1,1), g(1,1,1), tx);
        Vec4 c0  = lerp4(c00, c10, ty);
        Vec4 c1  = lerp4(c01, c11, ty);
        return lerp4(c0, c1, tz);
    }
    
    // Sample with fractional mip level (trilinear + mip interpolation)
    Vec4 sampleMip(float u, float v, float w, float mipLevel) const {
        int mipLo = (int)std::floor(mipLevel);
        int mipHi = mipLo + 1;
        float t = mipLevel - mipLo;
        
        mipLo = std::clamp(mipLo, 0, MIP_LEVELS - 1);
        mipHi = std::clamp(mipHi, 0, MIP_LEVELS - 1);
        
        Vec4 lo = sample(mipLo, u, v, w);
        Vec4 hi = sample(mipHi, u, v, w);
        return lo * (1-t) + hi * t;
    }
    
    // Build mipmaps by averaging
    void buildMipmaps() {
        for (int level = 1; level < MIP_LEVELS; level++) {
            int d = dims[level];
            for (int z = 0; z < d; z++) {
                for (int y = 0; y < d; y++) {
                    for (int x = 0; x < d; x++) {
                        // Average 2x2x2 from previous level
                        Vec4 sum(0);
                        float cnt = 0;
                        for (int dz = 0; dz < 2; dz++)
                        for (int dy = 0; dy < 2; dy++)
                        for (int dx = 0; dx < 2; dx++) {
                            Vec4 v = get(level-1, x*2+dx, y*2+dy, z*2+dz);
                            // Opacity-weighted average for color
                            sum += v;
                            cnt++;
                        }
                        if (cnt > 0) set(level, x, y, z, sum * (1.0f / cnt));
                    }
                }
            }
        }
    }
};

// ============================================================
// Scene Geometry (Cornell Box + Spheres)
// ============================================================

struct Material {
    Vec3 albedo;
    float roughness;
    float metallic;
    Vec3 emission;
    bool isLight;
    
    Material() : albedo(0.8f), roughness(1.0f), metallic(0.0f), emission(0), isLight(false) {}
};

struct HitRecord {
    float t;
    Vec3 point;
    Vec3 normal;
    Material mat;
    bool hit;
    HitRecord() : t(1e30f), hit(false) {}
};

// Axis-aligned box
struct Box {
    Vec3 minP, maxP;
    Material mat;
    bool isFlip; // flip normals (for room walls)
    
    bool intersect(const Vec3& ro, const Vec3& rd, HitRecord& rec) const {
        Vec3 invD;
        invD.x = 1.0f / (std::abs(rd.x) > 1e-9f ? rd.x : 1e-9f);
        invD.y = 1.0f / (std::abs(rd.y) > 1e-9f ? rd.y : 1e-9f);
        invD.z = 1.0f / (std::abs(rd.z) > 1e-9f ? rd.z : 1e-9f);
        
        Vec3 t0 = (minP - ro) * invD;
        Vec3 t1 = (maxP - ro) * invD;
        
        float tmin = std::max({std::min(t0.x, t1.x), std::min(t0.y, t1.y), std::min(t0.z, t1.z)});
        float tmax = std::min({std::max(t0.x, t1.x), std::max(t0.y, t1.y), std::max(t0.z, t1.z)});
        
        if (tmax < tmin || tmax < 1e-4f) return false;
        
        float t = (tmin > 1e-4f) ? tmin : tmax;
        if (t > rec.t) return false;
        
        rec.t = t;
        rec.hit = true;
        rec.point = ro + rd * t;
        rec.mat = mat;
        
        // Determine face normal
        Vec3 center = (minP + maxP) * 0.5f;
        Vec3 d = (rec.point - center);
        Vec3 size = (maxP - minP) * 0.5f;
        float bias = 1.0001f;
        Vec3 n(0);
        if (std::abs(d.x / size.x) > std::abs(d.y / size.y) && std::abs(d.x / size.x) > std::abs(d.z / size.z))
            n.x = (d.x > 0) ? 1.0f : -1.0f;
        else if (std::abs(d.y / size.y) > std::abs(d.z / size.z))
            n.y = (d.y > 0) ? 1.0f : -1.0f;
        else
            n.z = (d.z > 0) ? 1.0f : -1.0f;
        
        (void)bias;
        rec.normal = isFlip ? n * -1.0f : n;
        return true;
    }
};

struct Sphere {
    Vec3 center;
    float radius;
    Material mat;
    
    bool intersect(const Vec3& ro, const Vec3& rd, HitRecord& rec) const {
        Vec3 oc = ro - center;
        float b = oc.dot(rd);
        float c = oc.dot(oc) - radius * radius;
        float disc = b * b - c;
        if (disc < 0) return false;
        float sq = std::sqrt(disc);
        float t = -b - sq;
        if (t < 1e-4f) t = -b + sq;
        if (t < 1e-4f || t > rec.t) return false;
        rec.t = t;
        rec.hit = true;
        rec.point = ro + rd * t;
        rec.normal = (rec.point - center) / radius;
        rec.mat = mat;
        return true;
    }
};

struct Scene {
    std::vector<Box> boxes;
    std::vector<Sphere> spheres;
    
    // Area light
    Vec3 lightPos;
    Vec3 lightSize;  // half extents
    Vec3 lightColor;
    
    HitRecord intersect(const Vec3& ro, const Vec3& rd) const {
        HitRecord best;
        for (auto& b : boxes) b.intersect(ro, rd, best);
        for (auto& s : spheres) s.intersect(ro, rd, best);
        return best;
    }
    
    // Build Cornell Box style scene
    void build() {
        float S = 1.0f;  // half size of room
        
        // Room walls (flip normals - inside faces)
        // Floor (white)
        {
            Box b;
            b.minP = Vec3(-S, -S, -S);
            b.maxP = Vec3(S, -S + 0.02f, S);
            b.mat.albedo = Vec3(0.8f, 0.8f, 0.8f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Ceiling (white)
        {
            Box b;
            b.minP = Vec3(-S, S - 0.02f, -S);
            b.maxP = Vec3(S, S, S);
            b.mat.albedo = Vec3(0.8f, 0.8f, 0.8f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Back wall (white)
        {
            Box b;
            b.minP = Vec3(-S, -S, -S);
            b.maxP = Vec3(S, S, -S + 0.02f);
            b.mat.albedo = Vec3(0.8f, 0.8f, 0.8f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Left wall (red)
        {
            Box b;
            b.minP = Vec3(-S, -S, -S);
            b.maxP = Vec3(-S + 0.02f, S, S);
            b.mat.albedo = Vec3(0.85f, 0.1f, 0.1f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Right wall (green)
        {
            Box b;
            b.minP = Vec3(S - 0.02f, -S, -S);
            b.maxP = Vec3(S, S, S);
            b.mat.albedo = Vec3(0.1f, 0.75f, 0.1f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Short block (rotated box approximation - just use axis-aligned)
        {
            Box b;
            b.minP = Vec3(-0.55f, -1.0f, -0.55f);
            b.maxP = Vec3(-0.05f, -0.35f, 0.1f);
            b.mat.albedo = Vec3(0.75f, 0.75f, 0.75f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        // Tall block
        {
            Box b;
            b.minP = Vec3(0.1f, -1.0f, -0.6f);
            b.maxP = Vec3(0.65f, 0.35f, -0.1f);
            b.mat.albedo = Vec3(0.75f, 0.75f, 0.75f);
            b.isFlip = false;
            boxes.push_back(b);
        }
        
        // Metallic sphere
        {
            Sphere s;
            s.center = Vec3(-0.30f, -0.65f, -0.15f);
            s.radius = 0.25f;
            s.mat.albedo = Vec3(0.95f, 0.86f, 0.65f);
            s.mat.roughness = 0.05f;
            s.mat.metallic = 1.0f;
            spheres.push_back(s);
        }
        // Dielectric-like sphere (matte colored)
        {
            Sphere s;
            s.center = Vec3(0.35f, -0.60f, 0.2f);
            s.radius = 0.20f;
            s.mat.albedo = Vec3(0.3f, 0.4f, 0.9f);
            s.mat.roughness = 0.7f;
            s.mat.metallic = 0.0f;
            spheres.push_back(s);
        }
        
        // Area light
        lightPos = Vec3(0, 0.96f, 0);
        lightSize = Vec3(0.3f, 0.0f, 0.3f);
        lightColor = Vec3(15.0f, 14.0f, 12.0f);
        
        // Add emissive ceiling light box
        {
            Box b;
            b.minP = Vec3(-0.3f, 0.94f, -0.3f);
            b.maxP = Vec3(0.3f, 0.96f, 0.3f);
            b.mat.albedo = Vec3(1.0f, 1.0f, 1.0f);
            b.mat.emission = Vec3(15.0f, 14.0f, 12.0f);
            b.mat.isLight = true;
            b.isFlip = false;
            boxes.push_back(b);
        }
    }
};

// ============================================================
// Voxelization
// ============================================================

// Transform world point to voxel grid [0,1]^3
static const Vec3 SCENE_MIN(-1.0f, -1.0f, -1.0f);
static const Vec3 SCENE_SIZE(2.0f, 2.0f, 2.0f);

Vec3 worldToUVW(const Vec3& p) {
    return Vec3(
        (p.x - SCENE_MIN.x) / SCENE_SIZE.x,
        (p.y - SCENE_MIN.y) / SCENE_SIZE.y,
        (p.z - SCENE_MIN.z) / SCENE_SIZE.z
    );
}

Vec3 uvwToVoxel(const Vec3& uvw, int dim) {
    return uvw * float(dim);
}

void voxelizeScene(VoxelGrid& grid, const Scene& scene) {
    int D = VOXEL_DIM;
    float voxelSize = SCENE_SIZE.x / D;
    
    // Rasterize each voxel cell by ray casting
    for (int z = 0; z < D; z++) {
        for (int y = 0; y < D; y++) {
            for (int x = 0; x < D; x++) {
                // Voxel center in world space
                Vec3 center = SCENE_MIN + Vec3(
                    (x + 0.5f) * voxelSize,
                    (y + 0.5f) * voxelSize,
                    (z + 0.5f) * voxelSize
                );
                
                // Check if this voxel is inside any object
                // We test against all primitives with 6 directional samples
                Vec3 avgColor(0);
                float avgOpacity = 0;
                int hits = 0;
                
                // Cast rays from voxel center and check nearby hits
                // Simple approach: check if scene intersects near the voxel
                const Vec3 dirs[6] = {
                    {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}
                };
                
                for (auto& dir : dirs) {
                    // Ray from just outside voxel
                    Vec3 ro = center - dir * (voxelSize * 0.5f + 1e-3f);
                    HitRecord rec = scene.intersect(ro, dir);
                    if (rec.hit && rec.t < voxelSize + 2e-3f) {
                        Vec3 color = rec.mat.emission + rec.mat.albedo * 0.5f;
                        if (rec.mat.isLight) color = rec.mat.emission;
                        avgColor += color;
                        avgOpacity += 1.0f;
                        hits++;
                    }
                }
                
                if (hits > 0) {
                    avgColor = avgColor / float(hits);
                    float opacity = std::min(float(hits) / 6.0f, 1.0f) * 0.85f;
                    
                    // For lights, boost opacity
                    Vec3 ro = center;
                    Vec3 testDir(0.1f, 1.0f, 0.1f);
                    testDir = testDir.normalized();
                    HitRecord rec = scene.intersect(ro, testDir);
                    if (rec.hit && rec.mat.isLight) {
                        opacity = 1.0f;
                        avgColor = rec.mat.emission;
                    }
                    
                    grid.set(0, x, y, z, Vec4(avgColor, opacity));
                }
            }
        }
    }
    
    // Build mipmap chain
    grid.buildMipmaps();
}

// ============================================================
// Direct Lighting (Shadow Ray)
// ============================================================

Vec3 directLight(const Vec3& point, const Vec3& normal, const Material& mat,
                 const Scene& scene, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(-1, 1);
    
    // Sample area light
    Vec3 lightP = scene.lightPos + Vec3(
        dist(rng) * scene.lightSize.x,
        0,
        dist(rng) * scene.lightSize.z
    );
    
    Vec3 toLight = lightP - point;
    float distSq = toLight.dot(toLight);
    float dist2 = std::sqrt(distSq);
    Vec3 lightDir = toLight / dist2;
    
    float NdotL = std::max(0.0f, normal.dot(lightDir));
    if (NdotL < 1e-6f) return Vec3(0);
    
    // Shadow test
    HitRecord shadow = scene.intersect(point + normal * 1e-4f, lightDir);
    if (shadow.hit && shadow.t < dist2 - 0.01f && !shadow.mat.isLight) {
        return Vec3(0);
    }
    
    // Light attenuation: area light (facing down), use solid-angle style attenuation
    // lightNdotL: cosine of angle between light's emit direction (down) and direction to surface
    // For an area light facing downward, we want to illuminate all surfaces below it,
    // not just horizontal ones. Use 1/distance^2 falloff only.
    float attenuation = 1.0f / (distSq + 0.01f);
    
    Vec3 radiance = scene.lightColor * attenuation * 20.0f;
    
    // Diffuse BRDF
    Vec3 diffuse = mat.albedo / float(M_PI) * (1.0f - mat.metallic);
    
    // Specular (simplified GGX)
    Vec3 viewDir = (point - Vec3(0, 0, 3.0f)).normalized();
    Vec3 halfVec = (lightDir - viewDir).normalized();
    float NdotH = std::max(0.0f, normal.dot(halfVec));
    float alpha = mat.roughness * mat.roughness;
    float D = alpha * alpha / (float(M_PI) * std::pow(NdotH * NdotH * (alpha * alpha - 1) + 1, 2.0f));
    Vec3 F0 = mix(Vec3(0.04f), mat.albedo, mat.metallic);
    float HdotV = std::max(0.0f, halfVec.dot(-viewDir));
    Vec3 F = F0 + (Vec3(1.0f) - F0) * std::pow(1.0f - HdotV, 5.0f);
    Vec3 specular = F * D * 0.25f;
    
    return (diffuse + specular) * radiance * NdotL;
}

// ============================================================
// Voxel Cone Tracing
// ============================================================

// Trace a cone through the voxel grid
// aperture: half-angle of cone in radians
// Returns: (color, occlusion)
Vec4 traceCone(const VoxelGrid& grid, Vec3 origin, Vec3 direction,
               float aperture, float maxDist) {
    direction = direction.normalized();
    
    Vec4 accum(0, 0, 0, 0);  // RGBA, alpha = occlusion
    
    float stepDist = 0.0f;
    float voxelWorldSize = SCENE_SIZE.x / float(VOXEL_DIM);
    
    // Start slightly away from surface to avoid self-intersection
    stepDist = voxelWorldSize * 1.5f;
    
    while (stepDist < maxDist && accum.w < 0.97f) {
        // Cone diameter at this distance
        float coneDiameter = 2.0f * stepDist * std::tan(aperture);
        coneDiameter = std::max(coneDiameter, voxelWorldSize);
        
        // Determine which mip level to sample
        // mip 0 = voxelWorldSize, each mip = 2x larger
        float mipLevel = std::log2(coneDiameter / voxelWorldSize);
        mipLevel = std::clamp(mipLevel, 0.0f, float(MIP_LEVELS - 1));
        
        // Sample position in world space -> UVW
        Vec3 sampleWorld = origin + direction * stepDist;
        Vec3 uvw = worldToUVW(sampleWorld);
        
        // Check bounds
        if (uvw.x < 0 || uvw.x > 1 || uvw.y < 0 || uvw.y > 1 || uvw.z < 0 || uvw.z > 1) {
            break;
        }
        
        // Sample voxel grid
        Vec4 voxel = grid.sampleMip(uvw.x, uvw.y, uvw.z, mipLevel);
        
        // Front-to-back alpha compositing
        float alpha = voxel.w * (1.0f - accum.w);
        accum.x += voxel.x * alpha;
        accum.y += voxel.y * alpha;
        accum.z += voxel.z * alpha;
        accum.w += alpha;
        
        // Adaptive step size: larger cone -> fewer samples needed
        float stepSize = std::max(coneDiameter * 0.5f, voxelWorldSize);
        stepDist += stepSize;
    }
    
    return accum;
}

// Diffuse indirect lighting via cone tracing
// Cast multiple cones in hemisphere oriented around normal
Vec3 indirectDiffuse(const VoxelGrid& grid, const Vec3& point, const Vec3& normal,
                     const Vec3& albedo) {
    // Create local frame
    Vec3 up = (std::abs(normal.y) < 0.9f) ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 tangent = up.cross(normal).normalized();
    Vec3 bitangent = normal.cross(tangent);
    
    // 6 cones in hemisphere (from Crassin et al.)
    // 60 degree aperture cones at angles around hemisphere
    const float CONE_ANGLE = 0.523599f; // 30 degrees
    const float MAX_DIST = 2.0f;
    
    // Cone directions in local space (hemisphere sampling)
    const float sqrt5 = 2.2360679f;
    struct ConeDir { float x, y, z, weight; };
    const ConeDir coneDirs[] = {
        { 0.0f,          1.0f,  0.0f,          0.25f },  // straight up
        { 0.0f,          0.5f,  0.8660254f,     0.15f },  // 60 degrees forward
        { 0.8228756f,    0.5f,  0.2745967f,     0.15f },  // 60 degrees +120
        { 0.5f,          0.5f, -0.7071068f,     0.15f },  // 60 degrees +240
        {-0.5f,          0.5f, -0.7071068f,     0.15f },  // 60 degrees +300
        {-0.8228756f,    0.5f,  0.2745967f,     0.15f },  // 60 degrees +240
    };
    (void)sqrt5;
    
    Vec3 indirect(0);
    float totalWeight = 0;
    
    Vec3 offsetPoint = point + normal * (SCENE_SIZE.x / VOXEL_DIM * 2.0f);
    
    for (auto& cd : coneDirs) {
        // Transform cone direction from local to world space
        Vec3 localDir(cd.x, cd.y, cd.z);
        Vec3 worldDir = tangent * localDir.x + normal * localDir.y + bitangent * localDir.z;
        worldDir = worldDir.normalized();
        
        Vec4 result = traceCone(grid, offsetPoint, worldDir, CONE_ANGLE, MAX_DIST);
        
        // result.xyz = incoming radiance, result.w = occlusion
        indirect += Vec3(result.x, result.y, result.z) * cd.weight;
        totalWeight += cd.weight;
    }
    
    if (totalWeight > 0) indirect = indirect / totalWeight;
    
    // Modulate by albedo
    return indirect * albedo * 2.0f;
}

// Specular indirect lighting via single narrow cone
Vec3 indirectSpecular(const VoxelGrid& grid, const Vec3& point, const Vec3& normal,
                      const Vec3& viewDir, const Material& mat) {
    if (mat.roughness > 0.8f) return Vec3(0);  // Too rough for specular VCT
    
    Vec3 reflectDir = reflect(viewDir, normal);
    
    // Aperture based on roughness: rough = wider cone
    float aperture = std::max(0.01f, mat.roughness * 0.5f);
    
    Vec3 offsetPoint = point + normal * (SCENE_SIZE.x / VOXEL_DIM * 2.0f);
    Vec4 result = traceCone(grid, offsetPoint, reflectDir, aperture, 3.0f);
    
    Vec3 F0 = mix(Vec3(0.04f), mat.albedo, mat.metallic);
    float HdotV = std::max(0.0f, normal.dot(-viewDir));
    Vec3 F = F0 + (Vec3(1.0f) - F0) * std::pow(1.0f - HdotV, 5.0f);
    
    return Vec3(result.x, result.y, result.z) * F * (1.0f - mat.roughness);
}

// ============================================================
// Rendering
// ============================================================

struct Camera {
    Vec3 position;
    Vec3 target;
    Vec3 up;
    float fov;
    int width, height;
    
    void getLookAt(float u, float v, Vec3& ro, Vec3& rd) const {
        Vec3 forward = (target - position).normalized();
        Vec3 right = forward.cross(up).normalized();
        Vec3 camUp = right.cross(forward);
        
        float tanHalfFov = std::tan(fov * 0.5f * float(M_PI) / 180.0f);
        float aspect = float(width) / float(height);
        
        rd = (forward + right * (u * tanHalfFov * aspect) + camUp * (v * tanHalfFov)).normalized();
        ro = position;
    }
};

// Vec3-vec3 element-wise division
inline Vec3 vdiv(const Vec3& a, const Vec3& b) {
    return {a.x/b.x, a.y/b.y, a.z/b.z};
}

// Tone mapping
Vec3 ACESFilm(Vec3 x) {
    const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    Vec3 num = x * (x * a + Vec3(b));
    Vec3 den = x * (x * c + Vec3(d)) + Vec3(e);
    return vdiv(num, den).clamp(0, 1);
}

Vec3 gammaCorrect(const Vec3& c) {
    return Vec3(std::pow(c.x, 1.0f/2.2f), std::pow(c.y, 1.0f/2.2f), std::pow(c.z, 1.0f/2.2f));
}

// ============================================================
// PPM Writer
// ============================================================

void writePPM(const std::string& filename, int width, int height,
              const std::vector<Vec3>& pixels) {
    std::ofstream f(filename, std::ios::binary);
    f << "P6\n" << width << " " << height << "\n255\n";
    for (auto& p : pixels) {
        Vec3 c = p.clamp(0, 1);
        uint8_t r = uint8_t(c.x * 255.99f);
        uint8_t g = uint8_t(c.y * 255.99f);
        uint8_t b = uint8_t(c.z * 255.99f);
        f.write(reinterpret_cast<char*>(&r), 1);
        f.write(reinterpret_cast<char*>(&g), 1);
        f.write(reinterpret_cast<char*>(&b), 1);
    }
}

// ============================================================
// Main Render Function
// ============================================================

int main() {
    std::cout << "[VCT] Voxel Cone Tracing - Starting..." << std::endl;
    
    // Scene
    Scene scene;
    scene.build();
    std::cout << "[VCT] Scene built: " << scene.boxes.size() << " boxes, "
              << scene.spheres.size() << " spheres" << std::endl;
    
    // Voxelization
    std::cout << "[VCT] Voxelizing scene (" << VOXEL_DIM << "^3)..." << std::endl;
    VoxelGrid grid;
    voxelizeScene(grid, scene);
    std::cout << "[VCT] Voxelization complete. Building mipmaps..." << std::endl;
    // Mipmaps already built in voxelizeScene
    std::cout << "[VCT] Mipmaps built (" << MIP_LEVELS << " levels)" << std::endl;
    
    // Camera
    const int WIDTH = 512, HEIGHT = 512;
    Camera cam;
    cam.position = Vec3(0, 0, 3.5f);
    cam.target = Vec3(0, 0, 0);
    cam.up = Vec3(0, 1, 0);
    cam.fov = 45.0f;
    cam.width = WIDTH;
    cam.height = HEIGHT;
    
    std::mt19937 rng(42);
    
    // Render
    std::vector<Vec3> directPixels(WIDTH * HEIGHT);
    std::vector<Vec3> indirectPixels(WIDTH * HEIGHT);
    std::vector<Vec3> combinedPixels(WIDTH * HEIGHT);
    
    std::cout << "[VCT] Rendering " << WIDTH << "x" << HEIGHT << "..." << std::endl;
    
    const int SAMPLES = 4;  // anti-aliasing samples per pixel
    std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
    
    for (int py = 0; py < HEIGHT; py++) {
        if (py % 64 == 0) {
            std::cout << "[VCT] Row " << py << "/" << HEIGHT 
                      << " (" << int(100.0f * py / HEIGHT) << "%)" << std::endl;
        }
        for (int px = 0; px < WIDTH; px++) {
            Vec3 directColor(0), indirectColor(0);
            
            for (int s = 0; s < SAMPLES; s++) {
                float u = (px + jitter(rng) + 0.5f) / WIDTH * 2.0f - 1.0f;
                float v = 1.0f - (py + jitter(rng) + 0.5f) / HEIGHT * 2.0f;
                
                Vec3 ro, rd;
                cam.getLookAt(u, v, ro, rd);
                
                HitRecord hit = scene.intersect(ro, rd);
                
                if (hit.hit) {
                    if (hit.mat.isLight) {
                        // Light source - use emission
                        directColor += hit.mat.emission * 0.3f;
                    } else {
                        // Direct lighting
                        Vec3 direct = directLight(hit.point, hit.normal, hit.mat, scene, rng);
                        directColor += direct;
                        
                        // Indirect via VCT
                        Vec3 viewDir = rd;
                        Vec3 diffGI = indirectDiffuse(grid, hit.point, hit.normal, hit.mat.albedo);
                        Vec3 specGI = indirectSpecular(grid, hit.point, hit.normal, viewDir, hit.mat);
                        
                        indirectColor += diffGI + specGI;
                    }
                } else {
                    // Background - black
                    directColor += Vec3(0);
                }
            }
            
            directColor = directColor / float(SAMPLES);
            indirectColor = indirectColor / float(SAMPLES);
            
            directPixels[py * WIDTH + px] = directColor;
            indirectPixels[py * WIDTH + px] = indirectColor;
            combinedPixels[py * WIDTH + px] = directColor + indirectColor;
        }
    }
    
    std::cout << "[VCT] Render complete. Applying tone mapping..." << std::endl;
    
    // Tone mapping and gamma correction
    auto processBuffer = [&](std::vector<Vec3>& buf) {
        for (auto& p : buf) {
            p = ACESFilm(p);
            p = gammaCorrect(p);
        }
    };
    
    std::vector<Vec3> directTM = directPixels;
    std::vector<Vec3> indirectTM = indirectPixels;
    std::vector<Vec3> combinedTM = combinedPixels;
    
    processBuffer(directTM);
    processBuffer(indirectTM);
    processBuffer(combinedTM);
    
    // Write outputs
    writePPM("vct_direct.ppm", WIDTH, HEIGHT, directTM);
    writePPM("vct_indirect.ppm", WIDTH, HEIGHT, indirectTM);
    writePPM("vct_output.ppm", WIDTH, HEIGHT, combinedTM);
    
    std::cout << "[VCT] Written: vct_direct.ppm, vct_indirect.ppm, vct_output.ppm" << std::endl;
    
    // Stats
    float totalBrightness = 0;
    for (auto& p : combinedTM) totalBrightness += (p.x + p.y + p.z) / 3.0f;
    float avgBrightness = totalBrightness / (WIDTH * HEIGHT);
    std::cout << "[VCT] Average brightness: " << avgBrightness << std::endl;
    
    // Print pixel stats for validation
    Vec3 minP(1e9f), maxP(-1e9f), sumP(0);
    for (auto& p : combinedTM) {
        minP.x = std::min(minP.x, p.x);
        minP.y = std::min(minP.y, p.y);
        minP.z = std::min(minP.z, p.z);
        maxP.x = std::max(maxP.x, p.x);
        maxP.y = std::max(maxP.y, p.y);
        maxP.z = std::max(maxP.z, p.z);
        sumP += p;
    }
    sumP = sumP / float(WIDTH * HEIGHT);
    std::cout << "[VCT] Color range: [" 
              << minP.x << ", " << minP.y << ", " << minP.z << "] to ["
              << maxP.x << ", " << maxP.y << ", " << maxP.z << "]" << std::endl;
    std::cout << "[VCT] Average color: " << sumP.x << ", " << sumP.y << ", " << sumP.z << std::endl;
    std::cout << "[VCT] Done!" << std::endl;
    
    return 0;
}
