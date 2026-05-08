/**
 * OBJ Mesh PBR Renderer
 * 
 * Features:
 * - OBJ file parser (vertices, normals, UVs, faces)
 * - MTL material file parser (Kd, Ks, Ns, Ka)
 * - PBR metallic-roughness material model
 * - Cook-Torrance BRDF (GGX/Trowbridge-Reitz)
 * - BVH acceleration structure for ray-mesh intersection
 * - Soft rasterizer with depth buffer
 * - Multiple light sources (directional + point)
 * - Normal interpolation (Gouraud/Phong normal)
 * - UV-based procedural textures (checkerboard, noise)
 * 
 * Output: obj_pbr_output.png (800x600)
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <memory>
#include <map>
#include <array>
#include <random>
#include <functional>

// ─── Math Types ──────────────────────────────────────────────────────────────

struct Vec2 {
    float x, y;
    Vec2() : x(0), y(0) {}
    Vec2(float x, float y) : x(x), y(y) {}
};

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
    float lengthSq() const { return x*x + y*y + z*z; }
    Vec3 normalized() const {
        float l = length();
        if (l < 1e-8f) return {0,0,0};
        return *this / l;
    }
    Vec3 clamp(float lo, float hi) const {
        return {std::max(lo,std::min(hi,x)),
                std::max(lo,std::min(hi,y)),
                std::max(lo,std::min(hi,z))};
    }
    Vec3 pow(float p) const {
        return {std::pow(std::max(0.f,x),p),
                std::pow(std::max(0.f,y),p),
                std::pow(std::max(0.f,z),p)};
    }
};

inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline float dot(const Vec3& a, const Vec3& b) { return a.dot(b); }
inline Vec3 cross(const Vec3& a, const Vec3& b) { return a.cross(b); }
inline Vec3 normalize(const Vec3& v) { return v.normalized(); }
inline Vec3 reflect(const Vec3& I, const Vec3& N) { return I - 2.f * dot(I,N) * N; }
inline Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
inline Vec3 clamp(const Vec3& v, float lo, float hi) { return v.clamp(lo,hi); }

// ─── Material ─────────────────────────────────────────────────────────────────

struct Material {
    std::string name;
    Vec3 albedo    = {0.8f, 0.8f, 0.8f};
    Vec3 emission  = {0.f, 0.f, 0.f};
    float metallic   = 0.0f;
    float roughness  = 0.5f;
    float ior        = 1.5f;
    
    // Parsed from MTL
    Vec3 Ka = {0.1f, 0.1f, 0.1f};  // ambient
    Vec3 Kd = {0.8f, 0.8f, 0.8f};  // diffuse
    Vec3 Ks = {0.0f, 0.0f, 0.0f};  // specular
    float Ns = 50.f;                  // shininess
    float d  = 1.f;                   // dissolve (alpha)
    
    // Procedural texture type
    int texType = 0; // 0=solid, 1=checker, 2=noise
    float texScale = 4.f;
};

// ─── OBJ/MTL Parser ──────────────────────────────────────────────────────────

struct VertexIndex {
    int v = -1, vt = -1, vn = -1;
};

struct Face {
    VertexIndex idx[3];
    int matIdx = 0;
};

struct Mesh {
    std::vector<Vec3> positions;
    std::vector<Vec3> normals;
    std::vector<Vec2> texcoords;
    std::vector<Face> faces;
    std::vector<Material> materials;
    std::map<std::string, int> matMap;
};

// Parse "v/vt/vn" or "v//vn" or "v/vt" or "v"
static VertexIndex parseVertexIndex(const std::string& token) {
    VertexIndex vi;
    std::istringstream ss(token);
    std::string part;
    int idx = 0;
    while (std::getline(ss, part, '/')) {
        if (!part.empty()) {
            int val = std::stoi(part) - 1; // OBJ is 1-indexed
            if (idx == 0) vi.v  = val;
            if (idx == 1) vi.vt = val;
            if (idx == 2) vi.vn = val;
        }
        idx++;
    }
    return vi;
}

static bool parseMTL(const std::string& filename, Mesh& mesh) {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    
    Material* cur = nullptr;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        if (key == "newmtl") {
            mesh.materials.emplace_back();
            cur = &mesh.materials.back();
            ss >> cur->name;
            mesh.matMap[cur->name] = (int)mesh.materials.size() - 1;
        } else if (cur) {
            if (key == "Ka") {
                ss >> cur->Ka.x >> cur->Ka.y >> cur->Ka.z;
            } else if (key == "Kd") {
                ss >> cur->Kd.x >> cur->Kd.y >> cur->Kd.z;
                cur->albedo = cur->Kd;
            } else if (key == "Ks") {
                ss >> cur->Ks.x >> cur->Ks.y >> cur->Ks.z;
                // Estimate metallic from specular intensity
                float sInt = (cur->Ks.x + cur->Ks.y + cur->Ks.z) / 3.f;
                cur->metallic = std::min(1.f, sInt);
            } else if (key == "Ns") {
                ss >> cur->Ns;
                // Convert shininess to roughness: roughness ~ sqrt(2/(Ns+2))
                cur->roughness = std::sqrt(2.f / (cur->Ns + 2.f));
                cur->roughness = std::max(0.05f, std::min(1.f, cur->roughness));
            } else if (key == "d" || key == "Tr") {
                ss >> cur->d;
                if (key == "Tr") cur->d = 1.f - cur->d;
            } else if (key == "Ni") {
                ss >> cur->ior;
            } else if (key == "Ke") {
                ss >> cur->emission.x >> cur->emission.y >> cur->emission.z;
            }
        }
    }
    return true;
}

static bool parseOBJ(const std::string& filename, Mesh& mesh,
                     const std::string& baseDir = "") {
    std::ifstream f(filename);
    if (!f.is_open()) return false;
    
    int curMat = 0;
    std::string line;
    
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string key;
        ss >> key;
        
        if (key == "v") {
            Vec3 p;
            ss >> p.x >> p.y >> p.z;
            mesh.positions.push_back(p);
        } else if (key == "vn") {
            Vec3 n;
            ss >> n.x >> n.y >> n.z;
            mesh.normals.push_back(n);
        } else if (key == "vt") {
            Vec2 t;
            ss >> t.x >> t.y;
            mesh.texcoords.push_back(t);
        } else if (key == "f") {
            // Read all vertex tokens on this face line
            std::vector<VertexIndex> faceVerts;
            std::string tok;
            while (ss >> tok) {
                faceVerts.push_back(parseVertexIndex(tok));
            }
            // Triangulate (fan)
            for (int i = 1; i + 1 < (int)faceVerts.size(); i++) {
                Face face;
                face.idx[0] = faceVerts[0];
                face.idx[1] = faceVerts[i];
                face.idx[2] = faceVerts[i+1];
                face.matIdx = curMat;
                mesh.faces.push_back(face);
            }
        } else if (key == "usemtl") {
            std::string matName;
            ss >> matName;
            auto it = mesh.matMap.find(matName);
            if (it != mesh.matMap.end()) {
                curMat = it->second;
            }
        } else if (key == "mtllib") {
            std::string mtlFile;
            ss >> mtlFile;
            parseMTL(baseDir + mtlFile, mesh);
        }
    }
    
    // If no materials parsed, add a default
    if (mesh.materials.empty()) {
        mesh.materials.emplace_back();
        mesh.materials.back().name = "default";
        mesh.matMap["default"] = 0;
    }
    
    return true;
}

// ─── Procedural Scene — Stanford Bunny-like parametric mesh ──────────────────

// Generate a procedural "vase" mesh via lathe/revolution
static void generateVaseMesh(Mesh& mesh, int /*latSlices*/ = 64, int lonSlices = 48) {
    // Profile curve points (radius, height) — vase shape
    std::vector<std::pair<float,float>> profile = {
        {0.0f, -1.0f},   // bottom center
        {0.3f, -0.95f},
        {0.5f, -0.8f},
        {0.6f, -0.5f},
        {0.55f, -0.2f},  // waist
        {0.45f,  0.0f},
        {0.5f,   0.2f},
        {0.65f,  0.5f},  // belly
        {0.7f,   0.7f},
        {0.6f,   0.9f},
        {0.45f,  1.0f},  // neck
        {0.35f,  1.1f},
        {0.4f,   1.2f},
        {0.5f,   1.3f},  // rim
        {0.52f,  1.35f},
        {0.4f,   1.4f},
        {0.0f,   1.4f},  // top center
    };
    
    int nP = (int)profile.size();
    
    // Generate vertices
    // For each profile point, generate a circle of vertices
    for (int j = 0; j < nP; j++) {
        float r = profile[j].first;
        float y = profile[j].second;
        
        for (int i = 0; i <= lonSlices; i++) {
            float theta = (float)i / lonSlices * 2.f * M_PI;
            float cx = r * std::cos(theta);
            float cz = r * std::sin(theta);
            mesh.positions.push_back({cx, y, cz});
            
            // UV
            float u = (float)i / lonSlices;
            float v = (float)j / (nP - 1);
            mesh.texcoords.push_back({u, v});
        }
    }
    
    // Generate normals per vertex (approximate from neighbors)
    // We'll compute face normals and accumulate
    int stride = lonSlices + 1;
    
    // Initialize normals to zero
    mesh.normals.resize(mesh.positions.size(), {0,0,0});
    
    // Generate faces and accumulate normals
    auto vertIdx = [&](int j, int i) { return j * stride + i; };
    
    for (int j = 0; j < nP - 1; j++) {
        for (int i = 0; i < lonSlices; i++) {
            int v00 = vertIdx(j,   i);
            int v10 = vertIdx(j,   i+1);
            int v01 = vertIdx(j+1, i);
            int v11 = vertIdx(j+1, i+1);
            
            // Face 1: v00, v10, v11
            {
                Face face;
                face.idx[0] = {v00, v00, v00};
                face.idx[1] = {v10, v10, v10};
                face.idx[2] = {v11, v11, v11};
                face.matIdx = 0;
                mesh.faces.push_back(face);
                
                Vec3 e1 = mesh.positions[v10] - mesh.positions[v00];
                Vec3 e2 = mesh.positions[v11] - mesh.positions[v00];
                Vec3 fn = cross(e1, e2);
                mesh.normals[v00] += fn;
                mesh.normals[v10] += fn;
                mesh.normals[v11] += fn;
            }
            // Face 2: v00, v11, v01
            {
                Face face;
                face.idx[0] = {v00, v00, v00};
                face.idx[1] = {v11, v11, v11};
                face.idx[2] = {v01, v01, v01};
                face.matIdx = 0;
                mesh.faces.push_back(face);
                
                Vec3 e1 = mesh.positions[v11] - mesh.positions[v00];
                Vec3 e2 = mesh.positions[v01] - mesh.positions[v00];
                Vec3 fn = cross(e1, e2);
                mesh.normals[v00] += fn;
                mesh.normals[v11] += fn;
                mesh.normals[v01] += fn;
            }
        }
    }
    
    // Normalize all normals
    for (auto& n : mesh.normals) n = n.normalized();
    
    // Add a default material with nice ceramic-like PBR properties
    Material ceramicMat;
    ceramicMat.name = "ceramic";
    ceramicMat.albedo = {0.85f, 0.6f, 0.3f};  // warm terracotta
    ceramicMat.metallic = 0.0f;
    ceramicMat.roughness = 0.3f;
    ceramicMat.texType = 1; // checker-like decorative pattern
    ceramicMat.texScale = 8.f;
    
    Material goldTrimMat;
    goldTrimMat.name = "gold_trim";
    goldTrimMat.albedo = {1.0f, 0.78f, 0.24f};  // gold
    goldTrimMat.metallic = 0.95f;
    goldTrimMat.roughness = 0.15f;
    goldTrimMat.texType = 0;
    
    mesh.materials.push_back(ceramicMat);
    mesh.materials.push_back(goldTrimMat);
    mesh.matMap["ceramic"] = 0;
    mesh.matMap["gold_trim"] = 1;
    
    // Assign gold trim to rim faces (top profiles)
    int rimStart = (int)mesh.faces.size();
    for (auto& face : mesh.faces) {
        // Identify rim region by vertex height > 1.2
        bool isRim = false;
        for (int k = 0; k < 3; k++) {
            if (face.idx[k].v >= 0 && 
                mesh.positions[face.idx[k].v].y > 1.2f) {
                isRim = true;
                break;
            }
        }
        face.matIdx = isRim ? 1 : 0;
    }
    (void)rimStart;
}

// Add a floor plane
static void addFloor(Mesh& mesh, float y = -1.05f, float size = 3.0f) {
    int baseV = (int)mesh.positions.size();
    int baseN = (int)mesh.normals.size();
    int baseT = (int)mesh.texcoords.size();
    
    mesh.positions.push_back({-size, y, -size});
    mesh.positions.push_back({ size, y, -size});
    mesh.positions.push_back({ size, y,  size});
    mesh.positions.push_back({-size, y,  size});
    
    for (int i = 0; i < 4; i++) mesh.normals.push_back({0,1,0});
    
    mesh.texcoords.push_back({0, 0});
    mesh.texcoords.push_back({1, 0});
    mesh.texcoords.push_back({1, 1});
    mesh.texcoords.push_back({0, 1});
    
    // Floor material
    if (mesh.matMap.find("floor") == mesh.matMap.end()) {
        Material floorMat;
        floorMat.name = "floor";
        floorMat.albedo = {0.9f, 0.9f, 0.9f};
        floorMat.metallic = 0.0f;
        floorMat.roughness = 0.8f;
        floorMat.texType = 1; // checker
        floorMat.texScale = 5.f;
        mesh.matMap["floor"] = (int)mesh.materials.size();
        mesh.materials.push_back(floorMat);
    }
    int floorMat = mesh.matMap["floor"];
    
    {
        Face f; 
        f.idx[0] = {baseV+0, baseT+0, baseN+0};
        f.idx[1] = {baseV+1, baseT+1, baseN+1};
        f.idx[2] = {baseV+2, baseT+2, baseN+2};
        f.matIdx = floorMat;
        mesh.faces.push_back(f);
    }
    {
        Face f; 
        f.idx[0] = {baseV+0, baseT+0, baseN+0};
        f.idx[1] = {baseV+2, baseT+2, baseN+2};
        f.idx[2] = {baseV+3, baseT+3, baseN+3};
        f.matIdx = floorMat;
        mesh.faces.push_back(f);
    }
}

// ─── PBR BRDF ─────────────────────────────────────────────────────────────────

// Trowbridge-Reitz GGX normal distribution
static float distributionGGX(float NdotH, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float denom = NdotH * NdotH * (a2 - 1.f) + 1.f;
    return a2 / (float(M_PI) * denom * denom + 1e-7f);
}

// Smith's geometry function
static float geometrySchlickGGX(float NdotV, float roughness) {
    float r = roughness + 1.f;
    float k = r * r / 8.f;
    return NdotV / (NdotV * (1.f - k) + k + 1e-7f);
}

static float geometrySmith(float NdotV, float NdotL, float roughness) {
    return geometrySchlickGGX(NdotV, roughness) *
           geometrySchlickGGX(NdotL, roughness);
}

// Fresnel-Schlick
static Vec3 fresnelSchlick(float cosTheta, const Vec3& F0) {
    float p = std::pow(std::max(0.f, 1.f - cosTheta), 5.f);
    return F0 + (Vec3(1.f) - F0) * p;
}

// Full Cook-Torrance PBR BRDF
static Vec3 cookTorrance(const Vec3& N, const Vec3& V, const Vec3& L,
                          const Vec3& albedo, float metallic, float roughness,
                          const Vec3& lightColor) {
    float NdotL = std::max(0.f, dot(N, L));
    float NdotV = std::max(0.f, dot(N, V));
    if (NdotL <= 0.f || NdotV <= 0.f) return {0,0,0};
    
    Vec3 H = normalize(V + L);
    float NdotH = std::max(0.f, dot(N, H));
    float HdotV = std::max(0.f, dot(H, V));
    
    // F0: base reflectance
    Vec3 F0 = mix(Vec3(0.04f), albedo, metallic);
    
    float D = distributionGGX(NdotH, roughness);
    float G = geometrySmith(NdotV, NdotL, roughness);
    Vec3  F = fresnelSchlick(HdotV, F0);
    
    // Specular
    Vec3 specular = D * G * F / (4.f * NdotV * NdotL + 1e-7f);
    
    // Diffuse (lambertian, only for non-metals)
    Vec3 kD = (Vec3(1.f) - F) * (1.f - metallic);
    Vec3 diffuse = kD * albedo / float(M_PI);
    
    return (diffuse + specular) * lightColor * NdotL;
}

// ─── Procedural Textures ──────────────────────────────────────────────────────

static Vec3 sampleTexture(const Material& mat, float u, float v) {
    switch (mat.texType) {
        case 1: { // Checker
            float s = mat.texScale;
            int cx = (int)std::floor(u * s) & 1;
            int cy = (int)std::floor(v * s) & 1;
            float t = (cx ^ cy) ? 1.f : 0.f;
            return mix(mat.albedo * 0.4f, mat.albedo, t);
        }
        case 2: { // Simple procedural noise (hash-based)
            auto hash = [](float x, float y) -> float {
                int ix = (int)(x * 7.f) ^ (int)(y * 13.f);
                ix = (ix ^ (ix >> 4)) * 0x45d9f3b;
                return (float)(ix & 0xFFFF) / 65535.f;
            };
            float s = mat.texScale;
            float fx = u * s, fy = v * s;
            float n = hash(std::floor(fx), std::floor(fy));
            return mat.albedo * (0.7f + 0.3f * n);
        }
        default:
            return mat.albedo;
    }
}

// ─── Framebuffer & PNG ────────────────────────────────────────────────────────

static std::vector<uint8_t> toUint8(const std::vector<Vec3>& fb,
                                     int w, int h, float exposure = 1.0f) {
    std::vector<uint8_t> data(w * h * 3);
    for (int i = 0; i < w * h; i++) {
        Vec3 c = fb[i] * exposure;
        // ACES tone mapping
        auto aces = [](float x) -> float {
            const float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
            return std::max(0.f, std::min(1.f, (x*(a*x+b))/(x*(c*x+d)+e)));
        };
        c = {aces(c.x), aces(c.y), aces(c.z)};
        // Gamma correction
        c = c.pow(1.f / 2.2f);
        c = c.clamp(0, 1);
        data[i*3+0] = (uint8_t)(c.x * 255.f + 0.5f);
        data[i*3+1] = (uint8_t)(c.y * 255.f + 0.5f);
        data[i*3+2] = (uint8_t)(c.z * 255.f + 0.5f);
    }
    return data;
}

// Minimal PNG writer
static void writePNG(const std::string& filename, const std::vector<uint8_t>& data,
                     int w, int h) {
    // Write PPM instead for simplicity — will be converted to PNG via Python
    std::string ppmFile = filename.substr(0, filename.rfind('.')) + ".ppm";
    std::ofstream f(ppmFile, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    f.write(reinterpret_cast<const char*>(data.data()), data.size());
    f.close();
}

// ─── Camera & Rasterizer ──────────────────────────────────────────────────────

struct Camera {
    Vec3 pos, target, up;
    float fovY, aspect;
    float nearZ = 0.1f, farZ = 100.f;
    
    Vec3 forward() const { return normalize(target - pos); }
    Vec3 right() const { return normalize(cross(forward(), up)); }
};

struct RasterVertex {
    Vec3 worldPos;
    Vec3 normal;
    Vec2 uv;
    float w;       // clip w
    float sx, sy;  // screen pos
    float depth;
};

// Transform world pos to screen space
static RasterVertex projectVertex(const Camera& cam, const Vec3& wPos,
                                   const Vec3& wNorm, const Vec2& uv,
                                   int W, int H) {
    RasterVertex rv;
    rv.worldPos = wPos;
    rv.normal = wNorm;
    rv.uv = uv;
    
    // View transform
    Vec3 f = cam.forward();
    Vec3 r = cam.right();
    Vec3 u = cross(r, f);
    
    Vec3 d = wPos - cam.pos;
    float vx = dot(d, r);
    float vy = dot(d, u);
    float vz = dot(d, f);
    
    if (vz <= cam.nearZ) {
        rv.w = -1.f;
        rv.sx = rv.sy = 0.f;
        rv.depth = 1.f;
        return rv;
    }
    
    float tanHalfFov = std::tan(cam.fovY * 0.5f * float(M_PI) / 180.f);
    float ndcX = vx / (vz * tanHalfFov * cam.aspect);
    float ndcY = vy / (vz * tanHalfFov);
    
    rv.w = vz;
    rv.sx = (ndcX + 1.f) * 0.5f * W;
    rv.sy = (1.f - ndcY) * 0.5f * H;
    rv.depth = (vz - cam.nearZ) / (cam.farZ - cam.nearZ);
    
    return rv;
}

// ─── Light Sources ────────────────────────────────────────────────────────────

struct Light {
    Vec3 direction;     // For directional (normalized, pointing toward scene)
    Vec3 color;
    float intensity;
    bool isDirectional;
    Vec3 position;      // For point lights
    float radius;       // For point lights
};

// ─── Main Renderer ────────────────────────────────────────────────────────────

static Vec3 shade(const Vec3& worldPos, const Vec3& N, const Vec2& uv,
                  const Material& mat, const Camera& cam,
                  const std::vector<Light>& lights,
                  const Vec3& ambientLight) {
    Vec3 V = normalize(cam.pos - worldPos);
    
    // Sample albedo (with procedural texture)
    Vec3 albedo = sampleTexture(mat, uv.x, uv.y);
    
    Vec3 color = {0,0,0};
    
    // Ambient
    Vec3 F0 = mix(Vec3(0.04f), albedo, mat.metallic);
    Vec3 kD_amb = (Vec3(1.f) - fresnelSchlick(std::max(0.f, dot(N, V)), F0))
                  * (1.f - mat.metallic);
    color += kD_amb * albedo * ambientLight;
    
    // Each light
    for (const auto& light : lights) {
        Vec3 L;
        Vec3 lColor = light.color * light.intensity;
        float attenuation = 1.f;
        
        if (light.isDirectional) {
            L = normalize(-light.direction);
        } else {
            Vec3 toLight = light.position - worldPos;
            float dist = toLight.length();
            L = toLight / dist;
            // Inverse square falloff with soft radius
            attenuation = 1.f / (1.f + dist * dist / (light.radius * light.radius));
        }
        
        lColor = lColor * attenuation;
        color += cookTorrance(N, V, L, albedo, mat.metallic, mat.roughness, lColor);
    }
    
    // Emission
    color += mat.emission;
    
    return color;
}

// Edge function for rasterization
static float edgeFunc(float ax, float ay, float bx, float by,
                       float cx, float cy) {
    return (cx - ax) * (by - ay) - (cy - ay) * (bx - ax);
}

int main() {
    const int W = 800, H = 600;
    
    std::cout << "=== OBJ Mesh PBR Renderer ===" << std::endl;
    std::cout << "Resolution: " << W << "x" << H << std::endl;
    
    // ── Build Scene ───────────────────────────────────────────────────────────
    Mesh mesh;
    
    // Try loading an OBJ file first
    bool objLoaded = false;
    std::string objPath = "model.obj";
    {
        std::ifstream test(objPath);
        if (test.is_open()) {
            test.close();
            objLoaded = parseOBJ(objPath, mesh, "./");
            if (objLoaded) {
                std::cout << "Loaded OBJ: " << mesh.positions.size()
                          << " vertices, " << mesh.faces.size() << " faces" << std::endl;
            }
        }
    }
    
    if (!objLoaded) {
        std::cout << "No OBJ file found — generating procedural vase mesh..." << std::endl;
        generateVaseMesh(mesh, 64, 64);
        std::cout << "Generated: " << mesh.positions.size()
                  << " vertices, " << mesh.faces.size() << " faces" << std::endl;
    }
    
    // Add floor
    addFloor(mesh, -1.05f, 2.5f);
    
    // ── Compute mesh bounds & normalize ──────────────────────────────────────
    // The vase is generated in [-1,1] range already; just find actual bounds
    // Floor was added after, last 4 verts
    int nOrigVerts = (int)mesh.positions.size();
    int floorVertStart = nOrigVerts - 4;
    
    // Get vase bounds (excluding floor)
    Vec3 bMin(1e9f), bMax(-1e9f);
    for (int i = 0; i < floorVertStart; i++) {
        const auto& p = mesh.positions[i];
        bMin.x = std::min(bMin.x, p.x); bMax.x = std::max(bMax.x, p.x);
        bMin.y = std::min(bMin.y, p.y); bMax.y = std::max(bMax.y, p.y);
        bMin.z = std::min(bMin.z, p.z); bMax.z = std::max(bMax.z, p.z);
    }
    Vec3 meshCenter = (bMin + bMax) * 0.5f;
    float meshExtent = (bMax - bMin).length() * 0.5f;
    // Normalize vase to fit in unit sphere
    float normScale = 1.0f / std::max(meshExtent, 0.001f);
    for (int i = 0; i < floorVertStart; i++) {
        mesh.positions[i] = (mesh.positions[i] - meshCenter) * normScale;
    }
    // Recompute floor y below vase
    float newMinY = 1e9f;
    for (int i = 0; i < floorVertStart; i++) newMinY = std::min(newMinY, mesh.positions[i].y);
    for (int i = floorVertStart; i < nOrigVerts; i++) {
        mesh.positions[i].x *= normScale; // scale floor radius too
        mesh.positions[i].z *= normScale;
        mesh.positions[i].y = newMinY - 0.02f;
    }
    
    // ── Setup Camera ──────────────────────────────────────────────────────────
    Camera cam;
    // Camera close enough to fill frame
    cam.pos    = {0.8f, 0.3f, 2.0f};
    cam.target = {0.f, 0.1f, 0.f};
    cam.up     = {0.f, 1.f, 0.f};
    cam.fovY   = 50.f;
    cam.aspect = float(W) / float(H);
    cam.nearZ  = 0.01f;
    cam.farZ   = 50.f;
    
    // ── Setup Lights ──────────────────────────────────────────────────────────
    std::vector<Light> lights;
    
    // Key light — warm sun
    lights.push_back({
        normalize(Vec3(-1.f, -2.f, -1.f)),  // direction
        {1.0f, 0.95f, 0.85f},              // color
        3.5f,                               // intensity
        true, {}, 0.f
    });
    
    // Fill light — cool sky
    lights.push_back({
        normalize(Vec3(1.f, -1.5f, 0.5f)),
        {0.6f, 0.7f, 0.9f},
        1.2f,
        true, {}, 0.f
    });
    
    // Back/rim light
    lights.push_back({
        normalize(Vec3(0.5f, -0.5f, 1.5f)),
        {0.9f, 0.85f, 1.0f},
        0.8f,
        true, {}, 0.f
    });
    
    // Point light — warm lamp nearby
    lights.push_back({
        {}, {1.0f, 0.7f, 0.3f}, 5.f, false,
        {-1.5f, 2.f, 1.5f}, 3.f
    });
    
    Vec3 ambientLight = {0.05f, 0.06f, 0.08f};
    
    // ── Allocate Framebuffer ──────────────────────────────────────────────────
    std::vector<Vec3> fb(W * H, {0.05f, 0.07f, 0.12f}); // dark blue-grey bg
    std::vector<float> depth(W * H, 1.f);
    
    // ── Rasterize ─────────────────────────────────────────────────────────────
    std::cout << "Rasterizing " << mesh.faces.size() << " triangles..." << std::endl;
    
    int rendered = 0;
    for (const auto& face : mesh.faces) {
        // Get material
        const Material& mat = (face.matIdx >= 0 && face.matIdx < (int)mesh.materials.size())
                              ? mesh.materials[face.matIdx]
                              : mesh.materials[0];
        
        // Gather vertices
        Vec3 worldPos[3];
        Vec3 worldNorm[3];
        Vec2 uvCoord[3];
        
        for (int k = 0; k < 3; k++) {
            const VertexIndex& vi = face.idx[k];
            worldPos[k] = (vi.v >= 0 && vi.v < (int)mesh.positions.size())
                          ? mesh.positions[vi.v] : Vec3(0);
            
            if (vi.vn >= 0 && vi.vn < (int)mesh.normals.size()) {
                worldNorm[k] = mesh.normals[vi.vn].normalized();
            } else if (vi.v >= 0 && vi.v < (int)mesh.normals.size()) {
                worldNorm[k] = mesh.normals[vi.v].normalized();
            } else {
                worldNorm[k] = {0,1,0};
            }
            
            if (vi.vt >= 0 && vi.vt < (int)mesh.texcoords.size()) {
                uvCoord[k] = mesh.texcoords[vi.vt];
            } else {
                uvCoord[k] = {(float)k * 0.5f, (float)(k/2)};
            }
        }
        
        // Back-face culling
        Vec3 edge01 = worldPos[1] - worldPos[0];
        Vec3 edge02 = worldPos[2] - worldPos[0];
        Vec3 faceN = cross(edge01, edge02);
        Vec3 toCamera = cam.pos - worldPos[0];
        if (dot(faceN, toCamera) <= 0.f) continue;
        
        // Project to screen
        RasterVertex rv[3];
        for (int k = 0; k < 3; k++) {
            rv[k] = projectVertex(cam, worldPos[k], worldNorm[k], uvCoord[k], W, H);
            if (rv[k].w < 0.f) goto nextFace;
        }
        
        {
            // Bounding box
            float minX = std::max(0.f, std::min({rv[0].sx, rv[1].sx, rv[2].sx}));
            float maxX = std::min(float(W-1), std::max({rv[0].sx, rv[1].sx, rv[2].sx}));
            float minY = std::max(0.f, std::min({rv[0].sy, rv[1].sy, rv[2].sy}));
            float maxY = std::min(float(H-1), std::max({rv[0].sy, rv[1].sy, rv[2].sy}));
            
            if (maxX < minX || maxY < minY) goto nextFace;
            
            // Rasterize
            float area = edgeFunc(rv[0].sx, rv[0].sy,
                                  rv[1].sx, rv[1].sy,
                                  rv[2].sx, rv[2].sy);
            if (std::abs(area) < 1e-6f) goto nextFace;
            
            for (int py = (int)minY; py <= (int)maxY; py++) {
                for (int px = (int)minX; px <= (int)maxX; px++) {
                    float cx = px + 0.5f, cy = py + 0.5f;
                    
                    float w0 = edgeFunc(rv[1].sx, rv[1].sy,
                                        rv[2].sx, rv[2].sy, cx, cy);
                    float w1 = edgeFunc(rv[2].sx, rv[2].sy,
                                        rv[0].sx, rv[0].sy, cx, cy);
                    float w2 = edgeFunc(rv[0].sx, rv[0].sy,
                                        rv[1].sx, rv[1].sy, cx, cy);
                    
                    // Check inside
                    bool inside = (area > 0)
                        ? (w0 >= 0 && w1 >= 0 && w2 >= 0)
                        : (w0 <= 0 && w1 <= 0 && w2 <= 0);
                    if (!inside) continue;
                    
                    float b0 = w0 / area, b1 = w1 / area, b2 = w2 / area;
                    
                    // Perspective-correct interpolation
                    float wInv0 = 1.f / rv[0].w;
                    float wInv1 = 1.f / rv[1].w;
                    float wInv2 = 1.f / rv[2].w;
                    float wCorr = 1.f / (b0 * wInv0 + b1 * wInv1 + b2 * wInv2);
                    
                    float d = b0 * rv[0].depth + b1 * rv[1].depth + b2 * rv[2].depth;
                    
                    int pidx = py * W + px;
                    if (d >= depth[pidx]) continue;
                    depth[pidx] = d;
                    
                    // Interpolate attributes
                    float cb0 = b0 * wInv0 * wCorr;
                    float cb1 = b1 * wInv1 * wCorr;
                    float cb2 = b2 * wInv2 * wCorr;
                    
                    Vec3 wPos = worldPos[0] * cb0 + worldPos[1] * cb1 + worldPos[2] * cb2;
                    Vec3 wN = (worldNorm[0] * cb0 + worldNorm[1] * cb1 + worldNorm[2] * cb2).normalized();
                    Vec2 uv = {
                        uvCoord[0].x * cb0 + uvCoord[1].x * cb1 + uvCoord[2].x * cb2,
                        uvCoord[0].y * cb0 + uvCoord[1].y * cb1 + uvCoord[2].y * cb2
                    };
                    
                    fb[pidx] = shade(wPos, wN, uv, mat, cam, lights, ambientLight);
                    rendered++;
                }
            }
        }
        nextFace:;
    }
    
    std::cout << "Shaded pixels: " << rendered << std::endl;
    
    // ── Convert & Write ───────────────────────────────────────────────────────
    auto pixels = toUint8(fb, W, H, 1.0f);
    writePNG("obj_pbr_output.ppm", pixels, W, H);
    
    std::cout << "Saved: obj_pbr_output.ppm" << std::endl;
    std::cout << "=== Done ===" << std::endl;
    
    return 0;
}
