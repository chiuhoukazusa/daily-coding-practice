/*
 * Screen Space Ambient Occlusion (SSAO) Renderer
 * 
 * 技术要点：
 * - 软光栅化 3D 场景（Cornell Box + 多个球体）
 * - G-Buffer：深度、法线、颜色
 * - SSAO：半球采样核、随机旋转噪声、遮蔽积分
 * - 模糊Pass（Box blur平滑 SSAO 结果）
 * - 最终合成：环境光 × (1 - occlusion) + 漫射光 + 高光
 *
 * 输出：ssao_output.png （800x600）
 *   左半部：无 SSAO 的普通光照
 *   右半部：加了 SSAO 的光照（可见角落/缝隙变暗）
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

// ─── math ───────────────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& v) const { return {x*v.x, y*v.y, z*v.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    float dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3  cross(const Vec3& v) const {
        return {y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3  normalized() const {
        float l = length();
        return l > 1e-8f ? (*this / l) : Vec3(0,0,0);
    }
    float& operator[](int i) { return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }
};
inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b-a)*t; }
inline float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

// ─── camera / transforms ────────────────────────────────────────────────────

struct Mat4 {
    float m[4][4]{};
    static Mat4 identity() {
        Mat4 r; for(int i=0;i<4;i++) r.m[i][i]=1; return r;
    }
    Vec3 transformPoint(const Vec3& p) const {
        float w = m[3][0]*p.x + m[3][1]*p.y + m[3][2]*p.z + m[3][3];
        return {
            (m[0][0]*p.x + m[0][1]*p.y + m[0][2]*p.z + m[0][3]) / w,
            (m[1][0]*p.x + m[1][1]*p.y + m[1][2]*p.z + m[1][3]) / w,
            (m[2][0]*p.x + m[2][1]*p.y + m[2][2]*p.z + m[2][3]) / w
        };
    }
    Vec3 transformDir(const Vec3& d) const {
        return {
            m[0][0]*d.x + m[0][1]*d.y + m[0][2]*d.z,
            m[1][0]*d.x + m[1][1]*d.y + m[1][2]*d.z,
            m[2][0]*d.x + m[2][1]*d.y + m[2][2]*d.z
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++)
            for(int k=0;k<4;k++) r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 perspective(float fovY, float aspect, float nearZ, float farZ) {
    Mat4 r;
    float f = 1.0f / std::tan(fovY * 0.5f);
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (farZ + nearZ) / (nearZ - farZ);
    r.m[2][3] = (2.0f * farZ * nearZ) / (nearZ - farZ);
    r.m[3][2] = -1.0f;
    return r;
}

Mat4 lookAt(const Vec3& eye, const Vec3& at, const Vec3& up) {
    Vec3 z = (eye - at).normalized();
    Vec3 x = up.cross(z).normalized();
    Vec3 y = z.cross(x);
    Mat4 r = Mat4::identity();
    r.m[0][0]=x.x; r.m[0][1]=x.y; r.m[0][2]=x.z; r.m[0][3]=-x.dot(eye);
    r.m[1][0]=y.x; r.m[1][1]=y.y; r.m[1][2]=y.z; r.m[1][3]=-y.dot(eye);
    r.m[2][0]=z.x; r.m[2][1]=z.y; r.m[2][2]=z.z; r.m[2][3]=-z.dot(eye);
    r.m[3][3]=1;
    return r;
}

// ─── GBuffer ────────────────────────────────────────────────────────────────

const int W = 800, H = 600;

struct GBuffer {
    std::vector<float> depth;   // view-space depth (positive = in front)
    std::vector<Vec3>  normal;  // view-space normal
    std::vector<Vec3>  posView; // view-space position
    std::vector<Vec3>  albedo;  // diffuse color
    std::vector<float> ssao;    // occlusion [0,1]
    std::vector<float> ssaoBlur;

    GBuffer() : depth(W*H, -1e9f),
                normal(W*H, Vec3(0,0,0)),
                posView(W*H, Vec3(0,0,0)),
                albedo(W*H, Vec3(0,0,0)),
                ssao(W*H, 0.0f),
                ssaoBlur(W*H, 0.0f) {}
};

// ─── Rasterizer primitives ──────────────────────────────────────────────────

struct Vertex {
    Vec3 worldPos;
    Vec3 normal;
    Vec3 albedo;
};

// rasterize a triangle, write into GBuffer using view/proj transforms
void rasterizeTriangle(
    GBuffer& gb,
    const Mat4& view, const Mat4& proj,
    const Vertex& v0, const Vertex& v1, const Vertex& v2)
{
    // transform to view space
    Vec3 vp0 = view.transformPoint(v0.worldPos);
    Vec3 vp1 = view.transformPoint(v1.worldPos);
    Vec3 vp2 = view.transformPoint(v2.worldPos);

    // transform normals (no scale, so just rotate)
    Vec3 vn0 = view.transformDir(v0.normal).normalized();
    Vec3 vn1 = view.transformDir(v1.normal).normalized();
    Vec3 vn2 = view.transformDir(v2.normal).normalized();

    // clip (simple near-plane cull)
    float nearZ = -0.1f;
    if(vp0.z > nearZ && vp1.z > nearZ && vp2.z > nearZ) return;

    // project to NDC
    auto toNDC = [&](const Vec3& vp) -> Vec3 {
        // manual projection: proj * vec4(vp, 1)
        float x_ = proj.m[0][0]*vp.x                 + proj.m[0][3];
        float y_ =                 proj.m[1][1]*vp.y  + proj.m[1][3];
        float z_ =                                proj.m[2][2]*vp.z + proj.m[2][3];
        float w_ =                               -vp.z; // proj.m[3][2] = -1
        if(std::abs(w_) < 1e-7f) w_ = 1e-7f;
        return {x_/w_, y_/w_, z_/w_};
    };

    Vec3 n0 = toNDC(vp0);
    Vec3 n1 = toNDC(vp1);
    Vec3 n2 = toNDC(vp2);

    // NDC -> screen
    auto toScreen = [&](const Vec3& n) -> Vec3 {
        return { (n.x * 0.5f + 0.5f) * W, (n.y * 0.5f + 0.5f) * H, n.z };
    };

    Vec3 s0 = toScreen(n0);
    Vec3 s1 = toScreen(n1);
    Vec3 s2 = toScreen(n2);

    // AABB
    int minX = std::max(0,   (int)std::min({s0.x, s1.x, s2.x}));
    int maxX = std::min(W-1, (int)std::max({s0.x, s1.x, s2.x})+1);
    int minY = std::max(0,   (int)std::min({s0.y, s1.y, s2.y}));
    int maxY = std::min(H-1, (int)std::max({s0.y, s1.y, s2.y})+1);

    auto edge = [](const Vec3& a, const Vec3& b, const Vec3& p) {
        return (p.x - a.x)*(b.y - a.y) - (p.y - a.y)*(b.x - a.x);
    };

    float area = edge(s0, s1, s2);
    if(std::abs(area) < 1e-5f) return;

    for(int py = minY; py <= maxY; py++) {
        for(int px = minX; px <= maxX; px++) {
            Vec3 p{(float)px + 0.5f, (float)py + 0.5f, 0};
            float w0 = edge(s1, s2, p) / area;
            float w1 = edge(s2, s0, p) / area;
            float w2 = edge(s0, s1, p) / area;
            if(w0 < 0 || w1 < 0 || w2 < 0) continue;

            // perspective-correct interp of 1/w
            float invW0 = 1.0f / (-vp0.z);
            float invW1 = 1.0f / (-vp1.z);
            float invW2 = 1.0f / (-vp2.z);
            float invW = w0*invW0 + w1*invW1 + w2*invW2;

            // interpolate view-space depth
            float depth = 1.0f / invW;  // view-space depth (positive)

            int idx = py * W + px;
            if(depth < gb.depth[idx]) continue; // keep closest (largest depth from camera)
            gb.depth[idx] = depth;

            // interpolate view-space position
            Vec3 posV = (vp0 * (w0*invW0) + vp1 * (w1*invW1) + vp2 * (w2*invW2)) * depth;
            gb.posView[idx] = posV;

            // interpolate normal
            Vec3 nrmV = (vn0 * (w0*invW0) + vn1 * (w1*invW1) + vn2 * (w2*invW2)) * depth;
            gb.normal[idx] = nrmV.normalized();

            // interpolate albedo
            Vec3 alb = (v0.albedo*(w0*invW0) + v1.albedo*(w1*invW1) + v2.albedo*(w2*invW2)) * depth;
            gb.albedo[idx] = alb;
        }
    }
}

// ─── Scene: Cornell Box + 3 spheres ─────────────────────────────────────────

void addQuad(std::vector<std::array<Vertex,3>>& tris,
             Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3,
             Vec3 normal, Vec3 albedo)
{
    tris.push_back({ Vertex{p0,normal,albedo}, Vertex{p1,normal,albedo}, Vertex{p2,normal,albedo} });
    tris.push_back({ Vertex{p0,normal,albedo}, Vertex{p2,normal,albedo}, Vertex{p3,normal,albedo} });
}

// Sphere triangulation
void addSphere(std::vector<std::array<Vertex,3>>& tris,
               Vec3 center, float radius, Vec3 albedo, int stacks=24, int slices=24)
{
    for(int i=0; i<stacks; i++) {
        float phi0 = float(M_PI) * i / stacks;
        float phi1 = float(M_PI) * (i+1) / stacks;
        for(int j=0; j<slices; j++) {
            float th0 = 2*float(M_PI) * j / slices;
            float th1 = 2*float(M_PI) * (j+1) / slices;

            auto vtx = [&](float phi, float theta) -> Vertex {
                Vec3 n{ std::sin(phi)*std::cos(theta),
                        std::cos(phi),
                        std::sin(phi)*std::sin(theta) };
                return Vertex{ center + n * radius, n, albedo };
            };

            Vertex a = vtx(phi0, th0), b = vtx(phi0, th1);
            Vertex c = vtx(phi1, th0), d = vtx(phi1, th1);

            if(i > 0) tris.push_back({a, c, b});
            if(i < stacks-1) tris.push_back({b, c, d});
        }
    }
}

std::vector<std::array<Vertex,3>> buildScene() {
    std::vector<std::array<Vertex,3>> tris;

    float s = 5.0f; // half-size of box

    // Floor  (y = -s)
    addQuad(tris, {-s,-s,-s*2}, {s,-s,-s*2}, {s,-s,0}, {-s,-s,0},
            {0,1,0}, {0.85f,0.85f,0.85f});
    // Ceiling (y = +s)
    addQuad(tris, {-s,s,-s*2}, {s,s,-s*2}, {s,s,0}, {-s,s,0},
            {0,-1,0}, {0.85f,0.85f,0.85f});
    // Back wall (z = -s*2)
    addQuad(tris, {-s,-s,-s*2}, {s,-s,-s*2}, {s,s,-s*2}, {-s,s,-s*2},
            {0,0,1}, {0.85f,0.85f,0.85f});
    // Left wall  (x = -s, red)
    addQuad(tris, {-s,-s,-s*2}, {-s,-s,0}, {-s,s,0}, {-s,s,-s*2},
            {1,0,0}, {0.85f,0.15f,0.15f});
    // Right wall (x = +s, green)
    addQuad(tris, {s,-s,-s*2}, {s,s,-s*2}, {s,s,0}, {s,-s,0},
            {-1,0,0}, {0.15f,0.85f,0.15f});

    // Tall box
    float bx = -2.0f, bz = -7.0f, bw = 1.5f, bh = 3.5f;
    Vec3 wb{0.8f,0.8f,0.8f};
    // front
    addQuad(tris, {bx-bw,-s,bz+bw},{bx+bw,-s,bz+bw},{bx+bw,-s+bh*2,bz+bw},{bx-bw,-s+bh*2,bz+bw}, {0,0,1}, wb);
    // back
    addQuad(tris, {bx+bw,-s,bz-bw},{bx-bw,-s,bz-bw},{bx-bw,-s+bh*2,bz-bw},{bx+bw,-s+bh*2,bz-bw}, {0,0,-1}, wb);
    // left
    addQuad(tris, {bx-bw,-s,bz-bw},{bx-bw,-s,bz+bw},{bx-bw,-s+bh*2,bz+bw},{bx-bw,-s+bh*2,bz-bw}, {-1,0,0}, wb);
    // right
    addQuad(tris, {bx+bw,-s,bz+bw},{bx+bw,-s,bz-bw},{bx+bw,-s+bh*2,bz-bw},{bx+bw,-s+bh*2,bz+bw}, {1,0,0}, wb);
    // top
    addQuad(tris, {bx-bw,-s+bh*2,bz-bw},{bx+bw,-s+bh*2,bz-bw},{bx+bw,-s+bh*2,bz+bw},{bx-bw,-s+bh*2,bz+bw}, {0,1,0}, wb);

    // Spheres
    addSphere(tris, {2.0f, -s+1.2f, -6.0f},  1.2f, {0.85f,0.75f,0.20f});  // gold
    addSphere(tris, {-1.0f,-s+0.8f, -4.0f},  0.8f, {0.20f,0.55f,0.90f});  // blue
    addSphere(tris, {3.5f, -s+0.5f, -4.5f},  0.5f, {0.90f,0.35f,0.20f});  // red-small

    return tris;
}

// ─── SSAO ───────────────────────────────────────────────────────────────────

// Generate hemisphere sample kernel (in tangent space)
std::vector<Vec3> genSSAOKernel(int n, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    std::vector<Vec3> kernel;
    kernel.reserve(n);
    for(int i=0; i<n; i++) {
        Vec3 s{
            dist(rng)*2.0f - 1.0f,
            dist(rng)*2.0f - 1.0f,
            dist(rng)          // z > 0 (hemisphere)
        };
        s = s.normalized();
        s = s * dist(rng);
        // accelerating interpolation (more samples near origin)
        float scale = float(i) / n;
        scale = 0.1f + scale*scale * 0.9f;
        s = s * scale;
        kernel.push_back(s);
    }
    return kernel;
}

// Random rotation vectors (noise texture, 4x4 tiled)
std::vector<Vec3> genSSAONoise(int size, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::vector<Vec3> noise(size*size);
    for(auto& v : noise) {
        v = Vec3{ dist(rng), dist(rng), 0.0f }.normalized();
    }
    return noise;
}

void computeSSAO(GBuffer& gb, const Mat4& proj,
                 const std::vector<Vec3>& kernel,
                 const std::vector<Vec3>& noise,
                 float radius = 0.5f, float bias = 0.025f)
{
    const int noiseSize = 4;
    const int nSamples = (int)kernel.size();

    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            int idx = py*W + px;
            if(gb.depth[idx] < -1e8f) { gb.ssao[idx] = 0.0f; continue; }

            Vec3 fragPos = gb.posView[idx];
            Vec3 normal  = gb.normal[idx];
            if(normal.length() < 0.5f) { gb.ssao[idx] = 0.0f; continue; }

            // random rotation vector (tiled noise)
            const Vec3& randomVec = noise[(py%noiseSize)*noiseSize + (px%noiseSize)];

            // TBN matrix (tangent space → view space)
            Vec3 tangent   = (randomVec - normal * normal.dot(randomVec)).normalized();
            Vec3 bitangent = normal.cross(tangent);
            // TBN cols: tangent, bitangent, normal
            // to transform sample from tangent→view: sample.x*tangent + sample.y*bitangent + sample.z*normal
            auto TBN = [&](const Vec3& v) -> Vec3 {
                return tangent * v.x + bitangent * v.y + normal * v.z;
            };

            float occlusion = 0.0f;
            for(int i = 0; i < nSamples; i++) {
                // transform sample position
                Vec3 samplePos = fragPos + TBN(kernel[i]) * radius;

                // project sample position to get texture coords
                // proj * vec4(samplePos, 1)
                float x_ = proj.m[0][0]*samplePos.x + proj.m[0][3];
                float y_ = proj.m[1][1]*samplePos.y + proj.m[1][3];
                float w_ = -samplePos.z;
                if(w_ < 1e-5f) continue;
                float ndcX = x_/w_; float ndcY = y_/w_;

                int sx = (int)((ndcX * 0.5f + 0.5f) * W);
                int sy = (int)((ndcY * 0.5f + 0.5f) * H);
                sx = std::max(0, std::min(W-1, sx));
                sy = std::max(0, std::min(H-1, sy));

                float sampleDepth = gb.depth[sy*W + sx];  // positive view depth
                if(sampleDepth < -1e8f) continue;  // background

                // range check + bias
                float rangeCheck = 1.0f - std::min(1.0f, std::abs(-fragPos.z - sampleDepth) / radius);
                if(sampleDepth >= -samplePos.z + bias) {
                    occlusion += rangeCheck;
                }
            }
            gb.ssao[idx] = occlusion / nSamples;
        }
    }
}

void blurSSAO(GBuffer& gb, int radius = 2) {
    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            float sum = 0.0f; int cnt = 0;
            for(int dy = -radius; dy <= radius; dy++) {
                for(int dx = -radius; dx <= radius; dx++) {
                    int nx = px+dx, ny = py+dy;
                    if(nx<0||nx>=W||ny<0||ny>=H) continue;
                    sum += gb.ssao[ny*W+nx]; cnt++;
                }
            }
            gb.ssaoBlur[py*W+px] = cnt > 0 ? sum/cnt : 0.0f;
        }
    }
}

// ─── Lighting ───────────────────────────────────────────────────────────────

Vec3 shade(const GBuffer& gb, int idx, const Mat4& view,
           float occFactor, bool useSSAO)
{
    if(gb.depth[idx] < -1e8f) return {0.05f, 0.05f, 0.08f}; // background

    Vec3 albedo  = gb.albedo[idx];
    Vec3 normal  = gb.normal[idx];
    Vec3 posV    = gb.posView[idx];

    // Light in view space (came from world {2, 8, -2})
    Vec3 lightWorldPos{2.0f, 8.0f, -2.0f};
    Vec3 lightViewPos = view.transformPoint(lightWorldPos);
    Vec3 lightDir = (lightViewPos - posV).normalized();

    float diff = std::max(0.0f, normal.dot(lightDir));

    // Specular (Blinn-Phong)
    Vec3 viewDir = (-posV).normalized();
    Vec3 half_ = (lightDir + viewDir).normalized();
    float spec = std::pow(std::max(0.0f, normal.dot(half_)), 32.0f) * 0.3f;

    float ao = useSSAO ? (1.0f - occFactor * 0.85f) : 1.0f;

    Vec3 ambient  = albedo * 0.3f * ao;
    Vec3 diffuse  = albedo * diff * 0.7f;
    Vec3 specular = Vec3{1.0f,1.0f,1.0f} * spec;

    Vec3 color = ambient + diffuse + specular;
    // tone-map
    color.x = color.x / (color.x + 1.0f);
    color.y = color.y / (color.y + 1.0f);
    color.z = color.z / (color.z + 1.0f);
    return color;
}

// ─── PNG writer (tiny, no libpng) ───────────────────────────────────────────

static uint32_t crc32Table[256];
static bool crcInit = false;
void initCRC() {
    if(crcInit) return;
    for(uint32_t i=0;i<256;i++){
        uint32_t c=i;
        for(int j=0;j<8;j++) c=(c&1)?(0xEDB88320u^(c>>1)):(c>>1);
        crc32Table[i]=c;
    }
    crcInit=true;
}
uint32_t crc32(const uint8_t* data, size_t len, uint32_t crc=0xFFFFFFFFu) {
    initCRC();
    for(size_t i=0;i<len;i++) crc=crc32Table[(crc^data[i])&0xFF]^(crc>>8);
    return crc^0xFFFFFFFFu;
}

void writeU32BE(std::vector<uint8_t>& v, uint32_t x) {
    v.push_back(x>>24); v.push_back((x>>16)&0xFF);
    v.push_back((x>>8)&0xFF); v.push_back(x&0xFF);
}

void appendChunk(std::vector<uint8_t>& png, const char* type,
                 const std::vector<uint8_t>& data) {
    writeU32BE(png, (uint32_t)data.size());
    png.push_back(type[0]); png.push_back(type[1]);
    png.push_back(type[2]); png.push_back(type[3]);
    png.insert(png.end(), data.begin(), data.end());
    std::vector<uint8_t> crcInput;
    crcInput.push_back(type[0]); crcInput.push_back(type[1]);
    crcInput.push_back(type[2]); crcInput.push_back(type[3]);
    crcInput.insert(crcInput.end(), data.begin(), data.end());
    writeU32BE(png, crc32(crcInput.data(), crcInput.size()));
}

// Simple deflate: uncompressed blocks
std::vector<uint8_t> deflateUncompressed(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> out;
    // zlib header
    out.push_back(0x78); out.push_back(0x01);
    size_t pos = 0;
    while(pos < data.size()) {
        size_t blockLen = std::min((size_t)65535, data.size() - pos);
        bool last = (pos + blockLen >= data.size());
        out.push_back(last ? 0x01 : 0x00);
        out.push_back(blockLen & 0xFF); out.push_back((blockLen>>8)&0xFF);
        uint16_t nlen = (uint16_t)(~(uint16_t)blockLen);
        out.push_back(nlen & 0xFF); out.push_back((nlen>>8)&0xFF);
        for(size_t i=0; i<blockLen; i++) out.push_back(data[pos+i]);
        pos += blockLen;
    }
    // adler32
    uint32_t s1=1, s2=0;
    for(auto b : data) { s1=(s1+b)%65521; s2=(s2+s1)%65521; }
    uint32_t adler = (s2<<16)|s1;
    out.push_back(adler>>24); out.push_back((adler>>16)&0xFF);
    out.push_back((adler>>8)&0xFF); out.push_back(adler&0xFF);
    return out;
}

void savePNG(const std::string& path, int w, int h,
             const std::vector<Vec3>& pixels) {
    std::vector<uint8_t> png;
    // signature
    uint8_t sig[] = {137,80,78,71,13,10,26,10};
    png.insert(png.end(), sig, sig+8);

    // IHDR
    std::vector<uint8_t> ihdr(13);
    ihdr[0]=(w>>24)&0xFF; ihdr[1]=(w>>16)&0xFF; ihdr[2]=(w>>8)&0xFF; ihdr[3]=w&0xFF;
    ihdr[4]=(h>>24)&0xFF; ihdr[5]=(h>>16)&0xFF; ihdr[6]=(h>>8)&0xFF; ihdr[7]=h&0xFF;
    ihdr[8]=8; ihdr[9]=2; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0;
    appendChunk(png, "IHDR", ihdr);

    // raw scanlines
    std::vector<uint8_t> raw;
    raw.reserve((1+w*3)*h);
    for(int y=h-1; y>=0; y--) {   // flip Y: screen top = image top
        raw.push_back(0); // filter byte
        for(int x=0; x<w; x++) {
            const Vec3& p = pixels[y*w+x];
            raw.push_back((uint8_t)(clamp01(p.x)*255.0f));
            raw.push_back((uint8_t)(clamp01(p.y)*255.0f));
            raw.push_back((uint8_t)(clamp01(p.z)*255.0f));
        }
    }

    auto compressed = deflateUncompressed(raw);
    appendChunk(png, "IDAT", compressed);

    // IEND
    appendChunk(png, "IEND", {});

    std::ofstream f(path, std::ios::binary);
    f.write((char*)png.data(), png.size());
    std::cout << "Saved " << path << " (" << png.size()/1024 << " KB)\n";
}

// ─── main ────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== Screen Space Ambient Occlusion Renderer ===\n";

    // Camera
    Vec3 eye{0.0f, 1.0f, 4.0f};
    Vec3 at {0.0f, 0.0f, -5.0f};
    Vec3 up {0.0f, 1.0f, 0.0f};
    Mat4 view = lookAt(eye, at, up);
    float fovY = 60.0f * float(M_PI) / 180.0f;
    float aspect = float(W) / float(H);
    Mat4 proj = perspective(fovY, aspect, 0.1f, 50.0f);

    // Build scene and rasterize into GBuffer
    std::cout << "Building scene...\n";
    auto tris = buildScene();
    GBuffer gb;

    std::cout << "Rasterizing " << tris.size() << " triangles...\n";
    for(auto& tri : tris) {
        rasterizeTriangle(gb, view, proj, tri[0], tri[1], tri[2]);
    }

    // SSAO
    std::cout << "Computing SSAO...\n";
    std::mt19937 rng(42);
    auto kernel = genSSAOKernel(64, rng);
    auto noise  = genSSAONoise(4, rng);
    computeSSAO(gb, proj, kernel, noise, /*radius=*/1.5f, /*bias=*/0.05f);
    blurSSAO(gb, /*radius=*/2);

    // Render two halves: left = no SSAO, right = with SSAO
    std::cout << "Shading...\n";
    std::vector<Vec3> pixels(W*H);
    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            int idx = py*W + px;
            bool useSSAO = (px >= W/2);
            float occ = gb.ssaoBlur[idx];
            pixels[idx] = shade(gb, idx, view, occ, useSSAO);
        }
    }

    // Draw dividing line
    for(int py = 0; py < H; py++) {
        int px = W/2;
        pixels[py*W + px] = Vec3{1.0f, 1.0f, 0.0f};
    }

    // Labels area — draw text indicator as simple pixel art blocks
    // (Just leave them for now; description in console)

    savePNG("/root/.openclaw/workspace/daily-coding-practice/2026-03-25-ssao/ssao_output.png", W, H, pixels);

    // Stats
    float totalOcc = 0.0f; int validPx = 0;
    for(int i=0; i<W*H; i++) {
        if(gb.depth[i] > -1e8f) { totalOcc += gb.ssaoBlur[i]; validPx++; }
    }
    std::cout << "Valid pixels: " << validPx << "/" << (W*H) << "\n";
    std::cout << "Average SSAO occlusion: " << totalOcc/validPx << "\n";
    std::cout << "Left half: no SSAO  |  Right half: with SSAO\n";
    std::cout << "Done!\n";
    return 0;
}
