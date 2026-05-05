/**
 * Order-Independent Transparency (OIT) Renderer
 * Technique: Weighted Blended OIT (McGuire & Bavoil 2013)
 *
 * Algorithm overview:
 * 1. Render opaque objects normally
 * 2. For each transparent fragment, compute weighted accumulation:
 *    - accum.rgb += color.rgb * alpha * weight(z, alpha)
 *    - accum.a   += alpha * weight(z, alpha)
 *    - reveal    *= (1 - alpha)
 * 3. Composite: dst = (accum.rgb / accum.a) * (1 - reveal) + bg * reveal
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <string>
#include <cstring>
#include <cassert>
#include <cstdio>

// ─────────────────────────── Math ───────────────────────────
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
    Vec3 normalize() const { float l = length(); return l > 0 ? *this / l : *this; }
    float& operator[](int i) { return i==0?x:(i==1?y:z); }
    float operator[](int i) const { return i==0?x:(i==1?y:z); }
};
inline Vec3 mix(const Vec3& a, const Vec3& b, float t) { return a*(1-t) + b*t; }
inline float clamp(float v, float lo, float hi) { return v<lo?lo:(v>hi?hi:v); }
inline Vec3 clamp3(const Vec3& v, float lo, float hi) {
    return {clamp(v.x,lo,hi), clamp(v.y,lo,hi), clamp(v.z,lo,hi)};
}

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(0) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(Vec3 v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    Vec3 xyz() const { return {x,y,z}; }
};

struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) {
            r.m[i][j]=0;
            for(int k=0;k<4;k++) r.m[i][j]+=m[i][k]*o.m[k][j];
        }
        return r;
    }
};

Mat4 perspective(float fovy, float aspect, float near, float far) {
    Mat4 r;
    float f = 1.0f / std::tan(fovy * 0.5f);
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (far + near) / (near - far);
    r.m[2][3] = 2.0f * far * near / (near - far);
    r.m[3][2] = -1.0f;
    return r;
}

Mat4 translate(Vec3 t) {
    Mat4 m = Mat4::identity();
    m.m[0][3]=t.x; m.m[1][3]=t.y; m.m[2][3]=t.z;
    return m;
}

Mat4 rotateY(float angle) {
    Mat4 m = Mat4::identity();
    float c=std::cos(angle), s=std::sin(angle);
    m.m[0][0]=c; m.m[0][2]=s; m.m[2][0]=-s; m.m[2][2]=c;
    return m;
}

Mat4 rotateX(float angle) {
    Mat4 m = Mat4::identity();
    float c=std::cos(angle), s=std::sin(angle);
    m.m[1][1]=c; m.m[1][2]=-s; m.m[2][1]=s; m.m[2][2]=c;
    return m;
}

Mat4 scaleM(Vec3 s) {
    Mat4 m = Mat4::identity();
    m.m[0][0]=s.x; m.m[1][1]=s.y; m.m[2][2]=s.z;
    return m;
}

// ─────────────────────────── Framebuffer ───────────────────────────
const int W = 800, H = 600;

struct OITBuffers {
    std::vector<Vec4> accum;   // .xyz = weighted color sum, .w = weighted alpha sum
    std::vector<float> reveal; // product of (1 - alpha)
    OITBuffers() : accum(W*H, {0,0,0,0}), reveal(W*H, 1.0f) {}
    void clear() {
        std::fill(accum.begin(), accum.end(), Vec4{0,0,0,0});
        std::fill(reveal.begin(), reveal.end(), 1.0f);
    }
};

struct Framebuffer {
    std::vector<Vec3> color;
    std::vector<float> depth;
    Framebuffer() : color(W*H, {0,0,0}), depth(W*H, 1e9f) {}
    void clear(Vec3 bg) {
        std::fill(color.begin(), color.end(), bg);
        std::fill(depth.begin(), depth.end(), 1e9f);
    }
};

// ─────────────────────────── Image writer ───────────────────────────
void writePPM(const char* fname, const std::vector<Vec3>& pixels) {
    FILE* f = fopen(fname, "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (const auto& p : pixels) {
        unsigned char r = (unsigned char)(clamp(p.x, 0, 1) * 255.99f);
        unsigned char g = (unsigned char)(clamp(p.y, 0, 1) * 255.99f);
        unsigned char b = (unsigned char)(clamp(p.z, 0, 1) * 255.99f);
        fwrite(&r, 1, 1, f);
        fwrite(&g, 1, 1, f);
        fwrite(&b, 1, 1, f);
    }
    fclose(f);
}

// ─────────────────────────── Geometry ───────────────────────────
struct Vertex {
    Vec3 pos;
    Vec3 normal;
};

struct Triangle {
    Vertex v[3];
};

struct Mesh {
    std::vector<Triangle> tris; // world-space triangles
    Vec3 color;
    float alpha;
    bool opaque;
};

// Build a unit quad facing +Z, centered at origin, size 2x2
std::vector<Triangle> buildQuad() {
    std::vector<Triangle> tris;
    Vec3 n = {0, 0, 1};
    Triangle t1, t2;
    t1.v[0] = {{-1,-1,0}, n}; t1.v[1] = {{ 1,-1,0}, n}; t1.v[2] = {{ 1, 1,0}, n};
    t2.v[0] = {{-1,-1,0}, n}; t2.v[1] = {{ 1, 1,0}, n}; t2.v[2] = {{-1, 1,0}, n};
    tris.push_back(t1);
    tris.push_back(t2);
    return tris;
}

// Build a unit cube [-1,1]^3
std::vector<Triangle> buildBox() {
    std::vector<Triangle> tris;
    struct Face { Vec3 n; Vec3 c[4]; };
    float h = 1.0f;
    Face faces[6] = {
        {{ 0, 0, 1}, {{-h,-h, h},{ h,-h, h},{ h, h, h},{-h, h, h}}},
        {{ 0, 0,-1}, {{ h,-h,-h},{-h,-h,-h},{-h, h,-h},{ h, h,-h}}},
        {{ 1, 0, 0}, {{ h,-h, h},{ h,-h,-h},{ h, h,-h},{ h, h, h}}},
        {{-1, 0, 0}, {{-h,-h,-h},{-h,-h, h},{-h, h, h},{-h, h,-h}}},
        {{ 0, 1, 0}, {{-h, h, h},{ h, h, h},{ h, h,-h},{-h, h,-h}}},
        {{ 0,-1, 0}, {{-h,-h,-h},{ h,-h,-h},{ h,-h, h},{-h,-h, h}}},
    };
    for (auto& f : faces) {
        Triangle t1, t2;
        t1.v[0]={f.c[0],f.n}; t1.v[1]={f.c[1],f.n}; t1.v[2]={f.c[2],f.n};
        t2.v[0]={f.c[0],f.n}; t2.v[1]={f.c[2],f.n}; t2.v[2]={f.c[3],f.n};
        tris.push_back(t1); tris.push_back(t2);
    }
    return tris;
}

// Apply model matrix to geometry, returning world-space mesh
Mesh buildMesh(const std::vector<Triangle>& src, const Mat4& model,
               Vec3 color, float alpha, bool opaque) {
    Mesh mesh;
    mesh.color = color;
    mesh.alpha = alpha;
    mesh.opaque = opaque;
    for (const auto& tri : src) {
        Triangle t;
        for (int i = 0; i < 3; i++) {
            Vec4 p = model * Vec4(tri.v[i].pos, 1.0f);
            Vec4 n = model * Vec4(tri.v[i].normal, 0.0f);
            t.v[i].pos = p.xyz();
            t.v[i].normal = n.xyz().normalize();
        }
        mesh.tris.push_back(t);
    }
    return mesh;
}

// ─────────────────────────── Rasterizer ───────────────────────────
inline float edgeFunc(float ax, float ay, float bx, float by, float px, float py) {
    return (px - ax) * (by - ay) - (py - ay) * (bx - ax);
}

// OIT weight function (depth and alpha dependent)
float oitWeight(float z, float alpha) {
    // Clamp z to [0,1]
    z = clamp(z, 0.0f, 1.0f);
    float dz = z;
    // Weight function from the original paper
    float w = alpha * std::max(1e-2f, std::min(3e3f, 0.03f / (1e-5f + dz * dz * dz * dz)));
    return w;
}

struct Light {
    Vec3 dir;    // normalized, world space
    Vec3 color;
    float ambient;
};

Vec3 shade(Vec3 albedo, Vec3 normal, Vec3 viewDir, const Light& light) {
    Vec3 n = normal.normalize();
    // Lighting in world space
    float NdotL = std::max(0.0f, n.dot(light.dir));
    Vec3 diffuse = albedo * light.color * NdotL;
    Vec3 ambient = albedo * light.ambient;
    Vec3 h = (light.dir + viewDir).normalize();
    float spec = std::pow(std::max(0.0f, n.dot(h)), 32.0f) * 0.4f;
    Vec3 specular = light.color * spec;
    return clamp3(ambient + diffuse + specular, 0, 1);
}

struct ScreenTri {
    Vec3 ndc[3];  // NDC coords (x,y,z)
    Vec3 norm[3]; // world-space normals
    float sx[3], sy[3]; // screen coords
    bool valid;
};

ScreenTri projectTriangle(const Triangle& tri, const Mat4& VP) {
    ScreenTri st;
    st.valid = true;
    for (int i = 0; i < 3; i++) {
        Vec4 clip = VP * Vec4(tri.v[i].pos, 1.0f);
        if (std::abs(clip.w) < 1e-7f) { st.valid = false; return st; }
        st.ndc[i] = clip.xyz() / clip.w;
        // Clip check
        if (st.ndc[i].z < -1.0f || st.ndc[i].z > 1.0f) { st.valid = false; return st; }
        st.sx[i] = (st.ndc[i].x * 0.5f + 0.5f) * (W - 1);
        st.sy[i] = (1.0f - (st.ndc[i].y * 0.5f + 0.5f)) * (H - 1);
        st.norm[i] = tri.v[i].normal;
    }
    return st;
}

void rasterizeOpaqueTriangle(const ScreenTri& st, Vec3 color, bool backfaceCull,
                              Framebuffer& fb, const Light& light, Vec3 viewDir) {
    // Check winding using area
    float e1x = st.sx[1]-st.sx[0], e1y = st.sy[1]-st.sy[0];
    float e2x = st.sx[2]-st.sx[0], e2y = st.sy[2]-st.sy[0];
    float area = e1x * e2y - e1y * e2x;
    // Backface cull: front faces have area < 0 in screen space (due to y-flip from NDC)
    if (backfaceCull && area > 0) return;
    if (std::abs(area) < 1e-8f) return;
    // signFactor: normalize so inside points give positive barycentric weights
    // In screen-space (y-down): CCW triangles (area<0) have ef>0 inside, CW (area>0) have ef<0 inside
    float signFactor = (area > 0) ? -1.0f : 1.0f;

    int minX = (int)std::max(0.0f, std::min({st.sx[0],st.sx[1],st.sx[2]}));
    int maxX = (int)std::min((float)(W-1), std::max({st.sx[0],st.sx[1],st.sx[2]}));
    int minY = (int)std::max(0.0f, std::min({st.sy[0],st.sy[1],st.sy[2]}));
    int maxY = (int)std::min((float)(H-1), std::max({st.sy[0],st.sy[1],st.sy[2]}));

    for (int py = minY; py <= maxY; py++) {
        for (int px = minX; px <= maxX; px++) {
            float w0 = edgeFunc(st.sx[1], st.sy[1], st.sx[2], st.sy[2], (float)px, (float)py) * signFactor;
            float w1 = edgeFunc(st.sx[2], st.sy[2], st.sx[0], st.sy[0], (float)px, (float)py) * signFactor;
            float w2 = edgeFunc(st.sx[0], st.sy[0], st.sx[1], st.sy[1], (float)px, (float)py) * signFactor;
            if (w0 < 0 || w1 < 0 || w2 < 0) continue;
            float denom = w0 + w1 + w2;
            if (denom < 1e-8f) continue;
            float b0=w0/denom, b1=w1/denom, b2=w2/denom;
            float z = b0*st.ndc[0].z + b1*st.ndc[1].z + b2*st.ndc[2].z;
            int idx = py * W + px;
            if (z >= fb.depth[idx]) continue;
            fb.depth[idx] = z;
            Vec3 n = (st.norm[0]*b0 + st.norm[1]*b1 + st.norm[2]*b2).normalize();
            fb.color[idx] = shade(color, n, viewDir, light);
        }
    }
}

void rasterizeTransparentTriangle(const ScreenTri& st, Vec3 color, float alpha,
                                   const Framebuffer& opaqueFb, OITBuffers& oit,
                                   const Light& light, Vec3 viewDir) {
    // Double-sided rendering
    float e1x = st.sx[1]-st.sx[0], e1y = st.sy[1]-st.sy[0];
    float e2x = st.sx[2]-st.sx[0], e2y = st.sy[2]-st.sy[0];
    float area = e1x * e2y - e1y * e2x;
    if (std::abs(area) < 1e-8f) return;
    // Determine sign: ef values will all be positive for CCW (area<0), all negative for CW (area>0)
    // Use signFactor to normalize: inside points always give w >= 0
    float signFactor = (area > 0) ? -1.0f : 1.0f;

    int minX = (int)std::max(0.0f, std::min({st.sx[0],st.sx[1],st.sx[2]}));
    int maxX = (int)std::min((float)(W-1), std::max({st.sx[0],st.sx[1],st.sx[2]}));
    int minY = (int)std::max(0.0f, std::min({st.sy[0],st.sy[1],st.sy[2]}));
    int maxY = (int)std::min((float)(H-1), std::max({st.sy[0],st.sy[1],st.sy[2]}));

    for (int py = minY; py <= maxY; py++) {
        for (int px = minX; px <= maxX; px++) {
            float w0 = edgeFunc(st.sx[1], st.sy[1], st.sx[2], st.sy[2], (float)px, (float)py) * signFactor;
            float w1 = edgeFunc(st.sx[2], st.sy[2], st.sx[0], st.sy[0], (float)px, (float)py) * signFactor;
            float w2 = edgeFunc(st.sx[0], st.sy[0], st.sx[1], st.sy[1], (float)px, (float)py) * signFactor;
            // Inside if all barycentric weights >= 0
            bool inside = (w0 >= 0 && w1 >= 0 && w2 >= 0);
            if (!inside) continue;
            float denom = w0 + w1 + w2;
            if (denom < 1e-8f) continue;
            float b0=w0/denom, b1=w1/denom, b2=w2/denom;
            float z = b0*st.ndc[0].z + b1*st.ndc[1].z + b2*st.ndc[2].z;
            int idx = py * W + px;
            // Depth test against opaque geometry
            if (z >= opaqueFb.depth[idx]) continue;

            Vec3 n = (st.norm[0]*b0 + st.norm[1]*b1 + st.norm[2]*b2).normalize();
            // For back-face rendering, flip normal
            if (signFactor < 0) n = n * -1.0f;
            Vec3 litColor = shade(color, n, viewDir, light);

            // Map NDC z [-1,1] to linear [0,1]
            float linearZ = (z + 1.0f) * 0.5f;
            float w = oitWeight(linearZ, alpha);

            oit.accum[idx].x += litColor.x * alpha * w;
            oit.accum[idx].y += litColor.y * alpha * w;
            oit.accum[idx].z += litColor.z * alpha * w;
            oit.accum[idx].w += alpha * w;
            oit.reveal[idx] *= (1.0f - alpha);
        }
    }
}

Vec3 compositeOIT(int idx, const Framebuffer& opaqueFb, const OITBuffers& oit) {
    Vec3 bg = opaqueFb.color[idx];
    float revealage = oit.reveal[idx];
    float accumW = oit.accum[idx].w;
    if (accumW < 1e-5f) return bg;
    Vec3 accumColor = {oit.accum[idx].x, oit.accum[idx].y, oit.accum[idx].z};
    Vec3 avgColor = accumColor / accumW;
    // composite: transparent color over background
    return mix(bg, avgColor, 1.0f - revealage);
}

// Gradient background
void drawGradientBg(Framebuffer& fb) {
    for (int y = 0; y < H; y++) {
        float t = (float)y / (H - 1);
        Vec3 top = {0.08f, 0.10f, 0.15f};
        Vec3 bot = {0.20f, 0.16f, 0.13f};
        Vec3 c = mix(top, bot, t);
        for (int x = 0; x < W; x++) fb.color[y*W+x] = c;
    }
    std::fill(fb.depth.begin(), fb.depth.end(), 1e9f);
}

int main() {
    printf("Order-Independent Transparency (OIT) Renderer\n");
    printf("Technique: Weighted Blended OIT (McGuire & Bavoil 2013)\n");
    printf("Resolution: %dx%d\n\n", W, H);

    Framebuffer fb;
    OITBuffers oit;

    // Camera
    Vec3 eye    = {0, 2.5f, 8.0f};
    Vec3 center = {0, 0, 0};
    Vec3 up     = {0, 1, 0};
    Vec3 viewDir = (center - eye).normalize();

    // Build lookAt correctly
    Vec3 fwd = (center - eye).normalize();
    Vec3 rgt = fwd.cross(up).normalize();
    Vec3 upv = rgt.cross(fwd);
    Mat4 view = Mat4::identity();
    view.m[0][0]=rgt.x; view.m[0][1]=rgt.y; view.m[0][2]=rgt.z; view.m[0][3]=-rgt.dot(eye);
    view.m[1][0]=upv.x; view.m[1][1]=upv.y; view.m[1][2]=upv.z; view.m[1][3]=-upv.dot(eye);
    view.m[2][0]=-fwd.x;view.m[2][1]=-fwd.y;view.m[2][2]=-fwd.z;view.m[2][3]=fwd.dot(eye);

    Mat4 proj = perspective((float)(M_PI / 3.0), (float)W / H, 0.1f, 100.0f);
    Mat4 VP = proj * view;

    Light light;
    light.dir    = Vec3(1.0f, 2.0f, 1.5f).normalize();
    light.color  = {1.0f, 0.95f, 0.85f};
    light.ambient= 0.25f;

    // ─── Scene ───
    auto quadTris = buildQuad();
    auto boxTris  = buildBox();

    std::vector<Mesh> opaqueMeshes, transparentMeshes;

    // Opaque: back wall
    opaqueMeshes.push_back(buildMesh(quadTris,
        translate({0, 0, -3}) * rotateY(0) * scaleM({5, 4, 1}),
        {0.38f, 0.38f, 0.42f}, 1, true));

    // Opaque: floor
    opaqueMeshes.push_back(buildMesh(quadTris,
        translate({0, -2.0f, 0}) * rotateX((float)(M_PI/2.0)) * scaleM({5, 5, 1}),
        {0.42f, 0.40f, 0.36f}, 1, true));

    // Opaque: center pedestal
    opaqueMeshes.push_back(buildMesh(boxTris,
        translate({0, -1.3f, 0}) * scaleM({0.5f, 0.5f, 0.5f}),
        {0.55f, 0.50f, 0.45f}, 1, true));

    // Opaque: left pillar
    opaqueMeshes.push_back(buildMesh(boxTris,
        translate({-3.2f, 0.0f, -1.0f}) * scaleM({0.35f, 2.0f, 0.35f}),
        {0.48f, 0.45f, 0.42f}, 1, true));

    // Opaque: right pillar
    opaqueMeshes.push_back(buildMesh(boxTris,
        translate({ 3.2f, 0.0f, -1.0f}) * scaleM({0.35f, 2.0f, 0.35f}),
        {0.48f, 0.45f, 0.42f}, 1, true));

    // Transparent: 6 colored panels with various tilts
    struct PanelDef { float tx,ty,tz; float ry; float sx,sy; Vec3 col; float a; };
    PanelDef panels[] = {
        {-1.2f, 0.5f, 1.5f,  -(float)(M_PI/6),  1.2f, 1.8f,  {0.9f, 0.15f, 0.1f},  0.55f},
        { 0.9f, 0.2f, 0.8f,   (float)(M_PI/5),  1.3f, 2.0f,  {0.1f, 0.82f, 0.15f}, 0.50f},
        {-0.2f, 0.0f,-0.5f,   (float)(M_PI/12), 1.5f, 2.2f,  {0.1f, 0.3f,  0.95f}, 0.45f},
        { 0.4f,-0.3f, 2.1f,  -(float)(M_PI/8),  1.1f, 1.6f,  {0.95f,0.85f, 0.05f}, 0.40f},
        {-0.9f, 0.3f, 2.5f,   (float)(M_PI/7),  1.0f, 1.5f,  {0.05f,0.85f, 0.9f},  0.50f},
        { 0.1f, 1.2f, 1.0f,   (float)(M_PI/20), 0.9f, 0.9f,  {0.9f, 0.1f,  0.8f},  0.45f},
    };
    for (auto& p : panels) {
        transparentMeshes.push_back(buildMesh(quadTris,
            translate({p.tx,p.ty,p.tz}) * rotateY(p.ry) * scaleM({p.sx, p.sy, 1.0f}),
            p.col, p.a, false));
    }

    // Transparent: orange glass box
    transparentMeshes.push_back(buildMesh(boxTris,
        translate({1.3f, -0.5f, 1.9f}) * rotateY((float)(M_PI/5)) * scaleM({0.45f, 0.65f, 0.45f}),
        {0.95f, 0.5f, 0.05f}, 0.35f, false));

    // Transparent: white glass box
    transparentMeshes.push_back(buildMesh(boxTris,
        translate({-0.5f, 0.0f, 1.5f}) * rotateY((float)(M_PI/9)) * scaleM({0.55f, 0.85f, 0.55f}),
        {0.85f, 0.88f, 0.92f}, 0.3f, false));

    // Transparent: thin blue glass slab
    transparentMeshes.push_back(buildMesh(boxTris,
        translate({0.5f, 0.8f, 0.3f}) * rotateY((float)(-M_PI/15)) * scaleM({0.8f, 0.4f, 0.1f}),
        {0.4f, 0.6f, 1.0f}, 0.4f, false));

    // ── Pass 1: Render opaque geometry ──
    printf("Pass 1: Rendering opaque geometry...\n");
    drawGradientBg(fb);

    for (const auto& mesh : opaqueMeshes) {
        for (const auto& tri : mesh.tris) {
            ScreenTri st = projectTriangle(tri, VP);
            if (!st.valid) continue;
            rasterizeOpaqueTriangle(st, mesh.color, true, fb, light, viewDir);
        }
    }

    // ── Pass 2: Accumulate transparent fragments ──
    printf("Pass 2: Accumulating transparent fragments (OIT)...\n");
    oit.clear();

    for (const auto& mesh : transparentMeshes) {
        for (const auto& tri : mesh.tris) {
            ScreenTri st = projectTriangle(tri, VP);
            if (!st.valid) continue;
            rasterizeTransparentTriangle(st, mesh.color, mesh.alpha, fb, oit, light, viewDir);
        }
    }

    // ── Pass 3: Composite ──
    printf("Pass 3: Compositing OIT results...\n");
    std::vector<Vec3> finalPixels(W * H);
    for (int i = 0; i < W * H; i++) {
        finalPixels[i] = compositeOIT(i, fb, oit);
    }

    // ── Save ──
    writePPM("oit_output.ppm", finalPixels);
    printf("Saved: oit_output.ppm\n");

    int ret = system("convert oit_output.ppm oit_output.png 2>/dev/null");
    if (ret != 0) {
        ret = system("pnmtopng oit_output.ppm > oit_output.png 2>/dev/null");
    }
    if (ret == 0) {
        printf("Saved: oit_output.png\n");
    } else {
        system("cp oit_output.ppm oit_output.png");
        printf("Fallback: copied PPM as PNG\n");
    }

    // ── Statistics ──
    int tpix = 0;
    float totalReveal = 0;
    for (int i = 0; i < W*H; i++) {
        if (oit.accum[i].w > 1e-5f) tpix++;
        totalReveal += oit.reveal[i];
    }
    float avgReveal = totalReveal / (W * H);
    printf("\nStatistics:\n");
    printf("  Transparent pixels: %d / %d (%.1f%%)\n", tpix, W*H, 100.0f*tpix/(W*H));
    printf("  Average revealage: %.4f\n", avgReveal);
    printf("  Opaque meshes: %zu, Transparent meshes: %zu\n",
           opaqueMeshes.size(), transparentMeshes.size());
    printf("Done!\n");
    return 0;
}
