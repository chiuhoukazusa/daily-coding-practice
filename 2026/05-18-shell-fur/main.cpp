/**
 * Shell Texturing Fur Renderer
 * 
 * Technique: Shell Texturing - render N concentric shells of a sphere,
 * each shell slightly expanded along normals. Use procedural noise to
 * define fur strand presence at each shell layer. Higher shells = tips
 * of fur, lower shells = base. Gravity bends the fur strands.
 *
 * Features:
 * - Soft rasterizer with z-buffer
 * - Sphere base mesh with subdivided triangles
 * - N=32 fur shells, each expanded by shell_h * index
 * - Procedural hash noise for fur strand density mask
 * - Gravity bending via tangent-space displacement
 * - Kajiya-Kay fur lighting model
 * - Multiple fur colors (tiger stripe pattern)
 * - Background gradient sky
 *
 * Output: shell_fur_output.png (800x600)
 */

#include <cstdint>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cassert>
#include <array>
#include <random>
#include <iostream>
#include <limits>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

// ============================================================
// Math types
// ============================================================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
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
    Vec3 normalize() const { float l = length(); return l>1e-8f ? *this/l : Vec3(0,0,0); }
};
struct Vec4 { float x, y, z, w; };
struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 mul(Vec4 v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++)
            for(int k=0;k<4;k++) r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }
};

// ============================================================
// Framebuffer
// ============================================================
const int W = 800, H = 600;
struct Color { uint8_t r, g, b; };

std::vector<Color>  framebuf(W*H, {0,0,0});
std::vector<float>  zbuf(W*H, std::numeric_limits<float>::max());
std::vector<float>  alphabuf(W*H, 0.0f); // accumulated alpha for fur
std::vector<Vec3>   colorbuf(W*H, {0,0,0});

void clearBuffers(Color bg) {
    for(auto& c : framebuf) c = bg;
    std::fill(zbuf.begin(), zbuf.end(), std::numeric_limits<float>::max());
    std::fill(alphabuf.begin(), alphabuf.end(), 0.0f);
    std::fill(colorbuf.begin(), colorbuf.end(), Vec3(0,0,0));
}

// ============================================================
// Math helpers
// ============================================================
Mat4 perspective(float fovy, float aspect, float near, float far) {
    Mat4 m;
    float t = std::tan(fovy*0.5f);
    m.m[0][0] = 1.0f/(aspect*t);
    m.m[1][1] = 1.0f/t;
    m.m[2][2] = -(far+near)/(far-near);
    m.m[2][3] = -2.0f*far*near/(far-near);
    m.m[3][2] = -1.0f;
    return m;
}

Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = (center - eye).normalize();
    Vec3 r = f.cross(up).normalize();
    Vec3 u2 = r.cross(f);
    Mat4 m = Mat4::identity();
    m.m[0][0]=r.x;  m.m[0][1]=r.y;  m.m[0][2]=r.z;  m.m[0][3]=-r.dot(eye);
    m.m[1][0]=u2.x; m.m[1][1]=u2.y; m.m[1][2]=u2.z; m.m[1][3]=-u2.dot(eye);
    m.m[2][0]=-f.x; m.m[2][1]=-f.y; m.m[2][2]=-f.z; m.m[2][3]=f.dot(eye);
    m.m[3][3]=1;
    return m;
}

Mat4 rotateY(float angle) {
    Mat4 m = Mat4::identity();
    float c=std::cos(angle), s=std::sin(angle);
    m.m[0][0]=c; m.m[0][2]=s; m.m[2][0]=-s; m.m[2][2]=c;
    return m;
}

// ============================================================
// Procedural noise for fur strands
// ============================================================
// Hash-based noise for 2D (u,v) fur strand presence
float hash1(float n) {
    return std::fmod(std::sin(n) * 43758.5453f, 1.0f);
}
float hash2(float u, float v) {
    return hash1(u * 127.1f + v * 311.7f);
}
// Returns 1.0 if a fur strand exists at (u,v) cell
// shellFrac in [0,1]: 0=base, 1=tip
float furDensity(float u, float v, float shellFrac, float density) {
    // Scale UV to strand resolution
    float scale = 40.0f;
    float fu = std::floor(u * scale);
    float fv = std::floor(v * scale);
    // Each cell has one strand center
    float cx = (fu + hash2(fu, fv*1.3f)) / scale;
    float cy = (fv + hash2(fu*2.1f, fv)) / scale;
    float du = (u - cx) * scale;
    float dv = (v - cy) * scale;
    float dist2 = du*du + dv*dv;
    // Strand radius shrinks toward tip (tapered)
    float radius = density * (1.0f - shellFrac * 0.8f);
    return (dist2 < radius*radius) ? 1.0f : 0.0f;
}

// Tiger stripe pattern based on spherical coords
Vec3 furBaseColor(float theta, float phi, float shellFrac) {
    // Orange/black tiger stripes
    float stripe = std::sin(theta * 8.0f + std::cos(phi * 3.0f) * 1.5f);
    Vec3 orange(0.85f, 0.45f, 0.1f);
    Vec3 black_col(0.08f, 0.05f, 0.03f);
    Vec3 tip(0.95f, 0.75f, 0.4f); // lighter at tips
    float t = std::max(0.0f, stripe);
    Vec3 base = orange * t + black_col * (1.0f - t);
    // Blend toward tip color at high shell fractions
    return base * (1.0f - shellFrac * 0.3f) + tip * (shellFrac * 0.3f);
}

// ============================================================
// Sphere mesh generation (icosphere-like UV sphere)
// ============================================================
struct Vertex {
    Vec3 pos;
    Vec3 normal;
    float u, v;
};

struct Triangle {
    int a, b, c;
};

std::vector<Vertex>   gVerts;
std::vector<Triangle> gTris;

void buildUVSphere(int stacks, int slices, float radius) {
    gVerts.clear();
    gTris.clear();
    // Generate vertices
    for(int i=0; i<=stacks; i++) {
        float phi = float(i) / float(stacks) * float(M_PI);
        for(int j=0; j<=slices; j++) {
            float theta = float(j) / float(slices) * 2.0f * float(M_PI);
            Vec3 n = {
                std::sin(phi)*std::cos(theta),
                std::cos(phi),
                std::sin(phi)*std::sin(theta)
            };
            float u = float(j)/float(slices);
            float v = float(i)/float(stacks);
            gVerts.push_back({n*radius, n, u, v});
        }
    }
    // Generate triangles
    for(int i=0; i<stacks; i++) {
        for(int j=0; j<slices; j++) {
            int p0 = i*(slices+1)+j;
            int p1 = p0+1;
            int p2 = (i+1)*(slices+1)+j;
            int p3 = p2+1;
            gTris.push_back({p0, p2, p1});
            gTris.push_back({p1, p2, p3});
        }
    }
}

// ============================================================
// Gravity bending
// ============================================================
Vec3 applyGravityBend(Vec3 pos, Vec3 normal, float shellFrac, float gravity) {
    // Bend the shell position toward the gravity direction (-Y)
    Vec3 gravDir(0, -1, 0);
    // Tangent component of gravity
    float normalDotGrav = normal.dot(gravDir);
    Vec3 gravTangent = gravDir - normal * normalDotGrav;
    // The bending increases quadratically with shell fraction
    float bend = gravity * shellFrac * shellFrac;
    return pos + gravTangent * bend;
}

// ============================================================
// Kajiya-Kay fur lighting
// ============================================================
Vec3 kajiyaKay(Vec3 tangent, Vec3 lightDir, Vec3 viewDir, Vec3 furColor,
               Vec3 lightColor, float specPow) {
    // Diffuse: sin of angle between tangent and light
    float cosL = tangent.dot(lightDir);
    float sinL = std::sqrt(std::max(0.0f, 1.0f - cosL*cosL));
    Vec3 diffuse = furColor * lightColor * sinL;

    // Specular: specular highlight along tangent
    float cosV = tangent.dot(viewDir);
    float sinV = std::sqrt(std::max(0.0f, 1.0f - cosV*cosV));
    float spec = std::pow(std::max(0.0f, sinL*sinV - cosL*cosV), specPow);
    Vec3 specular = lightColor * spec * 0.4f;

    return diffuse + specular;
}

// ============================================================
// Rasterizer
// ============================================================
Vec3 toScreen(Vec4 clip) {
    float invW = 1.0f / clip.w;
    float ndcX = clip.x * invW;
    float ndcY = clip.y * invW;
    float ndcZ = clip.z * invW;
    return {
        (ndcX + 1.0f) * 0.5f * W,
        (1.0f - ndcY) * 0.5f * H,  // Y flipped: top=0
        ndcZ
    };
}

float edgeFunc(Vec3 a, Vec3 b, Vec3 c) {
    return (c.x - a.x)*(b.y - a.y) - (c.y - a.y)*(b.x - a.x);
}

// Draw a single shell triangle with fur alpha compositing
void rasterizeShellTri(
    const Vertex& va, const Vertex& vb, const Vertex& vc,
    const Mat4& mvp, const Mat4& model,
    float shellFrac, float furDens,
    Vec3 lightPos, Vec3 viewPos
) {
    // Transform vertices
    auto xform = [&](const Vertex& v) -> std::pair<Vec4, Vec3> {
        Vec4 clip = mvp.mul({v.pos.x, v.pos.y, v.pos.z, 1.0f});
        Vec4 wpos4 = model.mul({v.pos.x, v.pos.y, v.pos.z, 1.0f});
        Vec3 wpos = {wpos4.x/wpos4.w, wpos4.y/wpos4.w, wpos4.z/wpos4.w};
        return {clip, wpos};
    };

    auto [clipA, wpA] = xform(va);
    auto [clipB, wpB] = xform(vb);
    auto [clipC, wpC] = xform(vc);

    // Clip space culling
    auto inFrustum = [](Vec4 c) {
        return std::abs(c.x) <= c.w && std::abs(c.y) <= c.w &&
               c.z >= -c.w && c.z <= c.w && c.w > 0;
    };
    if(!inFrustum(clipA) && !inFrustum(clipB) && !inFrustum(clipC)) return;

    Vec3 sA = toScreen(clipA);
    Vec3 sB = toScreen(clipB);
    Vec3 sC = toScreen(clipC);

    // Backface cull
    float area = edgeFunc(sA, sB, sC);
    if(area <= 0) return;
    float invArea = 1.0f / area;

    // Bounding box
    int minX = std::max(0, (int)std::floor(std::min({sA.x, sB.x, sC.x})));
    int maxX = std::min(W-1, (int)std::ceil(std::max({sA.x, sB.x, sC.x})));
    int minY = std::max(0, (int)std::floor(std::min({sA.y, sB.y, sC.y})));
    int maxY = std::min(H-1, (int)std::ceil(std::max({sA.y, sB.y, sC.y})));

    for(int py=minY; py<=maxY; py++) {
        for(int px=minX; px<=maxX; px++) {
            Vec3 p = {px+0.5f, py+0.5f, 0};
            float w0 = edgeFunc(sB, sC, p) * invArea;
            float w1 = edgeFunc(sC, sA, p) * invArea;
            float w2 = edgeFunc(sA, sB, p) * invArea;
            if(w0<0||w1<0||w2<0) continue;

            float depth = w0*sA.z + w1*sB.z + w2*sC.z;

            // Interpolate UV
            float invWa = 1.0f/clipA.w, invWb = 1.0f/clipB.w, invWc = 1.0f/clipC.w;
            float wSum = w0*invWa + w1*invWb + w2*invWc;
            float u = (w0*va.u*invWa + w1*vb.u*invWb + w2*vc.u*invWc) / wSum;
            float v = (w0*va.v*invWa + w1*vb.v*invWb + w2*vc.v*invWc) / wSum;

            // Interpolate world normal
            Vec3 n = {
                w0*va.normal.x + w1*vb.normal.x + w2*vc.normal.x,
                w0*va.normal.y + w1*vb.normal.y + w2*vc.normal.y,
                w0*va.normal.z + w1*vb.normal.z + w2*vc.normal.z
            };
            n = n.normalize();

            // Interpolate world pos
            Vec3 wp = wpA*w0 + wpB*w1 + wpC*w2;

            // Check fur strand presence
            float fd = furDensity(u, v, shellFrac, furDens);
            if(fd < 0.5f) continue;

            // Only write to zbuffer for base shell (shell 0)
            // For upper shells, use additive alpha blending
            int idx = py*W + px;

            // Compute lighting
            // Tangent along phi direction (horizontal circle)
            Vec3 tangent = Vec3(-std::sin(v*float(M_PI)), 0, std::cos(v*float(M_PI))).normalize();
            // Gravity-bent tangent
            Vec3 gravDir(0,-1,0);
            float bt = shellFrac * 0.3f;
            tangent = (tangent + gravDir * bt).normalize();

            Vec3 lightDir = (lightPos - wp).normalize();
            Vec3 viewDir  = (viewPos  - wp).normalize();

            // Determine sphere angles for color
            float sphereTheta = std::atan2(n.z, n.x);
            float spherePhi   = std::acos(std::clamp(n.y, -1.0f, 1.0f));

            Vec3 furCol = furBaseColor(sphereTheta, spherePhi, shellFrac);
            Vec3 lightColor(1.0f, 0.95f, 0.85f);
            Vec3 ambient(0.15f, 0.12f, 0.1f);

            Vec3 lit = kajiyaKay(tangent, lightDir, viewDir, furCol, lightColor, 32.0f);
            Vec3 finalColor = lit + furCol * ambient;

            // Alpha: full at base, fades at tip
            float alpha = 1.0f - shellFrac * 0.7f;

            // Depth test for solid base
            if(shellFrac < 0.05f) {
                if(depth < zbuf[idx]) {
                    zbuf[idx] = depth;
                    colorbuf[idx] = finalColor;
                    alphabuf[idx] = 1.0f;
                }
            } else {
                // Alpha blend upper shells over existing
                if(depth < zbuf[idx] + 0.1f) {
                    float existAlpha = alphabuf[idx];
                    float newAlpha = existAlpha + alpha * (1.0f - existAlpha);
                    if(newAlpha > 1e-5f) {
                        colorbuf[idx] = colorbuf[idx] * existAlpha + finalColor * alpha * (1.0f - existAlpha);
                        colorbuf[idx] = colorbuf[idx] * (1.0f / newAlpha);
                        alphabuf[idx] = newAlpha;
                    }
                }
            }
        }
    }
}

// ============================================================
// Draw sky background
// ============================================================
void drawBackground() {
    for(int y=0; y<H; y++) {
        float t = float(y) / float(H);
        // Sky gradient: warm amber sky at top, dark horizon at bottom
        Vec3 top(0.12f, 0.08f, 0.05f);   // dark warm
        Vec3 bot(0.35f, 0.2f, 0.08f);    // warm amber horizon
        Vec3 c = top * (1.0f - t) + bot * t;
        for(int x=0; x<W; x++) {
            framebuf[y*W+x] = {
                uint8_t(std::min(255.0f, c.x*255)),
                uint8_t(std::min(255.0f, c.y*255)),
                uint8_t(std::min(255.0f, c.z*255))
            };
        }
    }
}

// ============================================================
// Composite colorbuf into framebuf
// ============================================================
void composite() {
    for(int i=0; i<W*H; i++) {
        if(alphabuf[i] > 0.01f) {
            Vec3 c = colorbuf[i];
            // Gamma correction
            float gamma = 1.0f / 2.2f;
            uint8_t r = uint8_t(std::min(255.0f, std::pow(std::max(0.0f,c.x), gamma)*255));
            uint8_t g = uint8_t(std::min(255.0f, std::pow(std::max(0.0f,c.y), gamma)*255));
            uint8_t b = uint8_t(std::min(255.0f, std::pow(std::max(0.0f,c.z), gamma)*255));
            // Alpha blend over background
            float a = alphabuf[i];
            framebuf[i].r = uint8_t(r*a + framebuf[i].r*(1.0f-a));
            framebuf[i].g = uint8_t(g*a + framebuf[i].g*(1.0f-a));
            framebuf[i].b = uint8_t(b*a + framebuf[i].b*(1.0f-a));
        }
    }
}

// ============================================================
// Main
// ============================================================
int main() {
    std::cout << "Shell Texturing Fur Renderer" << std::endl;
    std::cout << "Resolution: " << W << "x" << H << std::endl;

    // Build sphere mesh
    const int STACKS = 32, SLICES = 64;
    const float BASE_RADIUS = 1.0f;
    buildUVSphere(STACKS, SLICES, BASE_RADIUS);
    std::cout << "Sphere: " << gVerts.size() << " verts, " << gTris.size() << " tris" << std::endl;

    // Camera & transforms
    Vec3 eye(0, 0.5f, 3.5f);
    Vec3 center(0, 0, 0);
    Vec3 up(0, 1, 0);

    Mat4 view = lookAt(eye, center, up);
    Mat4 proj = perspective(float(M_PI)/4.0f, float(W)/float(H), 0.1f, 20.0f);
    Mat4 modelM = rotateY(0.3f);
    Mat4 vp = proj * view;
    Mat4 mvp = vp * modelM;

    // Lighting
    Vec3 lightPos(3.0f, 4.0f, 2.0f);

    // Shell parameters
    const int NUM_SHELLS = 32;
    const float FUR_LENGTH = 0.12f; // total fur length
    const float GRAVITY = 0.04f;
    const float FUR_DENSITY = 0.42f; // strand radius in cell

    std::cout << "Rendering " << NUM_SHELLS << " shells..." << std::endl;

    // Draw background
    drawBackground();

    // Render shells from outer to inner (back to front for alpha)
    // Actually render inner first for correct depth
    for(int shell=0; shell<NUM_SHELLS; shell++) {
        float shellFrac = float(shell) / float(NUM_SHELLS - 1);
        float expansion = shell * FUR_LENGTH / float(NUM_SHELLS - 1);

        // Expand vertices along normals and apply gravity bending
        std::vector<Vertex> shellVerts(gVerts.size());
        for(size_t i=0; i<gVerts.size(); i++) {
            Vec3 pos = gVerts[i].pos + gVerts[i].normal * expansion;
            pos = applyGravityBend(pos, gVerts[i].normal, shellFrac, GRAVITY);
            shellVerts[i] = {pos, gVerts[i].normal, gVerts[i].u, gVerts[i].v};
        }

        // Rasterize each triangle
        for(const auto& tri : gTris) {
            rasterizeShellTri(
                shellVerts[tri.a], shellVerts[tri.b], shellVerts[tri.c],
                mvp, modelM,
                shellFrac, FUR_DENSITY,
                lightPos, eye
            );
        }

        if(shell % 8 == 0) {
            std::cout << "  Shell " << shell+1 << "/" << NUM_SHELLS << std::endl;
        }
    }

    // Composite fur onto background
    composite();

    // Save image
    const char* outFile = "shell_fur_output.png";
    int result = stbi_write_png(outFile, W, H, 3,
        framebuf.data(), W*3);
    if(!result) {
        std::cerr << "Failed to write image!" << std::endl;
        return 1;
    }

    std::cout << "Saved: " << outFile << std::endl;
    return 0;
}
