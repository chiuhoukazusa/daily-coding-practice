/*
 * Radiance Cascade Global Illumination Renderer
 * 2026-05-02 Daily Coding Practice
 *
 * Implements a simplified Lumen-style Radiance Cascade GI:
 *   - Software rasterizer with G-Buffer (albedo, normal, position)
 *   - Multi-level radiance cascade probes for indirect illumination
 *   - Cascade 0: fine grid, 8 directions, short range
 *   - Cascade 1: medium grid, 16 directions, medium range
 *   - Cascade 2: coarse grid, 32 directions, long range
 *   - Merge cascades top-down for full GI
 *   - Final: direct lighting + indirect lighting = full GI scene
 *
 * Output: radiance_cascade_gi_output.png (800x600)
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

// ─────────────────────────────────────────────────────────────
// stb_image_write (single-header)
// ─────────────────────────────────────────────────────────────
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

// ─────────────────────────────────────────────────────────────
// Math helpers
// ─────────────────────────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b)  const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    Vec3& operator*=(float t)      { x*=t; y*=t; z*=t; return *this; }
    float dot(const Vec3& b)  const { return x*b.x + y*b.y + z*b.z; }
    Vec3  cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const {
        float l = length();
        return l > 1e-8f ? Vec3(x/l, y/l, z/l) : Vec3(0,1,0);
    }
    Vec3 clamp01() const {
        return {std::min(1.0f, std::max(0.0f, x)),
                std::min(1.0f, std::max(0.0f, y)),
                std::min(1.0f, std::max(0.0f, z))};
    }
};

static Vec3 mix(const Vec3& a, const Vec3& b, float t) {
    return a * (1-t) + b * t;
}

// ─────────────────────────────────────────────────────────────
// Canvas
// ─────────────────────────────────────────────────────────────
static const int W = 800, H = 600;

struct Canvas {
    std::vector<Vec3> color;
    std::vector<float> depth;
    Canvas() : color(W*H, Vec3(0,0,0)), depth(W*H, 1e30f) {}
    void set(int x, int y, const Vec3& c, float d) {
        if (x<0||x>=W||y<0||y>=H) return;
        int i = y*W+x;
        if (d < depth[i]) { depth[i]=d; color[i]=c; }
    }
    Vec3 get(int x, int y) const {
        if (x<0||x>=W||y<0||y>=H) return Vec3(0,0,0);
        return color[y*W+x];
    }
};

// ─────────────────────────────────────────────────────────────
// G-Buffer
// ─────────────────────────────────────────────────────────────
struct GBuf {
    std::vector<Vec3> albedo;
    std::vector<Vec3> normal;
    std::vector<Vec3> worldPos;
    std::vector<float> depth;
    std::vector<bool> valid;
    GBuf() : albedo(W*H), normal(W*H), worldPos(W*H),
             depth(W*H, 1e30f), valid(W*H, false) {}
    void set(int x, int y, const Vec3& a, const Vec3& n, const Vec3& wp, float d) {
        if (x<0||x>=W||y<0||y>=H) return;
        int i = y*W+x;
        if (d < depth[i]) {
            depth[i]=d; albedo[i]=a; normal[i]=n;
            worldPos[i]=wp; valid[i]=true;
        }
    }
};

// ─────────────────────────────────────────────────────────────
// Camera & projection
// ─────────────────────────────────────────────────────────────
struct Camera {
    Vec3 pos, target, up;
    float fov, near_, far_;
    Camera() : pos(0,3,8), target(0,1,0), up(0,1,0),
               fov(60.f*3.14159f/180.f), near_(0.1f), far_(100.f) {}
};

struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    static Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
        Vec3 f = (center - eye).normalize();
        Vec3 s = f.cross(up).normalize();
        Vec3 u = s.cross(f);
        Mat4 r = identity();
        r.m[0][0]=s.x; r.m[0][1]=s.y; r.m[0][2]=s.z;
        r.m[1][0]=u.x; r.m[1][1]=u.y; r.m[1][2]=u.z;
        r.m[2][0]=-f.x; r.m[2][1]=-f.y; r.m[2][2]=-f.z;
        r.m[0][3]=-s.dot(eye); r.m[1][3]=-u.dot(eye); r.m[2][3]=f.dot(eye);
        return r;
    }
    static Mat4 perspective(float fov, float aspect, float n, float f) {
        float t = std::tan(fov/2.f);
        Mat4 r;
        r.m[0][0]=1.f/(aspect*t); r.m[1][1]=1.f/t;
        r.m[2][2]=-(f+n)/(f-n);  r.m[2][3]=-2.f*f*n/(f-n);
        r.m[3][2]=-1.f;
        return r;
    }
    Vec3 transform(const Vec3& v, float w=1.f) const {
        float x=m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*w;
        float y=m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*w;
        float z=m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*w;
        float ww=m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*w;
        if (std::abs(ww)>1e-8f) { x/=ww; y/=ww; z/=ww; }
        return {x,y,z};
    }
};

// ─────────────────────────────────────────────────────────────
// Scene geometry (triangles)
// ─────────────────────────────────────────────────────────────
struct Vertex { Vec3 pos, normal; };
struct Triangle { Vertex v[3]; Vec3 albedo; };

// Cornell-box style scene
static std::vector<Triangle> buildScene() {
    std::vector<Triangle> tris;
    // Helper lambda: quad from 4 verts with given normal and color
    auto makeTriangle = [](Vec3 a, Vec3 b, Vec3 c, Vec3 na, Vec3 nb, Vec3 nc, Vec3 col) {
        Triangle t;
        t.v[0]={a,na}; t.v[1]={b,nb}; t.v[2]={c,nc}; t.albedo=col;
        return t;
    };
    auto quad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n, Vec3 col) {
        tris.push_back(makeTriangle(a,b,c, n,n,n, col));
        tris.push_back(makeTriangle(a,c,d, n,n,n, col));
    };

    Vec3 white(0.9f,0.9f,0.9f), red(0.8f,0.15f,0.15f),
         green(0.15f,0.75f,0.15f), blue(0.2f,0.3f,0.9f),
         yellow(0.9f,0.85f,0.1f), grey(0.6f,0.6f,0.65f);

    // Floor
    quad({-4,0,-4},{4,0,-4},{4,0,4},{-4,0,4},{0,1,0}, white);
    // Ceiling
    quad({-4,5,-4},{-4,5,4},{4,5,4},{4,5,-4},{0,-1,0}, white);
    // Back wall
    quad({-4,0,-4},{-4,5,-4},{4,5,-4},{4,0,-4},{0,0,1}, white);
    // Left wall (red)
    quad({-4,0,-4},{-4,0,4},{-4,5,4},{-4,5,-4},{1,0,0}, red);
    // Right wall (green)
    quad({4,0,-4},{4,5,-4},{4,5,4},{4,0,4},{-1,0,0}, green);

    // Tall box (blue)
    float bx=-1.5f, bz=-1.f, bh=2.8f;
    // Front
    quad({bx-0.7f,0,bz+0.7f},{bx+0.7f,0,bz+0.7f},
         {bx+0.7f,bh,bz+0.7f},{bx-0.7f,bh,bz+0.7f},{0,0,1},blue);
    // Back
    quad({bx+0.7f,0,bz-0.7f},{bx-0.7f,0,bz-0.7f},
         {bx-0.7f,bh,bz-0.7f},{bx+0.7f,bh,bz-0.7f},{0,0,-1},blue);
    // Left
    quad({bx-0.7f,0,bz-0.7f},{bx-0.7f,0,bz+0.7f},
         {bx-0.7f,bh,bz+0.7f},{bx-0.7f,bh,bz-0.7f},{-1,0,0},blue);
    // Right
    quad({bx+0.7f,0,bz+0.7f},{bx+0.7f,0,bz-0.7f},
         {bx+0.7f,bh,bz-0.7f},{bx+0.7f,bh,bz+0.7f},{1,0,0},blue);
    // Top
    quad({bx-0.7f,bh,bz-0.7f},{bx-0.7f,bh,bz+0.7f},
         {bx+0.7f,bh,bz+0.7f},{bx+0.7f,bh,bz-0.7f},{0,1,0},blue);

    // Short box (yellow)
    float sx=1.5f, sz=0.5f, sh=1.4f;
    quad({sx-0.8f,0,sz+0.8f},{sx+0.8f,0,sz+0.8f},
         {sx+0.8f,sh,sz+0.8f},{sx-0.8f,sh,sz+0.8f},{0,0,1},yellow);
    quad({sx+0.8f,0,sz-0.8f},{sx-0.8f,0,sz-0.8f},
         {sx-0.8f,sh,sz-0.8f},{sx+0.8f,sh,sz-0.8f},{0,0,-1},yellow);
    quad({sx-0.8f,0,sz-0.8f},{sx-0.8f,0,sz+0.8f},
         {sx-0.8f,sh,sz+0.8f},{sx-0.8f,sh,sz-0.8f},{-1,0,0},yellow);
    quad({sx+0.8f,0,sz+0.8f},{sx+0.8f,0,sz-0.8f},
         {sx+0.8f,sh,sz-0.8f},{sx+0.8f,sh,sz+0.8f},{1,0,0},yellow);
    quad({sx-0.8f,sh,sz-0.8f},{sx-0.8f,sh,sz+0.8f},
         {sx+0.8f,sh,sz+0.8f},{sx+0.8f,sh,sz-0.8f},{0,1,0},yellow);

    // Sphere approximation (icosphere-like cap on grey pedestal)
    // Small sphere as subdivided mesh - simplified as octahedron subdivision
    auto addSphere = [&](Vec3 center, float radius, Vec3 col, int subdivs=3) {
        // Build icosphere
        struct IcoVert { float x,y,z; };
        std::vector<IcoVert> verts;
        std::vector<std::array<int,3>> faces;
        float t = (1.f + std::sqrt(5.f)) / 2.f;
        auto addV = [&](float x, float y, float z) {
            float l = std::sqrt(x*x+y*y+z*z);
            verts.push_back({x/l, y/l, z/l});
        };
        addV(-1,t,0); addV(1,t,0); addV(-1,-t,0); addV(1,-t,0);
        addV(0,-1,t); addV(0,1,t); addV(0,-1,-t); addV(0,1,-t);
        addV(t,0,-1); addV(t,0,1); addV(-t,0,-1); addV(-t,0,1);
        faces = {{0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
                 {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
                 {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
                 {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}};
        // Subdivide
        for (int s=0; s<subdivs; ++s) {
            std::vector<std::array<int,3>> newFaces;
            for (auto& f : faces) {
                auto midpoint = [&](int a, int b) {
                    IcoVert va=verts[a], vb=verts[b];
                    addV(va.x+vb.x, va.y+vb.y, va.z+vb.z);
                    return (int)verts.size()-1;
                };
                int a=midpoint(f[0],f[1]), b=midpoint(f[1],f[2]), c=midpoint(f[2],f[0]);
                newFaces.push_back({f[0],a,c}); newFaces.push_back({f[1],b,a});
                newFaces.push_back({f[2],c,b}); newFaces.push_back({a,b,c});
            }
            faces=newFaces;
        }
        for (auto& f : faces) {
            IcoVert va=verts[f[0]], vb=verts[f[1]], vc=verts[f[2]];
            Vec3 pa(center.x+va.x*radius, center.y+va.y*radius, center.z+va.z*radius);
            Vec3 pb(center.x+vb.x*radius, center.y+vb.y*radius, center.z+vb.z*radius);
            Vec3 pc(center.x+vc.x*radius, center.y+vc.y*radius, center.z+vc.z*radius);
            Vec3 na(va.x,va.y,va.z), nb(vb.x,vb.y,vb.z), nc(vc.x,vc.y,vc.z);
            Triangle tri;
            tri.v[0]={pa,na}; tri.v[1]={pb,nb}; tri.v[2]={pc,nc};
            tri.albedo=col;
            tris.push_back(tri);
        }
    };
    addSphere(Vec3(0.f, 0.75f, 1.5f), 0.75f, grey, 3);

    return tris;
}

// ─────────────────────────────────────────────────────────────
// Lights
// ─────────────────────────────────────────────────────────────
struct Light {
    Vec3 pos, color;
    float intensity;
};

static std::vector<Light> buildLights() {
    return {
        { Vec3( 0.0f, 4.6f, 0.0f), Vec3(1.0f,0.95f,0.8f), 8.f },
        { Vec3(-2.5f, 3.0f, 2.0f), Vec3(0.5f,0.6f,1.0f),  4.f },
        { Vec3( 2.5f, 1.5f,-1.0f), Vec3(1.0f,0.6f,0.3f),  3.f },
    };
}

// ─────────────────────────────────────────────────────────────
// Software rasterizer: rasterize scene into G-Buffer
// ─────────────────────────────────────────────────────────────
static void rasterize(GBuf& gbuf,
                      const std::vector<Triangle>& scene,
                      const Mat4& view, const Mat4& proj)
{
    float aspect = float(W) / float(H);
    (void)aspect;
    for (auto& tri : scene) {
        // Transform vertices to clip space
        Vec3 clip[3], ndc[3];
        for (int i=0; i<3; ++i) {
            Vec3 p = tri.v[i].pos;
            Vec3 vp = view.transform(p);
            // perspective
            Vec3 cp = proj.transform(vp);
            ndc[i] = cp;
            clip[i] = cp;
        }
        // NDC to screen
        float sx[3], sy[3], sz[3];
        for (int i=0; i<3; ++i) {
            sx[i] = (ndc[i].x * 0.5f + 0.5f) * W;
            sy[i] = (1.f - (ndc[i].y * 0.5f + 0.5f)) * H;
            sz[i] = ndc[i].z;
        }
        // Bounding box
        int minX = (int)std::max(0.f, std::min({sx[0],sx[1],sx[2]}));
        int maxX = (int)std::min((float)(W-1), std::max({sx[0],sx[1],sx[2]}));
        int minY = (int)std::max(0.f, std::min({sy[0],sy[1],sy[2]}));
        int maxY = (int)std::min((float)(H-1), std::max({sy[0],sy[1],sy[2]}));
        if (minX>maxX||minY>maxY) continue;
        // Edge function
        auto edge = [](float ax, float ay, float bx, float by, float px, float py) {
            return (bx-ax)*(py-ay) - (by-ay)*(px-ax);
        };
        float area = edge(sx[0],sy[0],sx[1],sy[1],sx[2],sy[2]);
        if (std::abs(area)<1e-5f) continue;
        for (int y=minY; y<=maxY; ++y) {
            for (int x=minX; x<=maxX; ++x) {
                float px = x+0.5f, py = y+0.5f;
                float w0 = edge(sx[1],sy[1],sx[2],sy[2],px,py);
                float w1 = edge(sx[2],sy[2],sx[0],sy[0],px,py);
                float w2 = edge(sx[0],sy[0],sx[1],sy[1],px,py);
                if ((area>0 && (w0<0||w1<0||w2<0)) ||
                    (area<0 && (w0>0||w1>0||w2>0))) continue;
                float b0=w0/area, b1=w1/area, b2=w2/area;
                float depth = sz[0]*b0 + sz[1]*b1 + sz[2]*b2;
                if (depth<-1.f||depth>1.f) continue;
                // Interpolate normal & world pos
                Vec3 wn = (tri.v[0].normal*b0 + tri.v[1].normal*b1 + tri.v[2].normal*b2).normalize();
                Vec3 wp = tri.v[0].pos*b0 + tri.v[1].pos*b1 + tri.v[2].pos*b2;
                gbuf.set(x, y, tri.albedo, wn, wp, depth);
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────
// Direct lighting (Lambertian + Blinn-Phong, no shadows for speed)
// ─────────────────────────────────────────────────────────────
static Vec3 directLight(const Vec3& pos, const Vec3& normal, const Vec3& albedo,
                        const std::vector<Light>& lights, const Vec3& camPos)
{
    Vec3 result(0.02f, 0.02f, 0.02f); // ambient base
    for (auto& l : lights) {
        Vec3 toL = l.pos - pos;
        float dist = toL.length();
        Vec3 lightDir = toL * (1.f/dist);
        float attn = l.intensity / (1.f + 0.3f*dist + 0.1f*dist*dist);
        float ndotl = std::max(0.f, normal.dot(lightDir));
        Vec3 diff = albedo * l.color * (ndotl * attn);
        // Specular
        Vec3 viewDir = (camPos - pos).normalize();
        Vec3 halfV = (lightDir + viewDir).normalize();
        float spec = std::pow(std::max(0.f, normal.dot(halfV)), 32.f) * 0.3f * attn;
        result += diff + l.color * spec;
    }
    return result;
}

// ─────────────────────────────────────────────────────────────
// Radiance Cascade GI
// ─────────────────────────────────────────────────────────────

// A radiance probe stores incoming radiance in N directions
struct Probe {
    std::vector<Vec3> radiance; // per-direction
    Vec3 pos;
    bool valid = false;
};

// Cascade level description
struct CascadeLevel {
    int gridW, gridH;   // probe grid dimensions (in world space)
    int numDirs;        // angular resolution
    float spacing;      // world-space spacing between probes
    float maxRange;     // ray march range
    std::vector<Probe> probes;
};

// Precomputed uniform sphere directions
static std::vector<Vec3> buildDirections(int n) {
    std::vector<Vec3> dirs;
    dirs.reserve(n);
    // Fibonacci hemisphere / full sphere
    float golden = (1.f + std::sqrt(5.f)) * 0.5f;
    for (int i=0; i<n; ++i) {
        float theta = 2.f * 3.14159265f * i / golden;
        float phi = std::acos(1.f - 2.f*(i+0.5f)/n);
        dirs.push_back({std::sin(phi)*std::cos(theta),
                        std::cos(phi),
                        std::sin(phi)*std::sin(theta)});
    }
    return dirs;
}

// Simple scene ray intersection (brute force) for GI probes
static bool rayIntersectScene(const Vec3& orig, const Vec3& dir,
                               const std::vector<Triangle>& scene,
                               float maxDist,
                               float& tHit, Vec3& hitNorm, Vec3& hitAlbedo)
{
    tHit = maxDist;
    bool hit = false;
    for (auto& tri : scene) {
        Vec3 e1 = tri.v[1].pos - tri.v[0].pos;
        Vec3 e2 = tri.v[2].pos - tri.v[0].pos;
        Vec3 h = dir.cross(e2);
        float a = e1.dot(h);
        if (std::abs(a) < 1e-7f) continue;
        float f = 1.f/a;
        Vec3 s = orig - tri.v[0].pos;
        float u = f * s.dot(h);
        if (u<0||u>1) continue;
        Vec3 q = s.cross(e1);
        float v = f * dir.dot(q);
        if (v<0||u+v>1) continue;
        float t = f * e2.dot(q);
        if (t>1e-4f && t<tHit) {
            tHit = t; hit = true;
            float w = 1-u-v;
            hitNorm = (tri.v[0].normal*w + tri.v[1].normal*u + tri.v[2].normal*v).normalize();
            hitAlbedo = tri.albedo;
        }
    }
    return hit;
}

// Build one cascade level
static void buildCascade(CascadeLevel& cascade,
                          const std::vector<Triangle>& scene,
                          const std::vector<Light>& lights,
                          const std::vector<Vec3>& dirs,
                          float worldMinX, float worldMaxX,
                          float worldMinZ, float worldMaxZ,
                          float probeY)
{
    int GW = cascade.gridW, GH = cascade.gridH;
    cascade.probes.resize(GW * GH);

    for (int gz=0; gz<GH; ++gz) {
        for (int gx=0; gx<GW; ++gx) {
            int idx = gz*GW + gx;
            float px = worldMinX + (gx + 0.5f) * (worldMaxX-worldMinX) / GW;
            float pz = worldMinZ + (gz + 0.5f) * (worldMaxZ-worldMinZ) / GH;
            Vec3 probePos(px, probeY, pz);
            cascade.probes[idx].pos = probePos;
            cascade.probes[idx].radiance.resize(cascade.numDirs, Vec3(0,0,0));
            cascade.probes[idx].valid = true;

            for (int d=0; d<cascade.numDirs; ++d) {
                const Vec3& rayDir = dirs[d];
                float tHit; Vec3 hN, hA;
                if (rayIntersectScene(probePos, rayDir, scene,
                                      cascade.maxRange, tHit, hN, hA)) {
                    Vec3 hitPos = probePos + rayDir * tHit;
                    // Evaluate direct lighting at hit point
                    Vec3 lit = directLight(hitPos, hN, hA, lights, Vec3(0,3,8));
                    cascade.probes[idx].radiance[d] = lit;
                } else {
                    // Sky / background
                    float skyT = rayDir.y * 0.5f + 0.5f;
                    cascade.probes[idx].radiance[d] =
                        mix(Vec3(0.1f,0.15f,0.25f), Vec3(0.05f,0.05f,0.08f), skyT);
                }
            }
        }
    }
}

// Merge a coarser cascade into a finer cascade (radiance from higher levels
// fills in longer-range contributions)
static void mergeCascades(CascadeLevel& fine, const CascadeLevel& coarse,
                           const std::vector<Vec3>& fineDirs,
                           const std::vector<Vec3>& coarseDirs)
{
    // For each fine probe, for each direction, add radiance from coarse level
    // using nearest-probe lookup with weight proportional to dot product
    int GW = fine.gridW, GH = fine.gridH;
    for (int gz=0; gz<GH; ++gz) {
        for (int gx=0; gx<GW; ++gx) {
            int idx = gz*GW + gx;
            Vec3 fPos = fine.probes[idx].pos;
            // Find nearest coarse probe
            float best = 1e30f;
            int bestIdx = 0;
            for (int ci=0; ci<(int)coarse.probes.size(); ++ci) {
                float d2 = (coarse.probes[ci].pos - fPos).length();
                if (d2 < best) { best=d2; bestIdx=ci; }
            }
            // Blend: each fine direction, query the coarse probe
            for (int fd=0; fd<fine.numDirs; ++fd) {
                Vec3 fDir = fineDirs[fd];
                // Find best matching coarse direction
                float bestDot = -2.f; int bestCD = 0;
                for (int cd=0; cd<coarse.numDirs; ++cd) {
                    float dt = fDir.dot(coarseDirs[cd]);
                    if (dt > bestDot) { bestDot=dt; bestCD=cd; }
                }
                float weight = std::max(0.f, bestDot);
                // Blend in coarse contribution (far range adds low-intensity bounce)
                fine.probes[idx].radiance[fd] +=
                    coarse.probes[bestIdx].radiance[bestCD] * (weight * 0.25f);
            }
        }
    }
}

// Sample indirect radiance at a world-space point from a cascade
static Vec3 sampleCascade(const CascadeLevel& cascade,
                            const std::vector<Vec3>& dirs,
                            const Vec3& worldPos,
                            const Vec3& surfNormal)
{
    // Find nearest probe
    float bestDist = 1e30f; int bestIdx=0;
    for (int i=0; i<(int)cascade.probes.size(); ++i) {
        if (!cascade.probes[i].valid) continue;
        float d = (cascade.probes[i].pos - worldPos).length();
        if (d < bestDist) { bestDist=d; bestIdx=i; }
    }
    // Gather irradiance in upper hemisphere (cosine-weighted)
    Vec3 indirect(0,0,0);
    float totalW = 0.f;
    for (int d=0; d<(int)dirs.size(); ++d) {
        float ndotl = surfNormal.dot(dirs[d]);
        if (ndotl <= 0.f) continue;
        indirect += cascade.probes[bestIdx].radiance[d] * ndotl;
        totalW += ndotl;
    }
    if (totalW > 1e-7f) indirect *= (1.f / totalW);
    return indirect;
}

// ─────────────────────────────────────────────────────────────
// Tone mapping & gamma
// ─────────────────────────────────────────────────────────────
static Vec3 toneMap(Vec3 c) {
    // ACES approximate
    float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
    auto aces = [&](float x) {
        return std::min(1.f, std::max(0.f, (x*(a*x+b))/(x*(cc*x+d)+e)));
    };
    return {aces(c.x), aces(c.y), aces(c.z)};
}
static Vec3 gamma(Vec3 c) {
    float g = 1.f/2.2f;
    return {std::pow(std::max(0.f,c.x),g),
            std::pow(std::max(0.f,c.y),g),
            std::pow(std::max(0.f,c.z),g)};
}

// ─────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────
int main() {
    std::cout << "[RC-GI] Building scene..." << std::endl;
    auto scene  = buildScene();
    auto lights = buildLights();
    std::cout << "[RC-GI] Scene: " << scene.size() << " triangles, "
              << lights.size() << " lights" << std::endl;

    // Camera
    Camera cam;
    Mat4 view = Mat4::lookAt(cam.pos, cam.target, cam.up);
    Mat4 proj = Mat4::perspective(cam.fov, float(W)/float(H), cam.near_, cam.far_);

    // G-Buffer
    GBuf gbuf;
    std::cout << "[RC-GI] Rasterizing G-Buffer..." << std::endl;
    rasterize(gbuf, scene, view, proj);

    // ── Build Radiance Cascades ──
    std::cout << "[RC-GI] Building Radiance Cascades..." << std::endl;

    // World bounds
    float wxMin=-4.f, wxMax=4.f, wzMin=-4.f, wzMax=4.f;

    // Cascade 0: fine grid, 16 dirs, short range
    CascadeLevel c0;
    c0.gridW=16; c0.gridH=16; c0.numDirs=16; c0.maxRange=3.f; c0.spacing=0.5f;
    auto dirs0 = buildDirections(c0.numDirs);
    buildCascade(c0, scene, lights, dirs0, wxMin, wxMax, wzMin, wzMax, 1.5f);
    std::cout << "[RC-GI] Cascade 0 done (" << c0.probes.size() << " probes)" << std::endl;

    // Cascade 1: medium grid, 32 dirs, medium range
    CascadeLevel c1;
    c1.gridW=8; c1.gridH=8; c1.numDirs=32; c1.maxRange=6.f; c1.spacing=1.f;
    auto dirs1 = buildDirections(c1.numDirs);
    buildCascade(c1, scene, lights, dirs1, wxMin, wxMax, wzMin, wzMax, 2.5f);
    std::cout << "[RC-GI] Cascade 1 done (" << c1.probes.size() << " probes)" << std::endl;

    // Cascade 2: coarse grid, 64 dirs, long range
    CascadeLevel c2;
    c2.gridW=4; c2.gridH=4; c2.numDirs=64; c2.maxRange=12.f; c2.spacing=2.f;
    auto dirs2 = buildDirections(c2.numDirs);
    buildCascade(c2, scene, lights, dirs2, wxMin, wxMax, wzMin, wzMax, 3.0f);
    std::cout << "[RC-GI] Cascade 2 done (" << c2.probes.size() << " probes)" << std::endl;

    // Merge cascades top-down: c2 → c1, c1 → c0
    std::cout << "[RC-GI] Merging cascades..." << std::endl;
    mergeCascades(c1, c2, dirs1, dirs2);
    mergeCascades(c0, c1, dirs0, dirs1);

    // ── Deferred shading ──
    std::cout << "[RC-GI] Deferred shading pass..." << std::endl;
    Canvas canvas;

    for (int y=0; y<H; ++y) {
        for (int x=0; x<W; ++x) {
            int i = y*W+x;
            if (!gbuf.valid[i]) {
                // Background sky gradient
                float t = float(y)/H;
                canvas.color[i] = mix(Vec3(0.05f,0.08f,0.15f), Vec3(0.02f,0.03f,0.06f), t);
                continue;
            }
            Vec3 pos    = gbuf.worldPos[i];
            Vec3 normal = gbuf.normal[i];
            Vec3 albedo = gbuf.albedo[i];

            // Direct lighting
            Vec3 direct = directLight(pos, normal, albedo, lights, cam.pos);

            // Indirect lighting via Cascade 0
            Vec3 indirect = sampleCascade(c0, dirs0, pos, normal);

            // Combine: direct + indirect bounce * albedo
            Vec3 finalColor = direct + indirect * albedo * 0.5f;

            // Tone map + gamma
            finalColor = toneMap(finalColor);
            finalColor = gamma(finalColor);
            canvas.color[i] = finalColor.clamp01();
        }
    }

    // ── Write PNG ──
    std::cout << "[RC-GI] Writing output image..." << std::endl;
    std::vector<uint8_t> pixels(W * H * 3);
    for (int i=0; i<W*H; ++i) {
        pixels[i*3+0] = (uint8_t)(canvas.color[i].x * 255.f);
        pixels[i*3+1] = (uint8_t)(canvas.color[i].y * 255.f);
        pixels[i*3+2] = (uint8_t)(canvas.color[i].z * 255.f);
    }
    stbi_write_png("radiance_cascade_gi_output.png", W, H, 3, pixels.data(), W*3);
    std::cout << "[RC-GI] Done! Output: radiance_cascade_gi_output.png" << std::endl;
    return 0;
}
