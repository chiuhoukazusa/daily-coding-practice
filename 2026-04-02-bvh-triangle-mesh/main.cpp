/*
 * BVH Accelerated Triangle Mesh Renderer
 * 日期: 2026-04-02
 * 技术: BVH层次包围盒, Möller–Trumbore三角形相交, 法线插值,
 *       Phong着色, 程序化网格(Sphere/Plane/Bunny-like), 路径追踪
 *
 * 场景: Cornell Box + 多个三角形网格物体
 * 输出: bvh_output.png (800x600)
 */

#include <cmath>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>
#include <random>
#include <limits>
#include <cstring>
#include <cstdio>
#include <cassert>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "stb_image_write.h"
#pragma GCC diagnostic pop

// ========== Math ==========

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b)  const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b)       const { return x*b.x + y*b.y + z*b.z; }
    Vec3  cross(const Vec3& b)     const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float length2() const { return x*x + y*y + z*z; }
    float length()  const { return std::sqrt(length2()); }
    Vec3  normalized() const {
        float l = length();
        return (l > 1e-9f) ? (*this / l) : Vec3(0,1,0);
    }
    Vec3 abs() const { return {std::fabs(x), std::fabs(y), std::fabs(z)}; }
    float maxComp() const { return std::max({x, y, z}); }
    float minComp() const { return std::min({x, y, z}); }
};

inline Vec3 operator*(float t, const Vec3& v) { return v * t; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, float t) { return a + (b - a) * t; }
inline Vec3 clampVec(const Vec3& v, float lo, float hi) {
    return { std::max(lo, std::min(hi, v.x)),
             std::max(lo, std::min(hi, v.y)),
             std::max(lo, std::min(hi, v.z)) };
}
inline Vec3 reflect(const Vec3& d, const Vec3& n) { return d - 2.0f * d.dot(n) * n; }

// ========== Ray ==========

struct Ray {
    Vec3 origin, dir;
    mutable Vec3 invDir;
    mutable int sign[3];
    Ray() {}
    Ray(const Vec3& o, const Vec3& d) : origin(o), dir(d.normalized()) {
        invDir = Vec3(1.0f/dir.x, 1.0f/dir.y, 1.0f/dir.z);
        sign[0] = invDir.x < 0 ? 1 : 0;
        sign[1] = invDir.y < 0 ? 1 : 0;
        sign[2] = invDir.z < 0 ? 1 : 0;
    }
};

// ========== AABB ==========

struct AABB {
    Vec3 mn, mx;
    AABB() : mn(1e30f,1e30f,1e30f), mx(-1e30f,-1e30f,-1e30f) {}
    AABB(const Vec3& a, const Vec3& b) : mn(a), mx(b) {}
    void expand(const Vec3& p) {
        mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
    }
    void expand(const AABB& b) { expand(b.mn); expand(b.mx); }
    Vec3 centroid() const { return (mn + mx) * 0.5f; }
    float surfaceArea() const {
        Vec3 e = mx - mn;
        return 2.0f * (e.x*e.y + e.y*e.z + e.z*e.x);
    }
    bool intersect(const Ray& ray, float tmin, float tmax) const {
        // Slab method
        float bounds[2][3] = {
            {mn.x, mn.y, mn.z},
            {mx.x, mx.y, mx.z}
        };
        float tNear = tmin, tFar = tmax;
        for (int i = 0; i < 3; ++i) {
            float invD = (i==0) ? ray.invDir.x : (i==1) ? ray.invDir.y : ray.invDir.z;
            float t0 = (bounds[ray.sign[i]][i]   - ((i==0)?ray.origin.x:(i==1)?ray.origin.y:ray.origin.z)) * invD;
            float t1 = (bounds[1-ray.sign[i]][i] - ((i==0)?ray.origin.x:(i==1)?ray.origin.y:ray.origin.z)) * invD;
            tNear = std::max(tNear, t0);
            tFar  = std::min(tFar,  t1);
            if (tFar < tNear) return false;
        }
        return true;
    }
};

// ========== Material ==========

enum MaterialType { DIFFUSE, MIRROR, GLASS };

struct Material {
    Vec3 albedo;
    Vec3 emission;
    MaterialType type;
    float ior; // index of refraction for glass
    Material() : albedo(0.8f,0.8f,0.8f), emission(0,0,0), type(DIFFUSE), ior(1.5f) {}
    Material(Vec3 a, Vec3 e = Vec3(), MaterialType t = DIFFUSE, float i = 1.5f)
        : albedo(a), emission(e), type(t), ior(i) {}
};

// ========== Hit Info ==========

struct HitInfo {
    float t;
    Vec3 pos;
    Vec3 normal;
    Vec3 uv; // barycentric or texture coords
    Material mat;
    bool hit;
    HitInfo() : t(1e30f), hit(false) {}
};

// ========== Triangle ==========

struct Triangle {
    Vec3 v0, v1, v2;
    Vec3 n0, n1, n2; // per-vertex normals
    Material mat;
    AABB bounds() const {
        AABB b;
        b.expand(v0); b.expand(v1); b.expand(v2);
        // Tiny epsilon to avoid zero-thickness AABB
        b.mn = b.mn - Vec3(1e-4f, 1e-4f, 1e-4f);
        b.mx = b.mx + Vec3(1e-4f, 1e-4f, 1e-4f);
        return b;
    }
    Vec3 centroid() const { return (v0 + v1 + v2) * (1.0f/3.0f); }

    // Möller–Trumbore intersection
    bool intersect(const Ray& ray, float tmin, float tmax, HitInfo& hit) const {
        const float EPSILON = 1e-8f;
        Vec3 e1 = v1 - v0;
        Vec3 e2 = v2 - v0;
        Vec3 h  = ray.dir.cross(e2);
        float a = e1.dot(h);
        if (std::fabs(a) < EPSILON) return false;

        float f = 1.0f / a;
        Vec3 s  = ray.origin - v0;
        float u = f * s.dot(h);
        if (u < 0.0f || u > 1.0f) return false;

        Vec3 q  = s.cross(e1);
        float v = f * ray.dir.dot(q);
        if (v < 0.0f || u + v > 1.0f) return false;

        float t = f * e2.dot(q);
        if (t < tmin || t > tmax) return false;

        hit.t   = t;
        hit.pos = ray.origin + ray.dir * t;
        float w = 1.0f - u - v;
        // Interpolate normal
        hit.normal = (n0 * w + n1 * u + n2 * v).normalized();
        // Ensure normal faces the ray
        if (hit.normal.dot(ray.dir) > 0) hit.normal = hit.normal * -1.0f;
        hit.mat = mat;
        hit.hit = true;
        return true;
    }
};

// ========== BVH ==========

struct BVHNode {
    AABB bounds;
    int left;   // index into nodes, or -1 if leaf
    int right;  // index into nodes, or -1 if leaf
    int triStart, triCount; // for leaf nodes
};

class BVH {
public:
    std::vector<Triangle> tris;
    std::vector<BVHNode>  nodes;

    void build(std::vector<Triangle>&& triangles) {
        tris = std::move(triangles);
        nodes.clear();
        if (tris.empty()) return;
        std::vector<int> indices(tris.size());
        for (int i = 0; i < (int)tris.size(); ++i) indices[i] = i;
        buildRecursive(indices, 0, (int)indices.size());
    }

    bool intersect(const Ray& ray, float tmin, float tmax, HitInfo& hit) const {
        if (nodes.empty()) return false;
        return traverseNode(0, ray, tmin, tmax, hit);
    }

private:
    int buildRecursive(std::vector<int>& indices, int start, int end) {
        BVHNode node;
        node.triStart = 0; node.triCount = 0;
        node.left = node.right = -1;

        // Compute bounds
        AABB b;
        for (int i = start; i < end; ++i) b.expand(tris[indices[i]].bounds());
        node.bounds = b;

        int count = end - start;
        if (count <= 4) {
            // Leaf
            // Actually: rearrange tris so leaves are contiguous
            // We need a different approach: store sorted triangle list separately
            // For simplicity, store indices in node's leaf data
            // Use triStart as index into a separate leaf index array
            node.triStart = start;
            node.triCount = count;
            node.left = node.right = -1;
            int nodeIdx = (int)nodes.size();
            nodes.push_back(node);
            return nodeIdx;
        }

        // Find best split using SAH (simplified: midpoint split on longest axis)
        Vec3 ext = b.mx - b.mn;
        int axis = 0;
        if (ext.y > ext.x) axis = 1;
        if ((axis==0 ? ext.z > ext.x : ext.z > ext.y)) axis = 2;

        // Sort by centroid on axis
        std::sort(indices.begin() + start, indices.begin() + end, [&](int a, int b_) {
            Vec3 ca = tris[a].centroid();
            Vec3 cb = tris[b_].centroid();
            if (axis == 0) return ca.x < cb.x;
            if (axis == 1) return ca.y < cb.y;
            return ca.z < cb.z;
        });

        int mid = start + count / 2;
        int nodeIdx = (int)nodes.size();
        nodes.push_back(node); // placeholder

        int lc = buildRecursive(indices, start, mid);
        int rc = buildRecursive(indices, mid, end);

        nodes[nodeIdx].left      = lc;
        nodes[nodeIdx].right     = rc;
        nodes[nodeIdx].triCount  = 0; // not a leaf
        return nodeIdx;
    }

    // We need a sorted triangle array. Let's use an index array.
    // Actually, let's use a secondary sorted index array per leaf.
    // Since the recursive sort mutates `indices`, let's finalize by storing
    // a copy of the sorted indices in BVH and reference by triStart/triCount.

public:
    std::vector<int> sortedIndices; // set during build

    void buildWithSortedIndices(std::vector<Triangle>&& triangles) {
        tris = std::move(triangles);
        nodes.clear();
        sortedIndices.clear();
        if (tris.empty()) return;
        std::vector<int> indices(tris.size());
        for (int i = 0; i < (int)tris.size(); ++i) indices[i] = i;
        buildRec2(indices, 0, (int)indices.size());
    }

private:
    int buildRec2(std::vector<int>& indices, int start, int end) {
        BVHNode node;
        node.triStart = 0; node.triCount = 0;
        node.left = node.right = -1;

        AABB b;
        for (int i = start; i < end; ++i) b.expand(tris[indices[i]].bounds());
        node.bounds = b;

        int count = end - start;
        if (count <= 4) {
            // Leaf: append to sortedIndices
            node.triStart = (int)sortedIndices.size();
            node.triCount = count;
            for (int i = start; i < end; ++i) sortedIndices.push_back(indices[i]);
            int nodeIdx = (int)nodes.size();
            nodes.push_back(node);
            return nodeIdx;
        }

        // Split on longest axis
        Vec3 ext = b.mx - b.mn;
        int axis = 0;
        if (ext.y > ext.x) axis = 1;
        if ((axis==0 ? ext.z > ext.x : ext.z > ext.y)) axis = 2;

        std::sort(indices.begin() + start, indices.begin() + end, [&](int a, int bI) {
            Vec3 ca = tris[a].centroid();
            Vec3 cb = tris[bI].centroid();
            float va = (axis==0)?ca.x:(axis==1)?ca.y:ca.z;
            float vb = (axis==0)?cb.x:(axis==1)?cb.y:cb.z;
            return va < vb;
        });

        int mid = start + count / 2;
        int nodeIdx = (int)nodes.size();
        nodes.push_back(node); // placeholder

        int lc = buildRec2(indices, start, mid);
        int rc = buildRec2(indices, mid, end);

        nodes[nodeIdx].left      = lc;
        nodes[nodeIdx].right     = rc;
        nodes[nodeIdx].triCount  = 0;
        return nodeIdx;
    }

    bool traverseNode(int idx, const Ray& ray, float tmin, float tmax, HitInfo& best) const {
        const BVHNode& node = nodes[idx];
        if (!node.bounds.intersect(ray, tmin, tmax)) return false;

        if (node.triCount > 0) {
            // Leaf
            bool anyHit = false;
            for (int i = node.triStart; i < node.triStart + node.triCount; ++i) {
                HitInfo hit;
                if (tris[sortedIndices[i]].intersect(ray, tmin, tmax, hit)) {
                    if (hit.t < best.t) {
                        best = hit;
                        tmax = hit.t;
                        anyHit = true;
                    }
                }
            }
            return anyHit;
        }

        // Internal node
        bool hitLeft  = (node.left  >= 0) && traverseNode(node.left,  ray, tmin, tmax, best);
        float newMax  = hitLeft ? best.t : tmax;
        bool hitRight = (node.right >= 0) && traverseNode(node.right, ray, tmin, newMax, best);
        return hitLeft || hitRight;
    }
};

// ========== Mesh Generation ==========

// Generate a UV sphere mesh
void generateSphere(std::vector<Triangle>& tris, Vec3 center, float radius,
                    int stacks, int slices, Material mat) {
    // Generate vertices
    std::vector<Vec3> verts;
    std::vector<Vec3> normals;
    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * i / stacks;
        for (int j = 0; j <= slices; ++j) {
            float theta = 2.0f * M_PI * j / slices;
            Vec3 n(sinf(phi)*cosf(theta), cosf(phi), sinf(phi)*sinf(theta));
            verts.push_back(center + n * radius);
            normals.push_back(n);
        }
    }
    // Generate triangles
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int a = i*(slices+1) + j;
            int b = a + 1;
            int c = (i+1)*(slices+1) + j;
            int d = c + 1;
            Triangle t1;
            t1.v0=verts[a]; t1.v1=verts[c]; t1.v2=verts[b];
            t1.n0=normals[a]; t1.n1=normals[c]; t1.n2=normals[b];
            t1.mat = mat;
            tris.push_back(t1);

            Triangle t2;
            t2.v0=verts[b]; t2.v1=verts[c]; t2.v2=verts[d];
            t2.n0=normals[b]; t2.n1=normals[c]; t2.n2=normals[d];
            t2.mat = mat;
            tris.push_back(t2);
        }
    }
}

// Generate a quad (two triangles) for a flat surface
void addQuad(std::vector<Triangle>& tris,
             Vec3 v0, Vec3 v1, Vec3 v2, Vec3 v3, Material mat) {
    Vec3 e1 = v1 - v0, e2 = v2 - v0;
    Vec3 n = e1.cross(e2).normalized();
    Triangle t1, t2;
    t1.v0=v0; t1.v1=v1; t1.v2=v2;
    t1.n0=n;  t1.n1=n;  t1.n2=n;
    t1.mat = mat;

    t2.v0=v0; t2.v1=v2; t2.v2=v3;
    t2.n0=n;  t2.n1=n;  t2.n2=n;
    t2.mat = mat;

    tris.push_back(t1);
    tris.push_back(t2);
}

// Generate a box
void generateBox(std::vector<Triangle>& tris,
                 Vec3 mn, Vec3 mx, Material mat) {
    Vec3 a=mn, b=Vec3(mx.x,mn.y,mn.z), c=Vec3(mx.x,mx.y,mn.z), d=Vec3(mn.x,mx.y,mn.z);
    Vec3 e=Vec3(mn.x,mn.y,mx.z), f=Vec3(mx.x,mn.y,mx.z), g=mx, h=Vec3(mn.x,mx.y,mx.z);
    // 6 faces
    addQuad(tris, a, b, c, d, mat); // front (z=mn.z)
    addQuad(tris, f, e, h, g, mat); // back  (z=mx.z)
    addQuad(tris, e, a, d, h, mat); // left  (x=mn.x)
    addQuad(tris, b, f, g, c, mat); // right (x=mx.x)
    addQuad(tris, d, c, g, h, mat); // top   (y=mx.y)
    addQuad(tris, e, f, b, a, mat); // bottom(y=mn.y)
}

// Generate a torus mesh
void generateTorus(std::vector<Triangle>& tris, Vec3 center,
                   float R, float r, int nseg, int ntube, Material mat) {
    std::vector<Vec3> verts, normals;
    for (int i = 0; i <= nseg; ++i) {
        float u = 2.0f * M_PI * i / nseg;
        Vec3 circleCenter = center + Vec3(cosf(u)*R, 0, sinf(u)*R);
        for (int j = 0; j <= ntube; ++j) {
            float v = 2.0f * M_PI * j / ntube;
            Vec3 n(cosf(u)*cosf(v), sinf(v), sinf(u)*cosf(v));
            verts.push_back(circleCenter + n * r);
            normals.push_back(n);
        }
    }
    for (int i = 0; i < nseg; ++i) {
        for (int j = 0; j < ntube; ++j) {
            int a = i*(ntube+1) + j;
            int b = a + 1;
            int c = (i+1)*(ntube+1) + j;
            int d = c + 1;
            Triangle t1;
            t1.v0=verts[a]; t1.v1=verts[c]; t1.v2=verts[b];
            t1.n0=normals[a]; t1.n1=normals[c]; t1.n2=normals[b];
            t1.mat = mat;
            tris.push_back(t1);

            Triangle t2;
            t2.v0=verts[b]; t2.v1=verts[c]; t2.v2=verts[d];
            t2.n0=normals[b]; t2.n1=normals[c]; t2.n2=normals[d];
            t2.mat = mat;
            tris.push_back(t2);
        }
    }
}

// Generate an icosphere (higher quality sphere)
void generateIcosphere(std::vector<Triangle>& tris, Vec3 center, float radius,
                        int subdivisions, Material mat) {
    // Start with icosahedron
    const float phi = (1.0f + sqrtf(5.0f)) / 2.0f;
    std::vector<Vec3> verts = {
        {-1,  phi, 0}, { 1,  phi, 0}, {-1, -phi, 0}, { 1, -phi, 0},
        { 0, -1,  phi}, { 0,  1,  phi}, { 0, -1, -phi}, { 0,  1, -phi},
        { phi, 0, -1}, { phi, 0,  1}, {-phi, 0, -1}, {-phi, 0,  1}
    };
    for (auto& v : verts) v = v.normalized();
    std::vector<std::array<int,3>> faces = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    };
    // Subdivide
    for (int s = 0; s < subdivisions; ++s) {
        std::vector<std::array<int,3>> newFaces;
        for (auto& f : faces) {
            // Midpoints
            Vec3 m01 = (verts[f[0]] + verts[f[1]]).normalized();
            Vec3 m12 = (verts[f[1]] + verts[f[2]]).normalized();
            Vec3 m02 = (verts[f[0]] + verts[f[2]]).normalized();
            int i01 = (int)verts.size(); verts.push_back(m01);
            int i12 = (int)verts.size(); verts.push_back(m12);
            int i02 = (int)verts.size(); verts.push_back(m02);
            newFaces.push_back({f[0], i01, i02});
            newFaces.push_back({f[1], i12, i01});
            newFaces.push_back({f[2], i02, i12});
            newFaces.push_back({i01, i12, i02});
        }
        faces = std::move(newFaces);
    }
    for (auto& f : faces) {
        Triangle t;
        t.v0 = center + verts[f[0]] * radius;
        t.v1 = center + verts[f[1]] * radius;
        t.v2 = center + verts[f[2]] * radius;
        t.n0 = verts[f[0]];
        t.n1 = verts[f[1]];
        t.n2 = verts[f[2]];
        t.mat = mat;
        tris.push_back(t);
    }
}

// ========== Scene ==========

class Scene {
public:
    BVH bvh;
    Vec3 lightPos;
    Vec3 lightColor;
    float lightRadius;

    void build() {
        lightPos    = Vec3(0.0f, 2.8f, 0.0f);
        lightColor  = Vec3(15.0f, 12.0f, 9.0f);
        lightRadius = 0.4f;

        std::vector<Triangle> tris;

        // Cornell Box walls (600x600x600 cm scaled to [-3,3])
        float W = 3.0f, H = 3.0f, D = 3.0f;

        // Floor
        Material floor; floor.albedo = Vec3(0.85f, 0.82f, 0.75f);
        addQuad(tris,
            Vec3(-W,-H,-D), Vec3(W,-H,-D), Vec3(W,-H,D), Vec3(-W,-H,D), floor);

        // Ceiling
        Material ceiling; ceiling.albedo = Vec3(0.85f, 0.85f, 0.85f);
        addQuad(tris,
            Vec3(-W,H,-D), Vec3(-W,H,D), Vec3(W,H,D), Vec3(W,H,-D), ceiling);

        // Back wall
        Material backWall; backWall.albedo = Vec3(0.75f, 0.75f, 0.75f);
        addQuad(tris,
            Vec3(-W,-H,-D), Vec3(-W,H,-D), Vec3(W,H,-D), Vec3(W,-H,-D), backWall);

        // Left wall (red)
        Material leftWall; leftWall.albedo = Vec3(0.75f, 0.15f, 0.15f);
        addQuad(tris,
            Vec3(-W,-H,-D), Vec3(-W,-H,D), Vec3(-W,H,D), Vec3(-W,H,-D), leftWall);

        // Right wall (green)
        Material rightWall; rightWall.albedo = Vec3(0.15f, 0.65f, 0.15f);
        addQuad(tris,
            Vec3(W,-H,-D), Vec3(W,H,-D), Vec3(W,H,D), Vec3(W,-H,D), rightWall);

        // Front wall (invisible, camera side)
        Material frontWall; frontWall.albedo = Vec3(0.75f, 0.75f, 0.75f);
        addQuad(tris,
            Vec3(-W,-H,D), Vec3(W,-H,D), Vec3(W,H,D), Vec3(-W,H,D), frontWall);

        // Area light panel on ceiling
        Material lightMat;
        lightMat.albedo   = Vec3(1.0f, 1.0f, 1.0f);
        lightMat.emission = Vec3(15.0f, 12.0f, 9.0f);
        float lw = 0.5f;
        addQuad(tris,
            Vec3(-lw, H-0.01f, -lw), Vec3(lw, H-0.01f, -lw),
            Vec3(lw, H-0.01f,  lw), Vec3(-lw, H-0.01f,  lw), lightMat);

        // Icosphere (glossy white) — main sphere
        Material sphereMat1;
        sphereMat1.albedo = Vec3(0.95f, 0.9f, 0.85f);
        sphereMat1.type   = MIRROR;
        generateIcosphere(tris, Vec3(-1.2f, -H + 1.2f, -0.5f), 1.2f, 3, sphereMat1);

        // Smaller colored sphere (diffuse orange)
        Material sphereMat2;
        sphereMat2.albedo = Vec3(0.95f, 0.55f, 0.15f);
        generateIcosphere(tris, Vec3(1.3f, -H + 0.65f, 0.2f), 0.65f, 3, sphereMat2);

        // Torus (blue metallic)
        Material torusMat;
        torusMat.albedo = Vec3(0.2f, 0.4f, 0.9f);
        torusMat.type   = DIFFUSE;
        generateTorus(tris, Vec3(0.0f, -H + 0.35f, -0.8f), 0.7f, 0.25f, 24, 16, torusMat);

        // Tall box
        Material tallBox;
        tallBox.albedo = Vec3(0.85f, 0.80f, 0.70f);
        generateBox(tris, Vec3(0.8f, -H, -1.8f), Vec3(1.8f, -H+2.2f, -0.7f), tallBox);

        printf("Total triangles: %zu\n", tris.size());
        bvh.buildWithSortedIndices(std::move(tris));
        printf("BVH nodes: %zu\n", bvh.nodes.size());
    }

    HitInfo intersect(const Ray& ray) const {
        HitInfo hit;
        bvh.intersect(ray, 1e-4f, 1e30f, hit);
        return hit;
    }
};

// ========== Renderer ==========

// Thread-local RNG
thread_local std::mt19937 rng(std::random_device{}());
thread_local std::uniform_real_distribution<float> dist01(0.0f, 1.0f);

inline float rand01() { return dist01(rng); }

Vec3 sampleHemisphereCosine(const Vec3& normal) {
    float r1 = rand01(), r2 = rand01();
    float phi = 2.0f * M_PI * r1;
    float sinTheta = sqrtf(r2);
    float cosTheta = sqrtf(1.0f - r2);

    // Tangent space
    Vec3 up = (std::fabs(normal.x) < 0.9f) ? Vec3(1,0,0) : Vec3(0,1,0);
    Vec3 tangent = up.cross(normal).normalized();
    Vec3 bitangent = normal.cross(tangent);

    return (tangent * (cosf(phi)*sinTheta) +
            bitangent * (sinf(phi)*sinTheta) +
            normal * cosTheta).normalized();
}

Vec3 traceRay(const Scene& scene, Ray ray, int depth) {
    Vec3 color(0,0,0);
    Vec3 throughput(1,1,1);

    for (int d = 0; d <= depth; ++d) {
        HitInfo hit = scene.intersect(ray);
        if (!hit.hit) {
            // Sky
            float t = 0.5f * (ray.dir.y + 1.0f);
            Vec3 sky = Vec3(0.1f, 0.15f, 0.25f) * t + Vec3(0.05f, 0.05f, 0.08f) * (1-t);
            color += throughput * sky;
            break;
        }

        // Emission
        color += throughput * hit.mat.emission;

        if (d == depth) break;

        // Russian roulette
        if (d > 3) {
            float p = std::min(0.95f, throughput.maxComp());
            if (rand01() > p) break;
            throughput = throughput * (1.0f / p);
        }

        switch (hit.mat.type) {
        case DIFFUSE: {
            Vec3 newDir = sampleHemisphereCosine(hit.normal);
            throughput = throughput * hit.mat.albedo;
            ray = Ray(hit.pos + hit.normal * 1e-4f, newDir);
            break;
        }
        case MIRROR: {
            Vec3 newDir = reflect(ray.dir, hit.normal).normalized();
            throughput = throughput * hit.mat.albedo;
            ray = Ray(hit.pos + hit.normal * 1e-4f, newDir);
            break;
        }
        case GLASS: {
            float eta = 1.0f / hit.mat.ior;
            Vec3 n = hit.normal;
            float cosI = -ray.dir.dot(n);
            if (cosI < 0) { n = n * -1.0f; cosI = -cosI; eta = hit.mat.ior; }
            float cos2T = 1.0f - eta*eta*(1.0f - cosI*cosI);
            if (cos2T < 0) {
                // TIR
                Vec3 newDir = reflect(ray.dir, n).normalized();
                throughput = throughput * hit.mat.albedo;
                ray = Ray(hit.pos + n * 1e-4f, newDir);
            } else {
                Vec3 refracted = (ray.dir * eta + n * (eta*cosI - sqrtf(cos2T))).normalized();
                throughput = throughput * hit.mat.albedo;
                ray = Ray(hit.pos - n * 1e-4f, refracted);
            }
            break;
        }
        }
    }
    return color;
}

// ========== Camera ==========

struct Camera {
    Vec3 origin, lowerLeft, horizontal, vertical;
    Camera(Vec3 from, Vec3 at, Vec3 up, float fovY, float aspect) {
        float theta = fovY * M_PI / 180.0f;
        float h = tanf(theta / 2.0f);
        float vph = 2.0f * h;
        float vpw = aspect * vph;
        Vec3 w = (from - at).normalized();
        Vec3 u = up.cross(w).normalized();
        Vec3 v = w.cross(u);
        origin     = from;
        horizontal = u * vpw;
        vertical   = v * vph;
        lowerLeft  = origin - horizontal * 0.5f - vertical * 0.5f - w;
    }
    Ray getRay(float s, float t) const {
        Vec3 dir = lowerLeft + horizontal * s + vertical * t - origin;
        return Ray(origin, dir);
    }
};

// ========== Tone Mapping ==========

Vec3 acesFilm(Vec3 x) {
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    // Correct ACES:
    x = Vec3(
        (x.x * (a*x.x + b)) / (x.x * (c*x.x + d) + e),
        (x.y * (a*x.y + b)) / (x.y * (c*x.y + d) + e),
        (x.z * (a*x.z + b)) / (x.z * (c*x.z + d) + e)
    );
    return clampVec(x, 0, 1);
}

Vec3 gammaCorrect(Vec3 c) {
    return Vec3(powf(c.x, 1.0f/2.2f),
                powf(c.y, 1.0f/2.2f),
                powf(c.z, 1.0f/2.2f));
}

// ========== Main ==========

int main() {
    const int W = 800, H = 600;
    const int SPP = 64; // samples per pixel
    const int MAX_DEPTH = 6;

    printf("=== BVH Accelerated Triangle Mesh Renderer ===\n");
    printf("Resolution: %dx%d, SPP: %d, Max depth: %d\n", W, H, SPP, MAX_DEPTH);

    // Build scene
    Scene scene;
    scene.build();

    // Camera
    Camera cam(Vec3(0, 0, 5.5f), Vec3(0, 0, 0), Vec3(0, 1, 0), 50.0f, (float)W/H);

    // Render
    std::vector<uint8_t> pixels(W * H * 3);

    printf("Rendering...\n");
    for (int y = 0; y < H; ++y) {
        if (y % 50 == 0) {
            printf("  %.1f%%\n", 100.0f * y / H);
            fflush(stdout);
        }
        for (int x = 0; x < W; ++x) {
            Vec3 color(0,0,0);
            for (int s = 0; s < SPP; ++s) {
                float u = (x + rand01()) / W;
                float v = (y + rand01()) / H;
                // Flip V so Y=0 is bottom
                Ray ray = cam.getRay(u, 1.0f - v);
                color += traceRay(scene, ray, MAX_DEPTH);
            }
            color = color * (1.0f / SPP);
            color = acesFilm(color);
            color = gammaCorrect(color);
            color = clampVec(color, 0, 1);

            int idx = (y * W + x) * 3;
            pixels[idx+0] = (uint8_t)(color.x * 255.99f);
            pixels[idx+1] = (uint8_t)(color.y * 255.99f);
            pixels[idx+2] = (uint8_t)(color.z * 255.99f);
        }
    }
    printf("  100.0%% - Done!\n");

    // Save
    const char* filename = "bvh_output.png";
    if (!stbi_write_png(filename, W, H, 3, pixels.data(), W * 3)) {
        fprintf(stderr, "ERROR: Failed to write %s\n", filename);
        return 1;
    }

    printf("Output saved: %s\n", filename);

    // Quick stats
    float sum = 0, sumSq = 0;
    for (auto p : pixels) { float f = p/255.0f; sum += f; sumSq += f*f; }
    int N = (int)pixels.size();
    float mean = sum / N;
    float stddev = sqrtf(sumSq/N - mean*mean);
    printf("Pixel stats: mean=%.1f stddev=%.1f\n", mean*255, stddev*255);

    return 0;
}
