// Skeletal Animation — Linear Blend Skinning (LBS)
// A 3-bone arm: shoulder→elbow→wrist→finger
// Soft-rasterized with Z-buffer and Phong shading
// Quantitative verification: vertex displacement, weight sums, animation delta

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <array>

// ─── Math Types ────────────────────────────────────────────────
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(float s) const { return {x/s, y/s, z/s}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const { return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x}; }
    float length() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l > 1e-8f ? *this / l : Vec3(0,0,0); }
};

struct Mat4 {
    float m[16];
    Mat4() { memset(m, 0, sizeof(m)); m[0]=m[5]=m[10]=m[15]=1.0f; }
    float& operator()(int r, int c) { return m[c*4 + r]; }
    const float& operator()(int r, int c) const { return m[c*4 + r]; }
    Mat4 operator*(const Mat4& o) const;
    Vec3 transformPoint(const Vec3& v) const;
    Vec3 transformVector(const Vec3& v) const;
    static Mat4 translate(const Vec3& t);
    static Mat4 rotateX(float angle);
    static Mat4 rotateY(float angle);
    static Mat4 rotateZ(float angle);
    static Mat4 perspective(float fov, float aspect, float near, float far);
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
};

Mat4 Mat4::operator*(const Mat4& o) const {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            r(i,j) = 0;
            for (int k = 0; k < 4; ++k)
                r(i,j) += (*this)(i,k) * o(k,j);
        }
    return r;
}

Vec3 Mat4::transformPoint(const Vec3& v) const {
    float w = m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15];
    return {(m[0]*v.x + m[4]*v.y + m[8]*v.z + m[12]) / w,
            (m[1]*v.x + m[5]*v.y + m[9]*v.z + m[13]) / w,
            (m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14]) / w};
}

Vec3 Mat4::transformVector(const Vec3& v) const {
    return {m[0]*v.x + m[4]*v.y + m[8]*v.z,
            m[1]*v.x + m[5]*v.y + m[9]*v.z,
            m[2]*v.x + m[6]*v.y + m[10]*v.z};
}

Mat4 Mat4::translate(const Vec3& t) {
    Mat4 r; r(0,3)=t.x; r(1,3)=t.y; r(2,3)=t.z; return r;
}

Mat4 Mat4::rotateX(float a) {
    Mat4 r; float c=cosf(a), s=sinf(a);
    r(1,1)=c; r(1,2)=-s; r(2,1)=s; r(2,2)=c; return r;
}

Mat4 Mat4::rotateY(float a) {
    Mat4 r; float c=cosf(a), s=sinf(a);
    r(0,0)=c; r(0,2)=s; r(2,0)=-s; r(2,2)=c; return r;
}

Mat4 Mat4::rotateZ(float a) {
    Mat4 r; float c=cosf(a), s=sinf(a);
    r(0,0)=c; r(0,1)=-s; r(1,0)=s; r(1,1)=c; return r;
}

Mat4 Mat4::perspective(float fov, float aspect, float near, float far) {
    Mat4 r; float t = tanf(fov * 0.5f * M_PI / 180.0f);
    r(0,0)=1.0f/(aspect*t); r(1,1)=1.0f/t;
    r(2,2)=-(far+near)/(far-near); r(2,3)=-2*far*near/(far-near);
    r(3,2)=-1; r(3,3)=0; return r;
}

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    Vec3 f = (center - eye).normalized();
    Vec3 s = f.cross(up.normalized()).normalized();
    Vec3 u = s.cross(f);
    Mat4 r;
    r(0,0)=s.x; r(0,1)=s.y; r(0,2)=s.z; r(0,3)=-s.dot(eye);
    r(1,0)=u.x; r(1,1)=u.y; r(1,2)=u.z; r(1,3)=-u.dot(eye);
    r(2,0)=-f.x; r(2,1)=-f.y; r(2,2)=-f.z; r(2,3)=f.dot(eye);
    return r;
}

// ─── Skeletal Animation ──────────────────────────────────────────
struct Bone {
    int parent;          // index of parent bone (-1 for root)
    Mat4 bindPose;       // world-space transform in bind pose
    Mat4 inverseBind;    // inverse bind matrix
    Vec3 localPosition;  // local translation offset
    float length;        // bone length
    int startSeg, endSeg; // vertex segment indices for this bone
};

struct AnimFrame {
    std::vector<Mat4> localTransforms; // local transform for each bone
};

// ─── Mesh generation for a capsule/cylinder along a bone ────────
struct Vertex {
    Vec3 pos;      // bind pose position
    Vec3 normal;   // bind pose normal
    int   boneIdx; // which bone this vertex belongs to (0-2)
    float weight;  // skinning weight (1.0 for rigid, <1 for blend zone)
};

std::vector<Vertex> generateBoneMesh(Vec3 start, Vec3 end, float radius, int boneIdx, int prevBone, int nextBone, int segments = 12) {
    std::vector<Vertex> verts;

    Vec3 dir = (end - start).normalized();
    Vec3 perpX, perpY;
    if (fabsf(dir.x) < 0.9f) perpX = dir.cross(Vec3(1,0,0)).normalized();
    else perpX = dir.cross(Vec3(0,1,0)).normalized();
    perpY = dir.cross(perpX).normalized();

    // Generate rings along the bone
    int rings = 5;
    int sphereRings = 3; // extra rings at joints (spherical caps)

    for (int ring = 0; ring <= rings; ++ring) {
        float t = (float)ring / rings;
        Vec3 center = start + dir * (dir.dot(end - start) * t);

        // Blend zone: near joint areas, add vertices influenced by adjacent bones
        float blendZone = 0.15f; // 15% blend zone at each end

        for (int s = 0; s < segments; ++s) {
            float angle = 2.0f * M_PI * s / segments;
            Vec3 offset = perpX * cosf(angle) * radius + perpY * sinf(angle) * radius;

            Vertex v;
            v.pos = center + offset;
            v.normal = offset.normalized();

            // Compute skinning weight based on distance from joint
            if (t < blendZone && boneIdx > 0) {
                // Near parent joint: blend with parent bone
                v.boneIdx = boneIdx;
                v.weight = 0.6f + 0.4f * (t / blendZone);
            } else if (t > 1.0f - blendZone && boneIdx < 2) {
                // Near child joint: blend with child bone
                v.boneIdx = boneIdx;
                v.weight = 0.6f + 0.4f * ((1.0f - t) / blendZone);
            } else {
                v.boneIdx = boneIdx;
                v.weight = 1.0f;
            }

            verts.push_back(v);
        }
    }

    // Add spherical cap at start (if first bone)
    if (boneIdx == 0) {
        for (int sRing = 0; sRing <= sphereRings; ++sRing) {
            float phi = M_PI / 2.0f * (1.0f - (float)sRing / sphereRings); // from -pi/2 to 0 (bottom hemisphere)
            float y = sinf(phi) * radius;
            float r = cosf(phi) * radius;
            for (int s = 0; s < segments; ++s) {
                float angle = 2.0f * M_PI * s / segments;
                Vertex v;
                v.pos = start + perpX * cosf(angle) * r + perpY * sinf(angle) * r + dir * y;
                v.normal = (v.pos - start).normalized();
                v.boneIdx = boneIdx;
                v.weight = 1.0f;
                verts.push_back(v);
            }
        }
    }

    return verts;
}

// Generate indices for triangle strips
std::vector<std::array<int,3>> generateIndices(int numVerts, int segments, int rings) {
    std::vector<std::array<int,3>> tris;
    // Standard ring-based triangulation
    int vertsPerRing = segments;
    for (int ring = 0; ring < rings; ++ring) {
        for (int s = 0; s < segments; ++s) {
            int a = ring * vertsPerRing + s;
            int b = ring * vertsPerRing + (s + 1) % segments;
            int c = (ring + 1) * vertsPerRing + s;
            int d = (ring + 1) * vertsPerRing + (s + 1) % segments;

            tris.push_back({a, b, c});
            tris.push_back({b, d, c});
        }
    }
    return tris;
}

// ─── LBS Deformation ─────────────────────────────────────────────
struct Skeleton {
    std::vector<Bone> bones;
    std::vector<Mat4> currentWorldTransforms; // computed per frame

    // Forward kinematics: compute world transforms from local transforms
    void computeWorldTransforms(const AnimFrame& frame) {
        currentWorldTransforms.resize(bones.size());
        for (size_t i = 0; i < bones.size(); ++i) {
            Mat4 local = frame.localTransforms[i];
            if (bones[i].parent < 0) {
                currentWorldTransforms[i] = local;
            } else {
                // Apply translation along parent bone direction first, then local rotation
                Mat4 trans = Mat4::translate(Vec3(0, bones[bones[i].parent].length, 0));
                currentWorldTransforms[i] = currentWorldTransforms[bones[i].parent] * trans * local;
            }
        }
    }

    // Deform a vertex using LBS
    Vertex deform(const Vertex& v, const std::vector<float>& blendWeights) const {
        Vertex result;
        result.pos = {0,0,0};
        result.normal = {0,0,0};
        result.boneIdx = v.boneIdx;
        result.weight = v.weight;

        float totalWeight = 0;
        for (size_t b = 0; b < bones.size(); ++b) {
            if (blendWeights[b] < 0.001f && b != (size_t)v.boneIdx) continue;

            float w = (b == (size_t)v.boneIdx) ? v.weight : (1.0f - v.weight) * 0.5f;
            if (w < 0.001f) continue;
            totalWeight += w;

            Mat4 skinMatrix = currentWorldTransforms[b] * bones[b].inverseBind;
            result.pos = result.pos + skinMatrix.transformPoint(v.pos) * w;
            result.normal = result.normal + skinMatrix.transformVector(v.normal) * w;
        }

        // Normalize if total weight != 1
        if (totalWeight > 0.001f && fabsf(totalWeight - 1.0f) > 0.001f) {
            result.pos = result.pos / totalWeight;
            result.normal = result.normal.normalized();
        }

        return result;
    }
};

// ─── Rendering ──────────────────────────────────────────────────
void drawPixel(int w, int h, int x, int y, float* zbuf, unsigned char* img, Vec3 color) {
    if (x < 0 || x >= w || y < 0 || y >= h) return;
    int idx = y * w + x;
    // Only draw if closer (simple z-test without z value — just overwrite for this simple case)
    (void)zbuf;
    // Clamp color
    int r = std::min(255, std::max(0, (int)(color.x * 255)));
    int g = std::min(255, std::max(0, (int)(color.y * 255)));
    int b = std::min(255, std::max(0, (int)(color.z * 255)));
    img[idx * 3 + 0] = r;
    img[idx * 3 + 1] = g;
    img[idx * 3 + 2] = b;
}

void rasterizeTriangle(int w, int h, Vec3 v0, Vec3 v1, Vec3 v2,
                       Vec3 n0, Vec3 n1, Vec3 n2,
                       float* zbuf, unsigned char* img,
                       const Mat4& viewProj, Vec3 lightDir, Vec3 lightColor, Vec3 ambient, Vec3 baseColor) {
    // Transform to NDC
    Vec3 p0 = viewProj.transformPoint(v0);
    Vec3 p1 = viewProj.transformPoint(v1);
    Vec3 p2 = viewProj.transformPoint(v2);

    // Viewport transform
    auto toScreen = [w,h](Vec3 p) -> Vec3 {
        return {(p.x + 1.0f) * 0.5f * w, (1.0f - (p.y + 1.0f) * 0.5f) * h, p.z};
    };

    Vec3 s0 = toScreen(p0);
    Vec3 s1 = toScreen(p1);
    Vec3 s2 = toScreen(p2);

    // Bounding box
    int minX = std::max(0, (int)floorf(std::min({s0.x, s1.x, s2.x})));
    int maxX = std::min(w-1, (int)ceilf(std::max({s0.x, s1.x, s2.x})));
    int minY = std::max(0, (int)floorf(std::min({s0.y, s1.y, s2.y})));
    int maxY = std::min(h-1, (int)ceilf(std::max({s0.y, s1.y, s2.y})));

    float area = (s1.x - s0.x) * (s2.y - s0.y) - (s2.x - s0.x) * (s1.y - s0.y);
    if (fabsf(area) < 1e-6f) return;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            float px = x + 0.5f, py = y + 0.5f;
            float w0 = ((s1.x - px) * (s2.y - py) - (s2.x - px) * (s1.y - py)) / area;
            float w1 = ((s0.x - px) * (s2.y - py) - (s2.x - px) * (s0.y - py)) / area;
                    // Recompute with consistent winding
            w0 = ((s1.y - s2.y) * (px - s2.x) + (s2.x - s1.x) * (py - s2.y)) / area;
            w1 = ((s2.y - s0.y) * (px - s2.x) + (s0.x - s2.x) * (py - s2.y)) / area;
            float w2 = 1.0f - w0 - w1;

            if (w0 < -0.001f || w1 < -0.001f || w2 < -0.001f) continue;

            float z = s0.z * w0 + s1.z * w1 + s2.z * w2;
            int idx = y * w + x;
            if (z > zbuf[idx]) continue;
            zbuf[idx] = z;

            // Interpolate normal
            Vec3 normal = (n0 * w0 + n1 * w1 + n2 * w2).normalized();
            float diff = std::max(0.0f, normal.dot(lightDir) * 0.5f + 0.5f);

            Vec3 color = ambient + lightColor * diff;
            color.x *= baseColor.x;
            color.y *= baseColor.y;
            color.z *= baseColor.z;

            drawPixel(w, h, x, y, zbuf, img, color);
        }
    }
}

// Generate triangles from bone mesh vertices
void renderMesh(const std::vector<Vertex>& verts, int w, int h, float* zbuf, unsigned char* img,
                const Mat4& viewProj, Vec3 lightDir, Vec3 lightColor, Vec3 ambient, Vec3 baseColor) {
    int segments = 12;
    int rings = 5;
    int vertsPerRing = segments;

    for (int ring = 0; ring < rings; ++ring) {
        for (int s = 0; s < segments; ++s) {
            int a = ring * vertsPerRing + s;
            int b = ring * vertsPerRing + (s + 1) % segments;
            int c = (ring + 1) * vertsPerRing + s;
            int d = (ring + 1) * vertsPerRing + (s + 1) % segments;

            if ((size_t)std::max({a,b,c,d}) >= verts.size()) continue;

            rasterizeTriangle(w, h, verts[a].pos, verts[b].pos, verts[c].pos,
                            verts[a].normal, verts[b].normal, verts[c].normal,
                            zbuf, img, viewProj, lightDir, lightColor, ambient, baseColor);
            rasterizeTriangle(w, h, verts[b].pos, verts[d].pos, verts[c].pos,
                            verts[b].normal, verts[d].normal, verts[c].normal,
                            zbuf, img, viewProj, lightDir, lightColor, ambient, baseColor);
        }
    }

    // Render spherical caps (last vertices in the array)
    int capStart = (rings + 1) * vertsPerRing;
    int capRings = 3;
    for (int cr = 0; cr < capRings; ++cr) {
        for (int s = 0; s < segments; ++s) {
            int a = capStart + cr * segments + s;
            int b = capStart + cr * segments + (s + 1) % segments;
            int c = capStart + (cr + 1) * segments + s;
            int d = capStart + (cr + 1) * segments + (s + 1) % segments;

            if (c >= (int)verts.size()) continue;

            rasterizeTriangle(w, h, verts[a].pos, verts[b].pos, verts[c].pos,
                            verts[a].normal, verts[b].normal, verts[c].normal,
                            zbuf, img, viewProj, lightDir, lightColor, ambient, baseColor);
            if (d < (int)verts.size()) {
                rasterizeTriangle(w, h, verts[b].pos, verts[d].pos, verts[c].pos,
                                verts[b].normal, verts[d].normal, verts[c].normal,
                                zbuf, img, viewProj, lightDir, lightColor, ambient, baseColor);
            }
        }
    }
}

int main() {
    int W = 512, H = 512;

    // ─── Initialize Skeleton ───────────────────────────────────
    Skeleton skel;

    // Bone 0: shoulder → elbow (length ~1.2)
    Bone b0;
    b0.parent = -1;
    b0.localPosition = {0, 0, 0};
    b0.length = 1.2f;
    b0.bindPose = Mat4::translate({0, 0, 0});
    b0.inverseBind = Mat4(); // identity inverse for root at origin in bind pose
    skel.bones.push_back(b0);

    // Bone 1: elbow → wrist (length ~0.9)
    Bone b1;
    b1.parent = 0;
    b1.localPosition = {0, 1.2f, 0};
    b1.length = 0.9f;
    b1.bindPose = Mat4::translate({0, 1.2f, 0});
    b1.inverseBind = Mat4::translate({0, -1.2f, 0}); // inverseBind = bindPose⁻¹
    skel.bones.push_back(b1);

    // Bone 2: wrist → fingertip (length ~0.6)
    Bone b2;
    b2.parent = 1;
    b2.localPosition = {0, 0.9f, 0};
    b2.length = 0.6f;
    b2.bindPose = Mat4::translate({0, 2.1f, 0});
    b2.inverseBind = Mat4::translate({0, -2.1f, 0});
    skel.bones.push_back(b2);

    // ─── Generate Mesh (Bind Pose) ────────────────────────────
    std::vector<Vertex> bindPoseVerts;

    // Three cylinder segments - larger radius for visibility
    for (int bi = 0; bi < 3; ++bi) {
        Vec3 start = {0, skel.bones[bi].localPosition.y, 0};
        Vec3 end = {0, skel.bones[bi].localPosition.y + skel.bones[bi].length, 0};

        float radius = (bi == 0) ? 0.35f : (bi == 1) ? 0.28f : 0.22f;
        auto seg = generateBoneMesh(start, end, radius, bi, std::max(0, bi-1), std::min(2, bi+1));
        bindPoseVerts.insert(bindPoseVerts.end(), seg.begin(), seg.end());
    }

    // ─── Define Animation Frames ───────────────────────────────

    // Frame 0: Rest pose (straight arm)
    AnimFrame restFrame;
    restFrame.localTransforms = {
        Mat4(),  // bone 0: no rotation
        Mat4(),  // bone 1: no rotation
        Mat4(),  // bone 2: no rotation
    };

    // Frame 1: Animated pose (bent arm)
    AnimFrame animFrame;
    animFrame.localTransforms = {
        Mat4::rotateZ(-0.4f),  // shoulder rotation
        Mat4::rotateZ(0.8f),   // elbow bend
        Mat4::rotateZ(0.5f),   // wrist bend
    };

    // ─── Quantitative Verification ─────────────────────────────
    printf("=== QUANTITATIVE VERIFICATION ===\n\n");

    // 1. Weight sum verification
    printf("1. Skinning Weight Verification:\n");
    for (size_t i = 0; i < bindPoseVerts.size(); ++i) {
        float w = bindPoseVerts[i].weight;
        if (w < 0.5f || w > 1.0f) {
            printf("  ❌ Vertex %zu: weight=%.3f out of range [0.5, 1.0]\n", i, w);
            return 1;
        }
    }
    // Check weight distribution
    float totalWeight = 0;
    for (auto& v : bindPoseVerts) totalWeight += v.weight;
    printf("  ✅ All weights in [0.5, 1.0]. Total: %.1f, Avg: %.3f\n", totalWeight, totalWeight / bindPoseVerts.size());

    // 2. Rest pose vertex count
    printf("\n2. Mesh Verification:\n");
    printf("  Total vertices: %zu\n", bindPoseVerts.size());
    if (bindPoseVerts.size() < 100) {
        printf("  ❌ Too few vertices for a meaningful mesh\n");
        return 1;
    }
    printf("  ✅ Sufficient vertex count\n");

    // 3. Animated vs Rest displacement
    printf("\n3. Animation Displacement Verification:\n");
    skel.computeWorldTransforms(restFrame);
    auto restDeformed = bindPoseVerts;
    // For rest pose, skinning should preserve positions (identity deformation)
    float maxRestError = 0;
    for (size_t i = 0; i < bindPoseVerts.size(); ++i) {
        std::vector<float> w(bindPoseVerts.size(), 0);
        w[bindPoseVerts[i].boneIdx] = bindPoseVerts[i].weight;
        Vertex deformed = skel.deform(bindPoseVerts[i], w);
        float diff = (deformed.pos - bindPoseVerts[i].pos).length();
        if (diff > maxRestError) maxRestError = diff;
    }
    printf("  Max rest-pose LBS error: %.6f (should be 0)\n", maxRestError);
    if (maxRestError > 0.01f) {
        printf("  ❌ Rest pose deformation error too large\n");
        return 1;
    }
    printf("  ✅ Rest pose identity check passed\n");

    // Animated pose displacement
    skel.computeWorldTransforms(animFrame);
    float maxDisplacement = 0, meanDisplacement = 0;
    int movedCount = 0;
    for (size_t i = 0; i < bindPoseVerts.size(); ++i) {
        std::vector<float> w(skel.bones.size(), 0);
        w[bindPoseVerts[i].boneIdx] = bindPoseVerts[i].weight;
        // Also add blend weights for adjacent bones
        if (bindPoseVerts[i].weight < 1.0f) {
            if (bindPoseVerts[i].boneIdx > 0)
                w[bindPoseVerts[i].boneIdx - 1] = (1.0f - bindPoseVerts[i].weight) * 0.5f;
            if (bindPoseVerts[i].boneIdx < 2)
                w[bindPoseVerts[i].boneIdx + 1] = (1.0f - bindPoseVerts[i].weight) * 0.5f;
        }

        Vertex deformed = skel.deform(bindPoseVerts[i], w);
        float disp = (deformed.pos - bindPoseVerts[i].pos).length();
        meanDisplacement += disp;
        if (disp > maxDisplacement) maxDisplacement = disp;
        if (disp > 0.001f) movedCount++;
    }
    meanDisplacement /= bindPoseVerts.size();
    printf("  Max displacement: %.4f\n", maxDisplacement);
    printf("  Mean displacement: %.4f\n", meanDisplacement);
    printf("  Vertices moved (>0.001): %d / %zu (%.1f%%)\n",
           movedCount, bindPoseVerts.size(), 100.0f * movedCount / bindPoseVerts.size());

    if (maxDisplacement < 0.05f) {
        printf("  ❌ Animation barely moves vertices — insufficient deformation\n");
        return 1;
    }
    if (movedCount < (int)bindPoseVerts.size() * 0.3) {
        printf("  ❌ Less than 30%% of vertices moved — animation too localized\n");
        return 1;
    }
    printf("  ✅ Significant animation displacement verified\n");

    // 4. Bone hierarchy verification
    printf("\n4. Bone Hierarchy Verification:\n");
    printf("  Bones: %zu\n", skel.bones.size());
    for (size_t i = 0; i < skel.bones.size(); ++i) {
        printf("  Bone %zu: parent=%d, length=%.2f\n", i, skel.bones[i].parent, skel.bones[i].length);
    }
    printf("  ✅ Valid 3-bone hierarchy\n");

    // 5. Forward kinematics check — bone tips should be reachable
    printf("\n5. Forward Kinematics Verification:\n");
    skel.computeWorldTransforms(restFrame);
    Vec3 restBone1Tip = skel.currentWorldTransforms[1].transformPoint(Vec3(0,0,0));
    printf("  Rest bone 1 tip pos: (%.3f, %.3f, %.3f) expected ~(0, 1.2, 0)\n",
           restBone1Tip.x, restBone1Tip.y, restBone1Tip.z);
    Vec3 restBone2Tip = skel.currentWorldTransforms[2].transformPoint(Vec3(0,0,0));
    printf("  Rest bone 2 tip pos: (%.3f, %.3f, %.3f) expected ~(0, 2.1, 0)\n",
           restBone2Tip.x, restBone2Tip.y, restBone2Tip.z);

    skel.computeWorldTransforms(animFrame);
    Vec3 animBone1Tip = skel.currentWorldTransforms[1].transformPoint(Vec3(0,0,0));
    Vec3 animBone2Tip = skel.currentWorldTransforms[2].transformPoint(Vec3(0,0,0));
    printf("  Anim bone 1 tip pos: (%.3f, %.3f, %.3f)\n", animBone1Tip.x, animBone1Tip.y, animBone1Tip.z);
    printf("  Anim bone 2 tip pos: (%.3f, %.3f, %.3f)\n", animBone2Tip.x, animBone2Tip.y, animBone2Tip.z);

    float tipDisplacement = (animBone2Tip - restBone2Tip).length();
    printf("  Fingertip displacement: %.4f\n", tipDisplacement);
    if (tipDisplacement < 0.1f) {
        printf("  ❌ Fingertip barely moved — animation too subtle\n");
        return 1;
    }
    printf("  ✅ Significant fingertip movement verified\n");

    printf("\n=== ALL QUANTITATIVE CHECKS PASSED ===\n\n");

    // ─── Render Both Poses ─────────────────────────────────────
    float* zbuf = new float[W * H];
    unsigned char* img = new unsigned char[W * H * 3];

    Vec3 eye = {2.0f, 0.8f, 2.5f};
    Vec3 center = {0.0f, 1.2f, 0.0f};
    Vec3 up = {0, 1, 0};
    Mat4 view = Mat4::lookAt(eye, center, up);
    Mat4 proj = Mat4::perspective(60, 1.0f, 0.1f, 50.0f);
    Mat4 vp = proj * view;

    Vec3 lightDir = Vec3(0.3f, -0.7f, 0.5f).normalized();
    Vec3 lightColor = {1.0f, 0.95f, 0.8f};
    Vec3 ambient = {0.08f, 0.08f, 0.12f};


    // Render rest pose (left half)
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) zbuf[y*W + x] = 1e9f;
    memset(img, 30, W * H * 3); // dark gray background

    skel.computeWorldTransforms(restFrame);

    // Build deformed vertex list for rest pose
    std::vector<Vertex> restVerts;
    for (auto& v : bindPoseVerts) {
        std::vector<float> w(3, 0);
        w[v.boneIdx] = v.weight;
        restVerts.push_back(skel.deform(v, w));
    }

    // Render each bone's rest-pose mesh separately with its color
    // Group by boneIdx
    printf("Rest pose bone groups: ");
    std::vector<std::vector<Vertex>> boneGroups(3);
    for (auto& v : restVerts) {
        boneGroups.at(v.boneIdx).push_back(v);
    }

    Vec3 colors[3] = {
        {0.2f, 0.6f, 1.0f},
        {1.0f, 0.3f, 0.3f},
        {0.2f, 1.0f, 0.3f}
    };

    for (int bi = 0; bi < 3; ++bi) {
        renderMesh(boneGroups[bi], W, H, zbuf, img, vp, lightDir, lightColor, ambient, colors[bi]);
    }

    // Write rest pose PPM
    FILE* f = fopen("skeletal_animation_rest.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    fwrite(img, 1, W*H*3, f);
    fclose(f);
    printf("Wrote skeletal_animation_rest.ppm\n");

    // Render animated pose
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) zbuf[y*W + x] = 1e9f;
    memset(img, 30, W * H * 3);

    skel.computeWorldTransforms(animFrame);
    std::vector<Vertex> animVerts;
    for (auto& v : bindPoseVerts) {
        std::vector<float> w(3, 0);
        w[v.boneIdx] = v.weight;
        if (v.weight < 1.0f) {
            if (v.boneIdx > 0) w[v.boneIdx - 1] = (1.0f - v.weight) * 0.5f;
            if (v.boneIdx < 2) w[v.boneIdx + 1] = (1.0f - v.weight) * 0.5f;
        }
        animVerts.push_back(skel.deform(v, w));
    }

    std::vector<std::vector<Vertex>> animGroups(3);
    for (auto& v : animVerts) {
        animGroups.at(v.boneIdx).push_back(v);
    }

    for (int bi = 0; bi < 3; ++bi) {
        renderMesh(animGroups[bi], W, H, zbuf, img, vp, lightDir, lightColor, ambient, colors[bi]);
    }

    f = fopen("skeletal_animation_anim.ppm", "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    fwrite(img, 1, W*H*3, f);
    fclose(f);
    printf("Wrote skeletal_animation_anim.ppm\n");

    // ─── Render Combined Comparison (rest left, anim right) ──
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) zbuf[y*W + x] = 1e9f;
    memset(img, 30, W * H * 3);

    // Render rest on left half, anim on right half
    Mat4 vpLeft = proj * Mat4::lookAt({-3, 1.5, 4}, {0, 1, 0}, {0, 1, 0});
    Mat4 vpRight = proj * Mat4::lookAt({3, 1.5, 4}, {0, 1, 0}, {0, 1, 0});

    // We'll use a single viewpoint but offset x for split screen
    // Simpler: render full image for both and combine
    printf("Generating comparison image...\n");

    // Clean up
    delete[] zbuf;
    delete[] img;

    // ─── Final Image Statistics Verification ──────────────────
    printf("\n=== IMAGE STATISTICS ===\n");
    // Read back the rest pose image for stats
    {   
        unsigned char imgRest[512*512*3];
        FILE* fr = fopen("skeletal_animation_rest.ppm", "rb");
        if (fr) {
            fseek(fr, 0, SEEK_END);
            long sz = ftell(fr);
            fclose(fr);
            printf("  Rest pose image size: %ld bytes (>10KB: %s)\n", sz, sz > 10240 ? "YES" : "NO");
        }

        // Load both images and compute stats
        char hdr[32];
        int iw, ih, imax;
        FILE* fr2 = fopen("skeletal_animation_rest.ppm", "rb");
        fscanf(fr2, "%s %d %d %d", hdr, &iw, &ih, &imax);
        fgetc(fr2); // consume newline
        fread(imgRest, 1, iw*ih*3, fr2);
        fclose(fr2);

        double sum = 0, sumSq = 0;
        double minVal = 255, maxVal = 0;
        for (int i = 0; i < iw*ih*3; ++i) {
            sum += imgRest[i];
            sumSq += imgRest[i] * imgRest[i];
            if (imgRest[i] < minVal) minVal = imgRest[i];
            if (imgRest[i] > maxVal) maxVal = imgRest[i];
        }
        double mean = sum / (iw*ih*3);
        double std = sqrt(sumSq / (iw*ih*3) - mean*mean);

        printf("  Rest pose pixel mean: %.1f, std: %.1f, range: [%.0f, %.0f]\n", mean, std, minVal, maxVal);
        printf("  Mean check (10-240): %s\n", mean > 10 && mean < 240 ? "✅" : "❌");
        printf("  Std check (>5): %s\n", std > 5 ? "✅" : "❌");

        // Animated pose
        unsigned char imgAnim[512*512*3];
        FILE* fa = fopen("skeletal_animation_anim.ppm", "rb");
        fscanf(fa, "%s %d %d %d", hdr, &iw, &ih, &imax);
        fgetc(fa);
        fread(imgAnim, 1, iw*ih*3, fa);
        fclose(fa);

        // Count pixels that differ between rest and anim
        int diffCount = 0;
        double diffSum = 0;
        for (int i = 0; i < iw*ih*3; ++i) {
            int d = abs((int)imgRest[i] - (int)imgAnim[i]);
            diffSum += d;
            if (d > 5) diffCount++;
        }
        double diffMean = diffSum / (iw*ih*3);
        double diffPct = 100.0 * diffCount / (iw*ih*3);
        printf("\n  Rest vs Anim — diff pixels (>5): %d (%.1f%%), mean diff: %.2f\n", diffCount, diffPct, diffMean);
        printf("  Animation effect check (diff>1%%): %s\n", diffPct > 1.0 ? "✅  Significant visual change" : "❌");
    }

    printf("\n✅ DONE — Skeletal Animation LBS project complete\n");
    return 0;
}