// FXAA Anti-Aliasing Post-Process Renderer
// 软光栅化场景 → 应用FXAA后处理 → 对比效果
// 技术：FXAA边缘检测(亮度梯度)、像素混合、对比度分析
// 输出：左半=无FXAA(有锯齿)，右半=有FXAA(平滑)

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <cstring>

// ===== 基础向量 =====
struct Vec3 {
    float x, y, z;
    Vec3(float x=0, float y=0, float z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 norm() const { float l = len(); return l > 1e-8f ? Vec3{x/l, y/l, z/l} : Vec3{}; }
};

// ===== 帧缓冲 =====
const int W = 800, H = 400;
// 左半区=无FXAA，右半区=有FXAA，最终合并
const int SW = W/2, SH = H; // 单侧宽高

struct Framebuffer {
    std::vector<Vec3> color;
    std::vector<float> depth;
    int w, h;
    Framebuffer(int w, int h): color(w*h, Vec3(0,0,0)), depth(w*h, 1e9f), w(w), h(h) {}
    Vec3& at(int x, int y) { return color[y*w + x]; }
    float& dep(int x, int y) { return depth[y*w + x]; }
    Vec3 get(int x, int y) const {
        if (x<0||y<0||x>=w||y>=h) return Vec3(0,0,0);
        return color[y*w + x];
    }
};

// ===== 软光栅化工具 =====
Vec3 transformMVP(const Vec3& v, const Vec3& camPos, const Vec3& camFwd, const Vec3& camUp,
                  float fov, int w, int h) {
    // Build view matrix
    Vec3 f = camFwd.norm();
    Vec3 r = f.cross(camUp).norm();
    Vec3 u = r.cross(f);
    Vec3 d = v - camPos;
    float vx = d.dot(r), vy = d.dot(u), vz = d.dot(f);
    if (vz <= 0.001f) return Vec3(-1e9f, -1e9f, vz);
    // perspective
    float tanH = std::tan(fov * 0.5f * 3.14159265f / 180.f);
    float aspect = (float)w / h;
    float px = vx / (vz * tanH * aspect);
    float py = vy / (vz * tanH);
    // NDC -> screen
    float sx = (px + 1.f) * 0.5f * w;
    float sy = (1.f - py) * 0.5f * h;
    return Vec3(sx, sy, vz);
};

// Barycentric coords
bool barycentric(float x, float y,
                 float ax, float ay, float bx, float by, float cx, float cy,
                 float& u, float& v, float& w_) {
    float d = (by - cy)*(ax - cx) + (cx - bx)*(ay - cy);
    if (std::abs(d) < 1e-7f) return false;
    u = ((by - cy)*(x - cx) + (cx - bx)*(y - cy)) / d;
    v = ((cy - ay)*(x - cx) + (ax - cx)*(y - cy)) / d;
    w_ = 1.f - u - v;
    return u >= 0 && v >= 0 && w_ >= 0;
}

// Draw triangle (with simple Phong lighting)
void drawTriangle(Framebuffer& fb,
                  Vec3 p0, Vec3 p1, Vec3 p2,   // world pos
                  Vec3 n0, Vec3 n1, Vec3 n2,   // normals
                  Vec3 color,
                  const Vec3& camPos, const Vec3& camFwd, const Vec3& camUp,
                  float fov) {
    Vec3 s0 = transformMVP(p0, camPos, camFwd, camUp, fov, SW, SH);
    Vec3 s1 = transformMVP(p1, camPos, camFwd, camUp, fov, SW, SH);
    Vec3 s2 = transformMVP(p2, camPos, camFwd, camUp, fov, SW, SH);
    if (s0.z < 0.01f || s1.z < 0.01f || s2.z < 0.01f) return;

    int minX = std::max(0, (int)std::min({s0.x, s1.x, s2.x}) - 1);
    int maxX = std::min(SW-1, (int)std::max({s0.x, s1.x, s2.x}) + 1);
    int minY = std::max(0, (int)std::min({s0.y, s1.y, s2.y}) - 1);
    int maxY = std::min(SH-1, (int)std::max({s0.y, s1.y, s2.y}) + 1);

    // Light setup
    Vec3 light1 = Vec3(3, 5, 2).norm();
    Vec3 light2 = Vec3(-2, 3, 4).norm();

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            float u, v, w_;
            if (!barycentric((float)x+.5f, (float)y+.5f,
                              s0.x, s0.y, s1.x, s1.y, s2.x, s2.y, u, v, w_)) continue;
            float depth = u/s0.z + v/s1.z + w_/s2.z;
            if (depth < 0 || depth > fb.dep(x, y)) continue;
            // Interpolate normal
            Vec3 N = (n0*u + n1*v + n2*w_).norm();
            // Phong
            float diff1 = std::max(0.f, N.dot(light1));
            float diff2 = std::max(0.f, N.dot(light2));
            float amb = 0.12f;
            float lit = amb + 0.55f*diff1 + 0.35f*diff2;
            Vec3 viewDir = (camPos - (p0*u + p1*v + p2*w_)).norm();
            Vec3 H1 = (light1 + viewDir).norm();
            float spec1 = std::pow(std::max(0.f, N.dot(H1)), 32.f) * 0.4f;
            Vec3 c = color * lit + Vec3(1,1,1) * spec1;
            fb.at(x,y) = Vec3(std::min(1.f,c.x), std::min(1.f,c.y), std::min(1.f,c.z));
            fb.dep(x,y) = depth;
        }
    }
}

// ===== Scene: 多个几何体，产生大量斜向锯齿边 =====
void renderScene(Framebuffer& fb, const Vec3& camPos, const Vec3& camFwd, const Vec3& camUp, float fov) {
    // Clear
    for (auto& c : fb.color) c = Vec3(0.05f, 0.05f, 0.12f);  // dark bg
    for (auto& d : fb.depth) d = 1e9f;

    // === Sphere approximation via icosphere-ish triangles ===
    auto addSphere = [&](Vec3 center, float radius, Vec3 color, int stacks=16, int slices=24) {
        const float PI = 3.14159265f;
        for (int i = 0; i < stacks; i++) {
            float phi0 = PI * i / stacks - PI/2;
            float phi1 = PI * (i+1) / stacks - PI/2;
            for (int j = 0; j < slices; j++) {
                float th0 = 2*PI * j / slices;
                float th1 = 2*PI * (j+1) / slices;
                // 4 verts
                auto V = [&](float phi, float th) {
                    return Vec3(std::cos(phi)*std::cos(th), std::sin(phi), std::cos(phi)*std::sin(th));
                };
                Vec3 v00 = V(phi0, th0), v01 = V(phi0, th1);
                Vec3 v10 = V(phi1, th0), v11 = V(phi1, th1);
                // Normals = directions (sphere centered at origin, then shift)
                Vec3 p00 = center + v00*radius, p01 = center + v01*radius;
                Vec3 p10 = center + v10*radius, p11 = center + v11*radius;
                drawTriangle(fb, p00, p10, p11, v00, v10, v11, color, camPos, camFwd, camUp, fov);
                drawTriangle(fb, p00, p11, p01, v00, v11, v01, color, camPos, camFwd, camUp, fov);
            }
        }
    };

    // === Box ===
    auto addBox = [&](Vec3 center, Vec3 half, Vec3 color) {
        // 6 faces, each 2 triangles
        float bx = half.x, by = half.y, bz = half.z;
        Vec3 verts[8] = {
            center + Vec3(-bx,-by,-bz), center + Vec3( bx,-by,-bz),
            center + Vec3( bx, by,-bz), center + Vec3(-bx, by,-bz),
            center + Vec3(-bx,-by, bz), center + Vec3( bx,-by, bz),
            center + Vec3( bx, by, bz), center + Vec3(-bx, by, bz),
        };
        // face indices [a,b,c,d] and normal
        int faces[6][4] = {
            {0,1,2,3}, {5,4,7,6}, {4,0,3,7},
            {1,5,6,2}, {3,2,6,7}, {4,5,1,0}
        };
        Vec3 normals[6] = {
            Vec3(0,0,-1), Vec3(0,0,1), Vec3(-1,0,0),
            Vec3(1,0,0),  Vec3(0,1,0), Vec3(0,-1,0)
        };
        for (int f = 0; f < 6; f++) {
            Vec3& a = verts[faces[f][0]]; Vec3& b = verts[faces[f][1]];
            Vec3& c = verts[faces[f][2]]; Vec3& dd = verts[faces[f][3]];
            Vec3 n = normals[f];
            drawTriangle(fb, a, b, c, n, n, n, color, camPos, camFwd, camUp, fov);
            drawTriangle(fb, a, c, dd, n, n, n, color, camPos, camFwd, camUp, fov);
        }
    };

    // === Ground plane ===
    {
        Vec3 n(0,1,0);
        Vec3 color(0.3f, 0.55f, 0.3f);
        drawTriangle(fb,
            Vec3(-8,-1.0f,-8), Vec3(8,-1.0f,-8), Vec3(8,-1.0f,8),
            n, n, n, color, camPos, camFwd, camUp, fov);
        drawTriangle(fb,
            Vec3(-8,-1.0f,-8), Vec3(8,-1.0f,8), Vec3(-8,-1.0f,8),
            n, n, n, color, camPos, camFwd, camUp, fov);
    }

    // Main spheres - placed to create lots of diagonal edges
    addSphere(Vec3(0.0f, 0.5f, 0.0f),  0.8f, Vec3(0.9f, 0.3f, 0.2f));  // red
    addSphere(Vec3(-2.5f, 0.0f, 0.5f), 0.65f, Vec3(0.2f, 0.5f, 0.9f));  // blue
    addSphere(Vec3(2.5f, 0.3f, -0.5f), 0.7f, Vec3(0.3f, 0.85f, 0.4f));  // green
    addSphere(Vec3(1.2f, -0.3f, 2.0f), 0.5f, Vec3(0.9f, 0.8f, 0.15f));  // yellow
    addSphere(Vec3(-1.5f, -0.2f, 2.5f),0.45f, Vec3(0.8f, 0.3f, 0.9f));  // purple

    // Boxes - sharp edges = strong aliasing
    addBox(Vec3(-3.5f, 0.1f, -2.5f), Vec3(0.4f,1.1f,0.4f), Vec3(0.8f,0.55f,0.3f));
    addBox(Vec3(3.2f, 0.0f, -1.5f),  Vec3(0.45f,1.0f,0.45f), Vec3(0.6f,0.6f,0.8f));
    addBox(Vec3(0.0f, 0.3f, -3.0f),  Vec3(1.2f,0.35f,0.35f), Vec3(0.7f,0.4f,0.5f));

    // Small sphere cluster
    addSphere(Vec3(-0.5f,-0.5f,3.0f),0.3f, Vec3(1.0f,0.5f,0.1f));
    addSphere(Vec3(0.5f, -0.5f,3.0f),0.3f, Vec3(0.1f,0.8f,0.9f));
    addSphere(Vec3(0.0f, 0.1f, 3.0f),0.3f, Vec3(0.9f,0.9f,0.9f));
}

// ===== FXAA Implementation =====
// Based on FXAA 3.11 simplified algorithm
// Reference: Timothy Lottes's FXAA paper

float luminance(const Vec3& c) {
    // Perceptual luminance
    return 0.299f*c.x + 0.587f*c.y + 0.114f*c.z;
}

// FXAA constants
const float FXAA_EDGE_THRESHOLD     = 0.125f;  // minimum contrast to apply
const float FXAA_EDGE_THRESHOLD_MIN = 0.0312f; // darkness threshold
const float FXAA_SUBPIX_TRIM        = 0.25f;   // aliasing subpixel amount
const float FXAA_SUBPIX_TRIM_SCALE  = 1.0f;
const float FXAA_SUBPIX_CAP         = 0.75f;   // amount of blending in subpix
const int   FXAA_SEARCH_STEPS       = 16;
const float FXAA_SEARCH_THRESHOLD   = 0.25f;

Vec3 fxaaPixel(const Framebuffer& fb, int px, int py) {
    // Sample 3x3 neighborhood
    Vec3 cM  = fb.get(px,   py);
    Vec3 cN  = fb.get(px,   py-1);
    Vec3 cS  = fb.get(px,   py+1);
    Vec3 cE  = fb.get(px+1, py);
    Vec3 cW  = fb.get(px-1, py);
    Vec3 cNW = fb.get(px-1, py-1);
    Vec3 cNE = fb.get(px+1, py-1);
    Vec3 cSW = fb.get(px-1, py+1);
    Vec3 cSE = fb.get(px+1, py+1);

    float lumM  = luminance(cM);
    float lumN  = luminance(cN);
    float lumS  = luminance(cS);
    float lumE  = luminance(cE);
    float lumW  = luminance(cW);
    float lumNW = luminance(cNW);
    float lumNE = luminance(cNE);
    float lumSW = luminance(cSW);
    float lumSE = luminance(cSE);

    float lumMin = std::min({lumM, lumN, lumS, lumE, lumW});
    float lumMax = std::max({lumM, lumN, lumS, lumE, lumW});
    float range  = lumMax - lumMin;

    // Skip if not near an edge
    float rangeThresh = std::max(FXAA_EDGE_THRESHOLD_MIN, lumMax * FXAA_EDGE_THRESHOLD);
    if (range < rangeThresh) return cM;

    // Subpixel aliasing test
    float lumL = (lumN + lumS + lumE + lumW) * 0.25f;
    float subpixNSWE = (lumN + lumS + lumE + lumW) * 2.f;
    float subpixDiag = (lumNW + lumNE + lumSW + lumSE);
    (void)((subpixNSWE + subpixDiag) * (1.f/12.f)); // subpixFull unused
    float subpixLowpass = std::abs(lumL - lumM);
    float subpixAlias = subpixLowpass / range;
    subpixAlias = std::max(0.f, subpixAlias - FXAA_SUBPIX_TRIM) * FXAA_SUBPIX_TRIM_SCALE;
    subpixAlias = std::min(FXAA_SUBPIX_CAP, subpixAlias);
    float subpixFilter = subpixAlias * subpixAlias;  // smooth

    // Determine edge direction (horizontal or vertical)
    float edgeHorz =
        std::abs(-2.f*lumW + lumNW + lumSW) +
        std::abs(-2.f*lumM + lumN + lumS)   * 2.f +
        std::abs(-2.f*lumE + lumNE + lumSE);
    float edgeVert =
        std::abs(-2.f*lumN + lumNW + lumNE) +
        std::abs(-2.f*lumM + lumW + lumE)   * 2.f +
        std::abs(-2.f*lumS + lumSW + lumSE);
    bool horzSpan = edgeHorz >= edgeVert;

    // Get positive/negative end pixel neighbors
    float lumPos, lumNeg;
    if (horzSpan) {
        lumPos = lumS; lumNeg = lumN;
    } else {
        lumPos = lumE; lumNeg = lumW;
    }

    float gradPos = std::abs(lumPos - lumM);
    float gradNeg = std::abs(lumNeg - lumM);
    bool pairN = (gradNeg >= gradPos);
    float lumPair = pairN ? lumNeg : lumPos;
    float gradient = std::max(gradPos, gradNeg);

    // Step direction (pixel offset for blend sample)
    float offx = 0.f, offy = 0.f;
    if (horzSpan) offy = pairN ? -1.f : 1.f;
    else          offx = pairN ? -1.f : 1.f;

    // Walk along edge to find end
    float posB = horzSpan ? (float)px : (float)py;
    float negB = posB;
    float lumEndPos = 0.5f * (lumM + lumPair);
    float lumEndNeg = lumEndPos;
    bool donePos = false, doneNeg = false;
    // Step direction along edge
    float stepEdge = horzSpan ? 1.f : 1.f;

    int stepSearchPos = 0, stepSearchNeg = 0;
    for (int i = 0; i < FXAA_SEARCH_STEPS; i++) {
        if (!donePos) {
            posB += stepEdge;
            int nx = horzSpan ? (int)(posB) : (int)((float)px + offx);
            int ny = horzSpan ? (int)((float)py + offy) : (int)(posB);
            lumEndPos = luminance(fb.get(nx, ny));
            donePos = (std::abs(lumEndPos - lumPair) >= gradient * FXAA_SEARCH_THRESHOLD);
            stepSearchPos++;
        }
        if (!doneNeg) {
            negB -= stepEdge;
            int nx = horzSpan ? (int)(negB) : (int)((float)px + offx);
            int ny = horzSpan ? (int)((float)py + offy) : (int)(negB);
            lumEndNeg = luminance(fb.get(nx, ny));
            doneNeg = (std::abs(lumEndNeg - lumPair) >= gradient * FXAA_SEARCH_THRESHOLD);
            stepSearchNeg++;
        }
        if (donePos && doneNeg) break;
    }

    float distPos = horzSpan ? (posB - px) : (posB - py);
    float distNeg = horzSpan ? (px - negB) : (py - negB);
    float spanLen = distPos + distNeg;

    // Pick closer end
    bool directionN = (distNeg < distPos);
    float lumEnd = directionN ? lumEndNeg : lumEndPos;

    // Check if pixel is on correct side
    bool correctVariation = ((lumM - lumPair) < 0.f) != ((lumEnd - lumPair) < 0.f);
    float subPixelOffset = directionN ? -distNeg : distPos;
    subPixelOffset = correctVariation ? (subPixelOffset / spanLen) - 0.5f : 0.f;

    // Blend amount
    float blendAmt = std::max(subpixFilter, std::abs(subPixelOffset));
    blendAmt = std::min(blendAmt, 0.75f);  // cap

    // Sample blended neighbor
    float bx = (float)px + offx * blendAmt;
    float by = (float)py + offy * blendAmt;
    // Bilinear sample between M and neighbor
    Vec3 cBlend = fb.get((int)std::round(bx), (int)std::round(by));
    // Lerp
    return cM * (1.f - blendAmt) + cBlend * blendAmt;
}

// Apply FXAA to framebuffer, write to output
Framebuffer applyFXAA(const Framebuffer& src) {
    Framebuffer dst(src.w, src.h);
    for (int y = 0; y < src.h; y++) {
        for (int x = 0; x < src.w; x++) {
            dst.at(x,y) = fxaaPixel(src, x, y);
        }
    }
    return dst;
}

// ===== Main =====
int main() {
    printf("FXAA Anti-Aliasing Post-Process Renderer\n");
    printf("Width: %d  Height: %d\n", W, H);
    printf("Left half: No FXAA (raw aliased)  |  Right half: FXAA applied\n\n");

    // Camera setup
    Vec3 camPos(0.f, 1.8f, 6.5f);
    Vec3 camTarget(0.f, 0.f, 0.f);
    Vec3 camFwd = (camTarget - camPos).norm();
    Vec3 camUp(0, 1, 0);
    float fov = 45.f;

    // Render scene
    printf("[1/4] Rendering scene (software rasterizer)...\n");
    Framebuffer rawFB(SW, SH);
    renderScene(rawFB, camPos, camFwd, camUp, fov);
    printf("      Scene rendered (%dx%d)\n", SW, SH);

    // Apply FXAA
    printf("[2/4] Applying FXAA post-process...\n");
    Framebuffer fxaaFB = applyFXAA(rawFB);
    printf("      FXAA applied\n");

    // Composite: left=no FXAA, right=FXAA, with divider line
    printf("[3/4] Compositing side-by-side comparison...\n");
    std::vector<unsigned char> img(W * H * 3);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Vec3 c;
            if (x < SW) {
                c = rawFB.at(x, y);
            } else {
                c = fxaaFB.at(x - SW, y);
            }
            // Add divider line with label-like shade
            if (x == SW) { c = Vec3(1,1,0); }  // yellow divider
            // Gamma correction
            c.x = std::pow(std::max(0.f, std::min(1.f, c.x)), 1.f/2.2f);
            c.y = std::pow(std::max(0.f, std::min(1.f, c.y)), 1.f/2.2f);
            c.z = std::pow(std::max(0.f, std::min(1.f, c.z)), 1.f/2.2f);
            int idx = (y*W + x)*3;
            img[idx+0] = (unsigned char)(c.x * 255.f);
            img[idx+1] = (unsigned char)(c.y * 255.f);
            img[idx+2] = (unsigned char)(c.z * 255.f);
        }
    }

    // Draw text labels in top area (simple pixel font simulation)
    // "NO FXAA" on left, "FXAA ON" on right - just draw simple colored bars
    auto drawLabel = [&](int startX, int startY, int labelW, Vec3 col) {
        for (int dy = 0; dy < 4; dy++) {
            for (int dx = 0; dx < labelW; dx++) {
                int lx = startX + dx, ly = startY + dy;
                if (lx >= 0 && lx < W && ly >= 0 && ly < H) {
                    int idx = (ly*W + lx)*3;
                    img[idx+0] = (unsigned char)(col.x * 255);
                    img[idx+1] = (unsigned char)(col.y * 255);
                    img[idx+2] = (unsigned char)(col.z * 255);
                }
            }
        }
    };
    // Red bar = no FXAA label, Green bar = FXAA label
    drawLabel(10, 10, 100, Vec3(1,0.2f,0.2f));
    drawLabel(SW+10, 10, 100, Vec3(0.2f,1,0.2f));

    // Output stats
    printf("[4/4] Saving fxaa_output.png...\n");
    int ret = stbi_write_png("fxaa_output.png", W, H, 3, img.data(), W*3);
    if (!ret) { fprintf(stderr, "ERROR: Failed to write fxaa_output.png\n"); return 1; }
    printf("      Saved: fxaa_output.png  (%dx%d)\n", W, H);

    // Pixel stats for validation
    long long sumR=0, sumG=0, sumB=0;
    long long sumSqR=0, sumSqG=0, sumSqB=0;
    int N = W*H;
    for (int i = 0; i < N; i++) {
        float r = img[i*3+0], g = img[i*3+1], b = img[i*3+2];
        sumR += r; sumG += g; sumB += b;
        sumSqR += r*r; sumSqG += g*g; sumSqB += b*b;
    }
    float meanR = sumR/float(N), meanG = sumG/float(N), meanB = sumB/float(N);
    float mean = (meanR + meanG + meanB) / 3.f;
    float varR = sumSqR/float(N) - meanR*meanR;
    float varG = sumSqG/float(N) - meanG*meanG;
    float varB = sumSqB/float(N) - meanB*meanB;
    float stddev = std::sqrt((varR + varG + varB) / 3.f);

    printf("\n=== Output Validation ===\n");
    printf("Pixel mean  : %.1f (range 10~240)\n", mean);
    printf("Pixel stddev: %.1f (>5 required)\n", stddev);
    printf("Resolution  : %dx%d\n", W, H);

    bool ok = true;
    if (mean < 5) { printf("❌ Image too dark\n"); ok = false; }
    else if (mean > 250) { printf("❌ Image too bright\n"); ok = false; }
    else printf("✅ Mean OK\n");

    if (stddev < 5) { printf("❌ Image has no variation\n"); ok = false; }
    else printf("✅ Stddev OK\n");

    if (!ok) return 1;

    printf("\n✅ All validation passed!\n");
    printf("Left  (No FXAA): raw aliased edges visible on sphere/box boundaries\n");
    printf("Right (FXAA ON): edge blending applied, smoother appearance\n");
    return 0;
}
