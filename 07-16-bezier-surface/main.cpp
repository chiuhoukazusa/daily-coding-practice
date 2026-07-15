/**
 * Bezier Surface Renderer — Bicubic Bezier Patch
 * 
 * Features:
 *  - 4x4 control points → bicubic Bernstein polynomial evaluation
 *  - 50x50 adaptive tessellation
 *  - Phong lighting with ambient + diffuse + specular
 *  - Z-Buffer depth test
 *  - Background: gradient sky
 *  - Output: PPM image
 *
 * Quantitative validation:
 *  - Pixel mean/std check
 *  - Curvature check: normal vectors across surface must vary meaningfully
 *  - Corner position verification
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>

constexpr int WIDTH  = 1024;
constexpr int HEIGHT = 768;
constexpr int TESS   = 50; // tessellation resolution

// 3D vector
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const {
        float l = len();
        if (l < 1e-10f) return {0,0,0};
        return {x/l, y/l, z/l};
    }
};

// Color struct
struct Color {
    unsigned char r, g, b;
    Color() : r(0),g(0),b(0) {}
    Color(unsigned char _r, unsigned char _g, unsigned char _b) : r(_r),g(_g),b(_b) {}
    static Color fromFloat(float _r, float _g, float _b) {
        Color c;
        c.r = (unsigned char)std::min(255.0f, std::max(0.0f, _r * 255.0f));
        c.g = (unsigned char)std::min(255.0f, std::max(0.0f, _g * 255.0f));
        c.b = (unsigned char)std::min(255.0f, std::max(0.0f, _b * 255.0f));
        return c;
    }
};

// Bernstein polynomial
float bernstein(int i, int n, float t) {
    float coeff = 1.0f;
    for (int j = 1; j <= i; ++j) coeff = coeff * (n - j + 1) / j;
    return coeff * std::pow(t, i) * std::pow(1.0f - t, n - i);
}

// Derivative of Bernstein polynomial
float dbernstein(int i, float t) {
    if (i == 0) return -3.0f * std::pow(1.0f - t, 2);
    if (i == 1) return 3.0f * (1.0f - t) * (1.0f - 3.0f * t);
    if (i == 2) return 3.0f * t * (2.0f - 3.0f * t);
    return 3.0f * t * t; // i == 3
}

// Evaluate bicubic Bezier surface at parameter (u,v) in [0,1]^2
Vec3 evalBezier(const Vec3 ctrl[4][4], float u, float v) {
    Vec3 result;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result = result + ctrl[i][j] * (bernstein(i, 3, u) * bernstein(j, 3, v));
    return result;
}

// Partial derivative w.r.t u
Vec3 evaldU(const Vec3 ctrl[4][4], float u, float v) {
    Vec3 result;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result = result + ctrl[i][j] * (dbernstein(i, u) * bernstein(j, 3, v));
    return result;
}

// Partial derivative w.r.t v
Vec3 evaldV(const Vec3 ctrl[4][4], float u, float v) {
    Vec3 result;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            result = result + ctrl[i][j] * (bernstein(i, 3, u) * dbernstein(j, v));
    return result;
}

// Normal at surface point
Vec3 evalNormal(const Vec3 ctrl[4][4], float u, float v) {
    return evaldU(ctrl, u, v).cross(evaldV(ctrl, u, v)).normalize();
}

// Clamp
float clamp(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

int main() {
    // Control points for a saddle-like bezier surface
    // 4x4 grid arranged in a nice curved shape with a depression in the middle
    Vec3 ctrl[4][4] = {
        { {-3.0f, -3.0f,  2.0f}, {-1.0f, -3.0f,  2.0f}, {1.0f, -3.0f,  2.0f}, {3.0f, -3.0f,  2.0f} },
        { {-3.0f, -1.0f,  2.0f}, {-1.0f, -1.0f, -2.5f}, {1.0f, -1.0f, -2.5f}, {3.0f, -1.0f,  2.0f} },
        { {-3.0f,  1.0f,  2.0f}, {-1.0f,  1.0f, -2.5f}, {1.0f,  1.0f, -2.5f}, {3.0f,  1.0f,  2.0f} },
        { {-3.0f,  3.0f,  2.0f}, {-1.0f,  3.0f,  2.0f}, {1.0f,  3.0f,  2.0f}, {3.0f,  3.0f,  2.0f} }
    };

    // Z-buffer
    std::vector<float> zbuffer(WIDTH * HEIGHT, 1e10f);
    std::vector<Color> framebuffer(WIDTH * HEIGHT, Color{30, 50, 80});

    // Light direction
    Vec3 lightDir = Vec3(0.5f, 0.7f, -1.0f).normalize();
    
    // Material properties
    Vec3 matDiffuse(0.3f, 0.6f, 0.9f);
    Vec3 matSpecular(1.0f, 1.0f, 1.0f);
    float shininess = 32.0f;
    Vec3 ambient(0.15f, 0.15f, 0.15f);

    // Projection parameters
    float fov = 60.0f * M_PI / 180.0f;
    float scale = HEIGHT * 0.5f / std::tan(fov * 0.5f);
    float cx = WIDTH * 0.5f;
    float cy = HEIGHT * 0.5f;
    
    // Camera position: looking at origin from +z direction
    float camZ = 10.0f;

    // Tessellate the surface
    for (int iu = 0; iu < TESS; ++iu) {
        for (int iv = 0; iv < TESS; ++iv) {
            float u0 = (float)iu / TESS;
            float v0 = (float)iv / TESS;
            float u1 = (float)(iu+1) / TESS;
            float v1 = (float)(iv+1) / TESS;
            
            Vec3 p00 = evalBezier(ctrl, u0, v0);
            Vec3 p10 = evalBezier(ctrl, u1, v0);
            Vec3 p01 = evalBezier(ctrl, u0, v1);
            Vec3 p11 = evalBezier(ctrl, u1, v1);
            
            Vec3 n00 = evalNormal(ctrl, u0, v0);
            Vec3 n10 = evalNormal(ctrl, u1, v0);
            Vec3 n01 = evalNormal(ctrl, u0, v1);
            Vec3 n11 = evalNormal(ctrl, u1, v1);

            // Project to screen
            auto proj = [&](const Vec3& p) -> std::pair<int,int> {
                float px = p.x / (camZ - p.z) * scale + cx;
                float py = -p.y / (camZ - p.z) * scale + cy;
                return {(int)(px + 0.5f), (int)(py + 0.5f)};
            };
            
            auto [sx0, sy0] = proj(p00);
            auto [sx1, sy1] = proj(p10);
            auto [sx2, sy2] = proj(p11);
            auto [sx3, sy3] = proj(p01);

            int minX = std::max(0, std::min({sx0, sx1, sx2, sx3}));
            int maxX = std::min(WIDTH-1, std::max({sx0, sx1, sx2, sx3}));
            int minY = std::max(0, std::min({sy0, sy1, sy2, sy3}));
            int maxY = std::min(HEIGHT-1, std::max({sy0, sy1, sy2, sy3}));

            if (maxX <= minX || maxY <= minY) continue;

            float dx = (float)(maxX - minX);
            float dy = (float)(maxY - minY);
            if (dx < 1) dx = 1;
            if (dy < 1) dy = 1;

            // View direction for specular
            Vec3 viewDir(0, 0, 1);

            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    int idx = y * WIDTH + x;
                    
                    // Interpolate normal bilinearly in screen space
                    float ax = (float)(x - minX) / dx;
                    float ay = (float)(y - minY) / dy;
                    Vec3 n = (n00 * ((1-ax)*(1-ay)) + n10 * (ax*(1-ay)) 
                           + n01 * ((1-ax)*ay) + n11 * (ax*ay)).normalize();
                    
                    // Interpolate position for depth
                    Vec3 pos = p00 * ((1-ax)*(1-ay)) + p10 * (ax*(1-ay)) 
                             + p01 * ((1-ax)*ay) + p11 * (ax*ay);
                    
                    // Depth from camera
                    float depth = camZ - pos.z;
                    if (depth >= zbuffer[idx]) continue;
                    zbuffer[idx] = depth;

                    // Phong lighting
                    float NdotL = clamp(n.dot(lightDir), 0.0f, 1.0f);
                    Vec3 half = (lightDir + viewDir).normalize();
                    float spec = std::pow(clamp(n.dot(half), 0.0f, 1.0f), shininess);
                    
                    Vec3 col = ambient * 0.3f + matDiffuse * NdotL + matSpecular * (spec * 0.6f);
                    Color c;
                    c.r = (unsigned char)std::min(255.0f, std::max(0.0f, col.x * 255.0f));
                    c.g = (unsigned char)std::min(255.0f, std::max(0.0f, col.y * 255.0f));
                    c.b = (unsigned char)std::min(255.0f, std::max(0.0f, col.z * 255.0f));
                    framebuffer[idx] = c;
                }
            }
        }
    }

    // Gradient background
    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            int idx = y * WIDTH + x;
            if (zbuffer[idx] > 1e9f) {
                float t = (float)y / HEIGHT;
                Color c;
                c.r = (unsigned char)(30 + 40 * (1.0f - t));
                c.g = (unsigned char)(40 + 50 * (1.0f - t));
                c.b = (unsigned char)(60 + 60 * (1.0f - t));
                framebuffer[idx] = c;
            }
        }
    }

    // Write PPM
    FILE* f = fopen("bezier_surface.ppm", "wb");
    if (!f) { fprintf(stderr, "Cannot open output file\n"); return 1; }
    fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    fwrite(framebuffer.data(), 1, WIDTH * HEIGHT * 3, f);
    fclose(f);

    printf("Bezier surface rendered: %dx%d PPM\n", WIDTH, HEIGHT);

    // ====== QUANTITATIVE VALIDATION ======
    
    // 1. Corner position check: control point (0,0) and (3,3) must be at surface corners
    Vec3 c00 = evalBezier(ctrl, 0.0f, 0.0f);
    Vec3 c33 = evalBezier(ctrl, 1.0f, 1.0f);
    Vec3 c03 = evalBezier(ctrl, 0.0f, 1.0f);
    Vec3 c30 = evalBezier(ctrl, 1.0f, 0.0f);
    printf("\n=== Corner Position Verification ===\n");
    printf("P(0,0) = (%.3f, %.3f, %.3f)  expected ~(-3, -3, 2)\n", c00.x, c00.y, c00.z);
    printf("P(1,1) = (%.3f, %.3f, %.3f)  expected ~( 3,  3, 2)\n", c33.x, c33.y, c33.z);
    printf("P(0,1) = (%.3f, %.3f, %.3f)  expected ~(-3,  3, 2)\n", c03.x, c03.y, c03.z);
    printf("P(1,0) = (%.3f, %.3f, %.3f)  expected ~( 3, -3, 2)\n", c30.x, c30.y, c30.z);

    // 2. Endpoint interpolation: Bezier surface interpolates corner control points
    float cornerError = ((c00 - ctrl[0][0]).len() + (c33 - ctrl[3][3]).len() 
                       + (c03 - ctrl[0][3]).len() + (c30 - ctrl[3][0]).len()) / 4.0f;
    printf("\n=== Endpoint Interpolation Error ===\n");
    printf("Mean corner error: %.6f (should be < 0.001)\n", cornerError);

    // 3. Normal variation: sample normals across the surface
    printf("\n=== Normal Variation Check ===\n");
    std::vector<float> normalAngles;
    Vec3 n00_norm = evalNormal(ctrl, 0.0f, 0.0f);
    
    for (int i = 0; i <= 10; ++i) {
        for (int j = 0; j <= 10; ++j) {
            float u = i / 10.0f, v = j / 10.0f;
            Vec3 n = evalNormal(ctrl, u, v);
            float angle = std::acos(clamp(n00_norm.dot(n), -1.0f, 1.0f)) * 180.0f / M_PI;
            normalAngles.push_back(angle);
        }
    }
    float navg = 0, nstd = 0;
    for (float a : normalAngles) navg += a;
    navg /= normalAngles.size();
    for (float a : normalAngles) nstd += (a - navg) * (a - navg);
    nstd = std::sqrt(nstd / normalAngles.size());
    printf("Normal angle mean vs corner: %.2f deg\n", navg);
    printf("Normal angle std: %.2f deg\n", nstd);
    printf("Max normal deviation from corner: %.2f deg\n", 
           *std::max_element(normalAngles.begin(), normalAngles.end()));

    // 4. Curvature check: center point Z should differ from corners (saddle shape)
    Vec3 center = evalBezier(ctrl, 0.5f, 0.5f);
    float cornerAvgZ = (c00.z + c33.z + c03.z + c30.z) / 4.0f;
    printf("\n=== Curvature / Shape Check ===\n");
    printf("Center point: (%.3f, %.3f, %.3f)\n", center.x, center.y, center.z);
    printf("Corner avg Z: %.3f\n", cornerAvgZ);
    printf("Center Z vs corner avg Z: %.3f vs %.3f\n", center.z, cornerAvgZ);
    float zDiff = std::abs(center.z - cornerAvgZ);
    printf("Z difference (center vs corners): %.3f\n", zDiff);

    // 5. Surface area approximation
    float area = 0;
    for (int iu = 0; iu < TESS; ++iu) {
        for (int iv = 0; iv < TESS; ++iv) {
            float u = (iu + 0.5f) / TESS, v = (iv + 0.5f) / TESS;
            Vec3 du = evaldU(ctrl, u, v);
            Vec3 dv = evaldV(ctrl, u, v);
            area += du.cross(dv).len() * (1.0f / (TESS * TESS));
        }
    }
    printf("\n=== Surface Area ===\n");
    printf("Approx surface area: %.3f sq units\n", area);

    // 6. Pixel statistics from framebuffer
    printf("\n=== Pixel Statistics ===\n");
    double rsum = 0, gsum = 0, bsum = 0;
    double rvar = 0, gvar = 0, bvar = 0;
    int count = WIDTH * HEIGHT;
    for (int i = 0; i < count; ++i) {
        rsum += framebuffer[i].r;
        gsum += framebuffer[i].g;
        bsum += framebuffer[i].b;
    }
    double rmean = rsum / count;
    double gmean = gsum / count;
    double bmean = bsum / count;
    for (int i = 0; i < count; ++i) {
        rvar += (framebuffer[i].r - rmean) * (framebuffer[i].r - rmean);
        gvar += (framebuffer[i].g - gmean) * (framebuffer[i].g - gmean);
        bvar += (framebuffer[i].b - bmean) * (framebuffer[i].b - bmean);
    }
    double rstd = std::sqrt(rvar / count);
    double gstd = std::sqrt(gvar / count);
    double bstd = std::sqrt(bvar / count);
    double meanRGB = (rmean + gmean + bmean) / 3.0;
    double stdRGB = (rstd + gstd + bstd) / 3.0;
    printf("RGB mean: (%.1f, %.1f, %.1f)  avg: %.1f\n", rmean, gmean, bmean, meanRGB);
    printf("RGB std:  (%.1f, %.1f, %.1f)  avg: %.1f\n", rstd, gstd, bstd, stdRGB);

    // File size check
    FILE* f2 = fopen("bezier_surface.ppm", "rb");
    if (f2) {
        fseek(f2, 0, SEEK_END);
        long fsize = ftell(f2);
        fclose(f2);
        printf("\n=== File Size ===\n");
        printf("Output file size: %ld bytes (%.1f KB)\n", fsize, fsize / 1024.0);
    }

    // Validation assertions
    printf("\n=== VALIDATION RESULTS ===\n");
    bool pass = true;
    if (cornerError > 0.01f) { printf("❌ FAIL: Corner interpolation error too large (%.6f)\n", cornerError); pass = false; }
    else printf("✅ PASS: Corner interpolation correct (error=%.6f)\n", cornerError);
    
    if (zDiff < 0.5f) { printf("❌ FAIL: No meaningful curvature (zDiff=%.3f)\n", zDiff); pass = false; }
    else printf("✅ PASS: Surface has curvature (zDiff=%.3f)\n", zDiff);

    if (nstd < 1.0f) { printf("❌ FAIL: Normals too uniform (std=%.2f deg)\n", nstd); pass = false; }
    else printf("✅ PASS: Surface has normal variation (std=%.2f deg)\n", nstd);
    
    if (meanRGB < 10 || meanRGB > 240) { printf("❌ FAIL: Image too dark/bright (mean=%.1f)\n", meanRGB); pass = false; }
    else printf("✅ PASS: Pixel brightness in range (mean=%.1f)\n", meanRGB);

    if (stdRGB < 5) { printf("❌ FAIL: Image too uniform (std=%.1f)\n", stdRGB); pass = false; }
    else printf("✅ PASS: Image has content variation (std=%.1f)\n", stdRGB);

    printf("\n=== OVERALL RESULT ===\n");
    if (pass) printf("✅ ALL QUANTITATIVE CHECKS PASSED\n");
    else printf("❌ SOME CHECKS FAILED\n");

    return pass ? 0 : 1;
}
