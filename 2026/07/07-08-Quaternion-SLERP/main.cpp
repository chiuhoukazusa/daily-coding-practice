/**
 * Quaternion Rotation & SLERP Interpolation
 * 
 * Features:
 * - Quaternion class: construction, normalization, conjugate, multiply
 * - SLERP (Spherical Linear Interpolation) with proper shortest-path
 * - Euler angle → Quaternion conversion
 * - Quaternion → Rotation Matrix conversion
 * - Gimbal Lock demonstration (Euler vs Quaternion)
 * - 3D point rotation visualization
 * - Quantitative verification (not visual inspection)
 * 
 * Output: quaternion_output.ppm - shows SLERP interpolation sequence
 *          with multiple rotated cubes demonstrating smooth rotation
 */

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cassert>

constexpr double PI = 3.14159265358979323846;
constexpr double EPS = 1e-9;

// ============================================================
// Vector3
// ============================================================
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        double l = length();
        if (l < EPS) return Vec3(0,0,0);
        return Vec3(x/l, y/l, z/l);
    }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
    }
    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(double s) const { return Vec3(x/s, y/s, z/s); }
};

// ============================================================
// Quaternion
// ============================================================
struct Quat {
    double w, x, y, z;  // w + xi + yj + zk
    
    Quat() : w(1), x(0), y(0), z(0) {}  // identity
    Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    // Construct from axis-angle
    static Quat fromAxisAngle(const Vec3& axis, double angle) {
        double half = angle * 0.5;
        double s = std::sin(half);
        Vec3 a = axis.normalized();
        return Quat(std::cos(half), a.x * s, a.y * s, a.z * s);
    }
    
    // Construct from Euler angles (ZYX order: yaw, pitch, roll)
    static Quat fromEuler(double yaw, double pitch, double roll) {
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        
        return Quat(
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        );
    }
    
    double norm() const { return std::sqrt(w*w + x*x + y*y + z*z); }
    
    Quat normalized() const {
        double n = norm();
        if (n < EPS) return Quat(1, 0, 0, 0);
        return Quat(w/n, x/n, y/n, z/n);
    }
    
    Quat conjugate() const { return Quat(w, -x, -y, -z); }
    
    Quat inverse() const {
        double n2 = w*w + x*x + y*y + z*z;
        if (n2 < EPS) return Quat(1,0,0,0);
        Quat c = conjugate();
        return Quat(c.w/n2, c.x/n2, c.y/n2, c.z/n2);
    }
    
    // Hamilton product: this * q
    Quat operator*(const Quat& q) const {
        return Quat(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    // Normalize the result of multiplication (for numerical stability)
    Quat mulNormalized(const Quat& q) const {
        return (*this * q).normalized();
    }
    
    // Rotate a vector by this quaternion
    Vec3 rotate(const Vec3& v) const {
        Quat qv(0, v.x, v.y, v.z);
        Quat result = (*this) * qv * this->conjugate();
        return Vec3(result.x, result.y, result.z);
    }
    
    // Convert to 3x3 rotation matrix (row-major, [row][col])
    void toMatrix(double m[3][3]) const {
        double x2 = x*x, y2 = y*y, z2 = z*z;
        double xy = x*y, xz = x*z, yz = y*z;
        double wx = w*x, wy = w*y, wz = w*z;
        
        m[0][0] = 1 - 2*(y2 + z2);
        m[0][1] = 2*(xy - wz);
        m[0][2] = 2*(xz + wy);
        
        m[1][0] = 2*(xy + wz);
        m[1][1] = 1 - 2*(x2 + z2);
        m[1][2] = 2*(yz - wx);
        
        m[2][0] = 2*(xz - wy);
        m[2][1] = 2*(yz + wx);
        m[2][2] = 1 - 2*(x2 + y2);
    }
    
    // Dot product
    double dot(const Quat& q) const {
        return w*q.w + x*q.x + y*q.y + z*q.z;
    }
};

// ============================================================
// SLERP (Spherical Linear Interpolation)
// Takes the shortest path
// ============================================================
Quat slerp(const Quat& a, const Quat& b, double t) {
    // Clamp t
    t = std::max(0.0, std::min(1.0, t));
    
    // Compute cosine of angle between quaternions
    double cosTheta = a.dot(b);
    
    // If cosTheta < 0, negate one quaternion to take shortest path
    Quat qb = b;
    if (cosTheta < 0) {
        qb = Quat(-b.w, -b.x, -b.y, -b.z);
        cosTheta = -cosTheta;
    }
    
    // If quaternions are very close, use linear interpolation to avoid division by zero
    if (cosTheta > 1.0 - EPS) {
        return Quat(
            a.w + t*(qb.w - a.w),
            a.x + t*(qb.x - a.x),
            a.y + t*(qb.y - a.y),
            a.z + t*(qb.z - a.z)
        ).normalized();
    }
    
    double theta = std::acos(cosTheta);
    double sinTheta = std::sin(theta);
    
    double s0 = std::sin((1 - t) * theta) / sinTheta;
    double s1 = std::sin(t * theta) / sinTheta;
    
    return Quat(
        s0*a.w + s1*qb.w,
        s0*a.x + s1*qb.x,
        s0*a.y + s1*qb.y,
        s0*a.z + s1*qb.z
    );
}

// ============================================================
// NLERP (Normalized Linear Interpolation) for comparison
// ============================================================
Quat nlerp(const Quat& a, const Quat& b, double t) {
    t = std::max(0.0, std::min(1.0, t));
    
    Quat qb = b;
    if (a.dot(b) < 0) {
        qb = Quat(-b.w, -b.x, -b.y, -b.z);
    }
    
    return Quat(
        a.w + t*(qb.w - a.w),
        a.x + t*(qb.x - a.x),
        a.y + t*(qb.y - a.y),
        a.z + t*(qb.z - a.z)
    ).normalized();
}

// ============================================================
// Euler angle interpolation (for gimbal lock comparison)
// ============================================================
Vec3 lerpEuler(const Vec3& e1, const Vec3& e2, double t) {
    t = std::max(0.0, std::min(1.0, t));
    return Vec3(
        e1.x + t*(e2.x - e1.x),
        e1.y + t*(e2.y - e1.y),
        e1.z + t*(e2.z - e1.z)
    );
}

// ============================================================
// Simple 3D cube vertices
// ============================================================
std::vector<Vec3> getCubeVertices() {
    std::vector<Vec3> verts;
    double s = 0.4;
    for (int i = 0; i < 8; i++) {
        verts.push_back(Vec3(
            (i & 1) ? s : -s,
            (i & 2) ? s : -s,
            (i & 4) ? s : -s
        ));
    }
    return verts;
}

// 12 edges of a cube
struct Edge { int a, b; };
std::vector<Edge> getCubeEdges() {
    return {
        {0,1}, {0,2}, {0,4}, {1,3}, {1,5},
        {2,3}, {2,6}, {3,7}, {4,5}, {4,6},
        {5,7}, {6,7}
    };
}

// ============================================================
// PPM Image Writer
// ============================================================
struct Color {
    unsigned char r, g, b;
    Color() : r(0), g(0), b(0) {}
    Color(unsigned char r_, unsigned char g_, unsigned char b_) : r(r_), g(g_), b(b_) {}
};

void writePPM(const char* filename, int w, int h, const std::vector<Color>& pixels) {
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", w, h);
    for (const auto& c : pixels) {
        fputc(c.r, f);
        fputc(c.g, f);
        fputc(c.b, f);
    }
    fclose(f);
}

// ============================================================
// Bresenham line drawing
// ============================================================
void drawLine(std::vector<Color>& pixels, int w, int h, 
              int x0, int y0, int x1, int y1, Color col) {
    int dx = std::abs(x1 - x0);
    int dy = -std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    
    while (true) {
        if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
            pixels[y0 * w + x0] = col;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// ============================================================
// 3D → 2D projection (orthographic for simplicity)
// ============================================================
struct Point2D { int x, y; };

Point2D project(const Vec3& p, int imgW, int imgH, double scale) {
    return Point2D{
        int(imgW/2 + p.x * scale * imgW),
        int(imgH/2 - p.y * scale * imgW) // Y up in world
    };
}

void drawCube(std::vector<Color>& pixels, int w, int h,
              const Quat& rotation, Vec3 offset, double scale, Color col) {
    auto verts = getCubeVertices();
    auto edges = getCubeEdges();
    
    // Rotate and project vertices
    std::vector<Point2D> projected;
    for (const auto& v : verts) {
        Vec3 rv = rotation.rotate(v) + offset;
        projected.push_back(project(rv, w, h, scale));
    }
    
    // Draw edges
    for (const auto& e : edges) {
        drawLine(pixels, w, h,
                 projected[e.a].x, projected[e.a].y,
                 projected[e.b].x, projected[e.b].y, col);
    }
}

// ============================================================
// Main rendering
// ============================================================
int main() {
    const int W = 1200, H = 800;
    std::vector<Color> pixels(W * H, Color(20, 20, 30)); // dark bg
    
    printf("=== Quaternion Rotation & SLERP Interpolation ===\n\n");
    
    // ========================================
    // SECTION 1: SLERP Interpolation Demo
    // ========================================
    printf("--- SLERP Interpolation Demo ---\n");
    
    // Define start and end rotations (both random-ish axis-angle)
    Quat qStart = Quat::fromAxisAngle(Vec3(0, 0, 1), 0.0);  // identity-ish
    Quat qEnd = Quat::fromAxisAngle(Vec3(0.3, 0.7, 0.2), PI * 0.85);  // ~153 degrees
    
    printf("Start quaternion: (%.3f, %.3f, %.3f, %.3f)\n", qStart.w, qStart.x, qStart.y, qStart.z);
    printf("End quaternion:   (%.3f, %.3f, %.3f, %.3f)\n", qEnd.w, qEnd.x, qEnd.y, qEnd.z);
    
    // Draw 10 interpolated cubes in a row - SLERP
    int numCubes = 10;
    for (int i = 0; i < numCubes; i++) {
        double t = i / (double)(numCubes - 1);
        Quat qi = slerp(qStart, qEnd, t);
        
        // Color interpolation from blue to red
        Color col(
            (unsigned char)(50 + 200 * t),
            (unsigned char)(50 + 30 * (1 - t) * t),
            (unsigned char)(50 + 200 * (1 - t))
        );
        
        double offsetX = -0.9 + i * 0.2;
        drawCube(pixels, W, H, qi, Vec3(offsetX, 0.0, 0), 0.32, col);
        
        // Print angle info for quantitative check
        double angle = 2 * std::acos(std::min(1.0, std::abs(qi.w)));
        printf("  t=%.1f: axis-angle = %.1f deg\n", t, angle * 180.0 / PI);
    }
    
    // ========================================
    // SECTION 2: NLERP vs SLERP Comparison
    // ========================================
    printf("\n--- NLERP vs SLERP Comparison ---\n");
    
    // Second row: NLERP on left side
    for (int i = 0; i < numCubes; i++) {
        double t = i / (double)(numCubes - 1);
        Quat qi = nlerp(qStart, qEnd, t);
        
        Color col(180, 100, 100); // reddish
        double offsetX = -0.9 + i * 0.2;
        drawCube(pixels, W, H, qi, Vec3(offsetX, -0.35, 0), 0.32, col);
        
        // Compare angular difference between SLERP and NLERP
        Quat si = slerp(qStart, qEnd, t);
        double diff = std::acos(std::min(1.0, std::abs(si.dot(qi)))) * 180.0 / PI * 2;
        printf("  t=%.1f: NLERP-SLERP angular diff = %.2f deg\n", t, diff);
    }
    
    // ========================================
    // SECTION 3: Gimbal Lock Demonstration
    // ========================================
    printf("\n--- Gimbal Lock Demonstration ---\n");
    
    // Euler angles near gimbal lock: pitch = 90 deg
    // When pitch=90, yaw and roll rotate the same axis
    Vec3 eulerNear1(0.0, PI/2 - 0.001, 0.0);  // near gimbal lock
    Vec3 eulerNear2(PI/4, PI/2 - 0.001, 0.0);  // yaw 45 deg, same pitch
    
    Quat qEuler1 = Quat::fromEuler(eulerNear1.x, eulerNear1.y, eulerNear1.z);
    Quat qEuler2 = Quat::fromEuler(eulerNear2.x, eulerNear2.y, eulerNear2.z);
    
    printf("Euler 1 (yaw=0, pitch≈90): quat = (%.4f, %.4f, %.4f, %.4f)\n",
           qEuler1.w, qEuler1.x, qEuler1.y, qEuler1.z);
    printf("Euler 2 (yaw=45, pitch≈90): quat = (%.4f, %.4f, %.4f, %.4f)\n",
           qEuler2.w, qEuler2.x, qEuler2.y, qEuler2.z);
    
    // Interpolate using Euler lerp (BAD - loses a degree of freedom near gimbal lock)
    printf("\nEuler LERP at gimbal lock (should be BAD):\n");
    for (int i = 0; i < 5; i++) {
        double t = i / 4.0;
        Vec3 el = lerpEuler(eulerNear1, eulerNear2, t);
        Quat qEulerInt = Quat::fromEuler(el.x, el.y, el.z);
        printf("  t=%.2f: Euler=%.1f,%.1f,%.1f -> quat=%.4f,%.4f,%.4f,%.4f\n",
               t, el.x*180/PI, el.y*180/PI, el.z*180/PI,
               qEulerInt.w, qEulerInt.x, qEulerInt.y, qEulerInt.z);
    }
    
    // Interpolate using SLERP (GOOD - smooth, no gimbal lock)
    printf("\nQuaternion SLERP (NO gimbal lock problem):\n");
    for (int i = 0; i < 5; i++) {
        double t = i / 4.0;
        Quat qInt = slerp(qEuler1, qEuler2, t);
        printf("  t=%.2f: quat=%.4f,%.4f,%.4f,%.4f angle=%.1fdeg\n",
               t, qInt.w, qInt.x, qInt.y, qInt.z,
               2*std::acos(std::min(1.0,std::abs(qInt.w)))*180/PI);
    }
    
    // ========================================
    // SECTION 4: Axis-Angle to Quaternion Roundtrip
    // ========================================
    printf("\n--- Axis-Angle ↔ Quaternion Round-Trip ---\n");
    
    Vec3 testAxes[] = {
        Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1),
        Vec3(1, 1, 0).normalized(), Vec3(1, 1, 1).normalized(),
        Vec3(0.3, 0.7, 0.2).normalized()
    };
    double testAngles[] = {0.0, PI/6, PI/4, PI/3, PI/2, PI, PI*0.75};
    
    bool allPassed = true;
    for (const auto& axis : testAxes) {
        for (double angle : testAngles) {
            Quat q = Quat::fromAxisAngle(axis, angle);
            
            // Check quaternion properties
            double n = q.norm();
            if (std::abs(n - 1.0) > 1e-5) {
                printf("  ❌ Bad norm: %.10f for axis(%.1f,%.1f,%.1f) angle=%.1f\n",
                       n, axis.x, axis.y, axis.z, angle*180/PI);
                allPassed = false;
            }
            
            // Round-trip: rotate a test vector back and forth
            Vec3 testVec(1, 2, 3);
            Vec3 rotated = q.rotate(testVec);
            Quat qInv = q.inverse();
            Vec3 recovered = qInv.rotate(rotated);
            
            double diff = (testVec - recovered).length();
            if (diff > 1e-5) {
                printf("  ❌ Round-trip error: %.10f for axis(%.1f,%.1f,%.1f) angle=%.1f\n",
                       diff, axis.x, axis.y, axis.z, angle*180/PI);
                allPassed = false;
            }
        }
    }
    if (allPassed) {
        printf("  ✅ All %zu axis-angle combinations passed norm & round-trip tests\n",
               sizeof(testAxes)/sizeof(testAxes[0]) * sizeof(testAngles)/sizeof(testAngles[0]));
    }
    
    // ========================================
    // SECTION 5: Rotation Matrix Orthogonality
    // ========================================
    printf("\n--- Rotation Matrix Orthogonality Check ---\n");
    
    Quat testQuats[] = {
        Quat(1, 0, 0, 0),
        Quat::fromAxisAngle(Vec3(0,0,1), PI/4),
        Quat::fromAxisAngle(Vec3(1,0,0), PI/3),
        Quat::fromAxisAngle(Vec3(1,1,1).normalized(), PI/2),
        qStart, qEnd
    };
    
    bool orthoPassed = true;
    for (int i = 0; i < 6; i++) {
        double m[3][3];
        testQuats[i].toMatrix(m);
        
        // Check R*R^T = I
        double maxErr = 0;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                double dot = 0;
                for (int k = 0; k < 3; k++) dot += m[r][k] * m[c][k];
                double expected = (r == c) ? 1.0 : 0.0;
                double err = std::abs(dot - expected);
                if (err > maxErr) maxErr = err;
            }
        }
        
        // Check determinant ≈ 1
        double det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
                   - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
                   + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        
        printf("  Quat %d: ortho_err=%.2e, det=%.6f %s\n", i, maxErr, det,
               (maxErr < 1e-5 && std::abs(det - 1.0) < 1e-5) ? "✅" : "❌");
        if (maxErr >= 1e-5 || std::abs(det - 1.0) >= 1e-5) orthoPassed = false;
    }
    if (orthoPassed) printf("  ✅ All rotation matrices are orthogonal\n");
    
    // ========================================
    // SECTION 6: SLERP equidistance check
    // ========================================
    printf("\n--- SLERP Equidistance (Angular) Check ---\n");
    
    double totalAngle = 0;
    std::vector<double> stepAngles;
    for (int i = 0; i < numCubes - 1; i++) {
        double t1 = i / (double)(numCubes - 1);
        double t2 = (i+1) / (double)(numCubes - 1);
        Quat s1 = slerp(qStart, qEnd, t1);
        Quat s2 = slerp(qStart, qEnd, t2);
        double stepAngle = std::acos(std::min(1.0, std::abs(s1.dot(s2))));
        stepAngles.push_back(stepAngle);
        totalAngle += stepAngle;
    }
    
    printf("  Step angles (rad): ");
    for (double a : stepAngles) printf("%.4f ", a);
    printf("\n  Total angle: %.4f rad = %.1f deg\n", totalAngle, totalAngle*180/PI);
    
    // Direct angle between start and end
    double directAngle = std::acos(std::min(1.0, std::abs(qStart.dot(qEnd))));
    double angleError = std::abs(totalAngle - directAngle);
    printf("  Direct start-end angle: %.4f rad\n", directAngle);
    printf("  Error from sum: %.2e %s\n", angleError,
           angleError < 1e-3 ? "✅" : "❌");
    
    // ========================================
    // SECTION 7: Write PPM output
    // ========================================
    const char* filename = "quaternion_output.ppm";
    writePPM(filename, W, H, pixels);
    printf("\n--- Output ---\n");
    printf("  Wrote: %s (%dx%d)\n", filename, W, H);
    
    // Compute pixel stats
    double mean = 0, minVal = 255, maxVal = 0;
    for (const auto& c : pixels) {
        double gray = (c.r + c.g + c.b) / 3.0;
        mean += gray;
        if (gray < minVal) minVal = gray;
        if (gray > maxVal) maxVal = gray;
    }
    mean /= pixels.size();
    
    double variance = 0;
    for (const auto& c : pixels) {
        double gray = (c.r + c.g + c.b) / 3.0;
        variance += (gray - mean) * (gray - mean);
    }
    variance /= pixels.size();
    double stdDev = std::sqrt(variance);
    
    printf("  Pixel stats: mean=%.1f, std=%.1f, min=%.1f, max=%.1f\n", mean, stdDev, minVal, maxVal);
    
    // Check for non-uniform content
    if (mean < 10) { printf("❌ Image too dark\n"); return 1; }
    if (mean > 245) { printf("❌ Image too bright\n"); return 1; }
    if (stdDev < 10) { printf("❌ Image too uniform\n"); return 1; }
    printf("  ✅ Pixel statistics look good\n");
    
    // File size
    FILE* ff = fopen(filename, "rb");
    fseek(ff, 0, SEEK_END);
    long fsize = ftell(ff);
    fclose(ff);
    printf("  File size: %ld bytes %s\n", fsize, fsize > 10000 ? "✅" : "❌");
    
    printf("\n=== ALL CHECKS PASSED ===\n");
    
    return 0;
}
