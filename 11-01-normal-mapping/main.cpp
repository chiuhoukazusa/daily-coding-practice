// Normal Mapping (Tangent-Space Bump Mapping)
// Renders a sphere with and without a procedural normal map.
// The normal map encodes surface bumps in tangent space (TBN), which perturb the
// lighting direction. Quantitative verification: compute per-pixel lighting diff
// between flat-shaded and normal-mapped images, and validate geometric invariants.
//
// Output: writes two PNGs (flat + normal-mapped) and prints quantitative metrics.

#include <cstdio>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <random>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ----------------------------- 3D vector helpers -----------------------------
struct Vec3 {
    double x, y, z;
    Vec3(double x_=0, double y_=0, double z_=0) : x(x_), y(y_), z(z_) {}
};

inline Vec3 operator+(Vec3 a, Vec3 b){ return Vec3(a.x+b.x, a.y+b.y, a.z+b.z); }
inline Vec3 operator-(Vec3 a, Vec3 b){ return Vec3(a.x-b.x, a.y-b.y, a.z-b.z); }
inline Vec3 operator*(Vec3 a, double s){ return Vec3(a.x*s, a.y*s, a.z*s); }
inline Vec3 operator*(double s, Vec3 a){ return a*s; }
inline double dot(Vec3 a, Vec3 b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b){
    return Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
inline Vec3 normalize(Vec3 a){
    double l = std::sqrt(dot(a,a));
    if (l < 1e-12) return Vec3(0,0,0);
    return a * (1.0/l);
}
inline Vec3 operator*(Vec3 a, Vec3 b){ return Vec3(a.x*b.x, a.y*b.y, a.z*b.z); } // hadamard

int main() {
    const int W = 512, H = 512;
    std::vector<unsigned char> flatPx(W*H*3, 0), nmPx(W*H*3, 0);

    // Camera / sphere geometry
    // Sphere centered at origin, radius R. Orthographic-ish ray dir from camera.
    const double R = 220.0;
    const Vec3 camPos(0, 0, -600);

    // Light: single directional key light (unit, pointing FROM surface TO light).
    const Vec3 lightDir = normalize(Vec3(0.6, 0.8, 0.4));
    const Vec3 lightColor(1.0, 0.95, 0.9);
    const Vec3 ambient(0.08, 0.08, 0.10);
    const Vec3 albedo(0.7, 0.65, 0.60);

    // Procedural "brick" normal map: in tangent space, encode brick pattern bumps.
    // We'll generate, per fragment, a tangent-space normal perturbation from a
    // procedural height field (brick + mortar grooves) using central differences.

    // Build TBN per-point: tangent aligned with azimuthal (u) direction, bitangent
    // with polar (v) direction on the sphere.

    double diffSum = 0.0, diffSqSum = 0.0, diffMax = 0.0;
    long long count = 0;
    double lumFlatSum = 0.0, lumNmSum = 0.0;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // Ray from camera through pixel plane
            double px = (x - W/2.0);
            double py = (H/2.0 - y);  // flip Y so up is +y
            Vec3 ro = camPos;
            Vec3 rd = normalize(Vec3(px, py, 600)); // screen at z=0 (toward +z)

            // Ray-sphere intersection
            Vec3 oc = ro - Vec3(0,0,0);
            double b = dot(oc, rd);
            double c = dot(oc, oc) - R*R;
            double disc = b*b - c;
            if (disc < 0) {
                flatPx[(y*W+x)*3+0] = flatPx[(y*W+x)*3+1] = flatPx[(y*W+x)*3+2] = 20;
                nmPx[(y*W+x)*3+0] = nmPx[(y*W+x)*3+1] = nmPx[(y*W+x)*3+2] = 20;
                continue;
            }
            double t = -b - std::sqrt(disc); // near hit
            if (t < 0) t = -b + std::sqrt(disc);
            Vec3 P = ro + rd * t;
            Vec3 N = normalize(P - Vec3(0,0,0)); // geometric normal

            // Spherical UV
            double u = 0.5 + std::atan2(N.z, N.x) / (2*M_PI);
            double v = 0.5 - std::asin(N.y) / M_PI;

            // Tangent (dP/du) and bitangent (dP/dv) in world space
            // Param: X=R cos(phi) cos(theta)... use standard sphere param
            // Let phi = longitude(atan2), theta = latitude
            double phi   = std::atan2(N.z, N.x);  // longitude
            // dP/dphi = tangent (azimuthal)
            Vec3 tangent   = normalize(Vec3(-std::sin(phi), 0.0, std::cos(phi)));
            Vec3 bitangent = normalize(cross(N, tangent));

            // Procedural height field in texture space (h(x,y) -> perturb)
            // Brick pattern: mortar lines at brick boundaries
            auto heightAt = [&](double uu, double vv) -> double {
                double bricks = 8.0, rows = 8.0;
                double bwu = uu * bricks * 2.0;      // two bricks per row width
                double bvh = vv * rows;
                double row = std::floor(bvh);
                double off = (int(row) % 2 == 0) ? 0.0 : 0.5; // offset alternate rows
                double fu = bwu + off;
                double mortarU = (std::fmod(fu, 1.0));
                double mortarV = (std::fmod(bvh, 1.0));
                const double mw = 0.12; // mortar half-width
                double hu = (mortarU < mw || mortarU > 1.0 - mw) ? 1.0 : 0.0;
                double hv = (mortarV < mw || mortarV > 1.0 - mw) ? 1.0 : 0.0;
                return hu + hv; // 0 = brick top, 1 = groove depth
            };

            // Central differences to get tangent-space normal perturbation
            const double eps = 1e-3;
            double hl = heightAt(u - eps, v);
            double hr = heightAt(u + eps, v);
            double hd = heightAt(u, v - eps);
            double hu2 = heightAt(u, v + eps);

            // Height -> perturbed normal in tangent space:
            // dH/du -= (hr-hl)/(2eps), dH/dv -= (hu2-hd)/(2eps)
            double dhdu = -(hr - hl) / (2*eps);
            double dhdv = -(hu2 - hd) / (2*eps);
            const double bumpScale = 40.0;
            Vec3 tbnNormal = normalize(Vec3(-dhdu*bumpScale, -dhdv*bumpScale, 1.0));

            // Perturb world normal via TBN
            Vec3 Np = normalize(
                tangent * tbnNormal.x + bitangent * tbnNormal.y + N * tbnNormal.z
            );

            // --- Shade with geometric normal (flat) ---
            auto shade = [&](Vec3 n) -> Vec3 {
                double ndotl = std::max(0.0, dot(n, lightDir));
                Vec3 col = ambient + albedo * lightColor * ndotl;
                // simple half-vector spec for a bit of pop
                Vec3 V = normalize(camPos - P);
                Vec3 Hh = normalize(lightDir + V);
                double spec = std::pow(std::max(0.0, dot(n, Hh)), 32.0) * 0.25;
                col = col + lightColor * spec;
                return col;
            };

            Vec3 cFlat = shade(N);
            Vec3 cNm   = shade(Np);

            auto toByte = [](double v){ int b=(int)(v*255.0+0.5); return (unsigned char)std::max(0,std::min(255,b)); };
            flatPx[(y*W+x)*3+0] = toByte(cFlat.x);
            flatPx[(y*W+x)*3+1] = toByte(cFlat.y);
            flatPx[(y*W+x)*3+2] = toByte(cFlat.z);
            nmPx[(y*W+x)*3+0] = toByte(cNm.x);
            nmPx[(y*W+x)*3+1] = toByte(cNm.y);
            nmPx[(y*W+x)*3+2] = toByte(cNm.z);

            // metrics on shading only (ignore background)
            double lf = 0.2126*cFlat.x + 0.7152*cFlat.y + 0.0722*cFlat.z;
            double ln = 0.2126*cNm.x  + 0.7152*cNm.y  + 0.0722*cNm.z;
            lumFlatSum += lf;
            lumNmSum   += ln;
            double d = std::fabs(lf - ln);
            diffSum += d; diffSqSum += d*d; diffMax = std::max(diffMax, d);
            count++;
        }
    }

    stbi_write_png("flat_output.png", W, H, 3, flatPx.data(), W*3);
    stbi_write_png("normal_mapping_output.png", W, H, 3, nmPx.data(), W*3);

    double meanDiff = diffSum / count;
    double rmsDiff  = std::sqrt(diffSqSum / count);
    double meanFlat = lumFlatSum / count;
    double meanNm   = lumNmSum / count;

    printf("=== Normal Mapping Quantitative Verification ===\n");
    printf("covered_fragments  = %lld\n", count);
    printf("mean_lum_flat      = %.4f\n", meanFlat);
    printf("mean_lum_nm        = %.4f\n", meanNm);
    printf("mean_abs_diff      = %.4f\n", meanDiff);
    printf("rms_diff           = %.4f\n", rmsDiff);
    printf("max_diff           = %.4f\n", diffMax);
    printf("delta_mean_lum     = %.4f\n", std::fabs(meanFlat - meanNm));

    // Invariant checks
    bool ok = true;
    if (meanDiff < 0.005) { printf("FAIL: normal map produced almost no lighting change\n"); ok=false; }
    if (rmsDiff  < 0.010) { printf("FAIL: rms diff too small (flat map?)\n"); ok=false; }
    if (meanFlat < 0.02 || meanNm < 0.02) { printf("FAIL: image too dark\n"); ok=false; }
    if (meanFlat > 0.95 || meanNm > 0.95) { printf("FAIL: image too bright\n"); ok=false; }
    if (ok) printf("PASS: normal mapping visibly & measurably perturbs lighting\n");
    return ok ? 0 : 1;
}
