/**
 * Spherical Harmonics Environment Lighting
 *
 * 技术要点：
 * - L0-L2 球谐函数（9个系数），对低频环境光照进行投影
 * - 程序化天空光（蓝天 + 地面棕色 + 太阳方向白光）
 * - 用 SH 系数对物体表面法线方向求辐照度（Irradiance）
 * - 渲染 3 个不同材质的球：漫反射、金属感、混合材质
 * - 对比：直接MC采样 vs SH 近似的可视化
 *
 * 参考：Ramamoorthi & Hanrahan 2001, "An Efficient Representation for Irradiance Environment Maps"
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <random>
#include <functional>
#include <sstream>
#include <iomanip>

// ──────────────────────────────────────────────
// Math helpers
// ──────────────────────────────────────────────
const double PI = 3.14159265358979323846;

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double t)       const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(double t)       const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalize() const { double l = length(); return l>1e-12 ? *this/l : Vec3(0,0,0); }
    Vec3 clamp01() const {
        return {std::max(0.0, std::min(1.0, x)),
                std::max(0.0, std::min(1.0, y)),
                std::max(0.0, std::min(1.0, z))};
    }
    Vec3 neg() const { return {-x, -y, -z}; }
};

inline Vec3 operator*(double t, const Vec3& v) { return v*t; }
inline Vec3 mix(const Vec3& a, const Vec3& b, double t) { return a*(1-t) + b*t; }
inline Vec3 pow_vec(const Vec3& v, double p) {
    return {std::pow(std::max(0.0,v.x),p),
            std::pow(std::max(0.0,v.y),p),
            std::pow(std::max(0.0,v.z),p)};
}

// ──────────────────────────────────────────────
// Spherical Harmonics (L=0,1,2 → 9 coefficients)
// ──────────────────────────────────────────────
// Real SH basis evaluated at unit direction (x,y,z)
// Using Ramamoorthi & Hanrahan 2001 convention

inline double Y00 (const Vec3& /*d*/) { return 0.5 * std::sqrt(1.0/PI); }
inline double Y1m1(const Vec3& d) { return std::sqrt(3.0/(4*PI)) * d.y; }
inline double Y10 (const Vec3& d) { return std::sqrt(3.0/(4*PI)) * d.z; }
inline double Y11 (const Vec3& d) { return std::sqrt(3.0/(4*PI)) * d.x; }
inline double Y2m2(const Vec3& d) { return 0.5*std::sqrt(15.0/PI) * d.x*d.y; }
inline double Y2m1(const Vec3& d) { return 0.5*std::sqrt(15.0/PI) * d.y*d.z; }
inline double Y20 (const Vec3& d) { return 0.25*std::sqrt(5.0/PI)*(3*d.z*d.z - 1); }
inline double Y21 (const Vec3& d) { return 0.5*std::sqrt(15.0/PI) * d.x*d.z; }
inline double Y22 (const Vec3& d) { return 0.25*std::sqrt(15.0/PI)*(d.x*d.x - d.y*d.y); }

struct SHCoeffs {
    Vec3 c[9];
    SHCoeffs() { for(int i=0;i<9;i++) c[i] = Vec3(0,0,0); }
};

void shBasis(const Vec3& d, double b[9]) {
    b[0] = Y00(d);
    b[1] = Y1m1(d);
    b[2] = Y10(d);
    b[3] = Y11(d);
    b[4] = Y2m2(d);
    b[5] = Y2m1(d);
    b[6] = Y20(d);
    b[7] = Y21(d);
    b[8] = Y22(d);
}

SHCoeffs projectEnvToSH(std::function<Vec3(const Vec3&)> env, int numSamples) {
    SHCoeffs coeffs;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> distU(0.0, 1.0);

    double weight = 4.0 * PI / numSamples;

    for(int i = 0; i < numSamples; i++) {
        double u = distU(rng);
        double v = distU(rng);
        double theta = std::acos(1.0 - 2.0*u);
        double phi   = 2.0 * PI * v;
        Vec3 dir = {
            std::sin(theta)*std::cos(phi),
            std::sin(theta)*std::sin(phi),
            std::cos(theta)
        };

        Vec3 radiance = env(dir);
        double b[9];
        shBasis(dir, b);

        for(int j = 0; j < 9; j++) {
            coeffs.c[j] += radiance * b[j];
        }
    }

    for(int j = 0; j < 9; j++) {
        coeffs.c[j] = coeffs.c[j] * weight;
    }
    return coeffs;
}

// Reconstruct irradiance from SH at normal n
// E(n) = π * Σ A_l * L_lm * Y_lm(n)
// Lambertian convolution: A0 = π, A1 = 2π/3, A2 = π/4
Vec3 shIrradiance(const SHCoeffs& sh, const Vec3& n) {
    const double A0 = PI;
    const double A1 = 2.0*PI/3.0;
    const double A2 = PI/4.0;

    double b[9];
    shBasis(n, b);

    Vec3 irr(0,0,0);
    irr += sh.c[0] * (A0 * b[0]);
    irr += sh.c[1] * (A1 * b[1]);
    irr += sh.c[2] * (A1 * b[2]);
    irr += sh.c[3] * (A1 * b[3]);
    irr += sh.c[4] * (A2 * b[4]);
    irr += sh.c[5] * (A2 * b[5]);
    irr += sh.c[6] * (A2 * b[6]);
    irr += sh.c[7] * (A2 * b[7]);
    irr += sh.c[8] * (A2 * b[8]);

    return {std::max(0.0, irr.x), std::max(0.0, irr.y), std::max(0.0, irr.z)};
}

// ──────────────────────────────────────────────
// Procedural sky environment
// ──────────────────────────────────────────────
Vec3 skyColor(const Vec3& dir) {
    Vec3 sunDir = Vec3(0.3, 0.7, 0.5).normalize();
    double sunDot = std::max(0.0, dir.dot(sunDir));

    if(dir.y < 0) {
        double t = std::min(1.0, -dir.y * 3.0);
        return mix(Vec3(0.3, 0.25, 0.15), Vec3(0.25, 0.2, 0.12), t);
    }

    double horizon = std::exp(-dir.y * 4.0);
    Vec3 zenith   = Vec3(0.05, 0.12, 0.50);
    Vec3 horizonC = Vec3(0.50, 0.65, 0.85);
    Vec3 sky = mix(zenith, horizonC, horizon);

    double sunHalo = std::pow(sunDot, 32.0) * 2.5;
    double sunDisk = std::pow(sunDot, 512.0) * 10.0;
    Vec3 result = sky + Vec3(1.0, 0.95, 0.7) * (sunHalo + sunDisk);

    Vec3 horizonGlow = Vec3(0.9, 0.55, 0.2) * std::pow(horizon, 3.0) * 0.5;
    result += horizonGlow;

    return result;
}

// Reference: cosine-weighted hemisphere MC irradiance
Vec3 directIrradiance(const Vec3& normal, int numSamples) {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> distU(0.0, 1.0);

    // Build TBN
    Vec3 up = std::abs(normal.y) < 0.99 ? Vec3(0,1,0) : Vec3(1,0,0);
    Vec3 tangent   = normal.cross(up).normalize();
    Vec3 bitangent = normal.cross(tangent).normalize();

    Vec3 irr(0,0,0);
    for(int i = 0; i < numSamples; i++) {
        double u = distU(rng);
        double v = distU(rng);
        double r   = std::sqrt(u);
        double phi = 2.0*PI*v;

        // Local cosine-weighted sample
        Vec3 worldDir = (tangent   * (r*std::cos(phi)) +
                         bitangent * (r*std::sin(phi)) +
                         normal    * std::sqrt(1.0-u)).normalize();

        irr += skyColor(worldDir);
    }
    // pdf = NdotL / π, so E = π/N * Σ L
    return irr * (PI / numSamples);
}

// ──────────────────────────────────────────────
// Ray-sphere intersection
// ──────────────────────────────────────────────
struct Ray { Vec3 origin, dir; };
struct Sphere { Vec3 center; double radius; };

bool intersectSphere(const Ray& ray, const Sphere& sphere, double& t) {
    Vec3 oc = ray.origin - sphere.center;
    double a = ray.dir.dot(ray.dir);
    double b = 2.0 * oc.dot(ray.dir);
    double c = oc.dot(oc) - sphere.radius*sphere.radius;
    double disc = b*b - 4*a*c;
    if(disc < 0) return false;
    double sq = std::sqrt(disc);
    double t0 = (-b - sq) / (2*a);
    double t1 = (-b + sq) / (2*a);
    t = (t0 > 0.001) ? t0 : (t1 > 0.001 ? t1 : -1.0);
    return t > 0;
}

// ──────────────────────────────────────────────
// Tone mapping & gamma
// ──────────────────────────────────────────────
Vec3 acesTonemap(const Vec3& x) {
    const double a=2.51, b=0.03, c=2.43, d=0.59, e=0.14;
    auto f = [&](double v){ return (v*(a*v+b))/(v*(c*v+d)+e); };
    return Vec3(f(x.x), f(x.y), f(x.z)).clamp01();
}

Vec3 gammaCorrect(const Vec3& col) { return pow_vec(col.clamp01(), 1.0/2.2); }

// ──────────────────────────────────────────────
// Image
// ──────────────────────────────────────────────
struct Image {
    int W, H;
    std::vector<Vec3> pixels;
    Image(int w, int h) : W(w), H(h), pixels(w*h) {}
    Vec3& at(int x, int y) { return pixels[y*W+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*W+x]; }
    void savePPM(const std::string& path) const {
        std::ofstream f(path, std::ios::binary);
        f << "P6\n" << W << " " << H << "\n255\n";
        for(const auto& p : pixels) {
            unsigned char r = (unsigned char)(std::min(1.0,std::max(0.0,p.x))*255.999);
            unsigned char g = (unsigned char)(std::min(1.0,std::max(0.0,p.y))*255.999);
            unsigned char b = (unsigned char)(std::min(1.0,std::max(0.0,p.z))*255.999);
            f.write((char*)&r,1); f.write((char*)&g,1); f.write((char*)&b,1);
        }
    }
};

// ──────────────────────────────────────────────
// Materials & shading
// ──────────────────────────────────────────────
struct Material {
    Vec3 albedo;
    double metallic, roughness;
};

Vec3 fresnelSchlick(double cosTheta, const Vec3& F0) {
    double f = std::pow(1.0 - std::max(0.0, cosTheta), 5.0);
    return F0 + (Vec3(1,1,1) - F0) * f;
}

Vec3 shadeWithSH(const Vec3& normal, const Vec3& viewDir,
                 const Material& mat, const SHCoeffs& sh)
{
    Vec3 irradiance = shIrradiance(sh, normal);
    Vec3 F0 = mix(Vec3(0.04,0.04,0.04), mat.albedo, mat.metallic);
    Vec3 kS = fresnelSchlick(std::max(0.0, normal.dot(viewDir)), F0);
    Vec3 kD = (Vec3(1,1,1) - kS) * (1.0 - mat.metallic);
    Vec3 diffuse = kD * mat.albedo * irradiance;

    Vec3 sunDir = Vec3(0.3, 0.7, 0.5).normalize();
    Vec3 halfVec = (viewDir + sunDir).normalize();
    double NdotH = std::max(0.0, normal.dot(halfVec));
    double shininess = std::max(1.0, 512.0 * (1.0 - mat.roughness*mat.roughness));
    double spec = std::pow(NdotH, shininess) * (1.0 - mat.roughness);
    Vec3 specular = kS * Vec3(1.2,1.1,0.9) * 2.0 * spec;

    return diffuse + specular;
}

Vec3 shadeWithMC(const Vec3& normal, const Vec3& viewDir,
                 const Material& mat, int numSamples)
{
    Vec3 irradiance = directIrradiance(normal, numSamples);
    Vec3 F0 = mix(Vec3(0.04,0.04,0.04), mat.albedo, mat.metallic);
    Vec3 kS = fresnelSchlick(std::max(0.0, normal.dot(viewDir)), F0);
    Vec3 kD = (Vec3(1,1,1) - kS) * (1.0 - mat.metallic);
    Vec3 diffuse = kD * mat.albedo * irradiance;

    Vec3 sunDir = Vec3(0.3, 0.7, 0.5).normalize();
    Vec3 halfVec = (viewDir + sunDir).normalize();
    double NdotH = std::max(0.0, normal.dot(halfVec));
    double shininess = std::max(1.0, 512.0 * (1.0 - mat.roughness*mat.roughness));
    double spec = std::pow(NdotH, shininess) * (1.0 - mat.roughness);
    Vec3 specular = kS * Vec3(1.2,1.1,0.9) * 2.0 * spec;

    return diffuse + specular;
}

// ──────────────────────────────────────────────
// Scene rendering
// ──────────────────────────────────────────────
Image renderScene(int W, int H, const SHCoeffs* sh, bool useSH) {
    Image img(W, H);

    Vec3 camPos(0, 0.5, 4.0);
    double fov = 55.0 * PI / 180.0;
    double aspect = (double)W / H;

    std::vector<Sphere> spheres = {
        {{-1.8, 0, 0}, 0.85},
        {{ 0.0, 0, 0}, 0.85},
        {{ 1.8, 0, 0}, 0.85}
    };
    double groundY = -0.85;

    std::vector<Material> materials = {
        {Vec3(0.9, 0.2, 0.1), 0.0, 0.7},   // rough red diffuse
        {Vec3(0.8, 0.7, 0.1), 1.0, 0.1},   // gold metal
        {Vec3(0.2, 0.6, 0.9), 0.0, 0.3}    // smooth blue diffuse
    };

    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            double u = (px + 0.5) / W;
            double v = (py + 0.5) / H;

            double ndcX = (2*u - 1) * aspect * std::tan(fov/2);
            double ndcY = (1 - 2*v) * std::tan(fov/2);
            Vec3 rayDir = Vec3(ndcX, ndcY, -1).normalize();
            Ray ray = {camPos, rayDir};

            double tMin = 1e18;
            int hitSphere = -1;

            for(int i = 0; i < (int)spheres.size(); i++) {
                double t;
                if(intersectSphere(ray, spheres[i], t) && t < tMin) {
                    tMin = t; hitSphere = i;
                }
            }

            bool hitGround = false;
            if(ray.dir.y != 0.0) {
                double tG = (groundY - ray.origin.y) / ray.dir.y;
                if(tG > 0.001 && tG < tMin) {
                    tMin = tG; hitGround = true; hitSphere = -1;
                }
            }

            Vec3 color;

            if(hitSphere >= 0) {
                Vec3 hitPos = ray.origin + ray.dir * tMin;
                Vec3 normal = (hitPos - spheres[hitSphere].center).normalize();
                Vec3 viewDir = rayDir.neg().normalize();

                if(useSH) {
                    color = shadeWithSH(normal, viewDir, materials[hitSphere], *sh);
                } else {
                    color = shadeWithMC(normal, viewDir, materials[hitSphere], 512);
                }
            } else if(hitGround) {
                Vec3 hitPos = ray.origin + ray.dir * tMin;
                Vec3 normal = Vec3(0, 1, 0);
                Vec3 viewDir = rayDir.neg().normalize();

                double cx = std::floor(hitPos.x * 1.5);
                double cz = std::floor(hitPos.z * 1.5);
                double checker = std::fmod(std::abs(cx + cz), 2.0);
                Material groundMat;
                groundMat.albedo   = checker < 1.0 ? Vec3(0.85,0.82,0.75) : Vec3(0.4,0.38,0.33);
                groundMat.metallic = 0.0;
                groundMat.roughness = 0.9;

                if(useSH) {
                    color = shadeWithSH(normal, viewDir, groundMat, *sh);
                } else {
                    color = shadeWithMC(normal, viewDir, groundMat, 256);
                }
            } else {
                color = skyColor(rayDir);
            }

            color = acesTonemap(color);
            color = gammaCorrect(color);
            img.at(px, py) = color;
        }
    }

    return img;
}

// SH basis function visualization (3×3 grid)
Image renderSHBasis(int W, int H) {
    Image img(W, H);
    int cols = 3;
    int cellW = W / cols, cellH = H / cols;

    for(int idx = 0; idx < 9; idx++) {
        int cellCol = idx % cols;
        int cellRow = idx / cols;

        for(int cy = 0; cy < cellH; cy++) {
            for(int cx = 0; cx < cellW; cx++) {
                int px = cellCol * cellW + cx;
                int py = cellRow * cellH + cy;

                double u = (cx + 0.5) / cellW;
                double v = (cy + 0.5) / cellH;

                double theta = v * PI;
                double phi   = u * 2 * PI;
                Vec3 dir = {
                    std::sin(theta)*std::cos(phi),
                    std::cos(theta),
                    std::sin(theta)*std::sin(phi)
                };

                double b[9];
                shBasis(dir, b);
                double val = b[idx];

                Vec3 color;
                if(val > 0) {
                    color = Vec3(val * 2.0, val * 0.6, 0.0).clamp01();
                } else {
                    color = Vec3(0.0, 0.0, -val * 2.0).clamp01();
                }
                img.at(px, py) = color;
            }
        }
    }

    return img;
}

// Environment panorama
Image renderEnvPanorama(int W, int H,
                         bool useOriginal, const SHCoeffs* sh) {
    Image img(W, H);
    for(int py = 0; py < H; py++) {
        for(int px = 0; px < W; px++) {
            double u = (px + 0.5) / W;
            double v = (py + 0.5) / H;
            double theta = v * PI;
            double phi   = u * 2 * PI;
            Vec3 dir = {
                std::sin(theta)*std::cos(phi),
                std::cos(theta),
                std::sin(theta)*std::sin(phi)
            };

            Vec3 color;
            if(useOriginal) {
                color = skyColor(dir);
            } else {
                // Reconstruct from SH
                double b[9]; shBasis(dir, b);
                Vec3 recon(0,0,0);
                for(int j=0;j<9;j++) recon += sh->c[j] * b[j];
                color = {std::max(0.0,recon.x), std::max(0.0,recon.y), std::max(0.0,recon.z)};
            }
            color = acesTonemap(color);
            color = gammaCorrect(color);
            img.at(px, py) = color;
        }
    }
    return img;
}

bool ppmToPng(const std::string& ppm, const std::string& png) {
    // Try ImageMagick first, then Python PIL
    std::string cmd = "convert " + ppm + " " + png + " 2>/dev/null";
    if(system(cmd.c_str()) == 0) return true;
    // Fallback: Python PIL
    cmd = "python3 -c \"from PIL import Image; Image.open('" + ppm + "').save('" + png + "')\" 2>/dev/null";
    return system(cmd.c_str()) == 0;
}

// ──────────────────────────────────────────────
// Main
// ──────────────────────────────────────────────
int main() {
    std::cout << "=== Spherical Harmonics Environment Lighting ===" << std::endl;
    std::cout << "Projecting sky environment into L2 SH (9 coefficients)..." << std::endl;

    // Project sky to SH
    SHCoeffs sh = projectEnvToSH(skyColor, 65536);

    // Print coefficients
    const char* basisNames[9] = {"Y00","Y1-1","Y10","Y11","Y2-2","Y2-1","Y20","Y21","Y22"};
    for(int i = 0; i < 9; i++) {
        std::cout << "  " << std::setw(5) << basisNames[i] << ": ("
                  << std::fixed << std::setprecision(4)
                  << sh.c[i].x << ", " << sh.c[i].y << ", " << sh.c[i].z << ")\n";
    }

    // Render SH scene
    std::cout << "\nRendering SH-lit scene (800x400)..." << std::endl;
    {
        auto img = renderScene(800, 400, &sh, true);
        img.savePPM("/tmp/sh_output_tmp.ppm");
        if(ppmToPng("/tmp/sh_output_tmp.ppm", "sh_output.png"))
            std::cout << "  ✅ sh_output.png\n";
    }

    // Render MC reference scene
    std::cout << "\nRendering MC reference scene (800x400)..." << std::endl;
    {
        auto img = renderScene(800, 400, nullptr, false);
        img.savePPM("/tmp/sh_mc_tmp.ppm");
        if(ppmToPng("/tmp/sh_mc_tmp.ppm", "sh_mc_reference.png"))
            std::cout << "  ✅ sh_mc_reference.png\n";
    }

    // SH basis visualization
    std::cout << "\nRendering SH basis functions (600x600)..." << std::endl;
    {
        auto img = renderSHBasis(600, 600);
        img.savePPM("/tmp/sh_basis_tmp.ppm");
        if(ppmToPng("/tmp/sh_basis_tmp.ppm", "sh_basis.png"))
            std::cout << "  ✅ sh_basis.png\n";
    }

    // Environment probe comparison (original top, SH bottom)
    std::cout << "\nRendering env probe comparison (800x400)..." << std::endl;
    {
        auto orig  = renderEnvPanorama(800, 200, true,  nullptr);
        auto recon = renderEnvPanorama(800, 200, false, &sh);
        Image combined(800, 400);
        for(int y=0;y<200;y++) for(int x=0;x<800;x++) combined.at(x,y)=orig.at(x,y);
        for(int y=0;y<200;y++) for(int x=0;x<800;x++) combined.at(x,y+200)=recon.at(x,y);
        combined.savePPM("/tmp/sh_probe_tmp.ppm");
        if(ppmToPng("/tmp/sh_probe_tmp.ppm", "sh_probe_comparison.png"))
            std::cout << "  ✅ sh_probe_comparison.png\n";
    }

    // Side-by-side comparison (SH | MC)
    std::cout << "\nRendering SH vs MC comparison (1600x400)..." << std::endl;
    {
        auto shImg = renderScene(800, 400, &sh,   true);
        auto mcImg = renderScene(800, 400, nullptr, false);
        Image combined(1600, 400);
        for(int y=0;y<400;y++) {
            for(int x=0;x<800;x++) {
                combined.at(x,     y) = shImg.at(x, y);
                combined.at(x+800, y) = mcImg.at(x, y);
            }
        }
        combined.savePPM("/tmp/sh_comparison_tmp.ppm");
        if(ppmToPng("/tmp/sh_comparison_tmp.ppm", "sh_comparison.png"))
            std::cout << "  ✅ sh_comparison.png\n";
    }

    std::cout << "\n✅ All renders complete!" << std::endl;
    std::cout << "Files: sh_output.png, sh_mc_reference.png, sh_basis.png, "
                 "sh_probe_comparison.png, sh_comparison.png" << std::endl;

    return 0;
}
