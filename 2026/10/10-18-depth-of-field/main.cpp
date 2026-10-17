// Depth of Field Renderer — Thin-Lens Model with Monte Carlo Sampling
// 2026-10-18
//
// A path-traced style renderer that models a thin lens with a finite aperture.
// Rays are traced from random points on the lens disk toward a focal plane,
// so that objects at the focal distance stay sharp while objects nearer/farther
// blur proportionally to their circle-of-confusion radius.
//
// Quantitative verification (not visual):
//   * We render the SAME scene at 6 different aperture radii and measure
//     sharpness (local gradient energy) of a high-frequency test pattern at
//     three depths: in front of, at, and behind the focal plane.
//   * At the focal plane, sharpness must remain high regardless of aperture.
//   * Away from the focal plane, sharpness must DECREASE monotonically as
//     aperture increases (more depth blur).

#include <cstdio>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <string>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

// ---- Vec3 ----
struct Vec3 {
    double x, y, z;
    Vec3(double x_=0,double y_=0,double z_=0):x(x_),y(y_),z(z_){}
    Vec3 operator+(const Vec3&o)const{return Vec3(x+o.x,y+o.y,z+o.z);}
    Vec3 operator-(const Vec3&o)const{return Vec3(x-o.x,y-o.y,z-o.z);}
    Vec3 operator*(double s)const{return Vec3(x*s,y*s,z*s);}
    Vec3 operator/(double s)const{return Vec3(x/s,y/s,z/s);}
    double dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return Vec3(y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x);}
    double len()const{return std::sqrt(dot(*this));}
    Vec3 norm()const{double l=len();return l>0?(*this)/l:Vec3(0,0,0);}
};

// ---- Sphere ----
struct Sphere {
    Vec3 c;
    double r;
    Vec3 color;
    bool diffuse; // true = lambertian albedo, false = high-freq checker (test pattern)
};

static inline bool hit(const Sphere&s, const Vec3&o, const Vec3&d, double&t){
    Vec3 oc = o - s.c;
    double b = oc.dot(d);
    double c = oc.dot(oc) - s.r*s.r;
    double disc = b*b - c;
    if(disc < 0) return false;
    double sq = std::sqrt(disc);
    double t1 = -b - sq, t2 = -b + sq;
    t = (t1 > 1e-4) ? t1 : t2;
    return t > 1e-4;
}

// High-frequency checker pattern on a sphere (for sharpness measurement).
static inline Vec3 checker(const Vec3& p, double scale){
    double u = std::atan2(p.z, p.x);
    double v = std::acos(std::max(-1.0, std::min(1.0, p.y)));
    int cu = (int)std::floor(u * scale);
    int cv = (int)std::floor(v * scale);
    bool on = ((cu + cv) & 1) == 0;
    return on ? Vec3(0.95,0.95,0.95) : Vec3(0.05,0.08,0.12);
}

// ---- Scene ----
// Test spheres placed at three depths along +z:
//   near    z = 2.5  (in front of focal plane)
//   focal   z = 5.0  (at focal plane)
//   far     z = 8.0  (behind focal plane)
static const double focalDist = 5.0;

static std::vector<Sphere> makeScene(){
    std::vector<Sphere> s;
    s.push_back({Vec3(-2.5, 0, 2.5), 1.0, Vec3(1,1,1), false}); // near, checker
    s.push_back({Vec3( 0.0, 0, 5.0), 1.0, Vec3(1,1,1), false}); // focal, checker
    s.push_back({Vec3( 2.5, 0, 8.0), 1.0, Vec3(1,1,1), false}); // far, checker
    s.push_back({Vec3( 0.0,-101, 5.0), 100.0, Vec3(0.35,0.35,0.45), true}); // floor (no high-freq)
    return s;
}

// ---- Renderer ----
// Camera at origin looking down +z. Image plane at z = focalDist with FOV fixed.
// Thin lens: sensor at focal plane, aperture radius A; each ray passes through
// a random point on the lens disk at z=0.
static void render(std::vector<unsigned char>& img, int W, int H,
                   double aperture, int spp, uint32_t seed){
    auto scene = makeScene();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(0.0, 1.0);

    double halfH = std::tan(25.0*M_PI/180.0) * focalDist; // fixed FOV
    double halfW = halfH * (double)W / (double)H;

    img.assign(W*H*3, 0);

    for(int py=0; py<H; ++py){
        for(int px=0; px<W; ++px){
            double acc[3] = {0,0,0};
            // pixel center on the focal plane (sensor)
            double sx = (px + 0.5 - W/2.0) / (W/2.0) * halfW;
            double sy = (H/2.0 - (py + 0.5)) / (H/2.0) * halfH;
            Vec3 sensor(sx, sy, focalDist);

            for(int s=0; s<spp; ++s){
                // random point on lens disk
                double r = aperture * std::sqrt(U(rng));
                double a = 2.0*M_PI*U(rng);
                Vec3 lensPt(std::cos(a)*r, std::sin(a)*r, 0.0);

                // ray from lens point through the sensor pixel -> direction
                Vec3 dir = (sensor - lensPt).norm();
                Vec3 ro = lensPt;

                // intersect scene
                double tbest = 1e9;
                const Sphere* best = nullptr;
                for(const auto& sp : scene){
                    double t;
                    if(hit(sp, ro, dir, t) && t < tbest){ tbest = t; best = &sp; }
                }
                if(best == nullptr){
                    // sky gradient
                    double yy = 0.5*(dir.y + 1.0);
                    Vec3 sky = Vec3(0.6,0.7,0.85)*(1.0-yy) + Vec3(0.1,0.14,0.2)*yy;
                    acc[0]+=sky.x; acc[1]+=sky.y; acc[2]+=sky.z;
                } else {
                    Vec3 p = ro + dir*tbest;
                    Vec3 col;
                    if(best->diffuse){
                        col = best->color;
                    } else {
                        col = checker((p - best->c).norm(), 8.0);
                    }
                    // cheap lambert lighting from fixed key light
                    Vec3 n = (p - best->c).norm();
                    Vec3 L = Vec3(0.5,0.8,0.3).norm();
                    double ndl = std::max(0.0, n.dot(L));
                    col = col * (0.25 + 0.75*ndl);
                    acc[0]+=col.x; acc[1]+=col.y; acc[2]+=col.z;
                }
            }
            int idx = (py*W + px)*3;
            img[idx+0] = (unsigned char)(255.0 * std::min(1.0, acc[0]/spp));
            img[idx+1] = (unsigned char)(255.0 * std::min(1.0, acc[1]/spp));
            img[idx+2] = (unsigned char)(255.0 * std::min(1.0, acc[2]/spp));
        }
    }
}

// ---- Quantitative sharpness metric ----
// Local gradient energy over a region. High for sharp checker, low when blurred.
static double sharpness(const std::vector<unsigned char>& img, int W,
                        int x0, int y0, int x1, int y1){
    double e = 0; int n = 0;
    for(int y=y0; y<y1-1; ++y){
        for(int x=x0; x<x1-1; ++x){
            int i0 = (y*W+x)*3, i1=(y*W+x+1)*3, i2=((y+1)*W+x)*3;
            double g = 0;
            for(int c=0;c<3;++c){
                double dx = img[i1+c]-img[i0+c];
                double dy = img[i2+c]-img[i0+c];
                g += dx*dx + dy*dy;
            }
            e += std::sqrt(g/3.0);
            n++;
        }
    }
    return n? e/n : 0.0;
}

int main(){
    int W = 480, H = 480, spp = 400;
    // aperture sweep: 0 (pinhole) ... large
    std::vector<double> apertures = {0.0, 0.02, 0.04, 0.06, 0.08, 0.10, 0.14};

    // three depth bands (x ranges centered on each sphere)
    // near sphere at x ~ -2.5 -> window x in [-3.5,-1.5]
    // focal sphere x in [-1.0,1.0]
    // far sphere x in [1.5,3.5]
    struct Band { std::string name; int x0,x1; };
    std::vector<Band> bands = {
        {"near",  90, 150},   // in front of focal plane
        {"focal", 195, 285},  // at focal plane
        {"far",   330, 390},  // behind focal plane
    };
    int y0 = 190, y1 = 290;   // vertical band encloses sphere midlines

    printf("Aperture | ");
    for(auto&b:bands) printf("%-8s ", b.name.c_str());
    printf("\n");

    std::vector<std::vector<double>> table(apertures.size(),
                                           std::vector<double>(bands.size(), 0));

    for(size_t ai=0; ai<apertures.size(); ++ai){
        double ap = apertures[ai];
        std::vector<unsigned char> img;
        render(img, W, H, ap, spp, 12345 + ai*777);

        // save image for record
        char fn[64];
        snprintf(fn, sizeof(fn), "dof_ap_%.2f.png", ap);
        stbi_write_png(fn, W, H, 3, img.data(), W*3);

        printf("%8.2f | ", ap);
        for(size_t bi=0; bi<bands.size(); ++bi){
            double s = sharpness(img, W, bands[bi].x0, y0, bands[bi].x1, y1);
            table[ai][bi] = s;
            printf("%8.3f ", s);
        }
        printf("\n");
    }

    // ---- Automated verification ----
    printf("\n=== Quantitative Verification ===\n");

    // 1) Focal plane must stay sharp across all apertures (no blur even at large aperture)
    bool focalStable = true;
    for(size_t ai=1; ai<apertures.size(); ++ai){
        double ratio = table[ai][1] / table[0][1];
        if(ratio < 0.6) focalStable = false; // must retain >=60% sharpness vs pinhole
    }
    printf("[1] Focal plane stays sharp: %s\n", focalStable ? "PASS" : "FAIL");

    // 2) Near/far planes must blur as aperture increases (monotonic decrease)
    bool nearMonotonic = true, farMonotonic = true;
    for(size_t ai=1; ai<apertures.size(); ++ai){
        if(table[ai][0] > table[ai-1][0]) nearMonotonic = false;
        if(table[ai][2] > table[ai-1][2]) farMonotonic = false;
    }
    printf("[2] Near plane blurs with aperture: %s\n", nearMonotonic ? "PASS" : "FAIL");
    printf("[3] Far plane blurs with aperture: %s\n", farMonotonic ? "PASS" : "FAIL");

    // 3) At max aperture, off-focal planes must retain a SMALLER fraction of
    //    their pinhole sharpness than the focal plane does (relative blur).
    //    Circle-of-confusion grows with |z - focalDist|, so near & far blur more
    //    than the focal plane, as a RATIO of their own pinhole sharpness.
    double focalRetain = table.back()[1] / table[0][1];
    double nearRetain  = table.back()[0] / table[0][0];
    double farRetain   = table.back()[2] / table[0][2];
    bool depthDiscriminates = (nearRetain < focalRetain) && (farRetain < focalRetain);
    printf("[4] Off-focal blur ratio < focal blur ratio: %s\n",
           depthDiscriminates ? "PASS" : "FAIL");
    printf("     (focal retains %.1f%%, near %.1f%%, far %.1f%% of pinhole sharpness)\n",
           focalRetain*100, nearRetain*100, farRetain*100);

    bool allPass = focalStable && nearMonotonic && farMonotonic && depthDiscriminates;
    printf("\n%s\n", allPass ? "✅ ALL CHECKS PASSED" : "❌ SOME CHECKS FAILED");
    return allPass ? 0 : 1;
}
