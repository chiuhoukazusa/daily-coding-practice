// Anisotropic GGX Microfacet BRDF Renderer
// 2026-11-25 daily-coding-practice
// Renders a metallic sphere using Cook-Torrance with an anisotropic GGX
// normal distribution (two independent roughness axes along tangent/bitangent).
// Produces stretched specular highlights. Also renders an isotropic reference
// and prints quantitative metrics for verification.

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>

static const int W = 512, H = 512;

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator*(const Vec3& o) const { return Vec3(x*o.x, y*o.y, z*o.z); }
    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return Vec3(y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x);
    }
    double len() const { return std::sqrt(dot(*this)); }
    Vec3 norm() const { double l = len(); return (l > 1e-12) ? Vec3(x/l,y/l,z/l) : Vec3(); }
};

static inline Vec3 operator*(double s, const Vec3& v) { return v*s; }

static double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- Anisotropic GGX normal distribution ---- //
// D(h) using the two-axis roughness model (Ward / anisotropic GGX).
static double D_GGX_aniso(const Vec3& h, const Vec3& T, const Vec3& B, const Vec3& N,
                          double ax, double ay) {
    double hx = h.dot(T), hy = h.dot(B), hz = h.dot(N);
    if (hz <= 0.0) return 0.0;
    double ax2 = ax*ax, ay2 = ay*ay;
    double denom = hx*hx/(ax2) + hy*hy/(ay2) + hz*hz;
    double denom2 = denom*denom;
    return 1.0 / (M_PI * ax * ay * denom2);
}

static double D_GGX_iso(const Vec3& h, const Vec3& N, double a) {
    double hz = h.dot(N);
    if (hz <= 0.0) return 0.0;
    double a2 = a*a;
    double denom = hz*hz*(a2 - 1.0) + 1.0;
    denom = denom*denom;
    return a2 / (M_PI * denom);
}

static double G1_SmithGGX(double ndotv, double a) {
    double a2 = a*a;
    double ndv = clampd(ndotv, 0.0, 1.0);
    double t = ndv*ndv*(a2 - 1.0) + 1.0;
    return 2.0*ndv / (ndv + std::sqrt(t));
}

static double F_Schlick(double cosTheta, double f0) {
    return f0 + (1.0 - f0) * std::pow(clampd(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Multiply result by f0 for color (linear, gold-ish metal).
static Vec3 tonemap(double v) { // simple Reinhard + gamma for PPM display
    double t = v / (1.0 + v);
    return Vec3(std::pow(clampd(t,0.0,1.0), 1.0/2.2),
                std::pow(clampd(t,0.0,1.0), 1.0/2.2),
                std::pow(clampd(t,0.0,1.0), 1.0/2.2));
}

int main() {
    // Scene: single sphere centered at origin, radius 1.5, camera at +z.
    // Orthographic-ish perspective camera looking down -z.
    const double sphereR = 1.5;
    const Vec3 sphereC(0.0, 0.0, 0.0);
    const Vec3 lightDir = Vec3(0.4, 0.6, 0.75).norm(); // key light upper-right-front
    // Tangent/Bitangent: choose T along +x, B along +y (in a plane perpendicular to N,
    // we'll project). Similar across sphere -> "brushed metal" look.
    const double f0 = 0.9; // gold F0 ~0.9 at normal incidence

    // Configurations: (anisotropic | isotropic) and their roughness
    // (Config struct not needed; renders inline below)
    // We'll render two images: anisotropic (stretched) and isotropic reference.

    auto render = [&](double ax, double ay, bool iso, const std::string& tag) {
        std::vector<unsigned char> img(W*H*3, 0);
        // camera
        Vec3 camPos(0,0,5);
        // We raymarch sphere analytically.
        double maxInt=0;
        for (int y=0; y<H; ++y) {
            for (int x=0; x<W; ++x) {
                double px = (x - W/2.0) / (W/2.0);
                double py = (H/2.0 - y) / (H/2.0);
                Vec3 rd = Vec3(px*1.3, py*1.3, -1.0).norm(); // through near plane
                Vec3 ro = camPos;
                // ray-sphere
                Vec3 oc = ro - sphereC;
                double b = oc.dot(rd);
                double c = oc.dot(oc) - sphereR*sphereR;
                double disc = b*b - c;
                Vec3 col(0,0,0);
                if (disc >= 0) {
                    double t = -b - std::sqrt(disc);
                    if (t > 0) {
                        Vec3 p = ro + rd*t;
                        Vec3 n = (p - sphereC).norm();
                        // build tangent frame
                        Vec3 up = std::fabs(n.y) < 0.999 ? Vec3(0,1,0) : Vec3(1,0,0);
                        Vec3 T = up.cross(n).norm();
                        Vec3 Bg = n.cross(T).norm();
                        Vec3 v = (ro - p).norm();
                        Vec3 l = lightDir;
                        Vec3 h = (v + l).norm();
                        double ndotv = n.dot(v);
                        double ndotl = n.dot(l);
                        double hdotv = h.dot(v);
                        // lighting = D*G*F / (4 n.v n.l)  (Cook-Torrance denominator)
                        double D, G, F;
                        double alpha_eff;
                        if (iso) {
                            D = D_GGX_iso(h, n, ax);
                            alpha_eff = ax;
                        } else {
                            D = D_GGX_aniso(h, T, Bg, n, ax, ay);
                            alpha_eff = std::sqrt(ax*ay);
                        }
                        G = G1_SmithGGX(ndotv, alpha_eff) * G1_SmithGGX(ndotl, alpha_eff);
                        F = F_Schlick(hdotv, f0);
                        double denom = 4.0 * std::max(ndotv,1e-3) * std::max(ndotl,1e-3);
                        double spec = D*G*F / std::max(denom, 1e-12);
                        // diffuse term (small, gives base visibility)
                        double diff = 0.05 * std::max(ndotl, 0.0) / M_PI;
                        double L = spec + diff;
                        if (L > maxInt) maxInt = L;
                        Vec3 c = tonemap(L) * 255.0;
                        img[(y*W + x)*3 + 0] = (unsigned char)clampd(c.x,0,255);
                        img[(y*W + x)*3 + 1] = (unsigned char)clampd(c.y,0,255);
                        img[(y*W + x)*3 + 2] = (unsigned char)clampd(c.z,0,255);
                    }
                }
            }
        }
        // write PPM
        std::string fn = "aniso_" + tag + ".ppm";
        FILE* f = fopen(fn.c_str(), "wb");
        fprintf(f, "P6\n%d %d\n255\n", W, H);
        fwrite(img.data(), 1, W*H*3, f);
        fclose(f);
        // metrics
        // mean/std of grayscale
        double sum=0, sum2=0; long cnt=0;
        for (int i=0;i<W*H;i++){ double g=(img[i*3]+img[i*3+1]+img[i*3+2])/3.0; sum+=g; sum2+=g*g; cnt++; }
        double mean=sum/cnt, var=sum2/cnt-mean*mean, sd=std::sqrt(var);
        // highlight centroid + ellipse from bright pixels (>200)
        double sx=0,sy=0; long nBright=0;
        for (int y=0;y<H;y++) for(int x=0;x<W;x++){
            int i=(y*W+x)*3; double g=(img[i]+img[i+1]+img[i+2])/3.0;
            if (g>200){ sx+=x; sy+=y; nBright++; }
        }
        printf("[%s] mean=%.2f std=%.2f max=%.4f brightPixels=%ld\n", tag.c_str(), mean, sd, maxInt, nBright);
        if (nBright>0){
            double cx=sx/nBright, cy=sy/nBright;
            // second moments
            double mxx=0,myy=0,mxy=0;
            for (int y=0;y<H;y++) for(int x=0;x<W;x++){
                int i=(y*W+x)*3; double g=(img[i]+img[i+1]+img[i+2])/3.0;
                if (g>200){ double dx=x-cx,dy=y-cy; mxx+=dx*dx; myy+=dy*dy; mxy+=dx*dy; }
            }
            mxx/=nBright; myy/=nBright; mxy/=nBright;
            double tr=mxx+myy; double det=mxx*myy-mxy*mxy;
            double rad=std::sqrt(std::max(tr*tr/4 - det,0.0));
            double l1=tr/2+rad, l2=tr/2-rad;
            double aspect = std::sqrt(std::max(l1,1e-9)/std::max(l2,1e-9));
            printf("  highlight centroid=(%.1f,%.1f) eig=(%.2f,%.2f) aspectRatio=%.3f\n",
                   cx, cy, l1, l2, aspect);
        }
        // also compute highlight bounding box
        double minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
        for (int y=0;y<H;y++) for(int x=0;x<W;x++){
            int i=(y*W+x)*3; double g=(img[i]+img[i+1]+img[i+2])/3.0;
            if (g>200){ minx=std::min(minx,(double)x); maxx=std::max(maxx,(double)x);
                        miny=std::min(miny,(double)y); maxy=std::max(maxy,(double)y); }
        }
        if (nBright>0) printf("  bbox=[%.0f..%.0f]x[%.0f..%.0f] w=%d h=%d\n",
                              minx,maxx,miny,maxy,(int)(maxx-minx),(int)(maxy-miny));
        return fn;
    };

    render(0.3, 0.05, false, "anisotropic"); // stretched along bitangent (vertical)
    render(0.15, 0.15, true, "isotropic");   // round highlight reference
    return 0;
}
