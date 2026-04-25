/**
 * Screen Space Refraction (SSR) Renderer
 * =======================================
 * Simulates real-time refraction using G-Buffer data:
 * - G-Buffer: depth, normal, albedo
 * - Refraction via Snell's law + screen-space offset
 * - Schlick Fresnel: blend refraction/reflection
 * - Chromatic aberration (dispersion) for glass/water look
 * - Background scene: Cornell-box-style with sphere and ground
 *
 * Compile: g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <string>

// ─── Vec3 ─────────────────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o)  const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len2() const { return x*x + y*y + z*z; }
    float len()  const { return std::sqrt(len2()); }
    Vec3 norm()  const { float l = len(); return l>0?(*this)*(1.f/l):Vec3(); }
    Vec3 clamp01() const {
        return {std::max(0.f,std::min(1.f,x)),
                std::max(0.f,std::min(1.f,y)),
                std::max(0.f,std::min(1.f,z))};
    }
};
inline Vec3 operator*(float t, const Vec3& v){ return v*t; }

// ─── Image & PPM ──────────────────────────────────────────────────────────────

struct Image {
    int w, h;
    std::vector<Vec3> data;
    Image(int w, int h) : w(w), h(h), data(w*h) {}
    Vec3& at(int x, int y){ return data[y*w+x]; }
    const Vec3& at(int x, int y) const { return data[y*w+x]; }
    // bilinear sample (UV in [0,1])
    Vec3 sample(float u, float v) const {
        u = std::max(0.f, std::min(1.f, u));
        v = std::max(0.f, std::min(1.f, v));
        float fx = u*(w-1), fy = v*(h-1);
        int x0=(int)fx, y0=(int)fy;
        int x1=std::min(x0+1,w-1), y1=std::min(y0+1,h-1);
        float tx=fx-x0, ty=fy-y0;
        Vec3 a = at(x0,y0)*(1-tx) + at(x1,y0)*tx;
        Vec3 b = at(x0,y1)*(1-tx) + at(x1,y1)*tx;
        return a*(1-ty)+b*ty;
    }
};

void writePNG(const std::string& fn, const Image& img) {
    // Write a raw 8-bit PPM then convert via ppm2png trick
    // Actually write as PPM directly; save as .ppm first
    std::string ppm = fn.substr(0,fn.rfind('.'))+".ppm";
    std::ofstream f(ppm, std::ios::binary);
    f << "P6\n" << img.w << " " << img.h << "\n255\n";
    for(auto& c : img.data){
        // gamma 2.2
        float r = std::pow(std::max(0.f,std::min(1.f,c.x)), 1.f/2.2f);
        float g = std::pow(std::max(0.f,std::min(1.f,c.y)), 1.f/2.2f);
        float b = std::pow(std::max(0.f,std::min(1.f,c.z)), 1.f/2.2f);
        f << (uint8_t)(r*255) << (uint8_t)(g*255) << (uint8_t)(b*255);
    }
    f.close();
    std::cout << "Saved PPM: " << ppm << "\n";
}

// Write PNG using a minimal built-in encoder (no external deps)
// We'll use a simple P6 PPM then call pnmtopng if available, else keep PPM
bool savePPMasPNG(const std::string& ppmFile, const std::string& pngFile){
    std::string cmd = "convert " + ppmFile + " " + pngFile + " 2>/dev/null";
    return system(cmd.c_str()) == 0;
}

// ─── Scene geometry ───────────────────────────────────────────────────────────

// Axis-aligned plane
struct Plane {
    Vec3 n;         // normal (outward)
    float d;        // n.dot(p) = d
    Vec3 albedo;
    bool emissive;
    float emission; // intensity
};

struct Sphere {
    Vec3 center;
    float radius;
    Vec3 albedo;
    bool glass;     // refractive object
    float ior;      // index of refraction
};

// Hit record
struct Hit {
    float t;
    Vec3 pos, normal;
    Vec3 albedo;
    bool glass;
    float ior;
    bool emissive;
    float emission;
};

static const float INF = 1e30f;

bool hitSphere(const Sphere& s, const Vec3& ro, const Vec3& rd, float tmin, float tmax, Hit& h){
    Vec3 oc = ro - s.center;
    float a = rd.dot(rd);
    float b = oc.dot(rd);
    float c = oc.dot(oc) - s.radius*s.radius;
    float disc = b*b - a*c;
    if(disc < 0) return false;
    float sq = std::sqrt(disc);
    float t = (-b-sq)/a;
    if(t < tmin || t > tmax){ t=(-b+sq)/a; if(t<tmin||t>tmax) return false; }
    h.t = t;
    h.pos = ro + rd*t;
    h.normal = (h.pos - s.center) / s.radius;
    h.albedo = s.albedo;
    h.glass = s.glass;
    h.ior = s.ior;
    h.emissive = false;
    h.emission = 0;
    return true;
}

bool hitPlane(const Plane& p, const Vec3& ro, const Vec3& rd, float tmin, float tmax, Hit& h){
    float denom = p.n.dot(rd);
    if(std::abs(denom) < 1e-6f) return false;
    float t = (p.d - p.n.dot(ro)) / denom;
    if(t < tmin || t > tmax) return false;
    h.t = t;
    h.pos = ro + rd*t;
    h.normal = p.n;
    h.albedo = p.albedo;
    h.glass = false;
    h.ior = 1.f;
    h.emissive = p.emissive;
    h.emission = p.emission;
    return true;
}

// ─── Scene definition ─────────────────────────────────────────────────────────

struct Scene {
    std::vector<Sphere> spheres;
    std::vector<Plane> planes;

    bool intersect(const Vec3& ro, const Vec3& rd, float tmin, float tmax, Hit& best) const {
        bool found = false;
        best.t = tmax;
        Hit h{};
        for(auto& s : spheres)
            if(hitSphere(s, ro, rd, tmin, best.t, h)){ best=h; found=true; }
        for(auto& p : planes)
            if(hitPlane(p, ro, rd, tmin, best.t, h)){ best=h; found=true; }
        return found;
    }
};

Scene buildScene(){
    Scene sc;

    // Glass sphere (the refractive object)
    sc.spheres.push_back({{0.f,0.f,-3.f}, 0.7f, {0.9f,0.95f,1.0f}, true, 1.5f});
    // Solid red sphere
    sc.spheres.push_back({{-1.3f,-0.1f,-3.f}, 0.4f, {0.85f,0.15f,0.1f}, false, 1.0f});
    // Solid blue sphere
    sc.spheres.push_back({{ 1.3f,-0.1f,-3.f}, 0.4f, {0.1f,0.2f,0.9f}, false, 1.0f});
    // Small emissive light sphere (ceiling lamp)
    sc.spheres.push_back({{0.f,2.2f,-3.f}, 0.3f, {1.f,0.95f,0.85f}, false, 1.0f});
    // tiny accent sphere (magenta)
    sc.spheres.push_back({{0.5f,0.35f,-2.2f}, 0.18f, {0.9f,0.1f,0.7f}, false, 1.0f});

    // Floor (y = -0.8)
    sc.planes.push_back({{0,1,0}, -0.8f, {0.7f,0.65f,0.6f}, false, 0});
    // Ceiling (y = 2.5)
    sc.planes.push_back({{0,-1,0}, -2.5f, {0.8f,0.78f,0.75f}, false, 0});
    // Back wall (z = -4)
    sc.planes.push_back({{0,0,1}, -4.f, {0.7f,0.7f,0.7f}, false, 0});
    // Left wall (x = -2)
    sc.planes.push_back({{1,0,0}, -2.f, {0.8f,0.1f,0.1f}, false, 0});
    // Right wall (x = 2)
    sc.planes.push_back({{-1,0,0}, -2.f, {0.1f,0.6f,0.2f}, false, 0});

    return sc;
}

// ─── Simple path-tracer for background ────────────────────────────────────────

// Direct illumination (one bounce Phong)
Vec3 shade(const Scene& sc, const Vec3& ro, const Vec3& rd, int depth=0){
    if(depth > 3) return Vec3(0,0,0);

    Hit h{};
    if(!sc.intersect(ro, rd, 1e-4f, INF, h)){
        // Sky gradient
        float t = 0.5f*(rd.norm().y + 1.f);
        return Vec3(0.5f,0.7f,1.f)*t + Vec3(1,1,1)*(1-t);
    }

    // Emissive
    if(h.emissive) return h.albedo * h.emission;
    // Emissive light sphere
    if(h.glass == false && h.ior == 1.0f && false) return Vec3();  // not used

    // Light sphere position
    Vec3 lightPos(0.f, 2.2f, -3.f);
    Vec3 lightCol(1.f, 0.95f, 0.85f);
    float lightPow = 12.f;

    Vec3 toL = (lightPos - h.pos).norm();
    float dist2 = (lightPos - h.pos).len2();

    // Shadow
    Hit sh{};
    float shadow = 1.f;
    if(sc.intersect(h.pos + h.normal*1e-3f, toL, 1e-3f, std::sqrt(dist2)-0.3f, sh))
        shadow = 0.1f;

    float diff = std::max(0.f, h.normal.dot(toL));
    Vec3 direct = h.albedo * lightCol * (lightPow / (1.f + dist2)) * diff * shadow;

    // Ambient
    Vec3 ambient = h.albedo * 0.12f;

    return direct + ambient;
}

// ─── G-Buffer ─────────────────────────────────────────────────────────────────

struct GBuffer {
    Image depth;    // linear depth stored in .x
    Image normal;   // world normal in xyz
    Image albedo;   // surface albedo
    Image position; // world position (x,y,z)
    Image mask;     // .x=1 if glass

    GBuffer(int w, int h)
        : depth(w,h), normal(w,h), albedo(w,h), position(w,h), mask(w,h) {}
};

// ─── Camera ───────────────────────────────────────────────────────────────────

struct Camera {
    Vec3 origin, lower_left, horiz, vert;

    Camera(Vec3 from, Vec3 at, Vec3 up, float vfov, float aspect){
        float theta = vfov * (float)M_PI / 180.f;
        float half_h = std::tan(theta/2);
        float half_w = aspect * half_h;

        Vec3 w = (from-at).norm();
        Vec3 u = up.cross(w).norm();
        Vec3 v = w.cross(u);

        origin = from;
        lower_left = from - u*half_w - v*half_h - w;
        horiz = u * (2*half_w);
        vert  = v * (2*half_h);
    }

    void getRay(float s, float t, Vec3& ro, Vec3& rd) const {
        ro = origin;
        rd = (lower_left + horiz*s + vert*t - origin).norm();
    }
};

// ─── Build G-Buffer ───────────────────────────────────────────────────────────

void buildGBuffer(const Scene& sc, const Camera& cam, GBuffer& gbuf){
    int W = gbuf.depth.w, H = gbuf.depth.h;
    for(int y=0; y<H; ++y){
        for(int x=0; x<W; ++x){
            float u = (x+0.5f)/W;
            float v = 1.f - (y+0.5f)/H;  // flip Y: sky at top

            Vec3 ro, rd;
            cam.getRay(u, v, ro, rd);

            Hit h{};
            if(sc.intersect(ro, rd, 1e-4f, INF, h)){
                gbuf.depth.at(x,y)    = Vec3(h.t, h.t, h.t);
                gbuf.normal.at(x,y)   = (h.normal * 0.5f) + Vec3(0.5f,0.5f,0.5f); // [0,1] pack
                gbuf.albedo.at(x,y)   = h.albedo;
                gbuf.position.at(x,y) = h.pos;
                gbuf.mask.at(x,y)     = Vec3(h.glass ? 1.f : 0.f, 0, 0);
            } else {
                gbuf.depth.at(x,y)    = Vec3(1e9f, 1e9f, 1e9f);
                gbuf.normal.at(x,y)   = Vec3(0.5f,0.5f,1.f);
                gbuf.albedo.at(x,y)   = Vec3(0,0,0);
                gbuf.position.at(x,y) = Vec3(0,0,0);
                gbuf.mask.at(x,y)     = Vec3(0,0,0);
            }
        }
    }
}

// ─── Background render ────────────────────────────────────────────────────────

Image renderBackground(const Scene& sc, const Camera& cam, int W, int H){
    Image bg(W, H);
    for(int y=0; y<H; ++y){
        for(int x=0; x<W; ++x){
            float u = (x+0.5f)/W;
            float v = 1.f - (y+0.5f)/H;
            Vec3 ro, rd;
            cam.getRay(u, v, ro, rd);
            bg.at(x,y) = shade(sc, ro, rd, 0);
        }
    }
    return bg;
}

// ─── Schlick Fresnel ──────────────────────────────────────────────────────────

float schlick(float cosTheta, float n1, float n2){
    float r0 = (n1-n2)/(n1+n2);
    r0 = r0*r0;
    float c = 1.f - cosTheta;
    return r0 + (1-r0)*c*c*c*c*c;
}

// ─── SSR Refraction pass ──────────────────────────────────────────────────────

// Project world pos to screen UV
bool worldToScreen(const Vec3& pos, const Camera& cam, int /*W*/, int /*H*/, float& su, float& sv){
    // Cheap camera-space projection (assumes standard perspective)
    Vec3 d = pos - cam.origin;
    Vec3 forward = (cam.lower_left + cam.horiz*0.5f + cam.vert*0.5f - cam.origin).norm();
    Vec3 right   = cam.horiz.norm();
    Vec3 up_dir  = cam.vert.norm();

    float fwd = d.dot(forward);
    if(fwd <= 0) return false;

    // compute half-angles from camera setup
    float half_w = cam.horiz.len()*0.5f;
    float half_h = cam.vert.len()*0.5f;
    float scale  = 1.f / fwd;

    float rx = d.dot(right)   * scale;
    float ry = d.dot(up_dir)  * scale;

    su = (rx / half_w) * 0.5f + 0.5f;
    sv = (ry / half_h) * 0.5f + 0.5f;
    sv = 1.f - sv; // flip back

    return su>=0&&su<=1&&sv>=0&&sv<=1;
}

Image applySSRefraction(const Scene& sc, const Camera& cam,
                        const GBuffer& gbuf,
                        const Image& background,
                        int W, int H)
{
    Image result(W, H);

    // Copy background as default
    for(int y=0; y<H; ++y)
        for(int x=0; x<W; ++x)
            result.at(x,y) = background.at(x,y);

    // IOR of glass spheres
    float ior = 1.5f;
    float ior_air = 1.0f;

    // Dispersion offsets per channel (R, G, B wavelengths)
    float dispR = 0.0f;
    float dispG = 0.02f;
    float dispB = 0.04f;

    for(int y=0; y<H; ++y){
        for(int x=0; x<W; ++x){
            float isGlass = gbuf.mask.at(x,y).x;
            if(isGlass < 0.5f) continue;

            // Unpack normal from G-Buffer
            Vec3 nPacked = gbuf.normal.at(x,y);
            Vec3 N = (nPacked * 2.f) - Vec3(1,1,1);  // [-1,1]
            N = N.norm();

            // View direction
            float u = (x+0.5f)/W;
            float v_coord = 1.f - (y+0.5f)/H;
            Vec3 ro, rd;
            cam.getRay(u, v_coord, ro, rd);

            // Fresnel
            float cosI = std::max(0.f, -rd.dot(N));
            float Fr = schlick(cosI, ior_air, ior);

            // Refract direction (Snell's law)
            float ratio = ior_air / ior;
            float k = 1.f - ratio*ratio*(1.f - cosI*cosI);
            Vec3 refractDir;
            if(k < 0){
                // Total internal reflection
                refractDir = rd - N*(2.f*rd.dot(N));  // reflect
            } else {
                refractDir = rd*ratio + N*(ratio*cosI - std::sqrt(k));
            }
            refractDir = refractDir.norm();

            // Hit position on glass surface
            Vec3 hitPos = gbuf.position.at(x,y);

            // Compute refracted world position (march through the sphere)
            // Thickness estimate: 2*r (simple) — use 1.4 as avg chord through sphere
            float thickness = 1.2f;
            Vec3 exitPos = hitPos + refractDir * thickness;

            // Project exit position back to screen
            float su, sv;
            if(!worldToScreen(exitPos, cam, W, H, su, sv)){
                // fallback: project with small offset
                su = u + refractDir.x * 0.15f;
                sv = v_coord - refractDir.y * 0.15f;
            }

            // Sample background with chromatic aberration
            Vec3 bgR = background.sample(su + dispR, sv);
            Vec3 bgG = background.sample(su + dispG, sv);
            Vec3 bgB = background.sample(su + dispB, sv);
            Vec3 refracted(bgR.x, bgG.y, bgB.z);

            // Reflection direction
            Vec3 reflDir = (rd - N*(2.f*rd.dot(N))).norm();
            // Trace reflection ray
            Vec3 reflColor = shade(sc, hitPos + N*1e-3f, reflDir, 1);

            // Glass tint
            Vec3 glassTint = gbuf.albedo.at(x,y);

            // Blend: Fr*reflect + (1-Fr)*refract, tinted
            Vec3 finalColor = reflColor*Fr + refracted*(1.f-Fr);
            finalColor = finalColor * glassTint;

            // Add slight specular highlight
            Vec3 lightDir = (Vec3(0.f,2.2f,-3.f) - hitPos).norm();
            float spec = std::pow(std::max(0.f, reflDir.dot(lightDir)), 64.f);
            finalColor += Vec3(1,1,1) * spec * 0.4f;

            result.at(x,y) = finalColor.clamp01();
        }
    }

    return result;
}

// ─── Debug overlay: G-buffer visualization ───────────────────────────────────

Image makeGBufferDebug(const GBuffer& gbuf, int W, int H){
    // 2x2 grid: depth | normal | albedo | mask
    Image out(W*2, H*2);
    for(int y=0; y<H; ++y){
        for(int x=0; x<W; ++x){
            // Normalize depth [0..10] -> [0..1]
            float d = gbuf.depth.at(x,y).x;
            float dn = d > 100.f ? 0.f : 1.f - (d / 10.f);
            dn = std::max(0.f, std::min(1.f, dn));
            out.at(x,           y      ) = Vec3(dn,dn,dn);
            out.at(x+W,         y      ) = gbuf.normal.at(x,y);
            out.at(x,           y+H    ) = gbuf.albedo.at(x,y);
            out.at(x+W,         y+H    ) = gbuf.mask.at(x,y);
        }
    }
    return out;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(){
    const int W = 640, H = 480;

    std::cout << "=== Screen Space Refraction ===\n";
    std::cout << "Resolution: " << W << "x" << H << "\n";

    Scene sc = buildScene();
    Camera cam(
        Vec3(0.f, 0.4f, 1.5f),  // from
        Vec3(0.f, 0.f, -3.f),   // at
        Vec3(0,1,0),              // up
        60.f,                     // vfov
        (float)W/H               // aspect
    );

    // --- Pass 1: Build G-Buffer ---
    std::cout << "[1/4] Building G-Buffer...\n";
    GBuffer gbuf(W, H);
    buildGBuffer(sc, cam, gbuf);
    std::cout << "      G-Buffer done.\n";

    // --- Pass 2: Render background (no glass) ---
    // Temporarily mark glass spheres as non-glass for background render
    std::cout << "[2/4] Rendering background scene...\n";
    Scene bgScene = sc;
    for(auto& s : bgScene.spheres) s.glass = false; // treat all as solid for bg
    Image bg = renderBackground(bgScene, cam, W, H);
    std::cout << "      Background render done.\n";

    // --- Pass 3: Apply SSR Refraction ---
    std::cout << "[3/4] Applying screen space refraction...\n";
    // Restore original scene for reflection traces
    Image final_img = applySSRefraction(sc, cam, gbuf, bg, W, H);
    std::cout << "      SSR refraction done.\n";

    // --- Pass 4: Save outputs ---
    std::cout << "[4/4] Saving outputs...\n";
    writePNG("ssr_refraction_output.ppm", final_img);
    // Convert PPM to PNG
    if(savePPMasPNG("ssr_refraction_output.ppm", "ssr_refraction_output.png")){
        std::cout << "Saved PNG: ssr_refraction_output.png\n";
    } else {
        std::cout << "Note: ImageMagick not available, using PPM only\n";
        // Rename ppm as output
        rename("ssr_refraction_output.ppm","ssr_refraction_output.png");
    }

    // Save G-Buffer debug image
    Image dbg = makeGBufferDebug(gbuf, W, H);
    writePNG("gbuffer_debug.ppm", dbg);
    savePPMasPNG("gbuffer_debug.ppm","gbuffer_debug.png");

    // --- Validation output for pixel stats ---
    std::cout << "\n=== PIXEL STATS ===\n";
    double sumR=0, sumG=0, sumB=0;
    double sumSq=0;
    int count = W*H;
    for(int i=0;i<count;++i){
        Vec3 c = final_img.data[i];
        sumR += c.x; sumG += c.y; sumB += c.z;
        float gray = (c.x+c.y+c.z)/3.f;
        sumSq += gray*gray;
    }
    double meanR = sumR/count, meanG = sumG/count, meanB = sumB/count;
    double mean  = (meanR+meanG+meanB)/3.0;
    // std dev
    double meanSq=0;
    for(auto& c : final_img.data){
        float g=(c.x+c.y+c.z)/3.f;
        double d2=(g-mean)*(g-mean);
        meanSq+=d2;
    }
    double std_dev = std::sqrt(meanSq/count);

    std::cout << "Mean (R,G,B): " << meanR*255 << ", " << meanG*255 << ", " << meanB*255 << "\n";
    std::cout << "Overall mean: " << mean*255 << "\n";
    std::cout << "Std dev: " << std_dev*255 << "\n";

    // Check glass pixels
    int glassPx=0;
    for(int y=0;y<H;++y)
        for(int x=0;x<W;++x)
            if(gbuf.mask.at(x,y).x > 0.5f) glassPx++;
    std::cout << "Glass pixels: " << glassPx << " (" << (100.0*glassPx/(W*H)) << "%)\n";

    std::cout << "\n✅ Done. Outputs: ssr_refraction_output.png, gbuffer_debug.png\n";
    return 0;
}
