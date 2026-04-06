/*
 * Fur / Hair Rendering with Kajiya-Kay Model
 * Date: 2026-04-07
 *
 * Technique:
 *   - Kajiya-Kay (1989) hair shading model
 *     Diffuse:  kd * sin(θ_LT)  where θ_LT = angle between light and tangent
 *     Specular: ks * (cos(θ_TE))^p  where θ_TE = angle between eye and tangent
 *   - Procedural hair geometry (cylinder-mapped fiber curves on a sphere head)
 *   - Soft rasterizer + z-buffer, renders to PPM then converts to PNG
 *   - Simple directional + ambient lighting
 *
 * Output: fur_hair_output.png  (512×512)
 */

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include <string>
#include <sstream>

// ---------- Math types ----------

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x,y-o.y,z-o.z}; }
    Vec3 operator*(float s)       const { return {x*s,  y*s,  z*s};   }
    Vec3 operator*(const Vec3& o) const { return {x*o.x,y*o.y,z*o.z}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x;y+=o.y;z+=o.z;return *this; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    Vec3  cross(const Vec3& o) const {
        return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3  norm() const { float l=len(); return l>1e-6f?(*this)*(1.f/l):Vec3(0,0,0); }
    Vec3  clamp01() const {
        return { std::max(0.f,std::min(1.f,x)),
                 std::max(0.f,std::min(1.f,y)),
                 std::max(0.f,std::min(1.f,z)) };
    }
};

// ---------- Image ----------

struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    std::vector<float> zbuf;

    Image(int w,int h) : w(w), h(h), pixels(w*h,{0,0,0}),
                         zbuf(w*h, 1e9f) {}

    void setPixel(int x,int y,const Vec3& c,float z) {
        if(x<0||x>=w||y<0||y>=h) return;
        int idx = y*w+x;
        if(z < zbuf[idx]) { zbuf[idx]=z; pixels[idx]=c; }
    }

    bool savePPM(const std::string& path) const {
        std::ofstream f(path, std::ios::binary);
        if(!f) return false;
        f<<"P6\n"<<w<<" "<<h<<"\n255\n";
        for(auto& p : pixels) {
            // gamma correct (~2.2)
            float r = std::pow(std::max(0.f,std::min(1.f,p.x)), 1.f/2.2f);
            float g = std::pow(std::max(0.f,std::min(1.f,p.y)), 1.f/2.2f);
            float b = std::pow(std::max(0.f,std::min(1.f,p.z)), 1.f/2.2f);
            unsigned char cr = (unsigned char)(r*255.f+.5f);
            unsigned char cg = (unsigned char)(g*255.f+.5f);
            unsigned char cb = (unsigned char)(b*255.f+.5f);
            f<<cr<<cg<<cb;
        }
        return true;
    }
};

// ---------- Camera ----------

struct Camera {
    Vec3 origin, forward, right, up;
    float fovY, aspect, near;
    int W, H;

    Camera(Vec3 pos, Vec3 target, Vec3 upHint, float fovY, int W, int H)
        : origin(pos), fovY(fovY), aspect((float)W/H), near(0.1f), W(W), H(H)
    {
        forward = (target-pos).norm();
        right   = forward.cross(upHint).norm();
        up      = right.cross(forward);
    }

    // Project world point → screen pixel (x,y) + depth z (camera space)
    bool project(const Vec3& p, float& sx, float& sy, float& sz) const {
        Vec3 d = p - origin;
        float z = d.dot(forward);
        if(z < near) return false;
        float x = d.dot(right);
        float y = d.dot(up);
        float half_h = std::tan(fovY*0.5f) * z;
        float half_w = half_h * aspect;
        sx = ( x/half_w * 0.5f + 0.5f) * W;
        sy = (-y/half_h * 0.5f + 0.5f) * H;   // y flip
        sz = z;
        return true;
    }
};

// ---------- Kajiya-Kay shading ----------

// hair_tangent: unit tangent of hair strand
// light_dir:    unit vector toward light
// view_dir:     unit vector toward eye
// Returns (diffuse_factor, specular_factor)
inline float KK_diffuse(const Vec3& T, const Vec3& L) {
    float cosTheta = T.dot(L);
    float sinTheta = std::sqrt(std::max(0.f, 1.f - cosTheta*cosTheta));
    return sinTheta;                      // sin(angle L,T)
}

inline float KK_specular(const Vec3& T, const Vec3& V, float shininess) {
    float cosTheta_TL = T.dot(V);         // actually we want cos angle T-V
    float sinTheta = std::sqrt(std::max(0.f, 1.f - cosTheta_TL*cosTheta_TL));
    // shift specular lobe slightly along T
    float shifted = cosTheta_TL + 0.2f;
    float sinShifted = std::sqrt(std::max(0.f, 1.f - shifted*shifted));
    (void)sinTheta;
    return std::pow(std::max(0.f, sinShifted), shininess);
}

// ---------- Draw a hair strand as a thick line ----------

void drawStrand(Image& img,
                const Camera& cam,
                const std::vector<Vec3>& pts,          // world-space control points
                const Vec3& lightDir,                  // unit toward light
                const Vec3& viewDir,                   // unit toward eye
                const Vec3& hairColor,
                float thickness)    // pixel radius
{
    if(pts.size() < 2) return;
    const Vec3  ambient  = Vec3(0.10f,0.08f,0.05f);
    const float kd = 0.7f, ks = 0.4f, shininess = 60.f;

    for(size_t i = 0; i+1 < pts.size(); ++i) {
        const Vec3& A = pts[i];
        const Vec3& B = pts[i+1];

        // Tangent of this segment
        Vec3 tangent = (B-A).norm();

        // Kajiya-Kay factors
        float diff = KK_diffuse(tangent, lightDir);
        float spec = KK_specular(tangent, viewDir, shininess);

        Vec3 shadedColor = (ambient + hairColor * (kd * diff) + Vec3(1,1,0.8f) * (ks * spec)).clamp01();

        // Project endpoints
        float ax=0,ay=0,az=0, bx=0,by=0,bz=0;
        bool okA = cam.project(A, ax, ay, az);
        bool okB = cam.project(B, bx, by, bz);
        if(!okA && !okB) continue;
        // If only one endpoint projected, use the projected one for both
        if(!okA){ ax=bx; ay=by; az=bz; }
        if(!okB){ bx=ax; by=ay; bz=az; }

        // Rasterise line segment with pixel-width thickness
        int steps = (int)std::max(std::abs(bx-ax), std::abs(by-ay)) + 2;
        int r = (int)std::ceil(thickness);
        for(int s = 0; s <= steps; ++s) {
            float t = (float)s / steps;
            float px = ax + (bx-ax)*t;
            float py = ay + (by-ay)*t;
            float pz = az + (bz-az)*t;
            // Draw a small disc of radius r
            for(int dy=-r; dy<=r; ++dy)
            for(int dx=-r; dx<=r; ++dx) {
                if(dx*dx+dy*dy <= r*r) {
                    img.setPixel((int)(px+dx+.5f),(int)(py+dy+.5f), shadedColor, pz);
                }
            }
        }
    }
}

// ---------- Procedural head sphere (background fill) ----------

void renderBackground(Image& img) {
    // Simple gradient background: warm amber top → dark brown bottom
    for(int y=0;y<img.h;++y) {
        float t = (float)y/img.h;
        Vec3 top(0.12f,0.08f,0.05f);
        Vec3 bot(0.04f,0.02f,0.01f);
        Vec3 col = top*(1.f-t) + bot*t;
        for(int x=0;x<img.w;++x) {
            img.pixels[y*img.w+x] = col;
            img.zbuf [y*img.w+x]  = 1e9f;
        }
    }
}

// Rasterise a shaded sphere (head) to give context
void renderSphere(Image& img, const Camera& cam,
                  Vec3 center, float radius, Vec3 color,
                  Vec3 lightDir)
{
    // For each pixel, ray-sphere test
    float W = img.w, H = img.h;
    Vec3 eye = cam.origin;
    float half_h = std::tan(cam.fovY*0.5f);
    float half_w = half_h * cam.aspect;
    for(int y=0;y<img.h;++y)
    for(int x=0;x<img.w;++x) {
        float u = ((float)x+.5f)/W * 2.f - 1.f;
        float v = 1.f - ((float)y+.5f)/H * 2.f;
        Vec3 dir = (cam.forward + cam.right*(u*half_w) + cam.up*(v*half_h)).norm();
        // Sphere intersection
        Vec3 oc = eye - center;
        float a = 1.f;
        float b = 2.f*oc.dot(dir);
        float c = oc.dot(oc) - radius*radius;
        float disc = b*b - 4*a*c;
        if(disc < 0) continue;
        float t = (-b - std::sqrt(disc))/(2*a);
        if(t < cam.near) continue;
        Vec3 hit = eye + dir*t;
        Vec3 N   = (hit-center).norm();
        float diff = std::max(0.f, N.dot(lightDir));
        float spec = 0.f;
        {
            Vec3 R = lightDir - N*(2*N.dot(lightDir));
            float s = std::max(0.f,-R.dot(dir));
            spec = std::pow(s,32.f)*0.3f;
        }
        Vec3 ambient(0.12f,0.08f,0.05f);
        Vec3 shaded = (ambient + color*(0.7f*diff) + Vec3(0.9f,0.9f,0.8f)*spec).clamp01();
        img.setPixel(x,y,shaded,t);
    }
}

// ---------- Generate hair on sphere surface ----------

// Returns a set of hair strands growing from the sphere surface.
// Each strand is a sequence of world-space positions.
std::vector<std::vector<Vec3>> generateHair(
        Vec3 center, float headRadius,
        int numHairs,
        float hairLength,
        unsigned int seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u01(0.f,1.f);

    std::vector<std::vector<Vec3>> strands;
    strands.reserve(numHairs);

    for(int i=0;i<numHairs;++i) {
        // Fibonacci sphere distribution for even spacing (+ random jitter)
        float phi   = 2.f*3.14159265f * (float)i * 0.618033988f;
        float theta = std::acos(1.f - 2.f*(i+0.5f)/numHairs);
        // Random jitter
        phi   += (u01(rng)-0.5f)*0.4f;
        theta += (u01(rng)-0.5f)*0.2f;

        // Cover mostly upper hemisphere (hair on head, not chin underside)
        // Skip bottom 30%
        if(theta > 3.14159f * 0.8f) continue;

        Vec3 root = center + Vec3(
            std::sin(theta)*std::cos(phi),
            std::cos(theta),
            std::sin(theta)*std::sin(phi)
        ) * headRadius;

        Vec3 normal = (root - center).norm();

        // Wind direction: slight droop toward –Y (gravity) + side curl
        Vec3 windDir = Vec3(0.1f,-1.f,0.05f).norm();

        // Build strand with 8 segments
        const int SEG = 8;
        float segLen = hairLength / SEG;

        std::vector<Vec3> strand;
        strand.reserve(SEG+1);
        strand.push_back(root);

        Vec3 dir = normal;   // starts perpendicular to sphere
        for(int s=0;s<SEG;++s) {
            float blend = (float)s/SEG;  // 0→tip: more drooping
            // Blend from normal to windDir
            Vec3 d = (normal*(1.f-blend) + windDir*blend).norm();
            // Random curl
            Vec3 rndPerp = Vec3(u01(rng)-0.5f, u01(rng)-0.5f, u01(rng)-0.5f);
            rndPerp = rndPerp - normal*(rndPerp.dot(normal));  // project out normal
            d = (d + rndPerp * 0.15f).norm();
            dir = d;
            Vec3 prev = strand.back();
            strand.push_back(prev + dir * segLen);
        }
        strands.push_back(strand);
    }
    return strands;
}

// ---------- Main ----------

int main() {
    const int W = 512, H = 512;
    Image img(W, H);

    // Camera
    Vec3 camPos(0.f, 0.3f, 3.2f);
    Vec3 target(0.f, 0.1f, 0.f);
    Camera cam(camPos, target, Vec3(0,1,0), 0.45f, W, H);
    Vec3 viewDir = (camPos - target).norm();

    // Light
    Vec3 lightDir = Vec3(1.f, 1.5f, 1.f).norm();

    // Background
    renderBackground(img);

    // Head sphere
    Vec3 headCenter(0.f, 0.f, 0.f);
    float headRadius = 0.9f;
    Vec3 skinColor(0.82f, 0.62f, 0.48f);
    renderSphere(img, cam, headCenter, headRadius, skinColor, lightDir);

    // Generate hair
    int numHairs = 6000;
    float hairLength = 0.55f;
    auto strands = generateHair(headCenter, headRadius, numHairs, hairLength, 42u);

    // Sort strands back-to-front (painter's z for the line drawing)
    // Using root z-depth
    std::sort(strands.begin(), strands.end(), [&](const auto& a, const auto& b){
        float za=1e9f,zb=1e9f,dummy1=0,dummy2=0;
        if(!a.empty()) cam.project(a[0],dummy1,dummy2,za);
        if(!b.empty()) cam.project(b[0],dummy1,dummy2,zb);
        return za > zb;  // farther first
    });

    // Hair base color: warm golden-brown
    Vec3 hairColor(0.58f, 0.35f, 0.15f);
    float thickness = 0.7f;  // pixel radius

    for(auto& strand : strands) {
        drawStrand(img, cam, strand, lightDir, viewDir, hairColor, thickness);
    }

    // Save
    const std::string ppmPath = "fur_hair_output.ppm";
    const std::string pngPath = "fur_hair_output.png";
    if(!img.savePPM(ppmPath)) {
        std::cerr<<"Failed to write PPM\n";
        return 1;
    }
    std::cout<<"Saved "<<ppmPath<<"\n";

    // Convert to PNG via Python PIL (preferred) → ImageMagick → ffmpeg
    int ret = std::system(
        ("python3 -c \""
         "from PIL import Image; "
         "img=Image.open('" + ppmPath + "'); "
         "img.save('" + pngPath + "')\"").c_str()
    );
    if(ret != 0) {
        ret = std::system(("convert " + ppmPath + " " + pngPath).c_str());
    }
    if(ret != 0) {
        ret = std::system(("ffmpeg -y -i " + ppmPath + " " + pngPath + " 2>/dev/null").c_str());
    }
    if(ret != 0) {
        std::cerr<<"All conversions failed\n";
        return 1;
    }
    std::cout<<"Saved "<<pngPath<<"\n";
    return 0;
}
