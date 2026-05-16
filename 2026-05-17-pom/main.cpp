/*
 * Parallax Occlusion Mapping (POM) Renderer
 * ==========================================
 * Techniques:
 *   - Parallax Occlusion Mapping with binary search refinement
 *   - Self-shadowing via shadow ray marching along light direction
 *   - Procedural height maps (ridged noise, brick, waves)
 *   - Phong shading with normal perturbation from height gradients
 *   - Multiple material spheres / flat surface demo
 * Date: 2026-05-17
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <functional>

using std::min;
using std::max;
using std::clamp;

// -----------------------------------------------------------------------
// Math
// -----------------------------------------------------------------------
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float v = 0.f) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    float len2() const { return x*x + y*y + z*z; }
    float len()  const { return sqrtf(len2()); }
    Vec3  norm() const { float l = len(); return l>1e-9f?(*this)*(1.f/l):Vec3(0,1,0); }
    Vec3  neg()  const { return {-x,-y,-z}; }
};

inline Vec3 cross(const Vec3& a, const Vec3& b){
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline Vec3 lerp3(const Vec3& a, const Vec3& b, float t){ return a*(1-t)+b*t; }
inline float lerp(float a, float b, float t){ return a*(1-t)+b*t; }
inline float clampf(float v, float lo, float hi){ return v<lo?lo:v>hi?hi:v; }
inline Vec3 clamp3(const Vec3& v, float lo=0.f, float hi=1.f){
    return {clampf(v.x,lo,hi), clampf(v.y,lo,hi), clampf(v.z,lo,hi)};
}
inline float fract(float x){ return x - floorf(x); }
inline float mix(float a, float b, float t){ return a*(1-t)+b*t; }

// -----------------------------------------------------------------------
// Noise utilities
// -----------------------------------------------------------------------
inline float hash(float n){ return fract(sinf(n)*43758.5453f); }
inline float hash2(float x, float y){ return hash(x + y*127.1f); }

float smoothNoise(float x, float y){
    float ix = floorf(x), iy = floorf(y);
    float fx = x-ix, fy = y-iy;
    float ux = fx*fx*(3-2*fx), uy = fy*fy*(3-2*fy);
    float a = hash2(ix,   iy);
    float b = hash2(ix+1, iy);
    float c = hash2(ix,   iy+1);
    float d = hash2(ix+1, iy+1);
    return mix(mix(a,b,ux), mix(c,d,ux), uy);
}

float fbm(float x, float y, int octaves=6){
    float v=0, amp=0.5f, freq=1.f;
    for(int i=0;i<octaves;i++){
        v   += amp * smoothNoise(x*freq, y*freq);
        amp *= 0.5f;
        freq*= 2.f;
    }
    return v;
}

float ridgedFbm(float x, float y, int octaves=6){
    float v=0, amp=0.5f, freq=1.f;
    for(int i=0;i<octaves;i++){
        float n = smoothNoise(x*freq, y*freq);
        v   += amp * (1.f - fabsf(n*2-1));
        amp *= 0.5f;
        freq*= 2.f;
    }
    return clampf(v,0,1);
}

// -----------------------------------------------------------------------
// Height map definitions
// -----------------------------------------------------------------------
enum HeightMapType { HM_RIDGED, HM_BRICK, HM_WAVES, HM_CRATER };

float sampleHeight(float u, float v, HeightMapType type, float scale=4.f){
    u *= scale; v *= scale;
    switch(type){
    case HM_RIDGED:
        return ridgedFbm(u, v, 6);
    case HM_BRICK: {
        float row = floorf(v);
        float shift = (fmodf(row,2.f)<1.f)?0.5f:0.f;
        float uf = fract(u + shift)*8.f;
        float vf = fract(v)*4.f;
        float mx = sinf(uf*3.14159f)*sinf(vf*3.14159f);
        mx = clampf(mx*3.f, 0.f, 1.f);
        return 1.f - mx; // mortar is high (raised), brick surface lower
    }
    case HM_WAVES: {
        float wx = sinf(u*2.f*3.14159f)*0.5f + 0.5f;
        float wy = sinf(v*2.f*3.14159f + 0.3f)*0.5f + 0.5f;
        float base = (wx+wy)*0.5f;
        float detail = fbm(u,v,4)*0.3f;
        return clampf(base + detail, 0.f, 1.f);
    }
    case HM_CRATER: {
        // Several overlapping craters via distance to random points
        float h = fbm(u,v,4)*0.2f;
        // Add crater
        float cu = fmodf(fabsf(u), 2.f) - 1.f;
        float cv = fmodf(fabsf(v), 2.f) - 1.f;
        float d = sqrtf(cu*cu + cv*cv);
        float crater = clampf(1.f - d/0.7f, 0.f, 1.f);
        crater = crater*crater*(3-2*crater);
        float rim = clampf((d-0.6f)/0.15f, 0.f, 1.f)*(1.f-clampf((d-0.8f)/0.15f,0.f,1.f));
        h += rim*0.5f - crater*0.4f;
        return clampf(h + 0.5f, 0.f, 1.f);
    }
    }
    return 0.f;
}

// Compute normal from height map via central differences
Vec3 heightNormal(float u, float v, HeightMapType type, float scale=4.f, float eps=0.001f, float bumpScale=0.1f){
    float h  = sampleHeight(u,   v,   type, scale);
    float hx = sampleHeight(u+eps, v, type, scale);
    float hy = sampleHeight(u, v+eps, type, scale);
    float dhdx = (hx - h) / eps * bumpScale;
    float dhdy = (hy - h) / eps * bumpScale;
    // Tangent-space normal: perturb (0,0,1)
    Vec3 n = Vec3(-dhdx, -dhdy, 1.f).norm();
    return n;
}

// -----------------------------------------------------------------------
// POM: Parallax Occlusion Mapping
// -----------------------------------------------------------------------
// Given UV and view direction in tangent space (Vts), march through height
// field and find intersection. Returns refined UV.
Vec2 pomTrace(float u0, float v0, Vec3 Vts, HeightMapType type, float scale,
              float heightScale, int linearSteps=32, int binarySteps=8)
{
    // Vts should have negative z (looking into surface)
    // We march from (u0,v0,1) towards the surface
    float stepSize = 1.f / (float)linearSteps;
    float curDepth = 0.f;
    float cu = u0, cv = v0;
    // Parallax direction per unit depth step
    float du = -Vts.x / (Vts.z + 1e-6f) * heightScale;
    float dv = -Vts.y / (Vts.z + 1e-6f) * heightScale;

    (void)sampleHeight(cu, cv, type, scale); // initial call unused
    float prevDepth  = 0.f;

    for(int i=0; i<linearSteps; i++){
        curDepth += stepSize;
        float nu = cu + du * curDepth;
        float nv = cv + dv * curDepth;
        float h  = sampleHeight(nu, nv, type, scale);
        if(h > 1.f - curDepth){
            // Found crossing — binary refinement
            float lo = prevDepth, hi = curDepth;
            for(int j=0; j<binarySteps; j++){
                float mid = (lo+hi)*0.5f;
                float mu  = cu + du * mid;
                float mv  = cv + dv * mid;
                float mh  = sampleHeight(mu, mv, type, scale);
                if(mh > 1.f - mid) hi = mid;
                else                lo = mid;
            }
            float fd = (lo+hi)*0.5f;
            return {cu + du*fd, cv + dv*fd};
        }
        prevDepth  = curDepth;
    }
    return {cu + du*curDepth, cv + dv*curDepth};
}

// Self-shadow: march from displaced UV towards light in tangent space
float pomShadow(float u, float v, Vec3 Lts, HeightMapType type, float scale,
                float heightScale, float surfaceH, int steps=16)
{
    // Lts: light direction in tangent space
    if(Lts.z <= 0.f) return 0.f; // light below surface
    float shadowAccum = 0.f;
    float du = Lts.x / (Lts.z + 1e-6f) * heightScale;
    float dv = Lts.y / (Lts.z + 1e-6f) * heightScale;
    float stepSize = surfaceH / (float)steps;

    for(int i=1; i<=steps; i++){
        float t  = i * stepSize / surfaceH;
        float su = u + du * t;
        float sv = v + dv * t;
        float sh = sampleHeight(su, sv, type, scale);
        float expectedDepth = 1.f - surfaceH + stepSize * i;
        if(sh > expectedDepth){
            float occlusion = (sh - expectedDepth) * 10.f;
            shadowAccum = max(shadowAccum, clampf(occlusion, 0.f, 1.f));
        }
    }
    return 1.f - shadowAccum * 0.8f;
}

// -----------------------------------------------------------------------
// Shading
// -----------------------------------------------------------------------
Vec3 albedoFromType(HeightMapType type, float u, float v){
    switch(type){
    case HM_RIDGED:  return Vec3(0.6f,0.5f,0.35f); // sandy rock
    case HM_BRICK: {
        // Mortar: light gray, brick: terracotta
        float row = floorf(v*4.f*4.f);
        float shift = (fmodf(row,2.f)<1.f)?0.5f:0.f;
        float uf = fract(u*4.f + shift);
        float vf = fract(v*4.f);
        bool mortar = (uf < 0.06f || uf > 0.94f || vf < 0.08f || vf > 0.92f);
        return mortar ? Vec3(0.75f,0.75f,0.72f) : Vec3(0.72f, 0.32f, 0.18f);
    }
    case HM_WAVES:   return Vec3(0.2f, 0.45f, 0.75f); // ocean blue
    case HM_CRATER:  return Vec3(0.45f,0.42f,0.38f);  // moon gray
    }
    return Vec3(0.5f);
}

Vec3 shade(float u, float v, HeightMapType type, float hmScale,
           float heightScale, Vec3 viewDir, Vec3 lightDir, Vec3 lightColor,
           Vec3 ambientColor)
{
    // --- POM trace ---
    // Build TBN: assume flat quad, T=(1,0,0), B=(0,1,0), N=(0,0,1)
    Vec3 Vts = Vec3(-viewDir.x, -viewDir.y, -viewDir.z); // into surface
    // Normalize xy by z for correct parallax
    Vec2 uvNew = pomTrace(u, v, Vts, type, hmScale, heightScale);
    float uu = uvNew.x, vv = uvNew.y;

    // Height at displaced point
    float h = sampleHeight(uu, vv, type, hmScale);

    // Normal from height map (tangent space), transform to world space
    Vec3 nTS = heightNormal(uu, vv, type, hmScale, 0.001f, 0.15f);
    // TBN: T=(1,0,0) B=(0,1,0) N=(0,0,1) → world-space normal
    Vec3 N = Vec3(nTS.x, nTS.y, nTS.z).norm();

    // Albedo
    Vec3 albedo = albedoFromType(type, uu, vv);

    // Light in tangent space (same as world since flat surface)
    Vec3 Lts = lightDir; // tangent-space = world-space for flat quad

    // Self-shadow
    float shadowFactor = pomShadow(uu, vv, Lts, type, hmScale, heightScale, h);

    // Phong
    Vec3 L = lightDir.norm();
    Vec3 V = viewDir.norm();
    Vec3 H = (L + V).norm();

    float diff = max(0.f, N.dot(L));
    float spec = powf(max(0.f, N.dot(H)), 64.f);

    Vec3 ambient  = ambientColor * albedo * 0.15f;
    Vec3 diffuse  = lightColor * albedo * diff * shadowFactor;
    Vec3 specular = lightColor * Vec3(0.8f) * spec * shadowFactor;

    return ambient + diffuse + specular;
}

// -----------------------------------------------------------------------
// Framebuffer + tone-map
// -----------------------------------------------------------------------
struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    Image(int w, int h) : w(w), h(h), pixels(w*h, Vec3(0)) {}
    Vec3& at(int x, int y){ return pixels[y*w+x]; }
    void save(const char* path){
        std::vector<uint8_t> buf(w*h*3);
        for(int i=0;i<w*h;i++){
            // ACES approx tone map
            Vec3 c = pixels[i];
            auto aces = [](float v){
                float a=2.51f, b=0.03f, cc=2.43f, d=0.59f, e=0.14f;
                return clampf((v*(a*v+b))/(v*(cc*v+d)+e),0.f,1.f);
            };
            buf[i*3+0] = (uint8_t)(aces(c.x)*255.99f);
            buf[i*3+1] = (uint8_t)(aces(c.y)*255.99f);
            buf[i*3+2] = (uint8_t)(aces(c.z)*255.99f);
        }
        stbi_write_png(path, w, h, 3, buf.data(), w*3);
    }
};

// -----------------------------------------------------------------------
// Render a quad panel at z=0 with POM
// -----------------------------------------------------------------------
void renderPanel(Image& img, int x0, int y0, int pw, int ph,
                 HeightMapType type, float hmScale, float heightScale,
                 Vec3 eye, Vec3 lightDir, Vec3 lightColor, Vec3 ambColor)
{
    // Each pixel maps to UV [0,1] on the panel
    for(int py=0; py<ph; py++){
        for(int px=0; px<pw; px++){
            float u = (px + 0.5f) / pw;
            float v = 1.f - (py + 0.5f) / ph; // flip V (top = v=1)

            // View ray direction from eye to surface point on z=0 plane
            // Panel occupies [-1,1]x[-1,1] in world XY, z=0
            float wx = (u * 2.f - 1.f);
            float wy = (v * 2.f - 1.f);
            Vec3 surfPt(wx, wy, 0.f);
            Vec3 viewDir = (surfPt - eye).norm();

            Vec3 c = shade(u, v, type, hmScale, heightScale, viewDir.neg(), lightDir, lightColor, ambColor);

            // Tone map clamp
            c = clamp3(c, 0.f, 10.f);
            img.at(x0+px, y0+py) = c;
        }
    }
}

// Draw a label bar at bottom of panel
void drawBorder(Image& img, int x0, int y0, int pw, int ph, Vec3 color, int thickness=2){
    for(int t=0;t<thickness;t++){
        for(int px=0; px<pw; px++){
            img.at(x0+px, y0+t) = color;
            img.at(x0+px, y0+ph-1-t) = color;
        }
        for(int py=0; py<ph; py++){
            img.at(x0+t,    y0+py) = color;
            img.at(x0+pw-1-t, y0+py) = color;
        }
    }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main(){
    const int W = 1200, H = 700;
    Image img(W, H);

    printf("Rendering Parallax Occlusion Mapping...\n");

    // Eye position (slightly above, angled)
    Vec3 eye(0.f, -0.3f, 2.5f);

    // Light directions for each panel
    Vec3 lightDir = Vec3(0.6f, 0.8f, 1.2f).norm();
    Vec3 lightColor(1.4f, 1.3f, 1.1f);
    Vec3 ambColor(0.3f, 0.32f, 0.4f);

    // Layout: 2x2 grid of panels with labels
    const int PAD = 8;
    const int LABEL_H = 30;
    int panelW = (W - PAD*3) / 2;
    int panelH = (H - PAD*3 - LABEL_H*2) / 2;

    struct Panel { HeightMapType type; float hmScale; float heightScale; const char* name; Vec3 border; };
    Panel panels[4] = {
        { HM_RIDGED, 3.f, 0.12f, "Ridged FBM Rock",    Vec3(0.8f,0.6f,0.3f) },
        { HM_BRICK,  1.f, 0.15f, "Brick Wall (POM)",    Vec3(0.8f,0.3f,0.2f) },
        { HM_WAVES,  2.f, 0.10f, "Procedural Waves",    Vec3(0.2f,0.5f,0.9f) },
        { HM_CRATER, 2.f, 0.14f, "Lunar Craters",       Vec3(0.7f,0.7f,0.7f) },
    };

    int positions[4][2] = {
        {PAD, PAD},
        {PAD*2 + panelW, PAD},
        {PAD, PAD*2 + panelH + LABEL_H},
        {PAD*2 + panelW, PAD*2 + panelH + LABEL_H},
    };

    for(int i=0; i<4; i++){
        int px = positions[i][0];
        int py = positions[i][1];
        printf("  [%d/4] Rendering: %s ...\n", i+1, panels[i].name);
        fflush(stdout);

        renderPanel(img, px, py, panelW, panelH,
                    panels[i].type, panels[i].hmScale, panels[i].heightScale,
                    eye, lightDir, lightColor, ambColor);

        drawBorder(img, px, py, panelW, panelH, panels[i].border, 3);
        printf("         Done.\n");
    }

    // Fill background
    for(int y=0; y<H; y++){
        for(int x=0; x<W; x++){
            Vec3& p = img.at(x,y);
            bool inPanel = false;
            for(int i=0;i<4;i++){
                int px = positions[i][0], py = positions[i][1];
                if(x>=px && x<px+panelW && y>=py && y<py+panelH){ inPanel=true; break; }
            }
            if(!inPanel){
                // Dark gradient background
                float t = (float)y / H;
                p = Vec3(0.06f,0.07f,0.10f)*(1-t) + Vec3(0.12f,0.10f,0.08f)*t;
            }
        }
    }

    const char* outFile = "pom_output.png";
    img.save(outFile);
    printf("Saved: %s (%dx%d)\n", outFile, W, H);
    return 0;
}
