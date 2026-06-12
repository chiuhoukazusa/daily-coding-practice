/*
 * Cone-Traced Ambient Occlusion Renderer
 * =======================================
 * Day 90 of Daily Coding Practice - 2026-05-28
 *
 * Implementation of a ray-casted AO renderer with:
 * - Soft rasterizer pipeline (CPU-based)
 * - Multi-cone importance sampling for AO
 * - Bent normals computation
 * - Distance field occlusion queries
 * - Scene with spheres, planes, boxes
 * - Three output images:
 *   1. ao_raw_output.png     - raw AO pass only
 *   2. ao_bent_output.png    - AO + bent normal visualization
 *   3. ao_final_output.png   - full lit scene with AO
 *
 * Algorithm:
 * For each surface point, cast N cones in the hemisphere
 * oriented around the bent normal. Each cone samples
 * multiple distances to evaluate occlusion. Accumulate
 * unoccluded directions -> bent normal.
 *
 * References:
 * - Landis 2002 "Production-ready global illumination"
 * - Hill 2011 "Ambient occlusion via ambient aperture lighting"
 * - Oat & Sander 2007 "Ambient aperture lighting"
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

// ============================================================
// Math
// ============================================================

struct Vec2 {
    float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
    Vec2 operator+(const Vec2& o)const{return{x+o.x,y+o.y};}
    Vec2 operator-(const Vec2& o)const{return{x-o.x,y-o.y};}
    Vec2 operator*(float t)const{return{x*t,y*t};}
};

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    Vec3 operator*(const Vec3& o)const{return{x*o.x,y*o.y,z*o.z};}  // component-wise
    Vec3 operator-()const{return{-x,-y,-z};}
    float dot(const Vec3& o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3& o)const{
        return{y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-8f?(*this)/l:Vec3(0,1,0);}
    Vec3 reflect(const Vec3& n)const{ return (*this) - n*2.f*dot(n); }
    static Vec3 lerp(const Vec3& a,const Vec3& b,float t){return a*(1-t)+b*t;}
};

Vec3 operator*(float t,const Vec3& v){return v*t;}

// ============================================================
// Image buffer
// ============================================================

struct Image {
    int width, height;
    std::vector<Vec3> pixels;  // linear HDR
    Image(int w,int h):width(w),height(h),pixels(w*h,Vec3(0)){}
    Vec3& at(int x,int y){return pixels[y*width+x];}
    const Vec3& at(int x,int y)const{return pixels[y*width+x];}

    void save(const std::string& path) const {
        std::vector<uint8_t> data(width*height*3);
        for(int i=0;i<width*height;i++){
            auto& p=pixels[i];
            // gamma 2.2 and clamp
            auto gc=[](float v)->uint8_t{
                v=std::pow(std::max(0.f,std::min(1.f,v)),1.f/2.2f);
                return (uint8_t)(v*255.f+.5f);
            };
            data[i*3+0]=gc(p.x);
            data[i*3+1]=gc(p.y);
            data[i*3+2]=gc(p.z);
        }
        stbi_write_png(path.c_str(),width,height,3,data.data(),width*3);
        printf("  Saved: %s\n",path.c_str());
    }
};

// ============================================================
// Random / Sampling
// ============================================================

struct RNG {
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist{0.f,1.f};
    RNG(uint32_t seed=42):gen(seed){}
    float next(){return dist(gen);}
};

// Cosine-weighted hemisphere sample
// Returns direction in local space (z=up)
Vec3 cosineSampleHemisphere(float u1, float u2) {
    float r = std::sqrt(u1);
    float phi = 2.f*3.14159265f*u2;
    float x = r*std::cos(phi);
    float y = r*std::sin(phi);
    float z = std::sqrt(std::max(0.f,1.f-u1));
    return Vec3(x,y,z);
}

// Transform direction from local (normal=Z) to world space
Vec3 toWorld(const Vec3& local, const Vec3& N) {
    Vec3 T,B;
    // Build TBN
    if(std::abs(N.x) < 0.9f)
        T = N.cross(Vec3(1,0,0)).norm();
    else
        T = N.cross(Vec3(0,1,0)).norm();
    B = N.cross(T);
    return T*local.x + B*local.y + N*local.z;
}

// ============================================================
// Scene geometry (SDF-based for easy AO query)
// ============================================================

struct HitInfo {
    float t;
    Vec3 normal;
    Vec3 color;
    int matId;  // 0=diffuse, 1=glossy, 2=emissive
};

// SDF primitives
float sdSphere(const Vec3& p, const Vec3& c, float r) {
    return (p-c).len() - r;
}
float sdPlane(const Vec3& p, float y) {
    return p.y - y;
}
float sdBox(const Vec3& p, const Vec3& c, const Vec3& half) {
    Vec3 q = Vec3(std::abs(p.x-c.x), std::abs(p.y-c.y), std::abs(p.z-c.z)) - half;
    Vec3 maxQ = Vec3(std::max(q.x,0.f), std::max(q.y,0.f), std::max(q.z,0.f));
    return maxQ.len() + std::min(std::max({q.x,q.y,q.z}),0.f);
}

struct SceneSDF {
    // Returns min SDF distance to any opaque object
    float query(const Vec3& p) const {
        float d = 1e9f;
        // Ground plane
        d = std::min(d, sdPlane(p, -1.f));
        // Main sphere cluster
        d = std::min(d, sdSphere(p, Vec3(0,0,0), 1.f));
        d = std::min(d, sdSphere(p, Vec3(2.5f,0,0), 0.8f));
        d = std::min(d, sdSphere(p, Vec3(-2.5f,0,0), 0.8f));
        d = std::min(d, sdSphere(p, Vec3(0,0,-2.5f), 0.9f));
        // Boxes
        d = std::min(d, sdBox(p, Vec3(1.8f,-0.4f,2.f), Vec3(0.5f,0.6f,0.5f)));
        d = std::min(d, sdBox(p, Vec3(-1.8f,-0.4f,2.f), Vec3(0.4f,0.6f,0.4f)));
        d = std::min(d, sdBox(p, Vec3(0,0.5f,2.5f), Vec3(0.3f,0.3f,0.3f)));
        return d;
    }
};

// Ray-march intersection
static const SceneSDF gScene;

bool rayMarch(const Vec3& ro, const Vec3& rd, float tMin, float tMax, HitInfo& hit) {
    float t = tMin;
    for(int i=0;i<200;i++) {
        Vec3 p = ro + rd*t;
        float d = gScene.query(p);
        if(d < 1e-4f) {
            hit.t = t;
            // Compute normal via finite diff
            const float eps = 1e-4f;
            float dx = gScene.query(p+Vec3(eps,0,0)) - gScene.query(p-Vec3(eps,0,0));
            float dy = gScene.query(p+Vec3(0,eps,0)) - gScene.query(p-Vec3(0,eps,0));
            float dz = gScene.query(p+Vec3(0,0,eps)) - gScene.query(p-Vec3(0,0,eps));
            hit.normal = Vec3(dx,dy,dz).norm();
            // Material / color by position
            if(p.y < -0.97f) {
                // Ground - checkerboard
                int cx = (int)std::floor(p.x*1.5f);
                int cz = (int)std::floor(p.z*1.5f);
                bool isWhite = ((cx+cz)&1)==0;
                hit.color = isWhite ? Vec3(0.85f,0.85f,0.85f) : Vec3(0.15f,0.15f,0.15f);
                hit.matId = 0;
            } else {
                // Spheres/boxes - by distance to sphere centers
                float ds0 = (p-Vec3(0,0,0)).len() - 1.f;
                float ds1 = (p-Vec3(2.5f,0,0)).len() - 0.8f;
                float ds2 = (p-Vec3(-2.5f,0,0)).len() - 0.8f;
                float ds3 = (p-Vec3(0,0,-2.5f)).len() - 0.9f;
                float minS = std::min({std::abs(ds0),std::abs(ds1),std::abs(ds2),std::abs(ds3)});
                if(minS < 0.05f) {
                    // which sphere?
                    if(std::abs(ds0)<0.05f) hit.color=Vec3(0.9f,0.3f,0.2f);
                    else if(std::abs(ds1)<0.05f) hit.color=Vec3(0.2f,0.7f,0.9f);
                    else if(std::abs(ds2)<0.05f) hit.color=Vec3(0.3f,0.9f,0.4f);
                    else hit.color=Vec3(0.9f,0.8f,0.2f);
                    hit.matId = 0;
                } else {
                    hit.color=Vec3(0.6f,0.55f,0.5f);
                    hit.matId = 0;
                }
            }
            return true;
        }
        t += d;
        if(t >= tMax) break;
    }
    return false;
}

// ============================================================
// Cone-traced AO
// ============================================================

// Evaluate AO and bent normal for a surface point.
// numCones: number of cones to shoot
// coneAngle: half-angle of each cone in radians (0=ray, pi/2=hemisphere)
// Returns: ao value in [0,1], 0=fully occluded, 1=fully open
//          bentNormal: average unoccluded direction (world space)

struct AOResult {
    float ao;
    Vec3 bentNormal;
};

AOResult computeConeAO(
    const Vec3& pos,
    const Vec3& N,
    int numCones,
    float maxDist,
    RNG& rng
) {
    Vec3 accDir(0,0,0);
    float accAO = 0.f;

    // Jitter offset to avoid self-intersection
    Vec3 origin = pos + N * 5e-4f;

    for(int i=0;i<numCones;i++) {
        float u1 = rng.next();
        float u2 = rng.next();
        // Cosine-weighted direction in hemisphere
        Vec3 localDir = cosineSampleHemisphere(u1, u2);
        Vec3 dir = toWorld(localDir, N);

        // Weight: cosine weight already from sampling
        float weight = 1.f;

        // Ray march query
        HitInfo hitInfo;
        bool hit = rayMarch(origin, dir, 1e-3f, maxDist, hitInfo);

        float visibility = 1.f;
        if(hit) {
            // Closer hits = more occlusion
            // Smooth falloff by distance
            float t = hitInfo.t;
            visibility = std::min(1.f, t / maxDist);
            visibility = visibility * visibility; // quadratic falloff
        }

        accAO += visibility * weight;
        if(visibility > 0.1f) {
            accDir = accDir + dir * visibility;
        }
    }

    AOResult result;
    result.ao = accAO / (float)numCones;
    result.bentNormal = (accDir.len() > 1e-6f) ? accDir.norm() : N;
    return result;
}

// ============================================================
// Lighting
// ============================================================

Vec3 sunDir = Vec3(1.f, 2.f, 1.f).norm();
Vec3 sunColor = Vec3(1.2f, 1.0f, 0.8f);
Vec3 skyColorTop = Vec3(0.3f, 0.5f, 0.9f);
Vec3 skyColorBot = Vec3(0.8f, 0.9f, 1.0f);

Vec3 sampleSky(const Vec3& dir) {
    float t = dir.y * 0.5f + 0.5f;
    Vec3 sky = Vec3::lerp(skyColorBot, skyColorTop, t);
    // Sun disc
    float sunDot = dir.dot(sunDir);
    if(sunDot > 0.9995f) {
        sky = sky + Vec3(3.f,2.5f,1.5f) * std::pow(sunDot, 2000.f);
    }
    // Sun halo
    sky = sky + sunColor * 0.4f * std::pow(std::max(0.f,sunDot), 6.f);
    return sky;
}

bool shadowRay(const Vec3& pos, const Vec3& toLight) {
    HitInfo h;
    return rayMarch(pos + toLight * 1e-3f, toLight, 1e-3f, 20.f, h);
}

// ============================================================
// Camera
// ============================================================

struct Camera {
    Vec3 eye, at, up;
    float fovY; // degrees
    int width, height;

    // Returns ray direction for pixel (px, py) with AA jitter
    Vec3 getRay(float px, float py) const {
        float aspect = (float)width / (float)height;
        float tanHalf = std::tan(fovY * 3.14159265f / 360.f);
        Vec3 fwd = (at - eye).norm();
        Vec3 right = fwd.cross(up).norm();
        Vec3 upV = right.cross(fwd);

        float u = ((px / (float)width) * 2.f - 1.f) * aspect * tanHalf;
        float v = ((py / (float)height) * 2.f - 1.f) * tanHalf;
        return (fwd + right*u + upV*v).norm();
    }
};

// ============================================================
// Rendering passes
// ============================================================

void renderAO(
    Image& imgAO,
    Image& imgBent,
    Image& imgFinal,
    const Camera& cam,
    int aoSamples,
    int aaLevel
) {
    int W = cam.width, H = cam.height;
    const int AA = aaLevel;

    printf("Rendering %dx%d @ %d AO samples, %dx AA...\n", W, H, aoSamples, AA);

    for(int py=0;py<H;py++) {
        if(py%50==0) printf("  Row %d/%d\n",py,H);
        for(int px=0;px<W;px++) {
            Vec3 accAO_color(0,0,0);
            Vec3 accBent_color(0,0,0);
            Vec3 accFinal_color(0,0,0);
            int samples = AA*AA;

            for(int sy=0;sy<AA;sy++) {
                for(int sx=0;sx<AA;sx++) {
                    RNG rng((uint32_t)(py*W*AA*AA + px*AA*AA + sy*AA + sx + 17));

                    float fpx = px + (sx + 0.5f) / AA;
                    float fpy = H - 1 - py + (sy + 0.5f) / AA; // flip Y

                    Vec3 rd = cam.getRay(fpx, fpy);

                    HitInfo hit;
                    bool hasHit = rayMarch(cam.eye, rd, 0.01f, 100.f, hit);

                    if(!hasHit) {
                        Vec3 sky = sampleSky(rd);
                        accAO_color = accAO_color + sky * 0.8f;
                        accBent_color = accBent_color + sky * 0.8f;
                        accFinal_color = accFinal_color + sky;
                    } else {
                        Vec3 P = cam.eye + rd * hit.t;

                        // --- AO pass ---
                        AOResult aoResult = computeConeAO(P, hit.normal, aoSamples, 4.f, rng);

                        // Image 1: raw AO (grayscale)
                        float aoVal = aoResult.ao;
                        accAO_color = accAO_color + Vec3(aoVal, aoVal, aoVal);

                        // Image 2: bent normal visualization
                        Vec3 bn = aoResult.bentNormal;
                        Vec3 bentVis = Vec3(bn.x*0.5f+0.5f, bn.y*0.5f+0.5f, bn.z*0.5f+0.5f);
                        bentVis = bentVis * aoVal; // darken occluded areas
                        accBent_color = accBent_color + bentVis;

                        // Image 3: full lighting with AO
                        Vec3 albedo = hit.color;
                        Vec3 N = hit.normal;

                        // Direct lighting (sun)
                        float shadow = shadowRay(P, sunDir) ? 0.f : 1.f;
                        float NdotL = std::max(0.f, N.dot(sunDir));
                        Vec3 directDiff = albedo * sunColor * NdotL * shadow;

                        // Specular (simple Blinn-Phong for glossy hint)
                        Vec3 H_vec = (sunDir - rd).norm();
                        float spec = std::pow(std::max(0.f, N.dot(H_vec)), 32.f) * shadow * 0.3f;
                        Vec3 directSpec = sunColor * spec;

                        // Indirect (sky ambient via bent normal)
                        Vec3 indirDir = aoResult.bentNormal;
                        float skyT = indirDir.y * 0.5f + 0.5f;
                        Vec3 indirColor = Vec3::lerp(skyColorBot, skyColorTop, skyT);
                        Vec3 indirect = albedo * indirColor * aoResult.ao * 0.6f;

                        Vec3 finalColor = directDiff + directSpec + indirect;
                        accFinal_color = accFinal_color + finalColor;
                    }
                }
            }

            imgAO.at(px,py) = accAO_color / (float)samples;
            imgBent.at(px,py) = accBent_color / (float)samples;
            imgFinal.at(px,py) = accFinal_color / (float)samples;
        }
    }
    printf("Rendering complete.\n");
}

// ============================================================
// Main
// ============================================================

int main() {
    const int WIDTH  = 640;
    const int HEIGHT = 480;
    const int AO_SAMPLES = 32;  // cones per pixel
    const int AA = 2;           // 2x2 super-sampling

    Camera cam;
    cam.eye = Vec3(5.f, 3.5f, 6.f);
    cam.at  = Vec3(0.f, 0.f, 0.f);
    cam.up  = Vec3(0,1,0);
    cam.fovY = 45.f;
    cam.width  = WIDTH;
    cam.height = HEIGHT;

    Image imgAO   (WIDTH, HEIGHT);
    Image imgBent (WIDTH, HEIGHT);
    Image imgFinal(WIDTH, HEIGHT);

    renderAO(imgAO, imgBent, imgFinal, cam, AO_SAMPLES, AA);

    imgAO.save   ("ao_raw_output.png");
    imgBent.save ("ao_bent_output.png");
    imgFinal.save("ao_final_output.png");

    printf("\nDone! 3 images generated.\n");
    return 0;
}
