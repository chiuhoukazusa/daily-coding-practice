/*
 * Water Wave Simulation & Rendering
 * 
 * 技术要点：
 * - Gerstner Wave（Trochoidal Wave）多叠加海浪
 * - 软光栅化三角形网格
 * - Fresnel 反射（Schlick近似）
 * - 次表面散射近似（深度着色）
 * - 天空颜色环境光
 * - 方向光（太阳）+ 高光
 * - 输出 1024x512 PNG
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <string>
#include <random>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb_image_write.h"
#pragma GCC diagnostic pop

using namespace std;

// ─── Math ────────────────────────────────────────────────────────────────────

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b)const{return {x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3& b)const{return {x-b.x,y-b.y,z-b.z};}
    Vec3 operator*(float t)const{return {x*t,y*t,z*t};}
    Vec3 operator/(float t)const{return {x/t,y/t,z/t};}
    Vec3 operator*(const Vec3& b)const{return {x*b.x,y*b.y,z*b.z};}
    Vec3& operator+=(const Vec3& b){x+=b.x;y+=b.y;z+=b.z;return *this;}
    float dot(const Vec3& b)const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3& b)const{
        return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x};
    }
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?Vec3(x/l,y/l,z/l):Vec3(0,1,0);}
    Vec3 clamp01()const{
        return {max(0.f,min(1.f,x)),max(0.f,min(1.f,y)),max(0.f,min(1.f,z))};
    }
};

inline Vec3 lerp(const Vec3& a, const Vec3& b, float t){
    return a*(1.f-t)+b*t;
}

// ─── Image ───────────────────────────────────────────────────────────────────

const int W = 1024, H = 512;

struct Image {
    vector<uint8_t> data;
    vector<float>   zbuf;
    Image():data(W*H*3,0),zbuf(W*H,1e30f){}
    void set(int x,int y,const Vec3& c){
        if(x<0||x>=W||y<0||y>=H)return;
        int i=(y*W+x)*3;
        data[i  ]=(uint8_t)(min(c.x,1.f)*255.f);
        data[i+1]=(uint8_t)(min(c.y,1.f)*255.f);
        data[i+2]=(uint8_t)(min(c.z,1.f)*255.f);
    }
    bool testZ(int x,int y,float z){
        if(x<0||x>=W||y<0||y>=H)return false;
        int i=y*W+x;
        if(z<zbuf[i]){zbuf[i]=z;return true;}
        return false;
    }
};

// ─── Gerstner Wave ────────────────────────────────────────────────────────────

struct GerstnerWave {
    float amplitude;    // 波幅
    float wavelength;   // 波长
    float speed;        // 相速度
    float steepness;    // 陡峭度 [0,1]
    Vec3  direction;    // 传播方向 (xz平面)
};

Vec3 gerstnerDisplace(const vector<GerstnerWave>& waves, float px, float pz, float t) {
    Vec3 pos(px, 0, pz);
    for (auto& w : waves) {
        float k   = 2.f * M_PI / w.wavelength;
        float c   = w.speed;
        float d   = w.direction.x * px + w.direction.z * pz;
        float phi = k * d - c * t;
        float Q   = w.steepness / (k * w.amplitude * (float)waves.size());
        pos.x += Q * w.amplitude * w.direction.x * cosf(phi);
        pos.z += Q * w.amplitude * w.direction.z * cosf(phi);
        pos.y += w.amplitude * sinf(phi);
    }
    return pos;
}

Vec3 gerstnerNormal(const vector<GerstnerWave>& waves, float px, float pz, float t) {
    float dx = 0, dz = 0, dy = 1;
    for (auto& w : waves) {
        float k   = 2.f * M_PI / w.wavelength;
        float c   = w.speed;
        float d   = w.direction.x * px + w.direction.z * pz;
        float phi = k * d - c * t;
        float Q   = w.steepness / (k * w.amplitude * (float)waves.size());
        float WA  = k * w.amplitude;
        dx += -w.direction.x * WA * cosf(phi);
        dz += -w.direction.z * WA * cosf(phi);
        dy += -Q * WA * sinf(phi);
    }
    return Vec3(-dx, dy, -dz).norm();
}

// ─── Camera & Projection ─────────────────────────────────────────────────────

struct Camera {
    Vec3 eye, lookat, up;
    float fovY; // radians
    float nearZ, farZ;
};

Vec3 project(const Vec3& world, const Camera& cam, float& outZ) {
    Vec3 forward = (cam.lookat - cam.eye).norm();
    Vec3 right   = forward.cross(cam.up).norm();
    Vec3 camUp   = right.cross(forward).norm();
    Vec3 rel     = world - cam.eye;
    float x = rel.dot(right);
    float y = rel.dot(camUp);
    float z = rel.dot(forward);
    outZ = z;
    if(z < 0.01f) return {-1,-1,-1};
    float aspect = (float)W / H;
    float tanHalfFov = tanf(cam.fovY * 0.5f);
    float ndcX = x / (z * tanHalfFov * aspect);
    float ndcY = y / (z * tanHalfFov);
    float sx = (ndcX + 1.f) * 0.5f * W;
    float sy = (1.f - ndcY) * 0.5f * H;
    return {sx, sy, z};
}

// ─── Triangle Rasterizer ─────────────────────────────────────────────────────

struct Vertex {
    Vec3 worldPos;
    Vec3 normal;
    float depth; // camera-space z
    float sx, sy; // screen coords
};

void rasterizeTriangle(Image& img, Vertex v0, Vertex v1, Vertex v2,
                       const Vec3& sunDir, const Vec3& eyePos,
                       const Vec3& sunColor, const Vec3& skyColor,
                       const Vec3& waterDeep, const Vec3& waterShallow)
{
    // Bounding box
    float minX = min({v0.sx,v1.sx,v2.sx});
    float maxX = max({v0.sx,v1.sx,v2.sx});
    float minY = min({v0.sy,v1.sy,v2.sy});
    float maxY = max({v0.sy,v1.sy,v2.sy});
    int x0 = max(0,(int)floorf(minX));
    int x1 = min(W-1,(int)ceilf(maxX));
    int y0 = max(0,(int)floorf(minY));
    int y1 = min(H-1,(int)ceilf(maxY));

    // Edge function
    auto edge = [](float ax,float ay,float bx,float by,float px,float py)->float{
        return (px-ax)*(by-ay)-(py-ay)*(bx-ax);
    };

    float area = edge(v0.sx,v0.sy, v1.sx,v1.sy, v2.sx,v2.sy);
    if(fabsf(area)<0.5f)return;

    for(int y=y0;y<=y1;y++){
        for(int x=x0;x<=x1;x++){
            float px=x+0.5f, py=y+0.5f;
            float w0 = edge(v1.sx,v1.sy, v2.sx,v2.sy, px,py);
            float w1 = edge(v2.sx,v2.sy, v0.sx,v0.sy, px,py);
            float w2 = edge(v0.sx,v0.sy, v1.sx,v1.sy, px,py);
            if(area>0){if(w0<0||w1<0||w2<0)continue;}
            else{if(w0>0||w1>0||w2>0)continue;}
            float b0=w0/area, b1=w1/area, b2=w2/area;

            float z   = v0.depth*b0 + v1.depth*b1 + v2.depth*b2;
            Vec3 wpos = v0.worldPos*b0 + v1.worldPos*b1 + v2.worldPos*b2;
            Vec3 norm = (v0.normal*b0 + v1.normal*b1 + v2.normal*b2).norm();

            if(!img.testZ(x,y,z))continue;

            // View direction
            Vec3 viewDir = (eyePos - wpos).norm();

            // Fresnel (Schlick)
            float cosTheta = max(0.f, norm.dot(viewDir));
            float F0 = 0.04f;
            float fresnel = F0 + (1.f-F0)*powf(1.f-cosTheta, 5.f);
            fresnel = min(fresnel, 0.95f);

            // Water depth color (fake SSS)
            float waveHeight = wpos.y; // roughly -1..1
            float depthFactor = (waveHeight + 1.f) * 0.5f;
            depthFactor = powf(clamp(depthFactor, 0.f, 1.f), 0.7f);
            Vec3 waterColor = lerp(waterDeep, waterShallow, depthFactor);

            // Diffuse + specular (Blinn-Phong)
            float NdotL = max(0.f, norm.dot(sunDir));
            Vec3 halfV  = (sunDir + viewDir).norm();
            float spec  = powf(max(0.f, norm.dot(halfV)), 80.f);
            float spec2 = powf(max(0.f, norm.dot(halfV)), 8.f) * 0.15f;

            // Sky reflection color
            Vec3 reflDir = norm * (2.f * norm.dot(viewDir)) - viewDir;
            float skyBlend = max(0.f, reflDir.y);
            Vec3 reflColor = lerp(Vec3(0.4f,0.5f,0.6f), skyColor, skyBlend);

            // Subsurface scatter approximation (light from below)
            float sssAmt = max(0.f, -sunDir.dot(norm)) * 0.5f;
            Vec3 sssColor = Vec3(0.0f, 0.3f, 0.25f) * sssAmt;

            // Combine
            Vec3 color = waterColor * (NdotL * 0.6f + 0.2f)
                       + sunColor * (spec + spec2)
                       + sssColor;
            // Fresnel blend with sky reflection
            color = lerp(color, reflColor, fresnel);

            // Fog (depth-based)
            float fogFactor = min(1.f, (z - 5.f) / 60.f);
            fogFactor = fogFactor * fogFactor;
            Vec3 fogColor(0.7f, 0.82f, 0.9f);
            color = lerp(color, fogColor, fogFactor);

            // Tone map (simple Reinhard)
            color = Vec3(color.x/(color.x+1.f),
                         color.y/(color.y+1.f),
                         color.z/(color.z+1.f));

            img.set(x,y,color.clamp01());
        }
    }
}

// ─── Sky Rendering ───────────────────────────────────────────────────────────

Vec3 skyColor(float u, float v, const Vec3& sunDir) {
    // v=0 horizon, v=1 zenith
    Vec3 zenith(0.1f, 0.3f, 0.7f);
    Vec3 horizon(0.7f, 0.82f, 0.92f);
    Vec3 sky = lerp(horizon, zenith, powf(clamp(v,0.f,1.f), 0.5f));

    // Sun disk
    Vec3 rayDir(
        (u - 0.5f) * 2.f * 1.7f,
        v * 0.8f + 0.05f,
        1.f
    );
    rayDir = rayDir.norm();
    float sunBlend = max(0.f, rayDir.dot(sunDir));
    Vec3 sunDisk = Vec3(1.f,0.95f,0.7f) * powf(sunBlend, 200.f);
    Vec3 sunGlow = Vec3(1.f,0.8f,0.4f) * powf(sunBlend, 12.f) * 0.4f;

    // Horizon warmth near sun
    Vec3 warm(0.9f, 0.65f, 0.3f);
    float horizWarm = (1.f-v) * max(0.f, sunDir.y) * 0.8f;
    sky = lerp(sky, warm, horizWarm);

    return (sky + sunDisk + sunGlow).clamp01();
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    printf("Water Wave Simulation & Rendering\n");
    printf("Resolution: %dx%d\n", W, H);

    Image img;

    // Time
    float t = 2.5f;  // simulation time

    // Camera: slightly elevated, looking at the sea
    Camera cam;
    cam.eye    = Vec3(0.f, 3.5f, -2.f);
    cam.lookat = Vec3(0.f, 0.f,  12.f);
    cam.up     = Vec3(0.f, 1.f,   0.f);
    cam.fovY   = 60.f * M_PI / 180.f;
    cam.nearZ  = 0.1f;
    cam.farZ   = 200.f;

    // Sun direction (slightly above horizon, warm golden hour)
    Vec3 sunDir = Vec3(0.5f, 0.35f, 0.8f).norm();
    Vec3 sunColor(1.2f, 0.95f, 0.6f);
    Vec3 skyAmb(0.6f, 0.78f, 0.9f);

    // Water colors
    Vec3 waterDeep(0.02f, 0.12f, 0.25f);
    Vec3 waterShallow(0.05f, 0.35f, 0.45f);

    // ── Draw Sky (background) ──
    for(int y=0;y<H/2+30;y++){
        for(int x=0;x<W;x++){
            float u = (float)x/W;
            float v = 1.f - (float)y/(H/2+30);
            Vec3 sc = skyColor(u, v, sunDir);
            img.set(x, y, sc);
        }
    }
    // Gradient horizon fade into water
    for(int y=H/2-10;y<H/2+30;y++){
        for(int x=0;x<W;x++){
            float blend = (float)(y-(H/2-10))/40.f;
            blend = min(1.f, max(0.f, blend));
            float u = (float)x/W;
            float v = max(0.f, 1.f-(float)y/(H/2+30));
            Vec3 sc = skyColor(u, v, sunDir);
            Vec3 wc = waterDeep;
            img.set(x, y, lerp(sc, wc, blend));
        }
    }
    // Lower half: water base
    for(int y=H/2+20;y<H;y++){
        for(int x=0;x<W;x++){
            img.set(x, y, waterDeep);
        }
    }

    // ── Gerstner Waves setup ──
    vector<GerstnerWave> waves = {
        {0.8f,  10.f, 2.5f, 0.6f, Vec3(1.f,0,0.4f).norm()},
        {0.5f,  6.f,  3.0f, 0.5f, Vec3(0.8f,0,1.f).norm()},
        {0.3f,  4.f,  3.5f, 0.4f, Vec3(-0.5f,0,1.f).norm()},
        {0.15f, 2.5f, 4.0f, 0.3f, Vec3(0.3f,0,1.f).norm()},
        {0.1f,  1.8f, 5.0f, 0.25f,Vec3(-0.2f,0,0.9f).norm()},
    };

    // ── Build & render ocean mesh ──
    // Grid from Z=2 to Z=70, X=-30 to 30
    const int NX = 80, NZ = 90;
    const float X0=-28.f, X1=28.f, Z0=2.f, Z1=70.f;

    // Sample grid
    vector<vector<Vec3>>  gPos(NZ+1, vector<Vec3>(NX+1));
    vector<vector<Vec3>>  gNorm(NZ+1, vector<Vec3>(NX+1));
    vector<vector<float>> gDepth(NZ+1, vector<float>(NX+1, 0));
    vector<vector<float>> gSX(NZ+1, vector<float>(NX+1, 0));
    vector<vector<float>> gSY(NZ+1, vector<float>(NX+1, 0));
    vector<vector<bool>>  gVis(NZ+1, vector<bool>(NX+1, true));

    for(int iz=0;iz<=NZ;iz++){
        float z0 = Z0 + (Z1-Z0)*iz/NZ;
        for(int ix=0;ix<=NX;ix++){
            float x0 = X0 + (X1-X0)*ix/NX;
            Vec3 p   = gerstnerDisplace(waves, x0, z0, t);
            Vec3 n   = gerstnerNormal(waves, x0, z0, t);
            gPos[iz][ix]  = p;
            gNorm[iz][ix] = n;
            float d;
            Vec3 sc = project(p, cam, d);
            gDepth[iz][ix] = d;
            gSX[iz][ix] = sc.x;
            gSY[iz][ix] = sc.y;
            gVis[iz][ix] = (d > 0.05f && sc.x>=-50 && sc.x<=W+50 && sc.y>=-50 && sc.y<=H+50);
        }
    }

    // Rasterize back to front (painter's order by iz)
    int tris=0;
    for(int iz=NZ-1;iz>=0;iz--){
        for(int ix=0;ix<NX;ix++){
            // Quad -> 2 triangles
            // i00=iz,ix  i01=iz,ix+1  i10=iz+1,ix  i11=iz+1,ix+1
            for(int tri=0;tri<2;tri++){
                int ia,ib,ic;
                if(tri==0){ ia=0; ib=1; ic=2; }
                else       { ia=1; ib=3; ic=2; }

                int coords[4][2]={{iz,ix},{iz,ix+1},{iz+1,ix},{iz+1,ix+1}};
                int ca=coords[ia][0],xa=coords[ia][1];
                int cb=coords[ib][0],xb=coords[ib][1];
                int cc=coords[ic][0],xc=coords[ic][1];

                if(!gVis[ca][xa]&&!gVis[cb][xb]&&!gVis[cc][xc])continue;
                if(gDepth[ca][xa]<0.05f||gDepth[cb][xb]<0.05f||gDepth[cc][xc]<0.05f)continue;

                Vertex va{gPos[ca][xa],gNorm[ca][xa],gDepth[ca][xa],gSX[ca][xa],gSY[ca][xa]};
                Vertex vb{gPos[cb][xb],gNorm[cb][xb],gDepth[cb][xb],gSX[cb][xb],gSY[cb][xb]};
                Vertex vc{gPos[cc][xc],gNorm[cc][xc],gDepth[cc][xc],gSX[cc][xc],gSY[cc][xc]};

                rasterizeTriangle(img, va, vb, vc,
                                  sunDir, cam.eye,
                                  sunColor, skyAmb,
                                  waterDeep, waterShallow);
                tris++;
            }
        }
    }
    printf("Rendered %d triangles\n", tris);

    // ── Save PNG ──
    const char* fname = "water_output.png";
    stbi_write_png(fname, W, H, 3, img.data.data(), W*3);
    printf("Saved: %s\n", fname);
    return 0;
}
