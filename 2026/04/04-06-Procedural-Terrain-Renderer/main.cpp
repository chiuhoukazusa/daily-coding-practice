// Procedural Terrain Renderer
// Techniques: Perlin Noise heightmap, normal calculation, LOD-inspired grid,
//             Phong lighting, perspective projection, software rasterizer
// Output: terrain_output.png (800x600)

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <array>
#include <cassert>
#include <numeric>
#include <random>

// ─── Math ────────────────────────────────────────────────────────────────────

struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    float dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const { float l = length(); return l>1e-7f ? *this/l : Vec3(0,1,0); }
    Vec3 clamp01() const {
        return { std::clamp(x,0.f,1.f), std::clamp(y,0.f,1.f), std::clamp(z,0.f,1.f) };
    }
};

struct Vec4 { float x,y,z,w; };

// ─── Perlin Noise ─────────────────────────────────────────────────────────────

struct PerlinNoise {
    std::vector<int> p;

    PerlinNoise(unsigned seed = 42) {
        p.resize(512);
        std::iota(p.begin(), p.begin()+256, 0);
        std::mt19937 rng(seed);
        std::shuffle(p.begin(), p.begin()+256, rng);
        for(int i=0;i<256;i++) p[256+i]=p[i];
    }

    static float fade(float t) { return t*t*t*(t*(t*6-15)+10); }
    static float lerp(float a,float b,float t) { return a+t*(b-a); }
    static float grad(int hash, float x, float y, float z) {
        int h = hash & 15;
        float u = h<8?x:y, v = h<4?y:(h==12||h==14?x:z);
        return ((h&1)?-u:u) + ((h&2)?-v:v);
    }

    float noise(float x, float y, float z=0) const {
        int X = (int)std::floor(x) & 255;
        int Y = (int)std::floor(y) & 255;
        int Z = (int)std::floor(z) & 255;
        x -= std::floor(x); y -= std::floor(y); z -= std::floor(z);
        float u=fade(x), v=fade(y), w=fade(z);
        int A=p[X]+Y, AA=p[A]+Z, AB=p[A+1]+Z;
        int B=p[X+1]+Y, BA=p[B]+Z, BB=p[B+1]+Z;
        return lerp(
            lerp(lerp(grad(p[AA],x,y,z),grad(p[BA],x-1,y,z),u),
                 lerp(grad(p[AB],x,y-1,z),grad(p[BB],x-1,y-1,z),u),v),
            lerp(lerp(grad(p[AA+1],x,y,z-1),grad(p[BA+1],x-1,y,z-1),u),
                 lerp(grad(p[AB+1],x,y-1,z-1),grad(p[BB+1],x-1,y-1,z-1),u),v),w);
    }

    // Fractional Brownian Motion
    float fbm(float x, float y, int octaves=6, float persistence=0.5f, float lacunarity=2.0f) const {
        float val=0, amp=1, freq=1, maxVal=0;
        for(int i=0;i<octaves;i++){
            val += noise(x*freq, y*freq) * amp;
            maxVal += amp;
            amp *= persistence;
            freq *= lacunarity;
        }
        return val / maxVal;
    }
};

// ─── Image ───────────────────────────────────────────────────────────────────

struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    std::vector<float> depth;

    Image(int w, int h): w(w), h(h), pixels(w*h, Vec3(0,0,0)), depth(w*h, 1e18f) {}

    void set(int x, int y, const Vec3& c) {
        if(x>=0&&x<w&&y>=0&&y<h) pixels[y*w+x] = c;
    }
    Vec3 get(int x, int y) const {
        if(x>=0&&x<w&&y>=0&&y<h) return pixels[y*w+x];
        return {};
    }
    float& dep(int x, int y) { return depth[y*w+x]; }

    // Write PPM (raw)
    void writePNG(const std::string& fname) const {
        // Write as PPM then convert; if no convert, write PPM directly
        std::string ppm = fname;
        // replace .png with .ppm if needed for direct write
        auto pos = ppm.rfind(".png");
        if(pos!=std::string::npos) ppm.replace(pos,4,".ppm");

        std::ofstream f(ppm, std::ios::binary);
        f << "P6\n" << w << " " << h << "\n255\n";
        for(auto& p : pixels){
            // Reinhard tone-mapping + gamma
            auto c = p.clamp01();
            f.put((unsigned char)(std::pow(c.x,1/2.2f)*255.99f));
            f.put((unsigned char)(std::pow(c.y,1/2.2f)*255.99f));
            f.put((unsigned char)(std::pow(c.z,1/2.2f)*255.99f));
        }
        f.close();

        // Try to convert to PNG
        int ret = std::system(("convert " + ppm + " " + fname + " 2>/dev/null && rm " + ppm).c_str());
        (void)ret;
    }
};

// ─── Terrain ─────────────────────────────────────────────────────────────────

struct Terrain {
    int N;             // grid resolution
    float size;        // world size
    float heightScale;
    std::vector<float> heights;
    PerlinNoise pn;

    Terrain(int N=256, float size=10.f, float hScale=2.f, unsigned seed=123)
        : N(N), size(size), heightScale(hScale), heights(N*N), pn(seed)
    {
        build();
    }

    void build() {
        for(int j=0;j<N;j++){
            for(int i=0;i<N;i++){
                float nx = (float)i/(N-1) * 3.5f;
                float ny = (float)j/(N-1) * 3.5f;
                float h = pn.fbm(nx, ny, 7, 0.55f, 2.0f);
                // Normalize from [-1,1] to [0,1] and apply power for interesting shape
                h = (h + 1.f) * 0.5f;
                h = std::pow(h, 1.5f);
                heights[j*N+i] = h * heightScale;
            }
        }
    }

    float heightAt(int i, int j) const {
        i = std::clamp(i,0,N-1); j = std::clamp(j,0,N-1);
        return heights[j*N+i];
    }

    Vec3 worldPos(int i, int j) const {
        float x = ((float)i/(N-1) - 0.5f) * size;
        float z = ((float)j/(N-1) - 0.5f) * size;
        float y = heightAt(i,j);
        return {x, y, z};
    }

    Vec3 normal(int i, int j) const {
        float hL = heightAt(i-1,j), hR = heightAt(i+1,j);
        float hD = heightAt(i,j-1), hU = heightAt(i,j+1);
        float dx = size/(N-1);
        Vec3 n{ (hL-hR)/(2*dx), 2.0f, (hD-hU)/(2*dx) };
        return n.normalized();
    }

    Vec3 biomeColor(int i, int j) const {
        float h  = heightAt(i,j) / heightScale;  // normalized 0..1
        Vec3 n   = normal(i,j);
        float slope = 1.f - n.y; // how vertical the slope is

        // Snow cap
        if(h > 0.72f) {
            float t = std::min((h - 0.72f)/0.12f, 1.f);
            Vec3 snow{0.95f,0.97f,1.0f};
            Vec3 rock{0.55f,0.50f,0.45f};
            return snow*t + rock*(1-t);
        }
        // Rock (steep slopes or high altitude)
        if(slope > 0.25f || h > 0.50f) {
            float ht = std::clamp((h-0.4f)/0.2f, 0.f, 1.f);
            Vec3 darkRock{0.38f,0.32f,0.26f};
            Vec3 lightRock{0.60f,0.54f,0.44f};
            return darkRock*(1-ht) + lightRock*ht;
        }
        // Forest / grass
        if(h > 0.22f) {
            float t = (h-0.22f)/0.3f;
            Vec3 brightGrass{0.28f,0.58f,0.18f};
            Vec3 darkForest{0.14f,0.38f,0.10f};
            return brightGrass*(1-t) + darkForest*t;
        }
        // Beach / sand
        if(h > 0.14f) {
            float t = (h-0.14f)/0.08f;
            Vec3 sand{0.85f,0.78f,0.52f};
            Vec3 grass{0.32f,0.60f,0.22f};
            return sand*(1-t) + grass*t;
        }
        // Water
        {
            float t = h/0.14f;
            Vec3 deepWater{0.04f,0.12f,0.38f};
            Vec3 shallowWater{0.12f,0.38f,0.62f};
            return deepWater*(1-t) + shallowWater*t;
        }
    }
};

// ─── Camera & Projection ─────────────────────────────────────────────────────

struct Camera {
    Vec3 pos, target, up;
    float fovY, aspect, near_, far_;

    Camera(Vec3 p, Vec3 t, Vec3 u, float fov, float asp)
        : pos(p), target(t), up(u), fovY(fov), aspect(asp), near_(0.1f), far_(100.f) {}

    // View matrix columns
    Vec3 forward() const { return (target - pos).normalized(); }
    Vec3 right()   const { return forward().cross(up).normalized(); }
    Vec3 realUp()  const { return right().cross(forward()); }

    Vec4 project(const Vec3& world, int W, int H) const {
        // View transform
        Vec3 f = forward(), r = right(), u2 = realUp();
        Vec3 d = world - pos;
        float vx = d.dot(r), vy = d.dot(u2), vz = d.dot(f);  // positive = in front

        if(vz < near_) return {0,0,0,-1};  // vz = dot(d, forward) > 0 means in front

        float tanH = std::tan(fovY*0.5f * (float)M_PI/180.f);
        float px = vx / (vz * tanH * aspect);
        float py = vy / (vz * tanH);

        int sx = (int)((px + 1.f)*0.5f * W);
        int sy = (int)((1.f - (py + 1.f)*0.5f) * H);
        return { (float)sx, (float)sy, vz, 1.f };
    }
};

// ─── Rasterizer Helpers ──────────────────────────────────────────────────────

// Barycentric coordinates for 2D triangle
bool bary(float ax,float ay, float bx,float by, float cx,float cy,
          float px,float py, float& u,float& v,float& w) {
    float d = (by-cy)*(ax-cx)+(cx-bx)*(ay-cy);
    if(std::abs(d)<1e-6f) return false;
    u = ((by-cy)*(px-cx)+(cx-bx)*(py-cy)) / d;
    v = ((cy-ay)*(px-cx)+(ax-cx)*(py-cy)) / d;
    w = 1.f-u-v;
    return u>=-1e-4f && v>=-1e-4f && w>=-1e-4f;
}

// Draw triangle with depth test
void drawTriangle(Image& img,
                  Vec4 p0, Vec4 p1, Vec4 p2,
                  Vec3 c0, Vec3 c1, Vec3 c2)
{
    int W=img.w, H=img.h;
    int minX = std::max(0,(int)std::min({p0.x,p1.x,p2.x}));
    int maxX = std::min(W-1,(int)std::max({p0.x,p1.x,p2.x})+1);
    int minY = std::max(0,(int)std::min({p0.y,p1.y,p2.y}));
    int maxY = std::min(H-1,(int)std::max({p0.y,p1.y,p2.y})+1);

    for(int y=minY;y<=maxY;y++){
        for(int x=minX;x<=maxX;x++){
            float u,v,w;
            if(!bary(p0.x,p0.y, p1.x,p1.y, p2.x,p2.y, x+.5f,y+.5f, u,v,w)) continue;
            float depth = u*p0.z + v*p1.z + w*p2.z;
            if(depth >= img.dep(x,y)) continue;
            img.dep(x,y) = depth;
            Vec3 col = c0*u + c1*v + c2*w;
            img.set(x,y,col);
        }
    }
}

// ─── Rendering ───────────────────────────────────────────────────────────────

void renderTerrain(Image& img, const Terrain& terrain, const Camera& cam,
                   const Vec3& sunDir)
{
    int W = img.w, H = img.h;
    int N = terrain.N;

    // Background gradient (sky) - dramatic blue to warm horizon
    for(int y=0;y<H;y++){
        float t = (float)y/H;
        Vec3 topSky{0.20f,0.40f,0.75f};
        Vec3 horizSky{0.75f,0.82f,0.95f};
        Vec3 sunGlow{0.95f,0.80f,0.60f};
        // Sun glow near horizon (right side)
        Vec3 skyCol = topSky*(1-t) + horizSky*t;
        // Add warm sun glow at horizon
        if(t > 0.7f) {
            float g = (t-0.7f)/0.3f;
            skyCol = skyCol*(1-g*0.4f) + sunGlow*(g*0.4f);
        }
        for(int x=0;x<W;x++) img.set(x,y,skyCol);
    }

    // Render terrain grid as triangles
    // Use a step for LOD-style reduction (render every step-th grid cell)
    // Close to camera = step 1, far = step 2
    int step = 2; // simple uniform for performance

    for(int j=0;j<N-step;j+=step){
        for(int i=0;i<N-step;i+=step){
            // Four corners of the quad
            int i0=i, i1=i+step, j0=j, j1=j+step;
            i1=std::min(i1,N-1); j1=std::min(j1,N-1);

            Vec3 v00=terrain.worldPos(i0,j0), v10=terrain.worldPos(i1,j0);
            Vec3 v01=terrain.worldPos(i0,j1), v11=terrain.worldPos(i1,j1);
            Vec3 n00=terrain.normal(i0,j0), n10=terrain.normal(i1,j0);
            Vec3 n01=terrain.normal(i0,j1), n11=terrain.normal(i1,j1);

            Vec3 bc00=terrain.biomeColor(i0,j0), bc10=terrain.biomeColor(i1,j0);
            Vec3 bc01=terrain.biomeColor(i0,j1), bc11=terrain.biomeColor(i1,j1);

            // Phong shading per vertex
            Vec3 ambient{0.15f,0.15f,0.18f};
            Vec3 lightColor{1.0f,0.97f,0.88f};

            auto shade = [&](const Vec3& n, const Vec3& bc, const Vec3& pos) -> Vec3 {
                float diff = std::max(0.f, n.dot(sunDir));
                // Specular (subtle)
                Vec3 viewDir = (cam.pos - pos).normalized();
                Vec3 halfV = (sunDir + viewDir).normalized();
                float spec = std::pow(std::max(0.f, n.dot(halfV)), 48.f) * 0.20f;
                Vec3 col = bc * (ambient + lightColor*diff*0.90f) + lightColor*spec;
                // Light atmospheric fog (reduced to see terrain better)
                float dist = (pos - cam.pos).length();
                float fog = std::exp(-dist * 0.025f);
                Vec3 fogColor{0.65f,0.75f,0.92f};
                return col*fog + fogColor*(1-fog);
            };

            Vec3 c00=shade(n00,bc00,v00), c10=shade(n10,bc10,v10);
            Vec3 c01=shade(n01,bc01,v01), c11=shade(n11,bc11,v11);

            Vec4 p00=cam.project(v00,W,H), p10=cam.project(v10,W,H);
            Vec4 p01=cam.project(v01,W,H), p11=cam.project(v11,W,H);

            // Two triangles per quad (skip if any vertex behind camera)
            if(p00.w>0 && p10.w>0 && p01.w>0)
                drawTriangle(img, p00,p10,p01, c00,c10,c01);
            if(p10.w>0 && p11.w>0 && p01.w>0)
                drawTriangle(img, p10,p11,p01, c10,c11,c01);
        }
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main() {
    constexpr int W=800, H=600;
    Image img(W,H);

    // Build terrain
    Terrain terrain(200, 12.f, 3.5f, 42);

    // Camera: dramatic low angle looking across the terrain
    Vec3 camPos{-1.0f, 5.5f, -7.5f};
    Vec3 camTarget{0.5f, 1.0f, 3.0f};
    Camera cam(camPos, camTarget, {0,1,0}, 65.f, (float)W/H);

    // Sun direction (afternoon light)
    Vec3 sunDir = Vec3{1.2f, 2.5f, 0.8f}.normalized();

    renderTerrain(img, terrain, cam, sunDir);

    img.writePNG("terrain_output.png");
    std::cout << "Rendered terrain_output.png (" << W << "x" << H << ")\n";
    return 0;
}
