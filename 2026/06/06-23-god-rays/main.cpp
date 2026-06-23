// God Rays / Volumetric Light Shafts Renderer
// Screen-space ray marching from light source with single scattering
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <vector>

constexpr int W = 800, H = 600;
constexpr float PI = 3.141592653589793f;
constexpr float INF = 1e30f;

// ============ Vector Math ============
struct Vec2 { float x, y; Vec2():x(0),y(0){} Vec2(float x,float y):x(x),y(y){} };

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float s) const { float inv=1.0f/s; return {x*inv, y*inv, z*inv}; }
    
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float len() const { return sqrtf(x*x + y*y + z*z); }
    Vec3 norm() const { float l=len(); return l>1e-8f ? *this/l : Vec3(0,0,0); }
    Vec3 operator-() const { return {-x, -y, -z}; }
};

Vec3 operator*(float s, const Vec3& v) { return v*s; }

float clamp(float v, float lo, float hi) { return v<lo ? lo : v>hi ? hi : v; }
float saturate(float v) { return clamp(v, 0.0f, 1.0f); }

// ============ Matrix 4x4 ============
struct Mat4 {
    float m[4][4];
    Mat4() { memset(m, 0, sizeof(m)); }
    
    static Mat4 identity() {
        Mat4 r;
        for(int i=0;i<4;i++) r.m[i][i]=1;
        return r;
    }
    
    static Mat4 perspective(float fov, float aspect, float n, float f) {
        Mat4 r;
        float t = tanf(fov * 0.5f * PI / 180.0f);
        r.m[0][0] = 1.0f/(aspect*t);
        r.m[1][1] = 1.0f/t;
        r.m[2][2] = -(f+n)/(f-n);
        r.m[2][3] = -(2.0f*f*n)/(f-n);
        r.m[3][2] = -1.0f;
        return r;
    }
    
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = (center-eye).norm();
        Vec3 r = f.cross(up).norm();
        Vec3 u = r.cross(f);
        Mat4 m;
        m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
        m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
        m.m[2][0]=-f.x; m.m[2][1]=-f.y; m.m[2][2]=-f.z; m.m[2][3]=f.dot(eye);
        m.m[3][3]=1;
        return m;
    }
};

Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for(int i=0;i<4;i++)
        for(int k=0;k<4;k++)
            for(int j=0;j<4;j++)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

struct Vec4 { float x, y, z, w; };

Vec4 transform(const Mat4& m, const Vec3& v) {
    float x = m.m[0][0]*v.x + m.m[0][1]*v.y + m.m[0][2]*v.z + m.m[0][3];
    float y = m.m[1][0]*v.x + m.m[1][1]*v.y + m.m[1][2]*v.z + m.m[1][3];
    float z = m.m[2][0]*v.x + m.m[2][1]*v.y + m.m[2][2]*v.z + m.m[2][3];
    float w = m.m[3][0]*v.x + m.m[3][1]*v.y + m.m[3][2]*v.z + m.m[3][3];
    return {x, y, z, w};
}

// ============ Image Buffer ============
struct Image {
    int w, h;
    std::vector<unsigned char> data;
    
    Image(int w, int h) : w(w), h(h), data(w*h*3, 0) {}
    
    void set(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if(x<0||x>=w||y<0||y>=h) return;
        int idx = (y*w + x)*3;
        data[idx]=r; data[idx+1]=g; data[idx+2]=b;
    }
    
    void set(int x, int y, Vec3 c) {
        set(x, y,
            (unsigned char)(saturate(c.x)*255),
            (unsigned char)(saturate(c.y)*255),
            (unsigned char)(saturate(c.z)*255));
    }
    
    Vec3 get(int x, int y) const {
        if(x<0||x>=w||y<0||y>=h) return {0,0,0};
        int idx = (y*w + x)*3;
        return {data[idx]/255.0f, data[idx+1]/255.0f, data[idx+2]/255.0f};
    }
    
    void savePPM(const char* path) {
        FILE* f = fopen(path, "wb");
        fprintf(f, "P6\n%d %d\n255\n", w, h);
        fwrite(data.data(), 1, data.size(), f);
        fclose(f);
    }
};

// ============ Depth Buffer ============
struct DepthBuf {
    int w, h;
    std::vector<float> data;
    
    DepthBuf(int w, int h) : w(w), h(h), data(w*h, INF) {}
    
    float& at(int x, int y) { return data[y*w + x]; }
    float at(int x, int y) const { return data[y*w + x]; }
    
    void clear() { std::fill(data.begin(), data.end(), INF); }
};

// ============ Scene Objects ============
struct BoxObj {
    Vec3 center, halfExt, color;
};

struct Scene {
    Vec3 camPos, camTarget, camUp;
    float fov, camNear, camFar;
    std::vector<BoxObj> boxes;
    Vec3 lightPos;
    Vec3 lightColor;
    Vec3 ambient;
    Vec3 sunDir;
};

// ============ Rasterizer ============
struct Rasterizer {
    Image* img;
    DepthBuf* depth;
    
    void drawTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, Vec3 color) {
        auto toScreen = [this](Vec3 v) {
            v.x = (v.x + 1.0f) * 0.5f * img->w;
            v.y = (1.0f - v.y) * 0.5f * img->h;
            return v;
        };
        
        Vec3 a = toScreen(v0), b = toScreen(v1), c = toScreen(v2);
        
        int minX = std::max(0, (int)floorf(std::min({a.x, b.x, c.x})));
        int minY = std::max(0, (int)floorf(std::min({a.y, b.y, c.y})));
        int maxX = std::min(img->w-1, (int)ceilf(std::max({a.x, b.x, c.x})));
        int maxY = std::min(img->h-1, (int)ceilf(std::max({a.y, b.y, c.y})));
        
        float area = (b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y);
        if(fabsf(area) < 1e-6f) return;
        float invArea = 1.0f / area;
        
        for(int y=minY; y<=maxY; y++) {
            for(int x=minX; x<=maxX; x++) {
                float px = x + 0.5f, py = y + 0.5f;
                
                float w0 = ((b.x-px)*(c.y-py) - (c.x-px)*(b.y-py)) * invArea;
                float w1 = ((c.x-px)*(a.y-py) - (a.x-px)*(c.y-py)) * invArea;
                float w2 = 1.0f - w0 - w1;
                
                if(w0 < 0 || w1 < 0 || w2 < 0) continue;
                
                float z = v0.z*w0 + v1.z*w1 + v2.z*w2;
                if(z < -1.0f || z > 1.0f) continue;
                
                if(z < depth->at(x, y)) {
                    depth->at(x, y) = z;
                    img->set(x, y, color);
                }
            }
        }
    }
};

void rasterizeBox(Rasterizer& rast, const BoxObj& box, const Mat4& VP, const Vec3& lightDir) {
    Vec3 h = box.halfExt;
    Vec3 verts[8] = {
        box.center + Vec3(-h.x, -h.y, -h.z),
        box.center + Vec3( h.x, -h.y, -h.z),
        box.center + Vec3( h.x,  h.y, -h.z),
        box.center + Vec3(-h.x,  h.y, -h.z),
        box.center + Vec3(-h.x, -h.y,  h.z),
        box.center + Vec3( h.x, -h.y,  h.z),
        box.center + Vec3( h.x,  h.y,  h.z),
        box.center + Vec3(-h.x,  h.y,  h.z),
    };
    
    struct Face { int i[4]; Vec3 normal; };
    Face faces[6] = {
        {{0,1,2,3}, {0,0,-1}},  {{4,5,6,7}, {0,0,1}},
        {{0,1,5,4}, {0,-1,0}},  {{2,3,7,6}, {0,1,0}},
        {{0,3,7,4}, {-1,0,0}},  {{1,2,6,5}, {1,0,0}},
    };
    
    for(auto& face : faces) {
        Vec3 N = face.normal;
        float NdotL = saturate(N.dot(-lightDir));
        Vec3 shaded = box.color * (0.15f + NdotL * 0.85f);
        
        Vec4 t0 = transform(VP, verts[face.i[0]]);
        Vec4 t1 = transform(VP, verts[face.i[1]]);
        Vec4 t2 = transform(VP, verts[face.i[2]]);
        Vec4 t3 = transform(VP, verts[face.i[3]]);
        
        if(t0.w<=0||t1.w<=0||t2.w<=0||t3.w<=0) continue;
        
        Vec3 p0 = {t0.x/t0.w, t0.y/t0.w, t0.z/t0.w};
        Vec3 p1 = {t1.x/t1.w, t1.y/t1.w, t1.z/t1.w};
        Vec3 p2 = {t2.x/t2.w, t2.y/t2.w, t2.z/t2.w};
        Vec3 p3 = {t3.x/t3.w, t3.y/t3.w, t3.z/t3.w};
        
        rast.drawTriangle(p0, p1, p2, shaded);
        rast.drawTriangle(p0, p2, p3, shaded);
    }
}

// ============ God Rays Post-Process ============
struct GodRaysParams {
    int numSamples = 80;
    float density = 0.6f;
    float decay = 1.0f;
    float weight = 0.45f;
    float exposure = 1.8f;
    int blurRadius = 32;
};

// Compute screen-space light position from world-space
Vec2 worldToScreen(const Vec3& worldPos, const Mat4& VP, int w, int h) {
    Vec4 clip = transform(VP, worldPos);
    if(clip.w <= 0) return {-100, -100};
    float sx = (clip.x/clip.w + 1.0f) * 0.5f * w;
    float sy = (1.0f - clip.y/clip.w) * 0.5f * h;
    return {sx, sy};
}

Image computeGodRays(const Image& scene, const DepthBuf& depth, const Mat4& VP,
                     const Vec3& lightWorldPos, const Vec3& lightColor, const GodRaysParams& p) {
    Image rays(scene.w, scene.h);
    int w = scene.w, h = scene.h;
    
    // Create luminance map of bright regions only
    Image brightPass(w, h);
    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            Vec3 c = scene.get(x, y);
            float lum = (c.x + c.y + c.z) / 3.0f;
            float thresh = 0.3f;
            if(lum > thresh) {
                float a = (lum - thresh) / (1.0f - thresh);
                brightPass.set(x, y, {a, a, a}); // store as grayscale weight
            }
        }
    }
    
    Vec2 lightScreen = worldToScreen(lightWorldPos, VP, w, h);
    int lx = (int)(lightScreen.x + 0.5f);
    int ly = (int)(lightScreen.y + 0.5f);
    
    printf("Light screen pos: (%d, %d)\n", lx, ly);
    
    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            float dx = (float)(lx - x);
            float dy = (float)(ly - y);
            float dist = sqrtf(dx*dx + dy*dy);
            if(dist < 0.5f) { rays.set(x, y, lightColor * 3.0f); continue; }
            
            float stepX = dx / (float)p.numSamples;
            float stepY = dy / (float)p.numSamples;
            
            float curX = x + 0.5f;
            float curY = y + 0.5f;
            
            float accum = 0.0f;
            float totalWeight = 0.0f;
            
            for(int s=0; s<p.numSamples; s++) {
                curX += stepX;
                curY += stepY;
                
                int sx = (int)curX, sy = (int)curY;
                if(sx<0||sx>=w||sy<0||sy>=h) break;
                
                // Sample bright-pass luminance weight
                float weight = brightPass.get(sx, sy).x; // grayscale
                if(weight <= 0) continue;
                
                float t = (float)(s+1) / (float)p.numSamples;
                float decay = expf(-p.decay * t);
                
                float w = decay * weight * p.density;
                accum += w;
                totalWeight += 1.0f;
            }
            
            if(totalWeight > 0) accum = accum / totalWeight;
            
            // Scale and apply light color
            accum *= p.exposure;
            float distFade = expf(-dist / (float)std::max(w,h) * 2.0f);
            accum *= distFade;
            
            // Rays inherit light color, not scene color
            rays.set(x, y, lightColor * accum);
        }
    }
    
    return rays;
}

// Radial blur along light direction
Image radialBlur(const Image& rays, int lx, int ly, int radius) {
    Image result(rays.w, rays.h);
    int w = rays.w, h = rays.h;
    
    for(int y=0; y<h; y++) {
        for(int x=0; x<w; x++) {
            float dx = (float)(x - lx);
            float dy = (float)(y - ly);
            float dist = sqrtf(dx*dx + dy*dy);
            
            float dirX = dist > 0.01f ? dx/dist : 0;
            float dirY = dist > 0.01f ? dy/dist : 0;
            
            Vec3 accum(0,0,0);
            float totalWeight = 0.0f;
            
            for(int r = -radius; r <= radius; r++) {
                float t = (float)r * 0.7f;
                int sx = (int)(x + dirX * t + 0.5f);
                int sy = (int)(y + dirY * t + 0.5f);
                if(sx<0||sx>=w||sy<0||sy>=h) continue;
                
                float weight = expf(-fabsf(t) / (radius*0.4f));
                accum = accum + rays.get(sx, sy) * weight;
                totalWeight += weight;
            }
            
            if(totalWeight > 0) result.set(x, y, accum / totalWeight);
        }
    }
    
    return result;
}

// ============ Scene Setup ============
Scene buildForestScene() {
    Scene s;
    
    s.camPos = {6, 4, 12};
    s.camTarget = {0, 3, 0};
    s.camUp = {0, 1, 0};
    s.fov = 60; s.camNear = 0.1f; s.camFar = 100;
    
    s.lightPos = {0, 8, 0};
    s.lightColor = {1.0f, 0.95f, 0.7f};
    s.sunDir = (s.lightPos - Vec3(0,0,0)).norm();
    s.ambient = {0.08f, 0.1f, 0.18f};
    
    s.boxes.push_back({{0,-0.5f,0}, {20,0.5f,20}, {0.25f,0.45f,0.15f}});
    s.boxes.push_back({{-4, 2, -3}, {1, 2, 1}, {0.6f,0.55f,0.45f}});
    s.boxes.push_back({{-2, 3, -2}, {0.8f, 3, 0.8f}, {0.7f,0.65f,0.5f}});
    s.boxes.push_back({{0, 2.5f, -4}, {1.2f, 2.5f, 1}, {0.5f,0.5f,0.45f}});
    s.boxes.push_back({{2, 1.8f, -3}, {0.9f, 1.8f, 0.9f}, {0.65f,0.55f,0.4f}});
    s.boxes.push_back({{-5, 3.5f, 1}, {0.7f, 3.5f, 0.7f}, {0.55f,0.5f,0.4f}});
    s.boxes.push_back({{5, 4, 0}, {0.8f, 4, 0.8f}, {0.6f,0.5f,0.35f}});
    s.boxes.push_back({{-3, 4.5f, 2}, {0.6f, 4.5f, 0.6f}, {0.5f,0.45f,0.35f}});
    s.boxes.push_back({{6, 2.5f, -5}, {1.5f, 2.5f, 1.5f}, {0.5f,0.45f,0.38f}});
    s.boxes.push_back({{-6, 3, 2}, {0.6f, 3, 0.6f}, {0.5f,0.45f,0.35f}});
    
    return s;
}

void renderScene(const Vec3& lightPos, const char* suffix) {
    Scene scene = buildForestScene();
    scene.lightPos = lightPos;
    scene.sunDir = (scene.lightPos - Vec3(0,0,0)).norm();
    
    Image sceneImg(W, H);
    DepthBuf depthBuf(W, H);
    
    Mat4 V = Mat4::lookAt(scene.camPos, scene.camTarget, scene.camUp);
    Mat4 P = Mat4::perspective(scene.fov, (float)W/H, scene.camNear, scene.camFar);
    Mat4 VP = P * V;
    
    // Sky gradient
    for(int y=0; y<H; y++) {
        float t = (float)y / H;
        Vec3 sky = Vec3(0.9f,0.75f,0.5f)*(1-t) + Vec3(0.35f,0.55f,0.85f)*t;
        for(int x=0; x<W; x++) sceneImg.set(x, y, sky);
    }
    
    Rasterizer rast{&sceneImg, &depthBuf};
    
    printf("  Rasterizing %zu objects...\n", scene.boxes.size());
    for(auto& box : scene.boxes) rasterizeBox(rast, box, VP, scene.sunDir);
    
    GodRaysParams params;
    params.numSamples  = 120;
    params.density     = 0.7f;
    params.decay       = 1.2f;
    params.weight      = 0.8f;
    params.exposure    = 2.2f;
    params.blurRadius  = 16;
    
    printf("  Computing god rays...\n");
    Image rays = computeGodRays(sceneImg, depthBuf, VP, scene.lightPos, scene.lightColor, params);
    
    Vec2 ls = worldToScreen(scene.lightPos, VP, W, H);
    int lx = clamp((int)(ls.x+0.5f), 0, W-1);
    int ly = clamp((int)(ls.y+0.5f), 0, H-1);
    
    printf("  Radial blur...\n");
    Image blurred = radialBlur(rays, lx, ly, params.blurRadius);
    
    Image finalImg(W, H);
    for(int y=0; y<H; y++)
        for(int x=0; x<W; x++) {
            Vec3 sc = sceneImg.get(x, y);
            Vec3 gr = blurred.get(x, y);
            // Additive: warm rays stand out against scene colors
            finalImg.set(x, y, sc + gr * params.weight);
        }
    
    char buf[128];
    snprintf(buf, sizeof(buf), "scene_%s.ppm", suffix); sceneImg.savePPM(buf);
    snprintf(buf, sizeof(buf), "rays_raw_%s.ppm", suffix); rays.savePPM(buf);
    snprintf(buf, sizeof(buf), "rays_blur_%s.ppm", suffix); blurred.savePPM(buf);
    snprintf(buf, sizeof(buf), "output_%s.ppm", suffix); finalImg.savePPM(buf);
}

int main() {
    printf("=== God Rays / Volumetric Light Shafts ===\n");
    
    struct { Vec3 pos; const char* name; } lights[] = {
        {{2, 8, 2},  "sun_left"},
        {{-3, 7, 0}, "sun_right"},
        {{0, 8.5f, -1},"sun_center"},
    };
    
    for(auto& L : lights) {
        printf("\n--- Light: %s (%.0f,%.0f,%.0f) ---\n", L.name, L.pos.x,L.pos.y,L.pos.z);
        renderScene(L.pos, L.name);
    }
    
    // Default output with same naming for pipeline
    renderScene(lights[0].pos, "default");
    
    printf("\nDone.\n");
    return 0;
}
