/*
 * SSGI - Screen Space Global Illumination Renderer
 * 2026-05-22 | Daily Coding Practice
 *
 * Technique:
 *  - Software rasterizer: G-Buffer (albedo, normal, depth, position)
 *  - SSGI: hemisphere sampling in screen space to gather indirect diffuse
 *  - Importance sampling cosine-weighted hemisphere
 *  - Simple temporal accumulation / bilateral blur for denoising
 *  - Scene: Cornell-box style with colored walls, spheres, and area light
 *
 * Output: ssgi_output.png (800x600 PPM -> PNG via convert)
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <array>
#include <vector>
#include <string>
#include <fstream>
#include <cassert>
#include <random>
#include <functional>

// ─── Math primitives ─────────────────────────────────────────────────────────

struct Vec2 { float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
    Vec2 operator+(Vec2 o)const{return{x+o.x,y+o.y};}
    Vec2 operator*(float t)const{return{x*t,y*t};}
};

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(Vec3 o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(Vec3 o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator*(Vec3 o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    Vec3 operator/(Vec3 o)const{return{x/o.x,y/o.y,z/o.z};}
    Vec3 operator-()const{return{-x,-y,-z};}
    Vec3& operator+=(Vec3 o){x+=o.x;y+=o.y;z+=o.z;return*this;}
    float dot(Vec3 o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(Vec3 o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-8f?*this/l:Vec3(0,1,0);}
    Vec3 clamp01()const{return{std::max(0.f,std::min(1.f,x)),std::max(0.f,std::min(1.f,y)),std::max(0.f,std::min(1.f,z))};}
};

inline Vec3 lerp(Vec3 a, Vec3 b, float t){return a*(1-t)+b*t;}
inline float clamp01(float v){return std::max(0.f,std::min(1.f,v));}
inline float saturate(float v){return clamp01(v);}

struct Vec4 {
    float x, y, z, w;
    Vec4(float x=0,float y=0,float z=0,float w=1):x(x),y(y),z(z),w(w){}
    Vec3 xyz()const{return{x,y,z};}
};

// Row-major 4x4
struct Mat4 {
    float m[4][4];
    Mat4(){memset(m,0,sizeof(m));}
    static Mat4 identity(){Mat4 r;r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1;return r;}
    Vec4 operator*(Vec4 v)const{
        return {m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
                m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
                m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
                m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w};
    }
    Mat4 operator*(const Mat4& b)const{
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
            r.m[i][j]+=m[i][k]*b.m[k][j];
        return r;
    }
    static Mat4 perspective(float fovY, float asp, float near_, float far_){
        Mat4 r;
        float t=std::tan(fovY*0.5f);
        r.m[0][0]=1/(asp*t); r.m[1][1]=1/t;
        r.m[2][2]=-(far_+near_)/(far_-near_); r.m[2][3]=-2*far_*near_/(far_-near_);
        r.m[3][2]=-1;
        return r;
    }
    static Mat4 lookAt(Vec3 eye, Vec3 at, Vec3 up){
        Vec3 f=(at-eye).norm();
        Vec3 r=f.cross(up).norm();
        Vec3 u=r.cross(f);
        Mat4 m=identity();
        m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z;  m.m[0][3]=-r.dot(eye);
        m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z;  m.m[1][3]=-u.dot(eye);
        m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z; m.m[2][3]=f.dot(eye);
        m.m[3][3]=1;
        return m;
    }
    static Mat4 translate(Vec3 t){Mat4 m=identity();m.m[0][3]=t.x;m.m[1][3]=t.y;m.m[2][3]=t.z;return m;}
    static Mat4 scale(Vec3 s){Mat4 m=identity();m.m[0][0]=s.x;m.m[1][1]=s.y;m.m[2][2]=s.z;return m;}
    static Mat4 rotateY(float a){Mat4 m=identity();float c=std::cos(a),s=std::sin(a);
        m.m[0][0]=c;m.m[0][2]=s;m.m[2][0]=-s;m.m[2][2]=c;return m;}
    Mat4 transposed()const{Mat4 r;for(int i=0;i<4;i++)for(int j=0;j<4;j++)r.m[i][j]=m[j][i];return r;}
    // inverse for upper-left 3x3 + translate (affine)
    Mat4 affineInverse()const{
        // extract R and t
        Vec3 r0{m[0][0],m[0][1],m[0][2]};
        Vec3 r1{m[1][0],m[1][1],m[1][2]};
        Vec3 r2{m[2][0],m[2][1],m[2][2]};
        Vec3 t{m[0][3],m[1][3],m[2][3]};
        // R^T * (-t)
        Mat4 inv=identity();
        inv.m[0][0]=r0.x; inv.m[0][1]=r1.x; inv.m[0][2]=r2.x;
        inv.m[1][0]=r0.y; inv.m[1][1]=r1.y; inv.m[1][2]=r2.y;
        inv.m[2][0]=r0.z; inv.m[2][1]=r1.z; inv.m[2][2]=r2.z;
        inv.m[0][3]=-(r0.dot(t));
        inv.m[1][3]=-(r1.dot(t));
        inv.m[2][3]=-(r2.dot(t));
        return inv;
    }
};

// ─── Framebuffer / GBuffer ───────────────────────────────────────────────────

const int W = 800, H = 600;

struct GBuffer {
    std::vector<Vec3> albedo;   // diffuse color
    std::vector<Vec3> normal;   // world-space normal
    std::vector<Vec3> position; // world-space pos
    std::vector<float> depth;   // NDC depth [-1..1]
    std::vector<float> zbuf;    // viewport z (for rasterizer)
    std::vector<Vec3> emission; // emissive color
    GBuffer():albedo(W*H),normal(W*H),position(W*H),depth(W*H,2.f),zbuf(W*H,1e30f),emission(W*H){}
    void clear(){
        std::fill(albedo.begin(),albedo.end(),Vec3{});
        std::fill(normal.begin(),normal.end(),Vec3{});
        std::fill(position.begin(),position.end(),Vec3{});
        std::fill(depth.begin(),depth.end(),2.f);
        std::fill(zbuf.begin(),zbuf.end(),1e30f);
        std::fill(emission.begin(),emission.end(),Vec3{});
    }
    int idx(int x,int y)const{return y*W+x;}
};

struct Framebuffer {
    std::vector<Vec3> color;
    Framebuffer():color(W*H){}
    void clear(Vec3 c={}){std::fill(color.begin(),color.end(),c);}
    int idx(int x,int y)const{return y*W+x;}
};

// ─── Geometry ────────────────────────────────────────────────────────────────

struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec3 color;
};

struct Triangle {
    Vertex v[3];
};

struct Mesh {
    std::vector<Triangle> tris;
    Vec3 emission{};
    bool emissive = false;
};

// Build a quad (two triangles) given 4 corners (CCW) and color
static void addQuad(Mesh& mesh, Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 col, Vec3 normal){
    Vertex va{a,normal,col}, vb{b,normal,col}, vc{c,normal,col}, vd{d,normal,col};
    mesh.tris.push_back({va,vb,vc});
    mesh.tris.push_back({va,vc,vd});
}

// Sphere approximation with lat/lon triangles
static void addSphere(Mesh& mesh, Vec3 center, float radius, Vec3 col, int stacks=24, int slices=36){
    const float PI = 3.14159265f;
    auto v3 = [&](float lat, float lon)->Vec3{
        return { center.x + radius*std::cos(lat)*std::cos(lon),
                 center.y + radius*std::sin(lat),
                 center.z + radius*std::cos(lat)*std::sin(lon) };
    };
    for(int i=0;i<stacks;i++){
        float lat0 = PI*(-0.5f + float(i)/stacks);
        float lat1 = PI*(-0.5f + float(i+1)/stacks);
        for(int j=0;j<slices;j++){
            float lon0 = 2*PI*float(j)/slices;
            float lon1 = 2*PI*float(j+1)/slices;
            Vec3 p00=v3(lat0,lon0), p01=v3(lat0,lon1);
            Vec3 p10=v3(lat1,lon0), p11=v3(lat1,lon1);
            Vec3 n00=(p00-center).norm(), n01=(p01-center).norm();
            Vec3 n10=(p10-center).norm(), n11=(p11-center).norm();
            mesh.tris.push_back({{
                {p00,n00,col},{p01,n01,col},{p11,n11,col}
            }});
            mesh.tris.push_back({{
                {p00,n00,col},{p11,n11,col},{p10,n10,col}
            }});
        }
    }
}

// ─── Rasterizer ──────────────────────────────────────────────────────────────

// Clip a value in [lo, hi]
inline float clampf(float v, float lo, float hi){return std::max(lo,std::min(hi,v));}

struct Rasterizer {
    Mat4 mvp;
    Mat4 model;
    Mat4 modelIT; // inverse transpose for normals

    // Project vertex to NDC
    Vec4 project(Vec3 pos)const{
        return mvp * Vec4{pos.x,pos.y,pos.z,1};
    }

    // Rasterize one triangle into GBuffer
    void drawTriangle(GBuffer& gb, const Triangle& tri, bool emissive, Vec3 emitColor)const{
        Vec4 clip[3];
        for(int i=0;i<3;i++) clip[i] = project(tri.v[i].pos);

        // Simple near-plane clip: skip if all behind
        int behind=0;
        for(int i=0;i<3;i++) if(clip[i].w<=0) behind++;
        if(behind==3) return;

        // Perspective divide -> NDC -> viewport
        float px[3],py[3],pz[3];
        for(int i=0;i<3;i++){
            float w = clip[i].w;
            if(w<=0) w=1e-5f; // avoid div zero
            (void)w; // perspective divide only, w not stored
            px[i] = clip[i].x/w;
            py[i] = clip[i].y/w;
            pz[i] = clip[i].z/w;
        }

        // Viewport transform
        float sx[3],sy[3];
        for(int i=0;i<3;i++){
            sx[i] = (px[i]*0.5f+0.5f)*W;
            sy[i] = (1.f-(py[i]*0.5f+0.5f))*H; // flip Y
        }

        // Bounding box
        int minX=std::max(0,(int)std::floor(std::min({sx[0],sx[1],sx[2]}))-1);
        int maxX=std::min(W-1,(int)std::ceil(std::max({sx[0],sx[1],sx[2]})));
        int minY=std::max(0,(int)std::floor(std::min({sy[0],sy[1],sy[2]}))-1);
        int maxY=std::min(H-1,(int)std::ceil(std::max({sy[0],sy[1],sy[2]})));

        // Edge function
        auto edge=[&](float ax,float ay,float bx,float by,float cx,float cy)->float{
            return (bx-ax)*(cy-ay)-(by-ay)*(cx-ax);
        };
        float area = edge(sx[0],sy[0],sx[1],sy[1],sx[2],sy[2]);
        if(std::abs(area)<1e-6f) return;

        for(int y=minY;y<=maxY;y++){
            for(int x=minX;x<=maxX;x++){
                float fx=x+0.5f, fy=y+0.5f;
                float w0=edge(sx[1],sy[1],sx[2],sy[2],fx,fy)/area;
                float w1=edge(sx[2],sy[2],sx[0],sy[0],fx,fy)/area;
                float w2=edge(sx[0],sy[0],sx[1],sy[1],fx,fy)/area;
                if(w0<0||w1<0||w2<0) continue;

                // Perspective-correct interpolation
                float z = pz[0]*w0+pz[1]*w1+pz[2]*w2;
                int idx = gb.idx(x,y);
                if(z>=gb.zbuf[idx]) continue;
                gb.zbuf[idx]=z;
                gb.depth[idx]=z;

                // Interpolate world-space position
                // Use linear world-space interp (close enough for our FOV)
                auto& tv = tri.v;
                Vec3 wpos = tv[0].pos*w0 + tv[1].pos*w1 + tv[2].pos*w2;
                Vec3 wnor = (tv[0].normal*w0 + tv[1].normal*w1 + tv[2].normal*w2).norm();
                Vec3 wcol = tv[0].color*w0 + tv[1].color*w1 + tv[2].color*w2;

                gb.position[idx] = wpos;
                gb.normal[idx]   = wnor;
                gb.albedo[idx]   = wcol;
                gb.emission[idx] = emissive ? emitColor : Vec3{};
            }
        }
    }
};

// ─── Direct Lighting ─────────────────────────────────────────────────────────

struct Light {
    Vec3 pos;
    Vec3 color;
    float intensity;
};

Vec3 directLight(Vec3 pos, Vec3 normal, Vec3 albedo, const std::vector<Light>& lights){
    Vec3 result{};
    for(auto& l : lights){
        Vec3 dir = (l.pos - pos);
        float dist = dir.len();
        dir = dir / dist;
        float ndl = std::max(0.f, normal.dot(dir));
        float atten = 1.f / (1.f + dist*dist*0.08f);
        result += albedo * l.color * (l.intensity * ndl * atten);
    }
    // Ambient
    result += albedo * Vec3{0.06f,0.06f,0.08f};
    return result;
}

// ─── SSGI ────────────────────────────────────────────────────────────────────

// Build TBN from normal
static void buildTBN(Vec3 n, Vec3& t, Vec3& b){
    Vec3 up = (std::abs(n.y)<0.99f) ? Vec3{0,1,0} : Vec3{1,0,0};
    t = n.cross(up).norm();
    b = n.cross(t);
}

// Cosine-weighted hemisphere sample in world space
static Vec3 cosineSampleHemisphere(Vec3 n, float r1, float r2){
    // Map to disk then project to hemisphere
    float phi = 2.f * 3.14159265f * r1;
    float sinTheta = std::sqrt(r2);
    float cosTheta = std::sqrt(1.f - r2);
    Vec3 t, bi;
    buildTBN(n, t, bi);
    return (t*(sinTheta*std::cos(phi)) + bi*(sinTheta*std::sin(phi)) + n*cosTheta).norm();
}

// Low-discrepancy Halton sequence
static float halton(int index, int base){
    float f=1.f, r=0.f;
    int i=index;
    while(i>0){ f/=base; r+=f*(i%base); i/=base; }
    return r;
}

// SSGI: for each screen pixel, cast rays in hemisphere,
// look up hit point in screen space (depth-buffer reproject), gather indirect radiance
struct SSGI {
    const GBuffer& gb;
    const Framebuffer& directFb; // direct lighting result
    const Mat4& proj;
    const Mat4& view;
    int numSamples = 12;
    float radius = 0.8f;  // world-space max search radius
    int W, H;

    // Project world pos to screen UV
    bool worldToScreen(Vec3 wpos, float& u, float& v, float& ndcZ)const{
        Vec4 clip = proj * (view * Vec4{wpos.x,wpos.y,wpos.z,1});
        if(clip.w<=0) return false;
        ndcZ = clip.z/clip.w;
        if(ndcZ<-1||ndcZ>1) return false;
        u = clip.x/clip.w*0.5f+0.5f;
        v = 1.f-(clip.y/clip.w*0.5f+0.5f);
        return (u>=0&&u<1&&v>=0&&v<1);
    }

    Vec3 sampleIndirect(int px, int py, int frameIdx)const{
        int id = py*W+px;
        Vec3 pos    = gb.position[id];
        Vec3 normal = gb.normal[id];
        Vec3 albedo = gb.albedo[id];

        if(normal.len()<0.5f) return Vec3{};  // background pixel
        if(albedo.x+albedo.y+albedo.z < 1e-6f) return Vec3{};

        Vec3 indirect{};
        float totalWeight = 0.f;

        for(int s=0;s<numSamples;s++){
            // Halton + per-pixel offset for quasi-random
            float r1 = std::fmod(halton(frameIdx*numSamples+s, 2) + halton(px*13+py*7, 3), 1.f);
            float r2 = std::fmod(halton(frameIdx*numSamples+s, 3) + halton(px*17+py*11, 5), 1.f);

            // Sample direction (cosine hemisphere)
            Vec3 dir = cosineSampleHemisphere(normal, r1, r2);

            // Step along the ray in screen space
            // March a few steps to find intersection
            float hitRad = radius * (0.2f + r2 * 0.8f);
            Vec3 samplePos = pos + dir * hitRad;

            // Reproject samplePos to screen
            float su, sv, sndcZ;
            if(!worldToScreen(samplePos, su, sv, sndcZ)) continue;

            int sx = (int)(su * W);
            int sy = (int)(sv * H);
            if(sx<0||sx>=W||sy<0||sy>=H) continue;

            int sid = sy*W+sx;
            // Depth test: check if the reprojected depth is close to GBuffer depth
            float gbDepth = gb.depth[sid];
            float depthDiff = std::abs(sndcZ - gbDepth);
            if(depthDiff > 0.15f) continue;  // occlusion / depth discontinuity

            // Check normal agreement (reject backfacing)
            Vec3 sNormal = gb.normal[sid];
            if(sNormal.dot(normal) < -0.1f) continue;

            // Gather radiance from that screen pixel (direct + emission)
            Vec3 radiance = directFb.color[sid] + gb.emission[sid];

            // Weight by cosine (already importance sampled, so weight=1 for cosine)
            // Additional weight: normal agreement
            float nAgreement = std::max(0.f, sNormal.dot(dir));
            float w = 1.f + nAgreement;
            indirect += radiance * w;
            totalWeight += w;
        }

        if(totalWeight > 0.f){
            indirect = indirect / totalWeight;
        }

        // Modulate by albedo (indirect diffuse = albedo * incoming indirect)
        return albedo * indirect * 0.85f;
    }
};

// ─── Bilateral blur (spatial denoiser) ───────────────────────────────────────

static void bilateralBlur(const GBuffer& gb, const std::vector<Vec3>& input, std::vector<Vec3>& output, int W, int H, int radius=2){
    output.resize(W*H);
    const float sigmaColor = 0.3f;
    const float sigmaDepth = 0.08f;

    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            int id = y*W+x;
            Vec3 centerCol = input[id];
            float centerDepth = gb.depth[id];
            Vec3 n = gb.normal[id];
            if(n.len()<0.5f){ output[id]=centerCol; continue; }

            Vec3 sum{};
            float wsum = 0.f;
            for(int dy=-radius;dy<=radius;dy++){
                for(int dx=-radius;dx<=radius;dx++){
                    int nx=x+dx, ny=y+dy;
                    if(nx<0||nx>=W||ny<0||ny>=H) continue;
                    int nid=ny*W+nx;
                    Vec3 nc = input[nid];
                    float nd = gb.depth[nid];
                    Vec3 nn = gb.normal[nid];
                    float colorDiff = (nc-centerCol).len();
                    float depthDiff = std::abs(nd-centerDepth);
                    float normalSim = std::max(0.f,nn.dot(n));
                    float wc = std::exp(-colorDiff*colorDiff/(2*sigmaColor*sigmaColor));
                    float wd = std::exp(-depthDiff*depthDiff/(2*sigmaDepth*sigmaDepth));
                    float wn = normalSim*normalSim;
                    float w = wc*wd*wn;
                    sum += nc * w;
                    wsum += w;
                }
            }
            output[id] = wsum>0 ? sum/wsum : centerCol;
        }
    }
}

// ─── Tone mapping ────────────────────────────────────────────────────────────

static Vec3 aces(Vec3 x){
    const float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;
    return ((x*(x*a+Vec3{b,b,b}))*(Vec3{1,1,1}/((x*(x*c+Vec3{d,d,d})+Vec3{e,e,e})))).clamp01();
}

// ─── PPM export ──────────────────────────────────────────────────────────────

static void savePPM(const std::string& path, const std::vector<Vec3>& buf, int w, int h){
    std::ofstream f(path, std::ios::binary);
    f << "P6\n" << w << " " << h << "\n255\n";
    for(int i=0;i<w*h;i++){
        uint8_t r=(uint8_t)(std::min(1.f,buf[i].x)*255.f);
        uint8_t g=(uint8_t)(std::min(1.f,buf[i].y)*255.f);
        uint8_t b=(uint8_t)(std::min(1.f,buf[i].z)*255.f);
        f.put(r); f.put(g); f.put(b);
    }
}

// ─── Scene Setup ─────────────────────────────────────────────────────────────

static std::vector<Mesh> buildScene(){
    std::vector<Mesh> meshes;

    // Cornell box room:
    // Floor
    {   Mesh m;
        addQuad(m, {-3,-1,-1},{3,-1,-1},{3,-1,-7},{-3,-1,-7}, {0.75f,0.72f,0.65f}, {0,1,0});
        meshes.push_back(m);
    }
    // Ceiling
    {   Mesh m;
        addQuad(m, {-3,3,-1},{-3,3,-7},{3,3,-7},{3,3,-1}, {0.75f,0.72f,0.65f}, {0,-1,0});
        meshes.push_back(m);
    }
    // Back wall
    {   Mesh m;
        addQuad(m, {-3,-1,-7},{3,-1,-7},{3,3,-7},{-3,3,-7}, {0.85f,0.82f,0.78f}, {0,0,1});
        meshes.push_back(m);
    }
    // Left wall (red)
    {   Mesh m;
        addQuad(m, {-3,-1,-1},{-3,-1,-7},{-3,3,-7},{-3,3,-1}, {0.8f,0.15f,0.15f}, {1,0,0});
        meshes.push_back(m);
    }
    // Right wall (green)
    {   Mesh m;
        addQuad(m, {3,-1,-7},{3,-1,-1},{3,3,-1},{3,3,-7}, {0.15f,0.75f,0.15f}, {-1,0,0});
        meshes.push_back(m);
    }
    // Front wall (partial, behind camera - skip)

    // Area light quad on ceiling
    {   Mesh m;
        addQuad(m, {-0.8f,2.98f,-3.5f},{0.8f,2.98f,-3.5f},{0.8f,2.98f,-4.5f},{-0.8f,2.98f,-4.5f},
                {1.f,0.96f,0.88f}, {0,-1,0});
        m.emissive = true;
        m.emission = {2.0f, 1.9f, 1.7f};
        meshes.push_back(m);
    }

    // Sphere 1: white diffuse
    {   Mesh m;
        addSphere(m, {-1.2f,-0.2f,-4.5f}, 0.8f, {0.9f,0.85f,0.82f});
        meshes.push_back(m);
    }
    // Sphere 2: blue diffuse
    {   Mesh m;
        addSphere(m, {1.1f,-0.4f,-3.8f}, 0.6f, {0.2f,0.4f,0.9f});
        meshes.push_back(m);
    }
    // Sphere 3: orange
    {   Mesh m;
        addSphere(m, {0.2f,-0.7f,-5.5f}, 0.3f, {0.95f,0.5f,0.1f});
        meshes.push_back(m);
    }
    // Small emissive sphere
    {   Mesh m;
        addSphere(m, {-0.5f,0.5f,-3.0f}, 0.25f, {1.f,0.8f,0.4f});
        m.emissive = true;
        m.emission = {1.5f,1.0f,0.3f};
        meshes.push_back(m);
    }

    return meshes;
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(){
    printf("SSGI Screen Space Global Illumination Renderer\n");
    printf("Resolution: %dx%d\n", W, H);

    // Camera
    Vec3 eye{0,0.5f,1.5f}, at{0,0.3f,-3.f}, up{0,1,0};
    float fovY = 0.75f; // ~43 degrees
    float aspect = float(W)/H;
    float nearP = 0.1f, farP = 50.f;

    Mat4 view = Mat4::lookAt(eye, at, up);
    Mat4 proj = Mat4::perspective(fovY, aspect, nearP, farP);
    Mat4 mvp  = proj * view;

    // Lights (for direct)
    std::vector<Light> lights = {
        {{0.f, 2.7f, -4.f}, {1.f,0.96f,0.88f}, 4.f},
        {{-0.5f,0.5f,-3.0f}, {1.5f,1.0f,0.3f}, 1.5f},
    };

    // Scene
    auto meshes = buildScene();

    // GBuffer
    GBuffer gb;

    // Rasterize scene into GBuffer
    Rasterizer rast;
    rast.model = Mat4::identity();
    rast.mvp   = mvp;

    printf("Rasterizing scene...\n");
    for(auto& mesh : meshes){
        for(auto& tri : mesh.tris){
            rast.drawTriangle(gb, tri, mesh.emissive, mesh.emission);
        }
    }

    // Direct lighting pass
    Framebuffer directFb;
    directFb.clear({0.02f,0.02f,0.05f});
    printf("Direct lighting pass...\n");
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            int id=y*W+x;
            Vec3 n = gb.normal[id];
            if(n.len()<0.5f) continue;
            Vec3 col = directLight(gb.position[id], n, gb.albedo[id], lights);
            // Add emission
            col += gb.emission[id];
            directFb.color[id] = col;
        }
    }

    // SSGI pass
    printf("SSGI indirect lighting pass (%d samples/pixel)...\n", 16);
    std::vector<Vec3> indirectRaw(W*H);

    SSGI ssgi{gb, directFb, proj, view, 16, 0.9f, W, H};
    // Accumulate multiple frames (quasi-random)
    const int numFrames = 4;
    std::vector<Vec3> accumIndirect(W*H, Vec3{});
    for(int frame=0;frame<numFrames;frame++){
        printf("  Frame %d/%d...\n", frame+1, numFrames);
        for(int y=0;y<H;y++){
            for(int x=0;x<W;x++){
                int id=y*W+x;
                Vec3 ind = ssgi.sampleIndirect(x, y, frame*7+13);
                accumIndirect[id] += ind;
            }
        }
    }
    for(int i=0;i<W*H;i++) indirectRaw[i] = accumIndirect[i] / float(numFrames);

    // Bilateral denoise the indirect buffer
    printf("Denoising indirect buffer...\n");
    std::vector<Vec3> indirectDenoised;
    bilateralBlur(gb, indirectRaw, indirectDenoised, W, H, 3);

    // Composite: direct + indirect
    Framebuffer finalFb;
    finalFb.clear();
    printf("Compositing...\n");
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            int id=y*W+x;
            Vec3 direct   = directFb.color[id];
            Vec3 indirect = indirectDenoised[id];
            // Combine
            Vec3 hdr = direct + indirect;
            // Tone map
            Vec3 ldr = aces(hdr * 0.35f);
            // Gamma
            ldr.x=std::pow(ldr.x, 1.f/2.2f);
            ldr.y=std::pow(ldr.y, 1.f/2.2f);
            ldr.z=std::pow(ldr.z, 1.f/2.2f);
            finalFb.color[id] = ldr.clamp01();
        }
    }

    // Save PPM
    const std::string outPPM = "ssgi_output.ppm";
    const std::string outPNG = "ssgi_output.png";
    printf("Saving %s...\n", outPPM.c_str());
    savePPM(outPPM, finalFb.color, W, H);

    // Convert to PNG using Python/PIL
    int ret = std::system(("python3 -c \"from PIL import Image; Image.open('" + outPPM + "').save('" + outPNG + "')\" && echo PNG_SAVED").c_str());
    if(ret == 0){
        printf("Saved: %s\n", outPNG.c_str());
    } else {
        printf("PIL conversion failed (code %d), PPM saved: %s\n", ret, outPPM.c_str());
    }

    // Stats
    printf("Done. Output: %s\n", outPNG.c_str());
    return 0;
}
