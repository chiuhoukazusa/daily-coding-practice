// Cel Shading & Outline Renderer
// NPR卡通渲染: 色阶量化 + 轮廓描边 + 法线膨胀外描边
// 技术: Phong光照分级量化, 边缘检测, 法线膨胀描边, Specular卡通化
// 输出: cel_shading_output.png (800x600)

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <cstring>
#include <cstdint>
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <array>
#include <functional>

// ============================================================
// Math
// ============================================================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3 operator-()               const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float dot(const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    Vec3  cross(const Vec3& o)const { return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x}; }
    float len()  const { return std::sqrt(x*x+y*y+z*z); }
    Vec3  norm() const { float l=len(); return l>1e-7f ? (*this)/l : Vec3{0,0,0}; }
    Vec3  mul(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
};
inline Vec3 lerp(Vec3 a, Vec3 b, float t){ return a*(1-t)+b*t; }
inline Vec3 clampV(Vec3 v, float lo, float hi){
    return {std::clamp(v.x,lo,hi), std::clamp(v.y,lo,hi), std::clamp(v.z,lo,hi)};
}

struct Vec4 {
    float x,y,z,w;
    Vec4(){}
    Vec4(float x,float y,float z,float w):x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v,float w):x(v.x),y(v.y),z(v.z),w(w){}
};

struct Mat4 {
    float m[4][4];
    Mat4(){ memset(m,0,sizeof(m)); }
    static Mat4 identity(){
        Mat4 r;
        for(int i=0;i<4;i++) r.m[i][i]=1;
        return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
            m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
            m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
            m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w,
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++)
            for(int k=0;k<4;k++) r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 makeTranslate(float tx, float ty, float tz){
    Mat4 r=Mat4::identity();
    r.m[0][3]=tx; r.m[1][3]=ty; r.m[2][3]=tz;
    return r;
}
Mat4 makeScale(float sx, float sy, float sz){
    Mat4 r=Mat4::identity();
    r.m[0][0]=sx; r.m[1][1]=sy; r.m[2][2]=sz;
    return r;
}
Mat4 makeRotX(float a){
    Mat4 r=Mat4::identity();
    r.m[1][1]=std::cos(a); r.m[1][2]=-std::sin(a);
    r.m[2][1]=std::sin(a); r.m[2][2]= std::cos(a);
    return r;
}
Mat4 makeRotY(float a){
    Mat4 r=Mat4::identity();
    r.m[0][0]= std::cos(a); r.m[0][2]=std::sin(a);
    r.m[2][0]=-std::sin(a); r.m[2][2]=std::cos(a);
    return r;
}
Mat4 makeLookAt(Vec3 eye, Vec3 center, Vec3 up){
    Vec3 f=(center-eye).norm();
    Vec3 s=f.cross(up).norm();
    Vec3 u=s.cross(f);
    Mat4 r=Mat4::identity();
    r.m[0][0]=s.x; r.m[0][1]=s.y; r.m[0][2]=s.z; r.m[0][3]=-s.dot(eye);
    r.m[1][0]=u.x; r.m[1][1]=u.y; r.m[1][2]=u.z; r.m[1][3]=-u.dot(eye);
    r.m[2][0]=-f.x;r.m[2][1]=-f.y;r.m[2][2]=-f.z;r.m[2][3]=f.dot(eye);
    r.m[3][3]=1;
    return r;
}
Mat4 makePerspective(float fovY, float aspect, float near, float far){
    float tanHalf=std::tan(fovY*0.5f);
    Mat4 r;
    r.m[0][0]=1.0f/(aspect*tanHalf);
    r.m[1][1]=1.0f/tanHalf;
    r.m[2][2]=-(far+near)/(far-near);
    r.m[2][3]=-2.0f*far*near/(far-near);
    r.m[3][2]=-1.0f;
    return r;
}

// ============================================================
// Framebuffer
// ============================================================
const int W = 800, H = 600;
struct Color { uint8_t r,g,b; };

std::vector<Color>  fb(W*H);
std::vector<float>  depthBuf(W*H, std::numeric_limits<float>::infinity());
// normal buffer for outline edge detection (world-space)
std::vector<Vec3>   normalBuf(W*H, Vec3{0,0,0});
// object-id buffer (0 = background)
std::vector<int>    objBuf(W*H, 0);

void clearBuffers(){
    for(auto& c:fb) c={200,230,255}; // sky-ish background
    std::fill(depthBuf.begin(),depthBuf.end(),std::numeric_limits<float>::infinity());
    std::fill(normalBuf.begin(),normalBuf.end(),Vec3{0,0,0});
    std::fill(objBuf.begin(),objBuf.end(),0);
}

inline void setPixel(int x, int y, Color c){
    if(x<0||x>=W||y<0||y>=H) return;
    fb[y*W+x]=c;
}

// ============================================================
// PNG writer (minimal, no zlib dependency — using raw deflate store)
// ============================================================
static uint32_t crc32table[256];
static bool crcInited=false;
void initCRC(){
    if(crcInited) return;
    for(uint32_t i=0;i<256;i++){
        uint32_t c=i;
        for(int j=0;j<8;j++) c=(c&1)?(0xEDB88320^(c>>1)):(c>>1);
        crc32table[i]=c;
    }
    crcInited=true;
}
uint32_t crc32(const uint8_t* d, size_t n, uint32_t c=0xFFFFFFFF){
    initCRC();
    while(n--) c=crc32table[(c^*d++)&0xFF]^(c>>8);
    return c^0xFFFFFFFF;
}

void writeU32BE(std::vector<uint8_t>& v, uint32_t x){
    v.push_back(x>>24); v.push_back((x>>16)&0xFF);
    v.push_back((x>>8)&0xFF); v.push_back(x&0xFF);
}
void writeChunk(std::vector<uint8_t>& out, const char* type, const std::vector<uint8_t>& data){
    writeU32BE(out,(uint32_t)data.size());
    uint32_t t=0;
    for(int i=0;i<4;i++){out.push_back(type[i]); t=(t<<8)|(uint8_t)type[i];}
    out.insert(out.end(),data.begin(),data.end());
    std::vector<uint8_t> crcBuf(4+data.size());
    for(int i=0;i<4;i++) crcBuf[i]=(uint8_t)(t>>(24-8*i));
    memcpy(crcBuf.data()+4,data.data(),data.size());
    writeU32BE(out,crc32(crcBuf.data(),crcBuf.size()));
}

// Adler-32 and ZLIB store (no compression)
uint32_t adler32(const uint8_t* d, size_t n){
    uint32_t s1=1,s2=0;
    for(size_t i=0;i<n;i++){s1=(s1+d[i])%65521; s2=(s2+s1)%65521;}
    return (s2<<16)|s1;
}
std::vector<uint8_t> zlibStore(const std::vector<uint8_t>& raw){
    std::vector<uint8_t> out;
    out.push_back(0x78); out.push_back(0x01); // CMF, FLG
    size_t pos=0, total=raw.size();
    while(pos<total){
        size_t blockSize=std::min((size_t)65535,total-pos);
        bool last=(pos+blockSize>=total);
        out.push_back(last?0x01:0x00);
        out.push_back((uint8_t)(blockSize&0xFF));
        out.push_back((uint8_t)(blockSize>>8));
        out.push_back((uint8_t)(~blockSize&0xFF));
        out.push_back((uint8_t)((~blockSize)>>8));
        out.insert(out.end(),raw.begin()+pos,raw.begin()+pos+blockSize);
        pos+=blockSize;
    }
    uint32_t a=adler32(raw.data(),raw.size());
    out.push_back(a>>24); out.push_back((a>>16)&0xFF);
    out.push_back((a>>8)&0xFF); out.push_back(a&0xFF);
    return out;
}

bool savePNG(const std::string& path){
    std::vector<uint8_t> raw;
    raw.reserve((3*W+1)*H);
    for(int y=H-1;y>=0;y--){ // flip Y: row 0 = top
        raw.push_back(0); // filter byte
        for(int x=0;x<W;x++){
            auto& c=fb[y*W+x];
            raw.push_back(c.r); raw.push_back(c.g); raw.push_back(c.b);
        }
    }

    std::vector<uint8_t> png;
    // PNG signature
    const uint8_t sig[]={137,80,78,71,13,10,26,10};
    png.insert(png.end(),sig,sig+8);

    // IHDR
    std::vector<uint8_t> ihdr(13);
    ihdr[0]=W>>24; ihdr[1]=(W>>16)&0xFF; ihdr[2]=(W>>8)&0xFF; ihdr[3]=W&0xFF;
    ihdr[4]=H>>24; ihdr[5]=(H>>16)&0xFF; ihdr[6]=(H>>8)&0xFF; ihdr[7]=H&0xFF;
    ihdr[8]=8; ihdr[9]=2; ihdr[10]=0; ihdr[11]=0; ihdr[12]=0;
    writeChunk(png,"IHDR",ihdr);

    // IDAT
    auto compressed=zlibStore(raw);
    writeChunk(png,"IDAT",compressed);

    // IEND
    writeChunk(png,"IEND",{});

    std::ofstream f(path,std::ios::binary);
    if(!f){ std::cerr<<"Cannot write "<<path<<"\n"; return false; }
    f.write((char*)png.data(),png.size());
    return f.good();
}

// ============================================================
// Geometry: Icosphere
// ============================================================
struct Vertex { Vec3 pos; Vec3 normal; };
struct Triangle { int a,b,c; };
struct Mesh { std::vector<Vertex> verts; std::vector<Triangle> tris; };

Mesh buildIcosphere(int subdivisions){
    // Base icosahedron
    float phi=(1.0f+std::sqrt(5.0f))*0.5f;
    std::vector<Vec3> pts={
        {-1,phi,0},{1,phi,0},{-1,-phi,0},{1,-phi,0},
        {0,-1,phi},{0,1,phi},{0,-1,-phi},{0,1,-phi},
        {phi,0,-1},{phi,0,1},{-phi,0,-1},{-phi,0,1}
    };
    for(auto& p:pts) p=p.norm();

    std::vector<Triangle> tris={
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    };

    // Subdivide
    for(int d=0;d<subdivisions;d++){
        std::vector<Triangle> newTris;
        std::vector<Vec3> newPts=pts;
        auto midpoint=[&](int a, int b)->int{
            Vec3 m=((pts[a]+pts[b])*0.5f).norm();
            newPts.push_back(m);
            return (int)newPts.size()-1;
        };
        for(auto& t:tris){
            int ab=midpoint(t.a,t.b);
            int bc=midpoint(t.b,t.c);
            int ca=midpoint(t.c,t.a);
            newTris.push_back({t.a,ab,ca});
            newTris.push_back({t.b,bc,ab});
            newTris.push_back({t.c,ca,bc});
            newTris.push_back({ab,bc,ca});
        }
        pts=newPts; tris=newTris;
    }

    // Build mesh with per-vertex normals = position on unit sphere
    Mesh mesh;
    mesh.verts.resize(pts.size());
    for(size_t i=0;i<pts.size();i++){
        mesh.verts[i].pos=pts[i];
        mesh.verts[i].normal=pts[i]; // unit sphere: normal = pos
    }
    mesh.tris=tris;
    return mesh;
}

// Build torus
Mesh buildTorus(int majorSeg, int minorSeg, float R, float r){
    Mesh mesh;
    for(int i=0;i<=majorSeg;i++){
        float phi=2.0f*M_PI*i/majorSeg;
        for(int j=0;j<=minorSeg;j++){
            float theta=2.0f*M_PI*j/minorSeg;
            Vec3 center{R*std::cos(phi), 0, R*std::sin(phi)};
            Vec3 radial{std::cos(phi),0,std::sin(phi)};
            Vec3 normal{std::cos(phi)*std::cos(theta), std::sin(theta), std::sin(phi)*std::cos(theta)};
            Vec3 pos=center+normal*r;
            mesh.verts.push_back({pos, normal});
        }
    }
    for(int i=0;i<majorSeg;i++){
        for(int j=0;j<minorSeg;j++){
            int a=i*(minorSeg+1)+j;
            int b=a+1;
            int c=(i+1)*(minorSeg+1)+j;
            int d=c+1;
            mesh.tris.push_back({a,c,b});
            mesh.tris.push_back({b,c,d});
        }
    }
    return mesh;
}

// Build simple flat ground plane
Mesh buildPlane(float size, int divs){
    Mesh mesh;
    float step=size*2.0f/divs;
    for(int j=0;j<=divs;j++){
        for(int i=0;i<=divs;i++){
            float x=-size+i*step;
            float z=-size+j*step;
            mesh.verts.push_back({{x,0,z},{0,1,0}});
        }
    }
    for(int j=0;j<divs;j++){
        for(int i=0;i<divs;i++){
            int a=j*(divs+1)+i;
            int b=a+1;
            int c=(j+1)*(divs+1)+i;
            int d=c+1;
            mesh.tris.push_back({a,b,c});
            mesh.tris.push_back({b,d,c});
        }
    }
    return mesh;
}

// ============================================================
// Cel Shading
// ============================================================
// Quantize diffuse intensity into N steps
float celQuantize(float intensity, int steps){
    float s=(float)steps;
    return std::floor(intensity*s)/s;
}

Color toColor(Vec3 v){
    v=clampV(v,0,1);
    return {(uint8_t)(v.x*255),(uint8_t)(v.y*255),(uint8_t)(v.z*255)};
}

// Cel shading for a pixel
// returns: shaded color
Color celShade(Vec3 baseColor, Vec3 N, Vec3 L, Vec3 V, Vec3 lightColor, Vec3 ambientColor){
    float NdotL=std::max(0.0f, N.dot(L));
    // Quantize to 3 steps: dark, mid, bright
    float diffuseSteps=3.0f;
    float q=std::floor(NdotL*diffuseSteps)/diffuseSteps;

    // Specular (hard-edge)
    Vec3 H=(L+V).norm();
    float NdotH=std::max(0.0f, N.dot(H));
    float spec=(NdotH > 0.9f) ? 1.0f : 0.0f; // hard specular

    Vec3 diffuse=baseColor.mul(lightColor)*q;
    Vec3 specular=lightColor*spec*0.6f;
    Vec3 ambient=baseColor.mul(ambientColor)*0.3f;

    Vec3 result=ambient+diffuse+specular;
    return toColor(result);
}

// ============================================================
// Rasterizer
// ============================================================
struct VSOut {
    Vec4 clipPos;
    Vec3 worldPos;
    Vec3 worldNormal;
};

struct RenderObject {
    Mesh mesh;
    Mat4 model;
    Vec3 baseColor;
    int  id; // object id for edge detection
};

Mat4 viewMat, projMat;
Vec3 lightDir, lightColor, ambientColor, eyePos;

// Transform vertex to clip space
VSOut transformVertex(const Vertex& v, const Mat4& model){
    VSOut out;
    Vec4 wp=model*Vec4(v.pos,1);
    out.worldPos={wp.x/wp.w, wp.y/wp.w, wp.z/wp.w};
    Vec4 cp=projMat*(viewMat*wp);
    out.clipPos=cp;
    // Normal: assume uniform scale, use model (no transpose for simplicity in uniform-scale case)
    Vec4 wn=model*Vec4(v.normal,0);
    out.worldNormal=Vec3{wn.x,wn.y,wn.z}.norm();
    return out;
}

// NDC -> screen
Vec2 ndcToScreen(float nx, float ny){
    return {(nx*0.5f+0.5f)*W, (ny*0.5f+0.5f)*H};
}

// Edge function (signed area * 2)
float edgeFunc(Vec2 a, Vec2 b, Vec2 p){
    return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);
}

void rasterize(const RenderObject& obj){
    const auto& mesh=obj.mesh;
    for(const auto& tri:mesh.tris){
        VSOut v0=transformVertex(mesh.verts[tri.a], obj.model);
        VSOut v1=transformVertex(mesh.verts[tri.b], obj.model);
        VSOut v2=transformVertex(mesh.verts[tri.c], obj.model);

        // Clip: skip if any vertex behind near plane
        if(v0.clipPos.w<=0||v1.clipPos.w<=0||v2.clipPos.w<=0) continue;

        // Perspective divide
        float w0=v0.clipPos.w, w1=v1.clipPos.w, w2=v2.clipPos.w;
        Vec3 ndc0{v0.clipPos.x/w0, v0.clipPos.y/w0, v0.clipPos.z/w0};
        Vec3 ndc1{v1.clipPos.x/w1, v1.clipPos.y/w1, v1.clipPos.z/w1};
        Vec3 ndc2{v2.clipPos.x/w2, v2.clipPos.y/w2, v2.clipPos.z/w2};

        // NDC bounds check
        auto inRange=[](float v){ return v>=-1.5f && v<=1.5f; };
        if(!inRange(ndc0.x)||!inRange(ndc0.y)) continue;
        if(!inRange(ndc1.x)||!inRange(ndc1.y)) continue;
        if(!inRange(ndc2.x)||!inRange(ndc2.y)) continue;

        Vec2 s0=ndcToScreen(ndc0.x,ndc0.y);
        Vec2 s1=ndcToScreen(ndc1.x,ndc1.y);
        Vec2 s2=ndcToScreen(ndc2.x,ndc2.y);

        // Bounding box
        int minX=std::max(0,(int)std::floor(std::min({s0.x,s1.x,s2.x})));
        int maxX=std::min(W-1,(int)std::ceil(std::max({s0.x,s1.x,s2.x})));
        int minY=std::max(0,(int)std::floor(std::min({s0.y,s1.y,s2.y})));
        int maxY=std::min(H-1,(int)std::ceil(std::max({s0.y,s1.y,s2.y})));

        float area=edgeFunc(s0,s1,s2);
        if(std::abs(area)<1e-4f) continue;
        // Back-face culling: area>0 means counter-clockwise in screen space (front-facing)
        // We use left-handed convention after projection: discard if area<=0
        if(area<=0) continue;

        for(int py=minY;py<=maxY;py++){
            for(int px=minX;px<=maxX;px++){
                Vec2 p{(float)px+0.5f,(float)py+0.5f};
                float e0=edgeFunc(s0,s1,p);
                float e1=edgeFunc(s1,s2,p);
                float e2=edgeFunc(s2,s0,p);
                if(e0<0||e1<0||e2<0) continue;

                // Barycentric
                float b0=e1/area, b1=e2/area, b2=e0/area;

                // Perspective-correct depth
                float depth=b0*ndc0.z+b1*ndc1.z+b2*ndc2.z;
                int idx=py*W+px;
                if(depth>=depthBuf[idx]) continue;
                depthBuf[idx]=depth;

                // Perspective-correct normal interpolation
                float invW=b0/w0+b1/w1+b2/w2;
                Vec3 N=(v0.worldNormal*(b0/w0)+v1.worldNormal*(b1/w1)+v2.worldNormal*(b2/w2));
                if(invW>1e-7f) N=N*(1.0f/invW);
                N=N.norm();

                Vec3 wPos=(v0.worldPos*(b0/w0)+v1.worldPos*(b1/w1)+v2.worldPos*(b2/w2));
                if(invW>1e-7f) wPos=wPos*(1.0f/invW);

                Vec3 V=(eyePos-wPos).norm();

                fb[idx]=celShade(obj.baseColor, N, lightDir, V, lightColor, ambientColor);
                normalBuf[idx]=N;
                objBuf[idx]=obj.id;
            }
        }
    }
}

// ============================================================
// Outline pass: screen-space edge detection
// Draw outline where object-id changes or normal discontinuity
// ============================================================
void drawOutlines(Color outlineColor){
    // Copy framebuffer first
    std::vector<Color> orig=fb;

    for(int y=1;y<H-1;y++){
        for(int x=1;x<W-1;x++){
            int id=objBuf[y*W+x];
            if(id==0) continue; // background

            // Check 4-neighbors for object boundary
            bool isEdge=false;
            int neighbors[4][2]={{-1,0},{1,0},{0,-1},{0,1}};
            for(auto& n:neighbors){
                int nx=x+n[0], ny=y+n[1];
                if(objBuf[ny*W+nx]!=id){ isEdge=true; break; }
            }

            if(!isEdge){
                // Check normal discontinuity (soft edge)
                Vec3 cn=normalBuf[y*W+x];
                for(auto& n:neighbors){
                    int nx=x+n[0], ny=y+n[1];
                    if(cn.dot(normalBuf[ny*W+nx])<0.6f){ isEdge=true; break; }
                }
            }

            if(isEdge){
                fb[y*W+x]=outlineColor;
            }
        }
    }
}

// ============================================================
// Main
// ============================================================
int main(){
    clearBuffers();

    // Camera setup
    eyePos={0,2.5f,6};
    Vec3 target={0,0,0};
    Vec3 up={0,1,0};
    viewMat=makeLookAt(eyePos,target,up);
    projMat=makePerspective(0.7f,(float)W/H,0.1f,100.0f);

    // Lighting
    lightDir=Vec3{1,2,1}.norm();
    lightColor={1,1,1};
    ambientColor={0.3f,0.3f,0.5f};

    // Build objects
    // 1) Main sphere (icosphere)
    Mesh sphere=buildIcosphere(4);
    // Scale sphere to radius 1
    RenderObject mainSphere;
    mainSphere.mesh=sphere;
    mainSphere.model=makeTranslate(-1.5f,0.8f,0)*makeScale(1.0f,1.0f,1.0f);
    mainSphere.baseColor={0.9f,0.3f,0.2f}; // red
    mainSphere.id=1;

    // 2) Torus
    Mesh torus=buildTorus(40,20,0.6f,0.25f);
    RenderObject mainTorus;
    mainTorus.mesh=torus;
    mainTorus.model=makeTranslate(1.5f,0.8f,0)*makeRotX(0.4f);
    mainTorus.baseColor={0.2f,0.6f,0.9f}; // blue
    mainTorus.id=2;

    // 3) Small sphere (green accent)
    Mesh sphere2=buildIcosphere(3);
    RenderObject accentSphere;
    accentSphere.mesh=sphere2;
    accentSphere.model=makeTranslate(0,1.6f,-0.5f)*makeScale(0.5f,0.5f,0.5f);
    accentSphere.baseColor={0.3f,0.85f,0.3f}; // green
    accentSphere.id=3;

    // 4) Ground plane
    Mesh plane=buildPlane(4.0f,8);
    RenderObject ground;
    ground.mesh=plane;
    ground.model=Mat4::identity();
    ground.baseColor={0.7f,0.7f,0.5f}; // tan
    ground.id=4;

    // Rasterize (back to front or just with depth test)
    rasterize(ground);
    rasterize(mainSphere);
    rasterize(mainTorus);
    rasterize(accentSphere);

    // Outline pass
    drawOutlines({10,10,10}); // near-black outlines

    // Save
    if(!savePNG("cel_shading_output.png")){
        std::cerr<<"Failed to save PNG\n";
        return 1;
    }
    std::cout<<"Saved cel_shading_output.png ("<<W<<"x"<<H<<")\n";

    // Stats
    int objectPixels=0, outlinePixels=0, bgPixels=0;
    for(int i=0;i<W*H;i++){
        auto& c=fb[i];
        if(objBuf[i]==0) bgPixels++;
        else if(c.r<20&&c.g<20&&c.b<20) outlinePixels++;
        else objectPixels++;
    }
    std::cout<<"Background pixels: "<<bgPixels<<"\n";
    std::cout<<"Object pixels: "<<objectPixels<<"\n";
    std::cout<<"Outline pixels: "<<outlinePixels<<"\n";

    return 0;
}
