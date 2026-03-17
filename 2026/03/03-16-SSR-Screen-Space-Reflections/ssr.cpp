// SSR - Screen Space Reflections
// 实现基于屏幕空间Ray Marching的反射效果
// 技术：G-Buffer软光栅化 + 屏幕空间反射Ray March + Binary Search精化 + Roughness Fade
//
// 场景：地面镜面 + 多个彩色球体，展示反射效果
// 对比：无反射 / SSR反射 / 高光材质对比

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cassert>
#include <string>
#include <sstream>

// ============================================================
// 基础数学
// ============================================================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-()               const { return {-x,-y,-z}; }
    Vec3 operator*(float t)       const { return {x*t,  y*t,  z*t};   }
    Vec3 operator*(const Vec3& b) const { return {x*b.x,y*b.y,z*b.z}; }
    Vec3 operator/(float t)       const { return {x/t,  y/t,  z/t};   }
    Vec3& operator+=(const Vec3& b){ x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x+y*b.y+z*b.z; }
    Vec3  cross(const Vec3& b) const {
        return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x};
    }
    float length() const { return sqrtf(x*x+y*y+z*z); }
    Vec3  normalize() const { float l=length(); return l>1e-6f?(*this)/l:Vec3(0,0,1); }
    Vec3  reflect(const Vec3& n) const {
        return *this - n*(2.f*dot(n));
    }
    float& operator[](int i){ return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }
};
Vec3 operator*(float t, const Vec3& v){ return v*t; }

struct Vec4 {
    float x, y, z, w;
    Vec4(float x=0,float y=0,float z=0,float w=0):x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v, float w=1):x(v.x),y(v.y),z(v.z),w(w){}
    Vec3 xyz() const { return {x,y,z}; }
    float& operator[](int i){ return (&x)[i]; }
    float  operator[](int i) const { return (&x)[i]; }
};

struct Mat4 {
    float m[4][4];
    Mat4(){ memset(m,0,sizeof(m)); }
    static Mat4 identity(){
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) r[i]+=m[i][j]*v[j];
        return r;
    }
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
            r.m[i][j]+=m[i][k]*b.m[k][j];
        return r;
    }
};

inline float clamp(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
inline float lerp(float a, float b, float t){ return a+(b-a)*t; }
inline Vec3  lerp(Vec3 a, Vec3 b, float t){ return a+(b-a)*t; }
inline float smoothstep(float e0, float e1, float x){
    float t=clamp((x-e0)/(e1-e0),0.f,1.f);
    return t*t*(3.f-2.f*t);
}

// ============================================================
// 矩阵工具
// ============================================================
Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up){
    Vec3 f=(center-eye).normalize();
    Vec3 r=f.cross(up).normalize();
    Vec3 u=r.cross(f);
    Mat4 m=Mat4::identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]=f.dot(eye);
    return m;
}
Mat4 perspective(float fovY, float aspect, float zNear, float zFar){
    float tanH=tanf(fovY*0.5f);
    Mat4 m;
    m.m[0][0]=1.f/(aspect*tanH);
    m.m[1][1]=1.f/tanH;
    m.m[2][2]=-(zFar+zNear)/(zFar-zNear);
    m.m[2][3]=-2.f*zFar*zNear/(zFar-zNear);
    m.m[3][2]=-1.f;
    return m;
}

// ============================================================
// 图像缓冲区
// ============================================================
struct Image {
    int W, H;
    std::vector<Vec3> pixels;
    Image(int W, int H, Vec3 fill={0,0,0}): W(W),H(H),pixels(W*H,fill){}
    Vec3& at(int x, int y){ return pixels[y*W+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*W+x]; }
    Vec3 sample(float u, float v) const {  // bilinear, UV in [0,1]
        float px=u*W-0.5f, py=v*H-0.5f;
        int x0=std::max(0,(int)px), y0=std::max(0,(int)py);
        int x1=std::min(W-1,x0+1),  y1=std::min(H-1,y0+1);
        float fx=px-floorf(px), fy=py-floorf(py);
        return lerp(lerp(at(x0,y0),at(x1,y0),fx),
                    lerp(at(x0,y1),at(x1,y1),fx), fy);
    }
};

// 深度缓冲（float）
struct DepthBuffer {
    int W, H;
    std::vector<float> d;
    DepthBuffer(int W, int H): W(W),H(H),d(W*H,1e30f){}
    float& at(int x, int y){ return d[y*W+x]; }
    void clear(){ std::fill(d.begin(),d.end(),1e30f); }
};

// ============================================================
// G-Buffer（屏幕空间）
// ============================================================
struct GBuffer {
    int W, H;
    std::vector<Vec3> albedo;    // 漫反射颜色
    std::vector<Vec3> normal;    // 视空间法线
    std::vector<float> depth;    // 非线性深度 [-1,1] NDC z
    std::vector<float> roughness;// 0=镜面，1=粗糙
    std::vector<float> metalness;
    std::vector<Vec3> posVS;     // 视空间位置
    GBuffer(int W, int H): W(W),H(H),
        albedo(W*H),normal(W*H),depth(W*H,2.f),
        roughness(W*H,1.f),metalness(W*H,0.f),posVS(W*H){}
    void clear(){
        std::fill(albedo.begin(),albedo.end(),Vec3(0,0,0));
        std::fill(normal.begin(),normal.end(),Vec3(0,0,1));
        std::fill(depth.begin(),depth.end(),2.f);
        std::fill(roughness.begin(),roughness.end(),1.f);
        std::fill(metalness.begin(),metalness.end(),0.f);
        std::fill(posVS.begin(),posVS.end(),Vec3(0,0,0));
    }
    int idx(int x, int y) const { return y*W+x; }
};

// ============================================================
// 场景定义
// ============================================================
struct Material {
    Vec3  albedo;
    float roughness;
    float metalness;
    float emission;
    Material(){}
    Material(Vec3 a, float r, float m, float e=0)
        :albedo(a),roughness(r),metalness(m),emission(e){}
};

struct Sphere {
    Vec3     center;
    float    radius;
    Material mat;
};

struct TriMesh {
    struct Vertex { Vec3 pos, normal; Vec2 uv; };
    std::vector<Vertex>   verts;
    std::vector<uint32_t> indices;
    Material mat;
};

// 用三角形表示平面（地板）
TriMesh makePlane(float y, float half, Material mat){
    TriMesh m;
    m.mat=mat;
    // 两个三角形
    m.verts={
        {{-half,y,-half},{0,1,0},{0,0}},
        {{ half,y,-half},{0,1,0},{1,0}},
        {{ half,y, half},{0,1,0},{1,1}},
        {{-half,y, half},{0,1,0},{0,1}},
    };
    m.indices={0,1,2, 0,2,3};
    return m;
}

// ============================================================
// 软光栅化器（G-Buffer填充）
// ============================================================
// NDC -> 屏幕坐标
inline Vec2 ndcToScreen(float nx, float ny, int W, int H){
    return {(nx+1)*0.5f*W-0.5f, (1-ny)*0.5f*H-0.5f};
}

// 边缘函数（符号面积）
inline float edgeFunc(Vec2 a, Vec2 b, Vec2 c){
    return (c.x-a.x)*(b.y-a.y)-(c.y-a.y)*(b.x-a.x);
}

void rasterizeSphere(
    const Sphere& sph,
    const Mat4& view, const Mat4& proj,
    GBuffer& gb, DepthBuffer& zbuf)
{
    int W=gb.W, H=gb.H;
    // 球体光栅化：多个三角形细分
    const int stacks=24, slices=32;
    for(int si=0;si<stacks;si++){
        float phi0=M_PI*si/(float)stacks;
        float phi1=M_PI*(si+1)/(float)stacks;
        for(int sl=0;sl<slices;sl++){
            float th0=2*M_PI*sl/(float)slices;
            float th1=2*M_PI*(sl+1)/(float)slices;
            // 4角点（世界空间）
            Vec3 p[4];
            Vec3 n[4];
            auto sp=[&](float ph, float th)->Vec3{
                return Vec3(sinf(ph)*cosf(th), cosf(ph), sinf(ph)*sinf(th));
            };
            n[0]=sp(phi0,th0); p[0]=sph.center+n[0]*sph.radius;
            n[1]=sp(phi0,th1); p[1]=sph.center+n[1]*sph.radius;
            n[2]=sp(phi1,th1); p[2]=sph.center+n[2]*sph.radius;
            n[3]=sp(phi1,th0); p[3]=sph.center+n[3]*sph.radius;
            // 两个三角形
            int tris[2][3]={{0,1,2},{0,2,3}};
            for(auto& tri:tris){
                Vec4 vs[3],cs[3];
                Vec3 ns[3];
                bool behind=false;
                for(int k=0;k<3;k++){
                    Vec4 vw(p[tri[k]],1.f);
                    Vec4 ve=view*vw;
                    ns[k]=(view*(Vec4(n[tri[k]],0.f))).xyz().normalize();
                    if(ve.z>-0.1f){ behind=true; break; }
                    Vec4 vc=proj*ve;
                    vs[k]=ve;
                    cs[k]={vc.x/vc.w, vc.y/vc.w, vc.z/vc.w, 1.f/vc.w};
                }
                if(behind) continue;
                Vec2 s[3];
                for(int k=0;k<3;k++) s[k]=ndcToScreen(cs[k].x,cs[k].y,W,H);
                float area=edgeFunc(s[0],s[1],s[2]);
                if(area<=0) continue;
                int x0=std::max(0,(int)std::min({s[0].x,s[1].x,s[2].x}));
                int y0=std::max(0,(int)std::min({s[0].y,s[1].y,s[2].y}));
                int x1=std::min(W-1,(int)std::max({s[0].x,s[1].x,s[2].x})+1);
                int y1=std::min(H-1,(int)std::max({s[0].y,s[1].y,s[2].y})+1);
                for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++){
                    Vec2 px={x+0.5f,y+0.5f};
                    float w0=edgeFunc(s[1],s[2],px)/area;
                    float w1=edgeFunc(s[2],s[0],px)/area;
                    float w2=edgeFunc(s[0],s[1],px)/area;
                    if(w0<0||w1<0||w2<0) continue;
                    float invW=w0*cs[0].w+w1*cs[1].w+w2*cs[2].w;
                    float z=w0*cs[0].z+w1*cs[1].z+w2*cs[2].z; // NDC z
                    if(z<zbuf.at(x,y)){
                        zbuf.at(x,y)=z;
                        int id=gb.idx(x,y);
                        // 插值视空间位置
                        Vec3 vsInterp=vs[0].xyz()*(w0/invW)
                                     +vs[1].xyz()*(w1/invW)
                                     +vs[2].xyz()*(w2/invW);
                        // 插值法线
                        Vec3 nInterp=(ns[0]*w0+ns[1]*w1+ns[2]*w2).normalize();
                        gb.albedo[id]  =sph.mat.albedo;
                        gb.normal[id]  =nInterp;
                        gb.depth[id]   =z;
                        gb.roughness[id]=sph.mat.roughness;
                        gb.metalness[id]=sph.mat.metalness;
                        gb.posVS[id]   =vsInterp;
                    }
                }
            }
        }
    }
}

void rasterizeMesh(
    const TriMesh& mesh,
    const Mat4& view, const Mat4& proj,
    GBuffer& gb, DepthBuffer& zbuf)
{
    int W=gb.W, H=gb.H;
    int triCount=(int)mesh.indices.size()/3;
    for(int t=0;t<triCount;t++){
        uint32_t i0=mesh.indices[t*3],i1=mesh.indices[t*3+1],i2=mesh.indices[t*3+2];
        const auto& v0=mesh.verts[i0];
        const auto& v1=mesh.verts[i1];
        const auto& v2=mesh.verts[i2];
        Vec4 vs[3]; Vec4 cs[3]; Vec3 ns[3];
        bool behind=false;
        const TriMesh::Vertex* vv[3]={&v0,&v1,&v2};
        for(int k=0;k<3;k++){
            Vec4 vw(vv[k]->pos,1.f);
            Vec4 ve=view*vw;
            if(ve.z>-0.1f){ behind=true; break; }
            ns[k]=(view*(Vec4(vv[k]->normal,0.f))).xyz().normalize();
            Vec4 vc=proj*ve;
            vs[k]=ve;
            cs[k]={vc.x/vc.w, vc.y/vc.w, vc.z/vc.w, 1.f/vc.w};
        }
        if(behind) continue;
        Vec2 s[3];
        for(int k=0;k<3;k++) s[k]=ndcToScreen(cs[k].x,cs[k].y,W,H);
        float area=edgeFunc(s[0],s[1],s[2]);
        if(area<=0) continue;
        int x0=std::max(0,(int)std::min({s[0].x,s[1].x,s[2].x}));
        int y0=std::max(0,(int)std::min({s[0].y,s[1].y,s[2].y}));
        int x1=std::min(W-1,(int)std::max({s[0].x,s[1].x,s[2].x})+1);
        int y1=std::min(H-1,(int)std::max({s[0].y,s[1].y,s[2].y})+1);
        for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++){
            Vec2 px={x+0.5f,y+0.5f};
            float w0=edgeFunc(s[1],s[2],px)/area;
            float w1=edgeFunc(s[2],s[0],px)/area;
            float w2=edgeFunc(s[0],s[1],px)/area;
            if(w0<0||w1<0||w2<0) continue;
            float invW=w0*cs[0].w+w1*cs[1].w+w2*cs[2].w;
            float z=w0*cs[0].z+w1*cs[1].z+w2*cs[2].z;
            if(z<zbuf.at(x,y)){
                zbuf.at(x,y)=z;
                int id=gb.idx(x,y);
                Vec3 vsInterp=vs[0].xyz()*(w0/invW)
                             +vs[1].xyz()*(w1/invW)
                             +vs[2].xyz()*(w2/invW);
                Vec3 nInterp=(ns[0]*w0+ns[1]*w1+ns[2]*w2).normalize();
                gb.albedo[id]  =mesh.mat.albedo;
                gb.normal[id]  =nInterp;
                gb.depth[id]   =z;
                gb.roughness[id]=mesh.mat.roughness;
                gb.metalness[id]=mesh.mat.metalness;
                gb.posVS[id]   =vsInterp;
            }
        }
    }
}

// ============================================================
// 光照（Blinn-Phong）
// ============================================================
Vec3 shade(Vec3 posVS, Vec3 normalVS, Vec3 albedo,
           float roughness, float metalness,
           Vec3 lightDirVS, Vec3 lightColor,
           Vec3 ambientColor)
{
    // Diffuse
    float diff=std::max(0.f, normalVS.dot(lightDirVS));
    // Specular（Blinn-Phong）
    Vec3 viewDir=Vec3(0,0,0)-posVS; viewDir=viewDir.normalize();
    Vec3 H=(lightDirVS+viewDir).normalize();
    float spec=powf(std::max(0.f,normalVS.dot(H)), lerp(8.f,128.f,1.f-roughness));
    spec*=(1.f-roughness*roughness);

    Vec3 diffColor=albedo*lerp(1.f,0.f,metalness);
    Vec3 specColor=lerp(Vec3(0.04f,0.04f,0.04f),albedo,metalness);

    Vec3 result=ambientColor*albedo
                +lightColor*(diffColor*diff + specColor*spec);
    return result;
}

// ============================================================
// SSR Pass - 屏幕空间反射（线性 Ray March + Binary Search 精化）
// ============================================================
// 将视空间位置投影到屏幕UV
bool projectToScreen(Vec3 posVS, const Mat4& proj, int /*W*/, int /*H*/,
                     float& outU, float& outV, float& outNDCZ)
{
    Vec4 clip=proj*Vec4(posVS,1.f);
    if(clip.w<=0) return false;
    float ndcX=clip.x/clip.w, ndcY=clip.y/clip.w, ndcZ=clip.z/clip.w;
    if(ndcX<-1||ndcX>1||ndcY<-1||ndcY>1||ndcZ<-1||ndcZ>1) return false;
    outU=(ndcX+1)*0.5f;
    outV=(1-ndcY)*0.5f;
    outNDCZ=ndcZ;
    return true;
}

// 从屏幕UV采样G-Buffer深度并转换为NDC Z
float sampleDepth(const GBuffer& gb, float u, float v){
    int x=clamp(u*gb.W,0.f,(float)(gb.W-1));
    int y=clamp(v*gb.H,0.f,(float)(gb.H-1));
    return gb.depth[gb.idx((int)x,(int)y)];
}

// 线性步进 + Binary Search精化
// 返回：是否击中，以及击中的屏幕UV
bool ssrTrace(
    Vec3 posVS,   // 起点（视空间）
    Vec3 refVS,   // 反射方向（视空间，已归一化）
    const GBuffer& gb,
    const Mat4& proj,
    float& hitU, float& hitV,
    float startDepthNDC)  // 起点的 NDC 深度，用于自交检测
{
    const int LINEAR_STEPS=64;
    const int BINARY_STEPS=8;
    const float maxDist=10.f;
    const float thickness=0.12f; // 厚度：比球体半径小一个量级

    float stepLen=maxDist/(float)LINEAR_STEPS;
    Vec3  step=refVS*stepLen;

    // 起点偏移：固定偏移改为步长的1.5倍，避免自交同时与场景尺度适配
    Vec3  curPos=posVS+refVS*(stepLen*1.5f);

    float lastU=0, lastV=0;
    (void)lastU; (void)lastV;

    for(int i=0;i<LINEAR_STEPS;i++){
        float u,v,ndcZ;
        if(!projectToScreen(curPos,proj,gb.W,gb.H,u,v,ndcZ)){
            curPos=curPos+step;
            continue;
        }
        // 采样G-Buffer深度
        float gbDepth=sampleDepth(gb,u,v);
        if(gbDepth>=2.f){ // 未覆盖（背景）
            lastU=u; lastV=v;
            curPos=curPos+step;
            continue;
        }
        float depthDiff=ndcZ-gbDepth; // >0 表示光线在物体后面
        if(depthDiff>0 && depthDiff<thickness){
            // 自交过滤：命中点深度不能与起点深度太接近
            // startDepthNDC 是着色点自身的 NDC 深度（负值，越大越近相机）
            // gbDepth 是命中处 G-Buffer 的深度
            // 如果命中点深度与起点深度差值 < 0.05，说明是自身表面，跳过
            if(std::abs(gbDepth - startDepthNDC) < 0.05f){
                lastU=u; lastV=v;
                curPos=curPos+step;
                continue;
            }
            // 命中！做 Binary Search 精化
            Vec3 lo=curPos-step, hi=curPos;
            for(int b=0;b<BINARY_STEPS;b++){
                Vec3 mid=(lo+hi)*0.5f;
                float mu,mv,mndcZ;
                if(!projectToScreen(mid,proj,gb.W,gb.H,mu,mv,mndcZ)){
                    hi=mid; continue;
                }
                float mGbDepth=sampleDepth(gb,mu,mv);
                if(mndcZ-mGbDepth>0 && mndcZ-mGbDepth<thickness*2)
                    hi=mid;
                else
                    lo=mid;
            }
            // 最终点
            float fu,fv,fndcZ;
            if(projectToScreen(hi,proj,gb.W,gb.H,fu,fv,fndcZ)){
                hitU=fu; hitV=fv;
                return true;
            }
        }
        lastU=u; lastV=v;
        curPos=curPos+step;
    }
    return false;
}

// ============================================================
// Fresnel近似
// ============================================================
float fresnelSchlick(float cosTheta, float F0){
    return F0+(1.f-F0)*powf(1.f-clamp(cosTheta,0.f,1.f),5.f);
}

// ============================================================
// ACES 色调映射
// ============================================================
Vec3 aces(Vec3 x){
    float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;
    Vec3 num=x*(x*a+Vec3(b,b,b));
    Vec3 den=x*(x*c+Vec3(d,d,d))+Vec3(e,e,e);
    x=Vec3(num.x/den.x, num.y/den.y, num.z/den.z);
    x.x=clamp(x.x,0,1);x.y=clamp(x.y,0,1);x.z=clamp(x.z,0,1);
    return x;
}

// gamma correction
Vec3 gammaEncode(Vec3 c){ 
    c.x=powf(clamp(c.x,0,1),1.f/2.2f);
    c.y=powf(clamp(c.y,0,1),1.f/2.2f);
    c.z=powf(clamp(c.z,0,1),1.f/2.2f);
    return c;
}

// ============================================================
// PNG 写入（不依赖第三方库，使用 stb_image_write 风格）
// ============================================================
// 使用 stb_image_write.h（单头文件）
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void saveImage(const Image& img, const std::string& filename){
    std::vector<uint8_t> data(img.W*img.H*3);
    for(int y=0;y<img.H;y++) for(int x=0;x<img.W;x++){
        const Vec3& c=img.at(x,y);
        int idx=(y*img.W+x)*3;
        data[idx+0]=(uint8_t)clamp(c.x*255.f,0.f,255.f);
        data[idx+1]=(uint8_t)clamp(c.y*255.f,0.f,255.f);
        data[idx+2]=(uint8_t)clamp(c.z*255.f,0.f,255.f);
    }
    stbi_write_png(filename.c_str(),img.W,img.H,3,data.data(),img.W*3);
    printf("  Saved: %s\n",filename.c_str());
}

// ============================================================
// 主函数
// ============================================================
int main(){
    const int W=800, H=600;
    printf("=== SSR - Screen Space Reflections ===\n");
    printf("  Resolution: %dx%d\n",W,H);

    // --- 相机设置 ---
    Vec3 camPos(0,3.5f,7.f);
    Vec3 camTarget(0,0,0);
    Vec3 camUp(0,1,0);
    Mat4 view=lookAt(camPos,camTarget,camUp);
    Mat4 proj=perspective(45.f*M_PI/180.f,(float)W/(float)H,0.5f,50.f);

    // --- 场景构建 ---
    // 材质
    Material matFloor({0.8f,0.8f,0.9f},  0.02f,0.0f);   // 近镜面地板
    Material matBallRed({0.9f,0.2f,0.15f}, 0.05f,0.8f);  // 红色金属球
    Material matBallGreen({0.15f,0.8f,0.3f},0.08f,0.0f); // 绿色漫反射球
    Material matBallBlue({0.2f,0.4f,0.95f}, 0.1f,0.5f);  // 蓝色半金属球
    Material matBallGold({0.95f,0.8f,0.1f}, 0.03f,1.0f); // 金色金属球
    Material matBallWhite({0.95f,0.95f,0.95f},0.15f,0.0f);// 白色光滑球

    // 球体
    std::vector<Sphere> spheres={
        {{-2.5f,1.0f,-1.0f},1.0f,matBallRed},
        {{ 0.0f,0.8f,-0.5f},0.8f,matBallGreen},
        {{ 2.5f,1.0f,-1.0f},1.0f,matBallBlue},
        {{-1.2f,0.6f, 1.5f},0.6f,matBallGold},
        {{ 1.2f,0.6f, 1.5f},0.6f,matBallWhite},
    };
    // 地板
    TriMesh floor=makePlane(0.f,8.f,matFloor);

    // --- 光源（视空间）---
    // 主光源（右上前）
    Vec3 lightWS=Vec3(3,6,4).normalize();
    Vec4 lightVS4=view*(Vec4(lightWS,0.f));
    Vec3 lightVS=lightVS4.xyz().normalize();
    Vec3 lightColor(1.2f,1.1f,1.0f);
    Vec3 ambientColor(0.08f,0.08f,0.12f);

    // ==========================================================
    // Pass 1: G-Buffer 填充
    // ==========================================================
    printf("\n[Pass 1] Building G-Buffer...\n");
    GBuffer gb(W,H);
    DepthBuffer zbuf(W,H);
    gb.clear();

    // 光栅化地板
    rasterizeMesh(floor, view, proj, gb, zbuf);
    // 光栅化球体
    for(const auto& s:spheres) rasterizeSphere(s,view,proj,gb,zbuf);
    printf("  G-Buffer filled.\n");

    // ==========================================================
    // Pass 2: 光照 Pass（不含反射）
    // ==========================================================
    printf("[Pass 2] Lighting pass...\n");
    Image litNoSSR(W,H,{0.15f,0.18f,0.25f});  // 天空背景
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        int id=gb.idx(x,y);
        if(gb.depth[id]>=2.f) continue; // 背景
        Vec3 color=shade(gb.posVS[id],gb.normal[id],gb.albedo[id],
                         gb.roughness[id],gb.metalness[id],
                         lightVS,lightColor,ambientColor);
        litNoSSR.at(x,y)=color;
    }
    printf("  Lighting pass done.\n");

    // ==========================================================
    // Pass 3: SSR Pass
    // ==========================================================
    printf("[Pass 3] SSR pass (Ray Marching)...\n");
    Image ssrBuffer(W,H,{0,0,0});   // 反射颜色
    Image ssrMask(W,H,{0,0,0});     // 反射权重
    int hitCount=0;

    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        int id=gb.idx(x,y);
        if(gb.depth[id]>=2.f) continue;  // 背景
        float roughness=gb.roughness[id];
        if(roughness>0.5f) continue;     // 只对光滑表面做SSR

        Vec3 posVS=gb.posVS[id];
        Vec3 normVS=gb.normal[id];
        Vec3 viewVS=(Vec3(0,0,0)-posVS).normalize();

        // 反射方向（视空间）
        Vec3 reflVS=(-viewVS).reflect(normVS);
        if(reflVS.dot(normVS)<0) continue; // 背面反射跳过

        // 获取起点的 NDC 深度，用于自交过滤
        float startU0, startV0, startNdcZ;
        projectToScreen(posVS, proj, gb.W, gb.H, startU0, startV0, startNdcZ);

        float hitU,hitV;
        if(ssrTrace(posVS,reflVS,gb,proj,hitU,hitV,startNdcZ)){
            // 采样击中点颜色
            int hx=clamp(hitU*W,0.f,(float)(W-1));
            int hy=clamp(hitV*H,0.f,(float)(H-1));
            Vec3 reflColor=litNoSSR.at((int)hx,(int)hy);

            // Fresnel
            float F0=lerp(0.04f,0.95f,gb.metalness[id]);
            float cosTheta=viewVS.dot(normVS);
            float fresnel=fresnelSchlick(cosTheta,F0);
            fresnel=std::max(fresnel,0.05f);

            // 边缘衰减（屏幕边缘反射减弱）
            float edgeFadeU=smoothstep(0,0.1f,hitU)*smoothstep(1,0.9f,hitU);
            float edgeFadeV=smoothstep(0,0.1f,hitV)*smoothstep(1,0.9f,hitV);
            float edgeFade=edgeFadeU*edgeFadeV;

            // 粗糙度衰减
            float roughFade=smoothstep(0.5f,0.f,roughness);

            float weight=fresnel*edgeFade*roughFade;
            ssrBuffer.at(x,y)=reflColor;
            ssrMask.at(x,y)=Vec3(weight,weight,weight);
            hitCount++;
        }
    }
    printf("  SSR hits: %d pixels\n",hitCount);

    // ==========================================================
    // Pass 4: 合成（带SSR的最终图）
    // ==========================================================
    printf("[Pass 4] Compositing SSR...\n");
    Image finalSSR(W,H,{0.15f,0.18f,0.25f});
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        Vec3 lit=litNoSSR.at(x,y);
        float w=ssrMask.at(x,y).x;
        Vec3 refl=ssrBuffer.at(x,y);
        Vec3 combined=lit*(1.f-w)+refl*w;
        finalSSR.at(x,y)=combined;
    }
    printf("  Compositing done.\n");

    // ==========================================================
    // ACES + Gamma + 保存图片
    // ==========================================================
    printf("[Save] Writing images...\n");

    // 1. 无SSR版本
    Image outNoSSR(W,H);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        outNoSSR.at(x,y)=gammaEncode(aces(litNoSSR.at(x,y)));
    saveImage(outNoSSR,"ssr_no_reflection.png");

    // 2. 有SSR版本
    Image outSSR(W,H);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++)
        outSSR.at(x,y)=gammaEncode(aces(finalSSR.at(x,y)));
    saveImage(outSSR,"ssr_output.png");

    // 3. SSR遮罩可视化
    Image outMask(W,H);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        float w=ssrMask.at(x,y).x;
        outMask.at(x,y)={w,w,w};
    }
    saveImage(outMask,"ssr_reflection_mask.png");

    // 4. 对比图（左=无SSR，右=有SSR）
    Image outComp(W*2,H);
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            outComp.at(x,y)=outNoSSR.at(x,y);
            outComp.at(x+W,y)=outSSR.at(x,y);
        }
    }
    // 分割线
    for(int y=0;y<H;y++){
        outComp.at(W,y)={1,1,0};
        outComp.at(W-1,y)={1,1,0};
    }
    saveImage(outComp,"ssr_comparison.png");

    // ==========================================================
    // 量化验证
    // ==========================================================
    printf("\n=== Quantitative Validation ===\n");

    // 1. 检查文件大小（非空）
    FILE* f=fopen("ssr_output.png","rb");
    if(!f){ fprintf(stderr,"ERROR: ssr_output.png not found!\n"); return 1; }
    fseek(f,0,SEEK_END); long sz=ftell(f); fclose(f);
    printf("  ssr_output.png size: %ld bytes\n",sz);
    assert(sz>10000 && "Output file too small!");

    // 2. 检查SSR命中率（地板是镜面，应该有大量命中）
    printf("  SSR hit pixels: %d\n",hitCount);
    assert(hitCount>5000 && "Too few SSR hits! Ground reflection likely missing.");

    // 3. 像素级验证：检查SSR改变了颜色（在高mask权重区域）
    // 检查SSR mask>0.2的像素，看是否有颜色变化（合成有效）
    float totalAbsDiff=0.f;
    int ssrActivePixels=0;
    for(int y=0;y<H;y++){
        for(int x=0;x<W;x++){
            float w=outMask.at(x,y).x;
            if(w>0.2f){
                Vec3 c1=outSSR.at(x,y);
                Vec3 c2=outNoSSR.at(x,y);
                float diff=fabsf((c1.x+c1.y+c1.z)/3.f - (c2.x+c2.y+c2.z)/3.f);
                totalAbsDiff+=diff;
                ssrActivePixels++;
            }
        }
    }
    float avgDiff = ssrActivePixels>0 ? totalAbsDiff/ssrActivePixels : 0.f;
    printf("  SSR active pixels (mask>0.2): %d, avg color change: %.4f\n",
           ssrActivePixels, avgDiff);
    assert(ssrActivePixels>1000 && "Too few SSR active pixels!");
    assert(avgDiff>0.001f && "SSR compositing has no effect!");

    // 4. 检查SSR遮罩非全零
    float maskSum=0;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) maskSum+=outMask.at(x,y).x;
    printf("  SSR mask sum: %.1f (pixels with reflection)\n",maskSum);
    assert(maskSum>100.f && "SSR mask is empty!");

    printf("\n✅ All validations passed!\n");
    printf("  - ssr_no_reflection.png  : Scene without reflections\n");
    printf("  - ssr_output.png         : Scene with SSR\n");
    printf("  - ssr_reflection_mask.png: Fresnel reflection weights\n");
    printf("  - ssr_comparison.png     : Side-by-side comparison\n");
    return 0;
}
