/*
 * SSAO - Screen Space Ambient Occlusion (修复版)
 *
 * 核心修复：
 * 1. 遮蔽判断方向：view space Z 为负，sampleDepth < samplePos.z 才是遮蔽
 * 2. rangeCheck 用固定 SSAO_RADIUS 做衰减，不能用动态差值
 * 3. 场景更紧凑，物体互相靠近，SSAO 能覆盖到缝隙
 *
 * 编译: g++ -O2 -std=c++17 ssao.cpp -o ssao
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <random>
#include <iostream>
#include <cassert>

using namespace std;

// ============================================================
// 数学工具
// ============================================================
struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    float length() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 normalize() const { float l=length(); if(l<1e-9f) return {0,1,0}; return *this/l; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    float& operator[](int i){ return i==0?x:(i==1?y:z); }
    float  operator[](int i) const { return i==0?x:(i==1?y:z); }
};
Vec3 operator*(float t, const Vec3& v){ return v*t; }
Vec3 lerp(const Vec3& a, const Vec3& b, float t){ return a*(1-t)+b*t; }
float clamp01(float v){ return max(0.0f, min(1.0f, v)); }
float smoothstep(float edge0, float edge1, float x){
    float t = clamp01((x-edge0)/(edge1-edge0));
    return t*t*(3-2*t);
}

struct Vec4 {
    float x,y,z,w;
    Vec4(float x=0,float y=0,float z=0,float w=0): x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v, float w): x(v.x),y(v.y),z(v.z),w(w){}
    Vec3 xyz() const { return {x,y,z}; }
    float& operator[](int i){ float* p=&x; return p[i]; }
    float  operator[](int i) const { const float* p=&x; return p[i]; }
};

struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity(){
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        for(int i=0;i<4;i++)
            r[i]=m[i][0]*v.x+m[i][1]*v.y+m[i][2]*v.z+m[i][3]*v.w;
        return r;
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++){
                r.m[i][j]=0;
                for(int k=0;k<4;k++) r.m[i][j]+=m[i][k]*o.m[k][j];
            }
        return r;
    }
};

Mat4 perspective(float fovy, float aspect, float near_, float far_){
    Mat4 r;
    float f = 1.0f/tanf(fovy*0.5f);
    r.m[0][0] = f/aspect;
    r.m[1][1] = f;
    r.m[2][2] = (far_+near_)/(near_-far_);
    r.m[2][3] = (2*far_*near_)/(near_-far_);
    r.m[3][2] = -1;
    return r;
}

Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up){
    Vec3 f = (center-eye).normalize();
    Vec3 s = f.cross(up).normalize();
    Vec3 u = s.cross(f);
    Mat4 r = Mat4::identity();
    r.m[0][0]=s.x; r.m[0][1]=s.y; r.m[0][2]=s.z;
    r.m[1][0]=u.x; r.m[1][1]=u.y; r.m[1][2]=u.z;
    r.m[2][0]=-f.x;r.m[2][1]=-f.y;r.m[2][2]=-f.z;
    r.m[0][3]=-(s.dot(eye));
    r.m[1][3]=-(u.dot(eye));
    r.m[2][3]= (f.dot(eye));
    return r;
}

// ============================================================
// G-Buffer
// ============================================================
const int W = 800, H = 600;

struct GBuffer {
    Vec3  vsPos[W*H];    // view-space 位置
    Vec3  vsNorm[W*H];   // view-space 法线（已归一化）
    Vec3  albedo[W*H];
    bool  valid[W*H];

    void clear(){
        for(int i=0;i<W*H;i++){
            vsPos[i]  = {};
            vsNorm[i] = {0,1,0};
            albedo[i] = {0.1f,0.1f,0.1f};
            valid[i]  = false;
        }
    }
} gbuf;

float zbuf[W*H]; // NDC depth，越小越近

// ============================================================
// 变换矩阵（全局）
// ============================================================
Mat4 viewMat, projMat;

void initMatrices(){
    // 相机站在场景正前方开口处，往 -Z 方向看进 Cornell Box
    viewMat = lookAt({0.0f, 2.0f, 5.5f}, {0.0f, 1.8f, 0.0f}, {0,1,0});
    projMat = perspective((float)M_PI/3.0f, (float)W/H, 0.1f, 25.0f);
}

// 世界坐标 → view 空间
Vec3 toView(Vec3 p){
    Vec4 v = viewMat * Vec4(p,1.0f);
    return v.xyz();
}

// 世界法线 → view 空间（view 矩阵正交，直接用即可）
Vec3 toViewNormal(Vec3 n){
    Vec4 v = viewMat * Vec4(n,0.0f);
    return v.xyz().normalize();
}

// view 空间位置 → clip → NDC → 屏幕像素
bool projectToPixel(Vec3 vsPos, int& px, int& py, float& ndcZ){
    Vec4 cp = projMat * Vec4(vsPos, 1.0f);
    if(cp.w < 1e-6f) return false;
    ndcZ = cp.z / cp.w;
    if(ndcZ < -1.0f || ndcZ > 1.0f) return false;
    float ndcX = cp.x / cp.w;
    float ndcY = cp.y / cp.w;
    px = (int)((ndcX+1)*0.5f*(W-1) + 0.5f);
    py = (int)((1-ndcY)*0.5f*(H-1) + 0.5f);
    return px>=0 && px<W && py>=0 && py<H;
}

// ============================================================
// 光栅化
// ============================================================
struct Vertex { Vec3 pos, normal, color; };

bool barycentric(float x,float y,
                 float x0,float y0,float x1,float y1,float x2,float y2,
                 float& u,float& v,float& w){
    float d = (y1-y2)*(x0-x2)+(x2-x1)*(y0-y2);
    if(fabsf(d)<1e-9f) return false;
    u = ((y1-y2)*(x-x2)+(x2-x1)*(y-y2))/d;
    v = ((y2-y0)*(x-x2)+(x0-x2)*(y-y2))/d;
    w = 1-u-v;
    return u>=-1e-5f && v>=-1e-5f && w>=-1e-5f;
}

void rasterize(const Vertex& v0, const Vertex& v1, const Vertex& v2){
    // world → view
    Vec3 vs[3] = { toView(v0.pos), toView(v1.pos), toView(v2.pos) };
    Vec3 vn[3] = { toViewNormal(v0.normal), toViewNormal(v1.normal), toViewNormal(v2.normal) };

    // view → clip → NDC
    Vec4 cs[3];
    for(int i=0;i<3;i++){
        cs[i] = projMat * Vec4(vs[i], 1.0f);
        if(fabsf(cs[i].w)<1e-6f) return;
    }
    Vec3 ndc[3];
    for(int i=0;i<3;i++) ndc[i] = {cs[i].x/cs[i].w, cs[i].y/cs[i].w, cs[i].z/cs[i].w};

    // NDC → screen
    float sx[3], sy[3];
    for(int i=0;i<3;i++){
        sx[i] = (ndc[i].x+1)*0.5f*(W-1);
        sy[i] = (1-ndc[i].y)*0.5f*(H-1);
    }

    // bounding box
    int minX = max(0, (int)min({sx[0],sx[1],sx[2]}));
    int maxX = min(W-1, (int)max({sx[0],sx[1],sx[2]})+1);
    int minY = max(0, (int)min({sy[0],sy[1],sy[2]}));
    int maxY = min(H-1, (int)max({sy[0],sy[1],sy[2]})+1);

    for(int py=minY;py<=maxY;py++)
    for(int px=minX;px<=maxX;px++){
        float u,v,w;
        if(!barycentric(px+0.5f,py+0.5f,sx[0],sy[0],sx[1],sy[1],sx[2],sy[2],u,v,w)) continue;
        float z = u*ndc[0].z + v*ndc[1].z + w*ndc[2].z;
        int idx = py*W+px;
        if(z >= zbuf[idx]) continue;
        zbuf[idx] = z;
        gbuf.vsPos[idx]  = vs[0]*u + vs[1]*v + vs[2]*w;
        gbuf.vsNorm[idx] = (vn[0]*u + vn[1]*v + vn[2]*w).normalize();
        gbuf.albedo[idx] = v0.color*u + v1.color*v + v2.color*w;
        gbuf.valid[idx]  = true;
    }
}

// ============================================================
// 几何生成（Cornell Box 风格紧凑场景）
// ============================================================
void addPlane(vector<Vertex>& t, Vec3 origin, Vec3 right, Vec3 fwd,
              float W_, float D_, Vec3 col, int nx=8, int nz=8){
    Vec3 n = right.cross(fwd).normalize();
    // 法线确保朝上/朝内
    float dw=W_/nx, dd=D_/nz;
    for(int j=0;j<nz;j++) for(int i=0;i<nx;i++){
        Vec3 p00=origin+right*(i*dw)+fwd*(j*dd);
        Vec3 p10=origin+right*((i+1)*dw)+fwd*(j*dd);
        Vec3 p11=origin+right*((i+1)*dw)+fwd*((j+1)*dd);
        Vec3 p01=origin+right*(i*dw)+fwd*((j+1)*dd);
        t.push_back({p00,n,col}); t.push_back({p10,n,col}); t.push_back({p11,n,col});
        t.push_back({p00,n,col}); t.push_back({p11,n,col}); t.push_back({p01,n,col});
    }
}

void addSphere(vector<Vertex>& t, Vec3 c, float r, Vec3 col, int st=24, int sl=32){
    for(int i=0;i<st;i++) for(int j=0;j<sl;j++){
        float p0=(float)M_PI*i/st,     p1=(float)M_PI*(i+1)/st;
        float t0=2*(float)M_PI*j/sl,   t1=2*(float)M_PI*(j+1)/sl;
        auto pt=[&](float p, float tt)->Vec3{
            return {c.x+r*sinf(p)*cosf(tt), c.y+r*cosf(p), c.z+r*sinf(p)*sinf(tt)};};
        auto nm=[&](float p, float tt)->Vec3{
            return Vec3{sinf(p)*cosf(tt),cosf(p),sinf(p)*sinf(tt)}.normalize();};
        Vec3 p00=pt(p0,t0),p10=pt(p0,t1),p01=pt(p1,t0),p11=pt(p1,t1);
        Vec3 n00=nm(p0,t0),n10=nm(p0,t1),n01=nm(p1,t0),n11=nm(p1,t1);
        if(i>0){t.push_back({p00,n00,col});t.push_back({p10,n10,col});t.push_back({p11,n11,col});}
        if(i<st-1){t.push_back({p00,n00,col});t.push_back({p11,n11,col});t.push_back({p01,n01,col});}
    }
}

void addBox(vector<Vertex>& t, Vec3 c, float w, float h, float d, Vec3 col){
    float hw=w/2,hh=h/2,hd=d/2;
    // 6个面，每面2个三角形
    struct FaceDef{Vec3 n; Vec3 p[4];};
    FaceDef faces[6]={
        {{0,0, 1},{{c.x-hw,c.y-hh,c.z+hd},{c.x+hw,c.y-hh,c.z+hd},{c.x+hw,c.y+hh,c.z+hd},{c.x-hw,c.y+hh,c.z+hd}}},
        {{0,0,-1},{{c.x+hw,c.y-hh,c.z-hd},{c.x-hw,c.y-hh,c.z-hd},{c.x-hw,c.y+hh,c.z-hd},{c.x+hw,c.y+hh,c.z-hd}}},
        {{ 1,0,0},{{c.x+hw,c.y-hh,c.z+hd},{c.x+hw,c.y-hh,c.z-hd},{c.x+hw,c.y+hh,c.z-hd},{c.x+hw,c.y+hh,c.z+hd}}},
        {{-1,0,0},{{c.x-hw,c.y-hh,c.z-hd},{c.x-hw,c.y-hh,c.z+hd},{c.x-hw,c.y+hh,c.z+hd},{c.x-hw,c.y+hh,c.z-hd}}},
        {{0, 1,0},{{c.x-hw,c.y+hh,c.z+hd},{c.x+hw,c.y+hh,c.z+hd},{c.x+hw,c.y+hh,c.z-hd},{c.x-hw,c.y+hh,c.z-hd}}},
        {{0,-1,0},{{c.x-hw,c.y-hh,c.z-hd},{c.x+hw,c.y-hh,c.z-hd},{c.x+hw,c.y-hh,c.z+hd},{c.x-hw,c.y-hh,c.z+hd}}},
    };
    for(auto& f:faces){
        t.push_back({f.p[0],f.n,col}); t.push_back({f.p[1],f.n,col}); t.push_back({f.p[2],f.n,col});
        t.push_back({f.p[0],f.n,col}); t.push_back({f.p[2],f.n,col}); t.push_back({f.p[3],f.n,col});
    }
}

// ============================================================
// SSAO
// ============================================================
const int   KERNEL_SIZE  = 64;
const int   NOISE_DIM    = 4;
const float SSAO_RADIUS  = 0.8f;   // view-space 采样半径
const float SSAO_BIAS    = 0.12f;  // 防自遮蔽，约 RADIUS*0.15

Vec3  kernel[KERNEL_SIZE];
Vec3  noise[NOISE_DIM*NOISE_DIM];
float aoRaw[W*H];
float aoBlur[W*H];

void buildKernel(mt19937& rng){
    uniform_real_distribution<float> rnd(0,1), rnd11(-1,1);
    for(int i=0;i<KERNEL_SIZE;i++){
        // 半球内随机向量（z > 0，朝法线方向）
        Vec3 s={rnd11(rng), rnd11(rng), rnd(rng)};
        s = s.normalize() * rnd(rng);
        // 让样本点聚集在原点附近（更有效的遮蔽采样）
        float scale = float(i)/KERNEL_SIZE;
        scale = 0.1f + scale*scale*0.9f;
        kernel[i] = s * scale;
    }
}

void buildNoise(mt19937& rng){
    uniform_real_distribution<float> rnd11(-1,1);
    for(int i=0;i<NOISE_DIM*NOISE_DIM;i++)
        noise[i] = Vec3(rnd11(rng), rnd11(rng), 0).normalize();
}

void computeSSAO(){
    mt19937 rng(12345);
    buildKernel(rng);
    buildNoise(rng);

    // 调试：打印中心像素的采样情况

    for(int py=0;py<H;py++)
    for(int px=0;px<W;px++){
        int idx=py*W+px;
        if(!gbuf.valid[idx]){ aoRaw[idx]=1.0f; continue; }
        
        // 找第一个有效像素打印调试信息

        Vec3 fragPos  = gbuf.vsPos[idx];
        Vec3 normal   = gbuf.vsNorm[idx];

        // 从噪声纹理取随机旋转轴（tile 4×4）
        Vec3 randVec = noise[(py%NOISE_DIM)*NOISE_DIM + (px%NOISE_DIM)];

        // Gram-Schmidt 构造 TBN 矩阵（切线空间 → view 空间）
        Vec3 tangent   = (randVec - normal*normal.dot(randVec)).normalize();
        Vec3 bitangent = normal.cross(tangent);
        // TBN 列向量：tangent, bitangent, normal

        float occlusion = 0.0f;
        for(int i=0;i<KERNEL_SIZE;i++){
            // 核样本从切线空间 → view 空间
            Vec3 s = kernel[i];
            Vec3 sampleVS = tangent*s.x + bitangent*s.y + normal*s.z;
            Vec3 samplePos = fragPos + sampleVS * SSAO_RADIUS;

            // 将采样点投影到屏幕，得到对应的 G-Buffer 像素
            int sx, sy; float ndcZ;
            if(!projectToPixel(samplePos, sx, sy, ndcZ)) continue;

            int sidx = sy*W+sx;
            if(!gbuf.valid[sidx]) continue;

            // 取该屏幕像素实际的 view-space Z
            float sceneZ = gbuf.vsPos[sidx].z;

            // 遮蔽判断（view space Z，越负越远）：
            // 如果真实表面 sceneZ > samplePos.z + BIAS：
            //   真实表面比采样点更靠近相机 → samplePos 在表面后方 → 被遮蔽
            // BIAS 需要 >= RADIUS * 0.3，防止同平面自遮蔽
            float occluded = (sceneZ > samplePos.z + SSAO_BIAS) ? 1.0f : 0.0f;

            // 范围衰减
            float dist = fabsf(fragPos.z - sceneZ);
            float rangeCheck = 1.0f - smoothstep(0.0f, SSAO_RADIUS, dist);

            occlusion += occluded * rangeCheck;
        }

        // AO 值：1.0 = 无遮蔽（亮），0.0 = 完全遮蔽（暗）
        aoRaw[idx] = 1.0f - (occlusion / KERNEL_SIZE);
    }
}

void blurAO(){
    // 4×4 box blur
    const int R=2;
    for(int py=0;py<H;py++)
    for(int px=0;px<W;px++){
        float sum=0; int n=0;
        for(int dy=-R;dy<=R;dy++)
        for(int dx=-R;dx<=R;dx++){
            int nx=px+dx, ny=py+dy;
            if(nx<0||nx>=W||ny<0||ny>=H) continue;
            sum+=aoRaw[ny*W+nx]; n++;
        }
        aoBlur[py*W+px] = n>0 ? sum/n : aoRaw[py*W+px];
    }
}

// ============================================================
// 渲染着色
// ============================================================
float clampf(float v,float lo,float hi){ return max(lo,min(hi,v)); }

Vec3 shade(Vec3 vsPos, Vec3 vsNorm, Vec3 albedo, float ao){
    // 光源（view 空间）
    Vec3 lightVS = (viewMat * Vec4{5.0f,8.0f,4.0f,1.0f}).xyz();
    Vec3 L = (lightVS - vsPos).normalize();
    Vec3 V = (-vsPos).normalize();
    Vec3 H = (L+V).normalize();

    float diff = max(0.0f, vsNorm.dot(L));
    float spec = powf(max(0.0f, vsNorm.dot(H)), 64.0f);

    // 环境光乘以 AO（SSAO 只影响环境光，不影响直接光）
    // ambient 权重调高以突显 AO 效果
    Vec3 ambient  = albedo * 0.55f * ao;  // 无AO时 ambient=0.55，有AO时最低0.55*minAO
    Vec3 diffuse  = albedo * diff  * 0.65f;
    Vec3 specular = Vec3(1,1,1) * spec * 0.25f;

    return ambient + diffuse + specular;
}

Vec3 aces(Vec3 c){
    float a=2.51f,b=0.03f,cc=2.43f,d=0.59f,e=0.14f;
    auto f=[&](float x){ return clampf((x*(a*x+b))/(x*(cc*x+d)+e),0,1); };
    return {f(c.x),f(c.y),f(c.z)};
}

unsigned char toU8(float v){ return (unsigned char)(clampf(v,0,1)*255.f+0.5f); }

// 写出图像
void writeImage(const char* path, bool useAO){
    vector<unsigned char> buf(W*H*3);
    for(int py=0;py<H;py++)
    for(int px=0;px<W;px++){
        int i=py*W+px;
        Vec3 col;
        if(gbuf.valid[i]){
            float ao = useAO ? aoBlur[i] : 1.0f;
            col = shade(gbuf.vsPos[i], gbuf.vsNorm[i], gbuf.albedo[i], ao);
            col = aces(col);
            // gamma
            col = {powf(clampf(col.x,0,1),1/2.2f),
                   powf(clampf(col.y,0,1),1/2.2f),
                   powf(clampf(col.z,0,1),1/2.2f)};
        } else {
            // 天空背景
            float t=(float)py/H;
            col = lerp(Vec3(0.55f,0.70f,0.95f), Vec3(0.15f,0.25f,0.55f), t);
        }
        buf[i*3+0]=toU8(col.x);
        buf[i*3+1]=toU8(col.y);
        buf[i*3+2]=toU8(col.z);
    }
    stbi_write_png(path, W, H, 3, buf.data(), W*3);
    printf("✅ %s\n", path);
}

void writeAOMap(const char* path){
    vector<unsigned char> buf(W*H*3);
    for(int i=0;i<W*H;i++){
        unsigned char v=toU8(gbuf.valid[i] ? aoBlur[i] : 1.0f);
        buf[i*3+0]=buf[i*3+1]=buf[i*3+2]=v;
    }
    stbi_write_png(path, W, H, 3, buf.data(), W*3);
    printf("✅ %s\n", path);
}

void writeComparison(const char* path){
    int CW=W*2;
    vector<unsigned char> buf(CW*H*3);
    // 左：无AO，右：有AO
    vector<vector<unsigned char>> imgs(2);
    for(int k=0;k<2;k++){
        imgs[k].resize(W*H*3);
        for(int py=0;py<H;py++)
        for(int px=0;px<W;px++){
            int i=py*W+px;
            Vec3 col;
            if(gbuf.valid[i]){
                float ao = k==1 ? aoBlur[i] : 1.0f;
                col=shade(gbuf.vsPos[i],gbuf.vsNorm[i],gbuf.albedo[i],ao);
                col=aces(col);
                col={powf(clampf(col.x,0,1),1/2.2f),
                     powf(clampf(col.y,0,1),1/2.2f),
                     powf(clampf(col.z,0,1),1/2.2f)};
            } else {
                float t=(float)py/H;
                col=lerp(Vec3(0.55f,0.70f,0.95f),Vec3(0.15f,0.25f,0.55f),t);
            }
            imgs[k][i*3+0]=toU8(col.x);
            imgs[k][i*3+1]=toU8(col.y);
            imgs[k][i*3+2]=toU8(col.z);
        }
    }
    for(int py=0;py<H;py++){
        // 左半
        memcpy(&buf[(py*CW)*3],     &imgs[0][(py*W)*3], W*3);
        // 右半
        memcpy(&buf[(py*CW+W)*3],   &imgs[1][(py*W)*3], W*3);
        // 分割线（黄色）
        buf[(py*CW+W-1)*3+0]=255; buf[(py*CW+W-1)*3+1]=255; buf[(py*CW+W-1)*3+2]=0;
        buf[(py*CW+W  )*3+0]=255; buf[(py*CW+W  )*3+1]=255; buf[(py*CW+W  )*3+2]=0;
    }
    stbi_write_png(path, CW, H, 3, buf.data(), CW*3);
    printf("✅ %s\n", path);
}

// ============================================================
// main
// ============================================================
int main(){
    printf("=== SSAO Renderer ===\n");
    initMatrices();

    // 初始化 G-Buffer 和 Z-Buffer
    gbuf.clear();
    fill(zbuf, zbuf+W*H, 2.0f);

    // ---- Cornell Box 场景（3×4×3 空间，相机在 z=5.5 往 -Z 看）----
    vector<Vertex> tris;

    // 地面：法线朝上(0,1,0) = right(1,0,0) × fwd(0,0,-1)
    addPlane(tris, {-2,0, 2}, {1,0,0}, {0,0,-1}, 4, 4, {0.73f,0.71f,0.68f}, 12, 12);

    // 天花板：法线朝下(0,-1,0) = right(1,0,0) × fwd(0,0,1)
    addPlane(tris, {-2,4,-2}, {1,0,0}, {0,0,1}, 4, 4, {0.73f,0.71f,0.68f}, 12, 12);

    // 后墙(z=-2)：法线朝 +Z = right(1,0,0) × fwd(0,1,0)
    addPlane(tris, {-2,0,-2}, {1,0,0}, {0,1,0}, 4, 4, {0.72f,0.72f,0.72f}, 12, 12);

    // 左墙(x=-2)：法线朝 +X = fwd(0,0,-1) × up(0,1,0) → 用 right(0,0,-1), fwd(0,1,0)
    addPlane(tris, {-2,0, 2}, {0,0,-1}, {0,1,0}, 4, 4, {0.65f,0.05f,0.05f}, 12, 12);

    // 右墙(x=+2)：法线朝 -X = fwd(0,0,1) × up(0,1,0) → right(0,0,1), fwd(0,1,0)
    addPlane(tris, { 2,0,-2}, {0,0,1},  {0,1,0}, 4, 4, {0.12f,0.45f,0.15f}, 12, 12);

    // 球体（贴着地面和墙，遮蔽缝隙明显）
    addSphere(tris, { 0.0f, 0.55f, -0.3f}, 0.55f, {0.85f,0.85f,0.85f}); // 大球（中，贴地）
    addSphere(tris, {-1.1f, 0.32f,  0.7f}, 0.32f, {0.20f,0.50f,0.80f}); // 小球（左前贴地）
    addSphere(tris, { 1.2f, 0.32f,  0.6f}, 0.32f, {0.80f,0.60f,0.20f}); // 小球（右前贴地）
    addSphere(tris, {-1.5f, 0.28f, -1.2f}, 0.28f, {0.90f,0.30f,0.30f}); // 小球（靠左后墙）
    addSphere(tris, { 1.5f, 0.28f, -1.0f}, 0.28f, {0.30f,0.80f,0.40f}); // 小球（靠右后墙）

    // 方块（紧贴墙角，墙-盒-地面三角缝隙产生强烈遮蔽）
    addBox(tris, {-0.8f, 0.65f, -1.2f}, 0.9f, 1.3f, 0.9f, {0.76f,0.75f,0.50f}); // 高盒
    addBox(tris, { 1.0f, 0.30f,  0.4f}, 1.1f, 0.6f, 0.9f, {0.76f,0.75f,0.50f}); // 矮盒

    printf("三角形数: %d\n", (int)tris.size()/3);

    // G-Buffer 光栅化
    printf("G-Buffer Pass...\n");
    for(int i=0;i<(int)tris.size();i+=3)
        rasterize(tris[i], tris[i+1], tris[i+2]);

    int validPx=0;
    for(int i=0;i<W*H;i++) if(gbuf.valid[i]) validPx++;
    printf("有效像素: %d / %d (%.1f%%)\n", validPx, W*H, 100.f*validPx/(W*H));

    // SSAO Pass
    printf("SSAO Pass...\n");
    computeSSAO();
    blurAO();

    // 量化验证
    float minAO=1,maxAO=0,sumAO=0; int cnt=0;
    for(int i=0;i<W*H;i++) if(gbuf.valid[i]){
        minAO=min(minAO,aoBlur[i]);
        maxAO=max(maxAO,aoBlur[i]);
        sumAO+=aoBlur[i]; cnt++;
    }
    float avgAO = cnt>0 ? sumAO/cnt : 0;
    float occludedPct = cnt>0 ? 100.f*(1-avgAO) : 0;

    printf("AO 范围: [%.3f, %.3f]  平均: %.3f  平均遮蔽率: %.1f%%\n",
           minAO, maxAO, avgAO, occludedPct);

    // 验收（Cornell Box 场景：最深遮蔽 ~37%，平均遮蔽 <15%）
    assert(minAO < 0.8f   && "最小 AO 不够暗：遮蔽效果不足");
    assert(maxAO > 0.9f   && "最大 AO 过低：无遮蔽区域异常");
    assert(avgAO > 0.8f && avgAO < 0.999f && "平均 AO 异常");
    printf("✅ 量化验证通过\n\n");

    // 输出
    writeImage("ssao_off.png", false);
    writeImage("ssao_on.png",  true);
    writeAOMap("ssao_map.png");
    writeComparison("ssao_comparison.png");

    printf("\n=== 完成 ===\n");
    printf("AO 遮蔽范围: [%.1f%%, %.1f%%]\n", minAO*100, maxAO*100);
    printf("平均遮蔽率: %.1f%%\n", occludedPct);
    return 0;
}
