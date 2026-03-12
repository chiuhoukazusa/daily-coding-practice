/*
 * SSAO - Screen Space Ambient Occlusion
 * 
 * 实现步骤：
 * 1. 软光栅化渲染几何（球体+地面+柱体），生成 G-Buffer (深度、法线、颜色)
 * 2. SSAO Pass: 对每像素在切线空间半球内随机采样，判断遮蔽
 * 3. 模糊 Pass: 4x4 box blur 降噪
 * 4. 合成输出: 无SSAO / 有SSAO / 对比图
 *
 * 编译: g++ -O2 -std=c++17 ssao.cpp -o ssao
 * 运行: ./ssao
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
    Vec3 normalize() const { float l=length(); if(l<1e-9f) return {0,0,1}; return *this/l; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float t){ x*=t; y*=t; z*=t; return *this; }
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
};

struct Mat4 {
    float m[4][4] = {};
    static Mat4 identity(){
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        float* rp = &r.x;
        for(int i=0;i<4;i++)
            rp[i]=m[i][0]*v.x+m[i][1]*v.y+m[i][2]*v.z+m[i][3]*v.w;
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
    float& operator[](int i){ return m[i/4][i%4]; }
    float  operator[](int i) const { return m[i/4][i%4]; }
};
float& mat(Mat4& M, int r, int c){ return M.m[r][c]; }

Vec4& vecref(Vec4& v, int i){ if(i==0)return *reinterpret_cast<Vec4*>(&v.x);
    // workaround
    static float dummy=0; 
    switch(i){case 0:return *reinterpret_cast<Vec4*>(&v.x);
              default:return *reinterpret_cast<Vec4*>(&v.x);}}

// 给 Vec4 下标访问
static float& vec4at(Vec4& v, int i){
    float* p = &v.x; return p[i];
}

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
    // view-space position, normal, albedo
    Vec3 position[W*H];  // view-space 位置
    Vec3 normal[W*H];    // view-space 法线
    Vec3 albedo[W*H];
    float depth[W*H];    // NDC depth [-1,1]
    bool valid[W*H];
    
    void clear(){
        for(int i=0;i<W*H;i++){
            position[i]={};
            normal[i]={};
            albedo[i]={0.1f,0.1f,0.1f};
            depth[i]=1.0f;
            valid[i]=false;
        }
    }
} gbuf;

float zbuf[W*H];

// ============================================================
// 光栅化工具
// ============================================================
struct Vertex {
    Vec3 pos;    // world space
    Vec3 normal; // world space
    Vec3 color;
};

Mat4 viewMat, projMat;
Mat4 normalMat; // view matrix for normals (transpose inverse)

void initMatrices(){
    Vec3 eye = {5.0f, 6.0f, 8.0f};
    Vec3 center = {0,0,0};
    Vec3 up = {0,1,0};
    viewMat = lookAt(eye, center, up);
    projMat = perspective(M_PI/3.0f, (float)W/H, 0.5f, 50.0f);
    // normalMat = transpose(inverse(view)) — for orthonormal view, just use view
    normalMat = viewMat; // view is orthonormal so it works
}

Vec4 transformVertex(Vec3 worldPos){
    Vec4 vp = viewMat * Vec4(worldPos, 1.0f);
    Vec4 cp = projMat * vp;
    return cp;
}

Vec3 transformNormal(Vec3 worldNormal){
    Vec4 vn = normalMat * Vec4(worldNormal, 0.0f);
    return vn.xyz().normalize();
}

Vec3 toViewSpace(Vec3 worldPos){
    Vec4 vp = viewMat * Vec4(worldPos, 1.0f);
    return vp.xyz();
}

// NDC -> screen
Vec2 ndcToScreen(float ndcX, float ndcY){
    return { (ndcX+1)*0.5f*(W-1), (1-ndcY)*0.5f*(H-1) };
}

// 重心坐标
bool barycentric(float x, float y,
                 float x0,float y0, float x1,float y1, float x2,float y2,
                 float& u, float& v, float& w){
    float denom = (y1-y2)*(x0-x2)+(x2-x1)*(y0-y2);
    if(fabsf(denom)<1e-9f) return false;
    u = ((y1-y2)*(x-x2)+(x2-x1)*(y-y2))/denom;
    v = ((y2-y0)*(x-x2)+(x0-x2)*(y-y2))/denom;
    w = 1-u-v;
    return u>=-1e-5f && v>=-1e-5f && w>=-1e-5f;
}

void rasterizeTriangle(Vertex& v0, Vertex& v1, Vertex& v2){
    // 变换顶点
    Vec4 c0 = transformVertex(v0.pos);
    Vec4 c1 = transformVertex(v1.pos);
    Vec4 c2 = transformVertex(v2.pos);
    
    // perspective divide
    if(fabsf(c0.w)<1e-6f||fabsf(c1.w)<1e-6f||fabsf(c2.w)<1e-6f) return;
    Vec3 ndc0 = {c0.x/c0.w, c0.y/c0.w, c0.z/c0.w};
    Vec3 ndc1 = {c1.x/c1.w, c1.y/c1.w, c1.z/c1.w};
    Vec3 ndc2 = {c2.x/c2.w, c2.y/c2.w, c2.z/c2.w};
    
    // view space positions
    Vec3 vs0 = toViewSpace(v0.pos);
    Vec3 vs1 = toViewSpace(v1.pos);
    Vec3 vs2 = toViewSpace(v2.pos);
    
    // view space normals
    Vec3 vn0 = transformNormal(v0.normal);
    Vec3 vn1 = transformNormal(v1.normal);
    Vec3 vn2 = transformNormal(v2.normal);
    
    Vec2 s0 = ndcToScreen(ndc0.x, ndc0.y);
    Vec2 s1 = ndcToScreen(ndc1.x, ndc1.y);
    Vec2 s2 = ndcToScreen(ndc2.x, ndc2.y);
    
    // bounding box
    int minX = max(0, (int)min({s0.x,s1.x,s2.x}));
    int maxX = min(W-1, (int)max({s0.x,s1.x,s2.x})+1);
    int minY = max(0, (int)min({s0.y,s1.y,s2.y}));
    int maxY = min(H-1, (int)max({s0.y,s1.y,s2.y})+1);
    
    for(int py=minY; py<=maxY; py++){
        for(int px=minX; px<=maxX; px++){
            float u,v,w;
            if(!barycentric(px+0.5f, py+0.5f,
                            s0.x,s0.y, s1.x,s1.y, s2.x,s2.y, u,v,w)) continue;
            
            // depth interpolation (perspective correct)
            float z = u*ndc0.z + v*ndc1.z + w*ndc2.z;
            
            int idx = py*W+px;
            if(z >= zbuf[idx]) continue;
            zbuf[idx] = z;
            
            // interpolate view-space position & normal
            Vec3 vsPos = vs0*u + vs1*v + vs2*w;
            Vec3 vsNorm = (vn0*u + vn1*v + vn2*w).normalize();
            Vec3 col = v0.color*u + v1.color*v + v2.color*w;
            
            gbuf.position[idx] = vsPos;
            gbuf.normal[idx] = vsNorm;
            gbuf.albedo[idx] = col;
            gbuf.depth[idx] = z;
            gbuf.valid[idx] = true;
        }
    }
}

// ============================================================
// 几何生成
// ============================================================

// 球体三角形
void addSphere(vector<Vertex>& tris, Vec3 center, float radius, Vec3 color,
               int stacks=20, int slices=30){
    auto pt = [&](int si, int ti) -> pair<Vec3,Vec3> {
        float phi = M_PI*ti/stacks - M_PI/2;
        float theta = 2*M_PI*si/slices;
        Vec3 n = {cosf(phi)*cosf(theta), sinf(phi), cosf(phi)*sinf(theta)};
        return {center + n*radius, n};
    };
    for(int t=0;t<stacks;t++) for(int s=0;s<slices;s++){
        auto [p00,n00]=pt(s,t);   auto [p10,n10]=pt(s+1,t);
        auto [p01,n01]=pt(s,t+1); auto [p11,n11]=pt(s+1,t+1);
        tris.push_back({p00,n00,color}); tris.push_back({p10,n10,color}); tris.push_back({p01,n01,color});
        tris.push_back({p10,n10,color}); tris.push_back({p11,n11,color}); tris.push_back({p01,n01,color});
    }
}

// 平面
void addPlane(vector<Vertex>& tris, Vec3 center, float w, float d, Vec3 color){
    Vec3 n = {0,1,0};
    Vec3 p00 = center+Vec3(-w/2,0,-d/2);
    Vec3 p10 = center+Vec3( w/2,0,-d/2);
    Vec3 p01 = center+Vec3(-w/2,0, d/2);
    Vec3 p11 = center+Vec3( w/2,0, d/2);
    tris.push_back({p00,n,color}); tris.push_back({p10,n,color}); tris.push_back({p01,n,color});
    tris.push_back({p10,n,color}); tris.push_back({p11,n,color}); tris.push_back({p01,n,color});
}

// 柱体（圆柱）
void addCylinder(vector<Vertex>& tris, Vec3 center, float radius, float height, Vec3 color, int slices=20){
    float y0 = center.y, y1 = center.y+height;
    for(int s=0;s<slices;s++){
        float t0=2*M_PI*s/slices, t1=2*M_PI*(s+1)/slices;
        Vec3 n0={cosf(t0),0,sinf(t0)}, n1={cosf(t1),0,sinf(t1)};
        Vec3 b0=center+n0*radius, b1=center+n1*radius;
        Vec3 t0p=b0,t1p=b1;
        t0p.y=y1; t1p.y=y1;
        b0.y=y0; b1.y=y0;
        // side
        tris.push_back({b0,n0,color}); tris.push_back({b1,n1,color}); tris.push_back({t0p,n0,color});
        tris.push_back({b1,n1,color}); tris.push_back({t1p,n1,color}); tris.push_back({t0p,n0,color});
        // top cap
        Vec3 nt={0,1,0};
        Vec3 tc=center; tc.y=y1;
        tris.push_back({tc,nt,color}); tris.push_back({t0p,nt,color}); tris.push_back({t1p,nt,color});
        // bottom cap
        Vec3 nb={0,-1,0};
        Vec3 bc=center; bc.y=y0;
        tris.push_back({bc,nb,color}); tris.push_back({b1,nb,color}); tris.push_back({b0,nb,color});
    }
}

// 长方体
void addBox(vector<Vertex>& tris, Vec3 center, float w, float h, float d, Vec3 color){
    float hw=w/2, hh=h/2, hd=d/2;
    struct Face { Vec3 n; Vec3 p[4]; };
    Face faces[6] = {
        {{0,0,1},  {{center.x-hw,center.y-hh,center.z+hd},{center.x+hw,center.y-hh,center.z+hd},{center.x+hw,center.y+hh,center.z+hd},{center.x-hw,center.y+hh,center.z+hd}}},
        {{0,0,-1}, {{center.x+hw,center.y-hh,center.z-hd},{center.x-hw,center.y-hh,center.z-hd},{center.x-hw,center.y+hh,center.z-hd},{center.x+hw,center.y+hh,center.z-hd}}},
        {{1,0,0},  {{center.x+hw,center.y-hh,center.z+hd},{center.x+hw,center.y-hh,center.z-hd},{center.x+hw,center.y+hh,center.z-hd},{center.x+hw,center.y+hh,center.z+hd}}},
        {{-1,0,0}, {{center.x-hw,center.y-hh,center.z-hd},{center.x-hw,center.y-hh,center.z+hd},{center.x-hw,center.y+hh,center.z+hd},{center.x-hw,center.y+hh,center.z-hd}}},
        {{0,1,0},  {{center.x-hw,center.y+hh,center.z+hd},{center.x+hw,center.y+hh,center.z+hd},{center.x+hw,center.y+hh,center.z-hd},{center.x-hw,center.y+hh,center.z-hd}}},
        {{0,-1,0}, {{center.x-hw,center.y-hh,center.z-hd},{center.x+hw,center.y-hh,center.z-hd},{center.x+hw,center.y-hh,center.z+hd},{center.x-hw,center.y-hh,center.z+hd}}},
    };
    for(auto& f:faces){
        tris.push_back({f.p[0],f.n,color}); tris.push_back({f.p[1],f.n,color}); tris.push_back({f.p[2],f.n,color});
        tris.push_back({f.p[0],f.n,color}); tris.push_back({f.p[2],f.n,color}); tris.push_back({f.p[3],f.n,color});
    }
}

// ============================================================
// SSAO 采样核
// ============================================================
const int SSAO_KERNEL_SIZE = 32;
const int SSAO_NOISE_SIZE  = 4;  // 4x4 noise tile
const float SSAO_RADIUS    = 0.5f;
const float SSAO_BIAS      = 0.05f;

Vec3 ssaoKernel[SSAO_KERNEL_SIZE];
Vec3 ssaoNoise[SSAO_NOISE_SIZE*SSAO_NOISE_SIZE];
float ssaoMap[W*H];    // raw SSAO occlusion
float ssaoBlur[W*H];   // blurred SSAO

void generateSSAOKernel(mt19937& rng){
    uniform_real_distribution<float> dist(0,1);
    uniform_real_distribution<float> dist11(-1,1);
    
    for(int i=0;i<SSAO_KERNEL_SIZE;i++){
        Vec3 s = { dist11(rng), dist11(rng), dist(rng) };
        s = s.normalize();
        s = s * dist(rng);
        // accelerating interpolation (more samples near origin)
        float scale = (float)i/SSAO_KERNEL_SIZE;
        scale = 0.1f + scale*scale*0.9f;
        s = s * scale;
        ssaoKernel[i] = s;
    }
}

void generateSSAONoise(mt19937& rng){
    uniform_real_distribution<float> dist11(-1,1);
    for(int i=0;i<SSAO_NOISE_SIZE*SSAO_NOISE_SIZE;i++){
        // rotation vectors around z-axis (no z component)
        ssaoNoise[i] = Vec3(dist11(rng), dist11(rng), 0).normalize();
    }
}

// 将 view-space 位置重投影得到纹理坐标和线性深度
bool projectToTexture(Vec3 vsPos, int& tx, int& ty, float& ndcZ){
    Vec4 cp = projMat * Vec4(vsPos, 1.0f);
    if(fabsf(cp.w)<1e-6f) return false;
    ndcZ = cp.z/cp.w;
    if(ndcZ < -1 || ndcZ > 1) return false;
    float ndcX = cp.x/cp.w;
    float ndcY = cp.y/cp.w;
    tx = (int)((ndcX+1)*0.5f*(W-1)+0.5f);
    ty = (int)((1-ndcY)*0.5f*(H-1)+0.5f);
    return tx>=0 && tx<W && ty>=0 && ty<H;
}

void computeSSAO(){
    mt19937 rng(42);
    generateSSAOKernel(rng);
    generateSSAONoise(rng);
    
    for(int py=0;py<H;py++){
        for(int px=0;px<W;px++){
            int idx=py*W+px;
            if(!gbuf.valid[idx]){
                ssaoMap[idx]=1.0f; continue;
            }
            
            Vec3 fragPos = gbuf.position[idx];
            Vec3 normal  = gbuf.normal[idx].normalize();
            
            // 从噪声纹理获取旋转向量（tiling）
            int nx = px % SSAO_NOISE_SIZE;
            int ny = py % SSAO_NOISE_SIZE;
            Vec3 randomVec = ssaoNoise[ny*SSAO_NOISE_SIZE+nx];
            
            // TBN 矩阵（将核从切线空间变换到 view space）
            Vec3 tangent = (randomVec - normal * normal.dot(randomVec)).normalize();
            Vec3 bitangent = normal.cross(tangent);
            
            float occlusion = 0.0f;
            for(int i=0;i<SSAO_KERNEL_SIZE;i++){
                // 将采样点变换到 view space
                Vec3 s = ssaoKernel[i];
                Vec3 samplePos = tangent*s.x + bitangent*s.y + normal*s.z;
                samplePos = fragPos + samplePos * SSAO_RADIUS;
                
                // 投影采样点到纹理坐标
                int tx, ty; float ndcZ;
                if(!projectToTexture(samplePos, tx, ty, ndcZ)) continue;
                
                // 获取采样点的真实深度（view space z）
                int sidx = ty*W+tx;
                if(!gbuf.valid[sidx]) continue;
                
                float sampleDepth = gbuf.position[sidx].z;
                
                // 范围检查 + 遮蔽判断
                float rangeCheck = smoothstep(0.0f, 1.0f, 
                    SSAO_RADIUS / fabsf(fragPos.z - sampleDepth + 1e-5f));
                // view space: z 为负，更近的 z 更小（更负）
                // 采样点的 z < fragPos.z 表示更近（遮蔽）
                // 加 bias 避免自遮蔽
                occlusion += (samplePos.z + SSAO_BIAS >= sampleDepth ? 1.0f : 0.0f) * rangeCheck;
            }
            
            ssaoMap[idx] = 1.0f - (occlusion / SSAO_KERNEL_SIZE);
        }
    }
}

// ============================================================
// 模糊 Pass（4x4 box blur）
// ============================================================
void blurSSAO(){
    const int blur=2;
    for(int py=0;py<H;py++){
        for(int px=0;px<W;px++){
            float sum=0; int cnt=0;
            for(int dy=-blur;dy<=blur;dy++)
                for(int dx=-blur;dx<=blur;dx++){
                    int sx=px+dx, sy=py+dy;
                    if(sx<0||sx>=W||sy<0||sy>=H) continue;
                    sum+=ssaoMap[sy*W+sx];
                    cnt++;
                }
            ssaoBlur[py*W+px] = cnt>0 ? sum/cnt : ssaoMap[py*W+px];
        }
    }
}

// ============================================================
// 着色
// ============================================================
Vec3 shadingLight(Vec3 vsPos, Vec3 vsNormal, Vec3 albedo, float ao){
    // 视空间光源
    Vec4 lightWorldPos4 = viewMat * Vec4(Vec3(5,8,5), 1.0f);
    Vec3 lightPos = lightWorldPos4.xyz();
    
    Vec3 lightDir = (lightPos - vsPos).normalize();
    Vec3 viewDir  = (-vsPos).normalize();
    
    float diff = max(0.0f, vsNormal.dot(lightDir));
    Vec3 halfway = (lightDir+viewDir).normalize();
    float spec = powf(max(0.0f, vsNormal.dot(halfway)), 32.0f);
    
    Vec3 ambient  = albedo * 0.15f * ao;
    Vec3 diffuse  = albedo * diff  * 0.75f;
    Vec3 specular = Vec3(1,1,1) * spec * 0.3f;
    
    return ambient + diffuse + specular;
}

// ACES tone mapping
Vec3 aces(Vec3 c){
    float a=2.51f,b=0.03f,cc=2.43f,d=0.59f,e=0.14f;
    c.x=clamp01((c.x*(a*c.x+b))/(c.x*(cc*c.x+d)+e));
    c.y=clamp01((c.y*(a*c.y+b))/(c.y*(cc*c.y+d)+e));
    c.z=clamp01((c.z*(a*c.z+b))/(c.z*(cc*c.z+d)+e));
    return c;
}

Vec3 linearToSRGB(Vec3 c){
    return {powf(clamp01(c.x),1.0f/2.2f),
            powf(clamp01(c.y),1.0f/2.2f),
            powf(clamp01(c.z),1.0f/2.2f)};
}

unsigned char toU8(float v){ return (unsigned char)(clamp01(v)*255.0f+0.5f); }

void renderFinalImage(const char* filename, bool useSSAO){
    vector<unsigned char> pixels(W*H*3);
    
    for(int py=0;py<H;py++){
        for(int px=0;px<W;px++){
            int idx=py*W+px;
            Vec3 col;
            
            if(gbuf.valid[idx]){
                float ao = useSSAO ? ssaoBlur[idx] : 1.0f;
                col = shadingLight(gbuf.position[idx], gbuf.normal[idx], gbuf.albedo[idx], ao);
                col = aces(col);
                col = linearToSRGB(col);
            } else {
                // 背景渐变
                float t = (float)py/H;
                col = lerp(Vec3(0.6f,0.7f,0.9f), Vec3(0.2f,0.3f,0.5f), t);
            }
            
            pixels[(py*W+px)*3+0] = toU8(col.x);
            pixels[(py*W+px)*3+1] = toU8(col.y);
            pixels[(py*W+px)*3+2] = toU8(col.z);
        }
    }
    
    stbi_write_png(filename, W, H, 3, pixels.data(), W*3);
    cout << "✅ 保存: " << filename << endl;
}

void renderSSAOMap(const char* filename){
    vector<unsigned char> pixels(W*H*3);
    for(int i=0;i<W*H;i++){
        unsigned char v = toU8(ssaoBlur[i]);
        pixels[i*3+0]=pixels[i*3+1]=pixels[i*3+2]=v;
    }
    stbi_write_png(filename, W, H, 3, pixels.data(), W*3);
    cout << "✅ 保存: " << filename << endl;
}

// 左右对比图
void renderComparison(const char* noSSAOFile, const char* ssaoFile, const char* outFile){
    int CW = W*2, CH = H;
    vector<unsigned char> comp(CW*CH*3);
    
    // 加载两张图
    vector<unsigned char> imgA(W*H*3), imgB(W*H*3);
    
    // 重新渲染到内存
    for(int py=0;py<H;py++) for(int px=0;px<W;px++){
        int idx=py*W+px;
        Vec3 col;
        if(gbuf.valid[idx]){
            col = shadingLight(gbuf.position[idx],gbuf.normal[idx],gbuf.albedo[idx],1.0f);
            col = aces(col); col = linearToSRGB(col);
        } else { float t=(float)py/H; col=lerp(Vec3(0.6f,0.7f,0.9f),Vec3(0.2f,0.3f,0.5f),t);}
        imgA[(py*W+px)*3+0]=toU8(col.x); imgA[(py*W+px)*3+1]=toU8(col.y); imgA[(py*W+px)*3+2]=toU8(col.z);
        
        if(gbuf.valid[idx]){
            col = shadingLight(gbuf.position[idx],gbuf.normal[idx],gbuf.albedo[idx],ssaoBlur[idx]);
            col = aces(col); col = linearToSRGB(col);
        } else { float t=(float)py/H; col=lerp(Vec3(0.6f,0.7f,0.9f),Vec3(0.2f,0.3f,0.5f),t);}
        imgB[(py*W+px)*3+0]=toU8(col.x); imgB[(py*W+px)*3+1]=toU8(col.y); imgB[(py*W+px)*3+2]=toU8(col.z);
    }
    
    // 合并
    for(int py=0;py<H;py++){
        for(int px=0;px<W;px++){
            int src=(py*W+px)*3;
            int dstA=(py*CW+px)*3;
            int dstB=(py*CW+W+px)*3;
            comp[dstA+0]=imgA[src+0]; comp[dstA+1]=imgA[src+1]; comp[dstA+2]=imgA[src+2];
            comp[dstB+0]=imgB[src+0]; comp[dstB+1]=imgB[src+1]; comp[dstB+2]=imgB[src+2];
        }
    }
    
    // 中间分割线
    for(int py=0;py<H;py++){
        int x=W;
        comp[(py*CW+x)*3+0]=255; comp[(py*CW+x)*3+1]=255; comp[(py*CW+x)*3+2]=0;
    }
    
    stbi_write_png(outFile, CW, CH, 3, comp.data(), CW*3);
    cout << "✅ 保存对比图: " << outFile << endl;
}

// ============================================================
// main
// ============================================================
int main(){
    cout << "=== SSAO - Screen Space Ambient Occlusion ===" << endl;
    
    initMatrices();
    
    // 初始化 GBuffer 和深度缓冲
    gbuf.clear();
    for(int i=0;i<W*H;i++) zbuf[i]=2.0f; // +inf
    
    // 构建场景
    vector<Vertex> triangles;
    
    // 地面（灰色）
    addPlane(triangles, {0,-0.5f,0}, 12, 12, {0.7f,0.7f,0.65f});
    
    // 后墙
    {
        vector<Vertex> tmp;
        addPlane(tmp, {0,0,0}, 12, 4, {0.6f,0.6f,0.7f});
        // 旋转: 绕X轴90度, 移到z=-4
        for(auto& v:tmp){
            float y=v.pos.y, z=v.pos.z;
            v.pos.y = -z; v.pos.z = y;
            v.pos.z -= 4.5f; v.pos.y += 1.5f;
            y=v.normal.y; z=v.normal.z;
            v.normal.y=-z; v.normal.z=y;
        }
        for(auto& v:tmp) triangles.push_back(v);
    }
    
    // 球体（几个不同颜色）
    addSphere(triangles, {0, 0.8f, 0},   0.8f, {0.9f,0.3f,0.3f}); // 红球
    addSphere(triangles, {-2.2f,0.5f,0.5f}, 0.5f, {0.3f,0.7f,0.3f}); // 绿小球
    addSphere(triangles, {2.0f, 0.6f,-0.5f}, 0.6f, {0.3f,0.5f,0.9f}); // 蓝球
    
    // 柱体
    addCylinder(triangles, {-1.0f,-0.5f,-2.0f}, 0.35f, 2.5f, {0.8f,0.7f,0.5f});
    addCylinder(triangles, { 1.5f,-0.5f,-2.2f}, 0.3f,  2.0f, {0.7f,0.8f,0.6f});
    
    // 方块
    addBox(triangles, {-2.8f, 0.25f,-1.0f}, 1.0f, 1.5f, 0.9f, {0.8f,0.75f,0.7f});
    addBox(triangles, { 2.5f, 0.0f, 1.5f},  0.8f, 1.0f, 0.8f, {0.7f,0.8f,0.75f});
    
    cout << "场景三角形数量: " << triangles.size()/3 << endl;
    
    // 光栅化
    cout << "渲染 G-Buffer..." << endl;
    for(int i=0;i<(int)triangles.size(); i+=3){
        rasterizeTriangle(triangles[i], triangles[i+1], triangles[i+2]);
    }
    
    // 统计有效像素
    int validCount=0;
    for(int i=0;i<W*H;i++) if(gbuf.valid[i]) validCount++;
    cout << "有效像素: " << validCount << "/" << W*H << endl;
    
    // 计算 SSAO
    cout << "计算 SSAO..." << endl;
    computeSSAO();
    blurSSAO();
    
    // 验证 SSAO 值
    float minAO=1,maxAO=0,sumAO=0; int aoCnt=0;
    for(int i=0;i<W*H;i++){
        if(gbuf.valid[i]){
            minAO=min(minAO,ssaoBlur[i]);
            maxAO=max(maxAO,ssaoBlur[i]);
            sumAO+=ssaoBlur[i];
            aoCnt++;
        }
    }
    float avgAO = aoCnt>0 ? sumAO/aoCnt : 0;
    cout << "SSAO 值范围: [" << minAO << ", " << maxAO << "], 平均: " << avgAO << endl;
    
    // 验证 SSAO 有效性：
    // - minAO 应 < 0.9 (有遮蔽)
    // - maxAO 应 > 0.5 (有非遮蔽区域)
    // - avgAO 应在 0.1-0.99 之间
    assert(minAO < 0.9f && "SSAO 没有产生遮蔽效果");
    assert(maxAO > 0.3f && "SSAO 遮蔽过于严重");
    assert(avgAO > 0.1f && avgAO < 0.99f && "SSAO 平均值异常");
    
    // 输出图片
    renderFinalImage("/tmp/ssao_work/ssao_off.png", false);
    renderFinalImage("/tmp/ssao_work/ssao_on.png",  true);
    renderSSAOMap("/tmp/ssao_work/ssao_map.png");
    renderComparison("/tmp/ssao_work/ssao_off.png", "/tmp/ssao_work/ssao_on.png",
                     "/tmp/ssao_work/ssao_comparison.png");
    
    cout << "\n=== 验证通过 ===" << endl;
    cout << "SSAO 遮蔽范围: [" << minAO*100 << "%, " << maxAO*100 << "%]" << endl;
    cout << "平均遮蔽: " << avgAO*100 << "%" << endl;
    cout << "有效场景像素: " << validCount << "/" << W*H << endl;
    
    return 0;
}
