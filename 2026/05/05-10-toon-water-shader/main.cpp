// Toon Water Shader - NPR风格卡通水面渲染
// 技术要点：
//   - 深度泡沫效果（Edge Foam based on depth difference）
//   - Voronoi噪声驱动的水面波纹
//   - Fresnel反射（卡通风格色阶化）
//   - 折射扭曲（Normal map distortion）
//   - 程序化网格 + 软光栅化
//   - NPR色阶量化 + 描边
//   - 时间驱动动画（静态帧 t=0.5）

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <random>
#include <array>
#include <functional>
#include <cassert>

using namespace std;

// ─── 数学基础 ────────────────────────────────────────────────────
struct Vec2 { float x=0,y=0; };
struct Vec3 {
    float x=0,y=0,z=0;
    Vec3() = default;
    Vec3(float x,float y,float z):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float t)const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t)const{return{x/t,y/t,z/t};}
    Vec3 operator*(const Vec3&o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3& operator+=(const Vec3&o){x+=o.x;y+=o.y;z+=o.z;return*this;}
    Vec3& operator*=(float t){x*=t;y*=t;z*=t;return*this;}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float length()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 normalize()const{float l=length();return l>1e-6f?*this/l:Vec3(0,1,0);}
    Vec3 lerp(const Vec3&o,float t)const{return *this*(1-t)+o*t;}
    float& operator[](int i){return i==0?x:(i==1?y:z);}
    float  operator[](int i)const{return i==0?x:(i==1?y:z);}
};
inline Vec3 operator*(float t,const Vec3&v){return v*t;}

struct Vec4 {
    float x=0,y=0,z=0,w=1;
    Vec4()=default;
    Vec4(float x,float y,float z,float w):x(x),y(y),z(z),w(w){}
    Vec4(const Vec3&v,float w):x(v.x),y(v.y),z(v.z),w(w){}
};

struct Mat4 {
    float m[4][4]={};
    Mat4(){ m[0][0]=m[1][1]=m[2][2]=m[3][3]=1; }
    Vec4 operator*(const Vec4&v)const{
        Vec4 r;
        r.x=m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w;
        r.y=m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w;
        r.z=m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w;
        r.w=m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w;
        return r;
    }
    Mat4 operator*(const Mat4&o)const{
        Mat4 r; memset(r.m,0,sizeof(r.m));
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
            r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 perspective(float fovY,float aspect,float near,float far){
    Mat4 m; memset(m.m,0,sizeof(m.m));
    float f=1.0f/tanf(fovY*0.5f);
    m.m[0][0]=f/aspect;
    m.m[1][1]=f;
    m.m[2][2]=(far+near)/(near-far);
    m.m[2][3]=(2*far*near)/(near-far);
    m.m[3][2]=-1;
    return m;
}

Mat4 lookAt(Vec3 eye,Vec3 center,Vec3 up){
    Vec3 f=(center-eye).normalize();
    Vec3 r=f.cross(up).normalize();
    Vec3 u=r.cross(f);
    Mat4 m;
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]=f.dot(eye);
    m.m[3][0]=0;   m.m[3][1]=0;   m.m[3][2]=0;   m.m[3][3]=1;
    return m;
}

// ─── 颜色工具 ─────────────────────────────────────────────────────
inline float smoothstep(float edge0, float edge1, float x){
    float t=clamp((x-edge0)/(edge1-edge0+1e-9f),0.f,1.f);
    return t*t*(3.f-2.f*t);
}
inline float clamp(float v,float lo,float hi){return max(lo,min(hi,v));}

Vec3 clamp3(Vec3 v,float lo=0,float hi=1){
    return {max(lo,min(hi,v.x)),max(lo,min(hi,v.y)),max(lo,min(hi,v.z))};
}
Vec3 gamma(Vec3 c,float g=2.2f){
    return{powf(max(c.x,0.f),1.f/g),powf(max(c.y,0.f),1.f/g),powf(max(c.z,0.f),1.f/g)};
}

// ─── 噪声函数 ─────────────────────────────────────────────────────

// 哈希函数
float hash11(float p){
    p=fmodf(p*127.1f,3.14159f*2.f);
    return 0.5f+0.5f*sinf(p*43758.5453f+p*p);
}
float hash21(Vec2 p){
    float h=p.x*127.1f+p.y*311.7f;
    return fmodf(sinf(h)*43758.5453f,1.f);
    // 保证在0~1
}
Vec2 hash22(Vec2 p){
    float x=fmodf(sinf(p.x*127.1f+p.y*311.7f)*43758.5453f,1.f);
    float y=fmodf(sinf(p.x*269.5f+p.y*183.3f)*43758.5453f,1.f);
    // abs保证正数
    return{fabsf(x),fabsf(y)};
}

// Voronoi 噪声（卡通水面波纹核心）
// 返回到最近特征点的距离
float voronoi(Vec2 p, float t){
    Vec2 ip={floorf(p.x),floorf(p.y)};
    Vec2 fp={p.x-floorf(p.x),p.y-floorf(p.y)};
    float minDist=1e9f;
    for(int j=-2;j<=2;j++) for(int i=-2;i<=2;i++){
        Vec2 cell={ip.x+(float)i, ip.y+(float)j};
        Vec2 h=hash22(cell);
        // 特征点随时间移动
        float angle = h.x*6.2831f + t*2.f;
        float r = 0.35f + 0.15f*h.y;
        Vec2 o;
        o.x = (float)i + 0.5f + r*cosf(angle);
        o.y = (float)j + 0.5f + r*sinf(angle);
        Vec2 d={fp.x-o.x, fp.y-o.y};
        float dist=sqrtf(d.x*d.x+d.y*d.y);
        minDist=min(minDist,dist);
    }
    return minDist;
}

// 分形布朗运动（FBM）噪声
float fbm(Vec2 p){
    float value=0, amplitude=0.5f, freq=1.f;
    for(int i=0;i<5;i++){
        Vec2 sp={p.x*freq, p.y*freq};
        // 简单sin噪声
        float n=sinf(sp.x*1.7f+0.4f)*cosf(sp.y*1.3f+0.7f)*0.5f+0.5f;
        value += amplitude*n;
        amplitude*=0.5f; freq*=2.1f;
    }
    return value;
}

// 混合FBM（用于折射扭曲）
Vec2 fbm2(Vec2 p, float t){
    Vec2 d;
    d.x=sinf(p.x*2.1f+p.y*1.7f+t)*0.5f+cosf(p.x*3.2f-p.y*1.4f+t*1.3f)*0.3f;
    d.y=cosf(p.x*1.5f+p.y*2.4f-t*0.9f)*0.5f+sinf(p.x*2.8f+p.y*3.1f+t*0.7f)*0.3f;
    return d;
}

// ─── 渲染器常量 ───────────────────────────────────────────────────
const int WIDTH  = 800;
const int HEIGHT = 600;
const float TIME = 1.2f; // 动画时间（静态帧）

// ─── 帧缓冲 ───────────────────────────────────────────────────────
struct Framebuffer {
    int width, height;
    vector<Vec3> color;
    vector<float> depth;  // NDC z (越大越近观察者，z-fight用)
    vector<float> linearDepth; // 线性深度（0~1），用于深度泡沫

    Framebuffer(int w,int h): width(w),height(h),
        color(w*h,Vec3(0,0,0)), depth(w*h,-1e9f), linearDepth(w*h, 1.f){}

    void setPixel(int x,int y,Vec3 c){
        if(x<0||x>=width||y<0||y>=height)return;
        color[y*width+x]=c;
    }
    Vec3 getPixel(int x,int y)const{
        if(x<0||x>=width||y<0||y>=height)return{0,0,0};
        return color[y*width+x];
    }
    void setDepth(int x,int y,float d){ if(x>=0&&x<width&&y>=0&&y<height) depth[y*width+x]=d; }
    float getDepth(int x,int y)const{ return(x>=0&&x<width&&y>=0&&y<height)?depth[y*width+x]:-1e9f; }
    void setLinearDepth(int x,int y,float d){ if(x>=0&&x<width&&y>=0&&y<height) linearDepth[y*width+x]=d; }
    float getLinearDepth(int x,int y)const{ return(x>=0&&x<width&&y>=0&&y<height)?linearDepth[y*width+x]:1.f; }
};

// ─── 顶点结构 ─────────────────────────────────────────────────────
struct Vertex {
    Vec3 pos;    // 世界空间
    Vec3 normal; // 世界空间法线
    Vec2 uv;     // 纹理坐标
    float depth; // 视图空间深度（负号）
};

// ─── 投影 & 视图矩阵 ─────────────────────────────────────────────
Vec3 EYE    = {0.f, 8.f, 14.f};
Vec3 CENTER = {0.f, 0.f, 0.f};
Vec3 UP     = {0.f, 1.f, 0.f};
Vec3 LIGHT  = {6.f, 12.f, 8.f};

Mat4 VIEW, PROJ, VP;

void initMatrices(){
    VIEW = lookAt(EYE, CENTER, UP);
    PROJ = perspective(0.8f, (float)WIDTH/HEIGHT, 0.5f, 60.f);
    VP   = PROJ * VIEW;
}

// 世界→裁剪空间
Vec4 worldToClip(Vec3 p){
    Vec4 v4(p,1);
    return VP*v4;
}

// 裁剪→NDC→屏幕
Vec3 ndcToScreen(Vec4 clip){
    if(fabsf(clip.w)<1e-6f) return{-9999,-9999,0};
    float invW=1.f/clip.w;
    float nx=clip.x*invW, ny=clip.y*invW, nz=clip.z*invW;
    float sx=(nx+1.f)*0.5f*WIDTH;
    float sy=(1.f-ny)*0.5f*HEIGHT;
    return{sx,sy,nz};
}

// ─── 天空背景（卡通风格渐变） ─────────────────────────────────────
Vec3 skyColor(float t){
    // t=0 底部（地平线），t=1 顶部（天顶）
    Vec3 horizon = {0.85f, 0.95f, 1.0f};  // 浅蓝白
    Vec3 zenith  = {0.3f, 0.55f, 0.9f};   // 深蓝
    // 卡通色阶量化（3级）
    float step = floorf(t*3.f)/3.f;
    return horizon.lerp(zenith, step);
}

// ─── 卡通水面着色器 ───────────────────────────────────────────────
Vec3 toonWaterShader(Vec3 worldPos, Vec3 worldNormal, Vec2 uv,
                     float surfaceDepth, float sceneDepth,
                     Vec3 viewDir)
{
    // === 1. 泡沫效果（基于深度差） ===
    float depthDiff = max(0.f, sceneDepth - surfaceDepth);
    float foamThreshold = 0.3f; // 泡沫宽度
    float foamFactor = 1.f - smoothstep(0.f, foamThreshold, depthDiff);
    
    // Voronoi 驱动的泡沫纹理
    Vec2 foamUV = {uv.x*6.f + TIME*0.15f, uv.y*6.f + TIME*0.1f};
    float voronoiVal = voronoi(foamUV, TIME);
    float foamTex = 1.f - smoothstep(0.15f, 0.35f, voronoiVal);
    foamFactor *= (0.7f + 0.3f*foamTex);

    // === 2. 折射扭曲（模拟水下折射） ===
    Vec2 distortUV = {uv.x*4.f, uv.y*4.f};
    Vec2 distort = fbm2(distortUV, TIME);
    Vec2 refrUV = {uv.x + distort.x*0.04f, uv.y + distort.y*0.04f};

    // === 3. 水体颜色（浅水 vs 深水） ===
    // 深度映射到水色
    float depthT = clamp(depthDiff / 2.5f, 0.f, 1.f);
    Vec3 shallowColor = {0.4f, 0.85f, 0.8f};  // 浅水：青绿
    Vec3 deepColor    = {0.05f, 0.25f, 0.55f}; // 深水：深蓝

    // 折射纹理变化（模拟水底纹理）
    float pattern = fbm({refrUV.x*2.f, refrUV.y*2.f});
    Vec3 waterBase = shallowColor.lerp(deepColor, depthT);
    waterBase = waterBase * (0.85f + 0.15f*pattern);

    // === 4. Fresnel 反射（卡通色阶化） ===
    float fresnel = 1.f - max(0.f, worldNormal.dot(viewDir));
    fresnel = fresnel*fresnel*fresnel; // 菲涅尔近似
    // 卡通量化（2级：低角度 vs 高角度）
    float fresnelToon = fresnel > 0.4f ? 0.8f : 0.2f;
    
    // 天空反射色（简化）
    Vec3 reflColor = {0.75f, 0.88f, 1.0f};
    Vec3 waterColor = waterBase.lerp(reflColor, fresnelToon*0.6f);

    // === 5. 高光（卡通硬高光） ===
    Vec3 lightDir = (LIGHT - worldPos).normalize();
    Vec3 halfVec  = (viewDir + lightDir).normalize();
    float spec = powf(max(0.f, worldNormal.dot(halfVec)), 32.f);
    // 卡通量化
    float specToon = spec > 0.6f ? 1.f : (spec > 0.2f ? 0.4f : 0.f);
    Vec3 specColor = Vec3(1.f,1.f,1.f) * specToon * 0.9f;

    // === 6. 漫反射（卡通色阶） ===
    float diff = max(0.f, worldNormal.dot(lightDir));
    float diffToon = diff > 0.7f ? 1.f : (diff > 0.35f ? 0.65f : 0.3f);
    waterColor *= diffToon;

    // === 7. 泡沫合成 ===
    Vec3 foamColor = {0.92f, 0.97f, 1.f}; // 近白
    waterColor = waterColor.lerp(foamColor, foamFactor);

    // === 8. 高光叠加 ===
    waterColor += specColor*(1.f-foamFactor);

    // === 9. 边缘描边（后处理，此处标记） ===
    // 由后处理 pass 处理，此处不输出

    return clamp3(waterColor);
}

// ─── 水面顶点生成（Gerstner 波形变形） ────────────────────────────
struct WaterSurface {
    int gridW, gridH;
    float sizeX, sizeZ;
    vector<Vec3> verts;
    vector<Vec3> normals;
    vector<Vec2> uvs;
    vector<array<int,3>> tris; // 三角形索引

    WaterSurface(int gw,int gh,float sx,float sz)
        :gridW(gw),gridH(gh),sizeX(sx),sizeZ(sz)
    {
        verts.resize(gw*gh);
        normals.resize(gw*gh,Vec3(0,1,0));
        uvs.resize(gw*gh);
        buildGrid();
        buildTriangles();
    }

    void buildGrid(){
        for(int j=0;j<gridH;j++) for(int i=0;i<gridW;i++){
            float u=(float)i/(gridW-1), v=(float)j/(gridH-1);
            float x=(u-0.5f)*sizeX;
            float z=(v-0.5f)*sizeZ;
            float y=gerstner(x,z,TIME);
            verts[j*gridW+i]={x,y,z};
            uvs[j*gridW+i]={u,v};
        }
        computeNormals();
    }

    // Gerstner 波叠加
    float gerstner(float x,float z,float t){
        float h=0;
        // 波1
        {float kx=0.4f,kz=0.3f,amp=0.35f,speed=1.2f;
         h+=amp*sinf(kx*x+kz*z-speed*t);}
        // 波2
        {float kx=-0.3f,kz=0.5f,amp=0.2f,speed=0.9f;
         h+=amp*sinf(kx*x+kz*z-speed*t+1.1f);}
        // 波3（高频细节）
        {float kx=0.8f,kz=-0.6f,amp=0.1f,speed=1.5f;
         h+=amp*sinf(kx*x+kz*z-speed*t+2.3f);}
        return h;
    }

    void computeNormals(){
        for(int j=0;j<gridH;j++) for(int i=0;i<gridW;i++){
            int ip=min(i+1,gridW-1), im=max(i-1,0);
            int jp=min(j+1,gridH-1), jm=max(j-1,0);
            Vec3 dx=verts[j*gridW+ip]-verts[j*gridW+im];
            Vec3 dz=verts[jp*gridW+i]-verts[jm*gridW+i];
            normals[j*gridW+i]=dz.cross(dx).normalize();
        }
    }

    void buildTriangles(){
        for(int j=0;j<gridH-1;j++) for(int i=0;i<gridW-1;i++){
            int a=j*gridW+i, b=j*gridW+i+1;
            int c=(j+1)*gridW+i, d=(j+1)*gridW+i+1;
            tris.push_back({a,b,d});
            tris.push_back({a,d,c});
        }
    }
};

// ─── 陆地/海底几何 ────────────────────────────────────────────────
struct TerrainPatch {
    vector<Vec3> verts;
    vector<Vec3> normals;
    vector<array<int,3>> tris;

    // 简单倾斜平面（海底）+ 岛屿凸起
    void build(int gw,int gh,float sx,float sz,float baseY){
        verts.resize(gw*gh);
        normals.resize(gw*gh, Vec3(0,1,0));
        for(int j=0;j<gh;j++) for(int i=0;i<gw;i++){
            float u=(float)i/(gw-1), v=(float)j/(gh-1);
            float x=(u-0.5f)*sx, z=(v-0.5f)*sz;
            // 海底高度：中间高（岛礁）+ 倾斜
            float dist=sqrtf(x*x+z*z)/(sx*0.5f);
            float island=max(0.f,1.f-dist*1.5f)*3.5f;
            float seabed=baseY + island + fbm({u*4.f,v*4.f})*0.4f;
            verts[j*gw+i]={x,seabed,z};
        }
        // 计算法线
        for(int j=0;j<gh;j++) for(int i=0;i<gw;i++){
            int ip=min(i+1,gw-1),im=max(i-1,0);
            int jp=min(j+1,gh-1),jm=max(j-1,0);
            Vec3 dx=verts[j*gw+ip]-verts[j*gw+im];
            Vec3 dz=verts[jp*gw+i]-verts[jm*gw+i];
            normals[j*gw+i]=dz.cross(dx).normalize();
        }
        // 三角形
        for(int j=0;j<gh-1;j++) for(int i=0;i<gw-1;i++){
            int a=j*gw+i,b=j*gw+i+1;
            int c=(j+1)*gw+i,d=(j+1)*gw+i+1;
            tris.push_back({a,b,d});
            tris.push_back({a,d,c});
        }
    }
};

// ─── 软光栅化（三角形填充） ───────────────────────────────────────
struct RasterTriangle {
    Vec3 s[3]; // 屏幕坐标（含ndcZ）
    Vec3 v[3]; // 世界坐标
    Vec3 n[3]; // 世界法线
    Vec2 uv[3];
    float depth[3]; // 视图深度（正值，越大越近）
    float linearDepth[3]; // 0~1线性化深度（越小越近）

    function<Vec3(Vec3,Vec3,Vec2,float,float,Vec3)> shader;

    void rasterize(Framebuffer& fb) const {
        // AABB
        int minX=max(0,(int)min({s[0].x,s[1].x,s[2].x}));
        int maxX=min(fb.width-1,(int)max({s[0].x,s[1].x,s[2].x})+1);
        int minY=max(0,(int)min({s[0].y,s[1].y,s[2].y}));
        int maxY=min(fb.height-1,(int)max({s[0].y,s[1].y,s[2].y})+1);

        // 三角形面积
        float area=(s[1].x-s[0].x)*(s[2].y-s[0].y)
                   -(s[2].x-s[0].x)*(s[1].y-s[0].y);
        if(fabsf(area)<0.5f) return;

        for(int py=minY;py<=maxY;py++) for(int px=minX;px<=maxX;px++){
            float cx=px+0.5f, cy=py+0.5f;
            float w0=(s[1].x-cx)*(s[2].y-cy)-(s[2].x-cx)*(s[1].y-cy);
            float w1=(s[2].x-cx)*(s[0].y-cy)-(s[0].x-cx)*(s[2].y-cy);
            float w2=area-w0-w1;
            if(area>0){ if(w0<0||w1<0||w2<0) continue; }
            else       { if(w0>0||w1>0||w2>0) continue; }

            float b0=w0/area, b1=w1/area, b2=w2/area;
            float zv=b0*s[0].z+b1*s[1].z+b2*s[2].z;
            if(zv<fb.getDepth(px,py)) continue; // 深度测试（NDC z越大越近）
            fb.setDepth(px,py,zv);

            // 插值
            Vec3 worldP=v[0]*b0+v[1]*b1+v[2]*b2;
            Vec3 worldN=(n[0]*b0+n[1]*b1+n[2]*b2).normalize();
            Vec2 uvP={uv[0].x*b0+uv[1].x*b1+uv[2].x*b2,
                      uv[0].y*b0+uv[1].y*b1+uv[2].y*b2};
            float ld=linearDepth[0]*b0+linearDepth[1]*b1+linearDepth[2]*b2;

            float sceneLD = fb.getLinearDepth(px,py);

            Vec3 viewDir=(EYE-worldP).normalize();
            Vec3 col=shader(worldP,worldN,uvP,ld,sceneLD,viewDir);

            fb.setPixel(px,py,col);
            fb.setLinearDepth(px,py,ld);
        }
    }
};

// ─── 地形着色器 ───────────────────────────────────────────────────
Vec3 terrainShader(Vec3 worldPos, Vec3 worldNormal, Vec2 uv,
                   float /*ld*/, float /*sceneLD*/, Vec3 /*viewDir*/)
{
    Vec3 lightDir=(LIGHT-worldPos).normalize();
    float diff=max(0.f,worldNormal.dot(lightDir));
    float diffToon=diff>0.7f?1.f:(diff>0.35f?0.65f:0.3f);

    // 颜色：沙色 or 岩石 or 植被
    float h=worldPos.y;
    Vec3 sand  ={0.85f,0.75f,0.55f};
    Vec3 rock  ={0.5f,0.45f,0.4f};
    Vec3 grass ={0.35f,0.65f,0.3f};
    Vec3 baseCol;
    if(h>1.8f)      baseCol=grass;
    else if(h>0.5f) baseCol=sand.lerp(rock,smoothstep(0.5f,1.8f,h));
    else            baseCol=sand;

    // 噪声纹理
    float tex=fbm({uv.x*8.f,uv.y*8.f});
    baseCol=baseCol*(0.85f+0.15f*tex);
    baseCol=baseCol*diffToon;

    // 水下海底颜色（带水色调）
    if(h<0.2f){
        Vec3 underwaterTint={0.2f,0.55f,0.7f};
        float t=clamp(-h/2.f,0.f,1.f);
        baseCol=baseCol.lerp(baseCol*underwaterTint,t*0.5f);
    }
    return clamp3(baseCol);
}

// ─── 线性深度计算 ─────────────────────────────────────────────────
float computeLinearDepth(float ndcZ, float near=0.5f, float far=60.f){
    // NDC z to [0,1] linear
    float z=(ndcZ+1.f)*0.5f; // [0,1]
    float linZ=near/(far-(far-near)*z+1e-6f);
    return clamp(linZ/near,0.f,1.f);
}

// ─── 投影单个三角形 ───────────────────────────────────────────────
void projectTri(
    const array<int,3>& tri,
    const vector<Vec3>& positions,
    const vector<Vec3>& normalsVec,
    const vector<Vec2>& uvsVec,
    Framebuffer& fb,
    function<Vec3(Vec3,Vec3,Vec2,float,float,Vec3)> shader,
    bool updateLinearDepthOnly = false)
{
    RasterTriangle rt;
    rt.shader=shader;
    bool anyInside=false;
    for(int k=0;k<3;k++){
        int idx=tri[k];
        Vec4 clip=worldToClip(positions[idx]);
        if(clip.w>0.01f) anyInside=true; // 简单剔除
        Vec3 screen=ndcToScreen(clip);
        rt.s[k]=screen;
        rt.v[k]=positions[idx];
        rt.n[k]=normalsVec[idx];
        rt.uv[k]=uvsVec[idx];
        rt.depth[k]=clip.w>0?clip.w:-1.f;
        rt.linearDepth[k]=computeLinearDepth(screen.z);
    }
    if(!anyInside) return;

    // 背面剔除
    float area=(rt.s[1].x-rt.s[0].x)*(rt.s[2].y-rt.s[0].y)
              -(rt.s[2].x-rt.s[0].x)*(rt.s[1].y-rt.s[0].y);
    if(area<0) return; // 背面

    if(updateLinearDepthOnly){
        // 只更新深度缓冲（地形预pass）
        int minX=max(0,(int)min({rt.s[0].x,rt.s[1].x,rt.s[2].x}));
        int maxX=min(fb.width-1,(int)max({rt.s[0].x,rt.s[1].x,rt.s[2].x})+1);
        int minY=max(0,(int)min({rt.s[0].y,rt.s[1].y,rt.s[2].y}));
        int maxY=min(fb.height-1,(int)max({rt.s[0].y,rt.s[1].y,rt.s[2].y})+1);
        if(fabsf(area)<0.5f) return;
        for(int py=minY;py<=maxY;py++) for(int px=minX;px<=maxX;px++){
            float cx=px+0.5f, cy=py+0.5f;
            float w0=(rt.s[1].x-cx)*(rt.s[2].y-cy)-(rt.s[2].x-cx)*(rt.s[1].y-cy);
            float w1=(rt.s[2].x-cx)*(rt.s[0].y-cy)-(rt.s[0].x-cx)*(rt.s[2].y-cy);
            float w2=area-w0-w1;
            if(area>0){ if(w0<0||w1<0||w2<0) continue; }
            else       { if(w0>0||w1>0||w2>0) continue; }
            float b0=w0/area,b1=w1/area,b2=w2/area;
            float ld=rt.linearDepth[0]*b0+rt.linearDepth[1]*b1+rt.linearDepth[2]*b2;
            float zv=rt.s[0].z*b0+rt.s[1].z*b1+rt.s[2].z*b2;
            // 仅更新线性深度（不写color/depth）
            if(zv>=fb.getDepth(px,py)){
                fb.setLinearDepth(px,py,ld);
            }
        }
        return;
    }
    rt.rasterize(fb);
}

// ─── 后处理：轮廓描边（Sobel边缘检测 on depth buffer） ────────────
void outlinePass(Framebuffer& fb){
    vector<Vec3> orig=fb.color;
    for(int y=1;y<fb.height-1;y++) for(int x=1;x<fb.width-1;x++){
        // Sobel on depth
        float d[3][3];
        for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++)
            d[dy+1][dx+1]=fb.getDepth(x+dx,y+dy);

        float gx=(-d[0][0]+d[0][2]-2*d[1][0]+2*d[1][2]-d[2][0]+d[2][2]);
        float gy=(-d[0][0]-2*d[0][1]-d[0][2]+d[2][0]+2*d[2][1]+d[2][2]);
        float edge=sqrtf(gx*gx+gy*gy);

        if(edge>0.06f){
            fb.setPixel(x,y,Vec3(0.05f,0.08f,0.1f)); // 深蓝描边
        } else {
            fb.setPixel(x,y,orig[y*fb.width+x]);
        }
    }
}

// ─── PNG 导出（纯C++，无依赖） ────────────────────────────────────
// 使用 stb_image_write 的单头文件方案
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

void savePNG(const Framebuffer& fb, const string& filename){
    vector<uint8_t> pixels(fb.width*fb.height*3);
    for(int i=0;i<fb.width*fb.height;i++){
        Vec3 c=gamma(clamp3(fb.color[i]));
        pixels[i*3+0]=(uint8_t)(c.x*255.f);
        pixels[i*3+1]=(uint8_t)(c.y*255.f);
        pixels[i*3+2]=(uint8_t)(c.z*255.f);
    }
    stbi_write_png(filename.c_str(),fb.width,fb.height,3,pixels.data(),fb.width*3);
}

// ─── 主函数 ───────────────────────────────────────────────────────
int main(){
    cout<<"[Toon Water Shader] 初始化..."<<endl;
    initMatrices();

    Framebuffer fb(WIDTH,HEIGHT);

    // 初始化天空背景
    for(int y=0;y<HEIGHT;y++){
        float t=1.f-(float)y/HEIGHT;
        Vec3 sky=skyColor(t);
        for(int x=0;x<WIDTH;x++){
            fb.setPixel(x,y,sky);
            fb.setDepth(x,y,-2.f);  // 天空深度最小
            fb.setLinearDepth(x,y,1.f); // 天空线性深度最大
        }
    }

    // ─── Pass 1: 渲染地形（预更新深度/线性深度） ─────────────────
    cout<<"[Pass 1] 渲染地形..."<<endl;
    TerrainPatch terrain;
    terrain.build(80,80, 28.f,28.f, -2.5f);
    for(auto& tri:terrain.tris){
        // 先只更新深度
        projectTri(tri,terrain.verts,terrain.normals,
                   vector<Vec2>(terrain.verts.size(),{0,0}),
                   fb, terrainShader, true);
    }
    // 再正式渲染颜色
    // 重建UV
    {
        int gw=80,gh=80;
        vector<Vec2> tuv(gw*gh);
        for(int j=0;j<gh;j++) for(int i=0;i<gw;i++)
            tuv[j*gw+i]={(float)i/(gw-1),(float)j/(gh-1)};
        for(auto& tri:terrain.tris)
            projectTri(tri,terrain.verts,terrain.normals,tuv,fb,terrainShader,false);
    }

    // ─── Pass 2: 渲染水面 ─────────────────────────────────────────
    cout<<"[Pass 2] 渲染水面..."<<endl;
    WaterSurface water(120,120, 28.f,28.f);

    for(auto& tri:water.tris){
        RasterTriangle rt;
        bool anyIn=false;
        for(int k=0;k<3;k++){
            int idx=tri[k];
            Vec4 clip=worldToClip(water.verts[idx]);
            if(clip.w>0.01f) anyIn=true;
            rt.s[k]=ndcToScreen(clip);
            rt.v[k]=water.verts[idx];
            rt.n[k]=water.normals[idx];
            rt.uv[k]=water.uvs[idx];
            rt.depth[k]=clip.w;
            rt.linearDepth[k]=computeLinearDepth(rt.s[k].z);
        }
        if(!anyIn) continue;
        float area=(rt.s[1].x-rt.s[0].x)*(rt.s[2].y-rt.s[0].y)
                  -(rt.s[2].x-rt.s[0].x)*(rt.s[1].y-rt.s[0].y);
        if(area<0.5f) continue; // 背面剔除+极小三角

        rt.shader=[](Vec3 wp,Vec3 wn,Vec2 uv,float ld,float sld,Vec3 vd){
            return toonWaterShader(wp,wn,uv,ld,sld,vd);
        };
        rt.rasterize(fb);
    }

    // ─── Pass 3: 描边 ─────────────────────────────────────────────
    cout<<"[Pass 3] 描边..."<<endl;
    outlinePass(fb);

    // ─── 保存 ─────────────────────────────────────────────────────
    string outFile="toon_water_output.png";
    savePNG(fb,outFile);
    cout<<"[Done] 已保存: "<<outFile<<" ("<<WIDTH<<"x"<<HEIGHT<<")"<<endl;
    return 0;
}
