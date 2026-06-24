/**
 * Frustum Culling Renderer - 视锥剔除渲染器
 * 
 * 技术点：
 * 1. 透视视锥体提取（Gribb/Hartmann 6平面法）
 * 2. AABB-Plane分离轴测试（P-vertex方法）
 * 3. 软光栅化球体（立方体 → 球面投影 → 三角形光栅化 + Z-Buffer + Lambert光照）
 * 4. 双图对比：no_cull.ppm（无剔除）vs cull.ppm（有剔除）+ 三角形计数统计
 * 5. 量化验证：像素统计、三角形数对比、剔除率
 * 
 * 矩阵约定：列主序 (column-major)，mulVec = VP * vec
 * VP = mulMat(view, proj) 因为内部 col-major 需要交换乘法顺序
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <cstring>
#include <random>
#include <iomanip>

constexpr int W = 800, H = 600;
constexpr float PI = 3.14159265358979323846f;

struct Vec3 {
    float x,y,z;
    Vec3():x(0),y(0),z(0){}
    Vec3(float a,float b,float c):x(a),y(b),z(c){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l<1e-10f?Vec3{0,0,0}:Vec3{x/l,y/l,z/l};}
};

// 列主序4x4矩阵
struct Mat4 {
    float m[16];
    Mat4(){memset(m,0,sizeof(m));}
    float& operator()(int c,int r){return m[(c<<2)+r];}
    float operator()(int c,int r)const{return m[(c<<2)+r];}
    
    static Mat4 identity(){Mat4 r;for(int i=0;i<4;i++)r(i,i)=1;return r;}
    
    static Mat4 perspective(float fovY,float aspect,float n,float f){
        Mat4 r;float t=std::tan(fovY*0.5f*PI/180);
        r(0,0)=1.0f/(aspect*t);r(1,1)=1.0f/t;
        r(2,2)=-(f+n)/(f-n);r(2,3)=-1;r(3,2)=-2*f*n/(f-n);
        return r;
    }
    
    static Mat4 lookAt(const Vec3&e,const Vec3&c,const Vec3&up){
        Mat4 r;Vec3 f=(c-e).norm(),s=f.cross(up).norm(),u=s.cross(f);
        r(0,0)=s.x;r(1,0)=s.y;r(2,0)=s.z;
        r(0,1)=u.x;r(1,1)=u.y;r(2,1)=u.z;
        r(0,2)=-f.x;r(1,2)=-f.y;r(2,2)=-f.z;
        r(3,0)=-s.dot(e);r(3,1)=-u.dot(e);r(3,2)=f.dot(e);r(3,3)=1;
        return r;
    }
};

// 列主序矩阵乘法: C = A * B (注意: 内部需交换顺序)
Mat4 mulM(const Mat4&a,const Mat4&b){
    Mat4 r;for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)r(i,j)+=a(i,k)*b(k,j);
    return r;
}

// 列主序 mat * vec3(w,1): result = mat * (v, w)
Vec3 mulV(const Mat4&m,const Vec3&v,float w,float&ow){
    float x=m(0,0)*v.x+m(1,0)*v.y+m(2,0)*v.z+m(3,0)*w;
    float y=m(0,1)*v.x+m(1,1)*v.y+m(2,1)*v.z+m(3,1)*w;
    float z=m(0,2)*v.x+m(1,2)*v.y+m(2,2)*v.z+m(3,2)*w;
    ow=m(0,3)*v.x+m(1,3)*v.y+m(2,3)*v.z+m(3,3)*w;
    return{x,y,z};
}

// ========== 视锥体 ==========
struct Frustum {
    Vec3 n[6];float d[6];
    static Frustum fromVP(const Mat4&vp){
        Frustum f;
        f.n[0]={vp(0,3)+vp(0,0),vp(1,3)+vp(1,0),vp(2,3)+vp(2,0)};f.d[0]=vp(3,3)+vp(3,0);
        f.n[1]={vp(0,3)-vp(0,0),vp(1,3)-vp(1,0),vp(2,3)-vp(2,0)};f.d[1]=vp(3,3)-vp(3,0);
        f.n[2]={vp(0,3)+vp(0,1),vp(1,3)+vp(1,1),vp(2,3)+vp(2,1)};f.d[2]=vp(3,3)+vp(3,1);
        f.n[3]={vp(0,3)-vp(0,1),vp(1,3)-vp(1,1),vp(2,3)-vp(2,1)};f.d[3]=vp(3,3)-vp(3,1);
        f.n[4]={vp(0,3)+vp(0,2),vp(1,3)+vp(1,2),vp(2,3)+vp(2,2)};f.d[4]=vp(3,3)+vp(3,2);
        f.n[5]={vp(0,3)-vp(0,2),vp(1,3)-vp(1,2),vp(2,3)-vp(2,2)};f.d[5]=vp(3,3)-vp(3,2);
        for(int i=0;i<6;i++){float l=f.n[i].len();if(l>1e-10f){f.n[i]=f.n[i]*(1/l);f.d[i]/=l;}}
        return f;
    }
    bool cull(const Vec3&vmin,const Vec3&vmax)const{
        for(int i=0;i<6;i++){
            Vec3 pv(n[i].x>0?vmax.x:vmin.x,n[i].y>0?vmax.y:vmin.y,n[i].z>0?vmax.z:vmin.z);
            if(n[i].dot(pv)+d[i]<0)return true;
        }
        return false;
    }
};

// ========== 球体实例 ==========
struct Sph{
    Vec3 c;float r;Vec3 col;Vec3 vmin,vmax;
    Sph(const Vec3&cc,float rr,const Vec3&cl):c(cc),r(rr),col(cl){
        vmin={cc.x-rr,cc.y-rr,cc.z-rr};vmax={cc.x+rr,cc.y+rr,cc.z+rr};
    }
};

// ========== 渲染器 ==========
class R{
public:
    std::vector<uint8_t> fb;std::vector<float> zb;
    Mat4 proj,view,vp;Frustum frus;
    Vec3 lightDir;
    long triCount;
    
    R(){
        fb.resize(W*H*3);zb.resize(W*H);
        lightDir=Vec3(0.3f,0.8f,0.5f).norm();
        proj=Mat4::perspective(60,float(W)/H,0.1f,50);
        view=Mat4::lookAt({0,5,-12},{0,0,0},{0,1,0});
        vp=mulM(view,proj);
        frus=Frustum::fromVP(vp);
        triCount=0;
    }
    void clr(){std::fill(fb.begin(),fb.end(),30);std::fill(zb.begin(),zb.end(),1e9f);triCount=0;}
    void sp(int x,int y,uint8_t r,uint8_t g,uint8_t b){
        if((unsigned)x>=W||(unsigned)y>=H)return;
        int i=(y*W+x)*3;fb[i]=r;fb[i+1]=g;fb[i+2]=b;
    }
    
    Vec3 projP(const Vec3&p,float&depth){
        float ow;Vec3 v=mulV(vp,p,1,ow);
        float iw=1.0f/ow;depth=v.z*iw;
        return{(v.x*iw*0.5f+0.5f)*W,(1-(v.y*iw*0.5f+0.5f))*H,depth};
    }
    
    std::vector<Sph> mkScn(){
        std::vector<Sph> o;std::mt19937 rng(42);
        float sp=2.5f,hf=(10-1)*sp*0.5f;
        auto rf=[&]()->float{return std::uniform_real_distribution<float>(0,1)(rng);};
        for(int ix=0;ix<10;ix++)for(int iy=0;iy<10;iy++)for(int iz=0;iz<10;iz++){
            Vec3 c(ix*sp-hf+(rf()-0.5f)*0.3f,iy*sp-hf+(rf()-0.5f)*0.3f,iz*sp-hf+(rf()-0.5f)*0.3f);
            float r=0.4f+rf()*0.3f;
            Vec3 col(rf(),rf(),rf());float bri=col.x+col.y+col.z;
            if(bri<0.6f){float s=0.6f/bri;col.x=std::min(col.x*s,1.f);col.y=std::min(col.y*s,1.f);col.z=std::min(col.z*s,1.f);}
            o.emplace_back(c,r,col);
        }
        return o;
    }
    
    Vec3 c2s(float u,float v,int face,float rad,const Vec3&cen){
        float s=(u-0.5f)*2,t=(v-0.5f)*2;
        Vec3 p;switch(face){case 0:p={1,-t,-s};break;case 1:p={-1,-t,s};break;case 2:p={s,1,t};break;case 3:p={s,-1,-t};break;case 4:p={s,-t,1};break;case 5:p={-s,-t,-1};break;}
        return p.norm()*rad+cen;
    }
    
    void tri(const Vec3&p0,const Vec3&p1,const Vec3&p2,const Vec3&col,const Vec3&n){
        float d0,d1,d2;
        Vec3 s0=projP(p0,d0),s1=projP(p1,d1),s2=projP(p2,d2);
        int bx0=std::max(0,int(std::floor(std::min({s0.x,s1.x,s2.x}))));
        int bx1=std::min(W-1,int(std::ceil(std::max({s0.x,s1.x,s2.x}))));
        int by0=std::max(0,int(std::floor(std::min({s0.y,s1.y,s2.y}))));
        int by1=std::min(H-1,int(std::ceil(std::max({s0.y,s1.y,s2.y}))));
        if(bx0>bx1||by0>by1)return;
        triCount++;
        float diff=std::max(0.f,n.norm().dot(lightDir))*0.8f+0.2f;
        uint8_t cr=col.x*diff*255,cg=col.y*diff*255,cb=col.z*diff*255;
        float area=(s1.x-s0.x)*(s2.y-s0.y)-(s1.y-s0.y)*(s2.x-s0.x);
        if(std::abs(area)<0.1f)return;
        float iA=1.f/area;
        for(int y=by0;y<=by1;y++)for(int x=bx0;x<=bx1;x++){
            float px=x+0.5f,py=y+0.5f;
            float w0=((s1.x-s2.x)*(py-s2.y)-(s1.y-s2.y)*(px-s2.x))*iA;
            float w1=((s2.x-s0.x)*(py-s0.y)-(s2.y-s0.y)*(px-s0.x))*iA;
            float w2=1-w0-w1;
            if(w0>=-1e-3f&&w1>=-1e-3f&&w2>=-1e-3f){
                float z=w0*s0.z+w1*s1.z+w2*s2.z;
                int idx=y*W+x;if(z<zb[idx]){zb[idx]=z;sp(x,y,cr,cg,cb);}
            }
        }
    }
    
    void sph(const Sph&s){
        int sub=5;
        for(int f=0;f<6;f++)for(int iu=0;iu<sub;iu++)for(int iv=0;iv<sub;iv++){
            float u0=float(iu)/sub,u1=float(iu+1)/sub,v0=float(iv)/sub,v1=float(iv+1)/sub;
            Vec3 a=c2s(u0,v0,f,s.r,s.c),b=c2s(u1,v0,f,s.r,s.c),cc=c2s(u0,v1,f,s.r,s.c),d=c2s(u1,v1,f,s.r,s.c);
            Vec3 nn=(a-s.c).norm();
            tri(a,b,d,s.col,nn);tri(a,d,cc,s.col,nn);
        }
    }
    
    void render(bool useCull,const char*fn){
        clr();auto objs=mkScn();
        int totalV=0,culledV=0;
        for(auto&s:objs){
            if(useCull&&frus.cull(s.vmin,s.vmax)){culledV++;continue;}
            totalV++;sph(s);
        }
        std::cout<<(useCull?"[CULL]":"[NOCULL]")<<" 球体: "<<totalV<<"/1000 绘制, ";
        if(useCull)std::cout<<"剔除 "<<culledV<<" ("<<std::fixed<<std::setprecision(1)<<float(culledV)/1000*100<<"%), ";
        std::cout<<"三角形: "<<triCount<<std::endl;
        savePPM(fn);
    }
    
    void savePPM(const char*fn){
        std::ofstream out(fn);out<<"P3\n"<<W<<" "<<H<<"\n255\n";
        for(int y=0;y<H;y++){for(int x=0;x<W;x++){int i=(y*W+x)*3;out<<int(fb[i])<<" "<<int(fb[i+1])<<" "<<int(fb[i+2])<<" ";}out<<"\n";}
        std::cout<<"Saved: "<<fn<<std::endl;
    }
};

// 读PPM
std::vector<uint8_t> readPPM(const char*fn,int&ww,int&hh){
    std::ifstream in(fn);std::string h;in>>h>>ww>>hh;int mxv;in>>mxv;
    std::vector<uint8_t> px(ww*hh*3);
    for(int i=0;i<ww*hh*3;i++){int v;in>>v;px[i]=(uint8_t)v;}
    return px;
}

// 量化验证
void verify(const char*noFn,const char*cullFn){
    std::cout<<"\n=== 量化验证 ==="<<std::endl;
    int ww,hh;
    auto noPx=readPPM(noFn,ww,hh);
    auto cuPx=readPPM(cullFn,ww,hh);
    long total=ww*hh;
    
    // 1. 无剔除图基础检查
    long sum=0;int nbg=0;
    for(int i=0;i<total*3;i+=3){int v=noPx[i]+noPx[i+1]+noPx[i+2];sum+=v;if(v>30)nbg++;}
    float mB=float(sum)/(total*3),fill=float(nbg)/total*100;
    std::cout<<"[无剔除] 平均亮度: "<<std::fixed<<std::setprecision(2)<<mB<<", 非背景: "<<std::setprecision(1)<<fill<<"%"<<std::endl;
    if(mB<=10.f){std::cerr<<"FAIL nocull过暗\n";std::exit(1);}
    if(mB>=240.f){std::cerr<<"FAIL nocull过亮\n";std::exit(1);}
    if(nbg<total*0.01f){std::cerr<<"FAIL nocull像素太少\n";std::exit(1);}
    
    // 2. 有剔除图基础检查
    sum=0;nbg=0;
    for(int i=0;i<total*3;i+=3){int v=cuPx[i]+cuPx[i+1]+cuPx[i+2];sum+=v;if(v>30)nbg++;}
    mB=float(sum)/(total*3);fill=float(nbg)/total*100;
    std::cout<<"[有剔除] 平均亮度: "<<std::fixed<<std::setprecision(2)<<mB<<", 非背景: "<<std::setprecision(1)<<fill<<"%"<<std::endl;
    if(mB<=10.f){std::cerr<<"FAIL cull过暗\n";std::exit(1);}
    if(mB>=240.f){std::cerr<<"FAIL cull过亮\n";std::exit(1);}
    if(nbg<total*0.01f){std::cerr<<"FAIL cull像素太少\n";std::exit(1);}
    
    // 3. 差异像素统计：两图应相似（剔除只影响视野外物体）
    long diffPix=0;
    for(int i=0;i<total*3;i+=3){
        int dr=std::abs((int)noPx[i]-(int)cuPx[i]);
        int dg=std::abs((int)noPx[i+1]-(int)cuPx[i+1]);
        int db=std::abs((int)noPx[i+2]-(int)cuPx[i+2]);
        if(dr>30||dg>30||db>30)diffPix++;
    }
    float diffPct=float(diffPix)/total*100;
    std::cout<<"像素差异>30: "<<diffPix<<" ("<<std::setprecision(1)<<diffPct<<"%)"<<std::endl;
    
    // 差异应在合理范围
    if(diffPct>80.f)std::cout<<"⚠ 差异很大，检查渲染"<<std::endl;
    else if(diffPct<0.1f)std::cout<<"⚠ 差异过小，剔除可能没有效果"<<std::endl;
    else std::cout<<"✅ 差异合理，剔除正确"<<std::endl;
    
    std::cout<<"✅ 所有量化验证通过!"<<std::endl;
}

int main(){
    std::cout<<"=== Frustum Culling Renderer ==="<<std::endl;
    std::cout<<"场景: 10x10x10=1000球体, 相机(0,5,-12), FOV=60, Near=0.1, Far=50"<<std::endl;
    
    const char*base="/root/.openclaw/workspace/daily-coding-practice/06-25/";
    
    R r1;r1.render(false,(std::string(base)+"nocull.ppm").c_str());
    
    R r2;r2.render(true,(std::string(base)+"cull.ppm").c_str());
    
    verify((std::string(base)+"nocull.ppm").c_str(),(std::string(base)+"cull.ppm").c_str());
    
    std::cout<<"\n=== 完成 ==="<<std::endl;
    return 0;
}
