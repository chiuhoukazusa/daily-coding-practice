#include <cmath>
#include <vector>
#include <cstdio>
#include <cassert>
#include <cstring>
#include <zlib.h>

struct Vec3 {
    float x,y,z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3&o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3&o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    Vec3 operator*(const Vec3&o)const{return{x*o.x,y*o.y,z*o.z};}
    float dot(const Vec3&o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3&o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?*this/l:Vec3(0,1,0);}
    Vec3& operator+=(const Vec3&o){x+=o.x;y+=o.y;z+=o.z;return*this;}
};
inline Vec3 clamp3(Vec3 v,float lo,float hi){return{std::max(lo,std::min(hi,v.x)),std::max(lo,std::min(hi,v.y)),std::max(lo,std::min(hi,v.z))};}

struct Triangle{Vec3 v[3];Vec3 albedo;Vec3 emission;bool isLight=false;
    Vec3 normal()const{return(v[1]-v[0]).cross(v[2]-v[0]).norm();}};
struct Ray{Vec3 o,d;Ray(Vec3 o,Vec3 d):o(o),d(d.norm()){}};
struct HitInfo{float t=1e30f;Vec3 pos,norm,albedo,emission;bool hit=false,isLight=false;};

bool rayTri(const Ray&r,const Triangle&tri,float&t,float&u,float&v){
    Vec3 e1=tri.v[1]-tri.v[0],e2=tri.v[2]-tri.v[0];
    Vec3 h=r.d.cross(e2);float a=e1.dot(h);
    if(fabsf(a)<1e-7f)return false;
    float f=1/a;Vec3 s=r.o-tri.v[0];
    u=f*s.dot(h);if(u<0||u>1)return false;
    Vec3 q=s.cross(e1);v=f*r.d.dot(q);
    if(v<0||u+v>1)return false;
    t=f*e2.dot(q);return t>1e-4f;
}

HitInfo intersect(const Ray&ray,const std::vector<Triangle>&scene){
    HitInfo hit;
    for(const auto&tri:scene){
        float t,u,v;
        if(rayTri(ray,tri,t,u,v)&&t<hit.t){
            hit.t=t;hit.pos=ray.o+ray.d*t;hit.norm=tri.normal();
            if(hit.norm.dot(ray.d)>0)hit.norm=hit.norm*-1.0f;
            hit.albedo=tri.albedo;hit.emission=tri.emission;hit.isLight=tri.isLight;hit.hit=true;
        }
    }
    return hit;
}

static const Vec3 LIGHT_POS={0,2.45f,0};
static const Vec3 LIGHT_EMIT={12,12,10};

Vec3 directLight(Vec3 pos,Vec3 norm,Vec3 albedo,const std::vector<Triangle>&scene){
    Vec3 toLight=LIGHT_POS-pos;
    float dist=toLight.len();
    Vec3 lDir=toLight/dist;
    float ndotl=std::max(0.0f,norm.dot(lDir));
    if(ndotl<1e-5f)return{};
    Ray sray(pos+norm*0.005f,lDir);
    for(const auto&tri:scene){
        if(tri.isLight)continue;
        float t,u,v;
        if(rayTri(sray,tri,t,u,v)&&t<dist-0.01f)return{};
    }
    float attn=1.96f/(dist*dist+0.1f);
    return LIGHT_EMIT*albedo*(ndotl*attn/3.14159f);
}

std::vector<Triangle> buildScene(){
    std::vector<Triangle> tris;
    float s=2.5f;
    auto addQuad=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 col,bool isL=false,Vec3 emit={}){
        Triangle t1,t2;
        t1.v[0]=a;t1.v[1]=b;t1.v[2]=c;t1.albedo=col;t1.emission=emit;t1.isLight=isL;
        t2.v[0]=a;t2.v[1]=c;t2.v[2]=d;t2.albedo=col;t2.emission=emit;t2.isLight=isL;
        tris.push_back(t1);tris.push_back(t2);
    };
    addQuad({-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s},{0.85f,0.85f,0.85f});
    addQuad({-s,s,-s},{-s,s,s},{s,s,s},{s,s,-s},{0.85f,0.85f,0.85f});
    addQuad({-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s},{0.85f,0.85f,0.85f});
    addQuad({-s,-s,-s},{-s,-s,s},{-s,s,s},{-s,s,-s},{0.8f,0.1f,0.1f});
    addQuad({s,-s,-s},{s,s,-s},{s,s,s},{s,-s,s},{0.1f,0.7f,0.1f});
    float ls=0.7f,ly=s-0.05f;
    addQuad({-ls,ly,-ls},{ls,ly,-ls},{ls,ly,ls},{-ls,ly,ls},{1,1,1},true,{12,12,10});
    return tris;
}

float acesApprox(float x){float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;return std::max(0.0f,std::min(1.0f,(x*(a*x+b))/(x*(c*x+d)+e)));}
Vec3 toneMap(Vec3 v){return{acesApprox(v.x),acesApprox(v.y),acesApprox(v.z)};}
Vec3 gammaCorrect(Vec3 c){auto g=[](float v){return powf(std::max(0.0f,v),1/2.2f);};return{g(c.x),g(c.y),g(c.z)};}

static const int W=800,H=600;

int main(){
    auto scene=buildScene();
    Vec3 camPos={0,0,-4.5f};
    float halfH=tanf(60*3.14159f/360.f),halfW=halfH*W/H;
    
    // Test a few specific pixels
    for(int py:{0,H/2,H-1})for(int px:{0,W/2,W-1}){
        float u=(px+0.5f)/W*2-1, v=1-(py+0.5f)/H*2;
        Vec3 dir=(Vec3(0,0,1)+Vec3(1,0,0)*(u*halfW)+Vec3(0,1,0)*(v*halfH)).norm();
        Ray ray(camPos,dir);
        HitInfo hit=intersect(ray,scene);
        if(hit.hit){
            Vec3 direct=directLight(hit.pos,hit.norm,hit.albedo,scene);
            Vec3 final=gammaCorrect(clamp3(toneMap(direct),0,1));
            printf("px(%d,%d) hit=true t=%.2f pos=(%.2f,%.2f,%.2f) albedo=(%.2f,%.2f,%.2f) direct=(%.4f,%.4f,%.4f) final_byte=(%d,%d,%d)\n",
                px,py,hit.t,hit.pos.x,hit.pos.y,hit.pos.z,
                hit.albedo.x,hit.albedo.y,hit.albedo.z,
                direct.x,direct.y,direct.z,
                (int)(final.x*255+0.5f),(int)(final.y*255+0.5f),(int)(final.z*255+0.5f));
        }else{
            printf("px(%d,%d) NO HIT\n",px,py);
        }
    }
    return 0;
}
