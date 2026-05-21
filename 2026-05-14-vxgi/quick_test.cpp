// Quick test: just test basic render path without VXGI
#include <cmath>
#include <vector>
#include <cstdio>
#include <cassert>
#include <zlib.h>

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    float dot(const Vec3& o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3& o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?*this/l:Vec3(0,1,0);}
};

struct Triangle{Vec3 v[3];Vec3 col;};

bool rayTri(Vec3 ro,Vec3 rd,const Triangle&tri,float&t){
    Vec3 e1=tri.v[1]-tri.v[0],e2=tri.v[2]-tri.v[0];
    Vec3 h=rd.cross(e2);float a=e1.dot(h);
    if(fabsf(a)<1e-7f)return false;
    float f=1/a;Vec3 s=ro-tri.v[0];
    float u=f*s.dot(h);if(u<0||u>1)return false;
    Vec3 q=s.cross(e1);float v=f*rd.dot(q);
    if(v<0||u+v>1)return false;
    t=f*e2.dot(q);return t>1e-4f;
}

int main(){
    // Count how many pixels hit scene (simple check)
    float s=2.5f;
    std::vector<Triangle> scene;
    auto addQuad=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 col){
        Triangle t1,t2;
        t1.v[0]=a;t1.v[1]=b;t1.v[2]=c;t1.col=col;
        t2.v[0]=a;t2.v[1]=c;t2.v[2]=d;t2.col=col;
        scene.push_back(t1);scene.push_back(t2);
    };
    addQuad({-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s},{0.85f,0.85f,0.85f}); // floor
    addQuad({-s,s,-s},{-s,s,s},{s,s,s},{s,s,-s},{0.85f,0.85f,0.85f}); // ceiling
    addQuad({-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s},{0.85f,0.85f,0.85f}); // back wall
    addQuad({-s,-s,-s},{-s,-s,s},{-s,s,s},{-s,s,-s},{0.8f,0.1f,0.1f}); // left
    addQuad({s,-s,-s},{s,s,-s},{s,s,s},{s,-s,s},{0.1f,0.7f,0.1f}); // right

    Vec3 camPos={0,0,-4.5f};
    int W=800,H=600,hitCount=0;
    float halfH2=tanf(60.0f*3.14159f/360.0f),halfW2=halfH2*W/H;
    
    for(int py=0;py<H;py++){
        for(int px=0;px<W;px++){
            float u=(px+0.5f)/W*2-1, v=1-(py+0.5f)/H*2;
            Vec3 dir=(Vec3(0,0,1)+Vec3(1,0,0)*(u*halfW2)+Vec3(0,1,0)*(v*halfH2)).norm();
            float bestT=1e30f;
            for(auto&tri:scene){float t;if(rayTri(camPos,dir,tri,t)&&t<bestT)bestT=t;}
            if(bestT<1e29f)hitCount++;
        }
    }
    printf("Pixels that hit scene: %d / %d (%.1f%%)\n",hitCount,W*H,hitCount*100.0f/(W*H));
    return 0;
}
