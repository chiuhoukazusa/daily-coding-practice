#include <cmath>
#include <vector>
#include <cstdio>

struct Vec3{float x,y,z;Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(Vec3 o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(Vec3 o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator*(Vec3 o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    float dot(Vec3 o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(Vec3 o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?*this/l:Vec3(0,1,0);}};
struct Tri{Vec3 v[3];Vec3 alb;bool isL=false;Vec3 emit;Vec3 normal()const{return(v[1]-v[0]).cross(v[2]-v[0]).norm();}};
bool rayT(Vec3 ro,Vec3 rd,const Tri&t,float&tout){
    Vec3 e1=t.v[1]-t.v[0],e2=t.v[2]-t.v[0],h=rd.cross(e2);
    float a=e1.dot(h);if(fabsf(a)<1e-7f)return false;
    float f=1/a;Vec3 s=ro-t.v[0];float u=f*s.dot(h);if(u<0||u>1)return false;
    Vec3 q=s.cross(e1);float v=f*rd.dot(q);if(v<0||u+v>1)return false;
    tout=f*e2.dot(q);return tout>1e-4f;}
struct Hit{float t=1e30f;Vec3 pos,norm,alb,emit;bool hit=false,isL=false;};
Hit intersect(Vec3 ro,Vec3 rd,const std::vector<Tri>&sc){
    Hit h;for(const auto&tri:sc){float t;if(!rayT(ro,rd,tri,t))continue;if(t<h.t){h.t=t;h.pos=ro+rd*t;h.norm=tri.normal();if(h.norm.dot(rd)>0)h.norm=h.norm*-1;h.alb=tri.alb;h.emit=tri.emit;h.isL=tri.isL;h.hit=true;}}return h;}

int main(){
    float s=2.5f;std::vector<Tri> T;
    auto addQ=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 col,bool isL=false,Vec3 em={}){
        Tri t1,t2;t1.v[0]=a;t1.v[1]=b;t1.v[2]=c;t1.alb=col;t1.isL=isL;t1.emit=em;
        t2.v[0]=a;t2.v[1]=c;t2.v[2]=d;t2.alb=col;t2.isL=isL;t2.emit=em;T.push_back(t1);T.push_back(t2);};
    addQ({-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s},{0.85f,0.85f,0.85f});
    addQ({-s,s,-s},{-s,s,s},{s,s,s},{s,s,-s},{0.85f,0.85f,0.85f});
    addQ({-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s},{0.85f,0.85f,0.85f});
    addQ({-s,-s,-s},{-s,-s,s},{-s,s,s},{-s,s,-s},{0.85f,0.1f,0.1f});
    addQ({s,-s,-s},{s,s,-s},{s,s,s},{s,-s,s},{0.1f,0.75f,0.1f});
    float ls=0.7f,ly=s-0.05f;
    addQ({-ls,ly,-ls},{ls,ly,-ls},{ls,ly,ls},{-ls,ly,ls},{1,1,1},true,{12,12,10});
    
    // Test center ray
    Vec3 camPos={0,0,-4.5f};
    float halfH=tanf(float(M_PI)/6);
    Vec3 dir={0,0,1};
    Hit h=intersect(camPos,dir,T);
    printf("Center ray hit=%d t=%.3f pos=(%.2f,%.2f,%.2f) alb=(%.2f,%.2f,%.2f) isL=%d\n",
           h.hit,h.t,h.pos.x,h.pos.y,h.pos.z,h.alb.x,h.alb.y,h.alb.z,h.isL);
    
    // Direct light
    Vec3 LPOS={0,2.45f,0}; Vec3 LEMIT={12,12,10}; float LAREA=1.96f;
    Vec3 pos=h.pos, norm=h.norm, alb=h.alb;
    Vec3 toL=LPOS-pos;float dist=toL.len();Vec3 lDir=toL/dist;
    float ndotl=std::max(0.0f,norm.dot(lDir));
    printf("ndotl=%.4f dist=%.4f\n",ndotl,dist);
    Vec3 sro=pos+norm*0.005f;
    bool shadowed=false;
    for(const auto&tri:T){if(tri.isL)continue;float t;if(rayT(sro,lDir,tri,t)&&t<dist-0.01f){shadowed=true;printf("Shadowed by tri\n");break;}}
    if(!shadowed){
        float attn=LAREA/(dist*dist+0.1f);
        Vec3 r=LEMIT*alb*(ndotl*attn/float(M_PI));
        printf("Direct light: (%.4f,%.4f,%.4f)\n",r.x,r.y,r.z);
    }
    return 0;
}
