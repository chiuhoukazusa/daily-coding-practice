// Test single pixel render with full VXGI logic
#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cassert>

struct Vec3{float x,y,z;Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(Vec3 o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(Vec3 o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator*(Vec3 o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    Vec3& operator+=(Vec3 o){x+=o.x;y+=o.y;z+=o.z;return*this;}
    float dot(Vec3 o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(Vec3 o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?*this/l:Vec3(0,1,0);}
    void print(const char* label)const{printf("%s=(%.4f,%.4f,%.4f)",label,x,y,z);}};

struct Vec4{float x,y,z,w;Vec4(float x=0,float y=0,float z=0,float w=0):x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v,float w):x(v.x),y(v.y),z(v.z),w(w){}
    Vec4 operator+(Vec4 o)const{return{x+o.x,y+o.y,z+o.z,w+o.w};}
    Vec4& operator+=(Vec4 o){x+=o.x;y+=o.y;z+=o.z;w+=o.w;return*this;}
    Vec4 operator*(float s)const{return{x*s,y*s,z*s,w*s};}
    Vec3 xyz()const{return{x,y,z};}};

static const int VR=64;
static const float VS=6.0f/VR;

struct Voxel{Vec3 alb,nm,rad;float op=0;};
struct VoxelGrid{
    std::vector<Voxel> data;Vec3 origin={-3,-3,-3};
    VoxelGrid():data(VR*VR*VR){}
    int idx(int x,int y,int z)const{return x+VR*(y+VR*z);}
    bool ok(int x,int y,int z)const{return x>=0&&y>=0&&z>=0&&x<VR&&y<VR&&z<VR;}
    Voxel& at(int x,int y,int z){return data[idx(x,y,z)];}
    const Voxel& at(int x,int y,int z)const{return data[idx(x,y,z)];}
    Vec3 center(int x,int y,int z)const{return origin+Vec3((x+0.5f)*VS,(y+0.5f)*VS,(z+0.5f)*VS);}
    Vec4 sample(Vec3 p,float r)const{
        Vec3 lp=p-origin;
        float fx=lp.x/VS-0.5f,fy=lp.y/VS-0.5f,fz=lp.z/VS-0.5f;
        int x0=(int)floorf(fx),y0=(int)floorf(fy),z0=(int)floorf(fz);
        int kr=std::max(1,(int)ceilf(r/VS));kr=std::min(kr,3);
        Vec4 acc={};float totalW=0;
        for(int dz=-kr;dz<=kr+1;dz++)for(int dy=-kr;dy<=kr+1;dy++)for(int dx=-kr;dx<=kr+1;dx++){
            int cx=x0+dx,cy=y0+dy,cz=z0+dz;
            if(!ok(cx,cy,cz))continue;
            const Voxel& v=at(cx,cy,cz);if(v.op<0.01f)continue;
            float w=1.0f/(1.0f+dx*dx+dy*dy+dz*dz);
            acc+=Vec4(v.rad,v.op)*w;totalW+=w;
        }
        if(totalW>0)acc=acc*(1.0f/totalW);
        return acc;
    }
};

struct Triangle{Vec3 v[3];Vec3 alb,emit;bool isL=false;Vec3 normal()const{return(v[1]-v[0]).cross(v[2]-v[0]).norm();}};
bool rayT(Vec3 ro,Vec3 rd,const Triangle&tri,float&t){
    Vec3 e1=tri.v[1]-tri.v[0],e2=tri.v[2]-tri.v[0],h=rd.cross(e2);float a=e1.dot(h);
    if(fabsf(a)<1e-7f)return false;float f=1/a;Vec3 s=ro-tri.v[0];float u=f*s.dot(h);
    if(u<0||u>1)return false;Vec3 q=s.cross(e1);float v=f*rd.dot(q);
    if(v<0||u+v>1)return false;t=f*e2.dot(q);return t>1e-4f;}
struct Hit{float t=1e30f;Vec3 pos,nm,alb,emit;bool hit=false,isL=false;};
Hit intersect(Vec3 ro,Vec3 rd,const std::vector<Triangle>&sc){
    Hit h;for(const auto&tri:sc){float t;if(!rayT(ro,rd,tri,t))continue;if(t<h.t){h.t=t;h.pos=ro+rd*t;h.nm=tri.normal();if(h.nm.dot(rd)>0)h.nm=h.nm*-1;h.alb=tri.alb;h.emit=tri.emit;h.isL=tri.isL;h.hit=true;}}return h;}

static const Vec3 LPOS={0,2.45f,0};static const Vec3 LEMIT={12,12,10};
Vec3 directLight(Vec3 pos,Vec3 nm,Vec3 alb,const std::vector<Triangle>&sc){
    Vec3 toL=LPOS-pos;float dist=toL.len();Vec3 lDir=toL/dist;
    float ndotl=std::max(0.0f,nm.dot(lDir));if(ndotl<1e-5f)return{};
    Vec3 sro=pos+nm*0.005f;
    for(const auto&tri:sc){if(tri.isL)continue;float t;if(rayT(sro,lDir,tri,t)&&t<dist-0.01f)return{};}
    float attn=1.96f/(dist*dist+0.1f);return LEMIT*alb*(ndotl*attn/float(M_PI));}

Vec4 traceCone(const VoxelGrid& g,Vec3 pos,Vec3 dir,float ap,float maxD){
    float dist=VS*2.0f;Vec4 acc={};int steps=0;
    while(dist<maxD&&acc.w<0.95f){
        Vec3 p=pos+dir*dist;float cr=std::max(ap*dist,VS*0.5f);
        Vec4 s=g.sample(p,cr);float alpha=s.w*(1.0f-acc.w);
        acc.x+=alpha*s.x;acc.y+=alpha*s.y;acc.z+=alpha*s.z;acc.w+=alpha;
        dist+=std::max(VS,cr*0.5f);steps++;
    }
    printf("    traceCone(start=(%.2f,%.2f,%.2f) dir=(%.2f,%.2f,%.2f)): w=%.4f steps=%d\n",pos.x,pos.y,pos.z,dir.x,dir.y,dir.z,acc.w,steps);
    return acc;}

Vec3 buildTangent(Vec3 n){Vec3 up=fabsf(n.y)<0.9f?Vec3(0,1,0):Vec3(1,0,0);return n.cross(up).norm();}

float voxelAO(const VoxelGrid&g,Vec3 pos,Vec3 n){
    Vec3 t=buildTangent(n),b=n.cross(t).norm();
    float sa=sinf(0.5236f),ca=cosf(0.5236f);
    Vec3 dirs[5]={n,(n*ca+t*sa).norm(),(n*ca-t*sa).norm(),(n*ca+b*sa).norm(),(n*ca-b*sa).norm()};
    float occ=0;Vec3 start=pos+n*(VS*2.0f);
    printf("  AO start=(%.4f,%.4f,%.4f)\n",start.x,start.y,start.z);
    for(int i=0;i<5;i++){Vec4 c=traceCone(g,start,dirs[i],0.2f,2.5f);occ+=c.w;}
    float ao=1.0f-std::min(occ/5.0f,1.0f)*0.65f;
    printf("  AO occ=%.4f ao=%.4f\n",occ,ao);return ao;}

Vec3 diffuseGI(const VoxelGrid&g,Vec3 pos,Vec3 n,float str){
    Vec3 t=buildTangent(n),b=n.cross(t).norm();float sa=sinf(0.7854f);
    Vec3 dirs[5]={n,(n+t*sa).norm(),(n-t*sa).norm(),(n+b*sa).norm(),(n-b*sa).norm()};
    float wts[5]={0.28f,0.18f,0.18f,0.18f,0.18f};
    Vec3 gi={};Vec3 start=pos+n*(VS*3.0f);
    for(int i=0;i<5;i++){Vec4 c=traceCone(g,start,dirs[i],0.5f,5.0f);gi+=c.xyz()*(std::max(0.0f,dirs[i].dot(n))*wts[i]);}
    return gi*str;}

std::vector<Triangle> buildScene(){
    std::vector<Triangle> T;float s=2.5f;
    auto addQ=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 col,bool isL=false,Vec3 em={}){
        Triangle t1,t2;t1.v[0]=a;t1.v[1]=b;t1.v[2]=c;t1.alb=col;t1.isL=isL;t1.emit=em;
        t2.v[0]=a;t2.v[1]=c;t2.v[2]=d;t2.alb=col;t2.isL=isL;t2.emit=em;T.push_back(t1);T.push_back(t2);};
    addQ({-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s},{0.85f,0.85f,0.85f});
    addQ({-s,s,-s},{-s,s,s},{s,s,s},{s,s,-s},{0.85f,0.85f,0.85f});
    addQ({-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s},{0.85f,0.85f,0.85f});
    addQ({-s,-s,-s},{-s,-s,s},{-s,s,s},{-s,s,-s},{0.85f,0.15f,0.12f});
    addQ({s,-s,-s},{s,s,-s},{s,s,s},{s,-s,s},{0.12f,0.75f,0.15f});
    addQ({-s,-s,-s},{-s,s,-s},{s,s,-s},{s,-s,-s},{0.75f,0.75f,0.75f});
    float ls=0.7f,ly=s-0.05f;
    addQ({-ls,ly,-ls},{ls,ly,-ls},{ls,ly,ls},{-ls,ly,ls},{1,1,1},true,{12,12,10});
    {float bx=-1.0f,bz=0.8f,bw=0.6f,bh=1.2f;
     float x0=bx-bw,x1=bx+bw,y0=-s,y1=y0+2*bh,z0=bz-bw,z1=bz+bw;Vec3 col{0.8f,0.8f,0.8f};
     addQ({x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},col);addQ({x0,y0,z1},{x0,y1,z1},{x1,y1,z1},{x1,y0,z1},col);
     addQ({x0,y0,z0},{x0,y1,z0},{x0,y1,z1},{x0,y0,z1},col);addQ({x1,y0,z0},{x1,y0,z1},{x1,y1,z1},{x1,y1,z0},col);
     addQ({x0,y1,z0},{x0,y1,z1},{x1,y1,z1},{x1,y1,z0},col);}
    {float bx=1.0f,bz=0.2f,bw=0.6f,bh=0.6f;
     float x0=bx-bw,x1=bx+bw,y0=-s,y1=y0+2*bh,z0=bz-bw,z1=bz+bw;Vec3 col{0.8f,0.8f,0.8f};
     addQ({x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},col);addQ({x0,y0,z1},{x0,y1,z1},{x1,y1,z1},{x1,y0,z1},col);
     addQ({x0,y0,z0},{x0,y1,z0},{x0,y1,z1},{x0,y0,z1},col);addQ({x1,y0,z0},{x1,y0,z1},{x1,y1,z1},{x1,y1,z0},col);
     addQ({x0,y1,z0},{x0,y1,z1},{x1,y1,z1},{x1,y1,z0},col);}
    return T;}

int main(){
    auto scene=buildScene();
    VoxelGrid grid;
    // Simple voxelization
    static const Vec3 ds[6]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for(int iz=0;iz<VR;iz++)for(int iy=0;iy<VR;iy++)for(int ix=0;ix<VR;ix++){
        Vec3 c=grid.center(ix,iy,iz);Vec3 sumA={},sumN={};int hits=0;
        for(const auto&d:ds){float bestT=VS*0.8f;
            for(const auto&tri:scene){float t;if(rayT(c,d,tri,t)&&t<bestT){bestT=t;sumA+=tri.alb;sumN+=tri.normal();hits++;}}
        }
        if(hits>0){Voxel&v=grid.at(ix,iy,iz);v.alb=sumA*(1.0f/hits);v.nm=sumN.norm();v.op=1.0f;}}
    // Inject direct light
    for(int iz=0;iz<VR;iz++)for(int iy=0;iy<VR;iy++)for(int ix=0;ix<VR;ix++){
        Voxel&v=grid.at(ix,iy,iz);if(v.op<0.5f)continue;
        Vec3 c=grid.center(ix,iy,iz);
        v.rad=directLight(c,v.nm,v.alb,scene)+v.alb*0.05f;}
    
    // Test: center pixel
    Vec3 camPos={0,0,-4.5f};
    float halfH=tanf(float(M_PI)/6.0f),halfW=halfH*800.0f/600.0f;
    // px=400, py=300 (center)
    float u=0.001f,v=0.001f;
    Vec3 dir=(Vec3(0,0,1)+Vec3(1,0,0)*(u*halfW)+Vec3(0,1,0)*(v*halfH)).norm();
    printf("dir=(%.4f,%.4f,%.4f)\n",dir.x,dir.y,dir.z);
    Hit hit=intersect(camPos,dir,scene);
    printf("hit=%d pos=(%.4f,%.4f,%.4f) nm=(%.4f,%.4f,%.4f) alb=(%.4f,%.4f,%.4f)\n",
           hit.hit,hit.pos.x,hit.pos.y,hit.pos.z,hit.nm.x,hit.nm.y,hit.nm.z,hit.alb.x,hit.alb.y,hit.alb.z);
    if(hit.hit&&!hit.isL){
        Vec3 direct=directLight(hit.pos,hit.nm,hit.alb,scene);
        printf("direct=(%.4f,%.4f,%.4f)\n",direct.x,direct.y,direct.z);
        printf("Computing AO...\n");
        float ao=voxelAO(grid,hit.pos,hit.nm);
        printf("ao=%.4f\n",ao);
        Vec3 diffGI=diffuseGI(grid,hit.pos,hit.nm,1.2f)*hit.alb;
        printf("diffGI=(%.4f,%.4f,%.4f)\n",diffGI.x,diffGI.y,diffGI.z);
        Vec3 color=(direct+diffGI)*ao;
        printf("FINAL color=(%.4f,%.4f,%.4f)\n",color.x,color.y,color.z);
    }
    return 0;
}
