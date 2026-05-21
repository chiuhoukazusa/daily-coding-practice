#include <cmath>
#include <vector>
#include <cstdio>
#include <cassert>

struct Vec3{float x,y,z;Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(Vec3 o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(Vec3 o)const{return{x-o.x,y-o.y,z-o.z};}  // dummy
    Vec3 operator-(Vec3 o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    Vec3& operator+=(Vec3 o){x+=o.x;y+=o.y;z+=o.z;return*this;}
    float dot(Vec3 o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(Vec3 o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float len()const{return sqrtf(x*x+y*y+z*z);}
    Vec3 norm()const{float l=len();return l>1e-6f?*this/l:Vec3(0,1,0);}};

struct Vec4{float x,y,z,w;Vec4(float x=0,float y=0,float z=0,float w=0):x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v,float w):x(v.x),y(v.y),z(v.z),w(w){}
    Vec4 operator+(Vec4 o)const{return{x+o.x,y+o.y,z+o.z,w+o.w};}
    Vec4& operator+=(Vec4 o){x+=o.x;y+=o.y;z+=o.z;w+=o.w;return*this;}
    Vec4 operator*(float s)const{return{x*s,y*s,z*s,w*s};}
    Vec3 xyz()const{return{x,y,z};}};

static const int VR=64;
static const float VS=6.0f/VR;

struct Voxel{Vec3 alb,norm,rad;float op=0;};
struct VoxelGrid{
    std::vector<Voxel> data;Vec3 origin={-3,-3,-3};
    VoxelGrid():data(VR*VR*VR){}
    int idx(int x,int y,int z)const{return x+VR*(y+VR*z);}
    bool ok(int x,int y,int z)const{return x>=0&&y>=0&&z>=0&&x<VR&&y<VR&&z<VR;}
    Voxel& at(int x,int y,int z){return data[idx(x,y,z)];}
    const Voxel& at(int x,int y,int z)const{return data[idx(x,y,z)];}
    Vec3 center(int x,int y,int z)const{return origin+Vec3((x+0.5f)*VS,(y+0.5f)*VS,(z+0.5f)*VS);}
    Vec4 sample(Vec3 p,float r)const{
        Vec3 lp=p-origin;float fx=lp.x/VS-0.5f,fy=lp.y/VS-0.5f,fz=lp.z/VS-0.5f;
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

Vec4 traceCone(const VoxelGrid& g,Vec3 pos,Vec3 dir,float aperture,float maxD){
    float dist=VS*2.0f;Vec4 acc={};
    while(dist<maxD&&acc.w<0.95f){
        Vec3 p=pos+dir*dist;float cr=std::max(aperture*dist,VS*0.5f);
        Vec4 s=g.sample(p,cr);
        float alpha=s.w*(1.0f-acc.w);
        acc.x+=alpha*s.x;acc.y+=alpha*s.y;acc.z+=alpha*s.z;acc.w+=alpha;
        dist+=std::max(VS,cr*0.5f);
    }
    return acc;
}

Vec3 buildTangent(Vec3 n){Vec3 up=fabsf(n.y)<0.9f?Vec3(0,1,0):Vec3(1,0,0);return n.cross(up).norm();}

float voxelAO(const VoxelGrid& g,Vec3 pos,Vec3 n){
    Vec3 t=buildTangent(n),b=n.cross(t).norm();
    float sa=sinf(0.5236f),ca=cosf(0.5236f);
    Vec3 dirs[5]={n,(n*ca+t*sa).norm(),(n*ca-t*sa).norm(),(n*ca+b*sa).norm(),(n*ca-b*sa).norm()};
    float occ=0;Vec3 start=pos+n*(VS*2.0f);
    for(int i=0;i<5;i++){Vec4 c=traceCone(g,start,dirs[i],0.2f,2.5f);
        printf("  AO cone[%d]: dir=(%.2f,%.2f,%.2f) occ=%.4f\n",i,dirs[i].x,dirs[i].y,dirs[i].z,c.w);
        occ+=c.w;}
    return 1.0f-std::min(occ/5.0f,1.0f)*0.65f;
}

int main(){
    // Build minimal grid: floor at y=-2.5
    VoxelGrid g;
    // Fill a few floor voxels around the test point
    // Test pos: back wall center (0,0,2.5) with norm (0,0,-1)
    Vec3 pos={0,0,2.4f}; Vec3 n={0,0,-1};
    
    // Fill the back wall voxels
    int iz_wall=VR-1; // z ≈ 2.55
    for(int iy=0;iy<VR;iy++)for(int ix=0;ix<VR;ix++){
        Voxel& v=g.at(ix,iy,iz_wall);v.op=1;v.rad={0.3f,0.3f,0.3f};
    }
    
    printf("=== AO at back wall surface (should have low occlusion from behind) ===\n");
    printf("pos=(%.2f,%.2f,%.2f) norm=(%.2f,%.2f,%.2f)\n",pos.x,pos.y,pos.z,n.x,n.y,n.z);
    printf("start=pos+n*2VS=(%.2f,%.2f,%.2f)\n",pos.x+n.x*2*VS,pos.y+n.y*2*VS,pos.z+n.z*2*VS);
    float ao=voxelAO(g,pos,n);
    printf("AO result: %.4f (expected ~1.0 for open space)\n\n",ao);
    
    // With fully dense grid
    for(auto& v:g.data){v.op=1;v.rad={0.3f,0.3f,0.3f};}
    printf("=== AO with fully dense grid ===\n");
    ao=voxelAO(g,pos,n);
    printf("AO result: %.4f (expected ~0.35 or low)\n",ao);
    return 0;
}
