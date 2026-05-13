/*
 * VXGI - Voxel Global Illumination Renderer (CLEAN REWRITE)
 * 输出：vxgi_output.png
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cassert>
#include <zlib.h>

// ──────── Math ────────

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
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
};
inline Vec3 operator*(float s,Vec3 v){return v*s;}
inline Vec3 clamp3(Vec3 v,float lo,float hi){
    return{std::max(lo,std::min(hi,v.x)),std::max(lo,std::min(hi,v.y)),std::max(lo,std::min(hi,v.z))};
}
inline Vec3 vmax(Vec3 a,Vec3 b){return{std::max(a.x,b.x),std::max(a.y,b.y),std::max(a.z,b.z)};}

struct Vec4{
    float x,y,z,w;
    Vec4(float x=0,float y=0,float z=0,float w=0):x(x),y(y),z(z),w(w){}
    Vec4(Vec3 v,float w):x(v.x),y(v.y),z(v.z),w(w){}
    Vec4 operator+(Vec4 o)const{return{x+o.x,y+o.y,z+o.z,w+o.w};}
    Vec4& operator+=(Vec4 o){x+=o.x;y+=o.y;z+=o.z;w+=o.w;return*this;}
    Vec4 operator*(float s)const{return{x*s,y*s,z*s,w*s};}
    Vec3 xyz()const{return{x,y,z};}
};

// ──────── Scene ────────

struct Triangle {
    Vec3 v[3];
    Vec3 albedo;
    Vec3 emission;
    bool isLight = false;
    Vec3 normal() const { return (v[1]-v[0]).cross(v[2]-v[0]).norm(); }
};

static bool rayTriangle(Vec3 ro, Vec3 rd, const Triangle& tri, float& tOut) {
    Vec3 e1=tri.v[1]-tri.v[0], e2=tri.v[2]-tri.v[0];
    Vec3 h=rd.cross(e2); float a=e1.dot(h);
    if(fabsf(a)<1e-7f) return false;
    float f=1/a; Vec3 s=ro-tri.v[0];
    float u=f*s.dot(h); if(u<0||u>1) return false;
    Vec3 q=s.cross(e1); float v=f*rd.dot(q);
    if(v<0||u+v>1) return false;
    tOut=f*e2.dot(q); return tOut>1e-4f;
}

struct HitInfo {
    float t=1e30f;
    Vec3 pos,norm,albedo,emission;
    bool hit=false, isLight=false;
};

static HitInfo intersect(Vec3 ro, Vec3 rd, const std::vector<Triangle>& scene) {
    HitInfo h;
    for(const auto& tri:scene) {
        float t; if(!rayTriangle(ro,rd,tri,t)) continue;
        if(t<h.t) {
            h.t=t; h.pos=ro+rd*t; h.norm=tri.normal();
            if(h.norm.dot(rd)>0) h.norm=h.norm*-1;
            h.albedo=tri.albedo; h.emission=tri.emission;
            h.isLight=tri.isLight; h.hit=true;
        }
    }
    return h;
}

static std::vector<Triangle> buildScene() {
    std::vector<Triangle> T;
    float s=2.5f;
    auto addQuad=[&](Vec3 a,Vec3 b,Vec3 c,Vec3 d,Vec3 col,bool isL=false,Vec3 emit={}){
        Triangle t1,t2;
        t1.v[0]=a;t1.v[1]=b;t1.v[2]=c;t1.albedo=col;t1.emission=emit;t1.isLight=isL;
        t2.v[0]=a;t2.v[1]=c;t2.v[2]=d;t2.albedo=col;t2.emission=emit;t2.isLight=isL;
        T.push_back(t1);T.push_back(t2);
    };
    addQuad({-s,-s,-s},{s,-s,-s},{s,-s,s},{-s,-s,s},{0.85f,0.85f,0.85f});          // floor
    addQuad({-s,s,-s},{-s,s,s},{s,s,s},{s,s,-s},{0.85f,0.85f,0.85f});              // ceiling
    addQuad({-s,-s,s},{s,-s,s},{s,s,s},{-s,s,s},{0.85f,0.85f,0.85f});              // back wall
    addQuad({-s,-s,-s},{-s,-s,s},{-s,s,s},{-s,s,-s},{0.85f,0.15f,0.12f});          // left (red)
    addQuad({s,-s,-s},{s,s,-s},{s,s,s},{s,-s,s},{0.12f,0.75f,0.15f});              // right (green)
    // front wall removed - camera looks in from outside
    float ls=0.7f, ly=s-0.05f;
    addQuad({-ls,ly,-ls},{ls,ly,-ls},{ls,ly,ls},{-ls,ly,ls},{1,1,1},true,{12,12,10}); // light
    // Tall box
    {float bx=-1.0f,bz=0.8f,bw=0.6f,bh=1.2f;
     float x0=bx-bw,x1=bx+bw,y0=-s,y1=y0+2*bh,z0=bz-bw,z1=bz+bw;
     Vec3 col{0.8f,0.8f,0.8f};
     addQuad({x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},col);
     addQuad({x0,y0,z1},{x0,y1,z1},{x1,y1,z1},{x1,y0,z1},col);
     addQuad({x0,y0,z0},{x0,y1,z0},{x0,y1,z1},{x0,y0,z1},col);
     addQuad({x1,y0,z0},{x1,y0,z1},{x1,y1,z1},{x1,y1,z0},col);
     addQuad({x0,y1,z0},{x0,y1,z1},{x1,y1,z1},{x1,y1,z0},col);}
    // Short box
    {float bx=1.0f,bz=0.2f,bw=0.6f,bh=0.6f;
     float x0=bx-bw,x1=bx+bw,y0=-s,y1=y0+2*bh,z0=bz-bw,z1=bz+bw;
     Vec3 col{0.8f,0.8f,0.8f};
     addQuad({x0,y0,z0},{x1,y0,z0},{x1,y1,z0},{x0,y1,z0},col);
     addQuad({x0,y0,z1},{x0,y1,z1},{x1,y1,z1},{x1,y0,z1},col);
     addQuad({x0,y0,z0},{x0,y1,z0},{x0,y1,z1},{x0,y0,z1},col);
     addQuad({x1,y0,z0},{x1,y0,z1},{x1,y1,z1},{x1,y1,z0},col);
     addQuad({x0,y1,z0},{x0,y1,z1},{x1,y1,z1},{x1,y1,z0},col);}
    return T;
}

// ──────── Direct Light ────────

static const Vec3 LPOS={0,2.45f,0};
static const Vec3 LEMIT={12,12,10};
static const float LAREA=1.96f;

static Vec3 directLight(Vec3 pos, Vec3 norm, Vec3 albedo, const std::vector<Triangle>& scene) {
    Vec3 toL=LPOS-pos; float dist=toL.len(); Vec3 lDir=toL/dist;
    float ndotl=std::max(0.0f,norm.dot(lDir));
    if(ndotl<1e-5f) return {};
    Vec3 sro=pos+norm*0.005f;
    for(const auto& tri:scene) {
        if(tri.isLight) continue;
        float t; if(rayTriangle(sro,lDir,tri,t)&&t<dist-0.01f) return {};
    }
    float attn=LAREA/(dist*dist+0.1f);
    return LEMIT*albedo*(ndotl*attn/float(M_PI));
}

// ──────── Voxel Grid ────────

static const int VR=64;
static const float VS=6.0f/VR; // voxel size

struct Voxel {
    Vec3 albedo, normal, radiance;
    float opacity=0;
};

struct VoxelGrid {
    std::vector<Voxel> data;
    Vec3 origin={-3,-3,-3};
    VoxelGrid():data(VR*VR*VR){}
    
    int idx(int x,int y,int z)const{return x+VR*(y+VR*z);}
    bool ok(int x,int y,int z)const{return x>=0&&y>=0&&z>=0&&x<VR&&y<VR&&z<VR;}
    Voxel& at(int x,int y,int z){return data[idx(x,y,z)];}
    const Voxel& at(int x,int y,int z)const{return data[idx(x,y,z)];}
    
    Vec3 center(int x,int y,int z)const{
        return origin+Vec3((x+0.5f)*VS,(y+0.5f)*VS,(z+0.5f)*VS);
    }
    
    // Sample with spatial blur for cone LOD
    Vec4 sample(Vec3 p, float r) const {
        Vec3 lp=p-origin;
        float fx=lp.x/VS-0.5f, fy=lp.y/VS-0.5f, fz=lp.z/VS-0.5f;
        int x0=(int)floorf(fx), y0=(int)floorf(fy), z0=(int)floorf(fz);
        
        int kr=std::max(1,(int)ceilf(r/VS));
        kr=std::min(kr,3);
        
        Vec4 acc={};
        float totalW=0;
        for(int dz=-kr;dz<=kr+1;dz++)
        for(int dy=-kr;dy<=kr+1;dy++)
        for(int dx=-kr;dx<=kr+1;dx++) {
            int cx=x0+dx, cy=y0+dy, cz=z0+dz;
            if(!ok(cx,cy,cz)) continue;
            const Voxel& v=at(cx,cy,cz);
            if(v.opacity<0.01f) continue;
            float w=1.0f/(1.0f+dx*dx+dy*dy+dz*dz);
            acc+=Vec4(v.radiance,v.opacity)*w;
            totalW+=w;
        }
        if(totalW>0) acc=acc*(1.0f/totalW);
        return acc;
    }
};

static void voxelizeScene(VoxelGrid& g, const std::vector<Triangle>& scene) {
    // Sample each voxel center with 6 directional rays
    static const Vec3 dirs[6]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for(int iz=0;iz<VR;iz++)
    for(int iy=0;iy<VR;iy++)
    for(int ix=0;ix<VR;ix++) {
        Vec3 c=g.center(ix,iy,iz);
        Vec3 sumAlb={}, sumNorm={}; int hits=0;
        for(const auto& d:dirs) {
            float bestT=VS*0.8f;
            for(const auto& tri:scene) {
                float t; if(rayTriangle(c,d,tri,t)&&t<bestT) {
                    bestT=t; sumAlb+=tri.albedo; sumNorm+=tri.normal(); hits++;
                }
            }
        }
        if(hits>0) {
            Voxel& v=g.at(ix,iy,iz);
            v.albedo=sumAlb*(1.0f/hits);
            v.normal=sumNorm.norm();
            v.opacity=1.0f;
        }
    }
}

static void injectDirectLight(VoxelGrid& g, const std::vector<Triangle>& scene) {
    for(int iz=0;iz<VR;iz++)
    for(int iy=0;iy<VR;iy++)
    for(int ix=0;ix<VR;ix++) {
        Voxel& v=g.at(ix,iy,iz);
        if(v.opacity<0.5f) continue;
        Vec3 c=g.center(ix,iy,iz);
        Vec3 dlight=directLight(c,v.normal,v.albedo,scene);
        // Store radiance (direct * albedo for GI bounce)
        v.radiance=dlight + v.albedo*0.05f; // add small ambient
    }
}

// ──────── Cone Tracing ────────

static Vec4 traceCone(const VoxelGrid& g, Vec3 pos, Vec3 dir, float aperture, float maxD) {
    float dist=VS*2.0f;
    Vec4 acc={};
    while(dist<maxD && acc.w<0.95f) {
        Vec3 p=pos+dir*dist;
        float cr=std::max(aperture*dist, VS*0.5f);
        Vec4 s=g.sample(p,cr);
        float alpha=s.w*(1.0f-acc.w);
        acc.x+=alpha*s.x; acc.y+=alpha*s.y; acc.z+=alpha*s.z; acc.w+=alpha;
        dist+=std::max(VS, cr*0.5f);
    }
    return acc;
}

static Vec3 buildTangent(Vec3 n) {
    Vec3 up=fabsf(n.y)<0.9f?Vec3(0,1,0):Vec3(1,0,0);
    return n.cross(up).norm();
}

static Vec3 diffuseGI(const VoxelGrid& g, Vec3 pos, Vec3 n, float strength) {
    Vec3 t=buildTangent(n), b=n.cross(t).norm();
    float sa=sinf(0.7854f);
    Vec3 dirs[5]={n,(n+t*sa).norm(),(n-t*sa).norm(),(n+b*sa).norm(),(n-b*sa).norm()};
    float wts[5]={0.28f,0.18f,0.18f,0.18f,0.18f};
    Vec3 gi={};
    Vec3 start=pos+n*(VS*3.0f);
    for(int i=0;i<5;i++) {
        Vec4 c=traceCone(g,start,dirs[i],0.5f,5.0f);
        gi+=c.xyz()*(std::max(0.0f,dirs[i].dot(n))*wts[i]);
    }
    return gi*strength;
}

static float voxelAO(const VoxelGrid& g, Vec3 pos, Vec3 n) {
    Vec3 t=buildTangent(n), b=n.cross(t).norm();
    float sa=sinf(0.5236f), ca=cosf(0.5236f);
    Vec3 dirs[5]={n,(n*ca+t*sa).norm(),(n*ca-t*sa).norm(),(n*ca+b*sa).norm(),(n*ca-b*sa).norm()};
    float occ=0;
    Vec3 start=pos+n*(VS*2.0f);
    for(int i=0;i<5;i++) {
        Vec4 c=traceCone(g,start,dirs[i],0.2f,2.5f);
        occ+=c.w;
    }
    return 1.0f-std::min(occ/5.0f,1.0f)*0.65f;
}

static Vec3 specularGI(const VoxelGrid& g, Vec3 pos, Vec3 n, Vec3 viewDir, float roughness) {
    Vec3 refl=(viewDir-n*(2*n.dot(viewDir))).norm();
    Vec3 start=pos+n*(VS*2.0f);
    Vec4 c=traceCone(g,start,refl,roughness*0.3f+0.02f,5.0f);
    return c.xyz()*(1.0f-roughness);
}

// ──────── Render ────────

static const int W=800, H=600;

static float aces(float x){float a=2.51f,b=0.03f,c=2.43f,d=0.59f,e=0.14f;
    return std::max(0.0f,std::min(1.0f,(x*(a*x+b))/(x*(c*x+d)+e)));}
static Vec3 toneMap(Vec3 c){return{aces(c.x),aces(c.y),aces(c.z)};}
static Vec3 gammaCorrect(Vec3 c){
    auto g=[](float v){return powf(std::max(0.0f,v),1.0f/2.2f);};
    return{g(c.x),g(c.y),g(c.z)};
}

static void render(std::vector<Vec3>& fb, const std::vector<Triangle>& scene, const VoxelGrid& grid) {
    Vec3 camPos={0,0,-4.5f};
    float halfH=tanf(float(M_PI)/6.0f); // 60 FOV
    float halfW=halfH*float(W)/H;
    
    for(int py=0;py<H;py++)
    for(int px=0;px<W;px++) {
        float u=(px+0.5f)/W*2-1;
        float v=1-(py+0.5f)/H*2;
        Vec3 dir=(Vec3(0,0,1)+Vec3(1,0,0)*(u*halfW)+Vec3(0,1,0)*(v*halfH)).norm();
        
        HitInfo hit=intersect(camPos,dir,scene);
        Vec3 color={};
        
        if(!hit.hit) {
            color={0.02f,0.02f,0.03f};
        } else if(hit.isLight) {
            color=hit.emission*0.1f;
        } else {
            Vec3 direct=directLight(hit.pos,hit.norm,hit.albedo,scene);
            Vec3 diffGI=diffuseGI(grid,hit.pos,hit.norm,1.2f)*hit.albedo;
            Vec3 specGI=specularGI(grid,hit.pos,hit.norm,dir,0.7f);
            float ao=voxelAO(grid,hit.pos,hit.norm);
            color=(direct+diffGI+specGI)*ao;
        }
        fb[py*W+px]=color;
    }
}

// ──────── PNG Write ────────

static bool writePNG(const char* fn, const std::vector<Vec3>& fb) {
    // Convert to bytes
    std::vector<unsigned char> raw(W*H*3);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
        Vec3 c=fb[y*W+x];
        c=clamp3(toneMap(c),0,1);
        c=clamp3(gammaCorrect(c),0,1);
        int i=(y*W+x)*3;
        raw[i+0]=(unsigned char)(c.x*255+0.5f);
        raw[i+1]=(unsigned char)(c.y*255+0.5f);
        raw[i+2]=(unsigned char)(c.z*255+0.5f);
    }
    // Filter (type 0 = None)
    std::vector<unsigned char> filt;
    filt.reserve((W*3+1)*H);
    for(int y=0;y<H;y++) {
        filt.push_back(0);
        for(int x=0;x<W*3;x++) filt.push_back(raw[y*W*3+x]);
    }
    // Compress
    uLongf csz=compressBound((uLong)filt.size());
    std::vector<unsigned char> comp(csz);
    if(compress2(comp.data(),&csz,filt.data(),(uLong)filt.size(),6)!=Z_OK) return false;
    comp.resize(csz);
    // Write PNG
    FILE* f=fopen(fn,"wb");
    if(!f) return false;
    auto w32=[&](unsigned v){unsigned char b[4]={(unsigned char)(v>>24),(unsigned char)(v>>16),(unsigned char)(v>>8),(unsigned char)v};fwrite(b,1,4,f);};
    auto chunk=[&](const char* tag,const unsigned char* d,size_t l){
        w32((unsigned)l);fwrite(tag,1,4,f);
        if(l>0)fwrite(d,1,l,f);
        unsigned crc2=crc32(crc32(0L,Z_NULL,0),(const Bytef*)tag,4);
        if(l>0)crc2=crc32(crc2,d,(uInt)l);
        w32(crc2);
    };
    unsigned char sig[8]={137,80,78,71,13,10,26,10};fwrite(sig,1,8,f);
    unsigned char ihdr[13]={(unsigned char)(W>>24),(unsigned char)(W>>16),(unsigned char)(W>>8),(unsigned char)W,
                             (unsigned char)(H>>24),(unsigned char)(H>>16),(unsigned char)(H>>8),(unsigned char)H,
                             8,2,0,0,0};
    chunk("IHDR",ihdr,13);
    chunk("IDAT",comp.data(),comp.size());
    w32(0);fwrite("IEND",1,4,f);
    unsigned crc2=crc32(crc32(0L,Z_NULL,0),(const Bytef*)"IEND",4);w32(crc2);
    fclose(f);
    return true;
}

// ──────── Main ────────

int main() {
    printf("VXGI Voxel Global Illumination Renderer\n");
    printf("Image: %dx%d  Grid: %dx%dx%d\n",W,H,VR,VR,VR);

    printf("[1/5] Building scene...\n");
    auto scene=buildScene();
    printf("      Triangles: %zu\n",scene.size());

    printf("[2/5] Voxelizing...\n");
    VoxelGrid grid;
    voxelizeScene(grid,scene);
    int filled=0; for(const auto& v:grid.data) if(v.opacity>0.5f) filled++;
    printf("      Filled voxels: %d/%d\n",filled,VR*VR*VR);

    printf("[3/5] Injecting direct light...\n");
    injectDirectLight(grid,scene);

    printf("[4/5] Rendering (VXGI cone tracing)...\n");
    std::vector<Vec3> fb(W*H);
    render(fb,scene,grid);
    
    // DEBUG sample
    printf("      Sample fb[0,0]=(%.4f,%.4f,%.4f)\n",fb[0].x,fb[0].y,fb[0].z);
    printf("      Sample fb[%d,%d]=(%.4f,%.4f,%.4f)\n",W/2,H/2,
           fb[(H/2)*W+(W/2)].x,fb[(H/2)*W+(W/2)].y,fb[(H/2)*W+(W/2)].z);

    printf("[5/5] Writing output...\n");
    if(!writePNG("vxgi_output.png",fb)) { fprintf(stderr,"PNG write failed\n"); return 1; }
    printf("Done! → vxgi_output.png\n");
    return 0;
}
