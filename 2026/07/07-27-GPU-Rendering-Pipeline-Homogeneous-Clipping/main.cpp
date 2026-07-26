/**
 * GPU Rendering Pipeline: Homogeneous Clipping & Perspective-Correct Interpolation
 * 
 * Implements core GPU pipeline stages:
 * 1. Model-View-Projection transformation
 * 2. Homogeneous 6-plane clipping (Sutherland-Hodgman)
 * 3. Perspective division (clip → NDC)
 * 4. Viewport transform (NDC → screen)
 * 5. Perspective-correct vertex attribute interpolation using 1/w
 * 6. Side-by-side comparison: perspective-correct vs naive linear interpolation
 *
 * 2026-07-27
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <array>

// ---- 3D Math ----
struct Vec3 { float x,y,z; };
struct Vec4 { float x,y,z,w; };

struct Mat4 { float m[16]; 
    Mat4() { for(int i=0;i<16;i++) m[i]=(i%5==0)?1:0; }
    static Mat4 rotateY(float a){ Mat4 r; float c=cos(a),s=sin(a); r.m[0]=c;r.m[2]=s;r.m[8]=-s;r.m[10]=c; return r; }
    static Mat4 rotateX(float a){ Mat4 r; float c=cos(a),s=sin(a); r.m[5]=c;r.m[6]=-s;r.m[9]=s;r.m[10]=c; return r; }
    static Mat4 translate(float x,float y,float z){ Mat4 r; r.m[3]=x;r.m[7]=y;r.m[11]=z; return r; }
    static Mat4 perspective(float fovY,float aspect,float n,float f){
        Mat4 r; r.m[0]=1/(aspect*tan(fovY*0.5f)); r.m[5]=1/tan(fovY*0.5f);
        r.m[10]=-(f+n)/(f-n); r.m[11]=-2*f*n/(f-n); r.m[14]=-1; r.m[15]=0; return r;
    }
    Mat4 operator*(const Mat4& b) const {
        Mat4 r; for(int i=0;i<4;i++) for(int j=0;j<4;j++) { r.m[i*4+j]=0;
            for(int k=0;k<4;k++) r.m[i*4+j]+=m[i*4+k]*b.m[k*4+j]; } return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {m[0]*v.x+m[1]*v.y+m[2]*v.z+m[3]*v.w, m[4]*v.x+m[5]*v.y+m[6]*v.z+m[7]*v.w,
                m[8]*v.x+m[9]*v.y+m[10]*v.z+m[11]*v.w, m[12]*v.x+m[13]*v.y+m[14]*v.z+m[15]*v.w};
    }
};

// ---- Vertex data ----
struct ClipVertex { Vec4 clip; Vec3 color; };

ClipVertex transformVertex(Vec3 pos, Vec3 col, const Mat4& mvp) {
    Vec4 world{pos.x,pos.y,pos.z,1};
    Vec4 clip=mvp*world;
    return {clip, col};
}

// ---- Homogeneous Clipping (OpenGL convention: -w<=x<=w, -w<=y<=w, -w<=z<=w) ----
float dotPlane(Vec4 p, Vec4 v) { return p.x*v.x+p.y*v.y+p.z*v.z+p.w*v.w; }

Vec4 clipPlanes[] = {
    { 1,0,0, 1}, // left:   x + w >= 0  →  x >= -w
    {-1,0,0, 1}, // right: -x + w >= 0  →  x <=  w
    { 0,1,0, 1}, // bottom: y + w >= 0  →  y >= -w
    { 0,-1,0, 1}, // top:   -y + w >= 0  →  y <=  w
    { 0,0, 1, 1}, // near:   z + w >= 0  →  z >= -w
    { 0,0,-1, 1}, // far:   -z + w >= 0  →  z <=  w
};

ClipVertex lerpCV(const ClipVertex& a, const ClipVertex& b, float t) {
    ClipVertex r;
    r.clip.x=a.clip.x+(b.clip.x-a.clip.x)*t; r.clip.y=a.clip.y+(b.clip.y-a.clip.y)*t;
    r.clip.z=a.clip.z+(b.clip.z-a.clip.z)*t; r.clip.w=a.clip.w+(b.clip.w-a.clip.w)*t;
    r.color.x=a.color.x+(b.color.x-a.color.x)*t; r.color.y=a.color.y+(b.color.y-a.color.y)*t;
    r.color.z=a.color.z+(b.color.z-a.color.z)*t;
    return r;
}

std::vector<ClipVertex> clipPolygon(std::vector<ClipVertex> poly, const Vec4& plane) {
    std::vector<ClipVertex> out;
    if(poly.empty()) return out;
    int n=(int)poly.size();
    for(int i=0;i<n;i++){
        const ClipVertex& cur=poly[i],&nxt=poly[(i+1)%n];
        float dCur=dotPlane(plane,cur.clip), dNxt=dotPlane(plane,nxt.clip);
        bool ci=(dCur>=0), ni=(dNxt>=0);
        if(ci){ out.push_back(cur); if(!ni) out.push_back(lerpCV(cur,nxt,dCur/(dCur-dNxt))); }
        else if(ni) out.push_back(lerpCV(cur,nxt,dCur/(dCur-dNxt)));
    }
    return out;
}

std::vector<ClipVertex> clipTriangle(const ClipVertex& v0, const ClipVertex& v1, const ClipVertex& v2) {
    std::vector<ClipVertex> poly={v0,v1,v2};
    for(int p=0;p<6&&!poly.empty();p++) poly=clipPolygon(poly,clipPlanes[p]);
    return poly;
}

// ---- Perspective division & viewport ----
struct ScreenVertex { float x,y,depth,invW; Vec3 color; };

ScreenVertex toScreen(const ClipVertex& cv, int W, int H) {
    float inv=1.0f/cv.clip.w;
    return {(cv.clip.x*inv*0.5f+0.5f)*W, (cv.clip.y*inv*0.5f+0.5f)*H,
            cv.clip.z*inv*0.5f+0.5f, inv, cv.color};
}

// ---- Rasterization ----
float edgeFn(float ax,float ay,float bx,float by,float px,float py){ return (bx-ax)*(py-ay)-(by-ay)*(px-ax); }

Vec3 interpPerspectiveCorrect(float a,float b,float g,
                               const ScreenVertex& v0,const ScreenVertex& v1,const ScreenVertex& v2){
    float iw=a*v0.invW+b*v1.invW+g*v2.invW;
    if(iw<1e-10f) iw=1e-10f;
    return Vec3{
        (a*v0.color.x*v0.invW+b*v1.color.x*v1.invW+g*v2.color.x*v2.invW)/iw,
        (a*v0.color.y*v0.invW+b*v1.color.y*v1.invW+g*v2.color.y*v2.invW)/iw,
        (a*v0.color.z*v0.invW+b*v1.color.z*v1.invW+g*v2.color.z*v2.invW)/iw
    };
}

Vec3 interpLinear(float a,float b,float g,const ScreenVertex& v0,const ScreenVertex& v1,const ScreenVertex& v2){
    return Vec3{a*v0.color.x+b*v1.color.x+g*v2.color.x,
                a*v0.color.y+b*v1.color.y+g*v2.color.y,
                a*v0.color.z+b*v1.color.z+g*v2.color.z};
}

void rasterizeTriangle(ScreenVertex a,ScreenVertex b,ScreenVertex c,
                       std::vector<float>& zbuf,std::vector<uint8_t>& img,int W,int H,int mode){
    float minX=std::min({a.x,b.x,c.x}),maxX=std::max({a.x,b.x,c.x});
    float minY=std::min({a.y,b.y,c.y}),maxY=std::max({a.y,b.y,c.y});
    int bx0=std::max(0,(int)floor(minX)),bx1=std::min(W-1,(int)ceil(maxX));
    int by0=std::max(0,(int)floor(minY)),by1=std::min(H-1,(int)ceil(maxY));
    for(int y=by0;y<=by1;y++) for(int x=bx0;x<=bx1;x++){
        float px=x+0.5f,py=y+0.5f;
        float e0=edgeFn(b.x,b.y,c.x,c.y,px,py),e1=edgeFn(c.x,c.y,a.x,a.y,px,py),e2=edgeFn(a.x,a.y,b.x,b.y,px,py);
        bool inside=(e0>=0&&e1>=0&&e2>=0)||(e0<=0&&e1<=0&&e2<=0);
        if(!inside) continue;
        float area=e0+e1+e2; if(fabs(area)<1e-10f) continue;
        float ia=1/area, A=e0*ia, B=e1*ia, G=e2*ia;
        if(A<0||B<0||G<0) continue;
        float depth=A*a.depth+B*b.depth+G*c.depth;
        int idx=y*W+x;
        if(depth>=zbuf[idx]) continue;
        zbuf[idx]=depth;
        Vec3 col=(mode==0)?interpPerspectiveCorrect(A,B,G,a,b,c):interpLinear(A,B,G,a,b,c);
        int i3=idx*3; img[i3]=std::min(255,std::max(0,(int)(col.x*255)));
        img[i3+1]=std::min(255,std::max(0,(int)(col.y*255)));
        img[i3+2]=std::min(255,std::max(0,(int)(col.z*255)));
    }
}

void savePPM(const std::string& fn, const std::vector<uint8_t>& img, int W, int H) {
    std::ofstream f(fn,std::ios::binary);
    f<<"P6\n"<<W<<" "<<H<<"\n255\n"; f.write((const char*)img.data(),img.size());
    std::cout<<"Saved "<<fn<<" ("<<img.size()<<" bytes)\n";
}

// ---- Scene ----
struct Tri { Vec3 v0,v1,v2,c0,c1,c2; };

int main(){
    const int W=800, H=600;
    
    // Camera
    Mat4 view=Mat4::translate(0,-0.3f,-4)*Mat4::rotateX(-0.15f);
    Mat4 proj=Mat4::perspective(55*M_PI/180, (float)W/H, 0.5f, 20);
    Mat4 mvp=proj*view;
    
    // Scene: 3 cubes + ground, each face has different per-vertex colors
    Vec3 r{1,0,0},g{0,1,0},b{0,0,1},w{1,1,1},y{1,1,0},c{0,1,1},m{1,0,1},dg{0.3f,0.3f,0.3f};
    std::vector<Tri> scene;
    auto q=[&](Vec3 p0,Vec3 p1,Vec3 p2,Vec3 p3,Vec3 c0,Vec3 c1,Vec3 c2,Vec3 c3){
        scene.push_back({p0,p1,p2,c0,c1,c2}); scene.push_back({p0,p2,p3,c0,c2,c3});};
    // Cube 1 center
    float s=0.6f;
    q({-s,-s, s},{ s,-s, s},{ s, s, s},{-s, s, s},r,g,w,r);
    q({ s,-s,-s},{-s,-s,-s},{-s, s,-s},{ s, s,-s},b,b,g,b);
    q({-s,-s,-s},{-s,-s, s},{-s, s, s},{-s, s,-s},c,c,w,c);
    q({ s,-s, s},{ s,-s,-s},{ s, s,-s},{ s, s, s},m,m,y,m);
    q({-s, s, s},{ s, s, s},{ s, s,-s},{-s, s,-s},g,y,r,g);
    q({-s,-s,-s},{ s,-s,-s},{ s,-s, s},{-s,-s, s},dg,dg,dg,dg);
    // Cube 2 right + back
    float s2=0.35f, ox=1.3f, oy=0.25f, oz=-0.6f;
    q({ox-s2,oy-s2,oz+s2},{ox+s2,oy-s2,oz+s2},{ox+s2,oy+s2,oz+s2},{ox-s2,oy+s2,oz+s2},r,y,w,r);
    q({ox+s2,oy-s2,oz-s2},{ox-s2,oy-s2,oz-s2},{ox-s2,oy+s2,oz-s2},{ox+s2,oy+s2,oz-s2},b,g,g,b);
    q({ox-s2,oy-s2,oz-s2},{ox-s2,oy-s2,oz+s2},{ox-s2,oy+s2,oz+s2},{ox-s2,oy+s2,oz-s2},c,w,w,c);
    q({ox+s2,oy-s2,oz+s2},{ox+s2,oy-s2,oz-s2},{ox+s2,oy+s2,oz-s2},{ox+s2,oy+s2,oz+s2},m,y,r,m);
    q({ox-s2,oy+s2,oz+s2},{ox+s2,oy+s2,oz+s2},{ox+s2,oy+s2,oz-s2},{ox-s2,oy+s2,oz-s2},g,r,y,g);
    q({ox-s2,oy-s2,oz-s2},{ox+s2,oy-s2,oz-s2},{ox+s2,oy-s2,oz+s2},{ox-s2,oy-s2,oz+s2},dg,dg,dg,dg);
    // Cube 3 left + front
    float s3=0.35f, ox3=-1.3f, oy3=-0.1f, oz3=0.6f;
    q({ox3-s3,oy3-s3,oz3+s3},{ox3+s3,oy3-s3,oz3+s3},{ox3+s3,oy3+s3,oz3+s3},{ox3-s3,oy3+s3,oz3+s3},r,g,w,r);
    q({ox3+s3,oy3-s3,oz3-s3},{ox3-s3,oy3-s3,oz3-s3},{ox3-s3,oy3+s3,oz3-s3},{ox3+s3,oy3+s3,oz3-s3},b,y,g,b);
    q({ox3-s3,oy3-s3,oz3-s3},{ox3-s3,oy3-s3,oz3+s3},{ox3-s3,oy3+s3,oz3+s3},{ox3-s3,oy3+s3,oz3-s3},m,c,w,m);
    q({ox3+s3,oy3-s3,oz3+s3},{ox3+s3,oy3-s3,oz3-s3},{ox3+s3,oy3+s3,oz3-s3},{ox3+s3,oy3+s3,oz3+s3},y,r,m,y);
    q({ox3-s3,oy3+s3,oz3+s3},{ox3+s3,oy3+s3,oz3+s3},{ox3+s3,oy3+s3,oz3-s3},{ox3-s3,oy3+s3,oz3-s3},g,w,r,g);
    q({ox3-s3,oy3-s3,oz3-s3},{ox3+s3,oy3-s3,oz3-s3},{ox3+s3,oy3-s3,oz3+s3},{ox3-s3,oy3-s3,oz3+s3},dg,dg,dg,dg);
    // Ground
    float gs=4.5f, gy=-0.65f;
    q({-gs,gy, gs},{ gs,gy, gs},{ gs,gy,-gs},{-gs,gy,-gs},dg,dg,dg,dg);
    
    // Rendering
    std::vector<uint8_t> imgCorrect(W*H*3,0),imgLinear(W*H*3,0);
    std::vector<float> zbL(W*H,1e10f),zbR(W*H,1e10f);
    
    int triIn=0, triDrawn=0, triClipped=0;
    
    for(const auto& t : scene){
        triIn++;
        ClipVertex cv0=transformVertex(t.v0,t.c0,mvp), cv1=transformVertex(t.v1,t.c1,mvp), cv2=transformVertex(t.v2,t.c2,mvp);
        auto clipped=clipTriangle(cv0,cv1,cv2);
        if(clipped.size()<3) continue;
        triClipped++;
        for(size_t i=1;i+1<clipped.size();i++){
            ScreenVertex sv0=toScreen(clipped[0],W,H), sv1=toScreen(clipped[i],W,H), sv2=toScreen(clipped[i+1],W,H);
            float e=(sv1.x-sv0.x)*(sv2.y-sv0.y)-(sv2.x-sv0.x)*(sv1.y-sv0.y);
            if(e<=0) continue;
            triDrawn++;
            rasterizeTriangle(sv0,sv1,sv2,zbL,imgCorrect,W,H,0);
            rasterizeTriangle(sv0,sv1,sv2,zbR,imgLinear,W,H,1);
        }
    }
    std::cout<<"Pipeline stats: "<<triIn<<" in | "<<triClipped<<" survived clip | "<<triDrawn<<" rasterized\n";
    
    // Save perspective-correct
    savePPM("pipeline_persp_correct.ppm",imgCorrect,W,H);
    // Save linear
    savePPM("pipeline_linear.ppm",imgLinear,W,H);
    
    // Side-by-side comparison
    std::vector<uint8_t> cmp(W*H*3,0);
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        int i=y*W+x, i3=i*3;
        if(x<W/2){ cmp[i3]=imgCorrect[i3]; cmp[i3+1]=imgCorrect[i3+1]; cmp[i3+2]=imgCorrect[i3+2]; }
        else{ cmp[i3]=imgLinear[i3]; cmp[i3+1]=imgLinear[i3+1]; cmp[i3+2]=imgLinear[i3+2]; }
    }
    savePPM("pipeline_comparison.ppm",cmp,W,H);
    
    // Draw dividing line
    for(int y=0;y<H;y++){ int i=(y*W+W/2)*3; cmp[i]=cmp[i+1]=cmp[i+2]=255; }
    savePPM("pipeline_comparison_divider.ppm",cmp,W,H);
    
    // Compute difference map
    std::vector<uint8_t> diff(W*H*3,0);
    float sumDiff=0, maxDiff=0, sumPix=0, totalDiffPixels=0;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++){
        int i3=(y*W+x)*3;
        float dr=(int)imgCorrect[i3+0]-(int)imgLinear[i3+0];
        float dg=(int)imgCorrect[i3+1]-(int)imgLinear[i3+1];
        float db=(int)imgCorrect[i3+2]-(int)imgLinear[i3+2];
        float d=sqrt(dr*dr+dg*dg+db*db);
        sumDiff+=d; if(d>maxDiff) maxDiff=d;
        if(d>0.5f) totalDiffPixels++;
        int v=std::min(255,(int)(d*10));
        diff[i3]=diff[i3+1]=diff[i3+2]=v;
        sumPix++;
    }
    float meanDiff=sumDiff/sumPix;
    float diffPct=totalDiffPixels*100/sumPix;
    std::cout<<"\n=== Quantitative Verification ===\n";
    std::cout<<"Mean color difference (per-pixel): "<<meanDiff<<" (0-255 scale)\n";
    std::cout<<"Max color difference: "<<maxDiff<<"\n";
    std::cout<<"Pixels with noticeable difference (>0.5): "<<totalDiffPixels<<" ("<<diffPct<<"%)\n";
    savePPM("pipeline_diff_map.ppm",diff,W,H);
    
    // Verification assertions
    bool pass=true;
    if(maxDiff<1.0f){ std::cout<<"❌ FAIL: Difference too small - both images identical (bug?)\n"; pass=false; }
    if(meanDiff<0.2f){ std::cout<<"❌ FAIL: Mean difference too small\n"; pass=false; }
    if(diffPct<5.0f){ std::cout<<"❌ FAIL: Too few different pixels\n"; pass=false; }
    if(pass) std::cout<<"✅ PASS: Pipeline produces visibly different results between persp-correct and linear\n";
    
    // Pixel stats for both images
    auto pxStats=[&](const std::vector<uint8_t>& img, const char* label){
        double sum=0,sq=0; int nz=0;
        for(size_t i=0;i<img.size();i++){ sum+=img[i]; sq+=(double)img[i]*img[i]; if(img[i]==0) nz++; }
        int total=img.size();
        double mean=sum/total, stdv=sqrt(sq/total-mean*mean);
        double zeroPct=100.0*nz/total;
        std::cout<<"  "<<label<<" mean="<<mean<<" std="<<stdv<<" zero%="<<zeroPct<<"%\n";
        return mean>5&&mean<250&&stdv>5;
    };
    bool p1=pxStats(imgCorrect,"Persp-Correct"), p2=pxStats(imgLinear,"Linear       ");
    
    bool allPass=pass&&p1&&p2&&(maxDiff>=1.0f);
    std::cout<<"\nVERDICT: "<<(allPass?"✅ ALL VERIFICATIONS PASSED":"❌ SOME VERIFICATIONS FAILED")<<"\n";
    
    return allPass?0:1;
}
