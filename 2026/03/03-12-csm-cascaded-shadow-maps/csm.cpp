/*
 * Cascaded Shadow Maps (CSM) - CPU 软渲染器 v2
 *
 * 功能特性:
 *   - 4级联CSM (Cascade0最近~Cascade3最远)
 *   - 正交投影光照空间渲染
 *   - 3x3 PCF 软化采样
 *   - 场景: 大地面 + 多球体 + 山丘锥体
 *   - 输出: 主渲染 + 4个ShadowMap可视化 + 级联区域着色
 *
 * 编译: g++ -std=c++17 -O2 -o csm csm.cpp
 * 运行: ./csm
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <cstring>
#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#include <cstdint>

// ============================================================
// 数学
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& b) const{return{x+b.x,y+b.y,z+b.z};}
    Vec3 operator-(const Vec3& b) const{return{x-b.x,y-b.y,z-b.z};}
    Vec3 operator*(float t) const{return{x*t,y*t,z*t};}
    Vec3 operator/(float t) const{return{x/t,y/t,z/t};}
    Vec3 operator*(const Vec3& b) const{return{x*b.x,y*b.y,z*b.z};}
    float dot(const Vec3& b) const{return x*b.x+y*b.y+z*b.z;}
    Vec3 cross(const Vec3& b) const{return{y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x};}
    float length() const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 normalize() const{float l=length();return l<1e-9f?Vec3(0,0,0):*this*(1/l);}
    Vec3& operator+=(const Vec3& b){x+=b.x;y+=b.y;z+=b.z;return*this;}
};

// Row-major 4x4 matrix
struct Mat4 {
    float m[16]={};  // row-major: m[r*4+c]
    
    static Mat4 identity(){
        Mat4 r; r.m[0]=r.m[5]=r.m[10]=r.m[15]=1; return r;
    }
    
    // 矩阵乘法
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                for(int k=0;k<4;k++)
                    r.m[i*4+j] += m[i*4+k]*b.m[k*4+j];
        return r;
    }
    
    // 变换点 (w=1)
    Vec3 transformPoint(const Vec3& p) const {
        float x=m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3];
        float y=m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7];
        float z=m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11];
        float w=m[12]*p.x+m[13]*p.y+m[14]*p.z+m[15];
        return w!=0?Vec3(x/w,y/w,z/w):Vec3(x,y,z);
    }
    
    // 变换方向 (w=0)
    Vec3 transformDir(const Vec3& d) const {
        return Vec3(
            m[0]*d.x+m[1]*d.y+m[2]*d.z,
            m[4]*d.x+m[5]*d.y+m[6]*d.z,
            m[8]*d.x+m[9]*d.y+m[10]*d.z
        ).normalize();
    }
    
    // 返回 (clip.x, clip.y, clip.z, clip.w)
    void transform4(const Vec3& p, float& cx, float& cy, float& cz, float& cw) const {
        cx=m[0]*p.x+m[1]*p.y+m[2]*p.z+m[3];
        cy=m[4]*p.x+m[5]*p.y+m[6]*p.z+m[7];
        cz=m[8]*p.x+m[9]*p.y+m[10]*p.z+m[11];
        cw=m[12]*p.x+m[13]*p.y+m[14]*p.z+m[15];
    }
};

// LookAt (相机视图矩阵)
Mat4 lookAt(Vec3 eye, Vec3 at, Vec3 up) {
    Vec3 z = (eye-at).normalize();
    Vec3 x = up.cross(z).normalize();
    Vec3 y = z.cross(x);
    Mat4 r = Mat4::identity();
    r.m[0]=x.x; r.m[1]=x.y; r.m[2]=x.z; r.m[3]=-x.dot(eye);
    r.m[4]=y.x; r.m[5]=y.y; r.m[6]=y.z; r.m[7]=-y.dot(eye);
    r.m[8]=z.x; r.m[9]=z.y; r.m[10]=z.z; r.m[11]=-z.dot(eye);
    r.m[15]=1;
    return r;
}

// 透视投影矩阵 (右手坐标系, NDC z=-1到1)
Mat4 perspective(float fovY_deg, float aspect, float zn, float zf) {
    float fovY = fovY_deg * (float)M_PI / 180.0f;
    float f = 1.0f / std::tan(fovY/2);
    Mat4 r;
    r.m[0] = f/aspect;
    r.m[5] = f;
    r.m[10] = -(zf+zn)/(zf-zn);
    r.m[11] = -2*zf*zn/(zf-zn);
    r.m[14] = -1;
    return r;
}

// 正交投影矩阵 (右手坐标系)
Mat4 orthoProj(float l, float r2, float b, float t, float n, float f) {
    Mat4 res;
    res.m[0]  = 2/(r2-l);
    res.m[3]  = -(r2+l)/(r2-l);
    res.m[5]  = 2/(t-b);
    res.m[7]  = -(t+b)/(t-b);
    res.m[10] = -2/(f-n);
    res.m[11] = -(f+n)/(f-n);
    res.m[15] = 1;
    return res;
}

// 逆矩阵 (高斯消元)
Mat4 inverse(const Mat4& src) {
    float A[4][8];
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++) A[i][j]=src.m[i*4+j];
        for(int j=0;j<4;j++) A[i][4+j]=(i==j)?1.0f:0.0f;
    }
    for(int col=0;col<4;col++){
        int pivot=-1; float maxV=1e-12f;
        for(int row=col;row<4;row++) if(std::abs(A[row][col])>maxV){maxV=std::abs(A[row][col]);pivot=row;}
        if(pivot<0) continue;
        std::swap(A[col],A[pivot]);
        float inv=1/A[col][col];
        for(int j=0;j<8;j++) A[col][j]*=inv;
        for(int row=0;row<4;row++){
            if(row==col) continue;
            float f2=A[row][col];
            for(int j=0;j<8;j++) A[row][j]-=f2*A[col][j];
        }
    }
    Mat4 res;
    for(int i=0;i<4;i++) for(int j=0;j<4;j++) res.m[i*4+j]=A[i][4+j];
    return res;
}

// ============================================================
// 图像与深度缓冲
// ============================================================
struct Image {
    int W, H;
    std::vector<uint8_t> data; // RGB
    
    Image(int w, int h, Vec3 bg={0,0,0}):W(w),H(h),data(w*h*3){
        fill(bg);
    }
    void fill(Vec3 c){
        uint8_t r=(uint8_t)(c.x*255),g=(uint8_t)(c.y*255),b=(uint8_t)(c.z*255);
        for(int i=0;i<W*H;i++){data[i*3]=r;data[i*3+1]=g;data[i*3+2]=b;}
    }
    void setPixel(int x, int y, Vec3 c){
        if(x<0||x>=W||y<0||y>=H) return;
        int i=(y*W+x)*3;
        data[i]  =(uint8_t)std::clamp((int)(c.x*255),0,255);
        data[i+1]=(uint8_t)std::clamp((int)(c.y*255),0,255);
        data[i+2]=(uint8_t)std::clamp((int)(c.z*255),0,255);
    }
    Vec3 getPixel(int x, int y) const {
        x=std::clamp(x,0,W-1); y=std::clamp(y,0,H-1);
        int i=(y*W+x)*3;
        return {data[i]/255.f,data[i+1]/255.f,data[i+2]/255.f};
    }
};

struct DepthBuf {
    int W, H;
    std::vector<float> d;
    DepthBuf(int w,int h):W(w),H(h),d(w*h, 1e9f){}
    void clear(){std::fill(d.begin(),d.end(),1e9f);}
    bool test(int x,int y,float v){
        if(x<0||x>=W||y<0||y>=H) return false;
        float& cur=d[y*W+x];
        if(v<cur){cur=v;return true;}
        return false;
    }
};

// Shadow Map (深度纹理, 值域[0,1])
struct ShadowMap {
    int S;
    std::vector<float> d;
    ShadowMap(int s):S(s),d(s*s,1.0f){}
    void clear(){std::fill(d.begin(),d.end(),1.0f);}
    
    void set(int x, int y, float v){
        if(x<0||x>=S||y<0||y>=S) return;
        float& cur=d[y*S+x];
        if(v<cur) cur=v;
    }
    
    float sample(float u, float v) const {
        int x=(int)(u*(S-1)+0.5f), y=(int)(v*(S-1)+0.5f);
        x=std::clamp(x,0,S-1); y=std::clamp(y,0,S-1);
        return d[y*S+x];
    }
};

// PPM 文件写入
bool savePPM(const std::string& path, const Image& img) {
    FILE* f=fopen(path.c_str(),"wb");
    if(!f) return false;
    fprintf(f,"P6\n%d %d\n255\n",img.W,img.H);
    fwrite(img.data.data(),1,img.data.size(),f);
    fclose(f);
    printf("Saved: %s (%dx%d)\n",path.c_str(),img.W,img.H);
    return true;
}

// ============================================================
// 场景几何
// ============================================================
struct Tri {
    Vec3 v[3]; // 世界坐标
    Vec3 n[3]; // 顶点法线
    Vec3 color;
};

void addQuad(std::vector<Tri>& out, Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3, Vec3 col) {
    Vec3 n = (p1-p0).cross(p3-p0).normalize();
    out.push_back({{p0,p1,p2},{n,n,n},col});
    out.push_back({{p0,p2,p3},{n,n,n},col});
}

void addPlane(std::vector<Tri>& out,
              Vec3 origin, Vec3 right, Vec3 fwd,
              float W, float D, int nW, int nD, Vec3 col) {
    float dw=W/nW, dd=D/nD;
    for(int i=0;i<nD;i++) for(int j=0;j<nW;j++){
        Vec3 p00=origin+right*(j*dw)+fwd*(i*dd);
        Vec3 p10=origin+right*((j+1)*dw)+fwd*(i*dd);
        Vec3 p11=origin+right*((j+1)*dw)+fwd*((i+1)*dd);
        Vec3 p01=origin+right*(j*dw)+fwd*((i+1)*dd);
        addQuad(out,p00,p10,p11,p01,col);
    }
}

void addSphere(std::vector<Tri>& out, Vec3 center, float radius, int stacks, int slices, Vec3 col) {
    auto pt=[&](float phi, float theta)->Vec3{
        float sp=sinf(phi),cp=cosf(phi),st=sinf(theta),ct=cosf(theta);
        return{center.x+radius*sp*ct,center.y+radius*cp,center.z+radius*sp*st};
    };
    auto nm=[&](float phi, float theta)->Vec3{
        float sp=sinf(phi),cp=cosf(phi),st=sinf(theta),ct=cosf(theta);
        return Vec3{sp*ct,cp,sp*st}.normalize();
    };
    float PI=(float)M_PI;
    for(int i=0;i<stacks;i++) for(int j=0;j<slices;j++){
        float p0=PI*i/stacks, p1=PI*(i+1)/stacks;
        float t0=2*PI*j/slices, t1=2*PI*(j+1)/slices;
        Vec3 v00=pt(p0,t0),v10=pt(p0,t1),v01=pt(p1,t0),v11=pt(p1,t1);
        Vec3 n00=nm(p0,t0),n10=nm(p0,t1),n01=nm(p1,t0),n11=nm(p1,t1);
        if(i>0) out.push_back({{v00,v10,v11},{n00,n10,n11},col});
        if(i<stacks-1) out.push_back({{v00,v11,v01},{n00,n11,n01},col});
    }
}

void addCone(std::vector<Tri>& out, Vec3 base, float radius, float height, int slices, Vec3 col) {
    Vec3 apex=base+Vec3(0,height,0);
    float PI=(float)M_PI;
    for(int j=0;j<slices;j++){
        float t0=2*PI*j/slices, t1=2*PI*(j+1)/slices;
        Vec3 b0={base.x+radius*cosf(t0),base.y,base.z+radius*sinf(t0)};
        Vec3 b1={base.x+radius*cosf(t1),base.y,base.z+radius*sinf(t1)};
        Vec3 sn=((b1-b0).cross(apex-b0)).normalize();
        Vec3 bn={0,-1,0};
        out.push_back({{b0,b1,apex},{sn,sn,sn},col});
        out.push_back({{base,b1,b0},{bn,bn,bn},col});
    }
}

std::vector<Tri> buildScene() {
    std::vector<Tri> s;
    // 地面 80x80
    addPlane(s,{-40,0,-80},{1,0,0},{0,0,1},80,80,20,20,{0.38f,0.58f,0.28f});
    
    // 近景球 (z=-4..-9)
    addSphere(s,{ 0,1.5f,-5},  1.5f, 24,36, {0.85f,0.25f,0.2f});
    addSphere(s,{ 3.5f,1,-7},  1.0f, 20,28, {0.2f,0.5f,0.85f});
    addSphere(s,{-3,0.8f,-6},  0.8f, 18,24, {0.9f,0.78f,0.15f});
    
    // 中景球 (z=-15..-25)
    addSphere(s,{ 5,2,-18},    2.0f, 24,36, {0.7f,0.2f,0.8f});
    addSphere(s,{-6,1.5f,-22}, 1.5f, 20,28, {0.2f,0.75f,0.55f});
    addSphere(s,{ 0,2.5f,-20}, 2.5f, 24,36, {0.9f,0.5f,0.1f});
    
    // 远景球 (z=-38..-45)
    addSphere(s,{-8,3,-40},    3.0f, 24,36, {0.5f,0.15f,0.7f});
    addSphere(s,{ 10,2,-43},   2.0f, 20,28, {0.15f,0.65f,0.45f});
    
    // 极远景球 (z=-60..-70)
    addSphere(s,{-4,2.5f,-63}, 2.5f, 24,36, {0.6f,0.4f,0.2f});
    addSphere(s,{ 7,2,-67},    2.0f, 20,28, {0.3f,0.25f,0.8f});
    
    // 山丘
    addCone(s,{-15,0,-15},  5,4.5f,  14,{0.52f,0.42f,0.32f});
    addCone(s,{ 18,0,-28},  7,7.0f,  16,{0.48f,0.40f,0.30f});
    addCone(s,{-22,0,-48},  9,8.5f,  18,{0.50f,0.42f,0.31f});
    addCone(s,{ 12,0,-58},  6,6.0f,  14,{0.50f,0.40f,0.30f});
    
    return s;
}

// ============================================================
// CSM
// ============================================================
const int NUM_CASCADES = 4;
const int SM_SIZE = 512;

struct Cascade {
    float nearZ, farZ;
    Mat4 lightView;   // 光源看向场景
    Mat4 lightProj;   // 正交投影
    Mat4 lightVP;     // 组合矩阵
    ShadowMap sm;
    Vec3 debugColor;
    Cascade() : sm(SM_SIZE) {}
};

// CSM 分割平面 (对数线性混合)
std::array<float,NUM_CASCADES+1> cascadeSplits(float n, float f, float lambda=0.75f) {
    std::array<float,NUM_CASCADES+1> splits;
    splits[0]=n; splits[NUM_CASCADES]=f;
    float ratio=f/n;
    for(int i=1;i<NUM_CASCADES;i++){
        float p=(float)i/NUM_CASCADES;
        float logS=n*powf(ratio,p);
        float uniS=n+(f-n)*p;
        splits[i]=lambda*logS+(1-lambda)*uniS;
    }
    return splits;
}

// 计算视锥体8个角点 (世界空间)
// near_frac, far_frac: 0~1 相对于整个 [camNear, camFar]
std::array<Vec3,8> frustumCorners(const Mat4& invVP) {
    std::array<Vec3,8> c;
    float xs[]={-1,1,-1,1,-1,1,-1,1};
    float ys[]={-1,-1,1,1,-1,-1,1,1};
    float zs[]={-1,-1,-1,-1,1,1,1,1};
    for(int i=0;i<8;i++){
        float cx,cy,cz,cw;
        // invVP * (x,y,z,1)
        Vec3 ndc{xs[i],ys[i],zs[i]};
        // 手动做 invVP 变换（包含 w）
        // 用 Mat4::transform4
        cx=invVP.m[0]*xs[i]+invVP.m[1]*ys[i]+invVP.m[2]*zs[i]+invVP.m[3];
        cy=invVP.m[4]*xs[i]+invVP.m[5]*ys[i]+invVP.m[6]*zs[i]+invVP.m[7];
        cz=invVP.m[8]*xs[i]+invVP.m[9]*ys[i]+invVP.m[10]*zs[i]+invVP.m[11];
        cw=invVP.m[12]*xs[i]+invVP.m[13]*ys[i]+invVP.m[14]*zs[i]+invVP.m[15];
        c[i]={cx/cw,cy/cw,cz/cw};
    }
    return c;
}

// 给一组视锥体角点(已经对应近/远分割)构建光照矩阵
void buildLightMatrix(Cascade& casc, std::array<Vec3,8> corners, Vec3 lightDir) {
    // 视锥体中心
    Vec3 center={0,0,0};
    for(auto& c:corners) center+=c;
    center=center*(1.0f/8);
    
    // 光源"眼睛"位置
    Vec3 lightPos = center + lightDir * 200.0f;
    Vec3 up = std::abs(lightDir.y) < 0.99f ? Vec3(0,1,0) : Vec3(1,0,0);
    casc.lightView = lookAt(lightPos, center, up);
    
    // 计算视锥体在光照空间的 AABB
    float minX=1e9f,maxX=-1e9f,minY=1e9f,maxY=-1e9f;
    for(auto& c:corners){
        Vec3 lp = casc.lightView.transformPoint(c);
        minX=std::min(minX,lp.x); maxX=std::max(maxX,lp.x);
        minY=std::min(minY,lp.y); maxY=std::max(maxY,lp.y);
    }
    
    // 紧凑的XY边界，只覆盖该级联的可见区域
    // 略微扩展防止边界裁剪
    float margin = (maxX-minX)*0.05f + 1.0f;
    minX -= margin; maxX += margin;
    minY -= margin; maxY += margin;
    
    // Z范围：从场景 AABB 的光照空间 Z 计算，并向后延伸捕获投射体
    float minZ = 1e9f, maxZ = -1e9f;
    for(auto& c : corners) {
        Vec3 lp = casc.lightView.transformPoint(c);
        minZ = std::min(minZ, lp.z);
        maxZ = std::max(maxZ, lp.z);
    }
    // 光照视图是右手系，物体 Z 为负；正交投影 near/far 对应 -maxZ/-minZ
    // 向后再延伸 80 单位，捕获视锥体外的阴影投射体
    float orthoNear = -maxZ - 80.0f;
    float orthoFar  = -minZ + 10.0f;
    if(orthoNear < 0.1f) orthoNear = 0.1f;
    
    casc.lightProj = orthoProj(minX, maxX, minY, maxY, orthoNear, orthoFar);
    casc.lightVP = casc.lightProj * casc.lightView;
}

// ============================================================
// 光栅化
// ============================================================

// 将三角形光栅化到Shadow Map
void rasterizeShadowTri(const Tri& tri, Cascade& casc) {
    auto& sm = casc.sm;
    int S = sm.S;
    
    // 变换到光照裁剪空间
    float cx[3],cy[3],cz[3],cw[3];
    for(int i=0;i<3;i++) casc.lightVP.transform4(tri.v[i],cx[i],cy[i],cz[i],cw[i]);
    
    // 裁剪检查 (简单拒绝)
    for(int i=0;i<3;i++){
        if(cw[i]<0.001f) return; // 在相机后面
    }
    
    // NDC 坐标 (正交投影 w=1)
    float nx[3],ny[3],nz[3];
    for(int i=0;i<3;i++){
        float iw=1/cw[i];
        nx[i]=cx[i]*iw; ny[i]=cy[i]*iw; nz[i]=cz[i]*iw;
        // 不做深度裁剪，允许跨越近/远面的三角形
    }
    
    // 粗剪：如果 XY 全都在可见范围外则跳过
    if(std::min({nx[0],nx[1],nx[2]}) > 1.1f) return;
    if(std::max({nx[0],nx[1],nx[2]}) <-1.1f) return;
    if(std::min({ny[0],ny[1],ny[2]}) > 1.1f) return;
    if(std::max({ny[0],ny[1],ny[2]}) <-1.1f) return;
    
    // 转到 Shadow Map 像素坐标 [0, S-1]
    // NDC x,y 在 [-1,1], 映射到 [0,S]
    // NOTE: 不翻转Y (Shadow Map Y轴向上)
    float sx[3], sy[3], sd[3];
    for(int i=0;i<3;i++){
        sx[i] = (nx[i]*0.5f+0.5f)*S;
        sy[i] = (ny[i]*0.5f+0.5f)*S;
        sd[i] = nz[i]*0.5f+0.5f; // [0,1]
    }
    
    // 包围盒
    int x0=(int)std::max(0.0f,std::min({sx[0],sx[1],sx[2]}));
    int x1=(int)std::min((float)(S-1),std::max({sx[0],sx[1],sx[2]}));
    int y0=(int)std::max(0.0f,std::min({sy[0],sy[1],sy[2]}));
    int y1=(int)std::min((float)(S-1),std::max({sy[0],sy[1],sy[2]}));
    
    // 面积
    float area=(sx[1]-sx[0])*(sy[2]-sy[0])-(sx[2]-sx[0])*(sy[1]-sy[0]);
    if(std::abs(area)<0.5f) return;
    float invA=1/std::abs(area);
    float sign=(area>0)?1.f:-1.f;
    
    for(int py=y0;py<=y1;py++){
        for(int px=x0;px<=x1;px++){
            float fx=px+0.5f, fy=py+0.5f;
            float w0=((sy[1]-sy[2])*(fx-sx[2])+(sx[2]-sx[1])*(fy-sy[2]))*invA*sign;
            float w1=((sy[2]-sy[0])*(fx-sx[0])+(sx[0]-sx[2])*(fy-sy[0]))*invA*sign;
            float w2=1-w0-w1;
            if(w0<0||w1<0||w2<0) continue;
            float depth=w0*sd[0]+w1*sd[1]+w2*sd[2];
            // 深度裁剪 (NDC z 范围 [0,1])
            if(depth<0||depth>1) continue;
            sm.set(px, py, depth);
        }
    }
}

// PCF Shadow sampling (3x3)
float shadowPCF(const ShadowMap& sm, float u, float v, float compareDepth, float bias) {
    float sum=0;
    int N=3, total=N*N;
    float ts=1.0f/sm.S;
    for(int dy=-(N/2);dy<=N/2;dy++){
        for(int dx=-(N/2);dx<=N/2;dx++){
            float smD=sm.sample(u+dx*ts, v+dy*ts);
            sum += (compareDepth-bias <= smD) ? 1.0f : 0.0f;
        }
    }
    return sum/total;
}

// 主场景渲染 (含CSM阴影)
void renderMain(Image& img, DepthBuf& depthBuf,
                const std::vector<Tri>& scene,
                const Mat4& camVP,
                const Mat4& camView,
                const Vec3& camPos,
                const Vec3& lightDir,
                std::array<Cascade,NUM_CASCADES>& cascades,
                const std::array<float,NUM_CASCADES+1>& splits,
                bool cascadeVis) {
    
    int W=img.W, H=img.H;
    depthBuf.clear();
    
    // 每个级联的调试颜色
    Vec3 dbgC[4]={{1,.2f,.2f},{.2f,1,.2f},{.2f,.2f,1},{1,1,.2f}};
    
    for(auto& tri:scene){
        // 变换三个顶点到裁剪空间
        float cx[3],cy[3],cz[3],cw[3];
        for(int i=0;i<3;i++) camVP.transform4(tri.v[i],cx[i],cy[i],cz[i],cw[i]);
        
        // 基本裁剪
        bool skip=false;
        for(int i=0;i<3;i++){
            if(cw[i]<0.001f){skip=true;break;}
        }
        if(skip) continue;
        
        // NDC -> 屏幕空间
        float sx[3],sy[3],sz[3];
        for(int i=0;i<3;i++){
            float iw=1/cw[i];
            sx[i]=(cx[i]*iw*0.5f+0.5f)*W;
            sy[i]=(1-(cy[i]*iw*0.5f+0.5f))*H; // 翻转Y (屏幕Y向下)
            sz[i]=cz[i]*iw;
        }
        
        // 包围盒
        int x0=std::max(0,(int)std::min({sx[0],sx[1],sx[2]}));
        int x1=std::min(W-1,(int)std::max({sx[0],sx[1],sx[2]}));
        int y0=std::max(0,(int)std::min({sy[0],sy[1],sy[2]}));
        int y1=std::min(H-1,(int)std::max({sy[0],sy[1],sy[2]}));
        if(x0>x1||y0>y1) continue;
        
        // 面积 (屏幕空间)
        float area=(sx[1]-sx[0])*(sy[2]-sy[0])-(sx[2]-sx[0])*(sy[1]-sy[0]);
        if(std::abs(area)<0.5f) continue;
        // 统一为逆时针绕序 (面积正)，交换 v1 和 v2 使面积为正
        Vec3 tv[3] = {tri.v[0], tri.v[1], tri.v[2]};
        Vec3 tn[3] = {tri.n[0], tri.n[1], tri.n[2]};
        if(area < 0) {
            std::swap(sx[1],sx[2]); std::swap(sy[1],sy[2]); std::swap(sz[1],sz[2]);
            std::swap(tv[1],tv[2]); std::swap(tn[1],tn[2]);
            area = -area;
        }
        float invA=1/area;
        
        for(int py=y0;py<=y1;py++){
            for(int px=x0;px<=x1;px++){
                float fx=px+0.5f, fy=py+0.5f;
                float w0=((sy[1]-sy[2])*(fx-sx[2])+(sx[2]-sx[1])*(fy-sy[2]))*invA;
                float w1=((sy[2]-sy[0])*(fx-sx[0])+(sx[0]-sx[2])*(fy-sy[0]))*invA;
                float w2=1-w0-w1;
                if(w0<-0.001f||w1<-0.001f||w2<-0.001f) continue;
                w0=std::max(0.f,w0); w1=std::max(0.f,w1); w2=std::max(0.f,w2);
                
                float z=w0*sz[0]+w1*sz[1]+w2*sz[2];
                if(!depthBuf.test(px,py,z)) continue;
                
                // 插值世界空间 (使用重排后的顶点)
                Vec3 wPos = tv[0]*w0+tv[1]*w1+tv[2]*w2;
                Vec3 wNorm = (tn[0]*w0+tn[1]*w1+tn[2]*w2).normalize();
                
                // 确定所在级联 (基于视图空间Z)
                Vec3 vPos = camView.transformPoint(wPos);
                float viewZ = -vPos.z; // 右手系, 前向=-Z
                
                int cascIdx = NUM_CASCADES-1;
                for(int c=0;c<NUM_CASCADES;c++){
                    if(viewZ>=splits[c] && viewZ<splits[c+1]){cascIdx=c;break;}
                }
                auto& casc=cascades[cascIdx];
                
                // 采样 Shadow Map
                float lx,ly,lz,lw;
                casc.lightVP.transform4(wPos,lx,ly,lz,lw);
                float shadow=1.0f;
                if(lw>0.001f){
                    float u=lx/lw*0.5f+0.5f;
                    float v=ly/lw*0.5f+0.5f;
                    float d=lz/lw*0.5f+0.5f;
                    if(u>=0&&u<=1&&v>=0&&v<=1&&d>=0&&d<=1){
                        float cosA=std::abs(wNorm.dot(lightDir));
                        float bias=std::max(0.001f,0.008f*(1-cosA));
                        shadow=shadowPCF(casc.sm,u,v,d,bias);
                    }
                }
                
                // Phong 光照
                Vec3 viewDir=(camPos-wPos).normalize();
                Vec3 albedo=tri.color;
                if(cascadeVis) albedo=tri.color*0.6f+dbgC[cascIdx]*0.4f;
                
                Vec3 ambient=albedo*0.18f;
                float diffAmt=std::max(0.f,wNorm.dot(lightDir));
                Vec3 diffuse=albedo*Vec3(1,0.95f,0.9f)*diffAmt*shadow;
                Vec3 reflDir=wNorm*(2*wNorm.dot(lightDir))-lightDir;
                float specAmt=powf(std::max(0.f,viewDir.dot(reflDir)),32.f);
                Vec3 specular=Vec3(1,1,1)*specAmt*0.25f*shadow;
                
                Vec3 finalColor=ambient+diffuse+specular;
                img.setPixel(px,py,finalColor);
            }
        }
    }
}

// ============================================================
// Shadow Map 可视化
// ============================================================
Image smToImage(const ShadowMap& sm, const std::string& label) {
    // 找到有效深度范围
    float minD=1.0f, maxD=0.0f;
    int validCount=0;
    for(float d:sm.d){
        if(d<0.999f){ // 有几何体的像素
            minD=std::min(minD,d);
            maxD=std::max(maxD,d);
            validCount++;
        }
    }
    printf("  %s: %d valid pixels, depth range [%.4f, %.4f]\n",
           label.c_str(), validCount, minD, maxD);
    
    if(maxD<=minD) maxD=minD+0.01f;
    float rangeInv=1/(maxD-minD);
    
    Image img(sm.S, sm.S);
    for(int y=0;y<sm.S;y++){
        for(int x=0;x<sm.S;x++){
            float d=sm.d[y*sm.S+x];
            float v;
            if(d>=0.999f) v=0.1f; // 背景
            else v=1.0f-(d-minD)*rangeInv*0.9f; // 近=亮,远=暗
            img.setPixel(x,y,{v,v,v});
        }
    }
    return img;
}

// 拼接2x2网格
Image make2x2(const Image& a,const Image& b,const Image& c,const Image& d){
    int w=a.W, h=a.H;
    Image out(w*2+4, h*2+4, {0.1f,0.1f,0.1f});
    auto copy=[&](const Image& src, int ox, int oy){
        for(int y=0;y<src.H;y++) for(int x=0;x<src.W;x++)
            out.setPixel(ox+x,oy+y,src.getPixel(x,y));
    };
    copy(a,0,0); copy(b,w+4,0);
    copy(c,0,h+4); copy(d,w+4,h+4);
    return out;
}

// 横向拼接
Image hStack(const std::vector<Image>& imgs) {
    int totalW=0, maxH=0;
    for(auto& im:imgs){totalW+=im.W;maxH=std::max(maxH,im.H);}
    Image out(totalW,maxH,{0.08f,0.08f,0.08f});
    int ox=0;
    for(auto& im:imgs){
        for(int y=0;y<im.H;y++) for(int x=0;x<im.W;x++)
            out.setPixel(ox+x,y,im.getPixel(x,y));
        ox+=im.W;
    }
    return out;
}

// ============================================================
// main
// ============================================================
int main() {
    printf("=== Cascaded Shadow Maps (CSM) v2 ===\n");
    printf("Resolution: 800x500, Cascades: %d, SM: %d\n", NUM_CASCADES, SM_SIZE);
    
    auto scene = buildScene();
    printf("Scene: %zu triangles\n", scene.size());
    
    // 光源方向 (归一化, 斜向下照射)
    Vec3 lightDir = Vec3(0.4f, 1.0f, 0.3f).normalize();
    printf("Light dir: (%.3f, %.3f, %.3f)\n", lightDir.x, lightDir.y, lightDir.z);
    
    // 相机
    int W=800, H=500;
    Vec3 camPos={0, 6.0f, 10.0f};
    Vec3 camTarget={0, 0, -25.0f};
    Mat4 camView = lookAt(camPos, camTarget, {0,1,0});
    Mat4 camProj = perspective(55.0f, (float)W/H, 0.5f, 100.0f);
    Mat4 camVP = camProj * camView;
    
    float camNear=0.5f, camFar=100.0f;
    
    // CSM 分割
    auto splits = cascadeSplits(camNear, camFar, 0.72f);
    printf("Cascade splits:");
    for(int i=0;i<=NUM_CASCADES;i++) printf(" %.2f", splits[i]);
    printf("\n");
    
    // 初始化级联
    std::array<Cascade,NUM_CASCADES> cascades;
    Vec3 dbgColors[4]={{1,.3f,.3f},{.3f,1,.3f},{.3f,.3f,1},{1,1,.3f}};
    for(int i=0;i<NUM_CASCADES;i++){
        cascades[i].nearZ=splits[i];
        cascades[i].farZ=splits[i+1];
        cascades[i].debugColor=dbgColors[i];
    }
    
    // 计算光照矩阵
    printf("Computing light matrices...\n");
    for(int i=0;i<NUM_CASCADES;i++){
        // 提取该级联对应的视锥体
        // 计算近/远平面在NDC中的Z值
        float n=splits[i], f=splits[i+1];
        // 用子视锥体的相机矩阵
        Mat4 subProj = perspective(55.0f, (float)W/H, n, f);
        Mat4 subVP = subProj * camView;
        Mat4 invSubVP = inverse(subVP);
        
        auto corners = frustumCorners(invSubVP);
        buildLightMatrix(cascades[i], corners, lightDir);
        if(i==0){
            printf("  Cascade[0] corners world:\n");
            for(int j=0;j<8;j++)
                printf("    (%.2f,%.2f,%.2f)\n",corners[j].x,corners[j].y,corners[j].z);
            // Test near sphere
            Vec3 testPt = {0,1.5f,-5};
            float tx,ty,tz,tw;
            cascades[0].lightVP.transform4(testPt,tx,ty,tz,tw);
            printf("  Near sphere (0,1.5,-5) in light clip: (%.3f,%.3f,%.3f,%.3f)\n",tx,ty,tz,tw);
            printf("  NDC: (%.3f,%.3f,%.3f)\n",tx/tw,ty/tw,tz/tw);
        }
        printf("  Cascade[%d]: nearZ=%.2f farZ=%.2f\n",i,n,f);
    }
    
    // 渲染Shadow Maps
    printf("Rendering shadow maps...\n");
    for(int i=0;i<NUM_CASCADES;i++){
        cascades[i].sm.clear();
        for(auto& tri:scene) rasterizeShadowTri(tri, cascades[i]);
    }
    
    // 统计Shadow Map覆盖
    printf("Shadow map statistics:\n");
    for(int i=0;i<NUM_CASCADES;i++){
        int filled=0;
        for(float d:cascades[i].sm.d) if(d<0.999f) filled++;
        printf("  Cascade[%d]: %d/%d pixels filled (%.1f%%)\n",
               i, filled, SM_SIZE*SM_SIZE, 100.0f*filled/(SM_SIZE*SM_SIZE));
    }
    
    // 主渲染
    printf("Main render...\n");
    Image skyColor = Image(W, H, {0.43f, 0.65f, 0.82f});
    DepthBuf depth(W,H);
    renderMain(skyColor, depth, scene, camVP, camView, camPos, lightDir,
               cascades, splits, false);
    savePPM("csm_output.ppm", skyColor);
    
    // 级联可视化渲染
    printf("Cascade vis render...\n");
    Image cascVis(W,H,{0.43f,0.65f,0.82f});
    DepthBuf depth2(W,H);
    renderMain(cascVis, depth2, scene, camVP, camView, camPos, lightDir,
               cascades, splits, true);
    savePPM("csm_cascade_vis.ppm", cascVis);
    
    // Shadow Map 可视化
    printf("Shadow map visualization:\n");
    std::array<Image,NUM_CASCADES> smImgs = {
        smToImage(cascades[0].sm, "Cascade[0]"),
        smToImage(cascades[1].sm, "Cascade[1]"),
        smToImage(cascades[2].sm, "Cascade[2]"),
        smToImage(cascades[3].sm, "Cascade[3]")
    };
    Image smGrid = make2x2(smImgs[0], smImgs[1], smImgs[2], smImgs[3]);
    savePPM("csm_shadowmaps.ppm", smGrid);
    
    // 合并输出图 (渲染+可视化左右并排)
    Image finalOut = hStack({skyColor, cascVis});
    savePPM("csm_comparison.ppm", finalOut);
    
    printf("\n=== Done ===\n");
    printf("Output: csm_output.ppm, csm_cascade_vis.ppm, csm_shadowmaps.ppm, csm_comparison.ppm\n");
    return 0;
}
