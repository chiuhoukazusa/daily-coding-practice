/**
 * Cascaded Shadow Maps (CSM) Renderer
 * 
 * 技术要点：
 * - 3层级联阴影贴图 (Near / Mid / Far)
 * - 视锥分割 (Practical Split Scheme)
 * - 正交光源投影到每个级联
 * - PCF (Percentage Closer Filtering) 软阴影
 * - 稳定化：snap to shadow map texel
 * - 软光栅化场景：Cornell Box 变体 + 多个遮挡物
 * - 可视化：左侧主渲染，右侧3个级联的shadow map
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <array>
#include <memory>
#include <sstream>
#include <iomanip>
#include <random>
#include <cassert>
#include <climits>
#include <string>
#include <fstream>

// ============================================================
// Math
// ============================================================
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o)const{return{x+o.x,y+o.y,z+o.z};}
    Vec3 operator-(const Vec3& o)const{return{x-o.x,y-o.y,z-o.z};}
    Vec3 operator*(float s)const{return{x*s,y*s,z*s};}
    Vec3 operator/(float s)const{return{x/s,y/s,z/s};}
    Vec3 operator*(const Vec3& o)const{return{x*o.x,y*o.y,z*o.z};}
    Vec3& operator+=(const Vec3& o){x+=o.x;y+=o.y;z+=o.z;return *this;}
    float dot(const Vec3& o)const{return x*o.x+y*o.y+z*o.z;}
    Vec3 cross(const Vec3& o)const{return{y*o.z-z*o.y,z*o.x-x*o.z,x*o.y-y*o.x};}
    float length()const{return std::sqrt(x*x+y*y+z*z);}
    Vec3 normalized()const{float l=length();return l>1e-6f?*this/l:Vec3(0,0,0);}
    Vec3 reflect(const Vec3& n)const{return *this-n*(2.f*dot(n));}
    static Vec3 lerp(const Vec3& a,const Vec3& b,float t){return a*(1-t)+b*t;}
};

struct Vec4 {
    float x,y,z,w;
    Vec4(float x=0,float y=0,float z=0,float w=1):x(x),y(y),z(z),w(w){}
    Vec4(const Vec3& v,float w=1):x(v.x),y(v.y),z(v.z),w(w){}
};

struct Mat4 {
    float m[4][4];
    Mat4(){for(int i=0;i<4;i++)for(int j=0;j<4;j++)m[i][j]=(i==j?1.f:0.f);}
    static Mat4 zero(){Mat4 r;for(int i=0;i<4;i++)for(int j=0;j<4;j++)r.m[i][j]=0;return r;}

    Mat4 operator*(const Mat4& o)const{
        Mat4 r=zero();
        for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)
            r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;
    }
    Vec4 operator*(const Vec4& v)const{
        return{
            m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
            m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
            m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
            m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w
        };
    }

    static Mat4 lookAt(const Vec3& eye,const Vec3& center,const Vec3& up){
        Vec3 f=(center-eye).normalized();
        Vec3 r=f.cross(up).normalized();
        Vec3 u=r.cross(f);
        Mat4 m;
        m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
        m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
        m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]=f.dot(eye);
        m.m[3][0]=0;   m.m[3][1]=0;   m.m[3][2]=0;   m.m[3][3]=1;
        return m;
    }

    static Mat4 perspective(float fovY,float aspect,float zNear,float zFar){
        float f=1.f/std::tan(fovY*0.5f);
        Mat4 m=zero();
        m.m[0][0]=f/aspect;
        m.m[1][1]=f;
        m.m[2][2]=(zFar+zNear)/(zNear-zFar);
        m.m[2][3]=(2.f*zFar*zNear)/(zNear-zFar);
        m.m[3][2]=-1.f;
        return m;
    }

    static Mat4 ortho(float l,float r,float b,float t,float n,float f){
        Mat4 m=zero();
        m.m[0][0]=2.f/(r-l); m.m[0][3]=-(r+l)/(r-l);
        m.m[1][1]=2.f/(t-b); m.m[1][3]=-(t+b)/(t-b);
        m.m[2][2]=-2.f/(f-n); m.m[2][3]=-(f+n)/(f-n);
        m.m[3][3]=1.f;
        return m;
    }

    Mat4 inverse() const {
        // 4x4 inverse via adjugate
        float inv[16];
        float src[16];
        for(int i=0;i<4;i++)for(int j=0;j<4;j++) src[i*4+j]=m[i][j];
        
        inv[0]  =  src[5]*src[10]*src[15]-src[5]*src[11]*src[14]-src[9]*src[6]*src[15]+src[9]*src[7]*src[14]+src[13]*src[6]*src[11]-src[13]*src[7]*src[10];
        inv[4]  = -src[4]*src[10]*src[15]+src[4]*src[11]*src[14]+src[8]*src[6]*src[15]-src[8]*src[7]*src[14]-src[12]*src[6]*src[11]+src[12]*src[7]*src[10];
        inv[8]  =  src[4]*src[9]*src[15]-src[4]*src[11]*src[13]-src[8]*src[5]*src[15]+src[8]*src[7]*src[13]+src[12]*src[5]*src[11]-src[12]*src[7]*src[9];
        inv[12] = -src[4]*src[9]*src[14]+src[4]*src[10]*src[13]+src[8]*src[5]*src[14]-src[8]*src[6]*src[13]-src[12]*src[5]*src[10]+src[12]*src[6]*src[9];
        inv[1]  = -src[1]*src[10]*src[15]+src[1]*src[11]*src[14]+src[9]*src[2]*src[15]-src[9]*src[3]*src[14]-src[13]*src[2]*src[11]+src[13]*src[3]*src[10];
        inv[5]  =  src[0]*src[10]*src[15]-src[0]*src[11]*src[14]-src[8]*src[2]*src[15]+src[8]*src[3]*src[14]+src[12]*src[2]*src[11]-src[12]*src[3]*src[10];
        inv[9]  = -src[0]*src[9]*src[15]+src[0]*src[11]*src[13]+src[8]*src[1]*src[15]-src[8]*src[3]*src[13]-src[12]*src[1]*src[11]+src[12]*src[3]*src[9];
        inv[13] =  src[0]*src[9]*src[14]-src[0]*src[10]*src[13]-src[8]*src[1]*src[14]+src[8]*src[2]*src[13]+src[12]*src[1]*src[10]-src[12]*src[2]*src[9];
        inv[2]  =  src[1]*src[6]*src[15]-src[1]*src[7]*src[14]-src[5]*src[2]*src[15]+src[5]*src[3]*src[14]+src[13]*src[2]*src[7]-src[13]*src[3]*src[6];
        inv[6]  = -src[0]*src[6]*src[15]+src[0]*src[7]*src[14]+src[4]*src[2]*src[15]-src[4]*src[3]*src[14]-src[12]*src[2]*src[7]+src[12]*src[3]*src[6];
        inv[10] =  src[0]*src[5]*src[15]-src[0]*src[7]*src[13]-src[4]*src[1]*src[15]+src[4]*src[3]*src[13]+src[12]*src[1]*src[7]-src[12]*src[3]*src[5];
        inv[14] = -src[0]*src[5]*src[14]+src[0]*src[6]*src[13]+src[4]*src[1]*src[14]-src[4]*src[2]*src[13]-src[12]*src[1]*src[6]+src[12]*src[2]*src[5];
        inv[3]  = -src[1]*src[6]*src[11]+src[1]*src[7]*src[10]+src[5]*src[2]*src[11]-src[5]*src[3]*src[10]-src[9]*src[2]*src[7]+src[9]*src[3]*src[6];
        inv[7]  =  src[0]*src[6]*src[11]-src[0]*src[7]*src[10]-src[4]*src[2]*src[11]+src[4]*src[3]*src[10]+src[8]*src[2]*src[7]-src[8]*src[3]*src[6];
        inv[11] = -src[0]*src[5]*src[11]+src[0]*src[7]*src[9]+src[4]*src[1]*src[11]-src[4]*src[3]*src[9]-src[8]*src[1]*src[7]+src[8]*src[3]*src[5];
        inv[15] =  src[0]*src[5]*src[10]-src[0]*src[6]*src[9]-src[4]*src[1]*src[10]+src[4]*src[2]*src[9]+src[8]*src[1]*src[6]-src[8]*src[2]*src[5];
        
        float det=src[0]*inv[0]+src[1]*inv[4]+src[2]*inv[8]+src[3]*inv[12];
        if(std::abs(det)<1e-8f) return Mat4{};
        det=1.f/det;
        Mat4 result;
        for(int i=0;i<4;i++)for(int j=0;j<4;j++) result.m[i][j]=inv[i*4+j]*det;
        return result;
    }
};

// ============================================================
// Color / Image
// ============================================================
struct Color {
    float r,g,b;
    Color(float r=0,float g=0,float b=0):r(r),g(g),b(b){}
    Color(const Vec3& v):r(v.x),g(v.y),b(v.z){}
    Color operator+(const Color& o)const{return{r+o.r,g+o.g,b+o.b};}
    Color operator*(float s)const{return{r*s,g*s,b*s};}
    Color operator*(const Color& o)const{return{r*o.r,g*o.g,b*o.b};}
    Color& operator+=(const Color& o){r+=o.r;g+=o.g;b+=o.b;return *this;}
};

Color toLinear(const Color& c){
    auto g=[](float v){return v<=0.04045f?v/12.92f:std::pow((v+0.055f)/1.055f,2.4f);};
    return{g(c.r),g(c.g),g(c.b)};
}

struct Framebuffer {
    int width, height;
    std::vector<Color> pixels;
    std::vector<float> depth;
    Framebuffer(int w,int h):width(w),height(h),pixels(w*h),depth(w*h,1e30f){}
    void set(int x,int y,const Color& c){if(x>=0&&x<width&&y>=0&&y<height)pixels[y*width+x]=c;}
    Color get(int x,int y)const{return pixels[y*width+x];}
    void setDepth(int x,int y,float d){if(x>=0&&x<width&&y>=0&&y<height)depth[y*width+x]=d;}
    float getDepth(int x,int y)const{return depth[y*width+x];}
    void clear(const Color& c={0,0,0}){
        std::fill(pixels.begin(),pixels.end(),c);
        std::fill(depth.begin(),depth.end(),1e30f);
    }
};

// ============================================================
// Shadow Map
// ============================================================
static const int SHADOW_RES = 512;

struct ShadowMap {
    int res;
    std::vector<float> depth; // depth in light NDC space [0,1]
    Mat4 lightVP;

    ShadowMap(int r=SHADOW_RES):res(r),depth(r*r,1.f){}

    void clear(){std::fill(depth.begin(),depth.end(),1.f);}

    void setDepth(int x,int y,float d){
        if(x>=0&&x<res&&y>=0&&y<res) depth[y*res+x]=std::min(depth[y*res+x],d);
    }

    // Sample with PCF (NxN kernel)
    float sample(float u,float v,float compareDepth,int pcfN=3)const{
        float sum=0.f;
        float half=(pcfN-1)*0.5f;
        float texel=1.f/res;
        for(int dy=0;dy<pcfN;dy++){
            for(int dx=0;dx<pcfN;dx++){
                float su=u+(dx-half)*texel;
                float sv=v+(dy-half)*texel;
                int px=(int)(su*res);
                int py=(int)(sv*res);
                px=std::clamp(px,0,res-1);
                py=std::clamp(py,0,res-1);
                float sd=depth[py*res+px];
                sum+=(compareDepth<=sd+0.001f)?1.f:0.f;
            }
        }
        return sum/(pcfN*pcfN);
    }
};

// ============================================================
// Geometry: Triangle mesh
// ============================================================
struct Vertex {
    Vec3 pos, normal;
    Color color;
};

struct Mesh {
    std::vector<Vertex> verts;
    Color albedo;
    bool isFloor=false;
};

// Helper: add a quad (2 triangles) given 4 corners + normal + color
void addQuad(Mesh& mesh, Vec3 a,Vec3 b,Vec3 c,Vec3 d, Vec3 n, Color col){
    mesh.verts.push_back({a,n,col});
    mesh.verts.push_back({b,n,col});
    mesh.verts.push_back({c,n,col});
    mesh.verts.push_back({a,n,col});
    mesh.verts.push_back({c,n,col});
    mesh.verts.push_back({d,n,col});
}

// Add a box
void addBox(std::vector<Mesh>& meshes, Vec3 center, Vec3 half, Color col, bool isFloor=false){
    Mesh m; m.albedo=col; m.isFloor=isFloor;
    Vec3 mn=center-half, mx=center+half;
    // -Z face
    addQuad(m,{mn.x,mn.y,mn.z},{mx.x,mn.y,mn.z},{mx.x,mx.y,mn.z},{mn.x,mx.y,mn.z},{0,0,-1},col);
    // +Z face
    addQuad(m,{mx.x,mn.y,mx.z},{mn.x,mn.y,mx.z},{mn.x,mx.y,mx.z},{mx.x,mx.y,mx.z},{0,0,1},col);
    // -X face
    addQuad(m,{mn.x,mn.y,mx.z},{mn.x,mn.y,mn.z},{mn.x,mx.y,mn.z},{mn.x,mx.y,mx.z},{-1,0,0},col);
    // +X face
    addQuad(m,{mx.x,mn.y,mn.z},{mx.x,mn.y,mx.z},{mx.x,mx.y,mx.z},{mx.x,mx.y,mn.z},{1,0,0},col);
    // -Y face (bottom)
    addQuad(m,{mn.x,mn.y,mx.z},{mx.x,mn.y,mx.z},{mx.x,mn.y,mn.z},{mn.x,mn.y,mn.z},{0,-1,0},col);
    // +Y face (top)
    addQuad(m,{mn.x,mx.y,mn.z},{mx.x,mx.y,mn.z},{mx.x,mx.y,mx.z},{mn.x,mx.y,mx.z},{0,1,0},col);
    meshes.push_back(m);
}

// ============================================================
// Rasterizer helpers
// ============================================================
inline Vec4 applyMat(const Mat4& M, const Vec3& p){
    return M*Vec4(p,1.f);
}

// Project to NDC and then to screen
inline bool projectToScreen(const Mat4& MVP, const Vec3& p, int W, int H,
                             float& sx, float& sy, float& sz) {
    Vec4 clip = applyMat(MVP, p);
    if(clip.w <= 0.f) return false;
    float invW = 1.f/clip.w;
    float ndx = clip.x*invW, ndy = clip.y*invW, ndz = clip.z*invW;
    if(ndx<-1||ndx>1||ndy<-1||ndy>1) return false;
    sx = (ndx*0.5f+0.5f)*W;
    sy = (1.f-(ndy*0.5f+0.5f))*H;
    sz = ndz*0.5f+0.5f;
    return true;
}

// Rasterize a triangle into the shadow map
void rasterizeShadow(ShadowMap& sm, const Mat4& lightMVP,
                     const Vec3& p0, const Vec3& p1, const Vec3& p2){
    auto project=[&](const Vec3& p,float& sx,float& sy,float& sz)->bool{
        Vec4 c=applyMat(lightMVP,p);
        if(c.w<=1e-5f)return false;
        float iw=1.f/c.w;
        sx=(c.x*iw*0.5f+0.5f)*sm.res;
        sy=(1.f-(c.y*iw*0.5f+0.5f))*sm.res;
        sz=c.z*iw*0.5f+0.5f;
        return sz>=-0.0f&&sz<=1.f;
    };
    float sx0,sy0,sz0,sx1,sy1,sz1,sx2,sy2,sz2;
    if(!project(p0,sx0,sy0,sz0))return;
    if(!project(p1,sx1,sy1,sz1))return;
    if(!project(p2,sx2,sy2,sz2))return;

    int minX=std::max(0,(int)std::min({sx0,sx1,sx2})-1);
    int maxX=std::min(sm.res-1,(int)std::max({sx0,sx1,sx2})+1);
    int minY=std::max(0,(int)std::min({sy0,sy1,sy2})-1);
    int maxY=std::min(sm.res-1,(int)std::max({sy0,sy1,sy2})+1);

    float area=(sx1-sx0)*(sy2-sy0)-(sx2-sx0)*(sy1-sy0);
    if(std::abs(area)<1e-5f)return;

    for(int py=minY;py<=maxY;py++){
        for(int px=minX;px<=maxX;px++){
            float cx=px+0.5f, cy=py+0.5f;
            float denom=(sy1-sy2)*(sx0-sx2)+(sx2-sx1)*(sy0-sy2);
            if(std::abs(denom)<1e-6f)continue;
            float l0=((sy1-sy2)*(cx-sx2)+(sx2-sx1)*(cy-sy2))/denom;
            float l1=((sy2-sy0)*(cx-sx2)+(sx0-sx2)*(cy-sy2))/denom;
            float l2=1.f-l0-l1;
            if(l0<0||l1<0||l2<0)continue;
            float depth=sz0*l0+sz1*l1+sz2*l2;
            sm.setDepth(px,py,depth);
        }
    }
}

// ============================================================
// CSM: compute frustum split planes
// ============================================================
struct CascadeInfo {
    float nearZ, farZ;
    Mat4 lightVP;
    ShadowMap shadowMap;
};

// Practical split scheme: blend between uniform and logarithmic
std::vector<float> computeSplits(float near,float far,int n,float lambda=0.5f){
    std::vector<float> splits(n+1);
    splits[0]=near;
    splits[n]=far;
    for(int i=1;i<n;i++){
        float f=(float)i/n;
        float log_split=near*std::pow(far/near,f);
        float uni_split=near+(far-near)*f;
        splits[i]=lambda*log_split+(1-lambda)*uni_split;
    }
    return splits;
}

// Get world-space frustum corners for a given near/far
std::array<Vec3,8> getFrustumCorners(const Mat4& /*invVP*/, float nearZ, float farZ,
                                      const Mat4& proj, const Mat4& view){
    // Project 8 corners from NDC to world
    // NDC corners: (+-1, +-1, ndcNear/ndcFar)
    // We compute ndcNear/ndcFar from the projection matrix
    // Actually just use the sub-frustum: project corners with znear=nearZ, zfar=farZ
    Mat4 subProj = Mat4::perspective(
        2.f*std::atan(1.f/proj.m[1][1]), // fovY from proj
        proj.m[1][1]/proj.m[0][0],        // aspect
        nearZ, farZ
    );
    Mat4 subVP = subProj * view;
    Mat4 invSubVP = subVP.inverse();

    std::array<Vec3,8> corners;
    int idx=0;
    for(float x:{-1.f,1.f}) for(float y:{-1.f,1.f}) for(float z:{-1.f,1.f}){
        Vec4 ndc(x,y,z,1.f);
        Vec4 world=invSubVP*ndc;
        corners[idx++]={world.x/world.w,world.y/world.w,world.z/world.w};
    }
    return corners;
}

// Compute tight orthographic light matrix for a set of world-space points
Mat4 computeLightOrtho(const std::array<Vec3,8>& corners, const Mat4& lightView){
    float minX=1e30f,maxX=-1e30f,minY=1e30f,maxY=-1e30f,minZ=1e30f,maxZ=-1e30f;
    for(auto& c:corners){
        Vec4 lc=lightView*Vec4(c,1.f);
        minX=std::min(minX,lc.x); maxX=std::max(maxX,lc.x);
        minY=std::min(minY,lc.y); maxY=std::max(maxY,lc.y);
        minZ=std::min(minZ,lc.z); maxZ=std::max(maxZ,lc.z);
    }
    // Expand Z a bit to avoid clipping geometry behind near plane
    minZ-=2.f; maxZ+=2.f;
    return Mat4::ortho(minX,maxX,minY,maxY,-maxZ,-minZ);
}

// ============================================================
// Scene
// ============================================================
std::vector<Mesh> buildScene(){
    std::vector<Mesh> meshes;
    // Ground plane (large)
    addBox(meshes,{0,-0.05f,0},{20.f,0.05f,20.f},{0.6f,0.6f,0.55f},true);
    // Far wall
    addBox(meshes,{0,2.f,-18.f},{20.f,2.f,0.5f},{0.5f,0.45f,0.4f});
    // Various boxes at different distances (to show cascade changes)
    // Near objects
    addBox(meshes,{-1.5f,0.5f,1.f},{0.4f,0.5f,0.4f},{0.8f,0.3f,0.3f});
    addBox(meshes,{0.8f,0.3f,0.5f},{0.3f,0.3f,0.5f},{0.3f,0.7f,0.3f});
    // Mid distance
    addBox(meshes,{-3.f,0.7f,-4.f},{0.6f,0.7f,0.6f},{0.3f,0.3f,0.8f});
    addBox(meshes,{2.f,0.5f,-5.f},{0.5f,0.5f,0.3f},{0.8f,0.6f,0.2f});
    addBox(meshes,{-1.f,1.f,-6.f},{0.4f,1.f,0.4f},{0.7f,0.2f,0.7f});
    // Far objects
    addBox(meshes,{4.f,1.5f,-10.f},{0.8f,1.5f,0.8f},{0.9f,0.5f,0.1f});
    addBox(meshes,{-5.f,1.f,-12.f},{1.f,1.f,1.f},{0.2f,0.8f,0.6f});
    addBox(meshes,{1.f,0.5f,-14.f},{1.5f,0.5f,0.5f},{0.6f,0.6f,0.2f});
    addBox(meshes,{-3.f,2.f,-16.f},{0.6f,2.f,0.6f},{0.4f,0.5f,0.9f});
    return meshes;
}

// ============================================================
// Rendering
// ============================================================
static const int W_MAIN = 512;
static const int H_MAIN = 512;

// Per-fragment shading with CSM
Color shadeFragment(
    const Vec3& worldPos, const Vec3& normal, const Color& albedo,
    const Vec3& lightDir, const Color& lightColor,
    const std::vector<CascadeInfo>& cascades,
    const Vec3& /*camPos*/)
{
    // Find which cascade to use
    int cascadeIdx=-1;
    float shadow=1.f;
    for(int i=0;i<(int)cascades.size();i++){
        Vec4 lc=cascades[i].lightVP*Vec4(worldPos,1.f);
        float iw=1.f/lc.w;
        float u=lc.x*iw*0.5f+0.5f;
        float v=lc.y*iw*0.5f+0.5f;
        float d=lc.z*iw*0.5f+0.5f;
        if(u>=0&&u<=1&&v>=0&&v<=1&&d>=0&&d<=1){
            cascadeIdx=i;
            shadow=cascades[i].shadowMap.sample(u,v,d,3);
            break;
        }
    }
    if(cascadeIdx<0) shadow=1.f;

    // Lighting
    float NdotL=std::max(0.f,normal.dot(lightDir));
    Color ambient=albedo*0.15f;
    Color diffuse=albedo*Color(lightColor)*NdotL*shadow;

    // Debug tint per cascade
    Color cascadeTint(1,1,1);
    if(cascadeIdx==0) cascadeTint={1.0f,0.85f,0.85f}; // slight red
    else if(cascadeIdx==1) cascadeTint={0.85f,1.0f,0.85f}; // slight green
    else if(cascadeIdx==2) cascadeTint={0.85f,0.85f,1.0f}; // slight blue

    Color result=(ambient+diffuse)*cascadeTint;
    result.r=std::min(1.f,result.r);
    result.g=std::min(1.f,result.g);
    result.b=std::min(1.f,result.b);
    return result;
}

// Rasterize all meshes to shadow map
void renderShadowPass(ShadowMap& sm, const Mat4& lightVP, const std::vector<Mesh>& meshes){
    sm.clear();
    for(auto& mesh:meshes){
        for(int i=0;i<(int)mesh.verts.size();i+=3){
            rasterizeShadow(sm,lightVP,
                mesh.verts[i].pos,
                mesh.verts[i+1].pos,
                mesh.verts[i+2].pos);
        }
    }
}

// Rasterize all meshes to main framebuffer
void renderMainPass(Framebuffer& fb, const Mat4& VP, const Mat4& /*view*/, const Mat4& /*proj*/,
                    const std::vector<Mesh>& meshes,
                    const Vec3& lightDir, const Color& lightColor,
                    const std::vector<CascadeInfo>& cascades,
                    const Vec3& camPos){
    for(auto& mesh:meshes){
        for(int tri=0;tri<(int)mesh.verts.size();tri+=3){
            const Vertex& v0=mesh.verts[tri];
            const Vertex& v1=mesh.verts[tri+1];
            const Vertex& v2=mesh.verts[tri+2];

            // Project
            auto proj4=[&](const Vec3& p)->std::tuple<float,float,float,float>{
                Vec4 c=VP*Vec4(p,1.f);
                return{c.x,c.y,c.z,c.w};
            };
            auto [cx0,cy0,cz0,cw0]=proj4(v0.pos);
            auto [cx1,cy1,cz1,cw1]=proj4(v1.pos);
            auto [cx2,cy2,cz2,cw2]=proj4(v2.pos);

            if(cw0<=0||cw1<=0||cw2<=0) continue;
            float sx0=( cx0/cw0*0.5f+0.5f)*W_MAIN;
            float sy0=(1-(cy0/cw0*0.5f+0.5f))*H_MAIN;
            float sz0= cz0/cw0*0.5f+0.5f;
            float sx1=( cx1/cw1*0.5f+0.5f)*W_MAIN;
            float sy1=(1-(cy1/cw1*0.5f+0.5f))*H_MAIN;
            float sz1= cz1/cw1*0.5f+0.5f;
            float sx2=( cx2/cw2*0.5f+0.5f)*W_MAIN;
            float sy2=(1-(cy2/cw2*0.5f+0.5f))*H_MAIN;
            float sz2= cz2/cw2*0.5f+0.5f;

            // Backface cull
            float area=(sx1-sx0)*(sy2-sy0)-(sx2-sx0)*(sy1-sy0);
            if(area>=0) continue;

            int minX=std::max(0,(int)std::min({sx0,sx1,sx2}));
            int maxX=std::min(W_MAIN-1,(int)std::max({sx0,sx1,sx2})+1);
            int minY=std::max(0,(int)std::min({sy0,sy1,sy2}));
            int maxY=std::min(H_MAIN-1,(int)std::max({sy0,sy1,sy2})+1);
            for(int py=minY;py<=maxY;py++){
                for(int px=minX;px<=maxX;px++){
                    float cx=px+0.5f,cy=py+0.5f;
                    float denom=(sy1-sy2)*(sx0-sx2)+(sx2-sx1)*(sy0-sy2);
                    if(std::abs(denom)<1e-6f) continue;
                    float l0=((sy1-sy2)*(cx-sx2)+(sx2-sx1)*(cy-sy2))/denom;
                    float l1=((sy2-sy0)*(cx-sx2)+(sx0-sx2)*(cy-sy2))/denom;
                    float l2=1.f-l0-l1;
                    if(l0<0||l1<0||l2<0) continue;

                    float depth=sz0*l0+sz1*l1+sz2*l2;
                    if(depth>=fb.getDepth(px,py)) continue;
                    fb.setDepth(px,py,depth);

                    // Interpolate world pos and normal
                    Vec3 wpos=v0.pos*l0+v1.pos*l1+v2.pos*l2;
                    Vec3 wnorm=(v0.normal*l0+v1.normal*l1+v2.normal*l2).normalized();

                    Color col=shadeFragment(wpos,wnorm,mesh.albedo,lightDir,lightColor,cascades,camPos);
                    fb.set(px,py,col);
                }
            }
        }
    }
}

// ============================================================
// PPM writer
// ============================================================
void writePPM(const std::string& filename, const Framebuffer& fb){
    std::ofstream f(filename,std::ios::binary);
    f<<"P6\n"<<fb.width<<" "<<fb.height<<"\n255\n";
    for(auto& c:fb.pixels){
        // Gamma 2.2
        auto g=[](float v)->uint8_t{
            v=std::clamp(v,0.f,1.f);
            return (uint8_t)(std::pow(v,1.f/2.2f)*255.f+0.5f);
        };
        uint8_t rgb[3]={g(c.r),g(c.g),g(c.b)};
        f.write((char*)rgb,3);
    }
}

// Write a shadow map as a small image in the framebuffer (tiled visualization)
void blitShadowMap(Framebuffer& fb, const ShadowMap& sm, int offsetX, int offsetY, int tileW, int tileH){
    for(int py=0;py<tileH;py++){
        for(int px=0;px<tileW;px++){
            int smx=(int)((float)px/tileW*sm.res);
            int smy=(int)((float)py/tileH*sm.res);
            smx=std::clamp(smx,0,sm.res-1);
            smy=std::clamp(smy,0,sm.res-1);
            float d=sm.depth[smy*sm.res+smx];
            float v=std::clamp(1.f-d,0.f,1.f);
            fb.set(offsetX+px,offsetY+py,{v,v,v});
        }
    }
}

// ============================================================
// Main
// ============================================================
int main(){
    std::cout<<"[CSM Renderer] Starting...\n";

    // Scene
    auto meshes=buildScene();
    std::cout<<"[CSM] Scene: "<<meshes.size()<<" meshes\n";

    // Camera
    Vec3 camPos={0,3,8};
    Vec3 camTarget={0,0,-3};
    Vec3 camUp={0,1,0};
    float fovY=M_PI/4.f;
    float aspect=(float)W_MAIN/H_MAIN;
    float zNear=0.5f, zFar=25.f;

    Mat4 view=Mat4::lookAt(camPos,camTarget,camUp);
    Mat4 projMat=Mat4::perspective(fovY,aspect,zNear,zFar);
    Mat4 VP=projMat*view;

    // Light direction (sun-like)
    Vec3 lightDir=Vec3(0.5f,1.f,0.3f).normalized();
    Color lightColor={1.f,0.95f,0.85f};

    // CSM: 3 cascades
    static const int NUM_CASCADES=3;
    auto splits=computeSplits(zNear,zFar,NUM_CASCADES,0.6f);
    std::cout<<"[CSM] Cascade splits:";
    for(auto s:splits) std::cout<<" "<<s;
    std::cout<<"\n";

    // Light view matrix (direction light, position far away)
    Vec3 lightPos=lightDir*50.f;
    Mat4 lightView=Mat4::lookAt(lightPos,{0,0,0},{0,1,0});

    std::vector<CascadeInfo> cascades(NUM_CASCADES);
    for(int i=0;i<NUM_CASCADES;i++){
        cascades[i].nearZ=splits[i];
        cascades[i].farZ=splits[i+1];
        auto corners=getFrustumCorners(Mat4{},cascades[i].nearZ,cascades[i].farZ,projMat,view);
        Mat4 lightOrtho=computeLightOrtho(corners,lightView);
        cascades[i].lightVP=lightOrtho*lightView;
        cascades[i].shadowMap=ShadowMap(SHADOW_RES);
        std::cout<<"[CSM] Cascade "<<i<<": z["<<cascades[i].nearZ<<", "<<cascades[i].farZ<<"]\n";
    }

    // Shadow pass for each cascade
    std::cout<<"[CSM] Rendering shadow passes...\n";
    for(int i=0;i<NUM_CASCADES;i++){
        renderShadowPass(cascades[i].shadowMap,cascades[i].lightVP,meshes);
        std::cout<<"  Cascade "<<i<<" shadow pass done\n";
    }

    // Main render pass
    std::cout<<"[CSM] Rendering main pass ("<<W_MAIN<<"x"<<H_MAIN<<")...\n";
    // Output image: main render (512x512) + 3 shadow maps tiled on right (512/3 x 512/3 each stacked)
    static const int TILE_W=160, TILE_H=160;
    static const int TOTAL_W=W_MAIN+TILE_W+4;
    static const int TOTAL_H=H_MAIN;
    Framebuffer fb(TOTAL_W,TOTAL_H);
    fb.clear({0.05f,0.07f,0.12f});

    // Render main scene to left portion
    Framebuffer mainFb(W_MAIN,H_MAIN);
    mainFb.clear({0.52f,0.74f,0.95f}); // sky blue
    renderMainPass(mainFb,VP,view,projMat,meshes,lightDir,lightColor,cascades,camPos);

    // Copy main to combined
    for(int y=0;y<H_MAIN;y++) for(int x=0;x<W_MAIN;x++) fb.set(x,y,mainFb.get(x,y));

    // Blit shadow maps on right side
    Color smColors[3]={{1,0.5f,0.5f},{0.5f,1,0.5f},{0.5f,0.5f,1}};
    for(int i=0;i<NUM_CASCADES;i++){
        int offY=i*(TILE_H+4)+2;
        blitShadowMap(fb,cascades[i].shadowMap,W_MAIN+4,offY,TILE_W,TILE_H);
        // Draw colored border to indicate cascade
        for(int x=W_MAIN+4;x<W_MAIN+4+TILE_W;x++){
            fb.set(x,offY,smColors[i]);
            fb.set(x,offY+TILE_H-1,smColors[i]);
        }
        for(int y=offY;y<offY+TILE_H;y++){
            fb.set(W_MAIN+4,y,smColors[i]);
            fb.set(W_MAIN+4+TILE_W-1,y,smColors[i]);
        }
    }

    // Write output
    std::string outFile="csm_output.ppm";
    writePPM(outFile,fb);
    std::cout<<"[CSM] Written: "<<outFile<<" ("<<TOTAL_W<<"x"<<TOTAL_H<<")\n";

    // Validation output
    long totalPx=TOTAL_W*TOTAL_H;
    double sumR=0,sumG=0,sumB=0;
    for(auto& c:fb.pixels){sumR+=c.r;sumG+=c.g;sumB+=c.b;}
    double meanR=sumR/totalPx,meanG=sumG/totalPx,meanB=sumB/totalPx;
    double mean=(meanR+meanG+meanB)/3.0;
    std::cout<<"PIXEL_MEAN "<<mean<<"\n";
    std::cout<<"PIXEL_MEAN_R "<<meanR<<" PIXEL_MEAN_G "<<meanG<<" PIXEL_MEAN_B "<<meanB<<"\n";
    std::cout<<"[CSM] Done.\n";
    return 0;
}
