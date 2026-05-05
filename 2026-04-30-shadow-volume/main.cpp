/*
 * Shadow Volume Renderer (Software)
 * 
 * 技术要点：
 * 1. Shadow Volume: 将光源投影阴影形成封闭几何体
 * 2. Z-Fail (Carmack's Reverse): 摄像机在阴影体内也能正确工作
 * 3. Stencil Buffer: 软件模拟模板缓冲计数
 * 4. Silhouette Edge Detection: 检测轮廓边（一个面朝光，一个面背光）
 * 5. 软光栅化渲染管线（无依赖）
 * 
 * 场景：Cornell Box变体，多个物体，点光源，产生清晰硬阴影
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <limits>
#include <functional>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================
// Math Types
// ============================

struct Vec2 { float x, y; };

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x,y-o.y,z-o.z}; }
    Vec3 operator*(float s) const { return {x*s,y*s,z*s}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x,y*o.y,z*o.z}; }
    Vec3 operator/(float s) const { return {x/s,y/s,z/s}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x;y+=o.y;z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    float len2() const { return x*x+y*y+z*z; }
    Vec3 normalize() const { float l=len(); return (l>1e-8f)?(*this/l):Vec3(0,1,0); }
    Vec3 abs() const { return {fabsf(x),fabsf(y),fabsf(z)}; }
};
inline Vec3 operator*(float s, const Vec3& v) { return v*s; }

struct Vec4 {
    float x,y,z,w;
    Vec4(float x=0,float y=0,float z=0,float w=0): x(x),y(y),z(z),w(w){}
    Vec4(const Vec3& v, float w): x(v.x),y(v.y),z(v.z),w(w){}
    Vec3 xyz() const { return {x,y,z}; }
};

struct Mat4 {
    float m[4][4];
    Mat4() { memset(m, 0, sizeof(m)); }
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z+m[0][3]*v.w,
            m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z+m[1][3]*v.w,
            m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z+m[2][3]*v.w,
            m[3][0]*v.x+m[3][1]*v.y+m[3][2]*v.z+m[3][3]*v.w
        };
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++)
            r.m[i][j]+=m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 perspective(float fovY, float aspect, float nearZ, float farZ) {
    float f = 1.0f / tanf(fovY * 0.5f);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (farZ + nearZ) / (nearZ - farZ);
    r.m[2][3] = 2.0f * farZ * nearZ / (nearZ - farZ);
    r.m[3][2] = -1.0f;
    return r;
}

Mat4 lookAt(Vec3 eye, Vec3 at, Vec3 up) {
    Vec3 z = (eye - at).normalize();
    Vec3 x = up.cross(z).normalize();
    Vec3 y = z.cross(x);
    Mat4 r = Mat4::identity();
    r.m[0][0]=x.x; r.m[0][1]=x.y; r.m[0][2]=x.z; r.m[0][3]=-x.dot(eye);
    r.m[1][0]=y.x; r.m[1][1]=y.y; r.m[1][2]=y.z; r.m[1][3]=-y.dot(eye);
    r.m[2][0]=z.x; r.m[2][1]=z.y; r.m[2][2]=z.z; r.m[2][3]=-z.dot(eye);
    r.m[3][3]=1;
    return r;
}

Mat4 translate(Vec3 t) {
    Mat4 r = Mat4::identity();
    r.m[0][3]=t.x; r.m[1][3]=t.y; r.m[2][3]=t.z;
    return r;
}

Mat4 scale(Vec3 s) {
    Mat4 r = Mat4::identity();
    r.m[0][0]=s.x; r.m[1][1]=s.y; r.m[2][2]=s.z;
    return r;
}

Mat4 rotateY(float angle) {
    Mat4 r = Mat4::identity();
    float c=cosf(angle), s=sinf(angle);
    r.m[0][0]=c; r.m[0][2]=s;
    r.m[2][0]=-s; r.m[2][2]=c;
    return r;
}

Mat4 rotateX(float angle) {
    Mat4 r = Mat4::identity();
    float c=cosf(angle), s=sinf(angle);
    r.m[1][1]=c; r.m[1][2]=-s;
    r.m[2][1]=s; r.m[2][2]=c;
    return r;
}

// ============================
// Image / Framebuffer
// ============================

const int W = 800, H = 600;

struct Color {
    uint8_t r,g,b;
    Color(uint8_t r=0, uint8_t g=0, uint8_t b=0): r(r),g(g),b(b){}
};

struct Framebuffer {
    Color color[H][W];
    float depth[H][W];
    int   stencil[H][W];  // 模板缓冲（软件模拟）
    
    void clear(Color bg = {30, 30, 40}) {
        for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
            color[y][x] = bg;
            depth[y][x] = 1.0f;
            stencil[y][x] = 0;
        }
    }
} fb;

// ============================
// Mesh
// ============================

struct Triangle {
    Vec3 v[3];
    Vec3 n;  // 面法线
};

struct Mesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<int,3>> faces;
    std::vector<Vec3> faceNormals;
    Vec3 color;
    Mat4 transform;
    
    void computeFaceNormals() {
        faceNormals.resize(faces.size());
        for(size_t i=0;i<faces.size();i++) {
            Vec3 a=vertices[faces[i][0]], b=vertices[faces[i][1]], c=vertices[faces[i][2]];
            faceNormals[i] = (b-a).cross(c-a).normalize();
        }
    }
    
    // 获取变换后的世界空间顶点
    Vec3 worldVertex(int idx) const {
        Vec4 v4(vertices[idx], 1.0f);
        Vec4 r = transform * v4;
        return r.xyz() / r.w;
    }
    
    // 获取变换后的世界空间面法线（不考虑非均匀缩放，只旋转）
    Vec3 worldFaceNormal(int fi) const {
        // 简化处理：用前三顶点重新计算世界空间法线
        Vec3 a=worldVertex(faces[fi][0]), b=worldVertex(faces[fi][1]), c=worldVertex(faces[fi][2]);
        return (b-a).cross(c-a).normalize();
    }
};

// 创建立方体
Mesh makeCube(Vec3 color, Mat4 T) {
    Mesh m;
    m.color = color;
    m.transform = T;
    // 8个顶点（归一化，通过transform缩放）
    m.vertices = {
        {-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
        {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}
    };
    // 6个面，每面2个三角形
    m.faces = {
        {0,1,2},{0,2,3}, // 后
        {4,6,5},{4,7,6}, // 前
        {0,5,1},{0,4,5}, // 下
        {2,6,7},{2,7,3}, // 上
        {0,3,7},{0,7,4}, // 左
        {1,5,6},{1,6,2}  // 右
    };
    m.computeFaceNormals();
    return m;
}

// 创建球体（approximation）
Mesh makeSphere(Vec3 color, Mat4 T, int stacks=14, int slices=18) {
    Mesh m;
    m.color = color;
    m.transform = T;
    
    // 生成顶点
    for(int i=0;i<=stacks;i++) {
        float phi = (float)M_PI * i / stacks;
        for(int j=0;j<=slices;j++) {
            float theta = 2.0f*(float)M_PI*j/slices;
            m.vertices.push_back({
                sinf(phi)*cosf(theta),
                cosf(phi),
                sinf(phi)*sinf(theta)
            });
        }
    }
    // 生成三角形
    for(int i=0;i<stacks;i++) {
        for(int j=0;j<slices;j++) {
            int a = i*(slices+1)+j;
            int b = a+1;
            int c = (i+1)*(slices+1)+j;
            int d = c+1;
            m.faces.push_back({a,c,b});
            m.faces.push_back({b,c,d});
        }
    }
    m.computeFaceNormals();
    return m;
}

// ============================
// Shadow Volume Construction
// ============================

struct ShadowVolumeMesh {
    std::vector<Vec3> verts;  // 三角形顶点（3个一组）
};

// 检测轮廓边：某条边被一个朝光面和一个背光面共享
// Z-Fail方法：生成封闭阴影体
ShadowVolumeMesh buildShadowVolume(const Mesh& mesh, const Vec3& lightPos, float extrudeDist=50.0f) {
    ShadowVolumeMesh sv;
    int nFaces = (int)mesh.faces.size();
    
    // 判断每个面朝向光源
    std::vector<bool> facesLight(nFaces);
    for(int i=0;i<nFaces;i++) {
        Vec3 a = mesh.worldVertex(mesh.faces[i][0]);
        Vec3 normal = mesh.worldFaceNormal(i);
        Vec3 toLight = (lightPos - a).normalize();
        facesLight[i] = (normal.dot(toLight) > 0.0f);
    }
    
    // 构建边-面邻接
    // key: sorted edge (v0,v1), value: list of face indices
    struct EdgeKey {
        int a, b;
        bool operator==(const EdgeKey& o) const { return a==o.a && b==o.b; }
    };
    
    // 简化：遍历所有面对，找到共享边中一面朝光一面背光（轮廓边）
    // 用map存储每条边及其面的朝光情况
    struct EdgeInfo {
        int fi1, fi2;
        bool hasSecond;
        EdgeInfo(): fi1(-1), fi2(-1), hasSecond(false) {}
    };
    
    // 用边(排序顶点对)映射
    // 为了简化，使用 O(n^2) 检测（面数不多）
    // 构建 edge -> [face1, face2] 的映射
    std::vector<std::tuple<int,int,int,int>> edgeFaceList; 
    // (edgeA, edgeB, faceIdx, isLit)
    
    // 遍历所有面的每条边
    struct HalfEdge { int va, vb, fi; };
    std::vector<HalfEdge> halfEdges;
    halfEdges.reserve(nFaces*3);
    for(int fi=0;fi<nFaces;fi++) {
        for(int k=0;k<3;k++) {
            int va = mesh.faces[fi][k];
            int vb = mesh.faces[fi][(k+1)%3];
            halfEdges.push_back({va, vb, fi});
        }
    }
    
    // 找轮廓边：找到一对半边 (va,vb) 和 (vb,va)，且其面的朝光性不同
    std::vector<bool> used(halfEdges.size(), false);
    
    struct SilhouetteEdge {
        Vec3 va, vb; // 世界空间端点
        bool litSide; // true表示 va->vb 方向是朝光面
    };
    std::vector<SilhouetteEdge> silhouettes;
    
    for(size_t i=0;i<halfEdges.size();i++) {
        if(used[i]) continue;
        int va = halfEdges[i].va;
        int vb = halfEdges[i].vb;
        int fi = halfEdges[i].fi;
        
        // 找对边
        for(size_t j=i+1;j<halfEdges.size();j++) {
            if(used[j]) continue;
            if(halfEdges[j].va == vb && halfEdges[j].vb == va) {
                // 找到对边
                int fi2 = halfEdges[j].fi;
                if(facesLight[fi] != facesLight[fi2]) {
                    // 轮廓边！
                    // 确保方向：从朝光面方向出发
                    SilhouetteEdge se;
                    if(facesLight[fi]) {
                        se.va = mesh.worldVertex(va);
                        se.vb = mesh.worldVertex(vb);
                        se.litSide = true;
                    } else {
                        se.va = mesh.worldVertex(vb);
                        se.vb = mesh.worldVertex(va);
                        se.litSide = true;
                    }
                    silhouettes.push_back(se);
                    used[i]=used[j]=true;
                }
                break;
            }
        }
    }
    
    // 处理边界边（只有一个面的边）- 对于封闭网格这是顶点
    // 如果只有一个面，根据该面是否朝光决定是否加入
    for(size_t i=0;i<halfEdges.size();i++) {
        if(used[i]) continue;
        int fi = halfEdges[i].fi;
        if(facesLight[fi]) {
            // 这是一条只有朝光面的裸边 - 加入轮廓
            SilhouetteEdge se;
            se.va = mesh.worldVertex(halfEdges[i].va);
            se.vb = mesh.worldVertex(halfEdges[i].vb);
            se.litSide = true;
            silhouettes.push_back(se);
        }
    }
    
    // 从每条轮廓边生成阴影体四边形（两个三角形）
    // Z-Fail方法需要封闭阴影体：
    //   - 侧面四边形（从轮廓边延伸到无穷远）
    //   - 前盖（朝光面原始三角形）
    //   - 后盖（远处的投影三角形）
    
    for(auto& se : silhouettes) {
        // 计算投影点（沿远离光源方向延伸）
        Vec3 dirA = (se.va - lightPos).normalize();
        Vec3 dirB = (se.vb - lightPos).normalize();
        Vec3 farA = se.va + dirA * extrudeDist;
        Vec3 farB = se.vb + dirB * extrudeDist;
        
        // 四边形（两个三角形）：侧面
        sv.verts.push_back(se.va);
        sv.verts.push_back(se.vb);
        sv.verts.push_back(farB);
        
        sv.verts.push_back(se.va);
        sv.verts.push_back(farB);
        sv.verts.push_back(farA);
    }
    
    // 前盖：朝光的面（原始位置）
    for(int fi=0;fi<nFaces;fi++) {
        if(facesLight[fi]) {
            sv.verts.push_back(mesh.worldVertex(mesh.faces[fi][0]));
            sv.verts.push_back(mesh.worldVertex(mesh.faces[fi][1]));
            sv.verts.push_back(mesh.worldVertex(mesh.faces[fi][2]));
        }
    }
    
    // 后盖：背光面投影到远处
    for(int fi=0;fi<nFaces;fi++) {
        if(!facesLight[fi]) {
            Vec3 a = mesh.worldVertex(mesh.faces[fi][0]);
            Vec3 b = mesh.worldVertex(mesh.faces[fi][1]);
            Vec3 c = mesh.worldVertex(mesh.faces[fi][2]);
            Vec3 farA = a + (a - lightPos).normalize() * extrudeDist;
            Vec3 farB = b + (b - lightPos).normalize() * extrudeDist;
            Vec3 farC = c + (c - lightPos).normalize() * extrudeDist;
            // 反转绕序（后盖需要反向法线）
            sv.verts.push_back(farA);
            sv.verts.push_back(farC);
            sv.verts.push_back(farB);
        }
    }
    
    return sv;
}

// ============================
// Rasterizer
// ============================

Vec4 transformVertex(const Vec4& v, const Mat4& mvp) {
    return mvp * v;
}

struct VSOut {
    Vec4 clip;
    Vec3 world;
    Vec3 normal;
    Vec3 color;
};

// 透视除法+视口变换
struct ScreenPos {
    float x, y, z; // z是NDC深度 [-1,1]
    float w;       // 原始w用于透视除法
};

ScreenPos toScreen(const Vec4& clip) {
    float invW = 1.0f / clip.w;
    return {
        (clip.x * invW * 0.5f + 0.5f) * W,
        (1.0f - (clip.y * invW * 0.5f + 0.5f)) * H,  // y flip
        clip.z * invW,
        clip.w
    };
}

// 像素着色 - Phong光照
Vec3 shade(Vec3 worldPos, Vec3 normal, Vec3 matColor, Vec3 lightPos, Vec3 eyePos, bool inShadow) {
    Vec3 ambient = matColor * 0.15f;
    if(inShadow) return ambient;
    
    Vec3 L = (lightPos - worldPos).normalize();
    Vec3 N = normal.normalize();
    Vec3 V = (eyePos - worldPos).normalize();
    Vec3 H = (L + V).normalize();
    
    float NdotL = std::max(0.0f, N.dot(L));
    float NdotH = std::max(0.0f, N.dot(H));
    
    // 距离衰减
    float dist = (lightPos - worldPos).len();
    float attn = 1.0f / (1.0f + 0.01f * dist + 0.001f * dist * dist);
    
    Vec3 diffuse  = matColor * (NdotL * attn);
    Vec3 specular = Vec3(1,1,1) * (powf(NdotH, 32.0f) * attn * 0.5f);
    
    return ambient + diffuse + specular;
}

// 边函数（用于光栅化）
inline float edgeFunc(float ax, float ay, float bx, float by, float px, float py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

// 光栅化三角形（颜色+深度写入）
void rasterizeTriangle(
    Vec3 p0, Vec3 p1, Vec3 p2,  // 世界空间
    Vec3 n0, Vec3 n1, Vec3 n2,  // 世界空间法线
    Vec3 matColor,
    const Mat4& mvp,
    Vec3 lightPos, Vec3 eyePos,
    std::function<bool(int,int)> isInShadow  // 查询某像素是否在阴影中
) {
    Vec4 c0 = mvp * Vec4(p0, 1), c1 = mvp * Vec4(p1, 1), c2 = mvp * Vec4(p2, 1);
    
    // 背面剔除（对于主场景）
    // 不在这里做，让调用方决定
    
    ScreenPos s0 = toScreen(c0), s1 = toScreen(c1), s2 = toScreen(c2);
    
    // AABB
    int minX = std::max(0, (int)std::min({s0.x, s1.x, s2.x}));
    int maxX = std::min(W-1, (int)std::max({s0.x, s1.x, s2.x})+1);
    int minY = std::max(0, (int)std::min({s0.y, s1.y, s2.y}));
    int maxY = std::min(H-1, (int)std::max({s0.y, s1.y, s2.y})+1);
    
    float area = edgeFunc(s0.x,s0.y, s1.x,s1.y, s2.x,s2.y);
    if(fabsf(area) < 1e-6f) return;
    
    for(int py=minY;py<=maxY;py++) {
        for(int px=minX;px<=maxX;px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            float w0 = edgeFunc(s1.x,s1.y, s2.x,s2.y, fx,fy);
            float w1 = edgeFunc(s2.x,s2.y, s0.x,s0.y, fx,fy);
            float w2 = edgeFunc(s0.x,s0.y, s1.x,s1.y, fx,fy);
            
            if((area>0 && w0>=0 && w1>=0 && w2>=0) ||
               (area<0 && w0<=0 && w1<=0 && w2<=0)) {
                float bw0=w0/area, bw1=w1/area, bw2=w2/area;
                
                // 透视正确插值
                float invW0=1.0f/c0.w, invW1=1.0f/c1.w, invW2=1.0f/c2.w;
                float wInterp = bw0*invW0 + bw1*invW1 + bw2*invW2;
                
                float zNDC = bw0*(s0.z) + bw1*(s1.z) + bw2*(s2.z);
                float depth = zNDC * 0.5f + 0.5f; // [0,1]
                
                if(depth < fb.depth[py][px]) {
                    fb.depth[py][px] = depth;
                    
                    // 插值世界空间属性
                    float b0=bw0*invW0/wInterp, b1=bw1*invW1/wInterp, b2=bw2*invW2/wInterp;
                    Vec3 wPos = p0*b0 + p1*b1 + p2*b2;
                    Vec3 wNorm = (n0*b0 + n1*b1 + n2*b2).normalize();
                    
                    bool shadow = isInShadow(px, py);
                    Vec3 finalColor = shade(wPos, wNorm, matColor, lightPos, eyePos, shadow);
                    
                    finalColor.x = std::min(1.0f, std::max(0.0f, finalColor.x));
                    finalColor.y = std::min(1.0f, std::max(0.0f, finalColor.y));
                    finalColor.z = std::min(1.0f, std::max(0.0f, finalColor.z));
                    
                    fb.color[py][px] = {
                        (uint8_t)(finalColor.x * 255),
                        (uint8_t)(finalColor.y * 255),
                        (uint8_t)(finalColor.z * 255)
                    };
                }
            }
        }
    }
}

// 光栅化阴影体（只更新模板缓冲，Z-Fail方法）
void rasterizeShadowVolumeZFail(
    Vec3 p0, Vec3 p1, Vec3 p2,
    const Mat4& mvp
) {
    Vec4 c0 = mvp * Vec4(p0, 1), c1 = mvp * Vec4(p1, 1), c2 = mvp * Vec4(p2, 1);
    
    // 裁剪空间检查（避免w<=0）
    if(c0.w <= 0 || c1.w <= 0 || c2.w <= 0) return;
    
    ScreenPos s0 = toScreen(c0), s1 = toScreen(c1), s2 = toScreen(c2);
    
    int minX = std::max(0, (int)std::min({s0.x, s1.x, s2.x}));
    int maxX = std::min(W-1, (int)std::max({s0.x, s1.x, s2.x})+1);
    int minY = std::max(0, (int)std::min({s0.y, s1.y, s2.y}));
    int maxY = std::min(H-1, (int)std::max({s0.y, s1.y, s2.y})+1);
    
    float area = edgeFunc(s0.x,s0.y, s1.x,s1.y, s2.x,s2.y);
    if(fabsf(area) < 1e-6f) return;
    
    bool frontFace = (area > 0);
    
    for(int py=minY;py<=maxY;py++) {
        for(int px=minX;px<=maxX;px++) {
            float fx = px + 0.5f, fy = py + 0.5f;
            float w0 = edgeFunc(s1.x,s1.y, s2.x,s2.y, fx,fy);
            float w1 = edgeFunc(s2.x,s2.y, s0.x,s0.y, fx,fy);
            float w2 = edgeFunc(s0.x,s0.y, s1.x,s1.y, fx,fy);
            
            bool inside = frontFace ? (w0>=0&&w1>=0&&w2>=0) : (w0<=0&&w1<=0&&w2<=0);
            if(!inside) continue;
            
            // Z-Fail: 深度测试失败时更新模板
            // 计算此三角形的深度
            float bw0=w0/area, bw1=w1/area, bw2=w2/area;
            float zNDC = bw0*s0.z + bw1*s1.z + bw2*s2.z;
            float depth = zNDC * 0.5f + 0.5f;
            
            // Z-Fail规则：如果深度测试失败（三角形在场景几何后面）
            if(depth >= fb.depth[py][px]) {
                // Z-Fail:
                //   - 正面 -> stencil--
                //   - 背面 -> stencil++
                if(frontFace) {
                    fb.stencil[py][px]--;
                } else {
                    fb.stencil[py][px]++;
                }
            }
        }
    }
}

// ============================
// PPM Output
// ============================

void savePPM(const char* filename) {
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for(int y=0;y<H;y++) {
        for(int x=0;x<W;x++) {
            fwrite(&fb.color[y][x], 3, 1, f);
        }
    }
    fclose(f);
}

// ============================
// Main Rendering Pipeline
// ============================

int main() {
    printf("Shadow Volume Renderer (Z-Fail / Carmack's Reverse)\n");
    printf("Resolution: %dx%d\n", W, H);
    
    // --- 摄像机和光源 ---
    Vec3 eyePos = {0, 6, 10};
    Vec3 eyeAt  = {0, 1, 0};
    Vec3 lightPos = {3, 8, 5};
    
    float fovY = (float)M_PI / 3.0f;
    float aspect = (float)W / H;
    float nearZ = 0.1f, farZ = 100.0f;
    
    Mat4 view = lookAt(eyePos, eyeAt, {0,1,0});
    Mat4 proj = perspective(fovY, aspect, nearZ, farZ);
    Mat4 vp = proj * view;
    
    // --- 场景 ---
    // 地面（大白色平面）
    Mat4 floorT = translate({0,-1,0}) * scale({8, 0.2f, 8});
    Mesh floor = makeCube({0.8f, 0.8f, 0.7f}, floorT);
    
    // 后墙
    Mat4 backWallT = translate({0, 4, -6}) * scale({8, 6, 0.2f});
    Mesh backWall = makeCube({0.75f, 0.75f, 0.85f}, backWallT);
    
    // 左墙（红色）
    Mat4 leftWallT = translate({-8, 4, -2}) * scale({0.2f, 6, 8});
    Mesh leftWall = makeCube({0.85f, 0.2f, 0.2f}, leftWallT);
    
    // 右墙（绿色）
    Mat4 rightWallT = translate({8, 4, -2}) * scale({0.2f, 6, 8});
    Mesh rightWall = makeCube({0.2f, 0.75f, 0.2f}, rightWallT);
    
    // 投影物体1：球体（中心）
    Mat4 sphere1T = translate({0, 1.0f, 0}) * scale({1.2f, 1.2f, 1.2f});
    Mesh sphere1 = makeSphere({0.8f, 0.6f, 0.2f}, sphere1T);
    
    // 投影物体2：小立方体（右侧）
    Mat4 cube1T = translate({3, 0.5f, -1}) * rotateY(0.4f) * scale({0.8f, 1.5f, 0.8f});
    Mesh cube1 = makeCube({0.3f, 0.5f, 0.9f}, cube1T);
    
    // 投影物体3：小立方体（左侧）
    Mat4 cube2T = translate({-2.5f, 0.3f, 0.5f}) * rotateY(-0.3f) * scale({0.7f, 1.2f, 0.7f});
    Mesh cube2 = makeCube({0.6f, 0.3f, 0.7f}, cube2T);
    
    // 阴影投射物体列表
    std::vector<Mesh*> shadowCasters = {&sphere1, &cube1, &cube2};
    // 接收阴影的所有场景物体
    std::vector<Mesh*> allMeshes = {&floor, &backWall, &leftWall, &rightWall, &sphere1, &cube1, &cube2};
    
    printf("Building shadow volumes...\n");
    
    // 构建所有阴影体
    std::vector<ShadowVolumeMesh> shadowVolumes;
    for(auto* mesh : shadowCasters) {
        shadowVolumes.push_back(buildShadowVolume(*mesh, lightPos, 60.0f));
    }
    
    printf("Shadow volumes built: %zu volumes\n", shadowVolumes.size());
    int totalSVTris = 0;
    for(auto& sv : shadowVolumes) totalSVTris += (int)sv.verts.size()/3;
    printf("Total shadow volume triangles: %d\n", totalSVTris);
    
    // =========================================================
    // 渲染流程（Z-Fail方法）：
    // 1. 清空缓冲区
    // 2. 渲染整个场景（暂时不区分阴影，先填充深度缓冲）
    // 3. 清空模板缓冲，渲染阴影体更新模板
    // 4. 用模板缓冲决定哪些像素在阴影中，重新着色
    // =========================================================
    
    printf("Step 1: Clearing buffers...\n");
    fb.clear({15, 15, 25});
    
    printf("Step 2: Rendering scene (depth pass)...\n");
    
    // 先做一个简单的深度预通道（渲染所有面，填充深度缓冲）
    // 这步同时也渲染颜色，后面用模板修正
    for(auto* mesh : allMeshes) {
        int nFaces = (int)mesh->faces.size();
        for(int fi=0; fi<nFaces; fi++) {
            Vec3 p0 = mesh->worldVertex(mesh->faces[fi][0]);
            Vec3 p1 = mesh->worldVertex(mesh->faces[fi][1]);
            Vec3 p2 = mesh->worldVertex(mesh->faces[fi][2]);
            Vec3 wn = mesh->worldFaceNormal(fi);
            
            // 背面剔除
            Vec3 viewDir = (eyePos - p0).normalize();
            if(wn.dot(viewDir) < 0) continue;
            
            // 全光照渲染（假设不在阴影中）
            rasterizeTriangle(
                p0, p1, p2,
                wn, wn, wn,
                mesh->color,
                vp,
                lightPos, eyePos,
                [](int,int){ return false; }  // 先全部不是阴影
            );
        }
    }
    
    printf("Step 3: Computing stencil buffer with shadow volumes (Z-Fail)...\n");
    
    // 清空模板缓冲
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) fb.stencil[y][x]=0;
    
    // 渲染阴影体（更新模板缓冲）
    for(auto& sv : shadowVolumes) {
        int nTris = (int)sv.verts.size() / 3;
        for(int ti=0;ti<nTris;ti++) {
            Vec3 p0=sv.verts[ti*3+0], p1=sv.verts[ti*3+1], p2=sv.verts[ti*3+2];
            rasterizeShadowVolumeZFail(p0, p1, p2, vp);
        }
    }
    
    // 统计模板缓冲中阴影像素
    int shadowPixels = 0;
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
        if(fb.stencil[y][x] != 0) shadowPixels++;
    }
    printf("Shadow pixels (stencil != 0): %d / %d (%.1f%%)\n", 
           shadowPixels, W*H, 100.0f*shadowPixels/(W*H));
    
    printf("Step 4: Re-rendering scene with shadow information...\n");
    
    // 重新清除颜色和深度缓冲，用正确的阴影信息重新渲染
    // 先保存模板缓冲
    int savedStencil[H][W];
    memcpy(savedStencil, fb.stencil, sizeof(fb.stencil));
    
    // 重置颜色和深度
    for(int y=0;y<H;y++) for(int x=0;x<W;x++) {
        fb.color[y][x] = {15,15,25};
        fb.depth[y][x] = 1.0f;
    }
    
    // 用模板信息重新渲染
    for(auto* mesh : allMeshes) {
        int nFaces = (int)mesh->faces.size();
        for(int fi=0; fi<nFaces; fi++) {
            Vec3 p0 = mesh->worldVertex(mesh->faces[fi][0]);
            Vec3 p1 = mesh->worldVertex(mesh->faces[fi][1]);
            Vec3 p2 = mesh->worldVertex(mesh->faces[fi][2]);
            Vec3 wn = mesh->worldFaceNormal(fi);
            
            Vec3 viewDir = (eyePos - p0).normalize();
            if(wn.dot(viewDir) < 0) continue;
            
            rasterizeTriangle(
                p0, p1, p2,
                wn, wn, wn,
                mesh->color,
                vp,
                lightPos, eyePos,
                [&savedStencil](int px, int py){
                    return savedStencil[py][px] != 0;
                }
            );
        }
    }
    
    printf("Step 5: Drawing light source indicator...\n");
    
    // 在光源位置画一个亮点（小圆圈）
    Vec4 lightClip = vp * Vec4(lightPos, 1.0f);
    if(lightClip.w > 0) {
        ScreenPos ls = toScreen(lightClip);
        int lx = (int)ls.x, ly = (int)ls.y;
        for(int dy=-6; dy<=6; dy++) {
            for(int dx=-6; dx<=6; dx++) {
                if(dx*dx+dy*dy <= 36) {
                    int nx=lx+dx, ny=ly+dy;
                    if(nx>=0&&nx<W&&ny>=0&&ny<H) {
                        float r = sqrtf((float)(dx*dx+dy*dy));
                        if(r <= 4.0f)
                            fb.color[ny][nx] = {255, 255, 180};
                        else
                            fb.color[ny][nx] = {255, 200, 80};
                    }
                }
            }
        }
    }
    
    printf("Saving output...\n");
    savePPM("shadow_volume_output.ppm");
    
    // 转换为PNG
    int ret = system("convert shadow_volume_output.ppm shadow_volume_output.png 2>/dev/null");
    if(ret == 0) {
        printf("Saved: shadow_volume_output.png\n");
        remove("shadow_volume_output.ppm");
    } else {
        // 尝试Python转换
        ret = system("python3 -c \""
            "from PIL import Image; import struct\n"
            "with open('shadow_volume_output.ppm','rb') as f:\n"
            "    f.readline(); line=f.readline().split(); w,h=int(line[0]),int(line[1]); f.readline()\n"
            "    data=f.read()\n"
            "img=Image.frombytes('RGB',(w,h),data)\n"
            "img.save('shadow_volume_output.png')\n"
            "print('PNG saved via PIL')\n"
            "\" 2>/dev/null");
        if(ret == 0) {
            printf("Saved: shadow_volume_output.png (via PIL)\n");
            remove("shadow_volume_output.ppm");
        } else {
            printf("Saved: shadow_volume_output.ppm (PNG conversion failed)\n");
        }
    }
    
    printf("\n=== Render Complete ===\n");
    printf("Shadow volume triangles: %d\n", totalSVTris);
    printf("Shadow pixels: %d (%.1f%%)\n", shadowPixels, 100.0f*shadowPixels/(W*H));
    
    return 0;
}
