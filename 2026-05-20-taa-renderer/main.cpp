/*
 * Temporal Anti-Aliasing (TAA) Renderer
 * ======================================
 * 
 * 实现现代游戏引擎中的TAA技术，包含：
 * 1. Halton序列亚像素抖动 (Jitter) - 每帧偏移采样位置
 * 2. 运动矢量 (Motion Vectors) - 追踪物体运动
 * 3. 历史帧重投影 (Reprojection) - 利用上一帧结果
 * 4. 幽灵效果抑制 (Ghost Suppression) - Variance Clipping
 * 5. 混合权重自适应 - 动态调整历史帧权重
 * 
 * 渲染场景：旋转的彩色立方体阵列，展示TAA效果
 * 
 * 输出对比图像：
 * - 左列：无抗锯齿 (Raw - 锯齿明显)
 * - 中列：SSAA 4x (参考)
 * - 右列：TAA (16帧积累后的结果)
 * 
 * 输出文件：taa_output.png
 */

#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <random>
#include <numeric>

//=============================================================================
// 数学工具
//=============================================================================
static const float PI = 3.14159265358979323846f;

struct Vec2 { float x, y; };
struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0): x(x),y(y),z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x+y*b.y+z*b.z; }
    Vec3 cross(const Vec3& b) const { return {y*b.z-z*b.y, z*b.x-x*b.z, x*b.y-y*b.x}; }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>1e-6f ? *this/l : Vec3(0,0,1); }
    float& operator[](int i) { return ((float*)this)[i]; }
    float operator[](int i) const { return ((const float*)this)[i]; }
};

struct Vec4 {
    float x, y, z, w;
    Vec4(float x=0,float y=0,float z=0,float w=0): x(x),y(y),z(z),w(w) {}
    Vec4(Vec3 v, float w): x(v.x),y(v.y),z(v.z),w(w) {}
    Vec3 xyz() const { return {x,y,z}; }
};

// 4x4矩阵（列主序）
struct Mat4 {
    float m[16];
    Mat4() { memset(m, 0, sizeof(m)); }
    
    static Mat4 identity() {
        Mat4 r;
        r.m[0]=r.m[5]=r.m[10]=r.m[15]=1.0f;
        return r;
    }
    
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0]*v.x + m[4]*v.y + m[8]*v.z  + m[12]*v.w,
            m[1]*v.x + m[5]*v.y + m[9]*v.z  + m[13]*v.w,
            m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14]*v.w,
            m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15]*v.w
        };
    }
    
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for(int row=0; row<4; row++)
            for(int col=0; col<4; col++)
                for(int k=0; k<4; k++)
                    r.m[col*4+row] += m[k*4+row] * b.m[col*4+k];
        return r;
    }
    
    static Mat4 perspective(float fovy, float aspect, float znear, float zfar) {
        Mat4 r;
        float f = 1.0f / tanf(fovy * 0.5f);
        r.m[0]  = f / aspect;
        r.m[5]  = f;
        r.m[10] = (zfar + znear) / (znear - zfar);
        r.m[14] = (2.0f * zfar * znear) / (znear - zfar);
        r.m[11] = -1.0f;
        return r;
    }
    
    static Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
        Vec3 f = (center - eye).norm();
        Vec3 r = f.cross(up).norm();
        Vec3 u = r.cross(f);
        Mat4 mat = Mat4::identity();
        mat.m[0]=r.x;  mat.m[4]=r.y;  mat.m[8]=r.z;  mat.m[12]=-r.dot(eye);
        mat.m[1]=u.x;  mat.m[5]=u.y;  mat.m[9]=u.z;  mat.m[13]=-u.dot(eye);
        mat.m[2]=-f.x; mat.m[6]=-f.y; mat.m[10]=-f.z; mat.m[14]=f.dot(eye);
        mat.m[15]=1.0f;
        return mat;
    }
    
    static Mat4 rotate(float angle, Vec3 axis) {
        Mat4 r = Mat4::identity();
        float c = cosf(angle), s = sinf(angle);
        Vec3 a = axis.norm();
        r.m[0]  = c + a.x*a.x*(1-c);
        r.m[1]  = a.y*a.x*(1-c) + a.z*s;
        r.m[2]  = a.z*a.x*(1-c) - a.y*s;
        r.m[4]  = a.x*a.y*(1-c) - a.z*s;
        r.m[5]  = c + a.y*a.y*(1-c);
        r.m[6]  = a.z*a.y*(1-c) + a.x*s;
        r.m[8]  = a.x*a.z*(1-c) + a.y*s;
        r.m[9]  = a.y*a.z*(1-c) - a.x*s;
        r.m[10] = c + a.z*a.z*(1-c);
        return r;
    }
    
    static Mat4 translate(Vec3 t) {
        Mat4 r = Mat4::identity();
        r.m[12]=t.x; r.m[13]=t.y; r.m[14]=t.z;
        return r;
    }
    
    static Mat4 scale(float s) {
        Mat4 r = Mat4::identity();
        r.m[0]=r.m[5]=r.m[10]=s;
        return r;
    }
};

//=============================================================================
// Halton序列（低差异序列，用于TAA抖动）
//=============================================================================
float halton(int index, int base) {
    float f = 1.0f;
    float r = 0.0f;
    int i = index;
    while(i > 0) {
        f /= (float)base;
        r += f * (float)(i % base);
        i /= base;
    }
    return r;
}

// Halton(2,3)序列 - TAA标准抖动
Vec2 haltonJitter(int frameIndex) {
    // 返回 [-0.5, 0.5] 范围的抖动偏移
    return {
        halton(frameIndex+1, 2) - 0.5f,
        halton(frameIndex+1, 3) - 0.5f
    };
}

//=============================================================================
// 帧缓冲
//=============================================================================
struct Framebuffer {
    int width, height;
    std::vector<Vec3> color;
    std::vector<float> depth;
    // 运动矢量缓冲（像素空间）
    std::vector<Vec2> motion;
    
    Framebuffer(int w, int h) : width(w), height(h),
        color(w*h, Vec3(0,0,0)),
        depth(w*h, 1.0f),
        motion(w*h, {0,0}) {}
    
    void clear(Vec3 bgColor = Vec3(0.05f, 0.05f, 0.1f)) {
        std::fill(color.begin(), color.end(), bgColor);
        std::fill(depth.begin(), depth.end(), 1.0f);
        std::fill(motion.begin(), motion.end(), Vec2{0,0});
    }
    
    Vec3& pixel(int x, int y) { return color[y*width+x]; }
    float& depth_at(int x, int y) { return depth[y*width+x]; }
    Vec2& motion_at(int x, int y) { return motion[y*width+x]; }
};

//=============================================================================
// TAA历史缓冲
//=============================================================================
struct TAABuffer {
    int width, height;
    std::vector<Vec3> accumulated;  // 积累的颜色
    std::vector<Vec3> history;      // 上一帧颜色
    int frameCount;
    
    TAABuffer(int w, int h) : width(w), height(h),
        accumulated(w*h, Vec3(0,0,0)),
        history(w*h, Vec3(0,0,0)),
        frameCount(0) {}
    
    void swapBuffers() {
        std::swap(accumulated, history);
    }
};

//=============================================================================
// 软光栅化 - 三角形顶点
//=============================================================================
struct Vertex {
    Vec4 pos;    // 裁剪空间位置
    Vec4 prevPos; // 上一帧裁剪空间位置（用于运动矢量）
    Vec3 color;
    Vec3 normal;
};

//=============================================================================
// Variance Clipping - TAA幽灵效果抑制
// 将历史样本裁剪到当前帧邻域的颜色范围内
//=============================================================================
Vec3 clipToAABB(Vec3 histColor, Vec3 aabbMin, Vec3 aabbMax) {
    // 如果已在范围内，直接返回
    if(histColor.x >= aabbMin.x && histColor.x <= aabbMax.x &&
       histColor.y >= aabbMin.y && histColor.y <= aabbMax.y &&
       histColor.z >= aabbMin.z && histColor.z <= aabbMax.z) {
        return histColor;
    }
    // 将历史颜色裁剪到AABB内
    return Vec3(
        std::max(aabbMin.x, std::min(aabbMax.x, histColor.x)),
        std::max(aabbMin.y, std::min(aabbMax.y, histColor.y)),
        std::max(aabbMin.z, std::min(aabbMax.z, histColor.z))
    );
}

// Variance Clipping：基于统计方差计算颜色包围盒
void varianceClipping(const Framebuffer& fb, int px, int py,
                      Vec3& aabbMin, Vec3& aabbMax, int radius=1) {
    Vec3 m1(0,0,0), m2(0,0,0);
    int count = 0;
    for(int dy=-radius; dy<=radius; dy++) {
        for(int dx=-radius; dx<=radius; dx++) {
            int nx = px+dx, ny = py+dy;
            if(nx<0||nx>=fb.width||ny<0||ny>=fb.height) continue;
            Vec3 c = fb.color[ny*fb.width+nx];
            m1 += c;
            m2 += c*c;
            count++;
        }
    }
    if(count == 0) { aabbMin=aabbMax=fb.color[py*fb.width+px]; return; }
    m1 = m1 / (float)count;
    m2 = m2 / (float)count;
    Vec3 sigma = Vec3(
        sqrtf(std::max(0.0f, m2.x - m1.x*m1.x)),
        sqrtf(std::max(0.0f, m2.y - m1.y*m1.y)),
        sqrtf(std::max(0.0f, m2.z - m1.z*m1.z))
    );
    float gamma = 1.2f;
    aabbMin = m1 - sigma * gamma;
    aabbMax = m1 + sigma * gamma;
    // 钳制到[0,1]
    aabbMin.x = std::max(0.0f, aabbMin.x);
    aabbMin.y = std::max(0.0f, aabbMin.y);
    aabbMin.z = std::max(0.0f, aabbMin.z);
    aabbMax.x = std::min(1.0f, aabbMax.x);
    aabbMax.y = std::min(1.0f, aabbMax.y);
    aabbMax.z = std::min(1.0f, aabbMax.z);
}

//=============================================================================
// TAA混合 - 核心算法
//=============================================================================
void applyTAA(const Framebuffer& current, TAABuffer& taa, bool ghostSuppression=true) {
    int W = current.width, H = current.height;
    
    for(int y=0; y<H; y++) {
        for(int x=0; x<W; x++) {
            Vec3 currentColor = current.color[y*W+x];
            Vec2 motion = current.motion[y*W+x];
            
            // 重投影：根据运动矢量找到上一帧对应位置
            float prevX = (float)x - motion.x;
            float prevY = (float)y - motion.y;
            
            // 双线性采样历史帧
            int px0 = (int)prevX, py0 = (int)prevY;
            int px1 = px0+1, py1 = py0+1;
            float fx = prevX - (float)px0;
            float fy = prevY - (float)py0;
            
            Vec3 histColor;
            bool validHistory = (px0>=0 && px1<W && py0>=0 && py1<H);
            
            if(validHistory) {
                // 双线性插值
                Vec3 c00 = taa.history[py0*W+px0];
                Vec3 c10 = taa.history[py0*W+px1];
                Vec3 c01 = taa.history[py1*W+px0];
                Vec3 c11 = taa.history[py1*W+px1];
                histColor = (c00*(1-fx) + c10*fx)*(1-fy) + 
                            (c01*(1-fx) + c11*fx)*fy;
            } else {
                histColor = currentColor;
            }
            
            // 幽灵效果抑制（Variance Clipping）
            if(ghostSuppression && validHistory) {
                Vec3 aabbMin, aabbMax;
                varianceClipping(current, x, y, aabbMin, aabbMax);
                histColor = clipToAABB(histColor, aabbMin, aabbMax);
            }
            
            // 自适应混合权重
            // 运动大时降低历史权重，减少拖尾
            float motionLen = sqrtf(motion.x*motion.x + motion.y*motion.y);
            float blendFactor = 0.1f; // 基础混合：10% 当前帧 + 90% 历史
            if(motionLen > 0.5f) {
                blendFactor = std::min(0.5f, 0.1f + motionLen * 0.05f);
            }
            if(!validHistory) blendFactor = 1.0f;
            
            // 指数移动平均
            Vec3 taaColor = currentColor * blendFactor + histColor * (1.0f - blendFactor);
            
            taa.accumulated[y*W+x] = taaColor;
        }
    }
    
    taa.swapBuffers();
    taa.frameCount++;
}

//=============================================================================
// 软光栅化核心
//=============================================================================
void rasterizeTriangle(Framebuffer& fb, Vertex v0, Vertex v1, Vertex v2) {
    int W = fb.width, H = fb.height;
    
    auto toScreen = [&](Vec4 p) -> Vec3 {
        if(fabsf(p.w) < 1e-6f) return Vec3(0,0,0);
        float invW = 1.0f / p.w;
        return Vec3(
            (p.x * invW * 0.5f + 0.5f) * (float)(W-1),
            (1.0f - (p.y * invW * 0.5f + 0.5f)) * (float)(H-1),
            p.z * invW
        );
    };
    
    Vec3 s0 = toScreen(v0.pos);
    Vec3 s1 = toScreen(v1.pos);
    Vec3 s2 = toScreen(v2.pos);
    
    // 背面剔除（已在世界空间处理，这里跳过）
    
    // 包围盒
    int minX = std::max(0, (int)std::min({s0.x, s1.x, s2.x}));
    int maxX = std::min(W-1, (int)std::max({s0.x, s1.x, s2.x})+1);
    int minY = std::max(0, (int)std::min({s0.y, s1.y, s2.y}));
    int maxY = std::min(H-1, (int)std::max({s0.y, s1.y, s2.y})+1);
    
    float area = (s1.x-s0.x)*(s2.y-s0.y) - (s1.y-s0.y)*(s2.x-s0.x);
    if(fabsf(area) < 1e-6f) return;
    
    for(int py=minY; py<=maxY; py++) {
        for(int px=minX; px<=maxX; px++) {
            float fx = (float)px + 0.5f;
            float fy = (float)py + 0.5f;
            
            float w0 = ((s1.x-s0.x)*(fy-s0.y) - (s1.y-s0.y)*(fx-s0.x)) / area;
            float w1 = ((s2.x-s1.x)*(fy-s1.y) - (s2.y-s1.y)*(fx-s1.x)) / area;
            float w2 = 1.0f - w0 - w1;
            
            if(w0 < 0 || w1 < 0 || w2 < 0) continue;
            
            float z = s0.z*w2 + s1.z*w0 + s2.z*w1;
            if(z < -1.0f || z > 1.0f) continue;
            
            if(z >= fb.depth_at(px, py)) continue;
            fb.depth_at(px, py) = z;
            
            // 插值颜色和法线
            Vec3 color = v0.color*w2 + v1.color*w0 + v2.color*w1;
            Vec3 normal = (v0.normal*w2 + v1.normal*w0 + v2.normal*w1).norm();
            
            // 简单光照
            Vec3 lightDir = Vec3(1,2,1).norm();
            float diff = std::max(0.0f, normal.dot(lightDir));
            float ambient = 0.15f;
            Vec3 lit = color * (ambient + diff * 0.85f);
            
            // 高光
            Vec3 viewDir = Vec3(0,0,-1);
            Vec3 halfVec = (lightDir + viewDir).norm();
            float spec = powf(std::max(0.0f, normal.dot(halfVec)), 32.0f) * 0.4f;
            lit = lit + Vec3(spec, spec, spec);
            lit.x = std::min(1.0f, lit.x);
            lit.y = std::min(1.0f, lit.y);
            lit.z = std::min(1.0f, lit.z);
            
            fb.pixel(px, py) = lit;
            
            // 计算运动矢量
            // 将当前像素反投影到上一帧
            auto& mo = fb.motion_at(px, py);
            // 重心坐标插值上一帧裁剪空间位置
            Vec4 prevClip = Vec4(
                v0.prevPos.x*w2 + v1.prevPos.x*w0 + v2.prevPos.x*w1,
                v0.prevPos.y*w2 + v1.prevPos.y*w0 + v2.prevPos.y*w1,
                v0.prevPos.z*w2 + v1.prevPos.z*w0 + v2.prevPos.z*w1,
                v0.prevPos.w*w2 + v1.prevPos.w*w0 + v2.prevPos.w*w1
            );
            if(fabsf(prevClip.w) > 1e-6f) {
                float invPrevW = 1.0f / prevClip.w;
                float prevScreenX = (prevClip.x * invPrevW * 0.5f + 0.5f) * (float)(W-1);
                float prevScreenY = (1.0f - (prevClip.y * invPrevW * 0.5f + 0.5f)) * (float)(H-1);
                mo.x = (float)px - prevScreenX;
                mo.y = (float)py - prevScreenY;
            }
        }
    }
}

//=============================================================================
// 场景：立方体
//=============================================================================
// 立方体的8个顶点（局部坐标，-0.5~0.5）
static const Vec3 CUBE_VERTS[8] = {
    {-0.5f,-0.5f,-0.5f}, {0.5f,-0.5f,-0.5f},
    {0.5f, 0.5f,-0.5f},  {-0.5f,0.5f,-0.5f},
    {-0.5f,-0.5f, 0.5f}, {0.5f,-0.5f, 0.5f},
    {0.5f, 0.5f, 0.5f},  {-0.5f,0.5f, 0.5f}
};

// 6个面，每面2个三角形，每个三角形3个顶点索引，以及法线
struct Face {
    int idx[4];  // 4个顶点（矩形面）
    Vec3 normal;
};

static const Face CUBE_FACES[6] = {
    {{0,1,2,3}, {0,0,-1}},  // 前
    {{5,4,7,6}, {0,0, 1}},  // 后
    {{4,0,3,7}, {-1,0,0}},  // 左
    {{1,5,6,2}, { 1,0,0}},  // 右
    {{3,2,6,7}, {0, 1,0}},  // 上
    {{4,5,1,0}, {0,-1,0}},  // 下
};

struct CubeObject {
    Vec3 position;
    Vec3 color;
    float rotationY;
    float rotSpeed;
};

void renderCube(Framebuffer& fb, const CubeObject& cube,
                const Mat4& viewProj, const Mat4& /*prevViewProj*/,
                float jitterX, float jitterY, bool applyJitter) {
    int W = fb.width, H = fb.height;
    
    float angle = cube.rotationY;
    Mat4 model = Mat4::translate(cube.position) * Mat4::rotate(angle, Vec3(0,1,0.3f).norm());
    Mat4 prevAngle_model = Mat4::translate(cube.position) * Mat4::rotate(angle - cube.rotSpeed, Vec3(0,1,0.3f).norm());
    
    // 构造抖动矩阵（仅影响投影，不影响运动矢量计算）
    Mat4 jitterMat = Mat4::identity();
    if(applyJitter) {
        jitterMat.m[12] = jitterX * 2.0f / (float)W;
        jitterMat.m[13] = -jitterY * 2.0f / (float)H;
    }
    
    Mat4 mvp      = jitterMat * viewProj * model;
    Mat4 prevMVP  = viewProj * prevAngle_model; // 上一帧（无抖动用于运动矢量）
    for(const Face& face : CUBE_FACES) {
        // 背面剔除（世界空间法线检测）
        Mat4 normalMat = Mat4::rotate(angle, Vec3(0,1,0.3f).norm());
        Vec4 wn = normalMat * Vec4(face.normal, 0.0f);
        Vec3 worldNormal = wn.xyz().norm();
        Vec3 viewDir = Vec3(0,0,-1); // 相机看向-Z
        if(worldNormal.dot(viewDir) >= 0) continue; // 背面
        
        // 渲染两个三角形
        int quads[2][3] = {{face.idx[0], face.idx[1], face.idx[2]},
                           {face.idx[0], face.idx[2], face.idx[3]}};
        
        for(auto& tri : quads) {
            Vertex verts[3];
            for(int i=0; i<3; i++) {
                Vec4 lp(CUBE_VERTS[tri[i]], 1.0f);
                verts[i].pos      = mvp * lp;
                verts[i].prevPos  = prevMVP * lp;
                verts[i].color    = cube.color;
                Vec4 wn2 = normalMat * Vec4(face.normal, 0.0f);
                verts[i].normal   = wn2.xyz().norm();
            }
            // 视锥剔除（简单检查）
            bool allOutside = true;
            for(int i=0; i<3; i++) {
                float w = verts[i].pos.w;
                if(w > 0 &&
                   fabsf(verts[i].pos.x) <= w*1.1f &&
                   fabsf(verts[i].pos.y) <= w*1.1f &&
                   verts[i].pos.z >= -w && verts[i].pos.z <= w) {
                    allOutside = false;
                    break;
                }
            }
            if(allOutside) continue;
            
            rasterizeTriangle(fb, verts[0], verts[1], verts[2]);
        }
    }
}

//=============================================================================
// SSAA 4x（参考实现）
//=============================================================================
// renderPixelSSAA removed - using framebuffer-based SSAA instead

//=============================================================================
// PNG写入（纯C++，无外部库）
//=============================================================================
// Adler32校验
static uint32_t adler32(const uint8_t* data, size_t len) {
    uint32_t s1=1, s2=0;
    for(size_t i=0; i<len; i++) {
        s1 = (s1 + data[i]) % 65521;
        s2 = (s2 + s1) % 65521;
    }
    return (s2<<16)|s1;
}

// CRC32
static uint32_t crc32_table[256];
static bool crc32_inited = false;
static void initCRC32() {
    for(uint32_t i=0; i<256; i++) {
        uint32_t c=i;
        for(int j=0; j<8; j++) c = (c&1)?(0xEDB88320^(c>>1)):(c>>1);
        crc32_table[i]=c;
    }
    crc32_inited=true;
}
static uint32_t crc32(const uint8_t* data, size_t len) {
    if(!crc32_inited) initCRC32();
    uint32_t c=0xFFFFFFFF;
    for(size_t i=0; i<len; i++) c = crc32_table[(c^data[i])&0xFF]^(c>>8);
    return c^0xFFFFFFFF;
}

// DEFLATE（仅存储模式，无压缩）
static std::vector<uint8_t> deflateStore(const std::vector<uint8_t>& data) {
    std::vector<uint8_t> out;
    // zlib头
    out.push_back(0x78); out.push_back(0x01);
    
    size_t pos=0;
    const size_t BLOCK=65535;
    while(pos < data.size()) {
        size_t remaining = data.size() - pos;
        size_t blockSize = std::min(remaining, BLOCK);
        bool last = (pos + blockSize >= data.size());
        out.push_back(last ? 1 : 0);
        out.push_back((uint8_t)(blockSize & 0xFF));
        out.push_back((uint8_t)((blockSize>>8) & 0xFF));
        out.push_back((uint8_t)(~blockSize & 0xFF));
        out.push_back((uint8_t)((~blockSize>>8) & 0xFF));
        for(size_t i=0; i<blockSize; i++) out.push_back(data[pos+i]);
        pos += blockSize;
    }
    
    uint32_t a = adler32(data.data(), data.size());
    out.push_back((a>>24)&0xFF);
    out.push_back((a>>16)&0xFF);
    out.push_back((a>>8)&0xFF);
    out.push_back(a&0xFF);
    return out;
}

static void writeU32BE(std::vector<uint8_t>& buf, uint32_t v) {
    buf.push_back((v>>24)&0xFF);
    buf.push_back((v>>16)&0xFF);
    buf.push_back((v>>8)&0xFF);
    buf.push_back(v&0xFF);
}

static void writePNGChunk(std::vector<uint8_t>& out, const char* type, const std::vector<uint8_t>& data) {
    writeU32BE(out, (uint32_t)data.size());
    uint8_t t[4] = {(uint8_t)type[0],(uint8_t)type[1],(uint8_t)type[2],(uint8_t)type[3]};
    out.insert(out.end(), t, t+4);
    out.insert(out.end(), data.begin(), data.end());
    std::vector<uint8_t> crcData(t,t+4);
    crcData.insert(crcData.end(), data.begin(), data.end());
    writeU32BE(out, crc32(crcData.data(), crcData.size()));
}

bool savePNG(const std::string& filename, int W, int H, const std::vector<Vec3>& pixels) {
    // 构建raw图像数据（每行前加filter字节0）
    std::vector<uint8_t> raw;
    raw.reserve((size_t)(W*3+1)*H);
    for(int y=0; y<H; y++) {
        raw.push_back(0); // filter none
        for(int x=0; x<W; x++) {
            Vec3 c = pixels[y*W+x];
            // gamma correction
            c.x = powf(std::max(0.0f,std::min(1.0f,c.x)), 1.0f/2.2f);
            c.y = powf(std::max(0.0f,std::min(1.0f,c.y)), 1.0f/2.2f);
            c.z = powf(std::max(0.0f,std::min(1.0f,c.z)), 1.0f/2.2f);
            raw.push_back((uint8_t)(c.x*255));
            raw.push_back((uint8_t)(c.y*255));
            raw.push_back((uint8_t)(c.z*255));
        }
    }
    
    auto compressed = deflateStore(raw);
    
    std::vector<uint8_t> out;
    // PNG签名
    uint8_t sig[] = {137,80,78,71,13,10,26,10};
    out.insert(out.end(), sig, sig+8);
    
    // IHDR
    std::vector<uint8_t> ihdr;
    writeU32BE(ihdr, (uint32_t)W);
    writeU32BE(ihdr, (uint32_t)H);
    ihdr.push_back(8); ihdr.push_back(2); // 8-bit RGB
    ihdr.push_back(0); ihdr.push_back(0); ihdr.push_back(0);
    writePNGChunk(out, "IHDR", ihdr);
    
    // IDAT
    writePNGChunk(out, "IDAT", compressed);
    
    // IEND
    writePNGChunk(out, "IEND", {});
    
    std::ofstream f(filename, std::ios::binary);
    if(!f) return false;
    f.write((const char*)out.data(), (std::streamsize)out.size());
    return true;
}

//=============================================================================
// 主渲染循环
//=============================================================================
int main() {
    // 分辨率：输出图像为3列对比图（无AA / SSAA / TAA），每列200x300
    const int COL_W = 200, COL_H = 300;
    const int OUT_W = COL_W * 3 + 20; // 3列 + 分隔
    const int OUT_H = COL_H + 40;     // 顶部标题区域
    const int TAA_FRAMES = 20;        // TAA积累帧数
    
    // 场景：立方体阵列
    std::vector<CubeObject> cubes;
    // 3x2阵列的立方体，不同颜色
    std::vector<Vec3> cubeColors = {
        {1.0f, 0.3f, 0.2f},
        {0.3f, 0.8f, 0.3f},
        {0.2f, 0.4f, 1.0f},
        {1.0f, 0.8f, 0.1f},
        {0.8f, 0.3f, 0.9f},
        {0.2f, 0.9f, 0.9f}
    };
    
    int ci = 0;
    for(int row=0; row<2; row++) {
        for(int col=0; col<3; col++) {
            CubeObject c;
            c.position = Vec3((col-1) * 1.4f, (row==0 ? 0.6f : -0.6f), 0.0f);
            c.color = cubeColors[ci++];
            c.rotationY = 0.0f;
            c.rotSpeed = 0.04f + (float)ci * 0.008f; // 每帧旋转量
            cubes.push_back(c);
        }
    }
    
    // 相机和投影
    Vec3 eye(0, 0, 5);
    Vec3 center(0, 0, 0);
    Vec3 up(0, 1, 0);
    float fov = PI / 4.0f;
    float aspect = (float)COL_W / (float)COL_H;
    
    Mat4 view = Mat4::lookAt(eye, center, up);
    Mat4 proj = Mat4::perspective(fov, aspect, 0.1f, 100.0f);
    Mat4 viewProj = proj * view;
    
    // TAA帧缓冲
    Framebuffer fbTAA(COL_W, COL_H);
    TAABuffer taa(COL_W, COL_H);
    
    // 无AA帧缓冲（最后一帧状态）
    Framebuffer fbNoAA(COL_W, COL_H);
    
    // SSAA帧缓冲（使用2x超采样）
    const int SSAA = 2;
    Framebuffer fbSSAA_high(COL_W*SSAA, COL_H*SSAA);
    
    // ---- 渲染循环：积累TAA帧 ----
    for(int frame=0; frame<TAA_FRAMES; frame++) {
        // 更新立方体旋转（上一帧位置已保留在rotationY中）
        Mat4 prevViewProj = viewProj; // 相机不动，不需要更新
        
        // 当前帧旋转角度
        for(auto& cube : cubes) {
            cube.rotationY += cube.rotSpeed;
        }
        
        // Halton抖动（TAA核心特性）
        Vec2 jitter = haltonJitter(frame);
        
        // === 渲染无AA版本（最后几帧取最后一帧）===
        if(frame == TAA_FRAMES-1) {
            fbNoAA.clear();
            for(auto& cube : cubes)
                renderCube(fbNoAA, cube, viewProj, prevViewProj, 0,0, false);
        }
        
        // === 渲染SSAA版本（最后一帧超采样）===
        if(frame == TAA_FRAMES-1) {
            // 渲染高分辨率帧
            Framebuffer fbHigh(COL_W*SSAA, COL_H*SSAA);
            fbHigh.clear();
            
            // 构建高分辨率投影
            float aspectHigh = (float)(COL_W*SSAA) / (float)(COL_H*SSAA);
            Mat4 projHigh = Mat4::perspective(fov, aspectHigh, 0.1f, 100.0f);
            Mat4 vpHigh = projHigh * view;
            
            for(auto& cube : cubes)
                renderCube(fbHigh, cube, vpHigh, vpHigh, 0,0, false);
            
            // 降采样到输出分辨率
            fbSSAA_high.color = fbHigh.color;
            fbSSAA_high.width = COL_W*SSAA;
            fbSSAA_high.height = COL_H*SSAA;
        }
        
        // === 渲染TAA版本（带抖动，每帧积累）===
        fbTAA.clear();
        for(auto& cube : cubes)
            renderCube(fbTAA, cube, viewProj, prevViewProj, jitter.x, jitter.y, true);
        
        // 应用TAA
        applyTAA(fbTAA, taa);
    }
    
    // 将SSAA高分辨率降采样到COL_W x COL_H
    std::vector<Vec3> ssaaResult(COL_W * COL_H);
    for(int y=0; y<COL_H; y++) {
        for(int x=0; x<COL_W; x++) {
            Vec3 sum(0,0,0);
            for(int sy=0; sy<SSAA; sy++)
                for(int sx=0; sx<SSAA; sx++)
                    sum += fbSSAA_high.color[(y*SSAA+sy)*COL_W*SSAA+(x*SSAA+sx)];
            ssaaResult[y*COL_W+x] = sum / (float)(SSAA*SSAA);
        }
    }
    
    // TAA最终结果（history buffer中存的是最新积累结果）
    const auto& taaResult = taa.history;
    
    // ==== 拼合输出图像 ====
    // 颜色：深蓝灰背景
    Vec3 bgColor(0.08f, 0.09f, 0.12f);
    Vec3 titleBg(0.12f, 0.14f, 0.18f);
    Vec3 white(1,1,1);
    Vec3 yellow(1.0f, 0.9f, 0.3f);
    Vec3 green(0.4f, 1.0f, 0.5f);
    Vec3 red(1.0f, 0.4f, 0.3f);
    
    std::vector<Vec3> outPixels(OUT_W * OUT_H, bgColor);
    
    // 标题区背景
    for(int y=0; y<40; y++)
        for(int x=0; x<OUT_W; x++)
            outPixels[y*OUT_W+x] = titleBg;
    
    // 复制各列到输出
    auto copyCol = [&](const std::vector<Vec3>& src, int srcW, int dstOffX, int dstOffY) {
        for(int y=0; y<COL_H; y++)
            for(int x=0; x<COL_W; x++)
                outPixels[(y+dstOffY)*OUT_W + (x+dstOffX)] = src[y*srcW+x];
    };
    
    int offY = 40;
    copyCol(fbNoAA.color, COL_W, 0, offY);       // 左：无AA
    copyCol(ssaaResult, COL_W, COL_W+10, offY);   // 中：SSAA
    copyCol(taaResult, COL_W, COL_W*2+20, offY);  // 右：TAA
    
    // 绘制分隔线
    for(int y=offY; y<offY+COL_H; y++) {
        if(COL_W+5 < OUT_W) outPixels[y*OUT_W + COL_W+5] = Vec3(0.3f,0.3f,0.4f);
        if(COL_W*2+15 < OUT_W) outPixels[y*OUT_W + COL_W*2+15] = Vec3(0.3f,0.3f,0.4f);
    }
    
    // 在标题区绘制小点标识（标题用简单像素块）
    auto drawLabel = [&](int startX, int startY, Vec3 color, const char* text) {
        (void)text;
        // 绘制一个小色块作为标签指示
        for(int dy=0; dy<6; dy++)
            for(int dx=0; dx<20; dx++)
                if(startX+dx < OUT_W && startY+dy < OUT_H)
                    outPixels[(startY+dy)*OUT_W + startX+dx] = color;
    };
    
    // 标题色块（红=无AA，绿=SSAA，蓝=TAA）
    drawLabel(COL_W/2 - 10, 12, red, "NO AA");
    drawLabel(COL_W + 10 + COL_W/2 - 10, 12, yellow, "SSAA 2x");
    drawLabel(COL_W*2 + 20 + COL_W/2 - 10, 12, green, "TAA 20f");
    
    // 绘制水平分隔线（标题与内容间）
    for(int x=0; x<OUT_W; x++)
        outPixels[39*OUT_W+x] = Vec3(0.25f, 0.3f, 0.4f);
    
    // 保存PNG
    if(!savePNG("taa_output.png", OUT_W, OUT_H, outPixels)) {
        fprintf(stderr, "Failed to save taa_output.png\n");
        return 1;
    }
    
    printf("✅ TAA Renderer完成!\n");
    printf("   输出: taa_output.png (%dx%d)\n", OUT_W, OUT_H);
    printf("   对比: 无AA | SSAA 2x | TAA 20帧积累\n");
    printf("   立方体数量: %zu\n", cubes.size());
    printf("   TAA帧数: %d\n", TAA_FRAMES);
    printf("   Halton序列抖动: 启用 (base 2,3)\n");
    printf("   Variance Clipping: 启用\n");
    printf("   自适应混合权重: 启用\n");
    
    return 0;
}
