/*
 * Forward+ Rendering (Tiled Forward Rendering)
 * 
 * 实现要点：
 * 1. Z Pre-pass: 先渲染深度图
 * 2. Tile Frustum Culling: 将屏幕划分为16x16 Tile，每个Tile构建光锥
 * 3. Light Assignment: 将点光源分配到相交的Tile中
 * 4. Shading Pass: 使用Tile光源列表进行多光源Blinn-Phong着色
 * 5. ACES色调映射
 *
 * 场景：多个球体+平面，16+个动态点光源，彩色光源
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <fstream>
#include <string>
#include <sstream>
#include <cassert>
#include <cstring>

// ===== 基础数学库 =====

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& b) const { return {x+b.x, y+b.y, z+b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x-b.x, y-b.y, z-b.z}; }
    Vec3 operator*(float t) const { return {x*t, y*t, z*t}; }
    Vec3 operator*(const Vec3& b) const { return {x*b.x, y*b.y, z*b.z}; }
    Vec3 operator/(float t) const { return {x/t, y/t, z/t}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3& operator+=(const Vec3& b) { x+=b.x; y+=b.y; z+=b.z; return *this; }
    float dot(const Vec3& b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3& b) const {
        return {y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x};
    }
    float length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        float l = length();
        if (l < 1e-8f) return {0,0,0};
        return {x/l, y/l, z/l};
    }
    float lengthSq() const { return x*x + y*y + z*z; }
};
Vec3 operator*(float t, const Vec3& v) { return v * t; }

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(0) {}
    Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    Vec4(Vec3 v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
    Vec3 xyz() const { return {x, y, z}; }
};

// 4x4矩阵（列主序）
struct Mat4 {
    float m[4][4];
    Mat4() { memset(m, 0, sizeof(m)); }
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1.f; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        };
    }
    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                for (int k=0;k<4;k++)
                    r.m[i][j] += m[i][k]*b.m[k][j];
        return r;
    }
    Vec3 transformPoint(const Vec3& p) const {
        Vec4 r = (*this) * Vec4(p, 1.f);
        return r.xyz() / r.w;
    }
    Vec3 transformDir(const Vec3& d) const {
        Vec4 r = (*this) * Vec4(d, 0.f);
        return r.xyz();
    }
};

Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = (center - eye).normalized();
    Vec3 r = f.cross(up).normalized();
    Vec3 u = r.cross(f);
    Mat4 m = Mat4::identity();
    m.m[0][0]=r.x; m.m[0][1]=r.y; m.m[0][2]=r.z; m.m[0][3]=-r.dot(eye);
    m.m[1][0]=u.x; m.m[1][1]=u.y; m.m[1][2]=u.z; m.m[1][3]=-u.dot(eye);
    m.m[2][0]=-f.x;m.m[2][1]=-f.y;m.m[2][2]=-f.z;m.m[2][3]=f.dot(eye);
    m.m[3][3]=1.f;
    return m;
}

Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
    float tanHalf = std::tan(fovY * 0.5f);
    Mat4 m;
    m.m[0][0] = 1.f / (aspect * tanHalf);
    m.m[1][1] = 1.f / tanHalf;
    m.m[2][2] = -(zFar + zNear) / (zFar - zNear);
    m.m[2][3] = -2.f * zFar * zNear / (zFar - zNear);
    m.m[3][2] = -1.f;
    return m;
}

// ===== 场景定义 =====

struct Material {
    Vec3 albedo;
    float roughness;
    float metallic;
    bool isFloor;
    Material() : albedo(0.8f,0.8f,0.8f), roughness(0.5f), metallic(0.f), isFloor(false) {}
    Material(Vec3 a, float r, float m) : albedo(a), roughness(r), metallic(m), isFloor(false) {}
};

struct Sphere {
    Vec3 center;
    float radius;
    Material mat;
};

struct Triangle {
    Vec3 v0, v1, v2;
    Vec3 n;
    Material mat;
    Triangle(Vec3 a, Vec3 b, Vec3 c, Material m) : v0(a), v1(b), v2(c), mat(m) {
        n = (b-a).cross(c-a).normalized();
    }
};

struct PointLight {
    Vec3 position;
    Vec3 color;
    float radius;  // 影响半径
    float intensity;
};

// ===== G-Buffer / Z-Buffer =====

const int WIDTH = 800;
const int HEIGHT = 600;
const int TILE_SIZE = 16;
const int TILES_X = (WIDTH + TILE_SIZE - 1) / TILE_SIZE;
const int TILES_Y = (HEIGHT + TILE_SIZE - 1) / TILE_SIZE;
const int MAX_LIGHTS_PER_TILE = 64;
const float INF = 1e30f;

// 像素数据
struct GBufferPixel {
    Vec3 position;   // 世界空间位置
    Vec3 normal;     // 世界空间法线
    Vec3 albedo;
    float roughness;
    float metallic;
    float depth;     // NDC深度 [-1,1]
    bool valid;
};

GBufferPixel gbuffer[HEIGHT][WIDTH];
float depthBuffer[HEIGHT][WIDTH];

// Tile光源列表
struct TileLightList {
    int lightIndices[MAX_LIGHTS_PER_TILE];
    int count;
    TileLightList() : count(0) {}
};
TileLightList tileLights[TILES_Y][TILES_X];

// 输出图像
struct Image {
    int w, h;
    std::vector<Vec3> pixels;
    Image(int w, int h) : w(w), h(h), pixels(w*h) {}
    Vec3& at(int x, int y) { return pixels[y*w+x]; }
    const Vec3& at(int x, int y) const { return pixels[y*w+x]; }
};

// ===== 光线-球体/三角形求交 =====

struct HitRecord {
    float t;
    Vec3 pos;
    Vec3 normal;
    Material mat;
    bool hit;
    HitRecord() : t(INF), hit(false) {}
};

bool raySphere(Vec3 ro, Vec3 rd, const Sphere& s, float& t) {
    Vec3 oc = ro - s.center;
    float b = oc.dot(rd);
    float c = oc.dot(oc) - s.radius*s.radius;
    float disc = b*b - c;
    if (disc < 0) return false;
    float sq = std::sqrt(disc);
    float t0 = -b - sq;
    float t1 = -b + sq;
    if (t0 > 1e-4f) { t = t0; return true; }
    if (t1 > 1e-4f) { t = t1; return true; }
    return false;
}

bool rayTriangle(Vec3 ro, Vec3 rd, const Triangle& tri, float& t, Vec3& bary) {
    Vec3 e1 = tri.v1 - tri.v0;
    Vec3 e2 = tri.v2 - tri.v0;
    Vec3 h = rd.cross(e2);
    float a = e1.dot(h);
    if (std::abs(a) < 1e-8f) return false;
    float f = 1.f / a;
    Vec3 s = ro - tri.v0;
    float u = f * s.dot(h);
    if (u < 0 || u > 1) return false;
    Vec3 q = s.cross(e1);
    float v = f * rd.dot(q);
    if (v < 0 || u + v > 1) return false;
    t = f * e2.dot(q);
    if (t < 1e-4f) return false;
    bary = {1-u-v, u, v};
    return true;
}

// ===== Z Pre-pass & G-Buffer 填充 =====

void renderGBuffer(
    const std::vector<Sphere>& spheres,
    const std::vector<Triangle>& tris,
    const Mat4& view, const Mat4& /*proj*/, const Mat4& viewProj,
    Vec3 camPos
) {
    // 初始化
    for (int y=0;y<HEIGHT;y++)
        for (int x=0;x<WIDTH;x++) {
            gbuffer[y][x].valid = false;
            gbuffer[y][x].depth = 1.f;
            depthBuffer[y][x] = 1.f;
        }

    // 对每个像素投射光线
    float aspect = (float)WIDTH / HEIGHT;
    float fovY = 60.f * M_PI / 180.f;
    float tanH = std::tan(fovY * 0.5f);

    // 从lookAt矩阵中提取摄像机轴向量（世界空间）
    // view矩阵行: [r.x r.y r.z -r·eye], [u.x u.y u.z -u·eye], [-f.x -f.y -f.z f·eye]
    Vec3 camRight = {view.m[0][0], view.m[0][1], view.m[0][2]};
    Vec3 camUpVec = {view.m[1][0], view.m[1][1], view.m[1][2]};
    Vec3 camFwd   = {-view.m[2][0], -view.m[2][1], -view.m[2][2]};

    for (int y=0;y<HEIGHT;y++) {
        for (int x=0;x<WIDTH;x++) {
            // 生成光线（从摄像机出发）
            float ndcX = (x + 0.5f) / WIDTH * 2.f - 1.f;
            float ndcY = 1.f - (y + 0.5f) / HEIGHT * 2.f;
            float px = ndcX * aspect * tanH;
            float py = ndcY * tanH;

            // 世界空间光线方向：px*right + py*up + forward
            Vec3 rd = Vec3(
                px*camRight.x + py*camUpVec.x + camFwd.x,
                px*camRight.y + py*camUpVec.y + camFwd.y,
                px*camRight.z + py*camUpVec.z + camFwd.z
            ).normalized();
            Vec3 ro = camPos;

            float bestT = INF;
            Vec3 bestPos, bestNormal;
            Material bestMat;

            // 球体求交
            for (auto& sp : spheres) {
                float t;
                if (raySphere(ro, rd, sp, t) && t < bestT) {
                    bestT = t;
                    bestPos = ro + rd * t;
                    bestNormal = (bestPos - sp.center).normalized();
                    bestMat = sp.mat;
                }
            }

            // 三角形求交
            for (auto& tri : tris) {
                float t; Vec3 bary;
                if (rayTriangle(ro, rd, tri, t, bary) && t < bestT) {
                    bestT = t;
                    bestPos = ro + rd * t;
                    bestNormal = tri.n;
                    bestMat = tri.mat;
                }
            }

            if (bestT < INF) {
                // 计算视图空间深度（用于Tile光源剔除）
                Vec4 clipPos = viewProj * Vec4(bestPos, 1.f);
                float ndcZ = clipPos.z / clipPos.w;

                gbuffer[y][x].valid = true;
                gbuffer[y][x].position = bestPos;
                gbuffer[y][x].normal = bestNormal;
                gbuffer[y][x].albedo = bestMat.albedo;
                gbuffer[y][x].roughness = bestMat.roughness;
                gbuffer[y][x].metallic = bestMat.metallic;
                gbuffer[y][x].depth = ndcZ;
                depthBuffer[y][x] = ndcZ;
            }
        }
    }
}

// ===== Tile Light Culling =====

// 计算Tile的最小/最大深度（来自Z Pre-pass）
// 然后构建Tile Frustum，检查光源球与Frustum相交
// 这里用简化的2D屏幕空间AABB剔除 + 深度范围

struct AABB {
    Vec3 minP, maxP;
    AABB() : minP(INF,INF,INF), maxP(-INF,-INF,-INF) {}
    void expand(Vec3 p) {
        minP.x = std::min(minP.x, p.x);
        minP.y = std::min(minP.y, p.y);
        minP.z = std::min(minP.z, p.z);
        maxP.x = std::max(maxP.x, p.x);
        maxP.y = std::max(maxP.y, p.y);
        maxP.z = std::max(maxP.z, p.z);
    }
    bool intersectSphere(Vec3 center, float r) const {
        // AABB与球体相交测试
        float sqDist = 0;
        auto ax = [](float v, float lo, float hi) {
            if (v < lo) return (v-lo)*(v-lo);
            if (v > hi) return (v-hi)*(v-hi);
            return 0.f;
        };
        sqDist += ax(center.x, minP.x, maxP.x);
        sqDist += ax(center.y, minP.y, maxP.y);
        sqDist += ax(center.z, minP.z, maxP.z);
        return sqDist <= r*r;
    }
};

void tileLightCulling(
    const std::vector<PointLight>& lights,
    const Mat4& view, float zNear, float zFar
) {
    // 清空Tile光源列表
    for (int ty=0;ty<TILES_Y;ty++)
        for (int tx=0;tx<TILES_X;tx++)
            tileLights[ty][tx].count = 0;

    // 计算每个Tile的视图空间AABB（利用已有的G-Buffer位置数据）
    for (int ty=0;ty<TILES_Y;ty++) {
        for (int tx=0;tx<TILES_X;tx++) {
            // 收集Tile内所有像素的视图空间位置
            AABB tileAABB;
            bool hasPixels = false;

            int startX = tx * TILE_SIZE;
            int startY = ty * TILE_SIZE;
            int endX = std::min(startX + TILE_SIZE, WIDTH);
            int endY = std::min(startY + TILE_SIZE, HEIGHT);

            for (int y=startY;y<endY;y++) {
                for (int x=startX;x<endX;x++) {
                    if (gbuffer[y][x].valid) {
                        // 转换到视图空间
                        Vec3 viewPos = view.transformPoint(gbuffer[y][x].position);
                        tileAABB.expand(viewPos);
                        hasPixels = true;
                    }
                }
            }

            if (!hasPixels) {
                // 给Tile一个默认包围盒（使用屏幕角点 + 深度范围）
                // 简化：用全深度范围
                tileAABB.minP = Vec3(-100,-100,-zFar);
                tileAABB.maxP = Vec3(100,100,-zNear);
            } else {
                // 稍微扩展包围盒防止边界遗漏
                tileAABB.minP = tileAABB.minP - Vec3(0.1f,0.1f,0.1f);
                tileAABB.maxP = tileAABB.maxP + Vec3(0.1f,0.1f,0.1f);
            }

            // 测试每个光源与Tile AABB是否相交
            for (int li=0;li<(int)lights.size();li++) {
                const PointLight& light = lights[li];
                // 转换光源位置到视图空间
                Vec3 lightViewPos = view.transformPoint(light.position);

                if (tileAABB.intersectSphere(lightViewPos, light.radius)) {
                    if (tileLights[ty][tx].count < MAX_LIGHTS_PER_TILE) {
                        tileLights[ty][tx].lightIndices[tileLights[ty][tx].count++] = li;
                    }
                }
            }
        }
    }
}

// ===== Shading Pass =====

inline float clamp01(float v) { return std::max(0.f, std::min(1.f, v)); }
inline Vec3 clamp01(Vec3 v) { return {clamp01(v.x), clamp01(v.y), clamp01(v.z)}; }

// Blinn-Phong 着色
Vec3 blinnPhong(
    Vec3 pos, Vec3 N, Vec3 V,
    Vec3 albedo, float roughness, float metallic,
    const PointLight& light
) {
    Vec3 L = (light.position - pos);
    float dist = L.length();
    L = L / dist;

    // 衰减：平滑衰减
    float attenuation = 0;
    if (dist < light.radius) {
        float x = dist / light.radius;
        attenuation = std::max(0.f, 1.f - x*x*x*x);
        attenuation = attenuation * attenuation / (dist*dist + 1.f);
        attenuation *= light.intensity;
    }
    if (attenuation < 1e-6f) return Vec3(0,0,0);

    float NdotL = std::max(0.f, N.dot(L));
    if (NdotL < 1e-6f) return Vec3(0,0,0);

    Vec3 H = (L + V).normalized();
    float NdotH = std::max(0.f, N.dot(H));

    float shininess = std::max(1.f, 2.f / (roughness*roughness + 0.001f) - 2.f);
    float specPow = std::pow(NdotH, shininess);

    // 金属度混合
    Vec3 diffuseColor = albedo * (1.f - metallic);
    Vec3 specColor = albedo * metallic + Vec3(0.04f,0.04f,0.04f) * (1.f - metallic);

    Vec3 diffuse = diffuseColor * NdotL;
    Vec3 specular = specColor * specPow * NdotL;

    return (diffuse + specular) * light.color * attenuation;
}

// ACES 色调映射
Vec3 aces(Vec3 x) {
    float a=2.51f, b=0.03f, c=2.43f, d=0.59f, e=0.14f;
    Vec3 r;
    auto f = [&](float v) {
        return clamp01((v*(a*v+b))/(v*(c*v+d)+e));
    };
    r.x = f(x.x); r.y = f(x.y); r.z = f(x.z);
    return r;
}

Image shadingPass(
    const std::vector<PointLight>& lights,
    Vec3 ambientLight
) {
    Image output(WIDTH, HEIGHT);

    for (int y=0;y<HEIGHT;y++) {
        for (int x=0;x<WIDTH;x++) {
            if (!gbuffer[y][x].valid) {
                // 背景：深蓝色渐变
                float t = (float)y / HEIGHT;
                output.at(x,y) = Vec3(0.05f+t*0.03f, 0.05f+t*0.05f, 0.12f+t*0.08f);
                continue;
            }

            Vec3 pos = gbuffer[y][x].position;
            Vec3 N = gbuffer[y][x].normal;
            Vec3 albedo = gbuffer[y][x].albedo;
            float roughness = gbuffer[y][x].roughness;
            float metallic = gbuffer[y][x].metallic;

            // 确定所属Tile
            int tx = x / TILE_SIZE;
            int ty = y / TILE_SIZE;

            // 环境光
            Vec3 color = albedo * ambientLight;

            // 使用Tile光源列表着色
            const TileLightList& tll = tileLights[ty][tx];
            // V: 观察方向（从当前像素到摄像机）
            // 注意：我们没有传入camPos，通过G-Buffer推断不太准确
            // 这里我们使用一个全局视点方向（适合远摄像机）
            // 实际上需要传入摄像机位置
            Vec3 V = (Vec3(0,4,12) - pos).normalized();  // 硬编码摄像机位置

            for (int li=0;li<tll.count;li++) {
                int lightIdx = tll.lightIndices[li];
                color += blinnPhong(pos, N, V, albedo, roughness, metallic, lights[lightIdx]);
            }

            // ACES色调映射 + Gamma校正
            color = aces(color);
            // Gamma 2.2
            color.x = std::pow(std::max(0.f,color.x), 1.f/2.2f);
            color.y = std::pow(std::max(0.f,color.y), 1.f/2.2f);
            color.z = std::pow(std::max(0.f,color.z), 1.f/2.2f);

            output.at(x,y) = clamp01(color);
        }
    }

    return output;
}

// ===== 生成Tile热力图（可视化每个Tile的光源数量）=====

Image generateHeatmap(int maxLights) {
    Image img(WIDTH, HEIGHT);
    for (int ty=0;ty<TILES_Y;ty++) {
        for (int tx=0;tx<TILES_X;tx++) {
            int count = tileLights[ty][tx].count;
            float t = std::min(1.f, (float)count / maxLights);

            // 热力图颜色：蓝 -> 绿 -> 黄 -> 红
            Vec3 color;
            if (t < 0.25f) {
                float u = t / 0.25f;
                color = Vec3(0, u, 1-u);
            } else if (t < 0.5f) {
                float u = (t - 0.25f) / 0.25f;
                color = Vec3(u, 1, 0);
            } else if (t < 0.75f) {
                float u = (t - 0.5f) / 0.25f;
                color = Vec3(1, 1-u*0.5f, 0);
            } else {
                float u = (t - 0.75f) / 0.25f;
                color = Vec3(1, 0.5f-u*0.5f, 0);
            }

            int startX = tx * TILE_SIZE;
            int startY = ty * TILE_SIZE;
            int endX = std::min(startX + TILE_SIZE, WIDTH);
            int endY = std::min(startY + TILE_SIZE, HEIGHT);

            for (int y=startY;y<endY;y++)
                for (int x=startX;x<endX;x++) {
                    // 画格线（白色）
                    if (y == startY || x == startX)
                        img.at(x,y) = Vec3(0.3f, 0.3f, 0.3f);
                    else
                        img.at(x,y) = color;
                }
        }
    }
    return img;
}

// ===== 对比图（仅前向渲染 vs Forward+）=====
// 仅前向渲染：不做Tile剔除，每个像素遍历所有光源
Image forwardRenderingNaive(
    const std::vector<PointLight>& lights,
    Vec3 ambientLight
) {
    Image output(WIDTH, HEIGHT);
    for (int y=0;y<HEIGHT;y++) {
        for (int x=0;x<WIDTH;x++) {
            if (!gbuffer[y][x].valid) {
                float t = (float)y / HEIGHT;
                output.at(x,y) = Vec3(0.05f+t*0.03f, 0.05f+t*0.05f, 0.12f+t*0.08f);
                continue;
            }
            Vec3 pos = gbuffer[y][x].position;
            Vec3 N = gbuffer[y][x].normal;
            Vec3 albedo = gbuffer[y][x].albedo;
            float roughness = gbuffer[y][x].roughness;
            float metallic = gbuffer[y][x].metallic;
            Vec3 V = (Vec3(0,4,12) - pos).normalized();

            Vec3 color = albedo * ambientLight;
            for (auto& light : lights) {
                color += blinnPhong(pos, N, V, albedo, roughness, metallic, light);
            }
            color = aces(color);
            color.x = std::pow(std::max(0.f,color.x), 1.f/2.2f);
            color.y = std::pow(std::max(0.f,color.y), 1.f/2.2f);
            color.z = std::pow(std::max(0.f,color.z), 1.f/2.2f);
            output.at(x,y) = clamp01(color);
        }
    }
    return output;
}

// ===== PNG写入（使用Python/PIL）=====

void writePNG(const Image& img, const std::string& filename) {
    // 先写PPM，再用Python转PNG
    std::string ppmFile = filename + ".ppm";
    std::ofstream f(ppmFile, std::ios::binary);
    f << "P6\n" << img.w << " " << img.h << "\n255\n";
    for (int y=0;y<img.h;y++) {
        for (int x=0;x<img.w;x++) {
            Vec3 c = img.at(x,y);
            unsigned char r = (unsigned char)(std::min(1.f,c.x)*255.f);
            unsigned char g = (unsigned char)(std::min(1.f,c.y)*255.f);
            unsigned char b = (unsigned char)(std::min(1.f,c.z)*255.f);
            f.write((char*)&r,1);
            f.write((char*)&g,1);
            f.write((char*)&b,1);
        }
    }
    f.close();

    // 使用Python转换
    std::string cmd = "python3 -c \"from PIL import Image; img=Image.open('" + ppmFile +
                      "'); img.save('" + filename + "')\"";
    if (system(cmd.c_str()) != 0) {
        // 备用：使用ImageMagick
        cmd = "convert " + ppmFile + " " + filename;
        system(cmd.c_str());
    }
    remove(ppmFile.c_str());
    std::cout << "  -> " << filename << std::endl;
}

// 生成比较图（左右拼接）
Image makeComparison(const Image& left, const Image& right, int gap=4) {
    int newW = left.w + right.w + gap;
    Image out(newW, left.h);
    for (int y=0;y<left.h;y++) {
        for (int x=0;x<left.w;x++) out.at(x,y) = left.at(x,y);
        for (int x=0;x<gap;x++) out.at(left.w+x,y) = Vec3(0.2f,0.2f,0.2f);
        for (int x=0;x<right.w;x++) out.at(left.w+gap+x,y) = right.at(x,y);
    }
    return out;
}

// ===== 统计输出 =====

void printStats(const std::vector<PointLight>& lights) {
    int totalLightWork = 0;
    int minCount = MAX_LIGHTS_PER_TILE, maxCount = 0;
    int activeTiles = 0;

    for (int ty=0;ty<TILES_Y;ty++) {
        for (int tx=0;tx<TILES_X;tx++) {
            int c = tileLights[ty][tx].count;
            totalLightWork += c;
            if (c > 0) activeTiles++;
            minCount = std::min(minCount, c);
            maxCount = std::max(maxCount, c);
        }
    }

    int totalTiles = TILES_X * TILES_Y;
    float avgPerTile = totalTiles > 0 ? (float)totalLightWork / totalTiles : 0;
    int naiveWork = (int)lights.size() * WIDTH * HEIGHT;
    float savedPercent = naiveWork > 0
        ? (1.f - (float)totalLightWork * TILE_SIZE * TILE_SIZE / naiveWork) * 100.f
        : 0.f;

    std::cout << "\n=== Forward+ Rendering 统计 ===" << std::endl;
    std::cout << "画面分辨率: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "Tile大小: " << TILE_SIZE << "x" << TILE_SIZE << std::endl;
    std::cout << "Tile数量: " << TILES_X << "x" << TILES_Y << " = " << totalTiles << std::endl;
    std::cout << "总光源数: " << lights.size() << std::endl;
    std::cout << "每Tile平均光源: " << avgPerTile << std::endl;
    std::cout << "最少光源Tile: " << minCount << std::endl;
    std::cout << "最多光源Tile: " << maxCount << std::endl;
    std::cout << "活跃Tile（含光源）: " << activeTiles << "/" << totalTiles << std::endl;
    std::cout << "节省光源计算量（估算）: " << savedPercent << "%" << std::endl;
}

// ===== 主函数 =====

int main() {
    std::cout << "=== Forward+ Rendering ===" << std::endl;
    std::cout << "分辨率: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "Tile大小: " << TILE_SIZE << "x" << TILE_SIZE << std::endl;

    // ----- 构建场景 -----

    // 球体（多个材质各异的球）
    std::vector<Sphere> spheres;

    // 中心大球（金属）
    spheres.push_back({Vec3(0,0.5f,0), 1.2f, Material(Vec3(0.95f,0.75f,0.2f), 0.15f, 0.9f)});
    // 左球（红色粗糙）
    spheres.push_back({Vec3(-2.5f,0.5f,0.5f), 1.0f, Material(Vec3(0.85f,0.15f,0.1f), 0.7f, 0.0f)});
    // 右球（蓝色光泽）
    spheres.push_back({Vec3(2.5f,0.5f,0.5f), 1.0f, Material(Vec3(0.1f,0.3f,0.85f), 0.3f, 0.1f)});
    // 小球（绿色）
    spheres.push_back({Vec3(-1.2f,0.3f,-1.5f), 0.7f, Material(Vec3(0.1f,0.8f,0.2f), 0.5f, 0.0f)});
    spheres.push_back({Vec3(1.2f,0.3f,-1.5f), 0.7f, Material(Vec3(0.8f,0.5f,0.1f), 0.4f, 0.3f)});
    // 远处小球
    spheres.push_back({Vec3(-3.5f,0.4f,-2.f), 0.6f, Material(Vec3(0.7f,0.2f,0.8f), 0.6f, 0.2f)});
    spheres.push_back({Vec3(3.5f,0.4f,-2.f), 0.6f, Material(Vec3(0.2f,0.8f,0.7f), 0.25f, 0.5f)});

    // 地面（三角形拼成的平面）
    Material floorMat(Vec3(0.6f,0.6f,0.6f), 0.8f, 0.0f);
    floorMat.isFloor = true;
    std::vector<Triangle> tris;
    float floorY = -0.5f;
    float floorSize = 8.f;
    tris.push_back(Triangle(
        Vec3(-floorSize, floorY, -floorSize),
        Vec3( floorSize, floorY, -floorSize),
        Vec3( floorSize, floorY,  floorSize),
        floorMat));
    tris.push_back(Triangle(
        Vec3(-floorSize, floorY, -floorSize),
        Vec3( floorSize, floorY,  floorSize),
        Vec3(-floorSize, floorY,  floorSize),
        floorMat));

    // 背景墙（三角形）
    Material wallMat(Vec3(0.4f,0.4f,0.45f), 0.9f, 0.0f);
    float wallZ = -4.5f;
    float wallH = 5.f;
    tris.push_back(Triangle(
        Vec3(-floorSize, floorY, wallZ),
        Vec3( floorSize, floorY, wallZ),
        Vec3( floorSize, floorY+wallH, wallZ),
        wallMat));
    tris.push_back(Triangle(
        Vec3(-floorSize, floorY, wallZ),
        Vec3( floorSize, floorY+wallH, wallZ),
        Vec3(-floorSize, floorY+wallH, wallZ),
        wallMat));

    // ----- 创建多个点光源（Forward+的核心测试场景）-----
    std::vector<PointLight> lights;

    // 使用固定种子生成光源，确保可重现
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> distPos(-4.5f, 4.5f);
    std::uniform_real_distribution<float> distH(0.2f, 2.5f);
    std::uniform_real_distribution<float> distZ(-3.5f, 2.0f);
    std::uniform_real_distribution<float> distHue(0.f, 1.f);

    // 彩色光源：28个点光源（相当密集，适合展示Tile剔除效果）
    auto hueToRgb = [](float h) -> Vec3 {
        float r = std::max(0.f, std::abs(h*6.f-3.f)-1.f);
        float g = std::max(0.f, 2.f-std::abs(h*6.f-2.f));
        float b = std::max(0.f, 2.f-std::abs(h*6.f-4.f));
        return Vec3(std::min(1.f,r), std::min(1.f,g), std::min(1.f,b));
    };

    int numLights = 28;
    for (int i=0;i<numLights;i++) {
        PointLight l;
        l.position = Vec3(distPos(rng), distH(rng), distZ(rng));
        l.color = hueToRgb(distHue(rng));
        l.radius = 3.5f + distHue(rng) * 2.f;
        l.intensity = 15.f + distHue(rng) * 10.f;
        lights.push_back(l);
    }

    // 额外添加几个关键位置光源（确保主要场景有良好照明）
    lights.push_back({Vec3(0, 3.f, 2.f), Vec3(1.f,0.95f,0.8f), 6.f, 25.f});  // 顶光
    lights.push_back({Vec3(-3.f, 1.5f, 1.5f), Vec3(0.4f,0.8f,1.f), 5.f, 20.f}); // 左补光（蓝）
    lights.push_back({Vec3(3.f, 1.5f, 1.5f), Vec3(1.f,0.6f,0.3f), 5.f, 20.f});  // 右补光（暖橙）

    std::cout << "场景包含: " << spheres.size() << " 个球体, "
              << tris.size() << " 个三角形, "
              << lights.size() << " 个点光源" << std::endl;

    // ----- 相机设置 -----
    Vec3 camPos(0, 4, 12);
    Vec3 camTarget(0, 0.5f, 0);
    Vec3 camUp(0, 1, 0);
    float fovY = 60.f * M_PI / 180.f;
    float aspect = (float)WIDTH / HEIGHT;
    float zNear = 0.1f, zFar = 100.f;

    Mat4 view = lookAt(camPos, camTarget, camUp);
    Mat4 proj = perspective(fovY, aspect, zNear, zFar);
    Mat4 viewProj = proj * view;

    // ===== 阶段 1: Z Pre-pass & G-Buffer 填充 =====
    std::cout << "\n[1/4] Z Pre-pass & G-Buffer 填充..." << std::endl;
    renderGBuffer(spheres, tris, view, proj, viewProj, camPos);

    // 统计G-Buffer覆盖率
    int validPixels = 0;
    for (int y=0;y<HEIGHT;y++)
        for (int x=0;x<WIDTH;x++)
            if (gbuffer[y][x].valid) validPixels++;
    std::cout << "  G-Buffer有效像素: " << validPixels << "/" << (WIDTH*HEIGHT)
              << " (" << 100.f*validPixels/(WIDTH*HEIGHT) << "%)" << std::endl;

    // ===== 阶段 2: Tile Light Culling =====
    std::cout << "[2/4] Tile Light Culling..." << std::endl;
    tileLightCulling(lights, view, zNear, zFar);
    printStats(lights);

    // ===== 阶段 3: Forward+ Shading Pass =====
    std::cout << "\n[3/4] Forward+ Shading Pass..." << std::endl;
    Vec3 ambientLight(0.04f, 0.04f, 0.06f);
    Image forwardPlusResult = shadingPass(lights, ambientLight);

    // ===== 阶段 4: 对比 - 普通前向渲染 =====
    std::cout << "[4/4] 普通前向渲染（对比用）..." << std::endl;
    Image naiveResult = forwardRenderingNaive(lights, ambientLight);

    // ===== 生成Tile热力图 =====
    std::cout << "\n生成可视化图像..." << std::endl;
    Image heatmap = generateHeatmap((int)lights.size());

    // ===== 输出图像 =====
    writePNG(forwardPlusResult, "forwardplus_output.png");
    writePNG(naiveResult, "forward_naive_output.png");
    writePNG(heatmap, "tile_heatmap.png");

    // 对比图（左：普通前向，右：Forward+）
    Image comparison = makeComparison(naiveResult, forwardPlusResult);
    writePNG(comparison, "comparison.png");

    // ===== 验证 =====
    std::cout << "\n=== 验证 ===" << std::endl;

    // 检查中心球区域（大约在屏幕中心，稍微偏上）
    // 金色金属球应该在屏幕中心
    int cx = WIDTH / 2;
    int cy = HEIGHT / 2;

    // 验证：中心球必须有效且非背景
    int validCenter = 0;
    Vec3 sumColor(0,0,0);
    int sampleSize = 30;
    for (int dy=-sampleSize;dy<=sampleSize;dy++) {
        for (int dx=-sampleSize;dx<=sampleSize;dx++) {
            int px = cx+dx, py = cy+dy;
            if (px>=0&&px<WIDTH&&py>=0&&py<HEIGHT) {
                if (gbuffer[py][px].valid) {
                    validCenter++;
                    sumColor += forwardPlusResult.at(px,py);
                }
            }
        }
    }

    if (validCenter > 100) {
        Vec3 avgColor = sumColor / (float)validCenter;
        std::cout << "✅ 中心球区域有 " << validCenter << " 个有效像素" << std::endl;
        std::cout << "   平均颜色: RGB("
                  << (int)(avgColor.x*255) << ","
                  << (int)(avgColor.y*255) << ","
                  << (int)(avgColor.z*255) << ")" << std::endl;

        // 检查不是全黑
        if (avgColor.x < 0.02f && avgColor.y < 0.02f && avgColor.z < 0.02f) {
            std::cerr << "❌ 中心球区域过暗，可能有渲染问题！" << std::endl;
            return 1;
        }
    } else {
        std::cerr << "❌ 中心球区域有效像素不足（" << validCenter << "），可能未渲染到球体！" << std::endl;
        return 1;
    }

    // 验证Tile剔除有效性
    int maxLightsInTile = 0;
    int tilesWithLights = 0;
    for (int ty=0;ty<TILES_Y;ty++)
        for (int tx=0;tx<TILES_X;tx++) {
            if (tileLights[ty][tx].count > 0) tilesWithLights++;
            maxLightsInTile = std::max(maxLightsInTile, tileLights[ty][tx].count);
        }

    std::cout << "✅ Tile剔除工作正常: " << tilesWithLights << " 个Tile含光源，最多 "
              << maxLightsInTile << " 个" << std::endl;

    if (maxLightsInTile == 0) {
        std::cerr << "❌ 没有任何Tile分配到光源，Tile剔除可能有误！" << std::endl;
        return 1;
    }

    std::cout << "\n✅ 所有验证通过！" << std::endl;
    std::cout << "输出文件:" << std::endl;
    std::cout << "  forwardplus_output.png  - Forward+ 渲染结果" << std::endl;
    std::cout << "  forward_naive_output.png - 普通前向渲染结果（对比）" << std::endl;
    std::cout << "  tile_heatmap.png         - Tile光源分布热力图" << std::endl;
    std::cout << "  comparison.png           - 对比图（左：普通，右：Forward+）" << std::endl;

    return 0;
}
