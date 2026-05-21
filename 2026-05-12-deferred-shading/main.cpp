/**
 * Deferred Shading Multi-Light Renderer
 * 2026-05-12 Daily Coding Practice
 *
 * 延迟渲染管线：
 * Pass 1 (Geometry Pass): 渲染场景几何，写入 G-Buffer
 *   - gAlbedo: 漫反射颜色 + 粗糙度
 *   - gNormal: 世界空间法线
 *   - gPosition: 世界空间位置
 *   - gDepth: 深度
 * Pass 2 (Lighting Pass): 逐像素读取 G-Buffer，执行多光源光照计算
 *
 * 场景：多个球体 + 平面，16个彩色点光源
 */

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include <string>
#include <random>

// ============================================================
// 数学库
// ============================================================
struct Vec2 {
    float x, y;
    Vec2(float x=0,float y=0):x(x),y(y){}
};

struct Vec3 {
    float x, y, z;
    Vec3(float x=0,float y=0,float z=0):x(x),y(y),z(z){}
    Vec3 operator+(const Vec3& o) const { return {x+o.x,y+o.y,z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x,y-o.y,z-o.z}; }
    Vec3 operator*(float t) const { return {x*t,y*t,z*t}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x,y*o.y,z*o.z}; }
    Vec3 operator/(float t) const { return {x/t,y/t,z/t}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3& operator+=(const Vec3& o) { x+=o.x;y+=o.y;z+=o.z; return *this; }
    float dot(const Vec3& o) const { return x*o.x+y*o.y+z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z-z*o.y, z*o.x-x*o.z, x*o.y-y*o.x};
    }
    float len() const { return sqrtf(x*x+y*y+z*z); }
    Vec3 norm() const { float l=len(); return l>1e-8f?(*this)/l:Vec3(0,1,0); }
    float& operator[](int i) { return i==0?x:i==1?y:z; }
    float operator[](int i) const { return i==0?x:i==1?y:z; }
};

inline Vec3 operator*(float t, const Vec3& v) { return v*t; }
inline Vec3 reflect(const Vec3& I, const Vec3& N) {
    return I - N * (2.0f * N.dot(I));
}

struct Vec4 {
    float x, y, z, w;
    Vec4(float x=0,float y=0,float z=0,float w=1):x(x),y(y),z(z),w(w){}
    Vec4(const Vec3& v, float w=1):x(v.x),y(v.y),z(v.z),w(w){}
    Vec3 xyz() const { return {x,y,z}; }
};

struct Mat4 {
    float m[4][4];
    Mat4() { memset(m,0,sizeof(m)); }
    static Mat4 identity() {
        Mat4 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=r.m[3][3]=1; return r;
    }
    Vec4 operator*(const Vec4& v) const {
        Vec4 r;
        r.x = m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w;
        r.y = m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w;
        r.z = m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w;
        r.w = m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w;
        return r;
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r;
        for(int i=0;i<4;i++)
            for(int j=0;j<4;j++)
                for(int k=0;k<4;k++)
                    r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }
};

Mat4 perspective(float fovY, float aspect, float near, float far) {
    Mat4 r;
    float f = 1.0f / tanf(fovY * 0.5f);
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (far + near) / (near - far);
    r.m[2][3] = (2.0f * far * near) / (near - far);
    r.m[3][2] = -1.0f;
    r.m[3][3] = 0.0f;
    return r;
}

Mat4 lookAt(Vec3 eye, Vec3 center, Vec3 up) {
    Vec3 f = (center - eye).norm();
    Vec3 r = f.cross(up).norm();
    Vec3 u = r.cross(f);
    Mat4 mat = Mat4::identity();
    mat.m[0][0]=r.x; mat.m[0][1]=r.y; mat.m[0][2]=r.z; mat.m[0][3]=-r.dot(eye);
    mat.m[1][0]=u.x; mat.m[1][1]=u.y; mat.m[1][2]=u.z; mat.m[1][3]=-u.dot(eye);
    mat.m[2][0]=-f.x;mat.m[2][1]=-f.y;mat.m[2][2]=-f.z;mat.m[2][3]=f.dot(eye);
    mat.m[3][3]=1.0f;
    return mat;
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

// ============================================================
// G-Buffer & 帧缓冲
// ============================================================
const int WIDTH  = 800;
const int HEIGHT = 600;

struct GBuffer {
    // 每个像素存储：albedo(rgb) + normal(xyz) + worldPos(xyz) + depth
    Vec3 albedo[WIDTH * HEIGHT];
    Vec3 normal[WIDTH * HEIGHT];   // world-space normal
    Vec3 worldPos[WIDTH * HEIGHT]; // world-space position
    float depth[WIDTH * HEIGHT];
    bool valid[WIDTH * HEIGHT];    // 是否有有效几何

    void clear() {
        for(int i = 0; i < WIDTH * HEIGHT; i++) {
            albedo[i] = Vec3(0.1f, 0.1f, 0.15f); // 背景色
            normal[i] = Vec3(0,1,0);
            worldPos[i] = Vec3(0,0,0);
            depth[i] = 1e9f;
            valid[i] = false;
        }
    }
};

struct Framebuffer {
    Vec3 color[WIDTH * HEIGHT];
    void clear(Vec3 bg = Vec3(0,0,0)) {
        for(int i = 0; i < WIDTH * HEIGHT; i++) color[i] = bg;
    }
};

// ============================================================
// 点光源
// ============================================================
struct PointLight {
    Vec3 position;
    Vec3 color;
    float intensity;
    float radius;      // 影响半径（衰减用）
};

// ============================================================
// 场景几何（球 + 平面用三角形网格）
// ============================================================
struct Vertex {
    Vec3 pos;
    Vec3 normal;
    Vec3 albedo;
};

struct Triangle {
    Vertex v[3];
};

// 生成球体三角形网格
std::vector<Triangle> makeSphere(Vec3 center, float radius, Vec3 albedo,
                                  int lat=20, int lon=30) {
    std::vector<Triangle> tris;
    for(int i = 0; i < lat; i++) {
        float t0 = (float)i     / lat * M_PI;
        float t1 = (float)(i+1) / lat * M_PI;
        for(int j = 0; j < lon; j++) {
            float p0 = (float)j     / lon * 2.0f * M_PI;
            float p1 = (float)(j+1) / lon * 2.0f * M_PI;

            auto sph = [&](float t, float p) -> Vec3 {
                return {sinf(t)*cosf(p), cosf(t), sinf(t)*sinf(p)};
            };

            Vec3 n00 = sph(t0, p0), n10 = sph(t1, p0);
            Vec3 n01 = sph(t0, p1), n11 = sph(t1, p1);

            Vec3 p00 = center + n00 * radius;
            Vec3 p10 = center + n10 * radius;
            Vec3 p01 = center + n01 * radius;
            Vec3 p11 = center + n11 * radius;

            // 两个三角形
            Triangle t1, t2;
            t1.v[0] = {p00, n00, albedo};
            t1.v[1] = {p10, n10, albedo};
            t1.v[2] = {p11, n11, albedo};
            t2.v[0] = {p00, n00, albedo};
            t2.v[1] = {p11, n11, albedo};
            t2.v[2] = {p01, n01, albedo};
            tris.push_back(t1);
            tris.push_back(t2);
        }
    }
    return tris;
}

// 生成平面（xz平面上的网格）
std::vector<Triangle> makePlane(float y, float size, Vec3 albedo) {
    std::vector<Triangle> tris;
    int N = 4;
    float step = size * 2.0f / N;
    for(int i = 0; i < N; i++) {
        for(int j = 0; j < N; j++) {
            float x0 = -size + i * step;
            float z0 = -size + j * step;
            float x1 = x0 + step;
            float z1 = z0 + step;
            Vec3 n(0,1,0);
            Triangle t1, t2;
            t1.v[0] = {{x0,y,z0}, n, albedo};
            t1.v[1] = {{x1,y,z0}, n, albedo};
            t1.v[2] = {{x1,y,z1}, n, albedo};
            t2.v[0] = {{x0,y,z0}, n, albedo};
            t2.v[1] = {{x1,y,z1}, n, albedo};
            t2.v[2] = {{x0,y,z1}, n, albedo};
            tris.push_back(t1);
            tris.push_back(t2);
        }
    }
    return tris;
}

// ============================================================
// 软光栅化：将三角形渲染到 G-Buffer
// ============================================================
inline float edgeFunc(Vec2 a, Vec2 b, Vec2 p) {
    return (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
}

void rasterizeToGBuffer(const std::vector<Triangle>& scene,
                        const Mat4& view, const Mat4& proj,
                        GBuffer& gbuf) {

    for(const auto& tri : scene) {
        // 变换到裁剪空间
        Vec4 clip[3];
        Vec3 worldPos[3];
        Vec3 worldNormal[3];
        Vec3 albedo3 = tri.v[0].albedo;

        for(int k = 0; k < 3; k++) {
            worldPos[k] = tri.v[k].pos;
            worldNormal[k] = tri.v[k].normal;
            Vec4 vw(tri.v[k].pos, 1.0f);
            Vec4 vc = view * vw;
            clip[k] = proj * vc;
        }

        // 透视除法 → NDC
        Vec3 ndc[3];
        float invW[3];
        for(int k = 0; k < 3; k++) {
            if(fabsf(clip[k].w) < 1e-8f) goto next_tri;
            invW[k] = 1.0f / clip[k].w;
            ndc[k] = clip[k].xyz() * invW[k];
        }

        {
            // NDC → 屏幕坐标
            Vec2 screen[3];
            for(int k = 0; k < 3; k++) {
                screen[k].x = (ndc[k].x * 0.5f + 0.5f) * WIDTH;
                screen[k].y = (1.0f - (ndc[k].y * 0.5f + 0.5f)) * HEIGHT; // Y翻转
            }

            // 背面剔除
            float area = edgeFunc(screen[0], screen[1], screen[2]);
            if(area <= 0) goto next_tri;

            // Bounding box
            int minX = (int)std::max(0.0f, std::min({screen[0].x, screen[1].x, screen[2].x}));
            int maxX = (int)std::min((float)WIDTH  - 1, std::max({screen[0].x, screen[1].x, screen[2].x}));
            int minY = (int)std::max(0.0f, std::min({screen[0].y, screen[1].y, screen[2].y}));
            int maxY = (int)std::min((float)HEIGHT - 1, std::max({screen[0].y, screen[1].y, screen[2].y}));

            // 光栅化
            for(int py = minY; py <= maxY; py++) {
                for(int px = minX; px <= maxX; px++) {
                    Vec2 p((float)px + 0.5f, (float)py + 0.5f);
                    float w0 = edgeFunc(screen[1], screen[2], p);
                    float w1 = edgeFunc(screen[2], screen[0], p);
                    float w2 = edgeFunc(screen[0], screen[1], p);
                    if(w0 < 0 || w1 < 0 || w2 < 0) continue;

                    float b0 = w0 / area;
                    float b1 = w1 / area;
                    float b2 = w2 / area;

                    // 透视校正深度
                    float depth = b0 * ndc[0].z + b1 * ndc[1].z + b2 * ndc[2].z;

                    int idx = py * WIDTH + px;
                    if(depth >= gbuf.depth[idx]) continue;
                    gbuf.depth[idx] = depth;

                    // 透视校正插值
                    float pcb0 = b0 * invW[0];
                    float pcb1 = b1 * invW[1];
                    float pcb2 = b2 * invW[2];
                    float pcSum = pcb0 + pcb1 + pcb2;
                    if(pcSum < 1e-8f) continue;
                    pcb0 /= pcSum; pcb1 /= pcSum; pcb2 /= pcSum;

                    // 插值世界坐标和法线
                    Vec3 wp = worldPos[0]*pcb0 + worldPos[1]*pcb1 + worldPos[2]*pcb2;
                    Vec3 wn = (worldNormal[0]*pcb0 + worldNormal[1]*pcb1 + worldNormal[2]*pcb2).norm();

                    // 写入 G-Buffer
                    gbuf.albedo[idx]   = albedo3;
                    gbuf.normal[idx]   = wn;
                    gbuf.worldPos[idx] = wp;
                    gbuf.valid[idx]    = true;
                }
            }
        }
        next_tri:;
    }
}

// ============================================================
// Lighting Pass：延迟光照计算
// ============================================================
Vec3 clampVec(Vec3 v, float lo=0.0f, float hi=1.0f) {
    return {std::max(lo, std::min(hi, v.x)),
            std::max(lo, std::min(hi, v.y)),
            std::max(lo, std::min(hi, v.z))};
}

void lightingPass(const GBuffer& gbuf,
                  const std::vector<PointLight>& lights,
                  const Vec3& cameraPos,
                  Framebuffer& fb) {
    const Vec3 ambient(0.03f, 0.03f, 0.05f);

    for(int py = 0; py < HEIGHT; py++) {
        for(int px = 0; px < WIDTH; px++) {
            int idx = py * WIDTH + px;

            if(!gbuf.valid[idx]) {
                // 背景：渐变天空
                float t = (float)py / HEIGHT;
                Vec3 bg = Vec3(0.05f, 0.05f, 0.15f) * (1.0f - t) + Vec3(0.01f, 0.01f, 0.05f) * t;
                fb.color[idx] = bg;
                continue;
            }

            Vec3 albedo   = gbuf.albedo[idx];
            Vec3 N        = gbuf.normal[idx];
            Vec3 fragPos  = gbuf.worldPos[idx];
            Vec3 V        = (cameraPos - fragPos).norm();

            // 环境光
            Vec3 result = ambient * albedo;

            // 逐光源计算（延迟着色的核心：可扩展到任意多光源）
            for(const auto& light : lights) {
                Vec3 L = light.position - fragPos;
                float dist = L.len();

                // 光源影响范围剔除
                if(dist > light.radius) continue;

                L = L / dist;

                // 距离衰减（平方反比 + 半径截断）
                float attenuation = 1.0f / (1.0f + 0.09f * dist + 0.032f * dist * dist);
                float edgeFade = 1.0f - (dist / light.radius);
                edgeFade = edgeFade * edgeFade; // 平滑截断
                attenuation *= edgeFade;

                // Diffuse (Lambertian)
                float NdotL = std::max(0.0f, N.dot(L));
                Vec3 diffuse = albedo * light.color * (light.intensity * NdotL * attenuation);

                // Specular (Blinn-Phong)
                Vec3 H = (L + V).norm();
                float NdotH = std::max(0.0f, N.dot(H));
                float spec = powf(NdotH, 64.0f);
                Vec3 specular = light.color * (light.intensity * spec * attenuation * 0.3f);

                result += diffuse + specular;
            }

            // HDR 色调映射（Reinhard，逐分量）
            result.x = result.x / (result.x + 1.0f);
            result.y = result.y / (result.y + 1.0f);
            result.z = result.z / (result.z + 1.0f);

            // Gamma 校正 2.2
            result.x = powf(std::max(0.0f, result.x), 1.0f/2.2f);
            result.y = powf(std::max(0.0f, result.y), 1.0f/2.2f);
            result.z = powf(std::max(0.0f, result.z), 1.0f/2.2f);

            fb.color[idx] = clampVec(result);
        }
    }
}

// ============================================================
// PNG 写入（简单 PPM → PNG 管道，使用 ImageMagick 转换）
// ============================================================
void writePPM(const char* filename, const Framebuffer& fb) {
    FILE* f = fopen(filename, "wb");
    fprintf(f, "P6\n%d %d\n255\n", WIDTH, HEIGHT);
    for(int i = 0; i < WIDTH * HEIGHT; i++) {
        uint8_t r = (uint8_t)(fb.color[i].x * 255.0f + 0.5f);
        uint8_t g = (uint8_t)(fb.color[i].y * 255.0f + 0.5f);
        uint8_t b = (uint8_t)(fb.color[i].z * 255.0f + 0.5f);
        fwrite(&r, 1, 1, f);
        fwrite(&g, 1, 1, f);
        fwrite(&b, 1, 1, f);
    }
    fclose(f);
}

// ============================================================
// Main
// ============================================================
int main() {
    printf("=== Deferred Shading Multi-Light Renderer ===\n");
    printf("Resolution: %dx%d\n", WIDTH, HEIGHT);

    // ---- 场景 ----
    std::vector<Triangle> scene;

    // 地面平面
    auto plane = makePlane(-1.0f, 6.0f, Vec3(0.4f, 0.4f, 0.45f));
    scene.insert(scene.end(), plane.begin(), plane.end());

    // 各种球体（不同颜色，展示多光源彩色照明效果）
    struct SphereDesc { Vec3 c; float r; Vec3 albedo; };
    std::vector<SphereDesc> spheres = {
        // 中心大球
        {{0.0f,  0.2f,  0.0f}, 1.2f, {0.8f, 0.8f, 0.85f}},  // 银白色
        // 左侧球
        {{-3.0f, 0.0f, -1.0f}, 1.0f, {0.9f, 0.2f, 0.2f}},   // 红色
        // 右侧球
        {{ 3.0f, 0.0f, -1.0f}, 1.0f, {0.2f, 0.5f, 0.9f}},   // 蓝色
        // 后排
        {{-1.5f, 0.3f, -3.0f}, 0.8f, {0.2f, 0.9f, 0.3f}},   // 绿色
        {{ 1.5f, 0.3f, -3.0f}, 0.8f, {0.9f, 0.7f, 0.1f}},   // 金色
        // 小球群
        {{ 0.0f, -0.2f, -4.0f}, 0.5f, {0.8f, 0.3f, 0.9f}},  // 紫色
        {{-4.5f, 0.5f,  1.0f}, 1.3f, {0.3f, 0.85f, 0.8f}},  // 青色
        {{ 4.5f, 0.5f,  1.0f}, 1.3f, {0.95f,0.5f, 0.2f}},   // 橙色
    };

    for(const auto& s : spheres) {
        auto sph = makeSphere(s.c, s.r, s.albedo, 24, 36);
        scene.insert(scene.end(), sph.begin(), sph.end());
    }

    printf("Scene triangles: %zu\n", scene.size());

    // ---- 相机 ----
    Vec3 cameraPos(0.0f, 3.5f, 8.0f);
    Vec3 cameraTarget(0.0f, 0.0f, -1.0f);
    Vec3 cameraUp(0.0f, 1.0f, 0.0f);

    Mat4 view = lookAt(cameraPos, cameraTarget, cameraUp);
    Mat4 proj = perspective(45.0f * M_PI / 180.0f, (float)WIDTH/HEIGHT, 0.1f, 100.0f);

    // ---- 16 个彩色点光源（环绕场景） ----
    std::vector<PointLight> lights;
    const int NUM_LIGHTS = 16;

    // 彩色光源调色板
    std::vector<Vec3> lightColors = {
        {1.0f, 0.3f, 0.3f},  // 红
        {0.3f, 1.0f, 0.3f},  // 绿
        {0.3f, 0.3f, 1.0f},  // 蓝
        {1.0f, 1.0f, 0.3f},  // 黄
        {1.0f, 0.3f, 1.0f},  // 品红
        {0.3f, 1.0f, 1.0f},  // 青
        {1.0f, 0.6f, 0.2f},  // 橙
        {0.6f, 0.3f, 1.0f},  // 紫
        {0.9f, 0.9f, 0.9f},  // 白
        {1.0f, 0.4f, 0.1f},  // 暖橙
        {0.2f, 0.8f, 0.6f},  // 青绿
        {0.9f, 0.2f, 0.5f},  // 玫红
        {0.4f, 0.9f, 0.2f},  // 黄绿
        {0.2f, 0.4f, 0.9f},  // 天蓝
        {0.8f, 0.8f, 0.2f},  // 柠黄
        {0.5f, 0.2f, 0.8f},  // 深紫
    };

    for(int i = 0; i < NUM_LIGHTS; i++) {
        float angle = (float)i / NUM_LIGHTS * 2.0f * M_PI;
        float ringR = 4.5f;
        float height = 1.5f + sinf(angle * 2.0f) * 0.8f; // 高度变化

        PointLight pl;
        pl.position = {ringR * cosf(angle), height, ringR * sinf(angle) - 1.5f};
        pl.color = lightColors[i % lightColors.size()];
        pl.intensity = 3.5f;
        pl.radius = 8.0f;
        lights.push_back(pl);
    }

    // 额外：顶部主光源（接近白色）
    lights.push_back({{0.0f, 5.0f, 1.0f}, {1.0f, 0.95f, 0.9f}, 5.0f, 12.0f});

    printf("Lights count: %zu\n", lights.size());

    // ---- Pass 1: Geometry Pass → G-Buffer ----
    printf("Pass 1: Geometry Pass...\n");
    static GBuffer gbuf;
    gbuf.clear();
    rasterizeToGBuffer(scene, view, proj, gbuf);

    // 统计有效像素
    int validPixels = 0;
    for(int i = 0; i < WIDTH * HEIGHT; i++) if(gbuf.valid[i]) validPixels++;
    printf("  Valid G-Buffer pixels: %d / %d (%.1f%%)\n",
           validPixels, WIDTH*HEIGHT, 100.0f*validPixels/(WIDTH*HEIGHT));

    // ---- Pass 2: Lighting Pass ----
    printf("Pass 2: Lighting Pass...\n");
    static Framebuffer fb;
    fb.clear();
    lightingPass(gbuf, lights, cameraPos, fb);

    // ---- 输出 ----
    const char* ppmFile = "deferred_shading_output.ppm";
    const char* pngFile = "deferred_shading_output.png";

    printf("Writing output...\n");
    writePPM(ppmFile, fb);

    // 使用 ImageMagick 转 PNG
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "convert %s %s 2>/dev/null && rm -f %s", ppmFile, pngFile, ppmFile);
    if(system(cmd) != 0) {
        // 如果没有 ImageMagick，重命名
        snprintf(cmd, sizeof(cmd), "cp %s %s.ppm && cp %s deferred_shading_output.png", ppmFile, ppmFile, ppmFile);
        system(cmd);
        printf("⚠ ImageMagick not found, output as PPM\n");
    }

    printf("\n✅ Rendering complete!\n");
    printf("Output: %s\n", pngFile);

    // ---- 统计 ----
    float sum = 0.0f;
    float minV = 1e9f, maxV = -1e9f;
    for(int i = 0; i < WIDTH * HEIGHT; i++) {
        float v = (fb.color[i].x + fb.color[i].y + fb.color[i].z) / 3.0f;
        sum += v;
        minV = std::min(minV, v);
        maxV = std::max(maxV, v);
    }
    float mean = sum / (WIDTH * HEIGHT);
    printf("Pixel stats: mean=%.3f  min=%.3f  max=%.3f\n", mean, minV, maxV);

    // ---- G-Buffer 可视化（法线图）----
    {
        Framebuffer normalViz;
        for(int i = 0; i < WIDTH * HEIGHT; i++) {
            if(gbuf.valid[i]) {
                Vec3 n = gbuf.normal[i];
                normalViz.color[i] = {n.x*0.5f+0.5f, n.y*0.5f+0.5f, n.z*0.5f+0.5f};
            } else {
                normalViz.color[i] = Vec3(0.1f, 0.1f, 0.1f);
            }
        }
        writePPM("gbuffer_normal.ppm", normalViz);
        system("convert gbuffer_normal.ppm gbuffer_normal.png 2>/dev/null && rm -f gbuffer_normal.ppm");
    }

    return 0;
}
