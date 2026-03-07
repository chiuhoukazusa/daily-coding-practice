/**
 * Cloth Simulation 布料模拟
 * 
 * 技术要点：
 * - 质点弹簧系统 (Mass-Spring System)
 * - Verlet 积分法（位置更新）
 * - 结构弹簧 / 剪切弹簧 / 弯曲弹簧
 * - 重力、风力、碰撞检测（球体碰撞）
 * - 软体渲染：Phong着色 + 双面法线
 * - 多帧序列输出（静态pose + 垂落动画帧）
 * 
 * 输出：
 *   cloth_output.png      - 主渲染图（布料垂落最终状态）
 *   cloth_sequence.png    - 多帧对比图（0/15/30/45帧）
 * 
 * Date: 2026-03-08
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cassert>
#include <string>

// ─── 数学类型 ────────────────────────────────────────────────────────────────

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const      { return {x*s,   y*s,   z*s};   }
    Vec3 operator/(double s) const      { return {x/s,   y/s,   z/s};   }
    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }

    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x};
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    Vec3 normalized() const {
        double l = length();
        if (l < 1e-12) return {0,0,0};
        return *this / l;
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

// ─── 图像输出（stb_image_write 自实现最小版，写 PNG 用 PPM 转，或直接写 PPM→用 convert 转 PNG）
// 这里我们自己写一个最简 PPM 写入，然后用 convert 转 PNG

struct Image {
    int w, h;
    std::vector<uint8_t> pixels; // RGB

    Image(int w, int h) : w(w), h(h), pixels(w*h*3, 0) {}

    void setPixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (x < 0 || x >= w || y < 0 || y >= h) return;
        int idx = (y * w + x) * 3;
        pixels[idx+0] = r;
        pixels[idx+1] = g;
        pixels[idx+2] = b;
    }

    void setPixelf(int x, int y, double r, double g, double b) {
        auto clamp01 = [](double v){ return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); };
        setPixel(x, y,
            (uint8_t)(clamp01(r)*255.0 + 0.5),
            (uint8_t)(clamp01(g)*255.0 + 0.5),
            (uint8_t)(clamp01(b)*255.0 + 0.5));
    }

    bool savePPM(const std::string& path) const {
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) return false;
        fprintf(f, "P6\n%d %d\n255\n", w, h);
        fwrite(pixels.data(), 1, pixels.size(), f);
        fclose(f);
        return true;
    }
};

// ─── Z-Buffer 辅助 ───────────────────────────────────────────────────────────

struct ZBuffer {
    int w, h;
    std::vector<double> buf;
    ZBuffer(int w, int h) : w(w), h(h), buf(w*h, 1e18) {}
    double& at(int x, int y) { return buf[y*w+x]; }
    bool test(int x, int y, double z) {
        if (x < 0 || x >= w || y < 0 || y >= h) return false;
        if (z < at(x, y)) { at(x, y) = z; return true; }
        return false;
    }
};

// ─── 布料模拟 ────────────────────────────────────────────────────────────────

struct Particle {
    Vec3 pos;
    Vec3 prev_pos;  // Verlet 积分
    Vec3 force;
    double mass;
    bool pinned;    // 固定点（上边缘）

    Particle() : mass(1.0), pinned(false) {}
};

struct Spring {
    int a, b;
    double rest_length;
    double stiffness;
};

struct ClothSim {
    int rows, cols;
    std::vector<Particle> particles;
    std::vector<Spring> springs;

    // 球体碰撞体
    Vec3 sphere_center;
    double sphere_radius;

    // 物理参数
    double gravity        = -9.8;
    double damping        = 0.99;  // Verlet 阻尼
    double struct_k       = 800.0; // 结构弹簧刚度
    double shear_k        = 400.0; // 剪切弹簧刚度
    double bend_k         = 200.0; // 弯曲弹簧刚度

    // 风力
    Vec3 wind = {2.0, 0.0, -1.0};

    ClothSim(int rows, int cols, double cloth_size)
        : rows(rows), cols(cols),
          sphere_center(0.0, -1.2, 0.0),
          sphere_radius(0.6)
    {
        particles.resize(rows * cols);
        double dx = cloth_size / (cols - 1);
        double dy = cloth_size / (rows - 1);

        // 初始化：布料水平展开，顶部在 y=0 平面，z=0
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = r * cols + c;
                Particle& p = particles[idx];
                p.pos = {
                    (c - cols/2.0) * dx,
                    0.0,
                    r * dy - cloth_size * 0.5
                };
                p.prev_pos = p.pos;
                p.mass = 1.0;
                // 顶部一行固定（但保留一些端点）
                p.pinned = (r == 0);
            }
        }

        auto addSpring = [&](int a, int b, double k) {
            Spring s;
            s.a = a; s.b = b;
            s.stiffness = k;
            s.rest_length = (particles[a].pos - particles[b].pos).length();
            springs.push_back(s);
        };

        // 结构弹簧（横向和纵向邻居）
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = r * cols + c;
                if (c + 1 < cols) addSpring(idx, idx + 1, struct_k);
                if (r + 1 < rows) addSpring(idx, idx + cols, struct_k);
            }
        }

        // 剪切弹簧（对角线）
        for (int r = 0; r < rows - 1; r++) {
            for (int c = 0; c < cols - 1; c++) {
                int idx = r * cols + c;
                addSpring(idx, idx + cols + 1, shear_k);
                addSpring(idx + 1, idx + cols, shear_k);
            }
        }

        // 弯曲弹簧（隔一个点）
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols - 2; c++) {
                int idx = r * cols + c;
                addSpring(idx, idx + 2, bend_k);
            }
        }
        for (int r = 0; r < rows - 2; r++) {
            for (int c = 0; c < cols; c++) {
                int idx = r * cols + c;
                addSpring(idx, idx + cols * 2, bend_k);
            }
        }
    }

    int index(int r, int c) const { return r * cols + c; }

    void step(double dt, bool apply_wind) {
        // 清力
        for (auto& p : particles) p.force = {0, 0, 0};

        // 重力
        for (auto& p : particles) {
            if (!p.pinned) p.force += {0, gravity * p.mass, 0};
        }

        // 风力
        if (apply_wind) {
            for (auto& p : particles) {
                if (!p.pinned) p.force += wind * 0.1;
            }
        }

        // 弹簧力
        for (auto& s : springs) {
            Vec3 diff = particles[s.b].pos - particles[s.a].pos;
            double len = diff.length();
            if (len < 1e-12) continue;
            double extension = len - s.rest_length;
            Vec3 force_dir = diff.normalized();
            Vec3 f = force_dir * (s.stiffness * extension);
            if (!particles[s.a].pinned) particles[s.a].force += f;
            if (!particles[s.b].pinned) particles[s.b].force -= f;
        }

        // Verlet 积分
        for (auto& p : particles) {
            if (p.pinned) continue;
            Vec3 acceleration = p.force / p.mass;
            Vec3 vel = (p.pos - p.prev_pos) * damping;
            Vec3 new_pos = p.pos + vel + acceleration * (dt * dt);
            p.prev_pos = p.pos;
            p.pos = new_pos;
        }

        // 球体碰撞
        for (auto& p : particles) {
            if (p.pinned) continue;
            Vec3 diff = p.pos - sphere_center;
            double dist = diff.length();
            if (dist < sphere_radius + 0.02) {
                // 推出球面
                p.pos = sphere_center + diff.normalized() * (sphere_radius + 0.02);
            }
        }

        // 地面碰撞
        for (auto& p : particles) {
            if (p.pinned) continue;
            if (p.pos.y < -2.5) {
                p.pos.y = -2.5;
            }
        }
    }

    void simulate(int num_steps, double dt, bool wind = true) {
        for (int i = 0; i < num_steps; i++) {
            step(dt, wind);
        }
    }
};

// ─── 渲染 ────────────────────────────────────────────────────────────────────

struct Camera {
    Vec3 eye, target, up;
    double fov;
    int w, h;

    Camera(Vec3 eye, Vec3 target, Vec3 up, double fov, int w, int h)
        : eye(eye), target(target), up(up), fov(fov), w(w), h(h) {}

    // 将 3D 点投影到屏幕 (x, y, z=depth)
    std::array<double, 3> project(const Vec3& p) const {
        Vec3 forward = (target - eye).normalized();
        Vec3 right = forward.cross(up).normalized();
        Vec3 up_vec = right.cross(forward);

        Vec3 dir = p - eye;
        double d_forward = dir.dot(forward);
        double d_right = dir.dot(right);
        double d_up = dir.dot(up_vec);

        if (d_forward <= 0) return {-1, -1, -1};

        double aspect = (double)w / h;
        double tan_half_fov = std::tan(fov * 0.5 * M_PI / 180.0);
        double sx = d_right / (d_forward * tan_half_fov * aspect);
        double sy = d_up / (d_forward * tan_half_fov);

        double px = (sx + 1.0) * 0.5 * w;
        double py = (1.0 - (sy + 1.0) * 0.5) * h;
        return {px, py, d_forward};
    }
};

// Phong 着色
Vec3 phong(const Vec3& pos, const Vec3& normal, const Vec3& eye,
           const Vec3& light_pos, const Vec3& cloth_color, const Vec3& light_color)
{
    Vec3 N = normal.normalized();
    Vec3 L = (light_pos - pos).normalized();
    Vec3 V = (eye - pos).normalized();
    Vec3 R = (2.0 * N.dot(L) * N) - L;

    // 双面法线
    if (N.dot(V) < 0) N = {-N.x, -N.y, -N.z};

    double ambient = 0.15;
    double diffuse = std::max(0.0, N.dot(L));
    double specular = std::pow(std::max(0.0, R.dot(V)), 32.0);

    Vec3 amb = cloth_color * ambient;
    Vec3 diff = cloth_color * diffuse * 0.75;
    Vec3 spec = light_color * specular * 0.3;

    return amb + diff + spec;
}

// ACES 色调映射
Vec3 aces(Vec3 c) {
    double a = 2.51, b = 0.03, d = 2.43, e = 0.59, f = 0.14;
    return {
        (c.x*(a*c.x+b))/(c.x*(d*c.x+e)+f),
        (c.y*(a*c.y+b))/(c.y*(d*c.y+e)+f),
        (c.z*(a*c.z+b))/(c.z*(d*c.z+e)+f)
    };
}

// 绘制三角形（带 Z-Buffer）
void drawTriangle(Image& img, ZBuffer& zbuf, Camera& cam,
                  const Vec3& p0, const Vec3& p1, const Vec3& p2,
                  const Vec3& eye, const Vec3& light_pos,
                  const Vec3& cloth_color, const Vec3& light_color)
{
    // 投影
    auto s0 = cam.project(p0);
    auto s1 = cam.project(p1);
    auto s2 = cam.project(p2);

    if (s0[2] <= 0 || s1[2] <= 0 || s2[2] <= 0) return;

    // 法线（使用世界坐标）
    Vec3 edge1 = p1 - p0;
    Vec3 edge2 = p2 - p0;
    Vec3 normal = edge1.cross(edge2).normalized();

    // Bounding box
    int minX = (int)std::max(0.0, std::min({s0[0], s1[0], s2[0]}));
    int maxX = (int)std::min((double)(img.w-1), std::max({s0[0], s1[0], s2[0]}));
    int minY = (int)std::max(0.0, std::min({s0[1], s1[1], s2[1]}));
    int maxY = (int)std::min((double)(img.h-1), std::max({s0[1], s1[1], s2[1]}));

    // 重心坐标光栅化
    double area = (s1[0]-s0[0])*(s2[1]-s0[1]) - (s2[0]-s0[0])*(s1[1]-s0[1]);
    if (std::abs(area) < 1e-6) return;

    for (int y = minY; y <= maxY; y++) {
        for (int x = minX; x <= maxX; x++) {
            double px = x + 0.5, py = y + 0.5;
            double w0 = ((s1[0]-px)*(s2[1]-py) - (s2[0]-px)*(s1[1]-py)) / area;
            double w1 = ((s2[0]-px)*(s0[1]-py) - (s0[0]-px)*(s2[1]-py)) / area;
            double w2 = 1.0 - w0 - w1;

            if (w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4) continue;

            // 深度插值
            double depth = w0 * s0[2] + w1 * s1[2] + w2 * s2[2];
            if (!zbuf.test(x, y, depth)) continue;

            // 位置插值（用于光照计算）
            Vec3 pos = p0 * w0 + p1 * w1 + p2 * w2;

            // Phong 着色
            Vec3 color = phong(pos, normal, eye, light_pos, cloth_color, light_color);
            color = aces(color);
            img.setPixelf(x, y, color.x, color.y, color.z);
        }
    }
}

// 绘制线段（用于球体线框）
void drawLine(Image& img, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b) {
    int dx = std::abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        img.setPixel(x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 渲染一帧布料
void renderFrame(const ClothSim& cloth, const std::string& output_ppm,
                 int img_w, int img_h, bool draw_sphere = true)
{
    Image img(img_w, img_h);
    ZBuffer zbuf(img_w, img_h);

    // 背景：深蓝到黑色渐变
    for (int y = 0; y < img_h; y++) {
        double t = (double)y / img_h;
        for (int x = 0; x < img_w; x++) {
            img.setPixelf(x, y, t*0.05, t*0.07, t*0.15);
        }
    }

    // 相机设置
    Camera cam(
        Vec3(0, 1.0, 5.0),   // eye：略微俯视
        Vec3(0, -0.5, 0),     // target
        Vec3(0, 1, 0),        // up
        55.0,                 // fov
        img_w, img_h
    );

    Vec3 eye = cam.eye;
    Vec3 light_pos = {3.0, 4.0, 3.0};
    Vec3 light_color = {1.0, 0.95, 0.9};

    // 布料颜色（偏暖的红色/橙色，像中式丝绸）
    Vec3 cloth_color_front = {0.85, 0.25, 0.15};  // 正面红
    Vec3 cloth_color_back  = {0.6,  0.18, 0.10};  // 背面暗红

    // 渲染布料三角形（每个格子两个三角形）
    int rows = cloth.rows, cols = cloth.cols;
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols - 1; c++) {
            int i00 = cloth.index(r,   c);
            int i10 = cloth.index(r+1, c);
            int i01 = cloth.index(r,   c+1);
            int i11 = cloth.index(r+1, c+1);

            const Vec3& p00 = cloth.particles[i00].pos;
            const Vec3& p10 = cloth.particles[i10].pos;
            const Vec3& p01 = cloth.particles[i01].pos;
            const Vec3& p11 = cloth.particles[i11].pos;

            // 根据法线方向选择颜色（双面渲染）
            Vec3 n0 = (p10-p00).cross(p01-p00);
            Vec3 view0 = eye - p00;
            Vec3 color0 = (n0.dot(view0) > 0) ? cloth_color_front : cloth_color_back;

            Vec3 n1 = (p01-p11).cross(p10-p11);
            Vec3 view1 = eye - p11;
            Vec3 color1 = (n1.dot(view1) > 0) ? cloth_color_front : cloth_color_back;

            drawTriangle(img, zbuf, cam, p00, p10, p01, eye, light_pos, color0, light_color);
            drawTriangle(img, zbuf, cam, p11, p01, p10, eye, light_pos, color1, light_color);
        }
    }

    // 渲染球体（简单网格线框）
    if (draw_sphere) {
        Vec3 sc = cloth.sphere_center;
        double sr = cloth.sphere_radius;
        int sphere_segs = 20;
        for (int i = 0; i <= sphere_segs; i++) {
            double phi = M_PI * i / sphere_segs;
            for (int j = 0; j < sphere_segs; j++) {
                double theta0 = 2*M_PI * j / sphere_segs;
                double theta1 = 2*M_PI * (j+1) / sphere_segs;
                Vec3 pa = {
                    sc.x + sr * std::sin(phi) * std::cos(theta0),
                    sc.y + sr * std::cos(phi),
                    sc.z + sr * std::sin(phi) * std::sin(theta0)
                };
                Vec3 pb = {
                    sc.x + sr * std::sin(phi) * std::cos(theta1),
                    sc.y + sr * std::cos(phi),
                    sc.z + sr * std::sin(phi) * std::sin(theta1)
                };
                auto sa = cam.project(pa);
                auto sb = cam.project(pb);
                if (sa[2] > 0 && sb[2] > 0) {
                    drawLine(img, (int)sa[0], (int)sa[1], (int)sb[0], (int)sb[1],
                             160, 160, 180);
                }
            }
        }
    }

    img.savePPM(output_ppm);
}

// ─── 主程序 ──────────────────────────────────────────────────────────────────

int main() {
    printf("=== Cloth Simulation 布料模拟 ===\n");
    printf("质点弹簧系统 + Verlet积分 + 球体碰撞\n\n");

    const int ROWS = 25, COLS = 25;
    const double CLOTH_SIZE = 2.4;
    const int IMG_W = 800, IMG_H = 600;
    const double DT = 0.008;

    // ─── 阶段 1：模拟布料垂落过程（生成序列帧）
    printf("[1/3] 模拟布料垂落序列...\n");

    struct FrameInfo {
        int sim_steps;
        std::string ppm_path;
        std::string label;
    };

    std::vector<FrameInfo> frames = {
        { 0,   "/tmp/cloth_frame0.ppm",  "Frame 0 (initial)" },
        { 150, "/tmp/cloth_frame1.ppm",  "Frame 150 (early fall)" },
        { 300, "/tmp/cloth_frame2.ppm",  "Frame 300 (mid fall)" },
        { 600, "/tmp/cloth_frame3.ppm",  "Frame 600 (settled)" },
    };

    // 我们需要分别模拟到各个帧数（累加）
    // 重新创建模拟器，逐段推进
    ClothSim cloth_seq(ROWS, COLS, CLOTH_SIZE);

    int last_steps = 0;
    for (int fi = 0; fi < (int)frames.size(); fi++) {
        int target_steps = frames[fi].sim_steps;
        int steps_to_run = target_steps - last_steps;
        if (steps_to_run > 0) {
            cloth_seq.simulate(steps_to_run, DT);
        }
        last_steps = target_steps;
        renderFrame(cloth_seq, frames[fi].ppm_path, IMG_W, IMG_H);
        printf("  渲染帧 %d (%s) -> %s\n", fi, frames[fi].label.c_str(), frames[fi].ppm_path.c_str());
    }

    // ─── 阶段 2：继续模拟到最终状态（加入风力）
    printf("[2/3] 模拟最终状态（含风力）...\n");

    ClothSim cloth_final(ROWS, COLS, CLOTH_SIZE);
    // 先让布料自然垂落
    cloth_final.simulate(600, DT, false);
    // 加入风力
    cloth_final.simulate(200, DT, true);
    // 稳定
    cloth_final.simulate(100, DT, false);

    renderFrame(cloth_final, "/tmp/cloth_main.ppm", IMG_W, IMG_H);
    printf("  主渲染图完成: /tmp/cloth_main.ppm\n");

    // 统计粒子位置（用于验证）
    double min_y = 1e18, max_y = -1e18, avg_y = 0;
    for (const auto& p : cloth_final.particles) {
        if (!p.pinned) {
            min_y = std::min(min_y, p.pos.y);
            max_y = std::max(max_y, p.pos.y);
            avg_y += p.pos.y;
        }
    }
    int free_count = ROWS * COLS - COLS; // 除去固定的第一行
    avg_y /= free_count;
    printf("  粒子统计: min_y=%.3f, max_y=%.3f, avg_y=%.3f\n", min_y, max_y, avg_y);

    // 验证：布料已经垂落（平均 y 明显低于 0）
    if (avg_y > -0.3) {
        printf("WARNING: 布料可能没有正确垂落 (avg_y=%.3f)\n", avg_y);
    }

    // ─── 阶段 3：组合序列图（2x2 排列）
    printf("[3/3] 生成对比序列图...\n");

    const int SEQ_W = IMG_W * 2 + 20, SEQ_H = IMG_H * 2 + 60;
    Image seq_img(SEQ_W, SEQ_H);

    // 背景
    for (int y = 0; y < SEQ_H; y++)
        for (int x = 0; x < SEQ_W; x++)
            seq_img.setPixelf(x, y, 0.08, 0.08, 0.12);

    // 加载 4 帧图片并合并（读 PPM）
    auto loadPPM = [&](const std::string& path) -> Image {
        FILE* f = fopen(path.c_str(), "rb");
        Image empty(1, 1);
        if (!f) return empty;

        char magic[8];
        int w, h, maxval;
        if (fscanf(f, "%s %d %d %d\n", magic, &w, &h, &maxval) != 4) {
            fclose(f);
            return empty;
        }
        Image loaded(w, h);
        fread(loaded.pixels.data(), 1, w * h * 3, f);
        fclose(f);
        return loaded;
    };

    int offsets_x[] = {0, IMG_W + 20, 0, IMG_W + 20};
    int offsets_y[] = {0, 0, IMG_H + 40, IMG_H + 40};
    // const char* labels[] = {"0 frames", "150 frames", "300 frames", "600 frames"}; // reserved for future use

    for (int fi = 0; fi < 4; fi++) {
        Image frame = loadPPM(frames[fi].ppm_path);
        int ox = offsets_x[fi], oy = offsets_y[fi];
        for (int y = 0; y < frame.h && y < IMG_H; y++) {
            for (int x = 0; x < frame.w && x < IMG_W; x++) {
                int src_idx = (y * frame.w + x) * 3;
                seq_img.setPixel(
                    ox + x, oy + y,
                    frame.pixels[src_idx + 0],
                    frame.pixels[src_idx + 1],
                    frame.pixels[src_idx + 2]
                );
            }
        }
    }

    seq_img.savePPM("/tmp/cloth_sequence.ppm");
    printf("  序列图完成: /tmp/cloth_sequence.ppm\n");

    printf("\n✅ 模拟完成！\n");
    printf("主渲染: /tmp/cloth_main.ppm\n");
    printf("序列图: /tmp/cloth_sequence.ppm\n");

    return 0;
}
