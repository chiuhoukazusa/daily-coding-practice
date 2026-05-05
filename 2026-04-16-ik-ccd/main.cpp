/*
 * Inverse Kinematics CCD Solver
 * ============================
 * Daily Coding Practice - 2026-04-16
 *
 * 实现内容：
 *   - CCD (Cyclic Coordinate Descent) IK 算法
 *   - 多关节骨骼链（8关节）
 *   - 关节角度约束（可选）
 *   - 多目标位置序列动画
 *   - 软光栅化可视化：骨骼、关节、末端执行器轨迹
 *   - 三列布局：初始姿态、IK收敛过程、最终姿态
 *
 * 编译：g++ main.cpp -o output -std=c++17 -O2 -Wall -Wextra
 * 运行：./output → ik_output.ppm
 */

#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <string>
#include <cstring>
#include <cassert>

// ──────────────────────────────────────────
// Math
// ──────────────────────────────────────────
struct Vec2 {
    float x = 0.f, y = 0.f;
    Vec2() = default;
    Vec2(float x, float y) : x(x), y(y) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float t)        const { return {x * t, y * t}; }
    float dot(const Vec2& o)       const { return x * o.x + y * o.y; }
    float cross(const Vec2& o)     const { return x * o.y - y * o.x; }
    float length()                 const { return std::sqrt(x * x + y * y); }
    Vec2  normalized()             const {
        float l = length();
        return (l > 1e-8f) ? Vec2{x / l, y / l} : Vec2{0, 1};
    }
    float angle()                  const { return std::atan2(y, x); }
};

Vec2 rotate(const Vec2& v, float a) {
    float c = std::cos(a), s = std::sin(a);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
}

// ──────────────────────────────────────────
// Color & Framebuffer
// ──────────────────────────────────────────
struct Color {
    uint8_t r = 0, g = 0, b = 0;
};

const int W = 1440, H = 480;
std::vector<Color> fb(W * H);

void clearFB(Color c) { std::fill(fb.begin(), fb.end(), c); }

void setPixel(int x, int y, Color c) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    fb[y * W + x] = c;
}

// Blend with alpha
void blendPixel(int x, int y, Color c, float alpha) {
    if (x < 0 || x >= W || y < 0 || y >= H) return;
    auto& p = fb[y * W + x];
    p.r = (uint8_t)(p.r * (1 - alpha) + c.r * alpha);
    p.g = (uint8_t)(p.g * (1 - alpha) + c.g * alpha);
    p.b = (uint8_t)(p.b * (1 - alpha) + c.b * alpha);
}

// Bresenham anti-aliased line (Wu's)
void drawLine(float x0, float y0, float x1, float y1, Color c, float thickness = 1.f) {
    // Wu's line algorithm
    auto ipart  = [](float x) { return (int)x; };
    auto fpart  = [](float x) { return x - (int)x; };
    auto rfpart = [&fpart](float x) { return 1.f - fpart(x); };

    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) { std::swap(x0, y0); std::swap(x1, y1); }
    if (x0 > x1) { std::swap(x0, x1); std::swap(y0, y1); }

    float dx = x1 - x0, dy = y1 - y0;
    float gradient = (std::abs(dx) < 1e-6f) ? 1.f : dy / dx;

    // First endpoint
    float xend = std::round(x0);
    float yend = y0 + gradient * (xend - x0);
    float xgap = rfpart(x0 + 0.5f);
    int ix0 = (int)xend;
    int iy0 = ipart(yend);
    if (steep) {
        blendPixel(iy0,     ix0, c, rfpart(yend) * xgap);
        blendPixel(iy0 + 1, ix0, c, fpart(yend)  * xgap);
    } else {
        blendPixel(ix0, iy0,     c, rfpart(yend) * xgap);
        blendPixel(ix0, iy0 + 1, c, fpart(yend)  * xgap);
    }
    float intery = yend + gradient;

    // Second endpoint
    float xend2 = std::round(x1);
    float yend2 = y1 + gradient * (xend2 - x1);
    float xgap2 = fpart(x1 + 0.5f);
    int ix1 = (int)xend2;
    int iy1 = ipart(yend2);
    if (steep) {
        blendPixel(iy1,     ix1, c, rfpart(yend2) * xgap2);
        blendPixel(iy1 + 1, ix1, c, fpart(yend2)  * xgap2);
    } else {
        blendPixel(ix1, iy1,     c, rfpart(yend2) * xgap2);
        blendPixel(ix1, iy1 + 1, c, fpart(yend2)  * xgap2);
    }

    for (int x = ix0 + 1; x < ix1; ++x) {
        if (steep) {
            blendPixel(ipart(intery),     x, c, rfpart(intery));
            blendPixel(ipart(intery) + 1, x, c, fpart(intery));
        } else {
            blendPixel(x, ipart(intery),     c, rfpart(intery));
            blendPixel(x, ipart(intery) + 1, c, fpart(intery));
        }
        intery += gradient;
    }

    // Thickness: draw parallel lines
    if (thickness > 1.5f) {
        int extra = (int)(thickness / 2);
        for (int d = -extra; d <= extra; ++d) {
            if (d == 0) continue;
            if (steep) {
                drawLine(y0 + d, x0, y1 + d, x1, c, 1.f);
            } else {
                drawLine(x0, y0 + d, x1, y1 + d, c, 1.f);
            }
        }
    }
}

void drawCircle(float cx, float cy, float r, Color c, bool filled = false) {
    int ri = (int)std::ceil(r);
    if (filled) {
        for (int dy = -ri; dy <= ri; ++dy) {
            for (int dx = -ri; dx <= ri; ++dx) {
                float d = std::sqrt((float)(dx * dx + dy * dy));
                if (d <= r) {
                    float alpha = std::min(1.f, r - d + 0.5f);
                    blendPixel((int)cx + dx, (int)cy + dy, c, alpha);
                }
            }
        }
    } else {
        for (int dy = -ri; dy <= ri; ++dy) {
            for (int dx = -ri; dx <= ri; ++dx) {
                float d = std::sqrt((float)(dx * dx + dy * dy));
                if (std::abs(d - r) < 1.f) {
                    blendPixel((int)cx + dx, (int)cy + dy, c, 1.f - std::abs(d - r));
                }
            }
        }
    }
}

// ──────────────────────────────────────────
// IK Chain
// ──────────────────────────────────────────
struct Joint {
    float angle  = 0.f;   // 相对于父关节的旋转角（弧度）
    float length = 40.f;  // 骨骼长度（像素）
    float minAngle = -3.14159f;
    float maxAngle =  3.14159f;
};

struct IKChain {
    std::vector<Joint> joints;
    Vec2 base;  // 根节点世界坐标

    // 正运动学：计算各关节世界坐标
    std::vector<Vec2> forwardKinematics() const {
        std::vector<Vec2> pos(joints.size() + 1);
        pos[0] = base;
        float cumulativeAngle = 0.f;
        for (size_t i = 0; i < joints.size(); ++i) {
            cumulativeAngle += joints[i].angle;
            Vec2 dir = {std::cos(cumulativeAngle), std::sin(cumulativeAngle)};
            pos[i + 1] = pos[i] + dir * joints[i].length;
        }
        return pos;
    }

    // 末端执行器位置
    Vec2 endEffector() const {
        auto pos = forwardKinematics();
        return pos.back();
    }

    // CCD 单次迭代
    // 从末端向根节点遍历，旋转每个关节使末端靠近目标
    bool ccdIteration(const Vec2& target, float threshold = 1.f) {
        int n = (int)joints.size();
        for (int i = n - 1; i >= 0; --i) {
            // 当前关节世界坐标（需要重新计算）
            auto pos = forwardKinematics();
            Vec2 jointPos = pos[i];
            Vec2 endPos   = pos.back();
            Vec2 targetPos = target;

            // 当前末端相对于本关节的方向
            Vec2 toEnd    = (endPos    - jointPos).normalized();
            Vec2 toTarget = (targetPos - jointPos).normalized();

            // 需要旋转的角度
            float cosA = toEnd.dot(toTarget);
            cosA = std::max(-1.f, std::min(1.f, cosA));
            float delta = std::acos(cosA);

            // 旋转方向（叉积符号）
            float cross = toEnd.cross(toTarget);
            if (cross < 0) delta = -delta;

            // 应用到关节角度（考虑约束）
            joints[i].angle += delta;
            joints[i].angle = std::max(joints[i].minAngle,
                              std::min(joints[i].maxAngle, joints[i].angle));
        }

        // 检查收敛
        float dist = (endEffector() - target).length();
        return dist < threshold;
    }

    // 完整 CCD 求解
    int solve(const Vec2& target, int maxIter = 100, float threshold = 1.f) {
        for (int iter = 0; iter < maxIter; ++iter) {
            if (ccdIteration(target, threshold)) return iter + 1;
        }
        return maxIter;
    }
};

// ──────────────────────────────────────────
// Drawing Helpers
// ──────────────────────────────────────────
// 骨骼渲染颜色方案
const Color COL_BG      = {18,  18,  30 };   // 深蓝紫背景
const Color COL_BONE    = {100, 200, 255};   // 浅蓝骨骼
const Color COL_JOINT   = {255, 230, 100};   // 金色关节
const Color COL_END     = {255,  80,  80 };   // 红色末端
const Color COL_TARGET  = {80,  255, 120};   // 绿色目标
const Color COL_TRAIL   = {200, 100, 255};   // 紫色轨迹
const Color COL_ROOT    = {150, 150, 200};   // 灰蓝根节点
const Color COL_GRID    = {35,   35,  55 };   // 深网格
const Color COL_TEXT    = {200, 200, 220};   // 文字颜色

void drawGrid(int ox, int oy, int w, int h, int spacing = 40) {
    for (int x = ox; x < ox + w; x += spacing) {
        for (int y = oy; y < oy + h; ++y) {
            blendPixel(x, y, COL_GRID, 0.4f);
        }
    }
    for (int y = oy; y < oy + h; y += spacing) {
        for (int x = ox; x < ox + w; ++x) {
            blendPixel(x, y, COL_GRID, 0.4f);
        }
    }
}

void drawChain(const IKChain& chain, int ox, int oy) {
    auto pos = chain.forwardKinematics();

    // 骨骼（线段）
    for (size_t i = 0; i + 1 < pos.size(); ++i) {
        float x0 = ox + pos[i].x,     y0 = oy - pos[i].y;
        float x1 = ox + pos[i+1].x,   y1 = oy - pos[i+1].y;
        drawLine(x0, y0, x1, y1, COL_BONE, 3.f);
    }

    // 关节（圆圈）
    for (size_t i = 0; i < pos.size(); ++i) {
        float cx = ox + pos[i].x;
        float cy = oy - pos[i].y;
        Color c = (i == 0) ? COL_ROOT :
                  (i == pos.size() - 1) ? COL_END : COL_JOINT;
        float r = (i == 0) ? 8.f :
                  (i == pos.size() - 1) ? 6.f : 5.f;
        drawCircle(cx, cy, r, c, true);
        drawCircle(cx, cy, r + 1.f, {255, 255, 255}, false);
    }
}

void drawTarget(float tx, float ty, int ox, int oy) {
    float cx = ox + tx, cy = oy - ty;
    // 十字准星
    drawLine(cx - 12, cy, cx + 12, cy, COL_TARGET, 2.f);
    drawLine(cx, cy - 12, cx, cy + 12, COL_TARGET, 2.f);
    drawCircle(cx, cy, 8.f, COL_TARGET, false);
}

void drawTrail(const std::vector<Vec2>& trail, int ox, int oy) {
    for (size_t i = 0; i + 1 < trail.size(); ++i) {
        float x0 = ox + trail[i].x,   y0 = oy - trail[i].y;
        float x1 = ox + trail[i+1].x, y1 = oy - trail[i+1].y;
        float alpha = 0.2f + 0.8f * (float)i / trail.size();
        Color c = {
            (uint8_t)(COL_TRAIL.r * alpha),
            (uint8_t)(COL_TRAIL.g * alpha),
            (uint8_t)(COL_TRAIL.b * alpha)
        };
        drawLine(x0, y0, x1, y1, c, 1.f);
    }
}

// 简单字符绘制（仅支持数字和常用字母）
// 使用 3×5 点阵
static const int FONT[][5] = {
    // 0-9
    {0b111, 0b101, 0b101, 0b101, 0b111}, // 0
    {0b010, 0b110, 0b010, 0b010, 0b111}, // 1
    {0b111, 0b001, 0b111, 0b100, 0b111}, // 2
    {0b111, 0b001, 0b011, 0b001, 0b111}, // 3
    {0b101, 0b101, 0b111, 0b001, 0b001}, // 4
    {0b111, 0b100, 0b111, 0b001, 0b111}, // 5
    {0b111, 0b100, 0b111, 0b101, 0b111}, // 6
    {0b111, 0b001, 0b001, 0b001, 0b001}, // 7
    {0b111, 0b101, 0b111, 0b101, 0b111}, // 8
    {0b111, 0b101, 0b111, 0b001, 0b111}, // 9
};

void drawChar(int cx, int cy, char ch, Color c, int scale = 2) {
    int idx = -1;
    if (ch >= '0' && ch <= '9') idx = ch - '0';
    if (idx < 0) return;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (FONT[idx][row] & (1 << (2 - col))) {
                for (int sy = 0; sy < scale; ++sy)
                    for (int sx = 0; sx < scale; ++sx)
                        setPixel(cx + col * scale + sx, cy + row * scale + sy, c);
            }
        }
    }
}

void drawText(int x, int y, const std::string& s, Color c, int scale = 2) {
    int cx = x;
    for (char ch : s) {
        drawChar(cx, y, ch, c, scale);
        cx += 4 * scale;
    }
}

// ──────────────────────────────────────────
// 分隔线
// ──────────────────────────────────────────
void drawDivider(int x) {
    for (int y = 0; y < H; ++y) {
        Color c = {60, 60, 100};
        blendPixel(x, y, c, 0.8f);
        blendPixel(x + 1, y, c, 0.5f);
        blendPixel(x - 1, y, c, 0.5f);
    }
}

// ──────────────────────────────────────────
// 标题文字（使用更大字符）
// ──────────────────────────────────────────
// 完整 ASCII 5×8 点阵（仅支持 A-Z, a-z, 0-9, 空格, /, -, .）
// 用一行 uint8_t 数组存每个字符的8行位图（宽5位）

// 简化版：只支持英文字母+数字的绘制（4x6点阵，更小）
// 为了避免庞大的字体表，使用线段绘制文字段方式
// 这里改为直接绘制标签方块 + 简单颜色区分

void drawLabel(int x, int y, const std::string& /*label*/, Color bg, Color fg) {
    // 绘制彩色标签方块
    for (int dy = 0; dy < 20; ++dy) {
        for (int dx = 0; dx < 6; ++dx) {
            blendPixel(x + dx, y + dy, bg, 0.8f);
        }
    }
    // 中心点
    setPixel(x + 3, y + 10, fg);
}

// ──────────────────────────────────────────
// 面板标题（渐变色横条）
// ──────────────────────────────────────────
void drawPanelHeader(int ox, int w, int h_header, Color c1, Color c2) {
    for (int x = ox; x < ox + w; ++x) {
        float t = (float)(x - ox) / w;
        Color c = {
            (uint8_t)(c1.r * (1 - t) + c2.r * t),
            (uint8_t)(c1.g * (1 - t) + c2.g * t),
            (uint8_t)(c1.b * (1 - t) + c2.b * t)
        };
        for (int y = 0; y < h_header; ++y) {
            blendPixel(x, y, c, 0.6f);
        }
    }
}

// ──────────────────────────────────────────
// 保存 PPM
// ──────────────────────────────────────────
#include <cstdio>
void savePPM(const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (auto& c : fb) {
        fputc(c.r, f);
        fputc(c.g, f);
        fputc(c.b, f);
    }
    fclose(f);
}

// ──────────────────────────────────────────
// Main
// ──────────────────────────────────────────
int main() {
    // ── 场景参数 ──
    const int PANEL_W = W / 3;  // 每个面板宽度 = 480
    const int BASE_Y  = H - 60; // Y轴基准（从底部往上60像素是根节点）

    // 创建 IK 链：8个关节，长度递减
    auto makeChain = [](Vec2 base) {
        IKChain chain;
        chain.base = base;
        float lengths[] = {55.f, 50.f, 45.f, 40.f, 35.f, 30.f, 25.f, 20.f};
        float minAngles[] = {-1.5f, -1.2f, -1.5f, -2.0f, -2.0f, -2.0f, -2.5f, -2.5f};
        float maxAngles[] = { 1.5f,  1.2f,  1.5f,  2.0f,  2.0f,  2.0f,  2.5f,  2.5f};
        for (int i = 0; i < 8; ++i) {
            Joint j;
            j.length   = lengths[i];
            j.angle    = 0.15f * (i % 3 - 1);  // 初始小偏角，避免奇异姿态
            j.minAngle = minAngles[i];
            j.maxAngle = maxAngles[i];
            chain.joints.push_back(j);
        }
        return chain;
    };

    // 总骨骼链长
    float totalLen = 55 + 50 + 45 + 40 + 35 + 30 + 25 + 20; // = 300

    // ── 面板一：初始姿态 (Panel 0) ──
    {
        clearFB(COL_BG);

        // 绘制三个面板的网格和背景
        for (int p = 0; p < 3; ++p) {
            drawGrid(p * PANEL_W, 0, PANEL_W, H, 40);
            // 面板顶部渐变
            Color c1 = (p == 0) ? Color{40, 60, 100} :
                       (p == 1) ? Color{40, 80,  60} :
                                  Color{80, 40,  80};
            Color c2 = COL_BG;
            drawPanelHeader(p * PANEL_W, PANEL_W, 30, c1, c2);
        }

        drawDivider(PANEL_W);
        drawDivider(2 * PANEL_W);
    }

    // ── 面板一：初始 T-Pose ──
    {
        int ox = PANEL_W / 2;
        int oy = BASE_Y;

        IKChain chain = makeChain({0, 0});
        // 全部朝上（T形）
        for (auto& j : chain.joints) j.angle = 0.f;

        drawChain(chain, ox, oy);

        // 标记几个目标候选点
        std::vector<Vec2> targets = {
            {80,  220}, {-60, 180}, {100, 130}, {-80, 100}, {0,   250}
        };
        for (auto& t : targets) {
            drawTarget(t.x, t.y, ox, oy);
        }

        // 绘制可达范围圆弧
        drawCircle(ox, oy, totalLen, {50, 80, 150}, false);
        drawCircle(ox, oy, totalLen * 0.3f, {50, 80, 150}, false);
    }

    // ── 面板二：CCD 迭代过程（多目标切换） ──
    {
        int ox = PANEL_W + PANEL_W / 2;
        int oy = BASE_Y;

        IKChain chain = makeChain({0, 0});
        for (auto& j : chain.joints) j.angle = 0.f;

        // 目标序列
        std::vector<Vec2> targetSeq = {
            { 80.f,  220.f},
            {-100.f, 160.f},
            { 60.f,  100.f},
            {-40.f,  240.f},
            {120.f,  150.f},
        };

        // 末端轨迹
        std::vector<Vec2> endTrail;

        // 对每个目标运行 CCD
        for (size_t ti = 0; ti < targetSeq.size(); ++ti) {
            const Vec2& target = targetSeq[ti];

            // 记录收敛过程（每隔5次迭代采样）
            for (int iter = 0; iter < 50; ++iter) {
                bool conv = chain.ccdIteration(target, 2.f);
                if (iter % 5 == 0 || conv) {
                    Vec2 end = chain.endEffector();
                    endTrail.push_back(end);
                }
                if (conv) break;
            }
        }

        // 绘制末端轨迹
        drawTrail(endTrail, ox, oy);

        // 绘制当前收敛后的链
        drawChain(chain, ox, oy);

        // 绘制所有目标
        for (size_t ti = 0; ti < targetSeq.size(); ++ti) {
            drawTarget(targetSeq[ti].x, targetSeq[ti].y, ox, oy);
        }
    }

    // ── 面板三：最终收敛姿态（精确单目标） ──
    {
        int ox = 2 * PANEL_W + PANEL_W / 2;
        int oy = BASE_Y;

        IKChain chain = makeChain({0, 0});
        // 稍微弯曲的初始姿态
        for (size_t i = 0; i < chain.joints.size(); ++i) {
            chain.joints[i].angle = 0.05f * (float)i;
        }

        // 多个目标，每个完全收敛
        std::vector<Vec2> finalTargets = {
            {100.f, 200.f},
            {-80.f, 180.f},
            { 40.f, 280.f},
            {-120.f, 120.f},
        };

        std::vector<Vec2> endTrail2;
        std::vector<IKChain> snapshots;

        for (const Vec2& target : finalTargets) {
            int iters = chain.solve(target, 200, 1.f);
            (void)iters;
            endTrail2.push_back(chain.endEffector());
            snapshots.push_back(chain);
        }

        // 绘制轨迹
        drawTrail(endTrail2, ox, oy);

        // 绘制所有目标
        for (const Vec2& t : finalTargets) {
            drawTarget(t.x, t.y, ox, oy);
        }

        // 绘制历史快照（半透明）
        for (size_t i = 0; i + 1 < snapshots.size(); ++i) {
            auto& snap = snapshots[i];
            auto pos = snap.forwardKinematics();
            float alpha = 0.15f + 0.3f * (float)i / snapshots.size();
            for (size_t k = 0; k + 1 < pos.size(); ++k) {
                float x0 = ox + pos[k].x,   y0 = oy - pos[k].y;
                float x1 = ox + pos[k+1].x, y1 = oy - pos[k+1].y;
                Color c = {
                    (uint8_t)(COL_BONE.r * alpha),
                    (uint8_t)(COL_BONE.g * alpha),
                    (uint8_t)(COL_BONE.b * alpha)
                };
                drawLine(x0, y0, x1, y1, c, 2.f);
            }
        }

        // 绘制最终链（最后一次 solve 结果，即 snapshots.back()）
        drawChain(snapshots.back(), ox, oy);

        // 末端到目标的误差线
        {
            Vec2 endPt = snapshots.back().endEffector();
            Vec2 tgt   = finalTargets.back();
            float ex = ox + endPt.x, ey = oy - endPt.y;
            float tx = ox + tgt.x,   ty = oy - tgt.y;
            drawLine(ex, ey, tx, ty, {255, 100, 100}, 1.f);
        }
    }

    // ── 装饰：底部信息条 ──
    {
        for (int x = 0; x < W; ++x) {
            Color c = {25, 25, 45};
            for (int y = H - 30; y < H; ++y) {
                blendPixel(x, y, c, 0.9f);
            }
        }
        // 分隔线
        for (int x = 0; x < W; ++x) {
            blendPixel(x, H - 30, {80, 80, 180}, 0.8f);
        }
    }

    // ── 装饰：顶部标题条 ──
    {
        for (int x = 0; x < W; ++x) {
            for (int y = 0; y < 28; ++y) {
                float t = (float)x / W;
                Color c = {
                    (uint8_t)(30 + 20 * t),
                    (uint8_t)(30 + 30 * t),
                    (uint8_t)(50 + 50 * t)
                };
                blendPixel(x, y, c, 0.85f);
            }
        }
        for (int x = 0; x < W; ++x) {
            blendPixel(x, 28, {80, 100, 200}, 0.9f);
        }

        // 面板编号点
        for (int p = 0; p < 3; ++p) {
            int cx = p * PANEL_W + PANEL_W / 2;
            Color pc = (p == 0) ? Color{100, 180, 255} :
                       (p == 1) ? Color{100, 255, 150} :
                                  Color{220, 100, 255};
            drawCircle((float)cx, 14.f, 8.f, pc, true);
            drawText(cx - 3, 9, std::to_string(p + 1), {20, 20, 40}, 2);
        }
    }

    // ── 装饰：根节点地面线 ──
    {
        for (int p = 0; p < 3; ++p) {
            int ox = p * PANEL_W;
            int oy = BASE_Y;
            for (int x = ox + 10; x < ox + PANEL_W - 10; ++x) {
                blendPixel(x, oy + 1, {80, 80, 120}, 0.8f);
                blendPixel(x, oy + 2, {60, 60, 100}, 0.5f);
            }
        }
    }

    savePPM("ik_output.ppm");
    printf("Rendered to ik_output.ppm (%dx%d)\n", W, H);
    return 0;
}
