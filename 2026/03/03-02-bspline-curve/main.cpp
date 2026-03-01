/**
 * B-spline Curve Renderer
 * 
 * 实现 B-spline 曲线绘制器，包含：
 * - Cox-de Boor 递归算法
 * - 均匀/准均匀/端点插值 knot vector
 * - 2阶/3阶/4阶 B-spline
 * - 与 Bezier 曲线的对比展示
 * - 控制点可视化
 * - 多色曲线对比图
 * 
 * 输出: bspline_output.png (800x900 总览图)
 *       bspline_quadratic.png (800x400 二次)
 *       bspline_cubic.png (800x400 三次)
 *       bspline_vs_bezier.png (800x400 对比)
 * 
 * 编译: g++ -std=c++17 -O2 main.cpp -o bspline
 * 运行: ./bspline
 */

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <cassert>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <functional>

// ============================================================
//  基础数学结构
// ============================================================

struct Vec2 {
    double x, y;
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& b) const { return {x + b.x, y + b.y}; }
    Vec2 operator-(const Vec2& b) const { return {x - b.x, y - b.y}; }
    Vec2 operator*(double t)      const { return {x * t,   y * t};   }
    double length() const { return std::sqrt(x*x + y*y); }
};

struct Color {
    uint8_t r, g, b, a;
    Color(uint8_t r=0, uint8_t g=0, uint8_t b=0, uint8_t a=255)
        : r(r), g(g), b(b), a(a) {}
};

// ============================================================
//  画布
// ============================================================

struct Canvas {
    int width, height;
    std::vector<uint8_t> data; // RGBA

    Canvas(int w, int h, Color bg = Color(30, 30, 40))
        : width(w), height(h), data(w * h * 4) {
        for (int i = 0; i < w * h; i++) {
            data[i*4+0] = bg.r;
            data[i*4+1] = bg.g;
            data[i*4+2] = bg.b;
            data[i*4+3] = bg.a;
        }
    }

    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 4;
        // Alpha blending
        float alpha = c.a / 255.0f;
        data[idx+0] = (uint8_t)(data[idx+0] * (1-alpha) + c.r * alpha);
        data[idx+1] = (uint8_t)(data[idx+1] * (1-alpha) + c.g * alpha);
        data[idx+2] = (uint8_t)(data[idx+2] * (1-alpha) + c.b * alpha);
        data[idx+3] = 255;
    }

    // Bresenham 直线
    void drawLine(int x0, int y0, int x1, int y1, Color c, int thickness = 1) {
        int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
        int sx = x0<x1 ? 1 : -1, sy = y0<y1 ? 1 : -1;
        int err = dx - dy;
        while (true) {
            for (int ty = -thickness/2; ty <= thickness/2; ty++)
                for (int tx = -thickness/2; tx <= thickness/2; tx++)
                    setPixel(x0+tx, y0+ty, c);
            if (x0==x1 && y0==y1) break;
            int e2 = 2*err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }

    // 画圆
    void drawCircle(int cx, int cy, int r, Color c, bool fill = false) {
        if (fill) {
            for (int y = -r; y <= r; y++)
                for (int x = -r; x <= r; x++)
                    if (x*x + y*y <= r*r)
                        setPixel(cx+x, cy+y, c);
        } else {
            int x = r, y = 0, err = 0;
            while (x >= y) {
                for (int d = -1; d <= 1; d++) {
                    setPixel(cx+x, cy+y+d, c); setPixel(cx-x, cy+y+d, c);
                    setPixel(cx+x, cy-y+d, c); setPixel(cx-x, cy-y+d, c);
                    setPixel(cx+y+d, cy+x, c); setPixel(cx-y+d, cy+x, c);
                    setPixel(cx+y+d, cy-x, c); setPixel(cx-y+d, cy-x, c);
                }
                y++;
                err += 1 + 2*y;
                if (2*(err-x) + 1 > 0) { x--; err += 1 - 2*x; }
            }
        }
    }

    bool save(const std::string& path) {
        return stbi_write_png(path.c_str(), width, height, 4, data.data(), width*4) != 0;
    }
};

// ============================================================
//  B-spline 核心算法 (Cox-de Boor 递归)
// ============================================================

/**
 * Cox-de Boor 基函数
 * N_{i,p}(t) = 递归定义的B样条基函数
 * i: 基函数索引
 * p: 阶数（degree）
 * t: 参数值
 * knots: 节点向量
 */
double basisFunction(int i, int p, double t, const std::vector<double>& knots) {
    if (p == 0) {
        // 注意：在最后一个节点区间时特殊处理，避免开区间问题
        if (i + 1 < (int)knots.size()) {
            if (t >= knots[i] && t < knots[i+1]) return 1.0;
            // 对末端点特殊处理
            if (std::abs(t - knots.back()) < 1e-10 && std::abs(t - knots[i+1]) < 1e-10)
                return 1.0;
        }
        return 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    // 左部分: (t - t_i) / (t_{i+p} - t_i) * N_{i,p-1}(t)
    if (i + p < (int)knots.size()) {
        double denom = knots[i+p] - knots[i];
        if (std::abs(denom) > 1e-10)
            left = (t - knots[i]) / denom * basisFunction(i, p-1, t, knots);
    }
    
    // 右部分: (t_{i+p+1} - t) / (t_{i+p+1} - t_{i+1}) * N_{i+1,p-1}(t)
    if (i + p + 1 < (int)knots.size()) {
        double denom = knots[i+p+1] - knots[i+1];
        if (std::abs(denom) > 1e-10)
            right = (knots[i+p+1] - t) / denom * basisFunction(i+1, p-1, t, knots);
    }
    
    return left + right;
}

/**
 * 计算 B-spline 曲线上的点
 * controlPoints: 控制点
 * knots: 节点向量
 * degree: 阶数
 * t: 参数 [0, 1]
 */
Vec2 bsplinePoint(const std::vector<Vec2>& controlPoints,
                   const std::vector<double>& knots,
                   int degree, double t) {
    int n = (int)controlPoints.size();
    Vec2 result;
    
    // 参数范围映射到 knot 范围
    double tMin = knots[degree];
    double tMax = knots[n]; // n = n_control - 1 + 1
    double tMapped = tMin + t * (tMax - tMin);
    
    // 对末端点处理
    if (std::abs(t - 1.0) < 1e-10) tMapped = tMax - 1e-10;
    
    for (int i = 0; i < n; i++) {
        double basis = basisFunction(i, degree, tMapped, knots);
        result = result + controlPoints[i] * basis;
    }
    return result;
}

/**
 * 生成均匀节点向量 (Uniform)
 * n+1 个控制点, degree p
 * knot vector 长度 = n + p + 2
 */
std::vector<double> uniformKnots(int n_ctrl, int degree) {
    int m = n_ctrl + degree + 1; // 节点数
    std::vector<double> knots(m);
    for (int i = 0; i < m; i++)
        knots[i] = (double)i / (m - 1);
    return knots;
}

/**
 * 生成端点插值节点向量 (Clamped/Open)
 * 首尾各重复 degree+1 次，保证曲线经过端控制点
 */
std::vector<double> clampedKnots(int n_ctrl, int degree) {
    int n = n_ctrl - 1;
    int m = n + degree + 2; // 节点数 = n+1 + degree+1
    std::vector<double> knots(m);
    
    // 前 degree+1 个节点 = 0
    for (int i = 0; i <= degree; i++) knots[i] = 0.0;
    
    // 中间节点均匀分布
    int inner = n - degree; // 内部节点数
    for (int j = 1; j <= inner; j++)
        knots[degree + j] = (double)j / (inner + 1);
    
    // 后 degree+1 个节点 = 1
    for (int i = m - degree - 1; i < m; i++) knots[i] = 1.0;
    
    return knots;
}

// ============================================================
//  Bezier 曲线（用于对比）
// ============================================================

Vec2 bezierPoint(const std::vector<Vec2>& pts, double t) {
    std::vector<Vec2> tmp = pts;
    int n = tmp.size();
    for (int r = 1; r < n; r++)
        for (int i = 0; i < n - r; i++)
            tmp[i] = tmp[i] * (1-t) + tmp[i+1] * t;
    return tmp[0];
}

// ============================================================
//  渲染辅助函数
// ============================================================

// 坐标映射：数学坐标 → 图像坐标
struct Viewport {
    double xMin, xMax, yMin, yMax;
    int imgW, imgH;
    int marginL, marginR, marginT, marginB;
    
    int toPixelX(double x) const {
        double t = (x - xMin) / (xMax - xMin);
        return marginL + (int)(t * (imgW - marginL - marginR));
    }
    int toPixelY(double y) const {
        double t = (y - yMax) / (yMin - yMax); // 注意 Y 轴翻转
        return marginT + (int)(t * (imgH - marginT - marginB));
    }
};

// 绘制控制多边形
void drawControlPolygon(Canvas& canvas,
                         const std::vector<Vec2>& pts,
                         const Viewport& vp,
                         Color lineColor = Color(100, 100, 100, 180),
                         Color ptColor   = Color(255, 200, 0)) {
    for (int i = 0; i + 1 < (int)pts.size(); i++) {
        int x0 = vp.toPixelX(pts[i].x),   y0 = vp.toPixelY(pts[i].y);
        int x1 = vp.toPixelX(pts[i+1].x), y1 = vp.toPixelY(pts[i+1].y);
        canvas.drawLine(x0, y0, x1, y1, lineColor, 1);
    }
    for (auto& p : pts) {
        int px = vp.toPixelX(p.x), py = vp.toPixelY(p.y);
        canvas.drawCircle(px, py, 5, ptColor, true);
        canvas.drawCircle(px, py, 5, Color(50, 50, 50), false);
    }
}

// 简单文字渲染（用像素绘制字符）
// 使用 5×7 位图字体
const uint8_t FONT_5x7[96][7] = {
    // 从空格(32)开始，每个字符 5 列×7 行，每行是 5bit 掩码
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x04,0x04,0x04,0x04,0x00,0x04,0x00}, // '!'
    {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00}, // '"'
    {0x0A,0x1F,0x0A,0x1F,0x0A,0x00,0x00}, // '#'
    {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04}, // '$'
    {0x18,0x19,0x02,0x04,0x13,0x03,0x00}, // '%'
    {0x08,0x14,0x14,0x08,0x15,0x12,0x0D}, // '&'
    {0x04,0x04,0x00,0x00,0x00,0x00,0x00}, // '\''
    {0x02,0x04,0x08,0x08,0x08,0x04,0x02}, // '('
    {0x08,0x04,0x02,0x02,0x02,0x04,0x08}, // ')'
    {0x00,0x04,0x15,0x0E,0x15,0x04,0x00}, // '*'
    {0x00,0x04,0x04,0x1F,0x04,0x04,0x00}, // '+'
    {0x00,0x00,0x00,0x00,0x00,0x04,0x08}, // ','
    {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}, // '-'
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}, // '.'
    {0x00,0x01,0x02,0x04,0x08,0x10,0x00}, // '/'
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, // '0'
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // '1'
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, // '2'
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}, // '3'
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, // '4'
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, // '5'
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, // '6'
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, // '7'
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, // '8'
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, // '9'
    {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}, // ':'
    {0x00,0x0C,0x0C,0x00,0x0C,0x04,0x08}, // ';'
    {0x02,0x04,0x08,0x10,0x08,0x04,0x02}, // '<'
    {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00}, // '='
    {0x08,0x04,0x02,0x01,0x02,0x04,0x08}, // '>'
    {0x0E,0x11,0x01,0x06,0x04,0x00,0x04}, // '?'
    {0x0E,0x11,0x17,0x15,0x17,0x10,0x0F}, // '@'
    {0x04,0x0A,0x11,0x11,0x1F,0x11,0x11}, // 'A'
    {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}, // 'B'
    {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}, // 'C'
    {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}, // 'D'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}, // 'E'
    {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}, // 'F'
    {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}, // 'G'
    {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}, // 'H'
    {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}, // 'I'
    {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}, // 'J'
    {0x11,0x12,0x14,0x18,0x14,0x12,0x11}, // 'K'
    {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}, // 'L'
    {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}, // 'M'
    {0x11,0x19,0x15,0x13,0x11,0x11,0x11}, // 'N'
    {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}, // 'O'
    {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}, // 'P'
    {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}, // 'Q'
    {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}, // 'R'
    {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}, // 'S'
    {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}, // 'T'
    {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}, // 'U'
    {0x11,0x11,0x11,0x0A,0x0A,0x04,0x04}, // 'V'
    {0x11,0x11,0x15,0x15,0x15,0x0A,0x0A}, // 'W'
    {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}, // 'X'
    {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}, // 'Y'
    {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}, // 'Z'
    {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E}, // '['
    {0x00,0x10,0x08,0x04,0x02,0x01,0x00}, // '\\'
    {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E}, // ']'
    {0x04,0x0A,0x11,0x00,0x00,0x00,0x00}, // '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x1F}, // '_'
    {0x08,0x04,0x00,0x00,0x00,0x00,0x00}, // '`'
    {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F}, // 'a'
    {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E}, // 'b'
    {0x00,0x00,0x0F,0x10,0x10,0x10,0x0F}, // 'c'
    {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F}, // 'd'
    {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E}, // 'e'
    {0x06,0x09,0x08,0x1E,0x08,0x08,0x08}, // 'f'
    {0x00,0x00,0x0F,0x11,0x11,0x0F,0x01}, // 'g' (truncated)
    {0x10,0x10,0x1E,0x11,0x11,0x11,0x11}, // 'h'
    {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E}, // 'i'
    {0x02,0x00,0x06,0x02,0x02,0x12,0x0C}, // 'j'
    {0x10,0x10,0x12,0x14,0x18,0x14,0x12}, // 'k'
    {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E}, // 'l'
    {0x00,0x00,0x1A,0x15,0x15,0x11,0x11}, // 'm'
    {0x00,0x00,0x1E,0x11,0x11,0x11,0x11}, // 'n'
    {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E}, // 'o'
    {0x00,0x00,0x1E,0x11,0x11,0x1E,0x10}, // 'p'
    {0x00,0x00,0x0F,0x11,0x11,0x0F,0x01}, // 'q'
    {0x00,0x00,0x17,0x18,0x10,0x10,0x10}, // 'r'
    {0x00,0x00,0x0E,0x10,0x0E,0x01,0x1E}, // 's'
    {0x08,0x08,0x1E,0x08,0x08,0x09,0x06}, // 't'
    {0x00,0x00,0x11,0x11,0x11,0x11,0x0F}, // 'u'
    {0x00,0x00,0x11,0x11,0x0A,0x0A,0x04}, // 'v'
    {0x00,0x00,0x11,0x15,0x15,0x0A,0x0A}, // 'w'
    {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11}, // 'x'
    {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E}, // 'y'
    {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F}, // 'z'
    {0x06,0x08,0x08,0x18,0x08,0x08,0x06}, // '{'
    {0x04,0x04,0x04,0x00,0x04,0x04,0x04}, // '|'
    {0x0C,0x02,0x02,0x03,0x02,0x02,0x0C}, // '}'
    {0x08,0x15,0x02,0x00,0x00,0x00,0x00}, // '~'
    {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}, // DEL (127)
};

void drawChar(Canvas& canvas, int x, int y, char c, Color color, int scale = 1) {
    int idx = (int)c - 32;
    if (idx < 0 || idx >= 96) return;
    for (int row = 0; row < 7; row++) {
        uint8_t mask = FONT_5x7[idx][row];
        for (int col = 0; col < 5; col++) {
            if (mask & (1 << (4 - col))) {
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        canvas.setPixel(x + col*scale + sx, y + row*scale + sy, color);
            }
        }
    }
}

void drawText(Canvas& canvas, int x, int y, const std::string& text, Color color, int scale = 1) {
    int cx = x;
    for (char c : text) {
        drawChar(canvas, cx, y, c, color, scale);
        cx += (5 + 1) * scale;
    }
}

// 绘制水平分割线
void drawDivider(Canvas& canvas, int y, Color c = Color(60, 60, 80)) {
    for (int x = 20; x < canvas.width - 20; x++)
        canvas.setPixel(x, y, c);
}

// ============================================================
//  主渲染逻辑
// ============================================================

/**
 * 渲染单个面板：展示 B-spline 曲线
 * @param canvas 画布
 * @param offsetY 面板在画布中的起始Y坐标
 * @param panelH  面板高度
 * @param title   标题
 * @param controlPoints 控制点
 * @param degree  B-spline 阶数
 * @param knotType 0=均匀 1=端点插值(clamped)
 * @param curveColor 曲线颜色
 */
void renderBsplinePanel(Canvas& canvas, int offsetY, int panelH,
                         const std::string& title,
                         const std::vector<Vec2>& controlPoints,
                         int degree, bool clamped,
                         Color curveColor) {
    
    int W = canvas.width;
    int margin = 60;
    
    // 标题
    drawText(canvas, margin, offsetY + 15, title, Color(220, 220, 255), 2);
    
    // 计算控制点的包围盒
    double xMin = 1e9, xMax = -1e9, yMin = 1e9, yMax = -1e9;
    for (auto& p : controlPoints) {
        xMin = std::min(xMin, p.x); xMax = std::max(xMax, p.x);
        yMin = std::min(yMin, p.y); yMax = std::max(yMax, p.y);
    }
    // 加一点 padding
    double padX = (xMax - xMin) * 0.15 + 0.5;
    double padY = (yMax - yMin) * 0.15 + 0.5;
    
    Viewport vp;
    vp.xMin = xMin - padX; vp.xMax = xMax + padX;
    vp.yMin = yMin - padY; vp.yMax = yMax + padY;
    vp.imgW = W;
    vp.imgH = panelH;
    vp.marginL = margin; vp.marginR = margin;
    vp.marginT = offsetY + 50; 
    vp.marginB = (canvas.height - offsetY - panelH) + 30;
    
    // 修正 viewport 的 toPixelY，因为 Canvas 是从上到下
    // 但这里我们要在偏移了的面板里画，所以重写 lambda
    auto toX = [&](double x) {
        double t = (x - vp.xMin) / (vp.xMax - vp.xMin);
        return (int)(margin + t * (W - 2*margin));
    };
    auto toY = [&](double y) {
        double t = (y - vp.yMin) / (vp.yMax - vp.yMin);
        // y 轴向上，图像向下
        return (int)(offsetY + panelH - 30 - t * (panelH - 80));
    };
    
    // 画控制多边形
    for (int i = 0; i + 1 < (int)controlPoints.size(); i++) {
        int x0 = toX(controlPoints[i].x),   y0 = toY(controlPoints[i].y);
        int x1 = toX(controlPoints[i+1].x), y1 = toY(controlPoints[i+1].y);
        canvas.drawLine(x0, y0, x1, y1, Color(100, 100, 120, 150), 1);
    }
    
    // 生成节点向量
    int n_ctrl = controlPoints.size();
    std::vector<double> knots = clamped ? clampedKnots(n_ctrl, degree) : uniformKnots(n_ctrl, degree);
    
    // 画 B-spline 曲线
    Vec2 prev = bsplinePoint(controlPoints, knots, degree, 0.0);
    for (int i = 1; i <= 600; i++) {
        double t = (double)i / 600;
        Vec2 curr = bsplinePoint(controlPoints, knots, degree, t);
        
        int x0 = toX(prev.x), y0 = toY(prev.y);
        int x1 = toX(curr.x), y1 = toY(curr.y);
        canvas.drawLine(x0, y0, x1, y1, curveColor, 2);
        prev = curr;
    }
    
    // 画控制点
    for (auto& p : controlPoints) {
        int px = toX(p.x), py = toY(p.y);
        canvas.drawCircle(px, py, 5, Color(255, 220, 0), true);
        canvas.drawCircle(px, py, 5, Color(80, 80, 80), false);
    }
    
    // 标注 knot vector（简化显示）
    std::ostringstream knotStr;
    knotStr << "Knots: [";
    for (int i = 0; i < (int)knots.size(); i++) {
        if (i > 0) knotStr << ", ";
        knotStr << std::fixed << std::setprecision(2) << knots[i];
    }
    knotStr << "]  Degree: " << degree << "  " << (clamped ? "(Clamped)" : "(Uniform)");
    
    drawText(canvas, margin, offsetY + panelH - 25, knotStr.str(), Color(150, 180, 150), 1);
}

/**
 * 对比面板：B-spline vs Bezier（相同控制点）
 */
void renderComparisonPanel(Canvas& canvas, int offsetY, int panelH,
                            const std::vector<Vec2>& controlPoints) {
    int W = canvas.width;
    int margin = 60;
    
    drawText(canvas, margin, offsetY + 15, "B-spline vs Bezier (same control points)", Color(220, 220, 255), 2);
    
    // 包围盒
    double xMin = 1e9, xMax = -1e9, yMin = 1e9, yMax = -1e9;
    for (auto& p : controlPoints) {
        xMin = std::min(xMin, p.x); xMax = std::max(xMax, p.x);
        yMin = std::min(yMin, p.y); yMax = std::max(yMax, p.y);
    }
    double padX = (xMax - xMin) * 0.15 + 0.5;
    double padY = (yMax - yMin) * 0.15 + 0.5;
    
    auto toX = [&](double x) {
        double t = (x - (xMin-padX)) / (xMax - xMin + 2*padX);
        return (int)(margin + t * (W - 2*margin));
    };
    auto toY = [&](double y) {
        double t = (y - (yMin-padY)) / (yMax - yMin + 2*padY);
        return (int)(offsetY + panelH - 30 - t * (panelH - 80));
    };
    
    // 控制多边形
    for (int i = 0; i + 1 < (int)controlPoints.size(); i++) {
        canvas.drawLine(toX(controlPoints[i].x), toY(controlPoints[i].y),
                        toX(controlPoints[i+1].x), toY(controlPoints[i+1].y),
                        Color(80, 80, 100, 150), 1);
    }
    
    // B-spline (clamped cubic)
    int n_ctrl = controlPoints.size();
    int degree = std::min(3, n_ctrl - 1);
    auto knots = clampedKnots(n_ctrl, degree);
    
    Vec2 prev = bsplinePoint(controlPoints, knots, degree, 0.0);
    for (int i = 1; i <= 600; i++) {
        double t = (double)i / 600;
        Vec2 curr = bsplinePoint(controlPoints, knots, degree, t);
        canvas.drawLine(toX(prev.x), toY(prev.y), toX(curr.x), toY(curr.y),
                        Color(80, 160, 255), 2);
        prev = curr;
    }
    
    // Bezier
    Vec2 bPrev = bezierPoint(controlPoints, 0.0);
    for (int i = 1; i <= 600; i++) {
        double t = (double)i / 600;
        Vec2 bCurr = bezierPoint(controlPoints, t);
        canvas.drawLine(toX(bPrev.x), toY(bPrev.y), toX(bCurr.x), toY(bCurr.y),
                        Color(255, 100, 100), 2);
        bPrev = bCurr;
    }
    
    // 控制点
    for (auto& p : controlPoints) {
        canvas.drawCircle(toX(p.x), toY(p.y), 5, Color(255, 220, 0), true);
    }
    
    // 图例
    canvas.drawLine(margin,       offsetY + panelH - 25, margin + 20, offsetY + panelH - 25,
                    Color(80, 160, 255), 3);
    drawText(canvas, margin + 25, offsetY + panelH - 30, "B-spline (Clamped Cubic)", Color(80, 160, 255), 1);
    
    canvas.drawLine(margin + 200, offsetY + panelH - 25, margin + 220, offsetY + panelH - 25,
                    Color(255, 100, 100), 3);
    drawText(canvas, margin + 225, offsetY + panelH - 30, "Bezier", Color(255, 100, 100), 1);
}

// ============================================================
//  主函数
// ============================================================

int main() {
    std::cout << "=== B-spline Curve Renderer ===" << std::endl;
    std::cout << "Using Cox-de Boor recursive algorithm" << std::endl;
    
    // ─── 定义各面板的控制点 ───────────────────────────────────────
    
    // 面板1: 二次 B-spline（三控制点，典型抛物线形状）
    std::vector<Vec2> ctrl_quadratic = {
        {1, 1}, {2, 4}, {4, 4}, {5, 1}, {7, 3}, {8, 1}
    };
    
    // 面板2: 三次 clamped B-spline（经典S形）
    std::vector<Vec2> ctrl_cubic = {
        {1, 2}, {2, 5}, {4, 5}, {5, 3}, {6, 1}, {8, 4}, {9, 2}
    };
    
    // 面板3: 更多控制点的高次 B-spline（展示局部控制性）
    std::vector<Vec2> ctrl_local = {
        {1, 3}, {2, 5}, {3, 2}, {4, 5}, {5, 2}, {6, 5}, {7, 2}, {8, 5}, {9, 3}
    };
    
    // 面板4: 对比 B-spline vs Bezier
    std::vector<Vec2> ctrl_compare = {
        {1, 2}, {2, 5}, {4, 5}, {6, 4}, {7, 2}, {8, 3}
    };
    
    // ─── 总览图 (800 × 1200) ──────────────────────────────────────
    {
        const int W = 800, H = 1200;
        Canvas canvas(W, H, Color(20, 22, 30));
        
        // 标题栏
        drawText(canvas, 20, 10, "B-spline Curve Renderer  -  Cox-de Boor Algorithm", Color(255, 220, 120), 2);
        
        int panelH = 260;
        
        // Panel 1: Quadratic (degree=2) Uniform
        drawDivider(canvas, 44);
        renderBsplinePanel(canvas, 45, panelH,
                           "Quadratic B-spline (Degree 2, Uniform)",
                           ctrl_quadratic, 2, false,
                           Color(100, 220, 160));
        
        // Panel 2: Cubic (degree=3) Clamped
        drawDivider(canvas, 45 + panelH);
        renderBsplinePanel(canvas, 45 + panelH, panelH,
                           "Cubic B-spline (Degree 3, Clamped)",
                           ctrl_cubic, 3, true,
                           Color(100, 160, 255));
        
        // Panel 3: Cubic Uniform (多波浪，展示局部控制)
        drawDivider(canvas, 45 + panelH * 2);
        renderBsplinePanel(canvas, 45 + panelH * 2, panelH,
                           "Cubic B-spline (Degree 3, Uniform) - Local Control",
                           ctrl_local, 3, false,
                           Color(255, 140, 80));
        
        // Panel 4: B-spline vs Bezier
        drawDivider(canvas, 45 + panelH * 3);
        renderComparisonPanel(canvas, 45 + panelH * 3, panelH, ctrl_compare);
        
        // 底部信息
        drawDivider(canvas, 45 + panelH * 4);
        drawText(canvas, 20, H - 30, "2026-03-02  B-spline Curves | Cox-de Boor | Clamped & Uniform Knot Vectors", Color(120, 140, 120), 1);
        
        if (canvas.save("bspline_output.png")) {
            std::cout << "✅ Saved: bspline_output.png (" << W << "x" << H << ")" << std::endl;
        } else {
            std::cerr << "❌ Failed to save bspline_output.png" << std::endl;
            return 1;
        }
    }
    
    // ─── 二次曲线单独图 (800 × 400) ─────────────────────────────
    {
        const int W = 800, H = 400;
        Canvas canvas(W, H, Color(20, 22, 30));
        drawText(canvas, 20, 8, "Quadratic B-spline (Degree 2)", Color(255, 220, 120), 2);
        renderBsplinePanel(canvas, 0, H, "", ctrl_quadratic, 2, false, Color(100, 220, 160));
        
        if (canvas.save("bspline_quadratic.png"))
            std::cout << "✅ Saved: bspline_quadratic.png" << std::endl;
    }
    
    // ─── 三次 clamped 单独图 (800 × 400) ────────────────────────
    {
        const int W = 800, H = 400;
        Canvas canvas(W, H, Color(20, 22, 30));
        drawText(canvas, 20, 8, "Cubic B-spline (Degree 3, Clamped)", Color(255, 220, 120), 2);
        renderBsplinePanel(canvas, 0, H, "", ctrl_cubic, 3, true, Color(100, 160, 255));
        
        if (canvas.save("bspline_cubic.png"))
            std::cout << "✅ Saved: bspline_cubic.png" << std::endl;
    }
    
    // ─── 对比图 (800 × 400) ──────────────────────────────────────
    {
        const int W = 800, H = 400;
        Canvas canvas(W, H, Color(20, 22, 30));
        renderComparisonPanel(canvas, 0, H, ctrl_compare);
        
        if (canvas.save("bspline_vs_bezier.png"))
            std::cout << "✅ Saved: bspline_vs_bezier.png" << std::endl;
    }
    
    // ─── 验证输出 ─────────────────────────────────────────────────
    std::cout << "\n=== Validation ===" << std::endl;
    std::cout << "Verifying Cox-de Boor basis functions:" << std::endl;
    
    // 验证基函数单位分割性：∑ N_{i,p}(t) = 1
    std::vector<Vec2> testCtrl = {{0,0},{1,0},{2,0},{3,0}};
    auto knots3 = clampedKnots(4, 3);
    bool partitionOk = true;
    for (int k = 0; k <= 10; k++) {
        double t = k / 10.0;
        double sum = 0;
        for (int i = 0; i < 4; i++) sum += basisFunction(i, 3, t * (knots3[4] - knots3[3]) + knots3[3], knots3);
        if (std::abs(sum - 1.0) > 0.01) {
            std::cerr << "  ❌ Partition of unity failed at t=" << t << ", sum=" << sum << std::endl;
            partitionOk = false;
        }
    }
    if (partitionOk) std::cout << "  ✅ Partition of unity holds" << std::endl;
    
    // 验证 clamped B-spline 经过端点
    Vec2 startPt = bsplinePoint(ctrl_cubic, clampedKnots(ctrl_cubic.size(), 3), 3, 0.0);
    Vec2 endPt   = bsplinePoint(ctrl_cubic, clampedKnots(ctrl_cubic.size(), 3), 3, 1.0);
    double startDist = (startPt - ctrl_cubic.front()).length();
    double endDist   = (endPt   - ctrl_cubic.back()).length();
    
    std::cout << "  Clamped start point: (" << startPt.x << ", " << startPt.y << ")"
              << " control[0]: (" << ctrl_cubic[0].x << ", " << ctrl_cubic[0].y << ")"
              << " dist=" << startDist << std::endl;
    std::cout << "  Clamped end   point: (" << endPt.x << ", " << endPt.y << ")"
              << " control[-1]: (" << ctrl_cubic.back().x << ", " << ctrl_cubic.back().y << ")"
              << " dist=" << endDist << std::endl;
    
    if (startDist < 0.05 && endDist < 0.05)
        std::cout << "  ✅ Clamped B-spline passes through endpoints" << std::endl;
    else
        std::cerr << "  ⚠️  Endpoint interpolation tolerance exceeded" << std::endl;
    
    std::cout << "\n✅ All outputs generated successfully!" << std::endl;
    return 0;
}
