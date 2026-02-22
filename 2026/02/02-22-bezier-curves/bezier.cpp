#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

struct Vec2 {
    double x, y;
    Vec2(double x = 0, double y = 0) : x(x), y(y) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator*(double s) const { return Vec2(x * s, y * s); }
};

// De Casteljau 递归算法
Vec2 deCasteljau(const std::vector<Vec2>& points, double t) {
    std::vector<Vec2> temp = points;
    int n = temp.size();
    
    for (int k = 1; k < n; k++) {
        for (int i = 0; i < n - k; i++) {
            temp[i] = temp[i] * (1 - t) + temp[i + 1] * t;
        }
    }
    
    return temp[0];
}

// 简单的画布类
class Canvas {
public:
    int width, height;
    std::vector<unsigned char> pixels;
    
    Canvas(int w, int h) : width(w), height(h), pixels(w * h * 3, 255) {}
    
    void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        pixels[idx] = r;
        pixels[idx + 1] = g;
        pixels[idx + 2] = b;
    }
    
    // 画线（Bresenham算法）
    void drawLine(int x0, int y0, int x1, int y1, unsigned char r, unsigned char g, unsigned char b) {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            setPixel(x0, y0, r, g, b);
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
    
    // 画粗线（多次偏移）
    void drawThickLine(int x0, int y0, int x1, int y1, int thickness, unsigned char r, unsigned char g, unsigned char b) {
        for (int dx = -thickness/2; dx <= thickness/2; dx++) {
            for (int dy = -thickness/2; dy <= thickness/2; dy++) {
                if (dx*dx + dy*dy <= (thickness/2)*(thickness/2)) {
                    drawLine(x0 + dx, y0 + dy, x1 + dx, y1 + dy, r, g, b);
                }
            }
        }
    }
    
    // 画圆（控制点）
    void drawCircle(int cx, int cy, int radius, unsigned char r, unsigned char g, unsigned char b, bool filled = true) {
        for (int y = -radius; y <= radius; y++) {
            for (int x = -radius; x <= radius; x++) {
                if (x*x + y*y <= radius*radius) {
                    if (filled || x*x + y*y >= (radius-2)*(radius-2)) {
                        setPixel(cx + x, cy + y, r, g, b);
                    }
                }
            }
        }
    }
    
    // 绘制 Bezier 曲线
    void drawBezier(const std::vector<Vec2>& controlPoints, unsigned char r, unsigned char g, unsigned char b, int samples = 100) {
        Vec2 prev = deCasteljau(controlPoints, 0);
        
        for (int i = 1; i <= samples; i++) {
            double t = (double)i / samples;
            Vec2 curr = deCasteljau(controlPoints, t);
            drawThickLine((int)prev.x, (int)prev.y, (int)curr.x, (int)curr.y, 3, r, g, b);
            prev = curr;
        }
    }
    
    // 绘制控制多边形
    void drawControlPolygon(const std::vector<Vec2>& controlPoints) {
        for (size_t i = 0; i < controlPoints.size() - 1; i++) {
            drawLine((int)controlPoints[i].x, (int)controlPoints[i].y,
                    (int)controlPoints[i+1].x, (int)controlPoints[i+1].y,
                    200, 200, 200);
        }
    }
    
    // 绘制控制点
    void drawControlPoints(const std::vector<Vec2>& controlPoints) {
        for (size_t i = 0; i < controlPoints.size(); i++) {
            // 外圈（黑色边框）
            drawCircle((int)controlPoints[i].x, (int)controlPoints[i].y, 6, 0, 0, 0);
            // 内圈（红色填充）
            drawCircle((int)controlPoints[i].x, (int)controlPoints[i].y, 4, 255, 0, 0);
        }
    }
    
    void save(const std::string& filename) {
        stbi_write_png(filename.c_str(), width, height, 3, pixels.data(), width * 3);
    }
};

int main() {
    const int W = 800, H = 600;
    
    // 1. 二阶 Bezier 曲线（抛物线）
    {
        Canvas canvas(W, H);
        std::vector<Vec2> points = {
            Vec2(100, 500),
            Vec2(400, 100),
            Vec2(700, 500)
        };
        
        canvas.drawControlPolygon(points);
        canvas.drawBezier(points, 0, 100, 255, 100);
        canvas.drawControlPoints(points);
        canvas.save("bezier_quadratic.png");
    }
    
    // 2. 三阶 Bezier 曲线（S型）
    {
        Canvas canvas(W, H);
        std::vector<Vec2> points = {
            Vec2(100, 500),
            Vec2(200, 100),
            Vec2(600, 100),
            Vec2(700, 500)
        };
        
        canvas.drawControlPolygon(points);
        canvas.drawBezier(points, 255, 100, 0, 150);
        canvas.drawControlPoints(points);
        canvas.save("bezier_cubic.png");
    }
    
    // 3. 四阶 Bezier 曲线（波浪形）
    {
        Canvas canvas(W, H);
        std::vector<Vec2> points = {
            Vec2(100, 300),
            Vec2(250, 100),
            Vec2(400, 500),
            Vec2(550, 100),
            Vec2(700, 300)
        };
        
        canvas.drawControlPolygon(points);
        canvas.drawBezier(points, 100, 200, 50, 200);
        canvas.drawControlPoints(points);
        canvas.save("bezier_quartic.png");
    }
    
    // 4. 组合展示（多条曲线）
    {
        Canvas canvas(W, H);
        
        // 心形曲线（两个三阶Bezier）
        std::vector<Vec2> heart1 = {
            Vec2(400, 200),
            Vec2(200, 50),
            Vec2(200, 350),
            Vec2(400, 500)
        };
        std::vector<Vec2> heart2 = {
            Vec2(400, 200),
            Vec2(600, 50),
            Vec2(600, 350),
            Vec2(400, 500)
        };
        
        canvas.drawBezier(heart1, 255, 0, 100, 150);
        canvas.drawBezier(heart2, 255, 0, 100, 150);
        
        // 绘制控制点和多边形
        canvas.drawControlPolygon(heart1);
        canvas.drawControlPolygon(heart2);
        canvas.drawControlPoints(heart1);
        canvas.drawControlPoints(heart2);
        
        canvas.save("bezier_combined.png");
    }
    
    return 0;
}
