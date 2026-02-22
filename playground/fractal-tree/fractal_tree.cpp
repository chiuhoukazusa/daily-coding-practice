#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

const double PI = 3.14159265358979323846;

struct Color {
    unsigned char r, g, b;
    Color(int r, int g, int b) : r(r), g(g), b(b) {}
};

class Canvas {
public:
    int width, height;
    std::vector<unsigned char> pixels;
    
    Canvas(int w, int h, Color bg = Color(255, 255, 255)) 
        : width(w), height(h), pixels(w * h * 3) {
        for (int i = 0; i < w * h; i++) {
            pixels[i * 3] = bg.r;
            pixels[i * 3 + 1] = bg.g;
            pixels[i * 3 + 2] = bg.b;
        }
    }
    
    void setPixel(int x, int y, Color c) {
        if (x < 0 || x >= width || y < 0 || y >= height) return;
        int idx = (y * width + x) * 3;
        pixels[idx] = c.r;
        pixels[idx + 1] = c.g;
        pixels[idx + 2] = c.b;
    }
    
    // Bresenham 画线
    void drawLine(int x0, int y0, int x1, int y1, Color c, int thickness = 1) {
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            // 画粗线
            for (int dy = -thickness/2; dy <= thickness/2; dy++) {
                for (int dx = -thickness/2; dx <= thickness/2; dx++) {
                    setPixel(x0 + dx, y0 + dy, c);
                }
            }
            
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }
    
    // 画圆（用于叶子）
    void drawCircle(int cx, int cy, int radius, Color c) {
        for (int y = -radius; y <= radius; y++) {
            for (int x = -radius; x <= radius; x++) {
                if (x*x + y*y <= radius*radius) {
                    setPixel(cx + x, cy + y, c);
                }
            }
        }
    }
    
    void save(const char* filename) {
        stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
    }
};

// 分形树递归函数
void drawTree(Canvas& canvas, double x, double y, double length, double angle, 
              int depth, double branchAngle, double scaleFactor, 
              std::mt19937& rng, bool addLeaves = false, Color leafColor = Color(0, 200, 0)) {
    
    if (depth == 0) {
        if (addLeaves) {
            // 在末端画叶子
            canvas.drawCircle((int)x, (int)y, 3 + rng() % 3, leafColor);
        }
        return;
    }
    
    // 计算终点
    double endX = x + length * cos(angle);
    double endY = y - length * sin(angle);  // Y轴向下为正
    
    // 树枝颜色：从深棕到浅棕
    int brown = 139 - (12 - depth) * 10;
    brown = std::max(50, std::min(139, brown));
    Color branchColor(brown, brown / 2, 0);
    
    // 树枝粗细：根据深度
    int thickness = std::max(1, depth / 2);
    
    // 画树枝
    canvas.drawLine((int)x, (int)y, (int)endX, (int)endY, branchColor, thickness);
    
    // 递归画左右分支
    drawTree(canvas, endX, endY, length * scaleFactor, angle + branchAngle, 
             depth - 1, branchAngle, scaleFactor, rng, addLeaves, leafColor);
    drawTree(canvas, endX, endY, length * scaleFactor, angle - branchAngle, 
             depth - 1, branchAngle, scaleFactor, rng, addLeaves, leafColor);
}

// 随机树（不对称）
void drawRandomTree(Canvas& canvas, double x, double y, double length, double angle, 
                    int depth, std::mt19937& rng) {
    
    if (depth == 0) {
        // 叶子
        std::uniform_int_distribution<> leafDist(0, 2);
        Color leafColors[] = {
            Color(34, 139, 34),   // 绿色
            Color(0, 200, 0),     // 亮绿
            Color(50, 205, 50)    // 柠檬绿
        };
        canvas.drawCircle((int)x, (int)y, 3, leafColors[leafDist(rng)]);
        return;
    }
    
    double endX = x + length * cos(angle);
    double endY = y - length * sin(angle);
    
    int brown = 100 - (10 - depth) * 8;
    brown = std::max(40, std::min(120, brown));
    Color branchColor(brown, brown / 2, 10);
    
    int thickness = std::max(1, depth / 2);
    canvas.drawLine((int)x, (int)y, (int)endX, (int)endY, branchColor, thickness);
    
    // 随机参数
    std::uniform_real_distribution<> angleDist(0.3, 0.6);
    std::uniform_real_distribution<> scaleDist(0.65, 0.8);
    std::uniform_int_distribution<> branchDist(2, 3);
    
    int numBranches = branchDist(rng);
    for (int i = 0; i < numBranches; i++) {
        double newAngle = angle + (i - numBranches/2.0) * angleDist(rng);
        drawRandomTree(canvas, endX, endY, length * scaleDist(rng), newAngle, 
                      depth - 1, rng);
    }
}

int main() {
    std::random_device rd;
    std::mt19937 rng(42);  // 固定种子以便重现
    
    const int W = 800, H = 800;
    
    // 1. 对称二叉树
    {
        Canvas canvas(W, H, Color(240, 248, 255));  // 淡蓝背景
        double startX = W / 2;
        double startY = H - 50;
        double initialLength = 150;
        double angle = PI / 2;  // 向上
        int depth = 11;
        double branchAngle = PI / 6;  // 30度
        double scale = 0.75;
        
        drawTree(canvas, startX, startY, initialLength, angle, depth, 
                branchAngle, scale, rng, false);
        canvas.save("tree_symmetric.png");
    }
    
    // 2. 随机树
    {
        Canvas canvas(W, H, Color(135, 206, 235));  // 天蓝色
        double startX = W / 2;
        double startY = H - 50;
        
        drawRandomTree(canvas, startX, startY, 120, PI / 2, 9, rng);
        canvas.save("tree_random.png");
    }
    
    // 3. 樱花树
    {
        Canvas canvas(W, H, Color(255, 240, 245));  // 淡粉背景
        double startX = W / 2;
        double startY = H - 50;
        double initialLength = 140;
        double angle = PI / 2;
        int depth = 10;
        double branchAngle = PI / 5;  // 36度
        double scale = 0.72;
        
        Color cherryBlossomPink(255, 182, 193);
        drawTree(canvas, startX, startY, initialLength, angle, depth, 
                branchAngle, scale, rng, true, cherryBlossomPink);
        canvas.save("tree_cherry.png");
    }
    
    // 4. 秋天树（橙黄叶子）
    {
        Canvas canvas(W, H, Color(255, 250, 240));  // 花白色
        double startX = W / 2;
        double startY = H - 50;
        double initialLength = 130;
        double angle = PI / 2;
        int depth = 10;
        double branchAngle = PI / 7;  // 约25度
        double scale = 0.7;
        
        // 秋天的颜色
        std::vector<Color> autumnColors = {
            Color(255, 140, 0),   // 深橙
            Color(255, 165, 0),   // 橙色
            Color(255, 215, 0),   // 金色
            Color(218, 165, 32)   // 金棕色
        };
        
        // 绘制主树
        drawTree(canvas, startX, startY, initialLength, angle, depth, 
                branchAngle, scale, rng, false);
        
        // 手动添加秋叶（在枝头随机位置）
        std::uniform_int_distribution<> colorDist(0, 3);
        for (int i = 0; i < 150; i++) {
            std::uniform_real_distribution<> xDist(W/2 - 200, W/2 + 200);
            std::uniform_real_distribution<> yDist(100, H/2);
            canvas.drawCircle((int)xDist(rng), (int)yDist(rng), 
                            2 + rng() % 3, autumnColors[colorDist(rng)]);
        }
        
        canvas.save("tree_autumn.png");
    }
    
    return 0;
}
