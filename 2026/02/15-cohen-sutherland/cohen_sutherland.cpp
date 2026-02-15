#include <iostream>
#include <fstream>
#include <vector>
#include <string>

const int INSIDE = 0;   // 0000
const int LEFT   = 1;   // 0001
const int RIGHT  = 2;   // 0010
const int BOTTOM = 4;   // 0100
const int TOP    = 8;   // 1000

// 裁剪区域定义
const int X_MIN = 200;
const int X_MAX = 600;
const int Y_MIN = 150;
const int Y_MAX = 450;

// 计算点的区域码
int compute_code(int x, int y) {
    int code = INSIDE;
    
    if (x < X_MIN)         // 左边界
        code |= LEFT;
    else if (x > X_MAX)    // 右边界
        code |= RIGHT;
    
    if (y < Y_MIN)         // 下边界
        code |= BOTTOM;
    else if (y > Y_MAX)    // 上边界
        code |= TOP;
    
    return code;
}

// Cohen-Sutherland线段裁剪算法
bool cohen_sutherland_clip(int& x1, int& y1, int& x2, int& y2) {
    int code1 = compute_code(x1, y1);
    int code2 = compute_code(x2, y2);
    bool accept = false;
    
    while (true) {
        // 完全在裁剪区域内
        if (code1 == INSIDE && code2 == INSIDE) {
            accept = true;
            break;
        }
        // 完全在裁剪区域外
        else if (code1 & code2) {
            break;
        }
        // 部分相交，需要裁剪
        else {
            int code_out;
            int x, y;
            
            // 选择在区域外的点
            if (code1 != INSIDE)
                code_out = code1;
            else
                code_out = code2;
            
            // 计算交点
            if (code_out & TOP) {
                x = x1 + (x2 - x1) * (Y_MAX - y1) / (y2 - y1);
                y = Y_MAX;
            }
            else if (code_out & BOTTOM) {
                x = x1 + (x2 - x1) * (Y_MIN - y1) / (y2 - y1);
                y = Y_MIN;
            }
            else if (code_out & RIGHT) {
                y = y1 + (y2 - y1) * (X_MAX - x1) / (x2 - x1);
                x = X_MAX;
            }
            else if (code_out & LEFT) {
                y = y1 + (y2 - y1) * (X_MIN - x1) / (x2 - x1);
                x = X_MIN;
            }
            
            // 更新点的坐标
            if (code_out == code1) {
                x1 = x;
                y1 = y;
                code1 = compute_code(x1, y1);
            }
            else {
                x2 = x;
                y2 = y;
                code2 = compute_code(x2, y2);
            }
        }
    }
    
    return accept;
}

// Bresenham直线绘制算法
void draw_line(std::vector<std::vector<int>>& pixels, int x1, int y1, int x2, int y2, int value) {
    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;
    
    while (true) {
        if (x1 >= 0 && y1 >= 0 && 
            x1 < static_cast<int>(pixels[0].size()) && 
            y1 < static_cast<int>(pixels.size())) {
            pixels[y1][x1] = value;
        }
        
        if (x1 == x2 && y1 == y2) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y1 += sy;
        }
    }
}

// 保存为PPM格式
void save_ppm(const std::vector<std::vector<int>>& pixels, int width, int height, const std::string& filename) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }
    
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 根据像素值设置颜色
            if (pixels[y][x] == 1) {
                // 裁剪区域内的线 - 蓝色
                file << "0 0 255 ";
            }
            else if (pixels[y][x] == 2) {
                // 原始线 - 红色
                file << "255 0 0 ";
            }
            else if (pixels[y][x] == 3) {
                // 裁剪区域边界 - 黑色
                file << "0 0 0 ";
            }
            else {
                // 背景色 - 白色
                file << "255 255 255 ";
            }
        }
        file << "\n";
    }
    
    file.close();
    std::cout << "已保存文件: " << filename << std::endl;
}

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;
    
    // 创建像素矩阵
    std::vector<std::vector<int>> pixels(HEIGHT, std::vector<int>(WIDTH, 0));
    
    // 1. 绘制裁剪区域边界
    draw_line(pixels, X_MIN, Y_MIN, X_MAX, Y_MIN, 3);
    draw_line(pixels, X_MAX, Y_MIN, X_MAX, Y_MAX, 3);
    draw_line(pixels, X_MAX, Y_MAX, X_MIN, Y_MAX, 3);
    draw_line(pixels, X_MIN, Y_MAX, X_MIN, Y_MIN, 3);
    
    // 2. 定义多条测试线段
    struct Line {
        int x1, y1, x2, y2;
    };
    
    std::vector<Line> lines = {
        {100, 100, 300, 200},   // 部分进入
        {50, 300, 700, 300},    // 水平线
        {400, 100, 400, 500},   // 垂直线
        {0, 0, 799, 599},       // 完整对角线
        {300, 400, 500, 200},   // 完全在外
        {250, 200, 350, 400},   // 完全在内
    };
    
    // 3. 绘制所有线段并裁剪
    std::cout << "Cohen-Sutherland线段裁剪算法\n";
    std::cout << "裁剪区域: x=[" << X_MIN << "," << X_MAX << "] y=[" << Y_MIN << "," << Y_MAX << "]\n";
    
    for (size_t i = 0; i < lines.size(); ++i) {
        int x1 = lines[i].x1;
        int y1 = lines[i].y1;
        int x2 = lines[i].x2;
        int y2 = lines[i].y2;
        
        // 绘制原始线段（红色）
        draw_line(pixels, x1, y1, x2, y2, 2);
        
        // 裁剪线段
        int clip_x1 = x1, clip_y1 = y1;
        int clip_x2 = x2, clip_y2 = y2;
        bool accepted = cohen_sutherland_clip(clip_x1, clip_y1, clip_x2, clip_y2);
        
        if (accepted) {
            std::cout << "线段 " << i << ": 成功裁剪\n";
            std::cout << "  原始: (" << x1 << "," << y1 << ")->(" << x2 << "," << y2 << ")\n";
            std::cout << "  裁剪后: (" << clip_x1 << "," << clip_y1 << ")->(" << clip_x2 << "," << clip_y2 << ")\n";
            
            // 绘制裁剪后的线段（蓝色）
            draw_line(pixels, clip_x1, clip_y1, clip_x2, clip_y2, 1);
        }
        else {
            std::cout << "线段 " << i << ": 完全在外部，被丢弃\n";
        }
    }
    
    // 保存为PPM文件
    save_ppm(pixels, WIDTH, HEIGHT, "output.ppm");
    
    // 转换为PNG格式
    std::string convert_cmd = "convert output.ppm output.png";
    std::cout << "正在转换PNG: " << convert_cmd << std::endl;
    system(convert_cmd.c_str());
    
    // 清理
    remove("output.ppm");
    
    std::cout << "\n完成！程序已生成 output.png 图像文件\n";
    
    return 0;
}