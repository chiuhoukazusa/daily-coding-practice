#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <vector>
#include <algorithm>

struct Color {
    unsigned char r, g, b;
    Color(int r = 0, int g = 0, int b = 0) : r(r), g(g), b(b) {}
};

// HSV 转 RGB
Color hsvToRgb(double h, double s, double v) {
    double c = v * s;
    double x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
    double m = v - c;
    
    double r, g, b;
    if (h < 60) { r = c; g = x; b = 0; }
    else if (h < 120) { r = x; g = c; b = 0; }
    else if (h < 180) { r = 0; g = c; b = x; }
    else if (h < 240) { r = 0; g = x; b = c; }
    else if (h < 300) { r = x; g = 0; b = c; }
    else { r = c; g = 0; b = x; }
    
    return Color((r + m) * 255, (g + m) * 255, (b + m) * 255);
}

// 计算曼德勃罗集迭代次数
int mandelbrotIterations(double cr, double ci, int maxIter) {
    double zr = 0, zi = 0;
    int iter = 0;
    
    while (zr * zr + zi * zi <= 4.0 && iter < maxIter) {
        double temp = zr * zr - zi * zi + cr;
        zi = 2 * zr * zi + ci;
        zr = temp;
        iter++;
    }
    
    return iter;
}

// 生成曼德勃罗集图像
void generateMandelbrot(const char* filename, int width, int height,
                       double centerX, double centerY, double zoom,
                       int maxIter, int colorScheme = 0) {
    
    std::vector<unsigned char> pixels(width * height * 3);
    
    double rangeX = 3.5 / zoom;
    double rangeY = 2.0 / zoom;
    
    for (int py = 0; py < height; py++) {
        for (int px = 0; px < width; px++) {
            // 映射到复平面
            double cr = centerX + (px - width / 2.0) / width * rangeX;
            double ci = centerY + (py - height / 2.0) / height * rangeY;
            
            int iter = mandelbrotIterations(cr, ci, maxIter);
            
            Color color;
            if (iter == maxIter) {
                // 属于集合 → 黑色
                color = Color(0, 0, 0);
            } else {
                // 不属于集合 → 根据迭代次数着色
                switch (colorScheme) {
                    case 0: {
                        // 蓝色渐变
                        int intensity = (iter * 255) / maxIter;
                        color = Color(0, intensity / 2, intensity);
                        break;
                    }
                    case 1: {
                        // 火焰色
                        double t = (double)iter / maxIter;
                        if (t < 0.5) {
                            color = Color(255 * (t * 2), 0, 0);
                        } else {
                            color = Color(255, 255 * (t - 0.5) * 2, 0);
                        }
                        break;
                    }
                    case 2: {
                        // 彩虹色（HSV）
                        double hue = (360.0 * iter) / maxIter;
                        color = hsvToRgb(hue, 1.0, 1.0);
                        break;
                    }
                    case 3: {
                        // 紫-粉色
                        double t = (double)iter / maxIter;
                        color = Color(255 * t, 50, 255 * (1 - t));
                        break;
                    }
                }
            }
            
            int idx = (py * width + px) * 3;
            pixels[idx] = color.r;
            pixels[idx + 1] = color.g;
            pixels[idx + 2] = color.b;
        }
    }
    
    stbi_write_png(filename, width, height, 3, pixels.data(), width * 3);
}

int main() {
    const int W = 1200, H = 800;
    
    // 1. 基础全景
    generateMandelbrot("mandelbrot_basic.png", W, H, 
                       -0.5, 0.0, 1.0, 256, 0);
    
    // 2. 放大 "海马谷" (Seahorse Valley)
    generateMandelbrot("mandelbrot_zoom1.png", W, H,
                       -0.75, 0.1, 8.0, 512, 1);
    
    // 3. 深度放大 - 螺旋区域
    generateMandelbrot("mandelbrot_zoom2.png", W, H,
                       -0.7269, 0.1889, 200.0, 1000, 2);
    
    // 4. 彩虹配色全景
    generateMandelbrot("mandelbrot_rainbow.png", W, H,
                       -0.5, 0.0, 1.0, 256, 2);
    
    // 5. 经典"象谷"区域
    generateMandelbrot("mandelbrot_elephant.png", W, H,
                       0.3, 0.0, 3.0, 400, 3);
    
    return 0;
}
