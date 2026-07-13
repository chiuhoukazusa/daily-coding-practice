#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <iostream>
#include <limits>

struct Point {
    float x, y;
    unsigned char r, g, b;
};

// 计算两点之间的欧式距离
float distance(float x1, float y1, float x2, float y2) {
    float dx = x1 - x2;
    float dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
}

// 生成随机种子点
std::vector<Point> generateSeeds(int count, int width, int height) {
    std::vector<Point> seeds;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> disX(0, width - 1);
    std::uniform_int_distribution<> disY(0, height - 1);
    std::uniform_int_distribution<> disColor(50, 255);
    
    for (int i = 0; i < count; ++i) {
        Point p;
        p.x = disX(gen);
        p.y = disY(gen);
        p.r = disColor(gen);
        p.g = disColor(gen);
        p.b = disColor(gen);
        seeds.push_back(p);
    }
    
    return seeds;
}

// 生成Voronoi图
void generateVoronoi(unsigned char* image, int width, int height, const std::vector<Point>& seeds) {
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float minDist = std::numeric_limits<float>::max();
            int closestSeed = 0;
            
            // 找到最近的种子点
            for (size_t i = 0; i < seeds.size(); ++i) {
                float dist = distance(x, y, seeds[i].x, seeds[i].y);
                if (dist < minDist) {
                    minDist = dist;
                    closestSeed = i;
                }
            }
            
            // 设置像素颜色
            int idx = (y * width + x) * 3;
            image[idx + 0] = seeds[closestSeed].r;
            image[idx + 1] = seeds[closestSeed].g;
            image[idx + 2] = seeds[closestSeed].b;
        }
    }
}

// 绘制种子点（用白色标记）
void drawSeeds(unsigned char* image, int width, int height, const std::vector<Point>& seeds) {
    for (const auto& seed : seeds) {
        int x = static_cast<int>(seed.x);
        int y = static_cast<int>(seed.y);
        
        // 绘制一个3x3的白点
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int px = x + dx;
                int py = y + dy;
                if (px >= 0 && px < width && py >= 0 && py < height) {
                    int idx = (py * width + px) * 3;
                    image[idx + 0] = 255;
                    image[idx + 1] = 255;
                    image[idx + 2] = 255;
                }
            }
        }
    }
}

int main() {
    const int WIDTH = 800;
    const int HEIGHT = 600;
    const int SEED_COUNT = 50;
    
    std::cout << "生成Voronoi图..." << std::endl;
    std::cout << "图像尺寸: " << WIDTH << "x" << HEIGHT << std::endl;
    std::cout << "种子点数量: " << SEED_COUNT << std::endl;
    
    // 分配图像内存
    std::vector<unsigned char> image(WIDTH * HEIGHT * 3);
    
    // 生成种子点
    auto seeds = generateSeeds(SEED_COUNT, WIDTH, HEIGHT);
    std::cout << "种子点生成完成" << std::endl;
    
    // 生成Voronoi图
    generateVoronoi(image.data(), WIDTH, HEIGHT, seeds);
    std::cout << "Voronoi图计算完成" << std::endl;
    
    // 绘制种子点
    drawSeeds(image.data(), WIDTH, HEIGHT, seeds);
    std::cout << "种子点标记完成" << std::endl;
    
    // 保存图像
    if (stbi_write_png("voronoi.png", WIDTH, HEIGHT, 3, image.data(), WIDTH * 3)) {
        std::cout << "图像已保存: voronoi.png" << std::endl;
    } else {
        std::cerr << "图像保存失败!" << std::endl;
        return 1;
    }
    
    std::cout << "✅ 完成!" << std::endl;
    return 0;
}
