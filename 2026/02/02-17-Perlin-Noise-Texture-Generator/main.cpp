#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <iostream>

class PerlinNoise {
private:
    std::vector<int> permutation;
    
    double fade(double t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }
    
    double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }
    
    double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
public:
    PerlinNoise(unsigned int seed = 0) {
        permutation.resize(512);
        
        // 初始化排列表
        for (int i = 0; i < 256; i++) {
            permutation[i] = i;
        }
        
        // 使用种子打乱
        srand(seed);
        for (int i = 0; i < 256; i++) {
            int j = rand() % 256;
            std::swap(permutation[i], permutation[j]);
        }
        
        // 复制到后半部分
        for (int i = 0; i < 256; i++) {
            permutation[256 + i] = permutation[i];
        }
    }
    
    double noise(double x, double y, double z) {
        // 找到单位立方体的坐标
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        int Z = (int)floor(z) & 255;
        
        // 找到相对位置
        x -= floor(x);
        y -= floor(y);
        z -= floor(z);
        
        // 计算淡化曲线
        double u = fade(x);
        double v = fade(y);
        double w = fade(z);
        
        // 哈希立方体8个角的坐标
        int A = permutation[X] + Y;
        int AA = permutation[A] + Z;
        int AB = permutation[A + 1] + Z;
        int B = permutation[X + 1] + Y;
        int BA = permutation[B] + Z;
        int BB = permutation[B + 1] + Z;
        
        // 插值
        return lerp(w,
            lerp(v,
                lerp(u, grad(permutation[AA], x, y, z),
                        grad(permutation[BA], x - 1, y, z)),
                lerp(u, grad(permutation[AB], x, y - 1, z),
                        grad(permutation[BB], x - 1, y - 1, z))),
            lerp(v,
                lerp(u, grad(permutation[AA + 1], x, y, z - 1),
                        grad(permutation[BA + 1], x - 1, y, z - 1)),
                lerp(u, grad(permutation[AB + 1], x, y - 1, z - 1),
                        grad(permutation[BB + 1], x - 1, y - 1, z - 1))));
    }
    
    // 分形布朗运动（FBM）- 多层噪声叠加
    double fbm(double x, double y, int octaves = 6, double persistence = 0.5) {
        double total = 0.0;
        double frequency = 1.0;
        double amplitude = 1.0;
        double maxValue = 0.0;
        
        for (int i = 0; i < octaves; i++) {
            total += noise(x * frequency, y * frequency, 0.0) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2.0;
        }
        
        return total / maxValue;
    }
};

// 生成大理石纹理
void generateMarbleTexture(const char* filename, int width, int height) {
    PerlinNoise perlin(time(0));
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // 使用 FBM 生成噪声
            double nx = (double)x / width * 4.0;
            double ny = (double)y / height * 4.0;
            
            // 大理石效果：sin(x + 噪声)
            double noiseValue = perlin.fbm(nx, ny, 6, 0.5);
            double marble = sin((nx + noiseValue * 5.0) * M_PI);
            
            // 映射到 [0, 255]
            marble = (marble + 1.0) * 0.5;
            unsigned char color = (unsigned char)(marble * 255);
            
            int idx = (y * width + x) * 3;
            // 白色大理石带灰色纹理
            image[idx + 0] = color;
            image[idx + 1] = color;
            image[idx + 2] = color + 20; // 略带蓝色
        }
    }
    
    stbi_write_png(filename, width, height, 3, image.data(), width * 3);
    std::cout << "生成大理石纹理: " << filename << std::endl;
}

// 生成云朵纹理
void generateCloudTexture(const char* filename, int width, int height) {
    PerlinNoise perlin(time(0) + 1);
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double nx = (double)x / width * 6.0;
            double ny = (double)y / height * 6.0;
            
            // 云朵效果：多层噪声叠加
            double noiseValue = perlin.fbm(nx, ny, 8, 0.6);
            noiseValue = (noiseValue + 1.0) * 0.5; // 映射到 [0, 1]
            
            // 云朵颜色：白色到蓝色渐变
            unsigned char cloudDensity = (unsigned char)(noiseValue * 255);
            
            int idx = (y * width + x) * 3;
            image[idx + 0] = 200 + cloudDensity / 4; // R
            image[idx + 1] = 220 + cloudDensity / 8; // G
            image[idx + 2] = 255; // B
        }
    }
    
    stbi_write_png(filename, width, height, 3, image.data(), width * 3);
    std::cout << "生成云朵纹理: " << filename << std::endl;
}

// 生成木纹纹理
void generateWoodTexture(const char* filename, int width, int height) {
    PerlinNoise perlin(time(0) + 2);
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double nx = (double)x / width * 8.0;
            double ny = (double)y / height * 8.0;
            
            // 木纹效果：基于距离的环形噪声
            double distance = sqrt(nx * nx + ny * ny);
            double noiseValue = perlin.fbm(nx, ny, 4, 0.5);
            double wood = sin((distance + noiseValue * 3.0) * M_PI);
            
            wood = (wood + 1.0) * 0.5;
            
            // 木头颜色：棕色渐变
            unsigned char baseColor = (unsigned char)(wood * 100 + 100);
            
            int idx = (y * width + x) * 3;
            image[idx + 0] = baseColor + 50;  // R
            image[idx + 1] = baseColor / 2;   // G
            image[idx + 2] = baseColor / 4;   // B
        }
    }
    
    stbi_write_png(filename, width, height, 3, image.data(), width * 3);
    std::cout << "生成木纹纹理: " << filename << std::endl;
}

int main() {
    std::cout << "=== Perlin Noise 程序化纹理生成器 ===" << std::endl;
    
    const int WIDTH = 512;
    const int HEIGHT = 512;
    
    generateMarbleTexture("marble.png", WIDTH, HEIGHT);
    generateCloudTexture("clouds.png", WIDTH, HEIGHT);
    generateWoodTexture("wood.png", WIDTH, HEIGHT);
    
    std::cout << "所有纹理生成完成！" << std::endl;
    return 0;
}
