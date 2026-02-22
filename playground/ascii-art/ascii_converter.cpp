#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

// ASCII 字符集（从暗到亮）
const char ASCII_CHARS[] = " .'`^\",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
const int NUM_CHARS = sizeof(ASCII_CHARS) - 1;

// 将像素亮度映射到ASCII字符
char brightnessToChar(unsigned char r, unsigned char g, unsigned char b) {
    int brightness = (int)(0.299 * r + 0.587 * g + 0.114 * b);
    int index = (brightness * NUM_CHARS) / 256;
    return ASCII_CHARS[index];
}

// 生成ASCII艺术
void generateASCII(const char* inputImage, const char* outputFile, int outputWidth) {
    int width, height, channels;
    unsigned char* img = stbi_load(inputImage, &width, &height, &channels, 3);
    
    if (!img) {
        std::cerr << "Failed to load image: " << inputImage << std::endl;
        return;
    }
    
    double scale = (double)outputWidth / width;
    int outputHeight = (int)(height * scale * 0.55);
    
    std::ofstream out(outputFile);
    
    for (int y = 0; y < outputHeight; y++) {
        for (int x = 0; x < outputWidth; x++) {
            int srcX = (int)(x / scale);
            int srcY = (int)(y / (scale * 0.55));
            
            srcX = std::min(srcX, width - 1);
            srcY = std::min(srcY, height - 1);
            
            int idx = (srcY * width + srcX) * 3;
            out << brightnessToChar(img[idx], img[idx + 1], img[idx + 2]);
        }
        out << '\n';
    }
    
    out.close();
    stbi_image_free(img);
    std::cout << "Generated: " << outputFile << std::endl;
}

int main() {
    // 创建渐变测试图
    const int W = 200, H = 100;
    unsigned char* testImg = new unsigned char[W * H * 3];
    
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int idx = (y * W + x) * 3;
            unsigned char brightness = (x * 255) / W;
            testImg[idx] = brightness;
            testImg[idx + 1] = brightness;
            testImg[idx + 2] = brightness;
        }
    }
    
    stbi_write_png("test_gradient.png", W, H, 3, testImg, W * 3);
    delete[] testImg;
    
    // 转换成ASCII
    generateASCII("test_gradient.png", "ascii_gradient.txt", 80);
    generateASCII("../fractal-tree/tree_symmetric.png", "ascii_tree.txt", 80);
    
    return 0;
}
