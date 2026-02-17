#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

// STB Image Write library (header-only)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

class PerlinNoise {
private:
    std::vector<int> permutation;
    
    // Fade function: 6t^5 - 15t^4 + 10t^3
    double fade(double t) const {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }
    
    // Linear interpolation
    double lerp(double t, double a, double b) const {
        return a + t * (b - a);
    }
    
    // Gradient function
    double grad(int hash, double x, double y) const {
        int h = hash & 7;  // Use lower 3 bits
        double u = h < 4 ? x : y;
        double v = h < 4 ? y : x;
        return ((h & 1) ? -u : u) + ((h & 2) ? -v : v);
    }
    
public:
    PerlinNoise(unsigned int seed = 0) {
        // Create permutation table
        permutation.resize(512);
        
        // Fill with 0-255
        for (int i = 0; i < 256; i++) {
            permutation[i] = i;
        }
        
        // Shuffle
        std::default_random_engine engine(seed);
        std::shuffle(permutation.begin(), permutation.begin() + 256, engine);
        
        // Duplicate for overflow handling
        for (int i = 0; i < 256; i++) {
            permutation[256 + i] = permutation[i];
        }
    }
    
    // 2D Perlin noise
    double noise(double x, double y) const {
        // Find unit grid cell
        int X = static_cast<int>(std::floor(x)) & 255;
        int Y = static_cast<int>(std::floor(y)) & 255;
        
        // Relative position in cell
        x -= std::floor(x);
        y -= std::floor(y);
        
        // Compute fade curves
        double u = fade(x);
        double v = fade(y);
        
        // Hash coordinates of 4 corners
        int aa = permutation[permutation[X] + Y];
        int ab = permutation[permutation[X] + Y + 1];
        int ba = permutation[permutation[X + 1] + Y];
        int bb = permutation[permutation[X + 1] + Y + 1];
        
        // Blend results from 4 corners
        double result = lerp(v,
            lerp(u, grad(aa, x, y), grad(ba, x - 1, y)),
            lerp(u, grad(ab, x, y - 1), grad(bb, x - 1, y - 1))
        );
        
        return result;
    }
    
    // Octave noise (multi-frequency)
    double octaveNoise(double x, double y, int octaves, double persistence) const {
        double total = 0.0;
        double frequency = 1.0;
        double amplitude = 1.0;
        double maxValue = 0.0;
        
        for (int i = 0; i < octaves; i++) {
            total += noise(x * frequency, y * frequency) * amplitude;
            
            maxValue += amplitude;
            
            amplitude *= persistence;
            frequency *= 2.0;
        }
        
        return total / maxValue;
    }
};

// Generate terrain heightmap
void generateTerrain(int width, int height, const std::string& filename) {
    PerlinNoise perlin(12345);
    
    std::vector<unsigned char> image(width * height);
    
    double scale = 10.0;  // Zoom level
    int octaves = 6;
    double persistence = 0.5;
    
    std::cout << "Generating " << width << "x" << height << " terrain..." << std::endl;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Get noise value
            double nx = static_cast<double>(x) / width * scale;
            double ny = static_cast<double>(y) / height * scale;
            
            double noiseValue = perlin.octaveNoise(nx, ny, octaves, persistence);
            
            // Normalize to [0, 1]
            noiseValue = (noiseValue + 1.0) * 0.5;
            
            // Convert to grayscale [0, 255]
            unsigned char gray = static_cast<unsigned char>(noiseValue * 255);
            
            image[y * width + x] = gray;
        }
    }
    
    // Write PNG file
    if (stbi_write_png(filename.c_str(), width, height, 1, image.data(), width)) {
        std::cout << "✅ Successfully generated: " << filename << std::endl;
    } else {
        std::cerr << "❌ Failed to write image!" << std::endl;
    }
}

int main() {
    std::cout << "=== Perlin Noise Terrain Generator ===" << std::endl;
    
    // Generate different terrain maps
    generateTerrain(512, 512, "terrain_512.png");
    generateTerrain(1024, 1024, "terrain_1024.png");
    
    std::cout << "\n✅ All terrain maps generated successfully!" << std::endl;
    
    return 0;
}
