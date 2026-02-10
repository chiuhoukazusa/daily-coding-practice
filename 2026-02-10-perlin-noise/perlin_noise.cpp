#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <cstring>
#include <algorithm>

// Simple PNG writer using raw format (no external dependencies)
class PNGWriter {
public:
    static void writePNG(const std::string& filename, const std::vector<unsigned char>& data, int width, int height) {
        // For simplicity, we'll write a PPM file instead (easier format, no compression)
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }
        
        // PPM header
        file << "P6\n" << width << " " << height << "\n255\n";
        
        // Write RGB data
        for (size_t i = 0; i < data.size(); i++) {
            file.put(data[i]);
        }
        
        file.close();
        std::cout << "Image saved to " << filename << std::endl;
    }
};

// Perlin Noise generator
class PerlinNoise {
private:
    std::vector<int> permutation;
    
    // Fade function for smooth interpolation
    double fade(double t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }
    
    // Linear interpolation
    double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }
    
    // Gradient function
    double grad(int hash, double x, double y) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : (h == 12 || h == 14 ? x : 0);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
public:
    PerlinNoise(unsigned int seed = 0) {
        // Initialize permutation table
        permutation.resize(512);
        
        // Fill with values 0-255
        std::vector<int> p(256);
        for (int i = 0; i < 256; i++) {
            p[i] = i;
        }
        
        // Shuffle using seed
        std::default_random_engine engine(seed);
        std::shuffle(p.begin(), p.end(), engine);
        
        // Duplicate for easy wrapping
        for (int i = 0; i < 256; i++) {
            permutation[i] = p[i];
            permutation[256 + i] = p[i];
        }
    }
    
    // Get noise value at (x, y)
    double noise(double x, double y) {
        // Find unit square that contains point
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        
        // Find relative x, y in square
        x -= floor(x);
        y -= floor(y);
        
        // Compute fade curves
        double u = fade(x);
        double v = fade(y);
        
        // Hash coordinates of square corners
        int A = permutation[X] + Y;
        int AA = permutation[A];
        int AB = permutation[A + 1];
        int B = permutation[X + 1] + Y;
        int BA = permutation[B];
        int BB = permutation[B + 1];
        
        // Blend results from 4 corners
        double result = lerp(v,
            lerp(u, grad(permutation[AA], x, y), grad(permutation[BA], x - 1, y)),
            lerp(u, grad(permutation[AB], x, y - 1), grad(permutation[BB], x - 1, y - 1))
        );
        
        return result;
    }
    
    // Octave noise (multiple layers)
    double octaveNoise(double x, double y, int octaves, double persistence) {
        double total = 0;
        double frequency = 1;
        double amplitude = 1;
        double maxValue = 0;
        
        for (int i = 0; i < octaves; i++) {
            total += noise(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2;
        }
        
        return total / maxValue;
    }
};

// Generate different types of textures
void generateCloudTexture(const std::string& filename, int width, int height) {
    PerlinNoise perlin(12345);
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Use octave noise for natural clouds
            double nx = (double)x / width;
            double ny = (double)y / height;
            double value = perlin.octaveNoise(nx * 8, ny * 8, 6, 0.5);
            
            // Map to [0, 1]
            value = (value + 1.0) / 2.0;
            
            // Create cloud-like appearance
            unsigned char color = (unsigned char)(value * 255);
            
            int idx = (y * width + x) * 3;
            image[idx] = color;     // R
            image[idx + 1] = color; // G
            image[idx + 2] = color; // B
        }
    }
    
    PNGWriter::writePNG(filename, image, width, height);
}

void generateMarbleTexture(const std::string& filename, int width, int height) {
    PerlinNoise perlin(54321);
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double nx = (double)x / width;
            double ny = (double)y / height;
            
            // Create marble veins
            double value = perlin.octaveNoise(nx * 10, ny * 10, 4, 0.6);
            value = sin((nx * 20 + value * 5) * M_PI);
            
            // Map to [0, 1]
            value = (value + 1.0) / 2.0;
            
            // Marble colors (white with dark veins)
            unsigned char base = 230;
            unsigned char vein = (unsigned char)(value * 80);
            unsigned char color = base - vein;
            
            int idx = (y * width + x) * 3;
            image[idx] = color;         // R
            image[idx + 1] = color - 20; // G (slightly less)
            image[idx + 2] = color - 10; // B
        }
    }
    
    PNGWriter::writePNG(filename, image, width, height);
}

void generateWoodTexture(const std::string& filename, int width, int height) {
    PerlinNoise perlin(99999);
    std::vector<unsigned char> image(width * height * 3);
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double nx = (double)x / width - 0.5;
            double ny = (double)y / height - 0.5;
            
            // Distance from center (wood rings)
            double dist = sqrt(nx * nx + ny * ny);
            double noise = perlin.octaveNoise(nx * 5, ny * 5, 3, 0.5);
            
            // Create ring pattern
            double value = sin((dist + noise * 0.3) * 40) * 0.5 + 0.5;
            
            // Wood colors (brown tones)
            unsigned char r = (unsigned char)(139 * value + 80);
            unsigned char g = (unsigned char)(90 * value + 50);
            unsigned char b = (unsigned char)(43 * value + 20);
            
            int idx = (y * width + x) * 3;
            image[idx] = r;
            image[idx + 1] = g;
            image[idx + 2] = b;
        }
    }
    
    PNGWriter::writePNG(filename, image, width, height);
}

int main() {
    const int width = 512;
    const int height = 512;
    
    std::cout << "Generating Perlin Noise Textures..." << std::endl;
    std::cout << "Resolution: " << width << "x" << height << std::endl << std::endl;
    
    // Generate three different textures
    std::cout << "[1/3] Generating cloud texture..." << std::endl;
    generateCloudTexture("output_clouds.ppm", width, height);
    
    std::cout << "[2/3] Generating marble texture..." << std::endl;
    generateMarbleTexture("output_marble.ppm", width, height);
    
    std::cout << "[3/3] Generating wood texture..." << std::endl;
    generateWoodTexture("output_wood.ppm", width, height);
    
    std::cout << std::endl << "✅ All textures generated successfully!" << std::endl;
    std::cout << "Files: output_clouds.ppm, output_marble.ppm, output_wood.ppm" << std::endl;
    
    return 0;
}
