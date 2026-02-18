#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

class PerlinNoise {
private:
    std::vector<int> p; // Permutation vector
    
    // Fade function: 6t^5 - 15t^4 + 10t^3
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
        double v = h < 4 ? y : h == 12 || h == 14 ? x : 0;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
    
public:
    // Constructor: initialize permutation table
    PerlinNoise(unsigned int seed = 237) {
        p.resize(256);
        
        // Fill with values 0-255
        for (int i = 0; i < 256; ++i) {
            p[i] = i;
        }
        
        // Shuffle using seed
        std::default_random_engine engine(seed);
        std::shuffle(p.begin(), p.end(), engine);
        
        // Duplicate the permutation vector
        p.insert(p.end(), p.begin(), p.end());
    }
    
    // 2D Perlin noise
    double noise(double x, double y) {
        // Find unit square that contains point
        int X = static_cast<int>(std::floor(x)) & 255;
        int Y = static_cast<int>(std::floor(y)) & 255;
        
        // Find relative x, y of point in square
        x -= std::floor(x);
        y -= std::floor(y);
        
        // Compute fade curves
        double u = fade(x);
        double v = fade(y);
        
        // Hash coordinates of the 4 square corners
        int A = p[X] + Y;
        int AA = p[A];
        int AB = p[A + 1];
        int B = p[X + 1] + Y;
        int BA = p[B];
        int BB = p[B + 1];
        
        // Blend results from the 4 corners
        double res = lerp(v,
            lerp(u, grad(p[AA], x, y), grad(p[BA], x - 1, y)),
            lerp(u, grad(p[AB], x, y - 1), grad(p[BB], x - 1, y - 1))
        );
        
        return (res + 1.0) / 2.0; // Normalize to [0, 1]
    }
    
    // Fractal Brownian Motion (multiple octaves)
    double fbm(double x, double y, int octaves = 4, double persistence = 0.5) {
        double total = 0.0;
        double frequency = 1.0;
        double amplitude = 1.0;
        double maxValue = 0.0;
        
        for (int i = 0; i < octaves; ++i) {
            total += noise(x * frequency, y * frequency) * amplitude;
            
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= 2.0;
        }
        
        return total / maxValue;
    }
};

// Save as PPM image (simple format, no external libs needed)
void savePPM(const std::string& filename, const std::vector<std::vector<double>>& heightmap) {
    int width = heightmap.size();
    int height = heightmap[0].size();
    
    std::ofstream file(filename);
    file << "P3\n" << width << " " << height << "\n255\n";
    
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int value = static_cast<int>(heightmap[x][y] * 255);
            value = std::max(0, std::min(255, value));
            
            // Color gradient: blue (low) -> green -> brown (high)
            int r, g, b;
            if (value < 85) {
                // Water: blue
                r = 0;
                g = value;
                b = 128 + value;
            } else if (value < 170) {
                // Land: green
                r = value - 85;
                g = 128 + (value - 85);
                b = 0;
            } else {
                // Mountains: brown/white
                r = 128 + (value - 170);
                g = 128 + (value - 170) / 2;
                b = 64 + (value - 170) / 3;
            }
            
            file << r << " " << g << " " << b << " ";
        }
        file << "\n";
    }
    
    file.close();
}

int main() {
    const int width = 512;
    const int height = 512;
    const double scale = 0.02; // Scale of the noise
    
    std::cout << "Generating Perlin Noise terrain (" << width << "x" << height << ")...\n";
    
    PerlinNoise perlin(12345);
    
    // Generate heightmap
    std::vector<std::vector<double>> heightmap(width, std::vector<double>(height));
    
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            double nx = x * scale;
            double ny = y * scale;
            
            // Use FBM with 6 octaves for natural terrain
            heightmap[x][y] = perlin.fbm(nx, ny, 6, 0.5);
        }
        
        // Progress indicator
        if (x % 64 == 0) {
            std::cout << "Progress: " << (x * 100 / width) << "%\r" << std::flush;
        }
    }
    
    std::cout << "Progress: 100%\n";
    std::cout << "Saving to terrain.ppm...\n";
    
    savePPM("terrain.ppm", heightmap);
    
    std::cout << "✓ Terrain generation complete!\n";
    std::cout << "Output: terrain.ppm\n";
    
    return 0;
}
